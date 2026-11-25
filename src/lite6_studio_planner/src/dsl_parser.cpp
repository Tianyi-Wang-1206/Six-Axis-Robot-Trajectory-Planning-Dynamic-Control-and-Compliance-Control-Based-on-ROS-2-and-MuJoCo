#include "lite6_interfaces/motion_contract.hpp"
#include "lite6_studio_planner/dsl_parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace lite6_studio_planner
{
namespace
{

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr size_t kMaxCodeChars = 64;
constexpr size_t kDiagFixedOverheadBytes = 64;
constexpr size_t kReservedTruncationBytes = 512;

struct LineSpan
{
  size_t start = 0;
  size_t length = 0;
};

size_t utf8_sequence_length(const std::string_view sv, size_t pos)
{
  if (pos >= sv.size()) {
    return 0;
  }
  const unsigned char b0 = static_cast<unsigned char>(sv[pos]);
  if (b0 < 0x80) {
    return 1;
  }
  if ((b0 & 0xE0) == 0xC0 && pos + 1 < sv.size()) {
    const unsigned char b1 = static_cast<unsigned char>(sv[pos + 1]);
    if ((b1 & 0xC0) != 0x80) {
      return 0;
    }
    const uint32_t cp = ((b0 & 0x1Fu) << 6) | (b1 & 0x3Fu);
    return (cp >= 0x80u) ? 2 : 0;
  }
  if ((b0 & 0xF0) == 0xE0 && pos + 2 < sv.size()) {
    const unsigned char b1 = static_cast<unsigned char>(sv[pos + 1]);
    const unsigned char b2 = static_cast<unsigned char>(sv[pos + 2]);
    if ((b1 & 0xC0) != 0x80 || (b2 & 0xC0) != 0x80) {
      return 0;
    }
    const uint32_t cp = ((b0 & 0x0Fu) << 12) | ((b1 & 0x3Fu) << 6) | (b2 & 0x3Fu);
    return (cp >= 0x800u && (cp < 0xD800u || cp > 0xDFFFu)) ? 3 : 0;
  }
  if ((b0 & 0xF8) == 0xF0 && pos + 3 < sv.size()) {
    const unsigned char b1 = static_cast<unsigned char>(sv[pos + 1]);
    const unsigned char b2 = static_cast<unsigned char>(sv[pos + 2]);
    const unsigned char b3 = static_cast<unsigned char>(sv[pos + 3]);
    if ((b1 & 0xC0) != 0x80 || (b2 & 0xC0) != 0x80 || (b3 & 0xC0) != 0x80) {
      return 0;
    }
    const uint32_t cp =
      ((b0 & 0x07u) << 18) | ((b1 & 0x3Fu) << 12) | ((b2 & 0x3Fu) << 6) | (b3 & 0x3Fu);
    return (cp >= 0x10000u && cp <= 0x10FFFFu) ? 4 : 0;
  }
  return 0;
}

std::string sanitize_bounded_view(std::string_view text, size_t max_chars)
{
  if (max_chars == 0 || text.empty()) {
    return "";
  }
  std::string out;
  out.reserve(std::min(text.size(), max_chars));
  const size_t ellipsis_len = (max_chars >= 3) ? 3 : 0;
  const size_t soft_limit = max_chars - ellipsis_len;
  bool truncated = false;
  size_t i = 0;
  while (i < text.size()) {
    const unsigned char b = static_cast<unsigned char>(text[i]);
    char esc_buf[8];
    std::string_view piece;
    size_t advance_by = 1;
    if (b >= 0x20 && b <= 0x7E) {
      piece = text.substr(i, 1);
    } else if (b == '\t' || b == '\r' || b == '\n') {
      piece = " ";
    } else if (b >= 0x80) {
      const size_t ulen = utf8_sequence_length(text, i);
      if (ulen > 0) {
        piece = text.substr(i, ulen);
        advance_by = ulen;
      } else {
        std::snprintf(esc_buf, sizeof(esc_buf), "\\x%02x", static_cast<unsigned>(b));
        piece = std::string_view(esc_buf, 4);
      }
    } else {
      std::snprintf(esc_buf, sizeof(esc_buf), "\\x%02x", static_cast<unsigned>(b));
      piece = std::string_view(esc_buf, 4);
    }

    const bool has_more = (i + advance_by < text.size());
    const size_t limit = has_more ? soft_limit : max_chars;
    if (out.size() + piece.size() > limit) {
      if (has_more || out.size() + piece.size() > max_chars) {
        truncated = true;
        break;
      }
    }
    out.append(piece.data(), piece.size());
    i += advance_by;
  }
  if (truncated && ellipsis_len > 0) {
    while (out.size() + ellipsis_len > max_chars && !out.empty()) {
      // Back off cleanly to a UTF-8 codepoint boundary.
      while (!out.empty() && (static_cast<unsigned char>(out.back()) & 0xC0) == 0x80) {
        out.pop_back();
      }
      if (!out.empty()) {
        out.pop_back();
      }
    }
    out.append("...");
  }
  return out;
}

std::string format_unexpected_char(char c)
{
  const unsigned char u = static_cast<unsigned char>(c);
  if (u >= 0x20 && u <= 0x7E) {
    return std::string("'") + c + "'";
  }
  char buf[16];
  std::snprintf(buf, sizeof(buf), "0x%02x", static_cast<unsigned>(u));
  return std::string(buf);
}

size_t diagnostic_byte_cost(const DSLParser::Diagnostic & d)
{
  return d.code.size() + d.command_text.size() + d.message.size() + kDiagFixedOverheadBytes;
}

void sanitize_diagnostic_fields(DSLParser::Diagnostic & d)
{
  d.code = sanitize_bounded_view(d.code, kMaxCodeChars);
  d.command_text = sanitize_bounded_view(d.command_text, DSLParser::kMaxSnippetChars);
  d.message = sanitize_bounded_view(d.message, DSLParser::kMaxDiagnosticMessageChars);
}

class DiagnosticSink
{
public:
  using D = DSLParser::Diagnostic;
  static constexpr size_t kMaxRegularDiags = DSLParser::kMaxDiagnostics - 1;

  explicit DiagnosticSink(std::vector<D> & diags) : diags_(diags) {}

  bool can_accept_more() const
  {
    return diags_.size() < kMaxRegularDiags &&
           total_bytes_ + kReservedTruncationBytes < DSLParser::kMaxTotalDiagnosticBytes;
  }

  bool add(D d)
  {
    sanitize_diagnostic_fields(d);
    const size_t cost = diagnostic_byte_cost(d);
    if (diags_.size() >= kMaxRegularDiags ||
        total_bytes_ + cost + kReservedTruncationBytes > DSLParser::kMaxTotalDiagnosticBytes)
    {
      ++suppressed_;
      return false;
    }
    total_bytes_ += cost;
    if (d.severity == D::SEVERITY_ERROR) {
      ++error_count_;
    }
    diags_.push_back(std::move(d));
    return true;
  }

  void add_suppressed(size_t n) { suppressed_ += n; }
  void mark_stopped_early() { stopped_early_ = true; }
  size_t suppressed() const { return suppressed_; }
  bool stopped_early() const { return stopped_early_; }
  size_t error_count() const { return error_count_; }

  void finalize()
  {
    if (suppressed_ == 0 && !stopped_early_) {
      return;
    }
    D notice;
    notice.severity = D::SEVERITY_WARNING;
    notice.stage = diags_.empty() ? D::STAGE_LEX_SYNTAX : diags_.back().stage;
    notice.line_number = diags_.empty() ? 1 : diags_.back().line_number;
    notice.column_number = diags_.empty() ? 1 : diags_.back().column_number;
    notice.statement_index = -1;
    notice.code = "WARN_DIAGNOSTICS_TRUNCATED";
    std::ostringstream oss;
    oss << "Diagnostic output capped at " << diags_.size() << " entries";
    if (suppressed_ > 0) {
      oss << " (" << suppressed_ << " additional diagnostic(s) suppressed)";
    }
    if (stopped_early_) {
      oss << "; remaining source analysis skipped";
    }
    notice.message = sanitize_bounded_view(oss.str(), DSLParser::kMaxDiagnosticMessageChars);
    diags_.push_back(std::move(notice));
  }

private:
  std::vector<D> & diags_;
  size_t total_bytes_ = 0;
  size_t suppressed_ = 0;
  size_t error_count_ = 0;
  bool stopped_early_ = false;
};

enum class TokenKind
{
  IDENT,
  NUMBER,
  LBRACKET,
  RBRACKET,
  LPAREN,
  RPAREN,
  LBRACE,
  RBRACE,
  COMMA,
  EQUALS,
  END_OF_FILE
};

struct Token
{
  TokenKind kind = TokenKind::END_OF_FILE;
  std::string lexeme;
  double number = 0.0;
  int line = 1;
  int column = 1;
};

struct ParseError : public std::runtime_error
{
  uint8_t stage;
  int line;
  int column;
  int stmt_idx;
  std::string code;
  ParseError(
    uint8_t stg, int ln, int col, int sidx, const std::string & cd, const std::string & msg)
  : std::runtime_error(msg), stage(stg), line(ln), column(col), stmt_idx(sidx), code(cd)
  {
  }
};

// Builds a zero-copy O(N) index of line byte ranges in source.
// Returns false if line count exceeds DSLParser::kMaxSourceLines.
bool index_lines(const std::string & source, std::vector<LineSpan> & out_lines)
{
  out_lines.clear();
  size_t line_start = 0;
  const size_t n = source.size();
  for (size_t i = 0; i < n; ++i) {
    if (source[i] == '\n') {
      size_t len = i - line_start;
      if (len > 0 && source[line_start + len - 1] == '\r') {
        --len;
      }
      if (out_lines.size() >= DSLParser::kMaxSourceLines) {
        return false;
      }
      out_lines.push_back({line_start, len});
      line_start = i + 1;
    }
  }
  if (line_start < n || out_lines.empty()) {
    size_t len = n - line_start;
    if (len > 0 && source[line_start + len - 1] == '\r') {
      --len;
    }
    if (out_lines.size() >= DSLParser::kMaxSourceLines) {
      return false;
    }
    out_lines.push_back({line_start, len});
  }
  return true;
}

// Extracts an O(1)-bounded, UTF-8-safe snippet for a 1-based line and optional column.
std::string extract_line_snippet(
  const std::string & source,
  const std::vector<LineSpan> & lines,
  int ln,
  int col_1based = 1)
{
  if (ln < 1 || static_cast<size_t>(ln) > lines.size()) {
    return "";
  }
  const LineSpan span = lines[static_cast<size_t>(ln - 1)];
  size_t a = span.start;
  size_t b = span.start + span.length;
  while (a < b && std::isspace(static_cast<unsigned char>(source[a]))) {
    ++a;
  }
  while (b > a && std::isspace(static_cast<unsigned char>(source[b - 1]))) {
    --b;
  }
  if (a >= b) {
    return "";
  }
  const size_t trimmed_len = b - a;
  if (trimmed_len <= DSLParser::kMaxSnippetChars) {
    return sanitize_bounded_view(
      std::string_view(source.data() + a, trimmed_len), DSLParser::kMaxSnippetChars);
  }

  // Center a bounded window around the offending column so huge single-line inputs
  // never copy the full line.
  const size_t raw_col_offset =
    (col_1based > 1) ? static_cast<size_t>(col_1based - 1) : 0;
  const size_t focus = std::clamp(span.start + raw_col_offset, a, b);
  constexpr size_t kLeadContext = 32;
  size_t win_start = (focus > a + kLeadContext) ? (focus - kLeadContext) : a;
  if (win_start + DSLParser::kMaxSnippetChars > b) {
    win_start = b - DSLParser::kMaxSnippetChars;
  }
  // Advance past any UTF-8 continuation bytes at the window start.
  while (win_start < b &&
         (static_cast<unsigned char>(source[win_start]) & 0xC0) == 0x80)
  {
    ++win_start;
  }
  const bool prefix_ellipsis = (win_start > a);
  const size_t avail =
    DSLParser::kMaxSnippetChars - (prefix_ellipsis ? 3 : 0);
  const size_t win_len = std::min(b - win_start, avail + 1);
  std::string body = sanitize_bounded_view(
    std::string_view(source.data() + win_start, win_len), avail);
  if (prefix_ellipsis) {
    return "..." + body;
  }
  return body;
}

std::vector<Token> tokenize(
  const std::string & source,
  const std::vector<LineSpan> & lines,
  DiagnosticSink & sink)
{
  using D = DSLParser::Diagnostic;
  std::vector<Token> tokens;
  int line = 1;
  int col = 1;
  size_t i = 0;
  const size_t n = source.size();

  auto line_snippet = [&](int ln, int c_col) -> std::string {
    return extract_line_snippet(source, lines, ln, c_col);
  };

  auto push_token = [&](Token tok) -> bool {
    if (tokens.size() >= DSLParser::kMaxTokens) {
      D d;
      d.severity = D::SEVERITY_ERROR;
      d.stage = D::STAGE_LEX_SYNTAX;
      d.line_number = tok.line;
      d.column_number = tok.column;
      d.statement_index = -1;
      d.command_text = line_snippet(tok.line, tok.column);
      d.code = "ERR_TOKEN_LIMIT";
      d.message = "Program exceeds maximum token budget (" +
        std::to_string(DSLParser::kMaxTokens) + " tokens)";
      sink.add(std::move(d));
      sink.mark_stopped_early();
      return false;
    }
    tokens.push_back(std::move(tok));
    return true;
  };

  while (i < n) {
    const char c = source[i];
    if (c == '\r') {
      ++i;
      continue;
    }
    if (c == '\n') {
      ++line;
      col = 1;
      ++i;
      continue;
    }
    if (std::isspace(static_cast<unsigned char>(c)) || c == ';') {
      ++col;
      ++i;
      continue;
    }
    if (c == '#') {
      while (i < n && source[i] != '\n') {
        ++i;
      }
      continue;
    }
    const int start_col = col;
    if (c == '[') {
      if (!push_token({TokenKind::LBRACKET, "[", 0.0, line, start_col})) break;
      ++i;
      ++col;
      continue;
    }
    if (c == ']') {
      if (!push_token({TokenKind::RBRACKET, "]", 0.0, line, start_col})) break;
      ++i;
      ++col;
      continue;
    }
    if (c == '(') {
      if (!push_token({TokenKind::LPAREN, "(", 0.0, line, start_col})) break;
      ++i;
      ++col;
      continue;
    }
    if (c == ')') {
      if (!push_token({TokenKind::RPAREN, ")", 0.0, line, start_col})) break;
      ++i;
      ++col;
      continue;
    }
    if (c == '{') {
      if (!push_token({TokenKind::LBRACE, "{", 0.0, line, start_col})) break;
      ++i;
      ++col;
      continue;
    }
    if (c == '}') {
      if (!push_token({TokenKind::RBRACE, "}", 0.0, line, start_col})) break;
      ++i;
      ++col;
      continue;
    }
    if (c == ',') {
      if (!push_token({TokenKind::COMMA, ",", 0.0, line, start_col})) break;
      ++i;
      ++col;
      continue;
    }
    if (c == '=') {
      if (!push_token({TokenKind::EQUALS, "=", 0.0, line, start_col})) break;
      ++i;
      ++col;
      continue;
    }

    if (std::isdigit(static_cast<unsigned char>(c)) ||
        ((c == '+' || c == '-') && i + 1 < n &&
         (std::isdigit(static_cast<unsigned char>(source[i + 1])) || source[i + 1] == '.')) ||
        (c == '.' && i + 1 < n && std::isdigit(static_cast<unsigned char>(source[i + 1])))) {
      const size_t start = i;
      if (source[i] == '+' || source[i] == '-') {
        ++i;
        ++col;
      }
      bool has_dot = false;
      bool has_exp = false;
      while (i < n) {
        const char ch = source[i];
        if (std::isdigit(static_cast<unsigned char>(ch))) {
          ++i;
          ++col;
        } else if (ch == '.' && !has_dot && !has_exp) {
          has_dot = true;
          ++i;
          ++col;
        } else if ((ch == 'e' || ch == 'E') && !has_exp) {
          has_exp = true;
          ++i;
          ++col;
          if (i < n && (source[i] == '+' || source[i] == '-')) {
            ++i;
            ++col;
          }
        } else {
          break;
        }
      }
      const size_t lex_len = i - start;
      if (lex_len > DSLParser::kMaxTokenLength) {
        D d;
        d.severity = D::SEVERITY_ERROR;
        d.stage = D::STAGE_LEX_SYNTAX;
        d.line_number = line;
        d.column_number = start_col;
        d.statement_index = -1;
        d.command_text = line_snippet(line, start_col);
        d.code = "ERR_INVALID_NUMBER";
        d.message = "Numeric literal exceeds maximum token length (" +
          std::to_string(DSLParser::kMaxTokenLength) + " chars)";
        sink.add(std::move(d));
        if (!sink.can_accept_more()) {
          sink.mark_stopped_early();
          break;
        }
        continue;
      }
      const std::string lex = source.substr(start, lex_len);
      try {
        size_t idx = 0;
        const double val = std::stod(lex, &idx);
        if (idx != lex.size() || !std::isfinite(val)) {
          throw std::runtime_error("Non-finite numeric literal");
        }
        if (!push_token({TokenKind::NUMBER, lex, val, line, start_col})) break;
      } catch (const std::exception &) {
        D d;
        d.severity = D::SEVERITY_ERROR;
        d.stage = D::STAGE_LEX_SYNTAX;
        d.line_number = line;
        d.column_number = start_col;
        d.statement_index = -1;
        d.command_text = line_snippet(line, start_col);
        d.code = "ERR_INVALID_NUMBER";
        d.message = "Malformed numeric literal '" + lex + "'";
        sink.add(std::move(d));
        if (!sink.can_accept_more()) {
          sink.mark_stopped_early();
          break;
        }
      }
      continue;
    }

    if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
      const size_t start = i;
      while (i < n &&
             (std::isalnum(static_cast<unsigned char>(source[i])) || source[i] == '_')) {
        ++i;
        ++col;
      }
      const size_t lex_len = i - start;
      if (lex_len > DSLParser::kMaxTokenLength) {
        D d;
        d.severity = D::SEVERITY_ERROR;
        d.stage = D::STAGE_LEX_SYNTAX;
        d.line_number = line;
        d.column_number = start_col;
        d.statement_index = -1;
        d.command_text = line_snippet(line, start_col);
        d.code = "ERR_TOKEN_TOO_LONG";
        d.message = "Identifier exceeds maximum token length (" +
          std::to_string(DSLParser::kMaxTokenLength) + " chars)";
        sink.add(std::move(d));
        if (!sink.can_accept_more()) {
          sink.mark_stopped_early();
          break;
        }
        continue;
      }
      if (!push_token({TokenKind::IDENT, source.substr(start, lex_len), 0.0, line, start_col})) {
        break;
      }
      continue;
    }

    D d;
    d.severity = D::SEVERITY_ERROR;
    d.stage = D::STAGE_LEX_SYNTAX;
    d.line_number = line;
    d.column_number = start_col;
    d.statement_index = -1;
    d.command_text = line_snippet(line, start_col);
    d.code = "ERR_UNEXPECTED_CHAR";
    d.message = "Unexpected character " + format_unexpected_char(c) + " in source";
    sink.add(std::move(d));
    ++i;
    ++col;
    if (!sink.can_accept_more()) {
      sink.mark_stopped_early();
      break;
    }
  }

  tokens.push_back({TokenKind::END_OF_FILE, "<EOF>", 0.0, line, col});
  return tokens;
}

Eigen::Matrix3d rpy_to_rotation(double r, double p, double y)
{
  const Eigen::AngleAxisd rx(r, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(p, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rz(y, Eigen::Vector3d::UnitZ());
  return (rz * ry * rx).toRotationMatrix();
}

class ParserImpl
{
public:
  using D = DSLParser::Diagnostic;
  ParserImpl(
    const std::string & source,
    const std::vector<Token> & tokens,
    const std::vector<LineSpan> & lines)
  : source_(source), tokens_(tokens), lines_(lines)
  {
  }

  void parse_program(ASTProgram & prog, DiagnosticSink & sink)
  {
    while (!at_eof()) {
      if (!sink.can_accept_more()) {
        sink.mark_stopped_early();
        break;
      }
      try {
        parse_top_level(prog);
      } catch (const ParseError & err) {
        record_error(err, sink);
        if (!sink.can_accept_more()) {
          sink.mark_stopped_early();
          break;
        }
        synchronize();
      }
    }
    prog.total_leaf_statements = next_stmt_index_;
  }

private:
  const std::string & source_;
  const std::vector<Token> & tokens_;
  const std::vector<LineSpan> & lines_;
  size_t pos_ = 0;
  int next_stmt_index_ = 0;
  size_t total_parsed_statements_ = 0;
  std::map<std::string, std::array<double, 6>> joint_consts_;
  std::map<std::string, Eigen::Matrix4d> pose_consts_;

  void consume_statement_budget(const Token & kw, int stmt_idx)
  {
    if (total_parsed_statements_ >= DSLParser::kMaxStatements) {
      throw ParseError(
        D::STAGE_SEMANTIC, kw.line, kw.column, stmt_idx, "ERR_STATEMENT_LIMIT",
        "Program exceeds maximum statement budget (" +
          std::to_string(DSLParser::kMaxStatements) + " statements)");
    }
    ++total_parsed_statements_;
  }

  const Token & peek(size_t offset = 0) const
  {
    if (pos_ + offset >= tokens_.size()) {
      return tokens_.back();
    }
    return tokens_[pos_ + offset];
  }

  bool at_eof() const { return peek().kind == TokenKind::END_OF_FILE; }

  const Token & advance()
  {
    const Token & tok = peek();
    if (!at_eof()) {
      ++pos_;
    }
    return tok;
  }

  bool match(TokenKind kind)
  {
    if (peek().kind == kind) {
      advance();
      return true;
    }
    return false;
  }

  const Token & expect(TokenKind kind, const std::string & expected_desc, int stmt_idx)
  {
    const Token & tok = peek();
    if (tok.kind != kind) {
      throw ParseError(
        D::STAGE_LEX_SYNTAX, tok.line, tok.column, stmt_idx, "ERR_SYNTAX",
        "Expected " + expected_desc + ", but found '" + tok.lexeme + "'");
    }
    return advance();
  }

  std::string line_snippet(int ln, int col = 1) const
  {
    return extract_line_snippet(source_, lines_, ln, col);
  }

  void record_error(const ParseError & err, DiagnosticSink & sink) const
  {
    D d;
    d.severity = D::SEVERITY_ERROR;
    d.stage = err.stage;
    d.line_number = err.line;
    d.column_number = err.column;
    d.statement_index = err.stmt_idx;
    d.command_text = line_snippet(err.line, err.column);
    d.code = err.code;
    d.message = err.what();
    sink.add(std::move(d));
  }

  void synchronize()
  {
    const int err_line = peek().line;
    while (!at_eof()) {
      if (peek().kind == TokenKind::RBRACE) {
        advance();
        return;
      }
      if (peek().line > err_line && peek().kind == TokenKind::IDENT) {
        const std::string & id = peek().lexeme;
        if (id == "joint" || id == "pose" || id == "MoveJ" || id == "MoveP" ||
            id == "MoveL" || id == "MoveC" || id == "MovePVA" ||
            id == "ImpedanceCartesian" || id == "Gripper" || id == "Wait" ||
            id == "SetPayload") {
          return;
        }
      }
      advance();
    }
  }

  double parse_optional_angle_unit(int stmt_idx)
  {
    const int prev_line = pos_ > 0 ? tokens_[pos_ - 1].line : peek().line;
    if (peek().kind == TokenKind::IDENT && peek().line == prev_line) {
      if (peek().lexeme == "rad") {
        advance();
        return 1.0;
      }
      if (peek().lexeme == "deg") {
        advance();
        return kDegToRad;
      }
      throw ParseError(
        D::STAGE_LEX_SYNTAX, peek().line, peek().column, stmt_idx, "ERR_UNIT",
        "Unknown unit '" + peek().lexeme + "'; expected 'rad' or 'deg'");
    }
    return 1.0;
  }

  template <size_t N>
  std::array<double, N> parse_fixed_vector(int stmt_idx)
  {
    const Token & open = expect(TokenKind::LBRACKET, "'[' to start vector", stmt_idx);
    std::array<double, N> out{};
    for (size_t i = 0; i < N; ++i) {
      if (i > 0) {
        expect(TokenKind::COMMA, "',' between vector elements", stmt_idx);
      }
      const Token & num = expect(
        TokenKind::NUMBER,
        "numeric element " + std::to_string(i + 1) + " of " + std::to_string(N) + "-vector",
        stmt_idx);
      out[i] = num.number;
    }
    if (peek().kind == TokenKind::COMMA && peek(1).kind == TokenKind::NUMBER) {
      throw ParseError(
        D::STAGE_SEMANTIC, peek().line, peek().column, stmt_idx, "ERR_VECTOR_DIM",
        "Vector starting at line " + std::to_string(open.line) + " requires exactly " +
          std::to_string(N) + " elements");
    }
    expect(TokenKind::RBRACKET, "']' to close vector", stmt_idx);
    return out;
  }

  std::array<double, 6> parse_joint_value(int stmt_idx)
  {
    if (peek().kind == TokenKind::IDENT) {
      const Token & id = advance();
      auto it = joint_consts_.find(id.lexeme);
      if (it == joint_consts_.end()) {
        if (pose_consts_.count(id.lexeme)) {
          throw ParseError(
            D::STAGE_SEMANTIC, id.line, id.column, stmt_idx, "ERR_TYPE_MISMATCH",
            "Identifier '" + id.lexeme + "' is a Cartesian pose, not a 6-axis joint target");
        }
        throw ParseError(
          D::STAGE_SEMANTIC, id.line, id.column, stmt_idx, "ERR_UNDEFINED_SYMBOL",
          "Undefined joint constant '" + id.lexeme + "'");
      }
      return it->second;
    }
    auto vec = parse_fixed_vector<6>(stmt_idx);
    const double scale = parse_optional_angle_unit(stmt_idx);
    for (double & v : vec) {
      v *= scale;
    }
    return vec;
  }

  Eigen::Matrix4d parse_pose_value(int stmt_idx)
  {
    if (peek().kind == TokenKind::IDENT && peek().lexeme != "Pose") {
      const Token & id = advance();
      auto it = pose_consts_.find(id.lexeme);
      if (it == pose_consts_.end()) {
        if (joint_consts_.count(id.lexeme)) {
          throw ParseError(
            D::STAGE_SEMANTIC, id.line, id.column, stmt_idx, "ERR_TYPE_MISMATCH",
            "Identifier '" + id.lexeme + "' is a joint vector, not a Cartesian pose");
        }
        throw ParseError(
          D::STAGE_SEMANTIC, id.line, id.column, stmt_idx, "ERR_UNDEFINED_SYMBOL",
          "Undefined pose constant '" + id.lexeme + "'");
      }
      return it->second;
    }

    if (peek().kind == TokenKind::IDENT && peek().lexeme == "Pose") {
      const Token & pose_kw = advance();
      expect(TokenKind::LPAREN, "'(' after Pose", stmt_idx);
      bool has_xyz = false;
      bool has_ori = false;
      Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
      Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();

      while (peek().kind != TokenKind::RPAREN && !at_eof()) {
        const Token & key = expect(TokenKind::IDENT, "Pose parameter name (xyz/rpy/quat)", stmt_idx);
        expect(TokenKind::EQUALS, "'=' after Pose parameter '" + key.lexeme + "'", stmt_idx);
        if (key.lexeme == "xyz") {
          const auto v = parse_fixed_vector<3>(stmt_idx);
          xyz = Eigen::Vector3d(v[0], v[1], v[2]);
          has_xyz = true;
        } else if (key.lexeme == "rpy") {
          auto v = parse_fixed_vector<3>(stmt_idx);
          const double scale = parse_optional_angle_unit(stmt_idx);
          rot = rpy_to_rotation(v[0] * scale, v[1] * scale, v[2] * scale);
          has_ori = true;
        } else if (key.lexeme == "quat") {
          const auto qv = parse_fixed_vector<4>(stmt_idx);
          // Convention: [qx, qy, qz, qw]
          Eigen::Quaterniond q(qv[3], qv[0], qv[1], qv[2]);
          const double nrm = q.norm();
          if (nrm < 1e-6 || std::abs(nrm - 1.0) > 1e-2) {
            throw ParseError(
              D::STAGE_SEMANTIC, key.line, key.column, stmt_idx, "ERR_INVALID_QUAT",
              "Quaternion [qx, qy, qz, qw] norm=" + std::to_string(nrm) +
                " is not unit-normalized");
          }
          q.normalize();
          rot = q.toRotationMatrix();
          has_ori = true;
        } else {
          throw ParseError(
            D::STAGE_SEMANTIC, key.line, key.column, stmt_idx, "ERR_UNKNOWN_PARAM",
            "Unknown Pose parameter '" + key.lexeme + "'; expected 'xyz', 'rpy', or 'quat'");
        }
        if (!match(TokenKind::COMMA)) {
          break;
        }
      }
      expect(TokenKind::RPAREN, "')' to close Pose(...)", stmt_idx);
      if (!has_xyz || !has_ori) {
        throw ParseError(
          D::STAGE_SEMANTIC, pose_kw.line, pose_kw.column, stmt_idx, "ERR_INCOMPLETE_POSE",
          "Pose(...) requires both 'xyz' and orientation ('rpy' or 'quat')");
      }
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T.topLeftCorner<3, 3>() = rot;
      T.topRightCorner<3, 1>() = xyz;
      return T;
    }

    // Shorthand 6-vector: [x, y, z, r, p, y] [rad|deg]
    const auto v = parse_fixed_vector<6>(stmt_idx);
    const double scale = parse_optional_angle_unit(stmt_idx);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3, 3>() = rpy_to_rotation(v[3] * scale, v[4] * scale, v[5] * scale);
    T.topRightCorner<3, 1>() = Eigen::Vector3d(v[0], v[1], v[2]);
    return T;
  }

  void validate_motion_option(
    const Token & key, double val, MotionOptions & opts, int stmt_idx)
  {
    if (key.lexeme == "vel") {
      if (val < 0.01 || val > 1.0) {
        throw ParseError(
          D::STAGE_SEMANTIC, key.line, key.column, stmt_idx, "ERR_VEL_SCALE_RANGE",
          "Parameter 'vel'=" + std::to_string(val) + " must lie in [0.01, 1.0]");
      }
      opts.vel_scale = val;
    } else if (key.lexeme == "acc") {
      if (val < 0.01 || val > 1.0) {
        throw ParseError(
          D::STAGE_SEMANTIC, key.line, key.column, stmt_idx, "ERR_ACC_SCALE_RANGE",
          "Parameter 'acc'=" + std::to_string(val) + " must lie in [0.01, 1.0]");
      }
      opts.acc_scale = val;
    } else if (key.lexeme == "jerk") {
      if (val < 0.01 || val > 1.0) {
        throw ParseError(
          D::STAGE_SEMANTIC, key.line, key.column, stmt_idx, "ERR_JERK_SCALE_RANGE",
          "Parameter 'jerk'=" + std::to_string(val) + " must lie in [0.01, 1.0]");
      }
      opts.jerk_scale = val;
    } else if (key.lexeme == "blend") {
      if (val < 0.0 || val > 0.5) {
        throw ParseError(
          D::STAGE_SEMANTIC, key.line, key.column, stmt_idx, "ERR_BLEND_RANGE",
          "Parameter 'blend'=" + std::to_string(val) + " must lie in [0.0, 0.5]");
      }
      opts.blend_radius = val;
    } else {
      throw ParseError(
        D::STAGE_SEMANTIC, key.line, key.column, stmt_idx, "ERR_UNKNOWN_PARAM",
        "Unknown motion parameter '" + key.lexeme + "'");
    }
  }

  ASTStatement parse_motion_stmt(const Token & kw, bool inside_impedance)
  {
    const int stmt_idx = next_stmt_index_++;
    if (inside_impedance && (kw.lexeme == "MoveJ" || kw.lexeme == "MoveP")) {
      throw ParseError(
        D::STAGE_SEMANTIC, kw.line, kw.column, stmt_idx, "ERR_IMPEDANCE_SPACE",
        kw.lexeme + " is a joint-space motion and is not permitted inside ImpedanceCartesian block");
    }

    expect(TokenKind::LPAREN, "'(' after " + kw.lexeme, stmt_idx);
    MotionStatement m;
    m.loc = {kw.line, kw.column, stmt_idx, line_snippet(kw.line)};

    if (kw.lexeme == "MoveJ" || kw.lexeme == "MoveP" || kw.lexeme == "MoveL") {
      m.type = (kw.lexeme == "MoveJ") ? MotionCommandType::MOVE_J :
               (kw.lexeme == "MoveP") ? MotionCommandType::MOVE_P :
                                        MotionCommandType::MOVE_L;
      bool has_target = false;
      while (peek().kind != TokenKind::RPAREN && !at_eof()) {
        if (peek().kind == TokenKind::IDENT && peek(1).kind == TokenKind::EQUALS) {
          const Token & key = advance();
          advance();  // '='
          if (key.lexeme == "target") {
            if (m.type == MotionCommandType::MOVE_J) {
              m.joint_target = parse_joint_value(stmt_idx);
            } else {
              m.pose_target = parse_pose_value(stmt_idx);
            }
            has_target = true;
          } else {
            const Token & num = expect(TokenKind::NUMBER, "numeric option value", stmt_idx);
            validate_motion_option(key, num.number, m.options, stmt_idx);
          }
        } else if (!has_target) {
          if (m.type == MotionCommandType::MOVE_J) {
            m.joint_target = parse_joint_value(stmt_idx);
          } else {
            m.pose_target = parse_pose_value(stmt_idx);
          }
          has_target = true;
        } else {
          throw ParseError(
            D::STAGE_LEX_SYNTAX, peek().line, peek().column, stmt_idx, "ERR_SYNTAX",
            "Expected named option 'key=value' in " + kw.lexeme);
        }
        if (!match(TokenKind::COMMA)) {
          break;
        }
      }
      expect(TokenKind::RPAREN, "')' to close " + kw.lexeme, stmt_idx);
      if (!has_target) {
        throw ParseError(
          D::STAGE_SEMANTIC, kw.line, kw.column, stmt_idx, "ERR_MISSING_TARGET",
          kw.lexeme + " is missing required 'target' argument");
      }
    } else if (kw.lexeme == "MoveC") {
      bool has_via = false;
      bool has_target = false;
      bool has_center = false;
      bool has_axis = false;
      bool has_angle = false;

      while (peek().kind != TokenKind::RPAREN && !at_eof()) {
        const Token & key = expect(TokenKind::IDENT, "MoveC parameter name", stmt_idx);
        expect(TokenKind::EQUALS, "'=' after MoveC parameter '" + key.lexeme + "'", stmt_idx);
        if (key.lexeme == "via") {
          if (peek().kind == TokenKind::LBRACKET) {
            const auto p = parse_fixed_vector<3>(stmt_idx);
            m.pose_via = Eigen::Matrix4d::Identity();
            m.pose_via(0, 3) = p[0];
            m.pose_via(1, 3) = p[1];
            m.pose_via(2, 3) = p[2];
            m.via_has_orientation = false;
          } else {
            m.pose_via = parse_pose_value(stmt_idx);
            m.via_has_orientation = true;
          }
          has_via = true;
        } else if (key.lexeme == "target") {
          m.pose_target = parse_pose_value(stmt_idx);
          has_target = true;
        } else if (key.lexeme == "center") {
          const auto c = parse_fixed_vector<3>(stmt_idx);
          m.circle_center = Eigen::Vector3d(c[0], c[1], c[2]);
          has_center = true;
        } else if (key.lexeme == "axis") {
          const auto a = parse_fixed_vector<3>(stmt_idx);
          m.circle_axis = Eigen::Vector3d(a[0], a[1], a[2]);
          if (m.circle_axis.norm() < 1e-6) {
            throw ParseError(
              D::STAGE_SEMANTIC, key.line, key.column, stmt_idx, "ERR_ZERO_AXIS",
              "MoveC rotation 'axis' vector cannot be zero");
          }
          m.circle_axis.normalize();
          has_axis = true;
        } else if (key.lexeme == "angle") {
          const Token & ang = expect(TokenKind::NUMBER, "circle sweep angle", stmt_idx);
          const double scale = parse_optional_angle_unit(stmt_idx);
          m.circle_angle_rad = ang.number * scale;
          if (std::abs(m.circle_angle_rad) < 1e-5 ||
              std::abs(m.circle_angle_rad) > 2.0 * 3.14159265358979323846 + 1e-4) {
            throw ParseError(
              D::STAGE_SEMANTIC, key.line, key.column, stmt_idx, "ERR_CIRCLE_ANGLE",
              "MoveC 'angle'=" + std::to_string(m.circle_angle_rad) +
                " rad must be non-zero and within [-2*pi, 2*pi]");
          }
          has_angle = true;
        } else if (key.lexeme == "target_ori") {
          const Eigen::Matrix4d T_ori = parse_pose_value(stmt_idx);
          m.circle_target_rot = T_ori.topLeftCorner<3, 3>();
          m.has_explicit_target_ori = true;
        } else {
          const Token & num = expect(TokenKind::NUMBER, "numeric option value", stmt_idx);
          validate_motion_option(key, num.number, m.options, stmt_idx);
        }
        if (!match(TokenKind::COMMA)) {
          break;
        }
      }
      expect(TokenKind::RPAREN, "')' to close MoveC", stmt_idx);

      if (has_via && has_target && !has_center && !has_axis && !has_angle) {
        m.type = MotionCommandType::MOVE_C_THREE_POINT;
      } else if (has_center && has_axis && has_angle && !has_via && !has_target) {
        m.type = MotionCommandType::MOVE_C_CENTER_ANGLE;
      } else {
        throw ParseError(
          D::STAGE_SEMANTIC, kw.line, kw.column, stmt_idx, "ERR_MOVEC_ARGS",
          "MoveC must specify either (via=..., target=...) OR (center=..., axis=..., angle=...)");
      }
    }

    ASTStatement st;
    st.kind = StatementKind::MOTION;
    st.loc = m.loc;
    st.motion = m;
    return st;
  }

  ASTStatement parse_move_pva_stmt(const Token & kw, bool inside_impedance)
  {
    const int stmt_idx = next_stmt_index_++;
    expect(TokenKind::LPAREN, "'(' after MovePVA", stmt_idx);
    MovePVAStatement pva;
    pva.loc = {kw.line, kw.column, stmt_idx, line_snippet(kw.line)};
    bool has_space = false;

    while (peek().kind != TokenKind::RPAREN && !at_eof()) {
      const Token & key = expect(TokenKind::IDENT, "MovePVA parameter name", stmt_idx);
      expect(TokenKind::EQUALS, "'=' after '" + key.lexeme + "'", stmt_idx);
      if (key.lexeme == "space") {
        const Token & sp = expect(TokenKind::IDENT, "'JOINT' or 'CARTESIAN'", stmt_idx);
        if (sp.lexeme == "JOINT") {
          pva.space = SpaceType::JOINT;
        } else if (sp.lexeme == "CARTESIAN") {
          pva.space = SpaceType::CARTESIAN;
        } else {
          throw ParseError(
            D::STAGE_SEMANTIC, sp.line, sp.column, stmt_idx, "ERR_PVA_SPACE",
            "Unknown MovePVA space '" + sp.lexeme + "'; expected 'JOINT' or 'CARTESIAN'");
        }
        has_space = true;
      } else {
        const Token & num = expect(TokenKind::NUMBER, "numeric option value", stmt_idx);
        validate_motion_option(key, num.number, pva.options, stmt_idx);
      }
      if (!match(TokenKind::COMMA)) {
        break;
      }
    }
    expect(TokenKind::RPAREN, "')' after MovePVA options", stmt_idx);
    if (!has_space) {
      throw ParseError(
        D::STAGE_SEMANTIC, kw.line, kw.column, stmt_idx, "ERR_MISSING_SPACE",
        "MovePVA requires 'space=JOINT' or 'space=CARTESIAN'");
    }
    if (inside_impedance && pva.space == SpaceType::JOINT) {
      throw ParseError(
        D::STAGE_SEMANTIC, kw.line, kw.column, stmt_idx, "ERR_IMPEDANCE_SPACE",
        "MovePVA(space=JOINT) is not permitted inside ImpedanceCartesian; use space=CARTESIAN");
    }

    expect(TokenKind::LBRACE, "'{' to open MovePVA waypoint block", stmt_idx);
    while (peek().kind != TokenKind::RBRACE && !at_eof()) {
      const Token & pt_kw = expect(TokenKind::IDENT, "'point(...)' in MovePVA block", stmt_idx);
      if (pt_kw.lexeme != "point") {
        throw ParseError(
          D::STAGE_LEX_SYNTAX, pt_kw.line, pt_kw.column, stmt_idx, "ERR_SYNTAX",
          "Expected 'point(...)' inside MovePVA block, found '" + pt_kw.lexeme + "'");
      }
      if (pva.waypoints.size() >= DSLParser::kMaxPvaWaypointsPerStatement) {
        throw ParseError(
          D::STAGE_SEMANTIC, pt_kw.line, pt_kw.column, stmt_idx, "ERR_PVA_WAYPOINT_LIMIT",
          "MovePVA block exceeds maximum waypoint budget (" +
            std::to_string(DSLParser::kMaxPvaWaypointsPerStatement) + " waypoints)");
      }
      expect(TokenKind::LPAREN, "'(' after point", stmt_idx);
      PVAWaypoint wp;
      wp.loc = {pt_kw.line, pt_kw.column, stmt_idx, line_snippet(pt_kw.line)};
      bool has_p = false;

      while (peek().kind != TokenKind::RPAREN && !at_eof()) {
        const Token & k = expect(TokenKind::IDENT, "point field (p/v/a/dt)", stmt_idx);
        expect(TokenKind::EQUALS, "'=' after '" + k.lexeme + "'", stmt_idx);
        if (k.lexeme == "p") {
          if (pva.space == SpaceType::JOINT) {
            wp.joint_pos = parse_joint_value(stmt_idx);
          } else {
            wp.cart_pose = parse_pose_value(stmt_idx);
          }
          has_p = true;
        } else if (k.lexeme == "v") {
          wp.velocity = parse_fixed_vector<6>(stmt_idx);
        } else if (k.lexeme == "a") {
          wp.acceleration = parse_fixed_vector<6>(stmt_idx);
        } else if (k.lexeme == "dt") {
          const Token & dt_tok = expect(TokenKind::NUMBER, "segment duration dt [s]", stmt_idx);
          if (dt_tok.number <= 0.005 || dt_tok.number > 120.0) {
            throw ParseError(
              D::STAGE_SEMANTIC, dt_tok.line, dt_tok.column, stmt_idx, "ERR_PVA_DT",
              "Waypoint 'dt'=" + std::to_string(dt_tok.number) + " s must lie in (0.005, 120.0]");
          }
          wp.dt = dt_tok.number;
        } else {
          throw ParseError(
            D::STAGE_SEMANTIC, k.line, k.column, stmt_idx, "ERR_UNKNOWN_PARAM",
            "Unknown waypoint parameter '" + k.lexeme + "'; expected 'p', 'v', 'a', or 'dt'");
        }
        if (!match(TokenKind::COMMA)) {
          break;
        }
      }
      expect(TokenKind::RPAREN, "')' to close point(...)", stmt_idx);
      if (!has_p) {
        throw ParseError(
          D::STAGE_SEMANTIC, pt_kw.line, pt_kw.column, stmt_idx, "ERR_MISSING_WAYPOINT_POS",
          "point(...) is missing required position 'p=...'");
      }
      if (wp.acceleration.has_value() && !wp.velocity.has_value()) {
        throw ParseError(
          D::STAGE_SEMANTIC, pt_kw.line, pt_kw.column, stmt_idx, "ERR_PVA_MISSING_VEL",
          "point(...) specifies acceleration 'a' without velocity 'v'; provide (P), (P,V), or (P,V,A)");
      }
      pva.waypoints.push_back(wp);
    }
    expect(TokenKind::RBRACE, "'}' to close MovePVA block", stmt_idx);
    if (pva.waypoints.empty()) {
      throw ParseError(
        D::STAGE_SEMANTIC, kw.line, kw.column, stmt_idx, "ERR_EMPTY_PVA",
        "MovePVA block must contain at least one point(...) entry");
    }

    ASTStatement st;
    st.kind = StatementKind::MOVE_PVA;
    st.loc = pva.loc;
    st.move_pva = pva;
    return st;
  }

  std::array<double, 3> parse_scalar_or_vec3(int stmt_idx)
  {
    if (peek().kind == TokenKind::LBRACKET) {
      return parse_fixed_vector<3>(stmt_idx);
    }
    const Token & num = expect(TokenKind::NUMBER, "scalar or 3-vector gain value", stmt_idx);
    return {{num.number, num.number, num.number}};
  }

  ASTStatement parse_impedance_block(const Token & kw)
  {
    const int block_idx = next_stmt_index_;
    expect(TokenKind::LPAREN, "'(' after ImpedanceCartesian", block_idx);
    ImpedanceParams params;

    while (peek().kind != TokenKind::RPAREN && !at_eof()) {
      const Token & key = expect(TokenKind::IDENT, "ImpedanceCartesian gain parameter", block_idx);
      expect(TokenKind::EQUALS, "'=' after '" + key.lexeme + "'", block_idx);
      if (key.lexeme == "kt") {
        params.kt = parse_scalar_or_vec3(block_idx);
        for (double v : params.kt) {
          if (v < 0.0 || v > 1000.0) {
            throw ParseError(
              D::STAGE_SEMANTIC, key.line, key.column, block_idx, "ERR_IMPEDANCE_GAIN",
              "Translational stiffness 'kt'=" + std::to_string(v) + " N/m must lie in [0.0, 1000.0]");
          }
        }
      } else if (key.lexeme == "dt" || key.lexeme == "zeta_t") {
        params.dt = parse_scalar_or_vec3(block_idx);
        for (double v : params.dt) {
          if (v < 0.1 || v > 2.0) {
            throw ParseError(
              D::STAGE_SEMANTIC, key.line, key.column, block_idx, "ERR_IMPEDANCE_GAIN",
              "Translational damping ratio '" + key.lexeme + "'=" + std::to_string(v) +
                " must lie in [0.1, 2.0]");
          }
        }
      } else if (key.lexeme == "kr") {
        params.kr = parse_scalar_or_vec3(block_idx);
        for (double v : params.kr) {
          if (v < 0.0 || v > 50.0) {
            throw ParseError(
              D::STAGE_SEMANTIC, key.line, key.column, block_idx, "ERR_IMPEDANCE_GAIN",
              "Rotational stiffness 'kr'=" + std::to_string(v) + " Nm/rad must lie in [0.0, 50.0]");
          }
        }
      } else if (key.lexeme == "dr" || key.lexeme == "zeta_r") {
        params.dr = parse_scalar_or_vec3(block_idx);
        for (double v : params.dr) {
          if (v < 0.1 || v > 2.0) {
            throw ParseError(
              D::STAGE_SEMANTIC, key.line, key.column, block_idx, "ERR_IMPEDANCE_GAIN",
              "Rotational damping ratio '" + key.lexeme + "'=" + std::to_string(v) +
                " must lie in [0.1, 2.0]");
          }
        }
      } else if (key.lexeme == "null_damping") {
        const Token & num = expect(TokenKind::NUMBER, "null_damping value", block_idx);
        if (num.number < 0.0 || num.number > 20.0) {
          throw ParseError(
            D::STAGE_SEMANTIC, key.line, key.column, block_idx, "ERR_IMPEDANCE_GAIN",
            "Parameter 'null_damping'=" + std::to_string(num.number) + " must lie in [0.0, 20.0]");
        }
        params.null_damping = num.number;
      } else if (key.lexeme == "transition_time") {
        const Token & num = expect(TokenKind::NUMBER, "transition_time [s]", block_idx);
        if (num.number < 0.01 || num.number > 5.0) {
          throw ParseError(
            D::STAGE_SEMANTIC, key.line, key.column, block_idx, "ERR_IMPEDANCE_GAIN",
            "Parameter 'transition_time'=" + std::to_string(num.number) + " s must lie in [0.01, 5.0]");
        }
        params.transition_time = num.number;
      } else if (key.lexeme == "wrench_ff") {
        params.wrench_ff = parse_fixed_vector<6>(block_idx);
        for (int i = 0; i < 3; ++i) {
          if (std::abs(params.wrench_ff[static_cast<size_t>(i)]) > 30.0) {
            throw ParseError(
              D::STAGE_SEMANTIC, key.line, key.column, block_idx, "ERR_IMPEDANCE_GAIN",
              "Feedforward force component in 'wrench_ff' must lie in [-30.0, 30.0] N");
          }
          if (std::abs(params.wrench_ff[static_cast<size_t>(i + 3)]) > 5.0) {
            throw ParseError(
              D::STAGE_SEMANTIC, key.line, key.column, block_idx, "ERR_IMPEDANCE_GAIN",
              "Feedforward torque component in 'wrench_ff' must lie in [-5.0, 5.0] Nm");
          }
        }
      } else if (key.lexeme == "hold") {
        const Token & b = expect(TokenKind::IDENT, "'true' or 'false'", block_idx);
        if (b.lexeme == "true") {
          params.end_behavior = 1;
        } else if (b.lexeme == "false") {
          params.end_behavior = 0;
        } else {
          throw ParseError(
            D::STAGE_SEMANTIC, b.line, b.column, block_idx, "ERR_BOOL_VALUE",
            "Parameter 'hold' requires 'true' or 'false'");
        }
      } else {
        throw ParseError(
          D::STAGE_SEMANTIC, key.line, key.column, block_idx, "ERR_UNKNOWN_PARAM",
          "Unknown ImpedanceCartesian parameter '" + key.lexeme + "'");
      }
      if (!match(TokenKind::COMMA)) {
        break;
      }
    }
    expect(TokenKind::RPAREN, "')' after ImpedanceCartesian parameters", block_idx);
    expect(TokenKind::LBRACE, "'{' to open ImpedanceCartesian block", block_idx);

    ASTStatement block_stmt;
    block_stmt.kind = StatementKind::IMPEDANCE_BLOCK;
    block_stmt.loc = {kw.line, kw.column, block_idx, line_snippet(kw.line)};
    block_stmt.impedance_params = params;

    while (peek().kind != TokenKind::RBRACE && !at_eof()) {
      const Token & inner_kw = expect(
        TokenKind::IDENT, "statement inside ImpedanceCartesian block", next_stmt_index_);
      consume_statement_budget(inner_kw, next_stmt_index_);
      if (inner_kw.lexeme == "MoveL" || inner_kw.lexeme == "MoveC" ||
          inner_kw.lexeme == "MoveJ" || inner_kw.lexeme == "MoveP") {
        block_stmt.body.push_back(parse_motion_stmt(inner_kw, true));
      } else if (inner_kw.lexeme == "MovePVA") {
        block_stmt.body.push_back(parse_move_pva_stmt(inner_kw, true));
      } else if (inner_kw.lexeme == "Gripper") {
        block_stmt.body.push_back(parse_gripper_stmt(inner_kw));
      } else if (inner_kw.lexeme == "Wait") {
        block_stmt.body.push_back(parse_wait_stmt(inner_kw));
      } else if (inner_kw.lexeme == "SetPayload") {
        block_stmt.body.push_back(parse_set_payload_stmt(inner_kw));
      } else {
        throw ParseError(
          D::STAGE_SEMANTIC, inner_kw.line, inner_kw.column, next_stmt_index_,
          "ERR_INVALID_IMPEDANCE_STMT",
          "Statement '" + inner_kw.lexeme + "' is not allowed inside ImpedanceCartesian block");
      }
    }
    expect(TokenKind::RBRACE, "'}' to close ImpedanceCartesian block", block_idx);
    if (block_stmt.body.empty()) {
      throw ParseError(
        D::STAGE_SEMANTIC, kw.line, kw.column, block_idx, "ERR_EMPTY_BLOCK",
        "ImpedanceCartesian block cannot be empty");
    }
    return block_stmt;
  }

  ASTStatement parse_gripper_stmt(const Token & kw)
  {
    const int stmt_idx = next_stmt_index_++;
    expect(TokenKind::LPAREN, "'(' after Gripper", stmt_idx);
    GripperStatement g;
    g.loc = {kw.line, kw.column, stmt_idx, line_snippet(kw.line)};
    bool has_action = false;

    while (peek().kind != TokenKind::RPAREN && !at_eof()) {
      if (peek().kind == TokenKind::IDENT && peek(1).kind == TokenKind::EQUALS) {
        const Token & key = advance();
        advance();  // '='
        if (key.lexeme == "action") {
          const Token & act = expect(TokenKind::IDENT, "'OPEN', 'CLOSE', or 'STOP'", stmt_idx);
          if (act.lexeme == "OPEN") {
            g.action = GripperActionType::OPEN;
          } else if (act.lexeme == "CLOSE") {
            g.action = GripperActionType::CLOSE;
          } else if (act.lexeme == "STOP") {
            g.action = GripperActionType::STOP;
          } else {
            throw ParseError(
              D::STAGE_SEMANTIC, act.line, act.column, stmt_idx, "ERR_GRIPPER_ACTION",
              "Invalid Gripper action '" + act.lexeme + "'; expected 'OPEN', 'CLOSE', or 'STOP'");
          }
          has_action = true;
        } else if (key.lexeme == "wait") {
          const Token & b = expect(TokenKind::IDENT, "'true' or 'false'", stmt_idx);
          if (b.lexeme == "true") {
            g.wait_for_completion = true;
          } else if (b.lexeme == "false") {
            g.wait_for_completion = false;
          } else {
            throw ParseError(
              D::STAGE_SEMANTIC, b.line, b.column, stmt_idx, "ERR_BOOL_VALUE",
              "Parameter 'wait' requires 'true' or 'false'");
          }
        } else if (key.lexeme == "timeout") {
          const Token & num = expect(TokenKind::NUMBER, "timeout [s]", stmt_idx);
          if (num.number <= 0.1 || num.number > 30.0) {
            throw ParseError(
              D::STAGE_SEMANTIC, num.line, num.column, stmt_idx, "ERR_TIMEOUT_RANGE",
              "Gripper 'timeout'=" + std::to_string(num.number) + " s must lie in (0.1, 30.0]");
          }
          g.timeout_sec = num.number;
        } else {
          throw ParseError(
            D::STAGE_SEMANTIC, key.line, key.column, stmt_idx, "ERR_UNKNOWN_PARAM",
            "Unknown Gripper parameter '" + key.lexeme + "'");
        }
      } else if (!has_action && peek().kind == TokenKind::IDENT) {
        const Token & act = advance();
        if (act.lexeme == "OPEN") {
          g.action = GripperActionType::OPEN;
        } else if (act.lexeme == "CLOSE") {
          g.action = GripperActionType::CLOSE;
        } else if (act.lexeme == "STOP") {
          g.action = GripperActionType::STOP;
        } else {
          throw ParseError(
            D::STAGE_SEMANTIC, act.line, act.column, stmt_idx, "ERR_GRIPPER_ACTION",
            "Invalid Gripper action '" + act.lexeme + "'; expected 'OPEN', 'CLOSE', or 'STOP'");
        }
        has_action = true;
      } else {
        throw ParseError(
          D::STAGE_LEX_SYNTAX, peek().line, peek().column, stmt_idx, "ERR_SYNTAX",
          "Invalid argument in Gripper(...)");
      }
      if (!match(TokenKind::COMMA)) {
        break;
      }
    }
    expect(TokenKind::RPAREN, "')' to close Gripper(...)", stmt_idx);
    if (!has_action) {
      throw ParseError(
        D::STAGE_SEMANTIC, kw.line, kw.column, stmt_idx, "ERR_MISSING_ACTION",
        "Gripper(...) requires 'action=OPEN|CLOSE|STOP'");
    }

    ASTStatement st;
    st.kind = StatementKind::GRIPPER;
    st.loc = g.loc;
    st.gripper = g;
    return st;
  }

  ASTStatement parse_wait_stmt(const Token & kw)
  {
    const int stmt_idx = next_stmt_index_++;
    expect(TokenKind::LPAREN, "'(' after Wait", stmt_idx);
    if (peek().kind == TokenKind::IDENT && peek().lexeme == "duration" &&
        peek(1).kind == TokenKind::EQUALS) {
      advance();
      advance();
    }
    const Token & num = expect(TokenKind::NUMBER, "wait duration in seconds", stmt_idx);
    if (num.number <= 0.0 || num.number > 120.0) {
      throw ParseError(
        D::STAGE_SEMANTIC, num.line, num.column, stmt_idx, "ERR_WAIT_RANGE",
        "Wait duration=" + std::to_string(num.number) + " s must lie in (0.0, 120.0]");
    }
    expect(TokenKind::RPAREN, "')' to close Wait(...)", stmt_idx);
    WaitStatement w;
    w.loc = {kw.line, kw.column, stmt_idx, line_snippet(kw.line)};
    w.duration_sec = num.number;
    ASTStatement st;
    st.kind = StatementKind::WAIT;
    st.loc = w.loc;
    st.wait_stmt = w;
    return st;
  }

  ASTStatement parse_set_payload_stmt(const Token & kw)
  {
    const int stmt_idx = next_stmt_index_++;
    expect(TokenKind::LPAREN, "'(' after SetPayload", stmt_idx);
    PayloadStatement p;
    p.loc = {kw.line, kw.column, stmt_idx, line_snippet(kw.line)};
    bool has_mass = false;

    while (peek().kind != TokenKind::RPAREN && !at_eof()) {
      if (peek().kind == TokenKind::IDENT && peek(1).kind == TokenKind::EQUALS) {
        const Token & key = advance();
        advance();  // '='
        if (key.lexeme == "mass") {
          const Token & num = expect(TokenKind::NUMBER, "payload mass [kg]", stmt_idx);
          if (!std::isfinite(num.number) || num.number < lite6_interfaces::contract::kMinPayloadMassKg ||
              num.number > lite6_interfaces::contract::kMaxPayloadMassKg) {
            throw ParseError(
              D::STAGE_SEMANTIC, num.line, num.column, stmt_idx, "ERR_PAYLOAD_RANGE",
              "Payload 'mass'=" + std::to_string(num.number) + " kg exceeds the additional-payload simulation range [0.0, " +
                std::to_string(lite6_interfaces::contract::kMaxPayloadMassKg) + "]");
          }
          p.mass_kg = num.number;
          has_mass = true;
        } else if (key.lexeme == "com") {
          p.com_tcp = parse_fixed_vector<3>(stmt_idx);
          const double r2 =
            p.com_tcp[0] * p.com_tcp[0] +
            p.com_tcp[1] * p.com_tcp[1] +
            p.com_tcp[2] * p.com_tcp[2];
          if (!std::isfinite(r2) || r2 >
              lite6_interfaces::contract::kMaxPayloadComRadiusM * lite6_interfaces::contract::kMaxPayloadComRadiusM) {
            throw ParseError(
              D::STAGE_SEMANTIC, key.line, key.column, stmt_idx, "ERR_PAYLOAD_RANGE",
              "Payload 'com' offset norm=" + std::to_string(std::sqrt(r2)) +
                " m exceeds maximum 0.30 m");
          }
        } else {
          throw ParseError(
            D::STAGE_SEMANTIC, key.line, key.column, stmt_idx, "ERR_UNKNOWN_PARAM",
            "Unknown SetPayload parameter '" + key.lexeme + "'; expected 'mass' or 'com'");
        }
      } else if (!has_mass && peek().kind == TokenKind::NUMBER) {
        const Token & num = advance();
        if (!std::isfinite(num.number) || num.number < lite6_interfaces::contract::kMinPayloadMassKg ||
              num.number > lite6_interfaces::contract::kMaxPayloadMassKg) {
          throw ParseError(
            D::STAGE_SEMANTIC, num.line, num.column, stmt_idx, "ERR_PAYLOAD_RANGE",
            "Payload 'mass'=" + std::to_string(num.number) + " kg exceeds the additional-payload simulation range [0.0, " +
                std::to_string(lite6_interfaces::contract::kMaxPayloadMassKg) + "]");
        }
        p.mass_kg = num.number;
        has_mass = true;
      } else {
        throw ParseError(
          D::STAGE_LEX_SYNTAX, peek().line, peek().column, stmt_idx, "ERR_SYNTAX",
          "Invalid argument in SetPayload(...); expected mass=<kg>[, com=[x, y, z]]");
      }
      if (!match(TokenKind::COMMA)) {
        break;
      }
    }
    expect(TokenKind::RPAREN, "')' to close SetPayload(...)", stmt_idx);
    if (!has_mass) {
      throw ParseError(
        D::STAGE_SEMANTIC, kw.line, kw.column, stmt_idx, "ERR_MISSING_PAYLOAD_MASS",
        "SetPayload(...) requires 'mass=<kg>' (additional carried mass)");
    }

    ASTStatement st;
    st.kind = StatementKind::SET_PAYLOAD;
    st.loc = p.loc;
    st.payload_stmt = p;
    return st;
  }

  void parse_top_level(ASTProgram & prog)
  {
    const Token & kw = expect(TokenKind::IDENT, "statement or declaration keyword", -1);
    consume_statement_budget(kw, -1);
    if (kw.lexeme == "joint") {
      const Token & name = expect(TokenKind::IDENT, "joint constant identifier", -1);
      if (joint_consts_.count(name.lexeme) || pose_consts_.count(name.lexeme)) {
        throw ParseError(
          D::STAGE_SEMANTIC, name.line, name.column, -1, "ERR_DUPLICATE_SYMBOL",
          "Constant '" + name.lexeme + "' is already defined");
      }
      expect(TokenKind::EQUALS, "'=' after joint constant '" + name.lexeme + "'", -1);
      joint_consts_[name.lexeme] = parse_joint_value(-1);
    } else if (kw.lexeme == "pose") {
      const Token & name = expect(TokenKind::IDENT, "pose constant identifier", -1);
      if (joint_consts_.count(name.lexeme) || pose_consts_.count(name.lexeme)) {
        throw ParseError(
          D::STAGE_SEMANTIC, name.line, name.column, -1, "ERR_DUPLICATE_SYMBOL",
          "Constant '" + name.lexeme + "' is already defined");
      }
      expect(TokenKind::EQUALS, "'=' after pose constant '" + name.lexeme + "'", -1);
      pose_consts_[name.lexeme] = parse_pose_value(-1);
    } else if (
      kw.lexeme == "MoveJ" || kw.lexeme == "MoveP" ||
      kw.lexeme == "MoveL" || kw.lexeme == "MoveC") {
      prog.statements.push_back(parse_motion_stmt(kw, false));
    } else if (kw.lexeme == "MovePVA") {
      prog.statements.push_back(parse_move_pva_stmt(kw, false));
    } else if (kw.lexeme == "ImpedanceCartesian") {
      prog.statements.push_back(parse_impedance_block(kw));
    } else if (kw.lexeme == "Gripper") {
      prog.statements.push_back(parse_gripper_stmt(kw));
    } else if (kw.lexeme == "Wait") {
      prog.statements.push_back(parse_wait_stmt(kw));
    } else if (kw.lexeme == "SetPayload") {
      prog.statements.push_back(parse_set_payload_stmt(kw));
    } else {
      throw ParseError(
        D::STAGE_LEX_SYNTAX, kw.line, kw.column, -1, "ERR_UNKNOWN_COMMAND",
        "Unknown command or keyword '" + kw.lexeme + "'");
    }
  }
};

}  // namespace

std::string DSLParser::sanitize_bounded_text(const std::string & text, size_t max_chars)
{
  return sanitize_bounded_view(text, max_chars);
}

void DSLParser::enforce_diagnostic_budget(
  std::vector<Diagnostic> & diags,
  size_t additional_suppressed,
  bool stopped_early)
{
  if (diags.empty() && additional_suppressed == 0 && !stopped_early) {
    return;
  }
  std::vector<Diagnostic> bounded;
  bounded.reserve(std::min(diags.size() + 1, kMaxDiagnostics));
  DiagnosticSink sink(bounded);
  for (auto & d : diags) {
    if (d.code == "WARN_DIAGNOSTICS_TRUNCATED") {
      stopped_early = true;
      continue;
    }
    sink.add(std::move(d));
  }
  if (additional_suppressed > 0) {
    sink.add_suppressed(additional_suppressed);
  }
  if (stopped_early) {
    sink.mark_stopped_early();
  }
  sink.finalize();
  diags.swap(bounded);
}

bool DSLParser::parse(
  const std::string & source,
  ASTProgram & out_program,
  std::vector<Diagnostic> & out_diagnostics)
{
  out_program = ASTProgram{};
  out_diagnostics.clear();

  if (source.size() > kMaxSourceBytes) {
    Diagnostic d;
    d.severity = Diagnostic::SEVERITY_ERROR;
    d.stage = Diagnostic::STAGE_LEX_SYNTAX;
    d.line_number = 1;
    d.column_number = 1;
    d.statement_index = -1;
    d.code = "ERR_SOURCE_TOO_LARGE";
    d.message = "Source program size (" + std::to_string(source.size()) +
      " bytes) exceeds maximum limit (" + std::to_string(kMaxSourceBytes) + " bytes)";
    out_diagnostics.push_back(std::move(d));
    return false;
  }

  std::vector<LineSpan> lines;
  if (!index_lines(source, lines)) {
    Diagnostic d;
    d.severity = Diagnostic::SEVERITY_ERROR;
    d.stage = Diagnostic::STAGE_LEX_SYNTAX;
    d.line_number = 1;
    d.column_number = 1;
    d.statement_index = -1;
    d.code = "ERR_SOURCE_TOO_LARGE";
    d.message = "Source program exceeds maximum line count (" +
      std::to_string(kMaxSourceLines) + " lines)";
    out_diagnostics.push_back(std::move(d));
    return false;
  }

  DiagnosticSink sink(out_diagnostics);
  const auto tokens = tokenize(source, lines, sink);
  if (sink.error_count() > 0) {
    sink.finalize();
    return false;
  }

  ParserImpl parser(source, tokens, lines);
  parser.parse_program(out_program, sink);
  if (sink.error_count() > 0) {
    sink.finalize();
    return false;
  }

  if (out_program.statements.empty()) {
    Diagnostic d;
    d.severity = Diagnostic::SEVERITY_ERROR;
    d.stage = Diagnostic::STAGE_SEMANTIC;
    d.line_number = 1;
    d.column_number = 1;
    d.statement_index = -1;
    d.command_text = extract_line_snippet(source, lines, 1, 1);
    d.code = "ERR_EMPTY_PROGRAM";
    d.message = "Program contains no executable motion, gripper, or wait statements";
    sink.add(std::move(d));
    sink.finalize();
    return false;
  }

  sink.finalize();
  return true;
}

}  // namespace lite6_studio_planner
