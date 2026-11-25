#ifndef LITE6_STUDIO_PLANNER__DSL_PARSER_HPP_
#define LITE6_STUDIO_PLANNER__DSL_PARSER_HPP_

#include "lite6_studio_planner/dsl_ast.hpp"
#include "lite6_interfaces/msg/compile_diagnostic.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace lite6_studio_planner
{

class DSLParser
{
public:
  using Diagnostic = lite6_interfaces::msg::CompileDiagnostic;

  // Compilation resource budgets preventing quadratic diagnostic expansion and memory exhaustion.
  static constexpr size_t kMaxSourceBytes = 256 * 1024;
  static constexpr size_t kMaxSourceLines = 16384;
  static constexpr size_t kMaxTokenLength = 256;
  static constexpr size_t kMaxTokens = 65536;
  static constexpr size_t kMaxStatements = 2048;
  static constexpr size_t kMaxPvaWaypointsPerStatement = 256;
  static constexpr size_t kMaxDiagnostics = 64;
  static constexpr size_t kMaxSnippetChars = 160;
  static constexpr size_t kMaxDiagnosticMessageChars = 512;
  static constexpr size_t kMaxTotalDiagnosticBytes = 64 * 1024;

  // Sanitizes text into valid printable UTF-8 bounded to at most max_chars bytes.
  static std::string sanitize_bounded_text(const std::string & text, size_t max_chars);

  // Enforces field length bounds, valid UTF-8, maximum entry count, and total byte budget
  // across a diagnostic vector, appending a truncation notice if entries were omitted.
  static void enforce_diagnostic_budget(
    std::vector<Diagnostic> & diags,
    size_t additional_suppressed = 0,
    bool stopped_early = false);

  // Parses and semantically validates trajectory DSL source code.
  // Returns true iff zero errors occurred across Stages 1 (Lex/Syntax) and 2 (Semantic).
  static bool parse(
    const std::string & source,
    ASTProgram & out_program,
    std::vector<Diagnostic> & out_diagnostics);
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__DSL_PARSER_HPP_
