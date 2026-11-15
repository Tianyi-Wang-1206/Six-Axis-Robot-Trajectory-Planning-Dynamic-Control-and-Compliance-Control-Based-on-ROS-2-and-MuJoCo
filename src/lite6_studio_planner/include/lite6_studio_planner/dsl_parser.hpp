#ifndef LITE6_STUDIO_PLANNER__DSL_PARSER_HPP_
#define LITE6_STUDIO_PLANNER__DSL_PARSER_HPP_

#include "lite6_studio_planner/dsl_ast.hpp"
#include "lite6_interfaces/msg/compile_diagnostic.hpp"

#include <string>
#include <vector>

namespace lite6_studio_planner
{

class DSLParser
{
public:
  using Diagnostic = lite6_interfaces::msg::CompileDiagnostic;

  // Parses and semantically validates Lite6Script source code.
  // Returns true iff zero errors occurred across Stages 1 (Lex/Syntax) and 2 (Semantic).
  static bool parse(
    const std::string & source,
    ASTProgram & out_program,
    std::vector<Diagnostic> & out_diagnostics);
};

}  // namespace lite6_studio_planner

#endif  // LITE6_STUDIO_PLANNER__DSL_PARSER_HPP_
