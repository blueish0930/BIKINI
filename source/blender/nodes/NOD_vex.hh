/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <memory>
#include <string>

#include "BLI_index_mask.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "FN_field.hh"

#include "NOD_expression_complete.hh"

namespace blender::bke {
struct GeometrySet;
}

namespace blender::nodes::vex {

struct Program;

enum class Domain : int8_t {
  Point = 0,
  Edge = 1,
  Face = 2,
  Corner = 3,
  Instance = 4,
  Curve = 5,
};

struct Diagnostic {
  int offset = 0;
  int length = 1;
  bool is_error = true;
  std::string message;
};

struct CompileOutput {
  std::shared_ptr<Program> program;
  std::string error;
  std::string warning;
  Vector<Diagnostic> diags;
};

CompileOutput compile(StringRef source);
CompileOutput compile_shader_material(StringRef source);

struct ExecOutput {
  bool ok = true;
  std::string error;
  std::string warning;
};

struct ChField {
  std::string name;
  fn::GField field;

  ChField(std::string name, fn::GField field) : name(std::move(name)), field(std::move(field)) {}
};

/**
 * Run the program on \a geometry.
 * Extra geometry inputs are addressed as `point(1, ...)`, `point(2, ...)`.
 * Channel parameters (`chf("name")`, …) are looked up in \a parms.
 */
ExecOutput execute(const Program &program,
                   bke::GeometrySet &geometry,
                   Span<const bke::GeometrySet *> extra_geometry,
                   Domain domain,
                   const fn::Field<bool> &selection,
                   Span<ChField> parms = {});

/** Evaluate a script once with no geometry (for tests). Optional `return` value as int/float/string. */
struct PureEvalOutput {
  bool ok = true;
  std::string error;
  bool has_return = false;
  int return_int = 0;
  float return_float = 0.0f;
  float3 return_vec = float3(0.0f);
  std::string return_string;
};

PureEvalOutput execute_pure(const Program &program);

void gather_completions(Vector<std::string> &r_owned_names,
                        Vector<expression::CompletionItem> &r_items,
                        bool shader_material = false);

/** Comma-separated param types for a builtin, e.g. `"geo,int"` or `"geo,int,out,out"`. */
StringRef function_param_types(StringRef name);

}  // namespace blender::nodes::vex
