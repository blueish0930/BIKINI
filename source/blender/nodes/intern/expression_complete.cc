/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_expression_complete.hh"
#include "NOD_geo_expression.hh"

#include "DNA_node_types.h"

#include "BLI_utildefines.hh"

namespace blender::nodes::expression {

StringRefNull category_label(const CompletionCategory category)
{
  switch (category) {
    case CompletionCategory::Variable:
      return "Variable";
    case CompletionCategory::Operator:
      return "Operator";
    case CompletionCategory::Float:
      return "Float";
    case CompletionCategory::Vector:
      return "Vector";
    case CompletionCategory::Color:
      return "Color";
    case CompletionCategory::Matrix:
      return "Matrix";
    case CompletionCategory::Rotation:
      return "Rotation";
    case CompletionCategory::Member:
      return "Member";
  }
  return "";
}

Span<CompletionItem> builtin_completions()
{
  /* usage = list text only (how to write it). insert_text = what goes into the expression. */
  static const CompletionItem items[] = {
      /* ---- Operators ---- */
      {"+", "Operator", CompletionCategory::Operator, CompletionKind::Operator, "+", "a + b", false},
      {"-", "Operator", CompletionCategory::Operator, CompletionKind::Operator, "-", "a - b", false},
      {"*", "Operator", CompletionCategory::Operator, CompletionKind::Operator, "*", "a * b", false},
      {"*", "Operator", CompletionCategory::Operator, CompletionKind::Operator, "*", "m * v", false},
      {"/", "Operator", CompletionCategory::Operator, CompletionKind::Operator, "/", "a / b", false},
      {"<", "Operator", CompletionCategory::Operator, CompletionKind::Operator, "<", "a < b", false},
      {">", "Operator", CompletionCategory::Operator, CompletionKind::Operator, ">", "a > b", false},
      {"?:", "Operator", CompletionCategory::Operator, CompletionKind::Operator, "?:", "c ? a : b", false},

      /* ---- Float ---- */
      {"min", "Float", CompletionCategory::Float, CompletionKind::Function, "min", "min(a, b)", true},
      {"max", "Float", CompletionCategory::Float, CompletionKind::Function, "max", "max(a, b)", true},
      {"abs", "Float", CompletionCategory::Float, CompletionKind::Function, "abs", "abs(a)", true},
      {"floor", "Float", CompletionCategory::Float, CompletionKind::Function, "floor", "floor(a)", true},
      {"ceil", "Float", CompletionCategory::Float, CompletionKind::Function, "ceil", "ceil(a)", true},
      {"trunc", "Float", CompletionCategory::Float, CompletionKind::Function, "trunc", "trunc(a)", true},
      {"round", "Float", CompletionCategory::Float, CompletionKind::Function, "round", "round(a)", true},
      {"sin", "Float", CompletionCategory::Float, CompletionKind::Function, "sin", "sin(a)", true},
      {"cos", "Float", CompletionCategory::Float, CompletionKind::Function, "cos", "cos(a)", true},
      {"tan", "Float", CompletionCategory::Float, CompletionKind::Function, "tan", "tan(a)", true},
      {"asin", "Float", CompletionCategory::Float, CompletionKind::Function, "asin", "asin(a)", true},
      {"acos", "Float", CompletionCategory::Float, CompletionKind::Function, "acos", "acos(a)", true},
      {"atan", "Float", CompletionCategory::Float, CompletionKind::Function, "atan", "atan(a)", true},
      {"atan2", "Float", CompletionCategory::Float, CompletionKind::Function, "atan2", "atan2(a, b)", true},
      {"exp", "Float", CompletionCategory::Float, CompletionKind::Function, "exp", "exp(a)", true},
      {"log", "Float", CompletionCategory::Float, CompletionKind::Function, "log", "log(a)", true},
      {"sqrt", "Float", CompletionCategory::Float, CompletionKind::Function, "sqrt", "sqrt(a)", true},
      {"pow", "Float", CompletionCategory::Float, CompletionKind::Function, "pow", "pow(a, b)", true},
      {"radians", "Float", CompletionCategory::Float, CompletionKind::Function, "radians", "radians(a)", true},
      {"degrees", "Float", CompletionCategory::Float, CompletionKind::Function, "degrees", "degrees(a)", true},
      {"float", "Float", CompletionCategory::Float, CompletionKind::Function, "float", "float(a)", true},
      {"int", "Float", CompletionCategory::Float, CompletionKind::Function, "int", "int(a)", true},

      /* ---- Vector ---- */
      {"vec2", "Vector", CompletionCategory::Vector, CompletionKind::Function, "vec2", "vec2(a, b)", true},
      {"vec3", "Vector", CompletionCategory::Vector, CompletionKind::Function, "vec3", "vec3(a, b, c)", true},
      {"length", "Vector", CompletionCategory::Vector, CompletionKind::Function, "length", "length(v)", true},

      /* ---- Color ---- */
      {"rgb", "Color", CompletionCategory::Color, CompletionKind::Function, "rgb", "rgb(a, b, c)", true},
      {"rgba", "Color", CompletionCategory::Color, CompletionKind::Function, "rgba", "rgba(a, b, c, d)", true},

      /* ---- Matrix ---- */
      {"invert", "Matrix", CompletionCategory::Matrix, CompletionKind::Function, "invert", "invert(m)", true},
      {"inv", "Matrix", CompletionCategory::Matrix, CompletionKind::Function, "inv", "inv(m)", true},
      {"transpose",
       "Matrix",
       CompletionCategory::Matrix,
       CompletionKind::Function,
       "transpose",
       "transpose(m)",
       true},
      {"determinant",
       "Matrix",
       CompletionCategory::Matrix,
       CompletionKind::Function,
       "determinant",
       "determinant(m)",
       true},
      {"det", "Matrix", CompletionCategory::Matrix, CompletionKind::Function, "det", "det(m)", true},
      {"transform_point",
       "Matrix",
       CompletionCategory::Matrix,
       CompletionKind::Function,
       "transform_point",
       "transform_point(v, m)",
       true},
      {"transform_direction",
       "Matrix",
       CompletionCategory::Matrix,
       CompletionKind::Function,
       "transform_direction",
       "transform_direction(v, m)",
       true},
      {"project_point",
       "Matrix",
       CompletionCategory::Matrix,
       CompletionKind::Function,
       "project_point",
       "project_point(v, m)",
       true},
      {"combine_transform",
       "Matrix",
       CompletionCategory::Matrix,
       CompletionKind::Function,
       "combine_transform",
       "combine_transform(t, r, s)",
       true},
      {"translation",
       "Matrix",
       CompletionCategory::Matrix,
       CompletionKind::Function,
       "translation",
       "translation(m)",
       true},
      {"rotation",
       "Matrix",
       CompletionCategory::Matrix,
       CompletionKind::Function,
       "rotation",
       "rotation(m)",
       true},
      {"scale", "Matrix", CompletionCategory::Matrix, CompletionKind::Function, "scale", "scale(m)", true},
      {"svd_u", "Matrix", CompletionCategory::Matrix, CompletionKind::Function, "svd_u", "svd_u(m)", true},
      {"svd_s", "Matrix", CompletionCategory::Matrix, CompletionKind::Function, "svd_s", "svd_s(m)", true},
      {"svd_v", "Matrix", CompletionCategory::Matrix, CompletionKind::Function, "svd_v", "svd_v(m)", true},

      /* ---- Rotation ---- */
      {"rotate_vector",
       "Rotation",
       CompletionCategory::Rotation,
       CompletionKind::Function,
       "rotate_vector",
       "rotate_vector(v, r)",
       true},
      {"rotate_rotation",
       "Rotation",
       CompletionCategory::Rotation,
       CompletionKind::Function,
       "rotate_rotation",
       "rotate_rotation(r, b)",
       true},
      {"invert", "Rotation", CompletionCategory::Rotation, CompletionKind::Function, "invert", "invert(r)", true},
      {"inv", "Rotation", CompletionCategory::Rotation, CompletionKind::Function, "inv", "inv(r)", true},

      /* ---- Members ---- */
      {"x", "Member", CompletionCategory::Member, CompletionKind::Member, "x", "v.x", false},
      {"y", "Member", CompletionCategory::Member, CompletionKind::Member, "y", "v.y", false},
      {"z", "Member", CompletionCategory::Member, CompletionKind::Member, "z", "v.z", false},
      {"r", "Member", CompletionCategory::Member, CompletionKind::Member, "r", "c.r", false},
      {"g", "Member", CompletionCategory::Member, CompletionKind::Member, "g", "c.g", false},
      {"b", "Member", CompletionCategory::Member, CompletionKind::Member, "b", "c.b", false},
      {"a", "Member", CompletionCategory::Member, CompletionKind::Member, "a", "c.a", false},
      {"translation",
       "Member",
       CompletionCategory::Member,
       CompletionKind::Member,
       "translation",
       "m.translation",
       false},
      {"rotation",
       "Member",
       CompletionCategory::Member,
       CompletionKind::Member,
       "rotation",
       "m.rotation",
       false},
      {"scale", "Member", CompletionCategory::Member, CompletionKind::Member, "scale", "m.scale", false},
      {"m00", "Member", CompletionCategory::Member, CompletionKind::Member, "m00", "m.m00", false},
      {"m01", "Member", CompletionCategory::Member, CompletionKind::Member, "m01", "m.m01", false},
      {"m02", "Member", CompletionCategory::Member, CompletionKind::Member, "m02", "m.m02", false},
      {"m03", "Member", CompletionCategory::Member, CompletionKind::Member, "m03", "m.m03", false},
      {"m10", "Member", CompletionCategory::Member, CompletionKind::Member, "m10", "m.m10", false},
      {"m11", "Member", CompletionCategory::Member, CompletionKind::Member, "m11", "m.m11", false},
      {"m12", "Member", CompletionCategory::Member, CompletionKind::Member, "m12", "m.m12", false},
      {"m13", "Member", CompletionCategory::Member, CompletionKind::Member, "m13", "m.m13", false},
      {"m20", "Member", CompletionCategory::Member, CompletionKind::Member, "m20", "m.m20", false},
      {"m21", "Member", CompletionCategory::Member, CompletionKind::Member, "m21", "m.m21", false},
      {"m22", "Member", CompletionCategory::Member, CompletionKind::Member, "m22", "m.m22", false},
      {"m23", "Member", CompletionCategory::Member, CompletionKind::Member, "m23", "m.m23", false},
      {"m30", "Member", CompletionCategory::Member, CompletionKind::Member, "m30", "m.m30", false},
      {"m31", "Member", CompletionCategory::Member, CompletionKind::Member, "m31", "m.m31", false},
      {"m32", "Member", CompletionCategory::Member, CompletionKind::Member, "m32", "m.m32", false},
      {"m33", "Member", CompletionCategory::Member, CompletionKind::Member, "m33", "m.m33", false},
  };
  return Span(items, ARRAY_SIZE(items));
}

static bool is_ident_char(const char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
}

void gather_completions(const bNode &node,
                        Vector<std::string> &r_owned_names,
                        Vector<CompletionItem> &r_items)
{
  r_owned_names.clear();
  r_items.clear();

  for (const CompletionItem &item : builtin_completions()) {
    r_items.append(item);
  }

  if (node.typeinfo == nullptr || !node.is_type("NodeExpression"_ustr)) {
    return;
  }
  const NodeExpression *storage = static_cast<const NodeExpression *>(node.storage);
  if (!storage) {
    return;
  }
  for (const int i : IndexRange(storage->input_items.items_num)) {
    const NodeExpressionInputItem &item = storage->input_items.items[i];
    if (!item.name || item.name[0] == '\0') {
      continue;
    }
    r_owned_names.append(item.name);
    const StringRef name = r_owned_names.last();
    /* List and insert both use the input name (e.g. a, m, v). */
    r_items.append(CompletionItem{
        name, "Variable", CompletionCategory::Variable, CompletionKind::Variable, name, name, false});
  }
}

bool find_completion_token(const StringRef expression,
                           int &r_token_start,
                           int &r_token_len,
                           bool &r_is_member)
{
  r_token_start = 0;
  r_token_len = 0;
  r_is_member = false;

  if (expression.is_empty()) {
    return true;
  }

  int i = int(expression.size()) - 1;
  while (i >= 0 && is_ident_char(expression[i])) {
    i--;
  }
  r_token_start = i + 1;
  r_token_len = int(expression.size()) - r_token_start;

  if (i >= 0 && expression[i] == '.') {
    r_is_member = true;
    return true;
  }

  if (r_token_len == 0) {
    if (i < 0) {
      return true;
    }
    const char prev = expression[i];
    if (ELEM(prev, ' ', '\t', '\n', '(', ',', '+', '-', '*', '/', '?', ':', '<', '>', '=', '!')) {
      return true;
    }
    return false;
  }

  if (expression[r_token_start] >= '0' && expression[r_token_start] <= '9') {
    return false;
  }
  return true;
}

std::string apply_completion(const StringRef expression,
                             const int token_start,
                             const int token_len,
                             const StringRef completion,
                             const bool insert_call_parens)
{
  BLI_assert(token_start >= 0);
  BLI_assert(token_start + token_len <= int(expression.size()));

  std::string result;
  result.reserve(expression.size() + completion.size() + 2);
  result.append(expression.data(), token_start);
  result.append(completion.data(), completion.size());

  const int after = token_start + token_len;
  if (insert_call_parens) {
    const bool already_open = after < int(expression.size()) && expression[after] == '(';
    if (!already_open) {
      result.append("(");
    }
  }
  if (after < int(expression.size())) {
    result.append(expression.data() + after, expression.size() - after);
  }
  return result;
}

}  // namespace blender::nodes::expression
