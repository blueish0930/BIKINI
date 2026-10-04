/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <variant>

#include "BLI_dot_export_fwd.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

namespace blender::nodes::expression::ast {

class Expr;

class NumberLiteral {
 public:
  StringRef value;

  dot_export::Node &to_dot(dot_export::DirectedGraph &graph) const;
};

class StringLiteral {
 public:
  StringRef value;

  dot_export::Node &to_dot(dot_export::DirectedGraph &graph) const;
};

class Identifier {
 public:
  StringRef identifier;

  dot_export::Node &to_dot(dot_export::DirectedGraph &graph) const;
};

class MemberAccess {
 public:
  Expr *expr = nullptr;
  StringRef identifier;

  dot_export::Node &to_dot(dot_export::DirectedGraph &graph) const;
};

class BinaryOp {
 public:
  StringRef op;
  Expr *a = nullptr;
  Expr *b = nullptr;

  dot_export::Node &to_dot(dot_export::DirectedGraph &graph) const;
};

class UnaryOp {
 public:
  StringRef op;
  Expr *expr = nullptr;

  dot_export::Node &to_dot(dot_export::DirectedGraph &graph) const;
};

class ConditionalOp {
 public:
  Expr *condition = nullptr;
  Expr *true_expr = nullptr;
  Expr *false_expr = nullptr;

  dot_export::Node &to_dot(dot_export::DirectedGraph &graph) const;
};

class Call {
 public:
  Expr *function = nullptr;
  Vector<Expr *> args;

  dot_export::Node &to_dot(dot_export::DirectedGraph &graph) const;
};

class Expr {
 public:
  using ExprVariant = std::variant<NumberLiteral,
                                   StringLiteral,
                                   Identifier,
                                   BinaryOp,
                                   UnaryOp,
                                   ConditionalOp,
                                   MemberAccess,
                                   Call>;

  ExprVariant expr;

  Expr(ExprVariant expr) : expr(std::move(expr)) {}

  dot_export::Node &to_dot(dot_export::DirectedGraph &graph) const;
  std::string to_dot() const;
};

}  // namespace blender::nodes::expression::ast
