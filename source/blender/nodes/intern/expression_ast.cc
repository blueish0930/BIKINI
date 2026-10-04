/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <fmt/format.h>

#include "NOD_expression_ast.hh"

#include "BLI_dot_export.hh"

namespace blender::nodes::expression::ast {

dot_export::Node &NumberLiteral::to_dot(dot_export::DirectedGraph &graph) const
{
  return graph.new_node(this->value);
}

dot_export::Node &StringLiteral::to_dot(dot_export::DirectedGraph &graph) const
{
  return graph.new_node(this->value);
}

dot_export::Node &Identifier::to_dot(dot_export::DirectedGraph &graph) const
{
  return graph.new_node(this->identifier);
}

dot_export::Node &MemberAccess::to_dot(dot_export::DirectedGraph &graph) const
{
  dot_export::Node &expr_node = this->expr->to_dot(graph);
  dot_export::Node &member_access_node = graph.new_node(fmt::format(".{}", this->identifier));
  graph.new_edge(member_access_node, expr_node);
  return member_access_node;
}

dot_export::Node &BinaryOp::to_dot(dot_export::DirectedGraph &graph) const
{
  dot_export::Node &a_node = this->a->to_dot(graph);
  dot_export::Node &b_node = this->b->to_dot(graph);
  dot_export::Node &op_node = graph.new_node(this->op);
  graph.new_edge(op_node, a_node);
  graph.new_edge(op_node, b_node);
  return op_node;
}

dot_export::Node &UnaryOp::to_dot(dot_export::DirectedGraph &graph) const
{
  dot_export::Node &expr_node = this->expr->to_dot(graph);
  dot_export::Node &op_node = graph.new_node(this->op);
  graph.new_edge(op_node, expr_node);
  return op_node;
}

dot_export::Node &ConditionalOp::to_dot(dot_export::DirectedGraph &graph) const
{
  dot_export::Node &condition_node = this->condition->to_dot(graph);
  dot_export::Node &true_expr_node = this->true_expr->to_dot(graph);
  dot_export::Node &false_expr_node = this->false_expr->to_dot(graph);
  dot_export::Node &op_node = graph.new_node("if");
  graph.new_edge(op_node, condition_node);
  graph.new_edge(op_node, true_expr_node);
  graph.new_edge(op_node, false_expr_node);
  return op_node;
}

dot_export::Node &Call::to_dot(dot_export::DirectedGraph &graph) const
{
  dot_export::Node &call_node = graph.new_node("call");
  dot_export::Node &function_node = this->function->to_dot(graph);
  graph.new_edge(call_node, function_node);
  for (const Expr *arg : this->args) {
    dot_export::Node &arg_node = arg->to_dot(graph);
    graph.new_edge(call_node, arg_node);
  }
  return call_node;
}

dot_export::Node &Expr::to_dot(dot_export::DirectedGraph &graph) const
{
  return std::visit([&](const auto &expr) -> dot_export::Node & { return expr.to_dot(graph); },
                    this->expr);
}

std::string Expr::to_dot() const
{
  dot_export::DirectedGraph graph;
  graph.attributes.set("ordering", "out");
  this->to_dot(graph);
  return graph.to_dot_string();
}

}  // namespace blender::nodes::expression::ast
