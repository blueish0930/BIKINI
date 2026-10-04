/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "testing/testing.h"

#include "NOD_expression_parse.hh"

#include "expression_parse.hh"

namespace blender::nodes::expression::tests {

static void expect_tokens(const StringRef src, const Span<StringRef> expected_tokens)
{
  const TokenizeResult result = tokenize(src);
  const Vector<Token> *found_tokens = std::get_if<Vector<Token>>(&result.result);
  if (!found_tokens) {
    FAIL() << "Expected tokens, got error: " << std::get<std::string>(result.result);
    return;
  }
  EXPECT_EQ(found_tokens->size(), expected_tokens.size());
  for (const int i : expected_tokens.index_range()) {
    const StringRef expected_str = expected_tokens[i];
    const Token &found = (*found_tokens)[i];
    EXPECT_EQ(found.str, expected_str);
  }
}

TEST(nodes_expression, tokenize_empty)
{
  expect_tokens("", {});
  expect_tokens(" ", {});
  expect_tokens("  \n\n\t\n\t\t \n", {});
}

TEST(nodes_expression, tokenize_identifier)
{
  expect_tokens("a", {"a"});
  expect_tokens("abc qwe", {"abc", "qwe"});
  expect_tokens("abc\nqwe34 \n", {"abc", "qwe34"});
}

TEST(nodes_expression, tokenize_number)
{
  expect_tokens("0", {"0"});
  expect_tokens("123", {"123"});
  expect_tokens("123 634", {"123", "634"});
  expect_tokens("123.456", {"123.456"});
  expect_tokens("123.456.", {"123.456", "."});
  expect_tokens("123...", {"123.", ".", "."});
}

TEST(nodes_expression, tokenize_string)
{
  expect_tokens("\"abc\"", {"\"abc\""});
  expect_tokens("\"abc\n'qwe34 \n\" \"\"", {"\"abc\n'qwe34 \n\"", "\"\""});
}

TEST(nodes_expression, tokenize_string_unterminated)
{
  const TokenizeResult result = tokenize("\"abc");
  const StringRef error = std::get<std::string>(result.result);
  EXPECT_TRUE(error.startswith("Unterminated string"));
}

TEST(nodes_expression, tokenize_special)
{
  expect_tokens("+", {"+"});
  expect_tokens("-", {"-"});
  expect_tokens("*+", {"*", "+"});
  expect_tokens("<=", {"<="});
  expect_tokens("< =", {"<", "="});
  expect_tokens("==", {"=="});
  expect_tokens(">>>", {">>", ">"});
  expect_tokens("&&&|", {"&&", "&", "|"});
}

TEST(nodes_expression, tokenize_invalid_char)
{
  {
    const TokenizeResult result = tokenize("a\x1b");
    const StringRef error = std::get<std::string>(result.result);
    EXPECT_TRUE(error.startswith("Invalid character"));
  }
  {
    const TokenizeResult result = tokenize("a`");
    const StringRef error = std::get<std::string>(result.result);
    EXPECT_TRUE(error.startswith("Invalid character"));
  }
}

static void expect_ast_recursive(const ast::Expr &a, const ast::Expr &b)
{
  EXPECT_EQ(a.expr.index(), b.expr.index());
  if (const auto *a_ = std::get_if<ast::Identifier>(&a.expr)) {
    const auto *b_ = std::get_if<ast::Identifier>(&b.expr);
    EXPECT_EQ(a_->identifier, b_->identifier);
  }
  else if (const auto *a_ = std::get_if<ast::NumberLiteral>(&a.expr)) {
    const auto *b_ = std::get_if<ast::NumberLiteral>(&b.expr);
    EXPECT_EQ(a_->value, b_->value);
  }
  else if (const auto *a_ = std::get_if<ast::StringLiteral>(&a.expr)) {
    const auto *b_ = std::get_if<ast::StringLiteral>(&b.expr);
    EXPECT_EQ(a_->value, b_->value);
  }
  else if (const auto *a_ = std::get_if<ast::MemberAccess>(&a.expr)) {
    const auto *b_ = std::get_if<ast::MemberAccess>(&b.expr);
    expect_ast_recursive(*a_->expr, *b_->expr);
    EXPECT_EQ(a_->identifier, b_->identifier);
  }
  else if (const auto *a_ = std::get_if<ast::BinaryOp>(&a.expr)) {
    const auto *b_ = std::get_if<ast::BinaryOp>(&b.expr);
    EXPECT_EQ(a_->op, b_->op);
    expect_ast_recursive(*a_->a, *b_->a);
    expect_ast_recursive(*a_->b, *b_->b);
  }
  else if (const auto *a_ = std::get_if<ast::UnaryOp>(&a.expr)) {
    const auto *b_ = std::get_if<ast::UnaryOp>(&b.expr);
    EXPECT_EQ(a_->op, b_->op);
    expect_ast_recursive(*a_->expr, *b_->expr);
  }
  else if (const auto *a_ = std::get_if<ast::ConditionalOp>(&a.expr)) {
    const auto *b_ = std::get_if<ast::ConditionalOp>(&b.expr);
    expect_ast_recursive(*a_->condition, *b_->condition);
    expect_ast_recursive(*a_->true_expr, *b_->true_expr);
    expect_ast_recursive(*a_->false_expr, *b_->false_expr);
  }
  else if (const auto *a_ = std::get_if<ast::Call>(&a.expr)) {
    const auto *b_ = std::get_if<ast::Call>(&b.expr);
    expect_ast_recursive(*a_->function, *b_->function);
    EXPECT_EQ(a_->args.size(), b_->args.size());
    for (const int i : a_->args.index_range()) {
      expect_ast_recursive(*a_->args[i], *b_->args[i]);
    }
  }
  else {
    BLI_assert_unreachable();
  }
}

static void expect_ast(const StringRef src, const ast::Expr &b)
{
  ResourceScope scope;
  ParseResult result = parse(scope, src);
  if (const std::string *error = std::get_if<std::string>(&result)) {
    FAIL() << "Expected expression, got error: " << *error;
    return;
  }
  ast::Expr *expr = std::get<ast::Expr *>(result);
  expect_ast_recursive(*expr, b);
}

static void expect_parse_error(const StringRef src)
{
  ResourceScope scope;
  ParseResult result = parse(scope, src);
  EXPECT_TRUE(std::holds_alternative<std::string>(result));
}

static ResourceScope &get_static_scope()
{
  static ResourceScope scope;
  return scope;
}

static ast::Expr *id(const StringRef id)
{
  return &get_static_scope().construct<ast::Expr>(ast::Identifier{id});
}

static ast::Expr *number(const StringRef number)
{
  return &get_static_scope().construct<ast::Expr>(ast::NumberLiteral{number});
}

static ast::Expr *string(const StringRef string)
{
  return &get_static_scope().construct<ast::Expr>(ast::StringLiteral{string});
}

static ast::Expr *binary(const StringRef op, ast::Expr *a, ast::Expr *b)
{
  return &get_static_scope().construct<ast::Expr>(ast::BinaryOp{op, a, b});
}

static ast::Expr *unary(const StringRef op, ast::Expr *a)
{
  return &get_static_scope().construct<ast::Expr>(ast::UnaryOp{op, a});
}

static ast::Expr *conditional(ast::Expr *condition, ast::Expr *true_expr, ast::Expr *false_expr)
{
  return &get_static_scope().construct<ast::Expr>(
      ast::ConditionalOp{condition, true_expr, false_expr});
}

static ast::Expr *call(ast::Expr *function, Span<ast::Expr *> args)
{
  return &get_static_scope().construct<ast::Expr>(ast::Call{function, args});
}

static ast::Expr *member(ast::Expr *expr, const StringRef id)
{
  return &get_static_scope().construct<ast::Expr>(ast::MemberAccess{expr, id});
}

TEST(nodes_expression, parse_id)
{
  expect_ast("a", *id("a"));
  expect_ast("abc", *id("abc"));
  expect_parse_error("a b");
}

TEST(nodes_expression, parse_number)
{
  expect_ast("123", *number("123"));
  expect_ast("123.", *number("123."));
  expect_ast("123.456", *number("123.456"));
  expect_parse_error("123.456 4");
  expect_parse_error("(1");
}

TEST(nodes_expression, parse_string)
{
  expect_ast("\"\"", *string("\"\""));
  expect_ast("\"abc\"", *string("\"abc\""));
  expect_parse_error("\"abc");
}

TEST(nodes_expression, parse_binary)
{
  expect_ast("1 < 2", *binary("<", number("1"), number("2")));
  expect_ast("1 <= 2", *binary("<=", number("1"), number("2")));
  expect_ast("a < b && c <= d",
             *binary("&&", binary("<", id("a"), id("b")), binary("<=", id("c"), id("d"))));
  expect_ast("1 + 2", *binary("+", number("1"), number("2")));
  expect_ast("1 + 2 + 3", *binary("+", binary("+", number("1"), number("2")), number("3")));
  expect_ast("1 - 2 + 3", *binary("+", binary("-", number("1"), number("2")), number("3")));
  expect_ast("1 + 2 - 3", *binary("-", binary("+", number("1"), number("2")), number("3")));
  expect_ast(
      "2 * 3 + 4 * 5",
      *binary("+", binary("*", number("2"), number("3")), binary("*", number("4"), number("5"))));
  expect_parse_error("1 +");
  expect_parse_error("1 <");
  expect_parse_error("1 < 2 <");
}

TEST(nodes_expression, parse_unary)
{
  expect_ast("-1", *unary("-", number("1")));
  expect_ast("+1", *unary("+", number("1")));
  expect_ast("----1", *unary("-", unary("-", unary("-", unary("-", number("1"))))));
  expect_ast("+++1", *unary("+", unary("+", unary("+", number("1")))));
  expect_ast("-1 + +2", *binary("+", unary("-", number("1")), unary("+", number("2"))));
  expect_parse_error("-(");
}

TEST(nodes_expression, parse_conditional)
{
  expect_ast("1 ? 2 : 3", *conditional(number("1"), number("2"), number("3")));
  expect_ast("a < b ? c - 1 : (1 ? 10 : 20)",
             *conditional(binary("<", id("a"), id("b")),
                          binary("-", id("c"), number("1")),
                          conditional(number("1"), number("10"), number("20"))));
  expect_parse_error("1 ?");
  expect_parse_error("1 ?? 2 : 3");
  expect_parse_error("1 ? 2");
  expect_parse_error("1 ? 2 :");
  expect_parse_error("1 ? 2 :: 3");
  expect_parse_error("1 ? 2 : 3 ?");
}

TEST(nodes_expression, parse_call)
{
  expect_ast("a()", *call(id("a"), {}));
  expect_ast("a(1, 2)", *call(id("a"), {number("1"), number("2")}));
  expect_ast("a(1, 2)(3, 4)",
             *call(call(id("a"), {number("1"), number("2")}), {number("3"), number("4")}));
  expect_ast(
      "a(b(c(d(1, 2), 3)))",
      *call(id("a"),
            {
                call(id("b"),
                     {
                         call(id("c"), {call(id("d"), {number("1"), number("2")}), number("3")}),
                     }),
            }));
  expect_ast("a(\"b\")", *call(id("a"), {string("\"b\"")}));
  expect_parse_error("a(");
  expect_parse_error("a(3, ");
  expect_parse_error("a(3, 4");
}

TEST(nodes_expression, parse_member)
{
  expect_ast("a.b", *member(id("a"), "b"));
  expect_ast("a.b.c.d", *member(member(member(id("a"), "b"), "c"), "d"));
  expect_ast("a.b(c).d", *member(call(member(id("a"), "b"), {id("c")}), "d"));
  expect_parse_error("a.b..c");
  expect_parse_error("a.");
}

}  // namespace blender::nodes::expression::tests
