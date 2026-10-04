/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <fmt/format.h>
#include <iostream>

#include "NOD_expression_parse.hh"

#include "BLT_translation.hh"

#include "expression_parse.hh"

namespace blender::nodes::expression {

class Tokenizer {
 private:
  const StringRef full_str_;
  int64_t i_ = 0;
  const int64_t full_size_;
  std::optional<std::string> error_;
  Vector<Token> tokens_;

 public:
  Tokenizer(const StringRef full_str) : full_str_(full_str), full_size_(full_str.size())
  {
    /* This generally overallocates a bit, but avoids reallocations later. */
    tokens_.reserve(full_str.size());
  }

  TokenizeResult tokenize()
  {
    while (i_ < full_size_) {
      if (error_.has_value()) {
        break;
      }
      const char c = full_str_[i_];
      if (this->is_whitespace(c)) {
        i_++;
        continue;
      }
      if (this->is_number_start(c)) {
        this->tokenize_number();
        continue;
      }
      if (this->is_identifier_start(c)) {
        this->tokenize_identifier();
        continue;
      }
      if (this->is_string_start(c)) {
        this->tokenize_string();
        continue;
      }
      if (this->tokenize_special()) {
        continue;
      }
      this->set_invalid_char_error(c);
    }

    if (error_.has_value()) {
      return {std::move(*error_)};
    }
    this->assert_token_strings_in_full_str();
    return {std::move(tokens_)};
  }

  void tokenize_number()
  {
    const int64_t start = i_;
    i_++;
    bool has_dot = false;
    while (i_ < full_size_) {
      const char c = full_str_[i_];
      if (this->is_digit(c)) {
        i_++;
        continue;
      }
      if (c == '.') {
        if (!has_dot) {
          has_dot = true;
          i_++;
          continue;
        }
      }
      break;
    }
    const StringRef number = full_str_.substr(start, i_ - start);
    tokens_.append({TokenType::Number, number});
  }

  void tokenize_identifier()
  {
    const int64_t start = i_;
    i_++;
    while (i_ < full_size_) {
      const char c = full_str_[i_];
      if (!this->is_identifier_continue(c)) {
        break;
      }
      i_++;
    }
    const StringRef identifier = full_str_.substr(start, i_ - start);
    tokens_.append({TokenType::Identifier, identifier});
  }

  [[nodiscard]] bool tokenize_special()
  {
    const char first = full_str_[i_];
    const std::optional<char> second = i_ + 1 < full_size_ ?
                                           std::make_optional(full_str_[i_ + 1]) :
                                           std::nullopt;

    auto add_special = [&](const int length) {
      tokens_.append({TokenType::Special, StringRef(full_str_.data() + i_, length)});
      i_ += length;
      return true;
    };

    switch (first) {
      case '+':
      case '-':
      case '*':
      case '/':
      case '(':
      case ')':
      case ',':
      case ':':
      case '?':
      case '.': {
        return add_special(1);
      }
      case '<':
      case '>': {
        if (second == '=') {
          return add_special(2);
        }
        if (first == second) {
          return add_special(2);
        }
        return add_special(1);
      }
      case '=': {
        if (second == '=') {
          return add_special(2);
        }
        return add_special(1);
      }
      case '&':
      case '|': {
        if (first == second) {
          return add_special(2);
        }
        return add_special(1);
      }
      default: {
        return false;
      }
    }
  }

  void tokenize_string()
  {
    const int64_t start = i_;
    i_++;
    while (i_ < full_size_) {
      const char c = full_str_[i_];
      if (c == '"') {
        break;
      }
      i_++;
    }
    if (i_ == full_size_) {
      this->set_error(TIP_("Unterminated string"));
      return;
    }
    i_++;
    const StringRef string = full_str_.substr(start, i_ - start);
    tokens_.append({TokenType::String, string});
  }

  void set_invalid_char_error(const char c)
  {
    if (std::isprint(c)) {
      this->set_error(fmt::format("{}: '{}'", TIP_("Invalid character"), c));
    }
    else {
      this->set_error(fmt::format("{}: {:#x}", TIP_("Invalid character"), uint32_t(c)));
    }
  }

  void assert_token_strings_in_full_str()
  {
#ifndef NDEBUG
    for (const Token &token : tokens_) {
      const StringRef str = token.str;
      BLI_assert(full_str_.data() <= str.data());
      BLI_assert(str.data() + str.size() <= full_str_.data() + full_str_.size());
    }
#endif
  }

  void set_error(std::string error)
  {
    error_ = std::move(error);
  }

  bool is_number_start(const char c) const
  {
    return this->is_digit(c);
  }

  bool is_digit(const char c) const
  {
    return '0' <= c && c <= '9';
  }

  bool is_identifier_start(const char c) const
  {
    return this->is_lowercase_ascii(c) || this->is_uppercase_ascii(c) || c == '_';
  }

  bool is_identifier_continue(const char c) const
  {
    return this->is_identifier_start(c) || this->is_digit(c);
  }

  bool is_lowercase_ascii(const char c) const
  {
    return 'a' <= c && c <= 'z';
  }

  bool is_uppercase_ascii(const char c) const
  {
    return 'A' <= c && c <= 'Z';
  }

  bool is_whitespace(const char c) const
  {
    return ELEM(c, ' ', '\t', '\n', '\r');
  }

  bool is_string_start(const char c) const
  {
    return c == '"';
  }
};

TokenizeResult tokenize(const StringRef expression)
{
  Tokenizer tokenizer(expression);
  return tokenizer.tokenize();
}

class Parser {
 private:
  ResourceScope &scope_;
  const Span<Token> tokens_;
  int64_t i_ = 0;
  std::optional<std::string> error_;

 public:
  Parser(ResourceScope &scope, const Span<Token> tokens) : scope_(scope), tokens_(tokens) {}

  using ParseFn = ast::Expr *(Parser::*)();

  const std::optional<std::string> &error() const
  {
    return error_;
  }

  ast::Expr *parse_all_as_expression()
  {
    ast::Expr *expr = this->parse__expression();
    if (!this->is_at_end()) {
      this->set_unexpected_token_error();
      return nullptr;
    }
    return expr;
  }

  ast::Expr *parse__expression()
  {
    return this->parse__expression__ternary_conditional();
  }

  ast::Expr *parse__expression__ternary_conditional()
  {
    return this->parse__expression__generic_ternary(
        "?", ":", &Parser::parse__expression__logical_or);
  }

  ast::Expr *parse__expression__logical_or()
  {
    return this->parse__expression__generic_binary_multiple(
        {"||"}, &Parser::parse__expression__logical_and);
  }

  ast::Expr *parse__expression__logical_and()
  {
    return this->parse__expression__generic_binary_multiple({"&&"},
                                                            &Parser::parse__expression__equality);
  }

  ast::Expr *parse__expression__equality()
  {
    return this->parse__expression__generic_binary_single({"==", "!="},
                                                          &Parser::parse__expression__relational);
  }

  ast::Expr *parse__expression__relational()
  {
    return this->parse__expression__generic_binary_single({"<", ">", "<=", ">="},
                                                          &Parser::parse__expression__additive);
  }

  ast::Expr *parse__expression__additive()
  {
    return this->parse__expression__generic_binary_multiple(
        {"+", "-"}, &Parser::parse__expression__multiplicative);
  }

  ast::Expr *parse__expression__multiplicative()
  {
    return this->parse__expression__generic_binary_multiple({"*", "/", "%"},
                                                            &Parser::parse__expression__unary);
  }

  ast::Expr *parse__expression__unary()
  {
    return this->parse__expression__generic_unary({"+", "-", "!", "~"},
                                                  &Parser::parse__expression__dot_or_call);
  }

  ast::Expr *parse__expression__dot_or_call()
  {
    ast::Expr *expr = this->parse__expression__atom();
    if (!expr) {
      return nullptr;
    }
    while (true) {
      if (this->next_is(".")) {
        this->consume_next();
        const std::optional<StringRef> identifier = this->parse__identifier();
        if (!identifier.has_value()) {
          return nullptr;
        }
        expr = this->make_expr(ast::MemberAccess{expr, *identifier});
        continue;
      }
      if (this->next_is("(")) {
        const std::optional<Vector<ast::Expr *>> args = this->parse__argument_list();
        if (!args.has_value()) {
          return nullptr;
        }
        expr = this->make_expr(ast::Call{expr, std::move(*args)});
        continue;
      }
      return expr;
    }
  }

  std::optional<Vector<ast::Expr *>> parse__argument_list()
  {
    BLI_assert(this->next_is("("));
    this->consume_next();
    if (this->next_is(")")) {
      this->consume_next();
      return Vector<ast::Expr *>();
    }
    Vector<ast::Expr *> args;
    while (true) {
      ast::Expr *arg = this->parse__expression();
      if (!arg) {
        return std::nullopt;
      }
      args.append(arg);
      if (this->next_is(")")) {
        this->consume_next();
        return args;
      }
      if (!this->next_is(",")) {
        this->set_unexpected_token_error();
        return std::nullopt;
      }
      this->consume_next();
    }
  }

  ast::Expr *parse__expression__atom()
  {
    if (this->is_at_end()) {
      this->set_unexpected_end_error();
      return nullptr;
    }
    const Token &peek_token = tokens_[i_];
    switch (peek_token.type) {
      case TokenType::Number: {
        this->consume_next();
        return this->make_expr(ast::NumberLiteral{peek_token.str});
      }
      case TokenType::String: {
        this->consume_next();
        return this->make_expr(ast::StringLiteral{peek_token.str});
      }
      case TokenType::Identifier: {
        this->consume_next();
        return this->make_expr(ast::Identifier{peek_token.str});
      }
      case TokenType::Special: {
        const StringRef str = peek_token.str;
        if (str == "(") {
          this->consume_next();
          ast::Expr *expr = this->parse__expression();
          if (!expr) {
            return nullptr;
          }
          if (!this->next_is(")")) {
            this->set_error(TIP_("Expected ')'"));
            return nullptr;
          }
          this->consume_next();
          return expr;
        }
        this->set_unexpected_token_error();
        return nullptr;
      }
    }
    this->set_error("Unknown token");
    return nullptr;
  }

  std::optional<StringRef> parse__identifier()
  {
    if (this->is_at_end()) {
      this->set_unexpected_end_error();
      return std::nullopt;
    }
    const Token &peek_token = tokens_[i_];
    if (peek_token.type != TokenType::Identifier) {
      this->set_error(TIP_("Expected identifier"));
      return std::nullopt;
    }
    this->consume_next();
    return peek_token.str;
  }

  ast::Expr *parse__expression__generic_ternary(const StringRef delimiter_a,
                                                const StringRef delimiter_b,
                                                const ParseFn fn)
  {
    ast::Expr *condition = (this->*fn)();
    if (!condition) {
      return nullptr;
    }
    if (!this->next_is(delimiter_a)) {
      return condition;
    }
    this->consume_next();
    ast::Expr *true_expr = (this->*fn)();
    if (!true_expr) {
      this->error__expression__generic_ternary(delimiter_a, delimiter_b);
      return nullptr;
    }
    if (!this->next_is(delimiter_b)) {
      this->error__expression__generic_ternary(delimiter_a, delimiter_b);
      return nullptr;
    }
    this->consume_next();
    ast::Expr *false_expr = (this->*fn)();
    if (!false_expr) {
      this->error__expression__generic_ternary(delimiter_a, delimiter_b);
      return nullptr;
    }
    if (this->next_is_any({delimiter_a, delimiter_b})) {
      this->set_unexpected_token_error();
      return nullptr;
    }
    return this->make_expr(ast::ConditionalOp{condition, true_expr, false_expr});
  }

  ast::Expr *parse__expression__generic_binary_single(const Span<StringRef> ops, const ParseFn fn)
  {
    ast::Expr *a = (this->*fn)();
    if (!a) {
      return nullptr;
    }
    if (!this->next_is_any(ops)) {
      return a;
    }
    const StringRef op = this->consume_next().str;
    ast::Expr *b = (this->*fn)();
    if (!b) {
      this->error__expression__generic_binary(op);
      return nullptr;
    }
    if (this->next_is_any(ops)) {
      this->set_unexpected_token_error();
      return nullptr;
    }
    return this->make_expr(ast::BinaryOp{op, a, b});
  }

  ast::Expr *parse__expression__generic_binary_multiple(const Span<StringRef> ops,
                                                        const ParseFn fn)
  {
    ast::Expr *expr = (this->*fn)();
    if (!expr) {
      return nullptr;
    }
    while (true) {
      if (!this->next_is_any(ops)) {
        return expr;
      }
      const StringRef op = this->consume_next().str;
      ast::Expr *b = (this->*fn)();
      if (!b) {
        return nullptr;
      }
      expr = this->make_expr(ast::BinaryOp{op, expr, b});
    }
  }

  ast::Expr *parse__expression__generic_unary(const Span<StringRef> ops, const ParseFn fn)
  {
    if (!this->next_is_any(ops)) {
      return (this->*fn)();
    }
    const StringRef op = this->consume_next().str;
    ast::Expr *expr = this->parse__expression__generic_unary(ops, fn);
    if (!expr) {
      return nullptr;
    }
    return this->make_expr(ast::UnaryOp{op, expr});
  }

  void error__expression__generic_ternary(const StringRef delimiter_a, const StringRef delimiter_b)
  {
    this->set_error(fmt::format(
        "{}: a {} b {} c", TIP_("Expected expression of this form:"), delimiter_a, delimiter_b));
  }

  void error__expression__generic_binary(const StringRef delimiter)
  {
    this->set_error(
        fmt::format("{}: a {} b", TIP_("Expected expression of this form:"), delimiter));
  }

  bool next_is(const StringRef str)
  {
    if (this->is_at_end()) {
      return false;
    }
    return this->tokens_[i_].str == str;
  }

  bool next_is_any(const Span<StringRef> strs)
  {
    if (this->is_at_end()) {
      return false;
    }
    return strs.contains(this->tokens_[i_].str);
  }

  bool is_at_end() const
  {
    return i_ >= tokens_.size();
  }

  const Token &consume_next()
  {
    return tokens_[i_++];
  }

  void set_unexpected_token_error()
  {
    if (this->is_at_end()) {
      this->set_unexpected_end_error();
      return;
    }
    this->set_error(fmt::format("{}: {}", TIP_("Unexpected token"), tokens_[i_].str));
  }

  void set_unexpected_end_error()
  {
    this->set_error(TIP_("Unexpected end of expression"));
  }

  void set_error(std::string error)
  {
    error_ = std::move(error);
  }

  template<typename T> ast::Expr *make_expr(T &&value)
  {
    return &scope_.construct<ast::Expr>(std::forward<T>(value));
  }
};

ParseResult parse(ResourceScope &scope, const StringRef expression)
{
  const TokenizeResult tokenize_result = tokenize(expression);
  if (const auto *error = std::get_if<std::string>(&tokenize_result.result)) {
    return *error;
  }
  const Span<Token> tokens = std::get<Vector<Token>>(tokenize_result.result);
  Parser parser{scope, tokens};
  ast::Expr *expr = parser.parse_all_as_expression();
  if (!expr) {
    if (const std::optional<std::string> &error = parser.error()) {
      return *error;
    }
    return TIP_("Unknown error");
  }
  return expr;
}

}  // namespace blender::nodes::expression
