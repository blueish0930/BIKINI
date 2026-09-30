/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <variant>

#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

namespace blender::nodes::expression {

enum class TokenType {
  /**
   * A token that is likely a number (it starts with a digit). It still needs to be parsed properly
   * as a separate step. E.g. this may contain `123`, `0x4` but also `0.xxx03`.
   */
  Number,
  /**
   * An valid identifier that starts with an ascii letter or underscore. Following characters can
   * be the same and additionally digits.
   */
  Identifier,
  /**
   * A quoted string including the quotes.
   */
  String,
  /**
   * A set of special symbols like +-, etc.
   */
  Special,
};

struct Token {
  TokenType type;
  StringRef str;
};

struct TokenizeResult {
  std::variant<Vector<Token>, std::string> result;
};

TokenizeResult tokenize(const StringRef expression);

}  // namespace blender::nodes::expression
