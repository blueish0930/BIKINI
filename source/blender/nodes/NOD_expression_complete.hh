/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

struct bNode;

namespace blender::nodes::expression {

enum class CompletionCategory {
  Variable,
  Operator,
  Float,
  Vector,
  Color,
  Matrix,
  Rotation,
  Member,
};

enum class CompletionKind {
  Variable,
  Function,
  Operator,
  Member,
  Keyword,
  /** Non-selectable section header in the search list. */
  Header,
};

struct CompletionItem {
  /** Short identifier / operator (for matching & table lookup). */
  StringRef name;
  /** Panel / section label (Float, Matrix, …). */
  StringRef category_label;
  CompletionCategory category = CompletionCategory::Float;
  CompletionKind kind = CompletionKind::Function;
  /**
   * Text actually inserted into the expression (never a description).
   * Empty for #CompletionKind::Header.
   */
  StringRef insert_text;
  /**
   * Usage / signature shown in the search list (right-hand hint), e.g. "sin(x) — sine".
   * Never written into the expression field.
   */
  StringRef usage;
  /** When true, append `(` after #insert_text if not already present. */
  bool insert_call_parens = false;
};

/** Built-in operators / functions / members (static table). */
Span<CompletionItem> builtin_completions();

/**
 * Built-ins plus the Expression node's input variable names.
 * #r_owned_names holds storage for variable name strings so #CompletionItem::name stays valid.
 */
void gather_completions(const bNode &node,
                        Vector<std::string> &r_owned_names,
                        Vector<CompletionItem> &r_items);

/**
 * Find the identifier (or member after `.`) at the end of #expression that should be completed.
 * \return false if there is nothing to complete.
 */
bool find_completion_token(StringRef expression,
                           int &r_token_start,
                           int &r_token_len,
                           bool &r_is_member);

/** Replace the token range with #completion (optionally adding call parentheses). */
std::string apply_completion(StringRef expression,
                             int token_start,
                             int token_len,
                             StringRef completion,
                             bool insert_call_parens);

StringRefNull category_label(CompletionCategory category);

}  // namespace blender::nodes::expression
