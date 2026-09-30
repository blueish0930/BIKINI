/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <variant>

#include "NOD_expression_ast.hh"

#include "BLI_resource_scope.hh"

namespace blender::nodes::expression {

using ParseResult = std::variant<ast::Expr *, std::string>;

ParseResult parse(ResourceScope &scope, StringRef expression);

}  // namespace blender::nodes::expression
