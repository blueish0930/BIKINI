/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_string_ref.hh"
#include "BLI_utility_mixins.hh"

namespace blender {
struct bNodeSocket;
struct bNodeTree;
struct bNode;
struct Main;
}  // namespace blender

namespace blender::nodes::expression {

class ExpressionNodeGroup : NonCopyable, NonMovable {
 public:
  ~ExpressionNodeGroup();

  bNodeTree *tree = nullptr;
  std::string error;
};

std::shared_ptr<ExpressionNodeGroup> expression_node_to_group(Main *bmain,
                                                              const bNode &node,
                                                              StringRef tree_idname,
                                                              Span<StringRef> expressions,
                                                              Span<int> expr_indices);

}  // namespace blender::nodes::expression
