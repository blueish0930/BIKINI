/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <cstdint>
#include <memory>

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_hash.hh"
#include "BLI_span.hh"

#include "NOD_expression_to_nodes.hh"

#include "BKE_node_runtime.hh"

#include "COM_cached_expression_node_group.hh"

namespace blender::compositor {

/* --------------------------------------------------------------------
 * Cached Expression Node Group Key.
 */

uint64_t CachedExpressionNodeGroupKey::Input::hash() const
{
  return get_default_hash(this->name, this->type);
}

uint64_t CachedExpressionNodeGroupKey::Output::hash() const
{
  return get_default_hash(this->name, this->type, this->expression);
}

CachedExpressionNodeGroupKey::CachedExpressionNodeGroupKey(const bNode &node,
                                                           const Span<StringRef> expressions)
{
  const NodeExpression &storage = *static_cast<NodeExpression *>(node.storage);

  this->inputs = Array<Input>(storage.input_items.items_num);
  for (const int i : IndexRange(storage.input_items.items_num)) {
    const NodeExpressionInputItem &item = storage.input_items.items[i];
    this->inputs[i] = Input(item.name, eNodeSocketDatatype(item.socket_type));
  }

  this->outputs = Array<Output>(storage.expression_items.items_num);
  for (const int i : IndexRange(storage.expression_items.items_num)) {
    const NodeExpressionItem &item = storage.expression_items.items[i];
    this->outputs[i] = Output(item.name, eNodeSocketDatatype(item.socket_type), expressions[i]);
  }
}

uint64_t CachedExpressionNodeGroupKey::hash() const
{
  return get_default_hash(this->inputs, this->outputs);
}

/* --------------------------------------------------------------------
 * Cached Expression Node Group.
 */

CachedExpressionNodeGroup::CachedExpressionNodeGroup(const bNode &node,
                                                     const Span<StringRef> expressions)
{
  Array<int> expression_indices(expressions.size());
  array_utils::fill_index_range<int>(expression_indices);
  this->expression_node_group = nodes::expression::expression_node_to_group(
      nullptr, node, node.owner_tree().idname, expressions, expression_indices);
}

/* --------------------------------------------------------------------
 * Cached Expression Node Group Container.
 */

void CachedExpressionNodeGroupContainer::reset()
{
  /* First, delete all resources that are no longer needed. */
  map_.remove_if([](auto item) { return !item.value->needed; });

  /* Second, reset the needed status of the remaining resources to false to ready them to track
   * their needed status for the next evaluation. */
  for (auto &value : map_.values()) {
    value->needed = false;
  }
}

nodes::expression::ExpressionNodeGroup &CachedExpressionNodeGroupContainer::get(
    const bNode &node, const Span<StringRef> expressions)
{
  const CachedExpressionNodeGroupKey key(node, expressions);

  auto &cached_shader = *map_.lookup_or_add_cb(
      key, [&]() { return std::make_unique<CachedExpressionNodeGroup>(node, expressions); });

  cached_shader.needed = true;
  return *cached_shader.expression_node_group.get();
}

}  // namespace blender::compositor
