/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "BLI_map.hh"

#include "DNA_node_types.h"

#include "COM_cached_resource.hh"

namespace blender::nodes::expression {
class ExpressionNodeGroup;
}

namespace blender::compositor {

/* ------------------------------------------------------------------------------------------------
 * Cached Expression Node Group Key.
 */
class CachedExpressionNodeGroupKey {
  /* A sub-key structure for an input of the expression node. */
  class Input {
   public:
    std::string name;
    eNodeSocketDatatype type = SOCK_FLOAT;

    uint64_t hash() const;
    friend bool operator==(const Input &a, const Input &b) = default;
  };

  /* A sub-key structure for an output of the expression node. */
  class Output {
   public:
    std::string name;
    eNodeSocketDatatype type = SOCK_FLOAT;
    std::string expression;

    uint64_t hash() const;
    friend bool operator==(const Output &a, const Output &b) = default;
  };

 public:
  Array<Input> inputs;
  Array<Output> outputs;

  CachedExpressionNodeGroupKey(const bNode &node, Span<StringRef> expressions);

  uint64_t hash() const;
  friend bool operator==(const CachedExpressionNodeGroupKey &a,
                         const CachedExpressionNodeGroupKey &b) = default;
};

/* -------------------------------------------------------------------------------------------------
 * Cached Expression Node Group.
 *
 * A cached resource that constructs and caches an expression node group from the given expression
 * node and expressions. */
class CachedExpressionNodeGroup : public CachedResource {
 public:
  std::shared_ptr<nodes::expression::ExpressionNodeGroup> expression_node_group;

  CachedExpressionNodeGroup(const bNode &expression_node, const Span<StringRef> expressions);
};

/* ------------------------------------------------------------------------------------------------
 * Cached Expression Node Group Container.
 */
class CachedExpressionNodeGroupContainer : public CachedResourceContainer {
 private:
  Map<CachedExpressionNodeGroupKey, std::unique_ptr<CachedExpressionNodeGroup>> map_;

 public:
  void reset() override;

  /* Check if there is an available CachedExpressionNodeGroup cached resource for the given node
   * and expressions, if one exists, return its expression node group, otherwise, return a newly
   * created one and add it to the container. In both cases, tag the cached resource as needed to
   * keep it cached for the next evaluation. */
  nodes::expression::ExpressionNodeGroup &get(const bNode &expression_node,
                                              const Span<StringRef> expressions);
};

}  // namespace blender::compositor
