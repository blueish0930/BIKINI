/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_zones.hh"

#include "DNA_node_types.h"

#include "BLI_function_ref.hh"
#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"
#include "BLI_task.hh"
#include "BLI_ustring.hh"

#include "NOD_geometry_nodes_lazy_function.hh"
#include "NOD_node_declaration.hh"
#include "NOD_socket_usage_inference.hh"

namespace blender {

namespace bke {

static bool node_is_eval_output_terminal(const bNode &node)
{
  if (node.is_group_output()) {
    return true;
  }
  if (node.typeinfo && node.typeinfo->nclass == NODE_CLASS_OUTPUT) {
    return true;
  }
  /* Side-effect outputs that are not NODE_CLASS_OUTPUT. */
  if (node.is_type("GeometryNodeWarning"_ustr) ||
      node.is_type("GeometryNodeSetGroupInputDefault"_ustr))
  {
    return true;
  }
  return false;
}

Set<const bNode *> node_tree_nodes_affecting_output(const bNodeTree &tree)
{
  tree.ensure_topology_cache();
  Set<const bNode *> affecting;
  Stack<const bNode *> stack;
  for (const bNode *node : tree.all_nodes()) {
    if (node_is_eval_output_terminal(*node)) {
      stack.push(node);
    }
  }
  const bNodeTreeZones *zones = tree.zones();
  while (!stack.is_empty()) {
    const bNode *node = stack.pop();
    if (!affecting.add(node)) {
      continue;
    }
    for (const bNodeSocket *input : node->input_sockets()) {
      if (!input->is_available()) {
        continue;
      }
      for (const bNodeLink *link : input->directly_linked_links()) {
        if (link->is_muted() || !link->is_available() || link->fromnode == nullptr) {
          continue;
        }
        stack.push(link->fromnode);
      }
    }
    /* Zones pair input/output without an explicit inner link. */
    if (zones) {
      const bNodeTreeZone *zone = zones->get_zone_by_node(node->identifier);
      if (zone && zone->output_node() == node) {
        if (const bNode *input_node = zone->input_node()) {
          stack.push(input_node);
        }
      }
    }
  }
  return affecting;
}

}  // namespace bke

namespace bke::node_tree_runtime {

void preprocess_geometry_node_tree_for_evaluation(bNodeTree &tree_cow)
{
  BLI_assert(tree_cow.type == NTREE_GEOMETRY);
  /* Rebuild geometry nodes lazy function graph. */
  tree_cow.runtime->geometry_nodes_lazy_function_graph_info_mutex.tag_dirty();
  nodes::ensure_geometry_nodes_lazy_function_graph(tree_cow);
}

static void update_node_vector(const bNodeTree &ntree)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  const Span<bNode *> nodes = tree_runtime.nodes_by_id;
  tree_runtime.group_nodes.clear();
  tree_runtime.has_undefined_nodes_or_sockets = false;
  for (const int i : nodes.index_range()) {
    if (nodes[i] == nullptr || nodes[i]->runtime == nullptr) {
      continue;
    }
    bNode &node = *nodes[i];
    node.runtime->index_in_tree = i;
    node.runtime->owner_tree = const_cast<bNodeTree *>(&ntree);
    tree_runtime.has_undefined_nodes_or_sockets |= node.is_undefined();
    if (node.is_group()) {
      tree_runtime.group_nodes.append(&node);
    }
  }
}

static void update_link_vector(const bNodeTree &ntree)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  tree_runtime.links.clear();
  for (bNodeLink &link : ntree.links) {
    /* Check that the link connects nodes within this tree. */
    BLI_assert(tree_runtime.nodes_by_id.contains(link.fromnode));
    BLI_assert(tree_runtime.nodes_by_id.contains(link.tonode));

    tree_runtime.links.append(&link);
  }
}

static void update_socket_vectors_and_owner_node(const bNodeTree &ntree)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  tree_runtime.sockets.clear();
  tree_runtime.input_sockets.clear();
  tree_runtime.output_sockets.clear();
  for (bNode *node : tree_runtime.nodes_by_id) {
    if (node == nullptr || node->runtime == nullptr) {
      continue;
    }
    bNodeRuntime &node_runtime = *node->runtime;
    node_runtime.inputs.clear();
    node_runtime.outputs.clear();
    for (bNodeSocket &socket : node->inputs) {
      if (socket.runtime == nullptr) {
        continue;
      }
      socket.runtime->index_in_node = node_runtime.inputs.append_and_get_index(&socket);
      socket.runtime->index_in_all_sockets = tree_runtime.sockets.append_and_get_index(&socket);
      socket.runtime->index_in_inout_sockets = tree_runtime.input_sockets.append_and_get_index(
          &socket);
      socket.runtime->owner_node = node;
      tree_runtime.has_undefined_nodes_or_sockets |= socket.typeinfo ==
                                                     &bke::NodeSocketTypeUndefined;
    }
    for (bNodeSocket &socket : node->outputs) {
      if (socket.runtime == nullptr) {
        continue;
      }
      socket.runtime->index_in_node = node_runtime.outputs.append_and_get_index(&socket);
      socket.runtime->index_in_all_sockets = tree_runtime.sockets.append_and_get_index(&socket);
      socket.runtime->index_in_inout_sockets = tree_runtime.output_sockets.append_and_get_index(
          &socket);
      socket.runtime->owner_node = node;
      tree_runtime.has_undefined_nodes_or_sockets |= socket.typeinfo ==
                                                     &bke::NodeSocketTypeUndefined;
    }
  }
}

static void update_panels(const bNodeTree &ntree)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  for (bNode *node : tree_runtime.nodes_by_id) {
    if (node == nullptr || node->runtime == nullptr) {
      continue;
    }
    bNodeRuntime &node_runtime = *node->runtime;
    node_runtime.panels.reinitialize(node->num_panel_states);
  }
}

static void update_internal_link_inputs(const bNodeTree &ntree)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  for (bNode *node : tree_runtime.nodes_by_id) {
    if (node == nullptr || node->runtime == nullptr) {
      continue;
    }
    for (bNodeSocket *socket : node->runtime->outputs) {
      if (socket == nullptr || socket->runtime == nullptr) {
        continue;
      }
      socket->runtime->internal_link_input = nullptr;
    }
    for (bNodeInternalLink &link : node->runtime->internal_links) {
      link.out->runtime->internal_link_input = link.in;
    }
  }
}

static void update_directly_linked_links_and_sockets(const bNodeTree &ntree)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  for (bNode *node : tree_runtime.nodes_by_id) {
    if (node == nullptr || node->runtime == nullptr) {
      continue;
    }
    for (bNodeSocket *socket : node->runtime->inputs) {
      if (socket == nullptr || socket->runtime == nullptr) {
        continue;
      }
      socket->runtime->directly_linked_links.clear();
      socket->runtime->directly_linked_sockets.clear();
    }
    for (bNodeSocket *socket : node->runtime->outputs) {
      if (socket == nullptr || socket->runtime == nullptr) {
        continue;
      }
      socket->runtime->directly_linked_links.clear();
      socket->runtime->directly_linked_sockets.clear();
    }
    node->runtime->has_available_linked_inputs = false;
    node->runtime->has_available_linked_outputs = false;
  }
  for (bNodeLink *link : tree_runtime.links) {
    if (link->fromsock == nullptr || link->tosock == nullptr || link->fromnode == nullptr ||
        link->tonode == nullptr || link->fromsock->runtime == nullptr ||
        link->tosock->runtime == nullptr)
    {
      continue;
    }
    link->fromsock->runtime->directly_linked_links.append(link);
    link->fromsock->runtime->directly_linked_sockets.append(link->tosock);
    link->tosock->runtime->directly_linked_links.append(link);
    if (link->is_available()) {
      link->fromnode->runtime->has_available_linked_outputs = true;
      link->tonode->runtime->has_available_linked_inputs = true;
    }
    BLI_assert(link->fromsock->runtime->owner_node == link->fromnode);
    BLI_assert(link->tosock->runtime->owner_node == link->tonode);
  }
  for (bNodeSocket *socket : tree_runtime.input_sockets) {
    if (socket->flag & SOCK_MULTI_INPUT) {
      std::ranges::sort(socket->runtime->directly_linked_links,
                        [&](const bNodeLink *a, const bNodeLink *b) {
                          return a->multi_input_sort_id > b->multi_input_sort_id;
                        });
    }
  }
  for (bNodeSocket *socket : tree_runtime.input_sockets) {
    for (bNodeLink *link : socket->runtime->directly_linked_links) {
      /* Do this after sorting the input links. */
      socket->runtime->directly_linked_sockets.append(link->fromsock);
    }
  }
}

static void find_logical_origins_for_socket_recursive(
    bNodeSocket &input_socket,
    bool only_follow_first_input_link,
    Vector<bNodeSocket *, 16> &sockets_in_current_chain,
    Vector<bNodeSocket *> &r_logical_origins,
    Vector<bNodeSocket *> &r_skipped_origins)
{
  if (sockets_in_current_chain.contains(&input_socket)) {
    /* Protect against reroute recursions. */
    return;
  }
  sockets_in_current_chain.append(&input_socket);

  Span<bNodeLink *> links_to_check = input_socket.runtime->directly_linked_links;
  if (only_follow_first_input_link) {
    links_to_check = links_to_check.take_front(1);
  }
  for (bNodeLink *link : links_to_check) {
    if (link->is_muted()) {
      continue;
    }
    if (!link->is_available()) {
      continue;
    }
    bNodeSocket &origin_socket = *link->fromsock;
    bNode &origin_node = *link->fromnode;
    if (!origin_socket.is_available()) {
      /* Non available sockets are ignored. */
      continue;
    }
    if (origin_node.is_reroute()) {
      bNodeSocket &reroute_input = *origin_node.runtime->inputs[0];
      bNodeSocket &reroute_output = *origin_node.runtime->outputs[0];
      r_skipped_origins.append(&reroute_input);
      r_skipped_origins.append(&reroute_output);
      find_logical_origins_for_socket_recursive(
          reroute_input, false, sockets_in_current_chain, r_logical_origins, r_skipped_origins);
      continue;
    }
    /* Cordless named portals in shader/compositor/image: consumers of a Named Portal
     * logically originate at the matching Store Named Portal output. Geometry Nodes
     * keep their own lazy-function wiring and are excluded here. */
    if (node_is_named_portal(origin_node) && origin_node.type_legacy != GEO_NODE_NAMED_PORTAL) {
      r_skipped_origins.append(&origin_socket);
      /* Avoid topology-cache accessors: this runs while the cache is being rebuilt. */
      const bNodeTree *tree = origin_node.runtime->owner_tree;
      if (tree) {
        const bNode *store = node_find_unique_store_named_portal(*tree, node_named_portal_name(origin_node));
        if (store && origin_node.storage && store->storage) {
          const auto &get_storage = *static_cast<const NodeGeometryPortal *>(origin_node.storage);
          const auto &store_storage = *static_cast<const NodeGeometryPortal *>(store->storage);
          if (get_storage.data_type == store_storage.data_type &&
              !store->runtime->outputs.is_empty())
          {
            r_logical_origins.append(store->runtime->outputs[0]);
          }
        }
      }
      continue;
    }
    if (origin_node.is_muted()) {
      if (bNodeSocket *mute_input = origin_socket.runtime->internal_link_input) {
        r_skipped_origins.append(&origin_socket);
        r_skipped_origins.append(mute_input);
        find_logical_origins_for_socket_recursive(
            *mute_input, true, sockets_in_current_chain, r_logical_origins, r_skipped_origins);
      }
      continue;
    }
    r_logical_origins.append(&origin_socket);
  }

  sockets_in_current_chain.pop_last();
}

static void update_logically_linked_sockets(const bNodeTree &ntree)
{
  /* Compute logically linked sockets to inputs. */
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  Span<bNode *> nodes = tree_runtime.nodes_by_id;
  threading::parallel_for(nodes.index_range(), 128, [&](const IndexRange range) {
    for (const int i : range) {
      bNode *node = nodes[i];
      if (node == nullptr || node->runtime == nullptr) {
        continue;
      }
      for (bNodeSocket *socket : node->runtime->inputs) {
        if (socket == nullptr || socket->runtime == nullptr) {
          continue;
        }
        Vector<bNodeSocket *, 16> sockets_in_current_chain;
        socket->runtime->logically_linked_sockets.clear();
        socket->runtime->logically_linked_skipped_sockets.clear();
        find_logical_origins_for_socket_recursive(
            *socket,
            false,
            sockets_in_current_chain,
            socket->runtime->logically_linked_sockets,
            socket->runtime->logically_linked_skipped_sockets);
      }
    }
  });

  /* Cordless GET portals: hidden Value input originates at the matching Store. */
  for (bNode *node : nodes) {
    if (node == nullptr || node->runtime == nullptr) {
      continue;
    }
    if (!node_is_named_portal(*node) || node->type_legacy == GEO_NODE_NAMED_PORTAL) {
      continue;
    }
    if (node->runtime->inputs.is_empty() || !node->storage) {
      continue;
    }
    bNodeSocket &in = *node->runtime->inputs[0];
    if (!in.runtime->logically_linked_sockets.is_empty()) {
      continue;
    }
    const bNodeTree *tree = node->runtime->owner_tree;
    if (!tree) {
      continue;
    }
    const bNode *store = node_find_unique_store_named_portal(*tree, node_named_portal_name(*node));
    if (!store || !store->storage || store->runtime->outputs.is_empty()) {
      continue;
    }
    const auto &get_storage = *static_cast<const NodeGeometryPortal *>(node->storage);
    const auto &store_storage = *static_cast<const NodeGeometryPortal *>(store->storage);
    if (get_storage.data_type != store_storage.data_type) {
      continue;
    }
    in.runtime->logically_linked_sockets.append(store->runtime->outputs[0]);
  }

  /* Clear logically linked sockets to outputs. */
  threading::parallel_for(nodes.index_range(), 128, [&](const IndexRange range) {
    for (const int i : range) {
      bNode &node = *nodes[i];
      for (bNodeSocket *socket : node.runtime->outputs) {
        socket->runtime->logically_linked_sockets.clear();
      }
    }
  });

  /* Compute logically linked sockets to outputs using the previously computed logically linked
   * sockets to inputs. */
  for (const bNode *node : nodes) {
    for (bNodeSocket *input_socket : node->runtime->inputs) {
      for (bNodeSocket *output_socket : input_socket->runtime->logically_linked_sockets) {
        output_socket->runtime->logically_linked_sockets.append(input_socket);
      }
    }
  }
}

static void update_nodes_by_type(const bNodeTree &ntree)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  tree_runtime.nodes_by_type.clear();
  for (bNode *node : tree_runtime.nodes_by_id) {
    if (node == nullptr || node->typeinfo == nullptr) {
      continue;
    }
    tree_runtime.nodes_by_type.add(node->typeinfo, node);
  }
}

static void update_sockets_by_identifier(const bNodeTree &ntree)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  Span<bNode *> nodes = tree_runtime.nodes_by_id;
  threading::parallel_for(nodes.index_range(), 128, [&](const IndexRange range) {
    for (bNode *node : nodes.slice(range)) {
      if (node == nullptr || node->runtime == nullptr) {
        continue;
      }
      node->runtime->inputs_by_identifier.clear();
      node->runtime->outputs_by_identifier.clear();
      for (bNodeSocket *socket : node->runtime->inputs) {
        if (socket == nullptr || socket->runtime == nullptr) {
          continue;
        }
        BLI_assert(socket->identifier == socket->identifier_ustr());
        node->runtime->inputs_by_identifier.add_new(socket->identifier_ustr(), socket);
      }
      for (bNodeSocket *socket : node->runtime->outputs) {
        if (socket == nullptr || socket->runtime == nullptr) {
          continue;
        }
        BLI_assert(socket->identifier == socket->identifier_ustr());
        node->runtime->outputs_by_identifier.add_new(socket->identifier_ustr(), socket);
      }
    }
  });
}

enum class ToposortDirection {
  LeftToRight,
  RightToLeft,
};

struct ToposortNodeState {
  bool is_done = false;
  bool is_in_stack = false;
};

static StringRef portal_name(const bNode &node)
{
  if (!node.storage) {
    return {};
  }
  return static_cast<const NodeGeometryPortal *>(node.storage)->name;
}

static Vector<const bNode *> store_named_portal_sources(const bNodeTree &ntree,
                                                        const bNode &named_portal)
{
  Vector<const bNode *> sources;
  if (!node_is_named_portal(named_portal)) {
    return sources;
  }
  const StringRef name = portal_name(named_portal);
  if (name.is_empty()) {
    return sources;
  }
  for (const bNode *node : ntree.runtime->nodes_by_id) {
    if (node_is_store_named_portal(*node) && portal_name(*node) == name) {
      sources.append(node);
    }
  }
  if (sources.size() != 1) {
    sources.clear();
  }
  return sources;
}

static Vector<const bNode *> get_implicit_origin_nodes(const bNodeTree &ntree, bNode &node)
{
  Vector<const bNode *> origin_nodes;
  origin_nodes.extend(store_named_portal_sources(ntree, node));
  if (all_zone_output_node_types().contains(node.type_legacy)) {
    const bNodeZoneType *zone_type = zone_type_by_node_type(node.type_legacy);
    const bke::bNodeType *input_type = zone_type ?
                                            bke::node_type_find(zone_type->input_idname) :
                                            nullptr;
    if (zone_type && input_type) {
      /* Can't use #zone_type.get_corresponding_input because that expects the topology cache to be
       * build already, but we are still building it here. */
      for (const bNode *input_node : ntree.runtime->nodes_by_type.lookup(input_type)) {
        if (input_node == nullptr) {
          continue;
        }
        if (zone_type->output_id_requires_storage() && input_node->storage == nullptr) {
          continue;
        }
        if (zone_type->get_corresponding_output_id(*input_node) == node.identifier) {
          origin_nodes.append(input_node);
        }
      }
    }
  }
  return origin_nodes;
}

static Vector<const bNode *> get_implicit_target_nodes(const bNodeTree &ntree, bNode &node)
{
  Vector<const bNode *> target_nodes;
  if (node_is_store_named_portal(node)) {
    const StringRef name = portal_name(node);
    if (!name.is_empty()) {
      int source_count = 0;
      for (const bNode *source : ntree.runtime->nodes_by_id) {
        source_count += node_is_store_named_portal(*source) && portal_name(*source) == name;
      }
      if (source_count == 1) {
        for (const bNode *target : ntree.runtime->nodes_by_id) {
          if (node_is_named_portal(*target) && portal_name(*target) == name) {
            target_nodes.append(target);
          }
        }
      }
    }
  }
  if (all_zone_input_node_types().contains(node.type_legacy)) {
    const bNodeZoneType *zone_type = zone_type_by_node_type(node.type_legacy);
    if (zone_type) {
      if (const bNode *output_node = zone_type->get_corresponding_output(ntree, node)) {
        target_nodes.append(output_node);
      }
    }
  }
  return target_nodes;
}

static void toposort_from_start_node(const bNodeTree &ntree,
                                     const ToposortDirection direction,
                                     bNode &start_node,
                                     MutableSpan<ToposortNodeState> node_states,
                                     Vector<bNode *> &r_sorted_nodes,
                                     bool &r_cycle_detected)
{
  struct Item {
    bNode *node;
    int socket_index = 0;
    int link_index = 0;
    int implicit_link_index = 0;
  };

  Stack<Item, 64> nodes_to_check;
  nodes_to_check.push({&start_node});
  node_states[start_node.index()].is_in_stack = true;
  while (!nodes_to_check.is_empty()) {
    Item &item = nodes_to_check.peek();
    bNode &node = *item.node;
    bool pushed_node = false;

    auto handle_linked_node = [&](bNode &linked_node) {
      ToposortNodeState &linked_node_state = node_states[linked_node.index()];
      if (linked_node_state.is_done) {
        /* The linked node has already been visited. */
        return true;
      }
      if (linked_node_state.is_in_stack) {
        r_cycle_detected = true;
      }
      else {
        nodes_to_check.push({&linked_node});
        linked_node_state.is_in_stack = true;
        pushed_node = true;
      }
      return false;
    };

    if (node.runtime == nullptr) {
      nodes_to_check.pop();
      continue;
    }
    const Span<bNodeSocket *> sockets = (direction == ToposortDirection::LeftToRight) ?
                                            node.runtime->inputs :
                                            node.runtime->outputs;
    while (true) {
      if (item.socket_index == sockets.size()) {
        /* All sockets have already been visited. */
        break;
      }
      bNodeSocket *socket = sockets[item.socket_index];
      if (socket == nullptr || socket->runtime == nullptr) {
        item.socket_index++;
        item.link_index = 0;
        continue;
      }
      const Span<bNodeLink *> linked_links = socket->runtime->directly_linked_links;
      if (item.link_index == linked_links.size()) {
        /* All links connected to this socket have already been visited. */
        item.socket_index++;
        item.link_index = 0;
        continue;
      }
      bNodeLink *link = linked_links[item.link_index];
      if (link == nullptr || !link->is_available()) {
        /* Ignore unavailable links. */
        item.link_index++;
        continue;
      }
      if (item.link_index >= socket->runtime->directly_linked_sockets.size()) {
        item.link_index++;
        continue;
      }
      bNodeSocket *linked_socket = socket->runtime->directly_linked_sockets[item.link_index];
      if (linked_socket == nullptr || linked_socket->runtime == nullptr) {
        item.link_index++;
        continue;
      }
      bNode *owner = linked_socket->runtime->owner_node;
      if (owner == nullptr || owner->runtime == nullptr) {
        item.link_index++;
        continue;
      }
      bool owner_in_tree = false;
      for (bNode *tree_node : ntree.runtime->nodes_by_id) {
        if (tree_node == owner) {
          owner_in_tree = true;
          break;
        }
      }
      if (!owner_in_tree) {
        item.link_index++;
        continue;
      }
      if (handle_linked_node(*owner)) {
        /* The linked node has already been visited. */
        item.link_index++;
        continue;
      }
      break;
    }

    if (!pushed_node) {
      /* Some nodes are internally linked without an explicit `bNodeLink`. The toposort should
       * still order them correctly and find cycles. */
      const Vector<const bNode *> implicitly_linked_nodes =
          (direction == ToposortDirection::LeftToRight) ? get_implicit_origin_nodes(ntree, node) :
                                                          get_implicit_target_nodes(ntree, node);
      while (true) {
        if (item.implicit_link_index == implicitly_linked_nodes.size()) {
          /* All implicitly linked nodes have already been visited. */
          break;
        }
        const bNode *linked_node = implicitly_linked_nodes[item.implicit_link_index];
        if (linked_node == nullptr || linked_node->runtime == nullptr) {
          item.implicit_link_index++;
          continue;
        }
        if (handle_linked_node(const_cast<bNode &>(*linked_node))) {
          /* The implicitly linked node has already been visited. */
          item.implicit_link_index++;
          continue;
        }
        break;
      }
    }

    /* If no other element has been pushed, the current node can be pushed to the sorted list.
     */
    if (!pushed_node) {
      ToposortNodeState &node_state = node_states[node.index()];
      node_state.is_done = true;
      node_state.is_in_stack = false;
      r_sorted_nodes.append(&node);
      nodes_to_check.pop();
    }
  }
}

static void update_toposort(const bNodeTree &ntree,
                            const ToposortDirection direction,
                            Vector<bNode *> &r_sorted_nodes,
                            bool &r_cycle_detected)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  r_sorted_nodes.clear();
  r_sorted_nodes.reserve(tree_runtime.nodes_by_id.size());
  r_cycle_detected = false;

  Array<ToposortNodeState> node_states(tree_runtime.nodes_by_id.size());
  for (bNode *node : tree_runtime.nodes_by_id) {
    if (node == nullptr || node->runtime == nullptr) {
      continue;
    }
    if (node_states[node->index()].is_done) {
      /* Ignore nodes that are done already. */
      continue;
    }
    if ((direction == ToposortDirection::LeftToRight) ?
            node->runtime->has_available_linked_outputs :
            node->runtime->has_available_linked_inputs)
    {
      /* Ignore non-start nodes. */
      continue;
    }
    toposort_from_start_node(
        ntree, direction, *node, node_states, r_sorted_nodes, r_cycle_detected);
  }

  if (r_sorted_nodes.size() < tree_runtime.nodes_by_id.size()) {
    r_cycle_detected = true;
    for (bNode *node : tree_runtime.nodes_by_id) {
      if (node == nullptr || node->runtime == nullptr) {
        continue;
      }
      if (node_states[node->index()].is_done) {
        /* Ignore nodes that are done already. */
        continue;
      }
      /* Start toposort at this node which is somewhere in the middle of a loop. */
      toposort_from_start_node(
          ntree, direction, *node, node_states, r_sorted_nodes, r_cycle_detected);
    }
  }

  BLI_assert(tree_runtime.nodes_by_id.size() == r_sorted_nodes.size());
}

static void update_root_frames(const bNodeTree &ntree)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  Span<bNode *> nodes = tree_runtime.nodes_by_id;

  tree_runtime.root_frames.clear();

  for (bNode *node : nodes) {
    if (!node->parent && node->is_frame()) {
      tree_runtime.root_frames.append(node);
    }
  }
}

static void update_direct_frames_childrens(const bNodeTree &ntree)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  Span<bNode *> nodes = tree_runtime.nodes_by_id;

  for (bNode *node : nodes) {
    node->runtime->direct_children_in_frame.clear();
  }

  for (bNode *node : nodes) {
    if (const bNode *frame = node->parent) {
      frame->runtime->direct_children_in_frame.append(node);
    }
  }
}

static void update_group_output_node(const bNodeTree &ntree)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  const bke::bNodeType *node_type = bke::node_type_find("NodeGroupOutput"_ustr);
  if (node_type == nullptr) {
    tree_runtime.group_output_node = nullptr;
    return;
  }
  const Span<bNode *> group_output_nodes = tree_runtime.nodes_by_type.lookup(node_type);
  if (group_output_nodes.is_empty()) {
    tree_runtime.group_output_node = nullptr;
  }
  else if (group_output_nodes.size() == 1) {
    tree_runtime.group_output_node = group_output_nodes[0];
  }
  else {
    tree_runtime.group_output_node = nullptr;
    for (bNode *group_output : group_output_nodes) {
      if (group_output->flag & NODE_DO_OUTPUT) {
        tree_runtime.group_output_node = group_output;
        break;
      }
    }
  }
}

static void update_dangling_reroute_nodes(const bNodeTree &ntree)
{
  for (const bNode *node : ntree.runtime->toposort_left_to_right) {
    if (node == nullptr || node->runtime == nullptr) {
      continue;
    }
    bNodeRuntime &node_runtime = *node->runtime;
    if (!node->is_reroute()) {
      node_runtime.is_dangling_reroute = false;
      continue;
    }
    if (node_runtime.inputs.is_empty() || node_runtime.inputs[0] == nullptr ||
        node_runtime.inputs[0]->runtime == nullptr)
    {
      node_runtime.is_dangling_reroute = true;
      continue;
    }
    const Span<const bNodeLink *> links = node_runtime.inputs[0]->runtime->directly_linked_links;
    if (links.is_empty() || links.first() == nullptr || links.first()->fromnode == nullptr ||
        links.first()->fromnode->runtime == nullptr)
    {
      node_runtime.is_dangling_reroute = true;
      continue;
    }
    BLI_assert(links.size() == 1);
    const bNode &source_node = *links.first()->fromnode;
    node_runtime.is_dangling_reroute = source_node.runtime->is_dangling_reroute;
  }
}

static void ensure_topology_cache(const bNodeTree &ntree)
{
  bNodeTreeRuntime &tree_runtime = *ntree.runtime;
  tree_runtime.topology_cache_mutex.ensure([&]() {
    update_node_vector(ntree);
    update_link_vector(ntree);
    update_socket_vectors_and_owner_node(ntree);
    update_panels(ntree);
    update_internal_link_inputs(ntree);
    update_directly_linked_links_and_sockets(ntree);
    update_nodes_by_type(ntree);
    threading::parallel_invoke(
        tree_runtime.nodes_by_id.size() > 32,
        [&]() { update_logically_linked_sockets(ntree); },
        [&]() { update_sockets_by_identifier(ntree); },
        [&]() {
          update_toposort(ntree,
                          ToposortDirection::LeftToRight,
                          tree_runtime.toposort_left_to_right,
                          tree_runtime.has_available_link_cycle);
          for (const int i : tree_runtime.toposort_left_to_right.index_range()) {
            const bNode *node = tree_runtime.toposort_left_to_right[i];
            if (node == nullptr || node->runtime == nullptr) {
              continue;
            }
            node->runtime->toposort_left_to_right_index = i;
          }
        },
        [&]() {
          bool dummy;
          update_toposort(
              ntree, ToposortDirection::RightToLeft, tree_runtime.toposort_right_to_left, dummy);
          for (const int i : tree_runtime.toposort_right_to_left.index_range()) {
            const bNode *node = tree_runtime.toposort_right_to_left[i];
            if (node == nullptr || node->runtime == nullptr) {
              continue;
            }
            node->runtime->toposort_right_to_left_index = i;
          }
        },
        [&]() { update_root_frames(ntree); },
        [&]() { update_direct_frames_childrens(ntree); });
    update_group_output_node(ntree);
    update_dangling_reroute_nodes(ntree);
    tree_runtime.topology_cache_exists = true;
  });
}

}  // namespace bke::node_tree_runtime

namespace bke {

NodeLinkKey::NodeLinkKey(const bNodeLink &link)
{
  to_node_id_ = link.tonode->identifier;
  input_socket_index_ = link.tosock->index();
  input_link_index_ =
      const_cast<const bNodeSocket *>(link.tosock)->directly_linked_links().first_index(&link);
}

bNodeLink *NodeLinkKey::try_find(bNodeTree &ntree) const
{
  return const_cast<bNodeLink *>(this->try_find(const_cast<const bNodeTree &>(ntree)));
}

const bNodeLink *NodeLinkKey::try_find(const bNodeTree &ntree) const
{
  const bNode *to_node = ntree.node_by_id(to_node_id_);
  if (!to_node) {
    return nullptr;
  }
  if (input_socket_index_ >= to_node->input_sockets().size()) {
    return nullptr;
  }
  const bNodeSocket &input_socket = to_node->input_socket(input_socket_index_);
  if (input_link_index_ >= input_socket.directly_linked_links().size()) {
    return nullptr;
  }
  return input_socket.directly_linked_links()[input_link_index_];
}

}  // namespace bke

void bNodeTree::ensure_topology_cache() const
{
  if (this->runtime == nullptr) {
    return;
  }
  bke::node_tree_runtime::ensure_topology_cache(*this);
}

const bNestedNodeRef *bNodeTree::find_nested_node_ref(const int32_t nested_node_id) const
{
  for (const bNestedNodeRef &ref : this->nested_node_refs_span()) {
    if (ref.id == nested_node_id) {
      return &ref;
    }
  }
  return nullptr;
}

const bNestedNodeRef *bNodeTree::nested_node_ref_from_node_id_path(
    const Span<int32_t> node_ids) const
{
  if (node_ids.is_empty()) {
    return nullptr;
  }
  for (const bNestedNodeRef &ref : this->nested_node_refs_span()) {
    Vector<int> current_node_ids;
    if (this->node_id_path_from_nested_node_ref(ref.id, current_node_ids)) {
      if (current_node_ids.as_span() == node_ids) {
        return &ref;
      }
    }
  }
  return nullptr;
}

bool bNodeTree::node_id_path_from_nested_node_ref(const int32_t nested_node_id,
                                                  Vector<int> &r_node_ids) const
{
  const bNestedNodeRef *ref = this->find_nested_node_ref(nested_node_id);
  if (ref == nullptr) {
    return false;
  }
  const int32_t node_id = ref->path.node_id;
  const bNode *node = this->node_by_id(node_id);
  if (node == nullptr) {
    return false;
  }
  r_node_ids.append(node_id);
  if (!node->is_group()) {
    return true;
  }
  const bNodeTree *group = reinterpret_cast<const bNodeTree *>(node->id);
  if (group == nullptr) {
    return false;
  }
  return group->node_id_path_from_nested_node_ref(ref->path.id_in_node, r_node_ids);
}

const bNode *bNodeTree::find_nested_node(const int32_t nested_node_id,
                                         const bNodeTree **r_tree) const
{
  const bNestedNodeRef *ref = this->find_nested_node_ref(nested_node_id);
  if (ref == nullptr) {
    return nullptr;
  }
  const int32_t node_id = ref->path.node_id;
  const bNode *node = this->node_by_id(node_id);
  if (node == nullptr) {
    return nullptr;
  }
  if (!node->is_group()) {
    if (r_tree) {
      *r_tree = this;
    }
    return node;
  }
  const bNodeTree *group = reinterpret_cast<const bNodeTree *>(node->id);
  if (group == nullptr) {
    return nullptr;
  }
  return group->find_nested_node(ref->path.id_in_node, r_tree);
}

const bNodeSocket &bNode::socket_by_decl(const nodes::SocketDeclaration &decl) const
{
  return decl.in_out == SOCK_IN ? this->input_socket(decl.index) : this->output_socket(decl.index);
}

bNodeSocket &bNode::socket_by_decl(const nodes::SocketDeclaration &decl)
{
  return decl.in_out == SOCK_IN ? this->input_socket(decl.index) : this->output_socket(decl.index);
}

static void ensure_inference_usage_cache(const bNodeTree &tree)
{
  tree.runtime->inferenced_input_socket_usage_mutex.ensure([&]() {
    tree.runtime->inferenced_socket_usage = nodes::socket_usage_inference::infer_all_sockets_usage(
        tree);
  });
}

bool bNodeSocket::affects_node_output() const
{
  BLI_assert(this->is_input());
  BLI_assert(bke::node_tree_runtime::topology_cache_is_available(*this));
  const bNodeTree &tree = this->owner_tree();
  ensure_inference_usage_cache(tree);
  return tree.runtime->inferenced_socket_usage[this->index_in_tree()].is_used;
}

bool bNodeSocket::inferred_socket_visibility() const
{
  BLI_assert(bke::node_tree_runtime::topology_cache_is_available(*this));
  if (this->runtime == nullptr || this->runtime->owner_node == nullptr) {
    return true;
  }
  const bNode &node = this->owner_node();
  if (node.typeinfo == nullptr || node.typeinfo->ignore_inferred_input_socket_visibility) {
    return true;
  }
  const bNodeTree &tree = this->owner_tree();

  ensure_inference_usage_cache(tree);
  const int index = this->index_in_tree();
  if (!tree.runtime->inferenced_socket_usage.index_range().contains(index)) {
    return true;
  }
  return tree.runtime->inferenced_socket_usage[index].is_visible;
}

static Map<uint, ImageProcessTreeDirtyState> &image_process_tree_dirty_states()
{
  static Map<uint, ImageProcessTreeDirtyState> states;
  return states;
}

ImageProcessTreeDirtyState &image_process_tree_dirty_state(const uint session_uid)
{
  return image_process_tree_dirty_states().lookup_or_add_default(session_uid);
}

void image_process_tree_dirty_state_clear(const uint session_uid)
{
  if (ImageProcessTreeDirtyState *state = image_process_tree_dirty_states().lookup_ptr(session_uid))
  {
    state->dirty_node_ids.clear();
    state->topology_dirty = false;
    state->viewer_preview_only = false;
  }
}

}  // namespace blender
