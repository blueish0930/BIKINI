/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_listbase_iterator.hh"
#include "BLI_set.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

#include "BKE_main.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_node_tree_zones.hh"

#include "ED_node.hh"

#include "NOD_geo_closure.hh"
#include "NOD_socket_declarations.hh"

#include "node_intern.hh"

namespace blender::ed::space_node {

static bNodeTree *node_owner_tree_or_null(const bNode &node)
{
  if (node.runtime == nullptr) {
    return nullptr;
  }
  return node.runtime->owner_tree;
}

static bNodeTree *owner_tree_or_null(const bNode &node)
{
  return node_owner_tree_or_null(node);
}

static bNodeTree *owner_tree_or_null(const bNodeSocket &socket)
{
  if (socket.runtime == nullptr || socket.runtime->owner_node == nullptr) {
    return nullptr;
  }
  return node_owner_tree_or_null(*socket.runtime->owner_node);
}

static void ensure_node_owner_tree(bNodeTree &tree, const bNode &node)
{
  if (tree.runtime == nullptr) {
    return;
  }
  tree.ensure_topology_cache();
  if (node.runtime && node.runtime->owner_tree == nullptr) {
    tree.runtime->topology_cache_mutex.tag_dirty();
    tree.ensure_topology_cache();
  }
  if (node.runtime && node.runtime->owner_tree == nullptr &&
      tree.runtime->nodes_by_id.contains(const_cast<bNode *>(&node)))
  {
    node.runtime->owner_tree = &tree;
  }
}

static bool is_set_default_closure_input(const bNodeSocket &socket, const int index)
{
  return socket.owner_node().is_type("GeometryNodeSetClosureDefault"_ustr) && socket.is_input() &&
         socket.index() == index;
}

static bool is_set_default_closure_main_output(const bNodeSocket &socket)
{
  return socket.owner_node().is_type("GeometryNodeSetClosureDefault"_ustr) && socket.is_output() &&
         socket.index() == 0;
}

static const bNode *find_set_default_closure_upstream(const bNodeSocket &closure_socket)
{
  VectorSet<const bNodeSocket *> visited;
  Vector<const bNodeSocket *> stack;
  const bNode *nearest_to_zone = nullptr;

  auto push_socket = [&](const bNodeSocket *socket) {
    if (socket && socket->type == SOCK_CLOSURE && visited.add(socket)) {
      stack.append(socket);
    }
  };

  push_socket(&closure_socket);
  while (!stack.is_empty()) {
    const bNodeSocket &socket = *stack.pop_last();
    const bNode &node = socket.owner_node();

    if (socket.is_input()) {
      if (is_set_default_closure_input(socket, 1)) {
        return &node;
      }
      if (is_set_default_closure_input(socket, 0)) {
        /* Continue toward the Closure Zone. The last matching node is closest to the zone. */
        nearest_to_zone = &node;
      }
      for (const bNodeLink *link : socket.directly_linked_links()) {
        if (link->is_used()) {
          push_socket(link->fromsock);
        }
      }
      continue;
    }

    if (node.is_reroute() || is_set_default_closure_main_output(socket)) {
      push_socket(&node.input_socket(0));
      continue;
    }
    if (const nodes::SocketDeclaration *decl = socket.runtime->declaration) {
      if (const auto *closure_decl = dynamic_cast<const nodes::decl::Closure *>(decl)) {
        if (closure_decl->pass_through_input_index) {
          push_socket(&node.input_socket(*closure_decl->pass_through_input_index));
          continue;
        }
      }
    }
    if (node.is_group()) {
      if (bNodeTree *group = reinterpret_cast<bNodeTree *>(node.id)) {
        if (group->runtime != nullptr) {
          group->ensure_topology_cache();
          if (const bNode *group_output = group->group_output_node()) {
            push_socket(group_output->input_by_identifier(socket.identifier_ustr()));
          }
        }
      }
      continue;
    }
    if (node.is_group_input()) {
      continue;
    }
    for (const bNodeSocket *input_socket : node.input_sockets()) {
      if (input_socket->type == SOCK_CLOSURE) {
        push_socket(input_socket);
      }
    }
  }
  return nearest_to_zone;
}

static const bNode *find_set_default_closure_downstream(const bNodeSocket &closure_socket)
{
  VectorSet<const bNodeSocket *> visited;
  Vector<const bNodeSocket *> queue;
  int queue_index = 0;

  auto enqueue = [&](const bNodeSocket *socket) {
    if (socket && socket->type == SOCK_CLOSURE && visited.add(socket)) {
      queue.append(socket);
    }
  };

  enqueue(&closure_socket);
  while (queue_index < queue.size()) {
    const bNodeSocket &socket = *queue[queue_index++];
    const bNode &node = socket.owner_node();
    if (socket.is_output()) {
      for (const bNodeLink *link : socket.directly_linked_links()) {
        if (link->is_used()) {
          enqueue(link->tosock);
        }
      }
      continue;
    }

    if (is_set_default_closure_input(socket, 0)) {
      return &node;
    }
    if (node.is_reroute()) {
      enqueue(&node.output_socket(0));
      continue;
    }
    for (const bNodeSocket *output_socket : node.output_sockets()) {
      if (const nodes::SocketDeclaration *decl = output_socket->runtime->declaration) {
        if (const auto *closure_decl = dynamic_cast<const nodes::decl::Closure *>(decl)) {
          if (closure_decl->pass_through_input_index == socket.index()) {
            enqueue(output_socket);
          }
        }
      }
    }
  }
  return nullptr;
}

const bNode *find_nearest_set_default_closure_node(const bNodeSocket &closure_socket)
{
  if (closure_socket.type != SOCK_CLOSURE) {
    return nullptr;
  }
  /* New / copied nodes can sit in the tree before topology cache assigns owner_tree.
   * owner_tree() would then call ensure_topology_cache on a null pointer. */
  if (closure_socket.runtime == nullptr || closure_socket.runtime->owner_node == nullptr) {
    return nullptr;
  }
  bNodeTree *tree = node_owner_tree_or_null(*closure_socket.runtime->owner_node);
  if (tree == nullptr || tree->runtime == nullptr) {
    return nullptr;
  }
  tree->ensure_topology_cache();
  if (owner_tree_or_null(closure_socket) == nullptr) {
    return nullptr;
  }
  if (const bNode *node = find_set_default_closure_upstream(closure_socket)) {
    return node;
  }
  return find_set_default_closure_downstream(closure_socket);
}

static const bNode *find_default_closure_output_node(const bNode &set_default_node)
{
  const bNodeSocket &default_socket = set_default_node.input_socket(1);
  const bNodeLink *default_link = nullptr;
  for (const bNodeLink *link : default_socket.directly_linked_links()) {
    if (!link->is_used()) {
      continue;
    }
    if (default_link) {
      return nullptr;
    }
    default_link = link;
  }
  if (!default_link || !default_link->fromnode->is_type("NodeClosureOutput"_ustr) ||
      default_link->fromsock != &default_link->fromnode->output_socket(0))
  {
    return nullptr;
  }
  return default_link->fromnode;
}

static Vector<const bNode *> gather_default_closure_template_nodes(const bNode &set_default_node,
                                                                   const bNode &src_input,
                                                                   const bNode &src_output)
{
  const bNodeTree *tree = owner_tree_or_null(set_default_node);
  if (tree == nullptr || tree->runtime == nullptr) {
    return {};
  }
  const bke::bNodeTreeZones *zones = tree->zones();
  const bke::bNodeTreeZone *src_zone = zones ? zones->get_zone_by_node(src_output.identifier) :
                                               nullptr;
  if (!src_zone) {
    return {};
  }

  Set<const bNode *> template_node_set;
  for (const bNode &node : tree->nodes) {
    if (&node == &set_default_node || node.is_group_input() || node.is_group_output()) {
      continue;
    }
    if (&node == &src_input || &node == &src_output || src_zone->contains_node_recursively(node)) {
      for (const bNode *node_or_parent = &node; node_or_parent;
           node_or_parent = node_or_parent->parent)
      {
        template_node_set.add(node_or_parent);
      }
    }
  }

  Vector<const bNode *> template_nodes;
  for (const bNode &node : tree->nodes) {
    if (template_node_set.contains(&node)) {
      template_nodes.append(&node);
    }
  }
  return template_nodes;
}

static void remove_nodes(Main &bmain, bNodeTree &tree, const Span<bNode *> nodes)
{
  for (bNode *node : nodes) {
    if (tree.all_nodes().contains(node)) {
      bke::node_remove_node(&bmain, tree, *node, true);
    }
  }
}

std::optional<DefaultClosureZone> add_default_closure_zone_from_socket(
    Main &bmain, const bNodeSocket &closure_socket, bNodeTree &dst_tree, const float2 location)
{
  if (dst_tree.runtime == nullptr) {
    return std::nullopt;
  }
  dst_tree.ensure_topology_cache();
  if (const bNode *socket_node = closure_socket.runtime ? closure_socket.runtime->owner_node :
                                                          nullptr)
  {
    ensure_node_owner_tree(dst_tree, *socket_node);
  }

  const bNode *set_default_node = find_nearest_set_default_closure_node(closure_socket);
  if (!set_default_node) {
    return std::nullopt;
  }
  const bNode *src_output = find_default_closure_output_node(*set_default_node);
  if (!src_output) {
    return std::nullopt;
  }
  bNodeTree *src_tree_ptr = owner_tree_or_null(*src_output);
  if (src_tree_ptr == nullptr) {
    ensure_node_owner_tree(dst_tree, *src_output);
    src_tree_ptr = owner_tree_or_null(*src_output);
  }
  if (src_tree_ptr == nullptr || src_tree_ptr->runtime == nullptr) {
    return std::nullopt;
  }
  const bNodeTree &src_tree = *src_tree_ptr;
  src_tree.ensure_topology_cache();
  const bke::bNodeZoneType *zone_type = bke::zone_type_by_node_type(src_output->type_legacy);
  const bNode *src_input = zone_type ? zone_type->get_corresponding_input(src_tree, *src_output) :
                                       nullptr;
  if (!src_input) {
    return std::nullopt;
  }

  const Vector<const bNode *> template_nodes = gather_default_closure_template_nodes(
      *set_default_node, *src_input, *src_output);
  if (template_nodes.is_empty()) {
    return std::nullopt;
  }

  NodeSetCopy copied_nodes = NodeSetCopy::from_nodes(bmain, src_tree, template_nodes, dst_tree);
  bNode *dst_input = copied_nodes.node_map().lookup(src_input);
  bNode *dst_output = copied_nodes.node_map().lookup(src_output);

  DefaultClosureZone result;
  result.input_node = dst_input;
  result.output_node = dst_output;
  result.nodes.reserve(copied_nodes.node_map().size());
  for (bNode *node : copied_nodes.node_map().values()) {
    node->location[0] += location.x;
    node->location[1] += location.y;
    BKE_ntree_update_tag_node_new(&dst_tree, node);
    result.nodes.append(node);
  }

  auto &dst_input_storage = *static_cast<NodeClosureInput *>(dst_input->storage);
  dst_input_storage.output_node_id = dst_output->identifier;
  BKE_ntree_update_after_single_tree_change(bmain, dst_tree);
  return result;
}

static bool closure_zone_is_empty(const bNodeTree &tree,
                                  const bNode &closure_input_node,
                                  const bNode &closure_output_node)
{
  const bke::bNodeTreeZones *zones = tree.zones();
  const bke::bNodeTreeZone *zone = zones ?
                                       zones->get_zone_by_node(closure_output_node.identifier) :
                                       nullptr;
  if (!zone) {
    return false;
  }
  for (const bNode &node : tree.nodes) {
    if (ELEM(&node, &closure_input_node, &closure_output_node)) {
      continue;
    }
    if (zone->contains_node_recursively(node)) {
      return false;
    }
  }
  return true;
}

bool apply_default_closure_zone_to_existing(Main &bmain,
                                            const bNodeSocket &closure_socket,
                                            bNodeTree &dst_tree,
                                            bNode &closure_input_node,
                                            bNode &closure_output_node)
{
  if (dst_tree.runtime == nullptr) {
    return false;
  }
  dst_tree.ensure_topology_cache();
  ensure_node_owner_tree(dst_tree, closure_input_node);
  ensure_node_owner_tree(dst_tree, closure_output_node);
  if (const bNode *socket_node = closure_socket.runtime ? closure_socket.runtime->owner_node :
                                                          nullptr)
  {
    ensure_node_owner_tree(dst_tree, *socket_node);
  }
  if (owner_tree_or_null(closure_socket) == nullptr) {
    return false;
  }
  if (!closure_zone_is_empty(dst_tree, closure_input_node, closure_output_node)) {
    return false;
  }

  const bNode *set_default_node = find_nearest_set_default_closure_node(closure_socket);
  if (!set_default_node) {
    return false;
  }
  const bNode *src_output = find_default_closure_output_node(*set_default_node);
  /* Copying a zone into itself (Alt+C on the Default Closure socket, or sync of a newly
   * linked empty zone that is already the template) frees nodes the cache still points at. */
  if (src_output == nullptr || src_output == &closure_output_node) {
    return false;
  }

  const std::optional<DefaultClosureZone> zone = add_default_closure_zone_from_socket(
      bmain, closure_socket, dst_tree, float2(0.0f));
  if (!zone || !zone->input_node || !zone->output_node) {
    if (zone) {
      remove_nodes(bmain, dst_tree, zone->nodes);
    }
    return false;
  }

  bNode *template_input = zone->input_node;
  bNode *template_output = zone->output_node;
  const float2 input_delta = float2(closure_input_node.location) -
                             float2(template_input->location);
  for (bNode *node : zone->nodes) {
    if (!ELEM(node, template_input, template_output)) {
      node->location[0] += input_delta.x;
      node->location[1] += input_delta.y;
    }
  }

  dst_tree.ensure_topology_cache();
  struct LinkToAdd {
    bNode *from_node;
    bNodeSocket *from_socket;
    bNode *to_node;
    bNodeSocket *to_socket;
  };
  Vector<LinkToAdd> links_to_add;
  for (const bNodeLink &link : dst_tree.links) {
    if (link.fromnode == template_input) {
      if (bNodeSocket *dst_socket = bke::node_find_enabled_socket(
              closure_input_node, SOCK_OUT, link.fromsock->name))
      {
        links_to_add.append({&closure_input_node, dst_socket, link.tonode, link.tosock});
      }
    }
    if (link.tonode == template_output) {
      if (bNodeSocket *dst_socket = bke::node_find_enabled_socket(
              closure_output_node, SOCK_IN, link.tosock->name))
      {
        links_to_add.append({link.fromnode, link.fromsock, &closure_output_node, dst_socket});
      }
    }
  }

  Vector<bNode *> remove_list{template_input, template_output};
  remove_nodes(bmain, dst_tree, remove_list);
  for (const LinkToAdd &link : links_to_add) {
    if (!dst_tree.all_nodes().contains(link.from_node) ||
        !dst_tree.all_nodes().contains(link.to_node))
    {
      continue;
    }
    bke::node_add_link(
        dst_tree, *link.from_node, *link.from_socket, *link.to_node, *link.to_socket);
  }

  auto &dst_input_storage = *static_cast<NodeClosureInput *>(closure_input_node.storage);
  dst_input_storage.output_node_id = closure_output_node.identifier;
  BKE_ntree_update_after_single_tree_change(bmain, dst_tree);
  return true;
}

}  // namespace blender::ed::space_node
