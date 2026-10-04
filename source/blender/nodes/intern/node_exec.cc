/* SPDX-FileCopyrightText: 2007 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 */

#include "DNA_node_types.h"

#include "BLI_listbase.hh"
#include "BLI_utildefines.hh"

#include "BKE_global.hh"
#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_node_tree_zones.hh"

#include "MEM_guardedalloc.h"

#include "node_exec.hh"
#include "node_util.hh"

namespace blender {

static int node_exec_socket_use_stack(bNodeSocket *sock)
{
  /* NOTE: INT and BOOL supported as FLOAT. Only for EEVEE. */
  return ELEM(sock->type,
              SOCK_INT,
              SOCK_INT_VECTOR,
              SOCK_BOOLEAN,
              SOCK_FLOAT,
              SOCK_VECTOR,
              SOCK_RGBA,
              SOCK_SHADER,
              SOCK_ROTATION,
              SOCK_COLOR_RAMP,
              SOCK_CURVE);
}

bNodeStack *node_get_socket_stack(bNodeStack *stack, bNodeSocket *sock)
{
  if (stack && sock && sock->stack_index >= 0) {
    return stack + sock->stack_index;
  }
  return nullptr;
}

void node_get_stack(bNode *node, bNodeStack *stack, bNodeStack **in, bNodeStack **out)
{
  /* build pointer stack */
  if (in) {
    for (bNodeSocket &sock : node->inputs) {
      *(in++) = node_get_socket_stack(stack, &sock);
    }
  }

  if (out) {
    for (bNodeSocket &sock : node->outputs) {
      *(out++) = node_get_socket_stack(stack, &sock);
    }
  }
}

static void node_init_input_index(bNodeSocket *sock, int *index)
{
  /* Only consider existing link when the `from` socket is valid! */
  if (sock->link && !(sock->link->flag & NODE_LINK_MUTED) && sock->link->fromsock &&
      sock->link->fromsock->stack_index >= 0)
  {
    sock->stack_index = sock->link->fromsock->stack_index;
  }
  else {
    if (node_exec_socket_use_stack(sock)) {
      sock->stack_index = (*index)++;
    }
    else {
      sock->stack_index = -1;
    }
  }
}

static void node_init_output_index_muted(bNodeSocket *sock, int *index)
{
  /* Copy the stack index from the internally connected input to skip the node. */
  if (const bNodeSocket *internal_input = sock->runtime->internal_link_input) {
    sock->stack_index = internal_input->stack_index;
    return;
  }
  /* If not internally connected, assign a new stack index anyway to avoid bad stack access. */
  if (node_exec_socket_use_stack(sock)) {
    sock->stack_index = (*index)++;
  }
  else {
    sock->stack_index = -1;
  }
}

static void node_init_output_index(bNodeSocket *sock, int *index)
{
  if (node_exec_socket_use_stack(sock)) {
    sock->stack_index = (*index)++;
  }
  else {
    sock->stack_index = -1;
  }
}

/* basic preparation of socket stacks */
static bNodeStack *setup_stack(bNodeStack *stack, bNodeTree *ntree, bNode *node, bNodeSocket *sock)
{
  bNodeStack *ns = node_get_socket_stack(stack, sock);
  if (!ns) {
    return nullptr;
  }

  /* don't mess with remote socket stacks, these are initialized by other nodes! */
  if (sock->link && !(sock->link->flag & NODE_LINK_MUTED)) {
    return ns;
  }
  /* Outputs of muted nodes forward the internally linked input's stack value. */
  if (node->is_muted() && sock->runtime->internal_link_input != nullptr) {
    return ns;
  }
  if (sock->is_output() && node->is_reroute()) {
    return ns;
  }

  ns->sockettype = sock->type;

  switch (sock->type) {
    case SOCK_INT:
      ns->vec[0] = node_socket_get_int(ntree, node, sock);
      break;
    case SOCK_INT_VECTOR: {
      const bNodeSocketValueIntVector &value =
          *sock->default_value_typed<bNodeSocketValueIntVector>();
      ns->vec[0] = float(value.value[0]);
      ns->vec[1] = float(value.value[1]);
      ns->vec[2] = float(value.value[2]);
      ns->vec[3] = 0.0f;
      break;
    }
    case SOCK_BOOLEAN:
      ns->vec[0] = node_socket_get_bool(ntree, node, sock);
      break;
    case SOCK_FLOAT:
      ns->vec[0] = node_socket_get_float(ntree, node, sock);
      break;
    case SOCK_VECTOR:
      node_socket_get_vector(ntree, node, sock, ns->vec);
      break;
    case SOCK_RGBA:
      node_socket_get_color(ntree, node, sock, ns->vec);
      break;
    case SOCK_ROTATION:
      node_socket_get_rotation(ntree, node, sock, ns->vec);
      break;
    case SOCK_COLOR_RAMP:
      /* Data-node outputs must forward the edited input, not their own unused default. */
      if (const ColorBand *band = bke::node_socket_color_band_resolved(*sock)) {
        ns->data = const_cast<ColorBand *>(band);
      }
      else if (const bNodeSocketValueColorRamp *value =
                   sock->default_value_typed<bNodeSocketValueColorRamp>())
      {
        ns->data = value->band;
      }
      break;
    case SOCK_CURVE:
      if (CurveMapping *curve = bke::node_socket_curve_resolved(*sock)) {
        ns->data = curve;
      }
      else if (const bNodeSocketValueCurve *value =
                   sock->default_value_typed<bNodeSocketValueCurve>())
      {
        ns->data = value->curve;
      }
      break;
    default:
      break;
  }

  return ns;
}

static bool node_belongs_to_tree(const bNodeTree &ntree, const bNode *node)
{
  if (node == nullptr || ntree.runtime == nullptr) {
    return false;
  }
  for (const bNode *tree_node : ntree.runtime->nodes_by_id) {
    if (tree_node == node) {
      return true;
    }
  }
  return false;
}

static Vector<bNode *> get_node_code_gen_order(bNodeTree &ntree)
{
  ntree.ensure_topology_cache();
  /* Query zones before copying the toposort; zone discovery may rebuild topology. */
  const bke::bNodeTreeZones *zones = ntree.zones();
  Vector<bNode *> nodes;
  for (bNode *node : ntree.toposort_left_to_right()) {
    if (node_belongs_to_tree(ntree, node) && node->runtime != nullptr) {
      nodes.append(node);
    }
  }
  if (!zones) {
    return nodes;
  }
  /* Insertion sort to make sure that all nodes in a zone are packed together right before the zone
   * output. */
  for (int old_i = nodes.size() - 1; old_i >= 0; old_i--) {
    bNode *node = nodes[old_i];
    if (node == nullptr) {
      continue;
    }
    const bke::bNodeTreeZone *zone = zones->get_zone_by_node(node->identifier);
    if (!zone) {
      /* None outside of any zone can stay where they are. */
      continue;
    }
    if (zone->output_node_id == node->identifier) {
      /* The output of a zone should not be moved. */
      continue;
    }
    for (int new_i = old_i + 1; new_i < nodes.size(); new_i++) {
      bNode *next_node = nodes[new_i];
      if (next_node == nullptr) {
        continue;
      }
      const bke::bNodeTreeZone *zone_to_check = zones->get_zone_by_node(next_node->identifier);
      if (zone_to_check &&
          (zone == zone_to_check || zone->contains_zone_recursively(*zone_to_check)))
      {
        /* Don't move the node further than the next node in the zone. */
        break;
      }
      std::swap(nodes[new_i - 1], nodes[new_i]);
    }
  }
  return nodes;
}

bNodeTreeExec *ntree_exec_begin(bNodeExecContext *context,
                                bNodeTree *ntree,
                                bNodeInstanceKey parent_key)
{
  bNodeTreeExec *exec;
  bNode *node;
  bNodeExec *nodeexec;
  bNodeInstanceKey nodekey;
  bNodeStack *ns;
  int index;
  /* XXX: texture-nodes have threading issues with muting, have to disable it there. */

  /* ensure all sock->link pointers and node levels are correct */
  /* Using global main here is likely totally wrong, not sure what to do about that one though...
   * We cannot even check ntree is in global main,
   * since most of the time it won't be (thanks to ntree design)!!! */
  BKE_ntree_update_after_single_tree_change(*G.main, *ntree);

  ntree->ensure_topology_cache();
  Vector<bNode *> nodelist = get_node_code_gen_order(*ntree);

  /* XXX could let callbacks do this for specialized data */
  exec = MEM_new_zeroed<bNodeTreeExec>("node tree execution data");
  /* Back-pointer to node tree. */
  exec->nodetree = ntree;

  /* set stack indices */
  index = 0;
  for (const int n : nodelist.index_range()) {
    node = nodelist[n];
    if (node == nullptr || node->runtime == nullptr) {
      continue;
    }

    /* init node socket stack indexes */
    for (bNodeSocket &sock : node->inputs) {
      node_init_input_index(&sock, &index);
    }

    if (node->is_muted() || node->is_reroute() ||
        (bke::node_is_store_named_portal(*node) && node->type_legacy != GEO_NODE_STORE_NAMED_PORTAL))
    {
      for (bNodeSocket &sock : node->outputs) {
        node_init_output_index_muted(&sock, &index);
      }
    }
    else if (bke::node_is_named_portal(*node) && node->type_legacy != GEO_NODE_NAMED_PORTAL) {
      /* Named Portal outputs share the Store stack slot; assigned in a second pass. */
    }
    else {
      for (bNodeSocket &sock : node->outputs) {
        node_init_output_index(&sock, &index);
      }
    }
  }

  for (const int n : nodelist.index_range()) {
    node = nodelist[n];
    if (node->is_muted() || node->is_reroute()) {
      continue;
    }
    if (!(bke::node_is_named_portal(*node) && node->type_legacy != GEO_NODE_NAMED_PORTAL)) {
      continue;
    }
    const bNode *store = bke::node_find_unique_store_named_portal(
        *ntree, bke::node_named_portal_name(*node));
    for (bNodeSocket &sock : node->outputs) {
      if (store && store->storage && node->storage &&
          static_cast<const NodeGeometryPortal *>(store->storage)->data_type ==
              static_cast<const NodeGeometryPortal *>(node->storage)->data_type &&
          !store->output_sockets().is_empty())
      {
        sock.stack_index = store->output_socket(0).stack_index;
      }
      else {
        node_init_output_index(&sock, &index);
      }
    }
  }

  /* allocated exec data pointers for nodes */
  exec->totnodes = nodelist.size();
  exec->nodeexec = MEM_new_array_zeroed<bNodeExec>(exec->totnodes, "node execution data");
  /* allocate data pointer for node stack */
  exec->stacksize = index;
  exec->stack = MEM_new_array<bNodeStack>(exec->stacksize, "bNodeStack");

  /* all non-const results are considered inputs */
  int n;
  for (n = 0; n < exec->stacksize; n++) {
    exec->stack[n].hasinput = 1;
  }

  /* prepare all nodes for execution */
  for (n = 0, nodeexec = exec->nodeexec; n < nodelist.size(); n++, nodeexec++) {
    node = nodeexec->node = nodelist[n];
    nodeexec->free_exec_fn = node->typeinfo->free_exec_fn;

    /* tag inputs */
    for (bNodeSocket &sock : node->inputs) {
      /* disable the node if an input link is invalid */
      if (sock.link && !(sock.link->flag & NODE_LINK_VALID)) {
        node->runtime->need_exec = 0;
      }

      ns = setup_stack(exec->stack, ntree, node, &sock);
      if (ns) {
        ns->hasoutput = 1;
      }
    }

    /* tag all outputs */
    const bool skip_get_portal_setup = bke::node_is_named_portal(*node) &&
                                       node->type_legacy != GEO_NODE_NAMED_PORTAL;
    for (bNodeSocket &sock : node->outputs) {
      if (skip_get_portal_setup) {
        continue;
      }
      /* ns = */ setup_stack(exec->stack, ntree, node, &sock);
    }

    nodekey = bke::node_instance_key(parent_key, ntree, node);
    if (node->typeinfo->init_exec_fn) {
      nodeexec->data.data = node->typeinfo->init_exec_fn(context, node, nodekey);
    }
  }

  return exec;
}

void ntree_exec_end(bNodeTreeExec *exec)
{
  bNodeExec *nodeexec;
  int n;

  if (exec->stack) {
    MEM_delete(exec->stack);
  }

  for (n = 0, nodeexec = exec->nodeexec; n < exec->totnodes; n++, nodeexec++) {
    if (nodeexec->free_exec_fn) {
      nodeexec->free_exec_fn(nodeexec->data.data);
    }
  }

  if (exec->nodeexec) {
    MEM_delete(exec->nodeexec);
  }

  MEM_delete(exec);
}

}  // namespace blender
