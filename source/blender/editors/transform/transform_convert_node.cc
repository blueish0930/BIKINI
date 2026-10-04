/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edtransform
 */

#include "DNA_space_types.h"
#include "DNA_userdef_types.h"

#include "MEM_guardedalloc.h"

#include <algorithm>
#include <cfloat>
#include <cstdint>
#include <functional>

#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_rect.hh"
#include "BLI_time.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

#include "BKE_context.hh"
#include "BKE_main_invariants.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "ED_node.hh"

#include "UI_view2d.hh"

#include "transform.hh"
#include "transform_convert.hh"
#include "transform_snap.hh"

#include "WM_api.hh"
#include "WM_types.hh"

namespace blender::ed::transform {

/** Axis lock for U node align (Houdini network-editor style). */
enum class NodeAlignAxis {
  None = 0,
  /** Horizontal mouse motion: same top-Y, distribute along X. */
  Horizontal = 1,
  /** Vertical mouse motion: same left-X, distribute along Y. */
  Vertical = 2,
};

struct TransCustomDataNode {
  ui::View2DEdgePanData edgepan_data{};

  /* Compare if the view has changed so we can update with `transformViewUpdate`. */
  rctf viewrect_prev{};

  bool is_new_node = false;

  Map<bNode *, bNode *> old_parent_by_detached_node;

  /* Shake-to-detach (Houdini-style): track mouse direction flips while dragging. */
  float2 shake_prev_mval = float2(0.0f);
  float2 shake_prev_dir = float2(0.0f);
  int shake_dir_changes = 0;
  double shake_window_start = 0.0;
  bool shake_detached = false;
  bool shake_has_prev = false;

  /**
   * Node align (hold U + LMB drag): direction chooses flow axis; graph-aware branch layout.
   * Latched via #CTX_NODE_ALIGN when translate starts with node_align=True.
   */
  bool align_mod_was_on = false;
  float2 align_start_mval = float2(0.0f);
  NodeAlignAxis align_axis = NodeAlignAxis::None;
};

/** Auto-insert onto noodles unless attach is off or the move is a G-grab / hold-G. */
static bool node_transform_auto_insert_enabled(const TransInfo *t)
{
  if ((t->modifiers & MOD_NODE_ATTACH) == 0) {
    return false;
  }
  /* G-started translate: keep moving only, even after G is released. */
  if (t->launch_event == EVT_GKEY) {
    return false;
  }
  return true;
}

/* -------------------------------------------------------------------- */
/** \name Node Transform Creation
 * \{ */

static void create_transform_data_for_node(TransData &td,
                                           TransData2D &td2d,
                                           bNode &node,
                                           const float dpi_fac)
{
  /* Account for parents (nested nodes). */
  float2 loc = float2(node.location) * dpi_fac;

  /* Use top-left corner as the transform origin for nodes. */
  /* Weirdo - but the node system is a mix of free 2d elements and DPI sensitive UI. */
  td2d.loc[0] = loc.x;
  td2d.loc[1] = loc.y;
  td2d.loc[2] = 0.0f;
  td2d.loc2d = td2d.loc; /* Current location. */

  td.loc = td2d.loc;
  copy_v3_v3(td.iloc, td.loc);
  /* Use node center instead of origin (top-left corner). */
  td.center[0] = td2d.loc[0];
  td.center[1] = td2d.loc[1];
  td.center[2] = 0.0f;

  memset(td.axismtx, 0, sizeof(td.axismtx));
  td.axismtx[2][2] = 1.0f;

  td.val = nullptr;

  td.flag = TD_SELECTED;
  td.dist = 0.0f;

  unit_m3(td.mtx);
  unit_m3(td.smtx);

  td.extra = &node;
}

static bool is_node_parent_select(const bNode *node)
{
  while ((node = node->parent)) {
    if (node->is_selected()) {
      return true;
    }
  }
  return false;
}

/**
 * Some nodes are transformed together with other nodes:
 * - Parent frames with shrinking turned on are automatically resized based on their children.
 * - Child nodes of frames that are manually resizable are transformed together with their parent
 *   frame.
 */
static bool transform_tied_to_other_node(bNode *node, VectorSet<bNode *> transformed_nodes)
{
  /* Check for frame nodes that adjust their size based on the contained child nodes. */
  if (node->is_frame()) {
    const NodeFrame *data = static_cast<const NodeFrame *>(node->storage);
    const bool shrinking = data->flag & NODE_FRAME_SHRINK;
    const bool is_parent = !node->direct_children_in_frame().is_empty();

    if (is_parent && shrinking) {
      return true;
    }
  }

  /* Now check for child nodes of manually resized frames. */
  while ((node = node->parent)) {
    const NodeFrame *parent_data = static_cast<const NodeFrame *>(node->storage);
    const bool parent_shrinking = parent_data->flag & NODE_FRAME_SHRINK;
    const bool parent_transformed = transformed_nodes.contains(node);

    if (parent_transformed && !parent_shrinking) {
      return true;
    }
  }

  return false;
}

static VectorSet<bNode *> get_transformed_nodes(bNodeTree &node_tree)
{
  VectorSet<bNode *> nodes = node_tree.all_nodes();

  /* Keep only nodes that are selected or inside a frame that is selected. */
  nodes.remove_if([&](bNode *node) {
    const bool node_selected = node->is_selected();
    const bool parent_selected = is_node_parent_select(node);
    return (!node_selected && !parent_selected);
  });

  /* Remove nodes that are transformed together with their parent or child nodes. */
  nodes.remove_if([&](bNode *node) { return transform_tied_to_other_node(node, nodes); });

  return nodes;
}

static void createTransNodeData(bContext * /*C*/, TransInfo *t)
{
  SpaceNode *snode = t->area->spacedata.first_as<SpaceNode>();
  bNodeTree *node_tree = snode->edittree;
  if (!node_tree) {
    return;
  }

  /* Custom data to enable edge panning during the node transform. */
  TransCustomDataNode *customdata = MEM_new<TransCustomDataNode>(__func__);
  view2d_edge_pan_init(t->context,
                       &customdata->edgepan_data,
                       NODE_EDGE_PAN_INSIDE_PAD,
                       NODE_EDGE_PAN_OUTSIDE_PAD,
                       NODE_EDGE_PAN_SPEED_RAMP,
                       NODE_EDGE_PAN_MAX_SPEED,
                       NODE_EDGE_PAN_DELAY,
                       NODE_EDGE_PAN_ZOOM_INFLUENCE);
  customdata->viewrect_prev = customdata->edgepan_data.initial_rect;
  customdata->is_new_node = t->remove_on_cancel;

  if (t->region) {
    space_node::node_insert_on_link_flags_set(
        *snode, *t->region, node_transform_auto_insert_enabled(t), customdata->is_new_node);
    space_node::node_insert_on_frame_flag_set(*snode, *t->region, int2(t->mval));
  }

  t->custom.type.data = customdata;
  t->custom.type.free_cb = [](TransInfo *, TransDataContainer *, TransCustomData *custom_data) {
    TransCustomDataNode *data = static_cast<TransCustomDataNode *>(custom_data->data);
    MEM_delete(data);
    custom_data->data = nullptr;
  };

  TransDataContainer *tc = TRANS_DATA_CONTAINER_FIRST_SINGLE(t);

  /* Nodes don't support proportional editing and probably never will. */
  t->flag = t->flag & ~T_PROP_EDIT_ALL;

  VectorSet<bNode *> nodes = get_transformed_nodes(*node_tree);
  if (nodes.is_empty()) {
    return;
  }

  tc->data_len = nodes.size();
  tc->data = MEM_new_array_zeroed<TransData>(tc->data_len, __func__);
  tc->data_2d = MEM_new_array_zeroed<TransData2D>(tc->data_len, __func__);

  for (const int i : nodes.index_range()) {
    create_transform_data_for_node(tc->data[i], tc->data_2d[i], *nodes[i], UI_SCALE_FAC);
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Flush Transform Nodes
 * \{ */

static void node_snap_grid_apply(TransInfo *t)
{
  if (!(transform_snap_is_active(t) &&
        (t->tsnap.mode & (SCE_SNAP_TO_INCREMENT | SCE_SNAP_TO_GRID))))
  {
    return;
  }

  float2 grid_size = t->snap_spatial;
  if (t->modifiers & MOD_PRECISION) {
    grid_size *= t->snap_spatial_precision;
  }

  /* Early exit on unusable grid size. */
  if (math::is_zero(grid_size)) {
    return;
  }

  FOREACH_TRANS_DATA_CONTAINER (t, tc) {
    for (const int i : IndexRange(tc->data_len)) {
      TransData &td = tc->data[i];
      if (td.flag & TD_SKIP) {
        continue;
      }

      if ((t->flag & T_PROP_EDIT) && (td.factor == 0.0f)) {
        continue;
      }

      /* Nodes are snapped to the grid by first aligning their initial position to the grid and
       * then offsetting them in grid increments.
       *
       * This ensures that multiple unsnapped nodes snap to the grid in sync while moving.
       */

      const float2 inital_location = td.iloc;
      const float2 target_location = td.loc;
      const float2 offset = target_location - inital_location;

      const float2 snapped_inital_location = math::round(inital_location / grid_size) * grid_size;
      const float2 snapped_offset = math::round(offset / grid_size) * grid_size;
      const float2 snapped_target_location = snapped_inital_location + snapped_offset;

      copy_v2_v2(td.loc, snapped_target_location);
    }
  }
}

/**
 * Vertical offset of a socket from the node's top edge, in DPI/view space (same as #TransData.loc).
 * Prefers drawn #bNodeSocketRuntime::location; falls back to index-based estimate.
 */
static float socket_y_offset_from_top_dpi(const bNode &node,
                                          const bNodeSocket *sock,
                                          const float height_dpi,
                                          const float dpi_fac)
{
  if (sock && sock->runtime) {
    const float node_top_view = node.location[1] * dpi_fac;
    const float sock_y = sock->runtime->location.y;
    /* Valid after the tree has been drawn at least once. */
    if (math::abs(sock_y) > 1.0e-3f || math::abs(sock->runtime->location.x) > 1.0e-3f) {
      const float off = node_top_view - sock_y;
      if (off >= -2.0f * dpi_fac && off <= height_dpi + 50.0f * dpi_fac) {
        return math::clamp(off, 0.0f, height_dpi);
      }
    }
  }

  /* Fallback: header + evenly spaced sockets (matches typical node UI rhythm). */
  int index = 0;
  int count = 1;
  if (sock) {
    const Span<const bNodeSocket *> inputs = node.input_sockets();
    const Span<const bNodeSocket *> outputs = node.output_sockets();
    bool found = false;
    for (const int i : inputs.index_range()) {
      if (inputs[i] == sock) {
        index = i;
        count = math::max(1, int(inputs.size()));
        found = true;
        break;
      }
    }
    if (!found) {
      for (const int i : outputs.index_range()) {
        if (outputs[i] == sock) {
          index = i;
          count = math::max(1, int(outputs.size()));
          break;
        }
      }
    }
  }
  const float header = math::min(height_dpi * 0.28f, 1.35f * float(U.widget_unit));
  const float usable = math::max(height_dpi - header - 0.35f * float(U.widget_unit), float(U.widget_unit));
  if (count <= 1) {
    return header;
  }
  return header + usable * (float(index) / float(count - 1));
}

/**
 * Houdini-style align (hold U + LMB drag anywhere in the editor).
 *
 * - Horizontal drag → left→right layered flow; multi-input parents placed so **socket Y aligns**.
 * - Vertical drag → top→bottom layered flow; branches side-by-side with center alignment.
 *
 * Uses the selection subgraph only. Pack center follows free-transform center.
 * #td.loc is DPI-scaled. Requires ≥2 non-frame nodes.
 */
static void node_selection_align_apply(TransInfo *t)
{
  TransCustomDataNode *customdata = static_cast<TransCustomDataNode *>(t->custom.type.data);
  if (!customdata) {
    return;
  }

  const bool align_on = (t->options & CTX_NODE_ALIGN) != 0;
  if (!align_on || t->state == TRANS_CANCEL) {
    customdata->align_mod_was_on = false;
    customdata->align_axis = NodeAlignAxis::None;
    return;
  }

  if (!customdata->align_mod_was_on) {
    customdata->align_mod_was_on = true;
    customdata->align_start_mval = float2(t->mval);
    customdata->align_axis = NodeAlignAxis::None;
  }

  struct AlignItem {
    TransData *td = nullptr;
    bNode *node = nullptr;
    float width = 0.0f;
    float height = 0.0f;
  };

  Vector<AlignItem> items;
  FOREACH_TRANS_DATA_CONTAINER (t, tc) {
    for (int i = 0; i < tc->data_len; i++) {
      TransData &td = tc->data[i];
      if (td.flag & TD_SKIP) {
        continue;
      }
      bNode *node = static_cast<bNode *>(td.extra);
      if (!node || node->is_frame()) {
        continue;
      }
      /* draw_bounds / node_dimensions_get are already in view (DPI) space — same as td.loc.
       * Do NOT multiply by UI_SCALE_FAC again (that double-scales on high-DPI / UI Scale > 1). */
      const float2 dims = bke::node_dimensions_get(*node);
      /* Fallback if the tree has not been drawn yet (zero bounds). */
      const float w = (dims.x > 1.0f) ? dims.x : (node->width * UI_SCALE_FAC);
      const float h = (dims.y > 1.0f) ? dims.y : (2.0f * float(U.widget_unit));
      items.append(AlignItem{&td, node, w, h});
    }
  }
  if (items.size() < 2) {
    return;
  }

  constexpr float align_deadzone_px = 10.0f;
  if (customdata->align_axis == NodeAlignAxis::None) {
    const float2 delta = float2(t->mval) - customdata->align_start_mval;
    const float adx = math::abs(delta.x);
    const float ady = math::abs(delta.y);
    if (math::max(adx, ady) < align_deadzone_px) {
      for (AlignItem &item : items) {
        copy_v3_v3(item.td->loc, item.td->iloc);
      }
      return;
    }
    customdata->align_axis = (adx >= ady) ? NodeAlignAxis::Horizontal : NodeAlignAxis::Vertical;
  }

  SpaceNode *snode = t->area->spacedata.first_as<SpaceNode>();
  bNodeTree *ntree = (snode) ? snode->edittree : nullptr;
  if (!ntree) {
    return;
  }
  ntree->ensure_topology_cache();

  const int n = int(items.size());
  const float dpi_fac = UI_SCALE_FAC;
  Map<bNode *, int> index_of_node;
  index_of_node.reserve(n);
  for (int i = 0; i < n; i++) {
    index_of_node.add(items[i].node, i);
  }

  struct AlignEdge {
    int from = 0;
    int to = 0;
    float from_out_y_off = 0.0f;
    float to_in_y_off = 0.0f;
  };

  Vector<Vector<int>> children(n);
  Vector<Vector<int>> parents(n);
  Vector<AlignEdge> edges;
  for (const bNodeLink &link : ntree->links) {
    if (!link.is_used() || !link.fromnode || !link.tonode) {
      continue;
    }
    const int *from_i = index_of_node.lookup_ptr(link.fromnode);
    const int *to_i = index_of_node.lookup_ptr(link.tonode);
    if (!from_i || !to_i || *from_i == *to_i) {
      continue;
    }
    AlignEdge e;
    e.from = *from_i;
    e.to = *to_i;
    e.from_out_y_off = socket_y_offset_from_top_dpi(
        *items[e.from].node, link.fromsock, items[e.from].height, dpi_fac);
    e.to_in_y_off = socket_y_offset_from_top_dpi(
        *items[e.to].node, link.tosock, items[e.to].height, dpi_fac);
    edges.append(e);
    children[e.from].append(e.to);
    parents[e.to].append(e.from);
  }
  const int link_count = int(edges.size());

  float2 free_center = float2(0.0f);
  for (const AlignItem &item : items) {
    free_center.x += item.td->loc[0] + item.width * 0.5f;
    free_center.y += item.td->loc[1] - item.height * 0.5f;
  }
  free_center /= float(n);

  /* Gap in view space ≈ 20 node-location units (matches typical node width 120 rhythm).
   * Must use the same space as td.loc (location * UI_SCALE_FAC). */
  const float gap = 20.0f * dpi_fac;
  const bool flow_horizontal = (customdata->align_axis == NodeAlignAxis::Horizontal);

  /* No internal links: single-axis even pack. */
  if (link_count == 0) {
    if (flow_horizontal) {
      std::sort(items.begin(), items.end(), [](const AlignItem &a, const AlignItem &b) {
        return a.td->iloc[0] < b.td->iloc[0];
      });
      float total = 0.0f;
      for (const AlignItem &item : items) {
        total += item.width;
      }
      total += gap * float(n - 1);
      float x = free_center.x - total * 0.5f;
      float y_sum = 0.0f;
      for (const AlignItem &item : items) {
        y_sum += item.td->loc[1];
      }
      const float ref_y = y_sum / float(n);
      for (AlignItem &item : items) {
        item.td->loc[0] = x;
        item.td->loc[1] = ref_y;
        x += item.width + gap;
      }
    }
    else {
      std::sort(items.begin(), items.end(), [](const AlignItem &a, const AlignItem &b) {
        return a.td->iloc[1] > b.td->iloc[1];
      });
      float total = 0.0f;
      for (const AlignItem &item : items) {
        total += item.height;
      }
      total += gap * float(n - 1);
      float y = free_center.y + total * 0.5f;
      float x_sum = 0.0f;
      for (const AlignItem &item : items) {
        x_sum += item.td->loc[0];
      }
      const float ref_x = x_sum / float(n);
      for (AlignItem &item : items) {
        item.td->loc[0] = ref_x;
        item.td->loc[1] = y;
        y -= item.height + gap;
      }
    }
    return;
  }

  /* Connected components (undirected). */
  Vector<int> component(n, -1);
  int component_count = 0;
  Vector<int> stack;
  for (int seed = 0; seed < n; seed++) {
    if (component[seed] != -1) {
      continue;
    }
    const int cid = component_count++;
    stack.clear();
    stack.append(seed);
    component[seed] = cid;
    while (!stack.is_empty()) {
      const int i = stack.pop_last();
      for (const int j : children[i]) {
        if (component[j] == -1) {
          component[j] = cid;
          stack.append(j);
        }
      }
      for (const int j : parents[i]) {
        if (component[j] == -1) {
          component[j] = cid;
          stack.append(j);
        }
      }
    }
  }

  /* Longest path to a sink (nodes with no outgoing links in the selection).
   *
   * Previously layers were longest-path from sources (all roots → layer 0), which shoved every
   * orphan input to the far left. Distance-to-sink places a source in the column immediately
   * left of its children (e.g. a Value wired only into a late Math sits next to that Math).
   *
   * Packing still uses L=0 leftmost … L=max rightmost, so invert: layer = max_dist - dist. */
  Vector<int> dist_to_sink(n, 0);
  Vector<int8_t> visit(n, 0);
  std::function<int(int)> compute_dist_to_sink = [&](int i) -> int {
    if (visit[i] == 2) {
      return dist_to_sink[i];
    }
    if (visit[i] == 1) {
      /* Cycle break. */
      return 0;
    }
    visit[i] = 1;
    int best = 0;
    for (const int c : children[i]) {
      best = math::max(best, compute_dist_to_sink(c) + 1);
    }
    dist_to_sink[i] = best;
    visit[i] = 2;
    return best;
  };
  int max_dist = 0;
  for (int i = 0; i < n; i++) {
    max_dist = math::max(max_dist, compute_dist_to_sink(i));
  }
  Vector<int> layer(n, 0);
  for (int i = 0; i < n; i++) {
    layer[i] = max_dist - dist_to_sink[i];
  }

  /* Per-node edges for fast lookup. */
  Vector<Vector<const AlignEdge *>> edges_from(n);
  Vector<Vector<const AlignEdge *>> edges_to(n);
  for (const AlignEdge &e : edges) {
    edges_from[e.from].append(&e);
    edges_to[e.to].append(&e);
  }

  Vector<float2> rel(n, float2(0.0f));

  for (int cid = 0; cid < component_count; cid++) {
    Vector<int> members;
    int max_layer = 0;
    for (int i = 0; i < n; i++) {
      if (component[i] == cid) {
        members.append(i);
        max_layer = math::max(max_layer, layer[i]);
      }
    }
    if (members.is_empty()) {
      continue;
    }

    Vector<Vector<int>> by_layer(max_layer + 1);
    for (const int i : members) {
      by_layer[layer[i]].append(i);
    }

    /* Cross-axis coordinate: top Y for horizontal flow, left X for vertical flow. */
    Vector<float> cross(n, 0.0f);
    Vector<bool> placed(n, false);

    /* Seed sinks (max layer): pack by original cross position. */
    {
      Vector<int> &sinks = by_layer[max_layer];
      std::sort(sinks.begin(), sinks.end(), [&](const int a, const int b) {
        if (flow_horizontal) {
          return items[a].td->iloc[1] > items[b].td->iloc[1];
        }
        return items[a].td->iloc[0] < items[b].td->iloc[0];
      });
      float cursor = 0.0f;
      for (const int i : sinks) {
        if (flow_horizontal) {
          cross[i] = -cursor; /* top Y */
          cursor += items[i].height + gap;
        }
        else {
          cross[i] = cursor; /* left X */
          cursor += items[i].width + gap;
        }
        placed[i] = true;
      }
      /* Center the sink pack on 0. */
      if (!sinks.is_empty()) {
        float cmin = FLT_MAX;
        float cmax = -FLT_MAX;
        for (const int i : sinks) {
          if (flow_horizontal) {
            cmin = math::min(cmin, cross[i] - items[i].height);
            cmax = math::max(cmax, cross[i]);
          }
          else {
            cmin = math::min(cmin, cross[i]);
            cmax = math::max(cmax, cross[i] + items[i].width);
          }
        }
        const float mid = 0.5f * (cmin + cmax);
        for (const int i : sinks) {
          cross[i] -= mid;
        }
      }
    }

    /*
     * Walk layers right→left (or bottom→top): place each node so connected sockets align.
     * desired_cross(parent) = cross(child) - child_in_off + parent_out_off  (horizontal Y).
     * Multiple children: average. Then resolve overlaps preserving order.
     */
    for (int L = max_layer - 1; L >= 0; L--) {
      Vector<int> &layer_nodes = by_layer[L];
      if (layer_nodes.is_empty()) {
        continue;
      }

      Vector<float> desired(layer_nodes.size(), 0.0f);
      Vector<float> weight(layer_nodes.size(), 0.0f);
      for (int li = 0; li < layer_nodes.size(); li++) {
        const int i = layer_nodes[li];
        /* Prefer furthest-layer placed children for socket alignment.
         *
         * Example: Geometry → Join (long) and Geometry → Set Position → Join.
         * Aligning Geometry to Join (furthest) and Set Position to Join's other
         * input yields (1,0)/(2,-1)/(3,0) style branch layout, instead of
         * averaging every child and collapsing everything onto one row. */
        int max_child_layer = -1;
        for (const AlignEdge *e : edges_from[i]) {
          if (placed[e->to]) {
            max_child_layer = math::max(max_child_layer, layer[e->to]);
          }
        }
        float sum = 0.0f;
        float w = 0.0f;
        auto accumulate = [&](const AlignEdge *e) {
          if (flow_horizontal) {
            /* Align output Y of i with input Y of child. */
            sum += cross[e->to] - e->to_in_y_off + e->from_out_y_off;
          }
          else {
            /* Vertical flow: keep branch centers roughly aligned on X. */
            sum += cross[e->to] + items[e->to].width * 0.5f - items[i].width * 0.5f;
          }
          w += 1.0f;
        };
        if (max_child_layer >= 0) {
          for (const AlignEdge *e : edges_from[i]) {
            if (placed[e->to] && layer[e->to] == max_child_layer) {
              accumulate(e);
            }
          }
        }
        /* Fallback: any placed child (should be rare). */
        if (w <= 0.0f) {
          for (const AlignEdge *e : edges_from[i]) {
            if (placed[e->to]) {
              accumulate(e);
            }
          }
        }
        if (w > 0.0f) {
          desired[li] = sum / w;
          weight[li] = w;
        }
        else {
          /* Isolated in selection links: keep original relative order. */
          desired[li] = flow_horizontal ? (-items[i].td->iloc[1]) : items[i].td->iloc[0];
          weight[li] = 0.0f;
        }
      }

      /* Sort by desired cross (top→bottom or left→right). */
      Vector<int> order;
      order.reserve(layer_nodes.size());
      for (const int li : layer_nodes.index_range()) {
        order.append(li);
      }
      std::sort(order.begin(), order.end(), [&](const int a, const int b) {
        if (desired[a] != desired[b]) {
          /* Horizontal: higher Y first. Vertical: smaller X first. */
          return flow_horizontal ? (desired[a] > desired[b]) : (desired[a] < desired[b]);
        }
        return layer_nodes[a] < layer_nodes[b];
      });

      /* Place near desired positions without overlap. */
      Vector<float> placed_cross(layer_nodes.size(), 0.0f);
      for (int oi = 0; oi < order.size(); oi++) {
        const int li = order[oi];
        const int i = layer_nodes[li];
        float c = desired[li];
        if (oi > 0) {
          const int prev_li = order[oi - 1];
          const int prev_i = layer_nodes[prev_li];
          if (flow_horizontal) {
            /* Must sit fully below previous node (previous has higher top Y). */
            const float max_top = placed_cross[prev_li] - items[prev_i].height - gap;
            c = math::min(c, max_top);
          }
          else {
            const float min_left = placed_cross[prev_li] + items[prev_i].width + gap;
            c = math::max(c, min_left);
          }
        }
        placed_cross[li] = c;
        cross[i] = c;
        placed[i] = true;
      }

      /* Second pass upward/leftward: pull first nodes toward desired if trailing pack drifted. */
      for (int oi = int(order.size()) - 2; oi >= 0; oi--) {
        const int li = order[oi];
        const int i = layer_nodes[li];
        const int next_li = order[oi + 1];
        const int next_i = layer_nodes[next_li];
        if (flow_horizontal) {
          const float min_top = cross[next_i] + items[i].height + gap;
          if (cross[i] < min_top) {
            cross[i] = min_top;
          }
          /* Prefer desired if still valid. */
          const float max_top = (oi > 0) ? (cross[layer_nodes[order[oi - 1]]] -
                                            items[layer_nodes[order[oi - 1]]].height - gap) :
                                           FLT_MAX;
          cross[i] = math::clamp(desired[li],
                                 cross[next_i] + items[i].height + gap,
                                 max_top);
        }
        else {
          const float max_left = cross[next_i] - items[i].width - gap;
          if (cross[i] > max_left) {
            cross[i] = max_left;
          }
          const float min_left = (oi > 0) ? (cross[layer_nodes[order[oi - 1]]] +
                                             items[layer_nodes[order[oi - 1]]].width + gap) :
                                            -FLT_MAX;
          cross[i] = math::clamp(desired[li], min_left, cross[next_i] - items[i].width - gap);
        }
      }
    }

    /* Main-axis (layer) packing. */
    Vector<float> layer_main_size(max_layer + 1, 0.0f);
    for (int L = 0; L <= max_layer; L++) {
      float main_max = 0.0f;
      for (const int i : by_layer[L]) {
        main_max = math::max(main_max, flow_horizontal ? items[i].width : items[i].height);
      }
      layer_main_size[L] = main_max;
    }

    float main_cursor = 0.0f;
    for (int L = 0; L <= max_layer; L++) {
      for (const int i : by_layer[L]) {
        if (flow_horizontal) {
          rel[i].x = main_cursor;
          rel[i].y = cross[i];
        }
        else {
          rel[i].x = cross[i];
          rel[i].y = -main_cursor;
        }
      }
      main_cursor += layer_main_size[L] + gap;
    }

    /* Side-branch separation under long edges (A → C skips layers).
     *
     * When an intermediate node sits on the same Y as the long wire spine:
     * - Bare wire above: place node top *below the link Y* (socket height), not
     *   merely a fixed gap from the parent *header* — otherwise the node top
     *   still covers the wire (socket is ~header below node top).
     * - Node already on that band: stack below it by full height + gap.
     */
    if (flow_horizontal) {
      /* Padding between the link curve and the node header top. */
      const float wire_pad = math::max(gap * 0.35f, 6.0f * dpi_fac);
      for (const AlignEdge &e : edges) {
        if (component[e.from] != cid || component[e.to] != cid) {
          continue;
        }
        if (layer[e.to] < layer[e.from] + 2) {
          continue;
        }
        const float spine_y = rel[e.from].y;
        /* Link endpoints in view Y (node top − socket offset from top). */
        const float from_sock_y = rel[e.from].y - e.from_out_y_off;
        const float to_sock_y = rel[e.to].y - e.to_in_y_off;
        /* Bezier usually bows between sockets; clear below the lower socket so
         * the node header does not sit on the wire. */
        const float wire_y = math::min(from_sock_y, to_sock_y);
        /* Node top must be strictly under the wire. */
        const float clear_under_wire = wire_y - wire_pad;

        /* Collect intermediate descendants currently collapsed onto the spine. */
        Vector<int> side;
        for (const int i : members) {
          if (layer[i] <= layer[e.from] || layer[i] >= layer[e.to]) {
            continue;
          }
          /* Near spine top *or* still overlapping the wire band. */
          const bool near_spine = math::abs(rel[i].y - spine_y) < gap * 0.75f;
          const bool covers_wire = rel[i].y > clear_under_wire;
          if (!near_spine && !covers_wire) {
            continue;
          }
          bool under = false;
          Vector<int> st;
          Vector<uint8_t> seen(n, 0);
          st.append(e.from);
          seen[e.from] = 1;
          while (!st.is_empty()) {
            const int u = st.pop_last();
            if (u == i) {
              under = true;
              break;
            }
            for (const int v : children[u]) {
              if (!seen[v] && layer[v] <= layer[e.to]) {
                seen[v] = 1;
                st.append(v);
              }
            }
          }
          if (under) {
            side.append(i);
          }
        }
        if (side.is_empty()) {
          continue;
        }

        /* Stable order: top→bottom by current Y, then layer, then index. */
        std::sort(side.begin(), side.end(), [&](const int a, const int b) {
          if (rel[a].y != rel[b].y) {
            return rel[a].y > rel[b].y;
          }
          if (layer[a] != layer[b]) {
            return layer[a] < layer[b];
          }
          return a < b;
        });

        /* First: clear under bare wire. Later: stack by full node height. */
        float cursor_y = clear_under_wire;
        for (int si = 0; si < side.size(); si++) {
          const int i = side[si];
          float target_y = cursor_y;
          for (const int j : members) {
            if (j == i || layer[j] != layer[i]) {
              continue;
            }
            const bool j_on_spine = math::abs(rel[j].y - spine_y) < gap * 0.75f;
            const bool j_above = rel[j].y > target_y;
            if (j_on_spine || j_above) {
              /* Real node above / on spine → full height clearance. */
              target_y = math::min(target_y, rel[j].y - items[j].height - gap);
            }
          }
          /* Never leave the header overlapping the long link. */
          target_y = math::min(target_y, clear_under_wire);
          rel[i].y = target_y;
          cursor_y = rel[i].y - items[i].height - gap;
        }
      }
    }

    /* Center component at origin. */
    float2 comp_center = float2(0.0f);
    for (const int i : members) {
      comp_center.x += rel[i].x + items[i].width * 0.5f;
      comp_center.y += rel[i].y - items[i].height * 0.5f;
    }
    comp_center /= float(members.size());
    for (const int i : members) {
      rel[i] -= comp_center;
    }
  }

  /* Stack disconnected components on the cross axis. */
  if (component_count > 1) {
    Vector<float> comp_cross_min(component_count, FLT_MAX);
    Vector<float> comp_cross_max(component_count, -FLT_MAX);
    for (int i = 0; i < n; i++) {
      const int c = component[i];
      const float cross0 = flow_horizontal ? (rel[i].y - items[i].height) : rel[i].x;
      const float cross1 = flow_horizontal ? rel[i].y : (rel[i].x + items[i].width);
      comp_cross_min[c] = math::min(comp_cross_min[c], cross0);
      comp_cross_max[c] = math::max(comp_cross_max[c], cross1);
    }
    float total_cross = 0.0f;
    for (int c = 0; c < component_count; c++) {
      total_cross += comp_cross_max[c] - comp_cross_min[c];
    }
    total_cross += gap * 2.0f * float(component_count - 1);
    float cross_cursor = -total_cross * 0.5f;
    Vector<float> comp_cross_shift(component_count, 0.0f);
    for (int c = 0; c < component_count; c++) {
      const float span = comp_cross_max[c] - comp_cross_min[c];
      comp_cross_shift[c] = cross_cursor - comp_cross_min[c];
      cross_cursor += span + gap * 2.0f;
    }
    for (int i = 0; i < n; i++) {
      const float sh = comp_cross_shift[component[i]];
      if (flow_horizontal) {
        rel[i].y += sh;
      }
      else {
        rel[i].x += sh;
      }
    }
  }

  float2 layout_center = float2(0.0f);
  for (int i = 0; i < n; i++) {
    layout_center.x += rel[i].x + items[i].width * 0.5f;
    layout_center.y += rel[i].y - items[i].height * 0.5f;
  }
  layout_center /= float(n);
  const float2 offset = free_center - layout_center;

  for (int i = 0; i < n; i++) {
    items[i].td->loc[0] = rel[i].x + offset.x;
    items[i].td->loc[1] = rel[i].y + offset.y;
  }
}

static void move_child_nodes(bNode &node, const float2 &delta)
{
  for (bNode *child : node.direct_children_in_frame()) {
    child->location[0] += delta.x;
    child->location[1] += delta.y;
    if (child->is_frame()) {
      move_child_nodes(*child, delta);
    }
  }
}

static bool has_selected_parent(const bNode &node)
{
  for (bNode *parent = node.parent; parent; parent = parent->parent) {
    if (parent->is_selected()) {
      return true;
    }
  }
  return false;
}

/**
 * Houdini-style shake-to-detach: while dragging selected nodes, rapid mouse direction changes
 * detach external links (combo-internal links are preserved). Thresholds are in region pixels.
 */
static void node_transform_shake_detach_update(TransInfo *t, TransCustomDataNode &customdata)
{
  if (customdata.shake_detached || t->state == TRANS_CANCEL) {
    return;
  }
  SpaceNode *snode = t->area->spacedata.first_as<SpaceNode>();
  if (!snode || !snode->edittree) {
    return;
  }
  if (!space_node::node_selected_have_external_links(*snode->edittree)) {
    return;
  }

  constexpr float min_segment_px = 12.0f;
  constexpr float min_segment_px_sq = min_segment_px * min_segment_px;
  constexpr int required_dir_changes = 3;
  constexpr double window_seconds = 0.45;

  const float2 mval = t->mval;
  const double now = BLI_time_now_seconds();

  if (!customdata.shake_has_prev) {
    customdata.shake_prev_mval = mval;
    customdata.shake_has_prev = true;
    customdata.shake_window_start = now;
    customdata.shake_dir_changes = 0;
    customdata.shake_prev_dir = float2(0.0f);
    return;
  }

  if (now - customdata.shake_window_start > window_seconds) {
    customdata.shake_window_start = now;
    customdata.shake_dir_changes = 0;
    customdata.shake_prev_dir = float2(0.0f);
    customdata.shake_prev_mval = mval;
    return;
  }

  const float2 delta = mval - customdata.shake_prev_mval;
  const float dist_sq = math::length_squared(delta);
  if (dist_sq < min_segment_px_sq) {
    return;
  }

  const float2 dir = math::normalize(delta);
  if (math::length_squared(customdata.shake_prev_dir) > 0.0f) {
    const float dot = math::dot(dir, customdata.shake_prev_dir);
    /* Count a direction change when the mouse turns more than ~90 degrees. */
    if (dot < 0.0f) {
      customdata.shake_dir_changes++;
    }
  }
  customdata.shake_prev_dir = dir;
  customdata.shake_prev_mval = mval;

  if (customdata.shake_dir_changes < required_dir_changes) {
    return;
  }

  space_node::node_detach_selected_links(*snode->edittree);
  BKE_main_ensure_invariants(*CTX_data_main(t->context), snode->edittree->id);
  customdata.shake_detached = true;
  customdata.shake_dir_changes = 0;
  WM_event_add_notifier(t->context, NC_NODE | ND_DISPLAY, nullptr);
}

static void flushTransNodes(TransInfo *t)
{
  const float dpi_fac = UI_SCALE_FAC;
  SpaceNode *snode = t->area->spacedata.first_as<SpaceNode>();

  TransCustomDataNode *customdata = static_cast<TransCustomDataNode *>(t->custom.type.data);

  if (t->options & CTX_VIEW2D_EDGE_PAN) {
    if (t->state == TRANS_CANCEL) {
      view2d_edge_pan_cancel(t->context, &customdata->edgepan_data);
    }
    else {
      /* Edge panning functions expect window coordinates, mval is relative to region. */
      const int xy[2] = {
          t->region->winrct.xmin + int(t->mval[0]),
          t->region->winrct.ymin + int(t->mval[1]),
      };
      ui::view2d_edge_pan_apply(t->context, &customdata->edgepan_data, xy);
    }
  }

  float offset[2] = {0.0f, 0.0f};
  if (t->state != TRANS_CANCEL) {
    if (t->region &&
        !BLI_rctf_compare(&customdata->viewrect_prev, &t->region->v2d.cur, FLT_EPSILON))
    {
      /* Additional offset due to change in view2D rect. */
      BLI_rctf_transform_pt_v(&t->region->v2d.cur, &customdata->viewrect_prev, offset, offset);
      transformViewUpdate(t);
      customdata->viewrect_prev = t->region->v2d.cur;
    }
  }

  if (t->modifiers & MOD_NODE_FRAME) {
    t->modifiers &= ~MOD_NODE_FRAME;
    Vector<bNode *> nodes_to_detach;
    for (bNode *node : snode->edittree->all_nodes()) {
      if (!node->is_selected()) {
        continue;
      }
      if (has_selected_parent(*node)) {
        continue;
      }
      if (!node->parent) {
        continue;
      }
      customdata->old_parent_by_detached_node.add(node, node->parent);
      nodes_to_detach.append(node);
    }
    if (nodes_to_detach.is_empty()) {
      WM_operator_name_call(
          t->context, "NODE_OT_attach", wm::OpCallContext::InvokeDefault, nullptr, nullptr);
    }
    else {
      for (bNode *node : nodes_to_detach) {
        bke::node_detach_node(*snode->edittree, *node);
      }
    }
  }

  if (t->state != TRANS_CANCEL) {
    node_transform_shake_detach_update(t, *customdata);
  }

  /* Grid snap and U-key align+pack adjust proposed transform locations once for all nodes. */
  node_snap_grid_apply(t);
  node_selection_align_apply(t);

  FOREACH_TRANS_DATA_CONTAINER (t, tc) {
    /* Flush to 2d vector from internally used 3d vector. */
    for (int i = 0; i < tc->data_len; i++) {
      TransData *td = &tc->data[i];
      TransData2D *td2d = &tc->data_2d[i];
      bNode *node = static_cast<bNode *>(td->extra);

      float2 loc = float2(td2d->loc) + offset;

      /* Weirdo - but the node system is a mix of free 2d elements and DPI sensitive UI. */
      loc /= dpi_fac;

      if (node->is_frame()) {
        const float2 delta = loc - float2(node->location);
        move_child_nodes(*node, delta);
      }

      node->location[0] = loc.x;
      node->location[1] = loc.y;
    }

    /* Handle intersection with noodles (single node or multi-node combo). */
    if (tc->data_len >= 1 && t->region) {
      space_node::node_insert_on_link_flags_set(
          *snode, *t->region, node_transform_auto_insert_enabled(t), customdata->is_new_node);
    }
    if (t->region) {
      space_node::node_insert_on_frame_flag_set(*snode, *t->region, int2(t->mval));
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Special After Transform Node
 * \{ */

static void special_aftertrans_update__node(bContext *C, TransInfo *t)
{
  Main *bmain = CTX_data_main(C);
  SpaceNode *snode = t->area->spacedata.first_as<SpaceNode>();
  bNodeTree *ntree = snode->edittree;
  const TransCustomDataNode &customdata = *static_cast<TransCustomDataNode *>(t->custom.type.data);

  const bool canceled = (t->state == TRANS_CANCEL);

  if (canceled) {
    for (auto &&[node, parent] : customdata.old_parent_by_detached_node.items()) {
      bke::node_attach_node(*ntree, *node, *parent);
    }
  }
  if (canceled && t->remove_on_cancel) {
    /* Remove selected nodes on cancel. */
    if (ntree) {
      for (bNode &node : ntree->nodes.items_mutable()) {
        if (node.is_selected()) {
          bke::node_remove_node(bmain, *ntree, node, true);
        }
      }
      BKE_main_ensure_invariants(*bmain, ntree->id);
    }
  }

  if (!canceled) {
    ED_node_post_apply_transform(C, snode->edittree);
    if (node_transform_auto_insert_enabled(t)) {
      space_node::node_insert_on_link_flags(*bmain, *snode, customdata.is_new_node);
    }
  }

  space_node::node_insert_on_link_flags_clear(*ntree);
  space_node::node_insert_on_frame_flag_clear(*snode);

  wmOperatorType *ot = WM_operatortype_find("NODE_OT_insert_offset", true);
  BLI_assert(ot);
  PointerRNA ptr = WM_operator_properties_create_ptr(ot);
  WM_operator_name_call_ptr(C, ot, wm::OpCallContext::InvokeDefault, &ptr, nullptr);
  WM_operator_properties_free(&ptr);
}

/** \} */

TransConvertTypeInfo TransConvertType_Node = {
    /*flags*/ (T_POINTS | T_2D_EDIT),
    /*create_trans_data*/ createTransNodeData,
    /*recalc_data*/ flushTransNodes,
    /*special_aftertrans_update*/ special_aftertrans_update__node,
};

}  // namespace blender::ed::transform
