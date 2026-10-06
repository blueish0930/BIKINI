/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 *
 * Line-connected recursion tree. Each node is labeled with its depth. Forks at the same
 * depth (Fibonacci calls the group twice) are separate nodes. Clicking one shows that
 * invocation in the node editor. The editor does not let you enter the self group node.
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "MEM_guardedalloc.h"

#include "BLI_listbase.hh"
#include "BLI_rect.hh"
#include "BLI_string_ref.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_set.hh"
#include "BLI_span.hh"
#include "BLI_string_utf8.hh"
#include "BLI_vector.hh"

#include "BLT_translation.hh"

#include "BKE_compute_context_cache.hh"
#include "BKE_compute_contexts.hh"
#include "BKE_context.hh"
#include "BKE_main.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_node_runtime.hh"
#include "BKE_report.hh"
#include "BKE_screen.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"
#include "DNA_workspace_types.h"

#include "ED_node.hh"
#include "ED_screen.hh"
#include "ED_viewer_path.hh"

#include "GPU_immediate.hh"
#include "GPU_framebuffer.hh"
#include "GPU_matrix.hh"
#include "GPU_shader_builtin.hh"
#include "GPU_state.hh"
#include "GPU_viewport.hh"

#include "NOD_eval_log.hh"

#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_resources.hh"
#include "UI_view2d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "MOD_nodes.hh"

#include "node_intern.hh"

namespace blender::ed::space_node {

struct RecursionTreeActive {
  uint32_t uid = 0;
  bool has_hash = false;
  ComputeContextHash hash{};
};

static RecursionTreeActive g_recursion_tree_active;

/** A context step below the edit tree's context with the hash it was logged under. */
struct ChainStep {
  RecursiveInspectionStep step;
  ComputeContextHash hash;
};

struct PendingCall {
  ComputeContextHash hash;
  std::string name;
  Vector<ChainStep> chain;
};

struct GraphPoint {
  ComputeContextHash hash;
  int depth = 0;
  std::string tip;
  /** Steps from the edit tree's context to this call. Zone indices are resolved on click. */
  Vector<ChainStep> chain;
  Vector<int> children;
  float slot_x = 0.0f;
  float2 center = float2(0.0f);
};

static bool is_self_group_call(const bNodeTree &tree, const bNode *node)
{
  if (node == nullptr || node->id == nullptr || GS(node->id->name) != ID_NT) {
    return false;
  }
  const bNodeTree &called = *reinterpret_cast<const bNodeTree *>(node->id);
  if (&called == &tree || called.id.session_uid == tree.id.session_uid) {
    return true;
  }
  if (const bNodeTree *orig = DEG_get_original(&called)) {
    return orig == &tree || orig->id.session_uid == tree.id.session_uid;
  }
  return false;
}

static std::string node_call_name(const bNode &node)
{
  if (node.label[0] != '\0') {
    return node.label;
  }
  return node.name;
}

static std::string depth_tip(const int depth, const StringRef name)
{
  std::string tip = IFACE_("Depth ");
  tip += std::to_string(depth);
  if (!name.is_empty()) {
    tip += ": ";
    tip += name;
  }
  return tip;
}

static std::optional<RecursiveInspectionStep::Type> step_type_for_caller(const bNode &caller)
{
  if (caller.is_group()) {
    return RecursiveInspectionStep::Type::GroupNode;
  }
  switch (caller.type_legacy) {
    case GEO_NODE_SIMULATION_OUTPUT:
      return RecursiveInspectionStep::Type::SimulationZone;
    case GEO_NODE_REPEAT_OUTPUT:
      return RecursiveInspectionStep::Type::RepeatZone;
    case GEO_NODE_FOREACH_GEOMETRY_ELEMENT_OUTPUT:
      return RecursiveInspectionStep::Type::ForeachZone;
    default:
      return std::nullopt;
  }
}

static void collect_direct_calls(nodes::eval_log::NodesEvalLog &log,
                                 const bNodeTree &root_tree,
                                 const bNodeTree &context_tree,
                                 const ComputeContextHash hash,
                                 const Span<ChainStep> chain,
                                 Set<ComputeContextHash> &visiting,
                                 Vector<PendingCall> &r_calls)
{
  if (!visiting.add(hash)) {
    return;
  }
  /* Child contexts are merged from per-thread loggers, so their order changes between
   * evaluations. Sort them by the caller node's position so the layout stays put. */
  struct ChildCall {
    ComputeContextHash hash;
    const bNode *caller;
  };
  Vector<ChildCall> children;
  nodes::eval_log::NodeTreeLog &tree_log = log.get_tree_log(hash);
  tree_log.foreach_child_context_hash([&](const ComputeContextHash child_hash) {
    nodes::eval_log::NodeTreeLog &child_log = log.get_tree_log(child_hash);
    const bNode *caller = nullptr;
    if (const std::optional<int32_t> caller_id = child_log.caller_node_id()) {
      caller = context_tree.node_by_id(*caller_id);
    }
    children.append({child_hash, caller});
  });
  std::sort(children.begin(), children.end(), [](const ChildCall &a, const ChildCall &b) {
    if ((a.caller == nullptr) != (b.caller == nullptr)) {
      return b.caller == nullptr;
    }
    if (a.caller != nullptr && a.caller != b.caller) {
      /* Top to bottom, then left to right in the editor. */
      if (a.caller->location[1] != b.caller->location[1]) {
        return a.caller->location[1] > b.caller->location[1];
      }
      if (a.caller->location[0] != b.caller->location[0]) {
        return a.caller->location[0] < b.caller->location[0];
      }
      return a.caller->identifier < b.caller->identifier;
    }
    return std::pair(a.hash.v1, a.hash.v2) < std::pair(b.hash.v1, b.hash.v2);
  });

  for (const ChildCall &child_call : children) {
    const ComputeContextHash child_hash = child_call.hash;
    const bNode *caller = child_call.caller;
    Vector<ChainStep> child_chain(chain);
    if (caller != nullptr) {
      if (const std::optional<RecursiveInspectionStep::Type> type = step_type_for_caller(*caller))
      {
        RecursiveInspectionStep step;
        step.type = *type;
        step.node_id = caller->identifier;
        child_chain.append({step, child_hash});
      }
    }
    if (is_self_group_call(root_tree, caller)) {
      PendingCall call;
      call.hash = child_hash;
      call.name = node_call_name(*caller);
      call.chain = std::move(child_chain);
      r_calls.append(std::move(call));
      continue;
    }
    const bNodeTree *next_tree = &context_tree;
    if (caller != nullptr && caller->is_group() && caller->id != nullptr &&
        GS(caller->id->name) == ID_NT)
    {
      next_tree = reinterpret_cast<const bNodeTree *>(caller->id);
    }
    if (next_tree->type == NTREE_GEOMETRY) {
      collect_direct_calls(log,
                           root_tree,
                           *next_tree,
                           child_hash,
                           child_chain,
                           visiting,
                           r_calls);
    }
  }
}

static SpaceNode *find_source_editor(wmWindowManager &wm, const bNodeTree *tree)
{
  if (tree == nullptr) {
    return nullptr;
  }
  for (wmWindow &win : wm.windows) {
    bScreen *screen = WM_window_get_active_screen(&win);
    if (screen == nullptr) {
      continue;
    }
    for (ScrArea &area : screen->areabase) {
      if (area.spacetype != SPACE_NODE) {
        continue;
      }
      SpaceNode *snode = area.spacedata.first_as<SpaceNode>();
      if (snode->runtime && snode->runtime->recursion_tree_window) {
        continue;
      }
      if (snode->edittree == tree) {
        return snode;
      }
    }
  }
  return nullptr;
}

static void tag_tree_areas(wmWindowManager &wm, const uint32_t tree_uid)
{
  for (wmWindow &win : wm.windows) {
    bScreen *screen = WM_window_get_active_screen(&win);
    if (screen == nullptr) {
      continue;
    }
    for (ScrArea &area : screen->areabase) {
      if (area.spacetype != SPACE_NODE) {
        continue;
      }
      SpaceNode *snode = area.spacedata.first_as<SpaceNode>();
      const bool graph = snode->runtime && snode->runtime->recursion_tree_window &&
                         snode->runtime->recursion_tree_uid == tree_uid;
      const bool editor = snode->edittree && snode->edittree->id.session_uid == tree_uid;
      if (graph || editor) {
        ED_area_tag_redraw(&area);
      }
    }
  }
}

/** Stands in for a logged context of which only the hash is known. */
class LoggedHashComputeContext : public ComputeContext {
  ComputeContextHash hash_;

 public:
  LoggedHashComputeContext(const ComputeContextHash hash) : ComputeContext(nullptr), hash_(hash) {}

 private:
  ComputeContextHash compute_hash() const override
  {
    return hash_;
  }

  void print_current_in_line(std::ostream &stream) const override
  {
    stream << "Logged context";
  }
};

/**
 * Turns the logged chain into steps that rebuild the same compute context hashes. Zone iteration
 * indices are not logged, so they are searched. Returns an empty path when a step can't be found.
 */
static Vector<RecursiveInspectionStep> resolve_inspection_path(const ComputeContextHash base_hash,
                                                               const Span<ChainStep> chain)
{
  constexpr int max_zone_index = 1 << 16;
  Vector<RecursiveInspectionStep> path;
  ComputeContextHash parent_hash = base_hash;
  for (const ChainStep &item : chain) {
    const LoggedHashComputeContext parent(parent_hash);
    RecursiveInspectionStep step = item.step;
    bool found = false;
    switch (step.type) {
      case RecursiveInspectionStep::Type::GroupNode:
        found = bke::GroupNodeComputeContext(&parent, step.node_id).hash() == item.hash;
        break;
      case RecursiveInspectionStep::Type::SimulationZone:
        found = bke::SimulationZoneComputeContext(&parent, step.node_id).hash() == item.hash;
        break;
      case RecursiveInspectionStep::Type::RepeatZone:
        for (int i = 0; i < max_zone_index && !found; i++) {
          if (bke::RepeatZoneComputeContext(&parent, step.node_id, i).hash() == item.hash) {
            step.index = i;
            found = true;
          }
        }
        break;
      case RecursiveInspectionStep::Type::ForeachZone:
        for (int i = 0; i < max_zone_index && !found; i++) {
          if (bke::ForeachGeometryElementZoneComputeContext(&parent, step.node_id, i).hash() ==
              item.hash)
          {
            step.index = i;
            found = true;
          }
        }
        break;
    }
    if (!found) {
      return {};
    }
    path.append(step);
    parent_hash = item.hash;
  }
  return path;
}

/** Full evaluation, so the newly inspected call gets socket values and viewers logged. */
static void request_inspection_eval(bContext &C, ID &id)
{
  if (Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C)) {
    DEG_id_tag_update_for_side_effect_request(depsgraph, &id, ID_RECALC_GEOMETRY);
  }
  else {
    DEG_id_tag_update(&id, ID_RECALC_GEOMETRY);
  }
  WM_main_add_notifier(NC_NODE | ND_DISPLAY, nullptr);
}

/**
 * Evaluates only the inspected call with the inputs it had in the last evaluation, see
 * #MOD_nodes_evaluate_recursive_call. The edit tree's own context is logged by every evaluation,
 * so it only needs that when a viewer is active. Returns false when a full evaluation is needed.
 */
static bool evaluate_inspected_call(bContext &C, const SpaceNode &snode, const bool viewer_active)
{
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
  const std::optional<ObjectAndModifier> object_and_modifier =
      get_geometry_nodes_modifier_for_node_editor(snode);
  if (depsgraph == nullptr || !object_and_modifier || snode.edittree == nullptr) {
    return false;
  }
  Object &object = *const_cast<Object *>(object_and_modifier->object);
  NodesModifierData &nmd = *const_cast<NodesModifierData *>(object_and_modifier->nmd);
  if (!nmd.runtime->eval_log) {
    return false;
  }
  bke::ComputeContextCache compute_context_cache;
  const ComputeContext *base = compute_context_for_edittree_base(snode, compute_context_cache);
  const ComputeContext *call = compute_context_for_edittree(snode, compute_context_cache);
  if (base == nullptr || call == nullptr) {
    return false;
  }
  if (call->hash() == base->hash() && !viewer_active) {
    nmd.runtime->eval_log->set_inspection_overlay(nullptr, call->hash());
    return true;
  }
  if (call->hash() != base->hash() && snode.runtime->recursive_inspection_path.is_empty()) {
    /* Only a hash is known, nested contexts would not match the evaluation. */
    return false;
  }
  return MOD_nodes_evaluate_recursive_call(*depsgraph, object, nmd, *snode.edittree, *call);
}

static void tag_inspection_redraw(wmWindowManager &wm)
{
  for (wmWindow &win : wm.windows) {
    bScreen *screen = WM_window_get_active_screen(&win);
    if (screen == nullptr) {
      continue;
    }
    for (ScrArea &area : screen->areabase) {
      if (ELEM(area.spacetype, SPACE_NODE, SPACE_VIEW3D, SPACE_SPREADSHEET)) {
        ED_area_tag_redraw(&area);
      }
    }
  }
}

static void inspect_call(bContext &C,
                         const uint32_t tree_uid,
                         const ComputeContextHash hash,
                         const Span<ChainStep> chain)
{
  wmWindowManager *wm = CTX_wm_manager(&C);
  Main *bmain = CTX_data_main(&C);
  if (wm == nullptr || bmain == nullptr) {
    return;
  }
  for (wmWindow &win : wm->windows) {
    bScreen *screen = WM_window_get_active_screen(&win);
    if (screen == nullptr) {
      continue;
    }
    for (ScrArea &area : screen->areabase) {
      if (area.spacetype != SPACE_NODE) {
        continue;
      }
      SpaceNode *snode = area.spacedata.first_as<SpaceNode>();
      if (snode->runtime == nullptr || snode->runtime->recursion_tree_window) {
        continue;
      }
      if (snode->edittree == nullptr || snode->edittree->id.session_uid != tree_uid) {
        continue;
      }
      /* Socket values are only logged for contexts shown in an editor, and a viewer path
       * points at one specific call. Both have to follow the newly inspected call. */
      WorkSpace *workspace = WM_window_get_active_workspace(&win);
      bNode *active_viewer = workspace ? viewer_path::find_geometry_nodes_viewer(
                                             workspace->viewer_path, *snode) :
                                         nullptr;

      bke::ComputeContextCache compute_context_cache;
      const ComputeContext *base = compute_context_for_edittree_base(*snode,
                                                                     compute_context_cache);
      snode->runtime->recursive_inspection_tree_uid = tree_uid;
      snode->runtime->recursive_inspection_hash = hash;
      snode->runtime->recursive_inspection_path = base ? resolve_inspection_path(base->hash(),
                                                                                 chain) :
                                                         Vector<RecursiveInspectionStep>();

      if (snode->id == nullptr || GS(snode->id->name) != ID_OB) {
        continue;
      }
      if (active_viewer != nullptr) {
        viewer_path::activate_geometry_node(
            *bmain, *snode, *active_viewer, std::nullopt, false);
      }
      if (evaluate_inspected_call(C, *snode, active_viewer != nullptr)) {
        tag_inspection_redraw(*wm);
      }
      else {
        request_inspection_eval(C, *snode->id);
      }
    }
  }
  g_recursion_tree_active.uid = tree_uid;
  g_recursion_tree_active.has_hash = true;
  g_recursion_tree_active.hash = hash;
  tag_tree_areas(*wm, tree_uid);
}

static int append_calls(Vector<GraphPoint> &points,
                        const int parent,
                        nodes::eval_log::NodesEvalLog &log,
                        bNodeTree &tree,
                        const ComputeContextHash hash,
                        const int depth,
                        Set<ComputeContextHash> &visiting)
{
  Vector<PendingCall> calls;
  const Vector<ChainStep> chain = points[parent].chain;
  collect_direct_calls(log, tree, tree, hash, chain, visiting, calls);
  for (PendingCall &call : calls) {
    const int child = points.size();
    points.append({});
    GraphPoint &point = points.last();
    point.hash = call.hash;
    point.chain = std::move(call.chain);
    point.depth = depth + 1;
    point.tip = depth_tip(depth + 1, call.name);
    points[parent].children.append(child);
  }
  const int child_count = points[parent].children.size();
  for (const int i : IndexRange(child_count)) {
    const int child = points[parent].children[i];
    append_calls(points,
                 child,
                 log,
                 tree,
                 points[child].hash,
                 depth + 1,
                 visiting);
  }
  return child_count;
}

static void place_points(Vector<GraphPoint> &points, const int index, int &slot)
{
  GraphPoint &point = points[index];
  if (point.children.is_empty()) {
    point.slot_x = float(slot);
    slot++;
  }
  else {
    for (const int child : point.children) {
      place_points(points, child, slot);
    }
    float sum = 0.0f;
    for (const int child : point.children) {
      sum += points[child].slot_x;
    }
    point.slot_x = sum / float(point.children.size());
  }
}

/* Layout in View2D units before UI scale. Siblings are spaced wider than a button. */
static constexpr float tree_node_w = 52.0f;
static constexpr float tree_node_h = 28.0f;
static constexpr float tree_gap_x = tree_node_w + 12.0f;
static constexpr float tree_gap_y = 64.0f;
static constexpr float tree_margin = 36.0f;

/**
 * Places the points in View2D space with the root at the origin, so the view stays anchored
 * on the root when the tree grows or shrinks. Returns the bounds including node size.
 */
static rctf position_points(Vector<GraphPoint> &points)
{
  rctf bounds;
  BLI_rctf_init(&bounds, 0.0f, 0.0f, 0.0f, 0.0f);
  if (points.is_empty()) {
    return bounds;
  }
  int slot = 0;
  place_points(points, 0, slot);

  const float node_w = tree_node_w * UI_SCALE_FAC;
  const float node_h = tree_node_h * UI_SCALE_FAC;
  const float gap_x = tree_gap_x * UI_SCALE_FAC;
  const float gap_y = tree_gap_y * UI_SCALE_FAC;
  const float root_x = points[0].slot_x;
  BLI_rctf_init_minmax(&bounds);
  for (GraphPoint &point : points) {
    point.center.x = (point.slot_x - root_x) * gap_x;
    point.center.y = -float(point.depth) * gap_y;
    BLI_rctf_do_minmax_v(&bounds, point.center);
  }
  BLI_rctf_pad(&bounds, node_w * 0.5f, node_h * 0.5f);
  return bounds;
}

/** Initially show depths zero through seven, with the root centered horizontally. */
static void frame_tree(ARegion &region)
{
  View2D &v2d = region.v2d;
  const float win_w = std::max(float(region.winx), 1.0f);
  const float win_h = std::max(float(region.winy), 1.0f);
  const float view_h = 8.0f * tree_gap_y * UI_SCALE_FAC;
  const float view_w = view_h * win_w / win_h;
  const float top = 0.65f * tree_gap_y * UI_SCALE_FAC;
  BLI_rctf_init(&v2d.cur, -view_w * 0.5f, view_w * 0.5f, top - view_h, top);
  ui::view2d_curRect_validate(&v2d);
}

/** Draws a text line in region pixel space, unaffected by the tree zoom. */
static void draw_message(const bContext &C, ARegion &region, const StringRef text, const int y)
{
  ui::view2d_view_restore(&C);
  ui::Block *block = ui::block_begin(&C, &region, "recursion_tree_message", ui::EmbossType::Emboss);
  ui::uiDefBut(block,
               ui::ButtonType::Label,
               text,
               12,
               y,
               short(std::max(region.winx - 24, 20)),
               UI_UNIT_Y,
               nullptr,
               0,
               0,
               std::nullopt);
  ui::block_end(&C, block);
  ui::block_draw(&C, block);
}

static void draw_links(const Span<GraphPoint> points)
{
  int segments = 0;
  for (const GraphPoint &point : points) {
    if (point.children.is_empty()) {
      continue;
    }
    segments += 1;
    if (point.children.size() > 1) {
      segments += 1;
    }
    segments += point.children.size();
  }
  if (segments == 0) {
    return;
  }

  float color[4];
  ui::theme::get_color_4fv(TH_TEXT, color);
  color[3] = 0.65f;

  GPUVertFormat *format = immVertexFormat();
  const uint pos = GPU_vertformat_attr_add(format, "pos", gpu::VertAttrType::SFLOAT_32_32);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);
  immUniformColor4fv(color);
  GPU_line_width(std::max(1.5f * U.pixelsize, 1.0f));
  immBegin(GPU_PRIM_LINES, uint(segments) * 2);
  for (const GraphPoint &point : points) {
    if (point.children.is_empty()) {
      continue;
    }
    const float2 &parent = point.center;
    const float2 &first = points[point.children.first()].center;
    const float2 &last = points[point.children.last()].center;
    const float mid_y = (parent.y + first.y) * 0.5f;
    immVertex2f(pos, parent.x, parent.y);
    immVertex2f(pos, parent.x, mid_y);
    if (point.children.size() > 1) {
      immVertex2f(pos, first.x, mid_y);
      immVertex2f(pos, last.x, mid_y);
    }
    for (const int child : point.children) {
      const float2 &co = points[child].center;
      immVertex2f(pos, co.x, mid_y);
      immVertex2f(pos, co.x, co.y);
    }
  }
  immEnd();
  immUnbindProgram();
}

void node_recursive_tree_draw(const bContext &C, ARegion &region)
{
  /* The regular node editor limits zoom and view size. A complete recursion tree can be much
   * wider, so let this window keep zooming out as far as the View2D coordinates allow. */
  region.v2d.keepzoom &= ~V2D_LIMITZOOM;
  region.v2d.max[0] = std::numeric_limits<float>::max();
  region.v2d.max[1] = std::numeric_limits<float>::max();

  const SpaceNode *graph = CTX_wm_space_node(&C);
  /* Node editor regions are composited from the viewport overlay framebuffer. Drawing to the
   * window framebuffer leaves a completely black secondary window. */
  GPUViewport *viewport = WM_draw_region_get_viewport(&region);
  gpu::FrameBuffer *overlay = GPU_viewport_framebuffer_overlay_get(viewport);
  GPU_framebuffer_bind_no_srgb(overlay);
  ui::view2d_view_ortho(&region.v2d);
  ui::theme::frame_buffer_clear(TH_BACK);
  GPU_depth_test(GPU_DEPTH_NONE);
  GPU_blend(GPU_BLEND_ALPHA);
  if (graph == nullptr || graph->runtime == nullptr) {
    return;
  }
  /* The window's own tree pointer is remapped by undo and file reload, while session UIDs can
   * change when IDs are re-created. Keep the cached UID in sync with the tree pointer. */
  const bNodeTree *graph_tree = graph->edittree;
  if (graph_tree != nullptr) {
    graph->runtime->recursion_tree_uid = graph_tree->id.session_uid;
  }
  const uint32_t tree_uid = graph->runtime->recursion_tree_uid;
  wmWindowManager *wm = CTX_wm_manager(&C);
  SpaceNode *source = wm ? find_source_editor(*wm, graph_tree) : nullptr;

  if (source == nullptr || source->edittree == nullptr) {
    draw_message(C,
                 region,
                 IFACE_("Open this geometry node group in the node editor."),
                 region.winy / 2);
    return;
  }

  bNodeTree &tree = *source->edittree;
  tree.ensure_topology_cache();

  bke::ComputeContextCache compute_context_cache;
  const ComputeContext *root = compute_context_for_edittree_base(*source, compute_context_cache);
  nodes::eval_log::NodesEvalLog *log = nodes::eval_log::NodesEvalLog::from_space_node(*source);

  Vector<GraphPoint> points;
  if (root != nullptr) {
    points.append({});
    GraphPoint &root_point = points.last();
    root_point.hash = root->hash();
    root_point.depth = 0;
    root_point.tip = depth_tip(0, "");
    if (log != nullptr) {
      Set<ComputeContextHash> visiting;
      append_calls(points, 0, *log, tree, root->hash(), 0, visiting);
    }
  }
  const rctf bounds = position_points(points);
  region.v2d.tot = bounds;
  BLI_rctf_pad(&region.v2d.tot, tree_margin * UI_SCALE_FAC, tree_margin * UI_SCALE_FAC);
  region.v2d.scroll |= V2D_SCROLL_BOTTOM;
  region.v2d.scroll &= ~(V2D_SCROLL_HORIZONTAL_HIDE | V2D_SCROLL_HORIZONTAL_FULLR);
  region.v2d.alpha_hor = 255;

  /* Frame the tree when it first appears or its shape changes. Otherwise keep the user's zoom
   * and pan. Skip frames without a log so a pending evaluation does not make the view jump. */
  if (log != nullptr && !points.is_empty() &&
      graph->runtime->recursion_tree_framed_points != points.size())
  {
    graph->runtime->recursion_tree_framed_points = points.size();
    frame_tree(region);
    ui::view2d_view_ortho(&region.v2d);
  }

  if (source->runtime) {
    g_recursion_tree_active.uid = tree_uid;
    g_recursion_tree_active.has_hash = source->runtime->recursive_inspection_hash.has_value() &&
                                       source->runtime->recursive_inspection_tree_uid == tree_uid;
    if (g_recursion_tree_active.has_hash) {
      g_recursion_tree_active.hash = *source->runtime->recursive_inspection_hash;
    }
  }

  draw_links(points);

  ui::Block *block = ui::block_begin(&C, &region, "recursion_tree", ui::EmbossType::Emboss);
  const float bw = tree_node_w * UI_SCALE_FAC;
  const float bh = tree_node_h * UI_SCALE_FAC;
  Vector<std::string> labels;
  labels.reserve(points.size());
  for (GraphPoint &point : points) {
    labels.append(std::to_string(point.depth));
    ui::Button *but = ui::uiDefBut(block,
                                   ui::ButtonType::But,
                                   labels.last(),
                                   int(std::lround(point.center.x - bw * 0.5f)),
                                   int(std::lround(point.center.y - bh * 0.5f)),
                                   short(bw),
                                   short(bh),
                                   nullptr,
                                   0,
                                   0,
                                   point.tip);
    const ComputeContextHash hash = point.hash;
    ui::button_func_set(but, [hash, tree_uid, chain = point.chain](bContext &C) {
      inspect_call(C, tree_uid, hash, chain);
    });
    /* Highlight the inspected call with BUT_ACTIVE_DEFAULT. A pushed-state callback would rewrite
     * UI_SELECT when the press redraws the block, and the release would then cancel the click. */
    const bool is_active = g_recursion_tree_active.has_hash ? g_recursion_tree_active.hash == hash :
                                                              point.depth == 0;
    if (is_active) {
      ui::button_flag_enable(but, ui::BUT_ACTIVE_DEFAULT);
    }
  }
  ui::block_end(&C, block);
  ui::block_draw(&C, block);

  if (log == nullptr || root == nullptr) {
    draw_message(C, region, IFACE_("Evaluate the node group to list calls."), 8);
  }
  ui::view2d_view_restore(&C);
  ui::view2d_scrollers_draw(&region.v2d, nullptr);
}

static void recursion_tree_hide_chrome(bContext *C, ScrArea *area)
{
  for (ARegion &region : area->regionbase) {
    if (region.regiontype == RGN_TYPE_WINDOW) {
      region.flag &= ~(RGN_FLAG_HIDDEN | RGN_FLAG_HIDDEN_BY_USER);
      continue;
    }
    region.flag |= RGN_FLAG_HIDDEN | RGN_FLAG_HIDDEN_BY_USER;
  }
  for (ARegion &region : area->regionbase) {
    if (region.regiontype != RGN_TYPE_WINDOW) {
      ED_region_visibility_change_update(C, area, &region);
    }
  }
}

static void recursion_tree_strip_edit_keys(wmWindowManager *wm, ARegion *region)
{
  if (wm == nullptr || region == nullptr || region->runtime == nullptr) {
    return;
  }
  wmKeyMap *keymap = WM_keymap_ensure(
      wm->runtime->defaultconf, "Node Generic", SPACE_NODE, RGN_TYPE_WINDOW);
  WM_event_remove_keymap_handler(&region->runtime->handlers, keymap);
  keymap = WM_keymap_ensure(wm->runtime->defaultconf, "Node Editor", SPACE_NODE, RGN_TYPE_WINDOW);
  WM_event_remove_keymap_handler(&region->runtime->handlers, keymap);
}

static ScrArea *recursion_tree_area(wmWindow &win)
{
  bScreen *screen = WM_window_get_active_screen(&win);
  if (screen == nullptr) {
    return nullptr;
  }
  for (ScrArea &area : screen->areabase) {
    if (area.spacetype != SPACE_NODE) {
      continue;
    }
    SpaceNode *snode = area.spacedata.first_as<SpaceNode>();
    if (snode->runtime && snode->runtime->recursion_tree_window) {
      return &area;
    }
  }
  return nullptr;
}

static wmWindow *find_recursion_window(wmWindowManager &wm)
{
  for (wmWindow &win : wm.windows) {
    if (recursion_tree_area(win) != nullptr) {
      return &win;
    }
  }
  return nullptr;
}

static wmOperatorStatus recursion_tree_exec(bContext *C, wmOperator *op)
{
  SpaceNode *source = CTX_wm_space_node(C);
  if (source == nullptr || source->edittree == nullptr ||
      source->edittree->type != NTREE_GEOMETRY ||
      (source->edittree->flag & NTREE_GEOMETRY_RECURSIVE) == 0)
  {
    return OPERATOR_CANCELLED;
  }
  const uint32_t tree_uid = source->edittree->id.session_uid;
  wmWindowManager *wm = CTX_wm_manager(C);

  wmWindow *win = wm ? find_recursion_window(*wm) : nullptr;
  if (win == nullptr) {
    wmWindow *parent = CTX_wm_window(C);
    if (parent == nullptr) {
      return OPERATOR_CANCELLED;
    }
    const int width = std::max(int(780 * UI_SCALE_FAC), 480);
    const int height = std::max(int(640 * UI_SCALE_FAC), 400);
    const rcti rect = {0, width, 0, height};
    win = WM_window_open(C,
                         IFACE_("Recursion Tree"),
                         &rect,
                         SPACE_NODE,
                         true,
                         false,
                         true,
                         WIN_ALIGN_PARENT_CENTER,
                         nullptr,
                         nullptr);
    if (win == nullptr) {
      BKE_report(op->reports, RPT_ERROR, "Failed to open the recursion tree window");
      return OPERATOR_CANCELLED;
    }
  }

  ScrArea *area = nullptr;
  bScreen *screen = WM_window_get_active_screen(win);
  if (screen != nullptr) {
    area = screen->areabase.first();
  }
  if (area == nullptr || area->spacetype != SPACE_NODE) {
    area = recursion_tree_area(*win);
  }
  if (area == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Failed to open the recursion tree window");
    return OPERATOR_CANCELLED;
  }
  SpaceNode *graph = area->spacedata.first_as<SpaceNode>();
  if (graph->runtime == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Failed to open the recursion tree window");
    return OPERATOR_CANCELLED;
  }
  graph->runtime->recursion_tree_window = true;
  graph->runtime->recursion_tree_uid = tree_uid;
  STRNCPY_UTF8(graph->tree_idname, source->tree_idname);
  /* Pin the tree, otherwise the context sync replaces it with the active object's modifier tree
   * and the window can no longer find the editor it belongs to. */
  graph->flag |= SNODE_PIN;
  ARegion *window_region = BKE_area_find_region_type(area, RGN_TYPE_WINDOW);
  ED_node_tree_start(window_region, graph, source->edittree, source->id, source->from);
  graph->runtime->recursion_tree_window = true;
  graph->runtime->recursion_tree_uid = tree_uid;
  graph->runtime->recursion_tree_framed_points = -1;

  recursion_tree_hide_chrome(C, area);
  recursion_tree_strip_edit_keys(wm, window_region);
  /* The popup can be initialized before its Node Editor region is fully active. Keep the normal
   * UI region handler so depth buttons receive clicks after editing keymaps are removed. */
  ui::region_handlers_add(&window_region->runtime->handlers);
  ED_area_tag_redraw(area);
  WM_window_title_set(win, IFACE_("Recursion Tree"));
  WM_window_set_topmost(win, true);
  return OPERATOR_FINISHED;
}

static bool recursion_tree_poll(bContext *C)
{
  const SpaceNode *snode = CTX_wm_space_node(C);
  if (snode == nullptr || snode->edittree == nullptr) {
    return false;
  }
  if (snode->runtime && snode->runtime->recursion_tree_window) {
    return false;
  }
  const bNodeTree &tree = *snode->edittree;
  return tree.type == NTREE_GEOMETRY && (tree.flag & NTREE_GEOMETRY_RECURSIVE) != 0 &&
         (tree.id.flag & ID_FLAG_EMBEDDED_DATA) == 0;
}

void NODE_OT_recursion_tree(wmOperatorType *ot)
{
  ot->name = "Recursion Tree";
  ot->idname = "NODE_OT_recursion_tree";
  ot->description =
      "Open a window with the recursive call structure. Click a depth to inspect that call";
  ot->exec = recursion_tree_exec;
  ot->poll = recursion_tree_poll;
  ot->flag = OPTYPE_REGISTER;
}

}  // namespace blender::ed::space_node
