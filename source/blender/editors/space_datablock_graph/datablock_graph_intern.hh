/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_function_ref.hh"
#include "BLI_map.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_rect.hh"
#include "BLI_set.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "DNA_space_types.h"

#include "BKE_lib_query.hh"

struct ARegion;
struct Main;
struct bContext;
struct wmKeyConfig;
struct wmWindowManager;

namespace blender::ed::datablock_graph {

/**
 * Visual node kinds. Real ID data-blocks vs. non-ID intermediates (e.g. modifiers).
 * Modifiers are not IDs but sit between Object and Geometry Node trees in the user chain.
 */
enum class GraphNodeKind : int8_t {
  ID = 0,
  /** Geometry Nodes (or other) modifier instance on an Object — not a Main ID. */
  VirtualModifier = 1,
  /**
   * Editor space holding a USER_ONE ref (Image Editor, Node Editor, …).
   * Screens are filtered as noise in the Main walk, so these are injected manually.
   */
  VirtualUISpace = 2,
};

/**
 * One visual node instance inside a seed's independent ref-graph.
 * The same ID may appear as separate nodes under different seeds.
 */
struct GraphNode {
  ID *id = nullptr;
  /** ID.session_uid of the data-block this node shows (or synthetic for virtual nodes). */
  uint id_session_uid = 0;
  /** ID.session_uid of the seed that owns this independent tree. */
  uint seed_uid = 0;
  /** Left edge of box; y is vertical center. */
  float2 location = float2(0.0f);
  float2 size = float2(180.0f, 66.0f);
  bool is_seed = false;
  short depth_from_seed = 0;
  bool selected = false;
  bool position_locked = false;

  GraphNodeKind kind = GraphNodeKind::ID;
  /**
   * For virtual nodes: host Object (modifier) or Screen (UI space), if any.
   * Not used for selection keys (those use #id_session_uid).
   */
  ID *host_id = nullptr;
  /** Display name (modifier name / editor title for virtual nodes). */
  char display_name[64] = {};
  /** #ModifierType for virtual modifier nodes. */
  short modifier_type = 0;
  /** #eSpace_Type for virtual UI space nodes. */
  short space_type = 0;
};

struct GraphEdge {
  /** Indices into #SpaceDataBlockGraph_Runtime::nodes (stable within a rebuild). */
  int from_index = -1;
  int to_index = -1;
  uint seed_uid = 0;
  LibraryForeachIDCallbackFlag usage_flags = IDWALK_CB_NOP;
  bool is_cycle = false;
  /**
   * How many ID-pointer links this edge aggregates (e.g. same node group used 3× in a tree).
   * Always >= 1 for a live edge.
   */
  int usage_count = 1;
};

/** Force re-walk Main relations and rebuild visible graphs (keeps locked node positions). */
void datablock_graph_force_refresh(SpaceDataBlockGraph &sdbg, Main &bmain);

inline uint64_t datablock_graph_node_key(const uint seed_uid, const uint id_session_uid)
{
  return (uint64_t(seed_uid) << 32) | uint64_t(id_session_uid);
}

/**
 * One editor-local undo snapshot of graph DNA (seeds + node positions).
 * Global memfile undo often skips space-only Screen diffs; this stack is reliable for Ctrl+Z.
 */
struct GraphUndoSnapshot {
  Vector<uint> seed_session_uids;
  struct Pos {
    uint seed_session_uid = 0;
    uint id_session_uid = 0;
    float loc[2] = {};
    char flag = 0;
  };
  Vector<Pos> positions;
  /** Selected node keys (seed_uid<<32 | id_session_uid). */
  Vector<uint64_t> selected_keys;
  uint64_t active_key = 0;
};

struct SpaceDataBlockGraph_Runtime {
  Vector<GraphNode> nodes;
  Vector<GraphEdge> edges;
  /** (seed_uid, id_session_uid) -> node index. */
  Map<uint64_t, int> key_to_index;
  bool graph_dirty = true;
  bool layout_dirty = true;
  bool frame_view = false;
  /** Active node key, or 0. */
  uint64_t active_key = 0;
  int8_t last_built_max_depth = -1;
  uint32_t last_built_flag = 0;

  /** Editor-local undo/redo (in-session). */
  Vector<GraphUndoSnapshot> undo_stack;
  Vector<GraphUndoSnapshot> redo_stack;
};

void datablock_graph_runtime_free(SpaceDataBlockGraph &sdbg);
void datablock_graph_ensure_runtime(SpaceDataBlockGraph &sdbg);

/** Free DNA-stored node positions and browser collapse paths. */
void datablock_graph_free_persistent_state(SpaceDataBlockGraph &sdbg);
/** Write current runtime node locations into DNA (for .blend save). */
void datablock_graph_store_positions(SpaceDataBlockGraph &sdbg);
/** Apply DNA node locations onto runtime (after rebuild / file load). */
void datablock_graph_apply_positions(SpaceDataBlockGraph &sdbg);

/** Capture current DNA state onto the local undo stack (call before mutating). */
void datablock_graph_undo_push(SpaceDataBlockGraph &sdbg);
/** Restore previous local snapshot. \return false if nothing to undo. */
bool datablock_graph_undo_step(SpaceDataBlockGraph &sdbg, Main &bmain);
/** Restore next local snapshot. \return false if nothing to redo. */
bool datablock_graph_redo_step(SpaceDataBlockGraph &sdbg, Main &bmain);
bool datablock_graph_undo_can_undo(const SpaceDataBlockGraph &sdbg);
bool datablock_graph_undo_can_redo(const SpaceDataBlockGraph &sdbg);

bool datablock_graph_browser_path_collapsed(const SpaceDataBlockGraph &sdbg, StringRef path);
void datablock_graph_browser_path_set_collapsed(SpaceDataBlockGraph &sdbg,
                                               StringRef path,
                                               bool collapsed);

bool datablock_graph_has_seed(const SpaceDataBlockGraph &sdbg, const ID *id);

/**
 * Double-click / drop entry point.
 * - If \a id is already a seed: select (highlight) that seed's entire tree.
 * - Else: create a brand-new independent ref-graph for \a id, laid out outside existing nodes.
 * \a drop_view_co: optional view-space position (drop release); used when graph is empty or as
 * vertical guide for placement.
 */
void datablock_graph_activate_or_create(SpaceDataBlockGraph &sdbg,
                                        Main &bmain,
                                        ID *id,
                                        const float2 *drop_view_co);

void datablock_graph_clear_seeds(SpaceDataBlockGraph &sdbg);

/**
 * Remove editor graphs that contain any selected node (drops those seeds from the space).
 * Does not delete the underlying data-blocks in #Main.
 * \return number of seed trees removed.
 */
int datablock_graph_delete_selected(SpaceDataBlockGraph &sdbg);

void datablock_graph_tag_rebuild(SpaceDataBlockGraph &sdbg);
void datablock_graph_rebuild_if_needed(SpaceDataBlockGraph &sdbg, Main &bmain);
void datablock_graph_layout(SpaceDataBlockGraph &sdbg, bool force_all = false);
void datablock_graph_update_view2d(ARegion &region, SpaceDataBlockGraph &sdbg, bool frame_all);

rctf datablock_graph_node_rect(const GraphNode &node);
GraphNode *datablock_graph_find_node_at(SpaceDataBlockGraph &sdbg, float2 view_co);
GraphNode *datablock_graph_find_node_mut(SpaceDataBlockGraph &sdbg,
                                         uint seed_uid,
                                         uint id_session_uid);
const GraphNode *datablock_graph_find_node(const SpaceDataBlockGraph &sdbg,
                                           uint seed_uid,
                                           uint id_session_uid);

void datablock_graph_deselect_all(SpaceDataBlockGraph &sdbg);
void datablock_graph_select_all(SpaceDataBlockGraph &sdbg);
void datablock_graph_select_seed_graph(SpaceDataBlockGraph &sdbg, uint seed_uid);
/**
 * Select the full connected component of \a node within its seed tree
 * (all nodes reachable via edges, undirected). Sets active to \a node.
 */
void datablock_graph_select_connected_tree(SpaceDataBlockGraph &sdbg, GraphNode &node);
int datablock_graph_count_selected(const SpaceDataBlockGraph &sdbg);
void datablock_graph_foreach_selected(SpaceDataBlockGraph &sdbg,
                                      FunctionRef<void(GraphNode &)> fn);

bool datablock_graph_nodes_bounds(const SpaceDataBlockGraph &sdbg, rctf &r_bounds);

void datablock_graph_draw_main(const bContext *C, ARegion *region);
void datablock_graph_draw_browser(const bContext *C, ARegion *region);
void datablock_graph_browser_panels_register(ARegionType &region_type);

void datablock_graph_operatortypes();
void datablock_graph_keymap(wmKeyConfig *keyconf);
void datablock_graph_ensure_region_keymap(wmWindowManager *wm, ARegion *region);
void datablock_graph_dropboxes();

bool datablock_graph_align_selection(SpaceDataBlockGraph &sdbg, bool horizontal);

}  // namespace blender::ed::datablock_graph
