/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_index_range.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include "COM_context.hh"
#include "COM_node_operation.hh"

struct bNodeTree;

namespace blender::compositor {

/* Returns an instance of a new UndefinedNodeOperation for the given node. See the class for more
 * information, */
NodeOperation *get_undefined_node_operation(Context &context, const bNode &node);

/**
 * Pass each available output through from the input with the same socket name (when present).
 * Used for Geometry Repeat/Simulation zone boundary nodes in Image Process so evaluation does
 * not crash and color data can flow (with multi-iteration / sim cache support).
 */
NodeOperation *get_pass_through_matching_sockets_operation(Context &context, const bNode &node);

/** Image Process Repeat zone: current iteration index for multi-pass evaluation. */
void image_process_set_repeat_iteration(int iteration);
int image_process_get_repeat_iteration();

/**
 * Drop all Image Process Repeat / Simulation zone GPU caches.
 * Must run before #GPU_exit — static maps hold Result textures that free Vulkan images.
 */
void image_process_clear_zone_caches();

/**
 * Image Process Bake Image: when false (normal cook), Bake Image does not write files and is not
 * scheduled as an output. Set true only while the Bake operator runs.
 */
void image_process_set_bake_write_enabled(bool enabled);
bool image_process_bake_write_enabled();

/**
 * Image Process Paint: force-schedule a Paint node so Color input cooks without Viewer.
 * Pass node_identifier 0 / clear() to disable.
 */
void image_process_set_paint_force_schedule(uint tree_session_uid, int node_identifier);
void image_process_clear_paint_force_schedule();
bool image_process_paint_force_schedule_active();
uint image_process_paint_force_tree_session_uid();
int image_process_paint_force_node_identifier();

/**
 * Image Process multi-frame fill start. Simulation uses single-slot feedback only (always
 * returns `target_frame` for sim). Fluid may still request a short fill window.
 */
int image_process_sim_resume_frame(const bNodeTree &ntree, int target_frame);

/**
 * Invalidate multi-frame fluid history after `keep_frame`. Simulation has no dense history.
 */
void image_process_sim_invalidate_after(const bNodeTree &ntree, int keep_frame);

/**
 * Contiguous frame ranges held in Fluid zone caches (Simulation no longer reports multi-frame).
 * Used to draw the purple timeline cache strip (TH_SIMULATED_FRAMES).
 */
Vector<IndexRange> image_process_sim_cached_frame_ranges();

/** Fluid checkpoint frames only. Drawn as brighter ticks. */
Vector<IndexRange> image_process_sim_checkpoint_frame_ranges();

/** Set during multi-frame scrub fill so Fluid zones route to preview_interval. */
void image_process_set_fluid_scrub_fill(bool active);
bool image_process_get_fluid_scrub_fill();

/**
 * Image Process partial cook: only schedule these terminal node identifiers (Viewer / File Output
 * / Bake). Empty filter = cook all demand roots (full dirty edit). Used so frame updates on a
 * Fluid branch do not re-evaluate independent Image Output modules.
 */
void image_process_set_terminal_filter(Span<int> node_identifiers);
void image_process_clear_terminal_filter();
bool image_process_terminal_filter_active();
bool image_process_terminal_filter_contains(int node_identifier);
/** Current partial-cook terminal identifiers (empty when filter inactive). */
Span<int> image_process_terminal_filter_get();

/** Call once at the start of each Image Process cook (clears stale sim input snapshots). */
void image_process_prepare_cook();

/** Root Image Process tree session_uid for this cook. Fallback when node.owner_tree is null. */
void image_process_set_cook_tree_session_uid(uint session_uid);
uint image_process_cook_tree_session_uid();

/** Wipe Image Process Simulation feedback for one zone (by Output node id). */
void image_process_reset_sim_zone(uint session_uid, int32_t output_node_id);

}  // namespace blender::compositor
