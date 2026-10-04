/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup python
 *
 * Geometry / Geometry Nodes helpers for embedding Python evaluation.
 */

#pragma once

#include "BLI_string_ref.hh"

#include "BKE_geometry_set.hh"

namespace blender {

struct bContext;
struct bNode;
struct bNodeTree;

/**
 * Run a Geometry Nodes Python node script with injected locals:
 * - `geometry` / `geo`: #bpy.types.GeometrySet wrapping \a geometry_io (same object)
 * - `node`: RNA for the evaluating node
 * - `tree`: RNA for the node tree (`node.id_data`)
 *
 * On success, \a geometry_io is replaced with the GeometrySet held by the
 * `geometry` local (or left empty if that name was rebound to a non-GeometrySet).
 *
 * GIL is acquired internally. \a C may be null.
 *
 * \return false if the script raised (traceback printed); geometry may still be updated.
 */
bool BPY_run_geometry_node_script(bContext *C,
                                  StringRefNull script,
                                  bke::GeometrySet &geometry_io,
                                  bNodeTree *ntree,
                                  bNode *node);

}  // namespace blender
