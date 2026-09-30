/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup nodes
 *
 * Image Process Paint: Color input as base texture; paint on internal canvas Image.
 */

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

struct Image;
struct Main;
struct ReportList;
struct bNode;
struct bNodeTree;

namespace blender::nodes {

/** `node.custom1` int16 flags. */
enum NodeImagePaintFlag : int {
  NODE_IMAGE_PAINT_USE_CANVAS = (1 << 0),
};

bool is_paint_node(const bNode &node);

Image *image_paint_ensure_image(Main &bmain,
                                bNode &node,
                                const int2 resolution,
                                ReportList *reports);

void image_paint_cache_input(const bNodeTree &ntree,
                             int node_identifier,
                             Span<float> rgba_float4,
                             int2 size);

bool image_paint_get_cached_input(const bNodeTree &ntree,
                                  int node_identifier,
                                  Vector<float> &r_rgba,
                                  int2 &r_size);

bool image_paint_write_canvas(Image &image, Span<float> rgba_float4, int2 size);

void image_paint_set_use_canvas(bNode &node, bool enable);
bool image_paint_uses_canvas(const bNode &node);

}  // namespace blender::nodes
