/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 *
 * Draws Geometry Nodes Guide Geometry as a viewport overlay.
 * Only uses copied float/index buffers from the eval log. Never reads Mesh* / GeometrySet.
 */

#include "BLI_math_matrix.hh"

#include "BKE_main.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_node_types.h"
#include "DNA_windowmanager_types.h"

#include "GPU_batch.hh"
#include "GPU_index_buffer.hh"
#include "GPU_init_exit.hh"
#include "GPU_vertex_buffer.hh"
#include "GPU_vertex_format.hh"

#include "NOD_eval_log.hh"

#include "overlay_guide_geometry.hh"

#include <cstdio>

namespace blender::draw::overlay {

GuideGeometry::~GuideGeometry()
{
  owned_batches_.clear();
}

void GuideGeometry::begin_sync(Resources &res, const State &state)
{
  occluded_ps_.init();
  xray_ps_.init();
  this->discard_batches();

  enabled_ = state.is_space_v3d() && !res.is_selection() && !state.is_image_render &&
             !state.is_viewport_image_render && !state.is_depth_only_drawing && !state.hide_overlays;
  if (!enabled_) {
    return;
  }

  auto init_pass = [&](PassSimple &pass, const DRWState draw_state) {
    pass.state_set(draw_state, state.clipping_plane_count);
    pass.shader_set(res.shaders->uniform_color.get());
    pass.bind_ubo(OVERLAY_GLOBALS_SLOT, &res.globals_buf);
    pass.bind_ubo(DRW_CLIPPING_UBO_SLOT, &res.clip_planes_buf);
  };

  init_pass(occluded_ps_,
            DRW_STATE_WRITE_COLOR | DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_BLEND_ALPHA);
  init_pass(xray_ps_, DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA);
}

static gpu::VertBuf *positions_vbo(const Span<float3> positions)
{
  if (positions.is_empty()) {
    return nullptr;
  }
  GPUVertFormat format = {0};
  GPU_vertformat_attr_add(&format, "pos", gpu::VertAttrType::SFLOAT_32_32_32);
  gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
  if (!vbo) {
    return nullptr;
  }
  GPU_vertbuf_data_alloc(*vbo, uint(positions.size()));
  vbo->data<float3>().copy_from(positions);
  return vbo;
}

static gpu::Batch *make_batch(GPUPrimType prim, const Span<float3> positions, gpu::IndexBuf *ibo)
{
  gpu::VertBuf *vbo = positions_vbo(positions);
  if (!vbo) {
    if (ibo) {
      GPU_INDEXBUF_DISCARD_SAFE(ibo);
    }
    return nullptr;
  }
  const GPUBatchFlag flag = ibo ? GPUBatchFlag(GPU_BATCH_OWNS_VBO | GPU_BATCH_OWNS_INDEX) :
                                  GPU_BATCH_OWNS_VBO;
  return GPU_batch_create_ex(prim, vbo, ibo, flag);
}

void GuideGeometry::discard_batches()
{
  if (!GPU_is_init()) {
    owned_batches_.clear();
    return;
  }
  for (gpu::Batch *&batch : owned_batches_) {
    GPU_BATCH_DISCARD_SAFE(batch);
  }
  owned_batches_.clear();
}

static void submit_batch(Vector<gpu::Batch *> &owned,
                         PassSimple &occluded,
                         PassSimple &xray,
                         Manager &manager,
                         gpu::Batch *batch,
                         const float4x4 &transform,
                         const float4 &color,
                         const bool use_xray)
{
  if (!batch) {
    return;
  }
  if (batch->verts[0] == nullptr) {
    GPU_BATCH_DISCARD_SAFE(batch);
    return;
  }
  owned.append(batch);
  PassSimple::Sub &sub = (use_xray ? xray : occluded).sub("guide_v5");
  sub.push_constant("ucolor", color);
  sub.draw(batch, ResourceHandleRange(manager.resource_handle(transform)));
}

void GuideGeometry::object_sync(Manager &manager,
                                const ObjectRef &ob_ref,
                                Resources & /*res*/,
                                const State &state)
{
  if (!enabled_) {
    return;
  }
  Main *bmain = DEG_get_bmain(state.depsgraph);
  if (!bmain) {
    return;
  }
  wmWindowManager *wm = static_cast<wmWindowManager *>(bmain->wm.first());
  if (!wm) {
    return;
  }

  nodes::eval_log::foreach_visible_guide_geometry(
      *ob_ref.object, *wm, [&](const nodes::eval_log::GuideGeometryNodeLog &guide) {
        if (!guide.is_ready()) {
          return;
        }
        const float4x4 &transform = ob_ref.object->object_to_world();
        const bool draw_solid = ELEM(guide.display_mode,
                                     GEO_NODE_GUIDE_DISPLAY_SOLID,
                                     GEO_NODE_GUIDE_DISPLAY_WIRE_AND_SOLID);
        const bool draw_wire = ELEM(guide.display_mode,
                                    GEO_NODE_GUIDE_DISPLAY_WIRE,
                                    GEO_NODE_GUIDE_DISPLAY_WIRE_AND_SOLID);

        if (draw_solid && !guide.mesh_tris.is_empty() && !guide.mesh_positions.is_empty()) {
          const uint vert_len = uint(guide.mesh_positions.size());
          int valid_tris = 0;
          for (const int3 tri : guide.mesh_tris) {
            if (uint(tri[0]) < vert_len && uint(tri[1]) < vert_len && uint(tri[2]) < vert_len) {
              valid_tris++;
            }
          }
          if (valid_tris > 0) {
            GPUIndexBufBuilder ibo_builder;
            GPU_indexbuf_init(&ibo_builder, GPU_PRIM_TRIS, uint(valid_tris), vert_len);
            for (const int3 tri : guide.mesh_tris) {
              if (uint(tri[0]) < vert_len && uint(tri[1]) < vert_len && uint(tri[2]) < vert_len)
              {
                GPU_indexbuf_add_tri_verts(&ibo_builder, uint(tri[0]), uint(tri[1]), uint(tri[2]));
              }
            }
            float4 solid_color = guide.color;
            solid_color.w *= 0.28f;
            submit_batch(owned_batches_,
                         occluded_ps_,
                         xray_ps_,
                         manager,
                         make_batch(GPU_PRIM_TRIS,
                                    guide.mesh_positions,
                                    GPU_indexbuf_build(&ibo_builder)),
                         transform,
                         solid_color,
                         guide.xray);
          }
        }

        if (draw_wire && !guide.mesh_positions.is_empty()) {
          const uint vert_len = uint(guide.mesh_positions.size());
          int valid_edges = 0;
          for (const int2 edge : guide.mesh_edges) {
            if (uint(edge[0]) < vert_len && uint(edge[1]) < vert_len) {
              valid_edges++;
            }
          }
          if (valid_edges == 0) {
            submit_batch(owned_batches_,
                         occluded_ps_,
                         xray_ps_,
                         manager,
                         make_batch(GPU_PRIM_POINTS, guide.mesh_positions, nullptr),
                         transform,
                         guide.color,
                         guide.xray);
          }
          else {
            GPUIndexBufBuilder ibo_builder;
            GPU_indexbuf_init(&ibo_builder, GPU_PRIM_LINES, uint(valid_edges), vert_len);
            for (const int2 edge : guide.mesh_edges) {
              if (uint(edge[0]) < vert_len && uint(edge[1]) < vert_len) {
                GPU_indexbuf_add_line_verts(&ibo_builder, uint(edge[0]), uint(edge[1]));
              }
            }
            submit_batch(owned_batches_,
                         occluded_ps_,
                         xray_ps_,
                         manager,
                         make_batch(GPU_PRIM_LINES,
                                    guide.mesh_positions,
                                    GPU_indexbuf_build(&ibo_builder)),
                         transform,
                         guide.color,
                         guide.xray);
          }
        }

        if (!guide.curve_positions.is_empty() && guide.curve_offsets.size() >= 2) {
          const int pos_num = int(guide.curve_positions.size());
          const int curve_num = int(guide.curve_offsets.size()) - 1;
          int segment_num = 0;
          for (const int curve_i : IndexRange(curve_num)) {
            const int start = guide.curve_offsets[curve_i];
            const int end = guide.curve_offsets[curve_i + 1];
            if (start < 0 || end > pos_num || end - start < 2) {
              continue;
            }
            segment_num += end - start - 1;
            if (curve_i < guide.curve_cyclic.size() && guide.curve_cyclic[curve_i]) {
              segment_num += 1;
            }
          }
          if (segment_num == 0) {
            submit_batch(owned_batches_,
                         occluded_ps_,
                         xray_ps_,
                         manager,
                         make_batch(GPU_PRIM_POINTS, guide.curve_positions, nullptr),
                         transform,
                         guide.color,
                         guide.xray);
          }
          else {
            GPUIndexBufBuilder ibo_builder;
            GPU_indexbuf_init(&ibo_builder, GPU_PRIM_LINES, uint(segment_num), uint(pos_num));
            for (const int curve_i : IndexRange(curve_num)) {
              const int start = guide.curve_offsets[curve_i];
              const int end = guide.curve_offsets[curve_i + 1];
              if (start < 0 || end > pos_num || end - start < 2) {
                continue;
              }
              for (int i = start; i < end - 1; i++) {
                GPU_indexbuf_add_line_verts(&ibo_builder, uint(i), uint(i + 1));
              }
              if (curve_i < guide.curve_cyclic.size() && guide.curve_cyclic[curve_i]) {
                GPU_indexbuf_add_line_verts(&ibo_builder, uint(end - 1), uint(start));
              }
            }
            submit_batch(owned_batches_,
                         occluded_ps_,
                         xray_ps_,
                         manager,
                         make_batch(GPU_PRIM_LINES,
                                    guide.curve_positions,
                                    GPU_indexbuf_build(&ibo_builder)),
                         transform,
                         guide.color,
                         guide.xray);
          }
        }

        if (!guide.point_positions.is_empty()) {
          submit_batch(owned_batches_,
                       occluded_ps_,
                       xray_ps_,
                       manager,
                       make_batch(GPU_PRIM_POINTS, guide.point_positions, nullptr),
                       transform,
                       guide.color,
                       guide.xray);
        }
      });
}

void GuideGeometry::draw_line(Framebuffer &framebuffer, Manager &manager, View &view)
{
  if (!enabled_) {
    return;
  }
  GPU_framebuffer_bind(framebuffer);
  manager.submit(occluded_ps_, view);
}

void GuideGeometry::draw_color_only(Framebuffer &framebuffer, Manager &manager, View &view)
{
  if (!enabled_) {
    return;
  }
  GPU_framebuffer_bind(framebuffer);
  manager.submit(xray_ps_, view);
}

}  // namespace blender::draw::overlay
