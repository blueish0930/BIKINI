/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* IMAGE_NODES_MVP: entire header — GPU image node tree (Compositor/Copernicus-style).
 * Grep IMAGE_NODES_MVP to find all related patches for cleanup/revert. */

/** \file
 * \ingroup nodes
 */

#pragma once

#include "BLI_index_range.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

#include "DNA_object_enums.h"
#include "DNA_view3d_enums.h"

#include "IMB_imbuf_types.hh"

#include "BKE_node.hh"

struct bContext;

namespace blender {

struct Main;
struct Scene;
struct bNodeTree;
struct Depsgraph;
struct GPUOffScreen;
struct GPUViewport;
struct ImBuf;
struct Material;
struct Object;
struct View3DShading;
namespace gpu {
class Texture;
}  // namespace gpu

/* IMAGE_NODES_MVP begin */
extern bke::bNodeTreeType *ntreeType_Image;

/**
 * Offscreen 3D view render for Image Process Camera View (same signature as
 * #ED_view3d_draw_offscreen_imbuf_simple). Set by the node editor space type so
 * `bf_nodes_image` does not link editors (mirrors `seq::view3d_fn`).
 * Null in background / before space registration.
 */
using ImageProcessDrawViewFn = ImBuf *(*)(Depsgraph *depsgraph,
                                          Scene *scene,
                                          View3DShading *shading_override,
                                          eDrawType drawtype,
                                          Object *camera,
                                          int width,
                                          int height,
                                          ImBufFlags imbuf_flags,
                                          eV3DOffscreenDrawFlag draw_flags,
                                          int alpha_mode,
                                          const char *viewname,
                                          GPUOffScreen *ofs,
                                          GPUViewport *viewport,
                                          char err_out[256]);
extern ImageProcessDrawViewFn image_process_view3d_fn;

/**
 * GPU-primary camera preview used by Image Process Camera View.
 * Reuses offscreen + #GPUViewport like the 3D Viewport (no create/destroy every cook) and
 * returns a *borrowed* color texture (valid until the next call with a different size / destroy).
 * Prefer this over #image_process_view3d_fn to avoid GPU→CPU→GPU readback stalls.
 */
struct ImageProcessCameraViewGPUOut {
  gpu::Texture *color = nullptr;
  int2 size{0, 0};
};

using ImageProcessCameraViewRenderGPUFn = bool (*)(Depsgraph *depsgraph,
                                                   Scene *scene,
                                                   eDrawType drawtype,
                                                   Object *camera,
                                                   int width,
                                                   int height,
                                                   eV3DOffscreenDrawFlag draw_flags,
                                                   int alpha_mode,
                                                   ImageProcessCameraViewGPUOut *r_out,
                                                   char err_out[256]);
extern ImageProcessCameraViewRenderGPUFn image_process_camera_view_render_gpu_fn;

/**
 * Offscreen render of a UV-mapped plane with the given material (Image Process Render Material).
 * Set by the node editor space type so `bf_nodes_image` does not link editors.
 * Pass a temporary GPUViewport (nullptr ofs/viewport) — do not cache GPUViewport across exit.
 */
using ImageProcessRenderMaterialFn = ImBuf *(*)(Main *bmain,
                                                Scene *scene,
                                                Material *material,
                                                int width,
                                                int height,
                                                char err_out[256]);
extern ImageProcessRenderMaterialFn image_process_render_material_fn;

/** Free the Render Material scratch scene. Called from #image_process_clear_session_caches. */
using ImageProcessRenderMaterialFreeFn = void (*)();
extern ImageProcessRenderMaterialFreeFn image_process_render_material_free_fn;

/**
 * Free Image Process session caches that own #GeometrySet / Mesh GPU batches.
 * Must be called before #GPU_exit (same reason as #select_elements_free_all_caches).
 */
void image_process_clear_session_caches();

namespace nodes::node_image_shadertoy_cc {
void free_session_caches();
}

void register_node_tree_type_img();
void register_image_nodes();
/** Install Image Process COM for NodeCombineBundle (after geometry nodes register the type). */
void register_image_combine_bundle_compositor();
/** Defaults for Python-registered custom Image Process group types. */
void register_node_type_img_custom_group(bke::bNodeType *ntype);

/**
 * Evaluate an ImageNodeTree with the compositor GPU backend (CPU fallback if GPU unavailable).
 * Viewer output is written to the composite "Viewer Node" image (same as the compositor) so the
 * node editor can display it as a backdrop.
 * \param resolution Compositing domain size (texture resolution); defaults to 1024×1024.
 * \return true on success.
 */
bool ntreeImageNodesEvaluate(Main &bmain,
                             Scene &scene,
                             bNodeTree &ntree,
                             bool *r_used_gpu,
                             int2 resolution = int2(1024, 1024));

/**
 * Evaluate every ImageNodeTree (GPU Texture Editor) in #bmain.
 * Call after file load so Image Process trees are re-evaluated (runtime GPU/CPU cache is not
 * saved in the blend file).
 */
void ntreeImageNodesEvaluateAll(Main &bmain, Scene *scene = nullptr, int2 resolution = int2(1024, 1024));

/** Seed a new empty ImageNodeTree with Color → Viewer (default graph). */
void node_tree_image_default_init(const bContext *C, bNodeTree *ntree);

/** Register load-post hook that recooks all Image Process trees after opening a blend. */
void ntreeImageNodesRegisterLoadHooks();

/**
 * Contiguous frame ranges held by Image Process Simulation + Fluid runtime caches.
 * Drawn as the purple timeline cache strip.
 */
namespace nodes {
Vector<IndexRange> image_process_cached_frame_ranges();
Vector<IndexRange> image_process_checkpoint_frame_ranges();
}  // namespace nodes
/* IMAGE_NODES_MVP end */

}  // namespace blender
