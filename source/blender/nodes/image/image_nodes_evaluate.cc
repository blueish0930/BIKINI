/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* IMAGE_NODES_MVP: GPU-primary evaluation of ImageNodeTree via compositor backend. */

/** \file
 * \ingroup nodes
 */

#include <cmath>
#include <cstring>
#include <memory>

#include "MEM_guardedalloc.h"

#include "BLI_listbase.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_threads.hh"
#include "BLI_vector.hh"

#include "DNA_ID.h"
#include "DNA_node_types.h"
#include "DNA_scene_types.h"

#include "BKE_callbacks.hh"
#include "BKE_compute_contexts.hh"
#include "BKE_context.hh"
#include "BKE_global.hh"
#include "BKE_image.hh"
#include "BKE_main.hh"
#include "BKE_main_invariants.hh"
#include "BKE_node.hh"
#include "BKE_node_enum.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_scene.hh"

#include "DRW_engine.hh"

#include "GPU_context.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"
#include "GPU_texture_pool.hh"

#include "IMB_colormanagement.hh"
#include "IMB_imbuf.hh"
#include "IMB_partial_update.hh"

#include "COM_context.hh"
#include "COM_domain.hh"
#include "COM_input_descriptor.hh"
#include "COM_node_group_operation.hh"
#include "COM_realize_on_domain_operation.hh"
#include "COM_result.hh"
#include "COM_simple_operation.hh"
#include "COM_utilities.hh"
#include "COM_static_cache_manager.hh"
#include "COM_undefined_node_operation.hh"

#include "NOD_eval_log.hh"
#include "NOD_image.hh"
#include "NOD_image_points.hh"

namespace blender {

/* Offscreen camera render hooks; assigned in ED_spacetype_node (editors). */
ImageProcessDrawViewFn image_process_view3d_fn = nullptr;
ImageProcessCameraViewRenderGPUFn image_process_camera_view_render_gpu_fn = nullptr;
ImageProcessRenderMaterialFn image_process_render_material_fn = nullptr;
ImageProcessRenderMaterialFreeFn image_process_render_material_free_fn = nullptr;

void image_process_clear_session_caches()
{
  nodes::node_image_shadertoy_cc::free_session_caches();
  nodes::image_points::clear_session_geometry_caches();
  if (image_process_render_material_free_fn) {
    image_process_render_material_free_fn();
  }
}

namespace {

using namespace compositor;

/* IMAGE_NODES_MVP: evaluation context that prefers GPU and captures viewer pixels. */
class ImageNodesContext : public Context {
 private:
  Main &bmain_;
  Scene &scene_;
  bNodeTree &node_tree_;
  int2 resolution_;
  std::optional<ComputeContextHash> active_compute_context_hash_;
  bool gpu_supported_ = true;
  Vector<std::unique_ptr<Result>> owned_inputs_;
  nodes::eval_log::NodesEvalLog *eval_log_ = nullptr;

 public:
  ImageNodesContext(StaticCacheManager &cache_manager,
                    Main &bmain,
                    Scene &scene,
                    bNodeTree &node_tree,
                    const int2 resolution,
                    nodes::eval_log::NodesEvalLog *eval_log)
      : Context(cache_manager),
        bmain_(bmain),
        scene_(scene),
        node_tree_(node_tree),
        resolution_(math::max(resolution, int2(1))),
        eval_log_(eval_log)
  {
    const bke::DataBlockComputeContext base_context(nullptr, scene.id);
    active_compute_context_hash_ = base_context.hash();
  }

  nodes::eval_log::NodesEvalLog *nodes_evaluation_log() const override
  {
    return eval_log_;
  }

  const Main &get_main() const override
  {
    return bmain_;
  }

  const Scene &get_scene() const override
  {
    return scene_;
  }

  Domain get_compositing_domain() const override
  {
    return Domain(resolution_);
  }

  void set_gpu_supported(const bool supported)
  {
    gpu_supported_ = supported;
  }

  bool use_gpu() const override
  {
    /* GPU-primary path, but never report GPU if the context was dropped (TexturePool::get()
     * would then dereference a null GPUContext at offset ~0x230). */
    return gpu_supported_ && GPU_context_active_get() != nullptr;
  }

  bool is_image_process() const override
  {
    return true;
  }

  SideEffectOutputTypes needed_side_effect_output_types() const override
  {
    return SideEffectOutputTypes::ViewerNode | SideEffectOutputTypes::FileOutputNode |
           SideEffectOutputTypes::NodePreviews;
  }

  const std::optional<ComputeContextHash> &get_viewer_compute_context_hash() const override
  {
    return active_compute_context_hash_;
  }

  ResultPrecision get_precision() const override
  {
    return ResultPrecision::Full;
  }

  void write_viewer(Result &viewer_result) override
  {
    /* Same Viewer image as the compositor so the node editor backdrop displays results.
     *
     * Pure GPU path (GPU_texture_copy), matching render/intern/compositor.cc
     * #write_viewer_image. The old download-to-CPU + per-pixel blit + re-upload stalled the
     * UI on every Viewer cook (seconds at 1024²+). Image Output → shader without the editor
     * open felt fine because that path did not hit this hot loop every refresh.
     *
     * Scale / Rotate / Translate / Transform only mutate the domain matrix (lazy) — realize
     * onto the compositing domain before writing so those nodes affect the backdrop. */
    Result *result_to_write = &viewer_result;
    SimpleOperation *realization_operation = nullptr;
    Result realize_input = this->create_result(viewer_result.type(), viewer_result.precision());

    if (!viewer_result.is_single_value() && viewer_result.is_allocated()) {
      const InputDescriptor input_descriptor = {viewer_result.type(),
                                                InputRealizationMode::OperationDomain};
      realization_operation = RealizeOnDomainOperation::construct_if_needed(
          *this, viewer_result, input_descriptor, this->get_compositing_domain());
      if (realization_operation) {
        realize_input.share_data(viewer_result);
        realization_operation->map_input_to_result(&realize_input);
        realization_operation->evaluate();
        result_to_write = &realization_operation->get_result();
      }
    }

    Image *image = BKE_image_ensure_viewer(&bmain_, IMA_TYPE_COMPOSITE, "Viewer Node");

    /* Prefer result size after realization (matches compositor viewer). Fall back to the
     * Image Process compositing domain when the result is a single value. */
    int2 size = math::max(this->get_compositing_domain().data_size, int2(1));
    if (!result_to_write->is_single_value() && result_to_write->is_allocated()) {
      size = math::max(int2(1), result_to_write->domain().data_size);
    }

    ImageUser image_user = {nullptr};
    BLI_thread_lock(LOCK_DRAW_IMAGE);

    void *lock = nullptr;
    ImBuf *image_buffer = BKE_image_acquire_ibuf_gpu(image, &image_user, &lock);

    if (int2(image_buffer->x, image_buffer->y) != size) {
      IMB_free_byte_pixels(image_buffer);
      IMB_free_float_pixels(image_buffer);
      IMB_free_gpu_textures(image_buffer);
      image_buffer->x = size.x;
      image_buffer->y = size.y;
    }

    if (this->use_gpu()) {
      /* Drop host float buffer — backdrop draws from GPU texture only. */
      IMB_free_float_pixels(image_buffer);

      const gpu::TextureFormat format =
          (result_to_write->is_allocated() && !result_to_write->is_single_value()) ?
              result_to_write->get_gpu_texture_format() :
              gpu::TextureFormat::SFLOAT_16_16_16_16;

      if (!image_buffer->gpu.texture ||
          GPU_texture_format(image_buffer->gpu.texture) != format)
      {
        gpu::Texture *texture = GPU_texture_create_2d(
            __func__, size.x, size.y, 1, format, GPU_TEXTURE_USAGE_GENERAL, nullptr);
        IMB_assign_gpu_texture(image_buffer, texture);
      }

      if (result_to_write->is_single_value()) {
        Color clear_color(0.0f, 0.0f, 0.0f, 1.0f);
        if (result_to_write->type() == ResultType::Color) {
          clear_color = result_to_write->get_single_value_default<Color>();
        }
        else if (result_to_write->type() == ResultType::Float) {
          const float v = result_to_write->get_single_value_default<float>();
          clear_color = Color(v, v, v, 1.0f);
        }
        GPU_texture_clear(image_buffer->gpu.texture, GPU_DATA_FLOAT, &clear_color);
      }
      else if (result_to_write->is_stored_on_gpu() && result_to_write->gpu_texture() != nullptr) {
        GPU_texture_copy(image_buffer->gpu.texture, *result_to_write);
      }
      else if (result_to_write->is_allocated()) {
        /* CPU-fallback nodes: upload then copy (same as compositor viewer). */
        Result gpu_result = result_to_write->upload_to_gpu(false);
        if (gpu_result.is_stored_on_gpu() && gpu_result.gpu_texture() != nullptr) {
          GPU_texture_copy(image_buffer->gpu.texture, gpu_result);
        }
        gpu_result.release();
      }
      else {
        const float black[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        GPU_texture_clear(image_buffer->gpu.texture, GPU_DATA_FLOAT, black);
      }
      image_buffer->userflags |= IB_HOST_BUFFER_INVALID;
    }
    else {
      IMB_free_gpu_textures(image_buffer);
      IMB_free_float_pixels(image_buffer);
      IMB_alloc_float_pixels(image_buffer, 4, false);
      float *dst = image_buffer->float_data_for_write();
      const int pixel_count = size.x * size.y;
      std::memset(dst, 0, size_t(pixel_count) * 4 * sizeof(float));
      if (result_to_write->is_single_value() && result_to_write->type() == ResultType::Color) {
        const Color color = result_to_write->get_single_value_default<Color>();
        for (int i = 0; i < pixel_count; i++) {
          dst[i * 4 + 0] = color.r;
          dst[i * 4 + 1] = color.g;
          dst[i * 4 + 2] = color.b;
          dst[i * 4 + 3] = color.a;
        }
      }
      else if (result_to_write->is_allocated() && result_to_write->cpu_data().data() &&
               result_to_write->type() == ResultType::Color)
      {
        const int64_t nbytes = math::min(int64_t(pixel_count) * 4 * int64_t(sizeof(float)),
                                         result_to_write->size_in_bytes());
        std::memcpy(dst, result_to_write->cpu_data().data(), size_t(nbytes));
      }
    }

    if (result_to_write->meta_data.is_non_color_data) {
      /* Non-color / data images skip the scene view transform. */
      image->flag &= ~IMA_VIEW_AS_RENDER;
      IMB_colormanagement_assign_float_colorspace(
          image_buffer,
          IMB_colormanagement_role_colorspace_name_get(COLOR_ROLE_DATA));
    }
    else {
      image->flag |= IMA_VIEW_AS_RENDER;
      const char *to_colorspace = IMB_colormanagement_role_colorspace_name_get(
          COLOR_ROLE_SCENE_LINEAR);
      IMB_colormanagement_assign_float_colorspace(image_buffer, to_colorspace);
    }

    IMB_partial_update_mark_full(image_buffer);
    BKE_image_release_ibuf(image, image_buffer, lock);
    BLI_thread_unlock(LOCK_DRAW_IMAGE);

    if (realization_operation) {
      realization_operation->get_result().release();
      delete realization_operation;
    }
  }

  /**
   * When cooking a tree that has Group Input interfaces (e.g. editing a node group as the
   * root), map defaults from the interface socket data so Group Input → nodes work.
   * Nested groups get real linked values from GroupNodeOperation instead.
   */
  void fill_interface_input_default(Result &result, const bNodeTreeInterfaceSocket &input_socket)
  {
    using namespace blender::compositor;
    result.allocate_single_value();
    void *data = input_socket.socket_data;
    if (!data) {
      return;
    }
    const ResultType type = get_node_interface_socket_result_type(input_socket);
    switch (type) {
      case ResultType::Float:
        result.set_single_value(static_cast<bNodeSocketValueFloat *>(data)->value);
        break;
      case ResultType::Int:
        result.set_single_value(static_cast<bNodeSocketValueInt *>(data)->value);
        break;
      case ResultType::Bool:
        result.set_single_value(static_cast<bNodeSocketValueBoolean *>(data)->value != 0);
        break;
      case ResultType::Float2: {
        const float *v = static_cast<bNodeSocketValueVector *>(data)->value;
        result.set_single_value(float2(v[0], v[1]));
        break;
      }
      case ResultType::Float3: {
        const float *v = static_cast<bNodeSocketValueVector *>(data)->value;
        result.set_single_value(float3(v[0], v[1], v[2]));
        break;
      }
      case ResultType::Float4: {
        const float *v = static_cast<bNodeSocketValueVector *>(data)->value;
        result.set_single_value(float4(v[0], v[1], v[2], v[3]));
        break;
      }
      case ResultType::Color: {
        const float *v = static_cast<bNodeSocketValueRGBA *>(data)->value;
        result.set_single_value(Color(v[0], v[1], v[2], v[3]));
        break;
      }
      case ResultType::Menu: {
        /* Store a MenuValue, not the raw identifier. Result's single-value variant also
         * contains int32_t, and Menu Switch then std::get<MenuValue> throws. */
        const auto *menu = static_cast<bNodeSocketValueMenu *>(data);
        const bool multi = (menu->runtime_flag & bke::NODE_MENU_MULTI_SELECTION) != 0 ||
                           (input_socket.flag & NODE_INTERFACE_SOCKET_MENU_MULTI_SELECTION) != 0;
        result.set_single_value(nodes::MenuValue(menu->value, multi));
        break;
      }
      default:
        /* Leave zero-initialized single value from allocate_single_value. */
        break;
    }
  }

  void evaluate()
  {
    using namespace blender::compositor;
    /* Python `tree.evaluate()` can run before the node tree update flushes. Force a
     * topology rebuild so compositor socket iterators are not stale. */
    for (bNode &node : node_tree_.nodes) {
      if (node.runtime != nullptr) {
        node.runtime->owner_tree = &node_tree_;
      }
      /* Shader textures (Noise / Voronoi / …) need Normalized UV implicit inputs.
       * Python `tree.evaluate()` can run before a full ntree update rebuilt declarations. */
      bke::node_declaration_ensure(node_tree_, node);
    }
    node_tree_.runtime->topology_cache_mutex.tag_dirty();
    node_tree_.ensure_topology_cache();
    node_tree_.ensure_interface_cache();
    /* ensure_topology_cache only writes owner_tree for nodes_by_id. Python-added zone
     * nodes can miss that map; pin every ListBase node so pass-through can key caches. */
    compositor::image_process_set_cook_tree_session_uid(node_tree_.id.session_uid);
    for (bNode &node : node_tree_.nodes) {
      if (node.runtime != nullptr) {
        node.runtime->owner_tree = &node_tree_;
      }
    }

    const bke::DataBlockComputeContext base_compute_context(nullptr, scene_.id);
    /* Evaluate viewers and File Output (named Image datablock) nodes. */
    NodeGroupOperation node_group_operation(
        *this,
        node_tree_,
        NodeGroupOutputTypes::ViewerNode | NodeGroupOutputTypes::FileOutputNode,
        base_compute_context);

    for (const bNodeTreeInterfaceSocket *input_socket : node_tree_.interface_inputs()) {
      const ResultType type = get_node_interface_socket_result_type(*input_socket);
      auto input_result = std::make_unique<Result>(this->create_result(type, ResultPrecision::Full));
      this->fill_interface_input_default(*input_result, *input_socket);
      node_group_operation.map_input_to_result(input_socket->identifier, input_result.get());
      owned_inputs_.append(std::move(input_result));
    }

    for (const bNodeTreeInterfaceSocket *output_socket : node_tree_.interface_outputs()) {
      Result &output_result = node_group_operation.get_result(output_socket->identifier);
      output_result.set_reference_count(0);
    }

    node_group_operation.evaluate();
  }
};

}  // namespace

namespace nodes {

Vector<IndexRange> image_process_cached_frame_ranges()
{
  return compositor::image_process_sim_cached_frame_ranges();
}

Vector<IndexRange> image_process_checkpoint_frame_ranges()
{
  return compositor::image_process_sim_checkpoint_frame_ranges();
}

}  // namespace nodes

bool ntreeImageNodesEvaluate(
    Main &bmain, Scene &scene, bNodeTree &ntree, bool *r_used_gpu, const int2 resolution)
{
  if (ntree.type != NTREE_IMAGE) {
    if (r_used_gpu) {
      *r_used_gpu = false;
    }
    return false;
  }

  /* Nested multi-frame sim fill must not re-enter fill logic. */
  static thread_local int sim_fill_depth = 0;

  const int target_frame = scene.r.cfra;
  bool used_gpu = false;

  auto cook_once = [&](nodes::eval_log::NodesEvalLog *eval_log) {
    nodes::image_points::PointsGeometryCache points_cache;
    nodes::image_points::StampAttrMapsCache stamp_attr_cache;
    nodes::image_points::StampAttrGpuCache stamp_attr_gpu_cache;
    nodes::image_points::set_active_cache(&points_cache);
    nodes::image_points::set_active_stamp_attr_cache(&stamp_attr_cache);
    nodes::image_points::set_active_stamp_attr_gpu_cache(&stamp_attr_gpu_cache);

    StaticCacheManager cache_manager;
    ImageNodesContext context(cache_manager, bmain, scene, ntree, resolution, eval_log);

    DRW_gpu_context_enable();
    const bool gpu_enabled = DRW_gpu_context_is_enabled();
    context.set_gpu_supported(gpu_enabled);

    /* Fresh sim input snapshots each cook so Output→Input feedback is not skipped. */
    compositor::image_process_prepare_cook();

    context.evaluate();
    used_gpu = context.use_gpu();

    nodes::image_points::set_active_stamp_attr_gpu_cache(nullptr);
    nodes::image_points::set_active_stamp_attr_cache(nullptr);
    nodes::image_points::set_active_cache(nullptr);
    stamp_attr_gpu_cache.clear();
    stamp_attr_cache.clear();
    points_cache.clear();

    cache_manager.reset();
    if (gpu_enabled) {
      gpu::TexturePool::get().reset();
    }
    DRW_gpu_context_disable();
  };

  /* Timeshift-style fill: scrub outside dense cache seeks nearest checkpoint, then cooks
   * intermediate frames forward to the target (see Geometry Nodes Time Shift fill).
   *
   * NEVER fill while Paint Edit is force-scheduling a single Paint node — fill can re-cook
   * up to 64 frames and blocks Enter Paint (Rasterize/Fluid trees feel frozen). */
  if (sim_fill_depth == 0 && !compositor::image_process_paint_force_schedule_active()) {
    const int resume = compositor::image_process_sim_resume_frame(ntree, target_frame);
    if (resume < target_frame) {
      sim_fill_depth++;
      compositor::image_process_set_fluid_scrub_fill(true);
      for (int frame = resume; frame < target_frame; frame++) {
        scene.r.cfra = frame;
        cook_once(nullptr);
      }
      compositor::image_process_set_fluid_scrub_fill(false);
      scene.r.cfra = target_frame;
      sim_fill_depth--;
    }
  }

  /* Fresh log for the final (view) cook so timings match the latest evaluation. */
  auto eval_log = std::make_unique<nodes::eval_log::NodesEvalLog>();
  cook_once(eval_log.get());

  /* Publish log for Node Editor timing overlay (SN_OVERLAY_SHOW_TIMINGS). */
  ntree.runtime->image_process_eval_log = std::move(eval_log);

  /* Do NOT tag Point Stamp / call BKE_main_ensure_invariants here.
   * That rebuilds context-dependent sockets and can re-enter Image Process cook every frame
   * (severe lag / "memory explosion"). Bundle UI updates when the user Syncs Separate Bundle
   * or when Point Stamp update runs from real graph edits. */

  if (r_used_gpu) {
    *r_used_gpu = used_gpu;
  }

  return true;
}

void ntreeImageNodesEvaluateAll(Main &bmain, Scene *scene, const int2 resolution)
{
  if (!scene) {
    scene = static_cast<Scene *>(bmain.scenes.first());
  }
  if (!scene) {
    return;
  }

  /* Image Output pixels live in Image datablocks; evaluation runtime state does not.
   * Re-cook every GPU Texture Editor tree so materials do not go black after reload. */
  for (bNodeTree &ntree : bmain.nodetrees) {
    if (ntree.type != NTREE_IMAGE || ID_IS_LINKED(&ntree.id)) {
      continue;
    }
    ntree.ensure_topology_cache();
    /* Skip empty trees. */
    if (ntree.all_nodes().is_empty()) {
      continue;
    }
    bool used_gpu = false;
    ntreeImageNodesEvaluate(bmain, *scene, ntree, &used_gpu, resolution);
  }
}

static void image_process_load_post_cb(Main *bmain,
                                       PointerRNA ** /*pointers*/,
                                       const int /*num_pointers*/,
                                       void * /*arg*/)
{
  if (!bmain) {
    return;
  }
  /* Defer heavy cooks slightly: LOAD_POST runs before some depsgraph setup, but our evaluate
   * builds its own GPU context. Still safe to cook Image Outputs for shader reuse. */
  ntreeImageNodesEvaluateAll(*bmain, nullptr, int2(1024, 1024));
}

/** Undo/redo restores node trees but not Image Output pixel buffers written at cook time.
 * Recook so Image datablocks (and materials using them) do not go empty/black. */
static void image_process_undo_post_cb(Main *bmain,
                                       PointerRNA ** /*pointers*/,
                                       const int /*num_pointers*/,
                                       void * /*arg*/)
{
  if (!bmain) {
    return;
  }
  ntreeImageNodesEvaluateAll(*bmain, nullptr, int2(1024, 1024));
}

void ntreeImageNodesRegisterLoadHooks()
{
  static bCallbackFuncStore store_load = {nullptr};
  if (store_load.func) {
    return;
  }
  store_load.func = image_process_load_post_cb;
  store_load.alloc = 0;
  BKE_callback_add(&store_load, BKE_CB_EVT_LOAD_POST);

  static bCallbackFuncStore store_undo = {nullptr};
  store_undo.func = image_process_undo_post_cb;
  store_undo.alloc = 0;
  BKE_callback_add(&store_undo, BKE_CB_EVT_UNDO_POST);

  static bCallbackFuncStore store_redo = {nullptr};
  store_redo.func = image_process_undo_post_cb;
  store_redo.alloc = 0;
  BKE_callback_add(&store_redo, BKE_CB_EVT_REDO_POST);
}

void node_tree_image_default_init(const bContext *C, bNodeTree *ntree)
{
  BLI_assert(ntree != nullptr && ntree->type == NTREE_IMAGE);
  BLI_assert(ntree->nodes.count() == 0);

  /* Default graph: Color → Viewer (backdrop). Prefer Viewer over Image Output so new trees
   * cook immediately without relying on File Output string-socket init. */
  bNode *rgb = bke::node_add_node(C, *ntree, "CompositorNodeRGB"_ustr);
  rgb->location[0] = -200.0f;
  rgb->location[1] = 100.0f;

  bNode *viewer = bke::node_add_node(C, *ntree, "ImageNodeViewer"_ustr);
  viewer->location[0] = 200.0f;
  viewer->location[1] = 100.0f;

  /* ListBase sockets are valid immediately after node_add_node (topology cache may not be). */
  bNodeSocket *rgb_out = static_cast<bNodeSocket *>(rgb->outputs.first());
  bNodeSocket *viewer_in = static_cast<bNodeSocket *>(viewer->inputs.first());
  if (rgb_out && viewer_in) {
    bke::node_add_link(*ntree, *rgb, *rgb_out, *viewer, *viewer_in);
  }

  bke::node_set_active(*ntree, *rgb);
  BKE_ntree_update_after_single_tree_change(*CTX_data_main(C), *ntree);
}

}  // namespace blender
