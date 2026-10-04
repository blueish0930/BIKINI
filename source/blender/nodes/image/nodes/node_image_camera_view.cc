/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Image Process: Camera View — viewport-style offscreen preview from a Camera.
 *
 * Matches 3D Viewport shading modes (Wireframe / Solid / Material / Rendered preview),
 * not F12 final render. Per-frame when demand-reachable; isolated via terminal filter.
 *
 * IMPORTANT: Image Process cooks under DRW_gpu_context_enable(). Offscreen draw also
 * enables that context and locks a non-recursive ticket mutex → nested enable DEADLOCKS.
 * Always drop the outer context before calling #image_process_view3d_fn. */

#include <cstring>

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"
#include "BLI_task.hh"

#include "DNA_object_enums.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_view3d_enums.h"

#include "BKE_layer.hh"
#include "BKE_main.hh"
#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_scene.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "DRW_engine.hh"

#include "GPU_texture.hh"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

#include "RE_pipeline.h"
#include "render_types.h"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "NOD_image.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_camera_view_cc {

NODE_STORAGE_FUNCS(NodeImageCameraView)

/** Prevent re-entry if offscreen somehow re-enters Image Process evaluation. */
static thread_local int g_camera_view_depth = 0;

/** Soft cap: match full HD/4K compositor domains; only scale down for ultra-wide / 8k+ domains. */
static constexpr int k_max_preview_dim = 4096;

static void node_declare(NodeDeclarationBuilder &b)
{
  /* Object picker is drawn via RNA "camera" (filtered to Camera objects only). */
  b.add_input<decl::Object>("Camera"_ustr)
      .description("Camera object; empty uses the scene camera")
      .custom_draw([](CustomSocketDrawParams &params) {
        params.layout.alignment_set(ui::LayoutAlign::Expand);
        params.layout.prop(
            &params.node_ptr, "camera", ui::ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
      });
  b.add_output<decl::Color>("Image"_ustr)
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description(
          "Viewport-style preview from the camera (Wireframe/Solid/Material/Rendered modes)");
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  auto *storage = MEM_new<NodeImageCameraView>(__func__);
  storage->draw_type = OB_SOLID;
  storage->alpha_mode = 1; /* transparent */
  /* Off by default: annotations need scene.gpd and cost an extra overlay path. */
  storage->show_annotations = 0;
  node->storage = storage;
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "draw_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(ptr, "alpha_mode", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(ptr, "show_annotations", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

using namespace blender::compositor;

class CameraViewOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    Result &out = this->get_result("Image");
    if (!out.should_compute()) {
      return;
    }

    const Domain domain = this->compute_domain();
    const int2 size = math::max(int2(1), domain.data_size);

    if (g_camera_view_depth > 0) {
      this->allocate_transparent(out, domain);
      this->context().set_info_message("Camera View: nested evaluation skipped");
      return;
    }

    Scene &scene = const_cast<Scene &>(this->context().get_scene());

    Depsgraph *depsgraph = this->get_depsgraph_readonly();
    if (!depsgraph) {
      this->allocate_transparent(out, domain);
      this->context().set_info_message("Camera View: no depsgraph");
      return;
    }

    if (!image_process_view3d_fn) {
      this->allocate_transparent(out, domain);
      this->context().set_info_message("Camera View: offscreen render unavailable");
      return;
    }

    /* Nested viewport draw (e.g. during 3D View draw) is not supported. */
    if (DRW_draw_in_progress()) {
      this->allocate_transparent(out, domain);
      this->context().set_info_message("Camera View: cannot draw during viewport draw");
      return;
    }

    const NodeImageCameraView &storage = node_storage(this->node());
    eDrawType draw_type = this->normalize_draw_type(eDrawType(storage.draw_type));
    /* Wireframe: transparent premul so mesh faces stay clear (only wires). */
    const int alpha_mode = (draw_type == OB_WIRE || storage.alpha_mode != 0) ? R_ALPHAPREMUL :
                                                                              R_ADDSKY;
    eV3DOffscreenDrawFlag draw_flags = V3D_OFSDRAW_NONE;
    if (storage.show_annotations) {
      draw_flags |= V3D_OFSDRAW_SHOW_ANNOTATION;
    }

    /* Prefer node.id (filtered camera picker), then Object socket, then scene camera. */
    Object *camera_ob = reinterpret_cast<Object *>(this->node().id);
    if (!camera_ob) {
      camera_ob = this->get_input("Camera").get_single_value_default<Object *>();
    }
    if (!camera_ob) {
      camera_ob = scene.camera;
    }
    if (!camera_ob || camera_ob->type != OB_CAMERA) {
      this->allocate_transparent(out, domain);
      this->context().set_info_message("Camera View: no valid camera");
      return;
    }

    Object *camera_eval = DEG_get_evaluated(depsgraph, camera_ob);
    if (!camera_eval || camera_eval->type != OB_CAMERA) {
      this->allocate_transparent(out, domain);
      this->context().set_info_message("Camera View: camera not evaluated");
      return;
    }

    const int2 render_size = this->clamped_preview_size(size);

    g_camera_view_depth++;

    /* Drop Image Process GPU context so offscreen can take the ticket mutex. */
    const bool had_drw_context = DRW_gpu_context_is_enabled();
    if (had_drw_context) {
      DRW_gpu_context_disable();
    }

    char err_out[256] = "";
    ImBuf *ibuf = image_process_view3d_fn(depsgraph,
                                          &scene,
                                          nullptr,
                                          draw_type,
                                          camera_eval,
                                          render_size.x,
                                          render_size.y,
                                          ImBufFlags::FloatData,
                                          draw_flags,
                                          alpha_mode,
                                          "",
                                          nullptr,
                                          nullptr,
                                          err_out);

    if (had_drw_context) {
      DRW_gpu_context_enable();
    }

    g_camera_view_depth--;

    if (!ibuf) {
      this->allocate_transparent(out, domain);
      if (err_out[0] != '\0') {
        this->context().set_info_message(err_out);
      }
      else {
        this->context().set_info_message("Camera View: preview failed");
      }
      return;
    }

    if (!ibuf->float_buffer.data) {
      IMB_float_from_byte(ibuf);
    }
    if (!ibuf->float_buffer.data) {
      IMB_freeImBuf(ibuf);
      this->allocate_transparent(out, domain);
      this->context().set_info_message("Camera View: no float buffer");
      return;
    }

    this->copy_imbuf_to_result(*ibuf, out, domain);
    IMB_freeImBuf(ibuf);
  }

 private:
  static eDrawType normalize_draw_type(const eDrawType draw_type)
  {
    /* Four viewport shading modes only (preview engines, not F12). */
    if (ELEM(draw_type, OB_WIRE, OB_SOLID, OB_MATERIAL, OB_RENDER)) {
      return draw_type;
    }
    return OB_SOLID;
  }

  static int2 clamped_preview_size(const int2 size)
  {
    int2 out = math::max(size, int2(1));
    if (out.x > k_max_preview_dim || out.y > k_max_preview_dim) {
      const float scale = float(k_max_preview_dim) / float(math::max(out.x, out.y));
      out.x = math::max(1, int(float(out.x) * scale));
      out.y = math::max(1, int(float(out.y) * scale));
    }
    return out;
  }

  Depsgraph *get_depsgraph_readonly()
  {
    /* Prefer an already-evaluated depsgraph. Never re-tag geometry every cook. */
    const Scene &scene = this->context().get_scene();
    Render *render = RE_GetSceneRender(&scene);
    if (render && render->pipeline_depsgraph) {
      return render->pipeline_depsgraph;
    }
    Main &main = const_cast<Main &>(this->context().get_main());
    ViewLayer *vl = BKE_view_layer_default_view(&const_cast<Scene &>(scene));
    if (!vl) {
      return nullptr;
    }
    Depsgraph *dg = BKE_scene_get_depsgraph(&const_cast<Scene &>(scene), vl);
    if (!dg) {
      dg = BKE_scene_ensure_depsgraph(&main, &const_cast<Scene &>(scene), vl);
      if (dg) {
        BKE_scene_graph_evaluated_ensure(dg, &main);
      }
    }
    return dg;
  }

  void allocate_transparent(Result &out, const Domain &domain)
  {
    out.allocate_texture(domain);
    if (out.is_stored_on_gpu() && out.gpu_texture()) {
      const float clear[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      GPU_texture_clear(out.gpu_texture(), GPU_DATA_FLOAT, clear);
      return;
    }
    if (out.is_allocated() && out.cpu_data_for_write().data()) {
      std::memset(out.cpu_data_for_write().data(), 0, size_t(out.size_in_bytes()));
    }
  }

  void copy_imbuf_to_result(ImBuf &ibuf, Result &out, const Domain &domain)
  {
    const int2 size = domain.data_size;
    const int src_w = ibuf.x;
    const int src_h = ibuf.y;
    const int channels = (ibuf.channels > 0) ? ibuf.channels : 4;
    const float *src = ibuf.float_buffer.data;

    /* Fast path: same size, RGBA float → direct GPU upload (or bulk CPU share). */
    if (src && src_w == size.x && src_h == size.y && channels == 4) {
      if (this->context().use_gpu()) {
        out.allocate_texture(domain, true, ResultStorageType::GPUImage);
        if (out.gpu_texture()) {
          GPU_texture_update(out.gpu_texture(), GPU_DATA_FLOAT, src);
          return;
        }
      }
      Result cpu_tmp = Result(this->context(), ResultType::Color, ResultPrecision::Full);
      cpu_tmp.allocate_texture(domain, false, ResultStorageType::CPUImage);
      std::memcpy(cpu_tmp.cpu_data_for_write().data(),
                  src,
                  size_t(size.x) * size_t(size.y) * 4 * sizeof(float));
      out.share_data(cpu_tmp);
      cpu_tmp.release();
      return;
    }

    /* Scale into domain. Prefer bilinear so a rare downscale (domain > soft cap) stays smooth
     * instead of blocky nearest-neighbor pixels. */
    Result cpu_tmp = Result(this->context(), ResultType::Color, ResultPrecision::Full);
    cpu_tmp.allocate_texture(domain, false, ResultStorageType::CPUImage);

    const float inv_w = 1.0f / float(math::max(size.x, 1));
    const float inv_h = 1.0f / float(math::max(size.y, 1));
    const int src_w_m1 = math::max(src_w - 1, 0);
    const int src_h_m1 = math::max(src_h - 1, 0);

    auto sample_nearest = [&](const int sx, const int sy) -> float4 {
      if (!src || src_w <= 0 || src_h <= 0) {
        return float4(0.0f);
      }
      const int x = math::clamp(sx, 0, src_w_m1);
      const int y = math::clamp(sy, 0, src_h_m1);
      const int64_t idx = (int64_t(y) * src_w + x) * channels;
      if (channels >= 4) {
        return float4(src[idx + 0], src[idx + 1], src[idx + 2], src[idx + 3]);
      }
      if (channels == 3) {
        return float4(src[idx + 0], src[idx + 1], src[idx + 2], 1.0f);
      }
      if (channels == 1) {
        return float4(src[idx], src[idx], src[idx], 1.0f);
      }
      return float4(0.0f);
    };

    parallel_for(size, [&](const int2 texel) {
      float4 color(0.0f);
      if (src && src_w > 0 && src_h > 0) {
        /* Map output pixel centers into source continuous coords. */
        const float u = (float(texel.x) + 0.5f) * inv_w * float(src_w) - 0.5f;
        const float v = (float(texel.y) + 0.5f) * inv_h * float(src_h) - 0.5f;
        const int x0 = int(math::floor(u));
        const int y0 = int(math::floor(v));
        const float fx = u - float(x0);
        const float fy = v - float(y0);
        const float4 c00 = sample_nearest(x0, y0);
        const float4 c10 = sample_nearest(x0 + 1, y0);
        const float4 c01 = sample_nearest(x0, y0 + 1);
        const float4 c11 = sample_nearest(x0 + 1, y0 + 1);
        const float4 c0 = math::interpolate(c00, c10, fx);
        const float4 c1 = math::interpolate(c01, c11, fx);
        color = math::interpolate(c0, c1, fy);
      }
      cpu_tmp.store_pixel(texel, Color(color.x, color.y, color.z, color.w));
    });

    if (this->context().use_gpu()) {
      Result gpu_result = cpu_tmp.upload_to_gpu(true);
      out.share_data(gpu_result);
      gpu_result.release();
      cpu_tmp.release();
    }
    else {
      out.share_data(cpu_tmp);
      cpu_tmp.release();
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new CameraViewOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeCameraView"_ustr, IMG_NODE_CAMERA_VIEW);
  ntype.ui_name = "Camera View";
  ntype.ui_description =
      "Viewport-style preview from a camera (Wireframe/Solid/Material/Rendered). "
      "Updates every frame when used; does not re-evaluate unrelated node branches";
  ntype.enum_name_legacy = "CAMERA_VIEW";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.default_width = bke::NodeWidth::_160;
  bke::node_type_storage(
      ntype, "NodeImageCameraView", node_free_standard_storage, node_copy_standard_storage);
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_camera_view_cc
