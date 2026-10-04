/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Image Process: Render Material — assign a material to a UV plane and render to Color.
 *
 * UI is only the material picker. The plane fills the compositing domain; output is Color.
 *
 * IMPORTANT: Image Process cooks under DRW_gpu_context_enable(). Offscreen draw also
 * enables that context and locks a non-recursive ticket mutex → nested enable DEADLOCKS.
 * Always drop the outer context before calling #image_process_render_material_fn. */

#include <cstring>

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"

#include "DNA_ID.h"
#include "DNA_material_types.h"

#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"

#include "DRW_engine.hh"

#include "GPU_texture.hh"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

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

namespace blender::nodes::node_image_render_material_cc {

/** Prevent re-entry if offscreen somehow re-enters Image Process evaluation. */
static thread_local int g_render_material_depth = 0;

static constexpr int k_max_preview_dim = 4096;

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Color>("Color"_ustr)
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Material rendered on a UV-mapped plane that fills the image")
      .custom_draw([](CustomSocketDrawParams &params) {
        params.layout.alignment_set(ui::LayoutAlign::Expand);
        params.layout.prop(
            &params.node_ptr, "material", ui::ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
      });
}

using namespace blender::compositor;

class RenderMaterialOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    Result &out = this->get_result("Color");
    if (!out.should_compute()) {
      return;
    }

    const Domain domain = this->compute_domain();
    const int2 size = math::max(int2(1), domain.data_size);

    if (g_render_material_depth > 0) {
      this->allocate_transparent(out, domain);
      this->context().set_info_message("Render Material: nested evaluation skipped");
      return;
    }

    ID *id = this->node().id;
    Material *material = (id && GS(id->name) == ID_MA) ? reinterpret_cast<Material *>(id) :
                                                         nullptr;
    if (!material) {
      this->allocate_transparent(out, domain);
      this->context().set_info_message("Render Material: no material");
      return;
    }

    if (!image_process_render_material_fn) {
      this->allocate_transparent(out, domain);
      this->context().set_info_message("Render Material: offscreen render unavailable");
      return;
    }

    if (DRW_draw_in_progress()) {
      this->allocate_transparent(out, domain);
      this->context().set_info_message("Render Material: cannot draw during viewport draw");
      return;
    }

    const int2 render_size = this->clamped_preview_size(size);

    g_render_material_depth++;

    const bool had_drw_context = DRW_gpu_context_is_enabled();
    if (had_drw_context) {
      DRW_gpu_context_disable();
    }

    Main &main = const_cast<Main &>(this->context().get_main());
    Scene &scene = const_cast<Scene &>(this->context().get_scene());

    char err_out[256] = "";
    ImBuf *ibuf = image_process_render_material_fn(
        &main, &scene, material, render_size.x, render_size.y, err_out);

    if (had_drw_context) {
      DRW_gpu_context_enable();
    }

    g_render_material_depth--;

    if (!ibuf) {
      this->allocate_transparent(out, domain);
      if (err_out[0] != '\0') {
        this->context().set_info_message(err_out);
      }
      else {
        this->context().set_info_message("Render Material: preview failed");
      }
      return;
    }

    if (!ibuf->float_buffer.data) {
      IMB_float_from_byte(ibuf);
    }
    if (!ibuf->float_buffer.data) {
      IMB_freeImBuf(ibuf);
      this->allocate_transparent(out, domain);
      this->context().set_info_message("Render Material: no float buffer");
      return;
    }

    this->copy_imbuf_to_result(*ibuf, out, domain);
    IMB_freeImBuf(ibuf);
  }

 private:
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
  return new RenderMaterialOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeRenderMaterial"_ustr, IMG_NODE_RENDER_MATERIAL);
  ntype.ui_name = "Render Material";
  ntype.ui_description =
      "Temporarily assign a material to a UV plane and render it to a color texture";
  ntype.enum_name_legacy = "RENDER_MATERIAL";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.default_width = bke::NodeWidth::_180;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_render_material_cc
