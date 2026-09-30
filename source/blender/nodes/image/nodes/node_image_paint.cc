/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Image Process: Paint — Color input as base; Enter paints on internal canvas. */

#include <cstring>

#include "BLI_hash.hh"
#include "BLI_map.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_utf8.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"

#include "BKE_global.hh"
#include "BKE_image.hh"
#include "BKE_image_gpu.hh"
#include "BKE_image_paint_layers.hh"
#include "BKE_lib_id.hh"
#include "BKE_library.hh"
#include "BKE_main.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_report.hh"

#include "DNA_image_types.h"
#include "DNA_node_types.h"

#include "DEG_depsgraph.hh"

#include "MEM_guardedalloc.h"

#include "GPU_texture.hh"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"
#include "IMB_partial_update.hh"

#include "BLT_translation.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_access.hh"

#include "COM_algorithm_extract_alpha.hh"
#include "COM_node_operation.hh"
#include "COM_undefined_node_operation.hh"
#include "COM_utilities.hh"

#include "NOD_image_paint.hh"

#include "node_image_util.hh"

namespace blender::nodes {

struct PaintInputCacheKey {
  uint session_uid = 0;
  int node_identifier = 0;
  uint64_t hash() const
  {
    return get_default_hash(session_uid, node_identifier);
  }
  friend bool operator==(const PaintInputCacheKey &a, const PaintInputCacheKey &b)
  {
    return a.session_uid == b.session_uid && a.node_identifier == b.node_identifier;
  }
};

struct PaintInputCache {
  Vector<float> rgba;
  int2 size = int2(0);
};

static Map<PaintInputCacheKey, PaintInputCache> &input_cache_map()
{
  static Map<PaintInputCacheKey, PaintInputCache> map;
  return map;
}

bool is_paint_node(const bNode &node)
{
  return node.is_type("ImageNodePaint"_ustr);
}

bool image_paint_uses_canvas(const bNode &node)
{
  return (int(node.custom1) & NODE_IMAGE_PAINT_USE_CANVAS) != 0;
}

void image_paint_set_use_canvas(bNode &node, const bool enable)
{
  if (enable) {
    node.custom1 = int16_t(int(node.custom1) | NODE_IMAGE_PAINT_USE_CANVAS);
  }
  else {
    node.custom1 = int16_t(int(node.custom1) & ~NODE_IMAGE_PAINT_USE_CANVAS);
  }
}

void image_paint_cache_input(const bNodeTree &ntree,
                             const int node_identifier,
                             const Span<float> rgba_float4,
                             const int2 size)
{
  if (size.x < 1 || size.y < 1 || rgba_float4.size() < size_t(size.x) * size.y * 4) {
    return;
  }
  PaintInputCacheKey key;
  key.session_uid = ntree.id.session_uid;
  key.node_identifier = node_identifier;
  PaintInputCache &entry = input_cache_map().lookup_or_add_default(key);
  entry.size = size;
  entry.rgba.reinitialize(size_t(size.x) * size.y * 4);
  memcpy(entry.rgba.data(), rgba_float4.data(), entry.rgba.size() * sizeof(float));
}

bool image_paint_get_cached_input(const bNodeTree &ntree,
                                  const int node_identifier,
                                  Vector<float> &r_rgba,
                                  int2 &r_size)
{
  PaintInputCacheKey key;
  key.session_uid = ntree.id.session_uid;
  key.node_identifier = node_identifier;
  const PaintInputCache *entry = input_cache_map().lookup_ptr(key);
  if (!entry || entry->size.x < 1 || entry->size.y < 1 || entry->rgba.is_empty()) {
    return false;
  }
  r_size = entry->size;
  r_rgba = entry->rgba;
  return true;
}

bool image_paint_write_canvas(Image &image, const Span<float> rgba_float4, const int2 size)
{
  if (size.x < 1 || size.y < 1 || rgba_float4.size() < size_t(size.x) * size.y * 4) {
    return false;
  }

  ImageUser iuser;
  BKE_imageuser_default(&iuser);
  void *lock = nullptr;
  ImBuf *ibuf = BKE_image_acquire_ibuf(&image, &iuser, &lock);
  if (!ibuf) {
    return false;
  }

  const bool size_changed = (ibuf->x != size.x || ibuf->y != size.y);
  if (size_changed) {
    IMB_free_byte_pixels(ibuf);
    IMB_free_float_pixels(ibuf);
    IMB_free_gpu_textures(ibuf);
    ibuf->x = size.x;
    ibuf->y = size.y;
  }
  if (!ibuf->float_buffer.data) {
    if (!IMB_alloc_float_pixels(ibuf, 4, false)) {
      BKE_image_release_ibuf(&image, ibuf, lock);
      return false;
    }
  }
  float *dst = ibuf->float_data_for_write();
  if (!dst || ibuf->x != size.x || ibuf->y != size.y) {
    BKE_image_release_ibuf(&image, ibuf, lock);
    return false;
  }
  memcpy(dst, rgba_float4.data(), size_t(size.x) * size.y * 4 * sizeof(float));

  IMB_mark_dirty(ibuf);
  IMB_partial_update_mark_full(ibuf);
  if (ImageTile *tile = static_cast<ImageTile *>(image.tiles.first())) {
    tile->gen_x = size.x;
    tile->gen_y = size.y;
  }
  BKE_image_release_ibuf(&image, ibuf, lock);
  BKE_image_free_gpu_texture_caches(&image);
  /* Same flush as Image Output: GPU consumers re-upload at draw; geometry nodes
   * and other CPU users need GENERIC_DATABLOCK (flags=0 + SOURCE). */
  DEG_id_tag_update(&image.id, 0);
  DEG_id_tag_update(&image.id, ID_RECALC_SOURCE | ID_RECALC_EDITORS);
  if (Main *bmain = blender::G.main) {
    BKE_image_paint_layers_on_source_changed(*bmain, image);
  }
  return true;
}

Image *image_paint_ensure_image(Main &bmain,
                                bNode &node,
                                const int2 resolution,
                                ReportList *reports)
{
  if (!is_paint_node(node)) {
    return nullptr;
  }

  const int2 res = int2(math::max(1, resolution.x), math::max(1, resolution.y));
  Image *existing = reinterpret_cast<Image *>(node.id);
  if (existing) {
    if (ELEM(existing->type, IMA_TYPE_R_RESULT, IMA_TYPE_COMPOSITE)) {
      if (reports) {
        BKE_report(reports, RPT_ERROR, "Paint canvas cannot be Viewer or Render Result");
      }
      return nullptr;
    }
    if (!ID_IS_EDITABLE(existing) || ID_IS_OVERRIDE_LIBRARY(existing)) {
      if (reports) {
        BKE_report(reports, RPT_ERROR, "Paint canvas is not editable");
      }
      return nullptr;
    }
    return existing;
  }

  const float color[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  Image *ima = BKE_image_add_generated(&bmain,
                                       uint(res.x),
                                       uint(res.y),
                                       "IP-Paint",
                                       32,
                                       true,
                                       IMA_GENTYPE_BLANK,
                                       color,
                                       false,
                                       false,
                                       false);
  if (!ima) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "Failed to create Paint canvas");
    }
    return nullptr;
  }

  node.id = &ima->id;
  id_us_plus(node.id);
  if (ImageUser *iuser = static_cast<ImageUser *>(node.storage)) {
    BKE_imageuser_default(iuser);
    BKE_image_init_imageuser(ima, iuser);
  }
  return ima;
}

}  // namespace blender::nodes

namespace blender::nodes::node_image_paint_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Color"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Base texture to paint on");

  b.add_output<decl::Color>("Color"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Painted result, or pass-through before first paint");

  b.add_output<decl::Float>("Alpha"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("Painted alpha, or 1 before first paint");
}

static void node_init(bNodeTree * /*node_tree*/, bNode *node)
{
  node->flag |= NODE_PREVIEW | NODE_OPTIONS;

  ImageUser *iuser = MEM_new<ImageUser>(__func__);
  BKE_imageuser_default(iuser);
  iuser->frames = 1;
  iuser->sfra = 1;
  node->storage = iuser;
  node->custom1 = 0;
  node->custom2 = 0;
}

static void node_labelfunc(const bNodeTree * /*ntree*/,
                           const bNode * /*node*/,
                           char *label,
                           const int label_maxncpy)
{
  BLI_strncpy_utf8(label, IFACE_("Paint"), label_maxncpy);
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  const bNode *node = static_cast<const bNode *>(ptr->data);
  ui::Layout &row = layout.row(true);
  row.scale_y_set(1.2f);
  PointerRNA reset_ptr = row.op("node.paint_reset", IFACE_("Reset"), ICON_FILE_REFRESH);
  PointerRNA edit_ptr = row.op("node.paint_edit", IFACE_("Edit"), ICON_TPAINT_HLT);
  if (node) {
    RNA_int_set(&reset_ptr, "node_id", node->identifier);
    RNA_int_set(&edit_ptr, "node_id", node->identifier);
  }
}

using namespace blender::compositor;

class PaintOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    Result &color_out = this->get_result("Color");
    Result &alpha_out = this->get_result("Alpha");
    Result &color_in = this->get_input("Color");

    Image *ima = this->get_image();
    ImageUser *iuser = this->get_image_user();
    const bool use_canvas = image_paint_uses_canvas(this->node()) && ima != nullptr &&
                            iuser != nullptr;

    /* Bake Color→session cache ONLY when Enter/Edit force-schedules this Paint node.
     * Doing it every cook was ~100–300ms+ (GPU download + full-domain CPU resample) even for
     * pass-through / add-node / viewer updates. paint_edit falls back to force-eval if cache
     * is cold. */
    if (!use_canvas && this->should_cache_input_for_edit()) {
      this->cache_input_for_paint(color_in);
    }

    if (use_canvas) {
      if (color_out.should_compute()) {
        this->output_from_canvas(color_out, *ima, *iuser);
      }
      if (alpha_out.should_compute()) {
        this->output_alpha_from_canvas(alpha_out, *ima, *iuser);
      }
    }
    else {
      if (color_out.should_compute()) {
        this->output_pass_through_color(color_in, color_out);
      }
      if (alpha_out.should_compute()) {
        this->output_alpha_from_color(color_in, alpha_out);
      }
    }
  }

  bool should_cache_input_for_edit() const
  {
    using namespace blender::compositor;
    if (!image_process_paint_force_schedule_active()) {
      return false;
    }
    if (image_process_paint_force_node_identifier() != this->node().identifier) {
      return false;
    }
    /* Force schedule matches tree session_uid (root Image tree). Nested group Paint nodes
     * may not match; still allow cache when identifier matches so blank-canvas fallback is rare. */
    return true;
  }

  /** Safe Color pass-through (share_data asserts if unallocated / type mismatch). */
  void output_pass_through_color(Result &color_in, Result &color_out)
  {
    if (!color_in.is_allocated() && !color_in.is_single_value()) {
      color_out.allocate_invalid();
      return;
    }
    if (color_in.is_single_value()) {
      color_out.allocate_single_value();
      if (color_in.type() == ResultType::Color) {
        color_out.set_single_value(color_in.get_single_value_default<Color>());
      }
      else if (color_in.type() == ResultType::Float) {
        const float v = color_in.get_single_value_default<float>();
        color_out.set_single_value(Color(v, v, v, 1.0f));
      }
      else {
        color_out.set_single_value(Color(0.0f, 0.0f, 0.0f, 1.0f));
      }
      return;
    }
    /* Texture: only share when types match and source is allocated. */
    if (color_in.is_allocated() && color_in.type() == ResultType::Color) {
      if (color_out.type() != ResultType::Color) {
        color_out.set_type(ResultType::Color);
      }
      color_out.set_precision(color_in.precision());
      color_out.share_data(color_in);
      return;
    }
    if (color_in.is_allocated() && color_in.type() == ResultType::Float) {
      /* Expand float texture to Color for Paint Color output. */
      Result cpu = color_in.is_stored_on_gpu() ? color_in.download_to_cpu() :
                                                 this->context().create_result(ResultType::Float);
      if (!color_in.is_stored_on_gpu()) {
        cpu.share_data(color_in);
      }
      Result dst = this->context().create_result(ResultType::Color);
      dst.allocate_texture(color_in.domain(), false, ResultStorageType::CPUImage);
      parallel_for(color_in.domain().data_size, [&](const int2 texel) {
        const float v = cpu.load_pixel_zero<float>(texel);
        dst.store_pixel(texel, Color(v, v, v, 1.0f));
      });
      if (this->context().use_gpu()) {
        Result g = dst.upload_to_gpu(true);
        color_out.share_data(g);
        g.release();
      }
      else {
        color_out.share_data(dst);
      }
      dst.release();
      if (color_in.is_stored_on_gpu()) {
        cpu.release();
      }
      return;
    }
    color_out.allocate_invalid();
  }

  void cache_input_for_paint(Result &color_in)
  {
    if (!color_in.is_allocated() && !color_in.is_single_value()) {
      return;
    }

    Result cpu_owned = this->context().create_result(color_in.type());
    const Result *cpu = &color_in;
    bool own_cpu = false;
    if (color_in.is_stored_on_gpu() && color_in.is_allocated() && !color_in.is_single_value()) {
      cpu_owned = color_in.download_to_cpu();
      cpu = &cpu_owned;
      own_cpu = true;
    }

    constexpr int k_max_dim = 8192;
    int2 size = math::max(this->context().get_compositing_domain().data_size, int2(1));
    size.x = math::clamp(size.x, 1, k_max_dim);
    size.y = math::clamp(size.y, 1, k_max_dim);
    const int64_t pixel_count = int64_t(size.x) * int64_t(size.y);
    Vector<float> rgba(size_t(pixel_count) * 4);

    auto sample_color = [&]() -> Color {
      if (color_in.type() == ResultType::Color) {
        return color_in.get_single_value_default<Color>();
      }
      if (color_in.type() == ResultType::Float) {
        const float v = color_in.get_single_value_default<float>();
        return Color(v, v, v, 1.0f);
      }
      return Color(0.0f, 0.0f, 0.0f, 1.0f);
    };

    if (color_in.is_single_value()) {
      /* Solid fill — no per-pixel branching; still only runs on Edit force-schedule. */
      const Color c = sample_color();
      float *dst = rgba.data();
      for (int64_t i = 0; i < pixel_count; i++, dst += 4) {
        dst[0] = c.r;
        dst[1] = c.g;
        dst[2] = c.b;
        dst[3] = c.a;
      }
    }
    else if (cpu->is_allocated()) {
      const int2 src_size = math::max(cpu->domain().data_size, int2(1));
      parallel_for(size, [&](const int2 texel) {
        const int2 src(
            math::clamp(int(float(texel.x) * src_size.x / float(size.x)), 0, src_size.x - 1),
            math::clamp(int(float(texel.y) * src_size.y / float(size.y)), 0, src_size.y - 1));
        Color c(0.0f, 0.0f, 0.0f, 1.0f);
        if (cpu->type() == ResultType::Color) {
          c = cpu->load_pixel_zero<Color>(src);
        }
        else if (cpu->type() == ResultType::Float) {
          const float v = cpu->load_pixel_zero<float>(src);
          c = Color(v, v, v, 1.0f);
        }
        const size_t i = (size_t(texel.y) * size_t(size.x) + size_t(texel.x)) * 4;
        rgba[i + 0] = c.r;
        rgba[i + 1] = c.g;
        rgba[i + 2] = c.b;
        rgba[i + 3] = c.a;
      });
    }
    else {
      if (own_cpu) {
        cpu_owned.release();
      }
      return;
    }

    image_paint_cache_input(this->node().owner_tree(), this->node().identifier, rgba, size);
    if (own_cpu) {
      cpu_owned.release();
    }
  }

  void output_from_canvas(Result &result, Image &ima, ImageUser &iuser)
  {
    const Result &cached = this->context().cache_manager().cached_images.get(
        this->context(), ima, iuser, "Image");
    if (!cached.is_allocated()) {
      result.allocate_invalid();
      return;
    }
    result.set_type(cached.type());
    result.set_precision(cached.precision());
    result.share_data(cached);
    this->force_to_compositing_domain(result);
  }

  void output_alpha_from_canvas(Result &result, Image &ima, ImageUser &iuser)
  {
    const Result &cached = this->context().cache_manager().cached_images.get(
        this->context(), ima, iuser, "Image");
    if (!cached.is_allocated()) {
      result.allocate_invalid();
      return;
    }
    this->output_alpha_from_color(cached, result);
  }

  void output_alpha_from_color(const Result &color_in, Result &alpha_out)
  {
    if (!color_in.is_allocated() && !color_in.is_single_value()) {
      alpha_out.allocate_invalid();
      return;
    }
    if (color_in.type() == ResultType::Color) {
      extract_alpha(this->context(), color_in, alpha_out);
      return;
    }
    /* Non-color: opaque alpha. */
    if (color_in.is_single_value()) {
      alpha_out.allocate_single_value();
      alpha_out.set_single_value(1.0f);
    }
    else {
      alpha_out.allocate_texture(color_in.domain());
      if (alpha_out.is_stored_on_gpu()) {
        const float one = 1.0f;
        GPU_texture_clear(alpha_out, GPU_DATA_FLOAT, &one);
      }
      else {
        parallel_for(color_in.domain().data_size,
                     [&](const int2 texel) { alpha_out.store_pixel(texel, 1.0f); });
      }
    }
  }

  void force_to_compositing_domain(Result &result)
  {
    /* Match Compositor Image node force_to_compositing_domain, but always free before re-alloc
     * (share_data leaves the Result allocated; allocate_texture asserts !is_allocated). */
    const Domain target = this->context().get_compositing_domain();
    if (!result.is_allocated() && !result.is_single_value()) {
      return;
    }

    if (result.is_single_value()) {
      if (result.type() != ResultType::Color) {
        return;
      }
      const Color color = result.get_single_value_default<Color>();
      result.free();
      result.allocate_texture(target);
      if (this->context().use_gpu()) {
        GPU_texture_clear(result, GPU_DATA_FLOAT, color);
      }
      else {
        parallel_for(target.data_size, [&](const int2 texel) { result.store_pixel(texel, color); });
      }
      return;
    }

    if (result.domain().data_size == target.data_size) {
      return;
    }
    if (result.type() != ResultType::Color) {
      return;
    }

    const bool was_gpu = result.is_stored_on_gpu();
    Result src_cpu = was_gpu ? result.download_to_cpu() :
                               this->context().create_result(result.type());
    if (!was_gpu) {
      src_cpu.share_data(result);
    }
    if (!src_cpu.is_allocated()) {
      if (was_gpu) {
        src_cpu.release();
      }
      return;
    }

    Result dst = this->context().create_result(ResultType::Color);
    dst.allocate_texture(target, false, ResultStorageType::CPUImage);
    const int2 src_size = math::max(src_cpu.domain().data_size, int2(1));
    if (src_size.x > 0 && src_size.y > 0) {
      parallel_for(target.data_size, [&](const int2 texel) {
        const int2 src_texel(
            math::clamp(
                int(float(texel.x) * src_size.x / float(target.data_size.x)), 0, src_size.x - 1),
            math::clamp(
                int(float(texel.y) * src_size.y / float(target.data_size.y)), 0, src_size.y - 1));
        dst.store_pixel(texel, src_cpu.load_pixel_zero<Color>(src_texel));
      });
      result.free();
      if (this->context().use_gpu()) {
        Result gpu = dst.upload_to_gpu(true);
        result.share_data(gpu);
        gpu.release();
      }
      else {
        result.share_data(dst);
      }
    }
    dst.release();
    if (was_gpu) {
      src_cpu.release();
    }
  }

  Image *get_image()
  {
    return reinterpret_cast<Image *>(node().id);
  }

  ImageUser *get_image_user()
  {
    return static_cast<ImageUser *>(node().storage);
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new PaintOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodePaint"_ustr, IMG_NODE_PAINT);
  ntype.ui_name = "Paint";
  ntype.ui_description =
      "Paint on a Color texture. Press Edit or Enter to paint; Reset clears paint";
  ntype.enum_name_legacy = "PAINT";
  ntype.nclass = NODE_CLASS_OP_FILTER;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.labelfunc = node_labelfunc;
  bke::node_type_storage(
      ntype, "ImageUser", node_free_standard_storage, node_copy_standard_storage);
  ntype.get_compositor_operation = get_compositor_operation;
  ntype.flag |= NODE_PREVIEW;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_paint_cc
