/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Image Process Fluid Simulation zone (Input + Output).
 *
 * Full integration of Pavel Dobryakov's WebGL Fluid Simulation (MIT):
 *   https://github.com/PavelDoGreat/WebGL-Fluid-Simulation
 * Vendored at: extern/webgl_fluid_simulation/ (canonical script.js + extracted_shaders/).
 *
 * GPU path is ONLY his step(dt) — collocated velocity, Jacobi pressure, no Multigrid, no MAC.
 *   curl -> vorticity -> divergence -> pressure *= PRESSURE -> Jacobi x N ->
 *   gradient subtract -> advect velocity -> advect dye
 *
 * Blender I/O (zone sockets, GPU cache, domain resample) is the only custom glue.
 * Exterior Color/Velocity seed on cold start / start frame only; ongoing frames use GPU cache.
 * Optional Collision / Air / Temperature are no-ops when unconnected.
 */

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstring>
#include <functional>
#include <string>

#include "BLI_array.hh"
#include "BLI_color_types.hh"
#include "BLI_index_range.hh"
#include "BLI_map.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"
#include "BLI_string_utf8.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

#include "DNA_node_types.h"
#include "DNA_scene_types.h"

#include "BKE_context.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_zones.hh"
#include "BKE_report.hh"
#include "BKE_screen.hh"

#include "ED_node.hh"
#include "ED_screen.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "BLT_translation.hh"

#include "BLO_read_write.hh"

#include "COM_domain.hh"
#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_undefined_node_operation.hh"
#include "COM_utilities.hh"

#include "GPU_context.hh"
#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"

#include "NOD_image_fluid.hh"
#include "NOD_image_fluid_solver.hh"
#include "NOD_socket_declarations.hh"
#include "NOD_socket_items.hh"
#include "NOD_socket_items_blend.hh"
#include "NOD_socket_items_ops.hh"
#include "NOD_socket_items_ui.hh"

#include "node_image_util.hh"

namespace blender::nodes {

StructRNA **FluidColorItemsAccessor::item_srna = &RNA_NodeImageFluidColorItem;
int FluidColorItemsAccessor::node_type = IMG_NODE_FLUID_SIM_OUTPUT;

void FluidColorItemsAccessor::blend_write_item(BlendWriter *writer, const ItemT &item)
{
  writer->write_string(item.name);
}

void FluidColorItemsAccessor::blend_read_data_item(BlendDataReader *reader, ItemT &item)
{
  BLO_read_string(reader, &item.name);
}

}  // namespace blender::nodes

namespace blender::nodes::node_image_fluid_sim_cc {

using namespace blender::compositor;
using namespace blender::nodes::image_fluid;

/* -------------------------------------------------------------------- */
/** \name Shared declare helpers
 * \{ */

static void declare_color_items(NodeDeclarationBuilder &b, const bool is_input_node)
{
  const bNodeTree *tree = b.tree_or_null();
  const bNode *node = b.node_or_null();
  if (!node || !tree) {
    return;
  }

  /* Color items live on the Output node storage (paired). */
  const bNode *output_node = node;
  if (is_input_node) {
    const auto &in_storage = *static_cast<const NodeImageFluidSimInput *>(node->storage);
    output_node = tree->node_by_id(in_storage.output_node_id);
    if (!output_node) {
      return;
    }
  }
  const auto &storage = *static_cast<const NodeImageFluidSimOutput *>(output_node->storage);

  PanelDeclarationBuilder &color_panel = b.add_panel("Color"_ustr);
  for (const int i : IndexRange(storage.color_items_num)) {
    const NodeImageFluidColorItem &item = storage.color_items[i];
    const UString name(item.name);
    const UString identifier(FluidColorItemsAccessor::socket_identifier_for_item(item));
    color_panel.add_input<decl::Color>(name, identifier)
        .default_value({0.0f, 0.0f, 0.0f, 1.0f})
        .hide_value()
        .structure_type(StructureType::Dynamic)
        .socket_name_ptr(&tree->id, *FluidColorItemsAccessor::item_srna, &item, "name")
        .description("Density / dye color field advected by the fluid velocity")
        .custom_draw([i](CustomSocketDrawParams &params) {
          socket_items::ui::draw_item_socket_with_remove<FluidColorItemsAccessor>(params, i);
        });
    color_panel.add_output<decl::Color>(name, identifier)
        .structure_type(StructureType::Dynamic)
        .align_with_previous()
        .description("Advected color field");
  }
  color_panel.add_input<decl::Extend>(""_ustr, "__extend__"_ustr)
      .structure_type(StructureType::Dynamic)
      .custom_draw(socket_items::ui::draw_extend_socket_fn<FluidColorItemsAccessor>());
  color_panel.add_output<decl::Extend>(""_ustr, "__extend__"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous();
}

static void declare_properties_panel(PanelDeclarationBuilder &panel)
{
  panel.add_input<decl::Vector>("Velocity"_ustr)
      .default_value(float2(0.0f, 0.0f))
      .dimensions(2)
      .subtype(PROP_XYZ)
      .structure_type(StructureType::Dynamic)
      .description("2D fluid velocity (X,Y)");
  panel.add_output<decl::Vector>("Velocity"_ustr)
      .dimensions(2)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Simulated 2D velocity");

  panel.add_input<decl::Float>("Temperature"_ustr)
      .default_value(0.0f)
      .structure_type(StructureType::Dynamic)
      .description("Scalar temperature field (advected with the flow; diagnostic/export)");
  panel.add_output<decl::Float>("Temperature"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Advected temperature field");

  panel.add_input<decl::Float>("Divergence"_ustr)
      .default_value(0.0f)
      .structure_type(StructureType::Dynamic)
      .description("Divergence field (diagnostic / cold-start); solver recomputes RHS each step");
  panel.add_output<decl::Float>("Divergence"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Post-projection divergence (near zero when incompressible)");
}

static void declare_collision_panel(PanelDeclarationBuilder &panel)
{
  panel.add_input<decl::Float>("Collision Mask"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .max(1.0f)
      .structure_type(StructureType::Dynamic)
      .description("Solid obstacle mask (>0.5 = solid wall, blocks flow)");
  panel.add_output<decl::Float>("Collision Mask"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Collision mask (pass-through)");

  panel.add_input<decl::Vector>("Collider Velocity"_ustr)
      .default_value(float2(0.0f, 0.0f))
      .dimensions(2)
      .subtype(PROP_XYZ)
      .structure_type(StructureType::Dynamic)
      .description("Velocity of solid cells (collider)");
  panel.add_output<decl::Vector>("Collider Velocity"_ustr)
      .dimensions(2)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Collider velocity (pass-through)");

  panel.add_input<decl::Float>("Air Mask"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .max(1.0f)
      .structure_type(StructureType::Dynamic)
      .description(
          "Air / free space (>0.5 = air). Air is not incompressible liquid: pressure is 0 "
          "(free surface) so flow can pass freely. Use black=air white=liquid. Solid overrides air. "
          "Empty/unconnected = entire domain is liquid");
  panel.add_output<decl::Float>("Air Mask"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Air mask (pass-through)");
}

static void declare_aligned_panels(NodeDeclarationBuilder &b, const bool is_input_node)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  if (is_input_node) {
    b.add_output<decl::Float>("Delta Time"_ustr)
        .description("Time in seconds since the previous cached fluid state");
  }

  declare_color_items(b, is_input_node);

  PanelDeclarationBuilder &props = b.add_panel("Properties"_ustr);
  declare_properties_panel(props);

  PanelDeclarationBuilder &collision = b.add_panel("Collision"_ustr);
  declare_collision_panel(collision);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Multi-frame cache (dense + checkpoint + pressure warm-start)
 * \{ */

struct FluidCacheKey {
  uint session_uid = 0;
  int32_t output_node_id = 0;

  uint64_t hash() const
  {
    return get_default_hash(session_uid, output_node_id);
  }
  friend bool operator==(const FluidCacheKey &a, const FluidCacheKey &b)
  {
    return a.session_uid == b.session_uid && a.output_node_id == b.output_node_id;
  }
};

struct FluidFrameState {
  /** Velocity / pressure grid size (SIM_RESOLUTION). */
  int width = 0;
  int height = 0;
  /** Dye grid size (higher than sim 鈥?WebGL DYE_RESOLUTION). */
  int dye_width = 0;
  int dye_height = 0;
  /** Number of Color sockets (each stores RGB 鈫?3 channels in `colors`). */
  int color_count = 0;
  std::vector<float> vel_x;
  std::vector<float> vel_y;
  std::vector<float> temperature;
  std::vector<float> divergence;
  std::vector<float> pressure;
  /**
   * Dye channels at dye_width脳dye_height.
   * Layout: socket i 鈫?colors[i*3+0]=R, [i*3+1]=G, [i*3+2]=B (premultiplied).
   */
  std::vector<std::vector<float>> colors;
};

/**
 * GPU-resident last-frame MAC fields (avoids full-grid CPU upload/download).
 *   u: (W+1)脳H face velocity, v: W脳(H+1), dye/pressure: W脳H cell-centered.
 */
struct FluidGpuState {
  int width = 0;
  int height = 0;
  int dye_width = 0;
  int dye_height = 0;
  int frame = INT_MIN;
  /* Pavel/WebGL layout: collocated RG velocity and scalar pressure. */
  gpu::Texture *velocity = nullptr;
  Vector<gpu::Texture *> dyes;
  gpu::Texture *temperature = nullptr;
  gpu::Texture *divergence = nullptr;
  /* Legacy MAC fields are retained only so old in-memory state can be discarded safely. */
  gpu::Texture *u = nullptr;
  gpu::Texture *v = nullptr;
  gpu::Texture *dye = nullptr;
  gpu::Texture *pressure = nullptr;

  void free_textures()
  {
    GPU_TEXTURE_FREE_SAFE(velocity);
    for (gpu::Texture *&texture : dyes) {
      GPU_TEXTURE_FREE_SAFE(texture);
    }
    dyes.clear();
    GPU_TEXTURE_FREE_SAFE(temperature);
    GPU_TEXTURE_FREE_SAFE(divergence);
    GPU_TEXTURE_FREE_SAFE(u);
    GPU_TEXTURE_FREE_SAFE(v);
    GPU_TEXTURE_FREE_SAFE(dye);
    GPU_TEXTURE_FREE_SAFE(pressure);
    width = height = dye_width = dye_height = 0;
    frame = INT_MIN;
  }
};

struct FluidZoneCache {
  int last_solved = INT_MIN;
  /** Hash of solver DNA params — mismatch marks cache stale. */
  uint64_t solver_key = 0;
  bool hold_display = false;
  /** True when solver DNA changed and we haven't yet done the start-frame reset. */
  bool needs_param_reset = false;
  Map<int, FluidFrameState> dense;
  Map<int, FluidFrameState> checkpoints;
  /** Temporary cache: frames between the nearest checkpoint and a preview/scrub target.
   * Cleared when the preview interval changes (checkpoint anchor differs). */
  Map<int, FluidFrameState> preview_interval;
  /** First frame of the current preview interval (or INT_MIN). */
  int preview_interval_start = INT_MIN;
  FluidGpuState gpu;
};

/** RGB channel helpers for one Color socket index. */
static int dye_ch_r(const int socket_i)
{
  return socket_i * 3;
}
static int dye_ch_g(const int socket_i)
{
  return socket_i * 3 + 1;
}
static int dye_ch_b(const int socket_i)
{
  return socket_i * 3 + 2;
}
static int dye_channel_total(const int socket_count)
{
  return math::max(socket_count, 1) * 3;
}

static Map<FluidCacheKey, FluidZoneCache> &fluid_zone_caches()
{
  static Map<FluidCacheKey, FluidZoneCache> cache;
  return cache;
}

/** Lightweight global frame sets for timeline drawing (survives editor without sim cook context). */
static VectorSet<int> &fluid_timeline_dense_frames()
{
  static VectorSet<int> frames;
  return frames;
}
static VectorSet<int> &fluid_timeline_checkpoint_frames()
{
  static VectorSet<int> frames;
  return frames;
}

static void fluid_timeline_register(const int frame, const bool is_checkpoint)
{
  fluid_timeline_dense_frames().add(frame);
  if (is_checkpoint) {
    fluid_timeline_checkpoint_frames().add(frame);
  }
}

static void fluid_timeline_unregister_after(const int keep_frame)
{
  Vector<int> drop;
  for (const int f : fluid_timeline_dense_frames()) {
    if (f > keep_frame) {
      drop.append(f);
    }
  }
  for (const int f : drop) {
    fluid_timeline_dense_frames().remove(f);
    fluid_timeline_checkpoint_frames().remove(f);
  }
}

static Vector<int> fluid_all_cached_frames()
{
  VectorSet<int> frames = fluid_timeline_dense_frames();
  for (const FluidZoneCache &cache : fluid_zone_caches().values()) {
    for (const auto &item : cache.dense.items()) {
      frames.add(item.key);
    }
    for (const auto &item : cache.checkpoints.items()) {
      frames.add(item.key);
    }
  }
  Vector<int> sorted = frames.extract_vector();
  std::sort(sorted.begin(), sorted.end());
  return sorted;
}

static Vector<int> fluid_all_checkpoint_frames()
{
  VectorSet<int> frames = fluid_timeline_checkpoint_frames();
  for (const FluidZoneCache &cache : fluid_zone_caches().values()) {
    for (const auto &item : cache.checkpoints.items()) {
      frames.add(item.key);
    }
  }
  Vector<int> sorted = frames.extract_vector();
  std::sort(sorted.begin(), sorted.end());
  return sorted;
}

static void fluid_prune_dense(FluidZoneCache &cache, const int frame, const int cached_frames)
{
  const int keep = math::max(cached_frames, 1);
  const int min_keep = frame - keep + 1;
  Vector<int> remove;
  for (const auto &item : cache.dense.items()) {
    if (item.key < min_keep) {
      remove.append(item.key);
    }
  }
  for (const int f : remove) {
    cache.dense.remove(f);
  }
}

/** True only for full MAC snapshots (not empty timeline shells). */
static bool fluid_state_has_mac_data(const FluidFrameState &s)
{
  if (s.width < 2 || s.height < 2) {
    return false;
  }
  const int w = s.width;
  const int h = s.height;
  /* MAC faces or legacy cell-centered vel (cell: w*h, MAC: (w+1)*h / w*(h+1)). */
  const bool mac = int(s.vel_x.size()) == (w + 1) * h && int(s.vel_y.size()) == w * (h + 1);
  const bool cell = int(s.vel_x.size()) == w * h && int(s.vel_y.size()) == w * h;
  return (mac || cell) && !s.colors.empty();
}

static const FluidFrameState *fluid_find_state(const FluidZoneCache &cache, const int frame)
{
  if (const FluidFrameState *s = cache.preview_interval.lookup_ptr(frame)) {
    if (fluid_state_has_mac_data(*s)) {
      return s;
    }
  }
  if (const FluidFrameState *s = cache.dense.lookup_ptr(frame)) {
    if (fluid_state_has_mac_data(*s)) {
      return s;
    }
  }
  if (const FluidFrameState *s = cache.checkpoints.lookup_ptr(frame)) {
    if (fluid_state_has_mac_data(*s)) {
      return s;
    }
  }
  return nullptr;
}

static int fluid_nearest_at_or_before(const FluidZoneCache &cache, const int frame)
{
  int best = INT_MIN;
  for (const auto &item : cache.preview_interval.items()) {
    if (item.key <= frame && item.key > best && fluid_state_has_mac_data(item.value)) {
      best = item.key;
    }
  }
  for (const auto &item : cache.dense.items()) {
    if (item.key <= frame && item.key > best && fluid_state_has_mac_data(item.value)) {
      best = item.key;
    }
  }
  for (const auto &item : cache.checkpoints.items()) {
    if (item.key <= frame && item.key > best && fluid_state_has_mac_data(item.value)) {
      best = item.key;
    }
  }
  return best;
}

static bool fluid_should_checkpoint(const int frame, const int checkpoint_rate)
{
  if (checkpoint_rate <= 0) {
    return false;
  }
  if (frame <= 1) {
    return true;
  }
  return (frame % checkpoint_rate) == 0;
}

static void fluid_store_state(FluidZoneCache &cache,
                              const int frame,
                              FluidFrameState &&state,
                              const int cached_frames,
                              const int checkpoint_rate)
{
  const bool is_cp = fluid_should_checkpoint(frame, checkpoint_rate);
  /* During scrub-back fill, route intermediate frames to preview_interval
   * so they don't pollute the dense rolling window. */
  if (compositor::image_process_get_fluid_scrub_fill()) {
    /* Clear preview_interval if we've jumped to a different checkpoint interval. */
    const int anchor = frame - (frame % math::max(checkpoint_rate, 1));
    if (cache.preview_interval_start != INT_MIN && cache.preview_interval_start != anchor) {
      cache.preview_interval.clear();
    }
    cache.preview_interval_start = anchor;
    cache.preview_interval.add_overwrite(frame, std::move(state));
    if (is_cp) {
      cache.checkpoints.add_overwrite(frame, cache.preview_interval.lookup(frame));
    }
    cache.last_solved = frame;
    fluid_timeline_register(frame, is_cp);
    return;
  }
  /* Normal forward cook: store in dense window + checkpoints. */
  cache.dense.add_overwrite(frame, std::move(state));
  if (is_cp) {
    cache.checkpoints.add_overwrite(frame, cache.dense.lookup(frame));
  }
  cache.last_solved = frame;
  fluid_prune_dense(cache, frame, cached_frames);
  fluid_timeline_register(frame, is_cp);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Result 鈫?grid helpers
 * \{ */

static Domain fluid_domain(const Context &context)
{
  return context.get_compositing_domain();
}

static void ensure_cpu_color(Context &context, Result &in, Result &cpu_storage, const Result *&out)
{
  if (context.use_gpu() && in.is_allocated() && !in.is_single_value()) {
    cpu_storage = in.download_to_cpu();
    out = &cpu_storage;
  }
  else {
    out = &in;
  }
}

/**
 * Map destination pixel (x,y) in `dst_size` → source texel in the input Result.
 * MUST scale UV: inputs live at Image Process domain size; sim grids are often smaller.
 * Old code used load_pixel(x,y) with sim coords → only sampled bottom-left corner of
 * the full domain (broken advection + asymmetric color on first frame).
 */
static int2 map_dst_to_src_texel(const int x, const int y, const int2 dst_size, const int2 src_size)
{
  if (src_size.x <= 0 || src_size.y <= 0 || dst_size.x <= 0 || dst_size.y <= 0) {
    return int2(0);
  }
  if (src_size == dst_size) {
    return int2(math::clamp(x, 0, src_size.x - 1), math::clamp(y, 0, src_size.y - 1));
  }
  const int sx = math::clamp(
      int(std::floor((float(x) + 0.5f) * float(src_size.x) / float(dst_size.x))),
      0,
      src_size.x - 1);
  const int sy = math::clamp(
      int(std::floor((float(y) + 0.5f) * float(src_size.y) / float(dst_size.y))),
      0,
      src_size.y - 1);
  return int2(sx, sy);
}

static void fill_scalar_from_result(const Result &in,
                                    const int2 size,
                                    std::vector<float> &dst,
                                    const int channel)
{
  const int n = size.x * size.y;
  dst.assign(size_t(n), 0.0f);
  if (!in.is_allocated() && !in.is_single_value()) {
    return;
  }
  if (in.is_single_value()) {
    float v = 0.0f;
    if (in.type() == ResultType::Float) {
      v = in.get_single_value_default<float>();
    }
    else if (in.type() == ResultType::Color) {
      const Color c = in.get_single_value_default<Color>();
      v = (&c.r)[math::clamp(channel, 0, 3)];
    }
    else if (in.type() == ResultType::Float2) {
      const float2 c = in.get_single_value_default<float2>();
      v = channel == 0 ? c.x : c.y;
    }
    else if (in.type() == ResultType::Float3) {
      const float3 c = in.get_single_value_default<float3>();
      v = channel == 0 ? c.x : (channel == 1 ? c.y : c.z);
    }
    std::fill(dst.begin(), dst.end(), v);
    return;
  }
  const int2 src_size = math::max(in.domain().data_size, int2(1));
  for (int y = 0; y < size.y; y++) {
    for (int x = 0; x < size.x; x++) {
      const int2 texel = map_dst_to_src_texel(x, y, size, src_size);
      const int i = y * size.x + x;
      if (in.type() == ResultType::Float) {
        dst[i] = in.load_pixel<float>(texel);
      }
      else if (in.type() == ResultType::Color) {
        const Color c = in.load_pixel<Color>(texel);
        dst[i] = (&c.r)[math::clamp(channel, 0, 3)];
      }
      else if (in.type() == ResultType::Float2) {
        const float2 c = in.load_pixel<float2>(texel);
        dst[i] = channel == 0 ? c.x : c.y;
      }
      else if (in.type() == ResultType::Float3) {
        const float3 c = in.load_pixel<float3>(texel);
        dst[i] = channel == 0 ? c.x : (channel == 1 ? c.y : c.z);
      }
    }
  }
}

static void share_result_safe(Result &out, Result &src)
{
  /* Result::share_data asserts !out.is_allocated(); free first to avoid heap corruption. */
  if (out.is_allocated()) {
    out.free();
  }
  if (!src.is_allocated()) {
    return;
  }
  out.share_data(src);
}

static void write_float_result(Context &context,
                               Result &out,
                               const Domain &domain,
                               const std::vector<float> &src)
{
  if (!out.should_compute()) {
    return;
  }
  const int2 size = domain.data_size;
  if (size.x <= 0 || size.y <= 0) {
    return;
  }
  Result cpu = context.create_result(ResultType::Float);
  cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
  const size_t n = size_t(size.x) * size_t(size.y);
  for (int y = 0; y < size.y; y++) {
    for (int x = 0; x < size.x; x++) {
      const size_t i = size_t(y * size.x + x);
      cpu.store_pixel(int2(x, y), (i < src.size() && i < n) ? src[i] : 0.0f);
    }
  }
  if (context.use_gpu()) {
    Result gpu = cpu.upload_to_gpu(true);
    share_result_safe(out, gpu);
    gpu.release();
  }
  else {
    share_result_safe(out, cpu);
  }
  cpu.release();
}

/** Premul Color for fluid dye: opaque wherever density exists (no a=max(rgb) washout). */
static Color fluid_dye_color(const float r, const float g, const float b)
{
  const float rv = math::clamp(math::max(r, 0.0f), 0.0f, 1.0f);
  const float gv = math::clamp(math::max(g, 0.0f), 0.0f, 1.0f);
  const float bv = math::clamp(math::max(b, 0.0f), 0.0f, 1.0f);
  const float dens = math::max(rv, math::max(gv, bv));
  if (dens < 1e-4f) {
    return Color(0.0f, 0.0f, 0.0f, 0.0f);
  }
  return Color(rv, gv, bv, 1.0f);
}

/**
 * Write premultiplied RGB dye to a Color result.
 * Opaque (a=1) wherever dye density exists — a=max(rgb) was washing diluted
 * color into fully transparent viewer pixels.
 */
static void write_color_result_rgb(Context &context,
                                   Result &out,
                                   const Domain &domain,
                                   const std::vector<float> &r,
                                   const std::vector<float> &g,
                                   const std::vector<float> &b)
{
  if (!out.should_compute()) {
    return;
  }
  const int2 size = domain.data_size;
  if (size.x <= 0 || size.y <= 0) {
    return;
  }
  Result cpu = context.create_result(ResultType::Color);
  cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
  for (int y = 0; y < size.y; y++) {
    for (int x = 0; x < size.x; x++) {
      const size_t i = size_t(y * size.x + x);
      const float rv = (i < r.size()) ? r[i] : 0.0f;
      const float gv = (i < g.size()) ? g[i] : 0.0f;
      const float bv = (i < b.size()) ? b[i] : 0.0f;
      cpu.store_pixel(int2(x, y), fluid_dye_color(rv, gv, bv));
    }
  }
  if (context.use_gpu()) {
    Result gpu = cpu.upload_to_gpu(true);
    share_result_safe(out, gpu);
    gpu.release();
  }
  else {
    share_result_safe(out, cpu);
  }
  cpu.release();
}

/** Legacy single-channel write (grayscale dye) — kept for empty fallbacks. */
static void write_color_result(Context &context,
                               Result &out,
                               const Domain &domain,
                               const std::vector<float> &src)
{
  write_color_result_rgb(context, out, domain, src, src, src);
}

/** Load premul RGB dye; zero fully transparent texels (stops white garbage blocks). */
static void fill_color_rgb_from_result(const Result &in,
                                       const int2 size,
                                       std::vector<float> &r,
                                       std::vector<float> &g,
                                       std::vector<float> &b)
{
  const int n = size.x * size.y;
  r.assign(size_t(n), 0.0f);
  g.assign(size_t(n), 0.0f);
  b.assign(size_t(n), 0.0f);
  if (!in.is_allocated() && !in.is_single_value()) {
    return;
  }
  auto accept = [](const Color &c, float &ro, float &go, float &bo) {
    ro = math::max(c.r, 0.0f);
    go = math::max(c.g, 0.0f);
    bo = math::max(c.b, 0.0f);
    const float peak = math::max(ro, math::max(go, bo));
    /* Fully transparent AND no RGB → no dye. But many Image Process / shader
     * textures ship unassociated color with a=0 (or tiny a) while RGB is valid.
     * Zeroing those was a pure-black seed for fluid. */
    if (c.a < 1e-4f) {
      if (peak < 1e-6f) {
        ro = go = bo = 0.0f;
      }
      /* else: keep RGB as unassociated dye (treat as opaque paint). */
      return;
    }
    /* If rgb looks unassociated (any channel >> a), re-premultiply. */
    if (peak > c.a * 1.01f + 1e-4f && c.a < 0.999f) {
      ro *= c.a;
      go *= c.a;
      bo *= c.a;
    }
  };
  if (in.is_single_value()) {
    float ro = 0.0f, go = 0.0f, bo = 0.0f;
    if (in.type() == ResultType::Color) {
      accept(in.get_single_value_default<Color>(), ro, go, bo);
    }
    else if (in.type() == ResultType::Float) {
      ro = go = bo = math::max(in.get_single_value_default<float>(), 0.0f);
    }
    std::fill(r.begin(), r.end(), ro);
    std::fill(g.begin(), g.end(), go);
    std::fill(b.begin(), b.end(), bo);
    return;
  }
  /* Scale from input domain → destination grid (sim res), not 1:1 pixel crop. */
  const int2 src_size = math::max(in.domain().data_size, int2(1));
  for (int y = 0; y < size.y; y++) {
    for (int x = 0; x < size.x; x++) {
      const int2 texel = map_dst_to_src_texel(x, y, size, src_size);
      const int i = y * size.x + x;
      if (in.type() == ResultType::Color) {
        accept(in.load_pixel<Color>(texel), r[size_t(i)], g[size_t(i)], b[size_t(i)]);
      }
      else if (in.type() == ResultType::Float) {
        const float v = math::max(in.load_pixel<float>(texel), 0.0f);
        r[size_t(i)] = g[size_t(i)] = b[size_t(i)] = v;
      }
    }
  }
}

static uint64_t fluid_solver_key(const NodeImageFluidSimOutput &s)
{
  const uint64_t pressure_key = get_default_hash(
      s.pressure_vcycles, s.pressure_iterations, s.pressure_smooth_iterations);
  const uint64_t a = get_default_hash(
      pressure_key, s.viscosity, s.dissipation, s.buoyancy, s.vorticity);
  const uint64_t b = get_default_hash(
      s.dt_scale, s.sim_resolution, s.dye_resolution);
  const uint64_t c = get_default_hash(int(s.boundary_left),
                                      int(s.boundary_right),
                                      int(s.boundary_bottom),
                                      int(s.boundary_top),
                                      int(s.warm_start_pressure));
  return get_default_hash(a, b, c);
}

/** Params that change grid size require dropping GPU textures. */
static uint64_t fluid_resolution_key(const NodeImageFluidSimOutput &s)
{
  return get_default_hash(s.sim_resolution, s.dye_resolution);
}

/**
 * Domain-normalized Velocity socket → Pavel texel units.
 * User's velocity is taken as-is — one domain unit maps to one grid cell per second.
 * Minimal clamp only to prevent floating-point explosion.
 */
static float fluid_domain_vel_to_texel(const float domain_v, const int sim_edge)
{
  const float to_texel = float(math::max(sim_edge, 1));
  return math::clamp(domain_v * to_texel, -200.0f, 200.0f);
}

/** Zone dual-res resize 鈥?shipped helper also used by selftest feedback path. */
static void resize_field(const std::vector<float> &src,
                         const int sw,
                         const int sh,
                         std::vector<float> &dst,
                         const int dw,
                         const int dh)
{
  resize_scalar_field(src, sw, sh, dst, dw, dh);
}

/** Match script.js getResolution(resolution) for canvas aspect. */
static void fluid_sim_size(const int domain_w,
                           const int domain_h,
                           const int sim_resolution,
                           int &r_sw,
                           int &r_sh)
{
  /* getResolution: let aspectRatio = canvas.width / canvas.height
   * if (aspectRatio < 1) { min = resolution; max = round(resolution / aspectRatio) }
   * else { min = resolution; max = round(resolution * aspectRatio) } */
  const int res = math::clamp(sim_resolution, 16, 2048);
  if (domain_w <= 0 || domain_h <= 0) {
    r_sw = res;
    r_sh = res;
    return;
  }
  float aspect = float(domain_w) / float(domain_h);
  int mn = res;
  int mx = int(std::round(float(res) * (aspect < 1.0f ? (1.0f / aspect) : aspect)));
  mx = math::max(mx, mn);
  if (domain_w >= domain_h) {
    r_sw = mx;
    r_sh = mn;
  }
  else {
    r_sw = mn;
    r_sh = mx;
  }
}

static void write_velocity_result(Context &context,
                                  Result &out,
                                  const Domain &domain,
                                  const std::vector<float> &vx,
                                  const std::vector<float> &vy)
{
  if (!out.should_compute()) {
    return;
  }
  const int2 size = domain.data_size;
  if (size.x <= 0 || size.y <= 0) {
    return;
  }
  /* Prefer Float2 when possible; Color is a safe fallback for viewers. */
  Result cpu = context.create_result(ResultType::Float2);
  cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
  for (int y = 0; y < size.y; y++) {
    for (int x = 0; x < size.x; x++) {
      const size_t i = size_t(y * size.x + x);
      const float x_v = (i < vx.size()) ? vx[i] : 0.0f;
      const float y_v = (i < vy.size()) ? vy[i] : 0.0f;
      cpu.store_pixel(int2(x, y), float2(x_v, y_v));
    }
  }
  if (context.use_gpu()) {
    Result gpu = cpu.upload_to_gpu(true);
    share_result_safe(out, gpu);
    gpu.release();
  }
  else {
    share_result_safe(out, cpu);
  }
  cpu.release();
}

static const NodeImageFluidSimOutput *fluid_output_storage(const bNode &node)
{
  if (node.is_type("ImageNodeFluidSimOutput"_ustr)) {
    return static_cast<const NodeImageFluidSimOutput *>(node.storage);
  }
  if (node.is_type("ImageNodeFluidSimInput"_ustr)) {
    const auto &in = *static_cast<const NodeImageFluidSimInput *>(node.storage);
    if (const bNode *out = node.owner_tree().node_by_id(in.output_node_id)) {
      return static_cast<const NodeImageFluidSimOutput *>(out->storage);
    }
  }
  return nullptr;
}

static Vector<std::string> color_socket_ids(const bNode &node)
{
  Vector<std::string> ids;
  if (const NodeImageFluidSimOutput *storage = fluid_output_storage(node)) {
    for (const int i : IndexRange(storage->color_items_num)) {
      ids.append(FluidColorItemsAccessor::socket_identifier_for_item(storage->color_items[i]));
    }
  }
  if (ids.is_empty()) {
    ids.append("Color_0");
  }
  return ids;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Fluid Output 鈥?black-box step + cache store
 * \{ */

class FluidSimOutputOperation : public NodeOperation {
 public:
  FluidSimOutputOperation(Context &context, const bNode &node) : NodeOperation(context, node)
  {
    for (const std::string &id : color_socket_ids(node)) {
      InputDescriptor &desc = this->get_input_descriptor(id);
      desc.realization_mode = InputRealizationMode::None;
    }
    for (const char *id : {"Velocity",
                           "Temperature",
                           "Divergence",
                           "Collision Mask",
                           "Collider Velocity",
                           "Air Mask"})
    {
      InputDescriptor &desc = this->get_input_descriptor(id);
      desc.realization_mode = InputRealizationMode::None;
    }
  }

  Domain compute_domain() override
  {
    return fluid_domain(this->context());
  }

  void write_outputs_from_state(const Domain &domain,
                                const FluidFrameState &state,
                                const Vector<std::string> &color_ids,
                                const std::vector<float> &mask,
                                const std::vector<float> &cvel_x,
                                const std::vector<float> &cvel_y,
                                const std::vector<float> &air = {})
  {
    const int2 size = domain.data_size;
    const int sw = math::max(state.width, 1);
    const int sh = math::max(state.height, 1);
    const int dw = state.dye_width > 0 ? state.dye_width : sw;
    const int dh = state.dye_height > 0 ? state.dye_height : sh;

    for (int ci = 0; ci < int(color_ids.size()); ci++) {
      const std::string &id = color_ids[ci];
      if (!this->node().output_by_identifier(UString(id.c_str()))) {
        continue;
      }
      Result &out = this->get_result(id);
      if (!out.should_compute()) {
        continue;
      }
      const int ir = dye_ch_r(ci);
      const int ig = dye_ch_g(ci);
      const int ib = dye_ch_b(ci);
      if (ir < int(state.colors.size()) && ig < int(state.colors.size()) &&
          ib < int(state.colors.size()))
      {
        std::vector<float> up_r, up_g, up_b;
        /* Legacy caches: single grayscale channel per socket. */
        if (int(state.colors.size()) == state.color_count && state.color_count > 0) {
          resize_field(state.colors[size_t(ci)], sw, sh, up_r, size.x, size.y);
          write_color_result_rgb(this->context(), out, domain, up_r, up_r, up_r);
        }
        else {
          resize_field(state.colors[size_t(ir)], dw, dh, up_r, size.x, size.y);
          resize_field(state.colors[size_t(ig)], dw, dh, up_g, size.x, size.y);
          resize_field(state.colors[size_t(ib)], dw, dh, up_b, size.x, size.y);
          write_color_result_rgb(this->context(), out, domain, up_r, up_g, up_b);
        }
      }
    }
    if (this->node().output_by_identifier("Velocity"_ustr)) {
      std::vector<float> cell_vx, cell_vy, up_vx, up_vy;
      if (int(state.vel_x.size()) == (sw + 1) * sh &&
          int(state.vel_y.size()) == sw * (sh + 1))
      {
        mac_velocity_to_cell(state.vel_x, state.vel_y, sw, sh, cell_vx, cell_vy);
      }
      else {
        cell_vx = state.vel_x;
        cell_vy = state.vel_y;
      }
      /* Output domain-normalized: reverse cells/sec 鈫?domain/sec. */
      const float inv_sx = 1.0f / float(std::max(sw, 1));
      const float inv_sy = 1.0f / float(std::max(sh, 1));
      for (size_t i = 0; i < cell_vx.size(); i++) {
        cell_vx[i] *= inv_sx;
        cell_vy[i] *= inv_sy;
      }
      resize_field(cell_vx, sw, sh, up_vx, size.x, size.y);
      resize_field(cell_vy, sw, sh, up_vy, size.x, size.y);
      write_velocity_result(
          this->context(), this->get_result("Velocity"), domain, up_vx, up_vy);
    }
    if (this->node().output_by_identifier("Temperature"_ustr)) {
      std::vector<float> up_t;
      resize_field(state.temperature, sw, sh, up_t, size.x, size.y);
      write_float_result(this->context(), this->get_result("Temperature"), domain, up_t);
    }
    if (this->node().output_by_identifier("Divergence"_ustr)) {
      std::vector<float> up_d;
      resize_field(state.divergence, sw, sh, up_d, size.x, size.y);
      write_float_result(this->context(), this->get_result("Divergence"), domain, up_d);
    }
    if (this->node().output_by_identifier("Collision Mask"_ustr) && !mask.empty()) {
      write_float_result(this->context(), this->get_result("Collision Mask"), domain, mask);
    }
    if (this->node().output_by_identifier("Collider Velocity"_ustr) && !cvel_x.empty()) {
      write_velocity_result(this->context(),
                            this->get_result("Collider Velocity"),
                            domain,
                            cvel_x,
                            cvel_y);
    }
    if (this->node().output_by_identifier("Air Mask"_ustr) && !air.empty()) {
      write_float_result(this->context(), this->get_result("Air Mask"), domain, air);
    }
  }

  /* -------------------------------------------------------------------- */
  /** \name GPU realtime fluid (collocated WebGL-style compute passes)
   * \{ */

  Result gpu_upload_color_rgb(const Domain &domain,
                              const std::vector<float> &r,
                              const std::vector<float> &g,
                              const std::vector<float> &b)
  {
    Result cpu = this->context().create_result(ResultType::Color);
    cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    const int2 size = domain.data_size;
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        const size_t i = size_t(y * size.x + x);
        const float rv = i < r.size() ? r[i] : 0.0f;
        const float gv = i < g.size() ? g[i] : 0.0f;
        const float bv = i < b.size() ? b[i] : 0.0f;
        cpu.store_pixel(int2(x, y), fluid_dye_color(rv, gv, bv));
      }
    }
    Result gpu = cpu.upload_to_gpu(true);
    cpu.release();
    return gpu;
  }

  Result gpu_upload_vel(const Domain &domain,
                        const std::vector<float> &vx,
                        const std::vector<float> &vy)
  {
    Result cpu = this->context().create_result(ResultType::Color);
    cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    const int2 size = domain.data_size;
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        const size_t i = size_t(y * size.x + x);
        const float u = i < vx.size() ? vx[i] : 0.0f;
        const float v = i < vy.size() ? vy[i] : 0.0f;
        cpu.store_pixel(int2(x, y), Color(u, v, 0.0f, 1.0f));
      }
    }
    Result gpu = cpu.upload_to_gpu(true);
    cpu.release();
    return gpu;
  }

  Result gpu_upload_scalar(const Domain &domain, const std::vector<float> &f)
  {
    Result cpu = this->context().create_result(ResultType::Color);
    cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    const int2 size = domain.data_size;
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        const size_t i = size_t(y * size.x + x);
        const float s = i < f.size() ? f[i] : 0.0f;
        cpu.store_pixel(int2(x, y), Color(s, 0.0f, 0.0f, 1.0f));
      }
    }
    Result gpu = cpu.upload_to_gpu(true);
    cpu.release();
    return gpu;
  }

  void gpu_download_vel(const Result &gpu_vel,
                        const int2 size,
                        std::vector<float> &vx,
                        std::vector<float> &vy)
  {
    Result cpu = gpu_vel.download_to_cpu();
    vx.resize(size_t(size.x * size.y));
    vy.resize(size_t(size.x * size.y));
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        const Color c = cpu.load_pixel<Color>(int2(x, y));
        const size_t i = size_t(y * size.x + x);
        vx[i] = c.r;
        vy[i] = c.g;
      }
    }
    cpu.release();
  }

  void gpu_download_dye(const Result &gpu_dye,
                        const int2 size,
                        std::vector<float> &r,
                        std::vector<float> &g,
                        std::vector<float> &b)
  {
    Result cpu = gpu_dye.download_to_cpu();
    r.resize(size_t(size.x * size.y));
    g.resize(size_t(size.x * size.y));
    b.resize(size_t(size.x * size.y));
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        const Color c = cpu.load_pixel<Color>(int2(x, y));
        const size_t i = size_t(y * size.x + x);
        r[i] = c.r;
        g[i] = c.g;
        b[i] = c.b;
      }
    }
    cpu.release();
  }

  void gpu_download_scalar(const Result &gpu, const int2 size, std::vector<float> &f)
  {
    Result cpu = gpu.download_to_cpu();
    f.resize(size_t(size.x * size.y));
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        f[size_t(y * size.x + x)] = cpu.load_pixel<Color>(int2(x, y)).r;
      }
    }
    cpu.release();
  }
  bool gpu_shaders_ok()
  {
    /* Only shaders used by WebGL-Fluid-Simulation step(dt) + Blender I/O. */
    /* Shaders required for a valid Pavel step + continuous force inject. */
    static const char *names[] = {
        "compositor_fluid_pavel_advect",
        "compositor_fluid_pavel_advect_scalar",
        "compositor_fluid_pavel_advect_velocity",
        "compositor_fluid_pavel_add_velocity",
        "compositor_fluid_pavel_apply_obstacle",
        "compositor_fluid_pavel_clear",
        "compositor_fluid_pavel_curl",
        "compositor_fluid_pavel_divergence",
        "compositor_fluid_pavel_gradient",
        "compositor_fluid_pavel_mask_scalar",
        "compositor_fluid_pavel_pressure",
        "compositor_fluid_pavel_resample_color",
        "compositor_fluid_pavel_resample_float",
        "compositor_fluid_pavel_resample_float2",
        "compositor_fluid_pavel_vorticity",
        /* Multigrid pressure solve (replaces Pavel Jacobi). */
        "compositor_fluid_jacobi",
        "compositor_fluid_residual",
        "compositor_fluid_restrict",
        "compositor_fluid_prolong",
    };
    for (const char *name : names) {
      if (this->context().get_shader(name) == nullptr) {
        return false;
      }
    }
    return true;
  }

  static void gpu_ensure_texture(Result &dst, const Domain &domain)
  {
    if (dst.is_allocated()) {
      if (dst.domain().data_size == domain.data_size && dst.type() == ResultType::Color) {
        return;
      }
      dst.free();
    }
    if (dst.type() != ResultType::Color) {
      dst.set_type(ResultType::Color);
    }
    dst.allocate_texture(domain, false);
  }

  static void gpu_ensure_texture_typed(Result &dst,
                                       const Domain &domain,
                                       const ResultType type)
  {
    if (dst.is_allocated()) {
      if (dst.domain().data_size == domain.data_size && dst.type() == type) {
        return;
      }
      dst.free();
    }
    if (dst.type() != type) {
      dst.set_type(type);
    }
    dst.allocate_texture(domain, false);
  }

  static void gpu_move(Result &slot, Result &src)
  {
    if (!src.is_allocated()) {
      return;
    }
    if (slot.is_allocated()) {
      slot.free();
    }
    slot.share_data(src);
    src.release();
  }

  Result gpu_upload_scalar_field(const Domain &domain, const std::vector<float> &f)
  {
    Result cpu = this->context().create_result(ResultType::Color);
    cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    const int2 size = domain.data_size;
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        const size_t i = size_t(y * size.x + x);
        const float s = i < f.size() ? f[i] : 0.0f;
        cpu.store_pixel(int2(x, y), Color(s, 0.0f, 0.0f, 1.0f));
      }
    }
    Result gpu = cpu.upload_to_gpu(true);
    cpu.release();
    return gpu;
  }

  void gpu_download_scalar_field(const Result &gpu, const int2 size, std::vector<float> &f)
  {
    Result cpu = gpu.download_to_cpu();
    f.resize(size_t(size.x) * size_t(size.y));
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        f[size_t(y * size.x + x)] = cpu.load_pixel<Color>(int2(x, y)).r;
      }
    }
    cpu.release();
  }

  /** After compute imageStore, next pass may sample as texture — barrier is mandatory on Vulkan. */
  static void gpu_fluid_post_dispatch_barrier()
  {
    GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS | GPU_BARRIER_TEXTURE_FETCH |
                       GPU_BARRIER_TEXTURE_UPDATE);
  }

  bool gpu_dispatch(const char *name,
                    Result &out,
                    const Domain &domain,
                    const std::function<void(gpu::Shader *)> &bind_inputs)
  {
    gpu::Shader *shader = this->context().get_shader(name);
    if (!shader) {
      return false;
    }
    gpu_ensure_texture(out, domain);
    GPU_shader_bind(shader);
    bind_inputs(shader);
    out.bind_as_image(shader, "output_img");
    compute_dispatch_threads_at_least(shader, domain.data_size);
    out.unbind_as_image();
    GPU_shader_unbind();
    gpu_fluid_post_dispatch_barrier();
    return true;
  }

  bool gpu_dispatch_typed(const char *name,
                          Result &out,
                          const Domain &domain,
                          const ResultType type,
                          const std::function<void(gpu::Shader *)> &bind_inputs)
  {
    gpu::Shader *shader = this->context().get_shader(name);
    if (!shader) {
      return false;
    }
    gpu_ensure_texture_typed(out, domain, type);
    GPU_shader_bind(shader);
    bind_inputs(shader);
    out.bind_as_image(shader, "output_img");
    compute_dispatch_threads_at_least(shader, domain.data_size);
    out.unbind_as_image();
    GPU_shader_unbind();
    gpu_fluid_post_dispatch_barrier();
    return true;
  }

  Result gpu_upload_float_field(const Domain &domain, const std::vector<float> &values)
  {
    Result cpu = this->context().create_result(ResultType::Float);
    cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    const int2 size = domain.data_size;
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        const size_t i = size_t(y * size.x + x);
        cpu.store_pixel(int2(x, y), i < values.size() ? values[i] : 0.0f);
      }
    }
    Result gpu = cpu.upload_to_gpu(true);
    cpu.release();
    return gpu;
  }

  Result gpu_upload_float2_field(const Domain &domain,
                                 const std::vector<float> &x_values,
                                 const std::vector<float> &y_values)
  {
    Result cpu = this->context().create_result(ResultType::Float2);
    cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    const int2 size = domain.data_size;
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        const size_t i = size_t(y * size.x + x);
        cpu.store_pixel(int2(x, y),
                        float2(i < x_values.size() ? x_values[i] : 0.0f,
                               i < y_values.size() ? y_values[i] : 0.0f));
      }
    }
    Result gpu = cpu.upload_to_gpu(true);
    cpu.release();
    return gpu;
  }

  void gpu_download_float_field(const Result &gpu,
                                const int2 size,
                                std::vector<float> &values)
  {
    Result cpu = gpu.download_to_cpu();
    values.resize(size_t(size.x) * size_t(size.y));
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        values[size_t(y * size.x + x)] = cpu.load_pixel<float>(int2(x, y));
      }
    }
    cpu.release();
  }

  void gpu_download_float2_field(const Result &gpu,
                                 const int2 size,
                                 std::vector<float> &x_values,
                                 std::vector<float> &y_values)
  {
    Result cpu = gpu.download_to_cpu();
    x_values.resize(size_t(size.x) * size_t(size.y));
    y_values.resize(size_t(size.x) * size_t(size.y));
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        const size_t i = size_t(y * size.x + x);
        const float2 value = cpu.load_pixel<float2>(int2(x, y));
        x_values[i] = value.x;
        y_values[i] = value.y;
      }
    }
    cpu.release();
  }

  bool gpu_pavel_pressure_smooth(Result &pressure,
                                 Result &rhs,
                                 Result &tmp,
                                 const Domain &domain,
                                 const int iterations)
  {
    if (iterations <= 0) {
      return true;
    }
    gpu_ensure_texture_typed(pressure, domain, ResultType::Float);
    gpu_ensure_texture_typed(tmp, domain, ResultType::Float);

    Result *read = &pressure;
    Result *write = &tmp;
    for (int iteration = 0; iteration < iterations; iteration++) {
      if (!gpu_dispatch_typed("compositor_fluid_pavel_pressure",
                              *write,
                              domain,
                              ResultType::Float,
                              [&](gpu::Shader *shader) {
                                read->bind_as_texture(shader, "pressure_tx");
                                rhs.bind_as_texture(shader, "divergence_tx");
                              }))
      {
        return false;
      }
      read->unbind_as_texture();
      rhs.unbind_as_texture();
      std::swap(read, write);
    }
    /* Final result must live in `pressure` (odd iters leave it in tmp). */
    if (read != &pressure) {
      if (!pressure.is_allocated() || !tmp.is_allocated() || !pressure.gpu_texture() ||
          !tmp.gpu_texture())
      {
        return false;
      }
      GPU_texture_copy(pressure.gpu_texture(), tmp.gpu_texture());
      gpu_fluid_post_dispatch_barrier();
    }
    return true;
  }

  /**
   * Geometric Multigrid V-cycle pressure solve using GPU Gems Ch.38 shaders.
   * Replaces the Pavel single-level Jacobi for much better convergence.
   *   V-cycle: pre-smooth → residual → restrict → recurse → prolong → post-smooth
   */
  bool gpu_multigrid_pressure_solve(Result &pressure,
                                    Result &rhs,
                                    Result &tmp,
                                    const Domain &domain,
                                    const NodeImageFluidSimOutput &storage)
  {
    const int vcycles = math::clamp(storage.pressure_vcycles, 1, 6);
    const int pre = math::clamp(storage.pressure_smooth_iterations, 1, 8);
    const int post = pre;
    const int coarse_iters = math::max(storage.pressure_iterations, 1);
    const int2 size = domain.data_size;

    /* Build level hierarchy: halve until min dimension < 8. */
    struct Level {
      int2 size;
      Result pressure;
      Result rhs;
    };
    Vector<Level> levels;
    levels.append({size, Result(this->context().create_result(ResultType::Float)),
                   Result(this->context().create_result(ResultType::Float))});

    int2 cur = size;
    while (cur.x >= 8 && cur.y >= 8) {
      cur = math::max(cur / 2, int2(4));
      levels.append({cur, Result(this->context().create_result(ResultType::Float)),
                     Result(this->context().create_result(ResultType::Float))});
    }

    auto release_levels = [&]() {
      for (Level &lvl : levels) {
        if (lvl.pressure.is_allocated()) lvl.pressure.release();
        if (lvl.rhs.is_allocated()) lvl.rhs.release();
      }
    };

    /* CPU temp for resizing init (uniform 0 fill). */
    {
      Result cpu_zero = this->context().create_result(ResultType::Float);
      cpu_zero.allocate_texture(Domain(int2(4)), false, ResultStorageType::CPUImage);
      for (int y = 0; y < 4; y++)
        for (int x = 0; x < 4; x++)
          cpu_zero.store_pixel(int2(x, y), 0.0f);
      Result gpu_zero = cpu_zero.upload_to_gpu(true);
      for (int li = 1; li < int(levels.size()); li++) {
        Domain ldom(levels[li].size);
        gpu_ensure_texture_typed(levels[li].pressure, ldom, ResultType::Float);
        gpu_ensure_texture_typed(levels[li].rhs, ldom, ResultType::Float);
        /* Fill with zeros — not strictly needed after ensure but safe. */
      }
      gpu_zero.release();
      cpu_zero.release();
    }

    /* Jacobi sweep helper using GPU Gems Jacobi shader. */
    auto jacobi_sweep = [&](Result &p, Result &b, const Domain &dom, int iters) -> bool {
      Result *src = &p, *dst = &tmp;
      gpu_ensure_texture_typed(tmp, dom, ResultType::Float);
      for (int i = 0; i < iters; i++) {
        if (!gpu_dispatch_typed("compositor_fluid_jacobi",
                                *dst, dom, ResultType::Float,
                                [&](gpu::Shader *shader) {
                                  GPU_shader_uniform_1f(shader, "alpha", -1.0f);
                                  GPU_shader_uniform_1f(shader, "rBeta", 0.25f);
                                  src->bind_as_texture(shader, "x_tx");
                                  b.bind_as_texture(shader, "b_tx");
                                }))
          return false;
        src->unbind_as_texture();
        b.unbind_as_texture();
        std::swap(src, dst);
      }
      if (src != &p) {
        GPU_texture_copy(p.gpu_texture(), tmp.gpu_texture());
        gpu_fluid_post_dispatch_barrier();
      }
      return true;
    };

    /* Recursive V-cycle. */
    std::function<bool(int)> vcycle = [&](int li) -> bool {
      Level &lvl = levels[li];
      Domain ldom(lvl.size);

      if (li == int(levels.size()) - 1) {
        /* Coarsest level: many Jacobi iterations. */
        return jacobi_sweep(lvl.pressure, lvl.rhs, ldom, coarse_iters);
      }

      /* Pre-smooth. */
      if (!jacobi_sweep(lvl.pressure, lvl.rhs, ldom, pre)) return false;

      /* Residual: r = b - A·p. */
      if (!gpu_dispatch_typed("compositor_fluid_residual",
                              tmp, ldom, ResultType::Float,
                              [&](gpu::Shader *shader) {
                                lvl.pressure.bind_as_texture(shader, "x_tx");
                                lvl.rhs.bind_as_texture(shader, "b_tx");
                              }))
        return false;
      lvl.pressure.unbind_as_texture();
      lvl.rhs.unbind_as_texture();

      /* Restrict residual to next level. */
      Level &next = levels[li + 1];
      Domain ndom(next.size);
      if (!gpu_dispatch_typed("compositor_fluid_restrict",
                              next.rhs, ndom, ResultType::Float,
                              [&](gpu::Shader *shader) {
                                tmp.bind_as_texture(shader, "fine_tx");
                              }))
        return false;
      tmp.unbind_as_texture();

      /* Zero the coarse pressure correction. */
      {
        Result cpu_z = this->context().create_result(ResultType::Float);
        cpu_z.allocate_texture(ndom, false, ResultStorageType::CPUImage);
        for (int y = 0; y < next.size.y; y++)
          for (int x = 0; x < next.size.x; x++)
            cpu_z.store_pixel(int2(x, y), 0.0f);
        Result gpu_z = cpu_z.upload_to_gpu(true);
        gpu_ensure_texture_typed(next.pressure, ndom, ResultType::Float);
        GPU_texture_copy(next.pressure.gpu_texture(), gpu_z.gpu_texture());
        gpu_fluid_post_dispatch_barrier();
        gpu_z.release();
        cpu_z.release();
      }

      /* Recurse. */
      if (!vcycle(li + 1)) return false;

      /* Prolong correction back to this level. */
      if (!gpu_dispatch_typed("compositor_fluid_prolong",
                              tmp, ldom, ResultType::Float,
                              [&](gpu::Shader *shader) {
                                lvl.pressure.bind_as_texture(shader, "fine_tx");
                                next.pressure.bind_as_texture(shader, "coarse_tx");
                              }))
        return false;
      lvl.pressure.unbind_as_texture();
      next.pressure.unbind_as_texture();
      GPU_texture_copy(lvl.pressure.gpu_texture(), tmp.gpu_texture());
      gpu_fluid_post_dispatch_barrier();

      /* Post-smooth. */
      return jacobi_sweep(lvl.pressure, lvl.rhs, ldom, post);
    };

    /* Set up level 0 RHS and pressure. */
    {
      Domain l0(size);
      Level &l0lvl = levels[0];
      gpu_ensure_texture_typed(l0lvl.pressure, l0, ResultType::Float);
      gpu_ensure_texture_typed(l0lvl.rhs, l0, ResultType::Float);
      GPU_texture_copy(l0lvl.pressure.gpu_texture(), pressure.gpu_texture());
      GPU_texture_copy(l0lvl.rhs.gpu_texture(), rhs.gpu_texture());
      gpu_fluid_post_dispatch_barrier();
    }

    /* Run V-cycles. */
    for (int vc = 0; vc < vcycles; vc++) {
      if (!vcycle(0)) { release_levels(); return false; }
    }

    /* Copy result back to pressure. */
    GPU_texture_copy(pressure.gpu_texture(), levels[0].pressure.gpu_texture());
    gpu_fluid_post_dispatch_barrier();
    release_levels();
    return true;
  }

  /**
   * Pavel WebGL-Fluid-Simulation pressure solve only:
   *   for i in PRESSURE_ITERATIONS:
   *     p = (L+R+B+T - div) * 0.25
   * No Multigrid -- matches https://github.com/PavelDoGreat/WebGL-Fluid-Simulation step().
   */
  bool gpu_pavel_pressure_solve(Result &pressure,
                                Result &rhs,
                                Result &tmp,
                                const Domain &domain,
                                const NodeImageFluidSimOutput &storage)
  {
    /* script.js: PRESSURE_ITERATIONS default 20. */
    const int iterations = math::max(storage.pressure_iterations, 1);
    return gpu_pavel_pressure_smooth(pressure, rhs, tmp, domain, iterations);
  }

  /** Solid cells <- collider velocity (collocated). */
  bool gpu_pavel_apply_obstacle(Result &velocity,
                                Result &tmp,
                                Result &solid,
                                Result &collider_vel,
                                const Domain &domain)
  {
    if (!gpu_dispatch_typed("compositor_fluid_pavel_apply_obstacle",
                            tmp,
                            domain,
                            ResultType::Float2,
                            [&](gpu::Shader *shader) {
                              velocity.bind_as_texture(shader, "velocity_tx");
                              solid.bind_as_texture(shader, "solid_tx");
                              collider_vel.bind_as_texture(shader, "collider_vel_tx");
                            }))
    {
      return false;
    }
    velocity.unbind_as_texture();
    solid.unbind_as_texture();
    collider_vel.unbind_as_texture();
    gpu_move(velocity, tmp);
    gpu_ensure_texture_typed(tmp, domain, ResultType::Float2);
    return true;
  }

  /** mode: 0=solid|air→0, 1=air→0, 2=solid→0. */
  bool gpu_pavel_mask_scalar(Result &field,
                             Result &tmp,
                             Result &solid,
                             Result &air,
                             const Domain &domain,
                             const int mode)
  {
    if (!gpu_dispatch_typed("compositor_fluid_pavel_mask_scalar",
                            tmp,
                            domain,
                            ResultType::Float,
                            [&](gpu::Shader *shader) {
                              GPU_shader_uniform_1i(shader, "mode", mode);
                              field.bind_as_texture(shader, "input_tx");
                              solid.bind_as_texture(shader, "solid_tx");
                              air.bind_as_texture(shader, "air_tx");
                            }))
    {
      return false;
    }
    field.unbind_as_texture();
    solid.unbind_as_texture();
    air.unbind_as_texture();
    gpu_move(field, tmp);
    gpu_ensure_texture_typed(tmp, domain, ResultType::Float);
    return true;
  }

  /**
   * Direct port of Pavel Dobryakov WebGL Fluid Simulation GPU `step(dt)`:
   *   https://github.com/PavelDoGreat/WebGL-Fluid-Simulation
   * Pass order: curl → vorticity → divergence → pressure clear → Jacobi pressure →
   * gradient subtract → advect velocity → advect dye.
   * No Multigrid / no MAC custom solver — collocated RG velocity only.
   */
  bool execute_gpu_pavel()
  {
    const Domain display_domain = this->compute_domain();
    const int2 display = display_domain.data_size;
    if (display.x < 4 || display.y < 4) {
      return false;
    }

    const Scene &scene = this->context().get_scene();
    const int frame = scene.r.cfra;
    const int start_frame = scene.r.sfra;
    const uint session_uid = this->node().owner_tree().id.session_uid;
    const int32_t out_id = this->node().identifier;
    const auto &storage = *static_cast<const NodeImageFluidSimOutput *>(this->node().storage);

    int W, H, DW, DH;
    /* getResolution(SIM_RESOLUTION) / getResolution(DYE_RESOLUTION) from script.js. */
    fluid_sim_size(display.x, display.y, storage.sim_resolution, W, H);
    fluid_sim_size(display.x, display.y, storage.dye_resolution, DW, DH);
    W = math::max(W, 4);
    H = math::max(H, 4);
    DW = math::max(DW, 4);
    DH = math::max(DH, 4);
    const Domain sim_domain(int2(W, H));
    const Domain dye_domain(int2(DW, DH));

    const Vector<std::string> color_ids = color_socket_ids(this->node());
    const int color_count = math::max(int(color_ids.size()), 1);
    FluidZoneCache &cache = fluid_zone_caches().lookup_or_add_default(
        FluidCacheKey{session_uid, out_id});
    const uint64_t solver_key = fluid_solver_key(storage);
    if (cache.solver_key != 0 && cache.solver_key != solver_key) {
      /* Solver DNA changed. Keep current display, flag for reset at start frame. */
      cache.needs_param_reset = true;
      const bool res_ok = cache.gpu.velocity != nullptr && cache.gpu.width == W &&
                          cache.gpu.height == H && cache.gpu.dye_width == DW &&
                          cache.gpu.dye_height == DH;
      if (res_ok) {
        cache.hold_display = true;
      }
      else {
        cache.gpu.free_textures();
        cache.hold_display = false;
      }
    }
    if (cache.solver_key == 0) {
      cache.solver_key = solver_key;
    }

    /* Only force-reset when params changed AND user is at start frame.
     * First cook (solver_key was 0) never triggers reset.
     * Scrubbing back to frame 1 keeps all existing cache. */
    const bool have_gpu_snapshot = cache.gpu.velocity != nullptr && cache.gpu.width == W &&
                                   cache.gpu.height == H && cache.gpu.dye_width == DW &&
                                   cache.gpu.dye_height == DH && cache.gpu.frame != INT_MIN;
    const bool have_this_frame_gpu = have_gpu_snapshot && cache.gpu.frame == frame;
    const bool at_start = frame <= start_frame;
    const bool force_reset = cache.needs_param_reset && at_start && !have_this_frame_gpu;
    if (force_reset) {
      cache.dense.clear();
      cache.checkpoints.clear();
      cache.preview_interval.clear();
      cache.preview_interval_start = INT_MIN;
      cache.gpu.free_textures();
      cache.last_solved = INT_MIN;
      cache.hold_display = false;
      cache.needs_param_reset = false;
      cache.solver_key = 0;
      fluid_timeline_unregister_after(INT_MIN / 4);
    }

    /* Paused param tweak: same frame, hold_display set → re-blit GPU only, no step. */
    const bool display_only = cache.hold_display && have_gpu_snapshot &&
                              (cache.gpu.frame == frame || !at_start);

    auto release_if_allocated = [](Result &result) {
      if (result.is_allocated()) {
        result.release();
      }
    };

    auto pass_external_output = [&](const char *identifier) {
      if (!this->node().output_by_identifier(UString(identifier)) || !this->has_input(identifier)) {
        return;
      }
      Result &output = this->get_result(identifier);
      Result &input = this->get_input(identifier);
      if (!output.should_compute() || !(input.is_allocated() || input.is_single_value())) {
        return;
      }
      if (output.is_allocated()) {
        output.free();
      }
      output.share_data(input);
    };

    auto write_gpu_outputs = [&](Result &velocity,
                                 Vector<Result> &dyes,
                                 Result &temperature,
                                 Result &divergence) -> bool {
      for (const int color_index : color_ids.index_range()) {
        if (color_index >= dyes.size()) {
          continue;
        }
        const std::string &identifier = color_ids[color_index];
        if (!this->node().output_by_identifier(UString(identifier.c_str()))) {
          continue;
        }
        Result &output = this->get_result(identifier);
        if (!output.should_compute()) {
          continue;
        }
        /* Always resample to Image Process display domain so Viewer/backdrop
         * never receives a smaller dye buffer with mismatched domain metadata. */
        Result resized = this->context().create_result(ResultType::Color);
        if (!gpu_dispatch_typed("compositor_fluid_pavel_resample_color",
                                resized,
                                display_domain,
                                ResultType::Color,
                                [&](gpu::Shader *shader) {
                                  GPU_shader_uniform_1f(shader, "value_scale", 1.0f);
                                  dyes[color_index].bind_as_texture(shader, "input_tx");
                                }))
        {
          return false;
        }
        dyes[color_index].unbind_as_texture();
        share_result_safe(output, resized);
        resized.release();
      }

      if (this->node().output_by_identifier("Velocity"_ustr)) {
        Result &output = this->get_result("Velocity");
        if (output.should_compute()) {
          Result resized = this->context().create_result(ResultType::Float2);
          if (!gpu_dispatch_typed("compositor_fluid_pavel_resample_float2",
                                  resized,
                                  display_domain,
                                  ResultType::Float2,
                                  [&](gpu::Shader *shader) {
                                    GPU_shader_uniform_2f(shader,
                                                          "value_scale",
                                                          1.0f / float(W),
                                                          1.0f / float(H));
                                    velocity.bind_as_texture(shader, "input_tx");
                                  }))
          {
            return false;
          }
          velocity.unbind_as_texture();
          share_result_safe(output, resized);
          resized.release();
        }
      }

      auto write_scalar = [&](const char *identifier, Result &source) -> bool {
        if (!this->node().output_by_identifier(UString(identifier))) {
          return true;
        }
        Result &output = this->get_result(identifier);
        if (!output.should_compute()) {
          return true;
        }
        Result resized = this->context().create_result(ResultType::Float);
        if (!gpu_dispatch_typed("compositor_fluid_pavel_resample_float",
                                resized,
                                display_domain,
                                ResultType::Float,
                                [&](gpu::Shader *shader) {
                                  GPU_shader_uniform_1f(shader, "value_scale", 1.0f);
                                  source.bind_as_texture(shader, "input_tx");
                                }))
        {
          return false;
        }
        source.unbind_as_texture();
        share_result_safe(output, resized);
        resized.release();
        return true;
      };
      if (!write_scalar("Temperature", temperature) || !write_scalar("Divergence", divergence)) {
        return false;
      }
      pass_external_output("Collision Mask");
      pass_external_output("Collider Velocity");
      pass_external_output("Air Mask");
      return true;
    };

    auto copy_cached_state = [&](Result &velocity,
                                 Vector<Result> &dyes,
                                 Result &temperature,
                                 Result &divergence,
                                 Result &pressure) -> bool {
      const FluidGpuState &state = cache.gpu;
      if (!state.velocity || !state.pressure || !state.temperature || !state.divergence ||
          state.dyes.size() < color_count)
      {
        return false;
      }
      gpu_ensure_texture_typed(velocity, sim_domain, ResultType::Float2);
      gpu_ensure_texture_typed(pressure, sim_domain, ResultType::Float);
      gpu_ensure_texture_typed(temperature, sim_domain, ResultType::Float);
      gpu_ensure_texture_typed(divergence, sim_domain, ResultType::Float);
      GPU_texture_copy(velocity.gpu_texture(), state.velocity);
      GPU_texture_copy(pressure.gpu_texture(), state.pressure);
      GPU_texture_copy(temperature.gpu_texture(), state.temperature);
      GPU_texture_copy(divergence.gpu_texture(), state.divergence);
      for (const int color_index : IndexRange(color_count)) {
        Result dye = this->context().create_result(ResultType::Color);
        gpu_ensure_texture_typed(dye, dye_domain, ResultType::Color);
        GPU_texture_copy(dye.gpu_texture(), state.dyes[color_index]);
        dyes.append(std::move(dye));
      }
      return true;
    };

    Result velocity = this->context().create_result(ResultType::Float2);
    Result pressure = this->context().create_result(ResultType::Float);
    Result temperature = this->context().create_result(ResultType::Float);
    Result divergence = this->context().create_result(ResultType::Float);
    Vector<Result> dyes;
    dyes.reserve(color_count);

    const bool gpu_state_matches = cache.gpu.width == W && cache.gpu.height == H &&
                                   cache.gpu.dye_width == DW && cache.gpu.dye_height == DH;

    auto finish_display_only = [&]() -> bool {
      if (!copy_cached_state(velocity, dyes, temperature, divergence, pressure)) {
        return false;
      }
      const bool ok = write_gpu_outputs(velocity, dyes, temperature, divergence);
      release_if_allocated(velocity);
      release_if_allocated(pressure);
      release_if_allocated(temperature);
      release_if_allocated(divergence);
      for (Result &dye : dyes) {
        release_if_allocated(dye);
      }
      return ok;
    };

    /* Same-frame re-eval (paused scrub / param drag): never re-step. */
    if (!force_reset && gpu_state_matches && cache.gpu.frame == frame) {
      return finish_display_only();
    }
    /* Param drag while paused on a later frame: freeze last GPU image, no calc. */
    if (display_only && have_gpu_snapshot && cache.gpu.frame != frame) {
      /* hold_display but frame moved → real advance: clear hold and step below. */
      if (cache.gpu.frame < frame) {
        cache.hold_display = false;
      }
      else {
        return finish_display_only();
      }
    }
    else if (display_only && have_gpu_snapshot) {
      return finish_display_only();
    }

    /* Prefer GPU cache chain. Consecutive play: 1 step. Scrub gap: small catch-up
     * only (large multi-step was a major noise/explosion source). */
    int steps_to_run = 1;
    const bool have_gpu_base = !force_reset && gpu_state_matches && cache.gpu.velocity &&
                               cache.gpu.frame != INT_MIN && cache.gpu.frame < frame;
    bool body_velocity_loaded = false;
    if (have_gpu_base) {
      if (!copy_cached_state(velocity, dyes, temperature, divergence, pressure)) {
        return false;
      }
      /* Cap catch-up hard — 32 full pressure solves per cook was laggy and unstable. */
      steps_to_run = math::clamp(frame - cache.gpu.frame, 1, 4);
      cache.hold_display = false;

      /* Zone body Color/Velocity are the pre-step fields (Input feedback + user Mix/paint).
       * Previously we only stepped pure GPU cache and ignored Output Color inputs after the
       * first frame — new dye never entered the sim. Prefer body fields when allocated. */
      for (const int color_index : IndexRange(color_count)) {
        if (color_index >= color_ids.size() || !this->has_input(color_ids[color_index].c_str())) {
          continue;
        }
        Result &cin = this->get_input(color_ids[color_index].c_str());
        if (!(cin.is_allocated() || cin.is_single_value())) {
          continue;
        }
        Result cpu_storage = this->context().create_result(ResultType::Color);
        const Result *src = nullptr;
        ensure_cpu_color(this->context(), cin, cpu_storage, src);
        if (!src) {
          if (cpu_storage.is_allocated()) {
            cpu_storage.release();
          }
          continue;
        }
        std::vector<float> r, g, b;
        fill_color_rgb_from_result(*src, int2(DW, DH), r, g, b);
        if (cpu_storage.is_allocated()) {
          cpu_storage.release();
        }
        if (color_index < dyes.size()) {
          dyes[color_index].release();
        }
        while (dyes.size() <= color_index) {
          dyes.append(this->context().create_result(ResultType::Color));
        }
        dyes[color_index] = gpu_upload_color_rgb(dye_domain, r, g, b);
      }
      if (this->has_input("Velocity")) {
        Result &vin = this->get_input("Velocity");
        if (vin.is_allocated() || vin.is_single_value()) {
          Result cpu_storage = this->context().create_result(ResultType::Float2);
          const Result *src = nullptr;
          ensure_cpu_color(this->context(), vin, cpu_storage, src);
          if (src) {
            std::vector<float> cell_x(size_t(W * H), 0.0f), cell_y(size_t(W * H), 0.0f);
            fill_scalar_from_result(*src, int2(W, H), cell_x, 0);
            fill_scalar_from_result(*src, int2(W, H), cell_y, 1);
            const int sim_edge = math::max(W, H);
            for (size_t i = 0; i < cell_x.size(); i++) {
              cell_x[i] = fluid_domain_vel_to_texel(cell_x[i], sim_edge);
              cell_y[i] = fluid_domain_vel_to_texel(cell_y[i], sim_edge);
            }
            if (velocity.is_allocated()) {
              velocity.release();
            }
            velocity = gpu_upload_float2_field(sim_domain, cell_x, cell_y);
            body_velocity_loaded = true;
          }
          if (cpu_storage.is_allocated()) {
            cpu_storage.release();
          }
        }
      }
    }
    else {
      /* Dense MAC snapshot only for exact frame-1 (legacy CPU path). */
      const FluidFrameState *previous = nullptr;
      if (!force_reset) {
        previous = fluid_find_state(cache, frame - 1);
        if (previous && (previous->width != W || previous->height != H)) {
          previous = nullptr;
        }
      }

      if (previous) {
        std::vector<float> cell_x, cell_y;
        if (int(previous->vel_x.size()) == (W + 1) * H &&
            int(previous->vel_y.size()) == W * (H + 1))
        {
          mac_velocity_to_cell(previous->vel_x, previous->vel_y, W, H, cell_x, cell_y);
        }
        else {
          cell_x = previous->vel_x;
          cell_y = previous->vel_y;
        }
        velocity = gpu_upload_float2_field(sim_domain, cell_x, cell_y);
        pressure = gpu_upload_float_field(
            sim_domain,
            previous->pressure.empty() ? std::vector<float>(size_t(W * H), 0.0f) :
                                         previous->pressure);
        temperature = gpu_upload_float_field(
            sim_domain,
            previous->temperature.empty() ? std::vector<float>(size_t(W * H), 0.0f) :
                                            previous->temperature);
        divergence = gpu_upload_float_field(
            sim_domain,
            previous->divergence.empty() ? std::vector<float>(size_t(W * H), 0.0f) :
                                           previous->divergence);
        const int previous_dye_w = previous->dye_width > 0 ? previous->dye_width : W;
        const int previous_dye_h = previous->dye_height > 0 ? previous->dye_height : H;
        const bool rgb_layout = int(previous->colors.size()) >= color_count * 3;
        for (const int color_index : IndexRange(color_count)) {
          std::vector<float> r, g, b;
          if (rgb_layout) {
            resize_field(previous->colors[dye_ch_r(color_index)],
                         previous_dye_w,
                         previous_dye_h,
                         r,
                         DW,
                         DH);
            resize_field(previous->colors[dye_ch_g(color_index)],
                         previous_dye_w,
                         previous_dye_h,
                         g,
                         DW,
                         DH);
            resize_field(previous->colors[dye_ch_b(color_index)],
                         previous_dye_w,
                         previous_dye_h,
                         b,
                         DW,
                         DH);
          }
          else {
            if (color_index < previous->colors.size()) {
              resize_field(previous->colors[color_index],
                           previous_dye_w,
                           previous_dye_h,
                           r,
                           DW,
                           DH);
            }
            else {
              r.assign(size_t(DW * DH), 0.0f);
            }
            g = r;
            b = r;
          }
          dyes.append(gpu_upload_color_rgb(dye_domain, r, g, b));
        }
      }
      else {
        /* Cold seed ONCE: exterior Color/Velocity. Never re-enter this path while
         * GPU cache is valid for consecutive frames. */
        auto load_cpu_input = [&](const char *identifier,
                                  Result &cpu_storage,
                                  const Result *&result) {
          if (!this->has_input(identifier)) {
            return;
          }
          Result &input = this->get_input(identifier);
          if (!(input.is_allocated() || input.is_single_value())) {
            return;
          }
          ensure_cpu_color(this->context(), input, cpu_storage, result);
        };

        Result velocity_cpu = this->context().create_result(ResultType::Float2);
        Result temperature_cpu = this->context().create_result(ResultType::Float);
        const Result *velocity_input = nullptr;
        const Result *temperature_input = nullptr;
        load_cpu_input("Velocity", velocity_cpu, velocity_input);
        load_cpu_input("Temperature", temperature_cpu, temperature_input);
        std::vector<float> cell_x(size_t(W * H), 0.0f), cell_y(size_t(W * H), 0.0f);
        std::vector<float> temperature_values(size_t(W * H), 0.0f);
        if (velocity_input) {
          fill_scalar_from_result(*velocity_input, int2(W, H), cell_x, 0);
          fill_scalar_from_result(*velocity_input, int2(W, H), cell_y, 1);
        }
        if (temperature_input) {
          fill_scalar_from_result(*temperature_input, int2(W, H), temperature_values, 0);
        }
        /* Domain-normalized Velocity → texel units (no splat_force full-field gain). */
        const int sim_edge = math::max(W, H);
        for (size_t i = 0; i < cell_x.size(); i++) {
          cell_x[i] = fluid_domain_vel_to_texel(cell_x[i], sim_edge);
          cell_y[i] = fluid_domain_vel_to_texel(cell_y[i], sim_edge);
        }

        Vector<Result> color_cpu_storage;
        color_cpu_storage.reserve(color_count);
        for (const int color_index : IndexRange(color_count)) {
          Result cpu_storage = this->context().create_result(ResultType::Color);
          const Result *color_input = nullptr;
          load_cpu_input(color_ids[color_index].c_str(), cpu_storage, color_input);
          std::vector<float> r, g, b;
          if (color_input) {
            fill_color_rgb_from_result(*color_input, int2(DW, DH), r, g, b);
          }
          else {
            r.assign(size_t(DW * DH), 0.0f);
            g = r;
            b = r;
          }
          dyes.append(gpu_upload_color_rgb(dye_domain, r, g, b));
          color_cpu_storage.append(std::move(cpu_storage));
        }

        /* Cold seed only: mild dye-weighted swirl when Velocity is near-zero so
         * Color-only graphs still move. Keep strength small — old 80–400 range
         * instantly blew the field into noise. */
        float max_abs_v = 0.0f;
        for (size_t i = 0; i < cell_x.size(); i++) {
          max_abs_v = math::max(max_abs_v, math::max(math::abs(cell_x[i]), math::abs(cell_y[i])));
        }
        if (max_abs_v < 0.5f && !dyes.is_empty()) {
          /* Auto-impulse: gentle swirl when user provides dye but no velocity.
           * Hardcoded defaults — splat_radius/splat_force are vestigial WebGL demo params. */
          const float radius = 0.25f;
          const float strength = 12.0f;
          std::vector<float> lum_r, lum_g, lum_b;
          {
            Result cpu_storage = this->context().create_result(ResultType::Color);
            const Result *color_input = nullptr;
            load_cpu_input(color_ids[0].c_str(), cpu_storage, color_input);
            if (color_input) {
              fill_color_rgb_from_result(*color_input, int2(DW, DH), lum_r, lum_g, lum_b);
            }
            if (cpu_storage.is_allocated()) {
              cpu_storage.release();
            }
          }
          if (!lum_r.empty()) {
            for (int y = 0; y < H; y++) {
              for (int x = 0; x < W; x++) {
                const int dx = math::clamp((x * DW) / math::max(W, 1), 0, DW - 1);
                const int dy = math::clamp((y * DH) / math::max(H, 1), 0, DH - 1);
                const size_t di = size_t(dy * DW + dx);
                const float lum = (lum_r[di] + lum_g[di] + lum_b[di]) * (1.0f / 3.0f);
                if (lum < 0.03f) {
                  continue;
                }
                const float u = (float(x) + 0.5f) / float(W) - 0.5f;
                const float v = (float(y) + 0.5f) / float(H) - 0.5f;
                const float dist_sq = u * u + v * v;
                const float falloff = math::exp(-dist_sq / (2.0f * radius * radius));
                const size_t i = size_t(y * W + x);
                cell_x[i] += (-v * strength + u * strength * 0.15f) * lum * falloff;
                cell_y[i] += (u * strength + v * strength * 0.15f) * lum * falloff;
                cell_x[i] = math::clamp(cell_x[i], -80.0f, 80.0f);
                cell_y[i] = math::clamp(cell_y[i], -80.0f, 80.0f);
              }
            }
          }
        }

        velocity = gpu_upload_float2_field(sim_domain, cell_x, cell_y);
        pressure = gpu_upload_float_field(sim_domain, std::vector<float>(size_t(W * H), 0.0f));
        temperature = gpu_upload_float_field(sim_domain, temperature_values);
        divergence = gpu_upload_float_field(sim_domain, std::vector<float>(size_t(W * H), 0.0f));

        release_if_allocated(velocity_cpu);
        release_if_allocated(temperature_cpu);
        for (Result &cpu_storage : color_cpu_storage) {
          release_if_allocated(cpu_storage);
        }
        steps_to_run = 1;
      }
    }

    if (!velocity.is_allocated() || !pressure.is_allocated() || !temperature.is_allocated() ||
        dyes.size() < color_count)
    {
      release_if_allocated(velocity);
      release_if_allocated(pressure);
      release_if_allocated(temperature);
      release_if_allocated(divergence);
      for (Result &dye : dyes) {
        release_if_allocated(dye);
      }
      return false;
    }

    Result velocity_tmp = this->context().create_result(ResultType::Float2);
    Result pressure_tmp = this->context().create_result(ResultType::Float);
    Result curl = this->context().create_result(ResultType::Float);
    Result temperature_tmp = this->context().create_result(ResultType::Float);
    Result dye_tmp = this->context().create_result(ResultType::Color);
    gpu_ensure_texture_typed(velocity_tmp, sim_domain, ResultType::Float2);
    gpu_ensure_texture_typed(pressure_tmp, sim_domain, ResultType::Float);
    gpu_ensure_texture_typed(curl, sim_domain, ResultType::Float);
    gpu_ensure_texture_typed(temperature_tmp, sim_domain, ResultType::Float);
    gpu_ensure_texture_typed(dye_tmp, dye_domain, ResultType::Color);

    /* Collision/air: only load when sockets are actually connected (CPU upload kills 24fps). */
    const bool has_collision = this->has_input("Collision Mask") &&
                               (this->get_input("Collision Mask").is_allocated() ||
                                this->get_input("Collision Mask").is_single_value());
    const bool has_air = this->has_input("Air Mask") &&
                         (this->get_input("Air Mask").is_allocated() ||
                          this->get_input("Air Mask").is_single_value());
    Result solid_mask = this->context().create_result(ResultType::Float);
    Result air_mask = this->context().create_result(ResultType::Float);
    Result collider_vel = this->context().create_result(ResultType::Float2);
    if (has_collision || has_air) {
      auto load_mask_float = [&](const char *identifier, Result &out) {
        std::vector<float> values(size_t(W * H), 0.0f);
        if (this->has_input(identifier)) {
          Result &input = this->get_input(identifier);
          if (input.is_allocated() || input.is_single_value()) {
            Result cpu_storage = this->context().create_result(ResultType::Float);
            const Result *src = nullptr;
            ensure_cpu_color(this->context(), input, cpu_storage, src);
            if (src) {
              fill_scalar_from_result(*src, int2(W, H), values, 0);
            }
            if (cpu_storage.is_allocated()) {
              cpu_storage.release();
            }
          }
        }
        out = gpu_upload_float_field(sim_domain, values);
      };
      auto load_mask_float2 = [&](const char *identifier, Result &out) {
        std::vector<float> vx(size_t(W * H), 0.0f), vy(size_t(W * H), 0.0f);
        if (this->has_input(identifier)) {
          Result &input = this->get_input(identifier);
          if (input.is_allocated() || input.is_single_value()) {
            Result cpu_storage = this->context().create_result(ResultType::Float2);
            const Result *src = nullptr;
            ensure_cpu_color(this->context(), input, cpu_storage, src);
            if (src) {
              fill_scalar_from_result(*src, int2(W, H), vx, 0);
              fill_scalar_from_result(*src, int2(W, H), vy, 1);
            }
            if (cpu_storage.is_allocated()) {
              cpu_storage.release();
            }
          }
        }
        const int sim_edge = math::max(W, H);
        for (size_t i = 0; i < vx.size(); i++) {
          vx[i] = fluid_domain_vel_to_texel(vx[i], sim_edge);
          vy[i] = fluid_domain_vel_to_texel(vy[i], sim_edge);
        }
        out = gpu_upload_float2_field(sim_domain, vx, vy);
      };
      if (has_collision) {
        load_mask_float("Collision Mask", solid_mask);
        load_mask_float2("Collider Velocity", collider_vel);
      }
      else {
        solid_mask = gpu_upload_float_field(sim_domain, std::vector<float>(size_t(W * H), 0.0f));
        collider_vel = gpu_upload_float2_field(
            sim_domain, std::vector<float>(size_t(W * H), 0.0f), std::vector<float>(size_t(W * H), 0.0f));
      }
      if (has_air) {
        load_mask_float("Air Mask", air_mask);
      }
      else {
        air_mask = gpu_upload_float_field(sim_domain, std::vector<float>(size_t(W * H), 0.0f));
      }
    }

    Result exterior_force = this->context().create_result(ResultType::Float2);

    auto cleanup = [&]() {
      release_if_allocated(velocity);
      release_if_allocated(pressure);
      release_if_allocated(temperature);
      release_if_allocated(divergence);
      release_if_allocated(velocity_tmp);
      release_if_allocated(pressure_tmp);
      release_if_allocated(curl);
      release_if_allocated(temperature_tmp);
      release_if_allocated(dye_tmp);
      release_if_allocated(solid_mask);
      release_if_allocated(air_mask);
      release_if_allocated(collider_vel);
      release_if_allocated(exterior_force);
      for (Result &dye : dyes) {
        release_if_allocated(dye);
      }
    };
    auto fail = [&]() -> bool {
      cleanup();
      return false;
    };

    /* Optional Blender-only collider; not in original WebGL demo. */
    auto apply_collider = [&]() -> bool {
      if (!has_collision) {
        return true;
      }
      return gpu_pavel_apply_obstacle(
          velocity, velocity_tmp, solid_mask, collider_vel, sim_domain);
    };

    /* script.js calcDeltaTime: clamp to ~1/60. */
    const float fps = math::max(float(scene.r.frs_sec) / float(scene.r.frs_sec_base), 1e-6f);
    const float base_dt = math::min(1.0f / fps, 1.0f / 60.0f);
    const float scaled_dt = base_dt * math::max(storage.dt_scale, 0.0f);
    /* For dt_scale > 1, subdivide into at most 12 substeps so the slider
     * keeps working past ~4× without blowing up in a single giant step. */
    const int substeps = math::clamp(int(math::ceil(scaled_dt / base_dt)), 1, 12);
    const float dt = math::clamp(scaled_dt / float(substeps), 1e-4f, 0.05f);
    /* WebGL CURL default is 30; cap contribution so wild UI values cannot explode. */
    const float curl_str = math::clamp(storage.vorticity, 0.0f, 50.0f);
    const bool need_temp = this->node().output_by_identifier("Temperature"_ustr) &&
                           this->get_result("Temperature").should_compute();

    if (!apply_collider()) {
      return fail();
    }

    /* Continuous exterior Velocity as per-frame force (WebGL mouse splat analogue).
     * Without this, a single cold seed dies after one projection and dye freezes. */
    bool has_exterior_force = false;
    {
      std::vector<float> fx(size_t(W * H), 0.0f), fy(size_t(W * H), 0.0f);
      if (this->has_input("Velocity")) {
        Result &vin = this->get_input("Velocity");
        if (vin.is_allocated() || vin.is_single_value()) {
          Result cpu_storage = this->context().create_result(ResultType::Float2);
          const Result *src = nullptr;
          ensure_cpu_color(this->context(), vin, cpu_storage, src);
          if (src) {
            fill_scalar_from_result(*src, int2(W, H), fx, 0);
            fill_scalar_from_result(*src, int2(W, H), fy, 1);
            has_exterior_force = true;
          }
          if (cpu_storage.is_allocated()) {
            cpu_storage.release();
          }
        }
      }
      if (has_exterior_force) {
        const int sim_edge = math::max(W, H);
        float max_f = 0.0f;
        for (size_t i = 0; i < fx.size(); i++) {
          fx[i] = fluid_domain_vel_to_texel(fx[i], sim_edge);
          fy[i] = fluid_domain_vel_to_texel(fy[i], sim_edge);
          max_f = math::max(max_f, math::max(math::abs(fx[i]), math::abs(fy[i])));
        }
        /* Only inject when the field is non-trivial. */
        has_exterior_force = max_f > 1e-4f;
        if (has_exterior_force) {
          exterior_force = gpu_upload_float2_field(sim_domain, fx, fy);
        }
      }
    }
    /* Per-frame inject gain. Keep small — full-field inject every frame is not a
     * mouse splat and easily drives numerical blow-up. */
    const float inject_scale = 0.04f;

    /* --- step(dt) from WebGL-Fluid-Simulation/script.js (1..N times for scrub gaps) --- */
    const int total_steps = steps_to_run * substeps;
    /* Exterior Velocity force: inject ONCE per frame (not per substep).
     * Skip when zone body already provided a full Velocity field (feedback path). */
    if (has_exterior_force && !body_velocity_loaded) {
      if (!gpu_dispatch_typed("compositor_fluid_pavel_add_velocity",
                              velocity_tmp,
                              sim_domain,
                              ResultType::Float2,
                              [&](gpu::Shader *shader) {
                                GPU_shader_uniform_1f(shader, "force_scale", inject_scale);
                                velocity.bind_as_texture(shader, "velocity_tx");
                                exterior_force.bind_as_texture(shader, "force_tx");
                              }))
      {
        return fail();
      }
      velocity.unbind_as_texture();
      exterior_force.unbind_as_texture();
      gpu_move(velocity, velocity_tmp);
      gpu_ensure_texture_typed(velocity_tmp, sim_domain, ResultType::Float2);
    }
    for (int step_i = 0; step_i < total_steps; step_i++) {
      if (!gpu_dispatch_typed("compositor_fluid_pavel_curl",
                              curl,
                              sim_domain,
                              ResultType::Float,
                              [&](gpu::Shader *shader) {
                                velocity.bind_as_texture(shader, "velocity_tx");
                              }))
      {
        return fail();
      }
      velocity.unbind_as_texture();

      if (!gpu_dispatch_typed("compositor_fluid_pavel_vorticity",
                              velocity_tmp,
                              sim_domain,
                              ResultType::Float2,
                              [&](gpu::Shader *shader) {
                                GPU_shader_uniform_1f(shader, "curl_strength", curl_str);
                                GPU_shader_uniform_1f(shader, "timestep", dt);
                                velocity.bind_as_texture(shader, "velocity_tx");
                                curl.bind_as_texture(shader, "curl_tx");
                              }))
      {
        return fail();
      }
      velocity.unbind_as_texture();
      curl.unbind_as_texture();
      gpu_move(velocity, velocity_tmp);
      gpu_ensure_texture_typed(velocity_tmp, sim_domain, ResultType::Float2);

      if (!gpu_dispatch_typed("compositor_fluid_pavel_divergence",
                              divergence,
                              sim_domain,
                              ResultType::Float,
                              [&](gpu::Shader *shader) {
                                velocity.bind_as_texture(shader, "velocity_tx");
                              }))
      {
        return fail();
      }
      velocity.unbind_as_texture();
      if (has_collision || has_air) {
        if (!gpu_pavel_mask_scalar(divergence, pressure_tmp, solid_mask, air_mask, sim_domain, 0))
        {
          return fail();
        }
      }

      if (!gpu_dispatch_typed("compositor_fluid_pavel_clear",
                              pressure_tmp,
                              sim_domain,
                              ResultType::Float,
                              [&](gpu::Shader *shader) {
                                const float keep = storage.warm_start_pressure ?
                                                       math::clamp(storage.buoyancy, 0.0f, 1.0f) :
                                                       0.0f;
                                GPU_shader_uniform_1f(shader, "value", keep);
                                pressure.bind_as_texture(shader, "input_tx");
                              }))
      {
        return fail();
      }
      pressure.unbind_as_texture();
      gpu_move(pressure, pressure_tmp);
      gpu_ensure_texture_typed(pressure_tmp, sim_domain, ResultType::Float);
      if (has_air) {
        if (!gpu_pavel_mask_scalar(pressure, pressure_tmp, solid_mask, air_mask, sim_domain, 1)) {
          return fail();
        }
      }

      if (!gpu_multigrid_pressure_solve(pressure, divergence, pressure_tmp, sim_domain, storage)) {
        return fail();
      }
      if (has_air) {
        if (!gpu_pavel_mask_scalar(pressure, pressure_tmp, solid_mask, air_mask, sim_domain, 1)) {
          return fail();
        }
      }

      if (!gpu_dispatch_typed("compositor_fluid_pavel_gradient",
                              velocity_tmp,
                              sim_domain,
                              ResultType::Float2,
                              [&](gpu::Shader *shader) {
                                pressure.bind_as_texture(shader, "pressure_tx");
                                velocity.bind_as_texture(shader, "velocity_tx");
                              }))
      {
        return fail();
      }
      pressure.unbind_as_texture();
      velocity.unbind_as_texture();
      gpu_move(velocity, velocity_tmp);
      gpu_ensure_texture_typed(velocity_tmp, sim_domain, ResultType::Float2);

      if (!gpu_dispatch_typed("compositor_fluid_pavel_advect_velocity",
                              velocity_tmp,
                              sim_domain,
                              ResultType::Float2,
                              [&](gpu::Shader *shader) {
                                GPU_shader_uniform_1f(shader, "timestep", dt);
                                GPU_shader_uniform_1f(
                                    shader, "dissipation", math::max(storage.viscosity, 0.0f));
                                velocity.bind_as_texture(shader, "velocity_tx");
                              }))
      {
        return fail();
      }
      velocity.unbind_as_texture();
      gpu_move(velocity, velocity_tmp);
      gpu_ensure_texture_typed(velocity_tmp, sim_domain, ResultType::Float2);
      if (!apply_collider()) {
        return fail();
      }

      for (Result &dye : dyes) {
        if (!gpu_dispatch_typed("compositor_fluid_pavel_advect",
                                dye_tmp,
                                dye_domain,
                                ResultType::Color,
                                [&](gpu::Shader *shader) {
                                  GPU_shader_uniform_1f(shader, "timestep", dt);
                                  GPU_shader_uniform_1f(
                                      shader, "dissipation", math::max(storage.dissipation, 0.0f));
                                  velocity.bind_as_texture(shader, "velocity_tx");
                                  dye.bind_as_texture(shader, "source_tx");
                                }))
        {
          return fail();
        }
        velocity.unbind_as_texture();
        dye.unbind_as_texture();
        gpu_move(dye, dye_tmp);
        gpu_ensure_texture_typed(dye_tmp, dye_domain, ResultType::Color);
      }

      if (need_temp) {
        if (!gpu_dispatch_typed("compositor_fluid_pavel_advect_scalar",
                                temperature_tmp,
                                sim_domain,
                                ResultType::Float,
                                [&](gpu::Shader *shader) {
                                  GPU_shader_uniform_1f(shader, "timestep", dt);
                                  GPU_shader_uniform_1f(
                                      shader, "dissipation", math::max(storage.dissipation, 0.0f));
                                  velocity.bind_as_texture(shader, "velocity_tx");
                                  temperature.bind_as_texture(shader, "source_tx");
                                }))
        {
          return fail();
        }
        velocity.unbind_as_texture();
        temperature.unbind_as_texture();
        gpu_move(temperature, temperature_tmp);
        gpu_ensure_texture_typed(temperature_tmp, sim_domain, ResultType::Float);
      }
    }

    gpu_fluid_post_dispatch_barrier();
    {
      FluidGpuState &state = cache.gpu;
      const ResultPrecision prec = this->context().get_precision();
      const gpu::TextureFormat vel_fmt = Result::gpu_texture_format(ResultType::Float2, prec);
      const gpu::TextureFormat scalar_fmt = Result::gpu_texture_format(ResultType::Float, prec);
      const gpu::TextureFormat color_fmt = Result::gpu_texture_format(ResultType::Color, prec);
      /* Drop legacy F16 cache textures that cannot GPU_texture_copy with F32 Results. */
      if (state.velocity && GPU_texture_format(state.velocity) != vel_fmt) {
        state.free_textures();
      }
      const bool recreate = state.width != W || state.height != H || state.dye_width != DW ||
                            state.dye_height != DH || !state.velocity || !state.pressure ||
                            !state.temperature || !state.divergence ||
                            state.dyes.size() != color_count;
      if (recreate) {
        state.free_textures();
        /* MUST match Image Process Result Full precision formats.
         * GPU_texture_copy requires identical size+format — caching as F16 while
         * Results are F32 silently dropped velocity between frames (dye looked
         * frozen / "not moving"). */
        const eGPUTextureUsage usage = GPU_TEXTURE_USAGE_GENERAL;
        state.velocity = GPU_texture_create_2d(
            "img_fluid_velocity", W, H, 1, vel_fmt, usage, nullptr);
        state.pressure = GPU_texture_create_2d(
            "img_fluid_pressure", W, H, 1, scalar_fmt, usage, nullptr);
        state.temperature = GPU_texture_create_2d(
            "img_fluid_temperature", W, H, 1, scalar_fmt, usage, nullptr);
        state.divergence = GPU_texture_create_2d(
            "img_fluid_divergence", W, H, 1, scalar_fmt, usage, nullptr);
        for (const int color_index : IndexRange(color_count)) {
          state.dyes.append(GPU_texture_create_2d(("img_fluid_dye_" + std::to_string(color_index)).c_str(),
                                                  DW,
                                                  DH,
                                                  1,
                                                  color_fmt,
                                                  usage,
                                                  nullptr));
        }
        state.width = W;
        state.height = H;
        state.dye_width = DW;
        state.dye_height = DH;
      }
      if (!state.velocity || !state.pressure || !state.temperature || !state.divergence) {
        return fail();
      }
      GPU_texture_copy(state.velocity, velocity.gpu_texture());
      GPU_texture_copy(state.pressure, pressure.gpu_texture());
      GPU_texture_copy(state.temperature, temperature.gpu_texture());
      GPU_texture_copy(state.divergence, divergence.gpu_texture());
      for (const int color_index : IndexRange(color_count)) {
        if (!state.dyes[color_index]) {
          return fail();
        }
        GPU_texture_copy(state.dyes[color_index], dyes[color_index].gpu_texture());
      }
      state.frame = frame;
      cache.hold_display = false;
    }

    /* Realtime only: keep ephemeral GPU last-frame state above.
     * No multi-frame dense/checkpoint GPU→CPU download (removed 2026-08-02; see Notes). */
    cache.last_solved = frame;
    cache.dense.clear();
    cache.checkpoints.clear();
    cache.preview_interval.clear();
    cache.preview_interval_start = INT_MIN;

    const bool outputs_ok = write_gpu_outputs(velocity, dyes, temperature, divergence);
    cleanup();
    return outputs_ok;
  }

  void execute() override
  {
    /* Pavel WebGL-Fluid-Simulation step() only. No MAC/MG/CPU custom solver. */
    if (!this->context().use_gpu()) {
      this->write_fallback_outputs("Fluid needs GPU evaluation");
      return;
    }
    if (!this->gpu_shaders_ok()) {
      this->write_fallback_outputs("Fluid GPU shaders unavailable");
      return;
    }
    if (this->execute_gpu_pavel()) {
      return;
    }
    /* GPU path failed — still write something so Viewer does not keep a stale
     * previous preview (Checker / other nodes share the same Viewer ImBuf). */
    this->write_fallback_outputs("Fluid GPU step failed");
  }

  /**
   * Prefer pass-through of connected Color inputs so the Viewer is never pure black
   * when the GPU step fails (shaders missing / Vulkan glitch). Only fall back to
   * solid black when no usable input is available.
   */
  void write_fallback_outputs(const char *warning)
  {
    if (warning && warning[0]) {
      this->add_warning(NodeWarningType::Error, warning);
    }
    const Domain domain = this->compute_domain();
    for (const std::string &id : color_socket_ids(this->node())) {
      if (!this->node().output_by_identifier(UString(id.c_str()))) {
        continue;
      }
      Result &output = this->get_result(id);
      if (!output.should_compute()) {
        continue;
      }
      /* Pass through connected Color so the user still sees seed/input (not pure black). */
      if (this->has_input(id)) {
        Result &input = this->get_input(id);
        if (input.is_allocated() || input.is_single_value()) {
          if (output.is_allocated()) {
            output.free();
          }
          output.share_data(input);
          continue;
        }
      }
      Result black = this->context().create_result(ResultType::Color);
      black.allocate_texture(domain, false);
      if (black.is_allocated() && black.gpu_texture()) {
        const float zero[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        GPU_texture_clear(black.gpu_texture(), GPU_DATA_FLOAT, zero);
      }
      share_result_safe(output, black);
      black.release();
    }
  }
};


/** \} */

/* -------------------------------------------------------------------- */
/** \name Fluid Input 鈥?delta time + pass-through / cache feed
 * \{ */

class FluidSimInputOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    return fluid_domain(this->context());
  }

  void execute() override
  {
    const Domain domain = this->compute_domain();
    const int frame = this->context().get_scene().r.cfra;
    const uint session_uid = this->node().owner_tree().id.session_uid;
    const auto &storage = *static_cast<const NodeImageFluidSimInput *>(this->node().storage);
    const int32_t out_id = storage.output_node_id;

    if (this->node().output_by_identifier("Delta Time"_ustr)) {
      Result &dt_out = this->get_result("Delta Time");
      if (dt_out.should_compute()) {
        float dt = 0.0f;
        if (out_id != 0) {
          if (const FluidZoneCache *cache = fluid_zone_caches().lookup_ptr(
                  FluidCacheKey{session_uid, out_id}))
          {
            const int prev = fluid_nearest_at_or_before(*cache, frame - 1);
            if (prev != INT_MIN && prev < frame) {
              const Scene &scene = this->context().get_scene();
              const float fps = math::max(float(scene.r.frs_sec) / float(scene.r.frs_sec_base),
                                          1e-6f);
              dt = float(frame - prev) / fps;
            }
          }
        }
        dt_out.allocate_single_value();
        dt_out.set_single_value(dt);
      }
    }

    /*
     * Zone-body outputs:
     * - Cached prev 鈫?FEEDBACK only (exterior Color/Velocity IGNORED).
     * - Cold (no prev / start frame) 鈫?exterior inputs once.
     * Collision + Air always external.
     */
    auto pass_external = [&](const char *id, const ResultType fallback_type) {
      if (!this->node().output_by_identifier(UString(id))) {
        return;
      }
      Result &out = this->get_result(id);
      if (!out.should_compute()) {
        return;
      }
      if (this->has_input(id)) {
        Result &in = this->get_input(id);
        if (in.is_allocated() || in.is_single_value()) {
          out.share_data(in);
          return;
        }
      }
      const int n = domain.data_size.x * domain.data_size.y;
      if (fallback_type == ResultType::Float) {
        write_float_result(this->context(), out, domain, std::vector<float>(size_t(n), 0.0f));
      }
      else if (fallback_type == ResultType::Color) {
        write_color_result(this->context(), out, domain, std::vector<float>(size_t(n), 0.0f));
      }
      else {
        write_velocity_result(this->context(),
                              out,
                              domain,
                              std::vector<float>(size_t(n), 0.0f),
                              std::vector<float>(size_t(n), 0.0f));
      }
    };

    const int start_frame = this->context().get_scene().r.sfra;
    const FluidFrameState *prev = nullptr;
    bool gpu_has_history = false;
    if (out_id != 0 && frame > start_frame) {
      if (const FluidZoneCache *cache = fluid_zone_caches().lookup_ptr(
              FluidCacheKey{session_uid, out_id}))
      {
        prev = fluid_find_state(*cache, frame - 1);
        /* GPU path stores empty dense shells — treat last_solved / gpu.frame as
         * "simulation is running" so we do NOT re-feed exterior Color/Velocity
         * into the zone body every frame (that looks like feedback flicker). */
        gpu_has_history = (cache->last_solved >= frame - 1 && cache->last_solved != INT_MIN) ||
                          (cache->gpu.frame >= frame - 1 && cache->gpu.frame != INT_MIN);
      }
    }

    const bool dense_feedback = prev != nullptr && prev->width > 0 && prev->height > 0 &&
                                !prev->vel_x.empty() && !prev->vel_y.empty();
    const bool feedback = dense_feedback;

    if (feedback) {
      const int dw = domain.data_size.x;
      const int dh = domain.data_size.y;
      const int sw = prev->width;
      const int sh = prev->height;
      const int dye_w = prev->dye_width > 0 ? prev->dye_width : sw;
      const int dye_h = prev->dye_height > 0 ? prev->dye_height : sh;
      std::vector<float> up_vx, up_vy, up_t, up_d;
      /* Feedback velocity: MAC or cell 鈫?domain-normalized cell vectors. */
      if (this->node().output_by_identifier("Velocity"_ustr)) {
        std::vector<float> cell_vx, cell_vy;
        if (int(prev->vel_x.size()) == (sw + 1) * sh &&
            int(prev->vel_y.size()) == sw * (sh + 1))
        {
          mac_velocity_to_cell(prev->vel_x, prev->vel_y, sw, sh, cell_vx, cell_vy);
        }
        else if (int(prev->vel_x.size()) >= sw * sh && int(prev->vel_y.size()) >= sw * sh) {
          cell_vx.assign(prev->vel_x.begin(), prev->vel_x.begin() + sw * sh);
          cell_vy.assign(prev->vel_y.begin(), prev->vel_y.begin() + sw * sh);
        }
        else {
          cell_vx.assign(size_t(sw * sh), 0.0f);
          cell_vy.assign(size_t(sw * sh), 0.0f);
        }
        const float inv_sx = 1.0f / float(std::max(sw, 1));
        const float inv_sy = 1.0f / float(std::max(sh, 1));
        for (size_t i = 0; i < cell_vx.size(); i++) {
          cell_vx[i] *= inv_sx;
          cell_vy[i] *= inv_sy;
        }
        resize_field(cell_vx, sw, sh, up_vx, dw, dh);
        resize_field(cell_vy, sw, sh, up_vy, dw, dh);
        write_velocity_result(
            this->context(), this->get_result("Velocity"), domain, up_vx, up_vy);
      }
      if (this->node().output_by_identifier("Temperature"_ustr)) {
        resize_field(prev->temperature, sw, sh, up_t, dw, dh);
        write_float_result(this->context(), this->get_result("Temperature"), domain, up_t);
      }
      if (this->node().output_by_identifier("Divergence"_ustr)) {
        resize_field(prev->divergence, sw, sh, up_d, dw, dh);
        write_float_result(this->context(), this->get_result("Divergence"), domain, up_d);
      }
      const Vector<std::string> color_ids = color_socket_ids(this->node());
      const int n_sockets = int(color_ids.size());
      const bool rgb_layout = int(prev->colors.size()) >= n_sockets * 3;
      for (const int ci : color_ids.index_range()) {
        const std::string &id = color_ids[ci];
        if (!this->node().output_by_identifier(UString(id.c_str()))) {
          continue;
        }
        Result &out = this->get_result(id);
        if (!out.should_compute()) {
          continue;
        }
        std::vector<float> up_r, up_g, up_b;
        if (rgb_layout) {
          resize_field(prev->colors[size_t(dye_ch_r(ci))], dye_w, dye_h, up_r, dw, dh);
          resize_field(prev->colors[size_t(dye_ch_g(ci))], dye_w, dye_h, up_g, dw, dh);
          resize_field(prev->colors[size_t(dye_ch_b(ci))], dye_w, dye_h, up_b, dw, dh);
        }
        else if (ci < int(prev->colors.size())) {
          resize_field(prev->colors[size_t(ci)], dye_w, dye_h, up_r, dw, dh);
          up_g = up_r;
          up_b = up_r;
        }
        else {
          up_r.assign(size_t(dw * dh), 0.0f);
          up_g = up_r;
          up_b = up_r;
        }
        write_color_result_rgb(this->context(), out, domain, up_r, up_g, up_b);
      }
    }
    else if (gpu_has_history) {
      /* Realtime GPU last-frame state → zone body (Simulation-zone style feedback).
       * Exterior Color/Velocity are cold-start only; after history exists the body receives
       * previous sim fields so Mix/Add paint can inject new dye every frame. Zeros here made
       * body paint replace feedback with black and Output ignored body Color. */
      FluidZoneCache *cache = fluid_zone_caches().lookup_ptr(FluidCacheKey{session_uid, out_id});
      const FluidGpuState *gpu = (cache && cache->gpu.velocity && cache->gpu.frame != INT_MIN) ?
                                     &cache->gpu :
                                     nullptr;
      auto feed_from_gpu_tex = [&](const char *id,
                                   gpu::Texture *tex,
                                   const ResultType type,
                                   const float2 value_scale) {
        if (!tex || !this->node().output_by_identifier(UString(id))) {
          return;
        }
        Result &out = this->get_result(id);
        if (!out.should_compute()) {
          return;
        }
        Result src = this->context().create_result(type);
        src.set_reference_count(1);
        const Domain src_domain(int2(GPU_texture_width(tex), GPU_texture_height(tex)));
        src.allocate_texture(src_domain, false);
        if (!src.is_allocated() || !src.gpu_texture()) {
          src.release();
          return;
        }
        GPU_texture_copy(src.gpu_texture(), tex);
        Result resized = this->context().create_result(type);
        resized.set_reference_count(1);
        const char *shader_name = (type == ResultType::Color) ?
                                      "compositor_fluid_pavel_resample_color" :
                                      (type == ResultType::Float2 ?
                                           "compositor_fluid_pavel_resample_float2" :
                                           "compositor_fluid_pavel_resample_float");
        gpu::Shader *shader = this->context().get_shader(shader_name);
        if (!shader) {
          share_result_safe(out, src);
          src.release();
          return;
        }
        if (resized.is_allocated()) {
          resized.free();
        }
        if (resized.type() != type) {
          resized.set_type(type);
        }
        resized.allocate_texture(domain, false);
        GPU_shader_bind(shader);
        if (type == ResultType::Float2) {
          GPU_shader_uniform_2f(shader, "value_scale", value_scale.x, value_scale.y);
        }
        else {
          GPU_shader_uniform_1f(shader, "value_scale", value_scale.x);
        }
        src.bind_as_texture(shader, "input_tx");
        resized.bind_as_image(shader, "output_img");
        compute_dispatch_threads_at_least(shader, domain.data_size);
        resized.unbind_as_image();
        src.unbind_as_texture();
        GPU_shader_unbind();
        share_result_safe(out, resized);
        resized.release();
        src.release();
      };

      if (gpu) {
        const float inv_w = 1.0f / float(math::max(gpu->width, 1));
        const float inv_h = 1.0f / float(math::max(gpu->height, 1));
        feed_from_gpu_tex("Velocity", gpu->velocity, ResultType::Float2, float2(inv_w, inv_h));
        feed_from_gpu_tex("Temperature", gpu->temperature, ResultType::Float, float2(1.0f));
        feed_from_gpu_tex("Divergence", gpu->divergence, ResultType::Float, float2(1.0f));
        const Vector<std::string> color_ids = color_socket_ids(this->node());
        for (const int ci : color_ids.index_range()) {
          if (ci < gpu->dyes.size() && gpu->dyes[ci]) {
            feed_from_gpu_tex(
                color_ids[ci].c_str(), gpu->dyes[ci], ResultType::Color, float2(1.0f));
          }
        }
      }
      else {
        const int n = domain.data_size.x * domain.data_size.y;
        if (this->node().output_by_identifier("Velocity"_ustr) &&
            this->get_result("Velocity").should_compute())
        {
          write_velocity_result(this->context(),
                                this->get_result("Velocity"),
                                domain,
                                std::vector<float>(size_t(n), 0.0f),
                                std::vector<float>(size_t(n), 0.0f));
        }
        for (const std::string &id : color_socket_ids(this->node())) {
          if (!this->node().output_by_identifier(UString(id.c_str()))) {
            continue;
          }
          Result &out = this->get_result(id);
          if (out.should_compute()) {
            write_color_result(this->context(), out, domain, std::vector<float>(size_t(n), 0.0f));
          }
        }
      }
    }
    else {
      /* True cold start only: exterior Color/Velocity seed the body once. */
      pass_external("Velocity", ResultType::Float2);
      pass_external("Temperature", ResultType::Float);
      pass_external("Divergence", ResultType::Float);
      for (const std::string &id : color_socket_ids(this->node())) {
        pass_external(id.c_str(), ResultType::Color);
      }
    }

    /* Collision + air always external (can change every frame). */
    pass_external("Collision Mask", ResultType::Float);
    pass_external("Collider Velocity", ResultType::Float2);
    pass_external("Air Mask", ResultType::Float);
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Node registration
 * \{ */

static void node_gather_link_searches(GatherLinkSearchOpParams &params);

namespace fluid_input_node {
NODE_STORAGE_FUNCS(NodeImageFluidSimInput)

static void node_declare(NodeDeclarationBuilder &b)
{
  declare_aligned_panels(b, true);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeImageFluidSimInput *data = MEM_new<NodeImageFluidSimInput>(__func__);
  data->output_node_id = 0;
  node->storage = data;
}

static void node_label(const bNodeTree * /*ntree*/,
                       const bNode * /*node*/,
                       char *label,
                       const int label_maxncpy)
{
  BLI_strncpy_utf8(label, CTX_IFACE_(BLT_I18NCONTEXT_ID_NODETREE, "Fluid"), label_maxncpy);
}

static void draw_fluid_props(ui::Layout &layout, PointerRNA *output_ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  /* Match https://paveldogreat.github.io/WebGL-Fluid-Simulation/ control names. */
  layout.prop(output_ptr, "density_diffusion", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(output_ptr, "velocity_diffusion", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(output_ptr, "pressure", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(output_ptr, "vorticity", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(output_ptr, "pressure_vcycles", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(output_ptr, "pressure_iterations", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(output_ptr, "pressure_smooth_iterations", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(output_ptr, "sim_resolution", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(output_ptr, "dye_resolution", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(output_ptr, "dt_scale", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  /* No multi-frame cache UI — realtime GPU last-frame state only (Notes 2026-08-02). */
  layout.separator();
  layout.op("node.fluid_sim_reset", IFACE_("Reset Simulation"), ICON_FILE_REFRESH);
}

static void node_layout_ex(ui::Layout &layout, bContext *C, PointerRNA *current_node_ptr)
{
  bNodeTree &ntree = *reinterpret_cast<bNodeTree *>(current_node_ptr->owner_id);
  bNode *current_node = static_cast<bNode *>(current_node_ptr->data);
  const bke::bNodeTreeZones *zones = ntree.zones();
  if (!zones) {
    return;
  }
  const bke::bNodeTreeZone *zone = zones->get_zone_by_node(current_node->identifier);
  if (!zone || !zone->output_node_id) {
    return;
  }
  bNode &output_node = const_cast<bNode &>(*zone->output_node());
  PointerRNA output_ptr = RNA_pointer_create_discrete(
      current_node_ptr->owner_id, RNA_Node, &output_node);

  if (ui::Layout *panel = layout.panel(C, "fluid_color_items", false, IFACE_("Color Fields"))) {
    socket_items::ui::draw_items_list_with_operators<FluidColorItemsAccessor>(
        C, panel, ntree, output_node);
  }
  draw_fluid_props(layout, &output_ptr);
}

static bool node_insert_link(bke::NodeInsertLinkParams &params)
{
  bNode *output_node = params.ntree.node_by_id(node_storage(params.node).output_node_id);
  if (!output_node) {
    return true;
  }
  return socket_items::try_add_item_via_any_extend_socket<FluidColorItemsAccessor>(
      params.ntree, params.node, *output_node, params.link);
}

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new FluidSimInputOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;
  img_node_type_base(&ntype, "ImageNodeFluidSimInput"_ustr, IMG_NODE_FLUID_SIM_INPUT);
  ntype.ui_name = "Fluid Simulation Input";
  ntype.ui_description =
      "Input of a black-box 2D fluid simulation zone (color fields, velocity, temperature, "
      "divergence, collision)";
  ntype.enum_name_legacy = "FLUID_SIM_INPUT";
  ntype.nclass = NODE_CLASS_INTERFACE;
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.labelfunc = node_label;
  ntype.no_muting = true;
  ntype.draw_buttons_ex = node_layout_ex;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.insert_link = node_insert_link;
  ntype.get_compositor_operation = get_compositor_operation;
  bke::node_type_storage(
      ntype, "NodeImageFluidSimInput", node_free_standard_storage, node_copy_standard_storage);
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace fluid_input_node

/* Shared auto-connect: both nodes use this when dragged into the graph. */
static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const bNodeSocket &other_socket = params.other_socket();
  if (!FluidColorItemsAccessor::supports_socket_type(other_socket.type, params.node_tree().type)) {
    return;
  }
  const UString name(other_socket.name);
  const eNodeSocketDatatype socket_type = eNodeSocketDatatype(other_socket.type);
  const bool is_input = other_socket.in_out == SOCK_IN;
  params.add_item_full_name(IFACE_("Fluid Simulation"), [name, socket_type, is_input](LinkSearchOpParams &params) {
    bNode &input_node = params.add_node("ImageNodeFluidSimInput"_ustr);
    bNode &output_node = params.add_node("ImageNodeFluidSimOutput"_ustr);
    output_node.location[0] = 400;
    auto &input_storage = *static_cast<NodeImageFluidSimInput *>(input_node.storage);
    input_storage.output_node_id = output_node.identifier;
    socket_items::clear<FluidColorItemsAccessor>(output_node);
    socket_items::add_item_with_socket_type_and_name<FluidColorItemsAccessor>(
        params.node_tree, output_node, socket_type, name.c_str());
    update_node_declaration_and_sockets(params.node_tree, input_node);
    update_node_declaration_and_sockets(params.node_tree, output_node);
    if (is_input) {
      params.connect_available_socket(output_node, name);
    }
    else {
      params.connect_available_socket(input_node, name);
    }
  });
}

namespace fluid_output_node {

NODE_STORAGE_FUNCS(NodeImageFluidSimOutput)

static void node_declare(NodeDeclarationBuilder &b)
{
  declare_aligned_panels(b, false);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeImageFluidSimOutput *data = MEM_new<NodeImageFluidSimOutput>(__func__);
  /* Defaults match https://paveldogreat.github.io/WebGL-Fluid-Simulation/
   * Multi-frame dense/checkpoint cache disabled (realtime GPU state only). */
  data->cached_frames = 0;
  data->checkpoint_rate = 0;
  /* Multigrid V-cycles per projection (2-3 typical). */
  data->pressure_vcycles = 2;
  /* script.js config.PRESSURE_ITERATIONS — realtime note uses 12; keep mild default. */
  data->pressure_iterations = 12;
  data->pressure_smooth_iterations = 2;
  /* script.js SIM_RESOLUTION / DYE_RESOLUTION — dye 512 for realtime (Notes 2026-08-02). */
  data->sim_resolution = 128;
  data->dye_resolution = 512;
  /* VELOCITY_DISSIPATION / DENSITY_DISSIPATION */
  data->viscosity = 0.2f;
  data->dissipation = 1.0f;
  /* config.PRESSURE (pressure clear scale) */
  data->buoyancy = 0.8f;
  /* config.CURL */
  data->vorticity = 30.0f;
  data->dt_scale = 1.0f;
  data->splat_radius = 0.25f;
  data->splat_force = 6000.0f;
  data->warm_start_pressure = 1;
  data->boundary_left = NODE_IMAGE_FLUID_BOUNDARY_BOUNCE;
  data->boundary_right = NODE_IMAGE_FLUID_BOUNDARY_BOUNCE;
  data->boundary_bottom = NODE_IMAGE_FLUID_BOUNDARY_BOUNCE;
  data->boundary_top = NODE_IMAGE_FLUID_BOUNDARY_BOUNCE;
  data->color_next_identifier = 0;
  data->color_items = MEM_new_array<NodeImageFluidColorItem>(1, __func__);
  data->color_items[0].name = BLI_strdup(DATA_("Color"));
  data->color_items[0].identifier = data->color_next_identifier++;
  data->color_items_num = 1;
  data->color_active_index = 0;
  node->storage = data;
}

static void node_free_storage(bNode *node)
{
  socket_items::destruct_array<FluidColorItemsAccessor>(*node);
  MEM_delete(reinterpret_cast<NodeImageFluidSimOutput *>(node->storage));
}

static void node_copy_storage(bNodeTree * /*dst*/, bNode *dst_node, const bNode *src_node)
{
  const NodeImageFluidSimOutput &src = node_storage(*src_node);
  auto *dst = MEM_new<NodeImageFluidSimOutput>(__func__, dna::shallow_copy(src));
  dst_node->storage = dst;
  socket_items::copy_array<FluidColorItemsAccessor>(*src_node, *dst_node);
}

static void node_layout_ex(ui::Layout &layout, bContext *C, PointerRNA *ptr)
{
  bNodeTree &ntree = *reinterpret_cast<bNodeTree *>(ptr->owner_id);
  bNode &node = *static_cast<bNode *>(ptr->data);
  if (ui::Layout *panel = layout.panel(C, "fluid_color_items", false, IFACE_("Color Fields"))) {
    socket_items::ui::draw_items_list_with_operators<FluidColorItemsAccessor>(
        C, panel, ntree, node);
  }
  fluid_input_node::draw_fluid_props(layout, ptr);
}

static bool node_insert_link(bke::NodeInsertLinkParams &params)
{
  return socket_items::try_add_item_via_any_extend_socket<FluidColorItemsAccessor>(
      params.ntree, params.node, params.node, params.link);
}

static bool fluid_sim_reset_poll(bContext *C)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree || snode->edittree->type != NTREE_IMAGE) {
    return false;
  }
  const bNode *node = blender::bke::node_get_active(*snode->edittree);
  if (!node) {
    return false;
  }
  return node->is_type("ImageNodeFluidSimInput"_ustr) ||
         node->is_type("ImageNodeFluidSimOutput"_ustr);
}

static wmOperatorStatus fluid_sim_reset_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return OPERATOR_CANCELLED;
  }
  bNodeTree &ntree = *snode->edittree;
  bNode *node = blender::bke::node_get_active(ntree);
  if (!node) {
    BKE_report(op->reports, RPT_ERROR, "No active fluid simulation node");
    return OPERATOR_CANCELLED;
  }
  int32_t output_id = 0;
  if (node->is_type("ImageNodeFluidSimOutput"_ustr)) {
    output_id = node->identifier;
  }
  else if (node->is_type("ImageNodeFluidSimInput"_ustr) && node->storage) {
    output_id = static_cast<const NodeImageFluidSimInput *>(node->storage)->output_node_id;
  }
  if (output_id == 0) {
    BKE_report(op->reports, RPT_ERROR, "Fluid zone is not paired");
    return OPERATOR_CANCELLED;
  }
  /* Force-clear GPU + dense + preview cache for this zone. */
  blender::nodes::image_fluid_zone::reset(ntree, output_id);
  /* Full demand recook from cold start. */
  {
    ImageProcessTreeDirtyState &dirty = image_process_tree_dirty_state(ntree.id.session_uid);
    dirty.topology_dirty = true;
    dirty.dirty_node_ids.clear();
  }
  /* NC_NODE|NA_EDITED → SpaceNode image_eval_dirty + area refresh (full cook). */
  WM_event_add_notifier(C, NC_NODE | NA_EDITED, &ntree);
  ED_area_tag_refresh(CTX_wm_area(C));
  BKE_report(op->reports, RPT_INFO, "Fluid simulation cache cleared");
  return OPERATOR_FINISHED;
}

static void NODE_OT_fluid_sim_reset(wmOperatorType *ot)
{
  ot->name = "Reset Simulation";
  ot->idname = "NODE_OT_fluid_sim_reset";
  ot->description =
      "Clear all GPU and dense fluid caches for the active Fluid Simulation zone and force recook";
  ot->exec = fluid_sim_reset_exec;
  ot->poll = fluid_sim_reset_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static void node_operators()
{
  WM_operatortype_append(NODE_OT_fluid_sim_reset);
  socket_items::ops::make_common_operators<FluidColorItemsAccessor>();
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  socket_items::blend_write<FluidColorItemsAccessor>(&writer, node);
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  socket_items::blend_read_data<FluidColorItemsAccessor>(&reader, node);
}

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new FluidSimOutputOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;
  img_node_type_base(&ntype, "ImageNodeFluidSimOutput"_ustr, IMG_NODE_FLUID_SIM_OUTPUT);
  ntype.ui_name = "Fluid Simulation Output";
  ntype.ui_description =
      "Output of a black-box 2D fluid simulation zone (Pavel WebGL-Fluid-Simulation step)";
  ntype.enum_name_legacy = "FLUID_SIM_OUTPUT";
  ntype.nclass = NODE_CLASS_INTERFACE;
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.labelfunc = fluid_input_node::node_label;
  ntype.no_muting = true;
  ntype.draw_buttons_ex = node_layout_ex;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.insert_link = node_insert_link;
  ntype.register_operators = node_operators;
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  ntype.get_compositor_operation = get_compositor_operation;
  bke::node_type_storage(ntype, "NodeImageFluidSimOutput", node_free_storage, node_copy_storage);
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace fluid_output_node

/** \} */

/* Public helpers for multi-frame resume (called from compositor).
 * Realtime fluid: never multi-frame fill — cook only the target frame. */
int image_fluid_sim_resume_frame(const bNodeTree & /*ntree*/, const int target_frame)
{
  return target_frame;
}

Vector<IndexRange> image_fluid_sim_cached_frame_ranges()
{
  /* No multi-frame dense timeline for fluid (realtime GPU state only). */
  return {};
}

Vector<IndexRange> image_fluid_sim_checkpoint_frame_ranges()
{
  return {};
}

void image_fluid_sim_reset(const bNodeTree &ntree, const int32_t output_node_id)
{
  const FluidCacheKey key{ntree.id.session_uid, output_node_id};
  if (FluidZoneCache *cache = fluid_zone_caches().lookup_ptr(key)) {
    cache->dense.clear();
    cache->checkpoints.clear();
    cache->preview_interval.clear();
    cache->preview_interval_start = INT_MIN;
    cache->gpu.free_textures();
    cache->last_solved = INT_MIN;
    cache->hold_display = false;
    cache->needs_param_reset = false;
    cache->solver_key = 0;
  }
  fluid_timeline_unregister_after(INT_MIN / 4);
}

void image_fluid_sim_invalidate_after(const bNodeTree &ntree, const int keep_frame)
{
  const uint session_uid = ntree.id.session_uid;
  ntree.ensure_topology_cache();
  for (const bNode *node : ntree.nodes_by_type("ImageNodeFluidSimOutput"_ustr)) {
    if (node->is_muted()) {
      continue;
    }
    if (FluidZoneCache *cache = fluid_zone_caches().lookup_ptr(
            FluidCacheKey{session_uid, node->identifier}))
    {
      cache->preview_interval.clear();
      cache->preview_interval_start = INT_MIN;
      Vector<int> remove_dense;
      for (const auto &item : cache->dense.items()) {
        if (item.key > keep_frame) {
          remove_dense.append(item.key);
        }
      }
      for (const int f : remove_dense) {
        cache->dense.remove(f);
      }
      Vector<int> remove_cp;
      for (const auto &item : cache->checkpoints.items()) {
        if (item.key > keep_frame) {
          remove_cp.append(item.key);
        }
      }
      for (const int f : remove_cp) {
        cache->checkpoints.remove(f);
      }
      if (cache->last_solved > keep_frame) {
        cache->last_solved = fluid_nearest_at_or_before(*cache, keep_frame);
      }
      if (cache->gpu.frame > keep_frame) {
        cache->gpu.free_textures();
      }
    }
  }
  fluid_timeline_unregister_after(keep_frame);
}

}  // namespace blender::nodes::node_image_fluid_sim_cc

/* Expose resume/invalidate without header coupling to compositor. */
namespace blender::nodes::image_fluid_zone {

int resume_frame(const bNodeTree &ntree, const int target_frame)
{
  return node_image_fluid_sim_cc::image_fluid_sim_resume_frame(ntree, target_frame);
}

Vector<IndexRange> cached_frame_ranges()
{
  return node_image_fluid_sim_cc::image_fluid_sim_cached_frame_ranges();
}

Vector<IndexRange> checkpoint_frame_ranges()
{
  return node_image_fluid_sim_cc::image_fluid_sim_checkpoint_frame_ranges();
}

void invalidate_after(const bNodeTree &ntree, const int keep_frame)
{
  node_image_fluid_sim_cc::image_fluid_sim_invalidate_after(ntree, keep_frame);
}

void reset(const bNodeTree &ntree, const int32_t output_node_id)
{
  node_image_fluid_sim_cc::image_fluid_sim_reset(ntree, output_node_id);
}

}  // namespace blender::nodes::image_fluid_zone
