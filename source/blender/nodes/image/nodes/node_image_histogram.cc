/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Image Process: Histogram — live histogram, PS-style levels handles, Auto stretch. */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"

#include "DNA_color_types.h"
#include "DNA_node_types.h"

#include "BKE_node.hh"

#include "IMB_colormanagement.hh"

#include "BLT_translation.hh"

#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "node_image_util.hh"

#include "MEM_guardedalloc.h"

namespace blender::nodes::node_image_histogram_cc {

enum class HistogramChannel : int {
  RGB = 0,
  Red = 1,
  Green = 2,
  Blue = 3,
  Luma = 4,
};

static const EnumPropertyItem channel_items[] = {
    {int(HistogramChannel::RGB), "RGB", 0, N_("RGB"), N_("Adjust all RGB channels together")},
    {int(HistogramChannel::Red), "R", 0, N_("Red"), N_("Adjust red channel only")},
    {int(HistogramChannel::Green), "G", 0, N_("Green"), N_("Adjust green channel only")},
    {int(HistogramChannel::Blue), "B", 0, N_("Blue"), N_("Adjust blue channel only")},
    {int(HistogramChannel::Luma),
     "LUMA",
     0,
     N_("Luma"),
     N_("Adjust luminance (affects RGB together)")},
    {0, nullptr, 0, nullptr, nullptr},
};

NODE_STORAGE_FUNCS(NodeImageHistogram)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Color>("Color"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Input color whose histogram is displayed and remapped");
  b.add_output<decl::Color>("Color"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Color after levels adjustment");

  /* Histogram widget + Auto / mode (draw_buttons). */
  b.add_default_layout();

  b.add_input<decl::Menu>("Channel"_ustr)
      .default_value(HistogramChannel::RGB)
      .static_items(channel_items)
      .optional_label()
      .description("Which channel the histogram and levels operate on");

  b.add_input<decl::Bool>("Auto"_ustr)
      .default_value(false)
      .description(
          "Automatically stretch black/white to fill the full range from the input histogram "
          "(manual handles and Black/White are ignored while enabled)");

  b.add_input<decl::Float>("Clip"_ustr)
      .default_value(0.001f)
      .min(0.0f)
      .max(0.2f)
      .subtype(PROP_FACTOR)
      .description(
          "Auto mode: fraction of pixels to clip on each end (0.001 = 0.1%). "
          "Avoids single-pixel outliers defining black/white");

  /* Black / Gamma / White are edited via histogram vertical handles (+ RNA props). */

  b.add_input<decl::Float>("Output Black"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .description("Output black level after input remap");
  b.add_input<decl::Float>("Output White"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .description("Output white level after input remap");
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  node->flag |= NODE_OPTIONS;

  auto *storage = MEM_new<NodeImageHistogram>(__func__);
  storage->hist.channels = 3;
  storage->hist.x_resolution = 256;
  storage->hist.ymax = 1.0f;
  storage->hist.mode = HISTO_MODE_RGB;
  storage->hist.height = 100;
  storage->hist.flag = eHistogram_Flag(storage->hist.flag | HISTO_FLAG_LEVELS);
  storage->hist.levels_black = 0.0f;
  storage->hist.levels_white = 1.0f;
  storage->hist.levels_gamma = 1.0f;
  storage->hist.domain_min = 0.0f;
  storage->hist.domain_max = 1.0f;
  storage->use_auto = 0;
  storage->auto_clip = 0.001f;
  storage->out_black = 0.0f;
  storage->out_white = 1.0f;
  node->storage = storage;
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  bNode *node = static_cast<bNode *>(ptr->data);
  NodeImageHistogram *storage = node ? &node_storage(*node) : nullptr;
  if (storage) {
    /* Keep levels handles enabled for this widget. */
    storage->hist.flag = eHistogram_Flag(storage->hist.flag | HISTO_FLAG_LEVELS);
  }

  ui::Layout &col = layout.column(true);
  ui::template_histogram(&col, ptr, "histogram");

  PointerRNA hist_ptr = RNA_pointer_get(ptr, "histogram");
  if (hist_ptr.data) {
    ui::Layout &row = col.row(true);
    row.prop(&hist_ptr, "mode", ui::ITEM_R_EXPAND, std::nullopt, ICON_NONE);
    row.prop(&hist_ptr, "show_line", UI_ITEM_NONE, "", ICON_NONE);
  }

  /* Levels numbers (synced with handles; disabled when Auto is on). */
  if (storage) {
    ui::Layout &levels = col.column(true);
    levels.use_property_decorate_set(false);
    const bool is_auto = storage->use_auto != 0;
    ui::Layout &row = levels.row(true);
    row.active_set(!is_auto);
    row.prop(ptr, "levels_black", UI_ITEM_NONE, IFACE_("Black"), ICON_NONE);
    row.prop(ptr, "levels_gamma", UI_ITEM_NONE, IFACE_("Gamma"), ICON_NONE);
    row.prop(ptr, "levels_white", UI_ITEM_NONE, IFACE_("White"), ICON_NONE);
  }
}

/* Photoshop Levels: (v - black) / (white - black) → gamma → output range.
 * black/white are absolute scene-linear values (may be outside 0–1 for HDR). */
static float apply_levels(const float v,
                          const float black,
                          const float white,
                          const float gamma,
                          const float out_black,
                          const float out_white)
{
  float range = white - black;
  if (range < 1e-6f) {
    range = 1e-6f;
  }
  float t = math::clamp((v - black) / range, 0.0f, 1.0f);
  if (gamma != 1.0f && t > 0.0f) {
    t = std::pow(t, 1.0f / math::max(gamma, 1e-6f));
  }
  return t * (out_white - out_black) + out_black;
}

/** Map absolute value into 256 bins over [domain_min, domain_max].
 * Values are never forced into 0–1; only the bin index is clamped to the table.
 * Domain must already span the full data range so over/under-exposed samples count. */
static int bin_from_value(const float f, const float domain_min, const float domain_max)
{
  if (!std::isfinite(f)) {
    return 0;
  }
  float span = domain_max - domain_min;
  if (span < 1e-8f) {
    span = 1.0f;
  }
  const float t = (f - domain_min) / span;
  int bin = int(t * 255.0f + 0.5f);
  return math::clamp(bin, 0, 255);
}

static float bin_to_value(const int bin, const float domain_min, const float domain_max)
{
  float span = domain_max - domain_min;
  if (span < 1e-8f) {
    span = 1.0f;
  }
  return domain_min + (float(bin) / 255.0f) * span;
}

/**
 * Find black/white in absolute value space that stretch the histogram, clipping `clip`
 * fraction of total mass on each end (Photoshop Auto Levels style). Works for HDR domains.
 */
static void auto_levels_from_bins(const uint bins[256],
                                  const float clip,
                                  const float domain_min,
                                  const float domain_max,
                                  float &r_black,
                                  float &r_white)
{
  uint64_t total = 0;
  for (int i = 0; i < 256; i++) {
    total += bins[i];
  }
  if (total == 0) {
    r_black = domain_min;
    r_white = domain_max;
    return;
  }

  const float clip_clamped = math::clamp(clip, 0.0f, 0.2f);
  const uint64_t clip_count = uint64_t(double(total) * double(clip_clamped));

  uint64_t cum = 0;
  int black_bin = 0;
  for (int i = 0; i < 256; i++) {
    cum += bins[i];
    if (cum > clip_count) {
      black_bin = i;
      break;
    }
  }

  cum = 0;
  int white_bin = 255;
  for (int i = 255; i >= 0; i--) {
    cum += bins[i];
    if (cum > clip_count) {
      white_bin = i;
      break;
    }
  }

  if (white_bin <= black_bin) {
    white_bin = math::min(black_bin + 1, 255);
  }

  r_black = bin_to_value(black_bin, domain_min, domain_max);
  r_white = bin_to_value(white_bin, domain_min, domain_max);
}

using namespace blender::compositor;

class HistogramOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    const Result &input = this->get_input("Color");
    if (!input.is_single_value()) {
      return input.domain();
    }
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    Result &input = this->get_input("Color");
    Result &output = this->get_result("Color");

    NodeImageHistogram &storage = this->mutable_storage();
    storage.hist.flag = eHistogram_Flag(storage.hist.flag | HISTO_FLAG_LEVELS);

    const bool use_auto = this->get_input("Auto").get_single_value_default<bool>();
    storage.use_auto = use_auto ? 1 : 0;

    const float clip = this->get_input("Clip").get_single_value_default<float>();
    storage.auto_clip = clip;

    /* Black / white / gamma: histogram handles (+ RNA) are the source of truth. */
    float gamma = math::clamp(storage.hist.levels_gamma, 0.01f, 10.0f);

    const float out_black = this->get_input("Output Black").get_single_value_default<float>();
    const float out_white = this->get_input("Output White").get_single_value_default<float>();
    storage.out_black = out_black;
    storage.out_white = out_white;

    const HistogramChannel channel = HistogramChannel(
        this->get_input("Channel").get_single_value_default<MenuValue>().value);

    float luma_coeff[3];
    IMB_colormanagement_get_luminance_coefficients(luma_coeff);
    const float3 luma = float3(luma_coeff);

    float black = storage.hist.levels_black;
    float white = storage.hist.levels_white;

    if (input.is_single_value()) {
      const Color c = input.get_single_value<Color>();
      float data_min = math::min(c.r, math::min(c.g, c.b));
      float data_max = math::max(c.r, math::max(c.g, c.b));
      this->set_domain_from_data(data_min, data_max, black, white);
      this->update_histogram_single(c, luma);
      if (use_auto) {
        black = data_min;
        white = data_max;
        if (white <= black) {
          white = black + 1e-4f;
        }
        storage.hist.levels_black = black;
        storage.hist.levels_white = white;
      }
      if (output.should_compute()) {
        output.allocate_single_value();
        output.set_single_value(this->map_color(
            c, channel, black, white, gamma, out_black, out_white, luma));
      }
      storage.hist.levels_gamma = gamma;
      return;
    }

    Result input_cpu_storage = this->context().create_result(ResultType::Color);
    const Result *input_cpu = &input;
    if (this->context().use_gpu()) {
      input_cpu_storage = input.download_to_cpu();
      input_cpu = &input_cpu_storage;
    }

    const Domain domain = this->compute_domain();
    const int2 size = domain.data_size;

    /* Pass 1: true min/max of sampled pixels (no 0–1 clamp — keeps over/under range). */
    float data_min = std::numeric_limits<float>::max();
    float data_max = std::numeric_limits<float>::lowest();
    this->scan_data_range(*input_cpu, size, luma, data_min, data_max);
    if (!(data_max > data_min)) {
      data_min = 0.0f;
      data_max = 1.0f;
    }
    this->set_domain_from_data(data_min, data_max, black, white);

    uint bins_l[256] = {0};
    uint bins_r[256] = {0};
    uint bins_g[256] = {0};
    uint bins_b[256] = {0};
    uint bins_a[256] = {0};
    this->build_histogram_bins(*input_cpu, size, luma, bins_l, bins_r, bins_g, bins_b, bins_a);
    this->store_histogram_display(bins_l, bins_r, bins_g, bins_b, bins_a);

    if (use_auto) {
      this->compute_auto_levels(channel, bins_l, bins_r, bins_g, bins_b, clip, black, white);
      storage.hist.levels_black = black;
      storage.hist.levels_white = white;
    }
    else {
      black = storage.hist.levels_black;
      white = storage.hist.levels_white;
      if (white <= black) {
        white = black + 1e-4f;
      }
    }
    storage.hist.levels_gamma = gamma;

    if (!output.should_compute()) {
      if (this->context().use_gpu()) {
        input_cpu_storage.release();
      }
      return;
    }

    Result output_cpu = this->context().create_result(ResultType::Color);
    output_cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);

    parallel_for(size, [&](const int2 texel) {
      const Color c = input_cpu->load_pixel_zero<Color>(texel);
      output_cpu.store_pixel(
          texel, this->map_color(c, channel, black, white, gamma, out_black, out_white, luma));
    });

    if (this->context().use_gpu()) {
      Result output_gpu = output_cpu.upload_to_gpu(true);
      output.share_data(output_gpu);
      output_gpu.release();
    }
    else {
      output.share_data(output_cpu);
    }

    output_cpu.release();
    if (this->context().use_gpu()) {
      input_cpu_storage.release();
    }
  }

 private:
  NodeImageHistogram &mutable_storage()
  {
    return *const_cast<NodeImageHistogram *>(
        static_cast<const NodeImageHistogram *>(this->node().storage));
  }

  Color map_color(const Color &c,
                  const HistogramChannel channel,
                  const float black,
                  const float white,
                  const float gamma,
                  const float out_black,
                  const float out_white,
                  const float3 &luma) const
  {
    float4 v(c);
    switch (channel) {
      case HistogramChannel::Red:
        v.x = apply_levels(v.x, black, white, gamma, out_black, out_white);
        break;
      case HistogramChannel::Green:
        v.y = apply_levels(v.y, black, white, gamma, out_black, out_white);
        break;
      case HistogramChannel::Blue:
        v.z = apply_levels(v.z, black, white, gamma, out_black, out_white);
        break;
      case HistogramChannel::Luma: {
        const float y_in = math::dot(v.xyz(), luma);
        const float y_out = apply_levels(y_in, black, white, gamma, out_black, out_white);
        if (y_in > 1e-6f) {
          const float scale = y_out / y_in;
          v.x *= scale;
          v.y *= scale;
          v.z *= scale;
        }
        else {
          v.x = v.y = v.z = y_out;
        }
        break;
      }
      case HistogramChannel::RGB:
      default:
        v.x = apply_levels(v.x, black, white, gamma, out_black, out_white);
        v.y = apply_levels(v.y, black, white, gamma, out_black, out_white);
        v.z = apply_levels(v.z, black, white, gamma, out_black, out_white);
        break;
    }
    return Color(v.x, v.y, v.z, v.w);
  }

  void set_domain_from_data(const float data_min,
                            const float data_max,
                            const float levels_black,
                            const float levels_white)
  {
    Histogram &hist = this->mutable_storage().hist;
    /* Domain = full data range (HDR/negatives included) + current handles.
     * Also keep 0 and 1 in view as scene-linear reference ticks — never clamp
     * data into 0–1; overexposed samples (>1) must remain in the statistics. */
    float dmin = data_min;
    float dmax = data_max;
    if (!std::isfinite(dmin) || !std::isfinite(dmax) || !(dmax > dmin)) {
      dmin = 0.0f;
      dmax = 1.0f;
    }
    dmin = math::min(dmin, 0.0f);
    dmax = math::max(dmax, 1.0f);
    if (std::isfinite(levels_black) && std::isfinite(levels_white)) {
      dmin = math::min(dmin, math::min(levels_black, levels_white));
      dmax = math::max(dmax, math::max(levels_black, levels_white));
    }
    float span = dmax - dmin;
    if (span < 1e-6f) {
      dmax = dmin + 1.0f;
      span = 1.0f;
    }
    /* Pad so end peaks and handles stay off the widget border. */
    const float pad = math::max(span * 0.03f, 0.02f);
    hist.domain_min = dmin - pad;
    hist.domain_max = dmax + pad;
  }

  void scan_data_range(const Result &input_cpu,
                       const int2 size,
                       const float3 &luma,
                       float &r_min,
                       float &r_max)
  {
    /* Full (or dense) scan of true min/max — do NOT clamp to 0–1.
     * Overexposed HDR values (>1) and negatives must define the domain. */
    const int total = size.x * size.y;
    /* denser than display bins: miss fewer highlight outliers */
    const int step = math::max(1, total / (4 * 1024 * 1024));
    float mn = std::numeric_limits<float>::max();
    float mx = std::numeric_limits<float>::lowest();

    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x += step) {
        const Color c = input_cpu.load_pixel_zero<Color>(int2(x, y));
        auto consider = [&](float v) {
          if (std::isfinite(v)) {
            mn = math::min(mn, v);
            mx = math::max(mx, v);
          }
        };
        consider(c.r);
        consider(c.g);
        consider(c.b);
        consider(c.a);
        consider(math::dot(float3(c.r, c.g, c.b), luma));
      }
    }
    if (!(mx > mn) || !std::isfinite(mn) || !std::isfinite(mx)) {
      mn = 0.0f;
      mx = 1.0f;
    }
    r_min = mn;
    r_max = mx;
  }

  void update_histogram_single(const Color &c, const float3 &luma)
  {
    NodeImageHistogram &storage = this->mutable_storage();
    Histogram &hist = storage.hist;
    const float dmin = hist.domain_min;
    const float dmax = hist.domain_max;

    memset(hist.data_luma, 0, sizeof(hist.data_luma));
    memset(hist.data_r, 0, sizeof(hist.data_r));
    memset(hist.data_g, 0, sizeof(hist.data_g));
    memset(hist.data_b, 0, sizeof(hist.data_b));
    memset(hist.data_a, 0, sizeof(hist.data_a));

    hist.data_luma[bin_from_value(math::dot(float3(c.r, c.g, c.b), luma), dmin, dmax)] = 1.0f;
    hist.data_r[bin_from_value(c.r, dmin, dmax)] = 1.0f;
    hist.data_g[bin_from_value(c.g, dmin, dmax)] = 1.0f;
    hist.data_b[bin_from_value(c.b, dmin, dmax)] = 1.0f;
    hist.data_a[bin_from_value(c.a, dmin, dmax)] = 1.0f;

    hist.channels = 3;
    hist.x_resolution = 256;
    if (hist.ymax <= 0.0f) {
      hist.ymax = 1.0f;
    }
  }

  void build_histogram_bins(const Result &input_cpu,
                            const int2 size,
                            const float3 &luma,
                            uint bins_l[256],
                            uint bins_r[256],
                            uint bins_g[256],
                            uint bins_b[256],
                            uint bins_a[256])
  {
    const Histogram &hist = this->mutable_storage().hist;
    const float dmin = hist.domain_min;
    const float dmax = hist.domain_max;
    const int total = size.x * size.y;
    const int step = math::max(1, total / (1024 * 1024));

    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x += step) {
        const Color c = input_cpu.load_pixel_zero<Color>(int2(x, y));
        bins_l[bin_from_value(math::dot(float3(c.r, c.g, c.b), luma), dmin, dmax)]++;
        bins_r[bin_from_value(c.r, dmin, dmax)]++;
        bins_g[bin_from_value(c.g, dmin, dmax)]++;
        bins_b[bin_from_value(c.b, dmin, dmax)]++;
        bins_a[bin_from_value(c.a, dmin, dmax)]++;
      }
    }
  }

  void store_histogram_display(const uint bins_l[256],
                               const uint bins_r[256],
                               const uint bins_g[256],
                               const uint bins_b[256],
                               const uint bins_a[256])
  {
    uint max_l = 1, max_r = 1, max_g = 1, max_b = 1, max_a = 1;
    for (int i = 0; i < 256; i++) {
      max_l = math::max(max_l, bins_l[i]);
      max_r = math::max(max_r, bins_r[i]);
      max_g = math::max(max_g, bins_g[i]);
      max_b = math::max(max_b, bins_b[i]);
      max_a = math::max(max_a, bins_a[i]);
    }

    Histogram &hist = this->mutable_storage().hist;
    for (int i = 0; i < 256; i++) {
      hist.data_luma[i] = float(bins_l[i]) / float(max_l);
      hist.data_r[i] = float(bins_r[i]) / float(max_r);
      hist.data_g[i] = float(bins_g[i]) / float(max_g);
      hist.data_b[i] = float(bins_b[i]) / float(max_b);
      hist.data_a[i] = float(bins_a[i]) / float(max_a);
    }
    hist.channels = 3;
    hist.x_resolution = 256;
    if (hist.ymax <= 0.0f) {
      hist.ymax = 1.0f;
    }
  }

  void compute_auto_levels(const HistogramChannel channel,
                           const uint bins_l[256],
                           const uint bins_r[256],
                           const uint bins_g[256],
                           const uint bins_b[256],
                           const float clip,
                           float &r_black,
                           float &r_white) const
  {
    const NodeImageHistogram &storage = *static_cast<const NodeImageHistogram *>(
        this->node().storage);
    const float dmin = storage.hist.domain_min;
    const float dmax = storage.hist.domain_max;
    switch (channel) {
      case HistogramChannel::Red:
        auto_levels_from_bins(bins_r, clip, dmin, dmax, r_black, r_white);
        break;
      case HistogramChannel::Green:
        auto_levels_from_bins(bins_g, clip, dmin, dmax, r_black, r_white);
        break;
      case HistogramChannel::Blue:
        auto_levels_from_bins(bins_b, clip, dmin, dmax, r_black, r_white);
        break;
      case HistogramChannel::Luma:
        auto_levels_from_bins(bins_l, clip, dmin, dmax, r_black, r_white);
        break;
      case HistogramChannel::RGB:
      default: {
        /* Union of channel ranges: darkest black among RGB, brightest white among RGB. */
        float br, wr, bg, wg, bb, wb;
        auto_levels_from_bins(bins_r, clip, dmin, dmax, br, wr);
        auto_levels_from_bins(bins_g, clip, dmin, dmax, bg, wg);
        auto_levels_from_bins(bins_b, clip, dmin, dmax, bb, wb);
        r_black = math::min(br, math::min(bg, bb));
        r_white = math::max(wr, math::max(wg, wb));
        if (r_white <= r_black) {
          r_white = r_black + 1e-4f;
        }
        break;
      }
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new HistogramOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeHistogram"_ustr, IMG_NODE_HISTOGRAM);
  ntype.ui_name = "Histogram";
  ntype.ui_description =
      "Display input color histogram with Photoshop-style levels handles. "
      "Drag vertical lines for black / midtones / white; enable Auto to stretch to full range";
  ntype.enum_name_legacy = "HISTOGRAM";
  ntype.nclass = NODE_CLASS_OP_COLOR;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.draw_buttons_ex = node_layout;
  ntype.default_width = bke::NodeWidth::_240;
  bke::node_type_storage(
      ntype, "NodeImageHistogram", node_free_standard_storage, node_copy_standard_storage);
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_histogram_cc
