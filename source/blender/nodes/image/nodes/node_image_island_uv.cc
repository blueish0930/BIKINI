/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Each same-color 4-connected component is one island. That island's OBB is
 * mapped to UV by the Origin / Scale menus. */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "BLI_array.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

#include "BLT_translation.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "NOD_menu_value.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_island_uv_cc {

enum class IslandUVOrigin : int {
  Center = 0,
  BottomLeft = 1,
};

enum class IslandUVScale : int {
  Fit01 = 0,
  RealSize = 1,
};

static const EnumPropertyItem origin_items[] = {
    {int(IslandUVOrigin::Center),
     "CENTER",
     0,
     N_("Center"),
     N_("UV (0,0) is this island's OBB center (Fit 0-1 → [-0.5, 0.5])")},
    {int(IslandUVOrigin::BottomLeft),
     "BOTTOM_LEFT",
     0,
     N_("Bottom Left"),
     N_("UV (0,0) is this island's OBB bottom-left (Fit 0-1 → [0, 1])")},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem scale_items[] = {
    {int(IslandUVScale::Fit01),
     "FIT_0_1",
     0,
     N_("Fit 0-1"),
     N_("Stretch this island's OBB to a unit range (Center: [-0.5, 0.5], Bottom Left: [0, 1])")},
    {int(IslandUVScale::RealSize),
     "REAL_SIZE",
     0,
     N_("Real Size"),
     N_("Keep this island's real size in the texture's 0-1 UV (a quarter-wide island spans 0.25)")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Color>("ID"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description(
          "Color or ID image. Each 4-connected block of the same color is one island. "
          "Near-black / ID 0 is background");
  b.add_output<decl::Vector>("UV"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description(
          "Per-island UV. Each connected same-color blob has its own OBB mapping");

  b.add_input<decl::Menu>("Origin"_ustr)
      .default_value(IslandUVOrigin::Center)
      .static_items(origin_items)
      .optional_label()
      .description("Where UV (0,0) sits on this island's OBB");
  b.add_input<decl::Menu>("Scale"_ustr)
      .default_value(IslandUVScale::Fit01)
      .static_items(scale_items)
      .optional_label()
      .description(
          "Fit: normalize this island. Real Size: use texture 0-1 units, no per-island stretch");
}

using namespace blender::compositor;

struct IslandFrame {
  float2 center = float2(0.0f);
  float2 axis_u = float2(1.0f, 0.0f);
  float2 axis_v = float2(0.0f, 1.0f);
  float u_min = 0.0f;
  float u_max = 0.0f;
  float v_min = 0.0f;
  float v_max = 0.0f;
  int count = 0;
  bool valid = false;
};

static const Result &cpu_view(const Result &src, Result &storage)
{
  if (!src.is_single_value() && src.is_stored_on_gpu()) {
    storage = src.download_to_cpu();
    return storage;
  }
  return src;
}

static uint32_t pack_rgb8(const float r, const float g, const float b)
{
  auto q = [](const float v) -> uint32_t {
    if (!std::isfinite(v)) {
      return 0;
    }
    return uint32_t(std::clamp(int(math::round(v * 255.0f)), 0, 255));
  };
  return (q(r) << 16) | (q(g) << 8) | q(b);
}

/* 0 = background. Same color → same key. */
static uint32_t pixel_key(const Result &input, const int2 texel)
{
  switch (input.type()) {
    case ResultType::Color: {
      const Color c = input.load_pixel<Color>(texel);
      const float a = std::isfinite(c.a) ? c.a : 1.0f;
      const float peak = math::max(c.r, math::max(c.g, c.b)) * a;
      if (!std::isfinite(peak) || peak <= 0.001f) {
        return 0;
      }
      return pack_rgb8(c.r, c.g, c.b);
    }
    case ResultType::Float: {
      const float v = input.load_pixel<float>(texel);
      if (!std::isfinite(v) || v <= 0.0f) {
        return 0;
      }
      return uint32_t(math::max(int(math::round(v)), 0));
    }
    case ResultType::Int: {
      const int v = input.load_pixel<int>(texel);
      return v > 0 ? uint32_t(v) : 0;
    }
    case ResultType::Float3: {
      const float3 v = input.load_pixel<float3>(texel);
      const float peak = math::max(v.x, math::max(v.y, v.z));
      if (!std::isfinite(peak) || peak <= 0.001f) {
        return 0;
      }
      return pack_rgb8(v.x, v.y, v.z);
    }
    case ResultType::Float2: {
      const float2 v = input.load_pixel<float2>(texel);
      if (!std::isfinite(v.x) || v.x <= 0.0f) {
        return 0;
      }
      return uint32_t(math::max(int(math::round(v.x)), 0));
    }
    default:
      return 0;
  }
}

static Vector<float2> convex_hull_2d(Span<float2> pts)
{
  if (pts.size() <= 2) {
    Vector<float2> h;
    h.extend(pts);
    return h;
  }
  Vector<float2> p;
  p.extend(pts);
  std::sort(p.begin(), p.end(), [](const float2 &a, const float2 &b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  });
  /* Dedup. */
  int w = 1;
  for (int i = 1; i < p.size(); i++) {
    if (math::distance_squared(p[i], p[w - 1]) > 1.0e-8f) {
      p[w++] = p[i];
    }
  }
  p.resize(w);
  if (p.size() <= 2) {
    return p;
  }

  auto cross = [](const float2 &o, const float2 &a, const float2 &b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  };
  Vector<float2> hull;
  hull.reserve(p.size() + 1);
  for (const float2 &pt : p) {
    while (hull.size() >= 2 &&
           cross(hull[hull.size() - 2], hull.last(), pt) <= 0.0f)
    {
      hull.pop_last();
    }
    hull.append(pt);
  }
  const int lower = hull.size() + 1;
  for (int i = p.size() - 2; i >= 0; i--) {
    const float2 &pt = p[i];
    while (hull.size() >= lower &&
           cross(hull[hull.size() - 2], hull.last(), pt) <= 0.0f)
    {
      hull.pop_last();
    }
    hull.append(pt);
  }
  if (!hull.is_empty()) {
    hull.pop_last();
  }
  return hull;
}

static bool min_area_obb(const Span<float2> pts, IslandFrame &frame)
{
  frame.count = pts.size();
  if (pts.is_empty()) {
    return false;
  }
  if (pts.size() == 1) {
    frame.center = pts[0];
    frame.axis_u = float2(1.0f, 0.0f);
    frame.axis_v = float2(0.0f, 1.0f);
    frame.u_min = 0.0f;
    frame.u_max = 0.0f;
    frame.v_min = 0.0f;
    frame.v_max = 0.0f;
    frame.valid = true;
    return true;
  }

  const Vector<float2> hull = convex_hull_2d(pts);
  const Span<float2> src = hull.size() >= 2 ? hull.as_span() : pts;

  float best_area = std::numeric_limits<float>::max();
  float2 best_u(1.0f, 0.0f);
  float2 best_v(0.0f, 1.0f);
  float best_umin = 0.0f, best_umax = 0.0f, best_vmin = 0.0f, best_vmax = 0.0f;

  const int nedge = src.size();
  const int trials = math::max(nedge, 1);
  for (int e = 0; e < trials; e++) {
    float2 axis_u;
    if (nedge >= 2) {
      axis_u = src[(e + 1) % nedge] - src[e];
      if (math::length_squared(axis_u) < 1.0e-12f) {
        continue;
      }
      axis_u = math::normalize(axis_u);
    }
    else {
      axis_u = float2(1.0f, 0.0f);
    }
    if (axis_u.x < 0.0f) {
      axis_u = -axis_u;
    }
    float2 axis_v(-axis_u.y, axis_u.x);
    if (axis_v.y < 0.0f) {
      axis_v = -axis_v;
    }

    float umin = std::numeric_limits<float>::max();
    float umax = -std::numeric_limits<float>::max();
    float vmin = umin;
    float vmax = umax;
    for (const float2 &pt : src) {
      const float du = math::dot(pt, axis_u);
      const float dv = math::dot(pt, axis_v);
      umin = math::min(umin, du);
      umax = math::max(umax, du);
      vmin = math::min(vmin, dv);
      vmax = math::max(vmax, dv);
    }
    const float area = math::max(umax - umin, 0.0f) * math::max(vmax - vmin, 0.0f);
    if (area < best_area) {
      best_area = area;
      best_u = axis_u;
      best_v = axis_v;
      best_umin = umin;
      best_umax = umax;
      best_vmin = vmin;
      best_vmax = vmax;
    }
  }

  frame.axis_u = best_u;
  frame.axis_v = best_v;
  /* Project every pixel (not just hull) so the OBB covers the whole island. */
  frame.u_min = std::numeric_limits<float>::max();
  frame.u_max = -std::numeric_limits<float>::max();
  frame.v_min = frame.u_min;
  frame.v_max = frame.u_max;
  double cx = 0.0, cy = 0.0;
  for (const float2 &pt : pts) {
    cx += double(pt.x);
    cy += double(pt.y);
    const float du = math::dot(pt, frame.axis_u);
    const float dv = math::dot(pt, frame.axis_v);
    frame.u_min = math::min(frame.u_min, du);
    frame.u_max = math::max(frame.u_max, du);
    frame.v_min = math::min(frame.v_min, dv);
    frame.v_max = math::max(frame.v_max, dv);
  }
  frame.center = float2(float(cx / double(pts.size())), float(cy / double(pts.size())));
  frame.valid = true;
  return true;
}

class IslandUVOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    const Result &input_in = this->get_input("ID");
    Result &image_out = this->get_result("UV");
    const IslandUVOrigin origin = IslandUVOrigin(
        this->get_input("Origin").get_single_value_default<MenuValue>().value);
    const IslandUVScale scale = IslandUVScale(
        this->get_input("Scale").get_single_value_default<MenuValue>().value);
    if (input_in.is_single_value()) {
      image_out.allocate_single_value();
      image_out.set_single_value(float3(0.0f));
      return;
    }

    Result input_storage(this->context());
    const Result &input = cpu_view(input_in, input_storage);
    const Domain domain = this->compute_domain();
    const int2 size = domain.data_size;
    const int w = size.x;
    const int h = size.y;
    const int n = w * h;

    Result cpu_out = this->context().create_result(ResultType::Float3);
    cpu_out.allocate_texture(domain, false, ResultStorageType::CPUImage);

    Array<uint32_t> keys(n);
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        keys[y * w + x] = pixel_key(input, int2(x, y));
      }
    }

    /* 4-connected components of the same color. Each blob is its own island. */
    Array<int> labels(n, 0);
    Vector<Vector<float2>> islands;
    islands.append({}); /* label 0 unused */
    Vector<int2> stack;
    stack.reserve(256);
    const int2 delta[4] = {int2(1, 0), int2(-1, 0), int2(0, 1), int2(0, -1)};

    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        const int i0 = y * w + x;
        if (labels[i0] != 0 || keys[i0] == 0) {
          continue;
        }
        const uint32_t k = keys[i0];
        const int label = islands.size();
        Vector<float2> pts;
        stack.clear();
        stack.append(int2(x, y));
        labels[i0] = label;
        while (!stack.is_empty()) {
          const int2 p = stack.pop_last();
          /* Work in the texture's 0-1 UV, not pixel indices. */
          pts.append(float2((float(p.x) + 0.5f) / float(w), (float(p.y) + 0.5f) / float(h)));
          for (const int2 d : delta) {
            const int nx = p.x + d.x;
            const int ny = p.y + d.y;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
              continue;
            }
            const int ni = ny * w + nx;
            if (labels[ni] != 0 || keys[ni] != k) {
              continue;
            }
            labels[ni] = label;
            stack.append(int2(nx, ny));
          }
        }
        islands.append(std::move(pts));
      }
    }

    Array<IslandFrame> frames(islands.size());
    for (int i = 1; i < islands.size(); i++) {
      min_area_obb(islands[i].as_span(), frames[i]);
    }

    parallel_for(size, [&](const int2 texel) {
      const int label = labels[texel.y * w + texel.x];
      if (label <= 0 || label >= frames.size() || !frames[label].valid) {
        cpu_out.store_pixel(texel, float3(0.0f));
        return;
      }
      const IslandFrame &frame = frames[label];
      const float2 p((float(texel.x) + 0.5f) / float(w), (float(texel.y) + 0.5f) / float(h));
      const float du = math::dot(p, frame.axis_u);
      const float dv = math::dot(p, frame.axis_v);
      const float u_ext = math::max(frame.u_max - frame.u_min, 1.0e-6f);
      const float v_ext = math::max(frame.v_max - frame.v_min, 1.0e-6f);
      const float u_c = 0.5f * (frame.u_min + frame.u_max);
      const float v_c = 0.5f * (frame.v_min + frame.v_max);

      float u = 0.0f;
      float v = 0.0f;
      if (scale == IslandUVScale::Fit01) {
        if (origin == IslandUVOrigin::Center) {
          /* OBB fills [-0.5, 0.5], (0,0) at this island's center. */
          u = (du - u_c) / u_ext;
          v = (dv - v_c) / v_ext;
        }
        else {
          /* OBB fills [0, 1], (0,0) at this island's bottom-left. */
          u = (du - frame.u_min) / u_ext;
          v = (dv - frame.v_min) / v_ext;
        }
      }
      else if (origin == IslandUVOrigin::Center) {
        /* Real size in texture 0-1 UV. Island width 0.25 → U spans 0.25. */
        u = du - u_c;
        v = dv - v_c;
      }
      else {
        u = du - frame.u_min;
        v = dv - frame.v_min;
      }
      cpu_out.store_pixel(texel, float3(u, v, 0.0f));
    });

    if (this->context().use_gpu()) {
      Result gpu_out = cpu_out.upload_to_gpu(true);
      image_out.share_data(gpu_out);
      gpu_out.release();
    }
    else {
      image_out.share_data(cpu_out);
    }
    cpu_out.release();
    if (input_storage.is_allocated()) {
      input_storage.release();
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new IslandUVOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeIslandUV"_ustr);
  ntype.ui_name = "Island UV";
  ntype.ui_description =
      "Each same-color connected island gets its own UV from that island's OBB. "
      "Origin: OBB center (0 at center) or bottom-left (0 at min). "
      "Fit 0-1 normalizes the island; Real Size keeps its size in the texture 0-1 UV";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_island_uv_cc
