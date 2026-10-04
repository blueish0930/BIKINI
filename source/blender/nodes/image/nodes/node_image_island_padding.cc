/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Island padding: expand seed pixels into empty (near-black) regions by
 * copying the nearest seed's color. Jump Flooding Algorithm, radius clipped
 * by Distance in pixels. Islands meet at a Voronoi boundary (no color mix). */

#include <cfloat>
#include <utility>

#include "BLI_array.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_island_padding_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Color>("Image"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Image with colored islands on a dark background");
  b.add_output<decl::Color>("Image"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Image after nearest-seed padding");

  b.add_input<decl::Float>("Distance"_ustr)
      .default_value(8.0f)
      .min(0.0f)
      .max(4096.0f)
      .description("Padding radius in pixels");
  b.add_input<decl::Float>("Threshold"_ustr)
      .default_value(0.001f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .description("Pixels with max(RGB)*alpha at or below this are empty (not seeds)");
}

using namespace blender::compositor;

static const Result &cpu_view(const Result &src, Result &storage)
{
  if (!src.is_single_value() && src.is_stored_on_gpu()) {
    storage = src.download_to_cpu();
    return storage;
  }
  return src;
}

static bool is_seed_pixel(const Color &c, const float threshold)
{
  const float peak = math::max(c.r, math::max(c.g, c.b)) * c.a;
  return peak > threshold;
}

class IslandPaddingOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    const Result &input_in = this->get_input("Image");
    Result &image_out = this->get_result("Image");
    if (input_in.is_single_value()) {
      image_out.allocate_single_value();
      image_out.set_single_value(input_in.get_single_value<Color>());
      return;
    }

    Result input_storage(this->context());
    const Result &input = cpu_view(input_in, input_storage);
    const Domain domain = this->compute_domain();
    const int2 size = domain.data_size;
    const float distance = math::max(
        0.0f, this->get_input("Distance").get_single_value_default<float>());
    const float threshold = math::max(
        0.0f, this->get_input("Threshold").get_single_value_default<float>());

    Result cpu_out = this->context().create_result(ResultType::Color);
    cpu_out.allocate_texture(domain, false, ResultStorageType::CPUImage);

    const int width = size.x;
    const int height = size.y;
    const int pixel_count = width * height;
    if (pixel_count <= 0) {
      if (input_storage.is_allocated()) {
        input_storage.release();
      }
      cpu_out.release();
      return;
    }

    Array<int2> seed_a(pixel_count);
    Array<int2> seed_b(pixel_count);
    const int2 empty(-1, -1);

    auto index_of = [width](const int2 texel) { return texel.y * width + texel.x; };

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        const int2 texel(x, y);
        seed_a[index_of(texel)] = is_seed_pixel(input.load_pixel<Color>(texel), threshold) ?
                                      texel :
                                      empty;
      }
    }

    int jump = 1;
    while (jump < math::max(width, height)) {
      jump *= 2;
    }
    jump /= 2;

    Array<int2> *curr = &seed_a;
    Array<int2> *next = &seed_b;

    while (jump >= 1) {
      const int jump_size = jump;
      Array<int2> &src = *curr;
      Array<int2> &dst = *next;
      parallel_for(size, [&](const int2 texel) {
        int2 best = src[index_of(texel)];
        float best_d = (best.x < 0) ? FLT_MAX :
                                      math::distance_squared(float2(float(texel.x), float(texel.y)), float2(float(best.x), float(best.y)));
        for (int oy = -1; oy <= 1; oy++) {
          for (int ox = -1; ox <= 1; ox++) {
            if (ox == 0 && oy == 0) {
              continue;
            }
            const int2 nb(texel.x + ox * jump_size, texel.y + oy * jump_size);
            if (nb.x < 0 || nb.y < 0 || nb.x >= width || nb.y >= height) {
              continue;
            }
            const int2 candidate = src[index_of(nb)];
            if (candidate.x < 0) {
              continue;
            }
            const float d = math::distance_squared(float2(float(texel.x), float(texel.y)), float2(float(candidate.x), float(candidate.y)));
            if (d < best_d) {
              best_d = d;
              best = candidate;
            }
          }
        }
        dst[index_of(texel)] = best;
      });
      std::swap(curr, next);
      jump /= 2;
    }

    const Array<int2> &seeds = *curr;
    const float max_dist_sq = distance * distance;
    parallel_for(size, [&](const int2 texel) {
      const Color original = input.load_pixel<Color>(texel);
      if (is_seed_pixel(original, threshold)) {
        cpu_out.store_pixel(texel, original);
        return;
      }
      const int2 seed = seeds[index_of(texel)];
      if (seed.x < 0) {
        cpu_out.store_pixel(texel, Color(0.0f, 0.0f, 0.0f, 0.0f));
        return;
      }
      const float d2 = math::distance_squared(float2(float(texel.x), float(texel.y)),
                                              float2(float(seed.x), float(seed.y)));
      if (d2 <= max_dist_sq) {
        cpu_out.store_pixel(texel, input.load_pixel<Color>(seed));
      }
      else {
        cpu_out.store_pixel(texel, Color(0.0f, 0.0f, 0.0f, 0.0f));
      }
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
  return new IslandPaddingOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeIslandPadding"_ustr);
  ntype.ui_name = "Island Padding";
  ntype.ui_description =
      "Expand island colors into empty pixels by sampling the nearest seed, clipped by Distance";
  ntype.nclass = NODE_CLASS_OP_FILTER;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_island_padding_cc
