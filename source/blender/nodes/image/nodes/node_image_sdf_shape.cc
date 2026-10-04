/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Image Process SDF Shape — IQ 2D + 3D catalog, Distance only.
 */

#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"

#include "BKE_node.hh"

#include "DNA_node_types.h"

#include "BLT_translation.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "NOD_sdf_math.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_sdf_shape_cc {

using namespace blender::nodes::sdf_math;

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Vector"_ustr)
      .default_value({0.0f, 0.0f, 0.0f})
      .min(-10000.0f)
      .max(10000.0f)
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Sample position. Unconnected: domain UV → [-1,1]² (Z from this value's Z)");

  auto &radius = b.add_input<decl::Float>("Radius"_ustr)
                     .default_value(0.4f)
                     .min(0.0f)
                     .max(10000.0f)
                     .description("Primary radius / size")
                     .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_SPHERE; });
  auto &size = b.add_input<decl::Vector>("Size"_ustr)
                   .default_value({0.4f, 0.3f, 0.25f})
                   .min(0.0f)
                   .max(10000.0f)
                   .description("Half-extents / 2D size XY")
                   .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_BOX; });
  auto &height = b.add_input<decl::Float>("Height"_ustr)
                     .default_value(0.8f)
                     .min(0.0f)
                     .max(10000.0f)
                     .description("Height / half-height / cut depth")
                     .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_CAPSULE; });
  auto &minor = b.add_input<decl::Float>("Minor Radius"_ustr)
                    .default_value(0.12f)
                    .min(0.0f)
                    .max(10000.0f)
                    .description("Torus tube / secondary radius")
                    .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_TORUS; });
  auto &roundness = b.add_input<decl::Float>("Roundness"_ustr)
                        .default_value(0.08f)
                        .min(0.0f)
                        .max(10000.0f)
                        .description("Fillet / frame thickness")
                        .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_ROUND_BOX; });
  auto &top_r = b.add_input<decl::Float>("Top Radius"_ustr)
                    .default_value(0.05f)
                    .min(0.0f)
                    .max(10000.0f)
                    .description("Cone/capsule top radius")
                    .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_CAPPED_CONE; });
  auto &offset = b.add_input<decl::Float>("Offset"_ustr)
                     .default_value(0.0f)
                     .min(-10000.0f)
                     .max(10000.0f)
                     .description("Plane: n·p + Offset, n=(0,1,0)")
                     .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_PLANE; });
  auto &point_a = b.add_input<decl::Vector>("Point A"_ustr)
                      .default_value({-0.5f, 0.0f, 0.0f})
                      .description("Segment / triangle / oriented-box endpoint A")
                      .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_SEGMENT_2D; });
  auto &point_b = b.add_input<decl::Vector>("Point B"_ustr)
                      .default_value({0.5f, 0.0f, 0.0f})
                      .description("Segment / triangle / oriented-box endpoint B")
                      .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_SEGMENT_2D; });
  auto &point_c = b.add_input<decl::Vector>("Point C"_ustr)
                      .default_value({0.0f, 0.5f, 0.0f})
                      .description("Triangle third vertex")
                      .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_TRIANGLE_2D; });
  auto &angle = b.add_input<decl::Float>("Angle"_ustr)
                    .default_value(1.047198f)
                    .min(0.0f)
                    .max(3.141593f)
                    .description("Half-aperture (radians, 0–π). Above π the pie/arc/cone creases")
                    .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_CONE; });
  auto &thickness = b.add_input<decl::Float>("Thickness"_ustr)
                        .default_value(0.1f)
                        .min(0.0f)
                        .max(10000.0f)
                        .description("Arc/ring tube, cross pad, hollow cut")
                        .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_ARC_2D; });
  auto &count = b.add_input<decl::Float>("Count"_ustr)
                    .default_value(5.0f)
                    .min(2.0f)
                    .max(64.0f)
                    .description("Star points n / stairs steps")
                    .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_STAR_2D; });
  auto &factor = b.add_input<decl::Float>("Factor"_ustr)
                     .default_value(3.0f)
                     .min(-10000.0f)
                     .max(10000.0f)
                     .description("Star m / chamfer / skew / death-star d / vesica h")
                     .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_STAR_2D; });

  b.add_output<decl::Float>("Distance"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("Signed distance (outside > 0). Map Range → Viewer");

  /* Always set availability so unused params stay hidden when the node is first added. */
  const SDFShapeType shape = (b.node_or_null() != nullptr) ?
                                 SDFShapeType(b.node_or_null()->custom1) :
                                 SDFShapeType::Sphere;
  radius.available(sdf_uses_radius(shape));
  size.available(sdf_uses_size(shape));
  height.available(sdf_uses_height(shape));
  minor.available(sdf_uses_minor_radius(shape));
  roundness.available(sdf_uses_roundness(shape));
  top_r.available(sdf_uses_top_radius(shape));
  offset.available(sdf_uses_offset(shape));
  point_a.available(sdf_uses_point_a(shape));
  point_b.available(sdf_uses_point_b(shape));
  point_c.available(sdf_uses_point_c(shape));
  angle.available(sdf_uses_angle(shape));
  thickness.available(sdf_uses_thickness(shape));
  count.available(sdf_uses_count(shape));
  factor.available(sdf_uses_factor(shape));
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  ui::Layout &col = layout.column(true);
  col.prop(ptr, "shape_type", UI_ITEM_NONE, IFACE_("Shape"), ICON_NONE);
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = NODE_SDF_SHAPE_SPHERE;
}

using namespace blender::compositor;

class SDFShapeOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    Result &vector = this->get_input("Vector");
    if (!vector.is_single_value()) {
      return vector.domain();
    }
    return this->context().get_compositing_domain();
  }

  float read_float_sock(const char *name, const float fallback) const
  {
    const bNodeSocket *sock = bke::node_find_socket(this->node(), SOCK_IN, UString(name));
    if (sock == nullptr) {
      return fallback;
    }
    if (sock->is_available()) {
      return this->get_input(name).get_single_value_default<float>();
    }
    return sock->default_value_typed<bNodeSocketValueFloat>()->value;
  }

  blender::float3 read_vec_sock(const char *name, const blender::float3 &fallback) const
  {
    const bNodeSocket *sock = bke::node_find_socket(this->node(), SOCK_IN, UString(name));
    if (sock == nullptr) {
      return fallback;
    }
    if (sock->is_available()) {
      return this->get_input(name).get_single_value_default<blender::float3>();
    }
    const float *v = sock->default_value_typed<bNodeSocketValueVector>()->value;
    return blender::float3(v[0], v[1], v[2]);
  }

  SDFShapeParams read_params() const
  {
    SDFShapeParams p;
    p.radius = this->read_float_sock("Radius", 0.4f);
    const blender::float3 size = this->read_vec_sock("Size", blender::float3(0.4f, 0.3f, 0.25f));
    p.size = float3(size.x, size.y, size.z);
    p.height = this->read_float_sock("Height", 0.8f);
    p.minor_radius = this->read_float_sock("Minor Radius", 0.12f);
    p.roundness = this->read_float_sock("Roundness", 0.08f);
    p.top_radius = this->read_float_sock("Top Radius", 0.05f);
    p.offset = this->read_float_sock("Offset", 0.0f);
    const blender::float3 pa = this->read_vec_sock("Point A", blender::float3(-0.5f, 0.0f, 0.0f));
    const blender::float3 pb = this->read_vec_sock("Point B", blender::float3(0.5f, 0.0f, 0.0f));
    const blender::float3 pc = this->read_vec_sock("Point C", blender::float3(0.0f, 0.5f, 0.0f));
    p.point_a = float3(pa.x, pa.y, pa.z);
    p.point_b = float3(pb.x, pb.y, pb.z);
    p.point_c = float3(pc.x, pc.y, pc.z);
    p.angle = this->read_float_sock("Angle", 1.047198f);
    p.thickness = this->read_float_sock("Thickness", 0.1f);
    p.count = this->read_float_sock("Count", 5.0f);
    p.factor = this->read_float_sock("Factor", 3.0f);
    return p;
  }

  void execute() override
  {
    Result &vector_in = this->get_input("Vector");
    Result &dist_out = this->get_result("Distance");

    const SDFShapeParams params = this->read_params();
    const int shape = this->node().custom1;

    Result vector_cpu_storage = this->context().create_result(ResultType::Float3);
    const Result *vector_cpu = &vector_in;
    const bool need_download = this->context().use_gpu() && !vector_in.is_single_value();
    if (need_download) {
      vector_cpu_storage = vector_in.download_to_cpu();
      vector_cpu = &vector_cpu_storage;
    }

    const Domain domain = this->compute_domain();
    Result dist_cpu = this->context().create_result(ResultType::Float);
    dist_cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);

    const int2 size_px = domain.data_size;
    const bool vector_is_single = vector_in.is_single_value();
    const blender::float3 vector_single = vector_in.get_single_value_default<blender::float3>();

    parallel_for(size_px, [&](const int2 texel) {
      blender::float3 p;
      if (vector_is_single) {
        p.x = ((float(texel.x) + 0.5f) / float(size_px.x)) * 2.0f - 1.0f;
        p.y = ((float(texel.y) + 0.5f) / float(size_px.y)) * 2.0f - 1.0f;
        p.z = vector_single.z;
      }
      else {
        p = vector_cpu->load_pixel<blender::float3>(texel);
      }
      dist_cpu.store_pixel(
          texel, sdf_shape_distance(SDFShapeType(shape), to_sdf_float3(p), params));
    });

    if (this->context().use_gpu()) {
      Result dist_gpu = dist_cpu.upload_to_gpu(true);
      dist_out.share_data(dist_gpu);
      dist_gpu.release();
    }
    else {
      dist_out.share_data(dist_cpu);
    }

    dist_cpu.release();
    if (need_download) {
      vector_cpu_storage.release();
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new SDFShapeOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeSDFShape"_ustr, IMG_NODE_SDF_SHAPE);
  ntype.ui_name = "SDF Shape";
  ntype.ui_description =
      "Inigo Quilez exact 2D/3D signed-distance primitives (Distance only)";
  ntype.nclass = NODE_CLASS_TEXTURE;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_sdf_shape_cc
