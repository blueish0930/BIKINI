/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup shdnodes
 *
 * SDF Shape (Shader Editor) — IQ 2D + 3D catalog, Distance only.
 */

#include "node_shader_util.hh"

#include "BKE_context.hh"
#include "DNA_node_types.h"
#include "DNA_space_types.h"

#include "BLI_math_base.hh"

#include "NOD_multi_function.hh"
#include "NOD_sdf_math.hh"

#include <array>
#include <memory>
#include <optional>

#include "BLT_translation.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

namespace blender {

namespace nodes::node_shader_sdf_shape_cc {

using namespace blender::nodes::sdf_math;

static constexpr int k_shape_slots = 64;

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();

  /* Unconnected Vector:
   * - Shader (EEVEE GPU): Texture Coordinate → Object via node_shader_gpu_object_tex_coord
   * - Geometry: Position field (object-local), analogous to Object coords on surfaces
   *
   * Must use POSITION_FIELD (not VALUE/(0,0,0)): function-node constant folding would
   * otherwise collapse the whole SDF to a single distance and skip node_gpu entirely.
   * Same pattern as Noise / Wave / Voronoi Vector defaults. hide_value hides the unused
   * numeric field in the UI. */
  b.add_input<decl::Vector>("Vector"_ustr)
      .min(-10000.0f)
      .max(10000.0f)
      .hide_value()
      .default_input_type(NODE_DEFAULT_INPUT_POSITION_FIELD)
      .description(
          "Sample position. Unconnected: Object coordinates (origin-centered), "
          "same as Texture Coordinate → Object");
  b.add_input<decl::Float>("Scale"_ustr)
      .default_value(1.0f)
      .min(0.0001f)
      .max(10000.0f)
      .description("Multiplies position before the SDF");

  auto &radius = b.add_input<decl::Float>("Radius"_ustr)
                     .default_value(0.5f)
                     .min(0.0f)
                     .max(10000.0f)
                     .description("Primary radius / size")
                     .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_SPHERE; });
  auto &size = b.add_input<decl::Vector>("Size"_ustr)
                   .default_value({0.5f, 0.5f, 0.5f})
                   .min(0.0f)
                   .max(10000.0f)
                   .description("Half-extents / 2D size XY")
                   .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_BOX; });
  auto &height = b.add_input<decl::Float>("Height"_ustr)
                     .default_value(1.0f)
                     .min(0.0f)
                     .max(10000.0f)
                     .description("Height / half-height / cut depth")
                     .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_CAPSULE; });
  auto &minor = b.add_input<decl::Float>("Minor Radius"_ustr)
                    .default_value(0.15f)
                    .min(0.0f)
                    .max(10000.0f)
                    .description("Torus tube / secondary radius")
                    .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_TORUS; });
  auto &roundness = b.add_input<decl::Float>("Roundness"_ustr)
                        .default_value(0.1f)
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
                     .description("Star m / chamfer / skew / death-star distance / vesica h")
                     .make_available([](bNode &node) { node.custom1 = NODE_SDF_SHAPE_STAR_2D; });

  b.add_output<decl::Float>("Distance"_ustr)
      .description("Signed distance (outside > 0). Map Range → Base Color to visualize");

  /* Always set availability (even without node) so sockets hide correctly on add. */
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

static int node_gpu(GPUMaterial *mat,
                    bNode *node,
                    bNodeExecData * /*execdata*/,
                    GPUNodeStack *in,
                    GPUNodeStack *out)
{
  /* Unlinked Vector → Texture Coordinate → Object (origin-centered).
   * Socket default (0,0,0) becomes a constant GPU link; clear it first so
   * node_shader_gpu_object_tex_coord can inject local position (same as Noise uses
   * node_shader_gpu_default_tex_coord for Generated). Also pack +1000 into shape so
   * GLSL still uses Object coords if the inject path is skipped. */
  const bNodeSocket *vector_sock = bke::node_find_socket(*node, SOCK_IN, "Vector"_ustr);
  const bool vector_unlinked = (vector_sock == nullptr) || (vector_sock->link == nullptr) ||
                               ((vector_sock->link->flag & NODE_LINK_MUTED) != 0);
  if (vector_unlinked) {
    in[0].link = nullptr;
    node_shader_gpu_object_tex_coord(mat, node, &in[0].link);
  }
  float shape = float(node->custom1) + (vector_unlinked ? 1000.0f : 0.0f);
  return GPU_stack_link(mat, node, "node_sdf_shape", in, out, GPU_constant(&shape));
}

class SDFShapeFn : public mf::MultiFunction {
 public:
  int shape = 0;

  explicit SDFShapeFn(const int shape_in) : shape(shape_in)
  {
    static std::array<mf::Signature, k_shape_slots> signatures = []() {
      std::array<mf::Signature, k_shape_slots> sigs;
      for (int s = 0; s < k_shape_slots; s++) {
        mf::Signature signature;
        mf::SignatureBuilder b{"SDF Shape", signature};
        b.single_input<float3>("Vector");
        b.single_input<float>("Scale");
        const SDFShapeType st = SDFShapeType(s);
        if (sdf_shape_index_valid(s)) {
          if (sdf_uses_radius(st)) {
            b.single_input<float>("Radius");
          }
          if (sdf_uses_size(st)) {
            b.single_input<float3>("Size");
          }
          if (sdf_uses_height(st)) {
            b.single_input<float>("Height");
          }
          if (sdf_uses_minor_radius(st)) {
            b.single_input<float>("Minor Radius");
          }
          if (sdf_uses_roundness(st)) {
            b.single_input<float>("Roundness");
          }
          if (sdf_uses_top_radius(st)) {
            b.single_input<float>("Top Radius");
          }
          if (sdf_uses_offset(st)) {
            b.single_input<float>("Offset");
          }
          if (sdf_uses_point_a(st)) {
            b.single_input<float3>("Point A");
          }
          if (sdf_uses_point_b(st)) {
            b.single_input<float3>("Point B");
          }
          if (sdf_uses_point_c(st)) {
            b.single_input<float3>("Point C");
          }
          if (sdf_uses_angle(st)) {
            b.single_input<float>("Angle");
          }
          if (sdf_uses_thickness(st)) {
            b.single_input<float>("Thickness");
          }
          if (sdf_uses_count(st)) {
            b.single_input<float>("Count");
          }
          if (sdf_uses_factor(st)) {
            b.single_input<float>("Factor");
          }
        }
        else {
          b.single_input<float>("Radius");
        }
        b.single_output<float>("Distance");
        sigs[s] = std::move(signature);
      }
      return sigs;
    }();
    const int idx = math::clamp(shape_in, 0, k_shape_slots - 1);
    this->set_signature(&signatures[idx]);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const SDFShapeType st = SDFShapeType(shape);
    int pi = 0;
    const VArray<float3> &vector = params.readonly_single_input<float3>(pi++, "Vector");
    const VArray<float> &scale = params.readonly_single_input<float>(pi++, "Scale");

    std::optional<VArray<float>> radius;
    std::optional<VArray<float3>> size;
    std::optional<VArray<float>> height;
    std::optional<VArray<float>> minor;
    std::optional<VArray<float>> roundness;
    std::optional<VArray<float>> top_r;
    std::optional<VArray<float>> offset;
    std::optional<VArray<float3>> point_a;
    std::optional<VArray<float3>> point_b;
    std::optional<VArray<float3>> point_c;
    std::optional<VArray<float>> angle;
    std::optional<VArray<float>> thickness;
    std::optional<VArray<float>> count;
    std::optional<VArray<float>> factor;

    if (sdf_shape_index_valid(shape)) {
      if (sdf_uses_radius(st)) {
        radius = params.readonly_single_input<float>(pi++, "Radius");
      }
      if (sdf_uses_size(st)) {
        size = params.readonly_single_input<float3>(pi++, "Size");
      }
      if (sdf_uses_height(st)) {
        height = params.readonly_single_input<float>(pi++, "Height");
      }
      if (sdf_uses_minor_radius(st)) {
        minor = params.readonly_single_input<float>(pi++, "Minor Radius");
      }
      if (sdf_uses_roundness(st)) {
        roundness = params.readonly_single_input<float>(pi++, "Roundness");
      }
      if (sdf_uses_top_radius(st)) {
        top_r = params.readonly_single_input<float>(pi++, "Top Radius");
      }
      if (sdf_uses_offset(st)) {
        offset = params.readonly_single_input<float>(pi++, "Offset");
      }
      if (sdf_uses_point_a(st)) {
        point_a = params.readonly_single_input<float3>(pi++, "Point A");
      }
      if (sdf_uses_point_b(st)) {
        point_b = params.readonly_single_input<float3>(pi++, "Point B");
      }
      if (sdf_uses_point_c(st)) {
        point_c = params.readonly_single_input<float3>(pi++, "Point C");
      }
      if (sdf_uses_angle(st)) {
        angle = params.readonly_single_input<float>(pi++, "Angle");
      }
      if (sdf_uses_thickness(st)) {
        thickness = params.readonly_single_input<float>(pi++, "Thickness");
      }
      if (sdf_uses_count(st)) {
        count = params.readonly_single_input<float>(pi++, "Count");
      }
      if (sdf_uses_factor(st)) {
        factor = params.readonly_single_input<float>(pi++, "Factor");
      }
    }
    else {
      radius = params.readonly_single_input<float>(pi++, "Radius");
    }

    MutableSpan<float> distance = params.uninitialized_single_output<float>(pi, "Distance");

    mask.foreach_index([&](const int64_t i) {
      const float s = math::max(scale[i], 1e-6f);
      const float3 p = vector[i] * s;
      SDFShapeParams sp;
      if (radius) {
        sp.radius = (*radius)[i];
      }
      if (size) {
        sp.size = to_sdf_float3((*size)[i]);
      }
      if (height) {
        sp.height = (*height)[i];
      }
      if (minor) {
        sp.minor_radius = (*minor)[i];
      }
      if (roundness) {
        sp.roundness = (*roundness)[i];
      }
      if (top_r) {
        sp.top_radius = (*top_r)[i];
      }
      if (offset) {
        sp.offset = (*offset)[i];
      }
      if (point_a) {
        sp.point_a = to_sdf_float3((*point_a)[i]);
      }
      if (point_b) {
        sp.point_b = to_sdf_float3((*point_b)[i]);
      }
      if (point_c) {
        sp.point_c = to_sdf_float3((*point_c)[i]);
      }
      if (angle) {
        sp.angle = (*angle)[i];
      }
      if (thickness) {
        sp.thickness = (*thickness)[i];
      }
      if (count) {
        sp.count = (*count)[i];
      }
      if (factor) {
        sp.factor = (*factor)[i];
      }
      distance[i] = sdf_shape_distance(st, to_sdf_float3(p), sp);
    });
  }
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static SDFShapeFn *functions[k_shape_slots] = {nullptr};
  static bool init = false;
  if (!init) {
    for (int i = 0; i < k_shape_slots; i++) {
      functions[i] = new SDFShapeFn(i);
    }
    init = true;
  }
  const int shape = math::clamp(int(builder.node().custom1), 0, k_shape_slots - 1);
  builder.set_matching_fn(functions[shape]);
}

}  // namespace nodes::node_shader_sdf_shape_cc

static bool sdf_shader_poll(const bke::bNodeType * /*ntype*/,
                            const bNodeTree *ntree,
                            const char **r_disabled_hint)
{
  if (STR_ELEM(ntree->idname, "ShaderNodeTree", "GeometryNodeTree")) {
    return true;
  }
  *r_disabled_hint = RPT_("Not a shader or geometry node tree");
  return false;
}

static bool sdf_add_ui_poll(const bContext *C)
{
  const SpaceNode *snode = CTX_wm_space_node(C);
  if (snode && snode->edittree && snode->edittree->type == NTREE_GEOMETRY) {
    return true;
  }
  return object_shader_nodes_poll(C);
}

void register_node_type_sh_sdf_shape()
{
  namespace file_ns = nodes::node_shader_sdf_shape_cc;

  static bke::bNodeType ntype;

  sh_node_type_base(&ntype, "ShaderNodeSDFShape"_ustr, SH_NODE_SDF_SHAPE);
  ntype.ui_name = "SDF Shape";
  ntype.ui_description =
      "Inigo Quilez exact 2D/3D signed-distance primitives (Distance). "
      "Object coords default; Map Range → Base Color in EEVEE";
  ntype.enum_name_legacy = "SDF_SHAPE";
  ntype.nclass = NODE_CLASS_TEXTURE;
  ntype.declare = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.initfunc = file_ns::node_init;
  ntype.gpu_fn = file_ns::node_gpu;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.poll = sdf_shader_poll;
  ntype.add_ui_poll = sdf_add_ui_poll;

  bke::node_register_type(ntype);
}

}  // namespace blender
