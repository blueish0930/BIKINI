/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Triangle Remesh Geometry Node — pmp-library isotropic / adaptive remeshing
 * (Botsch & Kobbelt style, Houdini Remesh SOP family).
 *
 * Mode is a Menu socket on the node interface (not N-panel).
 */

#include "BLI_array.hh"
#include "BLI_math_base.hh"
#include "BLI_math_constants.hh"
#include "BLI_vector.hh"

#include "FN_field.hh"

#include "BKE_geometry_fields.hh"

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_triangle_remesh.hh"
#include "GEO_mesh_triangulate.hh"

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"
#include "DNA_node_types.h"

#include "node_geometry_util.hh"

#include <atomic>

namespace blender::nodes::node_geo_triangle_remesh_cc {

enum class RemeshMode {
  Uniform = 0,
  Adaptive = 1,
};

static const EnumPropertyItem mode_items[] = {
    {int(RemeshMode::Uniform),
     "UNIFORM",
     0,
     N_("Uniform"),
     N_("Target a single edge length (isotropic remesh)")},
    {int(RemeshMode::Adaptive),
     "ADAPTIVE",
     0,
     N_("Adaptive"),
     N_("Size edges by curvature between min and max length")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Mesh"_ustr)
      .supported_type(GeometryComponent::Type::Mesh)
      .is_default_link_socket()
      .description("Surface to remesh");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Isotropic triangle remesh (pmp-library / Botsch–Kobbelt)");

  b.add_input<decl::Menu>("Mode"_ustr)
      .static_items(mode_items)
      .default_value(RemeshMode::Uniform)
      .optional_label()
      .description("Uniform or curvature-adaptive triangle remeshing");

  b.add_input<decl::Float>("Edge Length"_ustr)
      .default_value(0.1f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Target edge length for uniform remeshing")
      .usage_by_menu("Mode"_ustr, int(RemeshMode::Uniform));

  b.add_input<decl::Float>("Min Edge Length"_ustr)
      .default_value(0.05f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Minimum edge length for adaptive remeshing")
      .usage_by_menu("Mode"_ustr, int(RemeshMode::Adaptive));
  b.add_input<decl::Float>("Max Edge Length"_ustr)
      .default_value(0.2f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Maximum edge length for adaptive remeshing")
      .usage_by_menu("Mode"_ustr, int(RemeshMode::Adaptive));
  b.add_input<decl::Float>("Approximation Error"_ustr)
      .default_value(0.01f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Maximum geometric approximation error for adaptive sizing")
      .usage_by_menu("Mode"_ustr, int(RemeshMode::Adaptive));

  b.add_input<decl::Int>("Iterations"_ustr)
      .default_value(10)
      .min(1)
      .max(100)
      .description("Number of split / collapse / flip / smooth remesh cycles");
  b.add_input<decl::Float>("Crease Angle"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .max(DEG2RADF(180.0f))
      .subtype(PROP_ANGLE)
      .description(
          "Dihedral angle above which edges are locked as features. "
          "0 disables crease locking (recommended unless you have sharp mechanical edges)");
  b.add_input<decl::Bool>("Project to Surface"_ustr)
      .default_value(true)
      .description("Project relaxed vertices back onto the input surface");
  b.add_input<decl::Bool>("Protect Boundaries"_ustr)
      .default_value(true)
      .description(
          "Treat open boundaries as a feature loop: no flip of boundary edges, "
          "no smoothing off the boundary. Boundary edges still split/collapse "
          "to the target length unless Preserve Boundary Topology is on");
  b.add_input<decl::Bool>("Preserve Boundary Topology"_ustr)
      .default_value(true)
      .description(
          "Keep the input open-boundary vertex ring: no split and no collapse "
          "of boundary edges (no vertices inserted or deleted on the boundary)");
  b.add_input<decl::Bool>("Hard Points"_ustr)
      .default_value(false)
      .hide_value()
      .evaluated_geometry_field()
      .description("Vertices that must be kept (feature vertices, not smoothed away)");
  b.add_input<decl::Bool>("Hard Edges"_ustr)
      .default_value(false)
      .hide_value()
      .evaluated_geometry_field()
      .description("Edges that must be kept as feature creases (not collapsed / smoothed off)");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = int16_t(RemeshMode::Uniform);
}

static geometry::TriangleRemeshMode to_geometry_mode(const RemeshMode mode)
{
  switch (mode) {
    case RemeshMode::Uniform:
      return geometry::TriangleRemeshMode::Uniform;
    case RemeshMode::Adaptive:
      return geometry::TriangleRemeshMode::Adaptive;
  }
  return geometry::TriangleRemeshMode::Uniform;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh"_ustr);
  const RemeshMode mode = params.extract_input<RemeshMode>("Mode"_ustr);
  const int iterations = params.extract_input<int>("Iterations"_ustr);
  const float crease_angle = params.extract_input<float>("Crease Angle"_ustr);
  const bool use_projection = params.extract_input<bool>("Project to Surface"_ustr);
  const bool protect_boundaries = params.extract_input<bool>("Protect Boundaries"_ustr);
  const bool preserve_boundary_topology = params.extract_input<bool>(
      "Preserve Boundary Topology"_ustr);
  const Field<bool> hard_points_field = params.extract_input<Field<bool>>("Hard Points"_ustr);
  const Field<bool> hard_edges_field = params.extract_input<Field<bool>>("Hard Edges"_ustr);
  const AttributeFilter &attribute_filter = params.get_attribute_filter("Mesh"_ustr);

  geometry::TriangleRemeshOptions options;
  options.mode = to_geometry_mode(mode);
  options.iterations = math::max(iterations, 1);
  options.crease_angle = crease_angle;
  options.use_projection = use_projection;
  options.protect_boundaries = protect_boundaries;
  options.preserve_boundary_topology = preserve_boundary_topology;

  switch (mode) {
    case RemeshMode::Uniform:
      options.edge_length = params.extract_input<float>("Edge Length"_ustr);
      break;
    case RemeshMode::Adaptive:
      options.min_edge_length = params.extract_input<float>("Min Edge Length"_ustr);
      options.max_edge_length = params.extract_input<float>("Max Edge Length"_ustr);
      options.approx_error = params.extract_input<float>("Approximation Error"_ustr);
      break;
  }

  std::atomic<bool> found_mesh = false;
  std::atomic<bool> empty_result = false;
  std::atomic<bool> bad_params = false;

  if (mode == RemeshMode::Uniform && !(options.edge_length > 0.0f)) {
    bad_params.store(true, std::memory_order_relaxed);
  }
  if (mode == RemeshMode::Adaptive) {
    if (!(options.min_edge_length > 0.0f) || !(options.max_edge_length > 0.0f) ||
        !(options.approx_error > 0.0f) || options.min_edge_length > options.max_edge_length)
    {
      bad_params.store(true, std::memory_order_relaxed);
    }
  }

  if (!bad_params.load(std::memory_order_relaxed)) {
    geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry) {
      const Mesh *src_mesh = geometry.get_mesh();
      if (!src_mesh) {
        return;
      }
      found_mesh.store(true, std::memory_order_relaxed);

      Vector<int> hard_verts;
      {
        const bke::MeshFieldContext context(*src_mesh, bke::AttrDomain::Point);
        fn::FieldEvaluator evaluator(context, src_mesh->verts_num);
        Array<bool> sel(src_mesh->verts_num);
        evaluator.add_with_destination(hard_points_field, sel.as_mutable_span());
        evaluator.evaluate();
        for (const int i : IndexRange(src_mesh->verts_num)) {
          if (sel[i]) {
            hard_verts.append(i);
          }
        }
      }
      Vector<int2> hard_edges;
      {
        const bke::MeshFieldContext context(*src_mesh, bke::AttrDomain::Edge);
        fn::FieldEvaluator evaluator(context, src_mesh->edges_num);
        Array<bool> sel(src_mesh->edges_num);
        evaluator.add_with_destination(hard_edges_field, sel.as_mutable_span());
        evaluator.evaluate();
        const Span<int2> edges = src_mesh->edges();
        for (const int e : edges.index_range()) {
          if (sel[e]) {
            hard_edges.append(edges[e]);
          }
        }
      }

      const Mesh *triangle_mesh = src_mesh;
      Mesh *owned_triangle_mesh = nullptr;
      if (src_mesh->corners_num != src_mesh->faces_num * 3) {
        const IndexMask selection(src_mesh->faces_num);
        if (std::optional<Mesh *> result = geometry::mesh_triangulate(
                *src_mesh,
                selection,
                geometry::TriangulateNGonMode::Beauty,
                geometry::TriangulateQuadMode::ShortEdge,
                attribute_filter))
        {
          owned_triangle_mesh = *result;
          triangle_mesh = owned_triangle_mesh;
        }
      }

      Mesh *result = geometry::mesh_triangle_remesh(
          *triangle_mesh, options, hard_verts.as_span(), hard_edges.as_span());
      if (owned_triangle_mesh) {
        BKE_id_free(nullptr, owned_triangle_mesh);
      }
      if (result->faces_num == 0 && triangle_mesh->faces_num > 0) {
        empty_result.store(true, std::memory_order_relaxed);
      }
      geometry.replace_mesh(result);
    });
  }

  if (bad_params.load(std::memory_order_relaxed)) {
    params.error_message_add(NodeWarningType::Error,
                             TIP_("Invalid edge length / adaptive size parameters"));
  }
  else if (!found_mesh.load(std::memory_order_relaxed)) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Input geometry does not contain a mesh"));
  }
  if (empty_result.load(std::memory_order_relaxed)) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("Remesh produced no faces; try a larger edge length or fewer iterations"));
  }

  params.set_output("Mesh"_ustr, std::move(geometry_set));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeTriangleRemesh"_ustr, GEO_NODE_TRIANGLE_REMESH);
  ntype.ui_name = "Triangle Remesh";
  ntype.ui_description =
      "Isotropic triangle remeshing (pmp-library Botsch–Kobbelt). "
      "Hard Points / Hard Edges lock selected vertices and edges as features "
      "(in addition to Crease Angle and Protect Boundaries). "
      "Preserve Boundary Topology freezes the open-boundary vertex ring";
  ntype.enum_name_legacy = "TRIANGLE_REMESH";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_triangle_remesh_cc
