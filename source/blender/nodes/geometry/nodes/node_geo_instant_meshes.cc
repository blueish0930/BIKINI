/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Instant Meshes Geometry Node — official Instant Meshes core (Jakob et al.).
 *
 * Mode (Quad/Triangle) and Target (size mode) are Menu sockets on the node
 * interface — not sidebar / N-panel enums.
 */

#include "BLI_array.hh"
#include "BLI_math_base.hh"
#include "BLI_math_constants.hh"
#include "BLI_vector.hh"

#include "FN_field.hh"

#include "BKE_geometry_fields.hh"

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_instant_meshes.hh"
#include "GEO_mesh_triangulate.hh"

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"
#include "DNA_node_types.h"

#include "node_geometry_util.hh"

#include <atomic>

namespace blender::nodes::node_geo_instant_meshes_cc {

enum class MeshMode {
  Quad = 0,
  Triangle = 1,
};

/** Mutual exclusive target (official CLI: only one of scale / faces / vertices). */
enum class TargetMode {
  Auto = 0,
  EdgeLength = 1,
  FaceCount = 2,
  VertexCount = 3,
};

static const EnumPropertyItem mode_items[] = {
    {int(MeshMode::Quad),
     "QUAD",
     0,
     N_("Quad"),
     N_("4-RoSy / 4-PoSy field-aligned quad remesh")},
    {int(MeshMode::Triangle),
     "TRIANGLE",
     0,
     N_("Triangle"),
     N_("6-RoSy / 3-PoSy field-aligned isotropic triangle remesh")},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem target_items[] = {
    {int(TargetMode::Auto),
     "AUTO",
     0,
     N_("Auto"),
     N_("Default density (~1/16 of input vertex count)")},
    {int(TargetMode::EdgeLength),
     "EDGE_LENGTH",
     0,
     N_("Edge Length"),
     N_("Target lattice edge length ρ")},
    {int(TargetMode::FaceCount),
     "FACE_COUNT",
     0,
     N_("Faces"),
     N_("Approximate desired face count")},
    {int(TargetMode::VertexCount),
     "VERTEX_COUNT",
     0,
     N_("Vertices"),
     N_("Approximate desired vertex count")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Mesh"_ustr)
      .supported_type(GeometryComponent::Type::Mesh)
      .is_default_link_socket()
      .description(
          "Surface to remesh; non-triangle faces are triangulated before Instant Meshes");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Field-aligned remesh (quad or triangle) via official Instant Meshes");

  b.add_input<decl::Menu>("Mode"_ustr)
      .static_items(mode_items)
      .default_value(MeshMode::Quad)
      .optional_label()
      .description("Output mesh element family (quad or triangle lattice)");

  b.add_input<decl::Menu>("Target"_ustr)
      .static_items(target_items)
      .default_value(TargetMode::Auto)
      .optional_label()
      .description("How to set the remesh resolution (only one, as in Instant Meshes CLI)");

  b.add_input<decl::Float>("Edge Length"_ustr)
      .default_value(0.1f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Target lattice edge length ρ")
      .usage_by_menu("Target"_ustr, int(TargetMode::EdgeLength));
  b.add_input<decl::Int>("Face Count"_ustr)
      .default_value(1000)
      .min(1)
      .description("Approximate desired face count")
      .usage_by_menu("Target"_ustr, int(TargetMode::FaceCount));
  b.add_input<decl::Int>("Vertex Count"_ustr)
      .default_value(1000)
      .min(1)
      .description("Approximate desired vertex count")
      .usage_by_menu("Target"_ustr, int(TargetMode::VertexCount));

  b.add_input<decl::Float>("Crease Angle"_ustr)
      .default_value(DEG2RADF(30.0f))
      .min(-1.0f)
      .max(DEG2RADF(180.0f))
      .subtype(PROP_ANGLE)
      .description(
          "Dihedral angle above which edges are creases. Set negative to disable creases");
  b.add_input<decl::Int>("Iterations"_ustr)
      .default_value(6)
      .min(1)
      .max(64)
      .description("Gauss–Seidel iterations per multiresolution hierarchy level");
  b.add_input<decl::Int>("Smooth Iterations"_ustr)
      .default_value(2)
      .min(0)
      .max(64)
      .description("Post-extract smoothing with ray reprojection onto the input");
  b.add_input<decl::Bool>("Extrinsic"_ustr)
      .default_value(true)
      .description("Extrinsic smoothness (feature snapping)");
  b.add_input<decl::Bool>("Align Boundaries"_ustr)
      .default_value(true)
      .description("Align orientation and position fields to open boundary edges");
  /* Quad-only: hidden when Mode is Triangle. */
  b.add_input<decl::Bool>("Pure Quad"_ustr)
      .default_value(true)
      .description("Pure quads after extraction (quad mode only)")
      .usage_by_menu("Mode"_ustr, int(MeshMode::Quad));
  b.add_input<decl::Bool>("Deterministic"_ustr)
      .default_value(true)
      .description("Prefer deterministic algorithms for reproducible results");
  b.add_input<decl::Bool>("Hard Points"_ustr)
      .default_value(false)
      .hide_value()
      .evaluated_geometry_field()
      .description("Pin these vertices as crease / position constraints");
  b.add_input<decl::Bool>("Hard Edges"_ustr)
      .default_value(false)
      .hide_value()
      .evaluated_geometry_field()
      .description("Align the orientation field to these edges (crease + edge direction)");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  /* Forward compatibility with old custom1/custom2 values if sockets not yet updated. */
  node->custom1 = int16_t(MeshMode::Quad);
  node->custom2 = int16_t(TargetMode::Auto);
}

static geometry::InstantMeshesMode to_geometry_mode(const MeshMode mode)
{
  switch (mode) {
    case MeshMode::Quad:
      return geometry::InstantMeshesMode::Quad;
    case MeshMode::Triangle:
      return geometry::InstantMeshesMode::Triangle;
  }
  return geometry::InstantMeshesMode::Quad;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh"_ustr);
  const MeshMode mode = params.extract_input<MeshMode>("Mode"_ustr);
  const TargetMode target = params.extract_input<TargetMode>("Target"_ustr);
  const float crease_angle = params.extract_input<float>("Crease Angle"_ustr);
  const int iterations = params.extract_input<int>("Iterations"_ustr);
  const int smooth_iterations = params.extract_input<int>("Smooth Iterations"_ustr);
  const bool extrinsic = params.extract_input<bool>("Extrinsic"_ustr);
  const bool align_boundaries = params.extract_input<bool>("Align Boundaries"_ustr);
  const bool deterministic = params.extract_input<bool>("Deterministic"_ustr);
  const Field<bool> hard_points_field = params.extract_input<Field<bool>>("Hard Points"_ustr);
  const Field<bool> hard_edges_field = params.extract_input<Field<bool>>("Hard Edges"_ustr);
  const AttributeFilter &attribute_filter = params.get_attribute_filter("Mesh"_ustr);

  /* Only extract Pure Quad when the socket is used (Quad mode). */
  bool pure_quad = true;
  if (mode == MeshMode::Quad) {
    pure_quad = params.extract_input<bool>("Pure Quad"_ustr);
  }

  geometry::InstantMeshesOptions options;
  options.mode = to_geometry_mode(mode);
  options.edge_length = 0.0f;
  options.face_count = 0;
  options.vertex_count = 0;
  switch (target) {
    case TargetMode::Auto:
      break;
    case TargetMode::EdgeLength:
      options.edge_length = params.extract_input<float>("Edge Length"_ustr);
      break;
    case TargetMode::FaceCount:
      options.face_count = params.extract_input<int>("Face Count"_ustr);
      break;
    case TargetMode::VertexCount:
      options.vertex_count = params.extract_input<int>("Vertex Count"_ustr);
      break;
  }
  options.crease_angle = crease_angle;
  options.iterations_per_level = math::max(iterations, 1);
  options.smooth_iterations = math::max(smooth_iterations, 0);
  options.extrinsic = extrinsic;
  options.align_to_boundaries = align_boundaries;
  options.pure_quad = pure_quad && mode == MeshMode::Quad;
  options.deterministic = deterministic;

  std::atomic<bool> triangulated = false;
  std::atomic<bool> found_mesh = false;
  std::atomic<bool> empty_result = false;

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
        triangulated.store(true, std::memory_order_relaxed);
      }
    }

    Mesh *result = geometry::mesh_instant_meshes(
        *triangle_mesh, options, hard_verts.as_span(), hard_edges.as_span());
    if (owned_triangle_mesh) {
      BKE_id_free(nullptr, owned_triangle_mesh);
    }
    if (result->faces_num == 0 && triangle_mesh->faces_num > 0) {
      empty_result.store(true, std::memory_order_relaxed);
    }
    geometry.replace_mesh(result);
  });

  if (!found_mesh.load(std::memory_order_relaxed)) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Input geometry does not contain a mesh"));
  }
  if (triangulated.load(std::memory_order_relaxed)) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("Non-triangle faces were triangulated before Instant Meshes"));
  }
  if (empty_result.load(std::memory_order_relaxed)) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("Instant Meshes extraction produced no faces; try a different target resolution"));
  }

  params.set_output("Mesh"_ustr, std::move(geometry_set));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeInstantMeshes"_ustr, GEO_NODE_INSTANT_MESHES);
  ntype.ui_name = "Instant Meshes";
  ntype.ui_description =
      "Field-aligned remesh (official Instant Meshes). "
      "Hard Points pin vertices; Hard Edges constrain orientation along those edges "
      "(in addition to Crease Angle / Align Boundaries)";
  ntype.enum_name_legacy = "INSTANT_MESHES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  /* No draw_buttons: Mode/Target are Menu sockets on the interface. */
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_instant_meshes_cc
