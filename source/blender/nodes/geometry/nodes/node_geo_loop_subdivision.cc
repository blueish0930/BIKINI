/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_loop_subdivide.hh"
#include "GEO_mesh_triangulate.hh"

#include "BKE_lib_id.hh"

#include "DNA_mesh_types.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_loop_subdivision_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .supported_type(GeometryComponent::Type::Mesh)
      .is_default_link_socket()
      .description("Mesh to subdivide; non-triangle faces are triangulated first");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Subdivided triangle mesh");
  b.add_input<decl::Int>("Level"_ustr)
      .default_value(1)
      .min(0)
      .max(6)
      .description("Number of Loop subdivision iterations");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh"_ustr);
  const int level = std::max(params.extract_input<int>("Level"_ustr), 0);
  const AttributeFilter &attribute_filter = params.get_attribute_filter("Mesh"_ustr);

  if (level == 0) {
    params.set_output("Mesh"_ustr, std::move(geometry_set));
    return;
  }
  if (level >= 11) {
    params.error_message_add(NodeWarningType::Error,
                             TIP_("Subdivision result mesh is too large"));
    params.set_default_remaining_outputs();
    return;
  }

  std::atomic<bool> triangulated = false;
  std::atomic<bool> found_mesh = false;
  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry) {
    const Mesh *src_mesh = geometry.get_mesh();
    if (!src_mesh) {
      return;
    }
    found_mesh.store(true, std::memory_order_relaxed);

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

    Mesh *result = geometry::mesh_loop_subdivide(*triangle_mesh, level, attribute_filter);
    if (owned_triangle_mesh) {
      BKE_id_free(nullptr, owned_triangle_mesh);
    }
    geometry.replace_mesh(result);
  });

  if (!found_mesh.load(std::memory_order_relaxed)) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Input geometry does not contain a mesh"));
  }
  if (triangulated.load(std::memory_order_relaxed)) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Non-triangle faces were triangulated before Loop subdivision"));
  }
  params.set_output("Mesh"_ustr, std::move(geometry_set));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeLoopSubdivision"_ustr, GEO_NODE_LOOP_SUBDIVISION);
  ntype.ui_name = "Loop Subdivision";
  ntype.ui_description =
      "Smoothly subdivide triangle meshes with the Loop scheme, triangulating other faces first";
  ntype.enum_name_legacy = "LOOP_SUBDIVISION";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_loop_subdivision_cc