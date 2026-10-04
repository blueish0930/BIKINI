/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "GEO_cgal.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_mesh_boolean_cc {

enum class BooleanOp {
  Union = 0,
  Difference = 1,
  Intersection = 2,
};

static const EnumPropertyItem operation_items[] = {
    {int(BooleanOp::Union), "UNION", 0, N_("Union"), N_("A union B")},
    {int(BooleanOp::Difference), "DIFFERENCE", 0, N_("Difference"), N_("A minus B")},
    {int(BooleanOp::Intersection), "INTERSECT", 0, N_("Intersect"), N_("A intersect B")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Mesh 1"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("First operand mesh (A).");
  b.add_input<decl::Geometry>("Mesh 2"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Second operand mesh (B).");
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous()
      .description("Boolean result mesh.");
  b.add_input<decl::Menu>("Operation"_ustr)
      .static_items(operation_items)
      .default_value(BooleanOp::Union)
      .optional_label()
      .description("Union (A or B), Difference (A minus B), or Intersection (A and B).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geo_a = params.extract_input<GeometrySet>("Mesh 1"_ustr);
  GeometrySet geo_b = params.extract_input<GeometrySet>("Mesh 2"_ustr);
  const BooleanOp op = params.extract_input<BooleanOp>("Operation"_ustr);

  const Mesh *mesh_a = geo_a.get_mesh();
  const Mesh *mesh_b = geo_b.get_mesh();
  if (!mesh_a || !mesh_b) {
    params.error_message_add(NodeWarningType::Info, TIP_("Both inputs need a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  geometry::CgalBooleanOperation gop = geometry::CgalBooleanOperation::Union;
  switch (op) {
    case BooleanOp::Difference:
      gop = geometry::CgalBooleanOperation::Difference;
      break;
    case BooleanOp::Intersection:
      gop = geometry::CgalBooleanOperation::Intersection;
      break;
    default:
      gop = geometry::CgalBooleanOperation::Union;
      break;
  }

  std::string error;
  Mesh *mesh = geometry::cgal_mesh_boolean(*mesh_a, *mesh_b, gop, error);
  if (mesh == nullptr || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("CGAL Mesh Boolean failed") : error);
    if (mesh) {
      BKE_id_free(nullptr, mesh);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalMeshBoolean"_ustr, GEO_NODE_CGAL_MESH_BOOLEAN);
  ntype.ui_name = "Mesh Boolean";
  ntype.ui_description =
      "Boolean solid operations (union / difference / intersection) via CGAL corefinement. Prefer closed, manifold triangle meshes.";
  ntype.enum_name_legacy = "CGAL_MESH_BOOLEAN";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_mesh_boolean_cc
