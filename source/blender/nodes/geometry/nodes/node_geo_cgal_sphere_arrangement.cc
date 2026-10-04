/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_sphere_arrangement_cc {


static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Face planes become great circles on the unit sphere around Origin.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Sampled circles (n-gons) of plane ∩ unit sphere. Face attribute circle_id.");
  b.add_input<decl::Vector>("Origin"_ustr)
      .default_value(float3(0, 0, 0))
      .description("Sphere center. Planes farther than distance 1 from Origin are skipped.");
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m || m->faces_num < 1) {
    params.error_message_add(NodeWarningType::Info, TIP_("Sphere Arrangement needs a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  const float3 o = params.extract_input<float3>("Origin"_ustr);
  std::string error;
  Mesh *out = geometry::cgal_mesh_sphere_arrangement(*m, o, error);
  if (!out || (out->faces_num == 0 && out->edges_num == 0)) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Sphere Arrangement failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}


static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalSphereArrangement"_ustr, GEO_NODE_CGAL_SPHERE_ARRANGEMENT);
  ntype.ui_name = "Sphere Arrangement";
  ntype.ui_description = "Great circles of face planes on the unit sphere around Origin.";
  ntype.enum_name_legacy = "CGAL_SPHERE_ARRANGEMENT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_sphere_arrangement_cc
