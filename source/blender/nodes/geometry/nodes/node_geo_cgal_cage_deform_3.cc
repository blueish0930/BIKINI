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

namespace blender::nodes::node_geo_cgal_cage_deform_3_cc {


static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Interior mesh to deform. Sits in the rest cage.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Interior mesh after applying rest→pose cage deltas (same topology).");
  b.add_input<decl::Geometry>("Cage Rest"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Cage in the bind pose. Must share vertex index order with Cage Pose.");
  b.add_input<decl::Geometry>("Cage Pose"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Deformed cage. If equal to Rest, the interior is unchanged.");
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m || m->faces_num < 1) {
    params.error_message_add(NodeWarningType::Info, TIP_("Cage Deform 3D needs a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  GeometrySet gr = params.extract_input<GeometrySet>("Cage Rest"_ustr);
  GeometrySet gp = params.extract_input<GeometrySet>("Cage Pose"_ustr);
  const Mesh *rest = gr.get_mesh();
  const Mesh *pose = gp.get_mesh();
  if (!rest || !pose) { params.set_output("Mesh"_ustr, GeometrySet()); return; }
  std::string error;
  Mesh *out = geometry::cgal_mesh_cage_deform_3(*rest, *pose, *m, error);
  if (!out || (out->faces_num == 0 && out->edges_num == 0)) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Cage Deform 3D failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}


static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalCageDeform3"_ustr, GEO_NODE_CGAL_CAGE_DEFORM_3);
  ntype.ui_name = "Cage Deform 3D";
  ntype.ui_description =
      "Cage-deform an interior mesh: inverse-distance weights on the rest cage, applied to "
      "(pose − rest) vertex deltas so two identical cages leave the interior unchanged. Rest and "
      "Pose must have the same vertex count and index order (duplicate a cube, then move Pose).";
  ntype.enum_name_legacy = "CGAL_CAGE_DEFORM_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_cage_deform_3_cc
