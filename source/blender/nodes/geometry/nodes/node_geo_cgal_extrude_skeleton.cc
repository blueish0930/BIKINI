/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Extrude Skeleton
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_extrude_skeleton_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Open-border islands on XY (with holes).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("3D roof mesh from straight-skeleton extrusion.");
  b.add_input<decl::Float>("Height"_ustr)
      .default_value(1.0f)
      .description("Maximum extrusion height (non-zero).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float height = params.extract_input<float>("Height"_ustr);
  const bool outward = false;
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh) {
    params.error_message_add(NodeWarningType::Info, TIP_("Extrude Skeleton needs a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  if (fabsf(height) < 1e-8f) {
    params.error_message_add(NodeWarningType::Info, TIP_("Height must be non-zero"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_extrude_skeleton(*mesh, height, outward, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Extrude Skeleton failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalExtrudeSkeleton"_ustr, GEO_NODE_CGAL_EXTRUDE_SKELETON);
  ntype.ui_name = "Extrude Skeleton";
  ntype.ui_description =
      "Raise an XY polygon into a hip-roof solid by extruding its straight skeleton "
      "(CGAL extrude_skeleton). How to use: plug a 2D island Mesh (holes allowed) → "
      "set Height to the ridge height → get a 3D roof. "
      "Related: Straight Skeleton 2D is the 2D ridge diagram; this turns it into volume.";
  ntype.enum_name_legacy = "CGAL_EXTRUDE_SKELETON";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_extrude_skeleton_cc
