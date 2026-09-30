/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * XY constrained Delaunay with per-vertex Z kept (2.5D terrain TIN).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_terrain_tin_cc {

enum class FillRule : int8_t {
  Hull = 0,
  EvenOdd = 1,
};

static const EnumPropertyItem fill_rule_items[] = {
    {int(FillRule::Hull),
     "HULL",
     0,
     N_("Hull"),
     N_("Every finite XY Delaunay triangle (convex hull of the sites).")},
    {int(FillRule::EvenOdd),
     "EVEN_ODD",
     0,
     N_("Even-Odd"),
     N_("Fill the even-odd domain of constraint edges / face borders (holes stay empty).")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Geometry"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::Mesh, GeometryComponent::Type::PointCloud})
      .description(
          "Sites in XY with height in Z. Mesh edges / face borders become constraints.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("2.5D triangle mesh (XY Delaunay, original Z).");
  b.add_input<decl::Menu>("Fill"_ustr)
      .static_items(fill_rule_items)
      .default_value(FillRule::Hull)
      .optional_label()
      .description("Hull fills the convex envelope; Even-Odd uses constraint loops as a domain.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry"_ustr);
  const FillRule fill = params.extract_input<FillRule>("Fill"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  const PointCloud *pc = geometry.get_pointcloud();
  std::string error;
  Mesh *out = nullptr;
  if (mesh && mesh->verts_num >= 3) {
    out = geometry::cgal_mesh_terrain_tin(*mesh, int(fill), error);
  }
  else if (pc && pc->totpoint >= 3) {
    Mesh *tmp = BKE_mesh_new_nomain(pc->totpoint, 0, 0, 0);
    tmp->vert_positions_for_write().copy_from(pc->positions());
    tmp->tag_loose_verts_none();
    tmp->tag_overlapping_none();
    out = geometry::cgal_mesh_terrain_tin(*tmp, 0, error);
    BKE_id_free(nullptr, tmp);
  }
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Terrain TIN failed") : error);
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
  /* Deleted: Terrain TIN. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalTerrainTin"_ustr, GEO_NODE_CGAL_TERRAIN_TIN);
  ntype.ui_name = "Terrain TIN";
  ntype.ui_description =
      "2.5D terrain: constrained Delaunay in XY while keeping each vertex Z "
      "(CGAL Constrained_Delaunay_triangulation_2). Delaunay 2D and Constrained "
      "Delaunay 2D flatten Z to a single height.";
  ntype.enum_name_legacy = "CGAL_TERRAIN_TIN";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_terrain_tin_cc
