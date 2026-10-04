/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Constrained Delaunay triangulation of an XY PSLG (no Steiner points).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_constrained_delaunay_2_cc {

enum class FillRule : int8_t {
  EvenOdd = 0,
  NonZero = 1,
};

static const EnumPropertyItem fill_rule_items[] = {
    {int(FillRule::EvenOdd),
     "EVEN_ODD",
     0,
     N_("Even-Odd"),
     N_("Like Fill Curve: odd contour crossings (or an odd number of overlapping faces).")},
    {int(FillRule::NonZero),
     "NON_ZERO",
     0,
     N_("Non-Zero"),
     N_("Like Fill Curve: nonzero winding of contours (or signed face overlap). Same-direction overlaps stay filled.")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("XY polygon / PSLG. Borders become constraint edges.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Constrained Delaunay triangles.");
  b.add_input<decl::Menu>("Fill Rule"_ustr)
      .static_items(fill_rule_items)
      .default_value(FillRule::EvenOdd)
      .optional_label()
      .description("Even-odd or nonzero winding on the outline / 2D silhouette.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const FillRule fill_rule = params.extract_input<FillRule>("Fill Rule"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->verts_num < 3) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Constrained Delaunay 2D needs a polygon mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_constrained_delaunay_2(*mesh, int(fill_rule), error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Constrained Delaunay 2D failed") : error);
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
  geo_node_type_base(
      &ntype, "GeometryNodeCgalConstrainedDelaunay2"_ustr, GEO_NODE_CGAL_CONSTRAINED_DELAUNAY_2);
  ntype.ui_name = "Constrained Delaunay 2D";
  ntype.ui_description =
      "Constrained Delaunay of the input vertices and edges. Wires use Fill Curve winding. "
      "2D faces use overlapping-face winding. A 3D mesh (front and back in XY) is filled "
      "by the union of the projected faces.";
  ntype.enum_name_legacy = "CGAL_CONSTRAINED_DELAUNAY_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_constrained_delaunay_2_cc
