/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * 2D boolean of XY border polygons (union / intersection / difference / xor).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_boolean_ops_2_cc {

enum class BooleanMode {
  Union = 0,
  Intersection = 1,
  Difference = 2,
  SymmetricDifference = 3,
};

static const EnumPropertyItem mode_items[] = {
    {int(BooleanMode::Union), "UNION", 0, N_("Union"), N_("A ∪ B")},
    {int(BooleanMode::Intersection),
     "INTERSECT",
     0,
     N_("Intersect"),
     N_("A ∩ B")},
    {int(BooleanMode::Difference),
     "DIFFERENCE",
     0,
     N_("Difference"),
     N_("A − B (region of A not in B)")},
    {int(BooleanMode::SymmetricDifference),
     "XOR",
     0,
     N_("Symmetric Difference"),
     N_("(A ∪ B) − (A ∩ B)")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh A"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("First 2D shape (flattened mesh: union of faces with XY area).");
  b.add_input<decl::Geometry>("Mesh B"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Second 2D shape (optional). Empty = silhouette outline of A only.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Filled 2D boolean region. Holes are empty (cut out), not extra covering faces.");
  b.add_input<decl::Menu>("Operation"_ustr)
      .static_items(mode_items)
      .default_value(BooleanMode::Union)
      .optional_label()
      .description("Union / Intersect / Difference / XOR over all islands of A and B.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet ga = params.extract_input<GeometrySet>("Mesh A"_ustr);
  GeometrySet gb = params.extract_input<GeometrySet>("Mesh B"_ustr);
  const BooleanMode op = params.extract_input<BooleanMode>("Operation"_ustr);
  const int mode = int(op);
  const Mesh *a = ga.get_mesh();
  const Mesh *b = gb.get_mesh();
  if (!a) {
    params.error_message_add(NodeWarningType::Info, TIP_("Boolean Ops 2D needs mesh A"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_boolean_ops_2(*a, b, mode, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Boolean Ops 2D failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalBooleanOps2"_ustr, GEO_NODE_CGAL_BOOLEAN_OPS_2);
  ntype.ui_name = "Boolean Ops 2D";
  ntype.ui_description =
      "2D boolean of flattened XY shapes (faces with XY area). "
      "Fill rule is non-zero winding: overlapping same-orientation faces stay inside, "
      "so Difference punches those overlaps; opposite-orientation overlaps cancel. "
      "How to use Difference: Mesh A = the stock (e.g. a bounding rectangle), "
      "Mesh B = the cutters (scattered grids / islands). The output is the leftover "
      "region with B actually punched out — holes stay empty. "
      "Leave Mesh B empty to get the silhouette of A. "
      "Join or Realize Instances on B first if the cutters are instances.";
  ntype.enum_name_legacy = "CGAL_BOOLEAN_OPS_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_boolean_ops_2_cc
