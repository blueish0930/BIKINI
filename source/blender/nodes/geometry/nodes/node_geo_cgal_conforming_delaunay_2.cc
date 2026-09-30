/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Insert Steiner points so constrained edges become Delaunay or Gabriel.
 * Distinct from Constrained Delaunay 2D (no Steiner) and deleted CDT Plus 2D.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_conforming_delaunay_2_cc {

enum class Mode {
  Delaunay = 0,
  Gabriel = 1,
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  static const EnumPropertyItem mode_items[] = {
      {int(Mode::Delaunay),
       "DELAUNAY",
       0,
       N_("Delaunay"),
       N_("Insert Steiner points until every constraint edge is a Delaunay edge")},
      {int(Mode::Gabriel),
       "GABRIEL",
       0,
       N_("Gabriel"),
       N_("Insert Steiner points until every constraint edge is a Gabriel edge")},
      {0, nullptr, 0, nullptr, nullptr},
  };
  b.add_input<decl::Menu>("Mode"_ustr)
      .static_items(mode_items)
      .default_value(MenuValue(Mode::Delaunay))
      .optional_label();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("XY polygon / PSLG. Borders become constraint edges.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Conforming triangulation (Steiner points on constraint edges).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mode mode = params.extract_input<Mode>("Mode"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->verts_num < 3) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Conforming Delaunay 2D needs a polygon mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_conforming_delaunay_2(*mesh, int(mode), error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Conforming Delaunay 2D failed") : error);
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
  /* Deleted: Conforming Delaunay 2D (legacy 2505 / 2592). */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalConformingDelaunay2"_ustr, GEO_NODE_CGAL_CONFORMING_DELAUNAY_2);
  ntype.ui_name = "Conforming Delaunay 2D";
  ntype.ui_description =
      "Insert Steiner points so every constraint edge is Delaunay or Gabriel "
      "(CGAL make_conforming_Delaunay_2). Constrained Delaunay 2D does not add Steiner points.";
  ntype.enum_name_legacy = "CGAL_CONFORMING_DELAUNAY_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_conforming_delaunay_2_cc
