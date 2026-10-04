/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_subdivision_cc {

enum class Mode { Loop = 0, CatmullClark = 1, DooSabin = 2, Sqrt3 = 3 };

static const EnumPropertyItem mode_items[] = {
    {int(Mode::Loop), "LOOP", 0, N_("Loop"), N_("Loop subdivision (triangles)")},
    {int(Mode::CatmullClark), "CATMULL_CLARK", 0, N_("Catmull-Clark"), N_("Catmull-Clark subdivision")},
    {int(Mode::DooSabin), "DOO_SABIN", 0, N_("Doo-Sabin"), N_("Doo-Sabin subdivision")},
    {int(Mode::Sqrt3), "SQRT3", 0, N_("Sqrt3"), N_("Sqrt(3) subdivision")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description(
          "Mesh to subdivide. Loop / Sqrt3 work best on triangles; "
          "Catmull-Clark / Doo-Sabin keep quads.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Subdivided mesh (4× faces per Loop/Catmull-Clark step).");
  b.add_input<decl::Menu>("Mode"_ustr)
      .static_items(mode_items)
      .default_value(Mode::Loop)
      .optional_label()
      .description(
          "Loop: triangles. Catmull-Clark: quads (smooth). "
          "Doo-Sabin: dual-style. Sqrt3: slower triangle growth.");
  b.add_input<decl::Int>("Steps"_ustr)
      .default_value(1)
      .min(1)
      .max(5)
      .description("Subdivision levels. Each step roughly quadruples faces — keep this low.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mode mode = params.extract_input<Mode>("Mode"_ustr);
  const int steps = params.extract_input<int>("Steps"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) { params.set_output("Mesh"_ustr, GeometrySet()); return; }
  std::string error;
  Mesh *mesh = geometry::cgal_mesh_subdivision(
      *mesh_in, geometry::CgalSubdivisionMode(int(mode)), steps, error);
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("CGAL Subdivision failed") : error);
    if (mesh) { BKE_id_free(nullptr, mesh); }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalSubdivision"_ustr, GEO_NODE_CGAL_SUBDIVISION);
  ntype.ui_name = "Subdivision";
  ntype.ui_description =
      "Classic surface subdivision (CGAL Subdivision_method_3). "
      "How to use: plug a Mesh → pick Mode → keep Steps at 1–2. "
      "Loop / Sqrt3 for triangles; Catmull-Clark for quads. "
      "This is smoothing subdivision, not isotropic remesh.";
  ntype.enum_name_legacy = "CGAL_SUBDIVISION";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_subdivision_cc
