/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Isolines of interpolated-corrected mean or Gaussian curvature.
 * Distinct from Curvature (field), Geodesic Isolines, Refine at Isolevel.
 */

#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_curvature_isolines_cc {

enum class Mode {
  Mean = 0,
  Gaussian = 1,
};

static Curves *polylines_to_curves(const Span<Vector<float3>> polylines)
{
  int points_num = 0;
  int curves_num = 0;
  for (const Vector<float3> &pl : polylines) {
    if (pl.size() >= 2) {
      points_num += pl.size();
      curves_num++;
    }
  }
  if (curves_num == 0 || points_num < 2) {
    return bke::curves_new_nomain(0, 0);
  }
  Curves *curves_id = bke::curves_new_nomain(points_num, curves_num);
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  MutableSpan<int> offsets = curves.offsets_for_write();
  MutableSpan<float3> positions = curves.positions_for_write();
  int p = 0;
  int c = 0;
  offsets[0] = 0;
  for (const Vector<float3> &pl : polylines) {
    if (pl.size() < 2) {
      continue;
    }
    for (const float3 &pt : pl) {
      positions[p++] = pt;
    }
    c++;
    offsets[c] = p;
  }
  curves.fill_curve_types(CURVE_TYPE_POLY);
  return curves_id;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  static const EnumPropertyItem mode_items[] = {
      {int(Mode::Mean), "MEAN", 0, N_("Mean"), N_("Isolines of mean curvature H")},
      {int(Mode::Gaussian), "GAUSSIAN", 0, N_("Gaussian"), N_("Isolines of Gaussian curvature K")},
      {0, nullptr, 0, nullptr, nullptr},
  };
  b.add_input<decl::Menu>("Mode"_ustr).static_items(mode_items).default_value(MenuValue(Mode::Mean)).optional_label();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle surface.");
  b.add_output<decl::Geometry>("Curves"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Curvature isolines as poly curves.");
  b.add_input<decl::Int>("Count"_ustr)
      .default_value(8)
      .min(1)
      .max(256)
      .description("Number of isolines between min and max curvature.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mode mode = params.extract_input<Mode>("Mode"_ustr);
  const int count = std::max(1, params.extract_input<int>("Count"_ustr));
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Curvature Isolines needs a mesh with faces"));
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Vector<Vector<float3>> polylines;
  if (!geometry::cgal_mesh_curvature_isolines(*mesh, int(mode), count, polylines, error)) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Curvature Isolines failed") : error);
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  Curves *curves = polylines_to_curves(polylines);
  if (curves->geometry.wrap().is_empty()) {
    params.error_message_add(NodeWarningType::Info, TIP_("No curvature isolines"));
  }
  params.set_output("Curves"_ustr, GeometrySet::from_curves(curves));
}

static void node_register()
{
  /* Deleted: Curvature Isolines. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalCurvatureIsolines"_ustr, GEO_NODE_CGAL_CURVATURE_ISOLINES);
  ntype.ui_name = "Curvature Isolines";
  ntype.ui_description =
      "Draw isolines of interpolated-corrected mean or Gaussian curvature "
      "(CGAL interpolated_corrected_curvatures). Curvature is a vertex field; "
      "Geodesic Isolines uses geodesic distance; Refine at Isolevel splits the mesh.";
  ntype.enum_name_legacy = "CGAL_CURVATURE_ISOLINES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_curvature_isolines_cc
