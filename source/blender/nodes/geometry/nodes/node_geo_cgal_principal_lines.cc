/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Short segments along principal curvature directions.
 * Distinct from Principal Curvature (fields) and deleted Curvature Isolines
 * (scalar isolines).
 */

#include "BKE_curves.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_principal_lines_cc {

enum class Mode : int8_t {
  Min = 0,
  Max = 1,
  Both = 2,
};

static const EnumPropertyItem mode_items[] = {
    {int(Mode::Min), "MIN", 0, N_("Min"), N_("Direction of the smaller principal curvature k_min.")},
    {int(Mode::Max), "MAX", 0, N_("Max"), N_("Direction of the larger principal curvature k_max.")},
    {int(Mode::Both), "BOTH", 0, N_("Both"), N_("Both principal directions at each sample.")},
    {0, nullptr, 0, nullptr, nullptr},
};

static Curves *segments_to_curves(const Vector<float3> &a, const Vector<float3> &b)
{
  const int n = int(a.size());
  if (n < 1 || int(b.size()) != n) {
    return bke::curves_new_nomain(0, 0);
  }
  Curves *curves_id = bke::curves_new_nomain(n * 2, n);
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  MutableSpan<int> offsets = curves.offsets_for_write();
  MutableSpan<float3> positions = curves.positions_for_write();
  offsets[0] = 0;
  for (int i = 0; i < n; i++) {
    positions[i * 2 + 0] = a[i];
    positions[i * 2 + 1] = b[i];
    offsets[i + 1] = (i + 1) * 2;
  }
  curves.fill_curve_types(CURVE_TYPE_POLY);
  return curves_id;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle surface.");
  b.add_output<decl::Geometry>("Curves"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Short poly segments along principal directions.");
  b.add_input<decl::Menu>("Mode"_ustr)
      .static_items(mode_items)
      .default_value(Mode::Max)
      .optional_label();
  b.add_input<decl::Float>("Length"_ustr)
      .default_value(0.05f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Half-length of each hatch segment.");
  b.add_input<decl::Int>("Stride"_ustr)
      .default_value(1)
      .min(1)
      .max(64)
      .description("Sample every Nth vertex (1 = all).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mode mode = params.extract_input<Mode>("Mode"_ustr);
  const float length = std::max(0.0f, params.extract_input<float>("Length"_ustr));
  const int stride = std::max(1, params.extract_input<int>("Stride"_ustr));
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->verts_num < 3 || mesh->faces_num < 1) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Principal Lines needs a mesh with faces"));
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  Array<float> kmin(mesh->verts_num, 0.0f);
  Array<float> kmax(mesh->verts_num, 0.0f);
  Array<float3> dmin(mesh->verts_num, float3(0.0f));
  Array<float3> dmax(mesh->verts_num, float3(0.0f));
  std::string error;
  if (!geometry::cgal_mesh_principal_curvatures(
          *mesh, kmin, kmax, dmin, dmax, error))
  {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Principal Lines failed") : error);
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  const Span<float3> pos = mesh->vert_positions();
  Vector<float3> a;
  Vector<float3> b;
  auto emit = [&](const float3 &p, float3 dir) {
    if (math::length_squared(dir) < 1.0e-12f) {
      return;
    }
    dir = math::normalize(dir);
    a.append(p - dir * length);
    b.append(p + dir * length);
  };
  for (int i = 0; i < mesh->verts_num; i += stride) {
    if (mode == Mode::Min || mode == Mode::Both) {
      emit(pos[i], dmin[i]);
    }
    if (mode == Mode::Max || mode == Mode::Both) {
      emit(pos[i], dmax[i]);
    }
  }
  Curves *curves = segments_to_curves(a, b);
  if (curves->geometry.wrap().is_empty()) {
    params.error_message_add(NodeWarningType::Info, TIP_("No principal direction samples"));
  }
  params.set_output("Curves"_ustr, GeometrySet::from_curves(curves));
}

static void node_register()
{
  /* Deleted: Principal Lines. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalPrincipalLines"_ustr, GEO_NODE_CGAL_PRINCIPAL_LINES);
  ntype.ui_name = "Principal Lines";
  ntype.ui_description =
      "Hatch a surface with short segments along principal curvature directions "
      "(CGAL interpolated corrected curvatures). Principal Curvature is a field; "
      "this draws the line field. Not scalar isolines.";
  ntype.enum_name_legacy = "CGAL_PRINCIPAL_LINES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_principal_lines_cc
