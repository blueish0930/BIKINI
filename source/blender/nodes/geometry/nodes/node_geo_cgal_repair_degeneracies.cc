/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL PMP remove_almost_degenerate_faces (needles/caps) + exact degenerates.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

#include <algorithm>
#include <cmath>

namespace blender::nodes::node_geo_cgal_repair_degeneracies_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description(
          "Triangle mesh that may contain zero-area faces, needle slivers, or cap triangles "
          "(one very flat angle). Use after boolean, remesh, or imported CAD.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Same mesh with exact degenerates deleted and needles/caps collapsed or removed. "
          "Attributes are reprojected when possible.");
  b.add_input<decl::Float>("Cap Angle"_ustr)
      .default_value(160.0f * float(M_PI) / 180.0f)
      .min(90.0f * float(M_PI) / 180.0f)
      .max(179.9f * float(M_PI) / 180.0f)
      .subtype(PROP_ANGLE)
      .description(
          "A triangle is a cap if one corner is at least this flat. "
          "Default 160° (CGAL). Raise toward 179° to only remove almost-straight corners (safer). "
          "Lower toward 90° also removes milder caps (more aggressive).");
  b.add_input<decl::Float>("Needle Threshold"_ustr)
      .default_value(4.0f)
      .min(1.0f)
      .description(
          "A face is a needle if longest_edge / shortest_edge ≥ this. "
          "Default 4 (CGAL's own default). Raise to 8–16 to only kill very skinny slivers. "
          "Lower toward 2 to also remove moderately skinny triangles.");
  b.add_input<decl::Float>("Collapse Length"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description(
          "Optional extra limit: an edge shorter than this may be collapsed while cleaning. "
          "0 = no extra length limit (recommended). Set a small world-space length only if "
          "tiny edges should be welded as well.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float cap_angle_rad = params.extract_input<float>("Cap Angle"_ustr);
  const float needle = params.extract_input<float>("Needle Threshold"_ustr);
  const float collapse = params.extract_input<float>("Collapse Length"_ustr);
  /* PROP_ANGLE is radians. CGAL wants cos(largest-angle). */
  const float cap = std::cos(std::clamp(cap_angle_rad, 0.0f, 3.14159265f));
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *mesh = geometry::cgal_mesh_repair_degeneracies(*mesh_in, cap, needle, collapse, error);
  if (!mesh || (mesh->faces_num == 0 && mesh->verts_num == 0)) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Repair Degeneracies failed") : error);
    if (mesh) {
      BKE_id_free(nullptr, mesh);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalRepairDegeneracies"_ustr, GEO_NODE_CGAL_REPAIR_DEGENERACIES);
  ntype.ui_name = "Repair Degeneracies";
  ntype.ui_description =
      "Clean zero-area, needle (very skinny) and cap (one almost-flat corner) triangles. "
      "How to use: plug a Mesh → leave Cap Angle at 160° and Needle at 4 (CGAL defaults) → "
      "leave Collapse Length at 0 unless you also want to weld shorter edges. "
      "Safer: Cap Angle 170–179°, Needle 8+. More aggressive: Cap Angle 120–150°, Needle 2–3. "
      "Finer than Remove Degenerate, which only deletes exact zero-area faces.";
  ntype.enum_name_legacy = "CGAL_REPAIR_DEGENERACIES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_repair_degeneracies_cc
