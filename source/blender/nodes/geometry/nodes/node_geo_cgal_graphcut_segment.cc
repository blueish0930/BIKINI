/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_graphcut_segment_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to split into feature patches (planar chunks cut at sharp edges).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Same mesh with face integer attribute Segment (one id per feature patch).");
  b.add_input<decl::Float>("Angle"_ustr)
      .default_value(30.0f)
      .min(1.0f)
      .max(180.0f)
      .description(
          "Feature angle in degrees. Adjacent faces whose normals differ by less than this "
          "stay in the same patch. 30 splits a cube into 6 faces; larger values merge more.");
  b.add_input<decl::Int>("Min Size"_ustr)
      .default_value(1)
      .min(1)
      .max(100000)
      .description("Merge patches with fewer faces into a neighbor. 1 = keep every patch.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float angle = params.extract_input<float>("Angle"_ustr);
  const int minsz = params.extract_input<int>("Min Size"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m || m->faces_num < 1) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_graphcut_segment(*m, angle, minsz, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Graphcut Segment failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalGraphcutSegment"_ustr, GEO_NODE_CGAL_GRAPHCUT_SEGMENT);
  ntype.ui_name = "Graphcut Segment";
  ntype.ui_description =
      "Split a mesh into feature patches: region-grow faces while the dihedral angle is below "
      "Angle, so planar areas stay together and cuts fall on sharp edges. Not a world-axis split. "
      "Writes face attribute Segment. Color by Named Attribute. Distinct from Region Growing "
      "(which fits primitives on points).";
  ntype.enum_name_legacy = "CGAL_GRAPHCUT_SEGMENT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_graphcut_segment_cc
