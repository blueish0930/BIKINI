/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_dissolve.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_dissolve_edges_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh whose edges to dissolve");
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Bool>("Selection"_ustr)
      .default_value(true)
      .hide_value()
      .evaluated_geometry_field()
      .description(
          "Edges to remove, the faces at both of their sides are joined. Only edges that are "
          "used by exactly two faces can be dissolved");
  b.add_input<decl::Bool>("Dissolve Vertices"_ustr)
      .default_value(true)
      .description(
          "Also remove the vertices of the dissolved edges that are only connected to two edges "
          "afterwards");
  b.add_input<decl::Float>("Angle Threshold"_ustr)
      .default_value(float(M_PI))
      .min(0.0f)
      .max(float(M_PI))
      .subtype(PROP_ANGLE)
      .description(
          "Vertices are kept if the direction changes by more than this angle from one of their "
          "two edges to the other, which preserves the corners of the joined faces");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection"_ustr);
  const bool dissolve_verts = params.extract_input<bool>("Dissolve Vertices"_ustr);
  const float angle_threshold = params.extract_input<float>("Angle Threshold"_ustr);

  GeometryComponentEditData::remember_deformed_positions_if_necessary(geometry_set);

  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry_set) {
    const Mesh *src_mesh = geometry_set.get_mesh();
    if (src_mesh == nullptr) {
      return;
    }
    const bke::MeshFieldContext context(*src_mesh, AttrDomain::Edge);
    fn::FieldEvaluator evaluator(context, src_mesh->edges_num);
    evaluator.set_selection(selection_field);
    evaluator.evaluate();
    const IndexMask mask = evaluator.get_evaluated_selection_as_mask();
    if (mask.is_empty()) {
      return;
    }

    if (std::optional<Mesh *> dst_mesh = geometry::dissolve_edges(*src_mesh,
                                                                  mask,
                                                                  dissolve_verts,
                                                                  angle_threshold,
                                                                  params.get_attribute_filter(
                                                                      "Mesh"_ustr)))
    {
      geometry_set.replace_mesh(*dst_mesh);
    }
  });

  params.set_output("Mesh"_ustr, std::move(geometry_set));
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeDissolveEdges"_ustr);
  ntype.ui_name = "Dissolve Edges";
  ntype.ui_description = "Remove edges and join the faces at both of their sides into one face";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_dissolve_edges_cc
