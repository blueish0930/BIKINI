/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "GEO_cgal.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_corefine_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh 1"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Primary mesh that receives intersection edges.");
  b.add_input<decl::Geometry>("Mesh 2"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Cutting mesh used only to compute intersections.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Mesh 1 after embedding intersection seams.");
  b.add_output<decl::Bool>("Seam Edges"_ustr)
      .anonymous_attribute_output()
      .description("Boolean Edge field: true on edges created/lying on the intersection seams.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet ga = params.extract_input<GeometrySet>("Mesh 1"_ustr);
  GeometrySet gb = params.extract_input<GeometrySet>("Mesh 2"_ustr);
  const Mesh *a = ga.get_mesh();
  const Mesh *b = gb.get_mesh();
  if (!a || !b) {
    params.error_message_add(NodeWarningType::Info, TIP_("Both inputs need a mesh"));
    params.set_default_remaining_outputs();
    return;
  }

  const std::optional<std::string> seam_attr_id =
      params.get_output_anonymous_attribute_id_if_needed("Seam Edges"_ustr);

  std::string error;
  Array<bool> seam_edges;
  Mesh *mesh = geometry::cgal_mesh_corefine(
      *a, *b, error, seam_attr_id ? &seam_edges : nullptr);
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("CGAL Corefine failed") : error);
    if (mesh) {
      BKE_id_free(nullptr, mesh);
    }
    params.set_default_remaining_outputs();
    return;
  }

  if (seam_attr_id && seam_edges.size() == mesh->edges_num) {
    bke::MutableAttributeAccessor attrs = mesh->attributes_for_write();
    bke::SpanAttributeWriter<bool> writer = attrs.lookup_or_add_for_write_only_span<bool>(
        *seam_attr_id, bke::AttrDomain::Edge);
    writer.span.copy_from(seam_edges);
    writer.finish();
  }

  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalCorefine"_ustr, GEO_NODE_CGAL_COREFINE);
  ntype.ui_name = "Corefine";
  ntype.ui_description =
      "Embed the intersection curves of Mesh 2 into Mesh 1 (PMP corefine). Outputs an Edge selection of those seams on Mesh 1.";
  ntype.enum_name_legacy = "CGAL_COREFINE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_corefine_cc
