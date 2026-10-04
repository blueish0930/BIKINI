/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "GEO_cgal.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_mark_self_intersect_cc {

static void write_face_bool(Mesh &mesh, const StringRef name, const Span<bool> values)
{
  if (mesh.faces_num != values.size()) {
    return;
  }
  bke::SpanAttributeWriter<bool> w = mesh.attributes_for_write().lookup_or_add_for_write_only_span<
      bool>(name, bke::AttrDomain::Face);
  if (w) {
    w.span.copy_from(values);
    w.finish();
  }
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to inspect (geometry kept; attributes added).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Same mesh with Face bools \"Self Intersect\" and \"Inside\".");
  b.add_output<decl::Bool>("Self Intersect"_ustr)
      .anonymous_attribute_output()
      .description(
          "Face: true if this face properly crosses another (segment through both interiors, "
          "or a bow-tie / fold-over). Vertex-touch, flush coplanar faces, and one-sided "
          "T-junctions are false.");
  b.add_output<decl::Bool>("Inside"_ustr)
      .anonymous_attribute_output()
      .description(
          "Face: lies in the self-intersection interior (winding >= 2). "
          "Intersection faces are false.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh_in = g.get_mesh();
  if (!mesh_in || mesh_in->faces_num == 0) {
    params.set_default_remaining_outputs();
    return;
  }

  const std::optional<std::string> hit_id =
      params.get_output_anonymous_attribute_id_if_needed("Self Intersect"_ustr);
  const std::optional<std::string> inside_id =
      params.get_output_anonymous_attribute_id_if_needed("Inside"_ustr);

  Array<bool> hit(mesh_in->faces_num, false);
  Array<bool> inside(mesh_in->faces_num, false);
  std::string error;
  if (!geometry::cgal_mesh_mark_self_intersect(*mesh_in, hit, inside, error)) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Mark Self Intersect failed") : error);
    params.set_output("Mesh"_ustr, std::move(g));
    params.set_default_remaining_outputs();
    return;
  }

  Mesh *mesh = g.get_mesh_for_write();
  if (!mesh) {
    params.set_output("Mesh"_ustr, std::move(g));
    params.set_default_remaining_outputs();
    return;
  }
  write_face_bool(*mesh, "Self Intersect", hit);
  write_face_bool(*mesh, "Inside", inside);
  if (hit_id) {
    write_face_bool(*mesh, *hit_id, hit);
  }
  if (inside_id) {
    write_face_bool(*mesh, *inside_id, inside);
  }
  params.set_output("Mesh"_ustr, std::move(g));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalMarkSelfIntersect"_ustr, GEO_NODE_CGAL_MARK_SELF_INTERSECT);
  ntype.ui_name = "Mark Self Intersect";
  ntype.ui_description =
      "Keep the mesh and mark faces: \"Self Intersect\" where two sheets properly cross, and "
      "\"Inside\" for faces in the overlap interior (winding >= 2). Intersection faces are never "
      "Inside.";
  ntype.enum_name_legacy = "CGAL_MARK_SELF_INTERSECT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_mark_self_intersect_cc
