/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL Variational Shape Approximation — planar proxy mesh approximation.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_vsa_approximate_cc {

enum class SeedingMethod {
  Hierarchical = 0,
  Incremental = 1,
  Random = 2,
};

static const EnumPropertyItem seeding_items[] = {
    {int(SeedingMethod::Hierarchical),
     "HIERARCHICAL",
     0,
     N_("Hierarchical"),
     N_("Hierarchical seeding (default, balanced proxies)")},
    {int(SeedingMethod::Incremental),
     "INCREMENTAL",
     0,
     N_("Incremental"),
     N_("Incremental seeding (add proxies one by one)")},
    {int(SeedingMethod::Random),
     "RANDOM",
     0,
     N_("Random"),
     N_("Random seeding")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle mesh to approximate with planar proxies.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Simplified triangle mesh from VSA anchors.");
  b.add_output<decl::Int>("Proxies"_ustr).description("Number of plane proxies used.");
  b.add_input<decl::Int>("Max Proxies"_ustr)
      .default_value(32)
      .min(1)
      .max(512)
      .description("Maximum number of plane proxies (higher = more detail, slower).");
  b.add_input<decl::Int>("Iterations"_ustr)
      .default_value(30)
      .min(1)
      .max(200)
      .description("Partitioning / fitting iterations after seeding.");
  b.add_input<decl::Menu>("Seeding"_ustr)
      .static_items(seeding_items)
      .default_value(SeedingMethod::Hierarchical)
      .optional_label()
      .description("Proxy seeding strategy.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const int max_proxies = params.extract_input<int>("Max Proxies"_ustr);
  const int iterations = params.extract_input<int>("Iterations"_ustr);
  const SeedingMethod seeding_method = params.extract_input<SeedingMethod>("Seeding"_ustr);
  const int seeding = int(seeding_method);

  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in || mesh_in->faces_num < 4) {
    params.error_message_add(NodeWarningType::Info, TIP_("VSA Approximate needs a mesh with faces"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Proxies"_ustr, 0);
    return;
  }

  std::string error;
  int proxy_count = 0;
  Mesh *out = geometry::cgal_mesh_vsa_approximate(
      *mesh_in, max_proxies, iterations, seeding, proxy_count, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("VSA Approximate failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Proxies"_ustr, 0);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
  params.set_output("Proxies"_ustr, proxy_count);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalVsaApproximate"_ustr, GEO_NODE_CGAL_VSA_APPROXIMATE);
  ntype.ui_name = "VSA Approximate";
  ntype.ui_description =
      "Approximate a triangle mesh with planar proxies (CGAL Variational Shape Approximation). "
      "Outputs a simplified mesh and proxy count. Useful for CAD-like planarization.";
  ntype.enum_name_legacy = "CGAL_VSA_APPROXIMATE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_vsa_approximate_cc
