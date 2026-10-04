/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * 2D convex decomposition: user-specified hull count.
 * Splits the XY point set and emits one convex n-gon hull per part.
 */

#include "BLI_math_matrix_types.hh"

#include "BKE_instances.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "GEO_cgal.hh"
#include "GEO_join_geometries.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_convex_partition_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("XY polygon or mesh. Vertices are clustered into convex hulls");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Joined convex n-gon hulls. Face attribute convex_piece stores the hull id");
  b.add_output<decl::Geometry>("Instances"_ustr)
      .description("One instance per convex hull");
  b.add_output<decl::Int>("Pieces"_ustr).description("Number of convex hulls produced");
  b.add_input<decl::Int>("Hull Count"_ustr)
      .default_value(8)
      .min(1)
      .max(256)
      .description("Maximum number of convex hulls");
}

static void output_hulls(GeoNodeExecParams &params, Vector<Mesh *> hulls)
{
  if (hulls.is_empty()) {
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Instances"_ustr, GeometrySet());
    params.set_output("Pieces"_ustr, 0);
    return;
  }

  Vector<GeometrySet> join_sets;
  join_sets.reserve(hulls.size());
  auto instances = std::make_unique<bke::Instances>(int(hulls.size()));
  MutableSpan<int> handles = instances->reference_handles_for_write();
  instances->transforms_for_write().fill(float4x4::identity());

  for (const int i : hulls.index_range()) {
    Mesh *owned = hulls[i];
    /* Copy first. Wrapping the same Mesh* as ReadOnly (join) and Owned (instances)
     * makes join_geometries share CustomData with a mesh that Instances will free. */
    join_sets.append(GeometrySet::from_mesh(BKE_mesh_copy_for_eval(*owned)));
    handles[i] = instances->add_reference(
        bke::InstanceReference{GeometrySet::from_mesh(owned)});
  }

  GeometrySet joined = geometry::join_geometries(join_sets, {});
  joined.ensure_owns_direct_data();

  params.set_output("Mesh"_ustr, std::move(joined));
  params.set_output("Instances"_ustr, GeometrySet::from_instances(std::move(instances)));
  params.set_output("Pieces"_ustr, int(hulls.size()));
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const int hull_count = math::clamp(params.extract_input<int>("Hull Count"_ustr), 1, 256);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->verts_num < 3) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Convex Partition 2D needs a mesh with at least 3 vertices"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Instances"_ustr, GeometrySet());
    params.set_output("Pieces"_ustr, 0);
    return;
  }
  std::string error;
  Vector<Mesh *> hulls = geometry::cgal_mesh_convex_hulls_2(*mesh, hull_count, error);
  if (hulls.is_empty()) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Convex Partition 2D failed") : error);
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Instances"_ustr, GeometrySet());
    params.set_output("Pieces"_ustr, 0);
    return;
  }
  output_hulls(params, std::move(hulls));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalConvexPartition2"_ustr, GEO_NODE_CGAL_CONVEX_PARTITION_2);
  ntype.ui_name = "Convex Partition 2D";
  ntype.ui_description =
      "Approximate an XY shape with a user-specified number of convex hulls. "
      "Much faster than CGAL polygon partition; each piece is a convex n-gon.";
  ntype.enum_name_legacy = "CGAL_CONVEX_PARTITION_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_convex_partition_2_cc
