/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Exact multi-source geodesic distance field (CGAL Surface_mesh_shortest_path).
 * Distinct from Blender Heat Geodesic (approximate heat method).
 */

#include "BKE_mesh.hh"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_exact_geodesic_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Bool>("Source"_ustr)
      .default_value(false)
      .hide_value()
      .structure_type(StructureType::Field)
      .description("Source vertices (Point domain). Distance is 0 at sources.");
  b.add_output<decl::Float>("Distance"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "Exact geodesic distance along the surface to the nearest source. "
          "Unreachable components get -1.");
}

static VArray<float> construct_distance(const Mesh &mesh,
                                        const Field<bool> &source_field,
                                        const AttrDomain domain)
{
  Array<bool> sources(mesh.verts_num, false);
  const bke::MeshFieldContext context{mesh, AttrDomain::Point};
  fn::FieldEvaluator evaluator{context, mesh.verts_num};
  evaluator.add(source_field);
  evaluator.evaluate();
  evaluator.get_evaluated<bool>(0).materialize(sources.as_mutable_span());

  Array<float> dist(mesh.verts_num, -1.0f);
  if (mesh.verts_num > 0 && mesh.faces_num > 0) {
    std::string error;
    geometry::cgal_mesh_exact_geodesic_distances(mesh, sources, dist, error);
  }
  return mesh.attributes().adapt_domain<float>(
      VArray<float>::from_container(std::move(dist)), AttrDomain::Point, domain);
}

class ExactGeodesicFieldInput final : public bke::MeshFieldInput {
  Field<bool> source_;

 public:
  explicit ExactGeodesicFieldInput(Field<bool> source)
      : bke::MeshFieldInput(CPPType::get<float>(), "CGAL Exact Geodesic Distance Field"),
        source_(std::move(source))
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    return construct_distance(mesh, source_, domain);
  }

  void foreach_recursive_field(FunctionRef<void(const GField &)> fn) const override
  {
    fn(source_);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep &deep_hash_cache) const override
  {
    static constexpr int8_t id = 10;
    hash.add(&id);
    hash.add(deep_hash_cache.ensure(source_));
  }

  std::optional<AttrDomain> preferred_domain(const Mesh & /*mesh*/) const override
  {
    return AttrDomain::Point;
  }

  bke::NativeFieldDomain native_domain_info(const Mesh & /*mesh*/) const override
  {
    return bke::NativeFieldDomain::Domain{AttrDomain::Point};
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  const Field<bool> source = params.extract_input<Field<bool>>("Source"_ustr);
  params.set_output("Distance"_ustr, Field<float>::from_input<ExactGeodesicFieldInput>(source));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalExactGeodesic"_ustr, GEO_NODE_CGAL_EXACT_GEODESIC);
  ntype.ui_name = "Exact Geodesic Distance";
  ntype.ui_description =
      "Exact multi-source geodesic distance on the mesh surface (CGAL Surface_mesh_shortest_path). "
      "Field node: mark Source verts → Distance per point. Unreachable = -1. "
      "Different from Heat Geodesic (approximate).";
  ntype.enum_name_legacy = "CGAL_EXACT_GEODESIC";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_exact_geodesic_cc
