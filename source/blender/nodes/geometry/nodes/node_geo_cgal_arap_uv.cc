/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL ARAP UV as a field node (context mesh evaluation).
 * Output: Corner-domain UV (float3, z=0) for Store Named Attribute / UVMap.
 */

#include "BKE_mesh.hh"

#include "GEO_cgal.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_arap_uv_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Bool>("Selection"_ustr)
      .default_value(true)
      .hide_value()
      .structure_type(StructureType::Field)
      .description("Faces to parameterize (Face domain).");
  b.add_input<decl::Bool>("Seam"_ustr)
      .default_value(false)
      .hide_value()
      .structure_type(StructureType::Field)
      .description("Edge seams that cut the mesh for unwrapping.");
  b.add_input<decl::Float>("Lambda"_ustr)
      .default_value(1000.0f)
      .min(0.0f)
      .description("ARAP energy weight: larger = more rigid (shape-preserving).");
  b.add_input<decl::Int>("Iterations"_ustr).default_value(50).min(1).max(500).description(
      "Max energy minimization iterations.");
  b.add_input<decl::Bool>("Normalize"_ustr)
      .default_value(true)
      .description(
          "Fit packed UV atlas into approximately [0,1]^2. Island sizes stay proportional to "
          "3D surface area (uniform texel density).");
  b.add_output<decl::Vector>("UV"_ustr)
      .structure_type(StructureType::Field)
      .propagate_references()
      .description("Corner UV (x,y), z=0. Store as Corner UVMap.");
}

static VArray<float3> construct_uv(const Mesh &mesh,
                                   const Field<bool> &selection_field,
                                   const Field<bool> &seam_field,
                                   const float lambda,
                                   const int iterations,
                                   const bool normalize,
                                   const AttrDomain domain)
{
  const bke::MeshFieldContext face_context{mesh, AttrDomain::Face};
  fn::FieldEvaluator face_evaluator{face_context, mesh.faces_num};
  face_evaluator.add(selection_field);
  face_evaluator.evaluate();
  Array<bool> face_bool(mesh.faces_num);
  face_evaluator.get_evaluated<bool>(0).materialize(face_bool.as_mutable_span());

  const bke::MeshFieldContext edge_context{mesh, AttrDomain::Edge};
  fn::FieldEvaluator edge_evaluator{edge_context, mesh.edges_num};
  edge_evaluator.add(seam_field);
  edge_evaluator.evaluate();
  Array<bool> seam_bool(mesh.edges_num);
  edge_evaluator.get_evaluated<bool>(0).materialize(seam_bool.as_mutable_span());

  Array<float2> corner_uv(mesh.corners_num, float2(0.0f));
  std::string error;
  if (!geometry::cgal_mesh_parameterize_uv_corners(mesh,
                                                   seam_bool.as_span(),
                                                   face_bool.as_span(),
                                                   normalize,
                                                   1,
                                                   iterations,
                                                   lambda,
                                                   corner_uv.as_mutable_span(),
                                                   error))
  {
    Array<float3> zeros(mesh.corners_num, float3(0.0f));
    return mesh.attributes().adapt_domain<float3>(
        VArray<float3>::from_container(std::move(zeros)), AttrDomain::Corner, domain);
  }
  Array<float3> uv3(mesh.corners_num);
  for (const int c : IndexRange(mesh.corners_num)) {
    uv3[c] = float3(corner_uv[c].x, corner_uv[c].y, 0.0f);
  }
  return mesh.attributes().adapt_domain<float3>(
      VArray<float3>::from_container(std::move(uv3)), AttrDomain::Corner, domain);
}

class ArapUVFieldInput final : public bke::MeshFieldInput {
  Field<bool> selection_;
  Field<bool> seam_;
  float lambda_;
  int iterations_;
  bool normalize_;

 public:
  ArapUVFieldInput(Field<bool> selection,
                   Field<bool> seam,
                   const float lambda,
                   const int iterations,
                   const bool normalize)
      : bke::MeshFieldInput(CPPType::get<float3>(), "CGAL ARAP UV Field"),
        selection_(std::move(selection)),
        seam_(std::move(seam)),
        lambda_(lambda),
        iterations_(iterations),
        normalize_(normalize)
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    return construct_uv(mesh, selection_, seam_, lambda_, iterations_, normalize_, domain);
  }

  void foreach_recursive_field(FunctionRef<void(const GField &)> fn) const override
  {
    fn(selection_);
    fn(seam_);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep &deep_hash_cache) const override
  {
    static constexpr int8_t id = 2;
    hash.add(&id);
    hash.add(deep_hash_cache.ensure(selection_));
    hash.add(deep_hash_cache.ensure(seam_));
    hash.add(lambda_);
    hash.add(iterations_);
    hash.add(normalize_);
  }

  std::optional<AttrDomain> preferred_domain(const Mesh & /*mesh*/) const override
  {
    return AttrDomain::Corner;
  }

  bke::NativeFieldDomain native_domain_info(const Mesh & /*mesh*/) const override
  {
    return bke::NativeFieldDomain::Domain{AttrDomain::Corner};
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  const Field<bool> selection = params.extract_input<Field<bool>>("Selection"_ustr);
  const Field<bool> seam = params.extract_input<Field<bool>>("Seam"_ustr);
  const float lambda = params.extract_input<float>("Lambda"_ustr);
  const int iterations = params.extract_input<int>("Iterations"_ustr);
  const bool normalize = params.extract_input<bool>("Normalize"_ustr);
  params.set_output(
      "UV"_ustr,
      Field<float3>::from_input<ArapUVFieldInput>(selection, seam, lambda, iterations, normalize));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalArapUV"_ustr, GEO_NODE_CGAL_ARAP_UV);
  ntype.ui_name = "ARAP UV";
  ntype.ui_description =
      "As-Rigid-As-Possible UV parameterization (CGAL ARAP_parameterizer_3). Field node: Seam + "
      "Selection + Normalize. Output Corner UV for UVMap.";
  ntype.enum_name_legacy = "CGAL_ARAP_UV";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_arap_uv_cc
