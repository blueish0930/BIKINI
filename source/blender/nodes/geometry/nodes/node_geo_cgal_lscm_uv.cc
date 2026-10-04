/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL LSCM as a field node (context mesh evaluation).
 * Inputs: face Selection, edge Seam, algorithm params.
 * Output: Corner-domain UV (float3 xyz with z=0), same convention as UV Unwrap / UVMap.
 */

#include "BKE_mesh.hh"

#include "GEO_cgal.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_lscm_uv_cc {

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
      .description("Edges where the mesh is cut for unwrapping (Edge domain). "
                  "Required on closed meshes to create a boundary.");
  b.add_input<decl::Bool>("Normalize"_ustr)
      .default_value(true)
      .description(
          "Fit packed UV atlas into approximately [0,1]^2. Island sizes stay proportional to "
          "3D surface area (uniform texel density).");
  b.add_output<decl::Vector>("UV"_ustr)
      .structure_type(StructureType::Field)
      .propagate_references()
      .description("UV coordinates (x,y) per face corner; z is 0. Store as Corner UVMap.");
}

static VArray<float3> construct_lscm_uv_varray(const Mesh &mesh,
                                               const Field<bool> &selection_field,
                                               const Field<bool> &seam_field,
                                               const bool normalize,
                                               const AttrDomain domain)
{
  const OffsetIndices faces = mesh.faces();
  const Span<int2> edges = mesh.edges();

  const bke::MeshFieldContext face_context{mesh, AttrDomain::Face};
  fn::FieldEvaluator face_evaluator{face_context, faces.size()};
  face_evaluator.add(selection_field);
  face_evaluator.evaluate();
  const VArray<bool> face_sel = face_evaluator.get_evaluated<bool>(0);

  const bke::MeshFieldContext edge_context{mesh, AttrDomain::Edge};
  fn::FieldEvaluator edge_evaluator{edge_context, edges.size()};
  edge_evaluator.add(seam_field);
  edge_evaluator.evaluate();
  const VArray<bool> seam_sel = edge_evaluator.get_evaluated<bool>(0);

  Array<bool> face_bool(mesh.faces_num);
  Array<bool> seam_bool(mesh.edges_num);
  face_sel.materialize(face_bool.as_mutable_span());
  seam_sel.materialize(seam_bool.as_mutable_span());

  Array<float2> corner_uv(mesh.corners_num, float2(0.0f));
  std::string error;
  if (!geometry::cgal_mesh_lscm_uv_corners(
          mesh, seam_bool.as_span(), face_bool.as_span(), normalize, corner_uv.as_mutable_span(), error))
  {
    /* Field nodes cannot easily emit warnings; return zeros on failure. */
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

class LscmUVFieldInput final : public bke::MeshFieldInput {
 private:
  Field<bool> selection_;
  Field<bool> seam_;
  bool normalize_;

 public:
  LscmUVFieldInput(Field<bool> selection, Field<bool> seam, const bool normalize)
      : bke::MeshFieldInput(CPPType::get<float3>(), "CGAL LSCM UV Field"),
        selection_(std::move(selection)),
        seam_(std::move(seam)),
        normalize_(normalize)
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    return construct_lscm_uv_varray(mesh, selection_, seam_, normalize_, domain);
  }

  void foreach_recursive_field(FunctionRef<void(const GField &)> fn) const override
  {
    fn(selection_);
    fn(seam_);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep &deep_hash_cache) const override
  {
    static constexpr int8_t id = 1;
    hash.add(&id);
    hash.add(deep_hash_cache.ensure(selection_));
    hash.add(deep_hash_cache.ensure(seam_));
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
  const bool normalize = params.extract_input<bool>("Normalize"_ustr);
  params.set_output("UV"_ustr,
                    Field<float3>::from_input<LscmUVFieldInput>(selection, seam, normalize));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalLscmUV"_ustr, GEO_NODE_CGAL_LSCM_UV);
  ntype.ui_name = "LSCM UV";
  ntype.ui_description =
      "Least Squares Conformal Map UV (CGAL LSCM). Field on context mesh → Corner UV. "
      "Each connected component is unwrapped as its own island (then packed if Normalize). "
      "Closed shells need Seam edges. Store with Store Named Attribute (Corner Vector/Float2).";
  ntype.enum_name_legacy = "CGAL_LSCM_UV";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_lscm_uv_cc
