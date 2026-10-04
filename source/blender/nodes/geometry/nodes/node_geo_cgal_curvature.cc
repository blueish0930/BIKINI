/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Curvature as field outputs (context evaluation on the mesh being evaluated).
 * No geometry sockets — Mean / Gaussian / Total Curvature are float fields.
 *
 * Mean and Gaussian come from CGAL interpolated-corrected curvatures (Point).
 * Total Curvature is Chen 2023 (arXiv:2305.12653): Dirichlet energy of the Gauss
 * map on each triangle via the cotangent stiffness matrix, then area-weighted
 * to vertices (or per-face density when evaluated on Face).
 */

#include "BKE_mesh.hh"

#include "BLI_math_geom.hh"
#include "BLI_math_vector.hh"

#include "GEO_cgal.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_curvature_cc {

enum class CurvatureKind {
  Mean = 0,
  Gaussian = 1,
  Total = 2,
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Float>("Mean"_ustr)
      .structure_type(StructureType::Field)
      .description("Mean curvature H at each vertex (sign depends on orientation).");
  b.add_output<decl::Float>("Gaussian"_ustr)
      .structure_type(StructureType::Field)
      .description("Gaussian curvature K (product of principal curvatures).");
  b.add_output<decl::Float>("Total Curvature"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "Total curvature density k1^2 + k2^2 from the Dirichlet energy of the Gauss map "
          "(Chen 2023). Point: 1-ring area-weighted; Face: per-triangle. "
          "Multiply by Face Area for the integrated per-triangle value.");
}

/**
 * Integrated total curvature on one triangle:
 *   κ_T = Trace(N S N^T) = 1/2 Σ_i cot(α_i) ||n_j - n_k||^2
 * where S is the cotangent stiffness matrix (0.5 cot opposite) and N holds
 * vertex normals. Matches Chen 2023 / libigl cotmatrix Dirichlet energy.
 */
static float triangle_integrated_total_curvature(const float3 &p0,
                                                 const float3 &p1,
                                                 const float3 &p2,
                                                 const float3 &n0,
                                                 const float3 &n1,
                                                 const float3 &n2)
{
  const float cot0 = math::cotangent_tri_weight(p0, p1, p2);
  const float cot1 = math::cotangent_tri_weight(p1, p2, p0);
  const float cot2 = math::cotangent_tri_weight(p2, p0, p1);
  const float k = 0.5f *
                  (cot0 * math::length_squared(n1 - n2) + cot1 * math::length_squared(n2 - n0) +
                   cot2 * math::length_squared(n0 - n1));
  return math::max(k, 0.0f);
}

static VArray<float> construct_total_curvature_varray(const Mesh &mesh, const AttrDomain domain)
{
  if (mesh.verts_num == 0 || mesh.faces_num == 0) {
    return mesh.attributes().adapt_domain<float>(
        VArray<float>::from_single(0.0f, mesh.verts_num), AttrDomain::Point, domain);
  }

  const Span<float3> positions = mesh.vert_positions();
  const Span<float3> normals = mesh.vert_normals();
  const Span<int> corner_verts = mesh.corner_verts();
  const Span<int3> corner_tris = mesh.corner_tris();
  const bool want_face = (domain == AttrDomain::Face);
  const Span<int> tri_faces = want_face ? mesh.corner_tri_faces() : Span<int>();
  Array<float> value_accum(want_face ? mesh.faces_num : mesh.verts_num, 0.0f);
  Array<float> area_accum(want_face ? mesh.faces_num : mesh.verts_num, 0.0f);

  for (const int t : corner_tris.index_range()) {
    const int3 corners = corner_tris[t];
    const int i0 = corner_verts[corners[0]];
    const int i1 = corner_verts[corners[1]];
    const int i2 = corner_verts[corners[2]];
    const float3 &p0 = positions[i0];
    const float3 &p1 = positions[i1];
    const float3 &p2 = positions[i2];
    const float area = math::area_tri(p0, p1, p2);
    if (area <= 1.0e-20f) {
      continue;
    }
    const float k = triangle_integrated_total_curvature(
        p0, p1, p2, normals[i0], normals[i1], normals[i2]);
    if (want_face) {
      const int f = tri_faces[t];
      value_accum[f] += k;
      area_accum[f] += area;
    }
    else {
      value_accum[i0] += k;
      value_accum[i1] += k;
      value_accum[i2] += k;
      area_accum[i0] += area;
      area_accum[i1] += area;
      area_accum[i2] += area;
    }
  }

  const AttrDomain native = want_face ? AttrDomain::Face : AttrDomain::Point;
  const int size = want_face ? mesh.faces_num : mesh.verts_num;
  Array<float> values(size, 0.0f);
  for (const int i : IndexRange(size)) {
    if (area_accum[i] > 1.0e-20f) {
      values[i] = value_accum[i] / area_accum[i];
    }
  }
  return mesh.attributes().adapt_domain<float>(
      VArray<float>::from_container(std::move(values)), native, domain);
}

static VArray<float> construct_curvature_varray(const Mesh &mesh,
                                                const AttrDomain domain,
                                                const CurvatureKind kind)
{
  if (kind == CurvatureKind::Total) {
    return construct_total_curvature_varray(mesh, domain);
  }

  Array<float> values(mesh.verts_num, 0.0f);
  if (mesh.verts_num > 0 && mesh.faces_num > 0) {
    std::string error;
    const bool ok = (kind == CurvatureKind::Mean) ?
                        geometry::cgal_mesh_mean_curvature(mesh, values, error) :
                        geometry::cgal_mesh_gaussian_curvature(mesh, values, error);
    if (!ok) {
      values.fill(0.0f);
    }
  }

  return mesh.attributes().adapt_domain<float>(
      VArray<float>::from_container(std::move(values)), AttrDomain::Point, domain);
}

static const char *curvature_field_debug_name(const CurvatureKind kind)
{
  switch (kind) {
    case CurvatureKind::Mean:
      return "CGAL Mean Curvature Field";
    case CurvatureKind::Gaussian:
      return "CGAL Gaussian Curvature Field";
    case CurvatureKind::Total:
      return "Total Curvature Field";
  }
  return "Curvature Field";
}

class CurvatureFieldInput final : public bke::MeshFieldInput {
 private:
  CurvatureKind kind_;

 public:
  explicit CurvatureFieldInput(const CurvatureKind kind)
      : bke::MeshFieldInput(CPPType::get<float>(), curvature_field_debug_name(kind)), kind_(kind)
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    return construct_curvature_varray(mesh, domain, kind_);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep & /*deep_hash_cache*/) const override
  {
    static constexpr int8_t mean_id = 1;
    static constexpr int8_t gaussian_id = 2;
    static constexpr int8_t total_id = 3;
    switch (kind_) {
      case CurvatureKind::Mean:
        hash.add(&mean_id);
        break;
      case CurvatureKind::Gaussian:
        hash.add(&gaussian_id);
        break;
      case CurvatureKind::Total:
        hash.add(&total_id);
        break;
    }
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
  params.set_output("Mean"_ustr,
                    Field<float>::from_input<CurvatureFieldInput>(CurvatureKind::Mean));
  params.set_output("Gaussian"_ustr,
                    Field<float>::from_input<CurvatureFieldInput>(CurvatureKind::Gaussian));
  params.set_output("Total Curvature"_ustr,
                    Field<float>::from_input<CurvatureFieldInput>(CurvatureKind::Total));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalCurvature"_ustr, GEO_NODE_CGAL_CURVATURE);
  ntype.ui_name = "Curvature";
  ntype.ui_description =
      "Field node: evaluate mean, Gaussian, and total curvature on the context mesh.";
  ntype.enum_name_legacy = "CGAL_CURVATURE";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_curvature_cc
