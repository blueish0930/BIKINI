/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_mesh_divergence_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Vector"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "Vector field on the mesh. Evaluated on vertices; face attributes are interpolated");
  b.add_output<decl::Float>("Divergence"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "Per-vertex discrete divergence (linear FEM on each triangle: ∂Vx/∂x + ∂Vy/∂y + "
          "∂Vz/∂z)");
}

/**
 * Linear FEM divergence on a triangle.
 *
 * For barycentric bases φi with gradients gi = n×e_i / |n|² (n = (p1-p0)×(p2-p0)):
 *   div V = V0·g0 + V1·g1 + V2·g2
 * which is exact for linear V and zero for constant V.
 *
 * Previous face-constant flux over closed face boundaries is always zero — that is why the
 * node appeared "stuck at 0" for typical inputs.
 */
static void accumulate_triangle_divergence(const float3 &p0,
                                           const float3 &p1,
                                           const float3 &p2,
                                           const float3 &v0,
                                           const float3 &v1,
                                           const float3 &v2,
                                           const int i0,
                                           const int i1,
                                           const int i2,
                                           MutableSpan<float> div_accum,
                                           MutableSpan<float> area_accum)
{
  const float3 n = math::cross(p1 - p0, p2 - p0);
  const float n_len_sq = math::length_squared(n);
  if (n_len_sq < 1.0e-20f) {
    return;
  }

  /* ∇φi for barycentric coordinates on the triangle. */
  const float3 g0 = math::cross(n, p2 - p1) / n_len_sq;
  const float3 g1 = math::cross(n, p0 - p2) / n_len_sq;
  const float3 g2 = math::cross(n, p1 - p0) / n_len_sq;

  const float area = 0.5f * math::length(n);
  if (area < 1.0e-20f) {
    return;
  }

  const float div_face = math::dot(v0, g0) + math::dot(v1, g1) + math::dot(v2, g2);
  /* Lumped mass: distribute ∫div dA = div_face * area equally to the three vertices. */
  const float weight = div_face * area / 3.0f;
  const float area_third = area / 3.0f;
  div_accum[i0] += weight;
  div_accum[i1] += weight;
  div_accum[i2] += weight;
  area_accum[i0] += area_third;
  area_accum[i1] += area_third;
  area_accum[i2] += area_third;
}

class DivergenceFieldInput final : public bke::MeshFieldInput {
 private:
  Field<float3> vector_;

 public:
  explicit DivergenceFieldInput(Field<float3> vector)
      : bke::MeshFieldInput(CPPType::get<float>(), "Divergence"), vector_(std::move(vector))
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    const int verts_num = mesh.verts_num;
    const int faces_num = mesh.faces_num;
    if (verts_num == 0) {
      return mesh.attributes().adapt_domain<float>(
          VArray<float>::from_single(0.0f, 0), AttrDomain::Point, domain);
    }
    if (faces_num == 0) {
      return mesh.attributes().adapt_domain<float>(
          VArray<float>::from_single(0.0f, verts_num), AttrDomain::Point, domain);
    }

    /* Evaluate on points so linear FEM has samples at triangle corners. Face-domain fields
     * are adapted to points by the field system. */
    const bke::MeshFieldContext point_context{mesh, AttrDomain::Point};
    FieldEvaluator point_evaluator{point_context, verts_num};
    point_evaluator.add(vector_);
    point_evaluator.evaluate();
    const VArraySpan<float3> vert_vectors = point_evaluator.get_evaluated<float3>(0);

    const OffsetIndices faces = mesh.faces();
    const Span<int> corner_verts = mesh.corner_verts();
    const Span<float3> vert_positions = mesh.vert_positions();

    Array<float> div_accum(verts_num, 0.0f);
    Array<float> area_accum(verts_num, 0.0f);

    for (const int face_index : IndexRange(faces_num)) {
      const IndexRange face = faces[face_index];
      if (face.size() < 3) {
        continue;
      }

      /* Fan triangulation from the first corner (same spirit as Mesh Gradient's first tri). */
      const int i0 = corner_verts[face[0]];
      const float3 &p0 = vert_positions[i0];
      const float3 &v0 = vert_vectors[i0];

      for (const int k : IndexRange(1, face.size() - 2)) {
        const int i1 = corner_verts[face[k]];
        const int i2 = corner_verts[face[k + 1]];
        accumulate_triangle_divergence(p0,
                                       vert_positions[i1],
                                       vert_positions[i2],
                                       v0,
                                       vert_vectors[i1],
                                       vert_vectors[i2],
                                       i0,
                                       i1,
                                       i2,
                                       div_accum,
                                       area_accum);
      }
    }

    Array<float> divergence(verts_num, 0.0f);
    for (const int i : IndexRange(verts_num)) {
      if (area_accum[i] > 1.0e-20f) {
        divergence[i] = div_accum[i] / area_accum[i];
      }
    }

    return mesh.attributes().adapt_domain<float>(
        VArray<float>::from_container(std::move(divergence)), AttrDomain::Point, domain);
  }

  void foreach_recursive_field(FunctionRef<void(const GField &)> fn) const override
  {
    fn(vector_);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep &deep_hash_cache) const override
  {
    static constexpr int8_t id = 1; /* Bump: algorithm change vs face-flux version. */
    hash.add(&id);
    hash.add(deep_hash_cache.ensure(vector_));
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
  Field<float3> vector_field = params.extract_input<Field<float3>>("Vector"_ustr);
  params.set_output("Divergence"_ustr,
                    Field<float>::from_input<DivergenceFieldInput>(std::move(vector_field)));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeMeshDivergence"_ustr, GEO_NODE_MESH_DIVERGENCE);
  ntype.ui_name = "Divergence";
  ntype.ui_description =
      "Compute per-vertex divergence of a mesh vector field using linear finite elements on "
      "triangles";
  ntype.enum_name_legacy = "MESH_DIVERGENCE";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_mesh_divergence_cc
