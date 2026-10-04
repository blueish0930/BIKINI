/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_mesh_gradient_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("Value"_ustr)
      .structure_type(StructureType::Field)
      .description("Scalar field defined on mesh vertices");
  b.add_output<decl::Vector>("Gradient"_ustr)
      .structure_type(StructureType::Field)
      .description("Per-face gradient of the scalar field");
}

class GradientFieldInput final : public bke::MeshFieldInput {
 private:
  Field<float> value_;

 public:
  explicit GradientFieldInput(Field<float> value)
      : bke::MeshFieldInput(CPPType::get<float3>(), "Gradient"), value_(std::move(value))
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    const int faces_num = mesh.faces_num;
    if (faces_num == 0) {
      return mesh.attributes().adapt_domain<float3>(
          VArray<float3>::from_single(float3(0.0f), 0), AttrDomain::Face, domain);
    }

    const bke::MeshFieldContext point_context{mesh, AttrDomain::Point};
    FieldEvaluator point_evaluator{point_context, mesh.verts_num};
    point_evaluator.add(value_);
    point_evaluator.evaluate();
    const VArray<float> vert_values = point_evaluator.get_evaluated<float>(0);

    const OffsetIndices faces = mesh.faces();
    const Span<int> corner_verts = mesh.corner_verts();
    const Span<float3> vert_positions = mesh.vert_positions();

    Array<float3> gradient(faces_num, float3(0.0f));

    for (const int face_index : IndexRange(faces_num)) {
      const IndexRange face = faces[face_index];
      if (face.size() < 3) {
        continue;
      }

      const int v0 = corner_verts[face[0]];
      const int v1 = corner_verts[face[1]];
      const int v2 = corner_verts[face[2]];

      const float3 p0 = vert_positions[v0];
      const float3 p1 = vert_positions[v1];
      const float3 p2 = vert_positions[v2];

      const float s0 = vert_values[v0];
      const float s1 = vert_values[v1];
      const float s2 = vert_values[v2];

      const float3 e01 = p1 - p0;
      const float3 e02 = p2 - p0;

      const float e01_dot_e01 = math::dot(e01, e01);
      const float e01_dot_e02 = math::dot(e01, e02);
      const float e02_dot_e02 = math::dot(e02, e02);

      const float det = e01_dot_e01 * e02_dot_e02 - e01_dot_e02 * e01_dot_e02;
      if (det < 1e-12f) {
        gradient[face_index] = float3(0.0f);
        continue;
      }

      const float ds01 = s1 - s0;
      const float ds02 = s2 - s0;

      const float alpha = (e02_dot_e02 * ds01 - e01_dot_e02 * ds02) / det;
      const float beta = (e01_dot_e01 * ds02 - e01_dot_e02 * ds01) / det;

      gradient[face_index] = alpha * e01 + beta * e02;
    }

    return mesh.attributes().adapt_domain<float3>(
        VArray<float3>::from_container(std::move(gradient)), AttrDomain::Face, domain);
  }

  void foreach_recursive_field(FunctionRef<void(const GField &)> fn) const override
  {
    fn(value_);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep &deep_hash_cache) const override
  {
    static constexpr int8_t id = 0;
    hash.add(&id);
    hash.add(deep_hash_cache.ensure(value_));
  }

  std::optional<AttrDomain> preferred_domain(const Mesh & /*mesh*/) const override
  {
    return AttrDomain::Face;
  }

  bke::NativeFieldDomain native_domain_info(const Mesh & /*mesh*/) const override
  {
    return bke::NativeFieldDomain::Domain{AttrDomain::Face};
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  Field<float> value_field = params.extract_input<Field<float>>("Value"_ustr);
  params.set_output("Gradient"_ustr,
                    Field<float3>::from_input<GradientFieldInput>(std::move(value_field)));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeMeshGradient"_ustr, GEO_NODE_MESH_GRADIENT);
  ntype.ui_name = "Gradient";
  ntype.ui_description =
      "Compute the per-face gradient of a scalar field defined on mesh vertices";
  ntype.enum_name_legacy = "MESH_GRADIENT";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_mesh_gradient_cc
