/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Geometry Node: Heat Geodesic (Keenan Crane et al.)
 *
 * Approximate geodesic distances on a mesh via short-time heat diffusion
 * (t = h², cotangent Laplacian), unit gradient, and a Poisson solve.
 * Backed by Mesh Laplacian + sparse Linear Solver (Eigen LLT).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "FN_field.hh"

#include "GEO_mesh_triangulate.hh"

#include "node_geometry_util.hh"

#include "mesh_heat_geodesic.hh"

namespace blender::nodes::node_geo_heat_geodesic_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Bool>("Source"_ustr)
      .default_value(false)
      .hide_value()
      .structure_type(StructureType::Field)
      .description("Source vertices (heat sources); geodesic distance is zero here");
  b.add_output<decl::Float>("Distance"_ustr)
      .structure_type(StructureType::Field)
      .propagate_references()
      .description("Approximate geodesic distance from the nearest Source vertex");
}

class HeatGeodesicDistanceFieldInput final : public bke::MeshFieldInput {
 private:
  Field<bool> source_;

 public:
  explicit HeatGeodesicDistanceFieldInput(Field<bool> source)
      : bke::MeshFieldInput(CPPType::get<float>(), "Heat Geodesic Distance"),
        source_(std::move(source))
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    const int n = mesh.verts_num;
    if (n == 0) {
      return mesh.attributes().adapt_domain<float>(
          VArray<float>::from_single(0.0f, 0), AttrDomain::Point, domain);
    }

    const bke::MeshFieldContext point_context{mesh, AttrDomain::Point};
    fn::FieldEvaluator point_evaluator{point_context, n};
    point_evaluator.add(source_);
    point_evaluator.evaluate();
    const VArray<bool> source_varray = point_evaluator.get_evaluated<bool>(0);

    Array<bool> is_source(n, false);
    source_varray.materialize(is_source.as_mutable_span());

    const Mesh *triangle_mesh = &mesh;
    Mesh *owned_triangle_mesh = nullptr;
    if (mesh.corners_num != mesh.faces_num * 3) {
      const IndexMask selection(mesh.faces_num);
      if (std::optional<Mesh *> result = geometry::mesh_triangulate(
              mesh,
              selection,
              geometry::TriangulateNGonMode::Beauty,
              geometry::TriangulateQuadMode::ShortEdge,
              bke::AttributeFilter{}))
      {
        owned_triangle_mesh = *result;
        triangle_mesh = owned_triangle_mesh;
      }
      else {
        return mesh.attributes().adapt_domain<float>(
            VArray<float>::from_single(0.0f, n), AttrDomain::Point, domain);
      }
    }

    const Span<float3> positions = triangle_mesh->vert_positions();
    const OffsetIndices faces = triangle_mesh->faces();
    const Span<int> corner_verts = triangle_mesh->corner_verts();
    Vector<int> tri_corners;
    tri_corners.reserve(faces.size() * 3);
    int faces_num = 0;
    for (const int face_i : faces.index_range()) {
      const IndexRange face = faces[face_i];
      if (face.size() != 3) {
        continue;
      }
      tri_corners.append(corner_verts[face[0]]);
      tri_corners.append(corner_verts[face[1]]);
      tri_corners.append(corner_verts[face[2]]);
      faces_num++;
    }

    const mesh_heat_geodesic::HeatGeodesicResult hg = mesh_heat_geodesic::compute(
        positions, tri_corners, faces_num, is_source.as_span());

    if (owned_triangle_mesh) {
      BKE_id_free(nullptr, owned_triangle_mesh);
    }

    Array<float> distance(n, 0.0f);
    if (hg.success && hg.distance.size() >= n) {
      for (const int i : IndexRange(n)) {
        distance[i] = hg.distance[i];
      }
    }

    return mesh.attributes().adapt_domain<float>(
        VArray<float>::from_container(std::move(distance)), AttrDomain::Point, domain);
  }

  void foreach_recursive_field(FunctionRef<void(const GField &)> fn) const override
  {
    fn(source_);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep &deep_hash_cache) const override
  {
    static constexpr int8_t id = 3; /* cot-only, fixed t=h² */
    hash.add(&id);
    hash.add(deep_hash_cache.ensure(source_));
  }

  std::optional<AttrDomain> preferred_domain(const Mesh & /*mesh*/) const override
  {
    return AttrDomain::Point;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  Field<bool> source = params.extract_input<Field<bool>>("Source"_ustr);
  params.set_output("Distance"_ustr,
                    Field<float>::from_input<HeatGeodesicDistanceFieldInput>(source));
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeHeatGeodesic"_ustr, GEO_NODE_HEAT_GEODESIC);
  ntype.ui_name = "Heat Geodesic";
  ntype.ui_description =
      "Approximate geodesic distance (Crane et al. heat method): "
      "cot Laplacian, t = mean_edge_length², then unit gradient and Poisson Lφ=div";
  ntype.enum_name_legacy = "HEAT_GEODESIC";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.default_width = bke::NodeWidth::_140;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_heat_geodesic_cc
