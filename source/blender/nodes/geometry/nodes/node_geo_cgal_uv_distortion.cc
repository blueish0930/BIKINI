/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * UV parameterization distortion metrics (face domain fields).
 * Area stretch + conformal (angle) stretch from 3D face vs corner UV.
 * Related to CGAL measure_distortion (L2 stretch family).
 */

#include "BKE_mesh.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_uv_distortion_cc {

enum class DistortionKind {
  Area = 0,
  Conformal = 1,
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("UV"_ustr)
      .hide_value()
      .structure_type(StructureType::Field)
      .description("Corner UV (x,y). Connect from LSCM/ARAP/… UV or Named Attribute.");
  b.add_input<decl::Bool>("Selection"_ustr)
      .default_value(true)
      .hide_value()
      .structure_type(StructureType::Field)
      .description("Faces to measure (Face domain).");
  b.add_output<decl::Float>("Area Stretch"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "Per-face area scale sigma_a = sqrt(A_uv / A_3d). 1 = area-preserving; "
          ">1 expands in UV; <1 compresses.");
  b.add_output<decl::Float>("Conformal Stretch"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "Per-face conformal stretch (max/min singular value of the 3D→UV map). "
          "1 = angle-preserving; larger = more shear/stretch anisotropy.");
}

/** Build 2x2 Gram-style stretch from one triangle (p0,p1,p2) -> (u0,u1,u2). */
static void triangle_stretch(const float3 &p0,
                             const float3 &p1,
                             const float3 &p2,
                             const float2 &uv0,
                             const float2 &uv1,
                             const float2 &uv2,
                             float &r_area_stretch,
                             float &r_conformal)
{
  const float3 e1 = p1 - p0;
  const float3 e2 = p2 - p0;
  const float3 n = math::cross(e1, e2);
  const float a3 = 0.5f * math::length(n);
  const float a2 = 0.5f * math::abs((uv1.x - uv0.x) * (uv2.y - uv0.y) -
                                    (uv2.x - uv0.x) * (uv1.y - uv0.y));
  if (a3 < 1e-20f) {
    r_area_stretch = 0.0f;
    r_conformal = 1.0f;
    return;
  }
  r_area_stretch = math::sqrt(math::max(a2, 0.0f) / a3);

  /* Orthonormal basis in triangle plane. */
  const float3 t1 = math::normalize(e1);
  float3 t2 = math::cross(math::normalize(n), t1);
  if (math::length_squared(t2) < 1e-20f) {
    r_conformal = 1.0f;
    return;
  }
  t2 = math::normalize(t2);

  /* 3D edge coords in (t1,t2). */
  const float x1 = math::dot(e1, t1);
  const float y1 = math::dot(e1, t2);
  const float x2 = math::dot(e2, t1);
  const float y2 = math::dot(e2, t2);
  const float det = x1 * y2 - x2 * y1;
  if (math::abs(det) < 1e-20f) {
    r_conformal = 1.0f;
    return;
  }
  /* Map Jacobian J: R2_local -> UV, columns are images of basis vectors.
   * Solve [x1 x2; y1 y2] * J^T = [du1 du2; dv1 dv2] wait:
   * (u,v) = J * (x,y) where (x,y) is local 3D.
   * J * [x1 x2; y1 y2] = [du1 du2; dv1 dv2]
   * J = UV_edges * inv(XY_edges) */
  const float du1 = uv1.x - uv0.x;
  const float dv1 = uv1.y - uv0.y;
  const float du2 = uv2.x - uv0.x;
  const float dv2 = uv2.y - uv0.y;
  const float inv = 1.0f / det;
  /* inv([x1 x2; y1 y2]) = (1/det)[y2 -x2; -y1 x1] */
  const float j00 = (du1 * y2 - du2 * y1) * inv;
  const float j01 = (-du1 * x2 + du2 * x1) * inv;
  const float j10 = (dv1 * y2 - dv2 * y1) * inv;
  const float j11 = (-dv1 * x2 + dv2 * x1) * inv;
  /* Singular values of J via eigenvalues of J^T J. */
  const float a = j00 * j00 + j10 * j10;
  const float b = j00 * j01 + j10 * j11;
  const float c = j01 * j01 + j11 * j11;
  const float tr = a + c;
  const float detm = a * c - b * b;
  const float disc = math::max(0.0f, tr * tr * 0.25f - detm);
  const float s1 = math::sqrt(math::max(0.0f, tr * 0.5f + math::sqrt(disc)));
  const float s2 = math::sqrt(math::max(0.0f, tr * 0.5f - math::sqrt(disc)));
  const float smax = math::max(s1, s2);
  const float smin = math::max(math::min(s1, s2), 1e-12f);
  r_conformal = smax / smin;
}

static void compute_face_distortion(const Mesh &mesh,
                                    const VArray<float3> &uv_corner,
                                    const Span<bool> face_ok,
                                    MutableSpan<float> area_out,
                                    MutableSpan<float> conf_out)
{
  const Span<float3> positions = mesh.vert_positions();
  const Span<int> corner_verts = mesh.corner_verts();
  const OffsetIndices faces = mesh.faces();

  area_out.fill(0.0f);
  conf_out.fill(0.0f);

  for (const int f : faces.index_range()) {
    if (!face_ok[f]) {
      continue;
    }
    const IndexRange corners = faces[f];
    if (corners.size() < 3) {
      continue;
    }
    /* Fan triangulation; average metrics. */
    float area_sum = 0.0f;
    float conf_sum = 0.0f;
    int ntri = 0;
    const int c0 = corners[0];
    const float3 p0 = positions[corner_verts[c0]];
    const float2 uv0(uv_corner[c0].x, uv_corner[c0].y);
    for (int k = 1; k + 1 < corners.size(); k++) {
      const int c1 = corners[k];
      const int c2 = corners[k + 1];
      float as, cs;
      triangle_stretch(p0,
                       positions[corner_verts[c1]],
                       positions[corner_verts[c2]],
                       uv0,
                       float2(uv_corner[c1].x, uv_corner[c1].y),
                       float2(uv_corner[c2].x, uv_corner[c2].y),
                       as,
                       cs);
      area_sum += as;
      conf_sum += cs;
      ntri++;
    }
    if (ntri > 0) {
      area_out[f] = area_sum / float(ntri);
      conf_out[f] = conf_sum / float(ntri);
    }
  }
}

static void evaluate_masks(const Mesh &mesh,
                           const Field<bool> &selection_field,
                           Array<bool> &face_bool)
{
  const bke::MeshFieldContext face_context{mesh, AttrDomain::Face};
  fn::FieldEvaluator face_evaluator{face_context, mesh.faces_num};
  face_evaluator.add(selection_field);
  face_evaluator.evaluate();
  face_bool.reinitialize(mesh.faces_num);
  face_evaluator.get_evaluated<bool>(0).materialize(face_bool.as_mutable_span());
}

static VArray<float> construct_distortion(const Mesh &mesh,
                                          const Field<float3> &uv_field,
                                          const Field<bool> &selection_field,
                                          const DistortionKind kind,
                                          const AttrDomain domain)
{
  Array<bool> face_bool;
  evaluate_masks(mesh, selection_field, face_bool);

  const bke::MeshFieldContext corner_context{mesh, AttrDomain::Corner};
  fn::FieldEvaluator uv_eval{corner_context, mesh.corners_num};
  uv_eval.add(uv_field);
  uv_eval.evaluate();
  const VArray<float3> uv_corner = uv_eval.get_evaluated<float3>(0);

  Array<float> area(mesh.faces_num, 0.0f);
  Array<float> conf(mesh.faces_num, 0.0f);
  compute_face_distortion(mesh, uv_corner, face_bool, area, conf);

  Array<float> out = (kind == DistortionKind::Area) ? std::move(area) : std::move(conf);
  return mesh.attributes().adapt_domain<float>(
      VArray<float>::from_container(std::move(out)), AttrDomain::Face, domain);
}

class UVDistortionFieldInput final : public bke::MeshFieldInput {
  Field<float3> uv_;
  Field<bool> selection_;
  DistortionKind kind_;

 public:
  UVDistortionFieldInput(Field<float3> uv, Field<bool> selection, DistortionKind kind)
      : bke::MeshFieldInput(CPPType::get<float>(),
                            kind == DistortionKind::Area ? "CGAL UV Area Stretch Field" :
                                                           "CGAL UV Conformal Stretch Field"),
        uv_(std::move(uv)),
        selection_(std::move(selection)),
        kind_(kind)
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    return construct_distortion(mesh, uv_, selection_, kind_, domain);
  }

  void foreach_recursive_field(FunctionRef<void(const GField &)> fn) const override
  {
    fn(uv_);
    fn(selection_);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep &deep_hash_cache) const override
  {
    static constexpr int8_t area_id = 8;
    static constexpr int8_t conf_id = 9;
    hash.add(kind_ == DistortionKind::Area ? &area_id : &conf_id);
    hash.add(deep_hash_cache.ensure(uv_));
    hash.add(deep_hash_cache.ensure(selection_));
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
  const Field<float3> uv = params.extract_input<Field<float3>>("UV"_ustr);
  const Field<bool> selection = params.extract_input<Field<bool>>("Selection"_ustr);
  params.set_output(
      "Area Stretch"_ustr,
      Field<float>::from_input<UVDistortionFieldInput>(uv, selection, DistortionKind::Area));
  params.set_output(
      "Conformal Stretch"_ustr,
      Field<float>::from_input<UVDistortionFieldInput>(uv, selection, DistortionKind::Conformal));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalUVDistortion"_ustr, GEO_NODE_CGAL_UV_DISTORTION);
  ntype.ui_name = "UV Distortion";
  ntype.ui_description =
      "Measure parameterization distortion per face from Corner UV vs 3D geometry. "
      "Area Stretch ≈ 1 means area-preserving; Conformal Stretch ≈ 1 means angle-preserving. "
      "Related to CGAL measure_distortion (L2 stretch family).";
  ntype.enum_name_legacy = "CGAL_UV_DISTORTION";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_uv_distortion_cc
