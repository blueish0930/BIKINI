/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * UV parameterization distortion metrics (face domain fields).
 * Texel, conformal and area stretch from 3D face vs corner UV.
 *
 * The texel stretch follows the "Texture Stretch Metric" of "Texture Mapping Progressive Meshes"
 * (P. Sander, J. Snyder, S. Gortler, H. Hoppe). The position derivatives over u and v of each
 * triangle give the two singular values of the UV -> surface map; the texel stretch is their L2
 * norm, the conformal stretch their ratio. Texel and area stretch are normalized with the global
 * UV to surface area ratio, so a uniform scale of the UV layout does not change them.
 */

#include "BKE_mesh.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_uv_distortion_cc {

enum class DistortionKind {
  Texel = 0,
  Conformal = 1,
  Area = 2,
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
      .description(
          "Faces to measure (Face domain). The global area normalization only uses these faces.");
  b.add_output<decl::Float>("Texel Stretch"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "How much the texture is stretched on each face (L2 stretch of Sander et al.), "
          "compared to the average of the whole mesh.\n"
          "1 = no distortion. The higher the value, the more the face is stretched or sheared "
          "(fewer texels per surface unit than on average). Below 1 the face is compressed "
          "(more texels per surface unit than on average).\n"
          "0 for faces that are not selected.");
  b.add_output<decl::Float>("Conformal Stretch"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "How much each face is sheared, as the ratio between the largest and the smallest "
          "stretch of the UV mapping. Independent of the UV size.\n"
          "1 = no distortion (angles are preserved). The higher the value, the more the face is "
          "stretched in one direction.\n"
          "0 for faces that are not selected or whose UV triangle has collapsed to a line.");
  b.add_output<decl::Float>("Area Stretch"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "How much UV area each face covers compared to its share of the surface area on "
          "average over the mesh.\n"
          "1 = no distortion (the UV area is proportional to the surface area). The further the "
          "value is from 1, the more the area is distorted: above 1 the face is enlarged in UV "
          "(more texels per surface unit), below 1 it is shrunk in UV (fewer texels).\n"
          "0 for faces that are not selected.");
}

struct TriangleStretch {
  /** L2 stretch before the global normalization. */
  float texel = 0.0f;
  float conformal = 0.0f;
  /** sqrt(A_uv / A_3d) before the global normalization. */
  float area = 0.0f;
  /** Twice the UV (absolute) and 3D triangle area, summed for the global normalization. */
  float uv_area_x2 = 0.0f;
  float mesh_area_x2 = 0.0f;
};

static TriangleStretch triangle_stretch(const float3 &p0,
                                        const float3 &p1,
                                        const float3 &p2,
                                        const float2 &uv0,
                                        const float2 &uv1,
                                        const float2 &uv2)
{
  TriangleStretch r;
  r.mesh_area_x2 = math::length(math::cross(p0 - p1, p2 - p1));
  const float signed_uv_area = (uv1.x - uv0.x) * (uv2.y - uv0.y) - (uv2.x - uv0.x) * (uv1.y - uv0.y);
  r.uv_area_x2 = math::abs(signed_uv_area);
  if (r.mesh_area_x2 < 1e-20f) {
    return r;
  }

  r.area = math::sqrt(r.uv_area_x2 / r.mesh_area_x2);

  /* Gradients of u and v over the triangle surface. */
  const float3 p[3] = {p0, p1, p2};
  const float2 uv[3] = {uv0, uv1, uv2};
  float3 gu(0.0f);
  float3 gv(0.0f);
  for (const int k : IndexRange(3)) {
    const int k_next = (k + 1) % 3;
    const int k_prev = (k + 2) % 3;
    gu += (uv[k_next].y - uv[k_prev].y) * p[k];
    gv += (uv[k_prev].x - uv[k_next].x) * p[k];
  }
  const float denominator = signed_uv_area != 0.0f ? signed_uv_area : 1e-10f;
  gu /= denominator;
  gv /= denominator;

  const float a = math::dot(gu, gu);
  const float b = math::dot(gu, gv);
  const float c = math::dot(gv, gv);
  const float root = math::sqrt((a - c) * (a - c) + 4.0f * b * b);
  const float max_sv = math::sqrt(math::max(0.0f, 0.5f * (a + c + root)));
  const float min_sv = math::sqrt(math::max(0.0f, 0.5f * (a + c - root)));
  r.texel = math::sqrt(0.5f * (a + c));
  /* A collapsed UV triangle has no valid ratio. Report 0 like faces that are not measured. */
  r.conformal = min_sv > max_sv * 1e-6f ? max_sv / min_sv : 0.0f;
  return r;
}

static void compute_face_distortion(const Mesh &mesh,
                                    const VArray<float3> &uv_corner,
                                    const Span<bool> face_ok,
                                    MutableSpan<float> texel_out,
                                    MutableSpan<float> conformal_out,
                                    MutableSpan<float> area_out)
{
  const Span<float3> positions = mesh.vert_positions();
  const Span<int> corner_verts = mesh.corner_verts();
  const OffsetIndices faces = mesh.faces();

  texel_out.fill(0.0f);
  conformal_out.fill(0.0f);
  area_out.fill(0.0f);

  double uv_area_x2_sum = 0.0;
  double mesh_area_x2_sum = 0.0;

  for (const int f : faces.index_range()) {
    if (!face_ok[f]) {
      continue;
    }
    const IndexRange corners = faces[f];
    if (corners.size() < 3) {
      continue;
    }
    /* Fan triangulation; average the metrics of the triangles. */
    float texel_sum = 0.0f;
    float conformal_sum = 0.0f;
    float area_sum = 0.0f;
    int tris_num = 0;
    const int c0 = corners[0];
    const float3 p0 = positions[corner_verts[c0]];
    const float2 uv0(uv_corner[c0].x, uv_corner[c0].y);
    for (int k = 1; k + 1 < corners.size(); k++) {
      const int c1 = corners[k];
      const int c2 = corners[k + 1];
      const TriangleStretch tri = triangle_stretch(p0,
                                                   positions[corner_verts[c1]],
                                                   positions[corner_verts[c2]],
                                                   uv0,
                                                   float2(uv_corner[c1].x, uv_corner[c1].y),
                                                   float2(uv_corner[c2].x, uv_corner[c2].y));
      texel_sum += tri.texel;
      conformal_sum += tri.conformal;
      area_sum += tri.area;
      uv_area_x2_sum += double(tri.uv_area_x2);
      mesh_area_x2_sum += double(tri.mesh_area_x2);
      tris_num++;
    }
    if (tris_num > 0) {
      texel_out[f] = texel_sum / float(tris_num);
      conformal_out[f] = conformal_sum / float(tris_num);
      area_out[f] = area_sum / float(tris_num);
    }
  }

  /* Global normalization: ratio of the total UV area to the total surface area. */
  const float scale = (uv_area_x2_sum > 0.0 && mesh_area_x2_sum > 0.0) ?
                          float(math::sqrt(uv_area_x2_sum / mesh_area_x2_sum)) :
                          1.0f;
  for (const int f : faces.index_range()) {
    texel_out[f] *= scale;
    area_out[f] /= scale;
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

  Array<float> texel(mesh.faces_num, 0.0f);
  Array<float> conformal(mesh.faces_num, 0.0f);
  Array<float> area(mesh.faces_num, 0.0f);
  compute_face_distortion(mesh, uv_corner, face_bool, texel, conformal, area);

  Array<float> out;
  switch (kind) {
    case DistortionKind::Texel:
      out = std::move(texel);
      break;
    case DistortionKind::Conformal:
      out = std::move(conformal);
      break;
    case DistortionKind::Area:
      out = std::move(area);
      break;
  }
  return mesh.attributes().adapt_domain<float>(
      VArray<float>::from_container(std::move(out)), AttrDomain::Face, domain);
}

static const char *distortion_field_name(const DistortionKind kind)
{
  switch (kind) {
    case DistortionKind::Texel:
      return "UV Texel Stretch Field";
    case DistortionKind::Conformal:
      return "CGAL UV Conformal Stretch Field";
    case DistortionKind::Area:
      return "CGAL UV Area Stretch Field";
  }
  return "";
}

class UVDistortionFieldInput final : public bke::MeshFieldInput {
  Field<float3> uv_;
  Field<bool> selection_;
  DistortionKind kind_;

 public:
  UVDistortionFieldInput(Field<float3> uv, Field<bool> selection, const DistortionKind kind)
      : bke::MeshFieldInput(CPPType::get<float>(), distortion_field_name(kind)),
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
    static constexpr int8_t texel_id = 10;
    const int8_t *kind_id = kind_ == DistortionKind::Area ?
                                &area_id :
                                (kind_ == DistortionKind::Conformal ? &conf_id : &texel_id);
    hash.add(kind_id);
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
      "Texel Stretch"_ustr,
      Field<float>::from_input<UVDistortionFieldInput>(uv, selection, DistortionKind::Texel));
  params.set_output(
      "Conformal Stretch"_ustr,
      Field<float>::from_input<UVDistortionFieldInput>(uv, selection, DistortionKind::Conformal));
  params.set_output(
      "Area Stretch"_ustr,
      Field<float>::from_input<UVDistortionFieldInput>(uv, selection, DistortionKind::Area));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalUVDistortion"_ustr, GEO_NODE_CGAL_UV_DISTORTION);
  ntype.ui_name = "UV Distortion";
  ntype.ui_description =
      "Measure UV distortion per face from Corner UV vs 3D geometry. All three outputs are 1 "
      "on a layout without distortion. Texel Stretch and Conformal Stretch get higher with more "
      "distortion, Area Stretch moves away from 1 in both directions. Texel and Area Stretch "
      "are relative to the total UV to surface area ratio of the mesh.";
  ntype.enum_name_legacy = "CGAL_UV_DISTORTION";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_uv_distortion_cc
