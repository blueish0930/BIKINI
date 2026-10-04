/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Tangent Field — smooth N-RoSy field via Directional (power_field),
 * with Houdini-like alignment: curvature, boundary, and optional guide vectors.
 *
 * N is an integer socket on the node body (odd and even allowed).
 * Writes N point vector attributes named `{Prefix}{0..N-1}`.
 */

#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_string_ref.hh"

#include "BKE_attribute.hh"

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_tangent_field.hh"
#include "GEO_mesh_triangulate.hh"

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "node_geometry_util.hh"

#include <atomic>
#include <fmt/format.h>

namespace blender::nodes::node_geo_tangent_field_cc {

static constexpr int k_n_min = 1;
static constexpr int k_n_max = 12;
static constexpr int k_n_default = 4;

static int clamp_n(const int n)
{
  return math::clamp(n, k_n_min, k_n_max);
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Mesh"_ustr)
      .supported_type(GeometryComponent::Type::Mesh)
      .is_default_link_socket()
      .description("Surface for the tangent field; non-triangles are triangulated first");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Mesh with N point vector attributes Prefix0 … Prefix{N-1}");

  b.add_input<decl::Int>("N"_ustr)
      .default_value(k_n_default)
      .min(k_n_min)
      .max(k_n_max)
      .description(
          "N-RoSy order of the Directional power field (odd and even allowed, e.g. 2, 4). "
          "Curvature alignment is used for N ≥ 2");
  b.add_input<decl::String>("Prefix"_ustr)
      .default_value("dir")
      .description(
          "Attribute name prefix. Direction k is stored as Prefix + k (e.g. dir0, dir1). "
          "Put a separator in the prefix if needed (e.g. dir_)");
  b.add_input<decl::Bool>("Normalize"_ustr)
      .default_value(true)
      .description("Normalize each of the N raw tangent directions to unit length");

  b.add_input<decl::Float>("Alignment"_ustr)
      .default_value(0.45f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .description(
          "Balance smoothness vs alignment (0 = pure smooth field, 1 = strongest soft "
          "alignment). Soft scale is gentle: mid values stay smooth while following "
          "curvature/boundary. Same role as Houdini Alignment Weight");
  b.add_input<decl::Float>("Curvature"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .description(
          "Align to principal curvature (N ≥ 2). Auto-masked by anisotropy so flat regions "
          "stay free (reduces messy fields). Relative channel weight; 0 disables");
  b.add_input<decl::Float>("Boundary"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .description(
          "Align to open-boundary edge tangents (sparse, usually stable). "
          "Relative channel weight; 0 disables");
  b.add_input<decl::Float>("Guide Weight"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .description(
          "Align to the Guide vector field on points (zero-length samples are ignored). "
          "0 disables");
  b.add_input<decl::Vector>("Guide"_ustr)
      .default_value(float3(0.0f))
      .evaluated_geometry_field()
      .description(
          "Optional guide directions on points (projected into the tangent plane). "
          "Only used where length > 0 and Guide Weight > 0");

  b.add_output<decl::Int>("Directions"_ustr)
      .description("Number of direction attributes written (equals N)");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh"_ustr);
  const int n = clamp_n(params.extract_input<int>("N"_ustr));
  const std::string prefix = params.extract_input<std::string>("Prefix"_ustr);
  const bool normalize = params.extract_input<bool>("Normalize"_ustr);
  const float alignment = params.extract_input<float>("Alignment"_ustr);
  const float curvature = params.extract_input<float>("Curvature"_ustr);
  const float boundary = params.extract_input<float>("Boundary"_ustr);
  const float guide_weight = params.extract_input<float>("Guide Weight"_ustr);
  Field<float3> guide_field = params.extract_input<Field<float3>>("Guide"_ustr);
  const AttributeFilter &attribute_filter = params.get_attribute_filter("Mesh"_ustr);

  if (prefix.empty()) {
    params.error_message_add(NodeWarningType::Error, TIP_("Prefix must not be empty"));
    params.set_default_remaining_outputs();
    return;
  }

  geometry::TangentFieldOptions options;
  options.n = n;
  options.attribute_prefix = prefix;
  options.normalize = normalize;
  options.alignment_weight = math::clamp(alignment, 0.0f, 1.0f);
  options.curvature_weight = math::max(curvature, 0.0f);
  options.boundary_weight = math::max(boundary, 0.0f);
  options.guide_weight = math::max(guide_weight, 0.0f);

  std::atomic<int> written = 0;
  std::atomic<bool> found_mesh = false;
  std::atomic<bool> failed = false;
  std::atomic<bool> triangulated = false;

  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry) {
    const Mesh *src = geometry.get_mesh();
    if (!src) {
      return;
    }
    found_mesh.store(true, std::memory_order_relaxed);

    Mesh *owned_tri = nullptr;
    if (src->corners_num != src->faces_num * 3) {
      const IndexMask selection(src->faces_num);
      if (std::optional<Mesh *> result = geometry::mesh_triangulate(
              *src,
              selection,
              geometry::TriangulateNGonMode::Beauty,
              geometry::TriangulateQuadMode::ShortEdge,
              attribute_filter))
      {
        owned_tri = *result;
        triangulated.store(true, std::memory_order_relaxed);
      }
      else {
        failed.store(true, std::memory_order_relaxed);
        return;
      }
    }

    Mesh *work = owned_tri ? owned_tri : BKE_mesh_copy_for_eval(*src);

    Array<float3> guide_vectors;
    if (options.guide_weight > 1.0e-8f) {
      guide_vectors.reinitialize(work->verts_num);
      guide_vectors.fill(float3(0.0f));
      const bke::MeshFieldContext context(*work, AttrDomain::Point);
      fn::FieldEvaluator evaluator(context, work->verts_num);
      evaluator.add(guide_field);
      evaluator.evaluate();
      evaluator.get_evaluated<float3>(0).materialize(guide_vectors.as_mutable_span());
      options.guide_vectors = guide_vectors.as_span();
    }
    else {
      options.guide_vectors = {};
    }

    const int wrote = geometry::mesh_tangent_field_store_attributes(*work, options);
    if (wrote <= 0) {
      failed.store(true, std::memory_order_relaxed);
    }
    else {
      written.store(wrote, std::memory_order_relaxed);
    }
    geometry.replace_mesh(work);
  });

  if (!found_mesh.load(std::memory_order_relaxed)) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Input geometry does not contain a mesh"));
  }
  if (triangulated.load(std::memory_order_relaxed)) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Non-triangle faces were triangulated"));
  }
  if (failed.load(std::memory_order_relaxed)) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("Tangent field failed (requires Directional build and a valid triangle mesh)"));
  }
  else if (written.load(std::memory_order_relaxed) > 0) {
    const int wn = written.load(std::memory_order_relaxed);
    params.error_message_add(
        NodeWarningType::Info,
        fmt::format(fmt::runtime(TIP_("Wrote attributes {}0 … {}{}")),
                    prefix,
                    prefix,
                    wn - 1));
  }

  params.set_output("Mesh"_ustr, std::move(geometry_set));
  params.set_output("Directions"_ustr,
                    written.load(std::memory_order_relaxed) > 0 ?
                        written.load(std::memory_order_relaxed) :
                        n);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeTangentField"_ustr, GEO_NODE_TANGENT_FIELD);
  ntype.ui_name = "Tangent Field";
  ntype.ui_description =
      "Compute a smooth N-RoSy tangent field (Directional power field) with optional "
      "alignment to principal curvature, open boundaries, and guide vectors "
      "(Houdini-style). Stores each of the N directions as Prefix + index. "
      "Each mesh island is solved separately";
  ntype.enum_name_legacy = "TANGENT_FIELD";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_tangent_field_cc
