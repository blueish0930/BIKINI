/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * 3D polyline simplification on Curves (Douglas–Peucker / iterative).
 * This is a Fréchet / max-deviation filter, NOT curvature-based resampling.
 */

#include "BKE_attribute.hh"
#include "BKE_attribute_filters.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_simplify_polyline_3_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Curve"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Curve)
      .description("Polylines to simplify. Each spline is simplified on its own.");
  b.add_output<decl::Geometry>("Curve"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Simplified polylines. Original sample points are kept (subset).");
  b.add_input<decl::Float>("Max Distance"_ustr)
      .default_value(0.01f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description(
          "Maximum deviation from the original polyline (Fréchet / Douglas–Peucker). "
          "Not curvature-based: it does not sample when curvature sums to a length.");
  b.add_input<decl::Bool>("Iterative"_ustr)
      .default_value(false)
      .description("Use iterative vertex removal instead of Douglas–Peucker.");
}

static Curves *simplify_curves(const Curves &src_id,
                               const float max_d,
                               const bool iterative,
                               std::string &error)
{
  const bke::CurvesGeometry &src = src_id.geometry.wrap();
  const int curve_n = src.curves_num();
  if (curve_n == 0 || src.points_num() < 2) {
    error = "Simplify Polyline 3D needs a curve with at least 2 points";
    return nullptr;
  }
  const OffsetIndices points_by_curve = src.points_by_curve();
  const Span<float3> src_pos = src.positions();
  const VArray cyclic = src.cyclic();

  Vector<float3> all_pos;
  Vector<int> offsets;
  Vector<bool> out_cyclic;
  Vector<int> src_point_of_out;
  offsets.append(0);
  for (const int c : src.curves_range()) {
    const IndexRange pr = points_by_curve[c];
    if (pr.size() < 2) {
      continue;
    }
    Vector<float3> out_pts;
    Vector<int> src_idx;
    const bool closed = cyclic[c];
    if (!geometry::cgal_simplify_polyline_xyz(
            src_pos.slice(pr), closed, max_d, iterative, out_pts, src_idx, error))
    {
      if (out_pts.size() < 2) {
        out_pts.clear();
        out_pts.extend(src_pos.slice(pr));
        src_idx.clear();
        for (const int i : pr.index_range()) {
          src_idx.append(int(pr[i]));
        }
      }
    }
    else {
      for (int &s : src_idx) {
        s = int(pr[s]);
      }
    }
    if (out_pts.size() < 2) {
      continue;
    }
    all_pos.extend(out_pts);
    src_point_of_out.extend(src_idx);
    out_cyclic.append(closed);
    offsets.append(all_pos.size());
  }
  if (offsets.size() < 2) {
    error = "Simplify Polyline 3D produced no curves";
    return nullptr;
  }
  Curves *dst_id = bke::curves_new_nomain(all_pos.size(), offsets.size() - 1);
  bke::CurvesGeometry &dst = dst_id->geometry.wrap();
  dst.offsets_for_write().copy_from(offsets);
  dst.positions_for_write().copy_from(all_pos);
  dst.cyclic_for_write().copy_from(out_cyclic);
  dst.fill_curve_types(CURVE_TYPE_POLY);

  if (dst.curves_num() == src.curves_num()) {
    const Span<StringRef> no_skip{};
    bke::copy_attributes(src.attributes(),
                         bke::AttrDomain::Curve,
                         bke::AttrDomain::Curve,
                         bke::attribute_filter_from_skip_ref(no_skip),
                         dst.attributes_for_write());
  }
  /* Point attributes: copy from the kept original samples. */
  src.attributes().foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.domain != bke::AttrDomain::Point || iter.name == "position") {
      return;
    }
    const GVArraySpan src_data = *iter.get();
    bke::GSpanAttributeWriter w = dst.attributes_for_write().lookup_or_add_for_write_only_span(
        iter.name, bke::AttrDomain::Point, iter.data_type);
    if (!w) {
      return;
    }
    bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
      const Span<T> s = src_data.typed<T>();
      MutableSpan<T> d = w.span.typed<T>();
      for (const int i : d.index_range()) {
        const int si = src_point_of_out[i];
        d[i] = (si >= 0 && si < s.size()) ? s[si] : T();
      }
    });
    w.finish();
  });
  return dst_id;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Curve"_ustr);
  const float max_d = params.extract_input<float>("Max Distance"_ustr);
  const bool iterative = params.extract_input<bool>("Iterative"_ustr);
  const Curves *curves = geometry.get_curves();
  if (!curves) {
    params.error_message_add(NodeWarningType::Info, TIP_("Simplify Polyline 3D needs a Curve"));
    params.set_output("Curve"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Curves *out = simplify_curves(*curves, max_d, iterative, error);
  if (!out) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Simplify Polyline 3D failed") : error);
    params.set_output("Curve"_ustr, GeometrySet());
    return;
  }
  params.set_output("Curve"_ustr, GeometrySet::from_curves(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalSimplifyPolyline3"_ustr, GEO_NODE_CGAL_SIMPLIFY_POLYLINE_3);
  ntype.ui_name = "Simplify Polyline 3D";
  ntype.ui_description =
      "Simplify each spline by dropping samples that stay within Max Distance of the "
      "original polyline (CGAL PMP experimental::simplify_polyline, Douglas–Peucker). "
      "This is a max-deviation filter, not curvature-sum resampling. Input/output: Curve.";
  ntype.enum_name_legacy = "CGAL_SIMPLIFY_POLYLINE_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_simplify_polyline_3_cc
