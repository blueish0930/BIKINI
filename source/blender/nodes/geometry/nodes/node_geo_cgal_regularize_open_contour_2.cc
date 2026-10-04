/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Regularize open (or cyclic) XY polylines: parallelism / orthogonality / collinearity.
 * Distinct from Regularize Contour 2D (closed islands → filled n-gons).
 */

#include "BKE_attribute.hh"
#include "BKE_attribute_filters.hh"
#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_regularize_open_contour_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Curve"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Curve)
      .description("Polylines on XY. Open splines use the open regularizer; cyclic ones close.");
  b.add_output<decl::Geometry>("Curve"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Regularized polylines (still curves, not filled faces).");
  b.add_input<decl::Float>("Max Offset"_ustr)
      .default_value(0.5f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description(
          "How far edges may slide sideways when snapping to the longest direction / its "
          "orthogonal. 0 = skip direction snap (Min Length only).");
  b.add_input<decl::Float>("Min Length"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Collapse / Douglas–Peucker short jogs before and after regularization. 0 = keep all.");
}

static Curves *regularize_curves(const Curves &src_id,
                                 const float max_offset,
                                 const float min_length,
                                 std::string &error)
{
  const bke::CurvesGeometry &src = src_id.geometry.wrap();
  const int curve_n = src.curves_num();
  if (curve_n == 0 || src.points_num() < 2) {
    error = "Regularize Open Contour 2D needs a curve with at least 2 points";
    return nullptr;
  }
  const OffsetIndices points_by_curve = src.points_by_curve();
  const Span<float3> src_pos = src.positions();
  const VArray cyclic = src.cyclic();

  Vector<float3> all_pos;
  Vector<int> offsets;
  Vector<bool> out_cyclic;
  offsets.append(0);
  for (const int c : src.curves_range()) {
    const IndexRange pr = points_by_curve[c];
    const bool closed = cyclic[c];
    if (pr.size() < (closed ? 3 : 2)) {
      continue;
    }
    Vector<float3> out_pts;
    std::string local_error;
    if (!geometry::cgal_regularize_open_polyline_xy(
            src_pos.slice(pr), closed, max_offset, min_length, out_pts, local_error))
    {
      out_pts.clear();
      out_pts.extend(src_pos.slice(pr));
      if (error.empty() && !local_error.empty()) {
        error = local_error;
      }
    }
    if (out_pts.size() < (closed ? 3 : 2)) {
      continue;
    }
    all_pos.extend(out_pts);
    out_cyclic.append(closed);
    offsets.append(all_pos.size());
  }
  if (offsets.size() < 2) {
    if (error.empty()) {
      error = "Regularize Open Contour 2D produced no curves";
    }
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
  dst.tag_topology_changed();
  dst.tag_positions_changed();
  return dst_id;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Curve"_ustr);
  const float max_offset = params.extract_input<float>("Max Offset"_ustr);
  const float min_length = params.extract_input<float>("Min Length"_ustr);
  const Curves *curves = geometry.get_curves();
  if (!curves) {
    params.set_output("Curve"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Curves *out = regularize_curves(*curves, max_offset, min_length, error);
  if (!out) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Regularize Open Contour 2D failed") : error);
    params.set_output("Curve"_ustr, GeometrySet());
    return;
  }
  params.set_output("Curve"_ustr, GeometrySet::from_curves(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype,
                     "GeometryNodeCgalRegularizeOpenContour2"_ustr,
                     GEO_NODE_CGAL_REGULARIZE_OPEN_CONTOUR_2);
  ntype.ui_name = "Regularize Open Contour 2D";
  ntype.ui_description =
      "Snap open XY polylines to their longest direction and its orthogonal "
      "(CGAL regularize_open_contour). Unlike Regularize Contour 2D this keeps curves, "
      "does not fill n-gons, and works on open strokes.";
  ntype.enum_name_legacy = "CGAL_REGULARIZE_OPEN_CONTOUR_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_regularize_open_contour_2_cc
