/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_math_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_deform.hh"
#include "BKE_mesh.hh"

#include "DNA_meshdata_types.h"

#include "GEO_mesh_clip_plane.hh"

#include "NOD_socket_usage_inference.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_geometry_clip_cc {

enum class Mode {
  Above = 0,
  Below = 1,
  Both = 2,
};

struct CurvePointSource {
  int index_a;
  int index_b;
  float factor;
};

struct CurveSegment {
  int src_curve;
  bool cyclic;
  Vector<CurvePointSource> points;
};

struct ClippedCurves {
  bke::CurvesGeometry above;
  bke::CurvesGeometry below;
  Array<bool> above_clip_boundary;
  Array<bool> below_clip_boundary;
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  static const EnumPropertyItem mode_items[] = {
      {int(Mode::Above), "ABOVE", 0, N_("Above"), N_("Keep the side pointed to by the plane normal")},
      {int(Mode::Below), "BELOW", 0, N_("Below"), N_("Keep the side opposite the plane normal")},
      {int(Mode::Both), "BOTH", 0, N_("Both"), N_("Output both clipped sides separately")},
      {0, nullptr, 0, nullptr, nullptr},
  };
  b.add_input<decl::Menu>("Mode"_ustr)
      .static_items(mode_items)
      .default_value(MenuValue(Mode::Above))
      .optional_label()
      .description("Which side of the plane to keep, or both as separate outputs");
  b.add_input<decl::Geometry>("Geometry"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::Mesh, GeometryComponent::Type::Curve})
      .is_default_link_socket()
      .description("Mesh and curve geometry to clip");
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .propagate_all_geometry()
      .description("Clipped geometry for Above or Below mode")
      .usage_inference([](const socket_usage_inference::SocketUsageParams &params) {
        return params.menu_input_may_be("Mode"_ustr, int(Mode::Above)) ||
               params.menu_input_may_be("Mode"_ustr, int(Mode::Below));
      });
  b.add_output<decl::Geometry>("Above"_ustr)
      .propagate_all_geometry()
      .description("Geometry on the positive side of the plane (Both mode)")
      .usage_inference([](const socket_usage_inference::SocketUsageParams &params) {
        return params.menu_input_may_be("Mode"_ustr, int(Mode::Both));
      });
  b.add_output<decl::Geometry>("Below"_ustr)
      .propagate_all_geometry()
      .description("Geometry on the negative side of the plane (Both mode)")
      .usage_inference([](const socket_usage_inference::SocketUsageParams &params) {
        return params.menu_input_may_be("Mode"_ustr, int(Mode::Both));
      });
  b.add_output<decl::Bool>("Clip Boundary"_ustr)
      .anonymous_attribute_output()
      .description("Point selection of vertices/curve points that lie on the clip plane");
  b.add_input<decl::Vector>("Plane Position"_ustr)
      .description("A point on the clipping plane");
  b.add_input<decl::Vector>("Plane Normal"_ustr)
      .default_value({0.0f, 0.0f, 1.0f})
      .description("Direction of the clipping plane; the positive side is above");
}


static GeometrySet geometry_set_from_components(Mesh *mesh,
                                                Curves *curves,
                                                const std::string &name)
{
  GeometrySet result;
  result.replace_mesh(mesh);
  result.replace_curves(curves);
  result.set_name(name);
  return result;
}

static void write_clip_boundary_selection(Mesh *mesh,
                                          Curves *curves,
                                          const std::optional<std::string> &attr_id,
                                          const Span<bool> mesh_sel,
                                          const Span<bool> curve_sel)
{
  if (!attr_id) {
    return;
  }
  if (mesh != nullptr && mesh_sel.size() == mesh->verts_num) {
    bke::SpanAttributeWriter<bool> writer =
        mesh->attributes_for_write().lookup_or_add_for_write_only_span<bool>(
            *attr_id, bke::AttrDomain::Point);
    writer.span.copy_from(mesh_sel);
    writer.finish();
  }
  if (curves != nullptr) {
    bke::CurvesGeometry &cg = curves->geometry.wrap();
    if (curve_sel.size() == cg.points_num()) {
      bke::SpanAttributeWriter<bool> writer =
          cg.attributes_for_write().lookup_or_add_for_write_only_span<bool>(
              *attr_id, bke::AttrDomain::Point);
      writer.span.copy_from(curve_sel);
      writer.finish();
    }
  }
}


static CurvePointSource source_point(const int index)
{
  return {index, index, 0.0f};
}

static CurvePointSource intersection_point(const int index_a,
                                           const int index_b,
                                           const float distance_a,
                                           const float distance_b)
{
  const float factor = std::clamp(distance_a / (distance_a - distance_b), 0.0f, 1.0f);
  if (factor <= 1.0e-6f) {
    return source_point(index_a);
  }
  if (factor >= 1.0f - 1.0e-6f) {
    return source_point(index_b);
  }
  return {index_a, index_b, factor};
}

static bool source_points_equal(const CurvePointSource &a, const CurvePointSource &b)
{
  if (a.index_a == a.index_b && b.index_a == b.index_b) {
    return a.index_a == b.index_a;
  }
  return a.index_a == b.index_a && a.index_b == b.index_b &&
         std::abs(a.factor - b.factor) <= 1.0e-6f;
}

static void append_source_point(Vector<CurvePointSource> &points, const CurvePointSource &point)
{
  if (points.is_empty() || !source_points_equal(points.last(), point)) {
    points.append(point);
  }
}

static void append_curve_segment(Vector<CurveSegment> &segments,
                                 const int src_curve,
                                 Vector<CurvePointSource> &points,
                                 const bool cyclic = false)
{
  if (!points.is_empty()) {
    segments.append({src_curve, cyclic, std::move(points)});
    points.clear();
  }
}

static void build_curve_segments(const bke::CurvesGeometry &curves,
                                 const Span<float> distances,
                                 const bool keep_above,
                                 Vector<CurveSegment> &r_segments)
{
  const OffsetIndices points_by_curve = curves.points_by_curve();
  const VArray<bool> cyclic = curves.cyclic();

  for (const int curve_i : curves.curves_range()) {
    const IndexRange points = points_by_curve[curve_i];
    if (points.is_empty()) {
      continue;
    }
    const auto is_inside = [&](const int point) {
      return keep_above ? distances[point] >= 0.0f : distances[point] <= 0.0f;
    };
    if (points.size() == 1) {
      if (is_inside(points.first())) {
        Vector<CurvePointSource> segment;
        segment.append(source_point(points.first()));
        append_curve_segment(r_segments, curve_i, segment);
      }
      continue;
    }

    if (cyclic[curve_i]) {
      bool all_inside = true;
      for (const int point : points) {
        if (!is_inside(point)) {
          all_inside = false;
          break;
        }
      }
      if (all_inside) {
        Vector<CurvePointSource> segment;
        segment.reserve(points.size());
        for (const int point : points) {
          segment.append(source_point(point));
        }
        append_curve_segment(r_segments, curve_i, segment, true);
        continue;
      }
    }

    Vector<Vector<CurvePointSource>> runs;
    Vector<CurvePointSource> current;
    const int segments_num = points.size() - 1 + int(cyclic[curve_i]);
    for (const int segment_i : IndexRange(segments_num)) {
      const int point_a = points[segment_i];
      const int point_b = segment_i + 1 < points.size() ? points[segment_i + 1] : points.first();
      const bool inside_a = is_inside(point_a);
      const bool inside_b = is_inside(point_b);

      if (inside_a) {
        append_source_point(current, source_point(point_a));
      }
      if (inside_a != inside_b) {
        append_source_point(current,
                            intersection_point(point_a,
                                               point_b,
                                               distances[point_a],
                                               distances[point_b]));
        if (!inside_b && !current.is_empty()) {
          runs.append(std::move(current));
          current.clear();
        }
      }
    }
    /* The final point of an open curve is never visited as `point_a`. Keep it explicitly. */
    if (!cyclic[curve_i] && is_inside(points.last())) {
      append_source_point(current, source_point(points.last()));
    }
    if (!current.is_empty()) {
      runs.append(std::move(current));
    }

    if (cyclic[curve_i] && runs.size() > 1 && is_inside(points.first())) {
      Vector<CurvePointSource> joined = std::move(runs.last());
      runs.pop_last();
      for (const CurvePointSource &point : runs.first()) {
        append_source_point(joined, point);
      }
      runs.first() = std::move(joined);
    }
    for (Vector<CurvePointSource> &run : runs) {
      append_curve_segment(r_segments, curve_i, run);
    }
  }
}

static void mix_point_attribute(const GVArray &src,
                                const CurvePointSource &source,
                                const GMutableSpan dst,
                                const int dst_index)
{
  const CPPType &type = src.type();
  if (source.index_a == source.index_b) {
    src.get(source.index_a, dst[dst_index]);
    return;
  }
  if (type.is<MStringProperty>()) {
    src.get(source.factor < 0.5f ? source.index_a : source.index_b, dst[dst_index]);
    return;
  }
  bke::attribute_math::to_static_type(type, [&]<typename T>() {
    const VArray<T> values = src.typed<T>();
    dst.typed<T>()[dst_index] = bke::attribute_math::mix2(
        source.factor, values[source.index_a], values[source.index_b]);
  });
}

static bke::CurvesGeometry create_curve_side(const bke::CurvesGeometry &src_curves,
                                             const Span<CurveSegment> segments,
                                             const AttributeFilter &attribute_filter,
                                             const Span<float> distances = {},
                                             Array<bool> *r_clip_boundary = nullptr)
{
  if (segments.is_empty()) {
    return {};
  }

  int points_num = 0;
  for (const CurveSegment &segment : segments) {
    points_num += segment.points.size();
  }
  bke::CurvesGeometry dst_curves(points_num, segments.size());
  BKE_defgroup_copy_list(&dst_curves.vertex_group_names, &src_curves.vertex_group_names);

  MutableSpan<int> offsets = dst_curves.offsets_for_write();
  int point_offset = 0;
  for (const int segment_i : segments.index_range()) {
    offsets[segment_i] = point_offset;
    point_offset += segments[segment_i].points.size();
  }
  offsets.last() = point_offset;
  dst_curves.fill_curve_types(CURVE_TYPE_POLY);
  MutableSpan<bool> cyclic = dst_curves.cyclic_for_write();
  for (const int segment_i : segments.index_range()) {
    cyclic[segment_i] = segments[segment_i].cyclic;
  }

  const AttributeAccessor src_attributes = src_curves.attributes();
  MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();
  src_attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (attribute_filter.allow_skip(iter.name)) {
      return;
    }
    if (ELEM(iter.name, "position", "curve_type", "cyclic")) {
      return;
    }
    const bke::GAttributeReader src = iter.get();
    const CommonVArrayInfo info = src.varray.common_info();
    if (info.type == CommonVArrayInfo::Type::Single) {
      const GPointer value(src.varray.type(), info.data);
      if (dst_attributes.add(iter.name,
                             iter.domain,
                             iter.data_type,
                             bke::AttributeInitValue(value)))
      {
        return;
      }
    }
    bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        iter.name, iter.domain, iter.data_type);
    if (!dst) {
      return;
    }
    if (iter.domain == AttrDomain::Curve) {
      for (const int segment_i : segments.index_range()) {
        src.varray.get(segments[segment_i].src_curve, dst.span[segment_i]);
      }
    }
    else if (iter.domain == AttrDomain::Point) {
      int dst_point = 0;
      for (const CurveSegment &segment : segments) {
        for (const CurvePointSource &source : segment.points) {
          mix_point_attribute(src.varray, source, dst.span, dst_point++);
        }
      }
    }
    dst.finish();
  });

  const Span<float3> src_positions = src_curves.positions();
  MutableSpan<float3> dst_positions = dst_curves.positions_for_write();
  int dst_point = 0;
  for (const CurveSegment &segment : segments) {
    for (const CurvePointSource &source : segment.points) {
      dst_positions[dst_point++] = source.index_a == source.index_b ?
                                       src_positions[source.index_a] :
                                       math::interpolate(src_positions[source.index_a],
                                                         src_positions[source.index_b],
                                                         source.factor);
    }
  }

  if (r_clip_boundary != nullptr) {
    r_clip_boundary->reinitialize(points_num);
    int dst_i = 0;
    for (const CurveSegment &segment : segments) {
      for (const CurvePointSource &source : segment.points) {
        bool on_cut = source.index_a != source.index_b;
        if (!on_cut && !distances.is_empty()) {
          on_cut = std::abs(distances[source.index_a]) <= 1.0e-6f;
        }
        (*r_clip_boundary)[dst_i++] = on_cut;
      }
    }
  }

  dst_curves.update_curve_types();
  dst_curves.remove_attributes_based_on_types();
  dst_curves.tag_topology_changed();
  return dst_curves;
}

static ClippedCurves clip_curves(const bke::CurvesGeometry &src_curves,
                                 const float3 &plane_position,
                                 const float3 &plane_normal,
                                 const bool need_above,
                                 const bool need_below,
                                 const AttributeFilter &above_filter,
                                 const AttributeFilter &below_filter)
{
  if (src_curves.is_empty()) {
    return {};
  }

  /* Build the evaluated offsets/positions caches before querying their sizes. */
  const Span<float3> evaluated_positions = src_curves.evaluated_positions();
  src_curves.ensure_can_interpolate_to_evaluated();
  bke::CurvesGeometry poly_curves(
      evaluated_positions.size(), src_curves.curves_num());
  BKE_defgroup_copy_list(&poly_curves.vertex_group_names, &src_curves.vertex_group_names);
  const OffsetIndices evaluated_points = src_curves.evaluated_points_by_curve();
  const int evaluated_offset = evaluated_points.data().first();
  MutableSpan<int> poly_offsets = poly_curves.offsets_for_write();
  for (const int i : poly_offsets.index_range()) {
    poly_offsets[i] = evaluated_points.data()[i] - evaluated_offset;
  }
  poly_curves.fill_curve_types(CURVE_TYPE_POLY);
  array_utils::copy(src_curves.cyclic(), poly_curves.cyclic_for_write());

  const AttributeAccessor src_attributes = src_curves.attributes();
  MutableAttributeAccessor poly_attributes = poly_curves.attributes_for_write();
  src_attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (ELEM(iter.name, "position", "curve_type", "cyclic")) {
      return;
    }
    const bke::GAttributeReader src = iter.get();
    const CommonVArrayInfo info = src.varray.common_info();
    if (info.type == CommonVArrayInfo::Type::Single) {
      const GPointer value(src.varray.type(), info.data);
      if (poly_attributes.add(iter.name,
                              iter.domain,
                              iter.data_type,
                              bke::AttributeInitValue(value)))
      {
        return;
      }
    }
    bke::GSpanAttributeWriter dst = poly_attributes.lookup_or_add_for_write_only_span(
        iter.name, iter.domain, iter.data_type);
    if (!dst) {
      return;
    }
    if (iter.domain == AttrDomain::Curve) {
      src.varray.materialize(dst.span.data());
    }
    else if (iter.domain == AttrDomain::Point) {
      if (iter.data_type == bke::AttrType::String) {
        const OffsetIndices src_points = src_curves.points_by_curve();
        const OffsetIndices dst_points = poly_curves.points_by_curve();
        const VArray<MStringProperty> src_strings = src.varray.typed<MStringProperty>();
        MutableSpan<MStringProperty> dst_strings = dst.span.typed<MStringProperty>();
        for (const int curve_i : src_curves.curves_range()) {
          const IndexRange src_range = src_points[curve_i];
          const IndexRange dst_range = dst_points[curve_i];
          if (src_range.is_empty()) {
            continue;
          }
          if (src_range.size() == dst_range.size()) {
            for (const int i : dst_range.index_range()) {
              dst_strings[dst_range[i]] = src_strings[src_range[i]];
            }
          }
          else {
            for (const int i : dst_range.index_range()) {
              const float factor = dst_range.size() > 1 ?
                                       float(i) / float(dst_range.size() - 1) :
                                       0.0f;
              const int src_i = std::clamp(
                  int(std::round(factor * float(src_range.size() - 1))),
                  0,
                  int(src_range.size() - 1));
              dst_strings[dst_range[i]] = src_strings[src_range[src_i]];
            }
          }
        }
      }
      else {
        const GVArraySpan src_span(src.varray);
        src_curves.interpolate_to_evaluated(src_span, dst.span);
      }
    }
    dst.finish();
  });
  poly_curves.positions_for_write().copy_from(evaluated_positions);
  poly_curves.update_curve_types();
  poly_curves.remove_attributes_based_on_types();
  poly_curves.tag_topology_changed();
  Array<float> distances(poly_curves.points_num());
  const Span<float3> positions = poly_curves.positions();
  threading::parallel_for(positions.index_range(), 4096, [&](const IndexRange range) {
    for (const int point : range) {
      distances[point] = math::dot(plane_normal, positions[point] - plane_position);
      if (std::abs(distances[point]) <= 1.0e-6f) {
        distances[point] = 0.0f;
      }
    }
  });

  ClippedCurves result;
  if (need_above) {
    Vector<CurveSegment> segments;
    build_curve_segments(poly_curves, distances, true, segments);
    result.above = create_curve_side(
        poly_curves, segments, above_filter, distances, &result.above_clip_boundary);
  }
  if (need_below) {
    Vector<CurveSegment> segments;
    build_curve_segments(poly_curves, distances, false, segments);
    result.below = create_curve_side(
        poly_curves, segments, below_filter, distances, &result.below_clip_boundary);
  }
  return result;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet input = params.extract_input<GeometrySet>("Geometry"_ustr);
  input.keep_only({GeometryComponent::Type::Mesh, GeometryComponent::Type::Curve});

  const Mode mode = params.extract_input<Mode>("Mode"_ustr);
  const float3 plane_position = params.extract_input<float3>("Plane Position"_ustr);
  float normal_length;
  const float3 plane_normal = math::normalize_and_get_length(
      params.extract_input<float3>("Plane Normal"_ustr), normal_length);
  if (normal_length == 0.0f) {
    params.error_message_add(NodeWarningType::Error, TIP_("Plane normal cannot be zero"));
    params.set_default_remaining_outputs();
    return;
  }

  const bool need_above = mode != Mode::Below &&
                          (mode != Mode::Both || params.output_is_required("Above"_ustr));
  const bool need_below = mode != Mode::Above &&
                          (mode != Mode::Both || params.output_is_required("Below"_ustr));
  const UString above_output = mode == Mode::Both ? "Above"_ustr : "Geometry"_ustr;
  const UString below_output = mode == Mode::Both ? "Below"_ustr : "Geometry"_ustr;
  const NodeAttributeFilter above_filter = params.get_attribute_filter(above_output);
  const NodeAttributeFilter below_filter = params.get_attribute_filter(below_output);

  const std::optional<std::string> clip_boundary_id =
      params.get_output_anonymous_attribute_id_if_needed("Clip Boundary"_ustr);

  Mesh *above_mesh = nullptr;
  Mesh *below_mesh = nullptr;
  Array<bool> above_mesh_boundary;
  Array<bool> below_mesh_boundary;
  if (const Mesh *mesh = input.get_mesh()) {
    geometry::MeshPlaneClipResult clipped = geometry::mesh_clip_by_plane_both(*mesh,
                                                                              plane_position,
                                                                              plane_normal,
                                                                              need_above,
                                                                              need_below,
                                                                              above_filter,
                                                                              below_filter);
    above_mesh = clipped.above;
    below_mesh = clipped.below;
    above_mesh_boundary = std::move(clipped.above_clip_boundary);
    below_mesh_boundary = std::move(clipped.below_clip_boundary);
  }

  Curves *above_curves = nullptr;
  Curves *below_curves = nullptr;
  Array<bool> above_curve_boundary;
  Array<bool> below_curve_boundary;
  if (const Curves *curves_id = input.get_curves()) {
    ClippedCurves clipped = clip_curves(curves_id->geometry.wrap(),
                                        plane_position,
                                        plane_normal,
                                        need_above,
                                        need_below,
                                        above_filter,
                                        below_filter);
    if (need_above && !clipped.above.is_empty()) {
      above_curves = bke::curves_new_nomain(std::move(clipped.above));
      bke::curves_copy_parameters(*curves_id, *above_curves);
      above_curve_boundary = std::move(clipped.above_clip_boundary);
    }
    if (need_below && !clipped.below.is_empty()) {
      below_curves = bke::curves_new_nomain(std::move(clipped.below));
      bke::curves_copy_parameters(*curves_id, *below_curves);
      below_curve_boundary = std::move(clipped.below_clip_boundary);
    }
  }

  if (need_above) {
    write_clip_boundary_selection(above_mesh,
                                  above_curves,
                                  clip_boundary_id,
                                  above_mesh_boundary,
                                  above_curve_boundary);
  }
  if (need_below) {
    write_clip_boundary_selection(below_mesh,
                                  below_curves,
                                  clip_boundary_id,
                                  below_mesh_boundary,
                                  below_curve_boundary);
  }

  if (mode == Mode::Both) {
    if (need_above) {
      params.set_output(
          "Above"_ustr,
          geometry_set_from_components(above_mesh, above_curves, input.name()));
    }
    if (need_below) {
      params.set_output(
          "Below"_ustr,
          geometry_set_from_components(below_mesh, below_curves, input.name()));
    }
  }
  else if (mode == Mode::Above) {
    params.set_output(
        "Geometry"_ustr,
        geometry_set_from_components(above_mesh, above_curves, input.name()));
  }
  else {
    params.set_output(
        "Geometry"_ustr,
        geometry_set_from_components(below_mesh, below_curves, input.name()));
  }
  params.set_default_remaining_outputs();
}


static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeGeometryClip"_ustr, GEO_NODE_GEOMETRY_CLIP);
  ntype.ui_name = "Geometry Clip";
  ntype.ui_description =
      "Clip mesh and curve geometry with a plane. Clip Boundary is a point selection "
      "of vertices created on the cut (and original points that already lie on the plane)";
  ntype.enum_name_legacy = "GEOMETRY_CLIP";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.default_width = bke::NodeWidth::_180;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_geometry_clip_cc
