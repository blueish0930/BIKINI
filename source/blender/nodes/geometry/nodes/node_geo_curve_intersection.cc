/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <map>
#include <numbers>

#include "DNA_pointcloud_types.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "BLI_kdopbvh.hh"
#include "BLI_listbase.hh"
#include "BLI_math_geom.hh"
#include "BLI_noise.hh"
#include "BLI_task.hh"

#include "GEO_foreach_geometry.hh"
#include "GEO_resample_curves.hh"

#include "NOD_socket_usage_inference.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curve_intersection_cc {

/* Epsilon values for curve intersections and bvh tree. */
constexpr float curve_isect_eps = 0.001f;
constexpr float pi_2_f = std::numbers::pi * 0.5f;
constexpr float min_angle_eps = 0.0001f;
constexpr float pi_2_f_eps = pi_2_f - min_angle_eps;

enum class PairData {
  PointsOnly = 0,
  FullPair = 1,
};

enum class IntersectionMode {
  Curve = 0,
  Plane = 1,
  Surface = 2,
  Curve_Project = 3,
};

static const EnumPropertyItem mode_items[] = {
    {int(IntersectionMode::Curve),
     "CURVE",
     0,
     N_("Curve"),
     N_("Find the intersection positions between curves in 3d space")},
    {int(IntersectionMode::Curve_Project),
     "CURVE_PROJECT",
     0,
     N_("Curve Project"),
     N_("Find all the intersection positions for all curves projected onto orthographic plane")},
    {int(IntersectionMode::Plane),
     "PLANE",
     0,
     N_("Plane"),
     N_("Find all the intersection positions for each curve in reference to a plane")},
    {int(IntersectionMode::Surface),
     "SURFACE",
     0,
     N_("Surface"),
     N_("Find all the intersection positions for each curve in reference to a mesh surface")},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem pair_data_mode_items[] = {
    {int(PairData::PointsOnly),
     "POINTS_ONLY",
     0,
     N_("Points Only"),
     N_("Return intersection points only")},
    {int(PairData::FullPair),
     "FULL_PAIR",
     0,
     N_("Full Data"),
     N_("Return all intersections and corresponding pair data")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  auto enable_output =
      [](const socket_usage_inference::SocketUsageParams &params) -> std::optional<bool> {
    return params.menu_input_may_be("Paired Data Mode"_ustr, int(PairData::FullPair)) ||
           params.menu_input_may_be("Mode"_ustr, int(IntersectionMode::Plane)) ||
           params.menu_input_may_be("Mode"_ustr, int(IntersectionMode::Surface));
  };

  b.use_custom_socket_order();
  b.allow_any_socket_order();

  /* Outputs. */
  b.add_output<decl::Geometry>("Points"_ustr);
  b.add_output<decl::Int>("Curve Index"_ustr).anonymous_attribute_output().usage_inference(enable_output);
  b.add_output<decl::Vector>("Direction"_ustr)
      .anonymous_attribute_output()
      .usage_inference(enable_output)
      .description(
          "The direction of the curve at the intersection point. For project mode, this is the "
          "projected direction");
  b.add_output<decl::Float>("Factor"_ustr)
      .anonymous_attribute_output()
      .usage_inference(enable_output)
      .description("The portion of the spline's total length at the intersection point");
  b.add_output<decl::Float>("Length"_ustr)
      .anonymous_attribute_output()
      .usage_inference(enable_output)
      .description("The distance along the spline at the intersection point");
  b.add_output<decl::Vector>("Normal"_ustr)
      .anonymous_attribute_output()
      .usage_by_menu("Mode"_ustr, int(IntersectionMode::Surface))
      .description("The normal of surface or plane intersection");

  /* Menus. */
  b.add_input<decl::Menu>("Mode"_ustr)
      .static_items(mode_items)
      .default_value(MenuValue(IntersectionMode::Curve))
      .optional_label();
  b.add_input<decl::Menu>("Paired Data Mode"_ustr)
      .static_items(pair_data_mode_items)
      .default_value(MenuValue(PairData::PointsOnly))
      .usage_by_menu("Mode"_ustr,
                     {int(IntersectionMode::Curve), int(IntersectionMode::Curve_Project)})
      .optional_label();

  /* Pair data outputs. */
  PanelDeclarationBuilder &pair_data = b.add_panel("Pair Data"_ustr)
                                           .default_closed(true)
                                           .description("Panel contains outputs for pair data");
  pair_data.add_output<decl::Vector>("Pair Position"_ustr)
      .anonymous_attribute_output()
      .usage_by_menu("Paired Data Mode"_ustr, {int(PairData::FullPair)})
      .description("Position of the oppposing pair point");
  pair_data.add_output<decl::Vector>("Pair Direction"_ustr)
      .anonymous_attribute_output()
      .usage_by_menu("Paired Data Mode"_ustr, {int(PairData::FullPair)})
      .description(
          "Direction of the oppposing pair point. For project mode, this is the "
          "projected direction");
  pair_data.add_output<decl::Bool>("Pair"_ustr)
      .anonymous_attribute_output()
      .usage_by_menu("Paired Data Mode"_ustr, {int(PairData::FullPair)})
      .description("If the intersection is one of a pair of matching intersections");
  pair_data.add_output<decl::Int>("Pair ID"_ustr)
      .anonymous_attribute_output()
      .usage_by_menu("Paired Data Mode"_ustr, {int(PairData::FullPair)})
      .description("Unique ID value for each pair");

  /* Inputs. */
  b.add_input<decl::Geometry>("Curve"_ustr).supported_type(GeometryComponent::Type::Curve);
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .usage_by_menu("Mode"_ustr, int(IntersectionMode::Surface));
  b.add_input<decl::Bool>("Self Intersections"_ustr)
      .default_value(false)
      .usage_by_menu("Mode"_ustr,
                     {int(IntersectionMode::Curve), int(IntersectionMode::Curve_Project)})
      .description("Include self intersections");
  b.add_input<decl::Bool>("All Intersections"_ustr)
      .default_value(true)
      .usage_by_menu("Mode"_ustr,
                     {int(IntersectionMode::Curve), int(IntersectionMode::Curve_Project)})
      .description("Include all intersections except self intersections");
  b.add_input<decl::Float>("Distance"_ustr)
      .subtype(PROP_DISTANCE)
      .min(0.0f)
      .evaluated_geometry_field()
      .usage_by_menu("Mode"_ustr,
                     {int(IntersectionMode::Curve), int(IntersectionMode::Curve_Project)})
      .description("Maximum distance between intersections");
  b.add_input<decl::Vector>("Direction"_ustr)
      .default_value({0.0f, 0.0f, 1.0f})
      .usage_by_menu("Mode"_ustr,
                     {int(IntersectionMode::Plane), int(IntersectionMode::Curve_Project)})
      .description("Direction of orthographic plane");
  b.add_input<decl::Vector>("Center"_ustr)
      .subtype(PROP_DISTANCE)
      .usage_by_menu("Mode"_ustr, int(IntersectionMode::Plane))
      .description("Center of plane");

  /* Common sockets. */
  b.add_input<decl::Float>("Min Angle"_ustr)
      .subtype(PROP_ANGLE)
      .min(0.0f)
      .max(pi_2_f)
      .description("Minimum shortest angle for intersections");
  b.add_input<decl::Float>("Max Angle"_ustr)
      .subtype(PROP_ANGLE)
      .default_value(pi_2_f)
      .min(0.0f)
      .max(pi_2_f)
      .description("Maximum shortest angle for intersections");
}

/* Attribute outputs. */
struct AttributeOutputs {
  std::optional<std::string> curve_index;
  std::optional<std::string> direction;
  std::optional<std::string> factor;
  std::optional<std::string> length;
  std::optional<std::string> normal;
  std::optional<std::string> pair_position;
  std::optional<std::string> pair_direction;
  std::optional<std::string> pair;
  std::optional<std::string> pair_id;
  std::optional<std::string> hash;
  bool id;
};

/* Store information from line intersection calculations. */
struct IntersectingLineInfo {
  float3 closest_ab;
  float3 closest_cd;
  float lambda_ab;
  float lambda_cd;
  float distance;
  bool is_intersection;
};

/* Store segment details. Start and End must be at the start for BVHTree search. */
struct Segment {
  float3 start;
  float3 end;
  float3 orig_start;
  float3 orig_end;
  float3 direction;
  float dist_start;
  float dist_end;
  float len_start;
  float len_end;
  float curve_length;
  int2 pos_index;
  int seg_index;
  int curve_id;
  int curve_index;
  bool is_cyclic_segment;
};

/* Store data that will be used for attributes and sorting. */
struct IntersectionData {
  Vector<float> sortkey;
  Vector<int> pos_index_a;
  Vector<int> pos_index_b;
  Vector<float> lambda;
  Vector<float3> position;
  Vector<int> curve_index;
  Vector<float3> direction;
  Vector<float> factor;
  Vector<float> length;
  Vector<float3> normal;
  Vector<float3> pair_position;
  Vector<float3> pair_direction;
  Vector<bool> pair;
  Vector<int> pair_id;
  Vector<int> hash;
};

using ThreadLocalData = threading::EnumerableThreadSpecific<IntersectionData>;

static void add_intersection_data(IntersectionData &data,
                                  const int2 pos_index,
                                  const float lambda,
                                  const float3 position,
                                  const int curve_index,
                                  const float3 direction,
                                  const float length,
                                  const float curve_length,
                                  const float3 normal,
                                  const float3 pair_position,
                                  const float3 pair_direction,
                                  const bool pair,
                                  const int pair_id,
                                  const AttributeOutputs &attribute_outputs)
{
  const float factor = math::safe_divide(length, curve_length);

  /* Create sortkey for index. Order by curve_index then factor. */
  const float sortkey = float(curve_index * 2) + factor;
  data.sortkey.append(sortkey);
  data.position.append(position);

  data.pos_index_a.append(pos_index.x);
  data.pos_index_b.append(pos_index.y);
  data.lambda.append(lambda);

  if (attribute_outputs.hash) {
    const int hash = noise::hash(curve_index, noise::hash_float(length));
    data.hash.append(hash);
  }
  if (attribute_outputs.curve_index) {
    data.curve_index.append(curve_index);
  }
  if (attribute_outputs.factor) {
    data.factor.append(factor);
  }
  if (attribute_outputs.length) {
    data.length.append(length);
  }
  if (attribute_outputs.direction) {
    data.direction.append(direction);
  }
  if (attribute_outputs.normal) {
    data.normal.append(normal);
  }
  if (attribute_outputs.pair_position) {
    data.pair_position.append(pair_position);
  }
  if (attribute_outputs.pair_direction) {
    data.pair_direction.append(pair_direction);
  }
  if (attribute_outputs.pair) {
    data.pair.append(pair);
  }
  if (attribute_outputs.pair_id) {
    data.pair_id.append(pair_id);
  }
}

static void gather_thread_storage(ThreadLocalData &thread_storage,
                                  IntersectionData &r_data,
                                  const AttributeOutputs &attribute_outputs)
{
  int64_t total_intersections = 0;
  for (const IntersectionData &local_data : thread_storage) {
    const int64_t local_size = local_data.position.size();
    BLI_assert(local_data.sortkey.size() == local_size);
    BLI_assert(attribute_outputs.curve_index && local_data.curve_index.size() == local_size);
    BLI_assert(attribute_outputs.hash && local_data.hash.size() == local_size);
    BLI_assert(attribute_outputs.factor && local_data.factor.size() == local_size);
    BLI_assert(attribute_outputs.length && local_data.length.size() == local_size);
    BLI_assert(attribute_outputs.direction && local_data.direction.size() == local_size);
    BLI_assert(attribute_outputs.normal && local_data.normal.size() == local_size);
    BLI_assert(attribute_outputs.pair_position && local_data.pair_position.size() == local_size);
    BLI_assert(attribute_outputs.pair_direction && local_data.pair_direction.size() == local_size);
    BLI_assert(attribute_outputs.pair && local_data.pair.size() == local_size);
    BLI_assert(attribute_outputs.pair_id && local_data.pair_id.size() == local_size);
    total_intersections += local_size;
  }
  const int64_t start_index = r_data.position.size();
  const int64_t new_size = start_index + total_intersections;
  r_data.position.reserve(new_size);
  r_data.sortkey.reserve(new_size);

  r_data.pos_index_a.reserve(new_size);
  r_data.pos_index_b.reserve(new_size);
  r_data.lambda.reserve(new_size);

  if (attribute_outputs.hash) {
    r_data.hash.reserve(new_size);
  }
  if (attribute_outputs.curve_index) {
    r_data.curve_index.reserve(new_size);
  }
  if (attribute_outputs.factor) {
    r_data.factor.reserve(new_size);
  }
  if (attribute_outputs.length) {
    r_data.length.reserve(new_size);
  }
  if (attribute_outputs.direction) {
    r_data.direction.reserve(new_size);
  }
  if (attribute_outputs.normal) {
    r_data.normal.reserve(new_size);
  }
  if (attribute_outputs.pair_position) {
    r_data.pair_position.reserve(new_size);
  }
  if (attribute_outputs.pair_direction) {
    r_data.pair_direction.reserve(new_size);
  }
  if (attribute_outputs.pair) {
    r_data.pair.reserve(new_size);
  }
  if (attribute_outputs.pair_id) {
    r_data.pair_id.reserve(new_size);
  }

  for (IntersectionData &local_data : thread_storage) {
    r_data.position.extend(local_data.position);
    r_data.sortkey.extend(local_data.sortkey);

    r_data.pos_index_a.extend(local_data.pos_index_a);
    r_data.pos_index_b.extend(local_data.pos_index_b);
    r_data.lambda.extend(local_data.lambda);

    if (attribute_outputs.hash) {
      r_data.hash.extend(local_data.hash);
    }
    if (attribute_outputs.curve_index) {
      r_data.curve_index.extend(local_data.curve_index);
    }
    if (attribute_outputs.factor) {
      r_data.factor.extend(local_data.factor);
    }
    if (attribute_outputs.length) {
      r_data.length.extend(local_data.length);
    }
    if (attribute_outputs.direction) {
      r_data.direction.extend(local_data.direction);
    }
    if (attribute_outputs.normal) {
      r_data.normal.extend(local_data.normal);
    }
    if (attribute_outputs.pair_position) {
      r_data.pair_position.extend(local_data.pair_position);
    }
    if (attribute_outputs.pair_direction) {
      r_data.pair_direction.extend(local_data.pair_direction);
    }
    if (attribute_outputs.pair) {
      r_data.pair.extend(local_data.pair);
    }
    if (attribute_outputs.pair_id) {
      r_data.pair_id.extend(local_data.pair_id);
    }
  }
}

/* Minimum angle between two vectors in 0-PI/2 (90 degree) range. */
static bool discard_angle(const float3 an,
                          const float3 bn,
                          const bool is_face_normal,
                          const float2 min_max_angle)
{
  float angle = math::abs(math::abs(angle_normalized_v3v3(an, bn)) - pi_2_f);
  angle = is_face_normal ? angle : pi_2_f - angle;
  return min_max_angle.x > angle || min_max_angle.y + 0.0001f < angle;
}

/* Check intersection between the line segments ab and cd. Return true only if intersection point
 * is located on both line segments. */
static IntersectingLineInfo intersecting_lines(const Segment &ab,
                                               const Segment &cd,
                                               const float distance,
                                               const bool use_distance_field)
{
  IntersectingLineInfo isectinfo{};
  isectinfo.is_intersection = false;
  if (isect_line_line_epsilon_v3(ab.start,
                                 ab.end,
                                 cd.start,
                                 cd.end,
                                 isectinfo.closest_ab,
                                 isectinfo.closest_cd,
                                 curve_isect_eps) != 0)
  {
    /* Discard intersections too far away. */
    const float isect_distance = math::distance(isectinfo.closest_ab, isectinfo.closest_cd);
    if (isect_distance > distance) {
      return isectinfo;
    }
    /* Check intersection is on both line segments ab and cd. */
    isectinfo.lambda_ab = closest_to_line_v3(
        isectinfo.closest_ab, isectinfo.closest_ab, ab.start, ab.end);
    if (isectinfo.lambda_ab <= -curve_isect_eps || isectinfo.lambda_ab >= 1.0f + curve_isect_eps) {
      return isectinfo;
    }
    isectinfo.lambda_cd = closest_to_line_v3(
        isectinfo.closest_cd, isectinfo.closest_cd, cd.start, cd.end);
    if (isectinfo.lambda_cd <= -curve_isect_eps || isectinfo.lambda_cd >= 1.0f + curve_isect_eps) {
      return isectinfo;
    }

    isectinfo.lambda_ab = math::clamp(isectinfo.lambda_ab, 0.0f, 1.0f);
    isectinfo.lambda_cd = math::clamp(isectinfo.lambda_cd, 0.0f, 1.0f);
    isectinfo.closest_ab = math::interpolate(ab.orig_start, ab.orig_end, isectinfo.lambda_ab);
    isectinfo.closest_cd = math::interpolate(cd.orig_start, cd.orig_end, isectinfo.lambda_cd);
    isectinfo.distance = math::distance(isectinfo.closest_ab, isectinfo.closest_cd);
    const float actual_distance =
        use_distance_field ?
            math::interpolate(ab.dist_start, ab.dist_end, isectinfo.lambda_ab) +
                math::interpolate(cd.dist_start, cd.dist_end, isectinfo.lambda_cd) :
            distance;
    if (isectinfo.distance <= actual_distance) {
      isectinfo.is_intersection = true;
      return isectinfo;
    }
  }
  return isectinfo;
}

static float3 project_v3_plane(const float3 vector, const float3 direction)
{
  return vector - math::project(vector, direction);
}

/* Build curve segment bvh. */
static BVHTree *create_curve_segment_bvhtree(const bke::CurvesGeometry &src_curves,
                                             const std::optional<VArray<float>> &distances,
                                             const VArray<int> &ids,
                                             Vector<Segment> *r_curve_segments,
                                             const float2 min_max_angle,
                                             const bool project,
                                             const float3 project_axis,
                                             const AttributeOutputs &attribute_outputs)
{
  /* Lengths are required to return factor information for sorting. */
  src_curves.ensure_evaluated_lengths();

  const int bvh_points_num = src_curves.evaluated_points_num() + src_curves.curves_num();
  BVHTree *bvhtree = BLI_bvhtree_new(bvh_points_num, curve_isect_eps, 8, 8);
  const VArray<bool> cyclic = src_curves.cyclic();
  const OffsetIndices evaluated_points_by_curve = src_curves.evaluated_points_by_curve();
  const bool use_distance_field = distances.has_value();
  Array<float> distance_values;
  if (use_distance_field) {
    distance_values = distances.value().get_internal_span();
  }
  const bool use_direction_data = min_max_angle.x > 0.0f || min_max_angle.y < pi_2_f ||
                                  attribute_outputs.direction || attribute_outputs.pair_direction;

  /* Preprocess curve segments for each curve. */

  for (const int64_t curve_i : src_curves.curves_range()) {
    const IndexRange points = evaluated_points_by_curve[curve_i];
    const int curve_start_index = points.start();
    const Span<float3> positions = src_curves.evaluated_positions().slice(points);
    Span<float> dist_by_curve;
    if (use_distance_field) {
      dist_by_curve = distance_values.as_span().slice(points);
    }
    const Span<float> lengths = src_curves.evaluated_lengths_for_curve(curve_i, cyclic[curve_i]);
    const float curve_length = src_curves.evaluated_length_total_for_curve(curve_i,
                                                                           cyclic[curve_i]);
    const int segment_count = positions.size() - 1;

    auto add_segment = [&](const bool is_cyclic,
                           const int2 pos_index,
                           const int index,
                           const float3 start,
                           const float3 end,
                           const float dist_start,
                           const float dist_end,
                           const float len_start,
                           const float len_end) {
      Segment segment;
      segment.pos_index = pos_index;
      segment.is_cyclic_segment = is_cyclic;
      segment.seg_index = index;
      segment.orig_start = start;
      segment.orig_end = end;
      segment.len_start = len_start;
      segment.len_end = len_end;
      segment.curve_id = attribute_outputs.id ? ids[curve_i] : curve_i;
      segment.curve_index = curve_i;
      segment.curve_length = curve_length;
      segment.dist_start = dist_start;
      segment.dist_end = dist_end;
      segment.start = project ? project_v3_plane(start, project_axis) : start;
      segment.end = project ? project_v3_plane(end, project_axis) : end;
      segment.direction = use_direction_data ? math::normalize(segment.end - segment.start) :
                                               float3(0.0f);
      const int bvh_index = r_curve_segments->append_and_get_index(segment);
      BLI_bvhtree_insert(bvhtree, bvh_index, reinterpret_cast<float *>(&segment), 2);
    };

    for (const int index : IndexRange(positions.size()).drop_back(1)) {
      const float3 start = positions[index];
      const float3 end = positions[1 + index];
      const float dist_start = use_distance_field ? dist_by_curve[index] : 0.0f;
      const float dist_end = use_distance_field ? dist_by_curve[1 + index] : 0.0f;
      const float len_start = (index == 0) ? 0.0f : lengths[index - 1];
      const float len_end = lengths[index];
      add_segment(false,
                  int2(index + curve_start_index, 1 + index + curve_start_index),
                  index,
                  start,
                  end,
                  dist_start,
                  dist_end,
                  len_start,
                  len_end);
    }
    if (cyclic[curve_i]) {
      const float3 start = positions.last();
      const float3 end = positions.first();
      const float dist_start = use_distance_field ? dist_by_curve.last() : 0.0f;
      const float dist_end = use_distance_field ? dist_by_curve.first() : 0.0f;
      const float len_start = lengths[segment_count - 1];
      const float len_end = curve_length;
      add_segment(true,
                  int2(curve_start_index + segment_count, curve_start_index),
                  segment_count,
                  start,
                  end,
                  dist_start,
                  dist_end,
                  len_start,
                  len_end);
    }
  }

  BLI_bvhtree_balance(bvhtree);

  return bvhtree;
}

/* Based on existing function `isect_line_plane_v3` but with following changes.
 * a) adds an check that lines cross between start and end points.^
 * b) stores lambda value.^^ */
static bool isect_line_plane_v3_crossing(const float3 point_1,
                                         const float3 point_2,
                                         const float3 surface_center,
                                         const float3 surface_normal,
                                         float3 &r_isect_co,
                                         float &r_lambda)
{
  const float3 u = point_2 - point_1;
  const float dot = math::dot(surface_normal, u);

  /* The segment is parallel to plane.^ */
  if (math::abs(dot) <= FLT_EPSILON) {
    return false;
  }
  const float3 h = point_1 - surface_center;
  r_lambda = -math::dot(surface_normal, h) / dot;

  /* Test lambda to check intersection is between the start and end points.^^ */
  if (r_lambda >= -curve_isect_eps && r_lambda <= 1.0f + curve_isect_eps) {
    /* Remove epsilon from lambda. */
    r_lambda = math::clamp(r_lambda, 0.0f, 1.0f);
    r_isect_co = point_1 + u * r_lambda;
    return true;
  }

  return false;
}

/* Calculate intersections between a curve and a plane. */
static void set_curve_intersections_plane(const bke::CurvesGeometry &src_curves,
                                          const float3 plane_center,
                                          const float3 plane_direction,
                                          const float2 min_max_angle,
                                          const AttributeOutputs &attribute_outputs,
                                          IntersectionData &r_data)
{
  const VArray<bool> cyclic = src_curves.cyclic();
  const OffsetIndices evaluated_points_by_curve = src_curves.evaluated_points_by_curve();
  src_curves.ensure_evaluated_lengths();
  const bool use_angle = min_max_angle.x > 0.0f || min_max_angle.y < pi_2_f;
  const bool use_direction_data = attribute_outputs.direction || attribute_outputs.pair_direction;

  /* Loop each curve for intersections. */
  ThreadLocalData thread_storage;
  threading::parallel_for(src_curves.curves_range(), 128, [&](IndexRange curve_range) {
    IntersectionData &local_data = thread_storage.local();
    threading::isolate_task([&]() {
      for (const int64_t curve_i : curve_range) {
        const IndexRange points = evaluated_points_by_curve[curve_i];
        const int curve_start_index = points.start();
        const Span<float3> positions = src_curves.evaluated_positions().slice(points);
        if (positions.size() <= 1) {
          continue;
        }
        const Span<float> lengths = src_curves.evaluated_lengths_for_curve(curve_i,
                                                                           cyclic[curve_i]);
        const float curve_length = src_curves.evaluated_length_total_for_curve(curve_i,
                                                                               cyclic[curve_i]);

        auto add_closest = [&](const int2 pos_index,
                               const float3 a,
                               const float3 b,
                               const float len_start,
                               const float len_end,
                               const float curve_length) {
          const float3 segment_direction = use_direction_data || use_angle ?
                                               math::normalize(b - a) :
                                               float3(0.0f);
          /* Discard by angle. */
          if (use_angle && discard_angle(segment_direction, plane_direction, true, min_max_angle))
          {
            return;
          }

          float3 closest = float3(0.0f);
          float lambda = 0.0f;
          if (isect_line_plane_v3_crossing(a, b, plane_center, plane_direction, closest, lambda)) {
            add_intersection_data(local_data,
                                  pos_index,
                                  lambda,
                                  closest,
                                  curve_i,
                                  segment_direction,
                                  math::interpolate(len_start, len_end, lambda),
                                  curve_length,
                                  float3(0.0f),
                                  float3(0.0f),
                                  float3(0.0f),
                                  false,
                                  0,
                                  attribute_outputs);
          }
        };

        /* Loop segments from start until we have an intersection. */
        const int segment_count = positions.size() - 1;
        for (const int index : IndexRange(positions.size()).drop_back(1)) {
          const float3 a = positions[index];
          const float3 b = positions[1 + index];
          const float len_start = (index == 0) ? 0.0f : lengths[index - 1];
          const float len_end = lengths[index];
          add_closest(int2(index + curve_start_index, 1 + index + curve_start_index),
                      a,
                      b,
                      len_start,
                      len_end,
                      curve_length);
        }
        if (cyclic[curve_i]) {
          const float3 a = positions.last();
          const float3 b = positions.first();
          const float len_start = lengths.last();
          const float len_end = curve_length;
          add_closest(int2(curve_start_index + segment_count, curve_start_index),
                      a,
                      b,
                      len_start,
                      len_end,
                      curve_length);
        }
      }
    });
  });
  gather_thread_storage(thread_storage, r_data, attribute_outputs);
}

/* Calculate intersections between curve and mesh surface. */
static void set_curve_intersections_mesh(GeometrySet &mesh_set,
                                         const bke::CurvesGeometry &src_curves,
                                         const VArray<int> &ids,
                                         const float2 min_max_angle,
                                         const AttributeOutputs &attribute_outputs,
                                         IntersectionData &r_data)
{
  /* Build bvh. */
  Vector<Segment> curve_segments;
  BVHTree *bvhtree = create_curve_segment_bvhtree(src_curves,
                                                  std::nullopt,
                                                  ids,
                                                  &curve_segments,
                                                  min_max_angle,
                                                  false,
                                                  float3(0.0f),
                                                  attribute_outputs);
  BLI_SCOPED_DEFER([&]() { BLI_bvhtree_free(bvhtree); });
  const bool use_angle = min_max_angle.x > 0.0f || min_max_angle.y < pi_2_f;
  const bool use_normal = use_angle || attribute_outputs.normal;

  /* Loop mesh data. */
  geometry::foreach_real_geometry(mesh_set, [&](GeometrySet &mesh_set) {
    if (!mesh_set.has_mesh()) {
      return;
    }
    const Mesh &mesh = *mesh_set.get_mesh();
    if (mesh.faces_num < 1) {
      return;
    }
    const Span<float3> positions = mesh.vert_positions();
    const Span<int> corner_verts = mesh.corner_verts();
    const Span<int3> corner_tris = mesh.corner_tris();

    /* Loop face data. */
    ThreadLocalData thread_storage;
    threading::parallel_for(corner_tris.index_range(), 128, [&](IndexRange range) {
      IntersectionData &local_data = thread_storage.local();
      threading::isolate_task([&]() {
        for (const int64_t face_index : range) {
          const int3 &tri = corner_tris[face_index];
          const int v0_loop = tri[0];
          const int v1_loop = tri[1];
          const int v2_loop = tri[2];
          const float3 &v0_pos = positions[corner_verts[v0_loop]];
          const float3 &v1_pos = positions[corner_verts[v1_loop]];
          const float3 &v2_pos = positions[corner_verts[v2_loop]];

          float3 center_pos;
          interp_v3_v3v3v3(center_pos, v0_pos, v1_pos, v2_pos, float3(1.0f / 3.0f));

          const float distance = math::max(
              math::max(math::distance(center_pos, v0_pos), math::distance(center_pos, v1_pos)),
              math::distance(center_pos, v2_pos));

          BLI_bvhtree_range_query_cpp(
              *bvhtree,
              center_pos,
              distance + curve_isect_eps,
              [&](const int index, const float3 & /*co*/, const float /*dist_sq*/) {
                const Segment seg = curve_segments[index];
                float lambda = 0.0f;

                float3 normal = float3(0.0f);
                if (use_normal) {
                  normal_tri_v3(normal, v0_pos, v1_pos, v2_pos);
                }

                /* Discard by angle. */
                if (use_angle && discard_angle(seg.direction, normal, true, min_max_angle)) {
                  return;
                }

                if (isect_line_segment_tri_epsilon_v3(seg.start,
                                                      seg.end,
                                                      v0_pos,
                                                      v1_pos,
                                                      v2_pos,
                                                      &lambda,
                                                      nullptr,
                                                      curve_isect_eps))
                {
                  const float len_at_isect = math::interpolate(seg.len_start, seg.len_end, lambda);
                  const float3 closest_position = math::interpolate(seg.start, seg.end, lambda);
                  add_intersection_data(local_data,
                                        seg.pos_index,
                                        lambda,
                                        closest_position,
                                        seg.curve_index,
                                        seg.direction,
                                        len_at_isect,
                                        seg.curve_length,
                                        normal,
                                        float3(0.0f),
                                        float3(0.0f),
                                        false,
                                        0,
                                        attribute_outputs);
                }
              });
        }
      });
    });
    gather_thread_storage(thread_storage, r_data, attribute_outputs);
  });
}

/* Calculate intersections between 3d curves, optionally projected onto 2d plane. */
static void set_curve_intersections(const bke::CurvesGeometry &src_curves,
                                    const Field<float> distance_field,
                                    const VArray<int> &ids,
                                    const bool self_intersect,
                                    const bool all_intersect,
                                    const float2 min_max_angle,
                                    const bool project,
                                    const float3 direction,
                                    const PairData &pair_data_mode,
                                    const AttributeOutputs &attribute_outputs,
                                    IntersectionData &r_data)
{
  const bke::GeometryFieldContext field_context{src_curves, AttrDomain::Point};
  fn::FieldEvaluator data_evaluator{field_context, src_curves.evaluated_points_num()};
  data_evaluator.add(distance_field);
  data_evaluator.evaluate();
  const VArray<float> distance_field_values = data_evaluator.get_evaluated<float>(0);
  const bool use_distance_field = !distance_field_values.is_single();
  const std::optional<float> distance_value = distance_field_values.get_if_single();
  const float distance = distance_value.value_or(0.0f);

  std::optional<VArray<float>> distances;
  if (use_distance_field) {
    distances = distance_field_values;
  }

  /* Build bvh. */
  Vector<Segment> curve_segments;
  BVHTree *bvhtree = create_curve_segment_bvhtree(src_curves,
                                                  distances,
                                                  ids,
                                                  &curve_segments,
                                                  min_max_angle,
                                                  project,
                                                  direction,
                                                  attribute_outputs);
  BLI_SCOPED_DEFER([&]() { BLI_bvhtree_free(bvhtree); });

  const int segment_count = curve_segments.size();
  const int curve_count = src_curves.curves_range().size();
  float max_search_distance = math::max(curve_isect_eps, distance);
  if (use_distance_field) {
    max_search_distance = curve_isect_eps;
    const Array<float> distance_values = distances.value().get_internal_span();
    for (const int i : distance_values.index_range()) {
      max_search_distance = math::max(max_search_distance, distance_values[i] * 2.0f);
    }
  }

  const bool use_angle = min_max_angle.x > 0.0f || min_max_angle.y < pi_2_f;

  /* Loop through segments. */
  ThreadLocalData thread_storage;
  threading::parallel_for(curve_segments.index_range(), 128, [&](IndexRange range) {
    IntersectionData &local_data = thread_storage.local();
    threading::isolate_task([&]() {
      int local_count = 0;
      for (const int64_t segment_index : range) {
        const Segment &ab = curve_segments[segment_index];
        BLI_bvhtree_range_query_cpp(
            *bvhtree,
            math::midpoint(ab.start, ab.end),
            math::distance(ab.start, ab.end) + max_search_distance,
            [&](const int index, const float3 & /*co*/, const float /*dist_sq*/) {
              if (segment_index <= index) {
                /* Skip matching segments or previously matched segments. */
                return;
              }
              const Segment &cd = curve_segments[index];
              /* Discard by angle. */
              if (use_angle && discard_angle(ab.direction, cd.direction, false, min_max_angle)) {
                return;
              }
              const bool same_id = ab.curve_id == cd.curve_id;
              const bool same_curve = ab.curve_index == cd.curve_index;
              const bool calc_all = (all_intersect && !same_curve && !same_id);
              /* Skip adjecent segments in same curve. */
              const bool not_adjacent = same_curve &&
                                        ((math::abs(ab.seg_index - cd.seg_index) > 1) &&
                                         !((ab.seg_index == 0 && cd.is_cyclic_segment) ||
                                           (cd.seg_index == 0 && ab.is_cyclic_segment)));
              const bool calc_self = (self_intersect &&
                                      (not_adjacent || (same_id && !same_curve)));

              if (calc_all || calc_self) {
                const IntersectingLineInfo isectinfo = intersecting_lines(
                    ab, cd, max_search_distance, use_distance_field);

                if (isectinfo.is_intersection) {
                  const bool pair_weight = ab.curve_index > cd.curve_index;
                  int pair_id = 0;
                  if (pair_data_mode == PairData::FullPair && attribute_outputs.pair_id) {
                    pair_id = (curve_count * segment_count * segment_index) +
                              (curve_count * ab.curve_index) + local_count++;
                  }
                  add_intersection_data(
                      local_data,
                      ab.pos_index,
                      isectinfo.lambda_ab,
                      isectinfo.closest_ab,
                      ab.curve_index,
                      ab.direction,
                      math::interpolate(ab.len_start, ab.len_end, isectinfo.lambda_ab),
                      ab.curve_length,
                      float3(0.0f),
                      isectinfo.closest_cd,
                      cd.direction,
                      !pair_weight,
                      pair_id,
                      attribute_outputs);

                  /* Only return both intersection points if required. */
                  if (pair_data_mode == PairData::FullPair ||
                      (pair_data_mode == PairData::PointsOnly &&
                       isectinfo.distance > curve_isect_eps))
                  {
                    add_intersection_data(
                        local_data,
                        cd.pos_index,
                        isectinfo.lambda_cd,
                        isectinfo.closest_cd,
                        cd.curve_index,
                        cd.direction,
                        math::interpolate(cd.len_start, cd.len_end, isectinfo.lambda_cd),
                        cd.curve_length,
                        float3(0.0f),
                        isectinfo.closest_ab,
                        ab.direction,
                        pair_weight,
                        pair_id,
                        attribute_outputs);
                  }
                }
              }
            });
      }
    });
  });
  gather_thread_storage(thread_storage, r_data, attribute_outputs);
}

/* Sorting intersections by sortkey (which is curve_index and factor postion on curve). */
static IntersectionData sort_intersection_data(IntersectionData &data,
                                               const AttributeOutputs &attribute_outputs)
{
  const int64_t data_size = data.position.size();

  Vector<std::pair<int64_t, float>> sort_index;
  sort_index.reserve(data_size);

  for (int64_t i = 0; i < data_size; i++) {
    sort_index.append(std::pair(i, data.sortkey[i]));
  }

  std::sort(sort_index.begin(),
            sort_index.end(),
            [&](const std::pair<int64_t, float> &a, const std::pair<int64_t, float> &b) {
              return (a.second < b.second);
            });

  /* Ignore sortdata for return data. */
  IntersectionData r_data;
  r_data.position.reserve(data_size);

  r_data.pos_index_a.reserve(data_size);
  r_data.pos_index_b.reserve(data_size);
  r_data.lambda.reserve(data_size);

  if (attribute_outputs.hash) {
    r_data.hash.reserve(data_size);
  }
  if (attribute_outputs.curve_index) {
    r_data.curve_index.reserve(data_size);
  }
  if (attribute_outputs.factor) {
    r_data.factor.reserve(data_size);
  }
  if (attribute_outputs.length) {
    r_data.length.reserve(data_size);
  }
  if (attribute_outputs.direction) {
    r_data.direction.reserve(data_size);
  }
  if (attribute_outputs.normal) {
    r_data.normal.reserve(data_size);
  }
  if (attribute_outputs.pair_position) {
    r_data.pair_position.reserve(data_size);
  }
  if (attribute_outputs.pair_direction) {
    r_data.pair_direction.reserve(data_size);
  }
  if (attribute_outputs.pair) {
    r_data.pair.reserve(data_size);
  }
  if (attribute_outputs.pair_id) {
    r_data.pair_id.reserve(data_size);
  }

  /* Dedupe hashed points. */
  std::map<int64_t, int64_t> dupes;
  if (attribute_outputs.hash) {
    for (int64_t i = 0; i < data_size; i++) {
      dupes[data.hash[i]] = i;
    }
  }

  const bool dedupe = attribute_outputs.hash && dupes.size() > 0 && dupes.size() < data_size;
  for (const std::pair key_val : sort_index) {
    const int64_t key_index = key_val.first;

    if (dedupe) {
      const int hash_key = data.hash[key_index];
      if (!(dupes.at(hash_key) == key_index)) {
        continue;
      }
    }

    r_data.position.append(data.position[key_index]);

    r_data.pos_index_a.append(data.pos_index_a[key_index]);
    r_data.pos_index_b.append(data.pos_index_b[key_index]);
    r_data.lambda.append(data.lambda[key_index]);

    if (attribute_outputs.curve_index) {
      r_data.curve_index.append(data.curve_index[key_index]);
    }
    if (attribute_outputs.factor) {
      r_data.factor.append(data.factor[key_index]);
    }
    if (attribute_outputs.length) {
      r_data.length.append(data.length[key_index]);
    }
    if (attribute_outputs.direction) {
      r_data.direction.append(data.direction[key_index]);
    }
    if (attribute_outputs.normal) {
      r_data.normal.append(data.normal[key_index]);
    }
    if (attribute_outputs.pair_position) {
      r_data.pair_position.append(data.pair_position[key_index]);
    }
    if (attribute_outputs.pair_direction) {
      r_data.pair_direction.append(data.pair_direction[key_index]);
    }
    if (attribute_outputs.pair) {
      r_data.pair.append(data.pair[key_index]);
    }
    if (attribute_outputs.pair_id) {
      r_data.pair_id.append(data.pair_id[key_index]);
    }
  }
  BLI_assert(data.position.size() == r_data.position.size());
  dupes.clear();
  return r_data;
}

static void calc_attributes(const AttributeAccessor src_attributes,
                            const AttrDomain src_domain,
                            const AttrDomain dst_domain,
                            const AttributeFilter &attribute_filter,
                            const Span<int> indices_a,
                            const Span<int> indices_b,
                            const Span<float> factors,
                            MutableAttributeAccessor dst_attributes)
{
  src_attributes.foreach_attribute([&](const AttributeIter &iter) {
    if (iter.domain != src_domain) {
      return;
    }
    if (iter.data_type == bke::AttrType::String) {
      return;
    }
    if (attribute_filter.allow_skip(iter.name)) {
      return;
    }

    const GAttributeReader src = iter.get(src_domain);
    GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        iter.name, dst_domain, iter.data_type);
    if (!dst) {
      return;
    }

    bke::attribute_math::gather_mix(src.varray, indices_a, indices_b, factors, dst.span);

    dst.finish();
  });
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const auto mode = params.extract_input<IntersectionMode>("Mode"_ustr);
  const auto pair_data_mode = params.extract_input<PairData>("Paired Data Mode"_ustr);
  const bool curve_mode = ELEM(mode, IntersectionMode::Curve, IntersectionMode::Curve_Project);
  const bool points_only_mode = curve_mode && pair_data_mode == PairData::PointsOnly;

  GeometrySet geometry_set = params.extract_input<GeometrySet>("Curve"_ustr);
  GeometryComponentEditData::remember_deformed_positions_if_necessary(geometry_set);

  lazy_threading::send_hint();

  AttributeOutputs attribute_outputs;

  if (!points_only_mode) {
    attribute_outputs.direction = params.get_output_anonymous_attribute_id_if_needed("Direction"_ustr);
    attribute_outputs.curve_index = params.get_output_anonymous_attribute_id_if_needed(
        "Curve Index"_ustr);
    attribute_outputs.factor = params.get_output_anonymous_attribute_id_if_needed("Factor"_ustr);
    attribute_outputs.length = params.get_output_anonymous_attribute_id_if_needed("Length"_ustr);
  }

  if (mode == IntersectionMode::Surface) {
    attribute_outputs.normal = params.get_output_anonymous_attribute_id_if_needed("Normal"_ustr);
    attribute_outputs.hash = "Hash";
  }

  if (curve_mode && pair_data_mode == PairData::FullPair) {
    attribute_outputs.pair_position = params.get_output_anonymous_attribute_id_if_needed(
        "Pair Position"_ustr);
    attribute_outputs.pair_direction = params.get_output_anonymous_attribute_id_if_needed(
        "Pair Direction"_ustr);
    attribute_outputs.pair = params.get_output_anonymous_attribute_id_if_needed("Pair"_ustr);
    attribute_outputs.pair_id = params.get_output_anonymous_attribute_id_if_needed("Pair ID"_ustr);
  }

  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry_set) {
    if (!geometry_set.has_curves()) {
      geometry_set.clear();
      return;
    }
    const Curves &src_curves_id = *geometry_set.get_curves();
    const bke::CurvesGeometry &src_curves = src_curves_id.geometry.wrap();

    if (src_curves.curves_range().is_empty()) {
      return;
    }

    bke::CurvesGeometry eval_curves = geometry::resample_to_evaluated(
        src_curves,
        bke::CurvesFieldContext(src_curves_id, AttrDomain::Curve),
        fn::Field<bool>(true));

    IntersectionData r_data;

    VArray<int> ids = *eval_curves.attributes().lookup<int>("id", bke::AttrDomain::Curve);
    if (ids.size() == eval_curves.curves_range().size()) {
      attribute_outputs.id = true;
    }
    else {
      attribute_outputs.id = false;
    }

    switch (mode) {
      case IntersectionMode::Curve: {
        const bool self = params.extract_input<bool>("Self Intersections"_ustr);
        const bool all = params.extract_input<bool>("All Intersections"_ustr);
        if (!self && !all) {
          geometry_set.clear();
          return;
        }
        const Field<float> distance_field = params.extract_input<Field<float>>("Distance"_ustr);
        const float min_angle = params.extract_input<float>("Min Angle"_ustr);
        const float max_angle = params.extract_input<float>("Max Angle"_ustr);
        set_curve_intersections(
            eval_curves,
            distance_field,
            ids,
            self,
            all,
            math::clamp(float2(min_angle, max_angle), min_angle_eps, pi_2_f_eps),
            false,
            float3(0.0f),
            pair_data_mode,
            attribute_outputs,
            r_data);
        break;
      }
      case IntersectionMode::Curve_Project: {
        const bool self = params.extract_input<bool>("Self Intersections"_ustr);
        const bool all = params.extract_input<bool>("All Intersections"_ustr);
        if (!self && !all) {
          geometry_set.clear();
          return;
        }
        const Field<float> distance_field = params.extract_input<Field<float>>("Distance"_ustr);
        const float3 direction = params.extract_input<float3>("Direction"_ustr);
        const float min_angle = params.extract_input<float>("Min Angle"_ustr);
        const float max_angle = params.extract_input<float>("Max Angle"_ustr);
        set_curve_intersections(
            eval_curves,
            distance_field,
            ids,
            self,
            all,
            math::clamp(float2(min_angle, max_angle), min_angle_eps, pi_2_f_eps),
            true,
            direction,
            pair_data_mode,
            attribute_outputs,
            r_data);
        break;
      }
      case IntersectionMode::Plane: {
        const float3 direction = params.extract_input<float3>("Direction"_ustr);
        const float3 plane_center = params.extract_input<float3>("Center"_ustr);
        const float min_angle = params.extract_input<float>("Min Angle"_ustr);
        const float max_angle = params.extract_input<float>("Max Angle"_ustr);
        set_curve_intersections_plane(
            eval_curves,
            plane_center,
            math::normalize(direction),
            math::clamp(float2(min_angle, max_angle), min_angle_eps, pi_2_f_eps),
            attribute_outputs,
            r_data);
        break;
      }
      case IntersectionMode::Surface: {
        GeometrySet mesh_set = params.extract_input<GeometrySet>("Mesh"_ustr);
        const float min_angle = params.extract_input<float>("Min Angle"_ustr);
        const float max_angle = params.extract_input<float>("Max Angle"_ustr);
        if (mesh_set.has_mesh()) {
          set_curve_intersections_mesh(
              mesh_set,
              eval_curves,
              ids,
              math::clamp(float2(min_angle, max_angle), min_angle_eps, pi_2_f_eps),
              attribute_outputs,
              r_data);
        }
        else {
          geometry_set.clear();
          return;
        }
        break;
      }
      default: {
        BLI_assert_unreachable();
        break;
      }
    }

    /* Gather and sort data for attributes. */
    if (r_data.position.size() > 0) {

      IntersectionData sorted_data = sort_intersection_data(r_data, attribute_outputs);

      PointCloud *pointcloud = BKE_pointcloud_new_nomain(PointCloudType::Points, sorted_data.position.size());
      MutableAttributeAccessor attributes = pointcloud->attributes_for_write();

      /* Builtin attributes. */
      SpanAttributeWriter<float3> point_positions =
          attributes.lookup_or_add_for_write_only_span<float3>("position", AttrDomain::Point);
      point_positions.span.copy_from(sorted_data.position);
      point_positions.finish();

      /* Gather and copy attributes. */
      calc_attributes(eval_curves.attributes(),
                      AttrDomain::Point,
                      AttrDomain::Point,
                      bke::attribute_filter_from_skip_ref({"position",
                                                           "radius",
                                                           "handle_left",
                                                           "handle_right",
                                                           "handle_type_left",
                                                           "handle_type_right",
                                                           "nurbs_weight",
                                                           ".selection",
                                                           ".selection_handle_left",
                                                           ".selection_handle_right"}),
                      sorted_data.pos_index_a.as_span(),
                      sorted_data.pos_index_b.as_span(),
                      sorted_data.lambda.as_span(),
                      attributes);

      /* Point cloud display radius (same default as Distribute Points). */
      SpanAttributeWriter<float> point_radii =
          attributes.lookup_or_add_for_write_only_span<float>("radius", AttrDomain::Point);
      point_radii.span.fill(0.05f);
      point_radii.finish();

      /* Output attributes. */
      if (attribute_outputs.curve_index) {
        SpanAttributeWriter<int> curve_index = attributes.lookup_or_add_for_write_only_span<int>(
            *attribute_outputs.curve_index, AttrDomain::Point);
        curve_index.span.copy_from(sorted_data.curve_index);
        curve_index.finish();
      }

      if (attribute_outputs.direction) {
        SpanAttributeWriter<float3> directions =
            attributes.lookup_or_add_for_write_only_span<float3>(*attribute_outputs.direction,
                                                                 AttrDomain::Point);
        directions.span.copy_from(sorted_data.direction);
        directions.finish();
      }

      if (attribute_outputs.normal) {
        SpanAttributeWriter<float3> normal = attributes.lookup_or_add_for_write_only_span<float3>(
            *attribute_outputs.normal, AttrDomain::Point);
        normal.span.copy_from(sorted_data.normal);
        normal.finish();
      }

      if (attribute_outputs.factor) {
        SpanAttributeWriter<float> factor = attributes.lookup_or_add_for_write_only_span<float>(
            *attribute_outputs.factor, AttrDomain::Point);
        factor.span.copy_from(sorted_data.factor);
        factor.finish();
      }

      if (attribute_outputs.length) {
        SpanAttributeWriter<float> length = attributes.lookup_or_add_for_write_only_span<float>(
            *attribute_outputs.length, AttrDomain::Point);
        length.span.copy_from(sorted_data.length);
        length.finish();
      }

      if (attribute_outputs.pair_position && pair_data_mode == PairData::FullPair) {
        SpanAttributeWriter<float3> pair_position =
            attributes.lookup_or_add_for_write_only_span<float3>(*attribute_outputs.pair_position,
                                                                 AttrDomain::Point);
        pair_position.span.copy_from(sorted_data.pair_position);
        pair_position.finish();
      }

      if (attribute_outputs.pair_direction && pair_data_mode == PairData::FullPair) {
        SpanAttributeWriter<float3> pair_direction =
            attributes.lookup_or_add_for_write_only_span<float3>(*attribute_outputs.pair_direction,
                                                                 AttrDomain::Point);
        pair_direction.span.copy_from(sorted_data.pair_direction);
        pair_direction.finish();
      }

      if (attribute_outputs.pair && pair_data_mode == PairData::FullPair) {
        SpanAttributeWriter<bool> pair = attributes.lookup_or_add_for_write_only_span<bool>(
            *attribute_outputs.pair, AttrDomain::Point);
        pair.span.copy_from(sorted_data.pair);
        pair.finish();
      }

      if (attribute_outputs.pair_id && pair_data_mode == PairData::FullPair) {
        SpanAttributeWriter<int> pair_id = attributes.lookup_or_add_for_write_only_span<int>(
            *attribute_outputs.pair_id, AttrDomain::Point);
        pair_id.span.copy_from(sorted_data.pair_id);
        pair_id.finish();
      }

      geometry_set.clear();
      geometry_set.replace_pointcloud(pointcloud);
    }
    else {
      geometry_set.clear();
    }
  });

  params.set_output("Points"_ustr, std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeCurveIntersections"_ustr);
  ntype.ui_name = "Curve Intersections";
  ntype.ui_description = "Calculate and output curve intersections as a point cloud";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_curve_intersection_cc
