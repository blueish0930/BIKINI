/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Discrete optimal transport (Monge / rectangular assignment) between two
 * point sets. Source geometry is passed through unchanged; each source point
 * gets an anonymous integer "Target Index" into the target cloud. Unmatched
 * source points (when |Source| > |Target|) are -1.
 */

#include <cmath>
#include <limits>
#include <vector>

#include "BLI_array.hh"
#include "BLI_math_vector.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_optimal_transport_cc {

/** Hard cap: Hungarian is O(n^2 m). 1500^3 is already a few seconds. */
static constexpr int k_max_points = 1500;

static Span<float3> pts_of(const GeometrySet &g)
{
  if (const PointCloud *pc = g.get_pointcloud()) {
    return pc->positions();
  }
  if (const Mesh *m = g.get_mesh()) {
    return m->vert_positions();
  }
  return {};
}

/**
 * Rectangular min-cost assignment (Kuhn–Munkres / Hungarian with potentials).
 * \a cost is n x m row-major. Writes \a row_to_col of size n: column index or -1.
 * Matches exactly min(n, m) pairs; leftover rows stay -1.
 */
static void linear_sum_assignment(const double *cost, const int n, const int m, int *row_to_col)
{
  for (int i = 0; i < n; i++) {
    row_to_col[i] = -1;
  }
  if (n <= 0 || m <= 0) {
    return;
  }

  const bool transpose = n > m;
  const int N = transpose ? m : n;
  const int M = transpose ? n : m;
  const int cols = M + 1;

  std::vector<double> a(size_t(N + 1) * size_t(cols), 0.0);
  auto at = [&](const int i, const int j) -> double & {
    return a[size_t(i) * size_t(cols) + size_t(j)];
  };
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < M; j++) {
      const double c = transpose ? cost[size_t(j) * size_t(m) + size_t(i)] :
                                   cost[size_t(i) * size_t(m) + size_t(j)];
      at(i + 1, j + 1) = std::isfinite(c) ? c : 1.0e30;
    }
  }

  std::vector<double> u(size_t(N + 1), 0.0);
  std::vector<double> v(size_t(M + 1), 0.0);
  std::vector<int> p(size_t(M + 1), 0);
  std::vector<int> way(size_t(M + 1), 0);

  for (int i = 1; i <= N; i++) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(size_t(M + 1), std::numeric_limits<double>::infinity());
    std::vector<char> used(size_t(M + 1), 0);
    do {
      used[size_t(j0)] = 1;
      const int i0 = p[size_t(j0)];
      int j1 = 0;
      double delta = std::numeric_limits<double>::infinity();
      for (int j = 1; j <= M; j++) {
        if (used[size_t(j)]) {
          continue;
        }
        const double cur = at(i0, j) - u[size_t(i0)] - v[size_t(j)];
        if (cur < minv[size_t(j)]) {
          minv[size_t(j)] = cur;
          way[size_t(j)] = j0;
        }
        if (minv[size_t(j)] < delta) {
          delta = minv[size_t(j)];
          j1 = j;
        }
      }
      for (int j = 0; j <= M; j++) {
        if (used[size_t(j)]) {
          u[size_t(p[size_t(j)])] += delta;
          v[size_t(j)] -= delta;
        }
        else {
          minv[size_t(j)] -= delta;
        }
      }
      j0 = j1;
    } while (p[size_t(j0)] != 0);
    do {
      const int j1 = way[size_t(j0)];
      p[size_t(j0)] = p[size_t(j1)];
      j0 = j1;
    } while (j0);
  }

  for (int j = 1; j <= M; j++) {
    if (p[size_t(j)] == 0) {
      continue;
    }
    const int r = p[size_t(j)] - 1;
    const int c = j - 1;
    if (transpose) {
      row_to_col[c] = r;
    }
    else {
      row_to_col[r] = c;
    }
  }
}

static void write_target_index(GeometrySet &geometry, const StringRef attr_id, const Span<int> indices)
{
  if (PointCloud *pc = geometry.get_pointcloud_for_write()) {
    if (pc->totpoint != indices.size()) {
      return;
    }
    bke::MutableAttributeAccessor attrs = pc->attributes_for_write();
    bke::SpanAttributeWriter<int> writer = attrs.lookup_or_add_for_write_only_span<int>(
        attr_id, bke::AttrDomain::Point);
    writer.span.copy_from(indices);
    writer.finish();
    return;
  }
  if (Mesh *mesh = geometry.get_mesh_for_write()) {
    if (mesh->verts_num != indices.size()) {
      return;
    }
    bke::MutableAttributeAccessor attrs = mesh->attributes_for_write();
    bke::SpanAttributeWriter<int> writer = attrs.lookup_or_add_for_write_only_span<int>(
        attr_id, bke::AttrDomain::Point);
    writer.span.copy_from(indices);
    writer.finish();
  }
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Source"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Point cloud (or mesh verts) to match from. Geometry is passed through unchanged.");
  b.add_output<decl::Geometry>("Source"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Same Source geometry, with Target Index stored as an anonymous point attribute.");
  b.add_output<decl::Int>("Target Index"_ustr)
      .anonymous_attribute_output()
      .description(
          "Index of the uniquely matched target point. -1 if this source point was left unmatched "
          "(when Source has more points than Target).");
  b.add_input<decl::Geometry>("Target"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Point cloud (or mesh verts) to match onto. Not modified.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet source = params.extract_input<GeometrySet>("Source"_ustr);
  GeometrySet target = params.extract_input<GeometrySet>("Target"_ustr);

  if (!params.output_is_required("Source"_ustr)) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("The Target Index output cannot be used without the Source geometry output"));
    params.set_default_remaining_outputs();
    return;
  }

  const std::optional<std::string> index_id = params.get_output_anonymous_attribute_id_if_needed(
      "Target Index"_ustr);
  if (!index_id) {
    params.set_output("Source"_ustr, std::move(source));
    return;
  }

  const Span<float3> src = pts_of(source);
  const Span<float3> tgt = pts_of(target);
  const int n = int(src.size());
  const int m = int(tgt.size());

  Array<int> assignment(n, -1);
  if (n == 0) {
    params.set_output("Source"_ustr, std::move(source));
    return;
  }
  if (m == 0) {
    write_target_index(source, *index_id, assignment);
    params.set_output("Source"_ustr, std::move(source));
    return;
  }
  if (n > k_max_points || m > k_max_points) {
    params.error_message_add(
        NodeWarningType::Error,
        TIP_("Optimal Transport exact matching is limited to 1500 points per cloud"));
    write_target_index(source, *index_id, assignment);
    params.set_output("Source"_ustr, std::move(source));
    return;
  }
  if (std::max(n, m) > 500) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("Optimal Transport is O(n³); large clouds may stall the node tree"));
  }

  Array<double> cost(size_t(n) * size_t(m));
  threading::parallel_for(IndexRange(n), 32, [&](const IndexRange range) {
    for (const int i : range) {
      const float3 p = src[i];
      double *row = cost.data() + size_t(i) * size_t(m);
      for (int j = 0; j < m; j++) {
        row[j] = double(math::distance_squared(p, tgt[j]));
      }
    }
  });

  linear_sum_assignment(cost.data(), n, m, assignment.data());
  write_target_index(source, *index_id, assignment);
  params.set_output("Source"_ustr, std::move(source));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalOptimalTransport"_ustr, GEO_NODE_CGAL_OPTIMAL_TRANSPORT);
  ntype.ui_name = "Optimal Transport";
  ntype.ui_description =
      "One-to-one match Source points onto Target by min total squared distance (discrete "
      "optimal transport / assignment). Outputs the original Source plus an anonymous Target "
      "Index attribute. Extra source points get -1 when Source has more points than Target.";
  ntype.enum_name_legacy = "CGAL_OPTIMAL_TRANSPORT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_optimal_transport_cc
