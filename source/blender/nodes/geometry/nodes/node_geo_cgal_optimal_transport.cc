/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Discrete assignment between an evaluation context (people) and a target
 * point set (tasks). Sample Position is read on the context domain. Each
 * person gets at most one task. The output is that task's index, or -1 when
 * there are more people than tasks and this person is left unmatched.
 *
 * Two solvers, chosen by the Algorithm menu:
 * - Jonker–Volgenant: shortest augmenting path with dual potentials. This is
 *   the O(n³) Kuhn–Munkres method, not the Wikipedia matrix steps.
 * - Hungarian Matrix: Wikipedia "Matrix interpretation" (Flood / Munkres)
 *   on the raw people × tasks matrix. No square padding. Starred zeros stop
 *   at min(people, tasks). Only the side that is fully matched is reduced
 *   up front (both sides when the matrix is square): reducing an unused
 *   row or column would change which assignment is optimal.
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "BLI_array.hh"
#include "BLI_math_vector.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"

#include "BKE_geometry_fields.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_optimal_transport_cc {

/** Jonker–Volgenant is O(n²m). 1500³ is already a few seconds. */
static constexpr int k_max_points = 1500;
/**
 * Each matrix adjustment scans the whole m×n matrix. Above this many matched
 * pairs (min(people, tasks)) the node writes -1 instead of stalling.
 * A long thin matrix is fine: the matched side is the short one.
 */
static constexpr int k_max_matrix_matched = 400;

enum class AlgorithmMode : int8_t {
  JonkerVolgenant = 0,
  HungarianMatrix = 1,
};

static const EnumPropertyItem algorithm_items[] = {
    {int(AlgorithmMode::JonkerVolgenant),
     "JONKER_VOLGENANT",
     0,
     N_("Jonker–Volgenant"),
     N_("Shortest augmenting path with dual potentials (Kuhn–Munkres, O(n³))")},
    {int(AlgorithmMode::HungarianMatrix),
     "HUNGARIAN_MATRIX",
     0,
     N_("Hungarian (Matrix)"),
     N_("Wikipedia matrix interpretation (Flood / Munkres) on the m×n cost matrix. "
        "Stops when min(people, tasks) zeros are starred. No square padding")},
    {0, nullptr, 0, nullptr, nullptr},
};

static Array<float3> target_positions_of(const GeometrySet &geometry)
{
  if (const PointCloud *pointcloud = geometry.get_pointcloud()) {
    if (pointcloud->totpoint > 0) {
      return Array<float3>(pointcloud->positions());
    }
  }
  if (const Mesh *mesh = geometry.get_mesh()) {
    if (mesh->verts_num > 0) {
      return Array<float3>(mesh->vert_positions());
    }
  }
  return {};
}

/**
 * Rectangular min-cost assignment (Jonker–Volgenant).
 * \a cost is people × tasks, row-major. Writes \a row_to_col of size people:
 * task index, or -1. Matches exactly min(people, tasks) pairs.
 */
static void linear_sum_assignment(const double *cost,
                                  const int people,
                                  const int tasks,
                                  int *row_to_col)
{
  for (int i = 0; i < people; i++) {
    row_to_col[i] = -1;
  }
  if (people <= 0 || tasks <= 0) {
    return;
  }

  const bool transpose = people > tasks;
  const int rows = transpose ? tasks : people;
  const int cols_n = transpose ? people : tasks;
  const int cols = cols_n + 1;

  std::vector<double> a(size_t(rows + 1) * size_t(cols), 0.0);
  auto at = [&](const int i, const int j) -> double & {
    return a[size_t(i) * size_t(cols) + size_t(j)];
  };
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols_n; j++) {
      const double c = transpose ? cost[size_t(j) * size_t(tasks) + size_t(i)] :
                                   cost[size_t(i) * size_t(tasks) + size_t(j)];
      at(i + 1, j + 1) = std::isfinite(c) ? c : 1.0e30;
    }
  }

  std::vector<double> u(size_t(rows + 1), 0.0);
  std::vector<double> v(size_t(cols_n + 1), 0.0);
  std::vector<int> p(size_t(cols_n + 1), 0);
  std::vector<int> way(size_t(cols_n + 1), 0);

  for (int i = 1; i <= rows; i++) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(size_t(cols_n + 1), std::numeric_limits<double>::infinity());
    std::vector<char> used(size_t(cols_n + 1), 0);
    do {
      used[size_t(j0)] = 1;
      const int i0 = p[size_t(j0)];
      int j1 = 0;
      double delta = std::numeric_limits<double>::infinity();
      for (int j = 1; j <= cols_n; j++) {
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
      for (int j = 0; j <= cols_n; j++) {
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

  for (int j = 1; j <= cols_n; j++) {
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

/**
 * Wikipedia matrix interpretation (Flood / Munkres) on an m×n cost matrix.
 * Rows are people, columns are tasks. Stops at min(people, tasks) starred
 * zeros; leftover people stay -1. Does not pad to a square.
 *
 * Step 1 / step 2 only reduce a side that every feasible matching uses.
 * Subtracting a minimum from an unused row or column changes the original
 * cost of different matchings by different amounts, so a zero assignment
 * would no longer be optimal. Square matrices reduce both sides.
 *
 * Returns false if the starred-zero search cannot prove optimality; the
 * caller then uses Jonker–Volgenant.
 */
static bool hungarian_matrix(const double *cost, const int people, const int tasks, int *row_to_col)
{
  for (int i = 0; i < people; i++) {
    row_to_col[i] = -1;
  }
  if (people <= 0 || tasks <= 0) {
    return true;
  }

  std::vector<double> matrix(size_t(people) * size_t(tasks), 0.0);
  auto at = [&](const int i, const int j) -> double & {
    return matrix[size_t(i) * size_t(tasks) + size_t(j)];
  };
  for (int i = 0; i < people; i++) {
    for (int j = 0; j < tasks; j++) {
      const double c = cost[size_t(i) * size_t(tasks) + size_t(j)];
      at(i, j) = std::isfinite(c) ? c : 1.0e30;
    }
  }

  /* People are fully matched when people <= tasks, so row reduction is valid. */
  if (people <= tasks) {
    for (int i = 0; i < people; i++) {
      double row_min = at(i, 0);
      for (int j = 1; j < tasks; j++) {
        row_min = std::min(row_min, at(i, j));
      }
      for (int j = 0; j < tasks; j++) {
        at(i, j) -= row_min;
      }
    }
  }
  /* Tasks are fully matched when tasks <= people. Square does both steps. */
  if (tasks <= people) {
    for (int j = 0; j < tasks; j++) {
      double col_min = at(0, j);
      for (int i = 1; i < people; i++) {
        col_min = std::min(col_min, at(i, j));
      }
      for (int i = 0; i < people; i++) {
        at(i, j) -= col_min;
      }
    }
  }

  /* Step 3: star a greedy set of independent zeros. */
  std::vector<int> star_col(size_t(people), -1);
  std::vector<int> star_row(size_t(tasks), -1);
  for (int i = 0; i < people; i++) {
    for (int j = 0; j < tasks; j++) {
      if (at(i, j) <= 0.0 && star_row[size_t(j)] < 0 && star_col[size_t(i)] < 0) {
        star_col[size_t(i)] = j;
        star_row[size_t(j)] = i;
        break;
      }
    }
  }

  std::vector<int> prime_col(size_t(people), -1);
  std::vector<char> row_covered(size_t(people), 0);
  std::vector<char> col_covered(size_t(tasks), 0);

  auto cover_starred_columns = [&]() {
    std::fill(row_covered.begin(), row_covered.end(), 0);
    for (int j = 0; j < tasks; j++) {
      col_covered[size_t(j)] = star_row[size_t(j)] >= 0 ? 1 : 0;
    }
  };
  auto starred_count = [&]() {
    int count = 0;
    for (int j = 0; j < tasks; j++) {
      if (star_row[size_t(j)] >= 0) {
        count++;
      }
    }
    return count;
  };

  cover_starred_columns();
  const int matched = std::min(people, tasks);
  const int64_t guard_limit = int64_t(people + tasks + 2) * int64_t(people + tasks + 2) *
                              int64_t(people + tasks + 2);
  int64_t guard = 0;
  while (starred_count() < matched) {
    if (++guard > guard_limit) {
      return false;
    }

    /* Step 4: prime an uncovered zero, or cover its row and uncover the star. */
    int found_row = -1;
    int found_col = -1;
    for (int i = 0; i < people && found_row < 0; i++) {
      if (row_covered[size_t(i)]) {
        continue;
      }
      for (int j = 0; j < tasks; j++) {
        if (col_covered[size_t(j)]) {
          continue;
        }
        if (at(i, j) <= 0.0) {
          found_row = i;
          found_col = j;
          break;
        }
      }
    }

    if (found_row < 0) {
      /* Step 5: subtract the lowest uncovered value; add it on double-covered cells. */
      double delta = std::numeric_limits<double>::infinity();
      for (int i = 0; i < people; i++) {
        if (row_covered[size_t(i)]) {
          continue;
        }
        for (int j = 0; j < tasks; j++) {
          if (col_covered[size_t(j)]) {
            continue;
          }
          delta = std::min(delta, at(i, j));
        }
      }
      if (!(delta > 0.0) || !std::isfinite(delta)) {
        return false;
      }
      for (int i = 0; i < people; i++) {
        for (int j = 0; j < tasks; j++) {
          const bool row_c = row_covered[size_t(i)] != 0;
          const bool col_c = col_covered[size_t(j)] != 0;
          if (!row_c && !col_c) {
            at(i, j) -= delta;
          }
          else if (row_c && col_c) {
            at(i, j) += delta;
          }
        }
      }
      continue;
    }

    prime_col[size_t(found_row)] = found_col;
    if (star_col[size_t(found_row)] >= 0) {
      row_covered[size_t(found_row)] = 1;
      col_covered[size_t(star_col[size_t(found_row)])] = 0;
      continue;
    }

    /* Alternating path: primed, starred, primed, ... Unstar first, then star. */
    std::vector<int> path_row;
    std::vector<int> path_col;
    path_row.push_back(found_row);
    path_col.push_back(found_col);
    int column = found_col;
    while (true) {
      const int starred_at = star_row[size_t(column)];
      if (starred_at < 0) {
        break;
      }
      const int primed = prime_col[size_t(starred_at)];
      if (primed < 0) {
        return false;
      }
      path_row.push_back(starred_at);
      path_col.push_back(column);
      path_row.push_back(starred_at);
      path_col.push_back(primed);
      column = primed;
    }
    for (int k = 1; k < int(path_row.size()); k += 2) {
      const int row = path_row[size_t(k)];
      const int col = path_col[size_t(k)];
      star_row[size_t(col)] = -1;
      star_col[size_t(row)] = -1;
    }
    for (int k = 0; k < int(path_row.size()); k += 2) {
      const int row = path_row[size_t(k)];
      const int col = path_col[size_t(k)];
      star_row[size_t(col)] = row;
      star_col[size_t(row)] = col;
    }
    std::fill(prime_col.begin(), prime_col.end(), -1);
    cover_starred_columns();
  }

  for (int i = 0; i < people; i++) {
    const int task = star_col[size_t(i)];
    row_to_col[i] = (task >= 0 && task < tasks) ? task : -1;
  }
  return true;
}

static void build_squared_cost(const Span<float3> people,
                               const Span<float3> tasks,
                               Array<double> &r_cost)
{
  const int people_num = int(people.size());
  const int tasks_num = int(tasks.size());
  r_cost.reinitialize(size_t(people_num) * size_t(tasks_num));
  threading::parallel_for(IndexRange(people_num), 32, [&](const IndexRange range) {
    for (const int person : range) {
      const float3 position = people[person];
      double *row = r_cost.data() + size_t(person) * size_t(tasks_num);
      for (int task = 0; task < tasks_num; task++) {
        const float3 target = tasks[task];
        const double dx = double(position.x) - double(target.x);
        const double dy = double(position.y) - double(target.y);
        const double dz = double(position.z) - double(target.z);
        const double distance_sq = dx * dx + dy * dy + dz * dz;
        row[task] = std::isfinite(distance_sq) ? distance_sq : 1.0e30;
      }
    }
  });
}

static void assign_people(const Span<float3> people,
                          const Span<float3> tasks,
                          const AlgorithmMode algorithm,
                          MutableSpan<int> r_task_index)
{
  const int people_num = int(people.size());
  const int tasks_num = int(tasks.size());
  BLI_assert(r_task_index.size() == people_num);
  r_task_index.fill(-1);
  if (people_num == 0 || tasks_num == 0) {
    return;
  }

  Array<double> cost;
  build_squared_cost(people, tasks, cost);

  if (algorithm == AlgorithmMode::HungarianMatrix) {
    if (hungarian_matrix(cost.data(), people_num, tasks_num, r_task_index.data())) {
      return;
    }
  }
  linear_sum_assignment(cost.data(), people_num, tasks_num, r_task_index.data());
}

class OptimalTransportFieldInput final : public bke::GeometryFieldInput {
 private:
  Array<float3> tasks_;
  Field<float3> sample_position_;
  AlgorithmMode algorithm_;

 public:
  OptimalTransportFieldInput(Array<float3> tasks,
                             Field<float3> sample_position,
                             const AlgorithmMode algorithm)
      : bke::GeometryFieldInput(CPPType::get<int>(), "Optimal Transport"),
        tasks_(std::move(tasks)),
        sample_position_(std::move(sample_position)),
        algorithm_(algorithm)
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask & /*mask*/) const final
  {
    const std::optional<bke::AttributeAccessor> attributes = context.attributes();
    if (!attributes) {
      return {};
    }
    const int64_t domain_size = attributes->domain_size(context.domain());
    if (domain_size <= 0) {
      return {};
    }

    const int people_num = int(domain_size);
    const int tasks_num = int(tasks_.size());
    const bool matrix_too_big = algorithm_ == AlgorithmMode::HungarianMatrix &&
                                std::min(people_num, tasks_num) > k_max_matrix_matched;
    if (people_num > k_max_points || tasks_num > k_max_points || matrix_too_big) {
      Array<int> unmatched(domain_size, -1);
      return VArray<int>::from_container(std::move(unmatched));
    }

    fn::FieldEvaluator evaluator{context, domain_size};
    evaluator.add(sample_position_);
    evaluator.evaluate();
    const VArray<float3> evaluated = evaluator.get_evaluated<float3>(0);
    Array<float3> people(domain_size);
    evaluated.materialize(people.as_mutable_span());

    Array<int> assignment(domain_size, -1);
    assign_people(people, tasks_, algorithm_, assignment);
    return VArray<int>::from_container(std::move(assignment));
  }

  void foreach_recursive_field(FunctionRef<void(const GField &)> fn) const override
  {
    fn(sample_position_);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep &deep_hash_cache) const override
  {
    static constexpr int8_t id = 17;
    hash.add(&id);
    hash.add(algorithm_);
    hash.add(tasks_.size());
    hash.add(tasks_.data());
    hash.add(deep_hash_cache.ensure(sample_position_));
  }
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Target"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description(
          "Tasks. Each point-cloud point or mesh vertex can be assigned to at most one person");

  b.add_input<decl::Vector>("Sample Position"_ustr)
      .default_input_type(NODE_DEFAULT_INPUT_POSITION_FIELD)
      .structure_type(StructureType::Field)
      .description(
          "Position of each person. Evaluated on the context domain. Defaults to Position, so the "
          "number of people is the context size");

  b.add_input<decl::Menu>("Algorithm"_ustr)
      .static_items(algorithm_items)
      .default_value(AlgorithmMode::JonkerVolgenant)
      .optional_label()
      .description("Solver for the minimum total squared-distance assignment");

  b.add_output<decl::Int>("Target Index"_ustr)
      .structure_type(StructureType::Field)
      .propagate_references()
      .description(
          "Index of the unique task assigned to this person. -1 when there are more people than "
          "tasks and this person is left unmatched");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet target = params.extract_input<GeometrySet>("Target"_ustr);
  Field<float3> sample_position = params.extract_input<Field<float3>>("Sample Position"_ustr);
  const AlgorithmMode algorithm = params.extract_input<AlgorithmMode>("Algorithm"_ustr);

  Array<float3> tasks = target_positions_of(target);
  const int tasks_num = int(tasks.size());
  if (tasks_num == 0 && !target.is_empty()) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Target has no point cloud or mesh vertices"));
  }

  if (tasks_num > k_max_points) {
    params.error_message_add(
        NodeWarningType::Error,
        TIP_("Optimal Transport exact matching is limited to 1500 points per cloud"));
    params.set_output("Target Index"_ustr, Field<int>(-1));
    return;
  }
  if (algorithm == AlgorithmMode::JonkerVolgenant && tasks_num > 500) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("Optimal Transport is O(n³); large clouds may stall the node tree"));
  }
  else if (algorithm == AlgorithmMode::HungarianMatrix && tasks_num > k_max_matrix_matched) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("Hungarian Matrix runs on the m×n matrix. If the context also has more than 400 "
             "elements, it writes -1. Use Jonker–Volgenant for two large sets."));
  }

  params.set_output("Target Index"_ustr,
                    Field<int>::from_input<OptimalTransportFieldInput>(
                        std::move(tasks), std::move(sample_position), algorithm));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalOptimalTransport"_ustr, GEO_NODE_CGAL_OPTIMAL_TRANSPORT);
  ntype.ui_name = "Optimal Transport";
  ntype.ui_description =
      "Assign each evaluated element (a person) to a unique target point (a task) by minimum "
      "total squared distance. Sample Position is read from the evaluation context. When there "
      "are more people than tasks, unmatched people get target index -1.";
  ntype.enum_name_legacy = "CGAL_OPTIMAL_TRANSPORT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_optimal_transport_cc
