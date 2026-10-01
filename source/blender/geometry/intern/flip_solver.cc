/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_flip_solver.hh"

#include "flip_solver_private.hh"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>

#include "BLI_array.hh"
#include "BLI_index_range.hh"
#include "BLI_math_vector.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"
#include "BLI_timeit.hh"
#include "BLI_vector.hh"

namespace blender::geometry {

namespace {

constexpr int64_t max_grid_cells = 16'000'000;
constexpr float weight_epsilon = 1.0e-12f;
constexpr float solver_epsilon = 1.0e-20f;

class ScopedPerformanceTimer {
 private:
  double *milliseconds_;
  timeit::TimePoint start_;

 public:
  explicit ScopedPerformanceTimer(double *milliseconds)
      : milliseconds_(milliseconds), start_(timeit::Clock::now())
  {
  }

  ~ScopedPerformanceTimer()
  {
    if (milliseconds_ != nullptr) {
      const std::chrono::duration<double, std::milli> elapsed = timeit::Clock::now() - start_;
      *milliseconds_ += elapsed.count();
    }
  }
};

static uint32_t hash_u32(uint32_t value)
{
  value ^= value >> 16;
  value *= 0x7feb352dU;
  value ^= value >> 15;
  value *= 0x846ca68bU;
  value ^= value >> 16;
  return value;
}

static float hash_unit(const uint32_t a, const uint32_t b)
{
  return float(hash_u32(a ^ hash_u32(b + 0x9e3779b9U)) & 0x00ffffffU) / float(0x01000000U);
}

struct PcgResult {
  int iterations = 0;
  double initial_residual = 0.0;
  double final_residual = 0.0;
  double relative_residual = 0.0;
  bool converged = false;
  bool breakdown = false;
};

struct PressureStencil {
  int xm = -1;
  int xp = -1;
  int ym = -1;
  int yp = -1;
  int zm = -1;
  int zp = -1;
  float diagonal = 0.0f;
};

/** One viscosity unknown. Off-diagonal slots stay unused when `rows[slot] < 0`. */
struct ViscosityStencil {
  int rows[6] = {-1, -1, -1, -1, -1, -1};
  float coeffs[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float diagonal = 1.0f;
};

struct LinearSolverScratch {
  Array<float> r;
  Array<float> z;
  Array<float> d;
  Array<float> q;
  Array<float> rhs;
  Array<float> solution;
  Array<float> inverse_diagonal;
  Array<uint8_t> gauges;
  Array<PressureStencil> pressure_stencils;
  Array<ViscosityStencil> viscosity_stencils;

  void resize_pcg(const int64_t size)
  {
    if (r.size() != size) {
      r.reinitialize(size);
      z.reinitialize(size);
      d.reinitialize(size);
      q.reinitialize(size);
    }
  }

  void resize_system(const int64_t size)
  {
    if (rhs.size() != size) {
      rhs.reinitialize(size);
      solution.reinitialize(size);
      inverse_diagonal.reinitialize(size);
      gauges.reinitialize(size);
      pressure_stencils.reinitialize(size);
    }
  }
};

static double dot_product(const Span<float> a, const Span<float> b)
{
  return threading::parallel_reduce(
      a.index_range(),
      4096,
      0.0,
      [&](const IndexRange range, const double init) {
        double result = init;
        for (const int i : range) {
          result += double(a[i]) * double(b[i]);
        }
        return result;
      },
      std::plus<double>());
}

template<typename ApplyOperator>
static PcgResult pcg_solve(const Span<float> b,
                           MutableSpan<float> x,
                           const Span<float> inverse_diagonal,
                           ApplyOperator &&apply_A,
                           const int max_iterations,
                           const float tolerance,
                           LinearSolverScratch &scratch,
                           const std::function<void(Span<float>, MutableSpan<float>)> &apply_custom_M = {})
{
  PcgResult result;
  if (b.is_empty()) {
    result.converged = true;
    return result;
  }
  scratch.resize_pcg(b.size());
  MutableSpan<float> r = scratch.r.as_mutable_span();
  MutableSpan<float> z = scratch.z.as_mutable_span();
  MutableSpan<float> d = scratch.d.as_mutable_span();
  MutableSpan<float> q = scratch.q.as_mutable_span();

  apply_A(x, q);
  threading::parallel_for(b.index_range(), 4096, [&](const IndexRange range) {
    for (const int i : range) {
      r[i] = b[i] - q[i];
      if (!apply_custom_M) {
        z[i] = r[i] * inverse_diagonal[i];
        d[i] = z[i];
      }
    }
  });
  if (apply_custom_M) {
    apply_custom_M(r, z);
    d.copy_from(z);
  }
  const double b_norm = std::sqrt(dot_product(b, b));
  result.initial_residual = std::sqrt(dot_product(r, r));
  result.final_residual = result.initial_residual;
  result.relative_residual = result.final_residual / std::max(b_norm, double(solver_epsilon));
  if (!std::isfinite(result.relative_residual)) {
    result.breakdown = true;
    return result;
  }
  if (result.relative_residual <= tolerance || result.final_residual <= solver_epsilon) {
    result.converged = true;
    return result;
  }

  double rz = dot_product(r, z);
  for (const int iteration : IndexRange(max_iterations)) {
    apply_A(d, q);
    const double denominator = dot_product(d, q);
    if (!std::isfinite(denominator) || std::abs(denominator) <= solver_epsilon ||
        !std::isfinite(rz))
    {
      result.breakdown = true;
      return result;
    }
    const double alpha = rz / denominator;
    const double residual_sq = threading::parallel_reduce(
        x.index_range(),
        4096,
        0.0,
        [&](const IndexRange range, const double init) {
          double sum = init;
          for (const int i : range) {
            x[i] += float(alpha) * d[i];
            r[i] -= float(alpha) * q[i];
            sum += double(r[i]) * double(r[i]);
          }
          return sum;
        },
        std::plus<double>());
    result.iterations = iteration + 1;
    result.final_residual = std::sqrt(residual_sq);
    result.relative_residual = result.final_residual / std::max(b_norm, double(solver_epsilon));
    if (!std::isfinite(result.relative_residual)) {
      result.breakdown = true;
      return result;
    }
    if (result.relative_residual <= tolerance) {
      result.converged = true;
      return result;
    }
    double next_rz;
    if (apply_custom_M) {
      apply_custom_M(r, z);
      next_rz = dot_product(r, z);
    }
    else {
      next_rz = threading::parallel_reduce(
          r.index_range(),
          4096,
          0.0,
          [&](const IndexRange range, const double init) {
            double sum = init;
            for (const int i : range) {
              z[i] = r[i] * inverse_diagonal[i];
              sum += double(r[i]) * double(z[i]);
            }
            return sum;
          },
          std::plus<double>());
    }
    if (!std::isfinite(next_rz) || std::abs(rz) <= solver_epsilon) {
      result.breakdown = true;
      return result;
    }
    const double beta = next_rz / rz;
    threading::parallel_for(d.index_range(), 4096, [&](const IndexRange range) {
      for (const int i : range) {
        d[i] = z[i] + float(beta) * d[i];
      }
    });
    rz = next_rz;
  }
  return result;
}

struct MgStencil {
  int neighbors[6] = {-1, -1, -1, -1, -1, -1};
  float weights[6] = {};
  float diagonal = 0.0f;
};

struct MgLevel {
  int3 dimensions;
  Vector<int3> coordinates;
  Array<MgStencil> stencils;
  Array<int> fine_to_coarse;
  Array<float> x;
  Array<float> b;
  Array<float> ax;

  void allocate_vectors()
  {
    x.reinitialize(coordinates.size());
    b.reinitialize(coordinates.size());
    ax.reinitialize(coordinates.size());
  }
};

/** Experimental Galerkin hierarchy for the FLIP pressure matrix. Each coarse unknown represents
 * a 2x2x2 aggregate of the previous level's active cells. */
class PressureMultigrid {
 private:
  Vector<MgLevel> levels_;

  static void apply_matrix(const MgLevel &level, const Span<float> x, MutableSpan<float> ax)
  {
    threading::parallel_for(level.coordinates.index_range(), 4096, [&](const IndexRange range) {
      for (const int row : range) {
        const MgStencil &stencil = level.stencils[row];
        float value = stencil.diagonal * x[row];
        for (const int slot : IndexRange(6)) {
          const int neighbor = stencil.neighbors[slot];
          if (neighbor >= 0) {
            value -= stencil.weights[slot] * x[neighbor];
          }
        }
        ax[row] = value;
      }
    });
  }

  static void smooth(MgLevel &level, const int sweeps)
  {
    for ([[maybe_unused]] const int sweep : IndexRange(sweeps)) {
      apply_matrix(level, level.x, level.ax);
      threading::parallel_for(level.coordinates.index_range(), 4096, [&](const IndexRange range) {
        for (const int row : range) {
          level.x[row] += 0.7f * (level.b[row] - level.ax[row]) /
                          std::max(level.stencils[row].diagonal, solver_epsilon);
        }
      });
    }
  }

  void v_cycle(const int level_index)
  {
    MgLevel &level = levels_[level_index];
    if (level_index == levels_.size() - 1) {
      smooth(level, 20);
      return;
    }
    smooth(level, 1);
    apply_matrix(level, level.x, level.ax);
    MgLevel &coarse = levels_[level_index + 1];
    coarse.b.fill(0.0f);
    coarse.x.fill(0.0f);
    for (const int row : level.coordinates.index_range()) {
      coarse.b[level.fine_to_coarse[row]] += level.b[row] - level.ax[row];
    }
    v_cycle(level_index + 1);
    threading::parallel_for(level.coordinates.index_range(), 4096, [&](const IndexRange range) {
      for (const int row : range) {
        level.x[row] += coarse.x[level.fine_to_coarse[row]];
      }
    });
    smooth(level, 1);
  }

 public:
  PressureMultigrid(const Span<PressureStencil> fine_stencils,
                    const Span<int3> fine_coordinates,
                    const int3 dimensions,
                    const float3 dx)
  {
    MgLevel fine;
    fine.dimensions = dimensions;
    for (const int3 coordinates : fine_coordinates) {
      fine.coordinates.append(coordinates);
    }
    fine.stencils.reinitialize(fine_stencils.size());
    const float coefficients[3] = {1.0f / (dx.x * dx.x),
                                   1.0f / (dx.y * dx.y),
                                   1.0f / (dx.z * dx.z)};
    for (const int row : fine_stencils.index_range()) {
      const PressureStencil &input = fine_stencils[row];
      MgStencil &output = fine.stencils[row];
      output.diagonal = input.diagonal;
      const int neighbors[6] = {input.xm, input.xp, input.ym, input.yp, input.zm, input.zp};
      for (const int slot : IndexRange(6)) {
        output.neighbors[slot] = neighbors[slot];
        output.weights[slot] = neighbors[slot] >= 0 ? coefficients[slot / 2] : 0.0f;
      }
    }
    fine.allocate_vectors();
    levels_.append(std::move(fine));

    for (const int level_index : IndexRange(8)) {
      MgLevel &parent = levels_[level_index];
      if (parent.coordinates.size() <= 64) {
        break;
      }
      MgLevel coarse;
      coarse.dimensions = int3((parent.dimensions.x + 1) / 2,
                               (parent.dimensions.y + 1) / 2,
                               (parent.dimensions.z + 1) / 2);
      Array<int> cell_to_row(int64_t(coarse.dimensions.x) * coarse.dimensions.y *
                                 coarse.dimensions.z,
                             -1);
      parent.fine_to_coarse.reinitialize(parent.coordinates.size());
      for (const int row : parent.coordinates.index_range()) {
        const int3 cell = parent.coordinates[row];
        const int3 coarse_cell(cell.x / 2, cell.y / 2, cell.z / 2);
        const int index = coarse_cell.x + coarse.dimensions.x *
                                              (coarse_cell.y + coarse.dimensions.y * coarse_cell.z);
        int &coarse_row = cell_to_row[index];
        if (coarse_row < 0) {
          coarse_row = coarse.coordinates.size();
          coarse.coordinates.append(coarse_cell);
        }
        parent.fine_to_coarse[row] = coarse_row;
      }
      coarse.stencils.reinitialize(coarse.coordinates.size());
      for (const int row : parent.coordinates.index_range()) {
        const int coarse_row = parent.fine_to_coarse[row];
        const MgStencil &fine_stencil = parent.stencils[row];
        MgStencil &coarse_stencil = coarse.stencils[coarse_row];
        coarse_stencil.diagonal += fine_stencil.diagonal;
        for (const int slot : IndexRange(6)) {
          const int fine_neighbor = fine_stencil.neighbors[slot];
          if (fine_neighbor < 0) {
            continue;
          }
          const int coarse_neighbor = parent.fine_to_coarse[fine_neighbor];
          if (coarse_neighbor == coarse_row) {
            coarse_stencil.diagonal -= fine_stencil.weights[slot];
          }
          else {
            coarse_stencil.neighbors[slot] = coarse_neighbor;
            coarse_stencil.weights[slot] += fine_stencil.weights[slot];
          }
        }
      }
      coarse.allocate_vectors();
      levels_.append(std::move(coarse));
    }
  }

  void apply(const Span<float> residual, MutableSpan<float> correction)
  {
    MgLevel &fine = levels_.first();
    fine.b.as_mutable_span().copy_from(residual);
    fine.x.fill(0.0f);
    v_cycle(0);
    correction.copy_from(fine.x.as_span());
  }
};

enum class FlipCellType : uint8_t {
  Air,
  Fluid,
  Solid,
};

/** Inclusive cell or face index box. An empty box has `max < min`. */
struct IndexBounds {
  int x_min = 0;
  int y_min = 0;
  int z_min = 0;
  int x_max = -1;
  int y_max = -1;
  int z_max = -1;

  bool is_empty() const
  {
    return x_max < x_min || y_max < y_min || z_max < z_min;
  }
};

static IndexBounds intersect_bounds(const IndexBounds &a, const IndexBounds &b)
{
  IndexBounds result;
  if (a.is_empty() || b.is_empty()) {
    return result;
  }
  result.x_min = std::max(a.x_min, b.x_min);
  result.y_min = std::max(a.y_min, b.y_min);
  result.z_min = std::max(a.z_min, b.z_min);
  result.x_max = std::min(a.x_max, b.x_max);
  result.y_max = std::min(a.y_max, b.y_max);
  result.z_max = std::min(a.z_max, b.z_max);
  if (result.is_empty()) {
    return {};
  }
  return result;
}

static IndexBounds union_bounds(const IndexBounds &a, const IndexBounds &b)
{
  if (a.is_empty()) {
    return b;
  }
  if (b.is_empty()) {
    return a;
  }
  IndexBounds result;
  result.x_min = std::min(a.x_min, b.x_min);
  result.y_min = std::min(a.y_min, b.y_min);
  result.z_min = std::min(a.z_min, b.z_min);
  result.x_max = std::max(a.x_max, b.x_max);
  result.y_max = std::max(a.y_max, b.y_max);
  result.z_max = std::max(a.z_max, b.z_max);
  return result;
}

static bool is_finite(const float3 value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

static float component(const float3 value, const int axis)
{
  return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
}

static void set_component(float3 &value, const int axis, const float component_value)
{
  if (axis == 0) {
    value.x = component_value;
  }
  else if (axis == 1) {
    value.y = component_value;
  }
  else {
    value.z = component_value;
  }
}

struct MacGrid {
  int nx;
  int ny;
  int nz;
  float3 domain_min;
  float3 domain_max;
  float3 dx;

  Array<float> u;
  Array<float> v;
  Array<float> w;
  Array<float> u_old;
  Array<float> v_old;
  Array<float> w_old;
  Array<float> u_weight;
  Array<float> v_weight;
  Array<float> w_weight;
  Array<uint8_t> u_valid;
  Array<uint8_t> v_valid;
  Array<uint8_t> w_valid;
  Array<float> solid_cell_phi;
  Array<float> u_solid_phi;
  Array<float> v_solid_phi;
  Array<float> w_solid_phi;
  Array<float> u_solid_velocity;
  Array<float> v_solid_velocity;
  Array<float> w_solid_velocity;
  Array<uint8_t> u_blocked;
  Array<uint8_t> v_blocked;
  Array<uint8_t> w_blocked;
  Array<FlipCellType> cell_type;
  Array<int> cell_particle_count;
  Array<int> cell_offsets;
  Array<int> particle_indices;
  Array<float> pressure;
  Array<int> fluid_to_row;
  Vector<int3> row_to_cell;

  MacGrid(
      const int nx, const int ny, const int nz, const float3 domain_min, const float3 domain_max)
      : nx(nx),
        ny(ny),
        nz(nz),
        domain_min(domain_min),
        domain_max(domain_max),
        dx((domain_max.x - domain_min.x) / nx,
           (domain_max.y - domain_min.y) / ny,
           (domain_max.z - domain_min.z) / nz),
        u(int64_t(nx + 1) * ny * nz, 0.0f),
        v(int64_t(nx) * (ny + 1) * nz, 0.0f),
        w(int64_t(nx) * ny * (nz + 1), 0.0f),
        u_old(u.size(), 0.0f),
        v_old(v.size(), 0.0f),
        w_old(w.size(), 0.0f),
        u_weight(u.size(), 0.0f),
        v_weight(v.size(), 0.0f),
        w_weight(w.size(), 0.0f),
        u_valid(u.size(), 0),
        v_valid(v.size(), 0),
        w_valid(w.size(), 0),
        solid_cell_phi(int64_t(nx) * ny * nz, std::numeric_limits<float>::infinity()),
        u_solid_phi(u.size(), std::numeric_limits<float>::infinity()),
        v_solid_phi(v.size(), std::numeric_limits<float>::infinity()),
        w_solid_phi(w.size(), std::numeric_limits<float>::infinity()),
        u_solid_velocity(u.size(), 0.0f),
        v_solid_velocity(v.size(), 0.0f),
        w_solid_velocity(w.size(), 0.0f),
        u_blocked(u.size(), 0),
        v_blocked(v.size(), 0),
        w_blocked(w.size(), 0),
        cell_type(int64_t(nx) * ny * nz, FlipCellType::Air),
        cell_particle_count(cell_type.size(), 0),
        cell_offsets(cell_type.size() + 1, 0),
        pressure(cell_type.size(), 0.0f),
        fluid_to_row(cell_type.size(), -1)
  {
  }

  int cell_index(const int i, const int j, const int k) const
  {
    return (k * ny + j) * nx + i;
  }

  int u_index(const int i, const int j, const int k) const
  {
    return (k * ny + j) * (nx + 1) + i;
  }

  int v_index(const int i, const int j, const int k) const
  {
    return (k * (ny + 1) + j) * nx + i;
  }

  int w_index(const int i, const int j, const int k) const
  {
    return (k * ny + j) * nx + i;
  }

  FlipCellType cell_type_at(const int i, const int j, const int k) const
  {
    if (i < 0 || i >= nx || j < 0 || j >= ny || k < 0 || k >= nz) {
      return FlipCellType::Solid;
    }
    return cell_type[cell_index(i, j, k)];
  }

  bool face_touches_fluid(const int axis, const int i, const int j, const int k) const
  {
    if (axis == 0) {
      return cell_type_at(i - 1, j, k) == FlipCellType::Fluid ||
             cell_type_at(i, j, k) == FlipCellType::Fluid;
    }
    if (axis == 1) {
      return cell_type_at(i, j - 1, k) == FlipCellType::Fluid ||
             cell_type_at(i, j, k) == FlipCellType::Fluid;
    }
    return cell_type_at(i, j, k - 1) == FlipCellType::Fluid ||
           cell_type_at(i, j, k) == FlipCellType::Fluid;
  }
};

class FlipSolverCore {
 private:
  const FlipSolverSettings &settings_;
  MacGrid grid_;
  float padding_;
  const FlipColliderQuery *collider_query_ = nullptr;
  float collider_friction_ = 0.0f;
  LinearSolverScratch linear_scratch_;
  /** Particles plus surface radius, extrapolation, and a safety margin. */
  IndexBounds particle_cells_;
  /** Particle region union the collider narrow band. */
  IndexBounds active_cells_;
  IndexBounds collider_cells_;
  Array<float> pressure_warm_;
  int pressure_solve_count_ = 0;
  Array<int> bin_cursor_;
  Array<float3> particle_grid_positions_;
  Array<float> extrap_values_;
  Array<uint8_t> extrap_valid_;
  Array<int> viscosity_face_to_row_;
  Vector<int3> viscosity_rows_;

 public:
  FlipSolverCore(const FlipSolverSettings &settings, const int nx, const int ny, const int nz)
      : settings_(settings), grid_(nx, ny, nz, settings.domain_min, settings.domain_max)
  {
    padding_ = std::clamp(
        settings.boundary_padding, 0.0f, 0.49f * std::min({grid_.dx.x, grid_.dx.y, grid_.dx.z}));
  }

  bool substep(FlipParticleData &particles,
               const float dt,
               const FlipColliderQuery *collider_query,
               const float collider_friction,
               const bool rebuild_collider_grid,
               const bool animated_collider,
               FlipSolverStats *stats,
               std::string &r_error)
  {
    collider_query_ = collider_query;
    collider_friction_ = collider_friction;
    this->sanitize_particles(particles);
    this->constrain_particles(particles.positions.as_mutable_span(),
                              particles.velocities.as_mutable_span());
    this->update_active_bounds(particles.positions);
    {
      ScopedPerformanceTimer timer(stats ? &stats->performance.collider_ms : nullptr);
      if (rebuild_collider_grid) {
        this->build_collider_grid(animated_collider);
      }
      this->resolve_particles_in_collider(particles, stats);
    }
    this->update_active_bounds(particles.positions);
    {
      ScopedPerformanceTimer timer(stats ? &stats->performance.particle_binning_ms : nullptr);
      this->build_particle_bins(particles.positions);
      this->classify_cells_particle_sdf(particles.positions);
    }
    this->finalize_collider_faces();
    if (settings_.reseeding) {
      {
        ScopedPerformanceTimer timer(stats ? &stats->performance.reseeding_ms : nullptr);
        this->reseed_and_cull_particles(particles, stats);
      }
      this->update_active_bounds(particles.positions);
      {
        ScopedPerformanceTimer timer(stats ? &stats->performance.particle_binning_ms : nullptr);
        this->build_particle_bins(particles.positions);
        this->classify_cells_particle_sdf(particles.positions);
      }
      this->finalize_collider_faces();
    }
    this->update_sampling_stats(stats, particles.positions.size());
    {
      ScopedPerformanceTimer timer(stats ? &stats->performance.p2g_ms : nullptr);
      this->clear_grid();
      this->particle_to_grid(particles.positions, particles.velocities);
    }
    {
      ScopedPerformanceTimer timer(stats ? &stats->performance.extrapolation_ms : nullptr);
      this->extrapolate_all(settings_.extrapolation_layers);
    }
    this->enforce_box_velocity_boundary();
    this->enforce_collider_velocity_boundary();
    grid_.u_old.as_mutable_span().copy_from(grid_.u.as_span());
    grid_.v_old.as_mutable_span().copy_from(grid_.v.as_span());
    grid_.w_old.as_mutable_span().copy_from(grid_.w.as_span());

    {
      ScopedPerformanceTimer timer(stats ? &stats->performance.gravity_ms : nullptr);
      this->apply_gravity(dt);
    }
    this->enforce_box_velocity_boundary();
    this->enforce_collider_velocity_boundary();
    {
      ScopedPerformanceTimer timer(stats ? &stats->performance.viscosity_ms : nullptr);
      if (!this->solve_implicit_viscosity(dt, stats, r_error)) {
        return false;
      }
    }
    this->enforce_box_velocity_boundary();
    this->enforce_collider_velocity_boundary();
    this->compute_divergence_stats(true, stats);
    if (!this->solve_pressure(dt, stats, r_error)) {
      return false;
    }
    {
      ScopedPerformanceTimer timer(stats ? &stats->performance.pressure_project_ms : nullptr);
      this->project_pressure(dt);
    }
    this->enforce_box_velocity_boundary();
    this->enforce_collider_velocity_boundary();
    this->compute_divergence_stats(false, stats);
    {
      ScopedPerformanceTimer timer(stats ? &stats->performance.extrapolation_ms : nullptr);
      this->extrapolate_all(settings_.extrapolation_layers);
    }
    this->enforce_box_velocity_boundary();
    this->enforce_collider_velocity_boundary();
    {
      ScopedPerformanceTimer timer(stats ? &stats->performance.g2p_ms : nullptr);
      this->transfer_advect_and_collide(particles, dt, stats);
    }
    return true;
  }

 private:
  Array<float> &face_values(const int axis)
  {
    return axis == 0 ? grid_.u : axis == 1 ? grid_.v : grid_.w;
  }

  const Array<float> &face_values(const int axis) const
  {
    return axis == 0 ? grid_.u : axis == 1 ? grid_.v : grid_.w;
  }

  Array<float> &face_weights(const int axis)
  {
    return axis == 0 ? grid_.u_weight : axis == 1 ? grid_.v_weight : grid_.w_weight;
  }

  Array<uint8_t> &face_valid(const int axis)
  {
    return axis == 0 ? grid_.u_valid : axis == 1 ? grid_.v_valid : grid_.w_valid;
  }

  Array<float> &face_solid_phi(const int axis)
  {
    return axis == 0 ? grid_.u_solid_phi : axis == 1 ? grid_.v_solid_phi : grid_.w_solid_phi;
  }

  const Array<float> &face_solid_velocity(const int axis) const
  {
    return axis == 0 ? grid_.u_solid_velocity :
           axis == 1 ? grid_.v_solid_velocity :
                       grid_.w_solid_velocity;
  }

  Array<float> &face_solid_velocity(const int axis)
  {
    return axis == 0 ? grid_.u_solid_velocity :
           axis == 1 ? grid_.v_solid_velocity :
                       grid_.w_solid_velocity;
  }

  const Array<uint8_t> &face_blocked(const int axis) const
  {
    return axis == 0 ? grid_.u_blocked : axis == 1 ? grid_.v_blocked : grid_.w_blocked;
  }

  Array<uint8_t> &face_blocked(const int axis)
  {
    return axis == 0 ? grid_.u_blocked : axis == 1 ? grid_.v_blocked : grid_.w_blocked;
  }

  const Array<float> &old_face_values(const int axis) const
  {
    return axis == 0 ? grid_.u_old : axis == 1 ? grid_.v_old : grid_.w_old;
  }

  int3 face_size(const int axis) const
  {
    if (axis == 0) {
      return int3(grid_.nx + 1, grid_.ny, grid_.nz);
    }
    if (axis == 1) {
      return int3(grid_.nx, grid_.ny + 1, grid_.nz);
    }
    return int3(grid_.nx, grid_.ny, grid_.nz + 1);
  }

  int face_index(const int axis, const int i, const int j, const int k) const
  {
    return axis == 0 ? grid_.u_index(i, j, k) :
           axis == 1 ? grid_.v_index(i, j, k) :
                       grid_.w_index(i, j, k);
  }

  float3 face_position(const int axis, const int i, const int j, const int k) const
  {
    float3 offset(float(i) + 0.5f, float(j) + 0.5f, float(k) + 0.5f);
    set_component(offset, axis, float(axis == 0 ? i : axis == 1 ? j : k));
    return grid_.domain_min +
           float3(offset.x * grid_.dx.x, offset.y * grid_.dx.y, offset.z * grid_.dx.z);
  }

  bool face_is_blocked(const int axis, const int i, const int j, const int k) const
  {
    return face_blocked(axis)[face_index(axis, i, j, k)] != 0;
  }

  bool cell_neighbor_face_is_blocked(const int3 cell, const int axis, const int direction) const
  {
    int3 face = cell;
    if (direction > 0) {
      if (axis == 0) {
        face.x++;
      }
      else if (axis == 1) {
        face.y++;
      }
      else {
        face.z++;
      }
    }
    return this->face_is_blocked(axis, face.x, face.y, face.z);
  }

  IndexBounds cell_bounds_from_aabb(const float3 &bmin, const float3 &bmax) const
  {
    IndexBounds bounds;
    if (bmax.x < grid_.domain_min.x || bmin.x > grid_.domain_max.x ||
        bmax.y < grid_.domain_min.y || bmin.y > grid_.domain_max.y ||
        bmax.z < grid_.domain_min.z || bmin.z > grid_.domain_max.z)
    {
      return bounds;
    }
    const auto first = [](const float value, const float origin, const float dx) {
      return int(std::floor((value - origin) / dx));
    };
    bounds.x_min = std::clamp(first(bmin.x, grid_.domain_min.x, grid_.dx.x), 0, grid_.nx - 1);
    bounds.y_min = std::clamp(first(bmin.y, grid_.domain_min.y, grid_.dx.y), 0, grid_.ny - 1);
    bounds.z_min = std::clamp(first(bmin.z, grid_.domain_min.z, grid_.dx.z), 0, grid_.nz - 1);
    bounds.x_max = std::clamp(first(bmax.x, grid_.domain_min.x, grid_.dx.x), 0, grid_.nx - 1);
    bounds.y_max = std::clamp(first(bmax.y, grid_.domain_min.y, grid_.dx.y), 0, grid_.ny - 1);
    bounds.z_max = std::clamp(first(bmax.z, grid_.domain_min.z, grid_.dx.z), 0, grid_.nz - 1);
    return bounds;
  }

  void update_active_bounds(const Span<float3> positions)
  {
    particle_cells_ = {};
    if (!positions.is_empty()) {
      float3 bmin = positions[0];
      float3 bmax = positions[0];
      for (const float3 position : positions) {
        bmin = math::min(bmin, position);
        bmax = math::max(bmax, position);
      }
      const float min_dx = std::min({grid_.dx.x, grid_.dx.y, grid_.dx.z});
      const float max_dx = std::max({grid_.dx.x, grid_.dx.y, grid_.dx.z});
      const float radius = std::clamp(settings_.classification_radius_scale, 0.0f, 1.0f) * min_dx;
      const float margin = radius + float(settings_.extrapolation_layers + 2) * max_dx;
      particle_cells_ = this->cell_bounds_from_aabb(bmin - float3(margin), bmax + float3(margin));
    }

    IndexBounds collider_bounds;
    if (collider_query_ != nullptr && collider_query_->is_built()) {
      const float band = 3.0f * std::max({grid_.dx.x, grid_.dx.y, grid_.dx.z});
      collider_bounds = this->cell_bounds_from_aabb(collider_query_->bounds_min() - float3(band),
                                                    collider_query_->bounds_max() + float3(band));
    }
    /* Sample the collider only where it can affect the fluid this substep. */
    collider_cells_ = intersect_bounds(collider_bounds, particle_cells_);
    active_cells_ = union_bounds(particle_cells_, collider_cells_);
  }

  void build_collider_grid(const bool animated_collider)
  {
    grid_.solid_cell_phi.fill(std::numeric_limits<float>::infinity());
    for (const int axis : IndexRange(3)) {
      face_solid_phi(axis).fill(std::numeric_limits<float>::infinity());
      face_solid_velocity(axis).fill(0.0f);
      face_blocked(axis).fill(0);
    }
    if (collider_query_ == nullptr || !collider_query_->is_built()) {
      return;
    }
    const float band = 3.0f * std::max({grid_.dx.x, grid_.dx.y, grid_.dx.z});
    const IndexBounds full_band = this->cell_bounds_from_aabb(
        collider_query_->bounds_min() - float3(band), collider_query_->bounds_max() + float3(band));
    /* Static colliders are reused for every substep, so the narrow band has to cover the whole
     * collider. Animated colliders can skip the build when they do not meet the fluid. */
    IndexBounds sample_cells = full_band;
    if (animated_collider) {
      sample_cells = intersect_bounds(full_band, particle_cells_);
    }
    if (sample_cells.is_empty()) {
      return;
    }

    threading::parallel_for(
        IndexRange(sample_cells.z_min, sample_cells.z_max - sample_cells.z_min + 1),
        1,
        [&](const IndexRange z_range) {
          for (const int z : z_range) {
            for (int j = sample_cells.y_min; j <= sample_cells.y_max; j++) {
              for (int i = sample_cells.x_min; i <= sample_cells.x_max; i++) {
                const int cell_index = grid_.cell_index(i, j, z);
                const FlipColliderSample sample = collider_query_->sample(
                    this->cell_center(int3(i, j, z)));
                if (sample.valid) {
                  grid_.solid_cell_phi[cell_index] = sample.phi;
                }
              }
            }
          }
        });
    for (const int axis : IndexRange(3)) {
      const int3 size = this->face_size(axis);
      const int i0 = std::clamp(sample_cells.x_min - (axis == 0 ? 0 : 1), 0, size.x - 1);
      const int i1 = std::clamp(sample_cells.x_max + 1, 0, size.x - 1);
      const int j0 = std::clamp(sample_cells.y_min - (axis == 1 ? 0 : 1), 0, size.y - 1);
      const int j1 = std::clamp(sample_cells.y_max + 1, 0, size.y - 1);
      const int k0 = std::clamp(sample_cells.z_min - (axis == 2 ? 0 : 1), 0, size.z - 1);
      const int k1 = std::clamp(sample_cells.z_max + 1, 0, size.z - 1);
      Array<float> &solid_phi = face_solid_phi(axis);
      Array<float> &solid_velocity = face_solid_velocity(axis);
      Array<uint8_t> &blocked = face_blocked(axis);
      threading::parallel_for(IndexRange(k0, k1 - k0 + 1), 1, [&](const IndexRange z_range) {
        for (const int k : z_range) {
          for (int j = j0; j <= j1; j++) {
            for (int i = i0; i <= i1; i++) {
              const int index = this->face_index(axis, i, j, k);
              const FlipColliderSample sample = collider_query_->sample(
                  this->face_position(axis, i, j, k));
              if (sample.valid) {
                solid_phi[index] = sample.phi;
                solid_velocity[index] = component(sample.velocity, axis);
                blocked[index] = sample.phi <= 0.0f;
              }
            }
          }
        }
      });
    }
  }

  void finalize_collider_faces()
  {
    if (collider_query_ == nullptr || !collider_query_->is_built() || collider_cells_.is_empty()) {
      return;
    }
    for (const int axis : IndexRange(3)) {
      const int3 size = this->face_size(axis);
      const int i0 = std::clamp(collider_cells_.x_min - 1, 0, size.x - 1);
      const int i1 = std::clamp(collider_cells_.x_max + 1, 0, size.x - 1);
      const int j0 = std::clamp(collider_cells_.y_min - 1, 0, size.y - 1);
      const int j1 = std::clamp(collider_cells_.y_max + 1, 0, size.y - 1);
      const int k0 = std::clamp(collider_cells_.z_min - 1, 0, size.z - 1);
      const int k1 = std::clamp(collider_cells_.z_max + 1, 0, size.z - 1);
      Array<uint8_t> &blocked = face_blocked(axis);
      for (int k = k0; k <= k1; k++) {
        for (int j = j0; j <= j1; j++) {
          for (int i = i0; i <= i1; i++) {
            const bool negative_solid = axis == 0 ? grid_.cell_type_at(i - 1, j, k) ==
                                                        FlipCellType::Solid :
                                        axis == 1 ? grid_.cell_type_at(i, j - 1, k) ==
                                                        FlipCellType::Solid :
                                                    grid_.cell_type_at(i, j, k - 1) ==
                                                        FlipCellType::Solid;
            const bool positive_solid = axis == 0 ?
                                            grid_.cell_type_at(i, j, k) == FlipCellType::Solid :
                                        axis == 1 ?
                                            grid_.cell_type_at(i, j, k) == FlipCellType::Solid :
                                            grid_.cell_type_at(i, j, k) == FlipCellType::Solid;
            const bool negative_in_domain = axis == 0 ? i > 0 : axis == 1 ? j > 0 : k > 0;
            const bool positive_in_domain = axis == 0 ? i < grid_.nx :
                                            axis == 1 ? j < grid_.ny :
                                                        k < grid_.nz;
            const int index = this->face_index(axis, i, j, k);
            blocked[index] |= (negative_in_domain && negative_solid) ||
                              (positive_in_domain && positive_solid);
          }
        }
      }
    }
  }

  void enforce_collider_velocity_boundary()
  {
    if (collider_query_ == nullptr || !collider_query_->is_built() || collider_cells_.is_empty()) {
      return;
    }
    for (const int axis : IndexRange(3)) {
      const int3 size = this->face_size(axis);
      const int i0 = std::clamp(collider_cells_.x_min - 1, 0, size.x - 1);
      const int i1 = std::clamp(collider_cells_.x_max + 1, 0, size.x - 1);
      const int j0 = std::clamp(collider_cells_.y_min - 1, 0, size.y - 1);
      const int j1 = std::clamp(collider_cells_.y_max + 1, 0, size.y - 1);
      const int k0 = std::clamp(collider_cells_.z_min - 1, 0, size.z - 1);
      const int k1 = std::clamp(collider_cells_.z_max + 1, 0, size.z - 1);
      Array<float> &values = face_values(axis);
      Array<uint8_t> &valid = face_valid(axis);
      const Array<float> &solid_velocity = face_solid_velocity(axis);
      const Array<uint8_t> &blocked = face_blocked(axis);
      for (int k = k0; k <= k1; k++) {
        for (int j = j0; j <= j1; j++) {
          for (int i = i0; i <= i1; i++) {
            const int index = this->face_index(axis, i, j, k);
            if (blocked[index]) {
              values[index] = solid_velocity[index];
              valid[index] = 1;
            }
          }
        }
      }
    }
  }

  void resolve_particles_in_collider(FlipParticleData &particles, FlipSolverStats *stats) const
  {
    if (collider_query_ == nullptr || !collider_query_->is_built()) {
      return;
    }
    const float margin = 0.01f * std::min({grid_.dx.x, grid_.dx.y, grid_.dx.z});
    int projected = 0;
    for (const int index : particles.positions.index_range()) {
      projected += resolve_flip_particle_collision(*collider_query_,
                                                   particles.positions[index],
                                                   particles.positions[index],
                                                   particles.velocities[index],
                                                   margin,
                                                   collider_friction_);
    }
    if (stats != nullptr) {
      stats->collider_particles_projected += projected;
    }
  }

  void clear_grid()
  {
    for (const int axis : IndexRange(3)) {
      face_values(axis).fill(0.0f);
      face_weights(axis).fill(0.0f);
      face_valid(axis).fill(0);
    }
    if (pressure_warm_.size() != grid_.pressure.size()) {
      pressure_warm_.reinitialize(grid_.pressure.size());
      pressure_warm_.fill(0.0f);
    }
    else {
      pressure_warm_.as_mutable_span().copy_from(grid_.pressure.as_span());
    }
    grid_.pressure.fill(0.0f);
    grid_.fluid_to_row.fill(-1);
    grid_.row_to_cell.clear();
  }

  void constrain_particles(MutableSpan<float3> positions, MutableSpan<float3> velocities) const
  {
    const float3 center = (settings_.domain_min + settings_.domain_max) * 0.5f;
    const float3 lower = settings_.domain_min + float3(padding_);
    const float3 upper = settings_.domain_max - float3(padding_);
    for (const int index : positions.index_range()) {
      float3 &position = positions[index];
      float3 &velocity = velocities[index];
      if (!is_finite(position)) {
        position = center;
      }
      if (!is_finite(velocity)) {
        velocity = float3(0.0f);
      }
      for (const int axis : IndexRange(3)) {
        const float p = component(position, axis);
        if (p < component(lower, axis)) {
          set_component(position, axis, component(lower, axis));
          if (component(velocity, axis) < 0.0f) {
            set_component(velocity, axis, 0.0f);
          }
        }
        else if (p > component(upper, axis)) {
          set_component(position, axis, component(upper, axis));
          if (component(velocity, axis) > 0.0f) {
            set_component(velocity, axis, 0.0f);
          }
        }
      }
    }
  }

  void sanitize_particles(FlipParticleData &particles) const
  {
    const float3 center = (settings_.domain_min + settings_.domain_max) * 0.5f;
    for (const int i : particles.positions.index_range()) {
      if (!is_finite(particles.positions[i])) {
        particles.positions[i] = center;
      }
      if (!is_finite(particles.velocities[i])) {
        particles.velocities[i] = float3(0.0f);
      }
    }
  }

  int3 position_to_cell(const float3 position) const
  {
    const float3 q((position.x - grid_.domain_min.x) / grid_.dx.x,
                   (position.y - grid_.domain_min.y) / grid_.dx.y,
                   (position.z - grid_.domain_min.z) / grid_.dx.z);
    return int3(std::clamp(int(std::floor(q.x)), 0, grid_.nx - 1),
                std::clamp(int(std::floor(q.y)), 0, grid_.ny - 1),
                std::clamp(int(std::floor(q.z)), 0, grid_.nz - 1));
  }

  int3 cell_coordinates(const int cell) const
  {
    const int k = cell / (grid_.nx * grid_.ny);
    const int remainder = cell - k * grid_.nx * grid_.ny;
    return int3(remainder % grid_.nx, remainder / grid_.nx, k);
  }

  float3 cell_center(const int3 cell) const
  {
    return grid_.domain_min + float3((float(cell.x) + 0.5f) * grid_.dx.x,
                                     (float(cell.y) + 0.5f) * grid_.dx.y,
                                     (float(cell.z) + 0.5f) * grid_.dx.z);
  }

  void build_particle_bins(const Span<float3> positions)
  {
    grid_.cell_particle_count.fill(0);
    for (const float3 position : positions) {
      const int3 cell = this->position_to_cell(position);
      grid_.cell_particle_count[grid_.cell_index(cell.x, cell.y, cell.z)]++;
    }
    grid_.cell_offsets[0] = 0;
    for (const int cell : grid_.cell_particle_count.index_range()) {
      grid_.cell_offsets[cell + 1] = grid_.cell_offsets[cell] + grid_.cell_particle_count[cell];
    }
    BLI_assert(grid_.cell_offsets.last() == positions.size());
    if (grid_.particle_indices.size() != positions.size()) {
      grid_.particle_indices.reinitialize(positions.size());
    }
    if (bin_cursor_.size() != grid_.cell_particle_count.size()) {
      bin_cursor_.reinitialize(grid_.cell_particle_count.size());
    }
    bin_cursor_.as_mutable_span().copy_from(grid_.cell_offsets.as_span().drop_back(1));
    for (const int particle : positions.index_range()) {
      const int3 cell = this->position_to_cell(positions[particle]);
      const int cell_index = grid_.cell_index(cell.x, cell.y, cell.z);
      grid_.particle_indices[bin_cursor_[cell_index]++] = particle;
    }
  }

  void classify_cells_particle_sdf(const Span<float3> positions)
  {
    grid_.cell_type.fill(FlipCellType::Air);
    if (active_cells_.is_empty()) {
      return;
    }
    const float min_dx = std::min({grid_.dx.x, grid_.dx.y, grid_.dx.z});
    const float radius = std::clamp(settings_.classification_radius_scale, 0.0f, 1.0f) * min_dx;
    /* A particle two cells away is at least 1.5 cell widths from this cell's center. Since the
     * classification radius never exceeds the smallest cell width, only immediate neighbors can
     * affect the fluid classification. */
    threading::parallel_for(
        IndexRange(active_cells_.z_min, active_cells_.z_max - active_cells_.z_min + 1),
        1,
        [&](const IndexRange z_range) {
          for (const int z : z_range) {
            for (int y = active_cells_.y_min; y <= active_cells_.y_max; y++) {
              for (int x = active_cells_.x_min; x <= active_cells_.x_max; x++) {
                const int3 cell(x, y, z);
                const int cell_index = grid_.cell_index(x, y, z);
                bool is_fluid = grid_.cell_particle_count[cell_index] > 0;
                if (!is_fluid && radius > 0.0f) {
                  const float3 center = this->cell_center(cell);
                  for (int k = std::max(0, z - 1); k <= std::min(grid_.nz - 1, z + 1) && !is_fluid;
                       k++)
                  {
                    for (int j = std::max(0, y - 1); j <= std::min(grid_.ny - 1, y + 1) && !is_fluid;
                         j++)
                    {
                      for (int i = std::max(0, x - 1);
                           i <= std::min(grid_.nx - 1, x + 1) && !is_fluid;
                           i++)
                      {
                        const int neighbor = grid_.cell_index(i, j, k);
                        for (int offset = grid_.cell_offsets[neighbor];
                             offset < grid_.cell_offsets[neighbor + 1];
                             offset++)
                        {
                          const float3 delta = center - positions[grid_.particle_indices[offset]];
                          const float distance = std::sqrt(delta.x * delta.x + delta.y * delta.y +
                                                           delta.z * delta.z);
                          if (distance < radius) {
                            is_fluid = true;
                            break;
                          }
                        }
                      }
                    }
                  }
                }
                FlipCellType type = is_fluid ? FlipCellType::Fluid : FlipCellType::Air;
                if (grid_.solid_cell_phi[cell_index] < 0.0f) {
                  type = FlipCellType::Solid;
                }
                grid_.cell_type[cell_index] = type;
              }
            }
          }
        });
  }

  int nearest_source_particle(const int3 cell, const Span<float3> positions) const
  {
    const float3 center = this->cell_center(cell);
    int best_particle = -1;
    float best_distance_squared = std::numeric_limits<float>::infinity();
    for (int radius = 0; radius <= 2 && best_particle < 0; radius++) {
      for (int k = std::max(0, cell.z - radius); k <= std::min(grid_.nz - 1, cell.z + radius); k++)
      {
        for (int j = std::max(0, cell.y - radius); j <= std::min(grid_.ny - 1, cell.y + radius);
             j++)
        {
          for (int i = std::max(0, cell.x - radius); i <= std::min(grid_.nx - 1, cell.x + radius);
               i++)
          {
            const int neighbor = grid_.cell_index(i, j, k);
            for (int offset = grid_.cell_offsets[neighbor];
                 offset < grid_.cell_offsets[neighbor + 1];
                 offset++)
            {
              const int particle = grid_.particle_indices[offset];
              const float3 delta = positions[particle] - center;
              const float distance_squared = delta.x * delta.x + delta.y * delta.y +
                                             delta.z * delta.z;
              if (distance_squared < best_distance_squared) {
                best_distance_squared = distance_squared;
                best_particle = particle;
              }
            }
          }
        }
      }
    }
    return best_particle;
  }

  void reseed_and_cull_particles(FlipParticleData &particles, FlipSolverStats *stats)
  {
    timeit::TimePoint phase_start = timeit::Clock::now();
    const int input_particle_count = particles.positions.size();
    const bool preserve_particle_budget = input_particle_count >=
                                          settings_.target_particles_per_cell;

    Array<uint8_t> keep(particles.positions.size(), 1);
    int culled = 0;
    for (const int cell : grid_.cell_particle_count.index_range()) {
      const int count = grid_.cell_particle_count[cell];
      if (count <= settings_.max_particles_per_cell) {
        continue;
      }
      Vector<int> sorted;
      sorted.reserve(count);
      for (int offset = grid_.cell_offsets[cell]; offset < grid_.cell_offsets[cell + 1]; offset++)
      {
        sorted.append(grid_.particle_indices[offset]);
      }
      std::sort(sorted.begin(), sorted.end(), [&](const int a, const int b) {
        const uint32_t ha = hash_u32(uint32_t(particles.ids[a]) ^ uint32_t(cell));
        const uint32_t hb = hash_u32(uint32_t(particles.ids[b]) ^ uint32_t(cell));
        return ha == hb ? particles.ids[a] < particles.ids[b] : ha < hb;
      });
      for (const int i : IndexRange(settings_.target_particles_per_cell,
                                    sorted.size() - settings_.target_particles_per_cell))
      {
        keep[sorted[i]] = 0;
        culled++;
      }
    }

    int64_t next_id = 0;
    for (const int64_t id : particles.ids) {
      next_id = std::max(next_id, id + 1);
    }
    FlipParticleData result;
    result.positions.reserve(particles.positions.size());
    result.velocities.reserve(particles.positions.size());
    result.ids.reserve(particles.positions.size());
    result.source_indices.reserve(particles.positions.size());
    for (const int particle : particles.positions.index_range()) {
      if (keep[particle]) {
        result.positions.append(particles.positions[particle]);
        result.velocities.append(particles.velocities[particle]);
        result.ids.append(particles.ids[particle]);
        result.source_indices.append(particles.source_indices[particle]);
      }
    }
    const int retained_particle_count = result.positions.size();
    if (stats != nullptr) {
      stats->performance.reseed_cull_ms +=
          std::chrono::duration<double, std::milli>(timeit::Clock::now() - phase_start).count();
    }
    phase_start = timeit::Clock::now();

    /* Defer construction of new markers until after the budget selection. The virtual ordering
     * and IDs below match the original append-then-balance path exactly, so discarded candidates
     * never need positions, velocities, attribute copies, or particle-array storage. */
    if (preserve_particle_budget &&
        (collider_query_ == nullptr || !collider_query_->is_built()))
    {
      struct Candidate {
        int cell_index;
        int slot;
        int parent;
        int64_t id;
      };
      Vector<Candidate> candidates;
      Array<int> candidate_parents(grid_.cell_type.size(), -1);
      threading::parallel_for(grid_.cell_type.index_range(), 1024, [&](const IndexRange range) {
        for (const int cell_index : range) {
          if (grid_.cell_type[cell_index] != FlipCellType::Fluid) {
            continue;
          }
          const int count = grid_.cell_particle_count[cell_index];
          if (count >= settings_.min_particles_per_cell) {
            continue;
          }
          const int3 cell = this->cell_coordinates(cell_index);
          if (count == 0) {
            const int neighbors[6][3] = {{cell.x - 1, cell.y, cell.z},
                                         {cell.x + 1, cell.y, cell.z},
                                         {cell.x, cell.y - 1, cell.z},
                                         {cell.x, cell.y + 1, cell.z},
                                         {cell.x, cell.y, cell.z - 1},
                                         {cell.x, cell.y, cell.z + 1}};
            bool is_surface_cell = false;
            for (const auto &neighbor : neighbors) {
              if (grid_.cell_type_at(neighbor[0], neighbor[1], neighbor[2]) !=
                  FlipCellType::Fluid)
              {
                is_surface_cell = true;
                break;
              }
            }
            if (is_surface_cell) {
              continue;
            }
          }
          candidate_parents[cell_index] = this->nearest_source_particle(cell,
                                                                         particles.positions);
        }
      });
      for (const int cell_index : candidate_parents.index_range()) {
        const int parent = candidate_parents[cell_index];
        if (parent >= 0) {
          for (int slot = grid_.cell_particle_count[cell_index];
               slot < settings_.target_particles_per_cell;
               slot++)
          {
            candidates.append({cell_index, slot, parent, next_id++});
          }
        }
      }
      const timeit::TimePoint balance_start = timeit::Clock::now();
      const int virtual_count = retained_particle_count + candidates.size();
      const auto virtual_id = [&](const int index) -> int64_t {
        return index < retained_particle_count ? result.ids[index] :
                                                 candidates[index - retained_particle_count].id;
      };
      Array<uint8_t> remove(virtual_count, 0);
      if (virtual_count > input_particle_count) {
        Array<int> cell_counts(grid_.cell_type.size(), 0);
        for (const float3 position : result.positions) {
          const int3 cell = this->position_to_cell(position);
          cell_counts[grid_.cell_index(cell.x, cell.y, cell.z)]++;
        }
        for (const Candidate &candidate : candidates) {
          cell_counts[candidate.cell_index]++;
        }
        Array<int> cell_offsets(cell_counts.size() + 1, 0);
        for (const int cell : cell_counts.index_range()) {
          cell_offsets[cell + 1] = cell_offsets[cell] + cell_counts[cell];
        }
        Array<int> cell_particle_indices(virtual_count);
        Array<int> cursor(cell_counts.size());
        cursor.as_mutable_span().copy_from(cell_offsets.as_span().drop_back(1));
        for (const int particle : result.positions.index_range()) {
          const int3 cell = this->position_to_cell(result.positions[particle]);
          const int cell_index = grid_.cell_index(cell.x, cell.y, cell.z);
          cell_particle_indices[cursor[cell_index]++] = particle;
        }
        for (const int candidate : candidates.index_range()) {
          const int cell_index = candidates[candidate].cell_index;
          cell_particle_indices[cursor[cell_index]++] = retained_particle_count + candidate;
        }

        Vector<int> removable;
        removable.reserve(virtual_count - input_particle_count);
        threading::parallel_for(cell_counts.index_range(), 1024, [&](const IndexRange range) {
          for (const int cell : range) {
            const int removable_in_cell = std::max(
                0, cell_counts[cell] - settings_.min_particles_per_cell);
            if (removable_in_cell == 0) {
              continue;
            }
            const int cell_start = cell_offsets[cell];
            const int cell_end = cell_offsets[cell + 1];
            const auto compare_cell_particles = [&](const int a, const int b) {
              const bool a_spawned = a >= retained_particle_count;
              const bool b_spawned = b >= retained_particle_count;
              if (a_spawned != b_spawned) {
                return a_spawned;
              }
              const uint32_t ha = hash_u32(uint32_t(virtual_id(a)) ^ uint32_t(cell));
              const uint32_t hb = hash_u32(uint32_t(virtual_id(b)) ^ uint32_t(cell));
              if (ha != hb) {
                return ha < hb;
              }
              return virtual_id(a) == virtual_id(b) ? a < b : virtual_id(a) < virtual_id(b);
            };
            std::sort(cell_particle_indices.begin() + cell_start,
                      cell_particle_indices.begin() + cell_end,
                      compare_cell_particles);
          }
        });
        for (const int cell : cell_counts.index_range()) {
          const int removable_in_cell = std::max(
              0, cell_counts[cell] - settings_.min_particles_per_cell);
          for (int offset = cell_offsets[cell];
               offset < cell_offsets[cell] + removable_in_cell;
               offset++)
          {
            removable.append(cell_particle_indices[offset]);
          }
        }
        int excess = virtual_count - input_particle_count;
        if (excess < removable.size()) {
          std::nth_element(removable.begin(),
                           removable.begin() + excess,
                           removable.end(),
                           [&](const int a, const int b) {
                             const uint32_t ha = hash_u32(uint32_t(virtual_id(a)));
                             const uint32_t hb = hash_u32(uint32_t(virtual_id(b)));
                             if (ha != hb) {
                               return ha < hb;
                             }
                             return virtual_id(a) == virtual_id(b) ? a < b : virtual_id(a) <
                                                                                 virtual_id(b);
                           });
        }
        for (const int particle : removable) {
          if (excess == 0) {
            break;
          }
          remove[particle] = 1;
          excess--;
        }
        if (excess > 0) {
          Vector<int> fallback;
          fallback.reserve(virtual_count - removable.size());
          for (const int particle : IndexRange(virtual_count)) {
            if (!remove[particle]) {
              fallback.append(particle);
            }
          }
          if (excess < fallback.size()) {
            std::nth_element(fallback.begin(),
                             fallback.begin() + excess,
                             fallback.end(),
                             [&](const int a, const int b) {
                               const bool a_spawned = a >= retained_particle_count;
                               const bool b_spawned = b >= retained_particle_count;
                               if (a_spawned != b_spawned) {
                                 return a_spawned;
                               }
                               const uint32_t ha = hash_u32(uint32_t(virtual_id(a)));
                               const uint32_t hb = hash_u32(uint32_t(virtual_id(b)));
                               if (ha != hb) {
                                 return ha < hb;
                               }
                               return virtual_id(a) == virtual_id(b) ? a < b : virtual_id(a) <
                                                                                   virtual_id(b);
                             });
          }
          for (int i = 0; i < excess; i++) {
            remove[fallback[i]] = 1;
          }
        }
      }
      if (stats != nullptr) {
        stats->performance.reseed_balance_ms +=
            std::chrono::duration<double, std::milli>(timeit::Clock::now() - balance_start)
                .count();
      }
      const timeit::TimePoint materialize_start = timeit::Clock::now();
      FlipParticleData balanced;
      balanced.positions.reserve(std::min(virtual_count, input_particle_count));
      balanced.velocities.reserve(std::min(virtual_count, input_particle_count));
      balanced.ids.reserve(std::min(virtual_count, input_particle_count));
      balanced.source_indices.reserve(std::min(virtual_count, input_particle_count));
      int culled_existing = 0;
      for (const int particle : result.positions.index_range()) {
        if (remove[particle]) {
          culled_existing++;
          continue;
        }
        balanced.positions.append(result.positions[particle]);
        balanced.velocities.append(result.velocities[particle]);
        balanced.ids.append(result.ids[particle]);
        balanced.source_indices.append(result.source_indices[particle]);
      }
      int seeded = 0;
      int cached_cell = -1;
      float3 average_velocity(0.0f);
      for (const int candidate_index : candidates.index_range()) {
        if (remove[retained_particle_count + candidate_index]) {
          continue;
        }
        const Candidate &candidate = candidates[candidate_index];
        if (cached_cell != candidate.cell_index) {
          cached_cell = candidate.cell_index;
          average_velocity = float3(0.0f);
          int samples = 0;
          for (int offset = grid_.cell_offsets[cached_cell];
               offset < grid_.cell_offsets[cached_cell + 1];
               offset++)
          {
            average_velocity += particles.velocities[grid_.particle_indices[offset]];
            samples++;
          }
          average_velocity = samples > 0 ? average_velocity / float(samples) :
                                           particles.velocities[candidate.parent];
        }
        const int3 cell = this->cell_coordinates(candidate.cell_index);
        const float3 cell_min = grid_.domain_min + float3(float(cell.x) * grid_.dx.x,
                                                          float(cell.y) * grid_.dx.y,
                                                          float(cell.z) * grid_.dx.z);
        const uint32_t seed = uint32_t(candidate.slot * 24);
        const float3 local(0.1f + 0.8f * hash_unit(uint32_t(candidate.cell_index), seed),
                           0.1f + 0.8f * hash_unit(uint32_t(candidate.cell_index), seed + 1),
                           0.1f + 0.8f * hash_unit(uint32_t(candidate.cell_index), seed + 2));
        balanced.positions.append(cell_min + float3(local.x * grid_.dx.x,
                                                    local.y * grid_.dx.y,
                                                    local.z * grid_.dx.z));
        balanced.velocities.append(average_velocity);
        balanced.ids.append(candidate.id);
        balanced.source_indices.append(particles.source_indices[candidate.parent]);
        seeded++;
      }
      particles = std::move(balanced);
      if (stats != nullptr) {
        stats->performance.reseed_seed_ms +=
            std::chrono::duration<double, std::milli>(balance_start - phase_start).count() +
            std::chrono::duration<double, std::milli>(timeit::Clock::now() - materialize_start)
                .count();
        stats->particles_seeded += seeded;
        stats->particles_culled += culled + culled_existing;
      }
      return;
    }

    int seeded = 0;
    for (const int cell_index : grid_.cell_type.index_range()) {
      if (grid_.cell_type[cell_index] != FlipCellType::Fluid) {
        continue;
      }
      const int count = grid_.cell_particle_count[cell_index];
      if (count >= settings_.min_particles_per_cell) {
        continue;
      }
      const int3 cell = this->cell_coordinates(cell_index);
      if (count == 0) {
        const int neighbors[6][3] = {{cell.x - 1, cell.y, cell.z},
                                     {cell.x + 1, cell.y, cell.z},
                                     {cell.x, cell.y - 1, cell.z},
                                     {cell.x, cell.y + 1, cell.z},
                                     {cell.x, cell.y, cell.z - 1},
                                     {cell.x, cell.y, cell.z + 1}};
        bool is_surface_cell = false;
        for (const auto &neighbor : neighbors) {
          if (grid_.cell_type_at(neighbor[0], neighbor[1], neighbor[2]) != FlipCellType::Fluid) {
            is_surface_cell = true;
            break;
          }
        }
        if (is_surface_cell) {
          continue;
        }
      }
      const int parent = this->nearest_source_particle(cell, particles.positions);
      if (parent < 0) {
        continue;
      }
      float3 average_velocity(0.0f);
      int velocity_samples = 0;
      for (int offset = grid_.cell_offsets[cell_index];
           offset < grid_.cell_offsets[cell_index + 1];
           offset++)
      {
        average_velocity += particles.velocities[grid_.particle_indices[offset]];
        velocity_samples++;
      }
      if (velocity_samples > 0) {
        average_velocity /= float(velocity_samples);
      }
      else {
        average_velocity = particles.velocities[parent];
      }
      for (int slot = count; slot < settings_.target_particles_per_cell; slot++) {
        const float3 cell_min = grid_.domain_min + float3(float(cell.x) * grid_.dx.x,
                                                          float(cell.y) * grid_.dx.y,
                                                          float(cell.z) * grid_.dx.z);
        float3 spawn_position;
        bool found_spawn_position = false;
        for (const int attempt : IndexRange(8)) {
          const uint32_t seed = uint32_t(slot * 24 + attempt * 3);
          const float3 local(0.1f + 0.8f * hash_unit(uint32_t(cell_index), seed),
                             0.1f + 0.8f * hash_unit(uint32_t(cell_index), seed + 1),
                             0.1f + 0.8f * hash_unit(uint32_t(cell_index), seed + 2));
          spawn_position = cell_min + float3(local.x * grid_.dx.x,
                                             local.y * grid_.dx.y,
                                             local.z * grid_.dx.z);
          if (collider_query_ == nullptr || !collider_query_->is_built()) {
            found_spawn_position = true;
            break;
          }
          const FlipColliderSample sample = collider_query_->sample(spawn_position);
          if (!sample.valid || sample.phi > 0.01f * std::min({grid_.dx.x, grid_.dx.y, grid_.dx.z}))
          {
            found_spawn_position = true;
            break;
          }
        }
        if (!found_spawn_position) {
          continue;
        }
        result.positions.append(spawn_position);
        result.velocities.append(average_velocity);
        result.ids.append(next_id++);
        result.source_indices.append(particles.source_indices[parent]);
        seeded++;
      }
    }

    if (stats != nullptr) {
      stats->performance.reseed_seed_ms +=
          std::chrono::duration<double, std::milli>(timeit::Clock::now() - phase_start).count();
    }
    phase_start = timeit::Clock::now();
    /* Once the current marker set reaches the minimum sampling density, reseeding should
     * redistribute markers rather than continuously increasing their global number whenever a
     * moving free surface straddles a new cell. Bootstrap genuinely undersampled inputs, but
     * otherwise deterministically remove the same number of redundant markers that were added. */
    if (preserve_particle_budget && result.positions.size() > input_particle_count) {
      Array<int> result_cell_counts(grid_.cell_type.size(), 0);
      for (const float3 position : result.positions) {
        const int3 cell = this->position_to_cell(position);
        result_cell_counts[grid_.cell_index(cell.x, cell.y, cell.z)]++;
      }

      Array<int> result_cell_offsets(result_cell_counts.size() + 1, 0);
      for (const int cell : result_cell_counts.index_range()) {
        result_cell_offsets[cell + 1] = result_cell_offsets[cell] + result_cell_counts[cell];
      }
      Array<int> result_particle_indices(result.positions.size());
      Array<int> cursor(result_cell_counts.size());
      cursor.as_mutable_span().copy_from(result_cell_offsets.as_span().drop_back(1));
      for (const int particle : result.positions.index_range()) {
        const int3 cell = this->position_to_cell(result.positions[particle]);
        const int cell_index = grid_.cell_index(cell.x, cell.y, cell.z);
        result_particle_indices[cursor[cell_index]++] = particle;
      }

      Array<uint8_t> remove(result.positions.size(), 0);
      Vector<int> removable;
      removable.reserve(result.positions.size() - input_particle_count);
      for (const int cell : result_cell_counts.index_range()) {
        const int removable_in_cell = std::max(
            0, result_cell_counts[cell] - settings_.min_particles_per_cell);
        if (removable_in_cell == 0) {
          continue;
        }
        const int cell_start = result_cell_offsets[cell];
        const int cell_end = result_cell_offsets[cell + 1];
        const auto compare_cell_particles = [&](const int a, const int b) {
          const bool a_spawned = a >= retained_particle_count;
          const bool b_spawned = b >= retained_particle_count;
          if (a_spawned != b_spawned) {
            return a_spawned;
          }
          const uint32_t ha = hash_u32(uint32_t(result.ids[a]) ^ uint32_t(cell));
          const uint32_t hb = hash_u32(uint32_t(result.ids[b]) ^ uint32_t(cell));
          if (ha != hb) {
            return ha < hb;
          }
          return result.ids[a] == result.ids[b] ? a < b : result.ids[a] < result.ids[b];
        };
        std::sort(result_particle_indices.begin() + cell_start,
                  result_particle_indices.begin() + cell_end,
                  compare_cell_particles);
        for (int offset = cell_start; offset < cell_start + removable_in_cell; offset++) {
          removable.append(result_particle_indices[offset]);
        }
      }
      int excess = result.positions.size() - input_particle_count;
      /* Only the selected set matters: survivors keep their original particle order below. */
      if (excess < removable.size()) {
        std::nth_element(removable.begin(),
                         removable.begin() + excess,
                         removable.end(),
                         [&](const int a, const int b) {
                           const uint32_t ha = hash_u32(uint32_t(result.ids[a]));
                           const uint32_t hb = hash_u32(uint32_t(result.ids[b]));
                           if (ha != hb) {
                             return ha < hb;
                           }
                           return result.ids[a] == result.ids[b] ? a < b :
                                                                    result.ids[a] < result.ids[b];
                         });
      }
      for (const int particle : removable) {
        if (excess == 0) {
          break;
        }
        remove[particle] = 1;
        excess--;
      }

      /* An SDF-classified interior hole can make the per-cell minimum exceed the available marker
       * budget. Preserve the global volume proxy even in that case, preferring newly spawned
       * markers for the remaining deterministic removals. */
      if (excess > 0) {
        Vector<int> fallback;
        fallback.reserve(result.positions.size() - removable.size());
        for (const int particle : result.positions.index_range()) {
          if (!remove[particle]) {
            fallback.append(particle);
          }
        }
        if (excess < fallback.size()) {
          std::nth_element(fallback.begin(),
                           fallback.begin() + excess,
                           fallback.end(),
                           [&](const int a, const int b) {
                             const bool a_spawned = a >= retained_particle_count;
                             const bool b_spawned = b >= retained_particle_count;
                             if (a_spawned != b_spawned) {
                               return a_spawned;
                             }
                             const uint32_t ha = hash_u32(uint32_t(result.ids[a]));
                             const uint32_t hb = hash_u32(uint32_t(result.ids[b]));
                             if (ha != hb) {
                               return ha < hb;
                             }
                             return result.ids[a] == result.ids[b] ? a < b :
                                                                      result.ids[a] < result.ids[b];
                           });
        }
        for (int i = 0; i < excess; i++) {
          remove[fallback[i]] = 1;
        }
      }

      FlipParticleData balanced;
      balanced.positions.reserve(input_particle_count);
      balanced.velocities.reserve(input_particle_count);
      balanced.ids.reserve(input_particle_count);
      balanced.source_indices.reserve(input_particle_count);
      for (const int particle : result.positions.index_range()) {
        if (!remove[particle]) {
          balanced.positions.append(result.positions[particle]);
          balanced.velocities.append(result.velocities[particle]);
          balanced.ids.append(result.ids[particle]);
          balanced.source_indices.append(result.source_indices[particle]);
        }
      }
      culled += result.positions.size() - balanced.positions.size();
      result = std::move(balanced);
    }
    particles = std::move(result);
    if (stats != nullptr) {
      stats->performance.reseed_balance_ms +=
          std::chrono::duration<double, std::milli>(timeit::Clock::now() - phase_start).count();
      stats->particles_seeded += seeded;
      stats->particles_culled += culled;
    }
  }

  void update_sampling_stats(FlipSolverStats *stats, const int particles_num) const
  {
    if (stats == nullptr) {
      return;
    }
    int fluid_cells = 0;
    int empty_cells = 0;
    int sparse_cells = 0;
    int max_count = 0;
    for (const int cell : grid_.cell_type.index_range()) {
      if (grid_.cell_type[cell] != FlipCellType::Fluid) {
        continue;
      }
      fluid_cells++;
      const int count = grid_.cell_particle_count[cell];
      empty_cells += count == 0;
      sparse_cells += count < settings_.min_particles_per_cell;
      max_count = std::max(max_count, count);
    }
    stats->fluid_cells = fluid_cells;
    stats->empty_fluid_cells = empty_cells;
    stats->sparse_fluid_cells = sparse_cells;
    stats->max_particles_in_cell = max_count;
    stats->average_particles_per_fluid_cell = fluid_cells > 0 ?
                                                  float(particles_num) / float(fluid_cells) :
                                                  0.0f;
    stats->particle_undersampled = stats->average_particles_per_fluid_cell < 2.0f;
  }

  static float tent(const float sample, const int face_coord)
  {
    const float distance = std::abs(sample - float(face_coord));
    return distance < 1.0f ? 1.0f - distance : 0.0f;
  }

  float3 position_to_grid_coordinates(const float3 position) const
  {
    return float3((position.x - grid_.domain_min.x) / grid_.dx.x,
                  (position.y - grid_.domain_min.y) / grid_.dx.y,
                  (position.z - grid_.domain_min.z) / grid_.dx.z);
  }

  /**
   * Staggered trilinear kernel support, in particle cell indices.
   * Along the face normal the support is two cells; the tangential axes are three.
   */
  static void face_particle_cell_range(const int axis,
                                       const int i,
                                       const int j,
                                       const int k,
                                       int &r_x0,
                                       int &r_x1,
                                       int &r_y0,
                                       int &r_y1,
                                       int &r_z0,
                                       int &r_z1)
  {
    r_x0 = i - 1;
    r_x1 = axis == 0 ? i : i + 1;
    r_y0 = j - 1;
    r_y1 = axis == 1 ? j : j + 1;
    r_z0 = k - 1;
    r_z1 = axis == 2 ? k : k + 1;
  }

  void gather_component(const int axis,
                        const Span<float3> grid_positions,
                        const Span<float3> velocities)
  {
    if (particle_cells_.is_empty()) {
      return;
    }
    const int3 size = face_size(axis);
    const int i0 = std::clamp(particle_cells_.x_min - (axis == 0 ? 0 : 1), 0, size.x - 1);
    const int i1 = std::clamp(particle_cells_.x_max + 1, 0, size.x - 1);
    const int j0 = std::clamp(particle_cells_.y_min - (axis == 1 ? 0 : 1), 0, size.y - 1);
    const int j1 = std::clamp(particle_cells_.y_max + 1, 0, size.y - 1);
    const int k0 = std::clamp(particle_cells_.z_min - (axis == 2 ? 0 : 1), 0, size.z - 1);
    const int k1 = std::clamp(particle_cells_.z_max + 1, 0, size.z - 1);
    if (i1 < i0 || j1 < j0 || k1 < k0) {
      return;
    }
    const int64_t count_x = int64_t(i1 - i0 + 1);
    const int64_t count_y = int64_t(j1 - j0 + 1);
    const int64_t count_z = int64_t(k1 - k0 + 1);
    Array<float> &values = face_values(axis);
    Array<uint8_t> &valid = face_valid(axis);
    threading::parallel_for(IndexRange(count_x * count_y * count_z), 256, [&](const IndexRange range) {
      for (const int64_t linear : range) {
        const int64_t z_local = linear / (count_x * count_y);
        const int64_t remainder = linear - z_local * count_x * count_y;
        const int i = i0 + int(remainder % count_x);
        const int j = j0 + int(remainder / count_x);
        const int k = k0 + int(z_local);
        int cx0, cx1, cy0, cy1, cz0, cz1;
        face_particle_cell_range(axis, i, j, k, cx0, cx1, cy0, cy1, cz0, cz1);
        cx0 = std::max(cx0, 0);
        cy0 = std::max(cy0, 0);
        cz0 = std::max(cz0, 0);
        cx1 = std::min(cx1, grid_.nx - 1);
        cy1 = std::min(cy1, grid_.ny - 1);
        cz1 = std::min(cz1, grid_.nz - 1);
        float weighted_velocity = 0.0f;
        float weight_sum = 0.0f;
        for (int cz = cz0; cz <= cz1; cz++) {
          for (int cy = cy0; cy <= cy1; cy++) {
            for (int cx = cx0; cx <= cx1; cx++) {
              const int cell = grid_.cell_index(cx, cy, cz);
              for (int offset = grid_.cell_offsets[cell]; offset < grid_.cell_offsets[cell + 1];
                   offset++)
              {
                const int particle = grid_.particle_indices[offset];
                 float3 sample = grid_positions[particle];
                if (axis != 0) {
                  sample.x -= 0.5f;
                }
                if (axis != 1) {
                  sample.y -= 0.5f;
                }
                if (axis != 2) {
                  sample.z -= 0.5f;
                }
                const float weight = tent(sample.x, i) * tent(sample.y, j) * tent(sample.z, k);
                if (weight <= 0.0f) {
                  continue;
                }
                weighted_velocity += component(velocities[particle], axis) * weight;
                weight_sum += weight;
              }
            }
          }
        }
        const int index = face_index(axis, i, j, k);
        if (weight_sum > weight_epsilon) {
          values[index] = weighted_velocity / weight_sum;
          valid[index] = 1;
        }
      }
    });
  }

  void particle_to_grid(const Span<float3> positions, const Span<float3> velocities)
  {
    if (particle_grid_positions_.size() != positions.size()) {
      particle_grid_positions_.reinitialize(positions.size());
    }
    threading::parallel_for(positions.index_range(), 4096, [&](const IndexRange range) {
      for (const int particle : range) {
        const float3 position = positions[particle];
        particle_grid_positions_[particle] = this->position_to_grid_coordinates(position);
      }
    });
    /* Face-centric gather: each face is written once, so U/V/W can run in parallel. */
    for (const int axis : IndexRange(3)) {
      this->gather_component(axis, particle_grid_positions_, velocities);
    }
  }

  void extrapolate_component(const int axis, const int layers)
  {
    if (layers <= 0 || active_cells_.is_empty()) {
      return;
    }
    Array<float> &values = face_values(axis);
    Array<uint8_t> &valid = face_valid(axis);
    const Array<uint8_t> &blocked = face_blocked(axis);
    const int3 size = face_size(axis);
    if (extrap_values_.size() < values.size()) {
      extrap_values_.reinitialize(values.size());
      extrap_valid_.reinitialize(values.size());
    }
    const int i0 = std::clamp(active_cells_.x_min - 1, 0, size.x - 1);
    const int i1 = std::clamp(active_cells_.x_max + 1, 0, size.x - 1);
    const int j0 = std::clamp(active_cells_.y_min - 1, 0, size.y - 1);
    const int j1 = std::clamp(active_cells_.y_max + 1, 0, size.y - 1);
    const int k0 = std::clamp(active_cells_.z_min - 1, 0, size.z - 1);
    const int k1 = std::clamp(active_cells_.z_max + 1, 0, size.z - 1);
    for ([[maybe_unused]] const int layer : IndexRange(layers)) {
      const int updated = threading::parallel_reduce(
          IndexRange(k0, k1 - k0 + 1),
          1,
          0,
          [&](const IndexRange z_range, const int init) {
            int count = init;
            for (const int k : z_range) {
              for (int j = j0; j <= j1; j++) {
                for (int i = i0; i <= i1; i++) {
                  const int index = face_index(axis, i, j, k);
                  if (valid[index] || blocked[index]) {
                    continue;
                  }
                  float sum = 0.0f;
                  int neighbors_found = 0;
                  const int neighbors[6][3] = {{i - 1, j, k},
                                               {i + 1, j, k},
                                               {i, j - 1, k},
                                               {i, j + 1, k},
                                               {i, j, k - 1},
                                               {i, j, k + 1}};
                  for (const auto &neighbor : neighbors) {
                    if (neighbor[0] < 0 || neighbor[0] >= size.x || neighbor[1] < 0 ||
                        neighbor[1] >= size.y || neighbor[2] < 0 || neighbor[2] >= size.z)
                    {
                      continue;
                    }
                    const int neighbor_index = face_index(
                        axis, neighbor[0], neighbor[1], neighbor[2]);
                    if (valid[neighbor_index] && !blocked[neighbor_index]) {
                      sum += values[neighbor_index];
                      neighbors_found++;
                    }
                  }
                  if (neighbors_found > 0) {
                    extrap_values_[index] = sum / float(neighbors_found);
                    extrap_valid_[index] = 1;
                    count++;
                  }
                }
              }
            }
            return count;
          },
          std::plus<int>());
      if (updated == 0) {
        break;
      }
      threading::parallel_for(IndexRange(k0, k1 - k0 + 1), 1, [&](const IndexRange z_range) {
        for (const int k : z_range) {
          for (int j = j0; j <= j1; j++) {
            for (int i = i0; i <= i1; i++) {
              const int index = face_index(axis, i, j, k);
              if (extrap_valid_[index] && !valid[index] && !blocked[index]) {
                values[index] = extrap_values_[index];
                valid[index] = 1;
                extrap_valid_[index] = 0;
              }
            }
          }
        }
      });
    }
  }

  void extrapolate_all(const int layers)
  {
    for (const int axis : IndexRange(3)) {
      this->extrapolate_component(axis, layers);
    }
  }

  void enforce_box_velocity_boundary()
  {
    for (const int k : IndexRange(grid_.nz)) {
      for (const int j : IndexRange(grid_.ny)) {
        grid_.u[grid_.u_index(0, j, k)] = 0.0f;
        grid_.u[grid_.u_index(grid_.nx, j, k)] = 0.0f;
      }
    }
    for (const int k : IndexRange(grid_.nz)) {
      for (const int i : IndexRange(grid_.nx)) {
        grid_.v[grid_.v_index(i, 0, k)] = 0.0f;
        grid_.v[grid_.v_index(i, grid_.ny, k)] = 0.0f;
      }
    }
    for (const int j : IndexRange(grid_.ny)) {
      for (const int i : IndexRange(grid_.nx)) {
        grid_.w[grid_.w_index(i, j, 0)] = 0.0f;
        grid_.w[grid_.w_index(i, j, grid_.nz)] = 0.0f;
      }
    }
  }

  void apply_gravity(const float dt)
  {
    if (active_cells_.is_empty()) {
      return;
    }
    for (const int axis : IndexRange(3)) {
      Array<float> &values = face_values(axis);
      const int3 size = face_size(axis);
      const int i0 = std::clamp(active_cells_.x_min, 0, size.x - 1);
      const int i1 = std::clamp(active_cells_.x_max + (axis == 0 ? 1 : 0), 0, size.x - 1);
      const int j0 = std::clamp(active_cells_.y_min, 0, size.y - 1);
      const int j1 = std::clamp(active_cells_.y_max + (axis == 1 ? 1 : 0), 0, size.y - 1);
      const int k0 = std::clamp(active_cells_.z_min, 0, size.z - 1);
      const int k1 = std::clamp(active_cells_.z_max + (axis == 2 ? 1 : 0), 0, size.z - 1);
      const float acceleration = component(settings_.gravity, axis) * dt;
      threading::parallel_for(IndexRange(k0, k1 - k0 + 1), 1, [&](const IndexRange z_range) {
        for (const int k : z_range) {
          for (int j = j0; j <= j1; j++) {
            for (int i = i0; i <= i1; i++) {
              if (grid_.face_touches_fluid(axis, i, j, k) && !this->face_is_blocked(axis, i, j, k))
              {
                values[face_index(axis, i, j, k)] += acceleration;
              }
            }
          }
        }
      });
    }
  }

  int3 face_coordinates(const int axis, const int index) const
  {
    const int3 size = this->face_size(axis);
    const int k = index / (size.x * size.y);
    const int remainder = index - k * size.x * size.y;
    return int3(remainder % size.x, remainder / size.x, k);
  }

  bool is_normal_boundary_face(const int axis, const int3 face) const
  {
    const int3 size = this->face_size(axis);
    const int coordinate = axis == 0 ? face.x : axis == 1 ? face.y : face.z;
    const int axis_size = axis == 0 ? size.x : axis == 1 ? size.y : size.z;
    return coordinate == 0 || coordinate == axis_size - 1;
  }

  bool solve_viscosity_component(const int axis,
                                 const float dt,
                                 FlipSolverStats *stats,
                                 std::string &r_error)
  {
    Array<float> &values = this->face_values(axis);
    if (viscosity_face_to_row_.size() != values.size()) {
      viscosity_face_to_row_.reinitialize(values.size());
    }
    viscosity_face_to_row_.fill(-1);
    viscosity_rows_.clear();
    const int3 size = this->face_size(axis);
    const bool limit_to_active = !active_cells_.is_empty();
    const int i0 = limit_to_active ? std::clamp(active_cells_.x_min - 1, 0, size.x - 1) : 0;
    const int i1 = limit_to_active ? std::clamp(active_cells_.x_max + 1, 0, size.x - 1) : size.x - 1;
    const int j0 = limit_to_active ? std::clamp(active_cells_.y_min - 1, 0, size.y - 1) : 0;
    const int j1 = limit_to_active ? std::clamp(active_cells_.y_max + 1, 0, size.y - 1) : size.y - 1;
    const int k0 = limit_to_active ? std::clamp(active_cells_.z_min - 1, 0, size.z - 1) : 0;
    const int k1 = limit_to_active ? std::clamp(active_cells_.z_max + 1, 0, size.z - 1) : size.z - 1;
    for (int k = k0; k <= k1; k++) {
      for (int j = j0; j <= j1; j++) {
        for (int i = i0; i <= i1; i++) {
          const int3 face(i, j, k);
          if (this->is_normal_boundary_face(axis, face) ||
              !grid_.face_touches_fluid(axis, i, j, k) || this->face_is_blocked(axis, i, j, k))
          {
            continue;
          }
          const int index = this->face_index(axis, i, j, k);
          viscosity_face_to_row_[index] = viscosity_rows_.size();
          viscosity_rows_.append(face);
        }
      }
    }
    if (viscosity_rows_.is_empty()) {
      return true;
    }

    linear_scratch_.resize_system(viscosity_rows_.size());
    if (linear_scratch_.viscosity_stencils.size() != viscosity_rows_.size()) {
      linear_scratch_.viscosity_stencils.reinitialize(viscosity_rows_.size());
    }
    MutableSpan<float> x = linear_scratch_.solution.as_mutable_span();
    MutableSpan<float> rhs = linear_scratch_.rhs.as_mutable_span();
    MutableSpan<float> inverse_diagonal = linear_scratch_.inverse_diagonal.as_mutable_span();
    MutableSpan<ViscosityStencil> stencils = linear_scratch_.viscosity_stencils.as_mutable_span();
    const float coefficients[3] = {1.0f / (grid_.dx.x * grid_.dx.x),
                                   1.0f / (grid_.dx.y * grid_.dx.y),
                                   1.0f / (grid_.dx.z * grid_.dx.z)};
    auto face_inside = [&](const int3 face) {
      return face.x >= 0 && face.x < size.x && face.y >= 0 && face.y < size.y && face.z >= 0 &&
             face.z < size.z;
    };
    for (const int row : viscosity_rows_.index_range()) {
      const int3 face = viscosity_rows_[row];
      const int index = this->face_index(axis, face.x, face.y, face.z);
      x[row] = values[index];
      rhs[row] = values[index];
      ViscosityStencil stencil;
      for (const int neighbor_axis : IndexRange(3)) {
        for (const int direction : {-1, 1}) {
          int3 neighbor = face;
          if (neighbor_axis == 0) {
            neighbor.x += direction;
          }
          else if (neighbor_axis == 1) {
            neighbor.y += direction;
          }
          else {
            neighbor.z += direction;
          }
          if (!face_inside(neighbor)) {
            continue;
          }
          const int neighbor_index = this->face_index(axis, neighbor.x, neighbor.y, neighbor.z);
          const float coefficient = dt * settings_.viscosity * coefficients[neighbor_axis];
          const int slot = neighbor_axis * 2 + (direction > 0 ? 1 : 0);
          const int neighbor_row = viscosity_face_to_row_[neighbor_index];
          if (neighbor_row >= 0) {
            stencil.diagonal += coefficient;
            stencil.rows[slot] = neighbor_row;
            stencil.coeffs[slot] = coefficient;
          }
          else if (this->face_is_blocked(axis, neighbor.x, neighbor.y, neighbor.z)) {
            stencil.diagonal += coefficient;
            rhs[row] += coefficient * face_solid_velocity(axis)[neighbor_index];
          }
          else if (neighbor_axis == axis && this->is_normal_boundary_face(axis, neighbor)) {
            stencil.diagonal += coefficient;
          }
        }
      }
      inverse_diagonal[row] = 1.0f / stencil.diagonal;
      stencils[row] = stencil;
    }

    const PcgResult result = pcg_solve(
        rhs,
        x,
        inverse_diagonal,
        [&](const Span<float> input, MutableSpan<float> output) {
          const Span<ViscosityStencil> apply_stencils =
              linear_scratch_.viscosity_stencils.as_span();
          threading::parallel_for(viscosity_rows_.index_range(), 4096, [&](const IndexRange range) {
            for (const int row : range) {
              const ViscosityStencil &stencil = apply_stencils[row];
              float value = stencil.diagonal * input[row];
              for (const int slot : IndexRange(6)) {
                const int neighbor_row = stencil.rows[slot];
                if (neighbor_row >= 0) {
                  value -= stencil.coeffs[slot] * input[neighbor_row];
                }
              }
              output[row] = value;
            }
          });
        },
        settings_.viscosity_max_iterations,
        settings_.viscosity_tolerance,
        linear_scratch_);
    if (stats != nullptr) {
      stats->viscosity_iterations += result.iterations;
      stats->viscosity_relative_residual = std::max(stats->viscosity_relative_residual,
                                                    result.relative_residual);
      stats->viscosity_converged = stats->viscosity_converged && result.converged;
      stats->viscosity_breakdown = stats->viscosity_breakdown || result.breakdown;
    }
    if (result.breakdown) {
      r_error = "FLIP viscosity solver broke down";
      return false;
    }
    for (const int row : viscosity_rows_.index_range()) {
      const int3 face = viscosity_rows_[row];
      values[this->face_index(axis, face.x, face.y, face.z)] = x[row];
    }
    return true;
  }

  bool solve_implicit_viscosity(const float dt, FlipSolverStats *stats, std::string &r_error)
  {
    if (settings_.viscosity <= 1.0e-12f) {
      return true;
    }
    for (const int axis : IndexRange(3)) {
      if (!this->solve_viscosity_component(axis, dt, stats, r_error)) {
        return false;
      }
    }
    return true;
  }

  float cell_divergence(const int i, const int j, const int k) const
  {
    return (grid_.u[grid_.u_index(i + 1, j, k)] - grid_.u[grid_.u_index(i, j, k)]) / grid_.dx.x +
           (grid_.v[grid_.v_index(i, j + 1, k)] - grid_.v[grid_.v_index(i, j, k)]) / grid_.dx.y +
           (grid_.w[grid_.w_index(i, j, k + 1)] - grid_.w[grid_.w_index(i, j, k)]) / grid_.dx.z;
  }

  void compute_divergence_stats(const bool before_projection, FlipSolverStats *stats) const
  {
    if (stats == nullptr) {
      return;
    }
    double sum = 0.0;
    double maximum = 0.0;
    int count = 0;
    if (!active_cells_.is_empty()) {
      for (int k = active_cells_.z_min; k <= active_cells_.z_max; k++) {
        for (int j = active_cells_.y_min; j <= active_cells_.y_max; j++) {
          for (int i = active_cells_.x_min; i <= active_cells_.x_max; i++) {
            if (grid_.cell_type_at(i, j, k) != FlipCellType::Fluid) {
              continue;
            }
            const double divergence = std::abs(double(this->cell_divergence(i, j, k)));
            sum += divergence;
            maximum = std::max(maximum, divergence);
            count++;
          }
        }
      }
    }
    const double mean = count > 0 ? sum / double(count) : 0.0;
    if (before_projection) {
      stats->mean_div_before = mean;
      stats->max_div_before = maximum;
    }
    else {
      stats->mean_div_after = mean;
      stats->max_div_after = maximum;
    }
  }

  void build_fluid_rows()
  {
    grid_.fluid_to_row.fill(-1);
    grid_.row_to_cell.clear();
    if (active_cells_.is_empty()) {
      return;
    }
    for (int k = active_cells_.z_min; k <= active_cells_.z_max; k++) {
      for (int j = active_cells_.y_min; j <= active_cells_.y_max; j++) {
        for (int i = active_cells_.x_min; i <= active_cells_.x_max; i++) {
          if (grid_.cell_type_at(i, j, k) != FlipCellType::Fluid) {
            continue;
          }
          grid_.fluid_to_row[grid_.cell_index(i, j, k)] = grid_.row_to_cell.size();
          grid_.row_to_cell.append(int3(i, j, k));
        }
      }
    }
  }

  Array<uint8_t> build_pressure_gauges() const
  {
    Array<uint8_t> gauges(grid_.row_to_cell.size(), 0);
    Array<uint8_t> visited(grid_.row_to_cell.size(), 0);
    Vector<int> stack;
    for (const int start_row : grid_.row_to_cell.index_range()) {
      if (visited[start_row]) {
        continue;
      }
      stack.clear();
      stack.append(start_row);
      visited[start_row] = 1;
      bool touches_air = false;
      while (!stack.is_empty()) {
        const int row = stack.pop_last();
        const int3 cell = grid_.row_to_cell[row];
        for (const int axis : IndexRange(3)) {
          for (const int direction : {-1, 1}) {
            if (this->cell_neighbor_face_is_blocked(cell, axis, direction)) {
              continue;
            }
            int3 neighbor = cell;
            if (axis == 0) {
              neighbor.x += direction;
            }
            else if (axis == 1) {
              neighbor.y += direction;
            }
            else {
              neighbor.z += direction;
            }
            const FlipCellType type = grid_.cell_type_at(neighbor.x, neighbor.y, neighbor.z);
            if (type == FlipCellType::Air) {
              touches_air = true;
            }
            if (type != FlipCellType::Fluid) {
              continue;
            }
            const int neighbor_row =
                grid_.fluid_to_row[grid_.cell_index(neighbor.x, neighbor.y, neighbor.z)];
            if (!visited[neighbor_row]) {
              visited[neighbor_row] = 1;
              stack.append(neighbor_row);
            }
          }
        }
      }
      if (!touches_air) {
        gauges[start_row] = 1;
      }
    }
    return gauges;
  }

  void build_pressure_stencils(const Span<uint8_t> gauges)
  {
    const float coefficients[3] = {1.0f / (grid_.dx.x * grid_.dx.x),
                                   1.0f / (grid_.dx.y * grid_.dx.y),
                                   1.0f / (grid_.dx.z * grid_.dx.z)};
    MutableSpan<PressureStencil> stencils = linear_scratch_.pressure_stencils.as_mutable_span();
    threading::parallel_for(grid_.row_to_cell.index_range(), 4096, [&](const IndexRange range) {
      for (const int row : range) {
        PressureStencil stencil;
        if (gauges[row]) {
          stencil.diagonal = 1.0f;
          stencils[row] = stencil;
          continue;
        }
        const int3 cell = grid_.row_to_cell[row];
        for (const int axis : IndexRange(3)) {
          for (const int direction : {-1, 1}) {
            if (this->cell_neighbor_face_is_blocked(cell, axis, direction)) {
              continue;
            }
            int3 neighbor = cell;
            if (axis == 0) {
              neighbor.x += direction;
            }
            else if (axis == 1) {
              neighbor.y += direction;
            }
            else {
              neighbor.z += direction;
            }
            const FlipCellType type = grid_.cell_type_at(neighbor.x, neighbor.y, neighbor.z);
            if (type == FlipCellType::Solid) {
              continue;
            }
            stencil.diagonal += coefficients[axis];
            if (type != FlipCellType::Fluid) {
              continue;
            }
            const int neighbor_row =
                grid_.fluid_to_row[grid_.cell_index(neighbor.x, neighbor.y, neighbor.z)];
            if (gauges[neighbor_row]) {
              continue;
            }
            int *slot = axis == 0 ? (direction < 0 ? &stencil.xm : &stencil.xp) :
                        axis == 1 ? (direction < 0 ? &stencil.ym : &stencil.yp) :
                                    (direction < 0 ? &stencil.zm : &stencil.zp);
            *slot = neighbor_row;
          }
        }
        if (stencil.diagonal <= solver_epsilon) {
          stencil.diagonal = 1.0f;
        }
        stencils[row] = stencil;
      }
    });
  }

  void apply_pressure_matrix(const Span<float> x, MutableSpan<float> y) const
  {
    const float cx = 1.0f / (grid_.dx.x * grid_.dx.x);
    const float cy = 1.0f / (grid_.dx.y * grid_.dx.y);
    const float cz = 1.0f / (grid_.dx.z * grid_.dx.z);
    const Span<PressureStencil> stencils = linear_scratch_.pressure_stencils.as_span();
    threading::parallel_for(grid_.row_to_cell.index_range(), 4096, [&](const IndexRange range) {
      for (const int row : range) {
        const PressureStencil &stencil = stencils[row];
        float value = stencil.diagonal * x[row];
        if (stencil.xm >= 0) {
          value -= cx * x[stencil.xm];
        }
        if (stencil.xp >= 0) {
          value -= cx * x[stencil.xp];
        }
        if (stencil.ym >= 0) {
          value -= cy * x[stencil.ym];
        }
        if (stencil.yp >= 0) {
          value -= cy * x[stencil.yp];
        }
        if (stencil.zm >= 0) {
          value -= cz * x[stencil.zm];
        }
        if (stencil.zp >= 0) {
          value -= cz * x[stencil.zp];
        }
        y[row] = value;
      }
    });
  }

  bool solve_pressure(const float dt, FlipSolverStats *stats, std::string &r_error)
  {
    const bool trace_pressure = std::getenv("BLENDER_FLIP_PRESSURE_TRACE") != nullptr;
    Array<uint8_t> gauges;
    Array<float> pressure;
    Array<float> pressure_initial;
    Array<float> rhs;
    Array<float> inverse_diagonal;
    std::unique_ptr<PressureMultigrid> multigrid;
    {
      ScopedPerformanceTimer timer(stats ? &stats->performance.pressure_build_ms : nullptr);
      this->build_fluid_rows();
      const int rows_num = grid_.row_to_cell.size();
      if (rows_num == 0) {
        return true;
      }
      gauges = this->build_pressure_gauges();
      pressure.reinitialize(rows_num);
      for (const int row : pressure.index_range()) {
        const int3 cell = grid_.row_to_cell[row];
        const int cell_index = grid_.cell_index(cell.x, cell.y, cell.z);
        pressure[row] = gauges[row] || cell_index >= pressure_warm_.size() ?
                            0.0f :
                            pressure_warm_[cell_index];
      }
      rhs.reinitialize(rows_num);
      rhs.fill(0.0f);
      Array<float> diagonal(rows_num, 1.0f);
      inverse_diagonal.reinitialize(rows_num);
      inverse_diagonal.fill(1.0f);

      for (const int row : grid_.row_to_cell.index_range()) {
        const int3 cell = grid_.row_to_cell[row];
        rhs[row] = -settings_.density / dt * cell_divergence(cell.x, cell.y, cell.z);
      }
      for (const int row : gauges.index_range()) {
        if (gauges[row]) {
          rhs[row] = 0.0f;
        }
      }
      if (linear_scratch_.pressure_stencils.size() != rows_num) {
        linear_scratch_.pressure_stencils.reinitialize(rows_num);
      }
      this->build_pressure_stencils(gauges.as_span());
      const Span<PressureStencil> stencils = linear_scratch_.pressure_stencils.as_span();
      for (const int row : diagonal.index_range()) {
        diagonal[row] = stencils[row].diagonal;
        inverse_diagonal[row] = 1.0f / std::max(diagonal[row], solver_epsilon);
      }
      if (settings_.use_multigrid_preconditioner ||
          std::getenv("BLENDER_FLIP_MG_PROTOTYPE") != nullptr)
      {
        multigrid = std::make_unique<PressureMultigrid>(
            stencils, grid_.row_to_cell, int3(grid_.nx, grid_.ny, grid_.nz), grid_.dx);
        pressure_initial.reinitialize(rows_num);
        pressure_initial.as_mutable_span().copy_from(pressure.as_span());
      }
    }
    const double rhs_norm = trace_pressure ? std::sqrt(dot_product(rhs, rhs)) : 0.0;
    std::function<void(Span<float>, MutableSpan<float>)> apply_custom_M;
    if (multigrid) {
      apply_custom_M = [&](const Span<float> residual, MutableSpan<float> correction) {
        multigrid->apply(residual, correction);
      };
    }
    PcgResult result;
    bool multigrid_fallback = false;
    {
      ScopedPerformanceTimer timer(stats ? &stats->performance.pressure_solve_ms : nullptr);
      result = pcg_solve(
          rhs.as_span(),
          pressure.as_mutable_span(),
          inverse_diagonal.as_span(),
          [&](const Span<float> x, MutableSpan<float> y) {
            this->apply_pressure_matrix(x, y);
          },
           settings_.pressure_max_iterations,
           settings_.pressure_tolerance,
           linear_scratch_,
           apply_custom_M);
      if (multigrid && (!result.converged || result.breakdown)) {
        multigrid_fallback = true;
        pressure.as_mutable_span().copy_from(pressure_initial.as_span());
        result = pcg_solve(
            rhs.as_span(),
            pressure.as_mutable_span(),
            inverse_diagonal.as_span(),
            [&](const Span<float> x, MutableSpan<float> y) {
              this->apply_pressure_matrix(x, y);
            },
            settings_.pressure_max_iterations,
            settings_.pressure_tolerance,
            linear_scratch_);
      }
    }
    if (stats != nullptr) {
      stats->pressure_iterations += result.iterations;
      stats->pressure_relative_residual = result.relative_residual;
      stats->pressure_converged = stats->pressure_converged && result.converged;
      stats->pressure_breakdown = stats->pressure_breakdown || result.breakdown;
    }
    if (trace_pressure) {
      std::fprintf(stderr,
                   "FLIP_PRESSURE_TRACE solve=%d rows=%lld warm=%d mg=%d mg_fallback=%d initial_rel=%.6g "
                   "final_rel=%.6g iterations=%d\n",
                   pressure_solve_count_,
                   static_cast<long long>(grid_.row_to_cell.size()),
                   int(pressure_solve_count_ > 0),
                   int(bool(multigrid)),
                   int(multigrid_fallback),
                   result.initial_residual / std::max(rhs_norm, double(solver_epsilon)),
                   result.relative_residual,
                   result.iterations);
    }
    pressure_solve_count_++;
    if (result.breakdown) {
      r_error = "FLIP pressure solver broke down";
      return false;
    }

    grid_.pressure.fill(0.0f);
    for (const int row : grid_.row_to_cell.index_range()) {
      const int3 cell = grid_.row_to_cell[row];
      grid_.pressure[grid_.cell_index(cell.x, cell.y, cell.z)] = pressure[row];
    }
    return true;
  }

  float pressure_at(const int i, const int j, const int k) const
  {
    return grid_.cell_type_at(i, j, k) == FlipCellType::Fluid ?
               grid_.pressure[grid_.cell_index(i, j, k)] :
               0.0f;
  }

  void project_pressure(const float dt)
  {
    const float scale_x = dt / (settings_.density * grid_.dx.x);
    for (const int k : IndexRange(grid_.nz)) {
      for (const int j : IndexRange(grid_.ny)) {
        for (const int i : IndexRange(1, grid_.nx - 1)) {
          if (grid_.face_touches_fluid(0, i, j, k) && !this->face_is_blocked(0, i, j, k)) {
            grid_.u[grid_.u_index(i, j, k)] -= scale_x *
                                               (pressure_at(i, j, k) - pressure_at(i - 1, j, k));
          }
        }
      }
    }
    const float scale_y = dt / (settings_.density * grid_.dx.y);
    for (const int k : IndexRange(grid_.nz)) {
      for (const int j : IndexRange(1, grid_.ny - 1)) {
        for (const int i : IndexRange(grid_.nx)) {
          if (grid_.face_touches_fluid(1, i, j, k) && !this->face_is_blocked(1, i, j, k)) {
            grid_.v[grid_.v_index(i, j, k)] -= scale_y *
                                               (pressure_at(i, j, k) - pressure_at(i, j - 1, k));
          }
        }
      }
    }
    const float scale_z = dt / (settings_.density * grid_.dx.z);
    for (const int k : IndexRange(1, grid_.nz - 1)) {
      for (const int j : IndexRange(grid_.ny)) {
        for (const int i : IndexRange(grid_.nx)) {
          if (grid_.face_touches_fluid(2, i, j, k) && !this->face_is_blocked(2, i, j, k)) {
            grid_.w[grid_.w_index(i, j, k)] -= scale_z *
                                               (pressure_at(i, j, k) - pressure_at(i, j, k - 1));
          }
        }
      }
    }
  }

  float sample_component(const Span<float> values, const float3 grid_position, const int axis) const
  {
    float3 sample_position = grid_position;
    if (axis != 0) {
      sample_position.x -= 0.5f;
    }
    if (axis != 1) {
      sample_position.y -= 0.5f;
    }
    if (axis != 2) {
      sample_position.z -= 0.5f;
    }
    const int3 size = face_size(axis);
    sample_position.x = std::clamp(sample_position.x, 0.0f, float(size.x - 1));
    sample_position.y = std::clamp(sample_position.y, 0.0f, float(size.y - 1));
    sample_position.z = std::clamp(sample_position.z, 0.0f, float(size.z - 1));
    const int i0 = int(std::floor(sample_position.x));
    const int j0 = int(std::floor(sample_position.y));
    const int k0 = int(std::floor(sample_position.z));
    const int i1 = std::min(i0 + 1, size.x - 1);
    const int j1 = std::min(j0 + 1, size.y - 1);
    const int k1 = std::min(k0 + 1, size.z - 1);
    const float fx = sample_position.x - i0;
    const float fy = sample_position.y - j0;
    const float fz = sample_position.z - k0;
    float result = 0.0f;
    for (const int z_offset : IndexRange(2)) {
      const int k = z_offset ? k1 : k0;
      const float wz = z_offset ? fz : 1.0f - fz;
      for (const int y_offset : IndexRange(2)) {
        const int j = y_offset ? j1 : j0;
        const float wy = y_offset ? fy : 1.0f - fy;
        for (const int x_offset : IndexRange(2)) {
          const int i = x_offset ? i1 : i0;
          const float wx = x_offset ? fx : 1.0f - fx;
          result += values[face_index(axis, i, j, k)] * wx * wy * wz;
        }
      }
    }
    return result;
  }

  float3 sample_velocity_grid(const float3 grid_position) const
  {
    return float3(sample_component(grid_.u.as_span(), grid_position, 0),
                  sample_component(grid_.v.as_span(), grid_position, 1),
                  sample_component(grid_.w.as_span(), grid_position, 2));
  }

  float3 sample_old_velocity_grid(const float3 grid_position) const
  {
    return float3(sample_component(grid_.u_old.as_span(), grid_position, 0),
                  sample_component(grid_.v_old.as_span(), grid_position, 1),
                  sample_component(grid_.w_old.as_span(), grid_position, 2));
  }

  void constrain_particle(float3 &position, float3 &velocity) const
  {
    const float3 center = (settings_.domain_min + settings_.domain_max) * 0.5f;
    const float3 lower = settings_.domain_min + float3(padding_);
    const float3 upper = settings_.domain_max - float3(padding_);
    if (!is_finite(position)) {
      position = center;
    }
    if (!is_finite(velocity)) {
      velocity = float3(0.0f);
    }
    for (const int axis : IndexRange(3)) {
      const float p = component(position, axis);
      if (p < component(lower, axis)) {
        set_component(position, axis, component(lower, axis));
        if (component(velocity, axis) < 0.0f) {
          set_component(velocity, axis, 0.0f);
        }
      }
      else if (p > component(upper, axis)) {
        set_component(position, axis, component(upper, axis));
        if (component(velocity, axis) > 0.0f) {
          set_component(velocity, axis, 0.0f);
        }
      }
    }
  }

  void transfer_advect_and_collide(FlipParticleData &particles,
                                   const float dt,
                                   FlipSolverStats *stats) const
  {
    const bool collide = collider_query_ != nullptr && collider_query_->is_built();
    const float margin = 0.01f * std::min({grid_.dx.x, grid_.dx.y, grid_.dx.z});
    const int projected = threading::parallel_reduce(
        particles.positions.index_range(),
        1024,
        0,
        [&](const IndexRange range, const int init) {
          int count = init;
          for (const int index : range) {
            float3 &position = particles.positions[index];
            float3 &velocity = particles.velocities[index];
            const float3 old_position = position;
            const float3 grid_position = particle_grid_positions_[index];
            const float3 pic_velocity = sample_velocity_grid(grid_position);
            const float3 grid_delta = pic_velocity - sample_old_velocity_grid(grid_position);
            const float3 flip_velocity = velocity + grid_delta;
            velocity = pic_velocity * (1.0f - settings_.flip_ratio) +
                       flip_velocity * settings_.flip_ratio;
            if (!is_finite(velocity)) {
              velocity = float3(0.0f);
            }
            const float3 midpoint = position + pic_velocity * (0.5f * dt);
            position += sample_velocity_grid(this->position_to_grid_coordinates(midpoint)) * dt;
            if (collide) {
              count += resolve_flip_particle_collision(*collider_query_,
                                                       old_position,
                                                       position,
                                                       velocity,
                                                       margin,
                                                       collider_friction_);
            }
            this->constrain_particle(position, velocity);
          }
          return count;
        },
        std::plus<int>());
    if (stats != nullptr) {
      stats->collider_particles_projected += projected;
    }
  }
};

static bool compute_grid_resolution(
    const FlipSolverSettings &settings, int &r_nx, int &r_ny, int &r_nz, std::string &r_error)
{
  const float3 extent = settings.domain_max - settings.domain_min;
  const double dimensions[3] = {std::ceil(double(extent.x) / settings.target_cell_size),
                                std::ceil(double(extent.y) / settings.target_cell_size),
                                std::ceil(double(extent.z) / settings.target_cell_size)};
  for (const double dimension : dimensions) {
    if (!std::isfinite(dimension) || dimension < 1.0 ||
        dimension > double(std::numeric_limits<int>::max()))
    {
      r_error = "FLIP grid resolution is invalid";
      return false;
    }
  }
  r_nx = std::max(1, int(dimensions[0]));
  r_ny = std::max(1, int(dimensions[1]));
  r_nz = std::max(1, int(dimensions[2]));
  const int64_t cells = int64_t(r_nx) * int64_t(r_ny) * int64_t(r_nz);
  if (cells > max_grid_cells) {
    r_error = "FLIP grid resolution is too large";
    return false;
  }
  return true;
}

}  // namespace

bool flip_solver_step(const FlipSolverSettings &settings,
                      FlipParticleData &particles,
                      const float frame_dt,
                      const FlipColliderInput *collider,
                      FlipSolverStats *stats,
                      std::string &r_error)
{
  ScopedPerformanceTimer total_timer(stats ? &stats->performance.total_ms : nullptr);
  if (particles.positions.size() != particles.velocities.size() ||
      particles.positions.size() != particles.ids.size() ||
      particles.positions.size() != particles.source_indices.size())
  {
    r_error = "FLIP particle arrays have inconsistent sizes";
    return false;
  }
  if (particles.positions.is_empty() || frame_dt <= 0.0f) {
    if (stats != nullptr) {
      stats->particle_count = particles.positions.size();
      stats->internal_substeps = 0;
    }
    return true;
  }

  FlipSolverSettings effective_settings = settings;
  if (settings.resolution_mode == FlipResolutionMode::ParticleRelative) {
    effective_settings.target_cell_size = settings.particle_spacing * settings.grid_scale;
  }
  if (!std::isfinite(effective_settings.target_cell_size) ||
      effective_settings.target_cell_size <= 0.0f)
  {
    r_error = "FLIP cell size is invalid";
    return false;
  }
  int nx, ny, nz;
  if (!compute_grid_resolution(effective_settings, nx, ny, nz, r_error)) {
    return false;
  }
  PreparedFlipCollider prepared_collider;
  {
    ScopedPerformanceTimer timer(stats ? &stats->performance.collider_ms : nullptr);
    if (collider != nullptr &&
        !prepare_flip_collider(*collider, frame_dt, prepared_collider, stats, r_error))
    {
      return false;
    }
  }
  FlipSolverCore solver(effective_settings, nx, ny, nz);
  const int min_substeps = std::clamp(settings.min_substeps, 1, 128);
  const int max_substeps = std::clamp(settings.max_substeps, min_substeps, 128);
  int required_substeps = min_substeps;
  int collider_required_substeps = min_substeps;
  const float min_dx = std::min({(settings.domain_max.x - settings.domain_min.x) / float(nx),
                                 (settings.domain_max.y - settings.domain_min.y) / float(ny),
                                 (settings.domain_max.z - settings.domain_min.z) / float(nz)});
  if (settings.adaptive_substeps) {
    float maximum_speed = 0.0f;
    for (const float3 velocity : particles.velocities) {
      maximum_speed = std::max(
          maximum_speed,
          std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z));
    }
    const float gravity_length = std::sqrt(settings.gravity.x * settings.gravity.x +
                                           settings.gravity.y * settings.gravity.y +
                                           settings.gravity.z * settings.gravity.z);
    const float estimated_speed = maximum_speed + gravity_length * frame_dt;
    if (estimated_speed > 1.0e-12f) {
      const float cfl_dt = std::max(settings.cfl_number, 1.0e-4f) * min_dx / estimated_speed;
      required_substeps = std::max(min_substeps, int(std::ceil(frame_dt / cfl_dt)));
    }
    if (prepared_collider.max_speed > 1.0e-12f) {
      const float collider_dt = 0.5f * min_dx / prepared_collider.max_speed;
      collider_required_substeps = std::max(
          min_substeps, int(std::ceil(frame_dt / std::max(collider_dt, 1.0e-12f))));
      required_substeps = std::max(required_substeps, collider_required_substeps);
    }
  }
  if (stats != nullptr) {
    stats->grid_x = nx;
    stats->grid_y = ny;
    stats->grid_z = nz;
    stats->cfl_clamped = required_substeps > max_substeps;
    stats->collider_cfl_clamped = collider_required_substeps > max_substeps;
    stats->collider_thickness_large = prepared_collider.thickness > 0.5f * min_dx;
  }
  const int substeps = std::clamp(required_substeps, min_substeps, max_substeps);
  const float substep_dt = frame_dt / float(substeps);
  FlipColliderQuery collider_query;
  if (prepared_collider.enabled && !prepared_collider.animated) {
    ScopedPerformanceTimer timer(stats ? &stats->performance.collider_ms : nullptr);
    if (!collider_query.build(prepared_collider, 1.0f, r_error)) {
      return false;
    }
  }
  for (const int substep : IndexRange(substeps)) {
    const FlipColliderQuery *query = nullptr;
    if (prepared_collider.enabled) {
      if (prepared_collider.animated) {
        const float alpha = (float(substep) + 0.5f) / float(substeps);
        ScopedPerformanceTimer timer(stats ? &stats->performance.collider_ms : nullptr);
        if (!collider_query.build(prepared_collider, alpha, r_error)) {
          return false;
        }
      }
      query = &collider_query;
    }
    const bool rebuild_collider_grid = substep == 0 || prepared_collider.animated;
    if (!solver.substep(particles,
                        substep_dt,
                        query,
                        prepared_collider.friction,
                        rebuild_collider_grid,
                        prepared_collider.animated,
                        stats,
                        r_error))
    {
      return false;
    }
  }
  if (stats != nullptr) {
    stats->particle_count = particles.positions.size();
    stats->internal_substeps = substeps;
  }
  return true;
}

}  // namespace blender::geometry
