/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <cstdint>
#include <string>

#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

namespace blender {
struct Mesh;
}

namespace blender::geometry {

enum class FlipResolutionMode : int8_t {
  Manual = 0,
  ParticleRelative = 1,
};

enum class FlipCollisionMode : int8_t {
  Disabled = 0,
  Static = 1,
  Animated = 2,
};

enum class FlipColliderMotionSource : int8_t {
  PreviousGeometry = 0,
  VelocityAttribute = 1,
};

struct FlipCollisionSettings {
  FlipCollisionMode mode = FlipCollisionMode::Disabled;
  FlipColliderMotionSource motion_source = FlipColliderMotionSource::PreviousGeometry;
  float thickness = 0.0f;
  float friction = 0.0f;
};

struct FlipColliderInput {
  const Mesh *current_mesh = nullptr;
  const Mesh *previous_mesh = nullptr;
  std::string velocity_attribute = "velocity";
  FlipCollisionSettings settings;
};

struct FlipSolverSettings {
  float3 domain_min = float3(-1.0f);
  float3 domain_max = float3(1.0f);

  FlipResolutionMode resolution_mode = FlipResolutionMode::Manual;
  float target_cell_size = 0.1f;
  float particle_spacing = 0.05f;
  float grid_scale = 2.0f;

  float3 gravity = float3(0.0f, 0.0f, -9.81f);
  float flip_ratio = 0.95f;
  float density = 1000.0f;

  bool adaptive_substeps = true;
  float cfl_number = 1.0f;
  int min_substeps = 1;
  int max_substeps = 16;

  bool reseeding = true;
  int target_particles_per_cell = 8;
  int min_particles_per_cell = 4;
  int max_particles_per_cell = 12;
  float classification_radius_scale = 0.75f;

  int pressure_max_iterations = 500;
  float pressure_tolerance = 1.0e-5f;

  float viscosity = 0.0f;
  int viscosity_max_iterations = 200;
  float viscosity_tolerance = 1.0e-5f;

  int extrapolation_layers = 3;
  float boundary_padding = 0.01f;
};

struct FlipParticleData {
  Vector<float3> positions;
  Vector<float3> velocities;
  Vector<int64_t> ids;

  /** Input point inherited by this output particle for point-domain attribute gathering. */
  Vector<int> source_indices;
};

struct FlipPerformanceStats {
  double total_ms = 0.0;

  double particle_binning_ms = 0.0;
  double reseeding_ms = 0.0;
  double collider_ms = 0.0;

  double p2g_ms = 0.0;
  double extrapolation_ms = 0.0;
  double gravity_ms = 0.0;
  double viscosity_ms = 0.0;

  double pressure_build_ms = 0.0;
  double pressure_solve_ms = 0.0;
  double pressure_project_ms = 0.0;

  double g2p_ms = 0.0;
  double advection_ms = 0.0;
  double output_geometry_ms = 0.0;
};

struct FlipSolverStats {
  FlipPerformanceStats performance;

  int particle_count = 0;
  int fluid_cells = 0;
  float average_particles_per_fluid_cell = 0.0f;
  int empty_fluid_cells = 0;
  int sparse_fluid_cells = 0;
  int max_particles_in_cell = 0;
  int particles_seeded = 0;
  int particles_culled = 0;

  int internal_substeps = 0;

  int pressure_iterations = 0;
  double pressure_relative_residual = 0.0;
  double mean_div_before = 0.0;
  double mean_div_after = 0.0;
  double max_div_before = 0.0;
  double max_div_after = 0.0;

  int viscosity_iterations = 0;
  double viscosity_relative_residual = 0.0;

  bool pressure_converged = true;
  bool viscosity_converged = true;
  bool cfl_clamped = false;
  bool particle_undersampled = false;
  bool pressure_breakdown = false;
  bool viscosity_breakdown = false;

  int grid_x = 0;
  int grid_y = 0;
  int grid_z = 0;

  float collider_max_speed = 0.0f;
  int collider_triangles = 0;
  int collider_particles_projected = 0;
  bool collider_cfl_clamped = false;
  bool collider_missing = false;
  bool collider_previous_missing = false;
  bool collider_topology_mismatch = false;
  bool collider_velocity_missing = false;
  bool collider_velocity_invalid_type = false;
  bool collider_non_finite = false;
  bool collider_thickness_large = false;
};

/**
 * Advance point-cloud marker particles by one frame.
 *
 * The solver is deterministic and owns no persistent/global state. Simulation Zone feedback is
 * responsible for persistence. Returns false and leaves a diagnostic in `r_error` on invalid
 * settings, allocation guards, or a numerical breakdown.
 */
bool flip_solver_step(const FlipSolverSettings &settings,
                      FlipParticleData &particles,
                      float frame_dt,
                      const FlipColliderInput *collider,
                      FlipSolverStats *stats,
                      std::string &r_error);

}  // namespace blender::geometry
