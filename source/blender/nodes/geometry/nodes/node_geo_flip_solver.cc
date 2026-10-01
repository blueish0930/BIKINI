/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

#include "BKE_attribute.hh"
#include "BKE_attribute_filters.hh"
#include "BKE_pointcloud.hh"

#include "BLI_timeit.hh"

#include "DNA_pointcloud_types.h"

#include "GEO_flip_solver.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_flip_solver_cc {

static const EnumPropertyItem resolution_mode_items[] = {
    {int(geometry::FlipResolutionMode::Manual),
     "MANUAL",
     0,
     N_("Manual"),
     N_("Set the grid cell size directly")},
    {int(geometry::FlipResolutionMode::ParticleRelative),
     "PARTICLE_RELATIVE",
     0,
     N_("Particle Relative"),
     N_("Derive cell size from particle spacing and grid scale")},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem collision_mode_items[] = {
    {int(geometry::FlipCollisionMode::Disabled),
     "DISABLED",
     0,
     N_("Disabled"),
     N_("Do not use mesh colliders")},
    {int(geometry::FlipCollisionMode::Static),
     "STATIC",
     0,
     N_("Static"),
     N_("Use a stationary closed mesh collider")},
    {int(geometry::FlipCollisionMode::Animated),
     "ANIMATED",
     0,
     N_("Animated"),
     N_("Interpolate an animated or deforming closed mesh collider")},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem collider_motion_source_items[] = {
    {int(geometry::FlipColliderMotionSource::PreviousGeometry),
     "PREVIOUS_GEOMETRY",
     0,
     N_("Previous Geometry"),
     N_("Derive per-vertex motion from matching previous-frame geometry")},
    {int(geometry::FlipColliderMotionSource::VelocityAttribute),
     "VELOCITY_ATTRIBUTE",
     0,
     N_("Velocity Attribute"),
     N_("Read per-vertex collider velocity from a vector attribute")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Particles"_ustr)
      .supported_type(GeometryComponent::Type::PointCloud)
      .description("Point cloud particles carrying velocity and stable id attributes");
  b.add_output<decl::Geometry>("Particles"_ustr).propagate_all_geometry().align_with_previous();

  {
    auto &panel = b.add_panel("Domain"_ustr).default_closed(true);
    panel.add_input<decl::Vector>("Domain Min"_ustr).default_value(float3(-1.0f));
    panel.add_input<decl::Vector>("Domain Max"_ustr).default_value(float3(1.0f));
    panel.add_input<decl::Float>("Boundary Padding"_ustr).default_value(0.01f).min(0.0f);
  }
  {
    auto &panel = b.add_panel("Time Stepping"_ustr).default_closed(true);
    panel.add_input<decl::Float>("Delta Time"_ustr).default_value(1.0f / 24.0f).min(0.0f);
    panel.add_input<decl::Bool>("Use Adaptive Substeps"_ustr).default_value(true);
    panel.add_input<decl::Float>("CFL Number"_ustr).default_value(1.0f).min(0.01f).max(10.0f);
    panel.add_input<decl::Int>("Substeps"_ustr)
        .default_value(2)
        .min(1)
        .max(128)
        .description("Fixed substeps when adaptive stepping is disabled")
        .usage_by_bool("Use Adaptive Substeps"_ustr, false);
    panel.add_input<decl::Int>("Min Substeps"_ustr)
        .default_value(1)
        .min(1)
        .max(128)
        .usage_by_bool("Use Adaptive Substeps"_ustr, true);
    panel.add_input<decl::Int>("Max Substeps"_ustr)
        .default_value(16)
        .min(1)
        .max(128)
        .usage_by_bool("Use Adaptive Substeps"_ustr, true);
  }
  {
    auto &panel = b.add_panel("Resolution"_ustr).default_closed(true);
    panel.add_input<decl::Menu>("Resolution Mode"_ustr)
        .static_items(resolution_mode_items)
        .default_value(MenuValue(geometry::FlipResolutionMode::Manual))
        .optional_label();
    panel.add_input<decl::Float>("Cell Size"_ustr)
        .default_value(0.1f)
        .min(1.0e-4f)
        .subtype(PROP_DISTANCE)
        .usage_by_menu("Resolution Mode"_ustr, int(geometry::FlipResolutionMode::Manual));
    panel.add_input<decl::Float>("Particle Spacing"_ustr)
        .default_value(0.05f)
        .min(1.0e-5f)
        .subtype(PROP_DISTANCE)
        .usage_by_menu("Resolution Mode"_ustr, int(geometry::FlipResolutionMode::ParticleRelative));
    panel.add_input<decl::Float>("Grid Scale"_ustr)
        .default_value(2.0f)
        .min(0.1f)
        .max(16.0f)
        .usage_by_menu("Resolution Mode"_ustr, int(geometry::FlipResolutionMode::ParticleRelative));
  }
  {
    auto &panel = b.add_panel("Forces"_ustr).default_closed(true);
    panel.add_input<decl::Vector>("Gravity"_ustr).default_value(float3(0.0f, 0.0f, -9.81f));
  }
  {
    auto &panel = b.add_panel("Transfer"_ustr).default_closed(true);
    panel.add_input<decl::Float>("FLIP Ratio"_ustr)
        .default_value(0.95f)
        .min(0.0f)
        .max(1.0f)
        .subtype(PROP_FACTOR);
  }
  {
    auto &panel = b.add_panel("Pressure"_ustr).default_closed(true);
    panel.add_input<decl::Float>("Density"_ustr).default_value(1000.0f).min(1.0e-6f);
    panel.add_input<decl::Int>("Max Pressure Iterations"_ustr)
        .default_value(500)
        .min(1)
        .max(10000);
    panel.add_input<decl::Float>("Pressure Tolerance"_ustr).default_value(1.0e-5f).min(1.0e-9f);
    panel.add_input<decl::Bool>("Use Multigrid Preconditioner"_ustr).default_value(false);
  }
  {
    auto &panel = b.add_panel("Viscosity"_ustr).default_closed(true);
    panel.add_input<decl::Float>("Viscosity"_ustr)
        .default_value(0.0f)
        .min(0.0f)
        .description("Kinematic viscosity in geometry local-space units squared per second");
    panel.add_input<decl::Int>("Max Viscosity Iterations"_ustr)
        .default_value(200)
        .min(1)
        .max(10000);
    panel.add_input<decl::Float>("Viscosity Tolerance"_ustr).default_value(1.0e-5f).min(1.0e-9f);
  }
  {
    auto &panel = b.add_panel("Particles"_ustr).default_closed(true);
    panel.add_input<decl::Bool>("Enable Reseeding"_ustr).default_value(true);
    panel.add_input<decl::Int>("Target Particles / Cell"_ustr).default_value(8).min(1).max(128);
    panel.add_input<decl::Int>("Min Particles / Cell"_ustr).default_value(4).min(1).max(128);
    panel.add_input<decl::Int>("Max Particles / Cell"_ustr).default_value(12).min(1).max(256);
  }
  {
    auto &panel = b.add_panel("Free Surface"_ustr).default_closed(true);
    panel.add_input<decl::Float>("Classification Radius Scale"_ustr)
        .default_value(0.75f)
        .min(0.0f)
        .max(1.0f);
  }
  {
    auto &panel = b.add_panel("Collision"_ustr).default_closed(true);
    panel.add_input<decl::Menu>("Collision Mode"_ustr)
        .static_items(collision_mode_items)
        .default_value(MenuValue(geometry::FlipCollisionMode::Disabled))
        .optional_label();
    panel.add_input<decl::Geometry>("Colliders"_ustr)
        .only_realized_data()
        .supported_type(GeometryComponent::Type::Mesh)
        .usage_by_menu("Collision Mode"_ustr,
                       {int(geometry::FlipCollisionMode::Static),
                        int(geometry::FlipCollisionMode::Animated)});
    panel.add_input<decl::Menu>("Motion Source"_ustr)
        .static_items(collider_motion_source_items)
        .default_value(MenuValue(geometry::FlipColliderMotionSource::PreviousGeometry))
        .usage_by_menu("Collision Mode"_ustr, int(geometry::FlipCollisionMode::Animated))
        .optional_label();
    panel.add_input<decl::Geometry>("Previous Colliders"_ustr)
        .only_realized_data()
        .supported_type(GeometryComponent::Type::Mesh)
        .usage_by_menu("Motion Source"_ustr,
                       int(geometry::FlipColliderMotionSource::PreviousGeometry));
    panel.add_input<decl::String>("Velocity Attribute"_ustr)
        .default_value("velocity")
        .usage_by_menu("Motion Source"_ustr,
                       int(geometry::FlipColliderMotionSource::VelocityAttribute));
    panel.add_input<decl::Float>("Collider Thickness"_ustr)
        .default_value(0.0f)
        .min(0.0f)
        .subtype(PROP_DISTANCE)
        .usage_by_menu("Collision Mode"_ustr,
                       {int(geometry::FlipCollisionMode::Static),
                        int(geometry::FlipCollisionMode::Animated)});
    panel.add_input<decl::Float>("Collider Friction"_ustr)
        .default_value(0.0f)
        .min(0.0f)
        .max(1.0f)
        .subtype(PROP_FACTOR)
        .usage_by_menu("Collision Mode"_ustr,
                       {int(geometry::FlipCollisionMode::Static),
                        int(geometry::FlipCollisionMode::Animated)});
  }
  {
    auto &panel = b.add_panel("Grid"_ustr).default_closed(true);
    panel.add_input<decl::Int>("Extrapolation Layers"_ustr).default_value(3).min(0).max(32);
  }
}

static bool finite_float3(const float3 value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

static geometry::FlipSolverSettings extract_settings(GeoNodeExecParams &params,
                                                     float &r_delta_time)
{
  geometry::FlipSolverSettings settings;
  settings.domain_min = params.extract_input<float3>("Domain Min"_ustr);
  settings.domain_max = params.extract_input<float3>("Domain Max"_ustr);
  r_delta_time = params.extract_input<float>("Delta Time"_ustr);
  settings.adaptive_substeps = params.extract_input<bool>("Use Adaptive Substeps"_ustr);
  settings.cfl_number = params.extract_input<float>("CFL Number"_ustr);
  const int fixed_substeps = std::clamp(params.extract_input<int>("Substeps"_ustr), 1, 128);
  settings.min_substeps = std::clamp(params.extract_input<int>("Min Substeps"_ustr), 1, 128);
  settings.max_substeps = std::clamp(
      params.extract_input<int>("Max Substeps"_ustr), settings.min_substeps, 128);
  if (!settings.adaptive_substeps) {
    settings.min_substeps = fixed_substeps;
    settings.max_substeps = fixed_substeps;
  }
  settings.resolution_mode = params.extract_input<geometry::FlipResolutionMode>(
      "Resolution Mode"_ustr);
  settings.target_cell_size = params.extract_input<float>("Cell Size"_ustr);
  settings.particle_spacing = params.extract_input<float>("Particle Spacing"_ustr);
  settings.grid_scale = params.extract_input<float>("Grid Scale"_ustr);
  settings.gravity = params.extract_input<float3>("Gravity"_ustr);
  settings.flip_ratio = std::clamp(params.extract_input<float>("FLIP Ratio"_ustr), 0.0f, 1.0f);
  settings.density = params.extract_input<float>("Density"_ustr);
  settings.reseeding = params.extract_input<bool>("Enable Reseeding"_ustr);
  settings.target_particles_per_cell = std::clamp(
      params.extract_input<int>("Target Particles / Cell"_ustr), 1, 128);
  settings.min_particles_per_cell = std::clamp(
      params.extract_input<int>("Min Particles / Cell"_ustr),
      1,
      settings.target_particles_per_cell);
  settings.max_particles_per_cell = std::clamp(
      params.extract_input<int>("Max Particles / Cell"_ustr),
      settings.target_particles_per_cell,
      256);
  settings.classification_radius_scale = std::clamp(
      params.extract_input<float>("Classification Radius Scale"_ustr), 0.0f, 1.0f);
  settings.pressure_max_iterations = std::clamp(
      params.extract_input<int>("Max Pressure Iterations"_ustr), 1, 10000);
  settings.pressure_tolerance = params.extract_input<float>("Pressure Tolerance"_ustr);
  settings.use_multigrid_preconditioner = params.extract_input<bool>(
      "Use Multigrid Preconditioner"_ustr);
  settings.viscosity = params.extract_input<float>("Viscosity"_ustr);
  settings.viscosity_max_iterations = std::clamp(
      params.extract_input<int>("Max Viscosity Iterations"_ustr), 1, 10000);
  settings.viscosity_tolerance = params.extract_input<float>("Viscosity Tolerance"_ustr);
  settings.extrapolation_layers = std::clamp(
      params.extract_input<int>("Extrapolation Layers"_ustr), 0, 32);
  settings.boundary_padding = std::max(params.extract_input<float>("Boundary Padding"_ustr), 0.0f);
  return settings;
}

static bool settings_are_valid(const geometry::FlipSolverSettings &settings,
                               const float delta_time)
{
  const float effective_cell_size = settings.resolution_mode ==
                                            geometry::FlipResolutionMode::ParticleRelative ?
                                        settings.particle_spacing * settings.grid_scale :
                                        settings.target_cell_size;
  return finite_float3(settings.domain_min) && finite_float3(settings.domain_max) &&
         finite_float3(settings.gravity) && std::isfinite(delta_time) && delta_time >= 0.0f &&
         std::isfinite(effective_cell_size) && effective_cell_size > 0.0f &&
         std::isfinite(settings.density) && settings.density > 0.0f &&
         std::isfinite(settings.cfl_number) && settings.cfl_number > 0.0f &&
         std::isfinite(settings.pressure_tolerance) && settings.pressure_tolerance > 0.0f &&
         std::isfinite(settings.viscosity) && settings.viscosity >= 0.0f &&
         std::isfinite(settings.viscosity_tolerance) && settings.viscosity_tolerance > 0.0f &&
         std::isfinite(settings.boundary_padding) &&
         settings.domain_max.x > settings.domain_min.x &&
         settings.domain_max.y > settings.domain_min.y &&
         settings.domain_max.z > settings.domain_min.z;
}

static geometry::FlipParticleData extract_particles(const PointCloud &pointcloud)
{
  geometry::FlipParticleData particles;
  particles.positions.extend(pointcloud.positions());
  particles.velocities.resize(pointcloud.totpoint);
  const bke::AttributeAccessor attributes = pointcloud.attributes();
  const VArray<float3> velocities = *attributes.lookup_or_default<float3>(
      "velocity", bke::AttrDomain::Point, float3(0.0f));
  velocities.materialize(particles.velocities.as_mutable_span());
  particles.ids.resize(pointcloud.totpoint);
  particles.source_indices.resize(pointcloud.totpoint);
  const bke::AttributeReader<int> ids = attributes.lookup<int>("id", bke::AttrDomain::Point);
  for (const int i : particles.positions.index_range()) {
    particles.ids[i] = ids ? int64_t(ids.varray[i]) : int64_t(i);
    particles.source_indices[i] = i;
  }
  return particles;
}

static PointCloud *build_output_pointcloud(const PointCloud &source,
                                           const geometry::FlipParticleData &particles)
{
  PointCloud *output = BKE_pointcloud_new_nomain(source.type, particles.positions.size());
  bke::gather_attributes(source.attributes(),
                         bke::AttrDomain::Point,
                         bke::AttrDomain::Point,
                         bke::attribute_filter_from_skip_ref({"position", "velocity", "id"}),
                         particles.source_indices.as_span(),
                         output->attributes_for_write());
  output->positions_for_write().copy_from(particles.positions.as_span());
  output->tag_positions_changed();
  bke::SpanAttributeWriter<float3> velocity =
      output->attributes_for_write().lookup_or_add_for_write_span<float3>("velocity",
                                                                          bke::AttrDomain::Point);
  if (velocity) {
    velocity.span.copy_from(particles.velocities.as_span());
    velocity.finish();
  }
  bke::SpanAttributeWriter<int> id =
      output->attributes_for_write().lookup_or_add_for_write_span<int>("id",
                                                                       bke::AttrDomain::Point);
  if (id) {
    for (const int i : particles.ids.index_range()) {
      id.span[i] = int(std::clamp<int64_t>(
          particles.ids[i], std::numeric_limits<int>::min(), std::numeric_limits<int>::max()));
    }
    id.finish();
  }
  return output;
}

static void report_warnings(GeoNodeExecParams &params, const geometry::FlipSolverStats &stats)
{
  if (stats.particle_undersampled) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("FLIP grid is undersampled. Increase particle density or increase the cell size"));
  }
  if (stats.cfl_clamped) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("Maximum FLIP substeps reached before satisfying the CFL condition"));
  }
  if (stats.collider_cfl_clamped) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("Maximum FLIP substeps reached before satisfying the collider CFL condition"));
  }
  if (stats.collider_missing) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Collision is enabled but no collider mesh was provided"));
  }
  if (stats.collider_previous_missing) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("Previous collider geometry is missing; using a static collider for this step"));
  }
  if (stats.collider_topology_mismatch) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("Collider topology does not match the previous geometry; using a static collider"));
  }
  if (stats.collider_velocity_missing) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("Collider velocity attribute is missing; using a static collider"));
  }
  if (stats.collider_velocity_invalid_type) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_(
            "Collider velocity must be a point-domain vector attribute; using a static collider"));
  }
  if (stats.collider_thickness_large) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("Collider thickness is large compared with the FLIP grid cell size"));
  }
  if (!stats.pressure_converged) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Pressure solve did not converge. Increase the maximum "
                                  "iterations or reduce the time step"));
  }
  if (!stats.viscosity_converged) {
    params.error_message_add(NodeWarningType::Warning, TIP_("Viscosity solve did not converge"));
  }
}

static void report_profile_if_requested(const geometry::FlipSolverStats &stats)
{
  if (std::getenv("BLENDER_FLIP_PROFILE") == nullptr) {
    return;
  }
  const geometry::FlipPerformanceStats &time = stats.performance;
  std::fprintf(stderr,
               "FLIP_PROFILE particles=%d grid=%dx%dx%d fluid_cells=%d substeps=%d "
               "pressure_iterations=%d pressure_residual=%.6g seeded=%d culled=%d "
               "total_ms=%.3f bin_ms=%.3f reseed_ms=%.3f reseed_cull_ms=%.3f "
               "reseed_seed_ms=%.3f reseed_balance_ms=%.3f collider_ms=%.3f "
               "p2g_ms=%.3f extrap_ms=%.3f gravity_ms=%.3f viscosity_ms=%.3f "
               "pressure_build_ms=%.3f pressure_solve_ms=%.3f pressure_project_ms=%.3f "
               "g2p_ms=%.3f advection_ms=%.3f output_ms=%.3f\n",
               stats.particle_count,
               stats.grid_x,
               stats.grid_y,
               stats.grid_z,
               stats.fluid_cells,
               stats.internal_substeps,
               stats.pressure_iterations,
               stats.pressure_relative_residual,
               stats.particles_seeded,
               stats.particles_culled,
               time.total_ms,
               time.particle_binning_ms,
               time.reseeding_ms,
               time.reseed_cull_ms,
               time.reseed_seed_ms,
               time.reseed_balance_ms,
               time.collider_ms,
               time.p2g_ms,
               time.extrapolation_ms,
               time.gravity_ms,
               time.viscosity_ms,
               time.pressure_build_ms,
               time.pressure_solve_ms,
               time.pressure_project_ms,
               time.g2p_ms,
               time.advection_ms,
               time.output_geometry_ms);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  bke::GeometrySet geometry = params.extract_input<bke::GeometrySet>("Particles"_ustr);
  bke::GeometrySet collider_geometry = params.extract_input<bke::GeometrySet>("Colliders"_ustr);
  bke::GeometrySet previous_collider_geometry = params.extract_input<bke::GeometrySet>(
      "Previous Colliders"_ustr);
  geometry::FlipColliderInput collider_input;
  collider_input.settings.mode = params.extract_input<geometry::FlipCollisionMode>(
      "Collision Mode"_ustr);
  collider_input.settings.motion_source = params.extract_input<geometry::FlipColliderMotionSource>(
      "Motion Source"_ustr);
  collider_input.settings.thickness = std::max(
      params.extract_input<float>("Collider Thickness"_ustr), 0.0f);
  collider_input.settings.friction = std::clamp(
      params.extract_input<float>("Collider Friction"_ustr), 0.0f, 1.0f);
  collider_input.velocity_attribute = params.extract_input<std::string>("Velocity Attribute"_ustr);
  collider_input.current_mesh = collider_geometry.get_mesh();
  collider_input.previous_mesh = previous_collider_geometry.get_mesh();
  if (!geometry.has_pointcloud()) {
    if (!geometry.is_empty()) {
      params.error_message_add(NodeWarningType::Warning,
                               TIP_("FLIP Solver requires a point cloud"));
    }
    params.set_output("Particles"_ustr, std::move(geometry));
    return;
  }
  const PointCloud *source = geometry.get_pointcloud();
  if (source == nullptr || source->totpoint == 0) {
    params.set_output("Particles"_ustr, std::move(geometry));
    return;
  }

  float delta_time;
  geometry::FlipSolverSettings settings = extract_settings(params, delta_time);
  if (!settings_are_valid(settings, delta_time)) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("FLIP Solver has an invalid numeric parameter or domain"));
    params.set_output("Particles"_ustr, std::move(geometry));
    return;
  }

  geometry::FlipParticleData particles = extract_particles(*source);
  geometry::FlipSolverStats stats;
  std::string error;
  const geometry::FlipColliderInput *collider = collider_input.settings.mode ==
                                                        geometry::FlipCollisionMode::Disabled ?
                                                    nullptr :
                                                    &collider_input;
  if (!geometry::flip_solver_step(settings, particles, delta_time, collider, &stats, error)) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("FLIP solve failed") : error.c_str());
    params.set_output("Particles"_ustr, std::move(geometry));
    return;
  }

  const timeit::TimePoint output_start = timeit::Clock::now();
  PointCloud *output = build_output_pointcloud(*source, particles);
  stats.performance.output_geometry_ms =
      std::chrono::duration<double, std::milli>(timeit::Clock::now() - output_start).count();
  stats.performance.total_ms += stats.performance.output_geometry_ms;
  geometry.replace_pointcloud(output);
  report_profile_if_requested(stats);
  report_warnings(params, stats);
  params.set_output("Particles"_ustr, std::move(geometry));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeFlipSolver"_ustr, GEO_NODE_FLIP_SOLVER);
  ntype.ui_name = "FLIP Solver";
  ntype.ui_description =
      "Advance point-cloud particles using a deterministic local-space CPU MAC FLIP solver";
  ntype.enum_name_legacy = "FLIP_SOLVER";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_flip_solver_cc
