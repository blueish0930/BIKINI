/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Image Process fluid — MAC staggered GPU slab-ops + geometric Multigrid.
 *
 * Layout (Harlow–Welch MAC):
 *   pressure / dye / div : cell centers, size W×H
 *   u (vel_x)            : vertical faces, size (W+1)×H
 *   v (vel_y)            : horizontal faces, size W×(H+1)
 * Grid spacing Δx = Δy = 1.
 */

#ifdef GPU_SHADER
#  pragma once
#  include "gpu_shader_compat.hh"
#endif

#include "gpu_shader_create_info.hh"

/* Semi-Lagrangian advection of a scalar field by MAC velocity (dye at cells). */
GPU_SHADER_CREATE_INFO(compositor_fluid_advect_dye)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, timestep)
PUSH_CONSTANT(float, dissipation)
SAMPLER(0, sampler2D, source_tx)
SAMPLER(1, sampler2D, u_tx)
SAMPLER(2, sampler2D, v_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_advect_dye.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* Advect u faces: texture size (W+1)×H. */
GPU_SHADER_CREATE_INFO(compositor_fluid_advect_u)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, timestep)
SAMPLER(0, sampler2D, u_tx)
SAMPLER(1, sampler2D, v_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_advect_u.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* Advect v faces: texture size W×(H+1). */
GPU_SHADER_CREATE_INFO(compositor_fluid_advect_v)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, timestep)
SAMPLER(0, sampler2D, u_tx)
SAMPLER(1, sampler2D, v_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_advect_v.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* MAC divergence → cell centers: div = (uR-uL) + (vT-vB). */
GPU_SHADER_CREATE_INFO(compositor_fluid_divergence_mac)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, u_tx)
SAMPLER(1, sampler2D, v_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_divergence_mac.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* MAC gradient subtract: u -= (pR-pL), v -= (pT-pB) with dx=1. */
GPU_SHADER_CREATE_INFO(compositor_fluid_gradient_mac)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(int, component) /* 0=u faces, 1=v faces */
SAMPLER(0, sampler2D, vel_tx)
SAMPLER(1, sampler2D, pressure_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_gradient_mac.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* Jacobi smoother for Poisson (cell-centered p or viscous). */
GPU_SHADER_CREATE_INFO(compositor_fluid_jacobi)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, alpha)
PUSH_CONSTANT(float, rBeta)
SAMPLER(0, sampler2D, x_tx)
SAMPLER(1, sampler2D, b_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_jacobi.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* Neumann pressure / no-slip MAC face BC. component: 0=p, 1=u, 2=v */
GPU_SHADER_CREATE_INFO(compositor_fluid_boundary_mac)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(int, field_kind) /* 0=pressure, 1=u, 2=v */
SAMPLER(0, sampler2D, field_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_boundary_mac.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* Multigrid residual r = b - A x, A = 5-point Laplacian (dx=1). */
GPU_SHADER_CREATE_INFO(compositor_fluid_residual)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, x_tx)
SAMPLER(1, sampler2D, b_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_residual.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_restrict)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, fine_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_restrict.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_prolong)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, fine_tx)
SAMPLER(1, sampler2D, coarse_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_prolong.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* Optional vorticity confinement on cell-centered curl → force on faces. */
GPU_SHADER_CREATE_INFO(compositor_fluid_vorticity_mac)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, curl_strength)
PUSH_CONSTANT(float, timestep)
PUSH_CONSTANT(int, component) /* 0=u, 1=v */
SAMPLER(0, sampler2D, u_tx)
SAMPLER(1, sampler2D, v_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_vorticity_mac.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* -------------------------------------------------------------------- */
/** Pavel Dobryakov WebGL Fluid Simulation compute port.
 *
 * Velocity is collocated RG16F at SIM_RESOLUTION, pressure/curl/divergence are R16F,
 * and every dye field is RGBA16F at independent DYE_RESOLUTION. */

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_advect)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, timestep)
PUSH_CONSTANT(float, dissipation)
SAMPLER(0, sampler2D, velocity_tx)
SAMPLER(1, sampler2D, source_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_advect.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_advect_velocity)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, timestep)
PUSH_CONSTANT(float, dissipation)
SAMPLER(0, sampler2D, velocity_tx)
IMAGE(0, SFLOAT_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_advect_velocity.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_advect_scalar)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, timestep)
PUSH_CONSTANT(float, dissipation)
SAMPLER(0, sampler2D, velocity_tx)
SAMPLER(1, sampler2D, source_tx)
IMAGE(0, SFLOAT_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_advect_scalar.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_clear)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, value)
SAMPLER(0, sampler2D, input_tx)
IMAGE(0, SFLOAT_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_clear.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_curl)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, velocity_tx)
IMAGE(0, SFLOAT_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_curl.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_vorticity)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, curl_strength)
PUSH_CONSTANT(float, timestep)
SAMPLER(0, sampler2D, velocity_tx)
SAMPLER(1, sampler2D, curl_tx)
IMAGE(0, SFLOAT_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_vorticity.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_divergence)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, velocity_tx)
IMAGE(0, SFLOAT_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_divergence.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_pressure)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, pressure_tx)
SAMPLER(1, sampler2D, divergence_tx)
IMAGE(0, SFLOAT_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_pressure.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_gradient)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, pressure_tx)
SAMPLER(1, sampler2D, velocity_tx)
IMAGE(0, SFLOAT_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_gradient.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_residual)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, pressure_tx)
SAMPLER(1, sampler2D, rhs_tx)
IMAGE(0, SFLOAT_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_residual.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_restrict)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, rhs_scale)
SAMPLER(0, sampler2D, fine_tx)
IMAGE(0, SFLOAT_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_restrict.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_prolong)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, fine_tx)
SAMPLER(1, sampler2D, coarse_tx)
IMAGE(0, SFLOAT_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_prolong.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* Continuous Velocity-socket force inject: out = vel + force * scale. */
GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_add_velocity)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, force_scale)
SAMPLER(0, sampler2D, velocity_tx)
SAMPLER(1, sampler2D, force_tx)
IMAGE(0, SFLOAT_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_add_velocity.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* Continuous Color-socket paint: out = max(dye, source * scale). */
GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_max_color)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, source_scale)
SAMPLER(0, sampler2D, dye_tx)
SAMPLER(1, sampler2D, source_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_max_color.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_resample_color)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, value_scale)
SAMPLER(0, sampler2D, input_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_resample_color.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_resample_float)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float, value_scale)
SAMPLER(0, sampler2D, input_tx)
IMAGE(0, SFLOAT_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_resample_float.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_resample_float2)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float2, value_scale)
SAMPLER(0, sampler2D, input_tx)
IMAGE(0, SFLOAT_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_resample_float2.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* Solid cells take collider velocity (collocated RG velocity). */
GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_apply_obstacle)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, velocity_tx)
SAMPLER(1, sampler2D, solid_tx)
SAMPLER(2, sampler2D, collider_vel_tx)
IMAGE(0, SFLOAT_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_apply_obstacle.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* Mask scalar: mode 0=solid|air→0, 1=air→0, 2=solid→0. */
GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_mask_scalar)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(int, mode)
SAMPLER(0, sampler2D, input_tx)
SAMPLER(1, sampler2D, solid_tx)
SAMPLER(2, sampler2D, air_tx)
IMAGE(0, SFLOAT_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_mask_scalar.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/* Domain edge BC for collocated velocity (0=Outflow, 1=Wrap, 2=Bounce). */
GPU_SHADER_CREATE_INFO(compositor_fluid_pavel_boundary)
LOCAL_GROUP_SIZE(16, 16)
SAMPLER(0, sampler2D, velocity_tx)
IMAGE(0, SFLOAT_16_16, write, image2D, output_img)
COMPUTE_SOURCE("compositor_fluid_pavel_boundary.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()
