/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef GPU_SHADER
#  pragma once
#  include "gpu_shader_compat.hh"
#endif

#include "gpu_shader_create_info.hh"

/**
 * Hardware triangle raster of XY-projected mesh occupancy for Image Process "Geo SDF".
 * Vertex XY is pre-projected to NDC on the CPU.
 */
GPU_SHADER_CREATE_INFO(compositor_geo_sdf_raster)
VERTEX_IN(0, float2, pos_ndc)
FRAGMENT_OUT(0, float4, out_occupancy)
VERTEX_SOURCE("compositor_geo_sdf_raster_vert.glsl")
FRAGMENT_SOURCE("compositor_geo_sdf_raster_frag.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()

/**
 * Convert JFA nearest-boundary texels into a 0–1 UV-space signed (or unsigned) distance.
 * pixel_size is the UV extent of one texel.
 */
GPU_SHADER_CREATE_INFO(compositor_geo_sdf_compute_distance)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(float2, pixel_size)
PUSH_CONSTANT(int, signed_mode)
SAMPLER(0, isampler2D, mask_tx)
SAMPLER(1, isampler2D, flooded_boundary_tx)
IMAGE(0, SFLOAT_16, write, image2D, distance_img)
COMPUTE_SOURCE("compositor_geo_sdf_compute_distance.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()
