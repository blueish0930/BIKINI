/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef GPU_SHADER
#  pragma once
#  include "gpu_shader_compat.hh"
#endif

#include "gpu_shader_create_info.hh"

GPU_SHADER_INTERFACE_INFO(compositor_rasterize_geometry_iface)
SMOOTH(float3, v_world_pos)
SMOOTH(float3, v_normal)
SMOOTH(float4, v_attr0)
SMOOTH(float4, v_attr1)
SMOOTH(float4, v_attr2)
SMOOTH(float4, v_attr3)
GPU_SHADER_INTERFACE_END()

/**
 * Hardware triangle rasterization for Image Process "Rasterize Geometry".
 * Vertex attributes are pre-projected to NDC on the CPU; the GPU only interpolates
 * and depth-tests (much faster than CPU per-pixel loops at 1k–4k).
 *
 * MRT:
 *  0 position.xyz + coverage in .w
 *  1 normal.xyz
 *  2 alpha in .x (float texture: only R written/used)
 *  3–6 optional user attributes (float4)
 */
GPU_SHADER_CREATE_INFO(compositor_rasterize_geometry)
VERTEX_IN(0, float3, pos_ndc) /* xy NDC, z depth 0..1 */
VERTEX_IN(1, float3, world_pos)
VERTEX_IN(2, float3, normal)
VERTEX_IN(3, float4, attr0)
VERTEX_IN(4, float4, attr1)
VERTEX_IN(5, float4, attr2)
VERTEX_IN(6, float4, attr3)
VERTEX_OUT(compositor_rasterize_geometry_iface)
FRAGMENT_OUT(0, float4, out_position)
FRAGMENT_OUT(1, float4, out_normal)
FRAGMENT_OUT(2, float4, out_alpha)
FRAGMENT_OUT(3, float4, out_attr0)
FRAGMENT_OUT(4, float4, out_attr1)
FRAGMENT_OUT(5, float4, out_attr2)
FRAGMENT_OUT(6, float4, out_attr3)
VERTEX_SOURCE("compositor_rasterize_geometry_vert.glsl")
FRAGMENT_SOURCE("compositor_rasterize_geometry_frag.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()
