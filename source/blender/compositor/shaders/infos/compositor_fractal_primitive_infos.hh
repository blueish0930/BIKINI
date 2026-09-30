/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef GPU_SHADER
#  pragma once
#  include "gpu_shader_compat.hh"
#endif

#include "gpu_shader_create_info.hh"

GPU_SHADER_CREATE_INFO(compositor_fractal_primitive)
LOCAL_GROUP_SIZE(16, 16)
PUSH_CONSTANT(int2, domain_size)
PUSH_CONSTANT(float, power)
PUSH_CONSTANT(int, iterations)
PUSH_CONSTANT(float, bailout)
PUSH_CONSTANT(float2, julia_c)
PUSH_CONSTANT(float, scale)
PUSH_CONSTANT(int, fractal_kind)
PUSH_CONSTANT(int, vector_is_single)
SAMPLER(0, sampler2D, vector_tx)
IMAGE(0, SFLOAT_16_16_16_16, write, image2D, color_img)
IMAGE(1, SFLOAT_16, write, image2D, distance_img)
COMPUTE_SOURCE("compositor_fractal_primitive.glsl")
DO_STATIC_COMPILATION()
GPU_SHADER_CREATE_END()
