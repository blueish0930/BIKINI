/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "gpu_shader_compat.hh"
#include "gpu_shader_material_interface.bsl.hh"

[[node]]
void node_ddx([[maybe_unused]] float value,
              [[maybe_unused]] float3 vector,
              [[maybe_unused]] float4 color,
              [[resource_table]] KernelGlobals &kg,
              float &result,
              float3 &vector_result,
              float4 &color_result)
{
#ifdef GPU_FRAGMENT_SHADER
  const float scale = derivative_scale_get(kg);
  result = gpu_dfdx(value) * scale;
  vector_result = gpu_dfdx(vector) * scale;
  color_result = gpu_dfdx(color) * scale;
#else
  result = 0.0f;
  vector_result = float3(0.0f);
  color_result = float4(0.0f);
#endif
}

[[node]]
void node_ddy([[maybe_unused]] float value,
              [[maybe_unused]] float3 vector,
              [[maybe_unused]] float4 color,
              [[resource_table]] KernelGlobals &kg,
              float &result,
              float3 &vector_result,
              float4 &color_result)
{
#ifdef GPU_FRAGMENT_SHADER
  const float scale = derivative_scale_get(kg);
  result = gpu_dfdy(value) * scale;
  vector_result = gpu_dfdy(vector) * scale;
  color_result = gpu_dfdy(color) * scale;
#else
  result = 0.0f;
  vector_result = float3(0.0f);
  color_result = float4(0.0f);
#endif
}

[[node]]
void node_fwidth([[maybe_unused]] float value,
                 [[maybe_unused]] float3 vector,
                 [[maybe_unused]] float4 color,
                 [[resource_table]] KernelGlobals &kg,
                 float &result,
                 float3 &vector_result,
                 float4 &color_result)
{
#ifdef GPU_FRAGMENT_SHADER
  const float scale = derivative_scale_get(kg);
  result = gpu_fwidth(value) * scale;
  vector_result = gpu_fwidth(vector) * scale;
  color_result = gpu_fwidth(color) * scale;
#else
  result = 0.0f;
  vector_result = float3(0.0f);
  color_result = float4(0.0f);
#endif
}
