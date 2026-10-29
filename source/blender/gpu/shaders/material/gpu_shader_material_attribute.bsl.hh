/* SPDX-FileCopyrightText: 2019-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "gpu_shader_common_math.bsl.hh"
#include "gpu_shader_material_interface.bsl.hh"

[[node]]
void node_attribute_matrix(float4 a, float4 b, float4 c, float4 d, float4x4 &matrix)
{
  matrix = float4x4(a, b, c, d);
}

[[node]]
void node_attribute_matrix_zero(float4x4 &matrix)
{
  matrix = float4x4(0.0f);
}

[[node]]
void node_attribute_color(KernelGlobals &kg, float4 attr, float4 &out_attr)
{
  out_attr = attr_load_color_post(kg, attr);
}

[[node]]
void node_attribute_temperature(KernelGlobals &kg, float4 attr, float4 &out_attr)
{
  float temperature = attr_load_temperature_post(kg, attr.x);
  out_attr.x = temperature;
  out_attr.y = temperature;
  out_attr.z = temperature;
  out_attr.w = 1.0f;
}

[[node]]
void node_attribute_density(float4 attr, float &out_attr)
{
  out_attr = attr.x;
}

[[node]]
void node_attribute_flame(float4 attr, float &out_attr)
{
  out_attr = attr.x;
}

[[node]]
void node_attribute_uniform(
    float4 attr, const float attr_hash, KernelGlobals &kg, const ShadingData &sd, float4 &out_attr)
{
  /* Temporary solution to support both old UBO attributes and new SSBO loading.
   * Old UBO load is already done through `attr` and will just be passed through. */
  out_attr = attr_load_uniform(kg, sd, attr, floatBitsToUint(attr_hash));
}

[[node]]
void node_attribute_light_is_sun(const float light_index, KernelGlobals &kg, float4 &out_attr)
{
  out_attr = float4(node_attribute_light_is_sun_impl(kg, int(light_index)));
}

[[node]]
void node_attribute_light_is_point(const float light_index, KernelGlobals &kg, float4 &out_attr)
{
  out_attr = float4(node_attribute_light_is_point_impl(kg, int(light_index)));
}

[[node]]
void node_attribute_light_is_spot(const float light_index, KernelGlobals &kg, float4 &out_attr)
{
  out_attr = float4(node_attribute_light_is_spot_impl(kg, int(light_index)));
}

[[node]]
void node_attribute_light_is_area(const float light_index, KernelGlobals &kg, float4 &out_attr)
{
  out_attr = float4(node_attribute_light_is_area_impl(kg, int(light_index)));
}

[[node]]
void node_attribute_light_cutoff_distance(const float light_index,
                                          KernelGlobals &kg,
                                          float4 &out_attr)
{
  out_attr = float4(node_attribute_light_cutoff_distance_impl(kg, int(light_index)));
}

[[node]]
void node_attribute_light(const float light_index,
                          const float attr_hash,
                          KernelGlobals &kg,
                          float4 &out_attr)
{
  out_attr = node_attribute_light_impl(kg, int(light_index), floatBitsToUint(attr_hash));
}

float4 attr_load_layer(KernelGlobals &kg, const uint attr_hash)
{
  return node_attribute_layer_impl(kg, attr_hash);
}

[[node]]
void node_attribute(float4 attr, float4 &outcol, float3 &outvec, float &outf, float &outalpha)
{
  outcol = float4(attr.xyz, 1.0f);
  outvec = attr.xyz;
  outf = math_average(attr.xyz);
  outalpha = attr.w;
}

[[node]]
void node_attribute_as_float(float4 attr, float &result)
{
  result = attr.x;
}

[[node]]
void node_attribute_as_int(float4 attr, float &result)
{
  result = trunc(attr.x);
}

[[node]]
void node_attribute_as_vector(float4 attr, float3 &result)
{
  result = attr.xyz;
}

[[node]]
void node_attribute_as_rotation(float4 attr, float4 &result)
{
  result = dot(attr, attr) > 0.0f ? normalize(attr) : float4(1.0f, 0.0f, 0.0f, 0.0f);
}

[[node]]
void node_attribute_as_color(float4 attr, float4 &result)
{
  result = attr;
}

[[node]]
void node_attribute_as_boolean(float4 attr, float &result)
{
  result = float(attr.x != 0.0f);
}

[[node]]
void node_attribute_as_matrix(float4 attr, float4x4 &result)
{
  result = float4x4(attr.x, 0.0f, 0.0f, 0.0f, 0.0f, attr.y, 0.0f, 0.0f, 0.0f, 0.0f, attr.z, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
}
