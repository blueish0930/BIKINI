/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

[[node]]
void node_object_transform(float4 c0, float4 c1, float4 c2, float4 c3, float4 quaternion,
                           float4 signed_scale, float4 object_seed, float4x4 &transform,
                           float3 &location, float4 &rotation, float3 &scale, int &seed)
{
  transform = float4x4(c0, c1, c2, c3);
  location = c3.xyz;
  rotation = quaternion;
  scale = signed_scale.xyz;
  seed = int(object_seed.x);
}
