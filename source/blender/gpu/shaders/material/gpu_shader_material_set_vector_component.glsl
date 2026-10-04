/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

[[node]]
void node_set_vector_component(float3 vector, int index, float value, float3 &result)
{
  result = vector;
  if (index >= 0 && index <= 2) {
    result[index] = value;
  }
}

[[node]]
void node_set_vector_component_float(float3 vector, float index, float value, float3 &result)
{
  node_set_vector_component(vector, int(index), value, result);
}
