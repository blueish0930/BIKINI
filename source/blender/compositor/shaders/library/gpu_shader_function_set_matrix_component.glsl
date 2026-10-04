/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

[[node]]
void node_function_set_matrix_component(
    float4x4 matrix, int column, int row, float value, float4x4 &result)
{
  result = matrix;
  if (column >= 0 && column <= 3 && row >= 0 && row <= 3) {
    result[column][row] = value;
  }
}

[[node]]
void node_function_set_matrix_component_float(
    float4x4 matrix, float column, float row, float value, float4x4 &result)
{
  node_function_set_matrix_component(matrix, int(column), int(row), value, result);
}
