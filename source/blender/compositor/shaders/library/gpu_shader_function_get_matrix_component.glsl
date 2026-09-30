/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

[[node]]
void node_function_get_matrix_component(float4x4 matrix, int column, int row, float &result)
{
  if (column >= 0 && column <= 3 && row >= 0 && row <= 3) {
    result = matrix[column][row];
  }
  else {
    result = 0.0f;
  }
}

[[node]]
void node_function_get_matrix_component_float(
    float4x4 matrix, float column, float row, float &result)
{
  node_function_get_matrix_component(matrix, int(column), int(row), result);
}
