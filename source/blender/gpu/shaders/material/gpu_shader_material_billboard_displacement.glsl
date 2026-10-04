/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "gpu_shader_material_interface.bsl.hh"
#include "gpu_shader_material_transform_utils.bsl.hh"
#include "gpu_shader_math_matrix_construct.bsl.hh"
#include "gpu_shader_math_rotation_conversion.bsl.hh"
#include "gpu_shader_math_rotation.bsl.hh"
#include "gpu_shader_math_vector.bsl.hh"
#include "gpu_shader_math_vector_safe.bsl.hh"

[[node]]
void object_location_get([[resource_table]] KernelGlobals &kg,
                         const ShadingData &sd,
                         float3 &location)
{
  location = kg.object_matrices_get(sd).model[3].xyz;
}

float3 billboard_safe_axis(float3 v, float3 fallback)
{
  const float len_sq = length_squared(v);
  return (len_sq > 1.0e-16f) ? v * inversesqrt(len_sq) : fallback;
}

void billboard_object_axes(KernelGlobals &kg,
                           const ShadingData &sd,
                           out float3 ox,
                           out float3 oy,
                           out float3 oz)
{
  const float4x4 model = kg.object_matrices_get(sd).model;
  ox = billboard_safe_axis(model[0].xyz, float3(1.0f, 0.0f, 0.0f));
  oy = billboard_safe_axis(model[1].xyz, float3(0.0f, 1.0f, 0.0f));
  oz = billboard_safe_axis(model[2].xyz, float3(0.0f, 0.0f, 1.0f));
}

void billboard_remap_axis(int axis, float3 bx, float3 by, float3 bz, out float3 ax, out float3 ay, out float3 az)
{
  /* bx = view +X (right), by = view +Y (up), bz = view +Z (towards viewer).
   * Keep object +X on view +X whenever that axis is not the one facing the camera.
   * Right-handed: X × Y = Z. */
  if (axis == 0) {
    /* +X faces the viewer. */
    ax = bz;
    az = by;
    ay = cross(az, ax);
  }
  else if (axis == 1) {
    /* +Y faces the viewer (Blender forward). +X = view X, +Z = view Y (up). */
    ax = bx;
    az = by;
    ay = cross(az, ax);
  }
  else {
    /* +Z faces the viewer (plane). +X = view X, +Y = view Y. */
    ax = bx;
    ay = by;
    az = bz;
  }
}

void billboard_view_basis(KernelGlobals &kg,
                          const ShadingData &sd,
                          int mode,
                          float3 pivot,
                          float3 up,
                          out float3 bx,
                          out float3 by,
                          out float3 bz)
{
  const ViewMatrices view = kg.view_matrices_get(sd);
  const float3 cam_x = billboard_safe_axis(view.viewinv[0].xyz, float3(1.0f, 0.0f, 0.0f));
  const float3 cam_y = billboard_safe_axis(view.viewinv[1].xyz, float3(0.0f, 1.0f, 0.0f));
  const float3 cam_z = billboard_safe_axis(view.viewinv[2].xyz, float3(0.0f, 0.0f, 1.0f));

  if (mode == 0) {
    /* Screen-aligned: object axes match the camera (including roll). */
    bx = cam_x;
    by = cam_y;
    bz = cam_z;
    return;
  }

  const float3 upn = billboard_safe_axis(up, float3(0.0f, 0.0f, 1.0f));
  float3 zdir = view.is_perspective() ? (view.position() - pivot) : cam_z;
  if (mode == 2) {
    /* Yaw-only: drop the component along Up. */
    zdir = zdir - upn * dot(zdir, upn);
  }
  bz = billboard_safe_axis(zdir, cam_z);

  float3 xdir = cross(upn, bz);
  if (length_squared(xdir) < 1.0e-16f) {
    xdir = cross(cam_y, bz);
    if (length_squared(xdir) < 1.0e-16f) {
      xdir = cam_x;
    }
  }
  bx = billboard_safe_axis(xdir, cam_x);
  by = cross(bz, bx);
}

float3x3 billboard_axes_mat(float3 x, float3 y, float3 z)
{
  /* Same [col][row] layout as from_rotation / Vector Rotate. Column 0 = X axis. */
  float3x3 m;
  m[0][0] = x.x;
  m[0][1] = x.y;
  m[0][2] = x.z;
  m[1][0] = y.x;
  m[1][1] = y.y;
  m[1][2] = y.z;
  m[2][0] = z.x;
  m[2][1] = z.y;
  m[2][2] = z.z;
  return m;
}

float3 billboard_rotation_euler(float3 ox, float3 oy, float3 oz,
                                float3 ax, float3 ay, float3 az,
                                float factor)
{
  /* R maps original object axes onto billboard axes: R * ox = ax, etc.
   * Same transform as the vertex displacement, for Vector Rotate on normals. */
  float3x3 R = billboard_axes_mat(ax, ay, az) * transpose(billboard_axes_mat(ox, oy, oz));
  if (factor <= 0.0f) {
    return float3(0.0f);
  }
  if (factor < 1.0f) {
    const Quaternion q = interpolate(Quaternion::identity(), to_quaternion(R, true), factor);
    R = from_rotation(q);
  }
  return to_euler(R, true).as_float3();
}

float3 billboard_rotate_dir(float3 ox, float3 oy, float3 oz,
                            float3 ax, float3 ay, float3 az,
                            float3 v,
                            float factor)
{
  const float3 local = float3(dot(v, ox), dot(v, oy), dot(v, oz));
  const float3 aligned = ax * local.x + ay * local.y + az * local.z;
  if (factor <= 0.0f) {
    return v;
  }
  if (factor >= 1.0f) {
    return aligned;
  }
  return safe_normalize(mix(v, aligned, factor));
}

[[node]]
void node_billboard_displacement(float3 position,
                                 float3 pivot,
                                 float3 up,
                                 float factor,
                                 float3 normal,
                                 float4 params,
                                 [[resource_table]] KernelGlobals &kg,
                                 const ShadingData &sd,
                                 float3 &displacement,
                                 float3 &out_position,
                                 float3 &out_rotation,
                                 float3 &out_normal)
{
  const int mode = int(params.x);
  const int axis = int(params.y);

  float3 ox, oy, oz;
  billboard_object_axes(kg, sd, ox, oy, oz);

  float3 bx, by, bz;
  billboard_view_basis(kg, sd, mode, pivot, up, bx, by, bz);

  float3 ax, ay, az;
  billboard_remap_axis(axis, bx, by, bz, ax, ay, az);

  const float3 rel = position - pivot;
  const float3 local = float3(dot(rel, ox), dot(rel, oy), dot(rel, oz));
  const float3 aligned = pivot + ax * local.x + ay * local.y + az * local.z;
  displacement = (aligned - position) * factor;
  out_position = position + displacement;
  out_rotation = billboard_rotation_euler(ox, oy, oz, ax, ay, az, factor);
  out_normal = billboard_rotate_dir(ox, oy, oz, ax, ay, az, normal, factor);
}
