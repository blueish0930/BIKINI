/* SPDX-FileCopyrightText: 2026 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/geom/object.h"
#include "kernel/globals.h"
#include "kernel/svm/node_types.h"
#include "kernel/svm/util.h"
#include "util/math_float3.h"
#include "util/math_float4.h"

CCL_NAMESPACE_BEGIN

ccl_device_inline float3 billboard_transform_column(const Transform &t, const int column)
{
  if (column == 0) {
    return make_float3(t.x.x, t.y.x, t.z.x);
  }
  if (column == 1) {
    return make_float3(t.x.y, t.y.y, t.z.y);
  }
  if (column == 2) {
    return make_float3(t.x.z, t.y.z, t.z.z);
  }
  return make_float3(t.x.w, t.y.w, t.z.w);
}

ccl_device_inline void svm_billboard_object_axes(KernelGlobals kg,
                                                 const ccl_private ShaderData *sd,
                                                 ccl_private float3 *ox,
                                                 ccl_private float3 *oy,
                                                 ccl_private float3 *oz)
{
  if (sd->object == OBJECT_NONE) {
    *ox = make_float3(1.0f, 0.0f, 0.0f);
    *oy = make_float3(0.0f, 1.0f, 0.0f);
    *oz = make_float3(0.0f, 0.0f, 1.0f);
    return;
  }

  const Transform tfm = object_fetch_transform(kg, sd->object, OBJECT_TRANSFORM);
  *ox = safe_normalize_fallback(billboard_transform_column(tfm, 0),
                                make_float3(1.0f, 0.0f, 0.0f));
  *oy = safe_normalize_fallback(billboard_transform_column(tfm, 1),
                                make_float3(0.0f, 1.0f, 0.0f));
  *oz = safe_normalize_fallback(billboard_transform_column(tfm, 2),
                                make_float3(0.0f, 0.0f, 1.0f));
}

ccl_device_inline void svm_billboard_remap_axis(const uint8_t axis,
                                                const float3 bx,
                                                const float3 by,
                                                const float3 bz,
                                                ccl_private float3 *ax,
                                                ccl_private float3 *ay,
                                                ccl_private float3 *az)
{
  if (axis == NODE_BILLBOARD_AXIS_X) {
    /* +X faces the viewer. */
    *ax = bz;
    *az = by;
    *ay = cross(*az, *ax);
  }
  else if (axis == NODE_BILLBOARD_AXIS_Y) {
    /* +Y faces the viewer. +X = view X, +Z = view Y (up). */
    *ax = bx;
    *az = by;
    *ay = cross(*az, *ax);
  }
  else {
    /* +Z faces the viewer (plane). +X = view X, +Y = view Y. */
    *ax = bx;
    *ay = by;
    *az = bz;
  }
}

ccl_device_inline void svm_billboard_view_basis(KernelGlobals kg,
                                                const uint8_t mode,
                                                const float3 pivot,
                                                const float3 up,
                                                ccl_private float3 *bx,
                                                ccl_private float3 *by,
                                                ccl_private float3 *bz)
{
  const Transform cam = kernel_data.cam.cameratoworld;
  const float3 cam_x = safe_normalize_fallback(billboard_transform_column(cam, 0),
                                               make_float3(1.0f, 0.0f, 0.0f));
  const float3 cam_y = safe_normalize_fallback(billboard_transform_column(cam, 1),
                                               make_float3(0.0f, 1.0f, 0.0f));
  const float3 cam_z = safe_normalize_fallback(billboard_transform_column(cam, 2),
                                               make_float3(0.0f, 0.0f, 1.0f));

  if (mode == NODE_BILLBOARD_VIEW) {
    *bx = cam_x;
    *by = cam_y;
    *bz = cam_z;
    return;
  }

  const float3 upn = safe_normalize_fallback(up, make_float3(0.0f, 0.0f, 1.0f));
  const float3 cam_pos = billboard_transform_column(cam, 3);
  float3 zdir = (kernel_data.cam.type == CAMERA_ORTHOGRAPHIC) ? cam_z : (cam_pos - pivot);
  if (mode == NODE_BILLBOARD_CYLINDRICAL) {
    zdir = zdir - upn * dot(zdir, upn);
  }
  *bz = safe_normalize_fallback(zdir, cam_z);

  float3 xdir = cross(upn, *bz);
  if (len_squared(xdir) < 1.0e-16f) {
    xdir = cross(cam_y, *bz);
    if (len_squared(xdir) < 1.0e-16f) {
      xdir = cam_x;
    }
  }
  *bx = safe_normalize_fallback(xdir, cam_x);
  *by = cross(*bz, *bx);
}

/* XYZ Euler of R = aligned * orig^T, matching Vector Rotate Euler. */
ccl_device_inline float3 svm_billboard_rotation_euler(const float3 ox,
                                                      const float3 oy,
                                                      const float3 oz,
                                                      const float3 ax,
                                                      const float3 ay,
                                                      const float3 az,
                                                      const float factor)
{
  if (factor <= 0.0f) {
    return zero_float3();
  }

  /* Columns of R. */
  float3 c0 = ax * ox.x + ay * oy.x + az * oz.x;
  float3 c1 = ax * ox.y + ay * oy.y + az * oz.y;
  float3 c2 = ax * ox.z + ay * oy.z + az * oz.z;

  if (factor < 1.0f) {
    /* Nlerp identity -> quat(R), then rebuild columns. Quat stored as (w, x, y, z). */
    const float m00 = c0.x, m01 = c1.x, m02 = c2.x;
    const float m10 = c0.y, m11 = c1.y, m12 = c2.y;
    const float m20 = c0.z, m21 = c1.z, m22 = c2.z;
    float4 q;
    const float trace = m00 + m11 + m22;
    if (trace > 0.0f) {
      const float s = 0.5f / sqrtf(trace + 1.0f);
      q = make_float4(0.25f / s, (m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s);
    }
    else if (m00 > m11 && m00 > m22) {
      const float s = 2.0f * sqrtf(1.0f + m00 - m11 - m22);
      q = make_float4((m21 - m12) / s, 0.25f * s, (m01 + m10) / s, (m02 + m20) / s);
    }
    else if (m11 > m22) {
      const float s = 2.0f * sqrtf(1.0f + m11 - m00 - m22);
      q = make_float4((m02 - m20) / s, (m01 + m10) / s, 0.25f * s, (m12 + m21) / s);
    }
    else {
      const float s = 2.0f * sqrtf(1.0f + m22 - m00 - m11);
      q = make_float4((m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25f * s);
    }
    if (q.x < 0.0f) {
      q = -q;
    }
    q = make_float4((1.0f - factor) + factor * q.x, factor * q.y, factor * q.z, factor * q.w);
    q = normalize(q);
    const float xx = q.y * q.y;
    const float yy = q.z * q.z;
    const float zz = q.w * q.w;
    const float xy = q.y * q.z;
    const float xz = q.y * q.w;
    const float yz = q.z * q.w;
    const float wx = q.x * q.y;
    const float wy = q.x * q.z;
    const float wz = q.x * q.w;
    c0 = make_float3(1.0f - 2.0f * (yy + zz), 2.0f * (xy + wz), 2.0f * (xz - wy));
    c1 = make_float3(2.0f * (xy - wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz + wx));
    c2 = make_float3(2.0f * (xz + wy), 2.0f * (yz - wx), 1.0f - 2.0f * (xx + yy));
  }

  const float cy = hypotf(c0.x, c0.y);
  float3 eul;
  if (cy > 16.0f * FLT_EPSILON) {
    eul.x = atan2f(c1.z, c2.z);
    eul.y = atan2f(-c0.z, cy);
    eul.z = atan2f(c0.y, c0.x);
  }
  else {
    eul.x = atan2f(-c2.y, c1.y);
    eul.y = atan2f(-c0.z, cy);
    eul.z = 0.0f;
  }
  return eul;
}

ccl_device_inline float3 svm_billboard_rotate_dir(const float3 ox,
                                                  const float3 oy,
                                                  const float3 oz,
                                                  const float3 ax,
                                                  const float3 ay,
                                                  const float3 az,
                                                  const float3 v,
                                                  const float factor)
{
  const float3 local = make_float3(dot(v, ox), dot(v, oy), dot(v, oz));
  const float3 aligned = ax * local.x + ay * local.y + az * local.z;
  if (factor <= 0.0f) {
    return v;
  }
  if (factor >= 1.0f) {
    return aligned;
  }
  return safe_normalize(interp(v, aligned, factor));
}

ccl_device_noinline void svm_node_billboard_displacement(
    KernelGlobals kg,
    ccl_private ShaderData *sd,
    ccl_private float *ccl_restrict stack,
    const ccl_global SVMNodeBillboardDisplacement &ccl_restrict node)
{
  const float3 P = stack_valid(node.position_offset) ? stack_load_float3(stack, node.position_offset) :
                                                       sd->P;
  const float3 pivot = stack_valid(node.pivot_offset) ? stack_load_float3(stack, node.pivot_offset) :
                                                        object_location(kg, sd);
  const float3 N = stack_valid(node.normal_offset) ? stack_load_float3(stack, node.normal_offset) :
                                                     sd->N;
  const float3 up = stack_load(stack, node.up);
  const float factor = stack_load(stack, node.factor);

  float3 ox, oy, oz;
  svm_billboard_object_axes(kg, sd, &ox, &oy, &oz);

  float3 bx, by, bz;
  svm_billboard_view_basis(kg, node.mode, pivot, up, &bx, &by, &bz);

  float3 ax, ay, az;
  svm_billboard_remap_axis(node.axis, bx, by, bz, &ax, &ay, &az);

  const float3 rel = P - pivot;
  const float3 local = make_float3(dot(rel, ox), dot(rel, oy), dot(rel, oz));
  const float3 aligned = pivot + ax * local.x + ay * local.y + az * local.z;
  const float3 displacement = (aligned - P) * factor;
  const float3 out_P = P + displacement;

  if (stack_valid(node.displacement_offset)) {
    stack_store_float3(stack, node.displacement_offset, displacement);
  }
  if (stack_valid(node.position_out_offset)) {
    stack_store_float3(stack, node.position_out_offset, out_P);
  }
  if (stack_valid(node.rotation_offset)) {
    stack_store_float3(stack,
                       node.rotation_offset,
                       svm_billboard_rotation_euler(ox, oy, oz, ax, ay, az, factor));
  }
  if (stack_valid(node.normal_out_offset)) {
    stack_store_float3(stack,
                       node.normal_out_offset,
                       svm_billboard_rotate_dir(ox, oy, oz, ax, ay, az, N, factor));
  }
}

CCL_NAMESPACE_END
