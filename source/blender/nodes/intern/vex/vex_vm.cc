/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <mutex>
#include <string>

#include "BLI_array.hh"
#include "BLI_enumerable_thread_specific.hh"
#include "BLI_execution_mode.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_constants.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_solvers.hh"
#include "BLI_math_quaternion.hh"
#include "BLI_math_rotation.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_noise.hh"
#include "BLI_span.hh"
#include "DNA_node_types.h"
#include "DNA_meshdata_types.h"
#include "BLI_string_ref.hh"
#include "BLI_task.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"

#include "vex_program.hh"

namespace blender::nodes::vex {

struct VMThreadLocalStorage {
  Vector<float4x4> mats;
  Vector<float3x3> mat3s;
  Vector<Vector<int>> iarr;
  Vector<Vector<float>> farr;
  Vector<Vector<float3>> varr;
  Vector<Vector<std::string>> sarr;
  Vector<Vector<float4x4>> marr;
  Vector<Vector<RayHit>> rarr;
  Vector<std::string> str_tmp;
  Vector<RayHit> rays;
  int mats_used = 0;
  int mat3s_used = 0;
  int iarr_used = 0;
  int farr_used = 0;
  int varr_used = 0;
  int sarr_used = 0;
  int marr_used = 0;
  int rarr_used = 0;
  int str_used = 0;
  int rays_used = 0;
};

static VMThreadLocalStorage &vm_thread_local_storage()
{
  /* TBB worker threads can outlive Blender's guarded allocator leak check. Keep the per-thread
   * caches owned by one enumerable container so its process-exit destructor releases every
   * worker's storage before the leak detector runs. The thread-local pointer keeps lookup cost
   * out of the VM's hot path. */
  static threading::EnumerableThreadSpecific<VMThreadLocalStorage> storage_by_thread;
  thread_local VMThreadLocalStorage *storage = &storage_by_thread.local();
  return *storage;
}

#define tls_mats (vm_thread_local_storage().mats)
#define tls_mat3s (vm_thread_local_storage().mat3s)
#define tls_iarr (vm_thread_local_storage().iarr)
#define tls_farr (vm_thread_local_storage().farr)
#define tls_varr (vm_thread_local_storage().varr)
#define tls_sarr (vm_thread_local_storage().sarr)
#define tls_marr (vm_thread_local_storage().marr)
#define tls_rarr (vm_thread_local_storage().rarr)
#define tls_str_tmp (vm_thread_local_storage().str_tmp)
#define tls_rays (vm_thread_local_storage().rays)
#define tls_mats_used (vm_thread_local_storage().mats_used)
#define tls_mat3s_used (vm_thread_local_storage().mat3s_used)
#define tls_iarr_used (vm_thread_local_storage().iarr_used)
#define tls_farr_used (vm_thread_local_storage().farr_used)
#define tls_varr_used (vm_thread_local_storage().varr_used)
#define tls_sarr_used (vm_thread_local_storage().sarr_used)
#define tls_marr_used (vm_thread_local_storage().marr_used)
#define tls_rarr_used (vm_thread_local_storage().rarr_used)
#define tls_str_used (vm_thread_local_storage().str_used)
#define tls_rays_used (vm_thread_local_storage().rays_used)

void tls_arrays_begin()
{
  tls_mats_used = 0;
  tls_mat3s_used = 0;
  tls_iarr_used = 0;
  tls_farr_used = 0;
  tls_varr_used = 0;
  tls_sarr_used = 0;
  tls_marr_used = 0;
  tls_rarr_used = 0;
  tls_str_used = 0;
  tls_rays_used = 0;
  tls_str_tmp.clear();
  /* Keep the outer pools from reallocating (which would invalidate iarr_mut
   * pointers held by foreach while the body calls nearestpoints again). */
  if (tls_iarr.capacity() < 1024) {
    tls_iarr.reserve(1024);
  }
  if (tls_farr.capacity() < 256) {
    tls_farr.reserve(256);
  }
  if (tls_varr.capacity() < 256) {
    tls_varr.reserve(256);
  }
}

Value Value::from_matrix(const float4x4 &mat)
{
  Value r;
  r.type = Type::Matrix;
  if (tls_mats_used >= tls_mats.size()) {
    tls_mats.append(mat);
  }
  else {
    tls_mats[tls_mats_used] = mat;
  }
  r.i = tls_mats_used++;
  return r;
}

Value Value::from_matrix3(const float3x3 &mat)
{
  Value r;
  r.type = Type::Matrix3;
  if (tls_mat3s_used >= tls_mat3s.size()) {
    tls_mat3s.append(mat);
  }
  else {
    tls_mat3s[tls_mat3s_used] = mat;
  }
  r.i = tls_mat3s_used++;
  return r;
}

static Value make_vec_value(const int n, const float x, const float y, const float z, const float w)
{
  if (n >= 4) {
    return Value::from_vec4(float4(x, y, z, w), Type::Vector4);
  }
  if (n == 2) {
    return Value::from_vec2(float2(x, y));
  }
  if (n == 1) {
    return Value::from_vec(float3(x, x, x));
  }
  return Value::from_vec(float3(x, y, z));
}

static float value_axis(const Value &a, const int axis)
{
  float f;
  if (axis == 1) {
    f = a.v.y;
  }
  else if (axis == 2) {
    f = a.v.z;
  }
  else if (axis == 3) {
    f = a.v.w;
  }
  else {
    f = a.v.x;
  }
  if (!type_has_xyzw(a.type) && a.type != Type::Matrix2) {
    f = a.as_float();
  }
  return f;
}

static Value apply_get_member(const Value &a, const int imm)
{
  const int n = swizzle_count(imm);
  if (n <= 1) {
    return Value::from_float(value_axis(a, swizzle_axis(imm, 0)));
  }
  const float x = value_axis(a, swizzle_axis(imm, 0));
  const float y = value_axis(a, swizzle_axis(imm, 1));
  const float z = (n >= 3) ? value_axis(a, swizzle_axis(imm, 2)) : 0.0f;
  const float w = (n >= 4) ? value_axis(a, swizzle_axis(imm, 3)) : 0.0f;
  return make_vec_value(n, x, y, z, w);
}

static Value apply_set_member(Value obj, const Value &mem, const int imm)
{
  if (!type_has_xyzw(obj.type) && obj.type != Type::Matrix2) {
    obj = Value::from_vec(obj.as_vec());
  }
  auto set_axis = [&](const int axis, const float f) {
    if (axis == 1) {
      obj.v.y = f;
    }
    else if (axis == 2) {
      obj.v.z = f;
    }
    else if (axis == 3) {
      obj.v.w = f;
    }
    else {
      obj.v.x = f;
    }
  };
  const int n = swizzle_count(imm);
  if (n <= 1) {
    set_axis(swizzle_axis(imm, 0), mem.as_float());
    return obj;
  }
  set_axis(swizzle_axis(imm, 0), mem.v.x);
  set_axis(swizzle_axis(imm, 1), mem.v.y);
  if (n >= 3) {
    set_axis(swizzle_axis(imm, 2), mem.v.z);
  }
  if (n >= 4) {
    set_axis(swizzle_axis(imm, 3), mem.v.w);
  }
  return obj;
}

template<typename T>
static int store_tls_array(Vector<Vector<T>> &pool, int &used, Vector<T> values)
{
  const int slot = used++;
  if (slot >= pool.size()) {
    pool.append(std::move(values));
  }
  else {
    /* Moving an empty temporary into an existing Vector releases its allocation. Most VEX array
     * code starts with array() and grows it with indexed writes, so that behavior turned the TLS
     * pool into one heap allocation per array per element. Keep the destination capacity and move
     * only the elements instead. */
    Vector<T> &dst = pool[slot];
    dst.clear();
    dst.reserve(values.size());
    for (T &value : values) {
      dst.append(std::move(value));
    }
  }
  return slot;
}

template<typename T>
static int prepare_tls_array(Vector<Vector<T>> &pool, int &used, const int size)
{
  const int slot = used++;
  if (slot >= pool.size()) {
    pool.append({});
  }
  Vector<T> &dst = pool[slot];
  dst.clear();
  dst.resize(std::max(size, 0));
  return slot;
}

Value value_from_int_array(Vector<int> values)
{
  Value r;
  r.type = Type::IntArray;
  r.i = store_tls_array(tls_iarr, tls_iarr_used, std::move(values));
  return r;
}
Value value_from_int_span(const Span<int> src)
{
  if (tls_iarr_used >= tls_iarr.size()) {
    tls_iarr.append({});
  }
  Vector<int> &dst = tls_iarr[tls_iarr_used];
  dst.clear();
  dst.extend(src);
  Value r;
  r.type = Type::IntArray;
  r.i = tls_iarr_used++;
  return r;
}
Value value_from_int_range(const int start, const int size)
{
  Value r;
  r.type = Type::IntArray;
  /* A negative pool slot encodes a contiguous lazy range. Keep integer bits in the float lanes so
   * very large meshes do not lose precision through an int -> float -> int conversion. */
  r.i = -1;
  const int safe_size = std::max(size, 0);
  static_assert(sizeof(r.v.x) == sizeof(start));
  memcpy(&r.v.x, &start, sizeof(start));
  memcpy(&r.v.y, &safe_size, sizeof(safe_size));
  return r;
}

static bool int_array_range(const Value &v, int &r_start, int &r_size)
{
  if (v.type != Type::IntArray || v.i != -1) {
    return false;
  }
  memcpy(&r_start, &v.v.x, sizeof(r_start));
  memcpy(&r_size, &v.v.y, sizeof(r_size));
  r_size = std::max(r_size, 0);
  return true;
}

Vector<int> int_array_values(const Value &v)
{
  int start = 0;
  int size = 0;
  if (int_array_range(v, start, size)) {
    Vector<int> values;
    values.reserve(size);
    for (int i = 0; i < size; i++) {
      values.append(start + i);
    }
    return values;
  }
  if (const Vector<int> *values = iarr_mut(v)) {
    return *values;
  }
  return {};
}

static Vector<int> *materialize_int_array(Value &v)
{
  int start = 0;
  int size = 0;
  if (int_array_range(v, start, size)) {
    Vector<int> values;
    values.reserve(size);
    for (int i = 0; i < size; i++) {
      values.append(start + i);
    }
    v = value_from_int_array(std::move(values));
  }
  return iarr_mut(v);
}

static Value face_corners_direct(const Inst &in, Span<Value> args, const VMEnv &env)
{
  int face = env.index;
  const int nargs = int(in.a);
  if (nargs >= 2) {
    face = args[1].as_int();
  }
  else if (nargs == 1) {
    face = args[0].as_int();
  }
  if (face < 0 || face + 1 >= env.face_offsets.size()) {
    return value_from_int_array({});
  }
  const int start = env.face_offsets[face];
  return value_from_int_range(start, env.face_offsets[face + 1] - start);
}
Value value_from_float_array(Vector<float> values)
{
  Value r;
  r.type = Type::FloatArray;
  r.i = store_tls_array(tls_farr, tls_farr_used, std::move(values));
  return r;
}
Value value_from_vec_array(Vector<float3> values)
{
  Value r;
  r.type = Type::VecArray;
  r.i = store_tls_array(tls_varr, tls_varr_used, std::move(values));
  return r;
}
Value value_from_string_array(Vector<std::string> values)
{
  Value r;
  r.type = Type::StringArray;
  r.i = store_tls_array(tls_sarr, tls_sarr_used, std::move(values));
  return r;
}
Value value_from_mat_array(Vector<float4x4> values)
{
  Value r;
  r.type = Type::MatArray;
  r.i = store_tls_array(tls_marr, tls_marr_used, std::move(values));
  return r;
}

Value value_from_ray(const RayHit &hit)
{
  if (tls_rays_used >= tls_rays.size()) {
    tls_rays.append(hit);
  }
  else {
    tls_rays[tls_rays_used] = hit;
  }
  Value r;
  r.type = Type::Ray;
  r.i = tls_rays_used++;
  r.v.x = (hit.hit != 0 || hit.face >= 0) ? 1.0f : 0.0f;
  return r;
}

Value value_from_ray_array(Vector<RayHit> hits)
{
  Value r;
  r.type = Type::RayArray;
  r.i = store_tls_array(tls_rarr, tls_rarr_used, std::move(hits));
  return r;
}

const RayHit *ray_from_value(const Value &v)
{
  if (v.type != Type::Ray || v.i < 0 || v.i >= int(tls_rays.size())) {
    return nullptr;
  }
  return &tls_rays[v.i];
}

static RayHit ray_hit_from_value(const Value &v)
{
  if (const RayHit *h = ray_from_value(v)) {
    return *h;
  }
  return {};
}

Vector<int> *iarr_mut(const Value &v)
{
  if (v.type != Type::IntArray || v.i < 0 || v.i >= int(tls_iarr.size())) {
    return nullptr;
  }
  return &tls_iarr[v.i];
}
Vector<float> *farr_mut(const Value &v)
{
  if (v.type != Type::FloatArray || v.i < 0 || v.i >= int(tls_farr.size())) {
    return nullptr;
  }
  return &tls_farr[v.i];
}
Vector<float3> *varr_mut(const Value &v)
{
  if (v.type != Type::VecArray || v.i < 0 || v.i >= int(tls_varr.size())) {
    return nullptr;
  }
  return &tls_varr[v.i];
}
Vector<std::string> *sarr_mut(const Value &v)
{
  if (v.type != Type::StringArray || v.i < 0 || v.i >= int(tls_sarr.size())) {
    return nullptr;
  }
  return &tls_sarr[v.i];
}
Vector<float4x4> *marr_mut(const Value &v)
{
  if (v.type != Type::MatArray || v.i < 0 || v.i >= int(tls_marr.size())) {
    return nullptr;
  }
  return &tls_marr[v.i];
}
Vector<RayHit> *rayarr_mut(const Value &v)
{
  if (v.type != Type::RayArray || v.i < 0 || v.i >= int(tls_rarr.size())) {
    return nullptr;
  }
  return &tls_rarr[v.i];
}

int array_len(const Value &v)
{
  int range_start = 0;
  int range_size = 0;
  if (int_array_range(v, range_start, range_size)) {
    return range_size;
  }
  if (const Vector<int> *a = iarr_mut(v)) {
    return int(a->size());
  }
  if (const Vector<float> *a = farr_mut(v)) {
    return int(a->size());
  }
  if (const Vector<float3> *a = varr_mut(v)) {
    return int(a->size());
  }
  if (const Vector<std::string> *a = sarr_mut(v)) {
    return int(a->size());
  }
  if (const Vector<float4x4> *a = marr_mut(v)) {
    return int(a->size());
  }
  if (const Vector<RayHit> *a = rayarr_mut(v)) {
    return int(a->size());
  }
  return 0;
}

int wrap_index(int i, const int n, const bool for_insert)
{
  if (n <= 0) {
    return for_insert ? 0 : -1;
  }
  if (i < 0) {
    i += n;
  }
  if (for_insert) {
    return std::clamp(i, 0, n);
  }
  if (i < 0 || i >= n) {
    return -1;
  }
  return i;
}

/** Past-the-end writes grow the array. A typo like `a[2000000000]` must not allocate. */
constexpr int k_array_grow_max = 1 << 20;

/**
 * Index for `a[i] = v`. A non-negative `i` at or past the end resizes to `i + 1` and fills the
 * new slots with \a fill (0, empty string, identity, …). Negative indices still wrap.
 */
template<typename T>
int array_write_index(Vector<T> &items, const int raw_i, const T &fill)
{
  const int n = int(items.size());
  if (raw_i >= n) {
    if (raw_i >= k_array_grow_max) {
      return -1;
    }
    items.resize(int64_t(raw_i) + 1, fill);
    return raw_i;
  }
  return wrap_index(raw_i, n, false);
}

Value empty_array(const Type type)
{
  if (type == Type::VecArray) {
    return value_from_vec_array({});
  }
  if (type == Type::FloatArray) {
    return value_from_float_array({});
  }
  if (type == Type::StringArray) {
    return value_from_string_array({});
  }
  if (type == Type::MatArray) {
    return value_from_mat_array({});
  }
  if (type == Type::RayArray) {
    return value_from_ray_array({});
  }
  return value_from_int_array({});
}

Value get_index_value(const Value &arr, const int raw_i)
{
  int range_start = 0;
  int range_size = 0;
  if (int_array_range(arr, range_start, range_size)) {
    const int i = wrap_index(raw_i, range_size, false);
    return Value::from_int(i >= 0 ? range_start + i : 0);
  }
  if (Vector<int> *a = iarr_mut(arr)) {
    const int i = wrap_index(raw_i, int(a->size()), false);
    return Value::from_int(i >= 0 ? (*a)[i] : 0);
  }
  if (Vector<float> *a = farr_mut(arr)) {
    const int i = wrap_index(raw_i, int(a->size()), false);
    return Value::from_float(i >= 0 ? (*a)[i] : 0.0f);
  }
  if (Vector<float3> *a = varr_mut(arr)) {
    const int i = wrap_index(raw_i, int(a->size()), false);
    return Value::from_vec(i >= 0 ? (*a)[i] : float3(0.0f));
  }
  if (Vector<std::string> *a = sarr_mut(arr)) {
    const int i = wrap_index(raw_i, int(a->size()), false);
    if (i < 0) {
      return Value::from_str_i(-1);
    }
    tls_str_tmp.append((*a)[i]);
    return Value::from_str_i(-2 - (int(tls_str_tmp.size()) - 1));
  }
  if (Vector<float4x4> *a = marr_mut(arr)) {
    const int i = wrap_index(raw_i, int(a->size()), false);
    return Value::from_matrix(i >= 0 ? (*a)[i] : float4x4::identity());
  }
  if (Vector<RayHit> *a = rayarr_mut(arr)) {
    const int i = wrap_index(raw_i, int(a->size()), false);
    return value_from_ray(i >= 0 ? (*a)[i] : RayHit{});
  }
  /* Column-major: m[col] is the column vector, v[i] is a component. */
  if (type_is_matrix(arr.type)) {
    const int n = type_linear_dim(arr.type);
    const int i = wrap_index(raw_i, n, false);
    if (arr.type == Type::Matrix2) {
      return Value::from_vec2(i >= 0 ? arr.as_matrix2()[i] : float2(0.0f));
    }
    if (arr.type == Type::Matrix3) {
      const float3x3 m = (arr.i >= 0 && arr.i < int(tls_mat3s.size())) ? tls_mat3s[arr.i] :
                                                                        float3x3::identity();
      return Value::from_vec(i >= 0 ? m[i] : float3(0.0f));
    }
    const float4x4 &m = (arr.i >= 0 && arr.i < int(tls_mats.size())) ? tls_mats[arr.i] :
                                                                      float4x4::identity();
    return Value::from_vec4(i >= 0 ? m[i] : float4(0.0f), Type::Vector4);
  }
  if (type_has_xyzw(arr.type)) {
    const int n = type_linear_dim(arr.type);
    const int i = wrap_index(raw_i, n, false);
    float f = 0.0f;
    if (i == 0) {
      f = arr.v.x;
    }
    else if (i == 1) {
      f = arr.v.y;
    }
    else if (i == 2) {
      f = arr.v.z;
    }
    else if (i == 3) {
      f = arr.v.w;
    }
    return Value::from_float(f);
  }
  return Value::from_float(0.0f);
}

Value set_index_value(Value arr, const int raw_i, const Value &elem, const VMEnv &env)
{
  /* `abc[3] = 1.2` on a shorter array grows it and zero-fills the gap. */
  if (Vector<int> *a = materialize_int_array(arr)) {
    const int i = array_write_index(*a, raw_i, 0);
    if (i >= 0) {
      (*a)[i] = elem.as_int();
    }
    return arr;
  }
  if (Vector<float> *a = farr_mut(arr)) {
    const int i = array_write_index(*a, raw_i, 0.0f);
    if (i >= 0) {
      (*a)[i] = elem.as_float();
    }
    return arr;
  }
  if (Vector<float3> *a = varr_mut(arr)) {
    const int i = array_write_index(*a, raw_i, float3(0.0f));
    if (i >= 0) {
      (*a)[i] = elem.as_vec();
    }
    return arr;
  }
  if (Vector<std::string> *a = sarr_mut(arr)) {
    const int i = array_write_index(*a, raw_i, std::string());
    if (i >= 0) {
      (*a)[i] = std::string(value_string(elem, env));
    }
    return arr;
  }
  if (Vector<float4x4> *a = marr_mut(arr)) {
    const int i = array_write_index(*a, raw_i, float4x4::identity());
    if (i >= 0) {
      if (elem.type == Type::Matrix && elem.i >= 0 && elem.i < int(tls_mats.size())) {
        (*a)[i] = tls_mats[elem.i];
      }
      else {
        (*a)[i] = float4x4::identity();
      }
    }
    return arr;
  }
  if (Vector<RayHit> *a = rayarr_mut(arr)) {
    const int i = array_write_index(*a, raw_i, RayHit{});
    if (i >= 0) {
      (*a)[i] = ray_hit_from_value(elem);
    }
    return arr;
  }
  if (type_is_matrix(arr.type)) {
    const int n = type_linear_dim(arr.type);
    const int i = wrap_index(raw_i, n, false);
    if (i < 0) {
      return arr;
    }
    if (arr.type == Type::Matrix2) {
      float2x2 m = arr.as_matrix2();
      m[i] = elem.as_vec2();
      return Value::from_matrix2(m);
    }
    if (arr.type == Type::Matrix3) {
      float3x3 m = (arr.i >= 0 && arr.i < int(tls_mat3s.size())) ? tls_mat3s[arr.i] :
                                                                  float3x3::identity();
      m[i] = elem.as_vec();
      return Value::from_matrix3(m);
    }
    float4x4 m = (arr.i >= 0 && arr.i < int(tls_mats.size())) ? tls_mats[arr.i] :
                                                               float4x4::identity();
    m[i] = elem.as_vec4();
    return Value::from_matrix(m);
  }
  if (type_has_xyzw(arr.type)) {
    const int n = type_linear_dim(arr.type);
    const int i = wrap_index(raw_i, n, false);
    if (i < 0) {
      return arr;
    }
    Value v = arr;
    const float f = elem.as_float();
    if (i == 0) {
      v.v.x = f;
    }
    else if (i == 1) {
      v.v.y = f;
    }
    else if (i == 2) {
      v.v.z = f;
    }
    else {
      v.v.w = f;
    }
    return v;
  }
  return arr;
}

bool is_geo_builtin(const Builtin id)
{
  switch (id) {
    case Builtin::CornersOfFace:
    case Builtin::CornersOfVertex:
    case Builtin::CornersOfEdge:
    case Builtin::EdgesOfVertex:
    case Builtin::EdgesOfCorner:
    case Builtin::FaceOfCorner:
    case Builtin::VertexOfCorner:
    case Builtin::OffsetCornerInFace:
    case Builtin::FacesOfVertex:
    case Builtin::Neighbours:
    case Builtin::PointNeighbours:
    case Builtin::PointEdges:
    case Builtin::PointFaces:
    case Builtin::PointCorners:
    case Builtin::EdgePoints:
    case Builtin::EdgeFaces:
    case Builtin::FacePoints:
    case Builtin::FaceEdges:
    case Builtin::FaceNeighbours:
    case Builtin::FaceCorners:
    case Builtin::CornerFace:
    case Builtin::PointsOfCurve:
    case Builtin::CurveOfPoint:
    case Builtin::PointCurve:
    case Builtin::NearestPoints:
    case Builtin::NearPoints:
    case Builtin::Raycast:
    case Builtin::RaycastAll:
    case Builtin::RayIsHit:
    case Builtin::RayHitPos:
    case Builtin::RayHitN:
    case Builtin::RayHitDist:
    case Builtin::RayIsHitArr:
    case Builtin::RayHitPosArr:
    case Builtin::RayHitNArr:
    case Builtin::RayHitDistArr:
    case Builtin::BBoxMin:
    case Builtin::BBoxMax:
    case Builtin::BoundingBox:
    case Builtin::GeometryProximity:
    case Builtin::SampleNearestSurface:
    case Builtin::DeleteGeometry:
    case Builtin::SetAttribute:
    case Builtin::Addpoint:
    case Builtin::Addprim:
    case Builtin::Accumulate:
    case Builtin::FieldAverage:
    case Builtin::FieldMin:
    case Builtin::FieldMax:
    case Builtin::FieldMinMax:
    case Builtin::Npoints:
    case Builtin::Nedges:
    case Builtin::Nfaces:
    case Builtin::Ncorners:
      return true;
    default:
      return false;
  }
}

Value packed_to_value(const bke::WrangleArrayValue &p)
{
  if (p.kind == bke::WrangleArrayKind::String) {
    Vector<std::string> vals;
    vals.reserve(p.count);
    for (int i = 0; i < p.count; i++) {
      vals.append(p.string_at(i));
    }
    return value_from_string_array(std::move(vals));
  }
  if (p.kind == bke::WrangleArrayKind::Float3) {
    Vector<float3> vals;
    vals.reserve(p.count);
    for (int i = 0; i < p.count; i++) {
      vals.append(float3(p.d.f[i * 3], p.d.f[i * 3 + 1], p.d.f[i * 3 + 2]));
    }
    return value_from_vec_array(std::move(vals));
  }
  if (p.kind == bke::WrangleArrayKind::Float) {
    Vector<float> vals;
    vals.reserve(p.count);
    for (int i = 0; i < p.count; i++) {
      vals.append(p.d.f[i]);
    }
    return value_from_float_array(std::move(vals));
  }
  if (p.kind == bke::WrangleArrayKind::Matrix) {
    Vector<float4x4> vals;
    vals.reserve(p.count);
    for (int i = 0; i < p.count; i++) {
      vals.append(p.matrix_at(i));
    }
    return value_from_mat_array(std::move(vals));
  }
  if (p.kind == bke::WrangleArrayKind::Matrix2) {
    return Value::from_matrix2(p.as_matrix2());
  }
  if (p.kind == bke::WrangleArrayKind::Matrix3) {
    return Value::from_matrix3(p.as_matrix3());
  }
  if (p.kind == bke::WrangleArrayKind::Ray) {
    Vector<RayHit> vals;
    vals.reserve(p.count);
    for (int i = 0; i < p.count; i++) {
      const float *f = p.d.f + i * bke::WrangleArrayValue::ray_floats;
      RayHit h;
      h.face = int(f[0]);
      h.hit = h.face >= 0 ? 1 : 0;
      h.pos = float3(f[1], f[2], f[3]);
      h.n = float3(f[4], f[5], f[6]);
      h.dist = f[7];
      vals.append(h);
    }
    return value_from_ray_array(std::move(vals));
  }
  Vector<int> vals;
  vals.reserve(p.count);
  for (int i = 0; i < p.count; i++) {
    vals.append(p.d.i[i]);
  }
  return value_from_int_array(std::move(vals));
}

void value_to_packed(const Value &v,
                     bke::WrangleArrayValue &p,
                     const Type dest,
                     const VMEnv *env)
{
  int range_start = 0;
  int range_size = 0;
  if (int_array_range(v, range_start, range_size)) {
    p.set_ints(int_array_values(v));
    return;
  }
  if (Vector<int> *a = iarr_mut(v)) {
    p.set_ints(*a);
    return;
  }
  if (Vector<float> *a = farr_mut(v)) {
    p.set_floats(*a);
    return;
  }
  if (Vector<float3> *a = varr_mut(v)) {
    p.set_float3s(*a);
    return;
  }
  if (Vector<std::string> *a = sarr_mut(v)) {
    p.set_strings(*a);
    return;
  }
  if (Vector<float4x4> *a = marr_mut(v)) {
    p.set_matrices(*a);
    return;
  }
  if (Vector<RayHit> *a = rayarr_mut(v)) {
    constexpr int stride = bke::WrangleArrayValue::ray_floats;
    const int n = int(a->size());
    Vector<float> packed(n * stride);
    for (int i = 0; i < n; i++) {
      const RayHit &h = (*a)[i];
      float *f = packed.data() + i * stride;
      f[0] = float(h.face);
      f[1] = h.pos.x;
      f[2] = h.pos.y;
      f[3] = h.pos.z;
      f[4] = h.n.x;
      f[5] = h.n.y;
      f[6] = h.n.z;
      f[7] = h.dist;
    }
    p.set_rays(n, packed.data());
    return;
  }
  if (v.type == Type::Ray) {
    const RayHit h = ray_hit_from_value(v);
    float packed[bke::WrangleArrayValue::ray_floats] = {
        float(h.face), h.pos.x, h.pos.y, h.pos.z, h.n.x, h.n.y, h.n.z, h.dist};
    p.set_rays(1, packed);
    return;
  }
  if (v.type == Type::Matrix) {
    float4x4 m = float4x4::identity();
    if (v.i >= 0 && v.i < int(tls_mats.size())) {
      m = tls_mats[v.i];
    }
    p.set_matrices(Span(&m, 1));
    return;
  }
  if (v.type == Type::Matrix2) {
    p.set_matrix2(v.as_matrix2());
    return;
  }
  if (v.type == Type::Matrix3) {
    if (v.i >= 0 && v.i < int(tls_mat3s.size())) {
      p.set_matrix3(tls_mat3s[v.i]);
    }
    else {
      p.set_matrix3(float3x3::identity());
    }
    return;
  }
  if (v.type == Type::String) {
    Vector<std::string> one;
    if (env) {
      one.append(std::string(value_string(v, *env)));
    }
    p.set_strings(one);
    return;
  }
  /* `{1, 2, 3}` is a vector literal; `i[]@ids = {1, 2, 3}` must still store a list. */
  if (type_is_vec_like(v.type)) {
    const float3 xyz = v.as_vec();
    if (dest == Type::VecArray) {
      p.set_float3s(Span(&xyz, 1));
      return;
    }
    if (dest == Type::FloatArray) {
      const float floats[3] = {xyz.x, xyz.y, xyz.z};
      p.set_floats(Span(floats, 3));
      return;
    }
    const int ints[3] = {int(xyz.x), int(xyz.y), int(xyz.z)};
    p.set_ints(Span(ints, 3));
    return;
  }
  if (v.type == Type::Int || v.type == Type::Bool) {
    const int n = v.as_int();
    p.set_ints(Span(&n, 1));
    return;
  }
  if (v.type == Type::Float) {
    const float n = v.as_float();
    p.set_floats(Span(&n, 1));
    return;
  }
  p.clear();
}

StringRef value_string(const Value &v, const VMEnv &env)
{
  if (v.type != Type::String) {
    return {};
  }
  if (v.i <= -2) {
    const int k = -2 - v.i;
    if (k >= 0 && k < int(tls_str_tmp.size())) {
      return tls_str_tmp[k];
    }
    return {};
  }
  if (v.i < 0) {
    return {};
  }
  if (v.i < env.const_s.size()) {
    return env.const_s[v.i];
  }
  if (env.runtime_s && v.i >= 100000) {
    const int ri = v.i - 100000;
    if (ri >= 0 && ri < env.runtime_s->size()) {
      return (*env.runtime_s)[ri];
    }
  }
  return {};
}

Value value_from_quat(const math::Quaternion &q)
{
  return Value::from_vec4(float4(q.x, q.y, q.z, q.w), Type::Rotation);
}

math::Quaternion value_as_quat(const Value &v)
{
  if (v.type == Type::Rotation) {
    return math::Quaternion(v.v.w, v.v.x, v.v.y, v.v.z);
  }
  const float3 eul = v.as_vec();
  return math::to_quaternion(math::EulerXYZ(eul.x, eul.y, eul.z));
}

namespace {

const float4x4 &value_as_matrix(const Value &v)
{
  static const float4x4 ident = float4x4::identity();
  if (v.type != Type::Matrix || v.i < 0 || v.i >= int(tls_mats.size())) {
    return ident;
  }
  return tls_mats[v.i];
}

float3x3 value_as_matrix3(const Value &v)
{
  if (v.type == Type::Matrix3 && v.i >= 0 && v.i < int(tls_mat3s.size())) {
    return tls_mat3s[v.i];
  }
  if (v.type == Type::Matrix) {
    return float3x3(value_as_matrix(v));
  }
  if (v.type == Type::Matrix2) {
    const float2x2 m = v.as_matrix2();
    return float3x3(float3(m[0].x, m[0].y, 0.0f),
                    float3(m[1].x, m[1].y, 0.0f),
                    float3(0.0f, 0.0f, 1.0f));
  }
  const float s = v.as_float();
  return float3x3::diagonal(s);
}

void store_mstring(MStringProperty &dst, const StringRef src)
{
  const int n = std::min(int(src.size()), 255);
  if (n > 0) {
    memcpy(dst.s, src.data(), n);
  }
  if (n < 255) {
    dst.s[n] = '\0';
  }
  dst.s_len = char(n);
}

Value intern_runtime_string(VMEnv &env, const StringRef s)
{
  if (env.runtime_s) {
    env.runtime_s->append(std::string(s));
    return Value::from_str_i(100000 + int(env.runtime_s->size()) - 1);
  }
  tls_str_tmp.append(std::string(s));
  return Value::from_str_i(-2 - (int(tls_str_tmp.size()) - 1));
}

Value load_attr_value(const AttrRT &a, const int index, VMEnv &env)
{
  if (index < 0 || index >= a.size) {
    switch (a.type) {
      case Type::Int:
      case Type::Bool:
        return Value::from_int(0);
      case Type::Vector:
        return Value::from_vec(float3(0.0f));
      case Type::Vector2:
        return Value::from_vec2(float2(0.0f));
      case Type::Vector4:
        return Value::from_vec4(float4(0.0f), Type::Vector4);
      case Type::Rotation:
        return value_from_quat(math::Quaternion::identity());
      case Type::Matrix2:
        return Value::from_matrix2(float2x2::identity());
      case Type::Matrix3:
        return Value::from_matrix3(float3x3::identity());
      case Type::IntArray:
        return value_from_int_array({});
      case Type::FloatArray:
        return value_from_float_array({});
      case Type::VecArray:
        return value_from_vec_array({});
      case Type::StringArray:
        return value_from_string_array({});
      case Type::MatArray:
        return value_from_mat_array({});
      case Type::RayArray:
        return value_from_ray_array({});
      default:
        return Value::from_float(0.0f);
    }
  }
  switch (a.type) {
    case Type::Bool:
      if (a.wb) {
        return Value::from_bool(a.wb[index]);
      }
      if (a.rb) {
        return Value::from_bool(a.rb[index]);
      }
      break;
    case Type::Int:
      if (a.wi) {
        return Value::from_int(a.wi[index]);
      }
      if (a.ri) {
        return Value::from_int(a.ri[index]);
      }
      break;
    case Type::Vector:
      if (a.wv) {
        return Value::from_vec(a.wv[index]);
      }
      if (a.rv) {
        return Value::from_vec(a.rv[index]);
      }
      break;
    case Type::Vector2:
      if (a.w2) {
        return Value::from_vec2(a.w2[index]);
      }
      if (a.r2) {
        return Value::from_vec2(a.r2[index]);
      }
      break;
    case Type::Vector4:
      if (a.w4) {
        return Value::from_vec4(a.w4[index], Type::Vector4);
      }
      if (a.r4) {
        return Value::from_vec4(a.r4[index], Type::Vector4);
      }
      break;
    case Type::Color:
      if (a.w4) {
        return Value::from_vec4(a.w4[index], Type::Color);
      }
      if (a.r4) {
        return Value::from_vec4(a.r4[index], Type::Color);
      }
      break;
    case Type::Rotation:
      if (a.wq) {
        return value_from_quat(a.wq[index]);
      }
      if (a.rq) {
        return value_from_quat(a.rq[index]);
      }
      break;
    case Type::Matrix:
      if (a.wm) {
        return Value::from_matrix(a.wm[index]);
      }
      if (a.rm) {
        return Value::from_matrix(a.rm[index]);
      }
      break;
    case Type::Matrix2:
    case Type::Matrix3:
    case Type::IntArray:
    case Type::FloatArray:
    case Type::VecArray:
    case Type::StringArray:
    case Type::MatArray:
    case Type::RayArray:
      if (a.warr) {
        if (a.type == Type::Matrix2) {
          return Value::from_matrix2(a.warr[index].as_matrix2());
        }
        if (a.type == Type::Matrix3) {
          return Value::from_matrix3(a.warr[index].as_matrix3());
        }
        return packed_to_value(a.warr[index]);
      }
      if (a.rarr) {
        if (a.type == Type::Matrix2) {
          return Value::from_matrix2(a.rarr[index].as_matrix2());
        }
        if (a.type == Type::Matrix3) {
          return Value::from_matrix3(a.rarr[index].as_matrix3());
        }
        return packed_to_value(a.rarr[index]);
      }
      if (a.type == Type::Matrix2) {
        return Value::from_matrix2(float2x2::identity());
      }
      if (a.type == Type::Matrix3) {
        return Value::from_matrix3(float3x3::identity());
      }
      if (a.type == Type::IntArray) {
        return value_from_int_array({});
      }
      if (a.type == Type::StringArray) {
        return value_from_string_array({});
      }
      if (a.type == Type::VecArray) {
        return value_from_vec_array({});
      }
      if (a.type == Type::MatArray) {
        return value_from_mat_array({});
      }
      if (a.type == Type::RayArray) {
        return value_from_ray_array({});
      }
      return value_from_float_array({});
    case Type::String: {
      const MStringProperty *s = a.ws ? a.ws : a.rs;
      if (s) {
        return intern_runtime_string(env, StringRef(s[index].s, uint8_t(s[index].s_len)));
      }
      return intern_runtime_string(env, "");
    }
    case Type::Float:
    default:
      if (a.wf) {
        return Value::from_float(a.wf[index]);
      }
      if (a.rf) {
        return Value::from_float(a.rf[index]);
      }
      break;
  }
  return Value::from_float(0.0f);
}

void store_attr_value(AttrRT &a, const int index, const Value &v, VMEnv &env)
{
  if (index < 0 || index >= a.size) {
    return;
  }
  switch (a.type) {
    case Type::Bool:
      if (a.wb) {
        a.wb[index] = v.as_bool();
      }
      break;
    case Type::Int:
      if (a.wi) {
        a.wi[index] = v.as_int();
      }
      break;
    case Type::Vector:
      if (a.wv) {
        a.wv[index] = v.as_vec();
      }
      break;
    case Type::Vector2:
      if (a.w2) {
        a.w2[index] = v.as_vec2();
      }
      break;
    case Type::Vector4:
      if (a.w4) {
        a.w4[index] = v.as_vec4();
      }
      break;
    case Type::Color:
      if (a.w4) {
        a.w4[index] = v.v;
      }
      break;
    case Type::Rotation:
      if (a.wq) {
        a.wq[index] = value_as_quat(v);
      }
      break;
    case Type::Matrix:
      if (a.wm) {
        a.wm[index] = value_as_matrix(v);
      }
      break;
    case Type::Matrix2:
    case Type::Matrix3:
    case Type::IntArray:
    case Type::FloatArray:
    case Type::VecArray:
    case Type::StringArray:
    case Type::MatArray:
    case Type::RayArray:
      if (a.warr) {
        value_to_packed(v, a.warr[index], a.type, &env);
      }
      break;
    case Type::String:
      if (a.ws) {
        store_mstring(a.ws[index], value_string(v, env));
      }
      break;
    case Type::Float:
    default:
      if (a.wf) {
        a.wf[index] = v.as_float();
      }
      break;
  }
}

void add_inplace(Value &a, const Value &b)
{
  if (a.type == Type::Matrix && b.type == Type::Matrix) {
    a = Value::from_matrix(value_as_matrix(a) + value_as_matrix(b));
    return;
  }
  if (a.type == Type::Matrix3 && b.type == Type::Matrix3) {
    a = Value::from_matrix3(value_as_matrix3(a) + value_as_matrix3(b));
    return;
  }
  if (a.type == Type::Matrix2 && b.type == Type::Matrix2) {
    a = Value::from_matrix2(a.as_matrix2() + b.as_matrix2());
    return;
  }
  if (a.type == Type::Vector4 || b.type == Type::Vector4) {
    a = Value::from_vec4(a.as_vec4() + b.as_vec4(), Type::Vector4);
    return;
  }
  if (a.type == Type::Vector || b.type == Type::Vector) {
    const float3 r = a.as_vec() + b.as_vec();
    a.type = Type::Vector;
    a.v = float4(r.x, r.y, r.z, 0.0f);
    return;
  }
  if (a.type == Type::Vector2 || b.type == Type::Vector2) {
    a = Value::from_vec2(a.as_vec2() + b.as_vec2());
    return;
  }
  if (a.type == Type::Int && b.type == Type::Int) {
    a.i += b.i;
    a.v.x = float(a.i);
    return;
  }
  a.type = Type::Float;
  a.v.x = a.as_float() + b.as_float();
}

void attr_add_inplace(AttrRT &a, const int index, const Value &v)
{
  if (index < 0 || index >= a.size) {
    return;
  }
  switch (a.type) {
    case Type::Vector:
      if (a.wv) {
        a.wv[index] += v.as_vec();
      }
      break;
    case Type::Vector2:
      if (a.w2) {
        a.w2[index] += v.as_vec2();
      }
      break;
    case Type::Vector4:
      if (a.w4) {
        a.w4[index] += v.as_vec4();
      }
      break;
    case Type::Int:
      if (a.wi) {
        a.wi[index] += v.as_int();
      }
      break;
    case Type::Bool:
      if (a.wb) {
        a.wb[index] = a.wb[index] || v.as_bool();
      }
      break;
    case Type::Float:
    default:
      if (a.wf) {
        a.wf[index] += v.as_float();
      }
      break;
  }
}

Value scale_value(const Value &v, const int k)
{
  const float s = float(k);
  if (v.type == Type::Vector4) {
    return Value::from_vec4(v.as_vec4() * s, Type::Vector4);
  }
  if (v.type == Type::Vector || v.type == Type::Color) {
    return Value::from_vec(v.as_vec() * s);
  }
  if (v.type == Type::Rotation) {
    return v;
  }
  if (v.type == Type::Vector2) {
    return Value::from_vec2(v.as_vec2() * s);
  }
  if (v.type == Type::Int) {
    return Value::from_int(v.as_int() * k);
  }
  return Value::from_float(v.as_float() * s);
}

struct CountedAttrLoop {
  int cond_pc = 0;
  int end_pc = 0;
  int i_slot = 0;
  int k_const_index = 0;
  int trip_count = 0;
  struct Add {
    Op src_op = Op::PushLocal;
    int src_imm = 0;
    int local = -1;
    int attr = 0;
  };
  Vector<Add, 8> adds;
};

bool match_counted_attr_loop(const Program &prog, const int cond_pc, CountedAttrLoop &out)
{
  const Span<Inst> c = prog.code;
  if (cond_pc < 0 || cond_pc + 6 >= c.size()) {
    return false;
  }
  int ip = cond_pc;
  if (c[ip].op != Op::PushLocal) {
    return false;
  }
  out.i_slot = c[ip].imm;
  ip++;
  if (c[ip].op != Op::PushI) {
    return false;
  }
  out.k_const_index = c[ip].imm;
  if (out.k_const_index < 0 || out.k_const_index >= prog.const_i.size()) {
    return false;
  }
  out.trip_count = prog.const_i[out.k_const_index];
  if (out.trip_count <= 0) {
    return false;
  }
  ip++;
  if (c[ip].op != Op::Lt) {
    return false;
  }
  ip++;
  if (c[ip].op != Op::JmpIfFalse) {
    return false;
  }
  out.end_pc = c[ip].imm;
  if (out.end_pc <= ip + 1 || out.end_pc > c.size()) {
    return false;
  }
  ip++;
  out.adds.clear();
  while (ip + 2 <= out.end_pc) {
    const Op src = c[ip].op;
    if (c[ip + 1].op != Op::AttrAdd) {
      break;
    }
    CountedAttrLoop::Add add;
    add.attr = c[ip + 1].imm;
    add.src_op = src;
    add.src_imm = c[ip].imm;
    if (src == Op::PushLocal) {
      if (c[ip].imm == out.i_slot) {
        return false;
      }
      add.local = c[ip].imm;
    }
    else if (ELEM(src, Op::PushV, Op::PushF, Op::PushI)) {
      add.local = -1;
    }
    else {
      break;
    }
    out.adds.append(add);
    ip += 2;
  }
  if (out.adds.is_empty()) {
    return false;
  }
  if (ip + 1 >= out.end_pc || ip + 1 >= c.size()) {
    return false;
  }
  if (c[ip].op != Op::IncLocal || c[ip].imm != out.i_slot) {
    return false;
  }
  ip++;
  if (c[ip].op != Op::Jmp || c[ip].imm != cond_pc) {
    return false;
  }
  ip++;
  if (ip != out.end_pc) {
    return false;
  }
  out.cond_pc = cond_pc;
  return true;
}

bool loop_counter_starts_at_zero(const Program &prog, const CountedAttrLoop &loop)
{
  if (loop.cond_pc < 2) {
    return false;
  }
  const Inst &push = prog.code[loop.cond_pc - 2];
  const Inst &store = prog.code[loop.cond_pc - 1];
  if (push.op != Op::PushI || store.op != Op::StoreLocal) {
    return false;
  }
  if (store.imm != loop.i_slot) {
    return false;
  }
  if (push.imm < 0 || push.imm >= prog.const_i.size()) {
    return false;
  }
  return prog.const_i[push.imm] == 0;
}

Value counted_loop_addend(const Program &program,
                          const CountedAttrLoop::Add &add,
                          const Value *locals,
                          const int local_n)
{
  switch (add.src_op) {
    case Op::PushLocal:
      if (add.local >= 0 && add.local < local_n) {
        return locals[add.local];
      }
      break;
    case Op::PushV:
      if (add.src_imm >= 0 && add.src_imm < program.const_v.size()) {
        return Value::from_vec(program.const_v[add.src_imm]);
      }
      break;
    case Op::PushF:
      if (add.src_imm >= 0 && add.src_imm < program.const_f.size()) {
        return Value::from_float(program.const_f[add.src_imm]);
      }
      break;
    case Op::PushI:
      if (add.src_imm >= 0 && add.src_imm < program.const_i.size()) {
        return Value::from_int(program.const_i[add.src_imm]);
      }
      break;
    default:
      break;
  }
  return Value::from_float(0.0f);
}

bool while_cmp_holds(const uint8_t cmp, const float x, const float limit)
{
  switch (cmp) {
    case 0:
      return x < limit;
    case 1:
      return x <= limit;
    case 2:
      return x > limit;
    default:
      return x >= limit;
  }
}

int while_cmp_iter_count(
    const uint8_t cmp, const float x, const float limit, const float step, const int max_n)
{
  if (!while_cmp_holds(cmp, x, limit)) {
    return 0;
  }
  const bool toward_up = cmp <= 1;
  if (toward_up ? (step <= 0.0f) : (step >= 0.0f)) {
    return 0;
  }
  const double dx = double(limit) - double(x);
  const double s = double(step);
  double n = 0.0;
  if (cmp == 0) {
    n = std::ceil(dx / s - 1e-12);
  }
  else if (cmp == 1) {
    n = std::floor(dx / s + 1e-12) + 1.0;
  }
  else if (cmp == 2) {
    n = std::ceil((-dx) / (-s) - 1e-12);
  }
  else {
    n = std::floor((-dx) / (-s) + 1e-12) + 1.0;
  }
  if (!(n > 0.0)) {
    return 0;
  }
  if (n > double(max_n)) {
    return max_n;
  }
  return int(n);
}

void while_cmp_add_counters(MutableSpan<AttrRT> attrs, const WhileCmpAddSpec &w, const int index, const int n)
{
  if (n == 0) {
    return;
  }
  for (const int slot : w.counters) {
    if (slot < 0 || slot >= attrs.size()) {
      continue;
    }
    AttrRT &c = attrs[slot];
    if (c.wi && index >= 0 && index < c.size) {
      c.wi[index] += n;
    }
  }
}

void while_cmp_add_one(MutableSpan<AttrRT> attrs, const WhileCmpAddSpec &w, const int index)
{
  if (w.attr < 0 || w.attr >= attrs.size()) {
    return;
  }
  AttrRT &a = attrs[w.attr];
  if (index < 0 || index >= a.size) {
    return;
  }
  constexpr int max_n = 10000000;
  if (a.wv && w.member >= 0 && w.member <= 2) {
    float3 &p = a.wv[index];
    const float x = (&p.x)[w.member];
    const float step = (&w.addend.x)[w.member];
    const int n = while_cmp_iter_count(w.cmp, x, w.limit, step, max_n);
    if (n > 0) {
      p += w.addend * float(n);
      while_cmp_add_counters(attrs, w, index, n);
    }
    return;
  }
  if (a.wf) {
    float &x = a.wf[index];
    const float step = w.addend.x;
    const int n = while_cmp_iter_count(w.cmp, x, w.limit, step, max_n);
    if (n > 0) {
      x += step * float(n);
      while_cmp_add_counters(attrs, w, index, n);
    }
  }
}

void while_cmp_add_mask(MutableSpan<AttrRT> attrs, const WhileCmpAddSpec &w, const IndexMask &mask)
{
  if (w.attr < 0 || w.attr >= attrs.size()) {
    return;
  }
  AttrRT &a = attrs[w.attr];
  constexpr int max_n = 10000000;
  if (a.wv && w.member >= 0 && w.member <= 2) {
    const int member = w.member;
    const float3 addend = w.addend;
    const float step = (&addend.x)[member];
    float3 *const dst = a.wv;
    const uint8_t cmp = w.cmp;
    const float limit = w.limit;
    mask.foreach_index(
        [&](const int64_t i) {
          float3 &p = dst[i];
          const float x = (&p.x)[member];
          const int n = while_cmp_iter_count(cmp, x, limit, step, max_n);
          if (n > 0) {
            p += addend * float(n);
            while_cmp_add_counters(attrs, w, int(i), n);
          }
        },
        exec_mode::grain_size(4096));
    return;
  }
  if (a.wf) {
    const float step = w.addend.x;
    float *const dst = a.wf;
    const uint8_t cmp = w.cmp;
    const float limit = w.limit;
    mask.foreach_index(
        [&](const int64_t i) {
          float &x = dst[i];
          const int n = while_cmp_iter_count(cmp, x, limit, step, max_n);
          if (n > 0) {
            x += step * float(n);
            while_cmp_add_counters(attrs, w, int(i), n);
          }
        },
        exec_mode::grain_size(4096));
  }
}

template<typename T, typename Fn> void for_mask_dense(T *dst, const IndexMask &mask, Fn &&fn)
{
  if (dst == nullptr) {
    return;
  }
  if (const std::optional<IndexRange> range = mask.to_range()) {
    threading::parallel_for(*range, 4096, [&](const IndexRange sub) {
      T *p = dst + sub.start();
      T *const end = dst + sub.one_after_last();
      for (; p != end; ++p) {
        fn(*p);
      }
    });
    return;
  }
  mask.foreach_index_optimized<int64_t>([&](const int64_t i) { fn(dst[i]); },
                                        exec_mode::grain_size(4096));
}

void attr_add_mask(AttrRT &a, const Value &v, const IndexMask &mask)
{
  if (a.size <= 0) {
    return;
  }
  switch (a.type) {
    case Type::Vector: {
      const float3 o = v.as_vec();
      for_mask_dense(a.wv, mask, [o](float3 &p) { p += o; });
      break;
    }
    case Type::Vector2: {
      const float2 o = v.as_vec2();
      for_mask_dense(a.w2, mask, [o](float2 &p) { p += o; });
      break;
    }
    case Type::Vector4: {
      const float4 o = v.as_vec4();
      for_mask_dense(a.w4, mask, [o](float4 &p) { p += o; });
      break;
    }
    case Type::Int: {
      const int o = v.as_int();
      for_mask_dense(a.wi, mask, [o](int &p) { p += o; });
      break;
    }
    case Type::Float:
    default: {
      const float o = v.as_float();
      for_mask_dense(a.wf, mask, [o](float &p) { p += o; });
      break;
    }
  }
}

void attr_store_mask(AttrRT &a, const Value &v, const IndexMask &mask, VMEnv &env)
{
  if (a.size <= 0) {
    return;
  }
  switch (a.type) {
    case Type::Vector: {
      const float3 o = v.as_vec();
      for_mask_dense(a.wv, mask, [o](float3 &p) { p = o; });
      break;
    }
    case Type::Vector2: {
      const float2 o = v.as_vec2();
      for_mask_dense(a.w2, mask, [o](float2 &p) { p = o; });
      break;
    }
    case Type::Vector4: {
      const float4 o = v.as_vec4();
      for_mask_dense(a.w4, mask, [o](float4 &p) { p = o; });
      break;
    }
    case Type::Int: {
      const int o = v.as_int();
      for_mask_dense(a.wi, mask, [o](int &p) { p = o; });
      break;
    }
    case Type::Bool: {
      const bool o = v.as_bool();
      for_mask_dense(a.wb, mask, [o](bool &p) { p = o; });
      break;
    }
    case Type::Color: {
      const float4 o = v.v;
      for_mask_dense(a.w4, mask, [o](float4 &p) { p = o; });
      break;
    }
    case Type::Rotation: {
      const math::Quaternion q = value_as_quat(v);
      if (a.wq) {
        for_mask_dense(a.wq, mask, [q](math::Quaternion &p) { p = q; });
      }
      break;
    }
    case Type::Matrix: {
      const float4x4 m = value_as_matrix(v);
      for_mask_dense(a.wm, mask, [m](float4x4 &p) { p = m; });
      break;
    }
    case Type::Matrix2:
    case Type::Matrix3:
    case Type::IntArray:
    case Type::FloatArray:
    case Type::VecArray:
    case Type::StringArray:
    case Type::MatArray:
    case Type::RayArray: {
      if (a.warr) {
        bke::WrangleArrayValue packed;
        value_to_packed(v, packed, a.type, &env);
        for_mask_dense(a.warr, mask, [packed](bke::WrangleArrayValue &p) { p = packed; });
      }
      break;
    }
    case Type::String: {
      if (a.ws) {
        MStringProperty s{};
        store_mstring(s, value_string(v, env));
        for_mask_dense(a.ws, mask, [s](MStringProperty &p) { p = s; });
      }
      break;
    }
    case Type::Float:
    default: {
      const float o = v.as_float();
      for_mask_dense(a.wf, mask, [o](float &p) { p = o; });
      break;
    }
  }
}

std::string pad_field(std::string s, const int width, const bool zero, const bool left)
{
  if (width <= int(s.size())) {
    return s;
  }
  const int n = width - int(s.size());
  if (left) {
    return s + std::string(size_t(n), ' ');
  }
  if (zero && !s.empty() && (s[0] == '-' || s[0] == '+')) {
    return std::string(1, s[0]) + std::string(size_t(n), '0') + s.substr(1);
  }
  return std::string(size_t(n), zero ? '0' : ' ') + s;
}

std::string valuetostring_int(const long long value, int before)
{
  before = std::max(before, 0);
  const bool neg = value < 0;
  unsigned long long mag = 0;
  if (neg) {
    mag = (value == (-9223372036854775807LL - 1)) ?
              9223372036854775808ULL :
              static_cast<unsigned long long>(-value);
  }
  else {
    mag = static_cast<unsigned long long>(value);
  }
  std::string digits = std::to_string(mag);
  if (int(digits.size()) < before) {
    digits.insert(0, size_t(before - int(digits.size())), '0');
  }
  return neg ? "-" + digits : digits;
}

std::string valuetostring_float(const double value, int before, int after)
{
  before = std::max(before, 0);
  after = std::max(after, 0);
  if (!std::isfinite(value)) {
    if (std::isnan(value)) {
      return "nan";
    }
    return value > 0.0 ? "inf" : "-inf";
  }
  const bool neg = value < 0.0 || (value == 0.0 && std::signbit(value));
  const double mag = std::fabs(value);
  long long scale = 1;
  for (int i = 0; i < after; i++) {
    scale *= 10;
  }
  long long scaled = std::llround(mag * double(scale));
  long long ip = after > 0 ? scaled / scale : scaled;
  long long fd = after > 0 ? scaled % scale : 0;
  if (fd < 0) {
    fd = -fd;
  }
  std::string idigits = std::to_string(ip < 0 ? -ip : ip);
  if (int(idigits.size()) < before) {
    idigits.insert(0, size_t(before - int(idigits.size())), '0');
  }
  std::string out = neg ? "-" + idigits : idigits;
  if (after > 0) {
    std::string fdigits = std::to_string(fd);
    if (int(fdigits.size()) < after) {
      fdigits.insert(0, size_t(after - int(fdigits.size())), '0');
    }
    out += '.';
    out += fdigits;
  }
  return out;
}

std::string value_default_string(const Value &v, VMEnv &env)
{
  switch (v.type) {
    case Type::String:
      return std::string(value_string(v, env));
    case Type::Int:
    case Type::Bool:
      return std::to_string(v.as_int());
    case Type::Float: {
      char buf[64];
      snprintf(buf, sizeof(buf), "%g", double(v.as_float()));
      return buf;
    }
    case Type::Vector2: {
      char buf[128];
      snprintf(buf, sizeof(buf), "{%g, %g}", double(v.v.x), double(v.v.y));
      return buf;
    }
    case Type::Vector:
    case Type::Vector4:
    case Type::Color:
    case Type::Rotation: {
      char buf[128];
      snprintf(buf, sizeof(buf), "{%g, %g, %g}", double(v.v.x), double(v.v.y), double(v.v.z));
      return buf;
    }
    default: {
      char buf[64];
      snprintf(buf, sizeof(buf), "%g", double(v.as_float()));
      return buf;
    }
  }
}

struct FmtSpec {
  bool zero = false;
  bool left = false;
  bool plus = false;
  int width = -1;
  int prec = -1;
  char type = 0;
};

std::string apply_fmt_spec(const Value &v, const FmtSpec &spec, VMEnv &env)
{
  char type = spec.type;
  if (type == 0) {
    if (v.type == Type::String) {
      type = 's';
    }
    else if (v.type == Type::Int || v.type == Type::Bool) {
      type = 'd';
    }
    else if (v.type == Type::Float) {
      type = spec.prec >= 0 ? 'f' : 'g';
    }
    else {
      return pad_field(value_default_string(v, env), spec.width, false, spec.left);
    }
  }
  std::string body;
  if (type == 's') {
    body = v.type == Type::String ? std::string(value_string(v, env)) : value_default_string(v, env);
  }
  else if (type == 'd' || type == 'i') {
    body = std::to_string(v.as_int());
    if (spec.plus && v.as_int() >= 0) {
      body.insert(body.begin(), '+');
    }
  }
  else if (type == 'f' || type == 'F') {
    const int prec = spec.prec >= 0 ? spec.prec : 6;
    body = valuetostring_float(double(v.as_float()), 0, prec);
    if (spec.plus && !body.empty() && body[0] != '-') {
      body.insert(body.begin(), '+');
    }
  }
  else if (type == 'g' || type == 'G') {
    char buf[64];
    if (spec.prec >= 0) {
      snprintf(buf, sizeof(buf), "%.*g", spec.prec, double(v.as_float()));
    }
    else {
      snprintf(buf, sizeof(buf), "%g", double(v.as_float()));
    }
    body = buf;
    if (spec.plus && !body.empty() && body[0] != '-') {
      body.insert(body.begin(), '+');
    }
  }
  else {
    body = value_default_string(v, env);
  }
  return pad_field(std::move(body), spec.width, spec.zero, spec.left);
}

bool parse_fmt_spec(const StringRef spec, FmtSpec &out)
{
  int i = 0;
  const int n = int(spec.size());
  while (i < n && (spec[i] == '0' || spec[i] == '-' || spec[i] == '+' || spec[i] == ' ')) {
    if (spec[i] == '0') {
      out.zero = true;
    }
    else if (spec[i] == '-') {
      out.left = true;
    }
    else if (spec[i] == '+') {
      out.plus = true;
    }
    i++;
  }
  if (i < n && spec[i] >= '0' && spec[i] <= '9') {
    int w = 0;
    while (i < n && spec[i] >= '0' && spec[i] <= '9') {
      w = w * 10 + (spec[i] - '0');
      i++;
    }
    out.width = w;
  }
  if (i < n && spec[i] == '.') {
    i++;
    int p = 0;
    bool any = false;
    while (i < n && spec[i] >= '0' && spec[i] <= '9') {
      p = p * 10 + (spec[i] - '0');
      i++;
      any = true;
    }
    out.prec = any ? p : 0;
  }
  if (i < n && ((spec[i] >= 'a' && spec[i] <= 'z') || (spec[i] >= 'A' && spec[i] <= 'Z'))) {
    out.type = spec[i];
    i++;
  }
  return i == n;
}

bool format_has_python_field(const StringRef fmt)
{
  for (int i = 0; i < int(fmt.size()); i++) {
    if (fmt[i] == '{') {
      if (i + 1 < int(fmt.size()) && fmt[i + 1] == '{') {
        i++;
        continue;
      }
      return true;
    }
  }
  return false;
}

std::string apply_python_format(const StringRef fmt,
                                const Span<Value> args,
                                VMEnv &env,
                                std::string &error)
{
  std::string out;
  int auto_i = 0;
  for (int i = 0; i < int(fmt.size());) {
    const char c = fmt[i];
    if (c == '{') {
      if (i + 1 < int(fmt.size()) && fmt[i + 1] == '{') {
        out += '{';
        i += 2;
        continue;
      }
      const int start = i + 1;
      int j = start;
      while (j < int(fmt.size()) && fmt[j] != '}') {
        j++;
      }
      if (j >= int(fmt.size())) {
        error = "Unmatched '{' in format string";
        return {};
      }
      const StringRef field = fmt.substr(start, j - start);
      int idx = auto_i;
      StringRef spec_s;
      const int64_t colon = field.find(':');
      const StringRef index_s = colon == StringRef::not_found ? field : field.substr(0, colon);
      if (colon != StringRef::not_found) {
        spec_s = field.substr(colon + 1);
      }
      if (index_s.is_empty()) {
        auto_i++;
      }
      else {
        int parsed = 0;
        for (const char ch : index_s) {
          if (ch < '0' || ch > '9') {
            error = "Invalid replacement field in format string";
            return {};
          }
          parsed = parsed * 10 + (ch - '0');
        }
        idx = parsed;
      }
      if (idx < 0 || idx >= int(args.size())) {
        error = "format argument index out of range";
        return {};
      }
      FmtSpec spec;
      if (!spec_s.is_empty() && !parse_fmt_spec(spec_s, spec)) {
        error = "Invalid format specifier";
        return {};
      }
      out += apply_fmt_spec(args[idx], spec, env);
      i = j + 1;
      continue;
    }
    if (c == '}') {
      if (i + 1 < int(fmt.size()) && fmt[i + 1] == '}') {
        out += '}';
        i += 2;
        continue;
      }
      error = "Single '}' in format string";
      return {};
    }
    out += c;
    i++;
  }
  return out;
}

std::string apply_printf_format(const StringRef fmt,
                                const Span<Value> args,
                                VMEnv &env,
                                std::string &error)
{
  std::string out;
  int arg_i = 0;
  for (int i = 0; i < int(fmt.size());) {
    if (fmt[i] != '%') {
      out += fmt[i];
      i++;
      continue;
    }
    i++;
    if (i < int(fmt.size()) && fmt[i] == '%') {
      out += '%';
      i++;
      continue;
    }
    const int spec_start = i;
    while (i < int(fmt.size()) &&
           (fmt[i] == '0' || fmt[i] == '-' || fmt[i] == '+' || fmt[i] == ' ' || fmt[i] == '#'))
    {
      i++;
    }
    while (i < int(fmt.size()) && fmt[i] >= '0' && fmt[i] <= '9') {
      i++;
    }
    if (i < int(fmt.size()) && fmt[i] == '.') {
      i++;
      while (i < int(fmt.size()) && fmt[i] >= '0' && fmt[i] <= '9') {
        i++;
      }
    }
    if (i >= int(fmt.size())) {
      error = "Incomplete '%' in format string";
      return {};
    }
    const StringRef spec_s = fmt.substr(spec_start, i + 1 - spec_start);
    i++;
    FmtSpec spec;
    if (!parse_fmt_spec(spec_s, spec)) {
      error = "Invalid format specifier";
      return {};
    }
    if (arg_i >= int(args.size())) {
      error = "format argument index out of range";
      return {};
    }
    out += apply_fmt_spec(args[arg_i], spec, env);
    arg_i++;
  }
  return out;
}

Value format_values(const Span<Value> args, VMEnv &env, std::string &error)
{
  if (args.is_empty() || args[0].type != Type::String) {
    error = "format() first argument must be a string";
    return intern_runtime_string(env, "");
  }
  const std::string fmt(value_string(args[0], env));
  const Span<Value> rest = args.drop_front(1);
  std::string out;
  if (format_has_python_field(fmt)) {
    out = apply_python_format(fmt, rest, env, error);
  }
  else if (fmt.find('%') != std::string::npos) {
    out = apply_printf_format(fmt, rest, env, error);
  }
  else {
    out = fmt;
  }
  if (!error.empty()) {
    return intern_runtime_string(env, "");
  }
  return intern_runtime_string(env, out);
}

Value numeric_sub(const Value &a, const Value &b)
{
  if (a.type == Type::Matrix && b.type == Type::Matrix) {
    return Value::from_matrix(value_as_matrix(a) - value_as_matrix(b));
  }
  if (a.type == Type::Matrix3 && b.type == Type::Matrix3) {
    return Value::from_matrix3(value_as_matrix3(a) - value_as_matrix3(b));
  }
  if (a.type == Type::Matrix2 && b.type == Type::Matrix2) {
    return Value::from_matrix2(a.as_matrix2() - b.as_matrix2());
  }
  if (a.type == Type::Vector4 || b.type == Type::Vector4) {
    return Value::from_vec4(a.as_vec4() - b.as_vec4(), Type::Vector4);
  }
  if (a.type == Type::Vector || b.type == Type::Vector) {
    return Value::from_vec(a.as_vec() - b.as_vec());
  }
  if (a.type == Type::Vector2 || b.type == Type::Vector2) {
    return Value::from_vec2(a.as_vec2() - b.as_vec2());
  }
  if (a.type == Type::Int && b.type == Type::Int) {
    return Value::from_int(a.as_int() - b.as_int());
  }
  return Value::from_float(a.as_float() - b.as_float());
}

static bool str_ieq(const StringRef a, const char *b)
{
  int i = 0;
  for (; i < int(a.size()) && b[i] != '\0'; i++) {
    char ac = a[i];
    char bc = b[i];
    if (ac >= 'A' && ac <= 'Z') {
      ac = char(ac + 32);
    }
    if (bc >= 'A' && bc <= 'Z') {
      bc = char(bc + 32);
    }
    if (ac != bc) {
      return false;
    }
  }
  return i == int(a.size()) && b[i] == '\0';
}

static int parse_tex_dimension(const Value &v, const VMEnv &env, const int fallback)
{
  if (v.type == Type::String) {
    const StringRef s = value_string(v, env);
    if (str_ieq(s, "1") || str_ieq(s, "1d")) {
      return 1;
    }
    if (str_ieq(s, "2") || str_ieq(s, "2d")) {
      return 2;
    }
    if (str_ieq(s, "4") || str_ieq(s, "4d")) {
      return 4;
    }
    return 3;
  }
  const int d = v.as_int();
  return (d >= 1 && d <= 4) ? d : fallback;
}

static int parse_noise_type(const Value &v, const VMEnv &env)
{
  if (v.type == Type::String) {
    const StringRef s = value_string(v, env);
    if (str_ieq(s, "multifractal") || str_ieq(s, "multi_fractal")) {
      return SHD_NOISE_MULTIFRACTAL;
    }
    if (str_ieq(s, "hybrid") || str_ieq(s, "hybrid_multifractal")) {
      return SHD_NOISE_HYBRID_MULTIFRACTAL;
    }
    if (str_ieq(s, "ridged") || str_ieq(s, "ridged_multifractal")) {
      return SHD_NOISE_RIDGED_MULTIFRACTAL;
    }
    if (str_ieq(s, "hetero") || str_ieq(s, "hetero_terrain")) {
      return SHD_NOISE_HETERO_TERRAIN;
    }
    return SHD_NOISE_FBM;
  }
  const int t = v.as_int();
  return (t >= 0 && t <= 4) ? t : SHD_NOISE_FBM;
}

static int parse_voronoi_feature(const Value &v, const VMEnv &env)
{
  if (v.type == Type::String) {
    const StringRef s = value_string(v, env);
    if (str_ieq(s, "f2")) {
      return SHD_VORONOI_F2;
    }
    if (str_ieq(s, "smooth") || str_ieq(s, "smooth_f1")) {
      return SHD_VORONOI_SMOOTH_F1;
    }
    if (str_ieq(s, "edge") || str_ieq(s, "distance_to_edge")) {
      return SHD_VORONOI_DISTANCE_TO_EDGE;
    }
    if (str_ieq(s, "radius") || str_ieq(s, "n_sphere") || str_ieq(s, "n_sphere_radius")) {
      return SHD_VORONOI_N_SPHERE_RADIUS;
    }
    return SHD_VORONOI_F1;
  }
  const int t = v.as_int();
  return (t >= 0 && t <= 4) ? t : SHD_VORONOI_F1;
}

static int parse_voronoi_metric(const Value &v, const VMEnv &env)
{
  if (v.type == Type::String) {
    const StringRef s = value_string(v, env);
    if (str_ieq(s, "manhattan")) {
      return SHD_VORONOI_MANHATTAN;
    }
    if (str_ieq(s, "chebychev") || str_ieq(s, "chebyshev")) {
      return SHD_VORONOI_CHEBYCHEV;
    }
    if (str_ieq(s, "minkowski")) {
      return SHD_VORONOI_MINKOWSKI;
    }
    return SHD_VORONOI_EUCLIDEAN;
  }
  const int t = v.as_int();
  return (t >= 0 && t <= 3) ? t : SHD_VORONOI_EUCLIDEAN;
}

static float eval_noise(const float3 p,
                        const float scale,
                        const float detail,
                        const float roughness,
                        const float lacunarity,
                        const float distortion,
                        const int dim,
                        const int type)
{
  const float d = math::clamp(detail, 0.0f, 15.0f);
  const float r = math::max(roughness, 0.0f);
  const float3 s = p * scale;
  switch (dim) {
    case 1:
      return noise::perlin_fractal_distorted(
          s.x, d, r, lacunarity, 0.0f, 0.0f, distortion, type, true);
    case 2:
      return noise::perlin_fractal_distorted(
          float2(s.x, s.y), d, r, lacunarity, 0.0f, 0.0f, distortion, type, true);
    case 4:
      return noise::perlin_fractal_distorted(float4(s.x, s.y, s.z, 0.0f),
                                             d,
                                             r,
                                             lacunarity,
                                             0.0f,
                                             0.0f,
                                             distortion,
                                             type,
                                             true);
    default:
      return noise::perlin_fractal_distorted(
          s, d, r, lacunarity, 0.0f, 0.0f, distortion, type, true);
  }
}

static float3 distort_coord(const float3 p, const float distortion)
{
  if (distortion == 0.0f) {
    return p;
  }
  return p +
         float3(noise::perlin_signed(p),
                noise::perlin_signed(p + float3(1.0f, 0.0f, 0.0f)),
                noise::perlin_signed(p + float3(0.0f, 1.0f, 0.0f))) *
             distortion;
}

template<typename T> static float eval_voronoi_coord(const noise::VoronoiParams &params, const T coord)
{
  if (params.feature == SHD_VORONOI_DISTANCE_TO_EDGE) {
    return noise::fractal_voronoi_distance_to_edge(params, coord);
  }
  if (params.feature == SHD_VORONOI_N_SPHERE_RADIUS) {
    return noise::voronoi_n_sphere_radius(params, coord);
  }
  return noise::fractal_voronoi_x_fx(params, coord, false).distance;
}

static float eval_voronoi(const float3 p, const noise::VoronoiParams &params, const int dim)
{
  const float3 s = p * params.scale;
  switch (dim) {
    case 1:
      return eval_voronoi_coord(params, s.x);
    case 2:
      return eval_voronoi_coord(params, float2(s.x, s.y));
    case 4:
      return eval_voronoi_coord(params, float4(s.x, s.y, s.z, 0.0f));
    default:
      return eval_voronoi_coord(params, s);
  }
}

static float4x4 matrix_from_diag_scale(const float3 s)
{
  /* xyz diagonal only; m44 (homogeneous) stays 1. */
  return math::from_scale<float4x4>(s);
}

static float4x4 matrix_from_uniform_scale(const float s)
{
  return matrix_from_diag_scale(float3(s));
}

static Value mul_mat_vec(const Value &m, const Value &v, const bool row_vec, std::string *err)
{
  const int md = type_linear_dim(m.type);
  const int vd = type_linear_dim(v.type);
  if (md == 0 || vd == 0 || md != vd) {
    if (err) {
      *err = "矩阵×向量维度不一致";
    }
    return Value::from_float(0.0f);
  }
  if (md == 2) {
    const float2x2 A = m.as_matrix2();
    const float2 x = v.as_vec2();
    return Value::from_vec2(row_vec ? (x * A) : (A * x));
  }
  if (md == 3) {
    const float3x3 A = value_as_matrix3(m);
    const float3 x = v.as_vec();
    return Value::from_vec(row_vec ? (x * A) : (A * x));
  }
  const float4x4 A = value_as_matrix(m);
  const float4 x = v.as_vec4();
  return Value::from_vec4(row_vec ? (x * A) : (A * x), Type::Vector4);
}

Value numeric_mul(const Value &a, const Value &b, std::string *err = nullptr)
{
  if (a.type == Type::Rotation && b.type == Type::Rotation) {
    return value_from_quat(value_as_quat(a) * value_as_quat(b));
  }
  if (a.type == Type::Rotation && type_is_vec_like(b.type) && b.type != Type::Rotation) {
    const float3 r = math::transform_point(value_as_quat(a), b.as_vec());
    if (b.type == Type::Vector2) {
      return Value::from_vec2(float2(r.x, r.y));
    }
    if (b.type == Type::Vector4) {
      return Value::from_vec4(float4(r.x, r.y, r.z, b.v.w), Type::Vector4);
    }
    return Value::from_vec(r);
  }
  if (b.type == Type::Rotation && type_is_vec_like(a.type) && a.type != Type::Rotation) {
    const float3 r = math::transform_point(value_as_quat(b), a.as_vec());
    if (a.type == Type::Vector2) {
      return Value::from_vec2(float2(r.x, r.y));
    }
    if (a.type == Type::Vector4) {
      return Value::from_vec4(float4(r.x, r.y, r.z, a.v.w), Type::Vector4);
    }
    return Value::from_vec(r);
  }
  if (a.type == Type::Matrix && b.type == Type::Matrix) {
    return Value::from_matrix(value_as_matrix(a) * value_as_matrix(b));
  }
  if (a.type == Type::Matrix3 && b.type == Type::Matrix3) {
    return Value::from_matrix3(value_as_matrix3(a) * value_as_matrix3(b));
  }
  if (a.type == Type::Matrix2 && b.type == Type::Matrix2) {
    return Value::from_matrix2(a.as_matrix2() * b.as_matrix2());
  }
  if (type_is_matrix(a.type) && type_is_matrix(b.type)) {
    if (err) {
      *err = "矩阵乘法维度不一致：两边阶数必须相同";
    }
    return Value::from_float(0.0f);
  }
  if (a.type == Type::Matrix && ELEM(b.type, Type::Float, Type::Int, Type::Bool)) {
    return Value::from_matrix(value_as_matrix(a) * matrix_from_uniform_scale(b.as_float()));
  }
  if (b.type == Type::Matrix && ELEM(a.type, Type::Float, Type::Int, Type::Bool)) {
    return Value::from_matrix(matrix_from_uniform_scale(a.as_float()) * value_as_matrix(b));
  }
  if (a.type == Type::Matrix3 && ELEM(b.type, Type::Float, Type::Int, Type::Bool)) {
    return Value::from_matrix3(value_as_matrix3(a) * float3x3::diagonal(b.as_float()));
  }
  if (b.type == Type::Matrix3 && ELEM(a.type, Type::Float, Type::Int, Type::Bool)) {
    return Value::from_matrix3(float3x3::diagonal(a.as_float()) * value_as_matrix3(b));
  }
  if (a.type == Type::Matrix2 && ELEM(b.type, Type::Float, Type::Int, Type::Bool)) {
    return Value::from_matrix2(a.as_matrix2() * float2x2::diagonal(b.as_float()));
  }
  if (b.type == Type::Matrix2 && ELEM(a.type, Type::Float, Type::Int, Type::Bool)) {
    return Value::from_matrix2(float2x2::diagonal(a.as_float()) * b.as_matrix2());
  }
  if (type_is_matrix(a.type) && type_is_vec_like(b.type)) {
    return mul_mat_vec(a, b, false, err);
  }
  if (type_is_vec_like(a.type) && type_is_matrix(b.type)) {
    return mul_mat_vec(b, a, true, err);
  }
  if (a.type == Type::Vector4 && b.type == Type::Vector4) {
    return Value::from_vec4(a.as_vec4() * b.as_vec4(), Type::Vector4);
  }
  if (a.type == Type::Vector4) {
    return Value::from_vec4(a.as_vec4() * b.as_float(), Type::Vector4);
  }
  if (b.type == Type::Vector4) {
    return Value::from_vec4(b.as_vec4() * a.as_float(), Type::Vector4);
  }
  if (a.type == Type::Vector && b.type == Type::Vector) {
    return Value::from_vec(a.as_vec() * b.as_vec());
  }
  if (a.type == Type::Vector) {
    return Value::from_vec(a.as_vec() * b.as_float());
  }
  if (b.type == Type::Vector) {
    return Value::from_vec(b.as_vec() * a.as_float());
  }
  if (a.type == Type::Vector2 && b.type == Type::Vector2) {
    return Value::from_vec2(a.as_vec2() * b.as_vec2());
  }
  if (a.type == Type::Vector2) {
    return Value::from_vec2(a.as_vec2() * b.as_float());
  }
  if (b.type == Type::Vector2) {
    return Value::from_vec2(b.as_vec2() * a.as_float());
  }
  if (a.type == Type::Int && b.type == Type::Int) {
    return Value::from_int(a.as_int() * b.as_int());
  }
  return Value::from_float(a.as_float() * b.as_float());
}

Value numeric_div(const Value &a, const Value &b)
{
  if (a.type == Type::Vector4 && b.type == Type::Vector4) {
    return Value::from_vec4(a.as_vec4() / b.as_vec4(), Type::Vector4);
  }
  if (a.type == Type::Vector4) {
    const float s = b.as_float();
    return Value::from_vec4(s != 0.0f ? a.as_vec4() / s : float4(0.0f), Type::Vector4);
  }
  if (a.type == Type::Vector && b.type == Type::Vector) {
    return Value::from_vec(a.as_vec() / b.as_vec());
  }
  if (a.type == Type::Vector) {
    const float s = b.as_float();
    return Value::from_vec(s != 0.0f ? a.as_vec() / s : float3(0.0f));
  }
  if (a.type == Type::Vector2 && b.type == Type::Vector2) {
    return Value::from_vec2(a.as_vec2() / b.as_vec2());
  }
  if (a.type == Type::Vector2) {
    const float s = b.as_float();
    return Value::from_vec2(s != 0.0f ? a.as_vec2() / s : float2(0.0f));
  }
  if (a.type == Type::Int && b.type == Type::Int) {
    const int d = b.as_int();
    return Value::from_int(d != 0 ? a.as_int() / d : 0);
  }
  const float d = b.as_float();
  return Value::from_float(d != 0.0f ? a.as_float() / d : 0.0f);
}

thread_local Value tls_decomp_u;
thread_local Value tls_decomp_s;
thread_local Value tls_decomp_v;

static void eigen_sym_2(const float2x2 &m, float2 &evals, float2x2 &evecs)
{
  const float a = m[0][0];
  const float b = 0.5f * (m[0][1] + m[1][0]);
  const float c = m[1][1];
  const float disc = sqrtf(std::max((a - c) * (a - c) + 4.0f * b * b, 0.0f));
  evals.x = 0.5f * (a + c + disc);
  evals.y = 0.5f * (a + c - disc);
  if (fabsf(b) > 1e-8f || fabsf(evals.x - a) > 1e-8f) {
    float2 v0 = math::normalize(float2(b, evals.x - a));
    if (math::length(v0) < 0.5f) {
      v0 = math::normalize(float2(evals.x - c, b));
    }
    evecs = float2x2(v0, float2(-v0.y, v0.x));
  }
  else {
    evecs = float2x2::identity();
  }
}

static void svd_2(const float2x2 &A, float2x2 &U, float2 &s, float2x2 &V)
{
  const float2x2 AtA = math::transpose(A) * A;
  float2 ev;
  eigen_sym_2(AtA, ev, V);
  s.x = sqrtf(std::max(ev.x, 0.0f));
  s.y = sqrtf(std::max(ev.y, 0.0f));
  const float2x2 Sinv(float2(s.x > 1e-8f ? 1.0f / s.x : 0.0f, 0.0f),
                      float2(0.0f, s.y > 1e-8f ? 1.0f / s.y : 0.0f));
  U = A * V * Sinv;
  const float2 u0 = math::normalize(U[0]);
  float2 u1 = U[1];
  if (math::length(u1) < 1e-6f) {
    u1 = float2(-u0.y, u0.x);
  }
  else {
    u1 = math::normalize(u1);
  }
  U = float2x2(u0, u1);
}

static void fill_decomp_from_matrix(const Value &m)
{
  tls_decomp_u = Value::from_matrix2(float2x2::identity());
  tls_decomp_s = Value::from_vec2(float2(1.0f));
  tls_decomp_v = tls_decomp_u;
  if (m.type == Type::Matrix2) {
    float2x2 U, V;
    float2 s;
    svd_2(m.as_matrix2(), U, s, V);
    tls_decomp_u = Value::from_matrix2(U);
    tls_decomp_s = Value::from_vec2(s);
    tls_decomp_v = Value::from_matrix2(V);
    return;
  }
  if (m.type == Type::Matrix3) {
    const float3x3 A = value_as_matrix3(m);
    float U[3][3], S[3], V[3][3];
    float src[3][3];
    memcpy(src, A.base_ptr(), sizeof(src));
    BLI_svd_m3(src, U, S, V);
    tls_decomp_u = Value::from_matrix3(float3x3(&U[0][0]));
    tls_decomp_s = Value::from_vec(float3(S[0], S[1], S[2]));
    tls_decomp_v = Value::from_matrix3(float3x3(&V[0][0]));
    return;
  }
  float U[4][4], s[4], V[4][4], src[4][4];
  memcpy(src, value_as_matrix(m).base_ptr(), sizeof(src));
  svd_m4(U, s, V, src);
  tls_decomp_u = Value::from_matrix(float4x4(&U[0][0]));
  tls_decomp_s = Value::from_vec4(float4(s[0], s[1], s[2], s[3]), Type::Vector4);
  tls_decomp_v = Value::from_matrix(float4x4(&V[0][0]));
}

static void fill_polar_from_matrix(const Value &m)
{
  if (m.type == Type::Matrix2) {
    float2x2 U, V;
    float2 s;
    svd_2(m.as_matrix2(), U, s, V);
    const float2x2 Vt = math::transpose(V);
    const float2x2 R = U * Vt;
    const float2x2 S = V * float2x2(float2(s.x, 0.0f), float2(0.0f, s.y)) * Vt;
    tls_decomp_u = Value::from_matrix2(R);
    tls_decomp_s = Value::from_matrix2(S);
    tls_decomp_v = tls_decomp_s;
    return;
  }
  float3x3 A;
  if (m.type == Type::Matrix3) {
    A = value_as_matrix3(m);
  }
  else {
    A = float3x3(value_as_matrix(m));
  }
  float src[3][3], R[3][3], P[3][3];
  memcpy(src, A.base_ptr(), sizeof(src));
  mat3_polar_decompose(src, R, P);
  if (m.type == Type::Matrix3) {
    tls_decomp_u = Value::from_matrix3(float3x3(&R[0][0]));
    tls_decomp_s = Value::from_matrix3(float3x3(&P[0][0]));
  }
  else {
    float4x4 Rm = float4x4(float3x3(&R[0][0]));
    float4x4 Pm = float4x4(float3x3(&P[0][0]));
    Rm.location() = value_as_matrix(m).location();
    tls_decomp_u = Value::from_matrix(Rm);
    tls_decomp_s = Value::from_matrix(Pm);
  }
  tls_decomp_v = tls_decomp_s;
}

static void fill_eigen_from_matrix(const Value &m)
{
  if (m.type == Type::Matrix2) {
    float2 evals;
    float2x2 evecs;
    eigen_sym_2(m.as_matrix2(), evals, evecs);
    tls_decomp_s = Value::from_vec2(evals);
    tls_decomp_u = Value::from_matrix2(evecs);
    tls_decomp_v = tls_decomp_u;
    return;
  }
  float3x3 A;
  if (m.type == Type::Matrix3) {
    A = value_as_matrix3(m);
  }
  else {
    A = float3x3(value_as_matrix(m));
  }
  A = (A + math::transpose(A)) * 0.5f;
  float src[3][3], evals[3], evecs[3][3];
  memcpy(src, A.base_ptr(), sizeof(src));
  BLI_eigen_solve_selfadjoint_m3(src, evals, evecs);
  tls_decomp_s = Value::from_vec(float3(evals[0], evals[1], evals[2]));
  if (m.type == Type::Matrix3) {
    tls_decomp_u = Value::from_matrix3(float3x3(&evecs[0][0]));
  }
  else {
    tls_decomp_u = Value::from_matrix(float4x4(float3x3(&evecs[0][0])));
  }
  tls_decomp_v = tls_decomp_u;
}

template<typename Fn> static Value map_unary(const Value &v, const Fn &fn)
{
  if (v.type == Type::Vector2) {
    const float2 x = v.as_vec2();
    return Value::from_vec2(float2(fn(x.x), fn(x.y)));
  }
  if (v.type == Type::Vector) {
    const float3 x = v.as_vec();
    return Value::from_vec(float3(fn(x.x), fn(x.y), fn(x.z)));
  }
  if (ELEM(v.type, Type::Vector4, Type::Color, Type::Rotation)) {
    const float4 x = v.as_vec4();
    return Value::from_vec4(float4(fn(x.x), fn(x.y), fn(x.z), fn(x.w)), v.type);
  }
  return Value::from_float(fn(v.as_float()));
}

template<typename Fn> static Value map_binary(const Value &a, const Value &b, const Fn &fn)
{
  const Type vec = dominant_vec_type(a.type, b.type);
  if (vec == Type::Vector2) {
    const float2 x = a.as_vec2();
    const float2 y = b.as_vec2();
    return Value::from_vec2(float2(fn(x.x, y.x), fn(x.y, y.y)));
  }
  if (vec == Type::Vector) {
    const float3 x = a.as_vec();
    const float3 y = b.as_vec();
    return Value::from_vec(float3(fn(x.x, y.x), fn(x.y, y.y), fn(x.z, y.z)));
  }
  if (vec == Type::Vector4 || vec == Type::Color) {
    const float4 x = a.as_vec4();
    const float4 y = b.as_vec4();
    return Value::from_vec4(float4(fn(x.x, y.x), fn(x.y, y.y), fn(x.z, y.z), fn(x.w, y.w)), vec);
  }
  return Value::from_float(fn(a.as_float(), b.as_float()));
}

template<typename Fn>
static Value map_ternary(const Value &a, const Value &b, const Value &c, const Fn &fn)
{
  const Type vec = dominant_vec_type(a.type, dominant_vec_type(b.type, c.type));
  if (vec == Type::Vector2) {
    const float2 x = a.as_vec2();
    const float2 y = b.as_vec2();
    const float2 z = c.as_vec2();
    return Value::from_vec2(float2(fn(x.x, y.x, z.x), fn(x.y, y.y, z.y)));
  }
  if (vec == Type::Vector) {
    const float3 x = a.as_vec();
    const float3 y = b.as_vec();
    const float3 z = c.as_vec();
    return Value::from_vec(float3(fn(x.x, y.x, z.x), fn(x.y, y.y, z.y), fn(x.z, y.z, z.z)));
  }
  if (vec == Type::Vector4 || vec == Type::Color) {
    const float4 x = a.as_vec4();
    const float4 y = b.as_vec4();
    const float4 z = c.as_vec4();
    return Value::from_vec4(
        float4(fn(x.x, y.x, z.x), fn(x.y, y.y, z.y), fn(x.z, y.z, z.z), fn(x.w, y.w, z.w)), vec);
  }
  return Value::from_float(fn(a.as_float(), b.as_float(), c.as_float()));
}

Value call_builtin(const Builtin id,
                   const Span<Value> args,
                   VMEnv &env,
                   std::string &error)
{
  auto arg = [&](const int i) -> const Value & {
    static const Value zero = Value::from_float(0.0f);
    return i < args.size() ? args[i] : zero;
  };

  switch (id) {
    case Builtin::Sin:
      return map_unary(arg(0), [](const float x) { return sinf(x); });
    case Builtin::Cos:
      return map_unary(arg(0), [](const float x) { return cosf(x); });
    case Builtin::Tan:
      return map_unary(arg(0), [](const float x) { return tanf(x); });
    case Builtin::Asin:
      return map_unary(arg(0), [](const float x) { return asinf(x); });
    case Builtin::Acos:
      return map_unary(arg(0), [](const float x) { return acosf(x); });
    case Builtin::Atan:
      return map_unary(arg(0), [](const float x) { return atanf(x); });
    case Builtin::Atan2:
      return map_binary(arg(0), arg(1), [](const float y, const float x) { return atan2f(y, x); });
    case Builtin::Abs: {
      if (arg(0).type == Type::Int) {
        return Value::from_int(abs(arg(0).as_int()));
      }
      return map_unary(arg(0), [](const float x) { return fabsf(x); });
    }
    case Builtin::Floor:
      return map_unary(arg(0), [](const float x) { return floorf(x); });
    case Builtin::Ceil:
      return map_unary(arg(0), [](const float x) { return ceilf(x); });
    case Builtin::Round:
      return map_unary(arg(0), [](const float x) { return roundf(x); });
    case Builtin::Trunc:
      return map_unary(arg(0), [](const float x) { return truncf(x); });
    case Builtin::Sqrt:
      return map_unary(arg(0), [](const float x) { return sqrtf(std::max(x, 0.0f)); });
    case Builtin::Exp:
      return map_unary(arg(0), [](const float x) { return expf(x); });
    case Builtin::Log:
      return map_unary(arg(0), [](const float x) { return logf(std::max(x, 1e-30f)); });
    case Builtin::Pow:
      return map_binary(arg(0), arg(1), [](const float x, const float y) { return powf(x, y); });
    case Builtin::Min:
      if (arg(0).type == Type::Int && arg(1).type == Type::Int &&
          dominant_vec_type(arg(0).type, arg(1).type) == Type::Void)
      {
        return Value::from_int(std::min(arg(0).as_int(), arg(1).as_int()));
      }
      return map_binary(arg(0), arg(1), [](const float x, const float y) { return std::min(x, y); });
    case Builtin::Max:
      if (arg(0).type == Type::Int && arg(1).type == Type::Int &&
          dominant_vec_type(arg(0).type, arg(1).type) == Type::Void)
      {
        return Value::from_int(std::max(arg(0).as_int(), arg(1).as_int()));
      }
      return map_binary(arg(0), arg(1), [](const float x, const float y) { return std::max(x, y); });
    case Builtin::Clamp: {
      const Value lo = args.size() > 1 ? arg(1) : Value::from_float(0.0f);
      const Value hi = args.size() > 2 ? arg(2) : Value::from_float(1.0f);
      if (type_is_vec_like(arg(0).type)) {
        auto clampf = [&](const float x, const float a, const float b) {
          return std::clamp(x, a, b);
        };
        const Type vec = dominant_vec_type(arg(0).type, dominant_vec_type(lo.type, hi.type));
        const Type out_ty = (vec == Type::Void) ? arg(0).type : vec;
        if (out_ty == Type::Vector2) {
          const float2 x = arg(0).as_vec2();
          const float2 a = lo.as_vec2();
          const float2 b = hi.as_vec2();
          return Value::from_vec2(float2(clampf(x.x, a.x, b.x), clampf(x.y, a.y, b.y)));
        }
        if (out_ty == Type::Vector) {
          const float3 x = arg(0).as_vec();
          const float3 a = lo.as_vec();
          const float3 b = hi.as_vec();
          return Value::from_vec(
              float3(clampf(x.x, a.x, b.x), clampf(x.y, a.y, b.y), clampf(x.z, a.z, b.z)));
        }
        const float4 x = arg(0).as_vec4();
        const float4 a = lo.as_vec4();
        const float4 b = hi.as_vec4();
        return Value::from_vec4(float4(clampf(x.x, a.x, b.x),
                                        clampf(x.y, a.y, b.y),
                                        clampf(x.z, a.z, b.z),
                                        clampf(x.w, a.w, b.w)),
                                 arg(0).type == Type::Color ? Type::Color : Type::Vector4);
      }
      return Value::from_float(std::clamp(arg(0).as_float(), lo.as_float(), hi.as_float()));
    }
    case Builtin::Length:
      if (type_is_array(arg(0).type)) {
        return Value::from_int(array_len(arg(0)));
      }
      if (arg(0).type == Type::Vector2) {
        return Value::from_float(math::length(arg(0).as_vec2()));
      }
      if (arg(0).type == Type::Vector4) {
        return Value::from_float(math::length(arg(0).as_vec4()));
      }
      return Value::from_float(math::length(arg(0).as_vec()));
    case Builtin::Distance: {
      const Type vec = dominant_vec_type(arg(0).type, arg(1).type);
      if (vec == Type::Vector2) {
        return Value::from_float(math::distance(arg(0).as_vec2(), arg(1).as_vec2()));
      }
      if (vec == Type::Vector4 || vec == Type::Color) {
        return Value::from_float(math::distance(arg(0).as_vec4(), arg(1).as_vec4()));
      }
      return Value::from_float(math::distance(arg(0).as_vec(), arg(1).as_vec()));
    }
    case Builtin::Dot:
      if (arg(0).type == Type::Vector2 || arg(1).type == Type::Vector2) {
        return Value::from_float(math::dot(arg(0).as_vec2(), arg(1).as_vec2()));
      }
      if (arg(0).type == Type::Vector4 || arg(1).type == Type::Vector4) {
        return Value::from_float(math::dot(arg(0).as_vec4(), arg(1).as_vec4()));
      }
      return Value::from_float(math::dot(arg(0).as_vec(), arg(1).as_vec()));
    case Builtin::Cross: {
      if (arg(0).type == Type::Vector2 || arg(1).type == Type::Vector2) {
        return Value::from_float(math::cross(arg(0).as_vec2(), arg(1).as_vec2()));
      }
      if (arg(0).type == Type::Vector4 || arg(1).type == Type::Vector4) {
        const float3 c = math::cross(arg(0).as_vec(), arg(1).as_vec());
        return Value::from_vec4(float4(c.x, c.y, c.z, 0.0f), Type::Vector4);
      }
      return Value::from_vec(math::cross(arg(0).as_vec(), arg(1).as_vec()));
    }
    case Builtin::Normalize:
      if (arg(0).type == Type::Vector2) {
        return Value::from_vec2(math::normalize(arg(0).as_vec2()));
      }
      if (arg(0).type == Type::Vector4) {
        return Value::from_vec4(math::normalize(arg(0).as_vec4()), Type::Vector4);
      }
      return Value::from_vec(math::normalize(arg(0).as_vec()));
    case Builtin::Radians:
      return map_unary(arg(0), [](const float x) { return x * float(M_PI / 180.0); });
    case Builtin::Degrees:
      return map_unary(arg(0), [](const float x) { return x * float(180.0 / M_PI); });
    case Builtin::Sign:
      return map_unary(arg(0), [](const float x) {
        return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f);
      });
    case Builtin::Fract:
      return map_unary(arg(0), [](const float x) { return x - floorf(x); });
    case Builtin::Ddx:
    case Builtin::Ddy:
    case Builtin::Fwidth:
      /* Screen-space derivatives are EEVEE fragment only. CPU / geometry wrangle → 0. */
      return map_unary(arg(0), [](const float /*x*/) { return 0.0f; });
    case Builtin::Smooth: {
      auto hermite = [](const float x) {
        const float t = std::clamp(x, 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
      };
      if (args.size() >= 3) {
        return map_ternary(arg(0), arg(1), arg(2), [&](const float a, const float b, const float v) {
          const float den = b - a;
          if (fabsf(den) < 1e-8f) {
            return v >= b ? 1.0f : 0.0f;
          }
          return hermite((v - a) / den);
        });
      }
      return map_unary(arg(0), hermite);
    }
    case Builtin::Map: {
      auto remap = [](const float v, const float a, const float b, const float c, const float d) {
        const float den = b - a;
        if (fabsf(den) < 1e-8f) {
          return c;
        }
        return c + (v - a) * (d - c) / den;
      };
      const Value &v = arg(0);
      const Value &a = arg(1);
      const Value &b = arg(2);
      const Value &c = arg(3);
      const Value &d = arg(4);
      if (type_is_vec_like(v.type)) {
        if (v.type == Type::Vector2) {
          const float2 x = v.as_vec2();
          const float2 lo = a.as_vec2();
          const float2 hi = b.as_vec2();
          const float2 t0 = c.as_vec2();
          const float2 t1 = d.as_vec2();
          return Value::from_vec2(float2(remap(x.x, lo.x, hi.x, t0.x, t1.x),
                                          remap(x.y, lo.y, hi.y, t0.y, t1.y)));
        }
        if (v.type == Type::Vector) {
          const float3 x = v.as_vec();
          const float3 lo = a.as_vec();
          const float3 hi = b.as_vec();
          const float3 t0 = c.as_vec();
          const float3 t1 = d.as_vec();
          return Value::from_vec(float3(remap(x.x, lo.x, hi.x, t0.x, t1.x),
                                         remap(x.y, lo.y, hi.y, t0.y, t1.y),
                                         remap(x.z, lo.z, hi.z, t0.z, t1.z)));
        }
        const float4 x = v.as_vec4();
        const float4 lo = a.as_vec4();
        const float4 hi = b.as_vec4();
        const float4 t0 = c.as_vec4();
        const float4 t1 = d.as_vec4();
        return Value::from_vec4(float4(remap(x.x, lo.x, hi.x, t0.x, t1.x),
                                         remap(x.y, lo.y, hi.y, t0.y, t1.y),
                                         remap(x.z, lo.z, hi.z, t0.z, t1.z),
                                         remap(x.w, lo.w, hi.w, t0.w, t1.w)),
                                  v.type == Type::Color ? Type::Color : Type::Vector4);
      }
      return Value::from_float(
          remap(v.as_float(), a.as_float(), b.as_float(), c.as_float(), d.as_float()));
    }
    case Builtin::Mix: {
      const float t = arg(2).as_float();
      if (arg(0).type == Type::Vector4 || arg(1).type == Type::Vector4) {
        return Value::from_vec4(arg(0).as_vec4() * (1.0f - t) + arg(1).as_vec4() * t, Type::Vector4);
      }
      if (arg(0).type == Type::Vector || arg(1).type == Type::Vector) {
        return Value::from_vec(arg(0).as_vec() * (1.0f - t) + arg(1).as_vec() * t);
      }
      if (arg(0).type == Type::Vector2 || arg(1).type == Type::Vector2) {
        return Value::from_vec2(arg(0).as_vec2() * (1.0f - t) + arg(1).as_vec2() * t);
      }
      return Value::from_float(arg(0).as_float() * (1.0f - t) + arg(1).as_float() * t);
    }
    case Builtin::Noise: {
      const float3 p = arg(0).as_vec();
      if (args.size() < 2) {
        return Value::from_float(noise::perlin_signed(p));
      }
      const float scale = args.size() > 1 ? arg(1).as_float() : 5.0f;
      const float detail = args.size() > 2 ? arg(2).as_float() : 2.0f;
      const float roughness = args.size() > 3 ? arg(3).as_float() : 0.5f;
      const float lacunarity = args.size() > 4 ? arg(4).as_float() : 2.0f;
      const float distortion = args.size() > 5 ? arg(5).as_float() : 0.0f;
      const int dim = args.size() > 6 ? parse_tex_dimension(arg(6), env, 3) : 3;
      const int type = args.size() > 7 ? parse_noise_type(arg(7), env) : SHD_NOISE_FBM;
      return Value::from_float(
          eval_noise(p, scale, detail, roughness, lacunarity, distortion, dim, type));
    }
    case Builtin::Voronoi: {
      noise::VoronoiParams params;
      params.scale = args.size() > 1 ? arg(1).as_float() : 5.0f;
      params.detail = args.size() > 2 ? arg(2).as_float() : 0.0f;
      params.roughness = args.size() > 3 ? arg(3).as_float() : 0.5f;
      params.lacunarity = args.size() > 4 ? arg(4).as_float() : 2.0f;
      params.smoothness = args.size() > 5 ? arg(5).as_float() : 1.0f;
      params.randomness = args.size() > 6 ? arg(6).as_float() : 1.0f;
      params.feature = args.size() > 7 ? parse_voronoi_feature(arg(7), env) : SHD_VORONOI_F1;
      params.metric = args.size() > 8 ? parse_voronoi_metric(arg(8), env) : SHD_VORONOI_EUCLIDEAN;
      const int dim = args.size() > 9 ? parse_tex_dimension(arg(9), env, 3) : 3;
      const float distortion = args.size() > 10 ? arg(10).as_float() : 0.0f;
      params.exponent = 0.5f;
      params.normalize = true;
      params.randomness = std::min(std::max(params.randomness, 0.0f), 1.0f);
      params.smoothness = (params.feature == SHD_VORONOI_SMOOTH_F1) ?
                              std::min(std::max(params.smoothness / 2.0f, 0.0f), 0.5f) :
                              0.0f;
      params.max_distance = (0.5f + 0.5f * params.randomness) *
                            ((params.feature == SHD_VORONOI_F2) ? 2.0f : 1.0f);
      const float3 p = distort_coord(arg(0).as_vec(), distortion);
      return Value::from_float(eval_voronoi(p, params, dim));
    }
    case Builtin::ColorFn: {
      if (args.is_empty()) {
        return Value::from_vec4(float4(0.0f, 0.0f, 0.0f, 1.0f), Type::Color);
      }
      if (args.size() >= 4) {
        return Value::from_vec4(
            float4(arg(0).as_float(), arg(1).as_float(), arg(2).as_float(), arg(3).as_float()),
            Type::Color);
      }
      if (args.size() == 3) {
        return Value::from_vec4(
            float4(arg(0).as_float(), arg(1).as_float(), arg(2).as_float(), 1.0f), Type::Color);
      }
      const Value &v = arg(0);
      if (v.type == Type::Color) {
        return v;
      }
      const float a = (v.type == Type::Rotation || v.v.w != 0.0f) ? v.v.w : 1.0f;
      return Value::from_vec4(float4(v.v.x, v.v.y, v.v.z, a), Type::Color);
    }
    case Builtin::Hash:
    case Builtin::Rand: {
      auto seed_u32 = [&](const Value &v) -> uint32_t {
        if (v.type == Type::String) {
          const StringRef s = blender::nodes::vex::value_string(v, env);
          uint32_t h = 2166136261u;
          for (const char c : s) {
            h ^= uint32_t(uint8_t(c));
            h *= 16777619u;
          }
          return h;
        }
        if (v.type == Type::Int || v.type == Type::Bool) {
          return uint32_t(v.as_int());
        }
        if (type_is_vec_like(v.type)) {
          return noise::hash_float(v.as_vec());
        }
        return noise::hash_float(v.as_float());
      };
      uint32_t h = 0;
      if (args.is_empty()) {
        h = uint32_t(env.index) ^ (uint32_t(env.rand_seq++) * 747796405u);
      }
      else if (args.size() == 1) {
        h = seed_u32(arg(0));
      }
      else {
        h = seed_u32(arg(0)) ^ (seed_u32(arg(1)) * 747796405u);
      }
      if (id == Builtin::Hash) {
        return Value::from_int(int(h));
      }
      return Value::from_float(noise::hash_to_float(h));
    }
    case Builtin::Npoints:
    case Builtin::Nedges:
    case Builtin::Nfaces:
    case Builtin::Ncorners:
      if (!args.is_empty() && env.topo_fn) {
        return env.topo_fn(env.topo_user, int(id), args, env, error);
      }
      if (id == Builtin::Npoints) {
        return Value::from_int(env.npoints);
      }
      if (id == Builtin::Nedges) {
        return Value::from_int(env.nedges);
      }
      if (id == Builtin::Nfaces) {
        return Value::from_int(env.nfaces);
      }
      return Value::from_int(env.ncorners);
    case Builtin::Point:
    case Builtin::Edge:
    case Builtin::Face:
    case Builtin::Corner:
    case Builtin::Curve:
    case Builtin::Instance: {
      int domain = 0;
      if (id == Builtin::Curve) {
        domain = 5;
      }
      else if (id == Builtin::Instance) {
        domain = 4;
      }
      else {
        domain = int(id) - int(Builtin::Point);
      }
      int geo_index = 0;
      StringRef name;
      int index = env.index;
      if (args.size() >= 3) {
        geo_index = arg(0).as_int();
        name = blender::nodes::vex::value_string(arg(1), env);
        index = arg(2).as_int();
      }
      else if (args.size() >= 2) {
        if (arg(0).type == Type::String) {
          name = blender::nodes::vex::value_string(arg(0), env);
          index = arg(1).as_int();
        }
        else {
          geo_index = arg(0).as_int();
          name = blender::nodes::vex::value_string(arg(1), env);
        }
      }
      else if (args.size() == 1) {
        if (arg(0).type == Type::String) {
          name = blender::nodes::vex::value_string(arg(0), env);
        }
        else {
          index = arg(0).as_int();
        }
      }
      if (name.is_empty()) {
        name = "P";
      }
      if (!env.load_elem) {
        return Value::from_float(0.0f);
      }
      const Type expect = (name == "P" || name == "position" || name == "N" || name == "normal") ?
                              Type::Vector :
                              Type::Float;
      return env.load_elem(env.elem_user, geo_index, domain, name, index, expect, error);
    }
    case Builtin::Set: {
      float c[8] = {};
      int n = 0;
      auto push = [&](const float f) {
        if (n < 8) {
          c[n++] = f;
        }
      };
      for (int i = 0; i < int(args.size()); i++) {
        const Value a = arg(i);
        switch (a.type) {
          case Type::Vector2:
            push(a.v.x);
            push(a.v.y);
            break;
          case Type::Vector:
            push(a.v.x);
            push(a.v.y);
            push(a.v.z);
            break;
          case Type::Vector4:
          case Type::Color:
          case Type::Rotation:
            push(a.v.x);
            push(a.v.y);
            push(a.v.z);
            push(a.v.w);
            break;
          default:
            push(a.as_float());
            break;
        }
      }
      if (n <= 2) {
        return Value::from_vec2(float2(n > 0 ? c[0] : 0.0f, n > 1 ? c[1] : 0.0f));
      }
      if (n == 3) {
        return Value::from_vec(float3(c[0], c[1], c[2]));
      }
      return Value::from_vec4(float4(c[0], c[1], c[2], c[3]), Type::Vector4);
    }
    case Builtin::Vec3Fn:
    case Builtin::Vec2Fn:
    case Builtin::Vec4Fn: {
      const int want = (id == Builtin::Vec2Fn) ? 2 : (id == Builtin::Vec4Fn) ? 4 : 3;
      Vector<float, 16> comps;
      for (int i = 0; i < int(args.size()); i++) {
        const Value a = arg(i);
        switch (a.type) {
          case Type::Vector2:
            comps.append(a.v.x);
            comps.append(a.v.y);
            break;
          case Type::Vector:
            comps.append(a.v.x);
            comps.append(a.v.y);
            comps.append(a.v.z);
            break;
          case Type::Vector4:
          case Type::Color:
          case Type::Rotation:
            comps.append(a.v.x);
            comps.append(a.v.y);
            comps.append(a.v.z);
            comps.append(a.v.w);
            break;
          default:
            comps.append(a.as_float());
            break;
        }
      }
      if (args.size() == 1 && comps.size() == 1) {
        const float s = comps[0];
        comps.clear();
        for (int i = 0; i < want; i++) {
          comps.append(s);
        }
      }
      while (comps.size() < want) {
        comps.append(0.0f);
      }
      return make_vec_value(want,
                            comps[0],
                            want > 1 ? comps[1] : 0.0f,
                            want > 2 ? comps[2] : 0.0f,
                            want > 3 ? comps[3] : 0.0f);
    }
    case Builtin::FloatFn:
      return Value::from_float(arg(0).as_float());
    case Builtin::IntFn:
      return Value::from_int(arg(0).as_int());
    case Builtin::BoolFn:
      return Value::from_bool(arg(0).as_bool());
    case Builtin::Invert: {
      if (arg(0).type == Type::Rotation) {
        return value_from_quat(math::invert(value_as_quat(arg(0))));
      }
      if (arg(0).type == Type::Matrix2) {
        bool success = true;
        const float2x2 inv = math::invert(arg(0).as_matrix2(), success);
        return Value::from_matrix2(success ? inv : float2x2::identity());
      }
      if (arg(0).type == Type::Matrix3) {
        bool success = true;
        const float3x3 inv = math::invert(value_as_matrix3(arg(0)), success);
        return Value::from_matrix3(success ? inv : float3x3::identity());
      }
      bool success = true;
      const float4x4 inv = math::invert(value_as_matrix(arg(0)), success);
      return Value::from_matrix(success ? inv : float4x4::identity());
    }
    case Builtin::Transpose:
      if (arg(0).type == Type::Matrix2) {
        return Value::from_matrix2(math::transpose(arg(0).as_matrix2()));
      }
      if (arg(0).type == Type::Matrix3) {
        return Value::from_matrix3(math::transpose(value_as_matrix3(arg(0))));
      }
      return Value::from_matrix(math::transpose(value_as_matrix(arg(0))));
    case Builtin::Determinant:
      if (arg(0).type == Type::Matrix2) {
        return Value::from_float(math::determinant(arg(0).as_matrix2()));
      }
      if (arg(0).type == Type::Matrix3) {
        return Value::from_float(math::determinant(value_as_matrix3(arg(0))));
      }
      return Value::from_float(math::determinant(value_as_matrix(arg(0))));
    case Builtin::TransformPoint:
      return Value::from_vec(
          math::transform_point(value_as_matrix(arg(1)), arg(0).as_vec()));
    case Builtin::TransformDirection:
      return Value::from_vec(
          math::transform_direction(value_as_matrix(arg(1)), arg(0).as_vec()));
    case Builtin::ProjectPoint:
      return Value::from_vec(math::project_point(value_as_matrix(arg(1)), arg(0).as_vec()));
    case Builtin::CombineTransform: {
      const float3 t = arg(0).as_vec();
      const float3 s = args.size() > 2 ? arg(2).as_vec() : float3(1.0f);
      math::Quaternion rot{1.0f, 0.0f, 0.0f, 0.0f};
      if (arg(1).type == Type::Rotation) {
        rot = math::Quaternion(arg(1).v.w, arg(1).v.x, arg(1).v.y, arg(1).v.z);
      }
      else {
        const float3 eul = arg(1).as_vec();
        rot = math::to_quaternion(math::EulerXYZ(eul.x, eul.y, eul.z));
      }
      return Value::from_matrix(math::from_loc_rot_scale<float4x4>(t, rot, s));
    }
    case Builtin::Identity:
      return Value::from_matrix(float4x4::identity());
    case Builtin::MatrixFn: {
      if (args.is_empty()) {
        return Value::from_matrix(float4x4::identity());
      }
      if (arg(0).type == Type::Matrix && args.size() == 1) {
        return arg(0);
      }
      float buf[16];
      for (float &f : buf) {
        f = 0.0f;
      }
      buf[0] = buf[5] = buf[10] = buf[15] = 1.0f;
      if (args.size() == 1) {
        if (const Vector<float> *a = farr_mut(arg(0))) {
          const int n = std::min(int(a->size()), 16);
          for (int i = 0; i < n; i++) {
            buf[i] = (*a)[i];
          }
          return Value::from_matrix(float4x4(buf));
        }
        if (arg(0).type == Type::IntArray) {
          const int n = std::min(array_len(arg(0)), 16);
          for (int i = 0; i < n; i++) {
            buf[i] = float(get_index_value(arg(0), i).as_int());
          }
          return Value::from_matrix(float4x4(buf));
        }
        /* One float/vector: put it on the xyz diagonal, leave m44 = 1. */
        if (ELEM(arg(0).type, Type::Vector, Type::Color, Type::Rotation)) {
          return Value::from_matrix(matrix_from_diag_scale(arg(0).as_vec()));
        }
        return Value::from_matrix(matrix_from_uniform_scale(arg(0).as_float()));
      }
      if (args.size() == 3 && !ELEM(arg(0).type, Type::Vector, Type::Color, Type::Rotation, Type::Matrix) &&
          !ELEM(arg(1).type, Type::Vector, Type::Color, Type::Rotation, Type::Matrix) &&
          !ELEM(arg(2).type, Type::Vector, Type::Color, Type::Rotation, Type::Matrix))
      {
        return Value::from_matrix(matrix_from_diag_scale(
            float3(arg(0).as_float(), arg(1).as_float(), arg(2).as_float())));
      }
      if (args.size() >= 4 &&
          ELEM(arg(0).type, Type::Vector, Type::Rotation, Type::Color))
      {
        auto col = [&](const int i) -> float4 {
          if (i >= args.size()) {
            return float4(0.0f, 0.0f, 0.0f, i == 3 ? 1.0f : 0.0f);
          }
          if (arg(i).type == Type::Rotation || arg(i).type == Type::Color) {
            return arg(i).v;
          }
          const float3 v = arg(i).as_vec();
          return float4(v.x, v.y, v.z, i == 3 ? 1.0f : 0.0f);
        };
        return Value::from_matrix(float4x4(col(0), col(1), col(2), col(3)));
      }
      const int n = std::min(int(args.size()), 16);
      for (int i = 0; i < n; i++) {
        buf[i] = arg(i).as_float();
      }
      if (n < 16) {
        /* Keep remaining identity entries already filled. */
      }
      return Value::from_matrix(float4x4(buf));
    }
    case Builtin::Matrix2Fn: {
      if (args.is_empty()) {
        return Value::from_matrix2(float2x2::identity());
      }
      if (args.size() == 1) {
        if (arg(0).type == Type::Matrix2) {
          return arg(0);
        }
        if (arg(0).type == Type::Matrix3) {
          const float3x3 m = value_as_matrix3(arg(0));
          return Value::from_matrix2(
              float2x2(float2(m[0].x, m[0].y), float2(m[1].x, m[1].y)));
        }
        if (arg(0).type == Type::Matrix) {
          const float4x4 m = value_as_matrix(arg(0));
          return Value::from_matrix2(
              float2x2(float2(m[0].x, m[0].y), float2(m[1].x, m[1].y)));
        }
        if (arg(0).type == Type::Vector2 || arg(0).type == Type::Vector) {
          const float2 s = arg(0).as_vec2();
          return Value::from_matrix2(float2x2(float2(s.x, 0.0f), float2(0.0f, s.y)));
        }
        const float s = arg(0).as_float();
        return Value::from_matrix2(float2x2::diagonal(s));
      }
      if (args.size() == 2 &&
          (arg(0).type == Type::Vector2 || arg(0).type == Type::Vector) &&
          (arg(1).type == Type::Vector2 || arg(1).type == Type::Vector))
      {
        return Value::from_matrix2(float2x2(arg(0).as_vec2(), arg(1).as_vec2()));
      }
      if (args.size() == 2) {
        const float sx = arg(0).as_float();
        const float sy = arg(1).as_float();
        return Value::from_matrix2(float2x2(float2(sx, 0.0f), float2(0.0f, sy)));
      }
      float buf[4] = {1.0f, 0.0f, 0.0f, 1.0f};
      const int n = std::min(int(args.size()), 4);
      for (int i = 0; i < n; i++) {
        buf[i] = arg(i).as_float();
      }
      return Value::from_matrix2(float2x2(buf));
    }
    case Builtin::Matrix3Fn: {
      if (args.is_empty()) {
        return Value::from_matrix3(float3x3::identity());
      }
      if (args.size() == 1) {
        if (arg(0).type == Type::Matrix3) {
          return arg(0);
        }
        if (arg(0).type == Type::Matrix) {
          return Value::from_matrix3(float3x3(value_as_matrix(arg(0))));
        }
        if (arg(0).type == Type::Matrix2) {
          const float2x2 m = arg(0).as_matrix2();
          return Value::from_matrix3(float3x3(float3(m[0].x, m[0].y, 0.0f),
                                              float3(m[1].x, m[1].y, 0.0f),
                                              float3(0.0f, 0.0f, 1.0f)));
        }
        if (ELEM(arg(0).type, Type::Vector, Type::Vector4, Type::Color, Type::Rotation)) {
          const float3 s = arg(0).as_vec();
          return Value::from_matrix3(float3x3(float3(s.x, 0.0f, 0.0f),
                                              float3(0.0f, s.y, 0.0f),
                                              float3(0.0f, 0.0f, s.z)));
        }
        return Value::from_matrix3(float3x3::diagonal(arg(0).as_float()));
      }
      if (args.size() == 3 && ELEM(arg(0).type, Type::Vector, Type::Vector4, Type::Color) &&
          ELEM(arg(1).type, Type::Vector, Type::Vector4, Type::Color) &&
          ELEM(arg(2).type, Type::Vector, Type::Vector4, Type::Color))
      {
        return Value::from_matrix3(
            float3x3(arg(0).as_vec(), arg(1).as_vec(), arg(2).as_vec()));
      }
      if (args.size() == 3) {
        return Value::from_matrix3(float3x3(float3(arg(0).as_float(), 0.0f, 0.0f),
                                            float3(0.0f, arg(1).as_float(), 0.0f),
                                            float3(0.0f, 0.0f, arg(2).as_float())));
      }
      float buf[9];
      for (int i = 0; i < 9; i++) {
        buf[i] = (i % 4 == 0) ? 1.0f : 0.0f;
      }
      const int n = std::min(int(args.size()), 9);
      for (int i = 0; i < n; i++) {
        buf[i] = arg(i).as_float();
      }
      return Value::from_matrix3(float3x3(buf));
    }
    case Builtin::TranslationFn:
      return Value::from_vec(value_as_matrix(arg(0)).location());
    case Builtin::RotationFn: {
      const math::EulerXYZ eul = math::to_euler(value_as_matrix(arg(0)));
      return Value::from_vec(float3(float(eul.x()), float(eul.y()), float(eul.z())));
    }
    case Builtin::ScaleFn:
      return Value::from_vec(math::to_scale(value_as_matrix(arg(0))));
    case Builtin::QuatFn: {
      if (args.is_empty()) {
        return value_from_quat(math::Quaternion::identity());
      }
      if (args.size() >= 4) {
        return Value::from_vec4(
            float4(arg(0).as_float(), arg(1).as_float(), arg(2).as_float(), arg(3).as_float()),
            Type::Rotation);
      }
      return value_from_quat(value_as_quat(arg(0)));
    }
    case Builtin::RotateRotation:
      return value_from_quat(value_as_quat(arg(0)) * value_as_quat(arg(1)));
    case Builtin::Chf:
    case Builtin::Chi:
    case Builtin::Chv:
    case Builtin::Chb:
    case Builtin::Chc:
    case Builtin::Chm:
    case Builtin::Chq:
    case Builtin::Chs: {
      const StringRef name = blender::nodes::vex::value_string(arg(0), env);
      Value loaded = Value::from_float(0.0f);
      if (env.load_parm) {
        loaded = env.load_parm(env.parm_user, name, env.index, error);
      }
      switch (id) {
        case Builtin::Chi:
          return Value::from_int(loaded.as_int());
        case Builtin::Chb:
          return Value::from_bool(loaded.as_bool());
        case Builtin::Chv:
          return Value::from_vec(loaded.as_vec());
        case Builtin::Chc: {
          if (loaded.type == Type::Color) {
            return loaded;
          }
          const float3 rgb = loaded.as_vec();
          return Value::from_vec4(float4(rgb.x, rgb.y, rgb.z, 1.0f), Type::Color);
        }
        case Builtin::Chm:
          if (loaded.type == Type::Matrix) {
            return loaded;
          }
          return Value::from_matrix(float4x4::identity());
        case Builtin::Chq:
          if (loaded.type == Type::Rotation) {
            return loaded;
          }
          return value_from_quat(value_as_quat(loaded));
        case Builtin::Chs:
          if (loaded.type == Type::String) {
            return loaded;
          }
          return Value::from_str_i(0);
        case Builtin::Chf:
        default:
          return Value::from_float(loaded.as_float());
      }
    }
    case Builtin::ArrayInt:
      return value_from_int_array({});
    case Builtin::ArrayFloat:
      return value_from_float_array({});
    case Builtin::ArrayVec:
      return value_from_vec_array({});
    case Builtin::ArrayStr:
      return value_from_string_array({});
    case Builtin::ArrayMat:
      return value_from_mat_array({});
    case Builtin::ArrayRay: {
      Vector<RayHit> vals;
      vals.reserve(args.size());
      for (const Value &v : args) {
        vals.append(ray_hit_from_value(v));
      }
      return value_from_ray_array(std::move(vals));
    }
    case Builtin::ArrayFn: {
      Type at = Type::IntArray;
      if (!args.is_empty()) {
        at = type_is_array(arg(0).type) ? arg(0).type : array_type_of(arg(0).type);
        for (const Value &v : args) {
          if (v.type == Type::String || v.type == Type::StringArray) {
            at = Type::StringArray;
            break;
          }
          if (v.type == Type::Matrix || v.type == Type::MatArray) {
            at = Type::MatArray;
            break;
          }
          if (v.type == Type::Ray || v.type == Type::RayArray) {
            at = Type::RayArray;
            break;
          }
          if (v.type == Type::Vector || v.type == Type::VecArray) {
            at = Type::VecArray;
            break;
          }
          if (v.type == Type::Float || v.type == Type::FloatArray) {
            at = Type::FloatArray;
          }
        }
      }
      if (at == Type::StringArray) {
        Vector<std::string> vals;
        for (const Value &v : args) {
          vals.append(std::string(value_string(v, env)));
        }
        return value_from_string_array(std::move(vals));
      }
      if (at == Type::MatArray) {
        Vector<float4x4> vals;
        for (const Value &v : args) {
          if (v.type == Type::Matrix && v.i >= 0 && v.i < int(tls_mats.size())) {
            vals.append(tls_mats[v.i]);
          }
          else if (Vector<float4x4> *src = marr_mut(v)) {
            vals.extend(*src);
          }
          else {
            vals.append(float4x4::identity());
          }
        }
        return value_from_mat_array(std::move(vals));
      }
      if (at == Type::RayArray) {
        Vector<RayHit> vals;
        for (const Value &v : args) {
          if (Vector<RayHit> *src = rayarr_mut(v)) {
            vals.extend(*src);
          }
          else {
            vals.append(ray_hit_from_value(v));
          }
        }
        return value_from_ray_array(std::move(vals));
      }
      if (at == Type::VecArray) {
        Vector<float3> vals;
        for (const Value &v : args) {
          vals.append(v.as_vec());
        }
        return value_from_vec_array(std::move(vals));
      }
      if (at == Type::FloatArray) {
        Vector<float> vals;
        for (const Value &v : args) {
          vals.append(v.as_float());
        }
        return value_from_float_array(std::move(vals));
      }
      Vector<int> vals;
      for (const Value &v : args) {
        vals.append(v.as_int());
      }
      return value_from_int_array(std::move(vals));
    }
    case Builtin::Append: {
      Value arr = arg(0);
      if (!type_is_array(arr.type)) {
        arr = empty_array(array_type_of(arr.type));
      }
      if (Vector<int> *a = materialize_int_array(arr)) {
        a->append(arg(1).as_int());
      }
      else if (Vector<float> *a = farr_mut(arr)) {
        a->append(arg(1).as_float());
      }
      else if (Vector<float3> *a = varr_mut(arr)) {
        a->append(arg(1).as_vec());
      }
      else if (Vector<std::string> *a = sarr_mut(arr)) {
        a->append(std::string(value_string(arg(1), env)));
      }
      else if (Vector<float4x4> *a = marr_mut(arr)) {
        if (arg(1).type == Type::Matrix && arg(1).i >= 0 && arg(1).i < int(tls_mats.size())) {
          a->append(tls_mats[arg(1).i]);
        }
        else {
          a->append(float4x4::identity());
        }
      }
      else if (Vector<RayHit> *a = rayarr_mut(arr)) {
        a->append(ray_hit_from_value(arg(1)));
      }
      return arr;
    }
    case Builtin::Insert: {
      Value arr = arg(0);
      if (!type_is_array(arr.type)) {
        arr = empty_array(array_type_of(arr.type));
      }
      if (Vector<int> *a = materialize_int_array(arr)) {
        a->insert(wrap_index(arg(1).as_int(), int(a->size()), true), arg(2).as_int());
      }
      else if (Vector<float> *a = farr_mut(arr)) {
        a->insert(wrap_index(arg(1).as_int(), int(a->size()), true), arg(2).as_float());
      }
      else if (Vector<float3> *a = varr_mut(arr)) {
        a->insert(wrap_index(arg(1).as_int(), int(a->size()), true), arg(2).as_vec());
      }
      else if (Vector<std::string> *a = sarr_mut(arr)) {
        a->insert(wrap_index(arg(1).as_int(), int(a->size()), true),
                  std::string(value_string(arg(2), env)));
      }
      else if (Vector<float4x4> *a = marr_mut(arr)) {
        float4x4 m = float4x4::identity();
        if (arg(2).type == Type::Matrix && arg(2).i >= 0 && arg(2).i < int(tls_mats.size())) {
          m = tls_mats[arg(2).i];
        }
        a->insert(wrap_index(arg(1).as_int(), int(a->size()), true), m);
      }
      else if (Vector<RayHit> *a = rayarr_mut(arr)) {
        a->insert(wrap_index(arg(1).as_int(), int(a->size()), true), ray_hit_from_value(arg(2)));
      }
      return arr;
    }
    case Builtin::RemoveIndex: {
      Value arr = arg(0);
      if (Vector<int> *a = materialize_int_array(arr)) {
        const int i = wrap_index(arg(1).as_int(), int(a->size()), false);
        if (i >= 0) {
          a->remove(i);
        }
      }
      else if (Vector<float> *a = farr_mut(arr)) {
        const int i = wrap_index(arg(1).as_int(), int(a->size()), false);
        if (i >= 0) {
          a->remove(i);
        }
      }
      else if (Vector<float3> *a = varr_mut(arr)) {
        const int i = wrap_index(arg(1).as_int(), int(a->size()), false);
        if (i >= 0) {
          a->remove(i);
        }
      }
      else if (Vector<float4x4> *a = marr_mut(arr)) {
        const int i = wrap_index(arg(1).as_int(), int(a->size()), false);
        if (i >= 0) {
          a->remove(i);
        }
      }
      else if (Vector<RayHit> *a = rayarr_mut(arr)) {
        const int i = wrap_index(arg(1).as_int(), int(a->size()), false);
        if (i >= 0) {
          a->remove(i);
        }
      }
      return arr;
    }
    case Builtin::RemoveValue: {
      Value arr = arg(0);
      if (Vector<int> *a = materialize_int_array(arr)) {
        const int x = arg(1).as_int();
        for (int i = int(a->size()) - 1; i >= 0; i--) {
          if ((*a)[i] == x) {
            a->remove(i);
          }
        }
      }
      else if (Vector<float> *a = farr_mut(arr)) {
        const float x = arg(1).as_float();
        for (int i = int(a->size()) - 1; i >= 0; i--) {
          if ((*a)[i] == x) {
            a->remove(i);
          }
        }
      }
      else if (Vector<float3> *a = varr_mut(arr)) {
        const float3 x = arg(1).as_vec();
        for (int i = int(a->size()) - 1; i >= 0; i--) {
          if ((*a)[i] == x) {
            a->remove(i);
          }
        }
      }
      return arr;
    }
    case Builtin::SortArr: {
      Value arr = arg(0);
      if (Vector<int> *a = materialize_int_array(arr)) {
        std::sort(a->begin(), a->end());
      }
      else if (Vector<float> *a = farr_mut(arr)) {
        std::sort(a->begin(), a->end());
      }
      else if (Vector<float3> *a = varr_mut(arr)) {
        std::sort(a->begin(), a->end(), [](const float3 &p, const float3 &q) {
          return p.x != q.x ? p.x < q.x : (p.y != q.y ? p.y < q.y : p.z < q.z);
        });
      }
      return arr;
    }
    case Builtin::Len:
      if (type_is_array(arg(0).type)) {
        return Value::from_int(array_len(arg(0)));
      }
      if (arg(0).type == Type::String) {
        return Value::from_int(int(value_string(arg(0), env).size()));
      }
      return Value::from_int(int(math::length(arg(0).as_vec())));
    case Builtin::ValueToString: {
      if (args.size() >= 3) {
        return intern_runtime_string(
            env, valuetostring_float(double(arg(0).as_float()), arg(1).as_int(), arg(2).as_int()));
      }
      return intern_runtime_string(env, valuetostring_int(arg(0).as_int(), arg(1).as_int()));
    }
    case Builtin::Format:
      return format_values(args, env, error);
    case Builtin::SvdFn:
      fill_decomp_from_matrix(arg(0));
      return Value::from_int(1);
    case Builtin::PolarDecompFn:
      fill_polar_from_matrix(arg(0));
      return Value::from_int(1);
    case Builtin::EigenFn:
      fill_eigen_from_matrix(arg(0));
      return Value::from_int(1);
    case Builtin::DecompU:
      return tls_decomp_u;
    case Builtin::DecompS:
      return tls_decomp_s;
    case Builtin::DecompV:
      return tls_decomp_v;
    case Builtin::Count:
      break;
    default:
      break;
  }
  if (is_geo_builtin(id) && env.topo_fn) {
    return env.topo_fn(env.topo_user, int(id), args, env, error);
  }
  return Value::from_float(0.0f);
}

}  // namespace

static Value load_static_sample(const int sample_slot,
                                const int index,
                                VMEnv &env,
                                std::string &r_error)
{
  if (sample_slot >= 0 && sample_slot < env.static_samples.size()) {
    const AttrRT &sample = env.static_samples[sample_slot];
    if (sample.size > 0) {
      return load_attr_value(sample, index, env);
    }
  }
  if (!env.load_sample) {
    return Value::from_float(0.0f);
  }
  return env.load_sample(env.elem_user, sample_slot, index, r_error);
}

static inline Value typed_length_v3(const Value &a)
{
  return Value::from_float(math::length(a.as_vec()));
}

static inline Value typed_dot_v3(const Value &a, const Value &b)
{
  return Value::from_float(math::dot(a.as_vec(), b.as_vec()));
}

static inline Value typed_cross_v3(const Value &a, const Value &b)
{
  return Value::from_vec(math::cross(a.as_vec(), b.as_vec()));
}

static inline Value typed_sqrt_f(const Value &a)
{
  return Value::from_float(sqrtf(std::max(a.as_float(), 0.0f)));
}

static inline Value typed_log_f(const Value &a)
{
  return Value::from_float(logf(std::max(a.as_float(), 1e-30f)));
}

static inline Value typed_abs_f(const Value &a)
{
  return Value::from_float(fabsf(a.as_float()));
}

static inline Value typed_array_len(const Value &a)
{
  return Value::from_int(array_len(a));
}

static Value call_static_sample(const Inst &in,
                                const Span<Value> args,
                                VMEnv &env,
                                std::string &r_error)
{
  const int index_arg = int(in.a >> 5) - 1;
  const int index = (index_arg >= 0 && index_arg < args.size()) ? args[index_arg].as_int() :
                                                                  env.index;
  return load_static_sample(in.imm, index, env, r_error);
}

static bool gather_samples_direct(const Program &program,
                                  const Inst &in,
                                  Value *locals,
                                  const int local_n,
                                  VMEnv &env,
                                  std::string &r_error)
{
  if (in.imm < 0 || in.imm >= program.gather_samples.size()) {
    r_error = "Invalid fused sample gather";
    return false;
  }
  const Program::GatherSamples &gather = program.gather_samples[in.imm];
  if (gather.index_array_local < 0 || gather.index_array_local >= local_n) {
    r_error = "Invalid fused sample index array";
    return false;
  }
  const Value &indices = locals[gather.index_array_local];
  const int size = array_len(indices);
  int range_start = 0;
  int range_size = 0;
  const bool contiguous_indices = int_array_range(indices, range_start, range_size) &&
                                  range_size == size;
  for (const int item : gather.output_locals.index_range()) {
    const int output_local = gather.output_locals[item];
    if (output_local < 0 || output_local >= local_n || item >= gather.sample_slots.size() ||
        item >= gather.output_types.size())
    {
      r_error = "Invalid fused sample output";
      return false;
    }
    const int sample_slot = gather.sample_slots[item];
    const AttrRT *sample = (sample_slot >= 0 && sample_slot < env.static_samples.size()) ?
                               &env.static_samples[sample_slot] :
                               nullptr;
    const bool wants_materialized = item >= gather.materialize.size() ||
                                    gather.materialize[item] != 0;
    bool has_direct_span = false;
    if (sample && sample->size > 0) {
      if (gather.output_types[item] == Type::VecArray) {
        has_direct_span = sample->wv || sample->rv;
      }
      else if (gather.output_types[item] == Type::FloatArray) {
        has_direct_span = sample->wf || sample->rf;
      }
      else if (gather.output_types[item] == Type::IntArray) {
        has_direct_span = sample->wi || sample->ri;
      }
    }
    if (!wants_materialized && has_direct_span) {
      continue;
    }
    switch (gather.output_types[item]) {
      case Type::VecArray: {
        const float3 *src = sample ? (sample->wv ? sample->wv : sample->rv) : nullptr;
        Value output;
        output.type = Type::VecArray;
        output.i = prepare_tls_array(tls_varr, tls_varr_used, size);
        Vector<float3> &values = tls_varr[output.i];
        for (int i = 0; i < size; i++) {
          const int index = contiguous_indices ? range_start + i :
                                                 get_index_value(indices, i).as_int();
          values[i] = (src && index >= 0 && index < sample->size) ? src[index] :
                                                                   load_static_sample(
                                                                       sample_slot,
                                                                       index,
                                                                       env,
                                                                       r_error)
                                                                       .as_vec();
          if (!r_error.empty()) {
            return false;
          }
        }
        locals[output_local] = output;
        break;
      }
      case Type::FloatArray: {
        const float *src = sample ? (sample->wf ? sample->wf : sample->rf) : nullptr;
        Value output;
        output.type = Type::FloatArray;
        output.i = prepare_tls_array(tls_farr, tls_farr_used, size);
        Vector<float> &values = tls_farr[output.i];
        for (int i = 0; i < size; i++) {
          const int index = contiguous_indices ? range_start + i :
                                                 get_index_value(indices, i).as_int();
          values[i] = (src && index >= 0 && index < sample->size) ? src[index] :
                                                                   load_static_sample(
                                                                       sample_slot,
                                                                       index,
                                                                       env,
                                                                       r_error)
                                                                       .as_float();
          if (!r_error.empty()) {
            return false;
          }
        }
        locals[output_local] = output;
        break;
      }
      case Type::IntArray: {
        const int *src = sample ? (sample->wi ? sample->wi : sample->ri) : nullptr;
        Value output;
        output.type = Type::IntArray;
        output.i = prepare_tls_array(tls_iarr, tls_iarr_used, size);
        Vector<int> &values = tls_iarr[output.i];
        for (int i = 0; i < size; i++) {
          const int index = contiguous_indices ? range_start + i :
                                                 get_index_value(indices, i).as_int();
          values[i] = (src && index >= 0 && index < sample->size) ? src[index] :
                                                                   load_static_sample(
                                                                       sample_slot,
                                                                       index,
                                                                       env,
                                                                       r_error)
                                                                       .as_int();
          if (!r_error.empty()) {
            return false;
          }
        }
        locals[output_local] = output;
        break;
      }
      default:
        r_error = "Unsupported fused sample output type";
        return false;
    }
  }
  return true;
}

static Value load_local_index(const Program &program,
                              const Inst &in,
                              const Value *locals,
                              const int local_n,
                              VMEnv *env)
{
  if (in.imm < 0 || in.imm >= program.local_index_loads.size()) {
    return Value::from_float(0.0f);
  }
  const Program::LocalIndexLoad &load = program.local_index_loads[in.imm];
  if (load.array_local < 0 || load.array_local >= local_n) {
    return Value::from_float(0.0f);
  }
  int index = 0;
  if (load.index_is_const) {
    if (load.index < 0 || load.index >= program.const_i.size()) {
      return Value::from_float(0.0f);
    }
    index = program.const_i[load.index];
  }
  else {
    if (load.index < 0 || load.index >= local_n) {
      return Value::from_float(0.0f);
    }
    index = locals[load.index].as_int();
  }
  if (env && load.sample_slot >= 0 && load.sample_slot < env->static_samples.size() &&
      load.sample_index_array_local >= 0 && load.sample_index_array_local < local_n)
  {
    const AttrRT &sample = env->static_samples[load.sample_slot];
    const Value &indices = locals[load.sample_index_array_local];
    int range_start = 0, range_size = 0;
    int element_index = -1;
    if (int_array_range(indices, range_start, range_size)) {
      if (index >= 0 && index < range_size) {
        element_index = range_start + index;
      }
    }
    else {
      element_index = get_index_value(indices, index).as_int();
    }
    if (element_index >= 0 && element_index < sample.size) {
      Value value = load_attr_value(sample, element_index, *env);
      if (load.member >= 0) {
        value = apply_get_member(value, load.member);
      }
      return value;
    }
  }
  Value value = get_index_value(locals[load.array_local], index);
  if (load.member >= 0) {
    value = apply_get_member(value, load.member);
  }
  return value;
}

static bool vm_interp(const Program &program,
                      VMEnv &env,
                      std::string &r_error,
                      Value *r_return,
                      const int start_pc,
                      Value *locals,
                      const int local_n,
                      const int depth);

bool vm_run(const Program &program, VMEnv &env, std::string &r_error, Value *r_return)
{
  env.const_s = program.const_s.as_span();
  env.rand_seq = 0;
  tls_arrays_begin();

  constexpr int stack_cap = 64;
  const int local_n = std::max(program.local_count, 1);
  Value local_storage[stack_cap];
  Vector<Value> local_heap;
  Value *locals;
  if (local_n <= stack_cap) {
    locals = local_storage;
    for (int i = 0; i < local_n; i++) {
      locals[i] = Value{};
    }
  }
  else {
    local_heap.resize(local_n);
    locals = local_heap.data();
  }
  return vm_interp(program, env, r_error, r_return, 0, locals, local_n, 0);
}

static bool vm_interp(const Program &program,
                      VMEnv &env,
                      std::string &r_error,
                      Value *r_return,
                      const int start_pc,
                      Value *locals,
                      const int local_n,
                      const int /*depth*/)
{
  /* Shared operand stack + software call frames. Recursing into this function
   * (a multi-thousand-line interpreter) made each extra CallUser ~tens of times
   * slower; jump-to-entry is O(body) per call. */
  Vector<Value, 64> stk;
  int sp = 0;

  auto push = [&](const Value &v) {
    if (sp < int(stk.size())) {
      stk[sp++] = v;
    }
    else {
      stk.append(v);
      sp++;
    }
  };
  auto pop = [&]() -> Value {
    if (sp <= 0) {
      return Value::from_float(0.0f);
    }
    return stk[--sp];
  };

  struct CallFrame {
    int return_pc = 0;
    int prev_base = -1;
    int prev_n = 0;
    int fn_n = 0;
  };
  constexpr int max_depth = 256;
  Vector<CallFrame, 16> frames;
  Vector<Value, 64> local_pool;
  int local_top = 0;
  int cur_base = -1;
  Value *loc = locals;
  int loc_n = local_n;
  auto refresh_loc = [&]() {
    loc = (cur_base < 0) ? locals : (local_pool.data() + cur_base);
  };

  constexpr int64_t max_jumps = 100000000;
  int64_t jumps = 0;
  const Span<Inst> code = program.code;
  const int code_n = int(code.size());

  for (int pc = start_pc; pc < code_n;) {
    const Inst &in = code[pc];
    switch (in.op) {
      case Op::Nop:
        break;
      case Op::PushF:
        push(Value::from_float(program.const_f[in.imm]));
        break;
      case Op::PushI:
        push(Value::from_int(program.const_i[in.imm]));
        break;
      case Op::PushV:
        push(Value::from_vec(program.const_v[in.imm]));
        break;
      case Op::PushB:
        push(Value::from_bool(in.imm != 0));
        break;
      case Op::PushS:
        push(Value::from_str_i(in.imm));
        break;
      case Op::PushLocal:
        if (in.imm >= 0 && in.imm < loc_n) {
          push(loc[in.imm]);
        }
        else {
          push(Value::from_float(0.0f));
        }
        break;
      case Op::PushLocalIndex:
        push(load_local_index(program, in, loc, loc_n, &env));
        break;
      case Op::PushLocalMember:
        if (in.imm >= 0 && in.imm < loc_n) {
          push(apply_get_member(loc[in.imm], int(in.a)));
        }
        else {
          push(Value::from_float(0.0f));
        }
        break;
      case Op::StoreLocal: {
        Value v = pop();
        if (in.imm >= 0 && in.imm < loc_n) {
          loc[in.imm] = v;
        }
        break;
      }
      case Op::IncLocal:
        if (in.imm >= 0 && in.imm < loc_n) {
          loc[in.imm].type = Type::Int;
          loc[in.imm].i += 1;
          loc[in.imm].v.x = float(loc[in.imm].i);
        }
        break;
      case Op::DecLocal:
        if (in.imm >= 0 && in.imm < loc_n) {
          loc[in.imm].type = Type::Int;
          loc[in.imm].i -= 1;
          loc[in.imm].v.x = float(loc[in.imm].i);
        }
        break;
      case Op::Dup:
        if (sp > 0) {
          push(stk[sp - 1]);
        }
        break;
      case Op::PushHitPos:
        push(Value::from_vec(env.hit_pos));
        break;
      case Op::Pop:
        if (sp > 0) {
          sp--;
        }
        break;
      case Op::PushAttr:
        if (in.imm >= 0 && in.imm < env.attrs.size()) {
          push(load_attr_value(env.attrs[in.imm], env.index, env));
        }
        else {
          push(Value::from_float(0.0f));
        }
        break;
      case Op::StoreAttr: {
        Value v = pop();
        if (in.imm >= 0 && in.imm < env.attrs.size()) {
          store_attr_value(env.attrs[in.imm], env.index, v, env);
        }
        break;
      }
      case Op::AttrAdd: {
        Value v = pop();
        if (in.imm >= 0 && in.imm < env.attrs.size()) {
          attr_add_inplace(env.attrs[in.imm], env.index, v);
        }
        break;
      }
      case Op::AttrMulAdd: {
        Value k = pop();
        Value b = pop();
        if (in.imm >= 0 && in.imm < env.attrs.size()) {
          attr_add_inplace(env.attrs[in.imm], env.index, scale_value(b, k.as_int()));
        }
        break;
      }
      case Op::PushIndex:
        push(Value::from_int(env.index));
        break;
      case Op::PushNpoints:
        push(Value::from_int(env.npoints));
        break;
      case Op::PushNedges:
        push(Value::from_int(env.nedges));
        break;
      case Op::PushNfaces:
        push(Value::from_int(env.nfaces));
        break;
      case Op::PushNcorners:
        push(Value::from_int(env.ncorners));
        break;
      case Op::Add: {
        if (sp >= 2) {
          Value b = pop();
          add_inplace(stk[sp - 1], b);
        }
        break;
      }
      case Op::Sub: {
        Value b = pop();
        Value a = pop();
        push(numeric_sub(a, b));
        break;
      }
      case Op::Mul: {
        Value b = pop();
        Value a = pop();
        push(numeric_mul(a, b, nullptr));
        break;
      }
      case Op::Div: {
        Value b = pop();
        Value a = pop();
        push(numeric_div(a, b));
        break;
      }
      case Op::AddFF: {
        const Value b = pop(), a = pop();
        push(Value::from_float(a.v.x + b.v.x));
        break;
      }
      case Op::AddVV: {
        const Value b = pop(), a = pop();
        push(Value::from_vec(float3(a.v.x + b.v.x, a.v.y + b.v.y, a.v.z + b.v.z)));
        break;
      }
      case Op::SubFF: {
        const Value b = pop(), a = pop();
        push(Value::from_float(a.v.x - b.v.x));
        break;
      }
      case Op::SubVV: {
        const Value b = pop(), a = pop();
        push(Value::from_vec(float3(a.v.x - b.v.x, a.v.y - b.v.y, a.v.z - b.v.z)));
        break;
      }
      case Op::MulFF: {
        const Value b = pop(), a = pop();
        push(Value::from_float(a.v.x * b.v.x));
        break;
      }
      case Op::MulFV: {
        const Value b = pop(), a = pop();
        push(Value::from_vec(float3(a.v.x * b.v.x, a.v.x * b.v.y, a.v.x * b.v.z)));
        break;
      }
      case Op::MulVF: {
        const Value b = pop(), a = pop();
        push(Value::from_vec(float3(a.v.x * b.v.x, a.v.y * b.v.x, a.v.z * b.v.x)));
        break;
      }
      case Op::MulVV: {
        const Value b = pop(), a = pop();
        push(Value::from_vec(float3(a.v.x * b.v.x, a.v.y * b.v.y, a.v.z * b.v.z)));
        break;
      }
      case Op::DivFF: {
        const Value b = pop(), a = pop();
        push(Value::from_float(b.v.x != 0.0f ? a.v.x / b.v.x : 0.0f));
        break;
      }
      case Op::DivVF: {
        const Value b = pop(), a = pop();
        const float inv = b.v.x != 0.0f ? 1.0f / b.v.x : 0.0f;
        push(Value::from_vec(float3(a.v.x * inv, a.v.y * inv, a.v.z * inv)));
        break;
      }
      case Op::DivVV: {
        const Value b = pop(), a = pop();
        push(Value::from_vec(float3(a.v.x / b.v.x, a.v.y / b.v.y, a.v.z / b.v.z)));
        break;
      }
      case Op::Mod: {
        Value b = pop();
        Value a = pop();
        if (a.type == Type::Int && b.type == Type::Int) {
          const int d = b.as_int();
          push(Value::from_int(d != 0 ? a.as_int() % d : 0));
        }
        else {
          const float d = b.as_float();
          push(Value::from_float(d != 0.0f ? fmodf(a.as_float(), d) : 0.0f));
        }
        break;
      }
      case Op::Neg: {
        Value a = pop();
        if (a.type == Type::Vector4) {
          push(Value::from_vec4(-a.as_vec4(), Type::Vector4));
        }
        else if (a.type == Type::Vector) {
          push(Value::from_vec(-a.as_vec()));
        }
        else if (a.type == Type::Vector2) {
          push(Value::from_vec2(-a.as_vec2()));
        }
        else if (a.type == Type::Matrix) {
          push(Value::from_matrix(-value_as_matrix(a)));
        }
        else if (a.type == Type::Matrix3) {
          push(Value::from_matrix3(-value_as_matrix3(a)));
        }
        else if (a.type == Type::Matrix2) {
          push(Value::from_matrix2(-a.as_matrix2()));
        }
        else if (a.type == Type::Int) {
          push(Value::from_int(-a.as_int()));
        }
        else {
          push(Value::from_float(-a.as_float()));
        }
        break;
      }
      case Op::Lt: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_float() < b.as_float()));
        break;
      }
      case Op::Le: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_float() <= b.as_float()));
        break;
      }
      case Op::Gt: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_float() > b.as_float()));
        break;
      }
      case Op::Ge: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_float() >= b.as_float()));
        break;
      }
      case Op::Eq: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_float() == b.as_float()));
        break;
      }
      case Op::Ne: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_float() != b.as_float()));
        break;
      }
      case Op::And: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_bool() && b.as_bool()));
        break;
      }
      case Op::Or: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_bool() || b.as_bool()));
        break;
      }
      case Op::Not:
        push(Value::from_bool(!pop().as_bool()));
        break;
      case Op::Jmp:
        if (++jumps > max_jumps) {
          r_error = "Instruction limit exceeded (infinite loop?)";
          return false;
        }
        pc = in.imm;
        continue;
      case Op::JmpIfFalse:
        if (++jumps > max_jumps) {
          r_error = "Instruction limit exceeded (infinite loop?)";
          return false;
        }
        if (!pop().as_bool()) {
          pc = in.imm;
          continue;
        }
        break;
      case Op::JmpIfTrue:
        if (++jumps > max_jumps) {
          r_error = "Instruction limit exceeded (infinite loop?)";
          return false;
        }
        if (pop().as_bool()) {
          pc = in.imm;
          continue;
        }
        break;
      case Op::LengthV3:
        push(typed_length_v3(pop()));
        break;
      case Op::DotV3: {
        const Value b = pop();
        const Value a = pop();
        push(typed_dot_v3(a, b));
        break;
      }
      case Op::CrossV3: {
        const Value b = pop();
        const Value a = pop();
        push(typed_cross_v3(a, b));
        break;
      }
      case Op::SqrtF:
        push(typed_sqrt_f(pop()));
        break;
      case Op::LogF:
        push(typed_log_f(pop()));
        break;
      case Op::AbsF:
        push(typed_abs_f(pop()));
        break;
      case Op::ArrayLen:
        push(typed_array_len(pop()));
        break;
      case Op::Call: {
        const int nargs = int(in.a);
        Value args_buf[32];
        const int n = std::min(nargs, 32);
        for (int i = n - 1; i >= 0; i--) {
          args_buf[i] = pop();
        }
        Value result = call_builtin(Builtin(in.imm), Span<Value>(args_buf, n), env, r_error);
        if (!r_error.empty()) {
          return false;
        }
        push(result);
        break;
      }
      case Op::SampleElem: {
        const int nargs = int(in.a & 0x1f);
        Value args_buf[32];
        const int n = std::min(nargs, 32);
        for (int i = n - 1; i >= 0; i--) {
          args_buf[i] = pop();
        }
        push(call_static_sample(in, Span<Value>(args_buf, n), env, r_error));
        if (!r_error.empty()) {
          return false;
        }
        break;
      }
      case Op::FaceCorners: {
        const int nargs = int(in.a);
        Value args_buf[2];
        const int n = std::min(nargs, 2);
        for (int i = n - 1; i >= 0; i--) {
          args_buf[i] = pop();
        }
        push(face_corners_direct(in, Span<Value>(args_buf, n), env));
        break;
      }
      case Op::GatherSamples:
        if (!gather_samples_direct(program, in, loc, loc_n, env, r_error)) {
          return false;
        }
        break;
      case Op::CallUser: {
        if (in.imm < 0 || in.imm >= int(program.user_fns.size())) {
          r_error = "无效的函数调用";
          return false;
        }
        const UserFn &fn = program.user_fns[in.imm];
        const int nargs = int(in.a);
        const int fn_n = std::max(fn.local_count, std::max(nargs, 1));
        Value arg_buf[32];
        const int narg = std::min(std::max(nargs, 0), 32);
        for (int i = narg - 1; i >= 0; i--) {
          arg_buf[i] = pop();
        }
        /* `return f(...)` — reuse this frame (linear recursion stays O(n)). */
        const bool tail = (pc + 1 < code_n && code[pc + 1].op == Op::Return);
        if (tail && fn_n <= loc_n) {
          for (int i = 0; i < narg; i++) {
            loc[i] = arg_buf[i];
          }
          pc = fn.entry;
          continue;
        }
        if (int(frames.size()) >= max_depth) {
          r_error = "函数递归太深";
          return false;
        }
        const int new_base = local_top;
        local_top += fn_n;
        if (local_top > int(local_pool.size())) {
          local_pool.resize(std::max(local_top, int(local_pool.size()) * 2 + 8));
        }
        CallFrame fr;
        fr.return_pc = pc + 1;
        fr.prev_base = cur_base;
        fr.prev_n = loc_n;
        fr.fn_n = fn_n;
        frames.append(fr);
        cur_base = new_base;
        loc_n = fn_n;
        refresh_loc();
        for (int i = 0; i < narg; i++) {
          loc[i] = arg_buf[i];
        }
        pc = fn.entry;
        continue;
      }
      case Op::Pow: {
        const Value b = pop();
        const Value a = pop();
        if (a.type == Type::Vector || b.type == Type::Vector) {
          const float3 av = a.as_vec();
          const float3 bv = b.as_vec();
          push(Value::from_vec(float3(powf(av.x, bv.x), powf(av.y, bv.y), powf(av.z, bv.z))));
        }
        else if (a.type == Type::Vector2 || b.type == Type::Vector2) {
          const float2 av = a.as_vec2();
          const float2 bv = b.as_vec2();
          push(Value::from_vec2(float2(powf(av.x, bv.x), powf(av.y, bv.y))));
        }
        else {
          push(Value::from_float(powf(a.as_float(), b.as_float())));
        }
        break;
      }
      case Op::GetMember: {
        push(apply_get_member(pop(), in.imm));
        break;
      }
      case Op::SetMember: {
        const Value obj = pop();
        const Value mem = pop();
        push(apply_set_member(obj, mem, in.imm));
        break;
      }
      case Op::GetIndex: {
        const Value idx = pop();
        const Value arr = pop();
        push(get_index_value(arr, idx.as_int()));
        break;
      }
      case Op::SetIndex: {
        const Value idx = pop();
        const Value arr = pop();
        const Value elem = pop();
        push(set_index_value(arr, idx.as_int(), elem, env));
        break;
      }
      case Op::MakeVec: {
        const int n = in.imm;
        float w = 0.0f;
        float z = 0.0f;
        float y = 0.0f;
        float x = 0.0f;
        if (n >= 4) {
          w = pop().as_float();
        }
        if (n >= 3) {
          z = pop().as_float();
        }
        if (n >= 2) {
          y = pop().as_float();
        }
        if (n >= 1) {
          x = pop().as_float();
        }
        push(make_vec_value(n, x, y, z, w));
        break;
      }
      case Op::WhileCmpAdd: {
        if (in.imm >= 0 && in.imm < program.while_adds.size() &&
            program.while_adds[in.imm].attr >= 0 &&
            program.while_adds[in.imm].attr < env.attrs.size())
        {
          while_cmp_add_one(env.attrs, program.while_adds[in.imm], env.index);
        }
        break;
      }
      case Op::Return: {
        Value ret = (sp > 0) ? pop() : Value::from_int(0);
        if (frames.is_empty()) {
          if (r_return) {
            *r_return = ret;
          }
          return true;
        }
        const CallFrame fr = frames.pop_last();
        local_top -= fr.fn_n;
        cur_base = fr.prev_base;
        loc_n = fr.prev_n;
        refresh_loc();
        pc = fr.return_pc;
        push(ret);
        continue;
      }
      default:
        break;
    }
    pc++;
  }
  if (r_return && sp > 0) {
    *r_return = stk[sp - 1];
  }
  return true;
}

static void infer_spatial_kernel(Program &program);

void optimize_program(Program &program)
{
  Span<Inst> code = program.code;
  for (int pc = 0; pc < code.size();) {
    CountedAttrLoop loop;
    if (match_counted_attr_loop(program, pc, loop) && loop_counter_starts_at_zero(program, loop))
    {
      int w = pc;
      for (const CountedAttrLoop::Add &add : loop.adds) {
        Inst a;
        a.op = add.src_op;
        a.imm = add.src_imm;
        program.code[w++] = a;
        Inst k;
        k.op = Op::PushI;
        k.imm = loop.k_const_index;
        program.code[w++] = k;
        Inst mad;
        mad.op = Op::AttrMulAdd;
        mad.imm = add.attr;
        program.code[w++] = mad;
      }
      Inst pushk;
      pushk.op = Op::PushI;
      pushk.imm = loop.k_const_index;
      program.code[w++] = pushk;
      Inst store;
      store.op = Op::StoreLocal;
      store.imm = loop.i_slot;
      program.code[w++] = store;
      while (w < loop.end_pc && w < program.code.size()) {
        program.code[w].op = Op::Nop;
        program.code[w].imm = 0;
        w++;
      }
      pc = loop.end_pc;
      continue;
    }
    pc++;
  }

  /* Lower `attr.x = attr.x + c` and `attr = attr + c` to AttrAdd so the
   * length-n C kernel runs instead of a per-element PushAttr interpret. */
  {
    Vector<Inst> &c = program.code;
    for (int pc = 0; pc < int(c.size()); pc++) {
      if (pc + 5 < int(c.size()) && c[pc].op == Op::PushAttr && c[pc + 1].op == Op::GetMember &&
          (c[pc + 2].op == Op::PushF || c[pc + 2].op == Op::PushI) &&
          ELEM(c[pc + 3].op, Op::Add, Op::AddFF) &&
          c[pc + 4].op == Op::SetMember && c[pc + 5].op == Op::StoreAttr &&
          c[pc].imm == c[pc + 5].imm && c[pc + 1].imm == c[pc + 4].imm &&
          (c[pc + 1].imm >> 8) == 0)
      {
        float s = 0.0f;
        if (c[pc + 2].op == Op::PushF && c[pc + 2].imm >= 0 &&
            c[pc + 2].imm < int(program.const_f.size()))
        {
          s = program.const_f[c[pc + 2].imm];
        }
        else if (c[pc + 2].op == Op::PushI && c[pc + 2].imm >= 0 &&
                 c[pc + 2].imm < int(program.const_i.size()))
        {
          s = float(program.const_i[c[pc + 2].imm]);
        }
        float3 add(0.0f);
        if (c[pc + 1].imm == 0) {
          add.x = s;
        }
        else if (c[pc + 1].imm == 1) {
          add.y = s;
        }
        else {
          add.z = s;
        }
        const int vid = int(program.const_v.size());
        program.const_v.append(add);
        c[pc].op = Op::PushV;
        c[pc].imm = vid;
        c[pc + 1].op = Op::AttrAdd;
        c[pc + 1].imm = c[pc + 5].imm;
        c[pc + 2].op = Op::Nop;
        c[pc + 3].op = Op::Nop;
        c[pc + 4].op = Op::Nop;
        c[pc + 5].op = Op::Nop;
        pc += 5;
        continue;
      }
      if (pc + 3 < int(c.size()) && c[pc].op == Op::PushAttr &&
          ELEM(c[pc + 1].op, Op::PushV, Op::PushF, Op::PushI, Op::PushLocal) &&
          ELEM(c[pc + 2].op, Op::Add, Op::AddFF, Op::AddVV) &&
          c[pc + 3].op == Op::StoreAttr && c[pc].imm == c[pc + 3].imm)
      {
        c[pc].op = Op::Nop;
        c[pc + 2].op = Op::AttrAdd;
        c[pc + 2].imm = c[pc + 3].imm;
        c[pc + 3].op = Op::Nop;
        pc += 3;
        continue;
      }
    }
  }

  /* A fused gather overwrites its output arrays before their first read. Remove the compiler's
   * default `array(); StoreLocal` initialization so every lane does not allocate empty TLS arrays
   * that are immediately discarded. */
  {
    Vector<Inst> &c = program.code;
    for (int gather_pc = 0; gather_pc < int(c.size()); gather_pc++) {
      if (c[gather_pc].op != Op::GatherSamples || c[gather_pc].imm < 0 ||
          c[gather_pc].imm >= program.gather_samples.size())
      {
        continue;
      }
      for (const int local : program.gather_samples[c[gather_pc].imm].output_locals) {
        int store_pc = -1;
        for (int pc = gather_pc - 1; pc >= 0; pc--) {
          if (c[pc].op == Op::PushLocal && c[pc].imm == local) {
            break;
          }
          if (c[pc].op == Op::StoreLocal && c[pc].imm == local) {
            store_pc = pc;
            break;
          }
        }
        if (store_pc <= 0 || c[store_pc - 1].op != Op::Call || c[store_pc - 1].a != 0) {
          continue;
        }
        const Builtin builtin = Builtin(c[store_pc - 1].imm);
        if (!ELEM(builtin,
                  Builtin::ArrayFn,
                  Builtin::ArrayInt,
                  Builtin::ArrayFloat,
                  Builtin::ArrayVec,
                  Builtin::ArrayStr,
                  Builtin::ArrayMat,
                  Builtin::ArrayRay))
        {
          continue;
        }
        c[store_pc - 1] = Inst{};
        c[store_pc] = Inst{};
      }
    }
  }

  /* Propagate immutable scalar locals initialized in the entry block. VEX source commonly uses
   * ordinary locals as compile-time switches (for example `int normalize = 0`). Keeping those as
   * runtime locals prevents the following conditional and its entire dead arm from being removed,
   * so every geometry element used to interpret code which could never run.
   *
   * Restrict candidates to literal stores before the first control-flow instruction and require a
   * single write in the complete program. This makes the transformation valid for loops,
   * conditionals and user functions without needing a full SSA construction. */
  {
    Vector<Inst> &c = program.code;
    const int local_n = std::max(program.local_count, 0);
    Array<int> write_counts(local_n, 0);
    Array<int> literal_pc(local_n, -1);
    Array<int> store_pc(local_n, -1);
    Array<uint8_t> is_target(c.size(), 0);
    int entry_end = int(c.size());

    for (int pc = 0; pc < int(c.size()); pc++) {
      const Inst &in = c[pc];
      if (ELEM(in.op, Op::Jmp, Op::JmpIfFalse, Op::JmpIfTrue)) {
        entry_end = std::min(entry_end, pc);
        if (in.imm >= 0 && in.imm < c.size()) {
          is_target[in.imm] = 1;
        }
      }
      if (in.op == Op::StoreLocal && in.imm >= 0 && in.imm < local_n) {
        write_counts[in.imm]++;
        store_pc[in.imm] = pc;
      }
      else if (ELEM(in.op, Op::IncLocal, Op::DecLocal) && in.imm >= 0 && in.imm < local_n) {
        /* Two writes is enough to reject the slot. */
        write_counts[in.imm] += 2;
      }
    }
    for (const UserFn &fn : program.user_fns) {
      if (fn.entry >= 0 && fn.entry < c.size()) {
        is_target[fn.entry] = 1;
      }
    }

    for (int slot = 0; slot < local_n; slot++) {
      const int dst_pc = store_pc[slot];
      if (write_counts[slot] != 1 || dst_pc <= 0 || dst_pc >= entry_end) {
        continue;
      }
      const Inst &source = c[dst_pc - 1];
      if (!ELEM(source.op, Op::PushF, Op::PushI, Op::PushV, Op::PushB, Op::PushS) ||
          is_target[dst_pc - 1] || is_target[dst_pc])
      {
        continue;
      }
      literal_pc[slot] = dst_pc - 1;
    }

    for (int pc = 0; pc < int(c.size()); pc++) {
      Inst &in = c[pc];
      if (in.op != Op::PushLocal || in.imm < 0 || in.imm >= local_n) {
        continue;
      }
      const int src_pc = literal_pc[in.imm];
      if (src_pc >= 0) {
        in = c[src_pc];
      }
    }
    for (int slot = 0; slot < local_n; slot++) {
      if (literal_pc[slot] >= 0) {
        c[literal_pc[slot]] = Inst{};
        c[store_pc[slot]] = Inst{};
      }
    }

    auto numeric_literal = [&](const Inst &in, float &r_value) -> bool {
      if (in.op == Op::PushI && in.imm >= 0 && in.imm < program.const_i.size()) {
        r_value = float(program.const_i[in.imm]);
        return true;
      }
      if (in.op == Op::PushF && in.imm >= 0 && in.imm < program.const_f.size()) {
        r_value = program.const_f[in.imm];
        return true;
      }
      if (in.op == Op::PushB) {
        r_value = in.imm != 0 ? 1.0f : 0.0f;
        return true;
      }
      return false;
    };

    bool changed = true;
    while (changed) {
      changed = false;
      for (int pc = 0; pc + 2 < int(c.size()); pc++) {
        float a = 0.0f, b = 0.0f;
        if (is_target[pc + 1] || is_target[pc + 2] || !numeric_literal(c[pc], a) ||
            !numeric_literal(c[pc + 1], b) ||
            !ELEM(c[pc + 2].op, Op::Lt, Op::Le, Op::Gt, Op::Ge, Op::Eq, Op::Ne))
        {
          continue;
        }
        bool result = false;
        switch (c[pc + 2].op) {
          case Op::Lt:
            result = a < b;
            break;
          case Op::Le:
            result = a <= b;
            break;
          case Op::Gt:
            result = a > b;
            break;
          case Op::Ge:
            result = a >= b;
            break;
          case Op::Eq:
            result = a == b;
            break;
          case Op::Ne:
            result = a != b;
            break;
          default:
            BLI_assert_unreachable();
            break;
        }
        c[pc].op = Op::PushB;
        c[pc].imm = result ? 1 : 0;
        c[pc].a = 0;
        c[pc + 1] = Inst{};
        c[pc + 2] = Inst{};
        changed = true;
      }
      for (int pc = 0; pc + 1 < int(c.size()); pc++) {
        if (c[pc].op != Op::PushB || is_target[pc]) {
          continue;
        }
        int jump_pc = pc + 1;
        while (jump_pc < int(c.size()) && c[jump_pc].op == Op::Nop && !is_target[jump_pc]) {
          jump_pc++;
        }
        if (jump_pc >= int(c.size()) || is_target[jump_pc] ||
            !ELEM(c[jump_pc].op, Op::JmpIfFalse, Op::JmpIfTrue))
        {
          continue;
        }
        const bool value = c[pc].imm != 0;
        const bool take = c[jump_pc].op == Op::JmpIfTrue ? value : !value;
        c[pc] = Inst{};
        if (take) {
          c[jump_pc].op = Op::Jmp;
          c[jump_pc].a = 0;
        }
        else {
          c[jump_pc] = Inst{};
        }
        changed = true;
      }
    }

    /* Mark reachable byte-code from the main entry and every user-function entry. Constant jump
     * folding above turns dead conditional arms into ordinary unreachable regions. */
    Array<uint8_t> reachable(c.size(), 0);
    Vector<int> pending;
    if (!c.is_empty()) {
      pending.append(0);
    }
    for (const UserFn &fn : program.user_fns) {
      if (fn.entry >= 0 && fn.entry < c.size()) {
        pending.append(fn.entry);
      }
    }
    while (!pending.is_empty()) {
      const int pc = pending.pop_last();
      if (pc < 0 || pc >= c.size() || reachable[pc]) {
        continue;
      }
      reachable[pc] = 1;
      const Inst &in = c[pc];
      if (in.op == Op::Jmp) {
        pending.append(in.imm);
      }
      else if (ELEM(in.op, Op::JmpIfFalse, Op::JmpIfTrue)) {
        pending.append(in.imm);
        pending.append(pc + 1);
      }
      else if (in.op != Op::Return) {
        pending.append(pc + 1);
      }
    }
    for (int pc = 0; pc < int(c.size()); pc++) {
      if (!reachable[pc]) {
        c[pc] = Inst{};
      }
    }
  }

  /* Small counted loops are unrolled by the front-end, but their induction variable and
   * expressions such as `(k + 1) % 3` used to remain runtime byte-code. Propagate integer locals
   * within a single basic block so array-index fusion below receives constants. Control-flow
   * boundaries clear the facts; this is deliberately a local, dominance-safe optimization rather
   * than speculative global constant propagation. */
  {
    Vector<Inst> &c = program.code;
    Array<uint8_t> is_target(c.size(), 0);
    for (const Inst &in : c) {
      if (ELEM(in.op, Op::Jmp, Op::JmpIfFalse, Op::JmpIfTrue) && in.imm >= 0 &&
          in.imm < c.size())
      {
        is_target[in.imm] = 1;
      }
    }
    Array<int> known_value(std::max(program.local_count, 0), 0);
    Array<uint8_t> known(std::max(program.local_count, 0), 0);
    auto clear_known = [&]() { known.fill(0); };
    auto int_literal = [&](const Inst &in, int &r_value) -> bool {
      if (in.op != Op::PushI || in.imm < 0 || in.imm >= program.const_i.size()) {
        return false;
      }
      r_value = program.const_i[in.imm];
      return true;
    };
    auto make_int_literal = [&](Inst &in, const int value) {
      in.op = Op::PushI;
      in.a = 0;
      in.imm = int(program.const_i.size());
      program.const_i.append(value);
    };

    for (int pc = 0; pc < int(c.size()); pc++) {
      if (is_target[pc]) {
        clear_known();
      }
      Inst &in = c[pc];
      if (in.op == Op::PushLocal && in.imm >= 0 && in.imm < known.size() && known[in.imm]) {
        make_int_literal(in, known_value[in.imm]);
      }

      /* Fold the exact integer ring-index shape emitted by common fixed-size array loops. */
      if (pc + 5 < int(c.size()) && !is_target[pc + 1] && !is_target[pc + 2] &&
          !is_target[pc + 3] && !is_target[pc + 4] && !is_target[pc + 5])
      {
        int a = 0, b = 0, divisor = 0;
        if (int_literal(c[pc], a) && int_literal(c[pc + 1], b) &&
            c[pc + 2].op == Op::Add && int_literal(c[pc + 3], divisor) &&
            c[pc + 4].op == Op::Mod && c[pc + 5].op == Op::StoreLocal && divisor != 0 &&
            c[pc + 5].imm >= 0 && c[pc + 5].imm < known.size())
        {
          const int value = (a + b) % divisor;
          make_int_literal(c[pc], value);
          c[pc + 1] = Inst{};
          c[pc + 2] = Inst{};
          c[pc + 3] = Inst{};
          c[pc + 4] = Inst{};
          known[c[pc + 5].imm] = 1;
          known_value[c[pc + 5].imm] = value;
          pc += 4;
          continue;
        }
      }

      if (in.op == Op::StoreLocal && in.imm >= 0 && in.imm < known.size()) {
        int value = 0;
        if (pc > 0 && int_literal(c[pc - 1], value)) {
          known[in.imm] = 1;
          known_value[in.imm] = value;
        }
        else {
          known[in.imm] = 0;
        }
      }
      else if (ELEM(in.op, Op::IncLocal, Op::DecLocal) && in.imm >= 0 &&
               in.imm < known.size())
      {
        if (known[in.imm]) {
          known_value[in.imm] += in.op == Op::IncLocal ? 1 : -1;
        }
      }
      if (ELEM(in.op, Op::Jmp, Op::JmpIfFalse, Op::JmpIfTrue, Op::Return)) {
        clear_known();
      }
    }
  }

  /* Collapse common local array/member loads into one dispatch. This is a general byte-code
   * peephole and does not depend on a particular VEX source formula. The opt-out is intentionally
   * undocumented and only used for same-binary profiler A/B checks. */
  if (std::getenv("BLENDER_VEX_DISABLE_LOCAL_LOAD_FUSION") == nullptr) {
    Vector<Inst> &c = program.code;
    Array<uint8_t> is_target(c.size(), 0);
    for (const Inst &in : c) {
      if (ELEM(in.op, Op::Jmp, Op::JmpIfFalse, Op::JmpIfTrue) && in.imm >= 0 &&
          in.imm < c.size())
      {
        is_target[in.imm] = 1;
      }
    }
    for (const UserFn &fn : program.user_fns) {
      if (fn.entry >= 0 && fn.entry < c.size()) {
        is_target[fn.entry] = 1;
      }
    }
    for (int pc = 0; pc < int(c.size()); pc++) {
      if (pc + 2 < int(c.size()) && c[pc].op == Op::PushLocal &&
          ELEM(c[pc + 1].op, Op::PushLocal, Op::PushI) && c[pc + 2].op == Op::GetIndex &&
          !is_target[pc + 1] && !is_target[pc + 2])
      {
        int end = pc + 2;
        int member = -1;
        if (pc + 3 < int(c.size()) && c[pc + 3].op == Op::GetMember &&
            (c[pc + 3].imm >> 8) == 0 && !is_target[pc + 3])
        {
          member = c[pc + 3].imm;
          end = pc + 3;
        }
        Program::LocalIndexLoad load;
        load.array_local = c[pc].imm;
        load.index = c[pc + 1].imm;
        load.member = member;
        load.index_is_const = c[pc + 1].op == Op::PushI;
        c[pc].op = Op::PushLocalIndex;
        c[pc].imm = int(program.local_index_loads.size());
        c[pc].a = 0;
        program.local_index_loads.append(load);
        for (int clear = pc + 1; clear <= end; clear++) {
          c[clear] = Inst{};
        }
        continue;
      }
      if (pc + 1 < int(c.size()) && c[pc].op == Op::PushLocal &&
          c[pc + 1].op == Op::GetMember && (c[pc + 1].imm >> 8) == 0 &&
          !is_target[pc + 1])
      {
        c[pc].op = Op::PushLocalMember;
        c[pc].a = uint8_t(c[pc + 1].imm);
        c[pc + 1] = Inst{};
      }
    }

    /* Connect fused local-index loads back to the static attribute span that produced the local
     * array. This avoids a second indirection through a tiny TLS Vector for every `values[k]`.
     * Output arrays with no remaining whole-array consumer do not need to be materialized at all;
     * computed attributes without a span still materialize at runtime as a correctness fallback. */
    if (!program.gather_samples.is_empty() && program.local_count > 0) {
      Array<int> gather_for_local(program.local_count, -1);
      Array<int> item_for_local(program.local_count, -1);
      for (const int gather_index : program.gather_samples.index_range()) {
        Program::GatherSamples &gather = program.gather_samples[gather_index];
        gather.materialize.reinitialize(gather.output_locals.size());
        gather.materialize.fill(0);
        for (const int item : gather.output_locals.index_range()) {
          const int local = gather.output_locals[item];
          if (local >= 0 && local < program.local_count) {
            gather_for_local[local] = gather_index;
            item_for_local[local] = item;
          }
        }
      }
      for (Program::LocalIndexLoad &load : program.local_index_loads) {
        if (load.array_local < 0 || load.array_local >= program.local_count) {
          continue;
        }
        const int gather_index = gather_for_local[load.array_local];
        const int item = item_for_local[load.array_local];
        if (gather_index < 0 || item < 0) {
          continue;
        }
        const Program::GatherSamples &gather = program.gather_samples[gather_index];
        if (item < gather.sample_slots.size()) {
          load.sample_slot = gather.sample_slots[item];
          load.sample_index_array_local = gather.index_array_local;
        }
      }
      for (const Inst &in : c) {
        if (!ELEM(in.op, Op::PushLocal, Op::PushLocalMember) || in.imm < 0 ||
            in.imm >= program.local_count)
        {
          continue;
        }
        const int gather_index = gather_for_local[in.imm];
        const int item = item_for_local[in.imm];
        if (gather_index >= 0 && item >= 0) {
          program.gather_samples[gather_index].materialize[item] = 1;
        }
      }
    }

  /* Every peephole pass above may leave Nops. Compact once and remap the byte-code indices used by
   * jumps and user-function entries. */
  {
    const int old_size = int(program.code.size());
    Array<int> old_to_new(old_size + 1, 0);
    int new_size = 0;
    for (int pc = 0; pc < old_size; pc++) {
      old_to_new[pc] = new_size;
      if (program.code[pc].op != Op::Nop) {
        new_size++;
      }
    }
    old_to_new[old_size] = new_size;
    Vector<Inst> compact;
    compact.reserve(new_size);
    for (const Inst &old : program.code) {
      if (old.op == Op::Nop) {
        continue;
      }
      Inst in = old;
      if (ELEM(in.op, Op::Jmp, Op::JmpIfFalse, Op::JmpIfTrue)) {
        in.imm = old_to_new[std::clamp(in.imm, 0, old_size)];
      }
      compact.append(in);
    }
    for (UserFn &fn : program.user_fns) {
      fn.entry = old_to_new[std::clamp(fn.entry, 0, old_size)];
    }
    program.code = std::move(compact);
  }
  }

  infer_spatial_kernel(program);
}

enum class SpatSym : int8_t {
  None = 0,
  Other,
  ProxPos,
  ProxDist,
  RayHit,
  RayPos,
  RayN,
  RayDist,
};

static bool spat_tagged(const SpatSym s)
{
  return s != SpatSym::None && s != SpatSym::Other;
}

static SpatSym spat_through(const SpatSym a, const SpatSym b)
{
  if (spat_tagged(a)) {
    return a;
  }
  if (spat_tagged(b)) {
    return b;
  }
  if (a == SpatSym::Other || b == SpatSym::Other) {
    return SpatSym::Other;
  }
  return SpatSym::None;
}

static void spat_add_attr(int &primary, Vector<int> &all, const int slot)
{
  if (slot < 0) {
    return;
  }
  for (const int s : all) {
    if (s == slot) {
      return;
    }
  }
  all.append(slot);
  if (primary < 0) {
    primary = slot;
  }
}

static void infer_spatial_kernel(Program &program)
{
  program.spatial_only = false;
  program.prox_pos_attr = -1;
  program.prox_dist_attr = -1;
  program.ray_hit_attr = -1;
  program.ray_pos_attr = -1;
  program.ray_n_attr = -1;
  program.ray_dist_attr = -1;
  program.prox_pos_attrs.clear();
  program.prox_dist_attrs.clear();
  program.ray_hit_attrs.clear();
  program.ray_pos_attrs.clear();
  program.ray_n_attrs.clear();
  program.ray_dist_attrs.clear();
  if (!program.prox_batch && !program.ray_batch) {
    return;
  }

  Vector<SpatSym> stack;
  const int local_n = std::max(program.local_count, 1);
  Array<SpatSym> locals(local_n, SpatSym::None);
  auto push = [&](const SpatSym s) { stack.append(s); };
  auto pop = [&]() -> SpatSym {
    if (stack.is_empty()) {
      return SpatSym::None;
    }
    const SpatSym s = stack.last();
    stack.remove_last();
    return s;
  };

  bool other = false;
  bool saw_spatial = false;
  for (const Inst &in : program.code) {
    switch (in.op) {
      case Op::Nop:
      case Op::IncLocal:
      case Op::DecLocal:
      case Op::Return:
        break;
      case Op::Pop:
        pop();
        break;
      case Op::Dup:
        push(stack.is_empty() ? SpatSym::None : stack.last());
        break;
      case Op::PushF:
      case Op::PushI:
      case Op::PushV:
      case Op::PushB:
      case Op::PushS:
      case Op::PushAttr:
      case Op::PushIndex:
      case Op::PushNpoints:
      case Op::PushNedges:
      case Op::PushNfaces:
      case Op::PushNcorners:
        push(SpatSym::None);
        break;
      case Op::PushHitPos:
        push(SpatSym::ProxPos);
        break;
      case Op::PushLocal:
        push((in.imm >= 0 && in.imm < local_n) ? locals[in.imm] : SpatSym::None);
        break;
      case Op::PushLocalIndex:
      case Op::PushLocalMember:
        other = true;
        push(SpatSym::Other);
        break;
      case Op::StoreLocal: {
        const SpatSym s = pop();
        if (in.imm >= 0 && in.imm < local_n) {
          locals[in.imm] = s;
        }
        break;
      }
      case Op::StoreAttr: {
        const SpatSym s = pop();
        switch (s) {
          case SpatSym::ProxPos:
            spat_add_attr(program.prox_pos_attr, program.prox_pos_attrs, in.imm);
            break;
          case SpatSym::ProxDist:
            spat_add_attr(program.prox_dist_attr, program.prox_dist_attrs, in.imm);
            break;
          case SpatSym::RayHit:
            spat_add_attr(program.ray_hit_attr, program.ray_hit_attrs, in.imm);
            break;
          case SpatSym::RayPos:
            spat_add_attr(program.ray_pos_attr, program.ray_pos_attrs, in.imm);
            break;
          case SpatSym::RayN:
            spat_add_attr(program.ray_n_attr, program.ray_n_attrs, in.imm);
            break;
          case SpatSym::RayDist:
            spat_add_attr(program.ray_dist_attr, program.ray_dist_attrs, in.imm);
            break;
          case SpatSym::None:
          case SpatSym::Other:
            /* Extra non-hit stores cannot be done by try_spatial_direct. */
            other = true;
            break;
        }
        break;
      }
      case Op::LengthV3:
      case Op::SqrtF:
      case Op::LogF:
      case Op::AbsF:
      case Op::ArrayLen:
        push(pop());
        break;
      case Op::DotV3:
      case Op::CrossV3: {
        const SpatSym b = pop();
        const SpatSym a = pop();
        push(spat_through(a, b));
        break;
      }
      case Op::Call: {
        const Builtin b = Builtin(in.imm);
        const int n = int(in.a);
        SpatSym args_sym = SpatSym::None;
        for (int i = 0; i < n; i++) {
          args_sym = spat_through(args_sym, pop());
        }
        if (b == Builtin::GeometryProximity) {
          saw_spatial = true;
          push(SpatSym::ProxDist);
        }
        else if (b == Builtin::Raycast) {
          saw_spatial = true;
          push(SpatSym::RayHit);
        }
        else if (b == Builtin::RayIsHit) {
          push(SpatSym::RayHit);
        }
        else if (b == Builtin::RayHitPos) {
          push(SpatSym::RayPos);
        }
        else if (b == Builtin::RayHitN) {
          push(SpatSym::RayN);
        }
        else if (b == Builtin::RayHitDist) {
          push(SpatSym::RayDist);
        }
        else if (is_geo_builtin(b)) {
          other = true;
          push(SpatSym::Other);
        }
        else {
          /* normalize() / vec3() coerce: keep spatial tags so out-param
           * stores still map onto ray_* / prox_* attrs. */
          push(args_sym);
        }
        break;
      }
      case Op::SampleElem: {
        const int n = int(in.a & 0x1f);
        for (int i = 0; i < n; i++) {
          pop();
        }
        other = true;
        push(SpatSym::Other);
        break;
      }
      case Op::FaceCorners: {
        const int n = int(in.a);
        for (int i = 0; i < n; i++) {
          pop();
        }
        other = true;
        push(SpatSym::Other);
        break;
      }
      case Op::GatherSamples:
        other = true;
        break;
      case Op::MakeVec: {
        const int n = std::max(int(in.imm), 0);
        SpatSym s = SpatSym::None;
        for (int i = 0; i < n; i++) {
          s = spat_through(s, pop());
        }
        push(s == SpatSym::None ? SpatSym::None : SpatSym::Other);
        break;
      }
      case Op::Neg:
      case Op::Not:
      case Op::GetMember: {
        const SpatSym s = pop();
        push(s == SpatSym::None ? SpatSym::None : SpatSym::Other);
        break;
      }
      case Op::SetMember: {
        pop();
        push(pop());
        break;
      }
      case Op::Add:
      case Op::Sub:
      case Op::Mul:
      case Op::Div:
      case Op::AddFF:
      case Op::AddVV:
      case Op::SubFF:
      case Op::SubVV:
      case Op::MulFF:
      case Op::MulFV:
      case Op::MulVF:
      case Op::MulVV:
      case Op::DivFF:
      case Op::DivVF:
      case Op::DivVV:
      case Op::Mod:
      case Op::Pow:
      case Op::Lt:
      case Op::Le:
      case Op::Gt:
      case Op::Ge:
      case Op::Eq:
      case Op::Ne:
      case Op::And:
      case Op::Or: {
        const SpatSym rhs = pop();
        const SpatSym lhs = pop();
        const SpatSym s = spat_through(lhs, rhs);
        push(s == SpatSym::None ? SpatSym::None : SpatSym::Other);
        break;
      }
      default:
        other = true;
        break;
    }
  }
  program.spatial_only = saw_spatial && !other;
}

bool builtin_is_uniform_value(const Builtin id)
{
  switch (id) {
    case Builtin::ArrayFn:
    case Builtin::ArrayInt:
    case Builtin::ArrayFloat:
    case Builtin::ArrayVec:
    case Builtin::ArrayStr:
    case Builtin::ArrayMat:
    case Builtin::ArrayRay:
    case Builtin::MatrixFn:
    case Builtin::Identity:
    case Builtin::ColorFn:
    case Builtin::Vec2Fn:
    case Builtin::Vec3Fn:
    case Builtin::Vec4Fn:
    case Builtin::FloatFn:
    case Builtin::IntFn:
    case Builtin::BoolFn:
    case Builtin::QuatFn:
    case Builtin::Invert:
    case Builtin::Transpose:
    case Builtin::Determinant:
    case Builtin::TranslationFn:
    case Builtin::RotationFn:
    case Builtin::ScaleFn:
    case Builtin::CombineTransform:
    case Builtin::Append:
    case Builtin::Insert:
    case Builtin::RemoveIndex:
    case Builtin::RemoveValue:
    case Builtin::SortArr:
    case Builtin::Len:
    case Builtin::ValueToString:
    case Builtin::Format:
      return true;
    default:
      return false;
  }
}

bool program_can_array_exec(const Program &program)
{
  if (!program.user_fns.is_empty()) {
    return false;
  }
  for (const Inst &in : program.code) {
    switch (in.op) {
      case Op::Pow:
      case Op::CallUser:
      case Op::SampleElem:
      case Op::FaceCorners:
      case Op::GatherSamples:
      case Op::PushLocalIndex:
      case Op::PushLocalMember:
        return false;
      case Op::PushAttr:
      case Op::PushIndex:
      case Op::GetMember:
      case Op::SetMember:
      case Op::GetIndex:
      case Op::SetIndex:
      case Op::PushHitPos:
        return false;
      case Op::Call:
        if (!builtin_is_uniform_value(Builtin(in.imm))) {
          return false;
        }
        break;
      default:
        break;
    }
  }
  return true;
}

bool program_has_jumps(const Program &program)
{
  for (const Inst &in : program.code) {
    if (ELEM(in.op, Op::Jmp, Op::JmpIfFalse, Op::JmpIfTrue)) {
      return true;
    }
  }
  return false;
}

static int vm_lane_grain(const int code_size)
{
  if (const char *override_value = std::getenv("BLENDER_VEX_GRAIN")) {
    const int value = std::atoi(override_value);
    if (value > 0) {
      return std::clamp(value, 1, 1 << 20);
    }
  }
  /* Target roughly tens of microseconds per scheduled chunk. Tiny grains create thousands of TBB
   * tasks for medium meshes and cost more than the interpreted work they distribute. */
  return code_size >= 128 ? 64 : (code_size >= 32 ? 256 : 512);
}

static bool program_can_wavefront_exec(const Program &program)
{
  if (!program.user_fns.is_empty() || program.max_stack > 48 || program.local_count > 48) {
    return false;
  }
  for (const Inst &in : program.code) {
    switch (in.op) {
      case Op::Nop:
      case Op::PushF:
      case Op::PushI:
      case Op::PushV:
      case Op::PushB:
      case Op::PushS:
      case Op::PushLocal:
      case Op::StoreLocal:
      case Op::Dup:
      case Op::Pop:
      case Op::PushAttr:
      case Op::StoreAttr:
      case Op::AttrAdd:
      case Op::AttrMulAdd:
      case Op::IncLocal:
      case Op::DecLocal:
      case Op::PushIndex:
      case Op::PushNpoints:
      case Op::PushNedges:
      case Op::PushNfaces:
      case Op::PushNcorners:
      case Op::Add:
      case Op::Sub:
      case Op::Mul:
      case Op::Div:
      case Op::AddFF:
      case Op::AddVV:
      case Op::SubFF:
      case Op::SubVV:
      case Op::MulFF:
      case Op::MulFV:
      case Op::MulVF:
      case Op::MulVV:
      case Op::DivFF:
      case Op::DivVF:
      case Op::DivVV:
      case Op::Mod:
      case Op::Pow:
      case Op::Neg:
      case Op::Lt:
      case Op::Le:
      case Op::Gt:
      case Op::Ge:
      case Op::Eq:
      case Op::Ne:
      case Op::And:
      case Op::Or:
      case Op::Not:
      case Op::Jmp:
      case Op::JmpIfFalse:
      case Op::JmpIfTrue:
      case Op::LengthV3:
      case Op::DotV3:
      case Op::CrossV3:
      case Op::SqrtF:
      case Op::LogF:
      case Op::AbsF:
      case Op::ArrayLen:
      case Op::Call:
      case Op::SampleElem:
      case Op::FaceCorners:
      case Op::GatherSamples:
      case Op::PushLocalIndex:
      case Op::PushLocalMember:
      case Op::GetMember:
      case Op::SetMember:
      case Op::MakeVec:
      case Op::GetIndex:
      case Op::SetIndex:
      case Op::Return:
      case Op::PushHitPos:
      case Op::WhileCmpAdd:
        break;
      case Op::CallUser:
        return false;
    }
  }
  return true;
}

/* Execute one byte-code instruction for all lanes currently at the same program counter. This
 * amortizes the large switch/dispatch cost over a 32-element wave while retaining independent
 * stacks, locals and control flow. Divergent branches are routed through per-PC lane masks and
 * reconverge naturally; loops revisit their target block without falling back to scalar dispatch. */
static bool vm_run_wavefront(const Program &program,
                             VMEnv &env,
                             const IndexMask &mask,
                             std::string &r_error)
{
  constexpr int TILE = 32;
  constexpr int ST = 48;
  std::atomic<bool> ok{true};
  std::mutex err_mutex;
  const Span<Inst> code = program.code;
  const int code_n = int(code.size());
  const int local_n = std::max(program.local_count, 1);
  const int lane_grain = vm_lane_grain(code_n);

  mask.foreach_segment(
      [&](const index_mask::IndexMaskSegment segment) {
        if (!ok.load(std::memory_order_relaxed)) {
          return;
        }
        for (int64_t off = 0; off < segment.size(); off += TILE) {
          const int n = int(std::min<int64_t>(TILE, segment.size() - off));
          tls_arrays_begin();
          Value stacks[TILE][ST];
          Value locals[TILE][ST];
          uint8_t stack_size[TILE] = {};
          int64_t jumps[TILE] = {};
          VMEnv lanes[TILE];
          for (int lane = 0; lane < n; lane++) {
            lanes[lane] = env;
            lanes[lane].const_s = program.const_s.as_span();
            lanes[lane].index = int(segment[off + lane]);
            lanes[lane].rand_seq = 0;
            for (int i = 0; i < local_n; i++) {
              locals[lane][i] = Value{};
            }
          }

          Array<uint32_t> pc_masks(code_n + 1, 0);
          Array<uint8_t> queued(code_n + 1, 0);
          Vector<int> work;
          auto enqueue = [&](const int pc, const uint32_t bits) {
            if (bits == 0 || pc < 0 || pc > code_n) {
              return;
            }
            pc_masks[pc] |= bits;
            if (!queued[pc]) {
              queued[pc] = 1;
              work.append(pc);
            }
          };
          auto for_lanes = [&](const uint32_t bits, const auto &fn) {
            for (int lane = 0; lane < n; lane++) {
              if (bits & (uint32_t(1) << lane)) {
                fn(lane);
              }
            }
          };
          auto push = [&](const int lane, const Value &value) {
            if (stack_size[lane] < ST) {
              stacks[lane][stack_size[lane]++] = value;
            }
          };
          auto pop = [&](const int lane) -> Value {
            if (stack_size[lane] == 0) {
              return Value::from_float(0.0f);
            }
            return stacks[lane][--stack_size[lane]];
          };
          std::string tile_error;
          enqueue(0, n == 32 ? 0xffffffffu : ((uint32_t(1) << n) - 1u));

          while (!work.is_empty() && tile_error.empty()) {
            const int pc = work.pop_last();
            queued[pc] = 0;
            const uint32_t bits = pc_masks[pc];
            pc_masks[pc] = 0;
            if (bits == 0 || pc == code_n) {
              continue;
            }
            const Inst &in = code[pc];
            bool route_next = true;
            switch (in.op) {
              case Op::Nop:
                break;
              case Op::PushF:
                for_lanes(bits, [&](const int l) { push(l, Value::from_float(program.const_f[in.imm])); });
                break;
              case Op::PushI:
                for_lanes(bits, [&](const int l) { push(l, Value::from_int(program.const_i[in.imm])); });
                break;
              case Op::PushV:
                for_lanes(bits, [&](const int l) { push(l, Value::from_vec(program.const_v[in.imm])); });
                break;
              case Op::PushB:
                for_lanes(bits, [&](const int l) { push(l, Value::from_bool(in.imm != 0)); });
                break;
              case Op::PushS:
                for_lanes(bits, [&](const int l) { push(l, Value::from_str_i(in.imm)); });
                break;
              case Op::PushLocal:
                for_lanes(bits, [&](const int l) {
                  push(l, in.imm >= 0 && in.imm < local_n ? locals[l][in.imm] : Value{});
                });
                break;
              case Op::PushLocalIndex:
                for_lanes(bits, [&](const int l) {
                  push(l, load_local_index(program, in, locals[l], local_n, &lanes[l]));
                });
                break;
              case Op::PushLocalMember:
                for_lanes(bits, [&](const int l) {
                  push(l,
                       in.imm >= 0 && in.imm < local_n ?
                           apply_get_member(locals[l][in.imm], int(in.a)) :
                           Value{});
                });
                break;
              case Op::StoreLocal:
                for_lanes(bits, [&](const int l) {
                  const Value value = pop(l);
                  if (in.imm >= 0 && in.imm < local_n) {
                    locals[l][in.imm] = value;
                  }
                });
                break;
              case Op::IncLocal:
              case Op::DecLocal:
                for_lanes(bits, [&](const int l) {
                  if (in.imm >= 0 && in.imm < local_n) {
                    Value &value = locals[l][in.imm];
                    value.type = Type::Int;
                    value.i += in.op == Op::IncLocal ? 1 : -1;
                    value.v.x = float(value.i);
                  }
                });
                break;
              case Op::Dup:
                for_lanes(bits, [&](const int l) {
                  if (stack_size[l] > 0) {
                    push(l, stacks[l][stack_size[l] - 1]);
                  }
                });
                break;
              case Op::Pop:
                for_lanes(bits, [&](const int l) {
                  if (stack_size[l] > 0) {
                    stack_size[l]--;
                  }
                });
                break;
              case Op::PushAttr:
                for_lanes(bits, [&](const int l) {
                  push(l,
                       in.imm >= 0 && in.imm < lanes[l].attrs.size() ?
                           load_attr_value(lanes[l].attrs[in.imm], lanes[l].index, lanes[l]) :
                           Value{});
                });
                break;
              case Op::StoreAttr:
                for_lanes(bits, [&](const int l) {
                  const Value value = pop(l);
                  if (in.imm >= 0 && in.imm < lanes[l].attrs.size()) {
                    store_attr_value(lanes[l].attrs[in.imm], lanes[l].index, value, lanes[l]);
                  }
                });
                break;
              case Op::AttrAdd:
                for_lanes(bits, [&](const int l) {
                  const Value value = pop(l);
                  if (in.imm >= 0 && in.imm < lanes[l].attrs.size()) {
                    attr_add_inplace(lanes[l].attrs[in.imm], lanes[l].index, value);
                  }
                });
                break;
              case Op::AttrMulAdd:
                for_lanes(bits, [&](const int l) {
                  const Value k = pop(l), b = pop(l);
                  if (in.imm >= 0 && in.imm < lanes[l].attrs.size()) {
                    attr_add_inplace(lanes[l].attrs[in.imm], lanes[l].index, scale_value(b, k.as_int()));
                  }
                });
                break;
              case Op::PushIndex:
                for_lanes(bits, [&](const int l) { push(l, Value::from_int(lanes[l].index)); });
                break;
              case Op::PushNpoints:
                for_lanes(bits, [&](const int l) { push(l, Value::from_int(lanes[l].npoints)); });
                break;
              case Op::PushNedges:
                for_lanes(bits, [&](const int l) { push(l, Value::from_int(lanes[l].nedges)); });
                break;
              case Op::PushNfaces:
                for_lanes(bits, [&](const int l) { push(l, Value::from_int(lanes[l].nfaces)); });
                break;
              case Op::PushNcorners:
                for_lanes(bits, [&](const int l) { push(l, Value::from_int(lanes[l].ncorners)); });
                break;
              case Op::PushHitPos:
                for_lanes(bits, [&](const int l) { push(l, Value::from_vec(lanes[l].hit_pos)); });
                break;
              case Op::Add:
                for_lanes(bits, [&](const int l) {
                  const Value b = pop(l);
                  if (stack_size[l] > 0) {
                    add_inplace(stacks[l][stack_size[l] - 1], b);
                  }
                });
                break;
              case Op::Sub:
              case Op::Mul:
              case Op::Div:
                for_lanes(bits, [&](const int l) {
                  const Value b = pop(l), a = pop(l);
                  push(l,
                       in.op == Op::Sub ? numeric_sub(a, b) :
                       in.op == Op::Mul ? numeric_mul(a, b, nullptr) : numeric_div(a, b));
                });
                break;
              case Op::AddFF:
              case Op::SubFF:
              case Op::MulFF:
              case Op::DivFF:
                for_lanes(bits, [&](const int l) {
                  const Value b = pop(l), a = pop(l);
                  const float value = in.op == Op::AddFF ? a.v.x + b.v.x :
                                      in.op == Op::SubFF ? a.v.x - b.v.x :
                                      in.op == Op::MulFF ? a.v.x * b.v.x :
                                                          (b.v.x != 0.0f ? a.v.x / b.v.x : 0.0f);
                  push(l, Value::from_float(value));
                });
                break;
              case Op::AddVV:
              case Op::SubVV:
              case Op::MulVV:
              case Op::DivVV:
                for_lanes(bits, [&](const int l) {
                  const Value b = pop(l), a = pop(l);
                  const float3 av = a.as_vec(), bv = b.as_vec();
                  push(l,
                       Value::from_vec(in.op == Op::AddVV ? av + bv :
                                       in.op == Op::SubVV ? av - bv :
                                       in.op == Op::MulVV ? av * bv : av / bv));
                });
                break;
              case Op::MulFV:
              case Op::MulVF:
              case Op::DivVF:
                for_lanes(bits, [&](const int l) {
                  const Value b = pop(l), a = pop(l);
                  if (in.op == Op::MulFV) {
                    push(l, Value::from_vec(a.v.x * b.as_vec()));
                  }
                  else {
                    const float factor = in.op == Op::DivVF ?
                                             (b.v.x != 0.0f ? 1.0f / b.v.x : 0.0f) :
                                             b.v.x;
                    push(l, Value::from_vec(a.as_vec() * factor));
                  }
                });
                break;
              case Op::Mod:
              case Op::Pow:
                for_lanes(bits, [&](const int l) {
                  const Value b = pop(l), a = pop(l);
                  if (in.op == Op::Pow) {
                    push(l, Value::from_float(powf(a.as_float(), b.as_float())));
                  }
                  else if (a.type == Type::Int && b.type == Type::Int) {
                    const int d = b.as_int();
                    push(l, Value::from_int(d != 0 ? a.as_int() % d : 0));
                  }
                  else {
                    const float d = b.as_float();
                    push(l, Value::from_float(d != 0.0f ? fmodf(a.as_float(), d) : 0.0f));
                  }
                });
                break;
              case Op::Neg:
                for_lanes(bits, [&](const int l) {
                  Value a = pop(l);
                  if (a.type == Type::Vector) {
                    push(l, Value::from_vec(-a.as_vec()));
                  }
                  else if (a.type == Type::Vector2) {
                    push(l, Value::from_vec2(-a.as_vec2()));
                  }
                  else if (a.type == Type::Int) {
                    push(l, Value::from_int(-a.as_int()));
                  }
                  else {
                    push(l, Value::from_float(-a.as_float()));
                  }
                });
                break;
              case Op::Lt:
              case Op::Le:
              case Op::Gt:
              case Op::Ge:
              case Op::Eq:
              case Op::Ne:
                for_lanes(bits, [&](const int l) {
                  const float b = pop(l).as_float(), a = pop(l).as_float();
                  const bool value = in.op == Op::Lt ? a < b :
                                     in.op == Op::Le ? a <= b :
                                     in.op == Op::Gt ? a > b :
                                     in.op == Op::Ge ? a >= b :
                                     in.op == Op::Eq ? a == b : a != b;
                  push(l, Value::from_bool(value));
                });
                break;
              case Op::And:
              case Op::Or:
                for_lanes(bits, [&](const int l) {
                  const bool b = pop(l).as_bool(), a = pop(l).as_bool();
                  push(l, Value::from_bool(in.op == Op::And ? a && b : a || b));
                });
                break;
              case Op::Not:
                for_lanes(bits, [&](const int l) { push(l, Value::from_bool(!pop(l).as_bool())); });
                break;
              case Op::Jmp:
                for_lanes(bits, [&](const int l) {
                  if (++jumps[l] > 100000000) {
                    tile_error = "Instruction limit exceeded (infinite loop?)";
                  }
                });
                enqueue(in.imm, bits);
                route_next = false;
                break;
              case Op::JmpIfFalse:
              case Op::JmpIfTrue: {
                uint32_t taken = 0, fallthrough = 0;
                for_lanes(bits, [&](const int l) {
                  if (++jumps[l] > 100000000) {
                    tile_error = "Instruction limit exceeded (infinite loop?)";
                    return;
                  }
                  const bool value = pop(l).as_bool();
                  const bool take = in.op == Op::JmpIfTrue ? value : !value;
                  (take ? taken : fallthrough) |= uint32_t(1) << l;
                });
                enqueue(in.imm, taken);
                enqueue(pc + 1, fallthrough);
                route_next = false;
                break;
              }
              case Op::LengthV3:
              case Op::SqrtF:
              case Op::LogF:
              case Op::AbsF:
              case Op::ArrayLen:
                for_lanes(bits, [&](const int l) {
                  const Value a = pop(l);
                  push(l,
                       in.op == Op::LengthV3 ? typed_length_v3(a) :
                       in.op == Op::SqrtF ? typed_sqrt_f(a) :
                       in.op == Op::LogF ? typed_log_f(a) :
                       in.op == Op::AbsF ? typed_abs_f(a) : typed_array_len(a));
                });
                break;
              case Op::DotV3:
              case Op::CrossV3:
                for_lanes(bits, [&](const int l) {
                  const Value b = pop(l), a = pop(l);
                  push(l, in.op == Op::DotV3 ? typed_dot_v3(a, b) : typed_cross_v3(a, b));
                });
                break;
              case Op::Call:
              case Op::SampleElem:
              case Op::FaceCorners:
                for_lanes(bits, [&](const int l) {
                  const int nargs = in.op == Op::SampleElem ? int(in.a & 0x1f) : int(in.a);
                  Value args[32];
                  const int na = std::min(nargs, 32);
                  for (int a = na - 1; a >= 0; a--) {
                    args[a] = pop(l);
                  }
                  std::string error;
                  Value value;
                  if (in.op == Op::Call) {
                    value = call_builtin(Builtin(in.imm), Span<Value>(args, na), lanes[l], error);
                  }
                  else if (in.op == Op::SampleElem) {
                    value = call_static_sample(in, Span<Value>(args, na), lanes[l], error);
                  }
                  else {
                    value = face_corners_direct(in, Span<Value>(args, na), lanes[l]);
                  }
                  if (!error.empty()) {
                    tile_error = error;
                  }
                  push(l, value);
                });
                break;
              case Op::GatherSamples:
                for_lanes(bits, [&](const int l) {
                  std::string error;
                  if (!gather_samples_direct(program, in, locals[l], local_n, lanes[l], error)) {
                    tile_error = error;
                  }
                });
                break;
              case Op::GetMember:
                for_lanes(bits, [&](const int l) { push(l, apply_get_member(pop(l), in.imm)); });
                break;
              case Op::SetMember:
                for_lanes(bits, [&](const int l) {
                  const Value obj = pop(l), member = pop(l);
                  push(l, apply_set_member(obj, member, in.imm));
                });
                break;
              case Op::GetIndex:
                for_lanes(bits, [&](const int l) {
                  const int index = pop(l).as_int();
                  push(l, get_index_value(pop(l), index));
                });
                break;
              case Op::SetIndex:
                for_lanes(bits, [&](const int l) {
                  const int index = pop(l).as_int();
                  const Value array = pop(l), elem = pop(l);
                  push(l, set_index_value(array, index, elem, lanes[l]));
                });
                break;
              case Op::MakeVec:
                for_lanes(bits, [&](const int l) {
                  float values[4] = {};
                  for (int i = std::min(in.imm, 4) - 1; i >= 0; i--) {
                    values[i] = pop(l).as_float();
                  }
                  push(l, make_vec_value(in.imm, values[0], values[1], values[2], values[3]));
                });
                break;
              case Op::WhileCmpAdd:
                for_lanes(bits, [&](const int l) {
                  if (in.imm >= 0 && in.imm < program.while_adds.size()) {
                    while_cmp_add_one(lanes[l].attrs, program.while_adds[in.imm], lanes[l].index);
                  }
                });
                break;
              case Op::Return:
                route_next = false;
                break;
              case Op::CallUser:
                BLI_assert_unreachable();
                tile_error = "Unsupported wavefront user function";
                route_next = false;
                break;
            }
            if (route_next) {
              enqueue(pc + 1, bits);
            }
          }
          if (!tile_error.empty()) {
            bool expected = true;
            if (ok.compare_exchange_strong(expected, false, std::memory_order_relaxed)) {
              std::lock_guard lock(err_mutex);
              r_error = tile_error;
            }
            return;
          }
        }
      },
      exec_mode::grain_size(lane_grain));
  return ok.load();
}

/* Scalar fixed-storage fallback for programs which cannot enter the wavefront executor. */
static bool vm_run_tiles_scalar(const Program &program,
                                VMEnv &env,
                                const IndexMask &mask,
                                std::string &r_error)
{
  constexpr int TILE = 64;
  constexpr int ST = 48;
  std::atomic<bool> ok{true};
  std::mutex err_mutex;
  const Span<Inst> code = program.code;
  const int code_n = int(code.size());
  const int local_n = std::max(program.local_count, 1);
  const int lane_grain = vm_lane_grain(code_n);

  mask.foreach_segment(
      [&](const index_mask::IndexMaskSegment segment) {
        if (!ok.load(std::memory_order_relaxed)) {
          return;
        }
        VMEnv local = env;
        local.const_s = program.const_s.as_span();
        tls_arrays_begin();
        Value stack[ST];
        Value locals_s[ST];
        for (int i = 0; i < local_n && i < ST; i++) {
          locals_s[i] = Value{};
        }
        for (int64_t off = 0; off < segment.size();) {
          if (!ok.load(std::memory_order_relaxed)) {
            return;
          }
          const int n = int(std::min<int64_t>(TILE, segment.size() - off));
          int idx[TILE];
          for (int t = 0; t < n; t++) {
            idx[t] = int(segment[off + t]);
          }
          /* Uniform locals across the tile (straight-line, no varying stores to
           * locals from attributes in the first pass — re-init per index). */
          for (int t = 0; t < n; t++) {
            if (!ok.load(std::memory_order_relaxed)) {
              return;
            }
            local.index = idx[t];
            local.rand_seq = 0;
            tls_arrays_begin();
            int sp = 0;
            int64_t jumps = 0;
            auto push = [&](const Value &v) {
              if (sp < ST) {
                stack[sp++] = v;
              }
            };
            auto pop = [&]() -> Value {
              if (sp <= 0) {
                return Value::from_float(0.0f);
              }
              return stack[--sp];
            };
            for (int i = 0; i < local_n && i < ST; i++) {
              locals_s[i] = Value{};
            }
            std::string err;
            bool lane_ok = true;
            for (int pc = 0; pc < code_n && lane_ok;) {
              const Inst &in = code[pc];
              switch (in.op) {
                case Op::Nop:
                  break;
                case Op::PushF:
                  push(Value::from_float(program.const_f[in.imm]));
                  break;
                case Op::PushI:
                  push(Value::from_int(program.const_i[in.imm]));
                  break;
                case Op::PushV:
                  push(Value::from_vec(program.const_v[in.imm]));
                  break;
                case Op::PushB:
                  push(Value::from_bool(in.imm != 0));
                  break;
                case Op::PushS:
                  push(Value::from_str_i(in.imm));
                  break;
                case Op::PushLocal:
                  push((in.imm >= 0 && in.imm < local_n) ? locals_s[in.imm] :
                                                           Value::from_float(0.0f));
                  break;
                case Op::PushLocalIndex:
                  push(load_local_index(program, in, locals_s, local_n, &local));
                  break;
                case Op::PushLocalMember:
                  if (in.imm >= 0 && in.imm < local_n) {
                    push(apply_get_member(locals_s[in.imm], int(in.a)));
                  }
                  else {
                    push(Value::from_float(0.0f));
                  }
                  break;
                case Op::StoreLocal: {
                  Value v = pop();
                  if (in.imm >= 0 && in.imm < local_n) {
                    locals_s[in.imm] = v;
                  }
                  break;
                }
                case Op::IncLocal:
                  if (in.imm >= 0 && in.imm < local_n) {
                    locals_s[in.imm].type = Type::Int;
                    locals_s[in.imm].i += 1;
                    locals_s[in.imm].v.x = float(locals_s[in.imm].i);
                  }
                  break;
                case Op::DecLocal:
                  if (in.imm >= 0 && in.imm < local_n) {
                    locals_s[in.imm].type = Type::Int;
                    locals_s[in.imm].i -= 1;
                    locals_s[in.imm].v.x = float(locals_s[in.imm].i);
                  }
                  break;
                case Op::Dup:
                  if (sp > 0 && sp < ST) {
                    stack[sp] = stack[sp - 1];
                    sp++;
                  }
                  break;
                case Op::Pop:
                  if (sp > 0) {
                    sp--;
                  }
                  break;
                case Op::PushAttr:
                  if (in.imm >= 0 && in.imm < local.attrs.size()) {
                    push(load_attr_value(local.attrs[in.imm], local.index, local));
                  }
                  else {
                    push(Value::from_float(0.0f));
                  }
                  break;
                case Op::StoreAttr: {
                  Value v = pop();
                  if (in.imm >= 0 && in.imm < local.attrs.size()) {
                    store_attr_value(local.attrs[in.imm], local.index, v, local);
                  }
                  break;
                }
                case Op::AttrAdd: {
                  Value v = pop();
                  if (in.imm >= 0 && in.imm < local.attrs.size()) {
                    attr_add_inplace(local.attrs[in.imm], local.index, v);
                  }
                  break;
                }
                case Op::AttrMulAdd: {
                  Value k = pop();
                  Value b = pop();
                  if (in.imm >= 0 && in.imm < local.attrs.size()) {
                    attr_add_inplace(
                        local.attrs[in.imm], local.index, scale_value(b, k.as_int()));
                  }
                  break;
                }
                case Op::PushIndex:
                  push(Value::from_int(local.index));
                  break;
                case Op::PushNpoints:
                  push(Value::from_int(local.npoints));
                  break;
                case Op::PushNedges:
                  push(Value::from_int(local.nedges));
                  break;
                case Op::PushNfaces:
                  push(Value::from_int(local.nfaces));
                  break;
                case Op::PushNcorners:
                  push(Value::from_int(local.ncorners));
                  break;
                case Op::PushHitPos:
                  push(Value::from_vec(local.hit_pos));
                  break;
                case Op::Add:
                  if (sp >= 2) {
                    Value b = pop();
                    add_inplace(stack[sp - 1], b);
                  }
                  break;
                case Op::Sub: {
                  Value b = pop();
                  Value a = pop();
                  push(numeric_sub(a, b));
                  break;
                }
                case Op::Mul: {
                  Value b = pop();
                  Value a = pop();
                  push(numeric_mul(a, b, nullptr));
                  break;
                }
                case Op::Div: {
                  Value b = pop();
                  Value a = pop();
                  push(numeric_div(a, b));
                  break;
                }
                case Op::AddFF: {
                  const Value b = pop(), a = pop();
                  push(Value::from_float(a.v.x + b.v.x));
                  break;
                }
                case Op::AddVV: {
                  const Value b = pop(), a = pop();
                  push(Value::from_vec(float3(a.v.x + b.v.x, a.v.y + b.v.y, a.v.z + b.v.z)));
                  break;
                }
                case Op::SubFF: {
                  const Value b = pop(), a = pop();
                  push(Value::from_float(a.v.x - b.v.x));
                  break;
                }
                case Op::SubVV: {
                  const Value b = pop(), a = pop();
                  push(Value::from_vec(float3(a.v.x - b.v.x, a.v.y - b.v.y, a.v.z - b.v.z)));
                  break;
                }
                case Op::MulFF: {
                  const Value b = pop(), a = pop();
                  push(Value::from_float(a.v.x * b.v.x));
                  break;
                }
                case Op::MulFV: {
                  const Value b = pop(), a = pop();
                  push(Value::from_vec(float3(a.v.x * b.v.x, a.v.x * b.v.y, a.v.x * b.v.z)));
                  break;
                }
                case Op::MulVF: {
                  const Value b = pop(), a = pop();
                  push(Value::from_vec(float3(a.v.x * b.v.x, a.v.y * b.v.x, a.v.z * b.v.x)));
                  break;
                }
                case Op::MulVV: {
                  const Value b = pop(), a = pop();
                  push(Value::from_vec(float3(a.v.x * b.v.x, a.v.y * b.v.y, a.v.z * b.v.z)));
                  break;
                }
                case Op::DivFF: {
                  const Value b = pop(), a = pop();
                  push(Value::from_float(b.v.x != 0.0f ? a.v.x / b.v.x : 0.0f));
                  break;
                }
                case Op::DivVF: {
                  const Value b = pop(), a = pop();
                  const float inv = b.v.x != 0.0f ? 1.0f / b.v.x : 0.0f;
                  push(Value::from_vec(float3(a.v.x * inv, a.v.y * inv, a.v.z * inv)));
                  break;
                }
                case Op::DivVV: {
                  const Value b = pop(), a = pop();
                  push(Value::from_vec(float3(a.v.x / b.v.x, a.v.y / b.v.y, a.v.z / b.v.z)));
                  break;
                }
                case Op::Mod: {
                  Value b = pop();
                  Value a = pop();
                  if (a.type == Type::Int && b.type == Type::Int) {
                    const int d = b.as_int();
                    push(Value::from_int(d != 0 ? a.as_int() % d : 0));
                  }
                  else {
                    const float d = b.as_float();
                    push(Value::from_float(d != 0.0f ? fmodf(a.as_float(), d) : 0.0f));
                  }
                  break;
                }
                case Op::Neg: {
                  Value a = pop();
                  if (a.type == Type::Vector4) {
                    push(Value::from_vec4(-a.as_vec4(), Type::Vector4));
                  }
                  else if (a.type == Type::Vector) {
                    push(Value::from_vec(-a.as_vec()));
                  }
                  else if (a.type == Type::Vector2) {
                    push(Value::from_vec2(-a.as_vec2()));
                  }
                  else if (a.type == Type::Matrix) {
                    push(Value::from_matrix(-value_as_matrix(a)));
                  }
                  else if (a.type == Type::Matrix3) {
                    push(Value::from_matrix3(-value_as_matrix3(a)));
                  }
                  else if (a.type == Type::Matrix2) {
                    push(Value::from_matrix2(-a.as_matrix2()));
                  }
                  else if (a.type == Type::Int) {
                    push(Value::from_int(-a.as_int()));
                  }
                  else {
                    push(Value::from_float(-a.as_float()));
                  }
                  break;
                }
                case Op::Lt: {
                  Value b = pop();
                  Value a = pop();
                  push(Value::from_bool(a.as_float() < b.as_float()));
                  break;
                }
                case Op::Le: {
                  Value b = pop();
                  Value a = pop();
                  push(Value::from_bool(a.as_float() <= b.as_float()));
                  break;
                }
                case Op::Gt: {
                  Value b = pop();
                  Value a = pop();
                  push(Value::from_bool(a.as_float() > b.as_float()));
                  break;
                }
                case Op::Ge: {
                  Value b = pop();
                  Value a = pop();
                  push(Value::from_bool(a.as_float() >= b.as_float()));
                  break;
                }
                case Op::Eq: {
                  Value b = pop();
                  Value a = pop();
                  push(Value::from_bool(a.as_float() == b.as_float()));
                  break;
                }
                case Op::Ne: {
                  Value b = pop();
                  Value a = pop();
                  push(Value::from_bool(a.as_float() != b.as_float()));
                  break;
                }
                case Op::And: {
                  Value b = pop();
                  Value a = pop();
                  push(Value::from_bool(a.as_bool() && b.as_bool()));
                  break;
                }
                case Op::Or: {
                  Value b = pop();
                  Value a = pop();
                  push(Value::from_bool(a.as_bool() || b.as_bool()));
                  break;
                }
                case Op::Not:
                  push(Value::from_bool(!pop().as_bool()));
                  break;
                case Op::Jmp:
                  if (++jumps > 100000000) {
                    err = "Instruction limit exceeded (infinite loop?)";
                    lane_ok = false;
                    break;
                  }
                  pc = in.imm;
                  continue;
                case Op::JmpIfFalse:
                  if (++jumps > 100000000) {
                    err = "Instruction limit exceeded (infinite loop?)";
                    lane_ok = false;
                    break;
                  }
                  if (!pop().as_bool()) {
                    pc = in.imm;
                    continue;
                  }
                  break;
                case Op::JmpIfTrue:
                  if (++jumps > 100000000) {
                    err = "Instruction limit exceeded (infinite loop?)";
                    lane_ok = false;
                    break;
                  }
                  if (pop().as_bool()) {
                    pc = in.imm;
                    continue;
                  }
                  break;
                case Op::LengthV3:
                  push(typed_length_v3(pop()));
                  break;
                case Op::DotV3: {
                  const Value b = pop();
                  const Value a = pop();
                  push(typed_dot_v3(a, b));
                  break;
                }
                case Op::CrossV3: {
                  const Value b = pop();
                  const Value a = pop();
                  push(typed_cross_v3(a, b));
                  break;
                }
                case Op::SqrtF:
                  push(typed_sqrt_f(pop()));
                  break;
                case Op::LogF:
                  push(typed_log_f(pop()));
                  break;
                case Op::AbsF:
                  push(typed_abs_f(pop()));
                  break;
                case Op::ArrayLen:
                  push(typed_array_len(pop()));
                  break;
                case Op::Call: {
                  const int nargs = int(in.a);
                  Value args_buf[32];
                  const int na = std::min(nargs, 32);
                  for (int a = na - 1; a >= 0; a--) {
                    args_buf[a] = pop();
                  }
                  Value result = call_builtin(
                      Builtin(in.imm), Span<Value>(args_buf, na), local, err);
                  if (!err.empty()) {
                    lane_ok = false;
                    break;
                  }
                  push(result);
                  break;
                }
                case Op::SampleElem: {
                  const int nargs = int(in.a & 0x1f);
                  Value args_buf[32];
                  const int na = std::min(nargs, 32);
                  for (int a = na - 1; a >= 0; a--) {
                    args_buf[a] = pop();
                  }
                  push(call_static_sample(in, Span<Value>(args_buf, na), local, err));
                  if (!err.empty()) {
                    lane_ok = false;
                  }
                  break;
                }
                case Op::FaceCorners: {
                  const int nargs = int(in.a);
                  Value args_buf[2];
                  const int na = std::min(nargs, 2);
                  for (int a = na - 1; a >= 0; a--) {
                    args_buf[a] = pop();
                  }
                  push(face_corners_direct(in, Span<Value>(args_buf, na), local));
                  break;
                }
                case Op::GatherSamples:
                  if (!gather_samples_direct(
                          program, in, locals_s, local_n, local, err))
                  {
                    lane_ok = false;
                  }
                  break;
                case Op::CallUser: {
                  if (in.imm < 0 || in.imm >= int(program.user_fns.size())) {
                    lane_ok = false;
                    err = "无效的函数调用";
                    break;
                  }
                  const UserFn &fn = program.user_fns[in.imm];
                  const int nargs = int(in.a);
                  const int fn_n = std::max(fn.local_count, std::max(nargs, 1));
                  Vector<Value> fn_locals(fn_n);
                  for (int i = nargs - 1; i >= 0; i--) {
                    fn_locals[i] = pop();
                  }
                  Value ret = Value::from_int(0);
                  if (!vm_interp(program, local, err, &ret, fn.entry, fn_locals.data(), fn_n, 1))
                  {
                    lane_ok = false;
                    break;
                  }
                  push(ret);
                  break;
                }
                case Op::Pow: {
                  const Value b = pop();
                  const Value a = pop();
                  push(Value::from_float(powf(a.as_float(), b.as_float())));
                  break;
                }
                case Op::GetMember: {
                  push(apply_get_member(pop(), in.imm));
                  break;
                }
                case Op::SetMember: {
                  const Value obj = pop();
                  const Value mem = pop();
                  push(apply_set_member(obj, mem, in.imm));
                  break;
                }
                case Op::GetIndex: {
                  const Value ix = pop();
                  const Value arr = pop();
                  push(get_index_value(arr, ix.as_int()));
                  break;
                }
                case Op::SetIndex: {
                  const Value ix = pop();
                  const Value arr = pop();
                  const Value elem = pop();
                  push(set_index_value(arr, ix.as_int(), elem, local));
                  break;
                }
                case Op::MakeVec: {
                  const int cn = in.imm;
                  float w = 0.0f;
                  float z = 0.0f;
                  float y = 0.0f;
                  float x = 0.0f;
                  if (cn >= 4) {
                    w = pop().as_float();
                  }
                  if (cn >= 3) {
                    z = pop().as_float();
                  }
                  if (cn >= 2) {
                    y = pop().as_float();
                  }
                  if (cn >= 1) {
                    x = pop().as_float();
                  }
                  push(make_vec_value(cn, x, y, z, w));
                  break;
                }
                case Op::WhileCmpAdd:
                  if (in.imm >= 0 && in.imm < program.while_adds.size() &&
                      program.while_adds[in.imm].attr >= 0 &&
                      program.while_adds[in.imm].attr < local.attrs.size())
                  {
                    while_cmp_add_one(local.attrs, program.while_adds[in.imm], local.index);
                  }
                  break;
                case Op::Return:
                  pc = code_n;
                  continue;
                default:
                  break;
              }
              pc++;
            }
            if (!lane_ok) {
              bool expected = true;
              if (ok.compare_exchange_strong(expected, false, std::memory_order_relaxed)) {
                std::lock_guard lock(err_mutex);
                r_error = err;
              }
              return;
            }
          }
          off += n;
        }
      },
      /* Interpreted VEX does enough work per element that 4096 left small/medium meshes on only
       * one or two workers. Keep chunks large enough to amortize scheduling while exposing useful
       * CPU parallelism for loop/array/spatial-heavy wrangles. */
      exec_mode::grain_size(lane_grain));
  return ok.load();
}

static bool vm_run_tiles(const Program &program,
                         VMEnv &env,
                         const IndexMask &mask,
                         std::string &r_error)
{
  /* Keep this tier opt-in until array-heavy workloads consistently beat the fixed scalar lane
   * executor. The implementation is retained for profiling and further SIMD work, but a new tier
   * must not become the default on the strength of instruction-dispatch theory alone. */
  const bool wavefront = std::getenv("BLENDER_VEX_ENABLE_WAVEFRONT") != nullptr &&
                         program_can_wavefront_exec(program);
  if (std::getenv("BLENDER_VEX_PROFILE") != nullptr) {
    std::fprintf(stderr,
                 "VEX_VM_EXEC path=%s code=%lld locals=%d stack=%d gathers=%lld\n",
                 wavefront ? "wavefront" : "scalar",
                 static_cast<long long>(program.code.size()),
                 program.local_count,
                 program.max_stack,
                 static_cast<long long>(program.gather_samples.size()));
  }
  if (wavefront)
  {
    return vm_run_wavefront(program, env, mask, r_error);
  }
  return vm_run_tiles_scalar(program, env, mask, r_error);
}

bool vm_run_lanes(const Program &program,
                  VMEnv &env,
                  const IndexMask &mask,
                  std::string &r_error)
{
  /* The fixed-storage lane interpreter now handles structured jumps as well. It avoids rebuilding
   * the general recursive VM stack and call-frame vectors for every geometry element, while still
   * preserving independent locals, temporary arrays and instruction limits per lane. */
  if (program.max_stack <= 48 && program.local_count <= 48) {
    return vm_run_tiles(program, env, mask, r_error);
  }
  std::atomic<bool> ok{true};
  std::mutex err_mutex;
  const int lane_grain = vm_lane_grain(program.code.size());
  mask.foreach_segment(
      [&](const index_mask::IndexMaskSegment segment) {
        if (!ok.load(std::memory_order_relaxed)) {
          return;
        }
        VMEnv local = env;
        std::string err;
        for (const int64_t i : segment) {
          if (!ok.load(std::memory_order_relaxed)) {
            return;
          }
          local.index = int(i);
          err.clear();
          if (!vm_run(program, local, err, nullptr)) {
            bool expected = true;
            if (ok.compare_exchange_strong(expected, false, std::memory_order_relaxed)) {
              std::lock_guard lock(err_mutex);
              r_error = err;
            }
            return;
          }
        }
      },
      exec_mode::grain_size(lane_grain));
  return ok.load();
}

/**
 * `m@m = {…}; f[]@fa = {…, f@f}; m[]@ma = {m@m, m@m};` is not array-exec
 * (PushAttr), but the matrix stores are uniform. Broadcast those once and Nop
 * them out of the per-point path so we don't rebuild 4×mat4 on every index.
 */
static bool try_broadcast_uniform_stores(const Program &program,
                                  VMEnv &env,
                                  const IndexMask &mask,
                                  std::string &r_error,
                                  Vector<Inst> &r_lane_code,
                                  int &r_varying_stores)
{
  r_varying_stores = 0;
  r_lane_code.clear();
  if (program_has_jumps(program)) {
    return false;
  }
  const Span<Inst> code = program.code;
  const int code_n = int(code.size());
  if (code_n <= 0) {
    return true;
  }

  struct Slot {
    Value v;
    int src = -1;
    bool varying = false;
  };

  tls_arrays_begin();
  constexpr int stack_cap = 64;
  Slot stack[stack_cap];
  int sp = 0;
  const int local_n = std::max(program.local_count, 1);
  Array<Slot> locals(local_n);
  Array<uint8_t> feeds(code_n, 0);
  Array<uint8_t> must_run(code_n, 0);
  Array<uint8_t> pushlocal_uniform(code_n, 0);
  Array<Vector<int>> ins(code_n);

  auto push = [&](const Slot &s) {
    if (sp < stack_cap) {
      stack[sp++] = s;
    }
  };
  auto pop = [&]() -> Slot {
    if (sp <= 0) {
      return Slot{};
    }
    return stack[--sp];
  };
  auto mark_feed = [&](const Slot &s) {
    if (s.src >= 0 && s.src < code_n) {
      feeds[s.src] = 1;
    }
  };
  auto add_in = [&](const int pc, const Slot &s) {
    if (s.src >= 0) {
      ins[pc].append(s.src);
    }
  };

  env.const_s = program.const_s.as_span();
  env.index = 0;

  for (int pc = 0; pc < code_n; pc++) {
    const Inst &in = code[pc];
    switch (in.op) {
      case Op::Nop:
        break;
      case Op::PushF:
        if (in.imm < 0 || in.imm >= program.const_f.size()) {
          return false;
        }
        push({Value::from_float(program.const_f[in.imm]), pc, false});
        break;
      case Op::PushI:
        if (in.imm < 0 || in.imm >= program.const_i.size()) {
          return false;
        }
        push({Value::from_int(program.const_i[in.imm]), pc, false});
        break;
      case Op::PushV:
        if (in.imm < 0 || in.imm >= program.const_v.size()) {
          return false;
        }
        push({Value::from_vec(program.const_v[in.imm]), pc, false});
        break;
      case Op::PushB:
        push({Value::from_bool(in.imm != 0), pc, false});
        break;
      case Op::PushS:
        push({Value::from_str_i(in.imm), pc, false});
        break;
      case Op::PushLocal:
        if (in.imm >= 0 && in.imm < local_n) {
          Slot s = locals[in.imm];
          if (!s.varying) {
            pushlocal_uniform[pc] = 1;
          }
          add_in(pc, s);
          s.src = pc;
          push(s);
        }
        else {
          push({Value::from_float(0.0f), pc, false});
        }
        break;
      case Op::StoreLocal: {
        Slot s = pop();
        add_in(pc, s);
        if (in.imm >= 0 && in.imm < local_n) {
          locals[in.imm] = s;
          locals[in.imm].src = pc;
        }
        break;
      }
      case Op::Dup:
        if (sp > 0 && sp < stack_cap) {
          Slot s = stack[sp - 1];
          add_in(pc, s);
          s.src = pc;
          stack[sp++] = s;
        }
        break;
      case Op::Pop:
        pop();
        break;
      case Op::PushNpoints:
        push({Value::from_int(env.npoints), pc, false});
        break;
      case Op::PushNedges:
        push({Value::from_int(env.nedges), pc, false});
        break;
      case Op::PushNfaces:
        push({Value::from_int(env.nfaces), pc, false});
        break;
      case Op::PushNcorners:
        push({Value::from_int(env.ncorners), pc, false});
        break;
      case Op::PushAttr:
      case Op::PushIndex:
      case Op::PushHitPos:
        push({Value::from_float(0.0f), pc, true});
        must_run[pc] = 1;
        break;
      case Op::StoreAttr: {
        Slot s = pop();
        add_in(pc, s);
        if (s.varying) {
          r_varying_stores++;
          must_run[pc] = 1;
          mark_feed(s);
        }
        else if (in.imm >= 0 && in.imm < env.attrs.size()) {
          attr_store_mask(env.attrs[in.imm], s.v, mask, env);
        }
        break;
      }
      case Op::LengthV3:
      case Op::SqrtF:
      case Op::LogF:
      case Op::AbsF:
      case Op::ArrayLen: {
        Slot a = pop();
        add_in(pc, a);
        if (a.varying) {
          must_run[pc] = 1;
          mark_feed(a);
          const Value zero = in.op == Op::ArrayLen ? Value::from_int(0) :
                                                     Value::from_float(0.0f);
          push({zero, pc, true});
        }
        else {
          Value result;
          switch (in.op) {
            case Op::LengthV3:
              result = typed_length_v3(a.v);
              break;
            case Op::SqrtF:
              result = typed_sqrt_f(a.v);
              break;
            case Op::LogF:
              result = typed_log_f(a.v);
              break;
            case Op::AbsF:
              result = typed_abs_f(a.v);
              break;
            case Op::ArrayLen:
              result = typed_array_len(a.v);
              break;
            default:
              BLI_assert_unreachable();
              result = Value::from_float(0.0f);
              break;
          }
          push({result, pc, false});
        }
        break;
      }
      case Op::DotV3:
      case Op::CrossV3: {
        Slot b = pop();
        Slot a = pop();
        add_in(pc, a);
        add_in(pc, b);
        if (a.varying || b.varying) {
          must_run[pc] = 1;
          mark_feed(a);
          mark_feed(b);
          const Value zero = in.op == Op::CrossV3 ? Value::from_vec(float3(0.0f)) :
                                                    Value::from_float(0.0f);
          push({zero, pc, true});
        }
        else {
          const Value result = in.op == Op::CrossV3 ? typed_cross_v3(a.v, b.v) :
                                                      typed_dot_v3(a.v, b.v);
          push({result, pc, false});
        }
        break;
      }
      case Op::Call: {
        const int nargs = int(in.a);
        if (nargs < 0 || nargs > 32) {
          return false;
        }
        Slot args_s[32];
        Value args_v[32];
        bool any_var = false;
        for (int i = nargs - 1; i >= 0; i--) {
          args_s[i] = pop();
          add_in(pc, args_s[i]);
          args_v[i] = args_s[i].v;
          any_var = any_var || args_s[i].varying;
        }
        if (any_var || !builtin_is_uniform_value(Builtin(in.imm))) {
          must_run[pc] = 1;
          for (int i = 0; i < nargs; i++) {
            mark_feed(args_s[i]);
          }
          push({Value::from_float(0.0f), pc, true});
          break;
        }
        Value result = call_builtin(
            Builtin(in.imm), Span<Value>(args_v, nargs), env, r_error);
        if (!r_error.empty()) {
          return false;
        }
        push({result, pc, false});
        break;
      }
      case Op::SampleElem: {
        const int nargs = int(in.a & 0x1f);
        if (nargs < 0 || nargs > 32) {
          return false;
        }
        Slot args_s[32];
        for (int i = nargs - 1; i >= 0; i--) {
          args_s[i] = pop();
          add_in(pc, args_s[i]);
          mark_feed(args_s[i]);
        }
        must_run[pc] = 1;
        push({Value::from_float(0.0f), pc, true});
        break;
      }
      case Op::FaceCorners: {
        const int nargs = int(in.a);
        if (nargs < 0 || nargs > 2) {
          return false;
        }
        Slot args_s[2];
        for (int i = nargs - 1; i >= 0; i--) {
          args_s[i] = pop();
          add_in(pc, args_s[i]);
          mark_feed(args_s[i]);
        }
        must_run[pc] = 1;
        push({Value::from_float(0.0f), pc, true});
        break;
      }
      case Op::GatherSamples:
        /* The gather mutates local arrays and depends on the current lane. Keep the full program;
         * the fixed-storage lane interpreter executes it directly. */
        return false;
      case Op::PushLocalIndex:
      case Op::PushLocalMember:
        /* These compact loads can depend on varying locals. The fixed-storage interpreter handles
         * them directly; do not attempt the uniform/varying split here. */
        return false;
      case Op::MakeVec: {
        const int n = std::max(int(in.imm), 0);
        Slot args_s[4];
        bool any_var = false;
        for (int i = n - 1; i >= 0 && i < 4; i--) {
          args_s[i] = pop();
          add_in(pc, args_s[i]);
          any_var = any_var || args_s[i].varying;
        }
        if (any_var) {
          must_run[pc] = 1;
          for (int i = 0; i < n && i < 4; i++) {
            mark_feed(args_s[i]);
          }
          push({Value::from_vec(float3(0.0f)), pc, true});
          break;
        }
        const float x = (n >= 1) ? args_s[0].v.as_float() : 0.0f;
        const float y = (n >= 2) ? args_s[1].v.as_float() : 0.0f;
        const float z = (n >= 3) ? args_s[2].v.as_float() : 0.0f;
        const float w = (n >= 4) ? args_s[3].v.as_float() : 0.0f;
        push({make_vec_value(n, x, y, z, w), pc, false});
        break;
      }
      case Op::IncLocal:
      case Op::DecLocal:
        if (in.imm >= 0 && in.imm < local_n && locals[in.imm].varying) {
          must_run[pc] = 1;
        }
        if (in.imm >= 0 && in.imm < local_n && !locals[in.imm].varying) {
          locals[in.imm].v.type = Type::Int;
          locals[in.imm].v.i += (in.op == Op::IncLocal) ? 1 : -1;
          locals[in.imm].v.v.x = float(locals[in.imm].v.i);
        }
        break;
      case Op::Return:
        break;
      default:
        return false;
    }
  }

  bool more = true;
  while (more) {
    more = false;
    for (int pc = 0; pc < code_n; pc++) {
      if (!must_run[pc] && !feeds[pc]) {
        continue;
      }
      for (const int src : ins[pc]) {
        if (src >= 0 && src < code_n && !feeds[src]) {
          feeds[src] = 1;
          more = true;
        }
      }
    }
  }
  for (int pc = 0; pc < code_n; pc++) {
    if (pushlocal_uniform[pc] && (must_run[pc] || feeds[pc])) {
      /* Varying code reads a uniform local — keep the original program. */
      return false;
    }
  }

  r_lane_code = program.code;
  for (int pc = 0; pc < code_n; pc++) {
    if (must_run[pc] || feeds[pc]) {
      continue;
    }
    r_lane_code[pc].op = Op::Nop;
  }
  return true;
}

bool vm_run_array(const Program &program,
                  VMEnv &env,
                  const IndexMask &mask,
                  std::string &r_error)
{
  if (!program_can_array_exec(program)) {
    if (!program_has_jumps(program)) {
      Vector<Inst> lane_code;
      int varying_stores = 0;
      std::string uniform_err;
      if (try_broadcast_uniform_stores(
              program, env, mask, uniform_err, lane_code, varying_stores) &&
          uniform_err.empty())
      {
        /* Uniform StoreAttr was applied during broadcast and Nop'd out. Remaining
         * ops include per-lane Calls (addpoint / delete_geometry / addprim). Skipping
         * when varying_stores==0 dropped those side effects unless the script had `if`. */
        bool any_op = false;
        for (const Inst &in : lane_code) {
          if (in.op != Op::Nop) {
            any_op = true;
            break;
          }
        }
        if (!any_op) {
          return true;
        }
        Program stripped = program;
        stripped.code = std::move(lane_code);
        return vm_run_lanes(stripped, env, mask, r_error);
      }
    }
    return vm_run_lanes(program, env, mask, r_error);
  }
  env.const_s = program.const_s.as_span();
  tls_arrays_begin();

  constexpr int stack_cap = 64;
  Value stack_storage[stack_cap];
  int sp = 0;
  const int local_n = std::max(program.local_count, 1);
  Value local_storage[stack_cap];
  Vector<Value> local_heap;
  Value *locals;
  if (local_n <= stack_cap) {
    locals = local_storage;
    for (int i = 0; i < local_n; i++) {
      locals[i] = Value{};
    }
  }
  else {
    local_heap.resize(local_n);
    locals = local_heap.data();
  }

  auto push = [&](const Value &v) {
    if (sp < stack_cap) {
      stack_storage[sp++] = v;
    }
  };
  auto pop = [&]() -> Value {
    if (sp <= 0) {
      return Value::from_float(0.0f);
    }
    return stack_storage[--sp];
  };

  constexpr int64_t max_jumps = 100000000;
  int64_t jumps = 0;
  const Span<Inst> code = program.code;
  const int code_n = int(code.size());

  for (int pc = 0; pc < code_n;) {
    CountedAttrLoop loop;
    if (match_counted_attr_loop(program, pc, loop)) {
      const int i0 = (loop.i_slot >= 0 && loop.i_slot < local_n) ? locals[loop.i_slot].as_int() :
                                                                  0;
      const int remain = loop.trip_count - i0;
      if (remain > 0) {
        for (const CountedAttrLoop::Add &add : loop.adds) {
          if (add.attr >= 0 && add.attr < env.attrs.size()) {
            attr_add_mask(env.attrs[add.attr],
                          scale_value(counted_loop_addend(program, add, locals, local_n), remain),
                          mask);
          }
        }
      }
      if (loop.i_slot >= 0 && loop.i_slot < local_n) {
        locals[loop.i_slot] = Value::from_int(loop.trip_count);
      }
      pc = loop.end_pc;
      continue;
    }

    const Inst &in = code[pc];
    switch (in.op) {
      case Op::Nop:
        break;
      case Op::PushF:
        push(Value::from_float(program.const_f[in.imm]));
        break;
      case Op::PushI:
        push(Value::from_int(program.const_i[in.imm]));
        break;
      case Op::PushV:
        push(Value::from_vec(program.const_v[in.imm]));
        break;
      case Op::PushB:
        push(Value::from_bool(in.imm != 0));
        break;
      case Op::PushS:
        push(Value::from_str_i(in.imm));
        break;
      case Op::PushLocal:
        if (in.imm >= 0 && in.imm < local_n) {
          push(locals[in.imm]);
        }
        else {
          push(Value::from_float(0.0f));
        }
        break;
      case Op::PushLocalIndex:
        push(load_local_index(program, in, locals, local_n, &env));
        break;
      case Op::PushLocalMember:
        if (in.imm >= 0 && in.imm < local_n) {
          push(apply_get_member(locals[in.imm], int(in.a)));
        }
        else {
          push(Value::from_float(0.0f));
        }
        break;
      case Op::StoreLocal: {
        Value v = pop();
        if (in.imm >= 0 && in.imm < local_n) {
          locals[in.imm] = v;
        }
        break;
      }
      case Op::IncLocal:
        if (in.imm >= 0 && in.imm < local_n) {
          locals[in.imm].type = Type::Int;
          locals[in.imm].i += 1;
          locals[in.imm].v.x = float(locals[in.imm].i);
        }
        break;
      case Op::DecLocal:
        if (in.imm >= 0 && in.imm < local_n) {
          locals[in.imm].type = Type::Int;
          locals[in.imm].i -= 1;
          locals[in.imm].v.x = float(locals[in.imm].i);
        }
        break;
      case Op::Dup:
        if (sp > 0 && sp < stack_cap) {
          stack_storage[sp] = stack_storage[sp - 1];
          sp++;
        }
        break;
      case Op::PushHitPos:
        push(Value::from_vec(env.hit_pos));
        break;
      case Op::Pop:
        if (sp > 0) {
          sp--;
        }
        break;
      case Op::StoreAttr: {
        Value v = pop();
        if (in.imm >= 0 && in.imm < env.attrs.size()) {
          attr_store_mask(env.attrs[in.imm], v, mask, env);
        }
        break;
      }
      case Op::AttrAdd: {
        Value v = pop();
        if (in.imm >= 0 && in.imm < env.attrs.size()) {
          attr_add_mask(env.attrs[in.imm], v, mask);
        }
        break;
      }
      case Op::AttrMulAdd: {
        Value k = pop();
        Value b = pop();
        if (in.imm >= 0 && in.imm < env.attrs.size()) {
          attr_add_mask(env.attrs[in.imm], scale_value(b, k.as_int()), mask);
        }
        break;
      }
      case Op::PushNpoints:
        push(Value::from_int(env.npoints));
        break;
      case Op::PushNedges:
        push(Value::from_int(env.nedges));
        break;
      case Op::PushNfaces:
        push(Value::from_int(env.nfaces));
        break;
      case Op::PushNcorners:
        push(Value::from_int(env.ncorners));
        break;
      case Op::Add: {
        if (sp >= 2) {
          Value b = pop();
          add_inplace(stack_storage[sp - 1], b);
        }
        break;
      }
      case Op::Sub: {
        Value b = pop();
        Value a = pop();
        push(numeric_sub(a, b));
        break;
      }
      case Op::Mul: {
        Value b = pop();
        Value a = pop();
        push(numeric_mul(a, b, nullptr));
        break;
      }
      case Op::Div: {
        Value b = pop();
        Value a = pop();
        push(numeric_div(a, b));
        break;
      }
      case Op::AddFF: {
        const Value b = pop(), a = pop();
        push(Value::from_float(a.v.x + b.v.x));
        break;
      }
      case Op::AddVV: {
        const Value b = pop(), a = pop();
        push(Value::from_vec(float3(a.v.x + b.v.x, a.v.y + b.v.y, a.v.z + b.v.z)));
        break;
      }
      case Op::SubFF: {
        const Value b = pop(), a = pop();
        push(Value::from_float(a.v.x - b.v.x));
        break;
      }
      case Op::SubVV: {
        const Value b = pop(), a = pop();
        push(Value::from_vec(float3(a.v.x - b.v.x, a.v.y - b.v.y, a.v.z - b.v.z)));
        break;
      }
      case Op::MulFF: {
        const Value b = pop(), a = pop();
        push(Value::from_float(a.v.x * b.v.x));
        break;
      }
      case Op::MulFV: {
        const Value b = pop(), a = pop();
        push(Value::from_vec(float3(a.v.x * b.v.x, a.v.x * b.v.y, a.v.x * b.v.z)));
        break;
      }
      case Op::MulVF: {
        const Value b = pop(), a = pop();
        push(Value::from_vec(float3(a.v.x * b.v.x, a.v.y * b.v.x, a.v.z * b.v.x)));
        break;
      }
      case Op::MulVV: {
        const Value b = pop(), a = pop();
        push(Value::from_vec(float3(a.v.x * b.v.x, a.v.y * b.v.y, a.v.z * b.v.z)));
        break;
      }
      case Op::DivFF: {
        const Value b = pop(), a = pop();
        push(Value::from_float(b.v.x != 0.0f ? a.v.x / b.v.x : 0.0f));
        break;
      }
      case Op::DivVF: {
        const Value b = pop(), a = pop();
        const float inv = b.v.x != 0.0f ? 1.0f / b.v.x : 0.0f;
        push(Value::from_vec(float3(a.v.x * inv, a.v.y * inv, a.v.z * inv)));
        break;
      }
      case Op::DivVV: {
        const Value b = pop(), a = pop();
        push(Value::from_vec(float3(a.v.x / b.v.x, a.v.y / b.v.y, a.v.z / b.v.z)));
        break;
      }
      case Op::Mod: {
        Value b = pop();
        Value a = pop();
        if (a.type == Type::Int && b.type == Type::Int) {
          const int d = b.as_int();
          push(Value::from_int(d != 0 ? a.as_int() % d : 0));
        }
        else {
          const float d = b.as_float();
          push(Value::from_float(d != 0.0f ? fmodf(a.as_float(), d) : 0.0f));
        }
        break;
      }
      case Op::Neg: {
        Value a = pop();
        if (a.type == Type::Vector4) {
          push(Value::from_vec4(-a.as_vec4(), Type::Vector4));
        }
        else if (a.type == Type::Vector) {
          push(Value::from_vec(-a.as_vec()));
        }
        else if (a.type == Type::Vector2) {
          push(Value::from_vec2(-a.as_vec2()));
        }
        else if (a.type == Type::Matrix) {
          push(Value::from_matrix(-value_as_matrix(a)));
        }
        else if (a.type == Type::Matrix3) {
          push(Value::from_matrix3(-value_as_matrix3(a)));
        }
        else if (a.type == Type::Matrix2) {
          push(Value::from_matrix2(-a.as_matrix2()));
        }
        else if (a.type == Type::Int) {
          push(Value::from_int(-a.as_int()));
        }
        else {
          push(Value::from_float(-a.as_float()));
        }
        break;
      }
      case Op::Lt: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_float() < b.as_float()));
        break;
      }
      case Op::Le: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_float() <= b.as_float()));
        break;
      }
      case Op::Gt: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_float() > b.as_float()));
        break;
      }
      case Op::Ge: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_float() >= b.as_float()));
        break;
      }
      case Op::Eq: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_float() == b.as_float()));
        break;
      }
      case Op::Ne: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_float() != b.as_float()));
        break;
      }
      case Op::And: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_bool() && b.as_bool()));
        break;
      }
      case Op::Or: {
        Value b = pop();
        Value a = pop();
        push(Value::from_bool(a.as_bool() || b.as_bool()));
        break;
      }
      case Op::Not:
        push(Value::from_bool(!pop().as_bool()));
        break;
      case Op::Jmp:
        if (++jumps > max_jumps) {
          r_error = "Instruction limit exceeded (infinite loop?)";
          return false;
        }
        pc = in.imm;
        continue;
      case Op::JmpIfFalse:
        if (++jumps > max_jumps) {
          r_error = "Instruction limit exceeded (infinite loop?)";
          return false;
        }
        if (!pop().as_bool()) {
          pc = in.imm;
          continue;
        }
        break;
      case Op::JmpIfTrue:
        if (++jumps > max_jumps) {
          r_error = "Instruction limit exceeded (infinite loop?)";
          return false;
        }
        if (pop().as_bool()) {
          pc = in.imm;
          continue;
        }
        break;
      case Op::GetIndex: {
        const Value idx = pop();
        const Value arr = pop();
        push(get_index_value(arr, idx.as_int()));
        break;
      }
      case Op::SetIndex: {
        const Value idx = pop();
        const Value arr = pop();
        const Value elem = pop();
        push(set_index_value(arr, idx.as_int(), elem, env));
        break;
      }
      case Op::MakeVec: {
        const int n = in.imm;
        float w = 0.0f;
        float z = 0.0f;
        float y = 0.0f;
        float x = 0.0f;
        if (n >= 4) {
          w = pop().as_float();
        }
        if (n >= 3) {
          z = pop().as_float();
        }
        if (n >= 2) {
          y = pop().as_float();
        }
        if (n >= 1) {
          x = pop().as_float();
        }
        push(make_vec_value(n, x, y, z, w));
        break;
      }
      case Op::WhileCmpAdd: {
        if (in.imm >= 0 && in.imm < program.while_adds.size() &&
            program.while_adds[in.imm].attr >= 0 &&
            program.while_adds[in.imm].attr < env.attrs.size())
        {
          while_cmp_add_mask(env.attrs, program.while_adds[in.imm], mask);
        }
        break;
      }
      case Op::LengthV3:
        push(typed_length_v3(pop()));
        break;
      case Op::DotV3: {
        const Value b = pop();
        const Value a = pop();
        push(typed_dot_v3(a, b));
        break;
      }
      case Op::CrossV3: {
        const Value b = pop();
        const Value a = pop();
        push(typed_cross_v3(a, b));
        break;
      }
      case Op::SqrtF:
        push(typed_sqrt_f(pop()));
        break;
      case Op::LogF:
        push(typed_log_f(pop()));
        break;
      case Op::AbsF:
        push(typed_abs_f(pop()));
        break;
      case Op::ArrayLen:
        push(typed_array_len(pop()));
        break;
      case Op::Call: {
        const int nargs = int(in.a);
        Value args_buf[32];
        const int n = std::min(nargs, 32);
        for (int i = n - 1; i >= 0; i--) {
          args_buf[i] = pop();
        }
        Value result = call_builtin(Builtin(in.imm), Span<Value>(args_buf, n), env, r_error);
        if (!r_error.empty()) {
          return false;
        }
        push(result);
        break;
      }
      case Op::SampleElem:
        r_error = "array exec does not support element samples";
        return false;
      case Op::FaceCorners:
        r_error = "array exec does not support topology arrays";
        return false;
      case Op::GatherSamples:
        r_error = "array exec does not support fused sample gathers";
        return false;
      case Op::CallUser:
        r_error = "array exec 不支持用户函数";
        return false;
      case Op::Pow: {
        const Value b = pop();
        const Value a = pop();
        push(Value::from_float(powf(a.as_float(), b.as_float())));
        break;
      }
      case Op::Return:
        return true;
      case Op::PushAttr:
      case Op::PushIndex:
      case Op::GetMember:
      case Op::SetMember:
        r_error = "Script needs per-element evaluation";
        return false;
      default:
        break;
    }
    pc++;
  }
  return true;
}

}  // namespace blender::nodes::vex
