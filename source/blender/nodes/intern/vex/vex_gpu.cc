/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <cstring>
#include <mutex>
#include <string>

#include "BLI_hash.hh"
#include "BLI_utildefines.hh"
#include "BLI_index_mask.hh"
#include "BLI_map.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_threads.hh"
#include "BLI_vector.hh"

#include "BKE_global.hh"
#include "BKE_wrangle_array.hh"

#include "DNA_meshdata_types.h"

#include "GPU_capabilities.hh"
#include "GPU_compute.hh"
#include "GPU_context.hh"
#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_storage_buffer.hh"

#include "gpu_shader_create_info.hh"

#include "DRW_engine.hh"

#include "vex_gpu.hh"

namespace blender::nodes::vex {
namespace {

constexpr int k_arr_ints = 120;
constexpr int k_group = 64;

const char *gpu_arr_struct()
{
  return R"(
struct WrangleArr {
  int count;
  int total;
  int kind;
  int flags;
  int data[120];
};
)";
}

const char *gpu_preamble()
{
  /* In two pieces: a single string literal this long is more than MSVC accepts. */
  static const std::string source = std::string(R"(
uint wr_rot(uint x, uint k)
{
  return (x << k) | (x >> (32u - k));
}

void wr_final(inout uint a, inout uint b, inout uint c)
{
  c ^= b;
  c -= wr_rot(b, 14u);
  a ^= c;
  a -= wr_rot(c, 11u);
  b ^= a;
  b -= wr_rot(a, 25u);
  c ^= b;
  c -= wr_rot(b, 16u);
  a ^= c;
  a -= wr_rot(c, 4u);
  b ^= a;
  b -= wr_rot(a, 14u);
  c ^= b;
  c -= wr_rot(b, 24u);
}

uint wr_hash(uint kx)
{
  uint a = 0xdeadbeefu + 17u;
  uint b = a;
  uint c = a;
  a += kx;
  wr_final(a, b, c);
  return c;
}

uint wr_hash2(uint kx, uint ky)
{
  uint a = 0xdeadbeefu + 21u;
  uint b = a;
  uint c = a;
  b += ky;
  a += kx;
  wr_final(a, b, c);
  return c;
}

uint wr_hash3(uint kx, uint ky, uint kz)
{
  uint a = 0xdeadbeefu + 25u;
  uint b = a;
  uint c = a;
  c += kz;
  b += ky;
  a += kx;
  wr_final(a, b, c);
  return c;
}

float wr_hash_to_float(uint kx)
{
  return float(wr_hash(kx)) / 4294967295.0;
}

float wr_rand0(int index, inout int seq)
{
  uint h = uint(index) ^ (uint(seq) * 747796405u);
  seq += 1;
  return wr_hash_to_float(h);
}

vec3 wr_hash_to_vec3(uint h)
{
  return vec3(float(wr_hash(h)), float(wr_hash2(h, 1u)), float(wr_hash2(h, 2u))) / 4294967295.0;
}

vec3 wr_rand0_vec(int index, inout int seq)
{
  uint h = uint(index) ^ (uint(seq) * 747796405u);
  seq += 1;
  return wr_hash_to_vec3(h);
}

WrangleArr wr_arr_new(int kind)
{
  WrangleArr a;
  a.count = 0;
  a.total = 0;
  a.kind = kind;
  a.flags = 0;
  for (int i = 0; i < 120; i++) {
    a.data[i] = 0;
  }
  return a;
}

/* Negative indices count from the end, like in the interpreter. */
int wr_arr_wrap(int i, int n)
{
  return (i < 0) ? i + n : i;
}

float wr_arr_f(WrangleArr a, int i)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0 || i >= a.count) {
    return 0.0;
  }
  if (a.kind == 2) {
    return intBitsToFloat(a.data[i * 3]);
  }
  return intBitsToFloat(a.data[i]);
}

int wr_arr_i(WrangleArr a, int i)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0 || i >= a.count) {
    return 0;
  }
  return a.data[i];
}

vec3 wr_arr_v(WrangleArr a, int i)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0 || i >= a.count) {
    return vec3(0.0);
  }
  int o = i * 3;
  return vec3(intBitsToFloat(a.data[o]),
              intBitsToFloat(a.data[o + 1]),
              intBitsToFloat(a.data[o + 2]));
}

WrangleArr wr_arr_set_f(WrangleArr a, int i, float v)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0) {
    return a;
  }
  if (i >= 120) {
    wr_ovf[0] = 1;
    return a;
  }
  if (i >= a.count) {
    int n = a.count;
    if (n < 0) {
      n = 0;
    }
    for (int k = n; k < i; k++) {
      a.data[k] = 0;
    }
    a.count = i + 1;
    a.total = a.count;
  }
  a.kind = 1;
  a.data[i] = floatBitsToInt(v);
  return a;
}

WrangleArr wr_arr_set_i(WrangleArr a, int i, int v)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0) {
    return a;
  }
  if (i >= 120) {
    wr_ovf[0] = 1;
    return a;
  }
  if (i >= a.count) {
    int n = a.count;
    if (n < 0) {
      n = 0;
    }
    for (int k = n; k < i; k++) {
      a.data[k] = 0;
    }
    a.count = i + 1;
    a.total = a.count;
  }
  a.kind = 0;
  a.data[i] = v;
  return a;
}

WrangleArr wr_arr_set_v(WrangleArr a, int i, vec3 v)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0) {
    return a;
  }
  if (i >= 40) {
    wr_ovf[0] = 1;
    return a;
  }
  if (i >= a.count) {
    int n = a.count;
    if (n < 0) {
      n = 0;
    }
    for (int k = n; k < i; k++) {
      int o = k * 3;
      a.data[o] = 0;
      a.data[o + 1] = 0;
      a.data[o + 2] = 0;
    }
    a.count = i + 1;
    a.total = a.count;
  }
  a.kind = 2;
  int o = i * 3;
  a.data[o] = floatBitsToInt(v.x);
  a.data[o + 1] = floatBitsToInt(v.y);
  a.data[o + 2] = floatBitsToInt(v.z);
  return a;
}

/* `matrix * number`: a uniform scale of X, Y and Z. */
mat4 wr_scale4(float s)
{
  return mat4(vec4(s, 0.0, 0.0, 0.0),
              vec4(0.0, s, 0.0, 0.0),
              vec4(0.0, 0.0, s, 0.0),
              vec4(0.0, 0.0, 0.0, 1.0));
}

/* Items of other sizes: 2D and 4D vectors, 2x2 / 3x3 / 4x4 matrices (by columns). */

float wr_arr_at(WrangleArr a, int o)
{
  return intBitsToFloat(a.data[o]);
}

/* Past-the-end writes grow the array. The gap is zero, or identity matrices of size `dim`. */
WrangleArr wr_arr_grow(WrangleArr a, int i, int stride, int kind, int dim)
{
  if (i >= a.count) {
    int n = a.count;
    if (n < 0) {
      n = 0;
    }
    for (int k = n; k < i; k++) {
      for (int c = 0; c < stride; c++) {
        int d = (dim > 0) ? dim : 1;
        bool diagonal = (dim > 0) && (c / d == c % d);
        a.data[k * stride + c] = diagonal ? floatBitsToInt(1.0) : 0;
      }
    }
    a.count = i + 1;
    a.total = a.count;
  }
  a.kind = kind;
  return a;
}

vec2 wr_arr_v2(WrangleArr a, int i)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0 || i >= a.count) {
    return vec2(0.0);
  }
  int o = i * 2;
  return vec2(wr_arr_at(a, o), wr_arr_at(a, o + 1));
}

vec4 wr_arr_v4(WrangleArr a, int i)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0 || i >= a.count) {
    return vec4(0.0);
  }
  int o = i * 4;
  return vec4(wr_arr_at(a, o), wr_arr_at(a, o + 1), wr_arr_at(a, o + 2), wr_arr_at(a, o + 3));
}

mat2 wr_arr_m2(WrangleArr a, int i)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0 || i >= a.count) {
    return mat2(1.0);
  }
  int o = i * 4;
  return mat2(wr_arr_at(a, o), wr_arr_at(a, o + 1), wr_arr_at(a, o + 2), wr_arr_at(a, o + 3));
}

mat3 wr_arr_m3(WrangleArr a, int i)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0 || i >= a.count) {
    return mat3(1.0);
  }
  int o = i * 9;
  return mat3(wr_arr_at(a, o), wr_arr_at(a, o + 1), wr_arr_at(a, o + 2),
              wr_arr_at(a, o + 3), wr_arr_at(a, o + 4), wr_arr_at(a, o + 5),
              wr_arr_at(a, o + 6), wr_arr_at(a, o + 7), wr_arr_at(a, o + 8));
}

mat4 wr_arr_m4(WrangleArr a, int i)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0 || i >= a.count) {
    return mat4(1.0);
  }
  int o = i * 16;
  return mat4(wr_arr_at(a, o), wr_arr_at(a, o + 1), wr_arr_at(a, o + 2), wr_arr_at(a, o + 3),
              wr_arr_at(a, o + 4), wr_arr_at(a, o + 5), wr_arr_at(a, o + 6), wr_arr_at(a, o + 7),
              wr_arr_at(a, o + 8), wr_arr_at(a, o + 9), wr_arr_at(a, o + 10), wr_arr_at(a, o + 11),
              wr_arr_at(a, o + 12), wr_arr_at(a, o + 13), wr_arr_at(a, o + 14),
              wr_arr_at(a, o + 15));
}

WrangleArr wr_arr_set_v2(WrangleArr a, int i, vec2 v)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0) {
    return a;
  }
  if (i >= 60) {
    wr_ovf[0] = 1;
    return a;
  }
  a = wr_arr_grow(a, i, 2, 8, 0);
  int o = i * 2;
  a.data[o] = floatBitsToInt(v.x);
  a.data[o + 1] = floatBitsToInt(v.y);
  return a;
}

WrangleArr wr_arr_set_v4(WrangleArr a, int i, vec4 v)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0) {
    return a;
  }
  if (i >= 30) {
    wr_ovf[0] = 1;
    return a;
  }
  a = wr_arr_grow(a, i, 4, 9, 0);
  int o = i * 4;
  a.data[o] = floatBitsToInt(v.x);
  a.data[o + 1] = floatBitsToInt(v.y);
  a.data[o + 2] = floatBitsToInt(v.z);
  a.data[o + 3] = floatBitsToInt(v.w);
  return a;
}

WrangleArr wr_arr_set_m2(WrangleArr a, int i, mat2 m)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0) {
    return a;
  }
  if (i >= 30) {
    wr_ovf[0] = 1;
    return a;
  }
  a = wr_arr_grow(a, i, 4, 5, 2);
  int o = i * 4;
  for (int c = 0; c < 2; c++) {
    for (int r = 0; r < 2; r++) {
      a.data[o + c * 2 + r] = floatBitsToInt(m[c][r]);
    }
  }
  return a;
}

WrangleArr wr_arr_set_m3(WrangleArr a, int i, mat3 m)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0) {
    return a;
  }
  if (i >= 13) {
    wr_ovf[0] = 1;
    return a;
  }
  a = wr_arr_grow(a, i, 9, 6, 3);
  int o = i * 9;
  for (int c = 0; c < 3; c++) {
    for (int r = 0; r < 3; r++) {
      a.data[o + c * 3 + r] = floatBitsToInt(m[c][r]);
    }
  }
  return a;
}

WrangleArr wr_arr_set_m4(WrangleArr a, int i, mat4 m)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0) {
    return a;
  }
  if (i >= 7) {
    wr_ovf[0] = 1;
    return a;
  }
  a = wr_arr_grow(a, i, 16, 4, 4);
  int o = i * 16;
  for (int c = 0; c < 4; c++) {
    for (int r = 0; r < 4; r++) {
      a.data[o + c * 4 + r] = floatBitsToInt(m[c][r]);
    }
  }
  return a;
}

)") + R"(
/* ---- Array functions. An item is `stride` words. `mode` is how two words compare: 0 as ints,
 * 1 as floats, 2 an int item against a float value. An array that needs more than the 120 words
 * sets `wr_ovf`, the interpreter then runs the program instead. */

bool wr_word_eq(int a, int b, int mode)
{
  if (mode == 0) {
    return a == b;
  }
  if (mode == 2) {
    return float(a) == intBitsToFloat(b);
  }
  return intBitsToFloat(a) == intBitsToFloat(b);
}

bool wr_arr_item_eq(WrangleArr a, int ia, WrangleArr b, int ib, int stride, int mode)
{
  for (int c = 0; c < stride; c++) {
    if (!wr_word_eq(a.data[ia * stride + c], b.data[ib * stride + c], mode)) {
      return false;
    }
  }
  return true;
}

/* Does `a` contain item `ib` of `b`? */
bool wr_arr_has(WrangleArr a, WrangleArr b, int ib, int stride, int mode)
{
  for (int i = 0; i < a.count; i++) {
    if (wr_arr_item_eq(a, i, b, ib, stride, mode)) {
      return true;
    }
  }
  return false;
}

WrangleArr wr_arr_i2f(WrangleArr a)
{
  for (int i = 0; i < a.count; i++) {
    a.data[i] = floatBitsToInt(float(a.data[i]));
  }
  a.kind = 1;
  return a;
}

/* `append(a, b)` with a whole array. `conv` 1 turns int items into floats, 2 floats into ints. */
WrangleArr wr_arr_extend(WrangleArr a, WrangleArr b, int stride, int kind, int conv)
{
  a.kind = kind;
  for (int i = 0; i < b.count; i++) {
    if ((a.count + 1) * stride > 120) {
      wr_ovf[0] = 1;
      break;
    }
    for (int c = 0; c < stride; c++) {
      int w = b.data[i * stride + c];
      if (conv == 1) {
        w = floatBitsToInt(float(w));
      }
      else if (conv == 2) {
        w = int(intBitsToFloat(w));
      }
      a.data[a.count * stride + c] = w;
    }
    a.count += 1;
  }
  a.total = a.count;
  return a;
}

int wr_arr_insert_index(int i, int n)
{
  if (n <= 0) {
    return 0;
  }
  if (i < 0) {
    i += n;
  }
  return clamp(i, 0, n);
}

/* Makes room for one item at `i`, the caller writes it. */
WrangleArr wr_arr_insert_gap(WrangleArr a, int i, int stride, int kind)
{
  i = wr_arr_insert_index(i, a.count);
  a.kind = kind;
  if ((a.count + 1) * stride > 120) {
    wr_ovf[0] = 1;
    return a;
  }
  for (int k = a.count * stride - 1; k >= i * stride; k--) {
    a.data[k + stride] = a.data[k];
  }
  a.count += 1;
  a.total = a.count;
  return a;
}

WrangleArr wr_arr_remove_at(WrangleArr a, int i, int stride)
{
  i = wr_arr_wrap(i, a.count);
  if (i < 0 || i >= a.count) {
    return a;
  }
  for (int k = i * stride; k < (a.count - 1) * stride; k++) {
    a.data[k] = a.data[k + stride];
  }
  a.count -= 1;
  a.total = a.count;
  return a;
}

/* `probe` holds the value as its first item. */
WrangleArr wr_arr_remove_value(WrangleArr a, WrangleArr probe, int stride, int mode)
{
  int n = 0;
  for (int i = 0; i < a.count; i++) {
    if (!wr_arr_item_eq(a, i, probe, 0, stride, mode)) {
      for (int c = 0; c < stride; c++) {
        a.data[n * stride + c] = a.data[i * stride + c];
      }
      n += 1;
    }
  }
  a.count = n;
  a.total = n;
  return a;
}

WrangleArr wr_arr_find(WrangleArr a, WrangleArr probe, int stride, int mode)
{
  WrangleArr r = wr_arr_new(0);
  for (int i = 0; i < a.count; i++) {
    if (wr_arr_item_eq(a, i, probe, 0, stride, mode)) {
      r.data[r.count] = i;
      r.count += 1;
    }
  }
  r.total = r.count;
  return r;
}

WrangleArr wr_arr_slice(
    WrangleArr a, int start_arg, int end_arg, bool has_end, int step, int stride)
{
  WrangleArr r = a;
  r.count = 0;
  r.total = 0;
  int size = a.count;
  if (size <= 0 || step == 0) {
    return r;
  }
  if (step > 0) {
    int start = (start_arg < 0) ? max(start_arg + size, 0) : min(start_arg, size);
    int end = !has_end ? size : ((end_arg < 0) ? max(end_arg + size, 0) : min(end_arg, size));
    for (int i = start; i < end; i += step) {
      for (int c = 0; c < stride; c++) {
        r.data[r.count * stride + c] = a.data[i * stride + c];
      }
      r.count += 1;
    }
  }
  else {
    int start = (start_arg < 0) ? max(start_arg + size, -1) : min(start_arg, size - 1);
    int end = !has_end ? -1 : ((end_arg < 0) ? max(end_arg + size, -1) : min(end_arg, size - 1));
    for (int i = start; i > end; i += step) {
      for (int c = 0; c < stride; c++) {
        r.data[r.count * stride + c] = a.data[i * stride + c];
      }
      r.count += 1;
    }
  }
  r.total = r.count;
  return r;
}

/* The first occurrence of every value, in the original order. */
WrangleArr wr_arr_unique(WrangleArr a, int stride, int mode)
{
  int n = 0;
  for (int i = 0; i < a.count; i++) {
    bool seen = false;
    for (int j = 0; j < n; j++) {
      if (wr_arr_item_eq(a, i, a, j, stride, mode)) {
        seen = true;
        break;
      }
    }
    if (!seen) {
      for (int c = 0; c < stride; c++) {
        a.data[n * stride + c] = a.data[i * stride + c];
      }
      n += 1;
    }
  }
  a.count = n;
  a.total = n;
  return a;
}

/* `op` 0 union, 1 subtract, 2 intersect. Every value once, in the order of `a` (then `b`). */
WrangleArr wr_arr_setop(WrangleArr a, WrangleArr b, int op, int stride, int mode)
{
  WrangleArr r = a;
  r.count = 0;
  for (int i = 0; i < a.count; i++) {
    bool in_b = wr_arr_has(b, a, i, stride, mode);
    bool keep = (op == 0) || (in_b == (op == 2));
    if (keep && !wr_arr_has(r, a, i, stride, mode)) {
      for (int c = 0; c < stride; c++) {
        r.data[r.count * stride + c] = a.data[i * stride + c];
      }
      r.count += 1;
    }
  }
  if (op == 0) {
    for (int i = 0; i < b.count; i++) {
      if (wr_arr_has(r, b, i, stride, mode)) {
        continue;
      }
      if ((r.count + 1) * stride > 120) {
        wr_ovf[0] = 1;
        break;
      }
      for (int c = 0; c < stride; c++) {
        r.data[r.count * stride + c] = b.data[i * stride + c];
      }
      r.count += 1;
    }
  }
  r.total = r.count;
  return r;
}

/* An empty array gives zero. Vectors are compared per component. */
int wr_arr_min_i(WrangleArr a, bool is_min)
{
  if (a.count <= 0) {
    return 0;
  }
  int r = a.data[0];
  for (int i = 1; i < a.count; i++) {
    r = is_min ? min(r, a.data[i]) : max(r, a.data[i]);
  }
  return r;
}

float wr_arr_min_f(WrangleArr a, bool is_min)
{
  if (a.count <= 0) {
    return 0.0;
  }
  float r = wr_arr_at(a, 0);
  for (int i = 1; i < a.count; i++) {
    r = is_min ? min(r, wr_arr_at(a, i)) : max(r, wr_arr_at(a, i));
  }
  return r;
}

vec3 wr_arr_min_v(WrangleArr a, bool is_min)
{
  if (a.count <= 0) {
    return vec3(0.0);
  }
  vec3 r = wr_arr_v(a, 0);
  for (int i = 1; i < a.count; i++) {
    r = is_min ? min(r, wr_arr_v(a, i)) : max(r, wr_arr_v(a, i));
  }
  return r;
}

vec2 wr_arr_min_v2(WrangleArr a, bool is_min)
{
  if (a.count <= 0) {
    return vec2(0.0);
  }
  vec2 r = wr_arr_v2(a, 0);
  for (int i = 1; i < a.count; i++) {
    r = is_min ? min(r, wr_arr_v2(a, i)) : max(r, wr_arr_v2(a, i));
  }
  return r;
}

int wr_arr_sum_i(WrangleArr a)
{
  int r = 0;
  for (int i = 0; i < a.count; i++) {
    r += a.data[i];
  }
  return r;
}

float wr_arr_sum_f(WrangleArr a)
{
  float r = 0.0;
  for (int i = 0; i < a.count; i++) {
    r += wr_arr_at(a, i);
  }
  return r;
}

vec3 wr_arr_sum_v(WrangleArr a)
{
  vec3 r = vec3(0.0);
  for (int i = 0; i < a.count; i++) {
    r += wr_arr_v(a, i);
  }
  return r;
}

vec2 wr_arr_sum_v2(WrangleArr a)
{
  vec2 r = vec2(0.0);
  for (int i = 0; i < a.count; i++) {
    r += wr_arr_v2(a, i);
  }
  return r;
}

/* Ascending. `kind` 0 ints, 1 floats, 2 vectors (by x, then y, then z). */
bool wr_arr_less(WrangleArr a, int i, int j, int kind)
{
  if (kind == 0) {
    return a.data[i] < a.data[j];
  }
  if (kind == 1) {
    return wr_arr_at(a, i) < wr_arr_at(a, j);
  }
  vec3 p = wr_arr_v(a, i);
  vec3 q = wr_arr_v(a, j);
  return (p.x != q.x) ? (p.x < q.x) : ((p.y != q.y) ? (p.y < q.y) : (p.z < q.z));
}

WrangleArr wr_arr_sort(WrangleArr a, int kind)
{
  int stride = (kind == 2) ? 3 : 1;
  for (int i = 1; i < a.count; i++) {
    for (int j = i; j > 0 && wr_arr_less(a, j, j - 1, kind); j--) {
      for (int c = 0; c < stride; c++) {
        int w = a.data[j * stride + c];
        a.data[j * stride + c] = a.data[(j - 1) * stride + c];
        a.data[(j - 1) * stride + c] = w;
      }
    }
  }
  return a;
}

int wr_clampi(int i, int n)
{
  return (n <= 0) ? 0 : clamp(i, 0, n - 1);
}

)";
  return source.c_str();
}

struct GpuCtxGuard {
  bool locked = false;
  bool enabled_here = false;
  bool ok = false;

  GpuCtxGuard()
  {
    if (GPU_context_active_get() != nullptr) {
      ok = true;
      return;
    }
    if (G.background && !BLI_thread_is_main()) {
      /* A background session has a single GPU context, and it stays with the main thread.
       * Enabling it from an evaluation thread as well corrupts it: the Vulkan backend then
       * throws "Bad optional access" for every later use and when Blender exits. The caller
       * falls back to the interpreter, which gives the same result. */
      return;
    }
    if (!BLI_thread_is_main()) {
      GPU_context_main_lock();
      locked = true;
    }
    if (DRW_gpu_context_try_enable()) {
      enabled_here = true;
      ok = DRW_gpu_context_is_enabled();
      return;
    }
    if (BLI_thread_is_main()) {
      DRW_gpu_context_enable();
      enabled_here = true;
      ok = DRW_gpu_context_is_enabled();
    }
  }

  ~GpuCtxGuard()
  {
    if (enabled_here) {
      DRW_gpu_context_disable();
    }
    if (locked) {
      GPU_context_main_unlock();
    }
  }
};

std::mutex &shader_mutex()
{
  static std::mutex m;
  return m;
}

Map<uint64_t, gpu::Shader *> &shader_cache()
{
  static Map<uint64_t, gpu::Shader *> map;
  return map;
}

const char *soa_ssbo_type(const Type t)
{
  if (ELEM(t, Type::Int, Type::Bool)) {
    return "int";
  }
  return "float";
}

gpu::Shader *shader_for_source(const Program &program, std::string &r_error)
{
  const std::string src = std::string(gpu_arr_struct()) + program.gpu_typedef +
                          gpu_preamble() + program.gpu_src;
  const uint64_t key = get_default_hash(src);
  std::lock_guard lock(shader_mutex());
  if (gpu::Shader **found = shader_cache().lookup_ptr(key)) {
    return *found;
  }

  using namespace blender::gpu::shader;
  ShaderCreateInfo info("pyGPU_Shader");
  info.local_group_size(k_group, 1, 1);
  Vector<std::string> name_store;
  name_store.reserve(8 + program.attrs.size() * 2 + program.gpu_ch.size());
  auto intern = [&](std::string s) -> const char * {
    name_store.append(std::move(s));
    return name_store.last().c_str();
  };
  int slot = 0;
  info.storage_buf(slot++, Qualifier::read, "int", "sel[]");
  /* Set by the shader when an array needs more items than a `WrangleArr` holds. */
  info.storage_buf(slot++, Qualifier::read_write, "int", "wr_ovf[]");
  if (program.gpu_neighbors) {
    info.storage_buf(slot++, Qualifier::read, "int", "nbr_off[]");
    info.storage_buf(slot++, Qualifier::read, "int", "nbr_idx[]");
  }
  if (!program.gpu_samples.is_empty()) {
    /* Attributes of other domains and of other geometry inputs, and how many elements each has. */
    info.storage_buf(slot++, Qualifier::read, "int", "gs_n[]");
    for (const Program::GpuSample &gs : program.gpu_samples) {
      info.storage_buf(slot++,
                       Qualifier::read,
                       soa_ssbo_type(gs.type),
                       intern("gs" + std::to_string(gs.sample) + "[]"));
    }
  }
  for (int kind = 0; kind < 5; kind++) {
    /* `@ptnum`, `@edgenum`, `@facenum`, `@cornernum`, `@curvenum`: one number per element. */
    if (program.gpu_elem_nums & (1 << kind)) {
      info.storage_buf(
          slot++, Qualifier::read, "int", intern("en" + std::to_string(kind) + "[]"));
    }
  }
  for (const int i : program.attrs.index_range()) {
    const char *ty = soa_ssbo_type(program.attrs[i].type);
    info.storage_buf(
        slot++, Qualifier::read, ty, intern("a" + std::to_string(i) + "_in[]"));
    if (program.attrs[i].write) {
      info.storage_buf(
          slot++, Qualifier::read_write, ty, intern("a" + std::to_string(i) + "_out[]"));
    }
  }
  for (const int ci : program.gpu_ch.index_range()) {
    const char *ty = soa_ssbo_type(program.gpu_ch[ci].type);
    info.storage_buf(slot++, Qualifier::read, ty, intern("ch" + std::to_string(ci) + "[]"));
  }
  info.push_constant(blender::gpu::shader::Type::int_t, "n_sel");
  info.push_constant(blender::gpu::shader::Type::int_t, "n_elem");
  info.push_constant(blender::gpu::shader::Type::int_t, "npoints");
  info.push_constant(blender::gpu::shader::Type::int_t, "nedges");
  info.push_constant(blender::gpu::shader::Type::int_t, "nfaces");
  info.push_constant(blender::gpu::shader::Type::int_t, "ncorners");
  info.push_constant(blender::gpu::shader::Type::int_t, "use_sel");
  info.typedef_source_generated = std::string(gpu_arr_struct()) + program.gpu_typedef;
  info.compute_source_generated = std::string(gpu_preamble()) + program.gpu_src;

  GPU_shader_clear_last_compile_error();
  gpu::Shader *shader = GPU_shader_create_from_info_python(
      reinterpret_cast<const GPUShaderCreateInfo *>(&info));
  if (shader == nullptr) {
    r_error = GPU_shader_get_last_compile_error();
    if (r_error.empty()) {
      r_error = "Wrangle GPU shader failed to compile";
    }
    shader_cache().add(key, nullptr);
    return nullptr;
  }
  shader_cache().add(key, shader);
  return shader;
}

void pack_attr(const AttrRT &a, const int index, void *dst, const Type type)
{
  switch (type) {
    case Type::Bool: {
      int v = 0;
      if (a.wb) {
        v = a.wb[index] ? 1 : 0;
      }
      else if (a.rb) {
        v = a.rb[index] ? 1 : 0;
      }
      memcpy(dst, &v, 4);
      break;
    }
    case Type::Int: {
      int v = 0;
      if (a.wi) {
        v = a.wi[index];
      }
      else if (a.ri) {
        v = a.ri[index];
      }
      memcpy(dst, &v, 4);
      break;
    }
    case Type::Float: {
      float v = 0.0f;
      if (a.wf) {
        v = a.wf[index];
      }
      else if (a.rf) {
        v = a.rf[index];
      }
      memcpy(dst, &v, 4);
      break;
    }
    case Type::Vector: {
      float v[4] = {0, 0, 0, 0};
      if (a.wv) {
        v[0] = a.wv[index].x;
        v[1] = a.wv[index].y;
        v[2] = a.wv[index].z;
      }
      else if (a.rv) {
        v[0] = a.rv[index].x;
        v[1] = a.rv[index].y;
        v[2] = a.rv[index].z;
      }
      memcpy(dst, v, 16);
      break;
    }
    case Type::Vector2: {
      float v[2] = {0, 0};
      if (a.w2) {
        v[0] = a.w2[index].x;
        v[1] = a.w2[index].y;
      }
      else if (a.r2) {
        v[0] = a.r2[index].x;
        v[1] = a.r2[index].y;
      }
      memcpy(dst, v, 8);
      break;
    }
    case Type::Vector4: {
      float v[4] = {0, 0, 0, 0};
      if (a.w4) {
        memcpy(v, &a.w4[index], 16);
      }
      else if (a.r4) {
        memcpy(v, &a.r4[index], 16);
      }
      memcpy(dst, v, 16);
      break;
    }
    case Type::Color: {
      float v[4] = {0, 0, 0, 1};
      if (a.w4) {
        memcpy(v, &a.w4[index], 16);
      }
      else if (a.r4) {
        memcpy(v, &a.r4[index], 16);
      }
      memcpy(dst, v, 16);
      break;
    }
    case Type::Rotation: {
      float v[4] = {0, 0, 0, 1};
      const math::Quaternion *q = a.wq ? a.wq : a.rq;
      if (q) {
        v[0] = q[index].x;
        v[1] = q[index].y;
        v[2] = q[index].z;
        v[3] = q[index].w;
      }
      memcpy(dst, v, 16);
      break;
    }
    case Type::Matrix: {
      float4x4 m = float4x4::identity();
      if (a.wm) {
        m = a.wm[index];
      }
      else if (a.rm) {
        m = a.rm[index];
      }
      memcpy(dst, &m, 64);
      break;
    }
    case Type::IntArray:
    case Type::FloatArray:
    case Type::VecArray:
    case Type::Vec2Array:
    case Type::Vec4Array:
    case Type::ColorArray:
    case Type::RotArray:
    case Type::Mat2Array:
    case Type::Mat3Array:
    case Type::StringArray:
    case Type::MatArray:
    case Type::RayArray: {
      int header[4] = {0, 0, 0, 0};
      int data[k_arr_ints] = {};
      const bke::WrangleArrayValue *p = a.warr ? a.warr : a.rarr;
      if (p) {
        header[0] = int(p[index].count);
        header[1] = int(p[index].total);
        header[2] = int(p[index].kind);
        header[3] = int(p[index].truncated);
        memcpy(data, p[index].d.i, sizeof(data));
      }
      memcpy(dst, header, 16);
      memcpy(static_cast<char *>(dst) + 16, data, sizeof(data));
      break;
    }
    default:
      break;
  }
}

void unpack_attr(AttrRT &a, const int index, const void *src, const Type type)
{
  if (!a.wb && !a.wi && !a.wf && !a.wv && !a.w2 && !a.w4 && !a.wq && !a.wm && !a.warr && !a.ws) {
    return;
  }
  switch (type) {
    case Type::Bool: {
      int v = 0;
      memcpy(&v, src, 4);
      if (a.wb) {
        a.wb[index] = v != 0;
      }
      break;
    }
    case Type::Int: {
      int v = 0;
      memcpy(&v, src, 4);
      if (a.wi) {
        a.wi[index] = v;
      }
      break;
    }
    case Type::Float: {
      float v = 0.0f;
      memcpy(&v, src, 4);
      if (a.wf) {
        a.wf[index] = v;
      }
      break;
    }
    case Type::Vector: {
      float v[4] = {};
      memcpy(v, src, 16);
      if (a.wv) {
        a.wv[index] = float3(v[0], v[1], v[2]);
      }
      break;
    }
    case Type::Vector2: {
      float v[2] = {};
      memcpy(v, src, 8);
      if (a.w2) {
        a.w2[index] = float2(v[0], v[1]);
      }
      break;
    }
    case Type::Vector4: {
      if (a.w4) {
        memcpy(&a.w4[index], src, 16);
      }
      break;
    }
    case Type::Color: {
      if (a.w4) {
        memcpy(&a.w4[index], src, 16);
      }
      break;
    }
    case Type::Rotation: {
      float v[4] = {};
      memcpy(v, src, 16);
      if (a.wq) {
        a.wq[index].x = v[0];
        a.wq[index].y = v[1];
        a.wq[index].z = v[2];
        a.wq[index].w = v[3];
      }
      break;
    }
    case Type::Matrix: {
      if (a.wm) {
        memcpy(&a.wm[index], src, 64);
      }
      break;
    }
    case Type::IntArray:
    case Type::FloatArray:
    case Type::VecArray:
    case Type::Vec2Array:
    case Type::Vec4Array:
    case Type::ColorArray:
    case Type::RotArray:
    case Type::Mat2Array:
    case Type::Mat3Array:
    case Type::StringArray:
    case Type::MatArray:
    case Type::RayArray: {
      if (!a.warr) {
        break;
      }
      int header[4] = {};
      memcpy(header, src, 16);
      a.warr[index].count = uint16_t(std::max(header[0], 0));
      a.warr[index].total = uint16_t(std::max(header[1], 0));
      a.warr[index].kind = bke::WrangleArrayKind(header[2]);
      a.warr[index].truncated = uint8_t(header[3] != 0);
      memcpy(a.warr[index].d.i, static_cast<const char *>(src) + 16, sizeof(int) * k_arr_ints);
      break;
    }
    default:
      break;
  }
}

}  // namespace

int soa_floats(const Type t)
{
  if (t == Type::Vector2) {
    return 2;
  }
  if (t == Type::Vector) {
    return 3;
  }
  if (ELEM(t, Type::Vector4, Type::Color, Type::Rotation, Type::Matrix2)) {
    return 4;
  }
  if (t == Type::Matrix3) {
    return 9;
  }
  if (t == Type::Matrix) {
    return 16;
  }
  return 1;
}

void soa_pack_floats(const AttrRT &a, const int n, MutableSpan<float> dst)
{
  dst.fill(0.0f);
  if (a.type == Type::Matrix) {
    const float4x4 *p = a.wm ? a.wm : a.rm;
    for (int i = 0; i < n; i++) {
      const float4x4 m = p ? p[i] : float4x4::identity();
      memcpy(dst.data() + i * 16, &m[0][0], sizeof(float) * 16);
    }
    return;
  }
  if (ELEM(a.type, Type::Matrix2, Type::Matrix3)) {
    /* `2@` / `3@` are stored in the array attribute type, by columns. */
    const bke::WrangleArrayValue *p = a.warr ? a.warr : a.rarr;
    const int floats = (a.type == Type::Matrix2) ? 4 : 9;
    for (int i = 0; i < n; i++) {
      if (a.type == Type::Matrix2) {
        const float2x2 m = p ? p[i].as_matrix2() : float2x2::identity();
        memcpy(dst.data() + i * floats, &m[0][0], sizeof(float) * floats);
      }
      else {
        const float3x3 m = p ? p[i].as_matrix3() : float3x3::identity();
        memcpy(dst.data() + i * floats, &m[0][0], sizeof(float) * floats);
      }
    }
    return;
  }
  if (a.type == Type::Vector2) {
    const float2 *p = a.w2 ? a.w2 : a.r2;
    if (p && n > 0) {
      memcpy(dst.data(), p, sizeof(float2) * size_t(n));
    }
    return;
  }
  if (a.type == Type::Vector) {
    const float3 *p = a.wv ? a.wv : a.rv;
    if (p && n > 0) {
      memcpy(dst.data(), p, sizeof(float3) * size_t(n));
    }
    return;
  }
  if (ELEM(a.type, Type::Vector4, Type::Color)) {
    const float4 *p = a.w4 ? a.w4 : a.r4;
    if (p && n > 0) {
      memcpy(dst.data(), p, sizeof(float4) * size_t(n));
    }
    return;
  }
  if (a.type == Type::Rotation) {
    const math::Quaternion *q = a.wq ? a.wq : a.rq;
    if (q && n > 0) {
      for (int i = 0; i < n; i++) {
        dst[i * 4 + 0] = q[i].x;
        dst[i * 4 + 1] = q[i].y;
        dst[i * 4 + 2] = q[i].z;
        dst[i * 4 + 3] = q[i].w;
      }
    }
    return;
  }
  const float *p = a.wf ? a.wf : a.rf;
  if (p && n > 0) {
    memcpy(dst.data(), p, sizeof(float) * size_t(n));
  }
}

void soa_unpack_floats(AttrRT &a, const int n, const Span<float> src)
{
  if (a.type == Type::Matrix) {
    if (a.wm) {
      for (int i = 0; i < n; i++) {
        a.wm[i] = float4x4(src.data() + i * 16);
      }
    }
    return;
  }
  if (a.type == Type::Matrix2) {
    if (a.warr) {
      for (int i = 0; i < n; i++) {
        a.warr[i].set_matrix2(float2x2(src.data() + i * 4));
      }
    }
    return;
  }
  if (a.type == Type::Matrix3) {
    if (a.warr) {
      for (int i = 0; i < n; i++) {
        a.warr[i].set_matrix3(float3x3(src.data() + i * 9));
      }
    }
    return;
  }
  if (!a.wv && !a.w2 && !a.wf && !a.w4 && !a.wq) {
    return;
  }
  if (a.type == Type::Vector2 && a.w2 && n > 0) {
    memcpy(a.w2, src.data(), sizeof(float2) * size_t(n));
    return;
  }
  if (a.type == Type::Vector && a.wv && n > 0) {
    memcpy(a.wv, src.data(), sizeof(float3) * size_t(n));
    return;
  }
  if (ELEM(a.type, Type::Vector4, Type::Color) && a.w4 && n > 0) {
    memcpy(a.w4, src.data(), sizeof(float4) * size_t(n));
    return;
  }
  if (a.type == Type::Rotation && a.wq && n > 0) {
    for (int i = 0; i < n; i++) {
      a.wq[i].x = src[i * 4 + 0];
      a.wq[i].y = src[i * 4 + 1];
      a.wq[i].z = src[i * 4 + 2];
      a.wq[i].w = src[i * 4 + 3];
    }
    return;
  }
  if (a.wf && n > 0) {
    memcpy(a.wf, src.data(), sizeof(float) * size_t(n));
  }
}

void soa_pack_ints(const AttrRT &a, const int n, MutableSpan<int> dst)
{
  dst.fill(0);
  if (a.type == Type::Bool) {
    const bool *p = a.wb ? a.wb : a.rb;
    for (int i = 0; i < n; i++) {
      dst[i] = (p && p[i]) ? 1 : 0;
    }
    return;
  }
  const int *p = a.wi ? a.wi : a.ri;
  if (p && n > 0) {
    memcpy(dst.data(), p, sizeof(int) * size_t(n));
  }
}

void soa_unpack_ints(AttrRT &a, const int n, const Span<int> src)
{
  if (a.type == Type::Bool && a.wb) {
    for (int i = 0; i < n; i++) {
      a.wb[i] = src[i] != 0;
    }
    return;
  }
  if (a.wi && n > 0) {
    memcpy(a.wi, src.data(), sizeof(int) * size_t(n));
  }
}

int ssbo_slot(gpu::Shader *shader, const char *name)
{
  int slot = GPU_shader_get_ssbo_binding(shader, name);
  if (slot < 0) {
    std::string alt = std::string(name) + "[]";
    slot = GPU_shader_get_ssbo_binding(shader, alt.c_str());
  }
  return slot;
}

bool gpu_try_run(const Program &program,
                 VMEnv &env,
                 const IndexMask &mask,
                 const int domain_size,
                 std::string &r_error)
{
  if (!program.gpu_ok || program.gpu_src.empty() || domain_size <= 0) {
    return false;
  }
  if (!program.gpu_soa && (program.gpu_stride <= 0 || program.gpu_stride > 128)) {
    return false;
  }
  if (mask.is_empty()) {
    return true;
  }
  if (program.gpu_pack.size() != program.attrs.size()) {
    return false;
  }

  GpuCtxGuard ctx;
  if (!ctx.ok) {
    return false;
  }

  gpu::Shader *shader = shader_for_source(program, r_error);
  if (shader == nullptr) {
    /* Leave r_error so the cook reports the GLSL failure instead of silently using the CPU VM. */
    return false;
  }

  const int n = domain_size;
  const bool full = mask.size() == domain_size;
  Vector<int> sel_mask(n, 0);
  if (full) {
    sel_mask.fill(1);
  }
  else {
    mask.foreach_index([&](const int64_t i) {
      if (i >= 0 && i < n) {
        sel_mask[int(i)] = 1;
      }
    });
  }

  struct AttrGpu {
    bool is_int = false;
    bool write = false;
    Vector<float> hf;
    Vector<int> hi;
    gpu::StorageBuf *in_ssbo = nullptr;
    gpu::StorageBuf *out_ssbo = nullptr;
    int in_slot = -1;
    int out_slot = -1;
  };
  Vector<AttrGpu> ag;
  ag.reinitialize(program.attrs.size());
  Vector<gpu::StorageBuf *> to_free;
  auto make_ssbo = [&](size_t bytes, const void *data, const char *name) -> gpu::StorageBuf * {
    gpu::StorageBuf *b = GPU_storagebuf_create_ex(
        std::max(bytes, size_t(4)), data, GPU_USAGE_DYNAMIC, name);
    if (b) {
      to_free.append(b);
    }
    return b;
  };

  gpu::StorageBuf *sel_ssbo = make_ssbo(
      size_t(n) * sizeof(int), sel_mask.data(), "wrangle_sel");
  int overflow = 0;
  gpu::StorageBuf *overflow_ssbo = make_ssbo(sizeof(int), &overflow, "wrangle_overflow");
  gpu::StorageBuf *nbr_off_ssbo = nullptr;
  gpu::StorageBuf *nbr_idx_ssbo = nullptr;
  if (program.gpu_neighbors) {
    Vector<int> off_copy;
    off_copy.extend(env.gpu_nbr_off);
    Vector<int> idx_copy;
    idx_copy.extend(env.gpu_nbr_idx);
    if (off_copy.is_empty()) {
      off_copy.append(0);
    }
    if (idx_copy.is_empty()) {
      idx_copy.append(0);
    }
    nbr_off_ssbo = make_ssbo(
        size_t(off_copy.size()) * sizeof(int), off_copy.data(), "wrangle_nbr_off");
    nbr_idx_ssbo = make_ssbo(
        size_t(idx_copy.size()) * sizeof(int), idx_copy.data(), "wrangle_nbr_idx");
  }

  /* Sampled attributes of other domains / geometry inputs. A sample the executor has no data for
   * is an empty buffer, the shader then reads the node's own attribute. */
  Vector<int> sample_sizes(std::max<int64_t>(program.element_samples.size(), 1), 0);
  Vector<gpu::StorageBuf *> sample_ssbo(program.gpu_samples.size(), nullptr);
  for (const int si : program.gpu_samples.index_range()) {
    const Program::GpuSample &gs = program.gpu_samples[si];
    const AttrRT *data = (gs.sample < env.static_samples.size()) ? &env.static_samples[gs.sample] :
                                                                   nullptr;
    const bool usable = data && data->size > 0 && data->type == gs.type;
    const int size = usable ? data->size : 0;
    if (gs.sample < sample_sizes.size()) {
      sample_sizes[gs.sample] = size;
    }
    if (ELEM(gs.type, Type::Int, Type::Bool)) {
      Vector<int> values(std::max(size, 1), 0);
      if (usable) {
        soa_pack_ints(*data, size, values);
      }
      sample_ssbo[si] = make_ssbo(size_t(values.size()) * sizeof(int), values.data(), "wr_gs");
    }
    else {
      Vector<float> values(std::max(size * soa_floats(gs.type), 1), 0.0f);
      if (usable) {
        soa_pack_floats(*data, size, values);
      }
      sample_ssbo[si] = make_ssbo(size_t(values.size()) * sizeof(float), values.data(), "wr_gs");
    }
  }
  gpu::StorageBuf *sample_sizes_ssbo = program.gpu_samples.is_empty() ?
                                           nullptr :
                                           make_ssbo(size_t(sample_sizes.size()) * sizeof(int),
                                                     sample_sizes.data(),
                                                     "wr_gs_n");

  gpu::StorageBuf *elem_num_ssbo[5] = {};
  for (int kind = 0; kind < 5; kind++) {
    if ((program.gpu_elem_nums & (1 << kind)) == 0) {
      continue;
    }
    /* Without numbers from the executor (the size does not match) there is no such element. */
    Vector<int> numbers(n, -1);
    if (env.gpu_elem_nums[kind].size() == n) {
      numbers.as_mutable_span().copy_from(env.gpu_elem_nums[kind]);
    }
    elem_num_ssbo[kind] = make_ssbo(size_t(n) * sizeof(int), numbers.data(), "wrangle_elem_num");
  }

  for (const int i : program.attrs.index_range()) {
    AttrGpu &g = ag[i];
    g.write = program.attrs[i].write;
    g.is_int = ELEM(program.attrs[i].type, Type::Int, Type::Bool);
    if (i >= env.attrs.size()) {
      continue;
    }
    if (g.is_int) {
      g.hi.resize(n);
      soa_pack_ints(env.attrs[i], n, g.hi);
      g.in_ssbo = make_ssbo(size_t(n) * sizeof(int), g.hi.data(), "wr_ai");
      if (g.write) {
        g.out_ssbo = make_ssbo(size_t(n) * sizeof(int), g.hi.data(), "wr_ao");
      }
    }
    else {
      const int nf = soa_floats(program.attrs[i].type) * n;
      g.hf.resize(std::max(nf, 1));
      soa_pack_floats(env.attrs[i], n, g.hf);
      g.in_ssbo = make_ssbo(size_t(g.hf.size()) * sizeof(float), g.hf.data(), "wr_af");
      if (g.write) {
        g.out_ssbo = make_ssbo(size_t(g.hf.size()) * sizeof(float), g.hf.data(), "wr_afo");
      }
    }
  }

  Vector<gpu::StorageBuf *> ch_ssbo(program.gpu_ch.size(), nullptr);
  Vector<Vector<float>> ch_hf(program.gpu_ch.size());
  Vector<Vector<int>> ch_hi(program.gpu_ch.size());
  for (const int ci : program.gpu_ch.index_range()) {
    const Program::GpuCh &ch = program.gpu_ch[ci];
    const VMEnv::ChSrc *src = (ci < env.gpu_ch_src.size()) ? &env.gpu_ch_src[ci] : nullptr;
    if (ELEM(ch.type, Type::Int, Type::Bool)) {
      ch_hi[ci].resize(n, 0);
      if (src && src->i) {
        memcpy(ch_hi[ci].data(), src->i, sizeof(int) * size_t(n));
      }
      ch_ssbo[ci] = make_ssbo(size_t(n) * sizeof(int), ch_hi[ci].data(), "wr_ch");
    }
    else if (ch.type == Type::Vector) {
      ch_hf[ci].resize(n * 3, 0.0f);
      if (src && src->v) {
        memcpy(ch_hf[ci].data(), src->v, sizeof(float3) * size_t(n));
      }
      ch_ssbo[ci] = make_ssbo(size_t(ch_hf[ci].size()) * sizeof(float), ch_hf[ci].data(), "wr_ch");
    }
    else {
      ch_hf[ci].resize(n, 0.0f);
      if (src && src->f) {
        memcpy(ch_hf[ci].data(), src->f, sizeof(float) * size_t(n));
      }
      ch_ssbo[ci] = make_ssbo(size_t(n) * sizeof(float), ch_hf[ci].data(), "wr_ch");
    }
  }

  auto fail_free = [&]() {
    for (gpu::StorageBuf *b : to_free) {
      if (b) {
        GPU_storagebuf_free(b);
      }
    }
    return false;
  };
  if (!sel_ssbo || !overflow_ssbo ||
      (program.gpu_neighbors && (!nbr_off_ssbo || !nbr_idx_ssbo)))
  {
    return fail_free();
  }
  for (const AttrGpu &g : ag) {
    if (g.in_ssbo == nullptr || (g.write && g.out_ssbo == nullptr)) {
      return fail_free();
    }
  }

  GPU_shader_bind(shader);
  const int sel_slot = ssbo_slot(shader, "sel");
  if (sel_slot < 0) {
    GPU_shader_unbind();
    return fail_free();
  }
  GPU_storagebuf_bind(sel_ssbo, sel_slot);
  const int overflow_slot = ssbo_slot(shader, "wr_ovf");
  if (overflow_slot < 0) {
    GPU_shader_unbind();
    return fail_free();
  }
  GPU_storagebuf_bind(overflow_ssbo, overflow_slot);
  if (program.gpu_neighbors) {
    const int off_slot = ssbo_slot(shader, "nbr_off");
    const int idx_slot = ssbo_slot(shader, "nbr_idx");
    if (off_slot < 0 || idx_slot < 0) {
      GPU_shader_unbind();
      return fail_free();
    }
    GPU_storagebuf_bind(nbr_off_ssbo, off_slot);
    GPU_storagebuf_bind(nbr_idx_ssbo, idx_slot);
  }
  if (!program.gpu_samples.is_empty()) {
    const int sizes_slot = ssbo_slot(shader, "gs_n");
    if (sizes_slot < 0 || sample_sizes_ssbo == nullptr) {
      GPU_shader_unbind();
      return fail_free();
    }
    GPU_storagebuf_bind(sample_sizes_ssbo, sizes_slot);
    for (const int si : program.gpu_samples.index_range()) {
      const std::string name = "gs" + std::to_string(program.gpu_samples[si].sample);
      const int slot = ssbo_slot(shader, name.c_str());
      if (slot < 0 || sample_ssbo[si] == nullptr) {
        GPU_shader_unbind();
        return fail_free();
      }
      GPU_storagebuf_bind(sample_ssbo[si], slot);
    }
  }
  for (int kind = 0; kind < 5; kind++) {
    if ((program.gpu_elem_nums & (1 << kind)) == 0) {
      continue;
    }
    const std::string name = "en" + std::to_string(kind);
    const int slot = ssbo_slot(shader, name.c_str());
    if (slot < 0 || elem_num_ssbo[kind] == nullptr) {
      GPU_shader_unbind();
      return fail_free();
    }
    GPU_storagebuf_bind(elem_num_ssbo[kind], slot);
  }
  for (const int i : program.attrs.index_range()) {
    AttrGpu &g = ag[i];
    const std::string in_name = "a" + std::to_string(i) + "_in";
    g.in_slot = ssbo_slot(shader, in_name.c_str());
    if (g.in_slot < 0) {
      GPU_shader_unbind();
      return fail_free();
    }
    if (g.write) {
      const std::string out_name = "a" + std::to_string(i) + "_out";
      g.out_slot = ssbo_slot(shader, out_name.c_str());
      if (g.out_slot < 0) {
        GPU_shader_unbind();
        return fail_free();
      }
    }
  }
  for (const int ci : program.gpu_ch.index_range()) {
    const std::string nm = "ch" + std::to_string(ci);
    const int slot = ssbo_slot(shader, nm.c_str());
    if (slot < 0 || ch_ssbo[ci] == nullptr) {
      GPU_shader_unbind();
      return fail_free();
    }
    GPU_storagebuf_bind(ch_ssbo[ci], slot);
  }

  GPU_shader_uniform_1i(shader, "n_sel", int(mask.size()));
  GPU_shader_uniform_1i(shader, "n_elem", domain_size);
  GPU_shader_uniform_1i(shader, "npoints", env.npoints);
  GPU_shader_uniform_1i(shader, "nedges", env.nedges);
  GPU_shader_uniform_1i(shader, "nfaces", env.nfaces);
  GPU_shader_uniform_1i(shader, "ncorners", env.ncorners);
  GPU_shader_uniform_1i(shader, "use_sel", full ? 0 : 1);

  const uint groups_1d = (uint(n) + uint(k_group) - 1u) / uint(k_group);
  const int max_g = std::max(GPU_max_work_group_count(0), 1);
  uint gx = groups_1d;
  uint gy = 1;
  if (gx > uint(max_g)) {
    gx = uint(max_g);
    gy = (groups_1d + gx - 1u) / gx;
  }

  const int passes = std::max(env.array_passes, 1);
  for (int pass = 0; pass < passes; pass++) {
    for (AttrGpu &g : ag) {
      GPU_storagebuf_bind(g.in_ssbo, g.in_slot);
      if (g.write) {
        GPU_storagebuf_bind(g.out_ssbo, g.out_slot);
      }
    }
    GPU_compute_dispatch(shader, gx, gy, 1);
    GPU_memory_barrier(GPUBarrier(GPU_BARRIER_SHADER_STORAGE | GPU_BARRIER_BUFFER_UPDATE));
    for (AttrGpu &g : ag) {
      if (g.write) {
        std::swap(g.in_ssbo, g.out_ssbo);
      }
    }
  }

  /* An array did not fit in the shader: nothing has been written back yet, the interpreter runs
   * the program instead and gives the complete result. */
  GPU_storagebuf_read(overflow_ssbo, &overflow);
  if (overflow != 0) {
    GPU_shader_unbind();
    for (gpu::StorageBuf *b : to_free) {
      if (b) {
        GPU_storagebuf_unbind(b);
        GPU_storagebuf_free(b);
      }
    }
    r_error = "an array is longer than the GPU arrays, using the interpreter";
    return false;
  }

  for (const int i : program.attrs.index_range()) {
    AttrGpu &g = ag[i];
    if (!g.write || i >= env.attrs.size()) {
      continue;
    }
    /* After swap, in_ssbo holds the last out. */
    if (g.is_int) {
      GPU_storagebuf_read(g.in_ssbo, g.hi.data());
      soa_unpack_ints(env.attrs[i], n, g.hi);
    }
    else {
      GPU_storagebuf_read(g.in_ssbo, g.hf.data());
      soa_unpack_floats(env.attrs[i], n, g.hf);
    }
  }

  GPU_shader_unbind();
  for (gpu::StorageBuf *b : to_free) {
    if (b) {
      GPU_storagebuf_unbind(b);
      GPU_storagebuf_free(b);
    }
  }
  return true;
}

}  // namespace blender::nodes::vex
