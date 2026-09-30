/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <string>

#include "BLI_index_mask_fwd.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_string_ref.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"

#include "BKE_wrangle_array.hh"
#include "DNA_meshdata_types.h"

namespace blender::nodes::vex {

enum class Type : uint8_t {
  Void = 0,
  Bool,
  Int,
  Float,
  Vector2,
  Vector,
  Vector4,
  Color,
  Rotation,
  Matrix2,
  Matrix3,
  Matrix,
  String,
  Ray,
  IntArray,
  FloatArray,
  VecArray,
  StringArray,
  MatArray,
  RayArray,
};

struct RayHit {
  int hit = 0;
  int face = -1;
  float3 pos = float3(0.0f);
  float3 n = float3(0.0f);
  float dist = 0.0f;
};

inline bool type_is_vec_like(const Type type)
{
  return type == Type::Vector2 || type == Type::Vector || type == Type::Vector4 ||
         type == Type::Color;
}

inline bool type_has_xyzw(const Type type)
{
  return type_is_vec_like(type) || type == Type::Rotation;
}

inline int swizzle_axis_from_char(const char c)
{
  switch (c) {
    case 'x':
    case 'r':
      return 0;
    case 'y':
    case 'g':
      return 1;
    case 'z':
    case 'b':
      return 2;
    case 'w':
    case 'a':
      return 3;
    default:
      return -1;
  }
}

/** Parse `.xy` / `.zxy` / `.rgba`. Mixes xyzw with rgba are rejected. */
inline bool parse_swizzle(const StringRef name, int r_axes[4], int &r_n)
{
  r_n = 0;
  if (name.is_empty() || name.size() > 4) {
    return false;
  }
  int set = 0;
  for (int i = 0; i < int(name.size()); i++) {
    const char c = name[i];
    const int axis = swizzle_axis_from_char(c);
    if (axis < 0) {
      return false;
    }
    const int s = ELEM(c, 'r', 'g', 'b', 'a') ? 2 : 1;
    if (set == 0) {
      set = s;
    }
    else if (set != s) {
      return false;
    }
    r_axes[r_n++] = axis;
  }
  return r_n >= 1;
}

/** Single component: 0..3. Multi: `(n << 8) | axes packed 2 bits each`. */
inline int pack_swizzle(const int *axes, const int n)
{
  if (n <= 1) {
    return n == 1 ? (axes[0] & 3) : 0;
  }
  int packed = n << 8;
  for (int i = 0; i < n; i++) {
    packed |= (axes[i] & 3) << (2 * i);
  }
  return packed;
}

inline int swizzle_count(const int imm)
{
  const int n = (imm >> 8) & 7;
  return n >= 2 ? n : 1;
}

inline int swizzle_axis(const int imm, const int i)
{
  if (((imm >> 8) & 7) < 2) {
    return imm & 3;
  }
  return (imm >> (2 * i)) & 3;
}

inline Type swizzle_result_type(const int n)
{
  if (n == 2) {
    return Type::Vector2;
  }
  if (n == 3) {
    return Type::Vector;
  }
  if (n >= 4) {
    return Type::Vector4;
  }
  return Type::Float;
}

inline bool type_is_matrix(const Type type)
{
  return type == Type::Matrix2 || type == Type::Matrix3 || type == Type::Matrix;
}

inline bool type_is_wrangle_packed(const Type type)
{
  return type == Type::IntArray || type == Type::FloatArray || type == Type::VecArray ||
         type == Type::StringArray || type == Type::MatArray || type == Type::RayArray ||
         type == Type::Matrix2 || type == Type::Matrix3;
}

inline int type_linear_dim(const Type type)
{
  switch (type) {
    case Type::Vector2:
    case Type::Matrix2:
      return 2;
    case Type::Vector:
    case Type::Matrix3:
      return 3;
    case Type::Vector4:
    case Type::Color:
    case Type::Rotation:
    case Type::Matrix:
      return 4;
    default:
      return 0;
  }
}

inline Type vec_type_for_dim(const int dim)
{
  if (dim == 2) {
    return Type::Vector2;
  }
  if (dim == 4) {
    return Type::Vector4;
  }
  return Type::Vector;
}

/** How many floats `set()` / `vector()` pack from one argument. */
inline int type_pack_comp_count(const Type type)
{
  switch (type) {
    case Type::Vector2:
      return 2;
    case Type::Vector:
      return 3;
    case Type::Vector4:
    case Type::Color:
    case Type::Rotation:
      return 4;
    default:
      return 1;
  }
}

/** Houdini `set()`: scalars and vector-like args are concatenated into vec2/3/4. */
inline Type set_fn_result_type(const Span<Type> arg_types)
{
  int n = 0;
  for (const Type t : arg_types) {
    n += type_pack_comp_count(t);
  }
  if (n <= 2) {
    return Type::Vector2;
  }
  if (n == 3) {
    return Type::Vector;
  }
  return Type::Vector4;
}

/** Keep vector/color type for component-wise math; scalars become float. */
inline Type unary_vec_keep(const Type type)
{
  return type_is_vec_like(type) ? type : Type::Float;
}

/** Wider of two vector-like types, or Void if neither is a vector. */
inline Type dominant_vec_type(const Type a, const Type b)
{
  if (ELEM(a, Type::Vector4, Type::Color) || ELEM(b, Type::Vector4, Type::Color)) {
    return (a == Type::Color && b == Type::Color) ? Type::Color : Type::Vector4;
  }
  if (a == Type::Vector || b == Type::Vector) {
    return Type::Vector;
  }
  if (a == Type::Vector2 || b == Type::Vector2) {
    return Type::Vector2;
  }
  return Type::Void;
}

/** nearestpoints mode: 0 = k-nearest, 1 = all in radius, 2 = at most k within radius. */
inline int nearestpoints_mode_from_name(const StringRef s)
{
  if (s == "rk" || s == "RK" || s == "kr" || s == "maxk" || s == "rmax" || s == "radiusk") {
    return 2;
  }
  if (s == "r" || s == "R" || s == "rnn" || s == "radius" || s == "Radius") {
    return 1;
  }
  return 0;
}

inline bool nearestpoints_name_is_mode(const StringRef s)
{
  return s == "k" || s == "K" || s == "knn" || s == "r" || s == "R" || s == "rnn" ||
         s == "radius" || s == "Radius" || s == "rk" || s == "RK" || s == "kr" || s == "maxk" ||
         s == "rmax" || s == "radiusk";
}

enum class Op : uint8_t {
  Nop = 0,
  PushF,
  PushI,
  PushV,
  PushB,
  PushS, /* const pool string */
  PushLocal,
  StoreLocal,
  Dup,
  Pop,
  PushAttr,
  StoreAttr,
  AttrAdd,  /* attrs[imm][index] += pop() */
  AttrMulAdd, /* attrs[imm][index] += pop_addend * pop_count */
  IncLocal, /* locals[imm]++ as int */
  DecLocal, /* locals[imm]-- as int */
  PushIndex,
  PushNpoints,
  PushNedges,
  PushNfaces,
  PushNcorners,
  Add,
  Sub,
  Mul,
  Div,
  Mod,
  Pow, /* a ** b */
  Neg,
  Lt,
  Le,
  Gt,
  Ge,
  Eq,
  Ne,
  And,
  Or,
  Not,
  Jmp,
  JmpIfFalse,
  JmpIfTrue,
  Call,
  /** Static point()/edge()/face()/corner()/curve()/instance() sample.
   * `imm` is an index into Program::element_samples. The low five bits of `a` store the number of
   * source arguments to discard; the high three bits store explicit-index-argument + 1 (zero means
   * use VMEnv::index). */
  SampleElem,
  /** facecorners() on geometry 0. `a` stores the source argument count. The mesh face-offset
   * span is bound directly in #VMEnv, avoiding the generic topology callback. */
  FaceCorners,
  /** Fused `for (i=0; i<len(indices); i++) out[i]=sample(...,indices[i])` loop. `imm`
   * indexes #Program::gather_samples. */
  GatherSamples,
  /** Fused `local[index]` load, optionally followed by a scalar member read. `imm` indexes
   * #Program::local_index_loads. */
  PushLocalIndex,
  /** Fused `local.member` load. `imm` is the local slot and `a` is the scalar member. */
  PushLocalMember,
  CallUser, /* imm = user fn index, a = nargs */
  GetMember,  /* imm: 0..3 single xyzw, or packed swizzle (n<<8 | axes) */
  SetMember,
  MakeVec, /* imm = component count 1..4 → vector2 / vector / vector4 */
  GetIndex, /* array[index] */
  SetIndex, /* value, array, index -> array */
  Return,
  PushHitPos, /* last spatial query position (same tree as dist, not a 2nd query) */
  WhileCmpAdd, /* C kernel: while (attr[member] cmp limit) attr += addend */
};

enum class Builtin : uint16_t {
  Sin = 0,
  Cos,
  Tan,
  Asin,
  Acos,
  Atan,
  Atan2,
  Abs,
  Floor,
  Ceil,
  Round,
  Trunc,
  Sqrt,
  Exp,
  Log,
  Pow,
  Min,
  Max,
  Clamp,
  Length,
  Distance,
  Dot,
  Cross,
  Normalize,
  Radians,
  Degrees,
  Sign,
  Fract,
  Mix,
  Noise,
  Hash,
  Npoints,
  Nedges,
  Nfaces,
  Ncorners,
  Point,
  Edge,
  Face,
  Corner,
  Addpoint,
  Set,
  FloatFn,
  IntFn,
  BoolFn,
  Vec3Fn,
  Invert,
  Transpose,
  Determinant,
  TransformPoint,
  TransformDirection,
  ProjectPoint,
  CombineTransform,
  Identity,
  TranslationFn,
  RotationFn,
  ScaleFn,
  Chf,
  Chi,
  Chv,
  Chb,
  Chc,
  Chm,
  Chq,
  Rand,
  QuatFn,
  RotateRotation,
  ArrayFn,
  ArrayInt,
  ArrayFloat,
  ArrayVec,
  ArrayStr,
  ArrayMat,
  ArrayRay,
  Append,
  Insert,
  RemoveIndex,
  RemoveValue,
  SortArr,
  Len,
  CornersOfFace,
  CornersOfVertex,
  CornersOfEdge,
  EdgesOfVertex,
  EdgesOfCorner,
  FaceOfCorner,
  VertexOfCorner,
  OffsetCornerInFace,
  FacesOfVertex,
  Neighbours,
  PointNeighbours,
  PointEdges,
  PointFaces,
  PointCorners,
  EdgePoints,
  EdgeFaces,
  FacePoints,
  FaceEdges,
  FaceNeighbours,
  FaceCorners,
  CornerFace,
  PointsOfCurve,
  CurveOfPoint,
  PointCurve,
  NearestPoints,
  NearPoints,
  Raycast,
  RayHitPos,
  RayHitN,
  RayHitDist,
  BBoxMin,
  BBoxMax,
  GeometryProximity,
  SampleNearestSurface,
  DeleteGeometry,
  SetAttribute,
  Accumulate,
  FieldAverage,
  FieldMin,
  FieldMax,
  FieldMinMax,
  Voronoi,
  ColorFn,
  Chs,
  BoundingBox,
  MatrixFn,
  Count,
  Curve,
  Instance,
  ValueToString,
  Format,
  Vec2Fn,
  Vec4Fn,
  Matrix2Fn,
  Matrix3Fn,
  SvdFn,
  PolarDecompFn,
  EigenFn,
  DecompU,
  DecompS,
  DecompV,
  SortOrder,
  SortByOrder,
  RayIsHit,
  Chu,
  Ch2,
  Ch3,
  Ch4,
  Chr,
  Map,
  Smooth,
  RaycastAll,
  RayIsHitArr,
  RayHitPosArr,
  RayHitNArr,
  RayHitDistArr,
  Addprim,
  Ddx,
  Ddy,
  Fwidth,
};

struct Value {
  Type type = Type::Float;
  int i = 0; /* int/bool, string pool index, or matrix pool index */
  float4 v{0.0f, 0.0f, 0.0f, 0.0f};

  static Value from_bool(const bool b)
  {
    Value r;
    r.type = Type::Bool;
    r.i = b ? 1 : 0;
    r.v.x = b ? 1.0f : 0.0f;
    return r;
  }
  static Value from_int(const int n)
  {
    Value r;
    r.type = Type::Int;
    r.i = n;
    r.v.x = float(n);
    return r;
  }
  static Value from_float(const float f)
  {
    Value r;
    r.type = Type::Float;
    r.v.x = f;
    return r;
  }
  static Value from_vec(const float3 p)
  {
    Value r;
    r.type = Type::Vector;
    r.v = float4(p.x, p.y, p.z, 0.0f);
    return r;
  }
  static Value from_vec2(const float2 p)
  {
    Value r;
    r.type = Type::Vector2;
    r.v = float4(p.x, p.y, 0.0f, 0.0f);
    return r;
  }
  static Value from_vec4(const float4 p, const Type t)
  {
    Value r;
    r.type = t;
    r.v = p;
    return r;
  }
  static Value from_matrix2(const float2x2 &mat)
  {
    Value r;
    r.type = Type::Matrix2;
    r.v = float4(mat[0].x, mat[0].y, mat[1].x, mat[1].y);
    return r;
  }
  static Value from_str_i(const int id)
  {
    Value r;
    r.type = Type::String;
    r.i = id;
    return r;
  }
  static Value from_matrix(const float4x4 &mat);
  static Value from_matrix3(const float3x3 &mat);

  bool as_bool() const
  {
    if (type == Type::Bool || type == Type::Int) {
      return i != 0;
    }
    return v.x != 0.0f;
  }
  int as_int() const
  {
    if (type == Type::Int || type == Type::Bool) {
      return i;
    }
    return int(v.x);
  }
  float as_float() const
  {
    if (type == Type::Int || type == Type::Bool) {
      return float(i);
    }
    return v.x;
  }
  float3 as_vec() const
  {
    if (type == Type::Vector || type == Type::Vector4 || type == Type::Color ||
        type == Type::Rotation)
    {
      return float3(v.x, v.y, v.z);
    }
    if (type == Type::Vector2) {
      return float3(v.x, v.y, 0.0f);
    }
    const float f = as_float();
    return float3(f, f, f);
  }
  float2 as_vec2() const
  {
    if (type_is_vec_like(type)) {
      return float2(v.x, v.y);
    }
    const float f = as_float();
    return float2(f, f);
  }
  float4 as_vec4() const
  {
    if (type == Type::Vector2) {
      return float4(v.x, v.y, 0.0f, 0.0f);
    }
    if (type == Type::Vector) {
      return float4(v.x, v.y, v.z, 0.0f);
    }
    if (type == Type::Vector4 || type == Type::Color || type == Type::Rotation) {
      return v;
    }
    const float f = as_float();
    return float4(f, f, f, f);
  }
  float2x2 as_matrix2() const
  {
    if (type == Type::Matrix2) {
      return float2x2(float2(v.x, v.y), float2(v.z, v.w));
    }
    const float s = as_float();
    return float2x2(float2(s, 0.0f), float2(0.0f, s));
  }
};

struct Inst {
  Op op = Op::Nop;
  uint8_t a = 0;
  int32_t imm = 0;
};

struct AttrInfo {
  std::string name;
  Type type = Type::Float;
  bool read = false;
  bool write = false;
};

struct WhileCmpAddSpec {
  int attr = 0;
  int member = -1; /* -1 = scalar float attr; 0..2 = vector component */
  uint8_t cmp = 0; /* 0 <, 1 <=, 2 >, 3 >= */
  float limit = 0.0f;
  float3 addend = float3(0.0f);
  /** Extra int attributes incremented once per iteration (`i@counter++`). */
  Vector<int> counters;
};

struct UserFn {
  std::string name;
  Type ret = Type::Void;
  Vector<Type> param_types;
  int entry = 0;
  int local_count = 0;
};

struct Program {
  Vector<Inst> code;
  Vector<UserFn> user_fns;
  Vector<float> const_f;
  Vector<int> const_i;
  Vector<float3> const_v;
  Vector<std::string> const_s;
  Vector<AttrInfo> attrs;
  Vector<WhileCmpAddSpec> while_adds;
  int local_count = 0;
  int max_stack = 32;
  /* When true, execute() precomputes geometry_proximity(pos,dist) in a C parallel loop. */
  bool prox_batch = false;
  int prox_geo = 0;
  int prox_domain = 2;
  uint8_t prox_geo_mask = 0;
  bool prox_geo_dynamic = false;
  /** Sample positions from this attr; -1 means `P` / `position`. */
  int prox_sample_attr = -1;
  /* Raycast: origin is current P or a vector attr; dir/length constant (or from an attr). */
  bool ray_batch = false;
  int ray_geo = 0;
  uint8_t ray_geo_mask = 0;
  bool ray_geo_dynamic = false;
  float3 ray_dir = float3(0.0f, 0.0f, -1.0f);
  float ray_len = 100.0f;
  int ray_dir_attr = -1;
  int ray_orig_attr = -1;
  bool ray_dir_normalize = false;
  /* sample_nearest_surface: sample pos is current P. */
  bool sample_batch = false;
  int sample_geo = 0;
  uint8_t sample_geo_mask = 0;
  bool sample_geo_dynamic = false;
  std::string sample_attr = "position";
  /**
   * Program is only batched proximity/raycast plus storing the hits. execute() writes the
   * precomputed arrays straight into attributes and skips the bytecode interpreter.
   */
  bool spatial_only = false;
  int prox_pos_attr = -1;
  int prox_dist_attr = -1;
  int ray_hit_attr = -1;
  int ray_pos_attr = -1;
  int ray_n_attr = -1;
  int ray_dist_attr = -1;
  /* Same hit can be stored to more than one attribute (`v@temp = rayhitpos(r); v@hitpos = hp`). */
  Vector<int> prox_pos_attrs;
  Vector<int> prox_dist_attrs;
  Vector<int> ray_hit_attrs;
  Vector<int> ray_pos_attrs;
  Vector<int> ray_n_attrs;
  Vector<int> ray_dist_attrs;

  /**
   * GPU compute: GLSL emitted from the AST. When #gpu_ok is true, execute() dispatches a
   * compute shader instead of interpreting bytecode on the CPU.
   */
  bool gpu_ok = false;
  std::string gpu_src;
  /**
   * Top-level GLSL helpers (user-defined VEX functions). Shader wrangle prepends this
   * before the material function; nested GLSL functions are not allowed.
   */
  std::string gpu_helpers;
  std::string gpu_error;
  std::string gpu_typedef;
  struct GpuPack {
    int offset = 0;
    int bytes = 0;
    Type type = Type::Float;
  };
  Vector<GpuPack> gpu_pack;
  int gpu_stride = 0;
  /* Tight SoA buffers (`float P[n*3]`) instead of fat per-element structs. */
  bool gpu_soa = false;

  /**
   * Counted `for (i = 0; i < N; i++)` whose body samples `point()` of attributes it also
   * writes is peeled to one Jacobi pass. The host (CPU or GPU) repeats the program N times
   * against the working attribute arrays so iteration k+1 reads writes from iteration k.
   */
  int array_passes = 1;
  std::string array_passes_ch;
  bool array_pass_peeled = false;
  /* Neighbor-average + lerp of sampled vector attrs (`v@P += (avg-P)*fac`). Same
   * algorithm as the Blur Attribute node: typed arrays, ping-pong, parallel_for. */
  bool jacobi_smooth = false;
  Vector<std::string> jacobi_sample_attrs;
  Vector<std::string> jacobi_avg_attrs;
  std::string jacobi_fac_ch;
  /** Use k-nearest (nearestpoints) instead of mesh topology for Jacobi. */
  bool jacobi_knn = false;
  int jacobi_knn_k = 8;
  int jacobi_knn_geo = 0;
  /** Precompute a CSR kNN table once; VM nearestpoints indexes it instead of querying per call. */
  bool knn_precompute = false;
  int knn_k = 8;
  int knn_geo = 0;
  bool gpu_neighbors = false;
  /**
   * Attributes sampled from geometry 0 through point()/edge()/face()/corner()/curve()/instance().
   * The executor only needs a Jacobi snapshot when one of these attributes is also written by the
   * same program. Dynamic geometry or attribute arguments conservatively request snapshots of all
   * writable attributes.
   */
  Vector<std::string> sampled_self_attrs;
  bool sampled_self_attr_dynamic = false;
  struct ElementSample {
    int geo = 0;
    int domain = 0;
    std::string name;
  };
  /** Static element samples prebound once per execution instead of doing an attribute lookup for
   * every point()/corner()/face() call in every VM lane. */
  Vector<ElementSample> element_samples;
  struct GatherSamples {
    int index_array_local = -1;
    Vector<int> output_locals;
    Vector<int> sample_slots;
    Vector<Type> output_types;
  };
  /** Static attribute gathers lowered from simple array traversal loops. */
  Vector<GatherSamples> gather_samples;
  struct LocalIndexLoad {
    int array_local = -1;
    /** Local slot or integer constant-pool slot, selected by #index_is_const. */
    int index = -1;
    int member = -1;
    bool index_is_const = false;
  };
  /** Peephole-lowered local array loads. Keeping the operands here avoids truncating local and
   * constant-pool indices to the compact instruction fields. */
  Vector<LocalIndexLoad> local_index_loads;
  struct GpuCh {
    std::string name;
    Type type = Type::Float;
    int offset = 0;
    int bytes = 0;
  };
  Vector<GpuCh> gpu_ch;
};

struct AttrRT {
  Type type = Type::Float;
  int size = 0;
  const float *rf = nullptr;
  float *wf = nullptr;
  const int *ri = nullptr;
  int *wi = nullptr;
  const bool *rb = nullptr;
  bool *wb = nullptr;
  const float3 *rv = nullptr;
  float3 *wv = nullptr;
  const float2 *r2 = nullptr;
  float2 *w2 = nullptr;
  const float4 *r4 = nullptr;
  float4 *w4 = nullptr;
  const float4x4 *rm = nullptr;
  float4x4 *wm = nullptr;
  const math::Quaternion *rq = nullptr;
  math::Quaternion *wq = nullptr;
  const bke::WrangleArrayValue *rarr = nullptr;
  bke::WrangleArrayValue *warr = nullptr;
  const MStringProperty *rs = nullptr;
  MStringProperty *ws = nullptr;
};

using LoadElemFn = Value (*)(void *user,
                             int geo_index,
                             int domain,
                             StringRef name,
                             int index,
                             Type expect,
                             std::string &r_error);

using LoadSampleFn = Value (*)(void *user,
                               int sample_slot,
                               int index,
                               std::string &r_error);

using LoadParmFn = Value (*)(void *user, StringRef name, int index, std::string &r_error);

struct VMEnv;

using TopoFn = Value (*)(void *user,
                         int builtin_id,
                         Span<Value> args,
                         VMEnv &env,
                         std::string &r_error);

struct VMEnv {
  int index = 0;
  int rand_seq = 0;
  int npoints = 0;
  int nedges = 0;
  int nfaces = 0;
  int ncorners = 0;
  Vector<float3> *addpoints = nullptr;
  MutableSpan<AttrRT> attrs;
  Span<std::string> const_s;
  Vector<std::string> *runtime_s = nullptr;
  void *elem_user = nullptr;
  LoadElemFn load_elem = nullptr;
  LoadSampleFn load_sample = nullptr;
  /** Typed spans for static cross-domain/secondary-geometry samples. Empty entries intentionally
   * fall back to #load_sample for computed attributes and same-domain Jacobi snapshots. */
  Span<AttrRT> static_samples;
  /** Mesh face offsets for the current geometry. Size is faces + 1. */
  Span<int> face_offsets;
  void *parm_user = nullptr;
  LoadParmFn load_parm = nullptr;
  void *topo_user = nullptr;
  TopoFn topo_fn = nullptr;
  float3 hit_pos = float3(0.0f);
  float3 hit_n = float3(0.0f);
  float hit_dist = 0.0f;
  int hit_face = -1;
  int array_passes = 1;
  Span<int> gpu_nbr_off;
  Span<int> gpu_nbr_idx;
  struct ChSrc {
    Type type = Type::Float;
    const int *i = nullptr;
    const float *f = nullptr;
    const float3 *v = nullptr;
  };
  Span<ChSrc> gpu_ch_src;
};

Value value_from_quat(const math::Quaternion &q);
math::Quaternion value_as_quat(const Value &v);

/** Returns false on error. If the script `return`s, #r_return is set. */
bool vm_run(const Program &program, VMEnv &env, std::string &r_error, Value *r_return);

/** Rewrite counted `for (i=0; i<K; i++) attr += x` into a single scaled add. */
void optimize_program(Program &program);

/** True when every op is uniform control + whole-array attr writes (no per-element interpret). */
bool program_can_array_exec(const Program &program);

/** Run the program once as C loops over \a mask (not once per element). */
bool vm_run_array(const Program &program,
                  VMEnv &env,
                  const blender::IndexMask &mask,
                  std::string &r_error);

StringRef value_string(const Value &v, const VMEnv &env);

std::string type_name(Type type);
Type prefix_type(char prefix);
bool type_is_numeric(Type type);
bool type_is_array(Type type);
Builtin matrix_ctor_of(Type type);
Type array_type_of(Type elem);
Type array_elem_type(Type type);

Value value_from_int_array(Vector<int> values);
Value value_from_int_span(Span<int> src);
Value value_from_int_range(int start, int size);
Vector<int> int_array_values(const Value &v);
Vector<int> *iarr_mut(const Value &v);
Value value_from_float_array(Vector<float> values);
Value value_from_vec_array(Vector<float3> values);
Value value_from_string_array(Vector<std::string> values);
Value value_from_mat_array(Vector<float4x4> values);
Value value_from_ray(const RayHit &hit);
Value value_from_ray_array(Vector<RayHit> hits);
const RayHit *ray_from_value(const Value &v);
Value packed_to_value(const bke::WrangleArrayValue &p);

}  // namespace blender::nodes::vex
