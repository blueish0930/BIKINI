/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <string>

#include "BLI_hash.hh"
#include "BLI_map.hh"
#include "BLI_math_vector.hh"
#include "BLI_string_ref.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"

#include "NOD_vex.hh"
#include "vex_program.hh"

namespace blender::nodes::vex {

std::string type_name(const Type type)
{
  switch (type) {
    case Type::Void:
      return "void";
    case Type::Bool:
      return "bool";
    case Type::Int:
      return "int";
    case Type::Float:
      return "float";
    case Type::Vector2:
      return "vector2";
    case Type::Vector:
      return "vector";
    case Type::Vector4:
      return "vector4";
    case Type::Color:
      return "color";
    case Type::Rotation:
      return "rotation";
    case Type::Matrix2:
      return "matrix2";
    case Type::Matrix3:
      return "matrix3";
    case Type::Matrix:
      return "matrix";
    case Type::String:
      return "string";
    case Type::Ray:
      return "ray";
    case Type::IntArray:
      return "int[]";
    case Type::FloatArray:
      return "float[]";
    case Type::VecArray:
      return "vector[]";
    case Type::StringArray:
      return "string[]";
    case Type::MatArray:
      return "matrix[]";
    case Type::RayArray:
      return "ray[]";
  }
  return "?";
}

static bool is_vector_ctor_name(const StringRef name)
{
  return name == "vector" || name == "vector2" || name == "vector4" || name == "vec2" ||
         name == "vec3" || name == "vec4" || name == "set";
}

static int vector_ctor_want(const StringRef name)
{
  if (name == "vector2" || name == "vec2") {
    return 2;
  }
  if (name == "vector4" || name == "vec4") {
    return 4;
  }
  return 3;
}

Type prefix_type(const char prefix)
{
  switch (prefix) {
    case 'f':
      return Type::Float;
    case 'i':
      return Type::Int;
    case 'b':
      return Type::Bool;
    case 'u':
      return Type::Vector2;
    case 'v':
      return Type::Vector;
    case 'q':
      return Type::Vector4;
    case 'c':
      return Type::Color;
    case 'r':
      return Type::Rotation;
    case '2':
      return Type::Matrix2;
    case '3':
      return Type::Matrix3;
    case '4':
    case 'm':
      return Type::Matrix;
    case 's':
      return Type::String;
    default:
      return Type::Void;
  }
}

Builtin matrix_ctor_of(const Type type)
{
  if (type == Type::Matrix2) {
    return Builtin::Matrix2Fn;
  }
  if (type == Type::Matrix3) {
    return Builtin::Matrix3Fn;
  }
  return Builtin::MatrixFn;
}

bool type_is_numeric(const Type type)
{
  return ELEM(type,
              Type::Bool,
              Type::Int,
              Type::Float,
              Type::Vector2,
              Type::Vector,
              Type::Vector4,
              Type::Color,
              Type::Rotation);
}

bool type_is_array(const Type type)
{
  return ELEM(type,
              Type::IntArray,
              Type::FloatArray,
              Type::VecArray,
              Type::StringArray,
              Type::MatArray,
              Type::RayArray);
}

static const char *expect_tag_name(const StringRef tag)
{
  if (tag == "geo") {
    return "int (geo)";
  }
  if (tag == "enum") {
    return "int or string";
  }
  if (tag == "int") {
    return "int";
  }
  if (tag == "float") {
    return "float";
  }
  if (tag == "vec") {
    return "vector";
  }
  if (tag == "str") {
    return "string";
  }
  if (tag == "mat") {
    return "matrix";
  }
  if (tag == "quat") {
    return "rotation";
  }
  if (tag == "arr") {
    return "array";
  }
  if (tag == "ray") {
    return "ray";
  }
  if (tag == "num") {
    return "float or vector";
  }
  return "value";
}

static bool type_fits_tag(const StringRef tag, const Type got)
{
  if (got == Type::Void || tag.is_empty() || tag == "out") {
    return true;
  }
  if (tag == "geo") {
    /* Geometry input index: 0, 1, 2…  Also allow float so `geo` placeholders type-check. */
    return ELEM(got, Type::Int, Type::Bool, Type::Float);
  }
  if (tag == "enum") {
    /* Domain / mode / texture type: `0` or `"point"` / `"fbm"`. */
    return ELEM(got, Type::Int, Type::Bool, Type::Float, Type::String);
  }
  if (tag == "int") {
    return ELEM(got, Type::Int, Type::Bool);
  }
  if (tag == "float") {
    return ELEM(got, Type::Int, Type::Float, Type::Bool);
  }
  if (tag == "vec") {
    return ELEM(got, Type::Vector2, Type::Vector, Type::Vector4, Type::Color);
  }
  if (tag == "num") {
    return ELEM(got,
                Type::Int,
                Type::Float,
                Type::Bool,
                Type::Vector2,
                Type::Vector,
                Type::Vector4,
                Type::Color);
  }
  if (tag == "str") {
    return got == Type::String;
  }
  if (tag == "mat") {
    return type_is_matrix(got);
  }
  if (tag == "quat") {
    return got == Type::Rotation;
  }
  if (tag == "ray") {
    return got == Type::Ray;
  }
  if (tag == "arr") {
    return type_is_array(got);
  }
  return true;
}

Type array_type_of(const Type elem)
{
  switch (elem) {
    case Type::Int:
    case Type::Bool:
      return Type::IntArray;
    case Type::Ray:
      return Type::RayArray;
    case Type::Vector:
    case Type::Vector2:
    case Type::Vector4:
      return Type::VecArray;
    case Type::String:
      return Type::StringArray;
    case Type::Matrix:
    case Type::Matrix2:
    case Type::Matrix3:
    case Type::MatArray:
      return Type::MatArray;
    case Type::Float:
    default:
      return Type::FloatArray;
  }
}

Type array_elem_type(const Type type)
{
  switch (type) {
    case Type::IntArray:
      return Type::Int;
    case Type::VecArray:
      return Type::Vector;
    case Type::StringArray:
      return Type::String;
    case Type::MatArray:
      return Type::Matrix;
    case Type::RayArray:
      return Type::Ray;
    case Type::FloatArray:
    default:
      return Type::Float;
  }
}

static void offset_to_line_col(
    const StringRef src, int64_t off, int &r_line, int &r_col, std::string &r_snippet)
{
  r_line = 1;
  r_col = 1;
  int64_t line_start = 0;
  off = std::max<int64_t>(0, std::min(off, int64_t(src.size())));
  for (int64_t i = 0; i < off; i++) {
    if (src[i] == '\n') {
      r_line++;
      r_col = 1;
      line_start = i + 1;
    }
    else {
      r_col++;
    }
  }
  int64_t line_end = line_start;
  while (line_end < int64_t(src.size()) && src[line_end] != '\n') {
    line_end++;
  }
  r_snippet = std::string(src.substr(line_start, std::min<int64_t>(80, line_end - line_start)));
}

struct ErrHint {
  std::string title;
  std::string hint;
};

static std::string unknown_func_fix(StringRef what);

static ErrHint err_hint(const StringRef what)
{
  ErrHint h;
  if (what.startswith("Unknown prefix") || what.find("attribute prefix") != StringRef::not_found) {
    h.title = "属性前缀不对";
    h.hint = "f@ float  i@ int  v@ vector  u@ vector2  q@ vector4  r@ rotation  2@/3@/4@ 矩阵";
    return h;
  }
  if (what.startswith("Unknown function")) {
    h.title = "没有这个函数";
    h.hint = unknown_func_fix(what);
    const int64_t q1 = what.find('\'');
    const int64_t q2 = (q1 == StringRef::not_found) ? StringRef::not_found :
                                                      what.find('\'', q1 + 1);
    if (q1 != StringRef::not_found && q2 != StringRef::not_found && q2 > q1 + 1) {
      h.title = "没有函数 " + std::string(what.substr(q1 + 1, q2 - q1 - 1));
    }
    return h;
  }
  if (what.startswith("Type mismatch") || what.find("类型不对") != StringRef::not_found) {
    h.title = std::string(what);
    if (StringRef(h.title).startswith("Type mismatch: ")) {
      h.title = h.title.substr(15);
    }
    h.hint = "对照函数签名，把参数换成它要的类型。";
    return h;
  }
  if (what.startswith("Unterminated string")) {
    h.title = "字符串少了结束引号";
    h.hint = "把 \" 或 ' 配对写完。";
    return h;
  }
  if (what.startswith("Unterminated comment")) {
    h.title = "块注释少了结束符 */";
    h.hint = "每个 /* 都必须有对应的 */。";
    return h;
  }
  if (what.startswith("Unexpected character")) {
    h.title = "这里有非法字符";
    h.hint = "属性写成 f@name / v@P，不要用 $ 或 #。";
    return h;
  }
  if (what.find("Expected ';'") != StringRef::not_found) {
    h.title = "少了分号";
    h.hint = "这句写完后加 ;";
    return h;
  }
  if (what.find("Expected ')'") != StringRef::not_found) {
    h.title = "少了 )";
    h.hint = "把 ( 和 ) 配齐。";
    return h;
  }
  if (what.find("Expected '('") != StringRef::not_found) {
    h.title = "少了 (";
    h.hint = "if / for / 函数名后面要跟 (。";
    return h;
  }
  if (what.find("Expected '}'") != StringRef::not_found) {
    h.title = "少了 }";
    h.hint = "每个 { 都要有对应的 }。";
    return h;
  }
  if (what.find("Expected '{'") != StringRef::not_found) {
    h.title = "少了 {";
    h.hint = "代码块用 { } 包起来。";
    return h;
  }
  if (what.find("Expected ']'") != StringRef::not_found) {
    h.title = "少了 ]";
    h.hint = "数组下标写成 xs[0]。";
    return h;
  }
  if (what.find("Expected ','") != StringRef::not_found) {
    h.title = "少了逗号";
    h.hint = "参数之间用逗号隔开。";
    return h;
  }
  if (what.find("Expected ':'") != StringRef::not_found) {
    h.title = "三元运算少了 :";
    h.hint = "写成 条件 ? 真 : 假";
    return h;
  }
  if (what.find("Expected '@'") != StringRef::not_found) {
    h.title = "属性要写成 前缀@名字";
    h.hint = "例: f@scale  i@index  v@P";
    return h;
  }
  if (what.find("Expected expression") != StringRef::not_found) {
    h.title = "这里缺一个值";
    h.hint = "写数字、变量、属性或函数调用。";
    return h;
  }
  if (what.find("Expected variable name") != StringRef::not_found) {
    h.title = "类型后面要跟变量名";
    h.hint = "例: int n = 0;";
    return h;
  }
  if (what.find("Expected attribute name") != StringRef::not_found) {
    h.title = "@ 后面要写属性名";
    h.hint = "例: f@scale = 2;";
    return h;
  }
  if (what.find("Expected member name") != StringRef::not_found ||
      what.find("Unknown member") != StringRef::not_found)
  {
    h.title = "分量名只能是 x/y/z/w、r/g/b/a，或它们的组合";
    h.hint = "例: v@P.z += 1;  p = p.zxy;  vector2 a = p.zx;";
    return h;
  }
  if (what.find("Cannot assign to i@index") != StringRef::not_found) {
    h.title = "i@index 是只读的";
    h.hint = "写到别的属性，例如 i@id = i@index;";
    return h;
  }
  if (what.find("Invalid assignment target") != StringRef::not_found) {
    h.title = "等号左边必须是变量或属性";
    h.hint = "例: f@scale = 2;";
    return h;
  }
  if (what.find("break outside loop") != StringRef::not_found) {
    h.title = "break 只能写在循环里";
    return h;
  }
  if (what.find("continue outside loop") != StringRef::not_found) {
    h.title = "continue 只能写在循环里";
    return h;
  }
  if (what.find("维度") != StringRef::not_found || what.find("左乘") != StringRef::not_found ||
      what.find("矩阵") != StringRef::not_found)
  {
    h.title = std::string(what);
    return h;
  }
  if (what.startswith("Expected")) {
    h.title = "这里的符号不对";
    h.hint = "对照 ^ 的位置：缺分号就加 ;，括号要成对。";
    return h;
  }
  h.title = std::string(what);
  if (h.title.size() > 80) {
    h.title.resize(80);
  }
  return h;
}

static std::string format_vex_error(const StringRef src,
                                    const int64_t off,
                                    const StringRef what,
                                    const int64_t /*len*/ = 1)
{
  int line = 1;
  int col = 1;
  std::string snippet;
  offset_to_line_col(src, off, line, col, snippet);
  const ErrHint h = err_hint(what);
  std::string msg = "L" + std::to_string(line) + ": " + h.title;
  if (!snippet.empty()) {
    msg += "\n  " + snippet;
    msg += "\n  ";
    const int caret = std::clamp(col - 1, 0, int(snippet.size()));
    msg += std::string(size_t(caret), ' ');
    msg += '^';
  }
  if (!h.hint.empty()) {
    msg += "\n  " + h.hint;
  }
  return msg;
}

static std::string unknown_func_fix(const StringRef what)
{
  std::string name;
  const int64_t q1 = what.find('\'');
  const int64_t q2 = (q1 == StringRef::not_found) ? StringRef::not_found :
                                                    what.find('\'', q1 + 1);
  if (q1 != StringRef::not_found && q2 != StringRef::not_found && q2 > q1 + 1) {
    name = std::string(what.substr(q1 + 1, q2 - q1 - 1));
  }
  auto has = [&](const char *s) {
    return name.find(s) != std::string::npos;
  };
  if (has("neigh") || has("neighbor")) {
    return "点邻居用 pointneighbours(geo, index)；面邻居用 faceneighbours(geo, index)。\n"
           "  例: i[]@nbr = pointneighbours(0, i@index);";
  }
  if (has("prox") || has("dist")) {
    return "靠近采样用 geometry_proximity(geo, \"face\", 采样点, 命中位置, 距离)。\n"
           "  例: vector hit; float d; geometry_proximity(1, \"face\", v@P, hit, d);";
  }
  if (has("nearest") || has("nearpoint") || has("knn") || has("rnn")) {
    return "近邻只有 nearestpoints(geo, k_or_r, mode)，返回 int[]。\n"
           "  mode 0/\"k\": k 近邻         i[]@pts = nearestpoints(0, 8, \"k\");\n"
           "  mode 1/\"r\": 半径内全部      i[]@pts = nearestpoints(0, 0.25, \"r\");\n"
           "  mode 2/\"rk\": 半径内最多 k 个 i[]@pts = nearestpoints(0, 0.25, 8, \"rk\");";
  }
  if (has("ray")) {
    return "射线用 raycast(geo, origin, dir, length, is_hit, hit_pos, hit_n, hit_dist)。\n"
           "  全部命中: ray hits[] = raycastall(...)，再用 rayishit(hits[i]) / rayhitpos(hits[i]) 读每条射线。\n"
           "  例: int hit; vector hp, hn; float hd; raycast(0, v@P, {0,0,-1}, 10, hit, hp, hn, hd);";
  }
  if (has("noise") || has("rand") || has("hash") || has("voronoi")) {
    return "噪声: noise(P, scale, detail, roughness, lacunarity, distortion, dim, type)。\n"
           "  dim 1-4；type: fbm|multifractal|hybrid|ridged|hetero。\n"
           "  voronoi 同样有 lacunarity / distortion / dimension / type(f1|f2|smooth_f1|edge|radius)。\n"
           "  随机: rand(i@index)。";
  }
  if (has("arr") || has("len") || has("append")) {
    return "数组: int[] xs = {1,2,3};  或  i[]@ids = array(1,2,3);  长度用 len(xs)。";
  }
  return "把函数名改成下面之一（注意全小写、不要空格）：\n"
         "  拓扑: pointneighbours / pointedges / pointfaces / facepoints / faceneighbours /\n"
         "        edgecorners / edgepoints / edgefaces / facecorners /\n"
         "        cornerpoint / corneredges / offsetcorner /\n"
         "        pointcurve / curvepoints\n"
         "  空间: nearestpoints / raycast / raycastall / geometry_proximity / sample_nearest_surface\n"
         "  写入: addpoint / addprim / setattribute / delete_geometry（geo 只能是 0）\n"
         "  数学: sin, cos, clamp, mix, map, smooth, length, normalize, noise, rand, array, len\n"
         "  屏幕导数(仅着色器 wrangle / EEVEE): ddx, ddy, fwidth\n"
         "  通道: chf(\"name\"), chi(\"name\"), chv(\"name\")\n"
         "  例: i[]@nbr = pointneighbours(0, i@index);";
}

static Diagnostic make_diag(const int64_t off,
                            const int64_t len,
                            const bool is_error,
                            const StringRef what)
{
  Diagnostic d;
  d.offset = int(std::max<int64_t>(off, 0));
  d.length = std::max(1, int(len));
  d.is_error = is_error;
  d.message = err_hint(what).title;
  return d;
}

#if 0
static void infer_why_fix_unused(const StringRef what, std::string &r_why, std::string &r_fix)
{
  if (what.startswith("Unknown prefix") || what.find("attribute prefix") != StringRef::not_found) {
    r_why = "属性前缀不对。";
    r_fix = "f@ float, i@ int, b@ bool, u@ vector2, v@ vector, q@ vector4, c@ color,\n"
            "  r@ rotation, 2@ matrix2, 3@ matrix3, 4@ / m@ matrix, s@ string。";
    return;
  }
  if (what.startswith("Unknown function")) {
    r_why = "没有这个函数名（拼写、大小写或旧名字都不行）。";
    r_fix = unknown_func_fix(what);
    return;
  }
  if (what.startswith("Type mismatch")) {
    r_why = "函数参数的类型对不上。";
    r_fix = "int / geo 参数不能传 vector。例: pointneighbours(0, i@index);\n"
            "  不要写 pointneighbours(0, v@P)。float 可以用 int；vector 用 v@P 或 {x,y,z}。";
    return;
  }
  if (what.startswith("Unterminated string")) {
    r_why = "字符串少了结束引号。";
    r_fix = "在这一行把 \" 或 ' 配对写完。例: s@name = \"hello\";  或  chf(\"amp\");";
    return;
  }
  if (what.startswith("Unexpected character")) {
    r_why = "这个字符不是 wrangle 语法的一部分。";
    r_fix = "删掉它，或放进字符串里。属性用 f@name / i@index / v@P，不要用 $ 或 #。";
    return;
  }
  if (what.find("Expected ';'") != StringRef::not_found) {
    r_why = "这句话写完了但少了分号。";
    r_fix = "在语句末尾加 ;  例:\n"
            "  v@P += {0, 0, 1};\n"
            "  i[]@nbr = pointneighbours(0, i@index);";
    return;
  }
  if (what.find("Expected ')'") != StringRef::not_found) {
    r_why = "括号没闭合，少了 )。";
    r_fix = "把 ( 和 ) 配齐。函数调用例: noise(v@P);  clamp(x, 0, 1);  if (i@index == 0) { ... }";
    return;
  }
  if (what.find("Expected '('") != StringRef::not_found) {
    r_why = "if / for / while / 函数后面必须跟 (。";
    r_fix = "例: if (x > 0) { v@P.z += 1; }\n"
            "     for (int i = 0; i < 10; i++) { ... }\n"
            "     float n = noise(v@P);";
    return;
  }
  if (what.find("Expected '}'") != StringRef::not_found) {
    r_why = "花括号没闭合，少了 }。";
    r_fix = "每个 { 都要有对应的 }。例: if (x > 0) { v@P.z += 1; }";
    return;
  }
  if (what.find("Expected '{'") != StringRef::not_found) {
    r_why = "这里需要用 { } 包住代码块。";
    r_fix = "例: if (i@index == 0) { v@P = {0,0,0}; }";
    return;
  }
  if (what.find("Expected ']'") != StringRef::not_found) {
    r_why = "方括号没闭合，少了 ]。";
    r_fix = "数组属性: i[]@ids = {1,2,3};  下标: xs[0] = 1;";
    return;
  }
  if (what.find("Expected ','") != StringRef::not_found) {
    r_why = "参数或列表项之间要用逗号隔开。";
    r_fix = "例: clamp(x, 0, 1);  vector v = {1, 0, 0};  array(1, 2, 3);";
    return;
  }
  if (what.find("Expected ':'") != StringRef::not_found) {
    r_why = "三元运算符少了冒号。";
    r_fix = "写法: 条件 ? 真值 : 假值;  例: float x = (i@index > 0) ? 1.0 : 0.0;";
    return;
  }
  if (what.find("Expected '@'") != StringRef::not_found) {
    r_why = "属性必须写成 类型前缀 + @ + 名字。";
    r_fix = "例: f@scale = 1;  i@id = i@index;  v@P += {0,0,1};  s@name = \"a\";  i[]@nbr = pointneighbours(0, i@index);";
    return;
  }
  if (what.find("Expected expression") != StringRef::not_found) {
    r_why = "这里需要一个值（数字、变量、属性或函数调用）。";
    r_fix = "例: float x = 1.0;  v@P += v@N;  int n = i@index;";
    return;
  }
  if (what.find("Expected variable name") != StringRef::not_found) {
    r_why = "类型后面要跟变量名。";
    r_fix = "例: int n = 0;  float d = 1.0;  vector p = v@P;  int[] ids = {1,2,3};";
    return;
  }
  if (what.find("Expected attribute name") != StringRef::not_found) {
    r_why = "@ 后面要写属性名字。";
    r_fix = "例: f@scale = 2;  i[]@ids = {1,2,3};  当前点编号是 i@index，位置是 v@P。";
    return;
  }
  if (what.find("Expected member name") != StringRef::not_found ||
      what.find("Unknown member") != StringRef::not_found)
  {
    r_why = "点号后面只能用分量 x/y/z/w、r/g/b/a，或组合（.xy .zx .zxy）。";
    r_fix = "例: v@P.z += 1;  p = p.zxy;  vector2 a = p.zx;";
    return;
  }
  if (what.find("Cannot assign to i@index") != StringRef::not_found) {
    r_why = "i@index 是只读的（当前元素编号），不能赋值。";
    r_fix = "写到别的属性。例: i@id = i@index;  或  f@score = float(i@index);";
    return;
  }
  if (what.find("Invalid assignment target") != StringRef::not_found) {
    r_why = "等号左边必须是变量或属性。";
    r_fix = "例: f@scale = 2;  int n = 1;  xs[0] = 3;  不能写  1 = x;  或  v@P.x.y = 0;";
    return;
  }
  if (what.find("break outside loop") != StringRef::not_found) {
    r_why = "break 只能写在 for / while 里面。";
    r_fix = "例: for (int i = 0; i < 10; i++) { if (i == 3) { break; } }";
    return;
  }
  if (what.find("continue outside loop") != StringRef::not_found) {
    r_why = "continue 只能写在 for / while 里面。";
    r_fix = "例: for (int i = 0; i < 10; i++) { if (i == 3) { continue; } v@P.z += 1; }";
    return;
  }
  if (what.startswith("Expected")) {
    r_why = "这一格的符号不对，编译器在等另一个符号。";
    r_fix = "对照 ^ 指向的位置：缺分号就加 ; ，括号要成对 () [] {}。\n"
            "  例: v@P += {0,0,1};  if (x > 0) { f@a = 1; }";
    return;
  }
  r_why = "这一行语法不符合 wrangle 规则。";
  r_fix = "看 ^ 指到的字符。常见写法:\n"
          "  v@P += {0, 0, 1};\n"
          "  i[]@nbr = pointneighbours(0, i@index);\n"
          "  if (i@index == 0) { f@a = 1; }";
}
#endif

static bool is_element_index_name(const StringRef name)
{
  return name == "index" || name == "ptnum" || name == "elemnum";
}

static std::string canonical_attr_name(const StringRef name)
{
  auto eq_ci = [](const StringRef a, const StringRef b) {
    if (a.size() != b.size()) {
      return false;
    }
    for (int64_t i = 0; i < a.size(); i++) {
      if (std::tolower(uchar(a[i])) != std::tolower(uchar(b[i]))) {
        return false;
      }
    }
    return true;
  };
  if (name == "P" || eq_ci(name, "position")) {
    return "position";
  }
  if (name == "N" || eq_ci(name, "normal")) {
    return "normal";
  }
  if (eq_ci(name, "output")) {
    return "Output";
  }
  return std::string(name);
}

enum class TokKind : uint8_t {
  Eof,
  Ident,
  Number,
  String,
  KwIf,
  KwElse,
  KwFor,
  KwForeach,
  KwWhile,
  KwBreak,
  KwContinue,
  KwReturn,
  KwFloat,
  KwInt,
  KwBool,
  KwVector,
  KwVector2,
  KwVector4,
  KwMatrix,
  KwMatrix2,
  KwMatrix3,
  KwRotation,
  KwString,
  KwColor,
  KwVoid,
  KwRay,
  Plus,
  Minus,
  Star,
  StarStar,
  Slash,
  Percent,
  PlusPlus,
  MinusMinus,
  PlusEq,
  MinusEq,
  StarEq,
  SlashEq,
  Eq,
  EqEq,
  NotEq,
  Lt,
  Gt,
  Le,
  Ge,
  AndAnd,
  OrOr,
  Not,
  LParen,
  RParen,
  LBrace,
  RBrace,
  LBracket,
  RBracket,
  Comma,
  Semi,
  Dot,
  At,
  Amp,
  Question,
  Colon,
};

struct Tok {
  TokKind kind = TokKind::Eof;
  StringRef text;
  bool is_float = false;
};

static std::string describe_tok(const Tok &t)
{
  switch (t.kind) {
    case TokKind::Eof:
      return "end of file";
    case TokKind::Ident:
      return "name '" + std::string(t.text) + "'";
    case TokKind::Number:
      return "number " + std::string(t.text);
    case TokKind::String:
      return "string " + std::string(t.text);
    case TokKind::KwIf:
      return "'if'";
    case TokKind::KwElse:
      return "'else'";
    case TokKind::KwFor:
      return "'for'";
    case TokKind::KwForeach:
      return "'foreach'";
    case TokKind::KwWhile:
      return "'while'";
    case TokKind::KwBreak:
      return "'break'";
    case TokKind::KwContinue:
      return "'continue'";
    case TokKind::KwReturn:
      return "'return'";
    case TokKind::Semi:
      return "';'";
    case TokKind::Comma:
      return "','";
    case TokKind::Dot:
      return "'.'";
    case TokKind::At:
      return "'@'";
    case TokKind::LParen:
      return "'('";
    case TokKind::RParen:
      return "')'";
    case TokKind::LBrace:
      return "'{'";
    case TokKind::RBrace:
      return "'}'";
    case TokKind::LBracket:
      return "'['";
    case TokKind::RBracket:
      return "']'";
    case TokKind::Eq:
      return "'='";
    case TokKind::Plus:
      return "'+'";
    case TokKind::Minus:
      return "'-'";
    default:
      if (!t.text.is_empty()) {
        return "'" + std::string(t.text) + "'";
      }
      return "unexpected token";
  }
}

struct Lexer {
  StringRef src;
  int64_t i = 0;
  int64_t fail_off = 0;
  int64_t fail_len = 1;
  std::string error;
  std::string error_kind;

  char peek(const int64_t off = 0) const
  {
    const int64_t k = i + off;
    return k < src.size() ? src[k] : '\0';
  }

  static bool is_ident_start(const char c)
  {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
  }
  static bool is_ident(const char c)
  {
    return is_ident_start(c) || (c >= '0' && c <= '9');
  }
  static bool is_digit(const char c)
  {
    return c >= '0' && c <= '9';
  }

  Tok make(const TokKind kind, const int64_t start, const int64_t len)
  {
    Tok t;
    t.kind = kind;
    t.text = src.substr(start, len);
    return t;
  }

  void skip()
  {
    while (i < src.size()) {
      const char c = src[i];
      if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
        i++;
        continue;
      }
      if (c == '/' && peek(1) == '/') {
        i += 2;
        while (i < src.size() && src[i] != '\n') {
          i++;
        }
        continue;
      }
      if (c == '/' && peek(1) == '*') {
        const int64_t comment_start = i;
        i += 2;
        while (i + 1 < src.size() && !(src[i] == '*' && src[i + 1] == '/')) {
          i++;
        }
        if (i + 1 < src.size()) {
          i += 2;
        }
        else {
          fail_off = comment_start;
          fail_len = 2;
          error_kind = "Unterminated comment";
          error = format_vex_error(src, fail_off, error_kind);
          i = src.size();
          return;
        }
        continue;
      }
      break;
    }
  }

  Tok next()
  {
    skip();
    if (!error.empty()) {
      return {};
    }
    if (i >= src.size()) {
      return make(TokKind::Eof, i, 0);
    }
    const int64_t start = i;
    const char c = src[i];

    if (is_ident_start(c)) {
      i++;
      while (i < src.size() && is_ident(src[i])) {
        i++;
      }
      const StringRef text = src.substr(start, i - start);
      Tok t = make(TokKind::Ident, start, i - start);
      if (text == "if") {
        t.kind = TokKind::KwIf;
      }
      else if (text == "else") {
        t.kind = TokKind::KwElse;
      }
      else if (text == "for") {
        t.kind = TokKind::KwFor;
      }
      else if (text == "foreach") {
        t.kind = TokKind::KwForeach;
      }
      else if (text == "while") {
        t.kind = TokKind::KwWhile;
      }
      else if (text == "break") {
        t.kind = TokKind::KwBreak;
      }
      else if (text == "continue") {
        t.kind = TokKind::KwContinue;
      }
      else if (text == "return") {
        t.kind = TokKind::KwReturn;
      }
      else if (text == "float") {
        t.kind = TokKind::KwFloat;
      }
      else if (text == "int") {
        t.kind = TokKind::KwInt;
      }
      else if (text == "bool" || text == "boolean") {
        t.kind = TokKind::KwBool;
      }
      else if (text == "vector2" || text == "vec2") {
        t.kind = TokKind::KwVector2;
      }
      else if (text == "vector4" || text == "vec4") {
        t.kind = TokKind::KwVector4;
      }
      else if (text == "vector" || text == "vec3") {
        t.kind = TokKind::KwVector;
      }
      else if (text == "matrix2" || text == "mat2") {
        t.kind = TokKind::KwMatrix2;
      }
      else if (text == "matrix3" || text == "mat3") {
        t.kind = TokKind::KwMatrix3;
      }
      else if (text == "matrix" || text == "matrix4" || text == "mat4") {
        t.kind = TokKind::KwMatrix;
      }
      else if (text == "rotation" || text == "quaternion" || text == "quat") {
        t.kind = TokKind::KwRotation;
      }
      else if (text == "string") {
        t.kind = TokKind::KwString;
      }
      else if (text == "color") {
        t.kind = TokKind::KwColor;
      }
      else if (text == "void") {
        t.kind = TokKind::KwVoid;
      }
      else if (text == "ray") {
        t.kind = TokKind::KwRay;
      }
      return t;
    }

    if (is_digit(c) || (c == '.' && is_digit(peek(1)))) {
      bool is_float = (c == '.');
      i++;
      while (i < src.size() && is_digit(src[i])) {
        i++;
      }
      if (!is_float && i < src.size() && src[i] == '.') {
        is_float = true;
        i++;
        while (i < src.size() && is_digit(src[i])) {
          i++;
        }
      }
      if (i < src.size() && (src[i] == 'e' || src[i] == 'E')) {
        const char nxt = peek(1);
        if (is_digit(nxt) || nxt == '+' || nxt == '-') {
          is_float = true;
          i++;
          if (src[i] == '+' || src[i] == '-') {
            i++;
          }
          while (i < src.size() && is_digit(src[i])) {
            i++;
          }
        }
      }
      Tok t = make(TokKind::Number, start, i - start);
      t.is_float = is_float;
      return t;
    }

    if (c == '"' || c == '\'') {
      const char quote = c;
      i++;
      while (i < src.size() && src[i] != quote) {
        if (src[i] == '\\' && i + 1 < src.size()) {
          i += 2;
        }
        else {
          i++;
        }
      }
      if (i >= src.size()) {
        error_kind = "Unterminated string";
        error = format_vex_error(src, start, error_kind);
        fail_off = start;
        fail_len = std::max<int64_t>(1, i - start);
        return {};
      }
      i++;
      return make(TokKind::String, start, i - start);
    }

    auto eat1 = [&](const TokKind k) {
      i++;
      return make(k, start, 1);
    };
    auto eat2 = [&](const TokKind k) {
      i += 2;
      return make(k, start, 2);
    };

    switch (c) {
      case '+':
        if (peek(1) == '+') {
          return eat2(TokKind::PlusPlus);
        }
        if (peek(1) == '=') {
          return eat2(TokKind::PlusEq);
        }
        return eat1(TokKind::Plus);
      case '-':
        if (peek(1) == '-') {
          return eat2(TokKind::MinusMinus);
        }
        if (peek(1) == '=') {
          return eat2(TokKind::MinusEq);
        }
        return eat1(TokKind::Minus);
      case '*':
        if (peek(1) == '*') {
          return eat2(TokKind::StarStar);
        }
        return peek(1) == '=' ? eat2(TokKind::StarEq) : eat1(TokKind::Star);
      case '/':
        return peek(1) == '=' ? eat2(TokKind::SlashEq) : eat1(TokKind::Slash);
      case '%':
        return eat1(TokKind::Percent);
      case '=':
        return peek(1) == '=' ? eat2(TokKind::EqEq) : eat1(TokKind::Eq);
      case '!':
        return peek(1) == '=' ? eat2(TokKind::NotEq) : eat1(TokKind::Not);
      case '<':
        return peek(1) == '=' ? eat2(TokKind::Le) : eat1(TokKind::Lt);
      case '>':
        return peek(1) == '=' ? eat2(TokKind::Ge) : eat1(TokKind::Gt);
      case '&':
        if (peek(1) == '&') {
          return eat2(TokKind::AndAnd);
        }
        return eat1(TokKind::Amp);
      case '|':
        if (peek(1) == '|') {
          return eat2(TokKind::OrOr);
        }
        break;
      case '(':
        return eat1(TokKind::LParen);
      case ')':
        return eat1(TokKind::RParen);
      case '{':
        return eat1(TokKind::LBrace);
      case '}':
        return eat1(TokKind::RBrace);
      case '[':
        return eat1(TokKind::LBracket);
      case ']':
        return eat1(TokKind::RBracket);
      case ',':
        return eat1(TokKind::Comma);
      case ';':
        return eat1(TokKind::Semi);
      case '.':
        return eat1(TokKind::Dot);
      case '@':
        return eat1(TokKind::At);
      case '?':
        return eat1(TokKind::Question);
      case ':':
        return eat1(TokKind::Colon);
      default:
        break;
    }
    error_kind = "Unexpected character";
    error = format_vex_error(src, start, error_kind);
    fail_off = start;
    fail_len = 1;
    return {};
  }
};

enum class ExprKind : uint8_t {
  LitInt,
  LitFloat,
  LitString,
  LitBool,
  Ident,
  Attr,
  Binary,
  Unary,
  Call,
  Member,
  Cond,
  VecLit,
  Assign,
  Postfix,
  Index,
};

struct Expr {
  ExprKind kind = ExprKind::Ident;
  Type type = Type::Void;
  Type attr_type = Type::Void;
  std::string name;
  std::string op;
  int member = 0;
  int i = 0;
  float f = 0.0f;
  bool lit_b = false;
  int src_off = 0;
  int src_len = 1;
  Expr *a = nullptr;
  Expr *b = nullptr;
  Expr *c = nullptr;
  Vector<Expr *> args;
};

static Type expr_arg_type(const Expr *e, const int i)
{
  if (!e || i >= int(e->args.size()) || !e->args[i]) {
    return Type::Void;
  }
  return e->args[i]->type;
}

static Type unary_math_result_type(const Expr *e)
{
  return unary_vec_keep(expr_arg_type(e, 0));
}

static Type binary_math_result_type(const Expr *e)
{
  const Type a = expr_arg_type(e, 0);
  const Type b = expr_arg_type(e, 1);
  const Type vec = dominant_vec_type(a, b);
  if (vec != Type::Void) {
    return vec;
  }
  if (a == Type::Int && b == Type::Int) {
    return Type::Int;
  }
  return Type::Float;
}

static Type cross_result_type(const Expr *e)
{
  const Type a = expr_arg_type(e, 0);
  const Type b = expr_arg_type(e, 1);
  if (a == Type::Vector2 || b == Type::Vector2) {
    return Type::Float;
  }
  if (a == Type::Vector4 || b == Type::Vector4) {
    return Type::Vector4;
  }
  return Type::Vector;
}

enum class StmtKind : uint8_t {
  Empty,
  Expr,
  Block,
  If,
  For,
  While,
  Break,
  Continue,
  Return,
  VarDecl,
  FnDef,
};

struct Stmt {
  StmtKind kind = StmtKind::Empty;
  Type var_type = Type::Void;
  std::string var_name;
  Expr *expr = nullptr;
  Stmt *s0 = nullptr;
  Stmt *s1 = nullptr;
  Vector<Stmt *> body;
  Vector<Type> param_types;
  Vector<std::string> param_names;
  int src_off = 0;
  int src_len = 1;
};

struct Parser {
  Lexer lex;
  Tok cur;
  Vector<std::unique_ptr<Expr>> expr_pool;
  Vector<std::unique_ptr<Stmt>> stmt_pool;
  std::string error;
  Vector<Diagnostic> diags;
  Vector<Stmt *> fns;
  int foreach_uid = 0;

  explicit Parser(const StringRef src) : lex{src}
  {
    cur = lex.next();
    if (!lex.error.empty()) {
      error = lex.error;
      diags.append(make_diag(lex.fail_off,
                             lex.fail_len,
                             true,
                             lex.error_kind.empty() ? StringRef("Unexpected character") :
                                                      StringRef(lex.error_kind)));
    }
  }

  Expr *new_expr()
  {
    expr_pool.append(std::make_unique<Expr>());
    Expr *e = expr_pool.last().get();
    if (cur.text.data() && lex.src.data()) {
      e->src_off = int(cur.text.data() - lex.src.data());
      e->src_len = std::max(1, int(cur.text.size()));
    }
    return e;
  }
  Stmt *new_stmt()
  {
    stmt_pool.append(std::make_unique<Stmt>());
    Stmt *s = stmt_pool.last().get();
    if (cur.text.data() && lex.src.data()) {
      s->src_off = int(cur.text.data() - lex.src.data());
      s->src_len = std::max(1, int(cur.text.size()));
    }
    return s;
  }

  bool ok() const
  {
    return error.empty();
  }

  void fail(const StringRef msg)
  {
    if (!error.empty()) {
      return;
    }
    const int64_t off = cur.text.data() ? int64_t(cur.text.data() - lex.src.data()) : lex.i;
    const int64_t len = cur.text.data() ? std::max<int64_t>(1, cur.text.size()) : 1;
    error = format_vex_error(lex.src, off, msg);
    diags.append(make_diag(off, len, true, msg));
  }

  bool eat(const TokKind k)
  {
    if (cur.kind != k) {
      return false;
    }
    cur = lex.next();
    if (!lex.error.empty()) {
      error = lex.error;
      if (diags.is_empty()) {
        diags.append(make_diag(lex.fail_off,
                               lex.fail_len,
                               true,
                               lex.error_kind.empty() ? StringRef("Unexpected character") :
                                                        StringRef(lex.error_kind)));
      }
    }
    return true;
  }

  bool expect(const TokKind k, const char *what)
  {
    if (eat(k)) {
      return true;
    }
    fail(what);
    return false;
  }

  Type eat_type_kw()
  {
    switch (cur.kind) {
      case TokKind::KwFloat:
        eat(cur.kind);
        return Type::Float;
      case TokKind::KwInt:
        eat(cur.kind);
        return Type::Int;
      case TokKind::KwBool:
        eat(cur.kind);
        return Type::Bool;
      case TokKind::KwVector:
        eat(cur.kind);
        return Type::Vector;
      case TokKind::KwVector2:
        eat(cur.kind);
        return Type::Vector2;
      case TokKind::KwVector4:
        eat(cur.kind);
        return Type::Vector4;
      case TokKind::KwMatrix:
        eat(cur.kind);
        return Type::Matrix;
      case TokKind::KwMatrix2:
        eat(cur.kind);
        return Type::Matrix2;
      case TokKind::KwMatrix3:
        eat(cur.kind);
        return Type::Matrix3;
      case TokKind::KwRotation:
        eat(cur.kind);
        return Type::Rotation;
      case TokKind::KwString:
        eat(cur.kind);
        return Type::String;
      case TokKind::KwColor:
        eat(cur.kind);
        return Type::Color;
      case TokKind::KwRay:
        eat(cur.kind);
        return Type::Ray;
      case TokKind::KwVoid:
        eat(cur.kind);
        return Type::Void;
      default:
        return Type::Void;
    }
  }

  Type eat_type_maybe_array()
  {
    const Type t = eat_type_kw();
    if (t == Type::Void) {
      return t;
    }
    if (eat(TokKind::LBracket)) {
      if (cur.kind == TokKind::Number) {
        eat(TokKind::Number);
      }
      expect(TokKind::RBracket, "Expected ']'");
      return array_type_of(t);
    }
    return t;
  }

  /* `int pts[]` / `int pts[8]` after the name. `int[] pts` is handled above. */
  void eat_name_array_suffix(Type &ty)
  {
    if (type_is_array(ty) || !eat(TokKind::LBracket)) {
      return;
    }
    if (cur.kind == TokKind::Number) {
      eat(TokKind::Number);
    }
    expect(TokKind::RBracket, "Expected ']'");
    ty = array_type_of(ty);
  }

  Expr *parse_primary()
  {
    if (!ok()) {
      return nullptr;
    }
    if (cur.kind == TokKind::Number) {
      Expr *e = new_expr();
      const StringRef t = cur.text;
      if (cur.is_float) {
        e->kind = ExprKind::LitFloat;
        e->type = Type::Float;
        const std::string tmp(t);
        e->f = float(std::atof(tmp.c_str()));
      }
      else {
        e->kind = ExprKind::LitInt;
        e->type = Type::Int;
        const std::string tmp(t);
        e->i = int(std::atoi(tmp.c_str()));
      }
      eat(TokKind::Number);
      return e;
    }
    if (cur.kind == TokKind::String) {
      Expr *e = new_expr();
      e->kind = ExprKind::LitString;
      e->type = Type::String;
      StringRef t = cur.text;
      if (t.size() >= 2) {
        std::string inner;
        inner.reserve(size_t(t.size() - 2));
        for (int64_t k = 1; k < t.size() - 1; k++) {
          if (t[k] == '\\' && k + 1 < t.size() - 1) {
            const char n = t[k + 1];
            if (n == 'n') {
              inner += '\n';
            }
            else if (n == 't') {
              inner += '\t';
            }
            else {
              inner += n;
            }
            k++;
          }
          else {
            inner += t[k];
          }
        }
        e->name = std::move(inner);
      }
      eat(TokKind::String);
      return e;
    }
    if (cur.kind == TokKind::Ident && (cur.text == "true" || cur.text == "false")) {
      Expr *e = new_expr();
      e->kind = ExprKind::LitBool;
      e->type = Type::Bool;
      e->lit_b = cur.text == "true";
      eat(TokKind::Ident);
      return e;
    }
    if (cur.kind == TokKind::LBrace) {
      eat(TokKind::LBrace);
      Expr *e = new_expr();
      e->kind = ExprKind::VecLit;
      e->type = Type::Vector;
      if (!eat(TokKind::RBrace)) {
        while (ok()) {
          Expr *c = parse_expr();
          if (!c) {
            break;
          }
          e->args.append(c);
          if (eat(TokKind::RBrace)) {
            break;
          }
          if (!expect(TokKind::Comma, "Expected ',' in vector literal")) {
            break;
          }
        }
      }
      return e;
    }
    if (cur.kind == TokKind::LParen) {
      eat(TokKind::LParen);
      if (eat(TokKind::RParen)) {
        fail("Empty parentheses");
        return nullptr;
      }
      Expr *first = parse_expr();
      if (eat(TokKind::Comma)) {
        /* `(vec3, float)` / `(vec2, vec2)` pack into a vector/matrix constructor. */
        Expr *e = new_expr();
        e->kind = ExprKind::VecLit;
        if (first) {
          e->args.append(first);
        }
        while (ok()) {
          Expr *a = parse_expr();
          if (a) {
            e->args.append(a);
          }
          if (eat(TokKind::RParen)) {
            break;
          }
          if (!expect(TokKind::Comma, "Expected ',' in vector list")) {
            break;
          }
        }
        return e;
      }
      expect(TokKind::RParen, "Expected ')'");
      return first;
    }
    /* `matrix(...)` / `float(...)` constructors: type keywords are not Ident. */
    if (ELEM(cur.kind,
             TokKind::KwFloat,
             TokKind::KwInt,
             TokKind::KwBool,
             TokKind::KwVector,
             TokKind::KwVector2,
             TokKind::KwVector4,
             TokKind::KwMatrix,
             TokKind::KwMatrix2,
             TokKind::KwMatrix3,
             TokKind::KwRotation,
             TokKind::KwString,
             TokKind::KwColor))
    {
      lex.skip();
      if (lex.peek() == '(') {
        const Type ty = eat_type_kw();
        eat(TokKind::LParen);
        Expr *e = new_expr();
        e->kind = ExprKind::Call;
        e->name = type_name(ty);
        if (!eat(TokKind::RParen)) {
          while (ok()) {
            Expr *a = parse_expr();
            if (!a) {
              break;
            }
            e->args.append(a);
            if (eat(TokKind::RParen)) {
              break;
            }
            if (!expect(TokKind::Comma, "Expected ',' in call")) {
              break;
            }
          }
        }
        return e;
      }
    }
    /* Optional Houdini-style out-parameter prefix: bounding_box(&min, &max). */
    eat(TokKind::Amp);
    /* f@name / 2@name / f[]@name / @name */
    if ((cur.kind == TokKind::Ident && cur.text.size() == 1 && peek_at_after_ident()) ||
        (cur.kind == TokKind::Number && !cur.is_float && cur.text.size() == 1 &&
         (cur.text[0] == '2' || cur.text[0] == '3' || cur.text[0] == '4') &&
         peek_at_after_ident()) ||
        cur.kind == TokKind::At)
    {
      return parse_attr();
    }
    if (cur.kind == TokKind::Ident) {
      Expr *e = new_expr();
      e->kind = ExprKind::Ident;
      e->name = std::string(cur.text);
      eat(TokKind::Ident);
      if (eat(TokKind::LParen)) {
        e->kind = ExprKind::Call;
        if (!eat(TokKind::RParen)) {
          while (ok()) {
            Expr *a = parse_expr();
            if (!a) {
              break;
            }
            e->args.append(a);
            if (eat(TokKind::RParen)) {
              break;
            }
            if (!expect(TokKind::Comma, "Expected ',' in call")) {
              break;
            }
          }
        }
      }
      return e;
    }
    fail("Expected expression");
    return nullptr;
  }

  bool peek_at_after_ident()
  {
    lex.skip();
    if (lex.peek() == '@') {
      return true;
    }
    if (lex.peek() == '[') {
      int64_t k = 1;
      while (lex.peek(k) == ' ' || lex.peek(k) == '\t') {
        k++;
      }
      if (lex.peek(k) != ']') {
        return false;
      }
      k++;
      while (lex.peek(k) == ' ' || lex.peek(k) == '\t') {
        k++;
      }
      return lex.peek(k) == '@';
    }
    return false;
  }

  Expr *parse_attr()
  {
    Type ptype = Type::Void;
    if (cur.kind == TokKind::Ident && cur.text.size() == 1) {
      ptype = prefix_type(cur.text[0]);
      eat(TokKind::Ident);
    }
    else if (cur.kind == TokKind::Number && !cur.is_float && cur.text.size() == 1 &&
             (cur.text[0] == '2' || cur.text[0] == '3' || cur.text[0] == '4'))
    {
      ptype = prefix_type(cur.text[0]);
      eat(TokKind::Number);
    }
    bool is_arr = false;
    if (eat(TokKind::LBracket)) {
      expect(TokKind::RBracket, "Expected ']'");
      is_arr = true;
    }
    if (!expect(TokKind::At, "Expected '@'")) {
      return nullptr;
    }
    if (cur.kind != TokKind::Ident) {
      fail("Expected attribute name after '@'");
      return nullptr;
    }
    Expr *e = new_expr();
    e->kind = ExprKind::Attr;
    e->name = std::string(cur.text);
    if (is_arr) {
      ptype = (ptype == Type::Void) ? Type::FloatArray : array_type_of(ptype);
    }
    if (ptype == Type::Void) {
      if (e->name == "P" || e->name == "N") {
        ptype = Type::Vector;
      }
      else {
        ptype = Type::Float;
      }
    }
    e->attr_type = ptype;
    e->type = ptype;
    eat(TokKind::Ident);
    return e;
  }

  Expr *parse_postfix()
  {
    Expr *e = parse_primary();
    while (e && ok()) {
      if (eat(TokKind::Dot)) {
        if (cur.kind != TokKind::Ident) {
          fail("Expected member name");
          return e;
        }
        Expr *m = new_expr();
        m->kind = ExprKind::Member;
        m->a = e;
        m->name = std::string(cur.text);
        int axes[4] = {0, 0, 0, 0};
        int n = 0;
        if (!parse_swizzle(cur.text, axes, n)) {
          fail("Unknown member");
        }
        else {
          m->member = pack_swizzle(axes, n);
          m->type = swizzle_result_type(n);
        }
        eat(TokKind::Ident);
        e = m;
        continue;
      }
      if (eat(TokKind::LBracket)) {
        Expr *idx = new_expr();
        idx->kind = ExprKind::Index;
        idx->a = e;
        idx->b = parse_expr();
        expect(TokKind::RBracket, "Expected ']'");
        e = idx;
        continue;
      }
      if (cur.kind == TokKind::PlusPlus || cur.kind == TokKind::MinusMinus) {
        Expr *p = new_expr();
        p->kind = ExprKind::Postfix;
        p->op = cur.kind == TokKind::PlusPlus ? "++" : "--";
        p->a = e;
        eat(cur.kind);
        e = p;
        continue;
      }
      break;
    }
    return e;
  }

  Expr *parse_unary()
  {
    if (cur.kind == TokKind::Minus || cur.kind == TokKind::Not || cur.kind == TokKind::Plus) {
      const std::string op = std::string(cur.text);
      eat(cur.kind);
      Expr *e = new_expr();
      e->kind = ExprKind::Unary;
      e->op = op;
      e->a = parse_unary();
      return e;
    }
    return parse_postfix();
  }

  Expr *parse_bin(const int min_prec)
  {
    Expr *left = parse_unary();
    while (left && ok()) {
      int prec = 0;
      std::string op;
      TokKind k = cur.kind;
      if (k == TokKind::StarStar) {
        prec = 60;
        op = "**";
      }
      else if (k == TokKind::Star || k == TokKind::Slash || k == TokKind::Percent) {
        prec = 50;
        op = std::string(cur.text);
      }
      else if (k == TokKind::Plus || k == TokKind::Minus) {
        prec = 40;
        op = std::string(cur.text);
      }
      else if (k == TokKind::Lt || k == TokKind::Gt || k == TokKind::Le || k == TokKind::Ge) {
        prec = 30;
        op = std::string(cur.text);
      }
      else if (k == TokKind::EqEq || k == TokKind::NotEq) {
        prec = 25;
        op = std::string(cur.text);
      }
      else if (k == TokKind::AndAnd) {
        prec = 20;
        op = "&&";
      }
      else if (k == TokKind::OrOr) {
        prec = 15;
        op = "||";
      }
      else {
        break;
      }
      if (prec < min_prec) {
        break;
      }
      const int op_off = cur.text.data() ? int(cur.text.data() - lex.src.data()) : 0;
      const int op_len = std::max(1, int(cur.text.size()));
      eat(k);
      const int next_min = (op == "**") ? prec : prec + 1;
      Expr *right = parse_bin(next_min);
      Expr *b = new_expr();
      b->kind = ExprKind::Binary;
      b->op = op;
      b->src_off = op_off;
      b->src_len = op_len;
      b->a = left;
      b->b = right;
      left = b;
    }
    return left;
  }

  Expr *parse_cond()
  {
    Expr *e = parse_bin(0);
    if (e && eat(TokKind::Question)) {
      Expr *t = parse_expr();
      expect(TokKind::Colon, "Expected ':'");
      Expr *f = parse_cond();
      Expr *c = new_expr();
      c->kind = ExprKind::Cond;
      c->a = e;
      c->b = t;
      c->c = f;
      return c;
    }
    return e;
  }

  bool is_assign_op(const TokKind k) const
  {
    return ELEM(k, TokKind::Eq, TokKind::PlusEq, TokKind::MinusEq, TokKind::StarEq, TokKind::SlashEq);
  }

  Expr *parse_expr()
  {
    Expr *e = parse_cond();
    if (e && is_assign_op(cur.kind)) {
      const std::string op = std::string(cur.text);
      eat(cur.kind);
      Expr *rhs = parse_expr();
      Expr *a = new_expr();
      a->kind = ExprKind::Assign;
      a->op = op;
      a->a = e;
      a->b = rhs;
      return a;
    }
    return e;
  }

  Expr *make_ident(const std::string &name)
  {
    Expr *e = new_expr();
    e->kind = ExprKind::Ident;
    e->name = name;
    return e;
  }

  Expr *make_int(const int v)
  {
    Expr *e = new_expr();
    e->kind = ExprKind::LitInt;
    e->i = v;
    e->type = Type::Int;
    return e;
  }

  bool parse_foreach_name(Type &r_type, std::string &r_name)
  {
    r_type = eat_type_maybe_array();
    if (cur.kind != TokKind::Ident) {
      if (r_type != Type::Void) {
        fail("Expected variable name");
      }
      return false;
    }
    r_name = std::string(cur.text);
    eat(TokKind::Ident);
    return true;
  }

  /* Houdini: foreach (int pt; pts) / foreach (int idx; int pt; pts)
   * and untyped foreach (pt; pts) / foreach (idx; pt; pts). */
  Stmt *parse_foreach()
  {
    expect(TokKind::LParen, "Expected '('");
    Type t1 = Type::Void;
    std::string n1;
    if (!parse_foreach_name(t1, n1)) {
      fail("Expected variable in foreach");
      return new_stmt();
    }
    expect(TokKind::Semi, "Expected ';'");

    std::string idx_name;
    std::string val_name;
    Type val_type = t1;
    const Type t2 = eat_type_maybe_array();
    if (t2 != Type::Void) {
      if (cur.kind != TokKind::Ident) {
        fail("Expected variable name");
        return new_stmt();
      }
      idx_name = n1;
      val_name = std::string(cur.text);
      val_type = t2;
      eat(TokKind::Ident);
      expect(TokKind::Semi, "Expected ';'");
    }
    else if (cur.kind == TokKind::Ident) {
      /* Look ahead: `foreach (idx; pt; arr)` vs `foreach (pt; arr)`. */
      const Lexer saved_lex = lex;
      const Tok saved_cur = cur;
      const std::string maybe = std::string(cur.text);
      eat(TokKind::Ident);
      if (cur.kind == TokKind::Semi) {
        idx_name = n1;
        val_name = maybe;
        val_type = Type::Void;
        eat(TokKind::Semi);
      }
      else {
        lex = saved_lex;
        cur = saved_cur;
        idx_name = "__foreach_i_" + std::to_string(foreach_uid++);
        val_name = n1;
      }
    }
    else {
      idx_name = "__foreach_i_" + std::to_string(foreach_uid++);
      val_name = n1;
    }
    Expr *arr_expr = parse_expr();
    expect(TokKind::RParen, "Expected ')'");
    Stmt *body = parse_stmt();

    const std::string arr_name = "__foreach_a_" + std::to_string(foreach_uid++);

    Stmt *arr_decl = new_stmt();
    arr_decl->kind = StmtKind::VarDecl;
    if (val_type == Type::Void) {
      arr_decl->var_type = Type::Void;
    }
    else if (type_is_array(val_type)) {
      arr_decl->var_type = val_type;
    }
    else {
      arr_decl->var_type = array_type_of(val_type);
    }
    arr_decl->var_name = arr_name;
    arr_decl->expr = arr_expr;

    Stmt *init = new_stmt();
    init->kind = StmtKind::VarDecl;
    init->var_type = Type::Int;
    init->var_name = idx_name;
    init->expr = make_int(0);

    Expr *len_call = new_expr();
    len_call->kind = ExprKind::Call;
    len_call->name = "len";
    len_call->args.append(make_ident(arr_name));

    Expr *cond = new_expr();
    cond->kind = ExprKind::Binary;
    cond->op = "<";
    cond->a = make_ident(idx_name);
    cond->b = len_call;

    Expr *incr_e = new_expr();
    incr_e->kind = ExprKind::Postfix;
    incr_e->op = "++";
    incr_e->a = make_ident(idx_name);
    Stmt *incr = new_stmt();
    incr->kind = StmtKind::Expr;
    incr->expr = incr_e;

    Expr *idx_e = new_expr();
    idx_e->kind = ExprKind::Index;
    idx_e->a = make_ident(arr_name);
    idx_e->b = make_ident(idx_name);
    Stmt *val_decl = new_stmt();
    val_decl->kind = StmtKind::VarDecl;
    val_decl->var_type = val_type;
    val_decl->var_name = val_name;
    val_decl->expr = idx_e;

    Stmt *inner = new_stmt();
    inner->kind = StmtKind::Block;
    inner->body.append(val_decl);
    if (body) {
      inner->body.append(body);
    }

    Stmt *for_s = new_stmt();
    for_s->kind = StmtKind::For;
    for_s->s0 = init;
    for_s->expr = cond;
    for_s->s1 = incr;
    for_s->body.append(inner);

    Stmt *block = new_stmt();
    block->kind = StmtKind::Block;
    block->body.append(arr_decl);
    block->body.append(for_s);
    return block;
  }

  Stmt *parse_stmt()
  {
    if (!ok()) {
      return nullptr;
    }
    if (eat(TokKind::Semi)) {
      return new_stmt();
    }
    if (eat(TokKind::LBrace)) {
      Stmt *s = new_stmt();
      s->kind = StmtKind::Block;
      while (ok() && cur.kind != TokKind::RBrace && cur.kind != TokKind::Eof) {
        Stmt *inner = parse_stmt();
        if (inner) {
          s->body.append(inner);
        }
        else {
          break;
        }
      }
      expect(TokKind::RBrace, "Expected '}'");
      return s;
    }
    if (eat(TokKind::KwIf)) {
      Stmt *s = new_stmt();
      s->kind = StmtKind::If;
      expect(TokKind::LParen, "Expected '('");
      s->expr = parse_expr();
      expect(TokKind::RParen, "Expected ')'");
      s->s0 = parse_stmt();
      if (eat(TokKind::KwElse)) {
        s->s1 = parse_stmt();
      }
      return s;
    }
    if (eat(TokKind::KwWhile)) {
      Stmt *s = new_stmt();
      s->kind = StmtKind::While;
      expect(TokKind::LParen, "Expected '('");
      s->expr = parse_expr();
      expect(TokKind::RParen, "Expected ')'");
      s->s0 = parse_stmt();
      return s;
    }
    if (eat(TokKind::KwForeach)) {
      return parse_foreach();
    }
    if (eat(TokKind::KwFor)) {
      Stmt *s = new_stmt();
      s->kind = StmtKind::For;
      expect(TokKind::LParen, "Expected '('");
      if (!eat(TokKind::Semi)) {
        s->s0 = parse_simple_stmt_no_semi();
        expect(TokKind::Semi, "Expected ';'");
      }
      if (!eat(TokKind::Semi)) {
        s->expr = parse_expr();
        expect(TokKind::Semi, "Expected ';'");
      }
      if (!eat(TokKind::RParen)) {
        s->s1 = parse_simple_stmt_no_semi();
        expect(TokKind::RParen, "Expected ')'");
      }
      Stmt *body = parse_stmt();
      s->body.append(body);
      return s;
    }
    if (eat(TokKind::KwBreak)) {
      Stmt *s = new_stmt();
      s->kind = StmtKind::Break;
      expect(TokKind::Semi, "Expected ';'");
      return s;
    }
    if (eat(TokKind::KwContinue)) {
      Stmt *s = new_stmt();
      s->kind = StmtKind::Continue;
      expect(TokKind::Semi, "Expected ';'");
      return s;
    }
    if (eat(TokKind::KwReturn)) {
      Stmt *s = new_stmt();
      s->kind = StmtKind::Return;
      if (!eat(TokKind::Semi)) {
        s->expr = parse_expr();
        expect(TokKind::Semi, "Expected ';'");
      }
      return s;
    }
    const Type ty = eat_type_maybe_array();
    if (ty != Type::Void) {
      return parse_var_decl_list(ty, true);
    }
    Stmt *s = new_stmt();
    s->kind = StmtKind::Expr;
    s->expr = parse_expr();
    expect(TokKind::Semi, "Expected ';'");
    return s;
  }

  Stmt *parse_one_var_decl(const Type base_ty)
  {
    Stmt *s = new_stmt();
    s->kind = StmtKind::VarDecl;
    s->var_type = base_ty;
    if (cur.kind != TokKind::Ident) {
      fail("Expected variable name");
      return s;
    }
    s->var_name = std::string(cur.text);
    eat(TokKind::Ident);
    eat_name_array_suffix(s->var_type);
    if (eat(TokKind::Eq)) {
      s->expr = parse_expr();
    }
    return s;
  }

  /* `vector a, b, c;` / `float a = 0.1, b = 0.2;` — extras live on first->body
   * (not a Block, which would pop the names at the end of the statement). */
  Stmt *parse_var_decl_list(const Type base_ty, const bool need_semi)
  {
    Stmt *first = parse_one_var_decl(base_ty);
    while (eat(TokKind::Comma)) {
      first->body.append(parse_one_var_decl(base_ty));
    }
    if (need_semi) {
      expect(TokKind::Semi, "Expected ';'");
    }
    return first;
  }

  Stmt *parse_simple_stmt_no_semi()
  {
    const Type ty = eat_type_maybe_array();
    if (ty != Type::Void) {
      return parse_var_decl_list(ty, false);
    }
    Stmt *s = new_stmt();
    s->kind = StmtKind::Expr;
    s->expr = parse_expr();
    return s;
  }

  bool looking_at_fn()
  {
    const Lexer saved_lex = lex;
    const Tok saved_cur = cur;
    if (cur.kind == TokKind::Ident && cur.text == "function") {
      eat(TokKind::Ident);
    }
    if (cur.kind == TokKind::KwVoid) {
      eat(TokKind::KwVoid);
    }
    else if (eat_type_maybe_array() == Type::Void) {
      lex = saved_lex;
      cur = saved_cur;
      return false;
    }
    if (cur.kind != TokKind::Ident) {
      lex = saved_lex;
      cur = saved_cur;
      return false;
    }
    eat(TokKind::Ident);
    const bool yes = cur.kind == TokKind::LParen;
    lex = saved_lex;
    cur = saved_cur;
    return yes;
  }

  Stmt *parse_fn_def()
  {
    if (cur.kind == TokKind::Ident && cur.text == "function") {
      eat(TokKind::Ident);
    }
    Type ret = Type::Void;
    if (cur.kind == TokKind::KwVoid) {
      eat(TokKind::KwVoid);
      ret = Type::Void;
    }
    else {
      ret = eat_type_maybe_array();
    }
    Stmt *s = new_stmt();
    s->kind = StmtKind::FnDef;
    s->var_type = ret;
    if (cur.kind != TokKind::Ident) {
      fail("Expected function name");
      return s;
    }
    s->var_name = std::string(cur.text);
    eat(TokKind::Ident);
    expect(TokKind::LParen, "Expected '('");
    /* Houdini VEX: `vector2 A, B, C` and `vector2 A, B; float t`.
     * C-style `int a, int b` still works (type after a comma starts a new group). */
    Type group_ty = Type::Void;
    while (ok() && cur.kind != TokKind::RParen && cur.kind != TokKind::Eof) {
      if (eat(TokKind::Semi)) {
        group_ty = Type::Void;
        continue;
      }
      const Type pt = eat_type_maybe_array();
      if (pt != Type::Void) {
        group_ty = pt;
      }
      else if (group_ty == Type::Void) {
        fail("Expected parameter type");
        break;
      }
      if (cur.kind != TokKind::Ident) {
        fail("Expected parameter name");
        break;
      }
      s->param_types.append(group_ty);
      s->param_names.append(std::string(cur.text));
      eat(TokKind::Ident);
      eat_name_array_suffix(s->param_types.last());
      if (eat(TokKind::Comma)) {
        continue;
      }
      if (eat(TokKind::Semi)) {
        group_ty = Type::Void;
        continue;
      }
      break;
    }
    expect(TokKind::RParen, "Expected ')'");
    s->s0 = parse_stmt();
    return s;
  }

  Vector<Stmt *> parse_program()
  {
    Vector<Stmt *> stmts;
    while (ok() && cur.kind != TokKind::Eof) {
      if (looking_at_fn()) {
        fns.append(parse_fn_def());
        continue;
      }
      Stmt *s = parse_stmt();
      if (s) {
        stmts.append(s);
      }
      else {
        break;
      }
    }
    return stmts;
  }
};

/* -------------------------------------------------------------------- */
/** \name Compiler
 * \{ */

struct Local {
  std::string name;
  Type type = Type::Float;
  int slot = 0;
};

struct Compiler {
  Program prog;
  std::string error;
  std::string warning;
  Vector<Diagnostic> diags;
  Vector<Local> locals;
  Vector<int> scope_stack;
  Vector<int> loop_break;
  Vector<int> loop_continue;
  /** Attribute name → local slot holding the last StoreAttr value (no PushAttr). */
  Map<std::string, int> attr_local;
  Parser *parser = nullptr;
  bool shader_material = false;
  /** Innermost AST node being compiled. Semantic diagnostics must not use parser.cur, which is at
   * EOF after parsing has completed. */
  const Expr *error_expr = nullptr;
  const Stmt *error_stmt = nullptr;

  void warn(const StringRef msg)
  {
    if (msg.is_empty()) {
      return;
    }
    if (!warning.empty()) {
      warning += "\n";
    }
    warning += msg;
  }

  void warn_at(const Expr *e, const StringRef msg)
  {
    warn(msg);
    const int off = e ? e->src_off : 0;
    const int len = e ? std::max(1, e->src_len) : 1;
    diags.append(make_diag(off, len, false, msg));
  }

  void fail(const StringRef msg)
  {
    if (!error.empty()) {
      return;
    }
    const StringRef src = parser ? parser->lex.src : StringRef();
    const int64_t off = error_expr ? error_expr->src_off :
                        error_stmt ? error_stmt->src_off :
                        parser && parser->cur.text.data() ?
                            int64_t(parser->cur.text.data() - parser->lex.src.data()) :
                            (parser ? parser->lex.i : 0);
    const int64_t len = error_expr ? std::max(1, error_expr->src_len) :
                        error_stmt ? std::max(1, error_stmt->src_len) :
                        parser && parser->cur.text.data() ?
                            std::max<int64_t>(1, parser->cur.text.size()) :
                            1;
    error = src.is_empty() ? std::string(msg) : format_vex_error(src, off, msg);
    diags.append(make_diag(off, len, true, msg));
  }

  void fail_at(const Expr *e, const StringRef msg)
  {
    if (!error.empty()) {
      return;
    }
    const StringRef src = parser ? parser->lex.src : StringRef();
    const int64_t off = e ? e->src_off : 0;
    const int64_t len = e ? std::max(1, e->src_len) : 1;
    error = src.is_empty() ? std::string(msg) : format_vex_error(src, off, msg);
    diags.append(make_diag(off, len, true, msg));
  }

  void emit(const Op op, const int32_t imm = 0)
  {
    Inst inst;
    inst.op = op;
    inst.imm = imm;
    prog.code.append(inst);
  }

  int emit_jmp(const Op op)
  {
    const int i = int(prog.code.size());
    emit(op, 0);
    return i;
  }

  void patch(const int at, const int target)
  {
    prog.code[at].imm = target;
  }

  int add_const_f(const float f)
  {
    const int i = int(prog.const_f.size());
    prog.const_f.append(f);
    return i;
  }
  int add_const_i(const int n)
  {
    const int i = int(prog.const_i.size());
    prog.const_i.append(n);
    return i;
  }
  int add_const_v(const float3 v)
  {
    const int i = int(prog.const_v.size());
    prog.const_v.append(v);
    return i;
  }
  int add_const_s(std::string s)
  {
    const int i = int(prog.const_s.size());
    prog.const_s.append(std::move(s));
    return i;
  }

  void push_scope()
  {
    scope_stack.append(int(locals.size()));
  }
  void pop_scope()
  {
    if (!scope_stack.is_empty()) {
      locals.resize(scope_stack.pop_last());
    }
  }

  Local *find_local(const StringRef name)
  {
    for (int i = int(locals.size()) - 1; i >= 0; i--) {
      if (locals[i].name == name) {
        return &locals[i];
      }
    }
    return nullptr;
  }

  Type ident_fallback_type(const StringRef name) const
  {
    const std::string aname = canonical_attr_name(name);
    for (const AttrInfo &a : prog.attrs) {
      if (a.name == aname) {
        return a.type;
      }
    }
    if (aname == "position" || aname == "normal" || aname == "Output") {
      return Type::Vector;
    }
    if (shader_material) {
      return Type::Vector;
    }
    return Type::Float;
  }

  void declare_out_if_needed(Expr *e, const Type t)
  {
    if (!e || e->kind != ExprKind::Ident) {
      return;
    }
    if (is_element_index_name(e->name) || find_local(e->name)) {
      return;
    }
    Local loc;
    loc.name = e->name;
    loc.type = t;
    loc.slot = prog.local_count++;
    locals.append(loc);
  }

  int attr_slot(const StringRef name, const Type type, const bool write)
  {
    for (const int i : prog.attrs.index_range()) {
      if (prog.attrs[i].name == name) {
        if (type != Type::Void && prog.attrs[i].type == Type::Void) {
          prog.attrs[i].type = type;
        }
        if (type_is_array(type) && !type_is_array(prog.attrs[i].type)) {
          prog.attrs[i].type = type;
        }
        if (write) {
          prog.attrs[i].write = true;
        }
        else {
          prog.attrs[i].read = true;
        }
        return i;
      }
    }
    AttrInfo info;
    info.name = std::string(name);
    info.type = type == Type::Void ? Type::Float : type;
    info.read = !write;
    info.write = write;
    prog.attrs.append(std::move(info));
    return int(prog.attrs.size()) - 1;
  }

  int ensure_attr_local(const StringRef aname)
  {
    const std::string key(aname);
    if (const int *s = attr_local.lookup_ptr(key)) {
      return *s;
    }
    const int slot = prog.local_count++;
    attr_local.add(key, slot);
    return slot;
  }

  void emit_store_attr(const std::string &aname, const Type t)
  {
    const int slot = attr_slot(aname, t, true);
    emit(Op::Dup);
    emit(Op::StoreAttr, slot);
    emit(Op::StoreLocal, ensure_attr_local(aname));
  }

  void emit_push_attr(const std::string &aname, const Type t)
  {
    if (const int *s = attr_local.lookup_ptr(aname)) {
      emit(Op::PushLocal, *s);
      return;
    }
    emit(Op::PushAttr, attr_slot(aname, t, false));
  }

  void drop_attr_local(const int slot)
  {
    if (slot < 0 || slot >= prog.attrs.size()) {
      return;
    }
    attr_local.remove(prog.attrs[slot].name);
  }

  int builtin_from_name(const StringRef name)
  {
    static const Map<std::string, int> table = []() {
      Map<std::string, int> m;
      m.add("sin", int(Builtin::Sin));
      m.add("cos", int(Builtin::Cos));
      m.add("tan", int(Builtin::Tan));
      m.add("asin", int(Builtin::Asin));
      m.add("acos", int(Builtin::Acos));
      m.add("atan", int(Builtin::Atan));
      m.add("atan2", int(Builtin::Atan2));
      m.add("abs", int(Builtin::Abs));
      m.add("floor", int(Builtin::Floor));
      m.add("ceil", int(Builtin::Ceil));
      m.add("round", int(Builtin::Round));
      m.add("trunc", int(Builtin::Trunc));
      m.add("sqrt", int(Builtin::Sqrt));
      m.add("exp", int(Builtin::Exp));
      m.add("log", int(Builtin::Log));
      m.add("pow", int(Builtin::Pow));
      m.add("min", int(Builtin::Min));
      m.add("max", int(Builtin::Max));
      m.add("clamp", int(Builtin::Clamp));
      m.add("length", int(Builtin::Length));
      m.add("distance", int(Builtin::Distance));
      m.add("dot", int(Builtin::Dot));
      m.add("cross", int(Builtin::Cross));
      m.add("normalize", int(Builtin::Normalize));
      m.add("radians", int(Builtin::Radians));
      m.add("degrees", int(Builtin::Degrees));
      m.add("sign", int(Builtin::Sign));
      m.add("frac", int(Builtin::Fract));
      m.add("fract", int(Builtin::Fract));
      m.add("ddx", int(Builtin::Ddx));
      m.add("dFdx", int(Builtin::Ddx));
      m.add("ddy", int(Builtin::Ddy));
      m.add("dFdy", int(Builtin::Ddy));
      m.add("fwidth", int(Builtin::Fwidth));
      m.add("mix", int(Builtin::Mix));
      m.add("lerp", int(Builtin::Mix));
      m.add("map", int(Builtin::Map));
      m.add("smooth", int(Builtin::Smooth));
      m.add("noise", int(Builtin::Noise));
      m.add("hash", int(Builtin::Hash));
      m.add("rand", int(Builtin::Rand));
      m.add("npoints", int(Builtin::Npoints));
      m.add("nfaces", int(Builtin::Nfaces));
      m.add("nedges", int(Builtin::Nedges));
      m.add("ncorners", int(Builtin::Ncorners));
      m.add("point", int(Builtin::Point));
      m.add("edge", int(Builtin::Edge));
      m.add("face", int(Builtin::Face));
      m.add("corner", int(Builtin::Corner));
      m.add("curve", int(Builtin::Curve));
      m.add("instance", int(Builtin::Instance));
      m.add("addpoint", int(Builtin::Addpoint));
      m.add("addprim", int(Builtin::Addprim));
      m.add("add_prim", int(Builtin::Addprim));
      m.add("setattribute", int(Builtin::SetAttribute));
      m.add("set", int(Builtin::Set));
      m.add("float", int(Builtin::FloatFn));
      m.add("int", int(Builtin::IntFn));
      m.add("bool", int(Builtin::BoolFn));
      m.add("vec3", int(Builtin::Vec3Fn));
      m.add("vector", int(Builtin::Vec3Fn));
      m.add("vec2", int(Builtin::Vec2Fn));
      m.add("vector2", int(Builtin::Vec2Fn));
      m.add("vec4", int(Builtin::Vec4Fn));
      m.add("vector4", int(Builtin::Vec4Fn));
      m.add("mat2", int(Builtin::Matrix2Fn));
      m.add("matrix2", int(Builtin::Matrix2Fn));
      m.add("mat3", int(Builtin::Matrix3Fn));
      m.add("matrix3", int(Builtin::Matrix3Fn));
      m.add("mat4", int(Builtin::MatrixFn));
      m.add("matrix4", int(Builtin::MatrixFn));
      m.add("svd", int(Builtin::SvdFn));
      m.add("polardecomp", int(Builtin::PolarDecompFn));
      m.add("eigendecomp", int(Builtin::EigenFn));
      m.add("invert", int(Builtin::Invert));
      m.add("rotate_rotation", int(Builtin::RotateRotation));
      m.add("to_euler", int(Builtin::RotationFn));
      m.add("array", int(Builtin::ArrayFn));
      m.add("stringarray", int(Builtin::ArrayStr));
      m.add("append", int(Builtin::Append));
      m.add("insert", int(Builtin::Insert));
      m.add("removeindex", int(Builtin::RemoveIndex));
      m.add("removevalue", int(Builtin::RemoveValue));
      m.add("sort", int(Builtin::SortArr));
      m.add("sortorder", int(Builtin::SortOrder));
      m.add("sortbyorder", int(Builtin::SortByOrder));
      m.add("len", int(Builtin::Len));
      m.add("pointneighbours", int(Builtin::PointNeighbours));
      m.add("pointedges", int(Builtin::PointEdges));
      m.add("pointfaces", int(Builtin::PointFaces));
      m.add("pointcorners", int(Builtin::PointCorners));
      m.add("edgepoints", int(Builtin::EdgePoints));
      m.add("edgefaces", int(Builtin::EdgeFaces));
      m.add("edgecorners", int(Builtin::CornersOfEdge));
      m.add("facepoints", int(Builtin::FacePoints));
      m.add("faceedges", int(Builtin::FaceEdges));
      m.add("faceneighbours", int(Builtin::FaceNeighbours));
      m.add("facecorners", int(Builtin::FaceCorners));
      m.add("cornerface", int(Builtin::CornerFace));
      m.add("corneredges", int(Builtin::EdgesOfCorner));
      m.add("cornerpoint", int(Builtin::VertexOfCorner));
      m.add("offsetcorner", int(Builtin::OffsetCornerInFace));
      m.add("curvepoints", int(Builtin::PointsOfCurve));
      m.add("pointcurve", int(Builtin::PointCurve));
      m.add("nearestpoints", int(Builtin::NearestPoints));
      m.add("transpose", int(Builtin::Transpose));
      m.add("determinant", int(Builtin::Determinant));
      m.add("transform_point", int(Builtin::TransformPoint));
      m.add("transform_direction", int(Builtin::TransformDirection));
      m.add("project_point", int(Builtin::ProjectPoint));
      m.add("combine_transform", int(Builtin::CombineTransform));
      m.add("ident", int(Builtin::Identity));
      m.add("matrix", int(Builtin::MatrixFn));
      m.add("translation", int(Builtin::TranslationFn));
      m.add("rotation", int(Builtin::QuatFn));
      m.add("scale", int(Builtin::ScaleFn));
      m.add("chf", int(Builtin::Chf));
      m.add("chi", int(Builtin::Chi));
      m.add("chu", int(Builtin::Chu));
      m.add("chv", int(Builtin::Chv));
      m.add("chq", int(Builtin::Chq));
      m.add("chb", int(Builtin::Chb));
      m.add("chc", int(Builtin::Chc));
      m.add("ch2", int(Builtin::Ch2));
      m.add("ch3", int(Builtin::Ch3));
      m.add("ch4", int(Builtin::Ch4));
      m.add("chm", int(Builtin::Chm));
      m.add("chr", int(Builtin::Chr));
      m.add("chs", int(Builtin::Chs));
      m.add("valuetostring", int(Builtin::ValueToString));
      m.add("format", int(Builtin::Format));
      m.add("color", int(Builtin::ColorFn));
      m.add("voronoi", int(Builtin::Voronoi));
      m.add("raycast", int(Builtin::Raycast));
      m.add("raycastall", int(Builtin::RaycastAll));
      m.add("raycast_all", int(Builtin::RaycastAll));
      m.add("rayishit", int(Builtin::RayIsHit));
      m.add("rayhitpos", int(Builtin::RayHitPos));
      m.add("rayhitnormal", int(Builtin::RayHitN));
      m.add("rayhitdist", int(Builtin::RayHitDist));
      m.add("bboxmin", int(Builtin::BBoxMin));
      m.add("bboxmax", int(Builtin::BBoxMax));
      m.add("boundingbox", int(Builtin::BoundingBox));
      m.add("geometry_proximity", int(Builtin::GeometryProximity));
      m.add("sample_nearest_surface", int(Builtin::SampleNearestSurface));
      m.add("delete_geometry", int(Builtin::DeleteGeometry));
      m.add("accumulate", int(Builtin::Accumulate));
      m.add("field_average", int(Builtin::FieldAverage));
      m.add("field_min", int(Builtin::FieldMin));
      m.add("field_max", int(Builtin::FieldMax));
      m.add("field_minmax", int(Builtin::FieldMinMax));
      return m;
    }();
    const std::string key(name);
    return table.lookup_default(key, -1);
  }

  Type compile_expr(Expr *e, const bool as_stmt = false, const Type hint = Type::Void);
  void emit_coerce(Type from, Type to);
  void compile_lvalue_store(Expr *lval, Type value_type = Type::Void);
  int attr_slot_of_lvalue(Expr *lval, bool write);
  void compile_stmt(Stmt *s);
  Type compile_assign(Expr *e, bool as_stmt);
  bool try_emit_while_cmp_add(Stmt *s);
  bool try_emit_gather_samples(Stmt *s);
};

static bool expr_lit_num(const Expr *e, float &r);
static bool expr_is_p(const Expr *e);
static bool expr_const_vec3(const Expr *e, float3 &r);

void Compiler::emit_coerce(const Type from, const Type to)
{
  if (to == Type::Void || from == to) {
    return;
  }
  Builtin fn = Builtin::FloatFn;
  bool wrap = true;
  if (to == Type::Color) {
    fn = Builtin::ColorFn;
  }
  else if (to == Type::Rotation) {
    fn = Builtin::QuatFn;
  }
  else if (to == Type::Vector2) {
    fn = Builtin::Vec2Fn;
  }
  else if (to == Type::Vector) {
    fn = Builtin::Vec3Fn;
  }
  else if (to == Type::Vector4) {
    fn = Builtin::Vec4Fn;
  }
  else if (type_is_matrix(to)) {
    fn = matrix_ctor_of(to);
  }
  else {
    wrap = false;
  }
  if (wrap) {
    emit(Op::Call, int(fn));
    prog.code.last().a = 1;
  }
  (void)from;
}

Type Compiler::compile_expr(Expr *e, const bool as_stmt, const Type hint)
{
  const Expr *previous_error_expr = error_expr;
  error_expr = e;
  struct RestoreErrorExpr {
    const Expr *&slot;
    const Expr *previous;
    ~RestoreErrorExpr()
    {
      slot = previous;
    }
  } restore_error_expr{error_expr, previous_error_expr};
  if (!e) {
    fail("Empty expression");
    return Type::Void;
  }
  switch (e->kind) {
    case ExprKind::LitInt:
      emit(Op::PushI, add_const_i(e->i));
      e->type = Type::Int;
      if (type_is_matrix(hint) || type_is_vec_like(hint)) {
        emit_coerce(Type::Int, hint);
        /* Keep e->type as the literal kind. GPU walks the AST and must not
         * treat `1.0` as a splat vec just because a caller passed a vec hint. */
        return hint;
      }
      return Type::Int;
    case ExprKind::LitFloat:
      emit(Op::PushF, add_const_f(e->f));
      e->type = Type::Float;
      if (type_is_matrix(hint) || type_is_vec_like(hint)) {
        emit_coerce(Type::Float, hint);
        return hint;
      }
      return Type::Float;
    case ExprKind::LitBool:
      emit(Op::PushB, e->lit_b ? 1 : 0);
      e->type = Type::Bool;
      if (type_is_matrix(hint) || type_is_vec_like(hint)) {
        emit_coerce(Type::Bool, hint);
        return hint;
      }
      return Type::Bool;
    case ExprKind::LitString:
      emit(Op::PushS, add_const_s(e->name));
      e->type = Type::String;
      return Type::String;
    case ExprKind::Ident: {
      if (is_element_index_name(e->name)) {
        emit(Op::PushIndex);
        e->type = Type::Int;
        return Type::Int;
      }
      if (e->name == "geo" || e->name == "geoself") {
        emit(Op::PushI, add_const_i(0));
        e->type = Type::Int;
        return Type::Int;
      }
      if (e->name == "numpt" || e->name == "numelem") {
        emit(Op::PushNpoints);
        e->type = Type::Int;
        return Type::Int;
      }
      if (Local *loc = find_local(e->name)) {
        emit(Op::PushLocal, loc->slot);
        e->type = loc->type;
        if (hint != Type::Void && hint != loc->type &&
            (type_is_matrix(hint) || type_is_vec_like(hint)))
        {
          emit_coerce(loc->type, hint);
          /* Bytecode is coerced; leave e->type as the local's real type so GPU
           * emit does not rewrite `vector2 i` into a float3 splat. */
          return hint;
        }
        return e->type;
      }
      /* Bare attribute name (Houdini-style P / pscale / previously written i[]@pts). */
      const Type t = ident_fallback_type(e->name);
      const std::string aname = canonical_attr_name(e->name);
      emit_push_attr(aname, t);
      e->type = t;
      return t;
    }
    case ExprKind::Attr: {
      if (is_element_index_name(e->name)) {
        emit(Op::PushIndex);
        e->type = Type::Int;
        return Type::Int;
      }
      if (e->name == "numpt" || e->name == "numelem") {
        emit(Op::PushNpoints);
        e->type = Type::Int;
        return Type::Int;
      }
      std::string aname = canonical_attr_name(e->name);
      Type t = e->attr_type;
      if (t == Type::Void) {
        t = (aname == "position" || aname == "normal") ? Type::Vector : Type::Float;
      }
      emit_push_attr(aname, t);
      e->type = t;
      return t;
    }
    case ExprKind::Unary: {
      const Type ta = compile_expr(e->a);
      if (e->op == "-") {
        emit(Op::Neg);
      }
      else if (e->op == "!") {
        emit(Op::Not);
      }
      else if (e->op == "+") {
        /* no-op */
      }
      e->type = ta;
      return ta;
    }
    case ExprKind::Binary: {
      const Type ta = compile_expr(e->a);
      const Type tb = compile_expr(e->b);
      (void)tb;
      if (e->op == "+") {
        emit(Op::Add);
      }
      else if (e->op == "-") {
        emit(Op::Sub);
      }
      else if (e->op == "*") {
        emit(Op::Mul);
      }
      else if (e->op == "/") {
        emit(Op::Div);
      }
      else if (e->op == "%") {
        emit(Op::Mod);
      }
      else if (e->op == "**") {
        emit(Op::Pow);
        if (ta == Type::Vector || tb == Type::Vector) {
          e->type = Type::Vector;
        }
        else if (ta == Type::Vector2 || tb == Type::Vector2) {
          e->type = Type::Vector2;
        }
        else if (ta == Type::Vector4 || tb == Type::Vector4) {
          e->type = Type::Vector4;
        }
        else {
          e->type = Type::Float;
        }
        return e->type;
      }
      else if (e->op == "<") {
        emit(Op::Lt);
        e->type = Type::Bool;
        return Type::Bool;
      }
      else if (e->op == "<=") {
        emit(Op::Le);
        e->type = Type::Bool;
        return Type::Bool;
      }
      else if (e->op == ">") {
        emit(Op::Gt);
        e->type = Type::Bool;
        return Type::Bool;
      }
      else if (e->op == ">=") {
        emit(Op::Ge);
        e->type = Type::Bool;
        return Type::Bool;
      }
      else if (e->op == "==") {
        emit(Op::Eq);
        e->type = Type::Bool;
        return Type::Bool;
      }
      else if (e->op == "!=") {
        emit(Op::Ne);
        e->type = Type::Bool;
        return Type::Bool;
      }
      else if (e->op == "&&") {
        emit(Op::And);
        e->type = Type::Bool;
        return Type::Bool;
      }
      else if (e->op == "||") {
        emit(Op::Or);
        e->type = Type::Bool;
        return Type::Bool;
      }
      if (e->op == "*") {
        if (ta == Type::Rotation && tb == Type::Rotation) {
          e->type = Type::Rotation;
          return e->type;
        }
        if (ta == Type::Rotation &&
            ELEM(tb, Type::Vector, Type::Vector2, Type::Vector4, Type::Color))
        {
          e->type = (tb == Type::Vector2) ? Type::Vector2 :
                    (tb == Type::Vector4) ? Type::Vector4 :
                                            Type::Vector;
          return e->type;
        }
        if (tb == Type::Rotation &&
            ELEM(ta, Type::Vector, Type::Vector2, Type::Vector4, Type::Color))
        {
          warn_at(e, "向量×旋转：请改成左乘 q * v。这里仍按 q 旋转 v。");
          e->type = (ta == Type::Vector2) ? Type::Vector2 :
                    (ta == Type::Vector4) ? Type::Vector4 :
                                            Type::Vector;
          return e->type;
        }
        const bool am = type_is_matrix(ta);
        const bool bm = type_is_matrix(tb);
        const bool av = type_is_vec_like(ta);
        const bool bv = type_is_vec_like(tb);
        const bool as = ELEM(ta, Type::Float, Type::Int, Type::Bool);
        const bool bs = ELEM(tb, Type::Float, Type::Int, Type::Bool);
        if (am && bm) {
          if (type_linear_dim(ta) != type_linear_dim(tb)) {
            fail_at(e,
                    "矩阵乘法维度不一致：" + type_name(ta) + " × " + type_name(tb) +
                        "。两边阶数必须相同。");
            return Type::Void;
          }
          e->type = ta;
          return e->type;
        }
        if (am && bv) {
          const int md = type_linear_dim(ta);
          const int vd = type_linear_dim(tb);
          if (md != vd) {
            fail_at(e,
                    "矩阵×向量维度不一致：" + type_name(ta) + " × " + type_name(tb) + "。左乘要 " +
                        std::to_string(md) + " 维向量。4×4 变换点请用 transform_point(v, M)。");
            return Type::Void;
          }
          e->type = vec_type_for_dim(md);
          return e->type;
        }
        if (av && bm) {
          const int vd = type_linear_dim(ta);
          const int md = type_linear_dim(tb);
          if (vd != md) {
            fail_at(e,
                    "向量×矩阵维度不一致：" + type_name(ta) + " × " + type_name(tb) +
                        "。请写成矩阵左乘 M * v，且两边维数相同。");
            return Type::Void;
          }
          warn_at(e, "向量×矩阵：请改成矩阵左乘 M * v。这里按 vᵀ × M 计算。");
          e->type = vec_type_for_dim(md);
          return e->type;
        }
        if ((am && bs) || (as && bm)) {
          e->type = am ? ta : tb;
          return e->type;
        }
        if (am || bm) {
          fail_at(e,
                  "不能做 " + type_name(ta) + " × " + type_name(tb) +
                      "。矩阵只能乘同阶矩阵、同维向量，或 float（缩放）。");
          return Type::Void;
        }
      }
      if (ta == Type::Vector4 || tb == Type::Vector4) {
        e->type = Type::Vector4;
      }
      else if (ta == Type::Vector || tb == Type::Vector) {
        e->type = Type::Vector;
      }
      else if (ta == Type::Vector2 || tb == Type::Vector2) {
        e->type = Type::Vector2;
      }
      else {
        e->type = ta;
      }
      return e->type;
    }
    case ExprKind::Call: {
      if (e->name == "geoself" || (e->name == "geo" && e->args.is_empty())) {
        emit(Op::PushI, add_const_i(0));
        e->type = Type::Int;
        return Type::Int;
      }
      const int bid = builtin_from_name(e->name);
      if (bid < 0) {
        int ufi = -1;
        for (const int i : prog.user_fns.index_range()) {
          if (prog.user_fns[i].name == e->name) {
            ufi = i;
            break;
          }
        }
        if (ufi < 0) {
          fail("Unknown function '" + e->name + "'");
          return Type::Void;
        }
        const UserFn &fn = prog.user_fns[ufi];
        if (int(e->args.size()) != int(fn.param_types.size())) {
          fail_at(e,
                  "函数 " + e->name + " 需要 " + std::to_string(int(fn.param_types.size())) +
                      " 个参数");
          return Type::Void;
        }
        for (int i = 0; i < int(e->args.size()); i++) {
          compile_expr(e->args[i], false, fn.param_types[i]);
        }
        emit(Op::CallUser, ufi);
        prog.code.last().a = uint8_t(e->args.size());
        e->type = fn.ret;
        return e->type;
      }
      Builtin builtin = Builtin(bid);
      int call_id = bid;
      if (builtin == Builtin::Identity && type_is_matrix(hint) && hint != Type::Matrix) {
        builtin = matrix_ctor_of(hint);
        call_id = int(builtin);
      }
      const int argc = int(e->args.size());
      int n_out = 0;
      Type out_types[4] = {Type::Float, Type::Float, Type::Float, Type::Float};
      if (builtin == Builtin::BoundingBox && argc >= 2) {
        n_out = 2;
        out_types[0] = Type::Vector;
        out_types[1] = Type::Vector;
      }
      else if (builtin == Builtin::Raycast && argc >= 7) {
        n_out = 4;
        out_types[0] = Type::Int;
        out_types[1] = Type::Vector;
        out_types[2] = Type::Vector;
        out_types[3] = Type::Float;
      }
      else if (builtin == Builtin::RaycastAll && argc >= 7) {
        n_out = 4;
        out_types[0] = Type::IntArray;
        out_types[1] = Type::VecArray;
        out_types[2] = Type::VecArray;
        out_types[3] = Type::FloatArray;
      }
      else if (builtin == Builtin::GeometryProximity && argc >= 5) {
        n_out = 2;
        out_types[0] = Type::Vector;
        out_types[1] = Type::Float;
      }
      else if (builtin == Builtin::SvdFn && argc >= 4) {
        n_out = 3;
        out_types[0] = Type::Matrix;
        out_types[1] = Type::Vector4;
        out_types[2] = Type::Matrix;
      }
      else if (builtin == Builtin::PolarDecompFn && argc >= 3) {
        n_out = 2;
        out_types[0] = Type::Matrix;
        out_types[1] = Type::Matrix;
      }
      else if (builtin == Builtin::EigenFn && argc >= 3) {
        n_out = 2;
        out_types[0] = Type::Vector4;
        out_types[1] = Type::Matrix;
      }
      else if (builtin == Builtin::SampleNearestSurface) {
        n_out = 0;
        Type st = Type::Vector;
        Expr *dt_arg = nullptr;
        if (argc >= 2 && e->args[1] && e->args[1]->kind == ExprKind::LitString) {
          dt_arg = e->args[1];
        }
        else if (argc >= 1 && e->args[0] && e->args[0]->kind == ExprKind::LitString) {
          dt_arg = e->args[0];
        }
        if (dt_arg) {
          const std::string &dt = dt_arg->name;
          if (dt == "int") {
            st = Type::Int;
          }
          else if (dt == "float") {
            st = Type::Float;
          }
          else if (dt == "color") {
            st = Type::Color;
          }
          else if (dt == "bool") {
            st = Type::Bool;
          }
          else if (dt == "vector2") {
            st = Type::Vector2;
          }
          else if (dt == "vector4") {
            st = Type::Vector4;
          }
          else {
            st = Type::Vector;
          }
        }
        e->type = st;
      }
      else if (builtin == Builtin::CornerFace && argc >= 3) {
        n_out = 2;
        out_types[0] = Type::Int;
        out_types[1] = Type::Int;
      }
      else if (builtin == Builtin::PointCurve && argc >= 3) {
        n_out = 2;
        out_types[0] = Type::Int;
        out_types[1] = Type::Int;
      }
      const int n_in = argc - n_out;
      auto force_geo0 = [&](Expr *arg, const char *fn) {
        if (!arg || arg->kind != ExprKind::LitInt || arg->i == 0) {
          return;
        }
        warn(std::string(fn) + ": geo 输入为 " + std::to_string(arg->i) +
             "，已自动改为 0（只支持当前几何 geo 0）");
        arg->i = 0;
      };
      if (builtin == Builtin::Addpoint && n_in >= 2) {
        force_geo0(e->args[0], "addpoint");
      }
      if (builtin == Builtin::Addprim && n_in >= 3 && e->args[0] &&
          e->args[0]->kind == ExprKind::LitInt && e->args[1] &&
          ELEM(e->args[1]->kind, ExprKind::LitString, ExprKind::LitInt))
      {
        force_geo0(e->args[0], "addprim");
      }
      if (builtin == Builtin::SetAttribute) {
        if (n_in >= 6) {
          force_geo0(e->args[0], "setattribute");
        }
        else if (n_in >= 5 && e->args[1] && e->args[1]->kind == ExprKind::LitString) {
          force_geo0(e->args[0], "setattribute");
        }
      }
      if (builtin == Builtin::DeleteGeometry) {
        if (n_in >= 4) {
          force_geo0(e->args[0], "delete_geometry");
        }
        else if (n_in >= 3 && e->args[1] && e->args[1]->kind == ExprKind::LitString) {
          force_geo0(e->args[0], "delete_geometry");
        }
      }
      if (builtin == Builtin::Map && n_in != 5) {
        fail("map(value, from_min, from_max, to_min, to_max) 需要 5 个参数");
        return Type::Void;
      }
      if (builtin == Builtin::Smooth && n_in != 1 && n_in != 3) {
        fail("smooth(value) 或 smooth(min, max, value)");
        return Type::Void;
      }
      if (builtin == Builtin::ValueToString && n_in != 2 && n_in != 3) {
        fail("valuetostring(value, digits) or valuetostring(value, before, after)");
        return Type::Void;
      }
      if (builtin == Builtin::Format && n_in < 1) {
        fail("format(fmt, ...) needs a format string");
        return Type::Void;
      }
      if (builtin == Builtin::SvdFn && argc < 4) {
        fail("svd(m, U, s, V) 需要矩阵和三个输出：U、奇异值、V");
        return Type::Void;
      }
      if (builtin == Builtin::PolarDecompFn && argc < 3) {
        fail("pd(m, R, S) 需要矩阵和两个输出：旋转 R、缩放 S");
        return Type::Void;
      }
      if (builtin == Builtin::EigenFn && argc < 3) {
        fail("eigen(m, evals, evecs) 需要矩阵和两个输出：特征值、特征向量");
        return Type::Void;
      }
      bool decomp_arg0_done = false;
      if (ELEM(builtin, Builtin::SvdFn, Builtin::PolarDecompFn, Builtin::EigenFn) && n_in >= 1 &&
          e->args[0])
      {
        compile_expr(e->args[0]);
        decomp_arg0_done = true;
        const Type mt = e->args[0]->type;
        if (!type_is_matrix(mt)) {
          fail_at(e->args[0], "svd / pd / eigen 的第一个参数必须是矩阵");
          return Type::Void;
        }
        const int dim = type_linear_dim(mt);
        if (builtin == Builtin::SvdFn) {
          out_types[0] = mt;
          out_types[1] = vec_type_for_dim(dim);
          out_types[2] = mt;
        }
        else if (builtin == Builtin::PolarDecompFn) {
          out_types[0] = mt;
          out_types[1] = mt;
        }
        else {
          out_types[0] = vec_type_for_dim(dim);
          out_types[1] = mt;
        }
      }
      for (int i = 0; i < n_out; i++) {
        declare_out_if_needed(e->args[n_in + i], out_types[i]);
      }
      /* Record the exact geometry inputs used by spatial builtins. Prewarming every input
       * geometry can build several unrelated BVHs before the first query. Literal geometry
       * indices get a compact mask; dynamic indices retain the conservative all-geometry path. */
      auto record_spatial_geo = [&](uint8_t &mask, bool &dynamic, const int arg_index) {
        if (arg_index < 0 || arg_index >= n_in || !e->args[arg_index]) {
          mask |= 1u;
          return;
        }
        const Expr *arg = e->args[arg_index];
        if (arg->kind == ExprKind::LitInt && arg->i >= 0 && arg->i < 8) {
          mask |= uint8_t(1u << arg->i);
        }
        else {
          dynamic = true;
        }
      };
      if (builtin == Builtin::GeometryProximity) {
        record_spatial_geo(prog.prox_geo_mask, prog.prox_geo_dynamic, 0);
      }
      else if (builtin == Builtin::Raycast || builtin == Builtin::RaycastAll) {
        record_spatial_geo(prog.ray_geo_mask, prog.ray_geo_dynamic, n_in >= 4 ? 0 : -1);
      }
      else if (builtin == Builtin::SampleNearestSurface) {
        record_spatial_geo(prog.sample_geo_mask, prog.sample_geo_dynamic, n_in >= 1 ? 0 : -1);
      }
      /* Fold geometry_proximity domain string to an int so the VM skips string compares. */
      if (builtin == Builtin::GeometryProximity && n_in >= 2 && e->args[1] &&
          e->args[1]->kind == ExprKind::LitString)
      {
        const std::string &s = e->args[1]->name;
        int domain = -1;
        if (s == "point" || s == "points" || s == "POINT" || s == "Points") {
          domain = 0;
        }
        else if (s == "edge" || s == "edges" || s == "EDGE" || s == "Edges") {
          domain = 1;
        }
        else if (s == "face" || s == "faces" || s == "FACE" || s == "Faces") {
          domain = 2;
        }
        if (domain >= 0) {
          e->args[1]->kind = ExprKind::LitInt;
          e->args[1]->type = Type::Int;
          e->args[1]->i = domain;
        }
      }
      /* Batch C nearest-query when sample is a vector attribute (v@P or other). */
      if (builtin == Builtin::GeometryProximity && n_in >= 3 && e->args[2]) {
        const Expr *sample = e->args[2];
        const bool sample_is_p = (sample->kind == ExprKind::Attr ||
                                  sample->kind == ExprKind::Ident) &&
                                 (sample->name == "P" || sample->name == "position");
        bool sample_ok = sample_is_p;
        int sample_attr = -1;
        if (!sample_ok && sample &&
            (sample->kind == ExprKind::Attr || sample->kind == ExprKind::Ident) &&
            (sample->kind != ExprKind::Ident || find_local(sample->name) == nullptr))
        {
          sample_attr = attr_slot(canonical_attr_name(sample->name), Type::Vector, false);
          sample_ok = true;
        }
        bool geo_ok = true;
        int geo = 0;
        int domain = 2;
        if (e->args[0]) {
          if (e->args[0]->kind == ExprKind::LitInt) {
            geo = e->args[0]->i;
          }
          else {
            geo_ok = false;
          }
        }
        if (e->args[1] && e->args[1]->kind == ExprKind::LitInt) {
          domain = e->args[1]->i;
        }
        else if (e->args[1] && e->args[1]->kind != ExprKind::LitString) {
          geo_ok = false;
        }
        if (sample_ok && geo_ok) {
          prog.prox_batch = true;
          prog.prox_geo = geo;
          prog.prox_domain = domain;
          prog.prox_sample_attr = sample_attr;
        }
      }
      if ((builtin == Builtin::NearestPoints || builtin == Builtin::NearPoints) && n_in >= 3) {
        for (int ai = 2; ai <= 3 && ai < n_in; ai++) {
          if (e->args[ai] && e->args[ai]->kind == ExprKind::LitString &&
              nearestpoints_name_is_mode(e->args[ai]->name))
          {
            e->args[ai]->kind = ExprKind::LitInt;
            e->args[ai]->type = Type::Int;
            e->args[ai]->i = nearestpoints_mode_from_name(e->args[ai]->name);
          }
        }
      }
      if ((builtin == Builtin::NearestPoints || builtin == Builtin::NearPoints) && n_in >= 2) {
        int knn_geo = 0;
        int knn_k = 8;
        int np_mode = builtin == Builtin::NearPoints ? 1 : 0;
        bool k_const = false;
        if (e->args[0] && e->args[0]->kind == ExprKind::LitInt) {
          knn_geo = e->args[0]->i;
        }
        if (e->args[1] && e->args[1]->kind == ExprKind::LitInt) {
          knn_k = e->args[1]->i;
          k_const = true;
        }
        else if (e->args[1] && e->args[1]->kind == ExprKind::LitFloat) {
          np_mode = 1;
        }
        if (n_in >= 4 && e->args[3] && e->args[3]->kind == ExprKind::LitInt && e->args[3]->i == 2) {
          np_mode = 2;
          if (e->args[2] && e->args[2]->kind == ExprKind::LitInt) {
            knn_k = e->args[2]->i;
            k_const = true;
          }
        }
        else if (n_in >= 3 && e->args[2] && e->args[2]->kind == ExprKind::LitInt) {
          if (e->args[2]->i == 2) {
            np_mode = 2;
          }
          else {
            np_mode = e->args[2]->i != 0 ? 1 : 0;
          }
        }
        const bool custom_pos = n_in >= 4 && e->args[3] && !expr_is_p(e->args[3]) &&
                                !(e->args[3]->kind == ExprKind::LitInt && e->args[3]->i >= 0 &&
                                  e->args[3]->i <= 2);
        if (np_mode == 0 && !custom_pos && k_const && knn_k > 0) {
          prog.knn_precompute = true;
          prog.knn_k = knn_k;
          prog.knn_geo = knn_geo;
        }
      }
      if (builtin == Builtin::Raycast) {
        Expr *orig = nullptr;
        Expr *dir = nullptr;
        Expr *len = nullptr;
        int geo = 0;
        bool geo_ok = true;
        if (n_in >= 4) {
          if (e->args[0] && e->args[0]->kind == ExprKind::LitInt) {
            geo = e->args[0]->i;
          }
          else if (e->args[0]) {
            geo_ok = false;
          }
          orig = e->args[1];
          dir = e->args[2];
          len = e->args[3];
        }
        else if (n_in == 3) {
          orig = e->args[0];
          dir = e->args[1];
          len = e->args[2];
        }
        else if (n_in == 2) {
          orig = e->args[0];
          dir = e->args[1];
        }
        else if (n_in == 1) {
          dir = e->args[0];
        }
        float3 d(0.0f, 0.0f, -1.0f);
        float L = 100.0f;
        bool orig_ok = orig == nullptr || expr_is_p(orig);
        int orig_attr = -1;
        if (!orig_ok && orig &&
            (orig->kind == ExprKind::Attr || orig->kind == ExprKind::Ident) &&
            (orig->kind != ExprKind::Ident || find_local(orig->name) == nullptr))
        {
          orig_attr = attr_slot(canonical_attr_name(orig->name), Type::Vector, false);
          orig_ok = true;
        }
        bool dir_ok = (dir == nullptr || expr_const_vec3(dir, d));
        int dir_attr = -1;
        bool dir_normalize = false;
        if (!dir_ok && dir && dir->kind == ExprKind::Call &&
            (dir->name == "normalize" || dir->name == "normalise") && !dir->args.is_empty() &&
            dir->args[0])
        {
          Expr *inner = dir->args[0];
          if ((inner->kind == ExprKind::Attr || inner->kind == ExprKind::Ident) &&
              (inner->kind != ExprKind::Ident || find_local(inner->name) == nullptr))
          {
            dir_attr = attr_slot(canonical_attr_name(inner->name), Type::Vector, false);
            dir_ok = true;
            dir_normalize = true;
          }
        }
        if (!dir_ok && dir &&
            (dir->kind == ExprKind::Attr || dir->kind == ExprKind::Ident) &&
            (dir->kind != ExprKind::Ident || find_local(dir->name) == nullptr))
        {
          dir_attr = attr_slot(canonical_attr_name(dir->name), Type::Vector, false);
          dir_ok = true;
        }
        if (geo_ok && orig_ok && dir_ok && (len == nullptr || expr_lit_num(len, L))) {
          prog.ray_batch = true;
          prog.ray_geo = geo;
          prog.ray_dir = d;
          prog.ray_len = L;
          prog.ray_dir_attr = dir_attr;
          prog.ray_orig_attr = orig_attr;
          prog.ray_dir_normalize = dir_normalize;
        }
      }
      if (builtin == Builtin::SampleNearestSurface && n_in >= 1) {
        Expr *orig = n_in >= 3 ? e->args[2] : (n_in >= 2 && e->args[1] &&
                                                       e->args[1]->kind != ExprKind::LitString ?
                                                   e->args[1] :
                                                   nullptr);
        if (orig == nullptr || expr_is_p(orig)) {
          int geo = 0;
          if (e->args[0] && e->args[0]->kind == ExprKind::LitInt) {
            geo = e->args[0]->i;
          }
          prog.sample_batch = true;
          prog.sample_geo = geo;
          if (n_in >= 3 && e->args[2] && e->args[2]->kind == ExprKind::LitString) {
            prog.sample_attr = e->args[2]->name;
          }
          else if (n_in >= 2 && e->args[1] && e->args[1]->kind == ExprKind::LitString) {
            prog.sample_attr = e->args[1]->name;
          }
        }
      }
      auto is_name_like = [&](const Expr *a) {
        return a && ELEM(a->kind, ExprKind::LitString, ExprKind::Ident, ExprKind::Attr);
      };
      const bool is_sample = ELEM(builtin,
                                  Builtin::Point,
                                  Builtin::Edge,
                                  Builtin::Face,
                                  Builtin::Corner,
                                  Builtin::Curve,
                                  Builtin::Instance);
      int sample_name_i = -1;
      int static_sample_i = -1;
      if (is_sample) {
        if (n_in >= 3) {
          sample_name_i = 1;
        }
        else if (n_in == 2) {
          sample_name_i = is_name_like(e->args[0]) ? 0 : 1;
        }
        else if (n_in == 1 && is_name_like(e->args[0])) {
          sample_name_i = 0;
        }

        bool may_sample_self = true;
        const int sample_geo_i = sample_name_i == 1 ? 0 : -1;
        if (sample_geo_i >= 0 && e->args[sample_geo_i]) {
          const Expr *geo_arg = e->args[sample_geo_i];
          if (geo_arg->kind == ExprKind::LitInt && geo_arg->i > 0) {
            may_sample_self = false;
          }
        }
        if (may_sample_self) {
          if (sample_name_i < 0 || e->args[sample_name_i] == nullptr) {
            if (!prog.sampled_self_attrs.contains("position")) {
              prog.sampled_self_attrs.append("position");
            }
          }
          else {
            const Expr *name_arg = e->args[sample_name_i];
            const Local *name_local = name_arg->kind == ExprKind::Ident ?
                                          find_local(name_arg->name) :
                                          nullptr;
            if (name_local && name_local->type == Type::String) {
              prog.sampled_self_attr_dynamic = true;
            }
            else if (ELEM(name_arg->kind,
                          ExprKind::LitString,
                          ExprKind::Ident,
                          ExprKind::Attr))
            {
              const std::string name = canonical_attr_name(name_arg->name);
              if (!prog.sampled_self_attrs.contains(name)) {
                prog.sampled_self_attrs.append(name);
              }
            }
            else {
              prog.sampled_self_attr_dynamic = true;
            }
          }
        }

        int static_geo = 0;
        bool geo_is_static = true;
        if (sample_geo_i >= 0 && e->args[sample_geo_i]) {
          const Expr *geo_arg = e->args[sample_geo_i];
          if (geo_arg->kind == ExprKind::LitInt) {
            static_geo = std::max(geo_arg->i, 0);
          }
          else {
            geo_is_static = false;
          }
        }
        std::string static_name = "position";
        bool name_is_static = true;
        if (sample_name_i >= 0 && e->args[sample_name_i]) {
          const Expr *name_arg = e->args[sample_name_i];
          const Local *name_local = name_arg->kind == ExprKind::Ident ?
                                        find_local(name_arg->name) :
                                        nullptr;
          if (name_local && name_local->type == Type::String) {
            name_is_static = false;
          }
          else if (ELEM(name_arg->kind,
                        ExprKind::LitString,
                        ExprKind::Ident,
                        ExprKind::Attr))
          {
            static_name = canonical_attr_name(name_arg->name);
          }
          else {
            name_is_static = false;
          }
        }
        if (geo_is_static && name_is_static) {
          int sample_domain = 0;
          if (builtin == Builtin::Edge) {
            sample_domain = 1;
          }
          else if (builtin == Builtin::Face) {
            sample_domain = 2;
          }
          else if (builtin == Builtin::Corner) {
            sample_domain = 3;
          }
          else if (builtin == Builtin::Instance) {
            sample_domain = 4;
          }
          else if (builtin == Builtin::Curve) {
            sample_domain = 5;
          }
          for (const int i : prog.element_samples.index_range()) {
            const Program::ElementSample &sample = prog.element_samples[i];
            if (sample.geo == static_geo && sample.domain == sample_domain &&
                sample.name == static_name)
            {
              static_sample_i = i;
              break;
            }
          }
          if (static_sample_i < 0) {
            Program::ElementSample sample;
            sample.geo = static_geo;
            sample.domain = sample_domain;
            sample.name = std::move(static_name);
            prog.element_samples.append(std::move(sample));
            static_sample_i = int(prog.element_samples.size()) - 1;
          }
        }
      }
      for (int i = 0; i < n_in; i++) {
        if (decomp_arg0_done && i == 0) {
          continue;
        }
        Expr *arg = e->args[i];
        if (i == sample_name_i && arg &&
            ELEM(arg->kind, ExprKind::Ident, ExprKind::Attr))
        {
          const Local *loc = (arg->kind == ExprKind::Ident) ? find_local(arg->name) : nullptr;
          if (!(loc && loc->type == Type::String)) {
            emit(Op::PushS, add_const_s(std::string(arg->name)));
            arg->type = Type::String;
            continue;
          }
        }
        compile_expr(arg);
      }
      {
        const StringRef tags = function_param_types(e->name);
        if (!tags.is_empty() && n_in > 0) {
          StringRef ins[16];
          int n_tag = 0;
          StringRef rest = tags;
          while (!rest.is_empty() && n_tag < 16) {
            const int64_t comma = rest.find(',');
            const StringRef one = (comma == StringRef::not_found) ? rest : rest.substr(0, comma);
            if (!one.startswith("out")) {
              ins[n_tag++] = one;
            }
            if (comma == StringRef::not_found) {
              break;
            }
            rest = rest.drop_prefix(comma + 1);
          }
          int offset = 0;
          int arg_base = 0;
          if (n_in < n_tag && ins[0] == "geo") {
            const bool first_is_geo = e->args[0] && type_fits_tag("geo", e->args[0]->type);
            const bool second_fits = n_in >= 2 && e->args[1] &&
                                     type_fits_tag(ins[1], e->args[1]->type);
            offset = (first_is_geo && second_fits) ? 0 : 1;
          }
          else if (n_in == n_tag + 1 && ins[0] != "geo" && e->args[0] &&
                   e->args[0]->type == Type::Int)
          {
            /* Extra leading geo index: delete_geometry(0, "point", i, "all"). */
            arg_base = 1;
          }
          const int n_check = std::min(n_in - arg_base, n_tag - offset);
          for (int i = 0; i < n_check; i++) {
            Expr *arg = e->args[arg_base + i];
            if (!arg) {
              continue;
            }
            const StringRef want = ins[offset + i];
            if (!type_fits_tag(want, arg->type)) {
              fail("Type mismatch: argument " + std::to_string(arg_base + i + 1) + " of '" +
                   e->name + "' expected " + expect_tag_name(want) + ", got " +
                   type_name(arg->type));
              return Type::Void;
            }
          }
        }
      }
      /* `float abc[] = array();` must be a float array, not the untyped int array(). */
      if (builtin == Builtin::ArrayFn && argc == 0 && type_is_array(hint)) {
        switch (hint) {
          case Type::FloatArray:
            builtin = Builtin::ArrayFloat;
            break;
          case Type::VecArray:
            builtin = Builtin::ArrayVec;
            break;
          case Type::StringArray:
            builtin = Builtin::ArrayStr;
            break;
          case Type::MatArray:
            builtin = Builtin::ArrayMat;
            break;
          case Type::RayArray:
            builtin = Builtin::ArrayRay;
            break;
          case Type::IntArray:
            builtin = Builtin::ArrayInt;
            break;
          default:
            break;
        }
        call_id = int(builtin);
      }
      const bool direct_face_corners = builtin == Builtin::FaceCorners &&
                                       (n_in < 2 ||
                                        (e->args[0] && e->args[0]->kind == ExprKind::LitInt &&
                                         e->args[0]->i == 0));
      if (direct_face_corners) {
        emit(Op::FaceCorners);
        prog.code.last().a = uint8_t(n_in);
      }
      else if (static_sample_i >= 0) {
        int index_arg = -1;
        if (n_in >= 3) {
          index_arg = 2;
        }
        else if (n_in == 2 && sample_name_i == 0) {
          index_arg = 1;
        }
        else if (n_in == 1 && sample_name_i < 0) {
          index_arg = 0;
        }
        emit(Op::SampleElem, static_sample_i);
        prog.code.last().a = uint8_t((n_in & 0x1f) | ((index_arg + 1) << 5));
      }
      else {
        emit(Op::Call, call_id);
        prog.code.last().a = uint8_t(n_in);
      }
      switch (builtin) {
        case Builtin::Length:
        case Builtin::Distance:
        case Builtin::Dot:
        case Builtin::Noise:
        case Builtin::Hash:
        case Builtin::Rand:
        case Builtin::FloatFn:
          if (builtin == Builtin::Length && !e->args.is_empty() && e->args[0] &&
              type_is_array(e->args[0]->type))
          {
            e->type = Type::Int;
          }
          else {
            e->type = Type::Float;
          }
          break;
        case Builtin::Sin:
        case Builtin::Cos:
        case Builtin::Tan:
        case Builtin::Asin:
        case Builtin::Acos:
        case Builtin::Atan:
        case Builtin::Floor:
        case Builtin::Ceil:
        case Builtin::Round:
        case Builtin::Trunc:
        case Builtin::Sqrt:
        case Builtin::Exp:
        case Builtin::Log:
        case Builtin::Sign:
        case Builtin::Fract:
        case Builtin::Ddx:
        case Builtin::Ddy:
        case Builtin::Fwidth:
        case Builtin::Radians:
        case Builtin::Degrees:
        case Builtin::Mix:
        case Builtin::Map:
          e->type = unary_math_result_type(e);
          break;
        case Builtin::Smooth:
          if (e->args.size() >= 3) {
            e->type = unary_vec_keep(expr_arg_type(e, 2));
          }
          else {
            e->type = unary_math_result_type(e);
          }
          break;
        case Builtin::Abs:
          if (expr_arg_type(e, 0) == Type::Int) {
            e->type = Type::Int;
          }
          else {
            e->type = unary_math_result_type(e);
          }
          break;
        case Builtin::Atan2:
        case Builtin::Pow:
        case Builtin::Min:
        case Builtin::Max:
          e->type = binary_math_result_type(e);
          break;
        case Builtin::Clamp:
          e->type = unary_math_result_type(e);
          break;
        case Builtin::Npoints:
        case Builtin::Nedges:
        case Builtin::Nfaces:
        case Builtin::Ncorners:
        case Builtin::IntFn:
        case Builtin::Addpoint:
        case Builtin::Addprim:
          e->type = Type::Int;
          break;
        case Builtin::Cross:
          e->type = cross_result_type(e);
          break;
        case Builtin::Normalize:
          if (type_is_vec_like(expr_arg_type(e, 0))) {
            e->type = expr_arg_type(e, 0);
          }
          else {
            e->type = Type::Vector;
          }
          break;
        case Builtin::Set: {
          Vector<Type, 4> arg_types;
          for (Expr *arg : e->args) {
            arg_types.append(arg ? arg->type : Type::Float);
          }
          e->type = set_fn_result_type(arg_types);
          break;
        }
        case Builtin::Vec3Fn:
        case Builtin::TransformPoint:
        case Builtin::TransformDirection:
        case Builtin::ProjectPoint:
        case Builtin::TranslationFn:
        case Builtin::RotationFn:
        case Builtin::ScaleFn:
        case Builtin::Chv:
          e->type = Type::Vector;
          break;
        case Builtin::Invert:
          if (!e->args.is_empty() && e->args[0] && e->args[0]->type == Type::Rotation) {
            e->type = Type::Rotation;
          }
          else if (!e->args.is_empty() && e->args[0] && type_is_matrix(e->args[0]->type)) {
            e->type = e->args[0]->type;
          }
          else {
            e->type = Type::Matrix;
          }
          break;
        case Builtin::Transpose:
          if (!e->args.is_empty() && e->args[0] && type_is_matrix(e->args[0]->type)) {
            e->type = e->args[0]->type;
          }
          else {
            e->type = Type::Matrix;
          }
          break;
        case Builtin::CombineTransform:
        case Builtin::Identity:
        case Builtin::MatrixFn:
        case Builtin::Chm:
          e->type = Type::Matrix;
          break;
        case Builtin::Matrix2Fn:
          e->type = Type::Matrix2;
          break;
        case Builtin::Matrix3Fn:
          e->type = Type::Matrix3;
          break;
        case Builtin::Vec2Fn:
          e->type = Type::Vector2;
          break;
        case Builtin::Vec4Fn:
          e->type = Type::Vector4;
          break;
        case Builtin::RotateRotation:
        case Builtin::QuatFn:
        case Builtin::Chq:
          e->type = Type::Rotation;
          break;
        case Builtin::Chs:
        case Builtin::ValueToString:
        case Builtin::Format:
          e->type = Type::String;
          break;
        case Builtin::ColorFn:
          e->type = Type::Color;
          break;
        case Builtin::Voronoi:
          e->type = Type::Float;
          break;
        case Builtin::FieldMinMax:
          e->type = Type::Vector;
          break;
        case Builtin::BoundingBox:
          e->type = Type::VecArray;
          break;
        case Builtin::Raycast:
          e->type = Type::Ray;
          break;
        case Builtin::RaycastAll:
          e->type = Type::RayArray;
          break;
        case Builtin::RayIsHitArr:
          e->type = Type::IntArray;
          break;
        case Builtin::RayHitPosArr:
        case Builtin::RayHitNArr:
          e->type = Type::VecArray;
          break;
        case Builtin::RayHitDistArr:
          e->type = Type::FloatArray;
          break;
        case Builtin::RayIsHit:
          e->type = Type::Int;
          break;
        case Builtin::DeleteGeometry:
        case Builtin::SetAttribute:
          e->type = Type::Int;
          break;
        case Builtin::RayHitPos:
        case Builtin::RayHitN:
        case Builtin::BBoxMin:
        case Builtin::BBoxMax:
          e->type = Type::Vector;
          break;
        case Builtin::RayHitDist:
        case Builtin::FieldAverage:
        case Builtin::FieldMin:
        case Builtin::FieldMax:
        case Builtin::GeometryProximity:
          e->type = Type::Float;
          break;
        case Builtin::SampleNearestSurface:
          break;
        case Builtin::Accumulate:
          if (!e->args.is_empty() && e->args[0] && e->args[0]->type == Type::Vector) {
            e->type = Type::Vector;
          }
          else {
            e->type = Type::Float;
          }
          break;
        case Builtin::ArrayInt:
        case Builtin::CornersOfFace:
        case Builtin::CornersOfVertex:
        case Builtin::CornersOfEdge:
        case Builtin::EdgesOfVertex:
        case Builtin::EdgesOfCorner:
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
        case Builtin::PointCurve:
        case Builtin::PointsOfCurve:
        case Builtin::NearestPoints:
        case Builtin::NearPoints:
          e->type = Type::IntArray;
          break;
        case Builtin::ArrayFloat:
          e->type = Type::FloatArray;
          break;
        case Builtin::ArrayVec:
          e->type = Type::VecArray;
          break;
        case Builtin::ArrayStr:
          e->type = Type::StringArray;
          break;
        case Builtin::ArrayMat:
          e->type = Type::MatArray;
          break;
        case Builtin::ArrayRay:
          e->type = Type::RayArray;
          break;
        case Builtin::ArrayFn:
        case Builtin::Append:
        case Builtin::Insert:
        case Builtin::RemoveIndex:
        case Builtin::RemoveValue:
        case Builtin::SortArr:
          if (!e->args.is_empty() && e->args[0]) {
            e->type = type_is_array(e->args[0]->type) ? e->args[0]->type :
                                                       array_type_of(e->args[0]->type);
          }
          else {
            e->type = Type::IntArray;
          }
          break;
        case Builtin::Len:
        case Builtin::FaceOfCorner:
        case Builtin::VertexOfCorner:
        case Builtin::OffsetCornerInFace:
        case Builtin::CurveOfPoint:
          e->type = Type::Int;
          break;
        case Builtin::Chc:
          e->type = Type::Color;
          break;
        case Builtin::Chb:
          e->type = Type::Bool;
          break;
        case Builtin::Chi:
          e->type = Type::Int;
          break;
        case Builtin::Chf:
          e->type = Type::Float;
          break;
        case Builtin::SvdFn:
        case Builtin::PolarDecompFn:
        case Builtin::EigenFn:
          e->type = Type::Int;
          break;
        case Builtin::Point:
        case Builtin::Edge:
        case Builtin::Face:
        case Builtin::Corner:
        case Builtin::Curve:
        case Builtin::Instance: {
          Type st = Type::Float;
          if (sample_name_i >= 0 && e->args[sample_name_i]) {
            const Expr *na = e->args[sample_name_i];
            std::string nm;
            if (na->kind == ExprKind::LitString || na->kind == ExprKind::Ident ||
                na->kind == ExprKind::Attr)
            {
              nm = canonical_attr_name(na->name);
            }
            if (nm == "position" || nm == "normal") {
              st = Type::Vector;
            }
            else if (nm == "id" || nm == "material_index") {
              st = Type::Int;
            }
            else if (nm == "color") {
              st = Type::Color;
            }
          }
          e->type = st;
          break;
        }
        default:
          e->type = Type::Float;
          break;
      }
      if (n_out > 0) {
        if (ELEM(builtin, Builtin::SvdFn, Builtin::PolarDecompFn, Builtin::EigenFn)) {
          emit(Op::Pop);
          emit(Op::Call, int(Builtin::DecompU));
          prog.code.last().a = 0;
          compile_lvalue_store(e->args[n_in], out_types[0]);
          emit(Op::Call, int(Builtin::DecompS));
          prog.code.last().a = 0;
          compile_lvalue_store(e->args[n_in + 1], out_types[1]);
          if (builtin == Builtin::SvdFn) {
            emit(Op::Call, int(Builtin::DecompV));
            prog.code.last().a = 0;
            compile_lvalue_store(e->args[n_in + 2], out_types[2]);
          }
          emit(Op::PushI, add_const_i(1));
          e->type = Type::Int;
        }
        else if (builtin == Builtin::BoundingBox) {
          emit(Op::Dup);
          emit(Op::Dup);
          emit(Op::PushI, add_const_i(0));
          emit(Op::GetIndex);
          compile_lvalue_store(e->args[n_in], out_types[0]);
          emit(Op::PushI, add_const_i(1));
          emit(Op::GetIndex);
          compile_lvalue_store(e->args[n_in + 1], out_types[1]);
          e->type = Type::VecArray;
        }
        else if (builtin == Builtin::Raycast) {
          emit(Op::Call, int(Builtin::RayIsHit));
          prog.code.last().a = 0;
          compile_lvalue_store(e->args[n_in], out_types[0]);
          emit(Op::Call, int(Builtin::RayHitPos));
          prog.code.last().a = 0;
          compile_lvalue_store(e->args[n_in + 1], out_types[1]);
          emit(Op::Call, int(Builtin::RayHitN));
          prog.code.last().a = 0;
          compile_lvalue_store(e->args[n_in + 2], out_types[2]);
          emit(Op::Call, int(Builtin::RayHitDist));
          prog.code.last().a = 0;
          compile_lvalue_store(e->args[n_in + 3], out_types[3]);
          e->type = Type::Ray;
        }
        else if (builtin == Builtin::RaycastAll) {
          emit(Op::Call, int(Builtin::RayIsHitArr));
          prog.code.last().a = 0;
          compile_lvalue_store(e->args[n_in], out_types[0]);
          emit(Op::Call, int(Builtin::RayHitPosArr));
          prog.code.last().a = 0;
          compile_lvalue_store(e->args[n_in + 1], out_types[1]);
          emit(Op::Call, int(Builtin::RayHitNArr));
          prog.code.last().a = 0;
          compile_lvalue_store(e->args[n_in + 2], out_types[2]);
          emit(Op::Call, int(Builtin::RayHitDistArr));
          prog.code.last().a = 0;
          compile_lvalue_store(e->args[n_in + 3], out_types[3]);
          e->type = Type::RayArray;
        }
        else if (builtin == Builtin::GeometryProximity) {
          /* One query: Call returns dist and fills env.hit_pos. PushHitPos is not a 2nd tree walk. */
          emit(Op::PushHitPos);
          compile_lvalue_store(e->args[n_in], out_types[0]);
          emit(Op::Dup);
          compile_lvalue_store(e->args[n_in + 1], out_types[1]);
          e->type = Type::Float;
        }
        else if (ELEM(builtin, Builtin::CornerFace, Builtin::PointCurve)) {
          emit(Op::Dup);
          emit(Op::Dup);
          emit(Op::PushI, add_const_i(0));
          emit(Op::GetIndex);
          compile_lvalue_store(e->args[n_in], out_types[0]);
          emit(Op::PushI, add_const_i(1));
          emit(Op::GetIndex);
          compile_lvalue_store(e->args[n_in + 1], out_types[1]);
          e->type = Type::IntArray;
        }
      }
      if (ELEM(builtin,
               Builtin::Append,
               Builtin::Insert,
               Builtin::RemoveIndex,
               Builtin::RemoveValue,
               Builtin::SortArr,
               Builtin::SortByOrder) &&
          !e->args.is_empty() && e->args[0] &&
          ELEM(e->args[0]->kind, ExprKind::Ident, ExprKind::Attr))
      {
        emit(Op::Dup);
        compile_lvalue_store(e->args[0]);
      }
      return e->type;
    }
    case ExprKind::Index: {
      const Type ta = compile_expr(e->a);
      compile_expr(e->b);
      emit(Op::GetIndex);
      if (type_is_array(ta)) {
        e->type = array_elem_type(ta);
      }
      else if (type_is_matrix(ta)) {
        e->type = vec_type_for_dim(type_linear_dim(ta));
      }
      else if (type_has_xyzw(ta)) {
        e->type = Type::Float;
      }
      else {
        fail_at(e, "下标只能用于数组、向量、矩阵或旋转");
        e->type = Type::Float;
      }
      return e->type;
    }
    case ExprKind::Member: {
      compile_expr(e->a);
      emit(Op::GetMember, e->member);
      e->type = swizzle_result_type(swizzle_count(e->member));
      return e->type;
    }
    case ExprKind::VecLit: {
      const int ncomp = int(e->args.size());
      if (type_is_matrix(hint)) {
        for (Expr *c : e->args) {
          compile_expr(c);
        }
        emit(Op::Call, int(matrix_ctor_of(hint)));
        prog.code.last().a = uint8_t(std::min(ncomp, 255));
        e->type = hint;
        return hint;
      }
      if (hint == Type::Color || hint == Type::Rotation || hint == Type::Vector2 ||
          hint == Type::Vector4 || hint == Type::Vector)
      {
        for (Expr *c : e->args) {
          compile_expr(c);
        }
        Builtin fn = Builtin::Vec3Fn;
        if (hint == Type::Color) {
          fn = Builtin::ColorFn;
        }
        else if (hint == Type::Rotation) {
          fn = Builtin::QuatFn;
        }
        else if (hint == Type::Vector2) {
          fn = Builtin::Vec2Fn;
        }
        else if (hint == Type::Vector4) {
          fn = Builtin::Vec4Fn;
        }
        emit(Op::Call, int(fn));
        prog.code.last().a = uint8_t(std::min(ncomp, 255));
        e->type = hint;
        return hint;
      }
      /* Brace lists assigned to arrays (or longer than a vector) are real lists, not `{x,y,z}`. */
      if (type_is_array(hint) || ncomp > 4 || (ncomp <= 1 && type_is_array(hint))) {
        Type at = hint;
        if (!type_is_array(at)) {
          bool any_str = false;
          bool any_vec = false;
          bool any_float = false;
          for (Expr *c : e->args) {
            if (!c) {
              continue;
            }
            if (c->kind == ExprKind::LitString || c->type == Type::String) {
              any_str = true;
            }
            else if (c->type == Type::Vector || c->kind == ExprKind::VecLit) {
              any_vec = true;
            }
            else if (c->type == Type::Matrix) {
              at = Type::MatArray;
            }
            else if (c->kind == ExprKind::LitFloat || c->type == Type::Float) {
              any_float = true;
            }
          }
          if (at != Type::MatArray) {
            at = any_str ? Type::StringArray :
                 any_vec ? Type::VecArray :
                 any_float ? Type::FloatArray :
                             Type::IntArray;
          }
        }
        for (Expr *c : e->args) {
          compile_expr(c);
        }
        if (at == Type::RayArray) {
          emit(Op::Call, int(Builtin::ArrayRay));
        }
        else {
          emit(Op::Call, int(Builtin::ArrayFn));
        }
        prog.code.last().a = uint8_t(ncomp);
        e->type = at;
        return at;
      }
      const Type lit_type = (ncomp >= 4) ? Type::Vector4 :
                            (ncomp == 2) ? Type::Vector2 :
                                           Type::Vector;
      if (ncomp >= 2 && ncomp <= 4) {
        bool all_lit = true;
        float4 v(0.0f);
        if (ncomp == 4) {
          v.w = 1.0f;
        }
        for (int i = 0; i < ncomp; i++) {
          Expr *c = e->args[i];
          float f = 0.0f;
          if (c && c->kind == ExprKind::LitFloat) {
            f = c->f;
          }
          else if (c && c->kind == ExprKind::LitInt) {
            f = float(c->i);
          }
          else {
            all_lit = false;
            break;
          }
          if (i == 0) {
            v.x = f;
          }
          else if (i == 1) {
            v.y = f;
          }
          else if (i == 2) {
            v.z = f;
          }
          else {
            v.w = f;
          }
        }
        if (all_lit && ncomp == 3) {
          emit(Op::PushV, add_const_v(float3(v.x, v.y, v.z)));
          e->type = Type::Vector;
          return Type::Vector;
        }
      }
      for (Expr *c : e->args) {
        compile_expr(c);
      }
      emit(Op::MakeVec, ncomp);
      e->type = lit_type;
      return lit_type;
    }
    case ExprKind::Cond: {
      compile_expr(e->a);
      const int jmp_false = emit_jmp(Op::JmpIfFalse);
      const Type tt = compile_expr(e->b);
      const int jmp_end = emit_jmp(Op::Jmp);
      patch(jmp_false, int(prog.code.size()));
      compile_expr(e->c);
      patch(jmp_end, int(prog.code.size()));
      e->type = tt;
      return tt;
    }
    case ExprKind::Assign:
      return compile_assign(e, as_stmt);
    case ExprKind::Postfix: {
      if (as_stmt && e->a && e->a->kind == ExprKind::Ident) {
        if (Local *loc = find_local(e->a->name)) {
          emit(e->op == "++" ? Op::IncLocal : Op::DecLocal, loc->slot);
          e->type = Type::Int;
          return Type::Int;
        }
      }
      /* i++  →  push i; dup; push 1; add; store i  (result is old value if used) */
      compile_expr(e->a);
      if (!as_stmt) {
        emit(Op::Dup);
      }
      emit(Op::PushI, add_const_i(1));
      emit(e->op == "++" ? Op::Add : Op::Sub);
      compile_lvalue_store(e->a);
      e->type = e->a->type;
      return e->type;
    }
  }
  return Type::Void;
}

static bool expr_lit_num(const Expr *e, float &r)
{
  if (!e) {
    return false;
  }
  if (e->kind == ExprKind::LitFloat) {
    r = e->f;
    return true;
  }
  if (e->kind == ExprKind::LitInt) {
    r = float(e->i);
    return true;
  }
  if (e->kind == ExprKind::Unary && e->a) {
    float inner = 0.0f;
    if (!expr_lit_num(e->a, inner)) {
      return false;
    }
    if (e->op == "-") {
      r = -inner;
      return true;
    }
    if (e->op == "+") {
      r = inner;
      return true;
    }
  }
  return false;
}

static bool expr_is_p(const Expr *e)
{
  if (!e) {
    return false;
  }
  if (e->kind != ExprKind::Attr && e->kind != ExprKind::Ident) {
    return false;
  }
  return e->name == "P" || e->name == "position";
}

static bool expr_const_vec3(const Expr *e, float3 &r)
{
  if (!e) {
    return false;
  }
  if (e->kind == ExprKind::Call &&
      (e->name == "set" || e->name == "vector" || e->name == "vec3") && e->args.size() >= 3)
  {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    if (!expr_lit_num(e->args[0], x) || !expr_lit_num(e->args[1], y) ||
        !expr_lit_num(e->args[2], z))
    {
      return false;
    }
    r = float3(x, y, z);
    return true;
  }
  if (e->kind != ExprKind::VecLit || e->args.size() < 3) {
    return false;
  }
  float x = 0.0f, y = 0.0f, z = 0.0f;
  if (!expr_lit_num(e->args[0], x) || !expr_lit_num(e->args[1], y) || !expr_lit_num(e->args[2], z))
  {
    return false;
  }
  r = float3(x, y, z);
  return true;
}

static bool expr_lvalue_attr(const Expr *e, std::string &r_name, int &r_member)
{
  r_member = -1;
  if (!e) {
    return false;
  }
  if (e->kind == ExprKind::Member && e->a) {
    r_member = (swizzle_count(e->member) == 1) ? (e->member & 3) : -1;
    e = e->a;
  }
  if (e->kind == ExprKind::Attr) {
    r_name = canonical_attr_name(e->name);
    return true;
  }
  if (e->kind == ExprKind::Ident) {
    if (is_element_index_name(e->name)) {
      return false;
    }
    r_name = canonical_attr_name(e->name);
    return true;
  }
  return false;
}

static bool stmt_is_int_attr_inc(const Stmt *s, std::string &r_name)
{
  if (!s || s->kind != StmtKind::Expr || !s->expr) {
    return false;
  }
  const Expr *e = s->expr;
  int member = -1;
  if (e->kind == ExprKind::Postfix && e->op == "++" && e->a) {
    return expr_lvalue_attr(e->a, r_name, member) && member < 0;
  }
  if (e->kind == ExprKind::Assign && e->op == "+=" && e->a) {
    float one = 0.0f;
    if (expr_lit_num(e->b, one) && one == 1.0f) {
      return expr_lvalue_attr(e->a, r_name, member) && member < 0;
    }
  }
  return false;
}

bool Compiler::try_emit_while_cmp_add(Stmt *s)
{
  if (!s || !s->expr || s->expr->kind != ExprKind::Binary) {
    return false;
  }
  uint8_t cmp = 255;
  if (s->expr->op == "<") {
    cmp = 0;
  }
  else if (s->expr->op == "<=") {
    cmp = 1;
  }
  else if (s->expr->op == ">") {
    cmp = 2;
  }
  else if (s->expr->op == ">=") {
    cmp = 3;
  }
  else {
    return false;
  }
  float limit = 0.0f;
  if (!expr_lit_num(s->expr->b, limit)) {
    return false;
  }
  std::string cname;
  int cmember = -1;
  if (!expr_lvalue_attr(s->expr->a, cname, cmember)) {
    return false;
  }
  if (find_local(cname)) {
    return false;
  }

  Vector<Stmt *> stmts;
  if (s->s0 && s->s0->kind == StmtKind::Block) {
    stmts.extend(s->s0->body);
  }
  else if (s->s0) {
    stmts.append(s->s0);
  }
  Stmt *add_stmt = nullptr;
  Vector<std::string> counter_names;
  for (Stmt *st : stmts) {
    std::string inc_name;
    if (stmt_is_int_attr_inc(st, inc_name)) {
      if (inc_name == cname) {
        return false;
      }
      counter_names.append(inc_name);
      continue;
    }
    if (add_stmt) {
      return false;
    }
    add_stmt = st;
  }
  if (!add_stmt || add_stmt->kind != StmtKind::Expr || !add_stmt->expr ||
      add_stmt->expr->kind != ExprKind::Assign || add_stmt->expr->op != "+=")
  {
    return false;
  }
  std::string bname;
  int bmember = -1;
  if (!expr_lvalue_attr(add_stmt->expr->a, bname, bmember) || bname != cname ||
      bmember != cmember)
  {
    return false;
  }
  float step = 0.0f;
  if (!expr_lit_num(add_stmt->expr->b, step)) {
    return false;
  }
  Type ty = Type::Float;
  if (cmember >= 0 || cname == "position" || cname == "normal") {
    ty = Type::Vector;
  }
  WhileCmpAddSpec spec;
  spec.attr = attr_slot(cname, ty, true);
  spec.member = cmember;
  spec.cmp = cmp;
  spec.limit = limit;
  if (cmember == 0) {
    spec.addend.x = step;
  }
  else if (cmember == 1) {
    spec.addend.y = step;
  }
  else if (cmember == 2) {
    spec.addend.z = step;
  }
  else {
    spec.addend.x = step;
  }
  for (const std::string &n : counter_names) {
    spec.counters.append(attr_slot(n, Type::Int, true));
  }
  drop_attr_local(spec.attr);
  const int id = int(prog.while_adds.size());
  prog.while_adds.append(spec);
  emit(Op::WhileCmpAdd, id);
  return true;
}

void Compiler::compile_lvalue_store(Expr *lval, const Type value_type)
{
  /* TOS is the value to store. */
  if (lval->kind == ExprKind::Member) {
    /* stack: value, need object — recompute object, set member, store object. */
    /* Current TOS = new member value. Push object, swap, SetMember, store object. */
    compile_expr(lval->a);
    /* stack: new_member, object — need object, member. Swap via Dup pattern:
     * We emitted object after value, so stack is [value, object]. SetMember uses object then
     * member... define SetMember as: tos=object, nos=member_value → object with member set.
     * Then store object. */
    emit(Op::SetMember, lval->member);
    compile_lvalue_store(lval->a, Type::Void);
    return;
  }
  if (lval->kind == ExprKind::Index) {
    /* TOS = new element. Push container then index; SetIndex → new container. */
    compile_expr(lval->a);
    compile_expr(lval->b);
    emit(Op::SetIndex);
    compile_lvalue_store(lval->a, lval->a ? lval->a->type : Type::Void);
    return;
  }
  if (lval->kind == ExprKind::Ident) {
    if (is_element_index_name(lval->name)) {
      fail("Cannot assign to i@index");
      return;
    }
    if (Local *loc = find_local(lval->name)) {
      emit_coerce(value_type, loc->type);
      emit(Op::StoreLocal, loc->slot);
      return;
    }
    Type t = ident_fallback_type(lval->name);
    if (type_is_array(value_type) && !type_is_array(t)) {
      t = value_type;
    }
    else if (shader_material && value_type != Type::Void && t == Type::Vector &&
             value_type != Type::Vector)
    {
      const std::string aname = canonical_attr_name(lval->name);
      if (aname != "position" && aname != "normal" && aname != "Output") {
        bool have = false;
        for (const AttrInfo &a : prog.attrs) {
          if (a.name == aname) {
            t = a.type;
            have = true;
            break;
          }
        }
        if (!have) {
          /* `p = uv * scale` must stay vector2, not a float3 attribute. */
          t = value_type;
        }
      }
    }
    emit_coerce(value_type, t);
    emit_store_attr(canonical_attr_name(lval->name), t);
    return;
  }
  if (lval->kind == ExprKind::Attr) {
    if (is_element_index_name(lval->name)) {
      fail("Cannot assign to i@index");
      return;
    }
    std::string aname = canonical_attr_name(lval->name);
    Type t = lval->attr_type;
    if (t == Type::Void) {
      t = (aname == "position" || aname == "normal") ? Type::Vector : Type::Float;
    }
    if (type_is_array(value_type) && !type_is_array(t)) {
      t = value_type;
      lval->attr_type = t;
    }
    emit_coerce(value_type, t);
    emit_store_attr(aname, t);
    return;
  }
  fail("Invalid assignment target");
}

int Compiler::attr_slot_of_lvalue(Expr *lval, const bool write)
{
  if (!lval) {
    return -1;
  }
  if (lval->kind == ExprKind::Ident) {
    if (find_local(lval->name)) {
      return -1;
    }
    if (is_element_index_name(lval->name)) {
      return -1;
    }
    const Type t = ident_fallback_type(lval->name);
    return attr_slot(canonical_attr_name(lval->name), t, write);
  }
  if (lval->kind == ExprKind::Attr) {
    if (is_element_index_name(lval->name)) {
      return -1;
    }
    std::string aname = canonical_attr_name(lval->name);
    Type t = lval->attr_type;
    if (t == Type::Void) {
      t = (aname == "position" || aname == "normal") ? Type::Vector : Type::Float;
    }
    return attr_slot(aname, t, write);
  }
  return -1;
}

Type Compiler::compile_assign(Expr *e, const bool as_stmt)
{
  if (as_stmt && e->op == "+=") {
    if (e->a && e->a->kind == ExprKind::Member) {
      const int aslot = attr_slot_of_lvalue(e->a->a, true);
      const int m = e->a->member;
      if (aslot >= 0 && swizzle_count(m) == 1) {
        if (e->b && (e->b->kind == ExprKind::LitFloat || e->b->kind == ExprKind::LitInt)) {
          float3 add(0.0f);
          const float s = e->b->kind == ExprKind::LitFloat ? e->b->f : float(e->b->i);
          if (m == 0) {
            add.x = s;
          }
          else if (m == 1) {
            add.y = s;
          }
          else if (m == 2) {
            add.z = s;
          }
          emit(Op::PushV, add_const_v(add));
          emit(Op::AttrAdd, aslot);
          drop_attr_local(aslot);
          e->type = Type::Vector;
          return e->type;
        }
        compile_expr(e->b);
        emit(Op::PushV, add_const_v(float3(0.0f)));
        emit(Op::SetMember, m);
        emit(Op::AttrAdd, aslot);
        drop_attr_local(aslot);
        e->type = Type::Vector;
        return e->type;
      }
    }
    const int aslot = attr_slot_of_lvalue(e->a, true);
    if (aslot >= 0) {
      compile_expr(e->b);
      emit(Op::AttrAdd, aslot);
      drop_attr_local(aslot);
      e->type = e->b ? e->b->type : Type::Float;
      return e->type;
    }
  }
  if (e->op != "=") {
    compile_expr(e->a);
    compile_expr(e->b);
    if (e->op == "+=") {
      emit(Op::Add);
    }
    else if (e->op == "-=") {
      emit(Op::Sub);
    }
    else if (e->op == "*=") {
      emit(Op::Mul);
    }
    else if (e->op == "/=") {
      emit(Op::Div);
    }
  }
  else {
    Type dest = Type::Void;
    if (e->a && e->a->kind == ExprKind::Attr) {
      dest = e->a->attr_type;
    }
    else if (e->a && e->a->kind == ExprKind::Ident) {
      if (Local *loc = find_local(e->a->name)) {
        dest = loc->type;
      }
    }
    compile_expr(e->b, false, dest);
    if (dest != Type::Void && e->b && e->b->kind == ExprKind::Call &&
        e->b->name == "sample_nearest_surface" && e->b->type != dest)
    {
      fail_at(e,
              "sample_nearest_surface 返回 " + type_name(e->b->type) + "，变量是 " +
                  type_name(dest));
    }
  }
  if (!as_stmt) {
    emit(Op::Dup);
  }
  compile_lvalue_store(e->a, e->b ? e->b->type : Type::Void);
  e->type = e->b ? e->b->type : Type::Float;
  return e->type;
}

bool Compiler::try_emit_gather_samples(Stmt *s)
{
  const bool debug = std::getenv("BLENDER_VEX_PROFILE") != nullptr;
  auto reject = [&](const char *reason) {
    if (debug) {
      std::fprintf(stderr, "VEX_GATHER_REJECT %s\n", reason);
    }
    return false;
  };
  if (!s || s->kind != StmtKind::For || !s->s0 ||
      s->s0->kind != StmtKind::VarDecl || s->s0->var_type != Type::Int || !s->s0->expr ||
      s->s0->expr->kind != ExprKind::LitInt || s->s0->expr->i != 0 || !s->expr ||
      !s->s1 || s->s1->kind != StmtKind::Expr || !s->s1->expr)
  {
    return reject("for-shape");
  }
  const std::string &loop_name = s->s0->var_name;
  const Expr *cond = s->expr;
  if (cond->kind != ExprKind::Binary || cond->op != "<" || !cond->a || !cond->b ||
      cond->a->kind != ExprKind::Ident || cond->a->name != loop_name ||
      cond->b->kind != ExprKind::Call || cond->b->name != "len" ||
      cond->b->args.size() != 1 || !cond->b->args[0] ||
      cond->b->args[0]->kind != ExprKind::Ident)
  {
    return reject("condition");
  }
  const Expr *incr = s->s1->expr;
  if (incr->kind != ExprKind::Postfix || incr->op != "++" || !incr->a ||
      incr->a->kind != ExprKind::Ident || incr->a->name != loop_name)
  {
    return reject("increment");
  }
  const Local *index_array = find_local(cond->b->args[0]->name);
  if (!index_array || index_array->type != Type::IntArray) {
    return reject("index-array");
  }
  if (s->body.is_empty() || !s->body[0]) {
    return reject("empty-body");
  }
  const Stmt *body = s->body[0];
  Span<Stmt *> statements = body->kind == StmtKind::Block ? body->body.as_span() :
                                                            Span<Stmt *>(&s->body[0], 1);
  if (statements.is_empty()) {
    return reject("empty-statements");
  }

  Program::GatherSamples gather;
  gather.index_array_local = index_array->slot;
  for (const Stmt *statement : statements) {
    if (!statement || statement->kind != StmtKind::Expr || !statement->expr ||
        statement->expr->kind != ExprKind::Assign || statement->expr->op != "=" ||
        !statement->expr->a || !statement->expr->b)
    {
      return reject("statement-shape");
    }
    const Expr *lhs = statement->expr->a;
    const Expr *rhs = statement->expr->b;
    if (lhs->kind != ExprKind::Index || !lhs->a || !lhs->b ||
        lhs->a->kind != ExprKind::Ident || lhs->b->kind != ExprKind::Ident ||
        lhs->b->name != loop_name || rhs->kind != ExprKind::Call || rhs->args.size() != 3)
    {
      return reject("assignment-shape");
    }
    Local *output = find_local(lhs->a->name);
    if (!output || !ELEM(output->type, Type::IntArray, Type::FloatArray, Type::VecArray)) {
      return reject("output-array");
    }
    const int bid = builtin_from_name(rhs->name);
    if (bid < 0 || !ELEM(Builtin(bid),
                         Builtin::Point,
                         Builtin::Edge,
                         Builtin::Face,
                         Builtin::Corner,
                         Builtin::Curve,
                         Builtin::Instance))
    {
      return reject("sample-builtin");
    }
    const Expr *geo = rhs->args[0];
    const Expr *name = rhs->args[1];
    const Expr *index = rhs->args[2];
    if (!geo || geo->kind != ExprKind::LitInt || geo->i < 0 || !name ||
        !ELEM(name->kind, ExprKind::LitString, ExprKind::Ident, ExprKind::Attr) || !index ||
        index->kind != ExprKind::Index || !index->a || !index->b ||
        index->a->kind != ExprKind::Ident || index->a->name != cond->b->args[0]->name ||
        index->b->kind != ExprKind::Ident || index->b->name != loop_name)
    {
      return reject("sample-arguments");
    }
    if (name->kind == ExprKind::Ident) {
      const Local *name_local = find_local(name->name);
      if (name_local && name_local->type == Type::String) {
        return reject("dynamic-name");
      }
    }
    int domain = 0;
    if (Builtin(bid) == Builtin::Edge) {
      domain = 1;
    }
    else if (Builtin(bid) == Builtin::Face) {
      domain = 2;
    }
    else if (Builtin(bid) == Builtin::Corner) {
      domain = 3;
    }
    else if (Builtin(bid) == Builtin::Instance) {
      domain = 4;
    }
    else if (Builtin(bid) == Builtin::Curve) {
      domain = 5;
    }
    const std::string attr_name = canonical_attr_name(name->name);
    int sample_slot = -1;
    for (const int i : prog.element_samples.index_range()) {
      const Program::ElementSample &sample = prog.element_samples[i];
      if (sample.geo == geo->i && sample.domain == domain && sample.name == attr_name) {
        sample_slot = i;
        break;
      }
    }
    if (sample_slot < 0) {
      Program::ElementSample sample;
      sample.geo = geo->i;
      sample.domain = domain;
      sample.name = attr_name;
      prog.element_samples.append(std::move(sample));
      sample_slot = int(prog.element_samples.size()) - 1;
    }
    if (geo->i == 0 && !prog.sampled_self_attrs.contains(attr_name)) {
      prog.sampled_self_attrs.append(attr_name);
    }
    gather.output_locals.append(output->slot);
    gather.sample_slots.append(sample_slot);
    gather.output_types.append(output->type);
  }
  if (gather.output_locals.is_empty()) {
    return reject("no-outputs");
  }
  prog.gather_samples.append(std::move(gather));
  emit(Op::GatherSamples, int(prog.gather_samples.size()) - 1);
  return true;
}

void Compiler::compile_stmt(Stmt *s)
{
  const Stmt *previous_error_stmt = error_stmt;
  error_stmt = s;
  struct RestoreErrorStmt {
    const Stmt *&slot;
    const Stmt *previous;
    ~RestoreErrorStmt()
    {
      slot = previous;
    }
  } restore_error_stmt{error_stmt, previous_error_stmt};
  if (!s) {
    return;
  }
  switch (s->kind) {
    case StmtKind::Empty:
      return;
    case StmtKind::Expr:
      if (s->expr) {
        const bool leftover = s->expr->kind != ExprKind::Assign &&
                              s->expr->kind != ExprKind::Postfix;
        compile_expr(s->expr, true);
        if (leftover) {
          emit(Op::Pop);
        }
      }
      return;
    case StmtKind::Block:
      push_scope();
      for (Stmt *b : s->body) {
        compile_stmt(b);
      }
      pop_scope();
      return;
    case StmtKind::VarDecl: {
      Local loc;
      loc.name = s->var_name;
      loc.type = s->var_type;
      loc.slot = prog.local_count++;
      locals.append(loc);
      if (s->expr) {
        const Type got = compile_expr(s->expr, false, loc.type);
        if (loc.type == Type::Void && got != Type::Void) {
          loc.type = got;
          locals.last().type = got;
        }
        emit_coerce(got, loc.type);
        emit(Op::StoreLocal, loc.slot);
      }
      else {
        if (loc.type == Type::Int || loc.type == Type::Bool) {
          emit(Op::PushI, add_const_i(0));
        }
        else if (loc.type == Type::Vector) {
          emit(Op::PushV, add_const_v(float3(0.0f)));
        }
        else if (loc.type == Type::Vector2) {
          emit(Op::Call, int(Builtin::Vec2Fn));
          prog.code.last().a = 0;
        }
        else if (loc.type == Type::Vector4) {
          emit(Op::Call, int(Builtin::Vec4Fn));
          prog.code.last().a = 0;
        }
        else if (loc.type == Type::Rotation) {
          emit(Op::Call, int(Builtin::QuatFn));
          prog.code.last().a = 0;
        }
        else if (loc.type == Type::Color) {
          emit(Op::Call, int(Builtin::ColorFn));
          prog.code.last().a = 0;
        }
        else if (loc.type == Type::String) {
          emit(Op::PushS, add_const_s(""));
        }
        else if (loc.type == Type::IntArray) {
          emit(Op::Call, int(Builtin::ArrayInt));
          prog.code.last().a = 0;
        }
        else if (loc.type == Type::FloatArray) {
          emit(Op::Call, int(Builtin::ArrayFloat));
          prog.code.last().a = 0;
        }
        else if (loc.type == Type::VecArray) {
          emit(Op::Call, int(Builtin::ArrayVec));
          prog.code.last().a = 0;
        }
        else if (loc.type == Type::StringArray) {
          emit(Op::Call, int(Builtin::ArrayStr));
          prog.code.last().a = 0;
        }
        else if (loc.type == Type::MatArray) {
          emit(Op::Call, int(Builtin::ArrayMat));
          prog.code.last().a = 0;
        }
        else if (loc.type == Type::RayArray) {
          emit(Op::Call, int(Builtin::ArrayRay));
          prog.code.last().a = 0;
        }
        else if (loc.type == Type::Matrix) {
          emit(Op::Call, int(Builtin::Identity));
          prog.code.last().a = 0;
        }
        else if (loc.type == Type::Matrix2) {
          emit(Op::Call, int(Builtin::Matrix2Fn));
          prog.code.last().a = 0;
        }
        else if (loc.type == Type::Matrix3) {
          emit(Op::Call, int(Builtin::Matrix3Fn));
          prog.code.last().a = 0;
        }
        else {
          emit(Op::PushF, add_const_f(0.0f));
        }
        emit(Op::StoreLocal, loc.slot);
      }
      for (Stmt *extra : s->body) {
        compile_stmt(extra);
      }
      return;
    }
    case StmtKind::If: {
      compile_expr(s->expr);
      const int jmp_else = emit_jmp(Op::JmpIfFalse);
      compile_stmt(s->s0);
      if (s->s1) {
        const int jmp_end = emit_jmp(Op::Jmp);
        patch(jmp_else, int(prog.code.size()));
        compile_stmt(s->s1);
        patch(jmp_end, int(prog.code.size()));
      }
      else {
        patch(jmp_else, int(prog.code.size()));
      }
      return;
    }
    case StmtKind::While: {
      if (try_emit_while_cmp_add(s)) {
        return;
      }
      const int cond_pc = int(prog.code.size());
      compile_expr(s->expr);
      const int jmp_end = emit_jmp(Op::JmpIfFalse);
      const int break_link = int(loop_break.size());
      loop_break.append(-1);
      loop_continue.append(cond_pc);
      compile_stmt(s->s0);
      emit(Op::Jmp, cond_pc);
      patch(jmp_end, int(prog.code.size()));
      /* patch breaks */
      for (int i = break_link; i < loop_break.size(); i++) {
        if (loop_break[i] >= 0) {
          /* stored as packed: we used a different scheme */
        }
      }
      /* Simpler break: store jmp instruction indices in a side list per loop. */
      (void)break_link;
      loop_break.pop_last();
      loop_continue.pop_last();
      /* Re-do break properly: use Vector<Vector<int>> */
      return;
    }
    case StmtKind::For: {
      push_scope();
      if (s->s0) {
        compile_stmt(s->s0);
      }
      if (try_emit_gather_samples(s)) {
        pop_scope();
        return;
      }
      const int cond_pc = int(prog.code.size());
      if (s->expr) {
        compile_expr(s->expr);
      }
      else {
        emit(Op::PushB, 1);
      }
      const int jmp_end = emit_jmp(Op::JmpIfFalse);
      const int incr_placeholder = int(prog.code.size());
      (void)incr_placeholder;
      compile_stmt(s->body.is_empty() ? nullptr : s->body[0]);
      const int continue_pc = int(prog.code.size());
      (void)continue_pc;
      if (s->s1) {
        compile_stmt(s->s1);
        if (s->s1->kind == StmtKind::Expr) {
          /* compile_stmt already popped */
        }
      }
      emit(Op::Jmp, cond_pc);
      const int end_pc = int(prog.code.size());
      patch(jmp_end, end_pc);
      /* Patch break/continue recorded as imm=-2/-3 during body... handled below with stacks. */
      pop_scope();
      return;
    }
    case StmtKind::Break:
      if (loop_break.is_empty()) {
        fail("break outside loop");
        return;
      }
      loop_break.last() = emit_jmp(Op::Jmp);
      return;
    case StmtKind::Continue:
      if (loop_continue.is_empty()) {
        fail("continue outside loop");
        return;
      }
      emit(Op::Jmp, loop_continue.last());
      return;
    case StmtKind::Return:
      if (s->expr) {
        compile_expr(s->expr);
      }
      else {
        emit(Op::PushI, add_const_i(0));
      }
      emit(Op::Return);
      return;
    case StmtKind::FnDef:
      return;
  }
}

static std::string glsl_ident(const StringRef name, const char *prefix)
{
  std::string s = prefix;
  /* HLSL `lerp` / GLSL `mix` must not appear as `ch_lerp` in EEVEE library GLSL. */
  if (ELEM(name,
           "mix",
           "lerp",
           "step",
           "smoothstep",
           "fract",
           "mod",
           "clamp",
           "min",
           "max",
           "saturate",
           "frac",
           "ddx",
           "ddy",
           "mad",
           "rcp"))
  {
    s += "p_";
  }
  for (const char c : name) {
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')) {
      s += c;
    }
    else {
      s += '_';
    }
  }
  if (s == prefix) {
    s += 'x';
  }
  return s;
}

static bool gpu_channel_arg_name(const Expr *e, std::string &r_name)
{
  if (!e) {
    return false;
  }
  if (!ELEM(e->kind, ExprKind::LitString, ExprKind::Ident, ExprKind::Attr)) {
    return false;
  }
  if (e->name.empty()) {
    return false;
  }
  r_name = e->name;
  return true;
}

static std::string glsl_float_lit(const float f)
{
  char buf[64];
  snprintf(buf, sizeof(buf), "%.9g", double(f));
  std::string s(buf);
  if (s.find('.') == std::string::npos && s.find('e') == std::string::npos &&
      s.find('E') == std::string::npos)
  {
    s += ".0";
  }
  return s;
}

static const char *glsl_comp(const int m)
{
  switch (m) {
    case 1:
      return "y";
    case 2:
      return "z";
    case 3:
      return "w";
    default:
      return "x";
  }
}

static int gpu_align(const int offset, const int a)
{
  return (offset + a - 1) / a * a;
}

static bool gpu_builtin_ok(const StringRef name)
{
  static const char *ok[] = {
      "sin",     "cos",        "tan",      "asin",     "acos",     "atan",     "atan2",
      "abs",     "floor",      "ceil",     "round",    "trunc",    "sqrt",     "exp",
      "log",     "pow",        "min",      "max",      "clamp",    "length",   "distance",
      "dot",     "cross",      "normalize","radians",  "degrees",  "sign",     "frac",
      "fract",   "mix",     "lerp",       "map",      "smooth",   "noise",    "hash",
      "ddx",     "ddy",        "fwidth",   "dFdx",     "dFdy",
      "rand",     "float",
      "int",     "bool",       "vec2",     "vec3",     "vec4",     "vector",   "set",
      "matrix",  "ident",
      "identity","invert",     "invert_matrix", "transpose", "determinant", "det",
      "transform_point", "transform_direction", "project_point", "array", "len",
      "npoints", "nedges",     "nfaces",   "ncorners", "numpt",    "point",    "edge",
      "face",    "prim",       "corner",   "vertex",   "curve",    "instance", "instances",
      "color",   "quaternion", "quat",     "combine_transform", "translation", "rotation",
      "vector2", "vector4",    "matrix2",  "matrix3",  "mat2",     "mat3",     "mat4",
      "scale",   "chf",        "chi",      "chv",      "chb",      "chc",      "chm",
      "chq",     "ch",
      "pointneighbours", "point_neighbours", "pointneighbors", "point_neighbors",
      "neighbours", "neighbors", nullptr};
  for (int i = 0; ok[i]; i++) {
    if (name == ok[i]) {
      return true;
    }
  }
  return false;
}

struct GpuVal {
  std::string s;
  Type type = Type::Float;
  /* Exact integer / `floor` result. Neighbor hashes must use integer add so
   * `(i+1)*c` is not CSE'd to `i*c+c` (not equivalent in float32). */
  bool lattice = false;
  bool intish = false;
};

static bool gpu_float_is_intish(const float f)
{
  return std::isfinite(f) && f == std::floor(f) && std::abs(f) <= 16777216.0f;
}

static bool gpu_shader_geo_fn(const StringRef name)
{
  static const char *const names[] = {
      "npoints",
      "nedges",
      "nfaces",
      "ncorners",
      "numpt",
      "point",
      "edge",
      "face",
      "prim",
      "corner",
      "vertex",
      "curve",
      "instance",
      "instances",
      "pointneighbours",
      "point_neighbours",
      "pointneighbors",
      "point_neighbors",
      "neighbours",
      "neighbors",
      "addpoint",
      "addprim",
      "add_prim",
      "raycast",
      "raycastall",
      "raycast_all",
      "geometry_proximity",
      "nearestpoints",
      "nearpoints",
      "bounding_box",
      "delete_geometry",
      "setattribute",
  };
  for (const char *n : names) {
    if (name == n) {
      return true;
    }
  }
  return false;
}

static void gpu_material_dialect(std::string &s)
{
  auto replace_word = [&](const StringRef from, const StringRef to) {
    size_t pos = 0;
    while ((pos = s.find(from.data(), pos, from.size())) != std::string::npos) {
      const bool left_ok = (pos == 0) ||
                           !(std::isalnum(uchar(s[pos - 1])) || s[pos - 1] == '_');
      const bool right_ok = (pos + from.size() >= s.size()) ||
                             !(std::isalnum(uchar(s[pos + from.size()])) ||
                               s[pos + from.size()] == '_');
      if (left_ok && right_ok) {
        s.replace(pos, from.size(), to.data(), to.size());
        pos += to.size();
      }
      else {
        pos += from.size();
      }
    }
  };
  replace_word("vec2", "float2");
  replace_word("vec3", "float3");
  replace_word("vec4", "float4");
  replace_word("mat2", "float2x2");
  replace_word("mat3", "float3x3");
  replace_word("mat4", "float4x4");
}

static void gpu_emit_program(const Vector<Stmt *> &stmts,
                             const Vector<Stmt *> &user_fn_stmts,
                             Program &prog,
                             const bool shader_material,
                             const uint64_t src_hash)
{
  prog.gpu_ok = false;
  prog.gpu_src.clear();
  prog.gpu_helpers.clear();
  prog.gpu_error.clear();
  prog.gpu_typedef.clear();
  prog.gpu_pack.clear();
  prog.gpu_stride = 0;

  std::string fail;
  bool has_loop = false;
  auto bad = [&](const StringRef m) {
    if (fail.empty()) {
      fail = m;
    }
  };
  auto abort_gpu = [&]() {
    prog.gpu_ok = false;
    prog.gpu_src.clear();
    prog.gpu_helpers.clear();
    if (!fail.empty()) {
      prog.gpu_error = fail;
    }
  };
  auto user_fn_index = [&](const StringRef name) -> int {
    for (const int i : prog.user_fns.index_range()) {
      if (prog.user_fns[i].name == name) {
        return i;
      }
    }
    return -1;
  };
  char uf_pfx[24];
  snprintf(uf_pfx, sizeof(uf_pfx), "wr%x_", uint32_t(src_hash));

  std::function<void(Expr *)> scan_e;
  std::function<void(Stmt *)> scan_s;
  scan_e = [&](Expr *e) {
    if (!e || !fail.empty()) {
      return;
    }
    if (e->kind == ExprKind::LitString) {
      /* String values are only used as sample names / channel names. */
    }
    if (e->kind == ExprKind::Call) {
      if (user_fn_index(e->name) < 0 && !gpu_builtin_ok(e->name)) {
        bad("GPU: unsupported function '" + e->name + "'");
        return;
      }
      if (shader_material && gpu_shader_geo_fn(e->name)) {
        bad("GPU: geometry function not available in shader wrangle");
        return;
      }
      if (ELEM(e->name,
               "point",
               "edge",
               "face",
               "prim",
               "corner",
               "vertex",
               "curve",
               "instance",
               "instances"))
      {
        auto add_sampled = [&](const StringRef raw) {
          if (raw.is_empty() || is_element_index_name(raw)) {
            return;
          }
          const std::string nm = canonical_attr_name(raw);
          Type t = Type::Float;
          if (nm == "position" || nm == "normal") {
            t = Type::Vector;
          }
          else if (nm == "id" || nm == "material_index") {
            t = Type::Int;
          }
          else if (nm == "color") {
            t = Type::Color;
          }
          for (AttrInfo &a : prog.attrs) {
            if (a.name == nm) {
              a.read = true;
              return;
            }
          }
          AttrInfo info;
          info.name = nm;
          info.type = t;
          info.read = true;
          info.write = false;
          prog.attrs.append(info);
        };
        const int n = int(e->args.size());
        int name_i = -1;
        if (n >= 3) {
          name_i = 1;
        }
        else if (n == 2 && e->args[0] &&
                 ELEM(e->args[0]->kind, ExprKind::LitString, ExprKind::Ident, ExprKind::Attr))
        {
          name_i = 0;
        }
        else if (n == 1 && e->args[0] &&
                 ELEM(e->args[0]->kind, ExprKind::LitString, ExprKind::Ident, ExprKind::Attr))
        {
          name_i = 0;
        }
        else if (n == 2) {
          name_i = 1;
        }
        if (name_i >= 0 && e->args[name_i] &&
            ELEM(e->args[name_i]->kind, ExprKind::LitString, ExprKind::Ident, ExprKind::Attr))
        {
          add_sampled(e->args[name_i]->name);
        }
        else {
          add_sampled("P");
        }
      }
      if (ELEM(e->name,
               "pointneighbours",
               "point_neighbours",
               "pointneighbors",
               "point_neighbors",
               "neighbours",
               "neighbors"))
      {
        prog.gpu_neighbors = true;
      }
      std::string ch_nm;
      if (ELEM(e->name, "chi", "chf", "chv", "chb", "chc", "chm", "chq", "ch") &&
          !e->args.is_empty() && gpu_channel_arg_name(e->args[0], ch_nm))
      {
        Type ct = Type::Float;
        if (e->name == "chi") {
          ct = Type::Int;
        }
        else if (e->name == "chb") {
          ct = Type::Bool;
        }
        else if (e->name == "chv") {
          ct = Type::Vector;
        }
        else if (e->name == "chc") {
          ct = Type::Color;
        }
        else if (e->name == "chm") {
          ct = Type::Matrix;
        }
        else if (e->name == "chq") {
          ct = Type::Rotation;
        }
        bool found = false;
        for (const Program::GpuCh &ch : prog.gpu_ch) {
          if (ch.name == ch_nm) {
            found = true;
            break;
          }
        }
        if (!found) {
          Program::GpuCh ch;
          ch.name = std::move(ch_nm);
          ch.type = ct;
          prog.gpu_ch.append(std::move(ch));
        }
      }
    }
    scan_e(e->a);
    scan_e(e->b);
    scan_e(e->c);
    for (Expr *a : e->args) {
      scan_e(a);
    }
  };
  scan_s = [&](Stmt *s) {
    if (!s || !fail.empty()) {
      return;
    }
    if (ELEM(s->kind, StmtKind::While, StmtKind::For)) {
      has_loop = true;
    }
    scan_e(s->expr);
    scan_s(s->s0);
    scan_s(s->s1);
    for (Stmt *b : s->body) {
      scan_s(b);
    }
  };
  for (Stmt *s : stmts) {
    scan_s(s);
  }
  for (Stmt *fn : user_fn_stmts) {
    if (fn == nullptr) {
      continue;
    }
    if (type_is_array(fn->var_type) || fn->var_type == Type::String) {
      bad("GPU: user function return type is not supported");
      break;
    }
    for (const Type pt : fn->param_types) {
      if (type_is_array(pt) || pt == Type::String) {
        bad("GPU: user function array/string parameters");
        break;
      }
    }
    scan_s(fn->s0);
  }
  if (!fail.empty()) {
    abort_gpu();
    return;
  }
  if (!shader_material) {
    /* Straight-line stores (matrix/array/rand assigns) are faster as a C tile
     * kernel than GPU context + SSBO round-trip. GPU is for per-point loops
     * and Jacobi passes (peeled counted loops over point() of written attrs). */
    if (!has_loop && !prog.array_pass_peeled) {
      return;
    }
    for (const AttrInfo &a : prog.attrs) {
      if (ELEM(a.type, Type::String, Type::StringArray) || type_is_array(a.type) ||
          type_is_matrix(a.type))
      {
        return;
      }
    }

    prog.gpu_pack.reinitialize(prog.attrs.size());
    for (const int i : prog.attrs.index_range()) {
      const Type t = prog.attrs[i].type;
      Program::GpuPack pk;
      pk.type = t;
      pk.offset = 0;
      switch (t) {
        case Type::Bool:
        case Type::Int:
        case Type::Float:
          pk.bytes = 4;
          break;
        case Type::Vector:
          pk.bytes = 12;
          break;
        case Type::Vector2:
          pk.bytes = 8;
          break;
        case Type::Vector4:
        case Type::Color:
        case Type::Rotation:
          pk.bytes = 16;
          break;
        default:
          return;
      }
      prog.gpu_pack[i] = pk;
    }
    for (Program::GpuCh &ch : prog.gpu_ch) {
      ch.offset = 0;
      ch.bytes = (ch.type == Type::Vector || ch.type == Type::Color) ? 12 : 4;
    }
    prog.gpu_typedef.clear();
    prog.gpu_stride = 0;
    prog.gpu_soa = true;
  }

  struct GpuLocal {
    std::string name;
    Type type = Type::Float;
    bool lattice = false;
    bool intish = false;
  };
  Vector<GpuLocal> local_stack;
  auto find_local = [&](const StringRef n) -> GpuLocal * {
    for (int i = int(local_stack.size()) - 1; i >= 0; i--) {
      if (local_stack[i].name == n) {
        return &local_stack[i];
      }
    }
    return nullptr;
  };
  auto is_local = [&](const StringRef n) { return find_local(n) != nullptr; };
  auto local_type = [&](const StringRef n) -> Type {
    if (const GpuLocal *loc = find_local(n)) {
      return loc->type;
    }
    return Type::Void;
  };
  auto attr_i = [&](const StringRef raw) -> int {
    const std::string nm = canonical_attr_name(raw);
    for (const int i : prog.attrs.index_range()) {
      if (prog.attrs[i].name == nm) {
        return i;
      }
    }
    return -1;
  };
  auto attr_local = [&](const int i) {
    return glsl_ident(prog.attrs[i].name, "a_");
  };
  auto gl_type = [&](const Type t) -> const char * {
    switch (t) {
      case Type::Bool:
        return "bool";
      case Type::Int:
        return "int";
      case Type::Vector2:
        return "vec2";
      case Type::Vector:
        return "vec3";
      case Type::Vector4:
      case Type::Color:
      case Type::Rotation:
        return "vec4";
      case Type::Matrix2:
        return "mat2";
      case Type::Matrix3:
        return "mat3";
      case Type::Matrix:
        return "mat4";
      case Type::IntArray:
      case Type::FloatArray:
      case Type::VecArray:
      case Type::MatArray:
      case Type::RayArray:
        return "WrangleArr";
      default:
        return "float";
    }
  };
  auto load_field = [&](const int i, const StringRef idx) {
    const std::string ix = "(" + std::string(idx) + ")";
    const std::string b = "a" + std::to_string(i) + "_in";
    switch (prog.attrs[i].type) {
      case Type::Vector2:
        return "vec2(" + b + "[" + ix + "*2], " + b + "[" + ix + "*2+1])";
      case Type::Vector:
        return "vec3(" + b + "[" + ix + "*3], " + b + "[" + ix + "*3+1], " + b + "[" + ix +
               "*3+2])";
      case Type::Vector4:
      case Type::Color:
      case Type::Rotation:
        return "vec4(" + b + "[" + ix + "*4], " + b + "[" + ix + "*4+1], " + b + "[" + ix +
               "*4+2], " + b + "[" + ix + "*4+3])";
      default:
        return b + "[" + ix + "]";
    }
  };
  auto store_field = [&](const int i, const StringRef idx, const StringRef val) {
    const std::string ix = "(" + std::string(idx) + ")";
    const std::string b = "a" + std::to_string(i) + "_out";
    const std::string v = std::string(val);
    if (prog.attrs[i].type == Type::Vector2) {
      return b + "[" + ix + "*2] = (" + v + ").x; " + b + "[" + ix + "*2+1] = (" + v + ").y";
    }
    if (prog.attrs[i].type == Type::Vector) {
      return b + "[" + ix + "*3] = (" + v + ").x; " + b + "[" + ix + "*3+1] = (" + v + ").y; " +
             b + "[" + ix + "*3+2] = (" + v + ").z";
    }
    if (ELEM(prog.attrs[i].type, Type::Vector4, Type::Color, Type::Rotation)) {
      return b + "[" + ix + "*4] = (" + v + ").x; " + b + "[" + ix + "*4+1] = (" + v + ").y; " +
             b + "[" + ix + "*4+2] = (" + v + ").z; " + b + "[" + ix + "*4+3] = (" + v + ").w";
    }
    return b + "[" + ix + "] = " + v;
  };

  std::function<GpuVal(Expr *, Type)> em;
  std::function<std::string(Stmt *)> ems;
  bool in_user_fn = false;
  Type user_fn_ret = Type::Void;

  auto cast_to = [&](const GpuVal &v, const Type t) {
    if (v.type == t) {
      return v.s;
    }
    if (t == Type::Float) {
      if (type_is_vec_like(v.type) || v.type == Type::Rotation) {
        return "(" + v.s + ").x";
      }
      return "float(" + v.s + ")";
    }
    if (t == Type::Int) {
      return "int(" + v.s + ")";
    }
    if (t == Type::Bool) {
      return "bool(" + v.s + ")";
    }
    if (t == Type::Vector2) {
      if (v.type == Type::Float || v.type == Type::Int || v.type == Type::Bool) {
        return "vec2(" + v.s + ")";
      }
      if (ELEM(v.type, Type::Vector, Type::Vector4, Type::Color, Type::Rotation)) {
        return "(" + v.s + ").xy";
      }
      return "vec2(" + v.s + ")";
    }
    if (t == Type::Vector) {
      if (v.type == Type::Float || v.type == Type::Int) {
        return "vec3(" + v.s + ")";
      }
      if (v.type == Type::Vector2) {
        return "vec3(" + v.s + ", 0.0)";
      }
      if (ELEM(v.type, Type::Color, Type::Rotation, Type::Vector4)) {
        return "(" + v.s + ").xyz";
      }
      return "vec3(" + v.s + ")";
    }
    if (t == Type::Vector4) {
      if (v.type == Type::Vector2) {
        return "vec4(" + v.s + ", 0.0, 0.0)";
      }
      if (v.type == Type::Vector) {
        return "vec4(" + v.s + ", 0.0)";
      }
      return "vec4(" + v.s + ")";
    }
    if (t == Type::Color || t == Type::Rotation) {
      if (v.type == Type::Vector) {
        return "vec4(" + v.s + ", " + (t == Type::Rotation ? "1.0" : "1.0") + ")";
      }
      return "vec4(" + v.s + ")";
    }
    if (t == Type::Matrix2) {
      if (ELEM(v.type, Type::Float, Type::Int, Type::Bool)) {
        const std::string s = (v.type == Type::Float) ? v.s : ("float(" + v.s + ")");
        return "mat2(" + s + ", 0.0, 0.0, " + s + ")";
      }
      return "mat2(" + v.s + ")";
    }
    if (t == Type::Matrix3) {
      if (ELEM(v.type, Type::Float, Type::Int, Type::Bool)) {
        const std::string s = (v.type == Type::Float) ? v.s : ("float(" + v.s + ")");
        return "mat3(" + s + ")";
      }
      return "mat3(" + v.s + ")";
    }
    if (t == Type::Matrix) {
      /* GLSL mat4(s) fills all 4 diagonal entries including m44. We want scale:
       * xyz diagonal = s, m44 stays 1. */
      if (ELEM(v.type, Type::Float, Type::Int, Type::Bool)) {
        const std::string s = (v.type == Type::Float) ? v.s : ("float(" + v.s + ")");
        return "mat4(" + s + ",0.0,0.0,0.0, 0.0," + s + ",0.0,0.0, 0.0,0.0," + s +
               ",0.0, 0.0,0.0,0.0,1.0)";
      }
      if (ELEM(v.type, Type::Vector, Type::Color, Type::Rotation)) {
        return "mat4((" + v.s + ").x,0.0,0.0,0.0, 0.0,(" + v.s +
               ").y,0.0,0.0, 0.0,0.0,(" + v.s + ").z,0.0, 0.0,0.0,0.0,1.0)";
      }
      return "mat4(" + v.s + ")";
    }
    return v.s;
  };

  auto gpu_lerp = [&](const std::string &a, const std::string &b, const std::string &t) {
    /* Named helper so `t` (often a swizzle) is evaluated once, and so Jenkins
     * hash.glsl `#define mix` cannot rewrite the interpolation. */
    return "wr_interp(" + a + ", " + b + ", float(" + t + "))";
  };

  /* Force a value copy so `hash21(i)` cannot mutate the caller's lattice index
   * if a backend passes floatN by reference. Emit `vecN` so shader-material
   * dialect can rewrite it to `floatN`. */
  auto gpu_vec_copy = [&](const std::string &s, const Type t) -> std::string {
    if (t == Type::Vector2) {
      return "vec2((" + s + ").x, (" + s + ").y)";
    }
    if (t == Type::Vector) {
      return "vec3((" + s + ").x, (" + s + ").y, (" + s + ").z)";
    }
    if (ELEM(t, Type::Vector4, Type::Color, Type::Rotation)) {
      return "vec4((" + s + ").x, (" + s + ").y, (" + s + ").z, (" + s + ").w)";
    }
    return s;
  };

  auto gpu_floor_call = [&](const std::string &xs, const Type rt) -> std::string {
    if (rt == Type::Vector2) {
      return "wr_floor2(" + xs + ")";
    }
    if (rt == Type::Vector) {
      return "wr_floor3(" + xs + ")";
    }
    if (ELEM(rt, Type::Vector4, Type::Color, Type::Rotation)) {
      return "wr_floor4(" + xs + ")";
    }
    return "wr_floor1(" + xs + ")";
  };

  auto gpu_fract_call = [&](const std::string &xs, const Type rt) -> std::string {
    if (rt == Type::Vector2) {
      return "wr_fract2(" + xs + ")";
    }
    if (rt == Type::Vector) {
      return "wr_fract3(" + xs + ")";
    }
    if (ELEM(rt, Type::Vector4, Type::Color, Type::Rotation)) {
      return "wr_fract4(" + xs + ")";
    }
    return "wr_fract1(" + xs + ")";
  };

  auto gpu_lat_call = [&](const std::string &xs, const Type rt) -> std::string {
    if (rt == Type::Vector2) {
      return "wr_lat2(" + xs + ")";
    }
    if (rt == Type::Vector) {
      return "wr_lat3(" + xs + ")";
    }
    if (ELEM(rt, Type::Vector4, Type::Color, Type::Rotation)) {
      return "wr_lat4(" + xs + ")";
    }
    return "wr_lat1(" + xs + ")";
  };

  auto gpu_int_type = [&](const Type t) -> const char * {
    if (t == Type::Vector2) {
      return "int2";
    }
    if (t == Type::Vector) {
      return "int3";
    }
    if (ELEM(t, Type::Vector4, Type::Color, Type::Rotation)) {
      return "int4";
    }
    return "int";
  };

  auto gpu_vec_ctor = [&](const Type t) -> const char * {
    if (t == Type::Vector2) {
      return "vec2";
    }
    if (t == Type::Vector) {
      return "vec3";
    }
    if (ELEM(t, Type::Vector4, Type::Color, Type::Rotation)) {
      return "vec4";
    }
    return "float";
  };

  auto gpu_vector_from_args = [&](const int want, const Vector<GpuVal> &args) -> GpuVal {
    GpuVal r;
    Vector<std::string, 16> comps;
    for (const GpuVal &a : args) {
      const int nc = type_pack_comp_count(a.type);
      if (nc <= 1) {
        comps.append(cast_to(a, Type::Float));
      }
      else {
        const char *sw = "xyzw";
        for (int k = 0; k < nc; k++) {
          comps.append("(" + a.s + ")." + sw[k]);
        }
      }
    }
    if (args.size() == 1 && comps.size() == 1) {
      const std::string splat = comps[0];
      while (int(comps.size()) < want) {
        comps.append(splat);
      }
    }
    while (int(comps.size()) < want) {
      comps.append("0.0");
    }
    const char *ctor = (want == 2) ? "vec2" : (want == 4) ? "vec4" : "vec3";
    std::string s = ctor;
    s += "(";
    for (int i = 0; i < want; i++) {
      if (i > 0) {
        s += ", ";
      }
      s += comps[i];
    }
    s += ")";
    r.s = std::move(s);
    r.type = (want == 2) ? Type::Vector2 : (want == 4 ? Type::Vector4 : Type::Vector);
    if (args.size() == 1) {
      r.intish = args[0].intish || args[0].lattice;
      r.lattice = r.intish;
    }
    return r;
  };

  std::function<bool(Expr *)> expr_is_intish;
  expr_is_intish = [&](Expr *e) -> bool {
    if (!e) {
      return false;
    }
    if (e->kind == ExprKind::LitInt) {
      return true;
    }
    if (e->kind == ExprKind::LitFloat) {
      return gpu_float_is_intish(e->f);
    }
    if (e->kind == ExprKind::Unary && e->op == "-" && e->a) {
      return expr_is_intish(e->a);
    }
    if (e->kind == ExprKind::VecLit) {
      if (e->args.is_empty()) {
        return false;
      }
      for (Expr *a : e->args) {
        if (!expr_is_intish(a)) {
          return false;
        }
      }
      return true;
    }
    if (e->kind == ExprKind::Call && is_vector_ctor_name(e->name))
    {
      if (e->args.is_empty()) {
        return false;
      }
      for (Expr *a : e->args) {
        if (!expr_is_intish(a)) {
          return false;
        }
      }
      return true;
    }
    return false;
  };

  auto gpu_src_comp_count = [&](const Type t) -> int {
    if (type_has_xyzw(t)) {
      return type_pack_comp_count(t);
    }
    if (ELEM(t, Type::Float, Type::Int, Type::Bool)) {
      return 1;
    }
    return 0;
  };

  std::function<std::string(Expr *)> lvalue;
  lvalue = [&](Expr *e) -> std::string {
    if (!e) {
      bad("GPU: missing lvalue");
      return "0.0";
    }
    if (e->kind == ExprKind::Ident) {
      if (is_element_index_name(e->name)) {
        bad("GPU: cannot assign to index");
        return "elem_i";
      }
      if (is_local(e->name)) {
        return glsl_ident(e->name, "l_");
      }
      const int ai = attr_i(e->name);
      if (ai >= 0) {
        return attr_local(ai);
      }
      bad("GPU: unknown lvalue");
      return "elem_i";
    }
    if (e->kind == ExprKind::Attr) {
      const int ai = attr_i(e->name);
      if (ai >= 0) {
        return attr_local(ai);
      }
      bad("GPU: unknown attribute");
      return "elem_i";
    }
    if (e->kind == ExprKind::Member) {
      if (!e->name.empty()) {
        return lvalue(e->a) + "." + e->name;
      }
      return lvalue(e->a) + "." + glsl_comp(e->member);
    }
    if (e->kind == ExprKind::Index) {
      return lvalue(e->a) + "[" + em(e->b, Type::Int).s + "]";
    }
    bad("GPU: unsupported lvalue");
    return "elem_i";
  };

  em = [&](Expr *e, Type hint) -> GpuVal {
    GpuVal r;
    if (!e) {
      r.s = "0.0";
      return r;
    }
    switch (e->kind) {
      case ExprKind::LitInt:
        r.s = std::to_string(e->i);
        r.type = Type::Int;
        r.intish = true;
        r.lattice = true;
        return r;
      case ExprKind::LitFloat:
        r.s = glsl_float_lit(e->f);
        r.type = Type::Float;
        r.intish = gpu_float_is_intish(e->f);
        r.lattice = r.intish;
        return r;
      case ExprKind::LitBool:
        r.s = e->lit_b ? "true" : "false";
        r.type = Type::Bool;
        return r;
      case ExprKind::LitString:
        r.s = "0";
        r.type = Type::String;
        return r;
      case ExprKind::Ident: {
        if (is_element_index_name(e->name)) {
          if (shader_material) {
            bad("GPU: i@index is not available in shader wrangle");
          }
          r.s = "elem_i";
          r.type = Type::Int;
          return r;
        }
        if (e->name == "numpt" || e->name == "numelem") {
          if (shader_material) {
            bad("GPU: npoints is not available in shader wrangle");
          }
          r.s = "npoints";
          r.type = Type::Int;
          return r;
        }
        if (GpuLocal *loc = find_local(e->name)) {
          r.s = glsl_ident(e->name, "l_");
          r.type = loc->type != Type::Void ? loc->type :
                   (e->type != Type::Void ? e->type : Type::Float);
          r.lattice = loc->lattice;
          r.intish = loc->intish;
          return r;
        }
        if (in_user_fn) {
          bad("GPU: user functions cannot read attributes");
          r.s = "0.0";
          return r;
        }
        const int ai = attr_i(e->name);
        if (ai >= 0) {
          r.s = attr_local(ai);
          r.type = prog.attrs[ai].type;
          return r;
        }
        bad("GPU: unknown identifier");
        r.s = "0.0";
        return r;
      }
      case ExprKind::Attr: {
        if (in_user_fn) {
          bad("GPU: user functions cannot read attributes");
          r.s = "0.0";
          return r;
        }
        if (is_element_index_name(e->name)) {
          if (shader_material) {
            bad("GPU: i@index is not available in shader wrangle");
          }
          r.s = "elem_i";
          r.type = Type::Int;
          return r;
        }
        const int ai = attr_i(e->name);
        if (ai >= 0) {
          r.s = attr_local(ai);
          r.type = prog.attrs[ai].type;
          return r;
        }
        bad("GPU: unknown attribute");
        r.s = "0.0";
        return r;
      }
      case ExprKind::Unary: {
        GpuVal a = em(e->a, hint);
        if (e->op == "-") {
          r.s = "-(" + a.s + ")";
          r.type = a.type;
          r.lattice = a.lattice;
          r.intish = a.intish;
        }
        else if (e->op == "!") {
          r.s = "!(" + a.s + ")";
          r.type = Type::Bool;
        }
        else {
          r = a;
        }
        return r;
      }
      case ExprKind::Binary: {
        GpuVal a = em(e->a, Type::Void);
        GpuVal b = em(e->b, Type::Void);
        if (e->op == "*" && a.type == Type::Rotation && b.type == Type::Rotation) {
          r.s = "wr_qmul(" + a.s + ", " + b.s + ")";
          r.type = Type::Rotation;
          return r;
        }
        if (e->op == "*" && a.type == Type::Rotation &&
            ELEM(b.type, Type::Vector, Type::Vector2, Type::Vector4, Type::Color))
        {
          r.s = "wr_qrot(" + a.s + ", " + cast_to(b, Type::Vector) + ")";
          r.type = Type::Vector;
          return r;
        }
        if (e->op == "*" && b.type == Type::Rotation &&
            ELEM(a.type, Type::Vector, Type::Vector2, Type::Vector4, Type::Color))
        {
          r.s = "wr_qrot(" + b.s + ", " + cast_to(a, Type::Vector) + ")";
          r.type = Type::Vector;
          return r;
        }
        if (e->op == "*" && type_is_matrix(a.type) && type_is_vec_like(b.type)) {
          r.s = "((" + a.s + ")*(" + b.s + "))";
          r.type = vec_type_for_dim(type_linear_dim(a.type));
          return r;
        }
        if (e->op == "*" && type_is_vec_like(a.type) && type_is_matrix(b.type)) {
          r.s = "((" + a.s + ")*(" + b.s + "))";
          r.type = vec_type_for_dim(type_linear_dim(b.type));
          return r;
        }
        if (e->op == "*" && type_is_matrix(a.type) &&
            ELEM(b.type, Type::Float, Type::Int, Type::Bool))
        {
          r.s = "((" + a.s + ")*(" + cast_to(b, Type::Float) + "))";
          r.type = a.type;
          return r;
        }
        if (e->op == "*" && type_is_matrix(b.type) &&
            ELEM(a.type, Type::Float, Type::Int, Type::Bool))
        {
          r.s = "((" + cast_to(a, Type::Float) + ")*(" + b.s + "))";
          r.type = b.type;
          return r;
        }
        Type rt = a.type;
        if (a.type == Type::Vector4 || b.type == Type::Vector4) {
          rt = Type::Vector4;
        }
        else if (a.type == Type::Vector || b.type == Type::Vector) {
          rt = Type::Vector;
        }
        else if (a.type == Type::Vector2 || b.type == Type::Vector2) {
          rt = Type::Vector2;
        }
        else if (a.type == Type::Matrix || b.type == Type::Matrix) {
          rt = Type::Matrix;
        }
        else if (a.type == Type::Matrix3 || b.type == Type::Matrix3) {
          rt = Type::Matrix3;
        }
        else if (a.type == Type::Matrix2 || b.type == Type::Matrix2) {
          rt = Type::Matrix2;
        }
        else if (a.type == Type::Float || b.type == Type::Float) {
          rt = Type::Float;
        }
        else if (a.type == Type::Int && b.type == Type::Int) {
          rt = Type::Int;
        }
        const std::string as = cast_to(a, rt);
        const std::string bs = cast_to(b, rt);
        if (e->op == "**") {
          r.s = "pow(" + as + ", " + bs + ")";
          r.type = rt == Type::Int ? Type::Float : rt;
        }
        else if ((e->op == "+" || e->op == "-") && shader_material &&
                 (a.lattice || a.intish) &&
                 (b.lattice || b.intish || expr_is_intish(e->b)))
        {
          /* Integer add for lattice ± offset so hash21(i+(1,0)) is
           * `int(i)+1`, not a float add the backend can rewrite into
           * `i*c+c` after inlining `p*c`. */
          const char *it = gpu_int_type(rt);
          const char *ft = gpu_vec_ctor(rt);
          r.s = std::string(ft) + "(" + it + "(" + as + ")" + e->op + it + "(" + bs + "))";
          r.type = rt;
          r.lattice = true;
          r.intish = true;
        }
        else if (e->op == "*" && shader_material &&
                 (rt == Type::Float || type_is_vec_like(rt) ||
                  ELEM(rt, Type::Color, Type::Rotation)))
        {
          /* `precise` multiply: `(i+1)*c` must not become `i*c+c`. */
          r.s = "wr_mul(" + as + ", " + bs + ")";
          r.type = rt;
        }
        else if (e->op == "+" || e->op == "-" || e->op == "*" || e->op == "/") {
          r.s = "((" + as + ")" + e->op + "(" + bs + "))";
          r.type = rt;
          if ((e->op == "+" || e->op == "-") && a.lattice &&
              (b.lattice || b.intish))
          {
            r.lattice = true;
            r.intish = true;
          }
        }
        else if (e->op == "%") {
          if (rt == Type::Int) {
            r.s = "((" + as + ")%(" + bs + "))";
          }
          else {
            r.s = "mod(" + as + ", " + bs + ")";
          }
          r.type = rt;
        }
        else if (e->op == "<" || e->op == "<=" || e->op == ">" || e->op == ">=" || e->op == "==" ||
                 e->op == "!=")
        {
          if (rt == Type::Vector && (e->op == "==" || e->op == "!=")) {
            r.s = std::string(e->op == "==" ? "" : "!") + "all(equal(" + as + ", " + bs + "))";
          }
          else {
            r.s = "((" + as + ")" + e->op + "(" + bs + "))";
          }
          r.type = Type::Bool;
        }
        else if (e->op == "&&" || e->op == "||") {
          r.s = "((" + as + ")" + e->op + "(" + bs + "))";
          r.type = Type::Bool;
        }
        else {
          bad("GPU: unsupported operator");
          r.s = "0.0";
        }
        return r;
      }
      case ExprKind::Member: {
        GpuVal a = em(e->a, Type::Void);
        const int n = swizzle_count(e->member);
        const int src_n = gpu_src_comp_count(a.type);
        bool native = src_n >= 2;
        for (int i = 0; i < n && native; i++) {
          if (swizzle_axis(e->member, i) >= src_n) {
            native = false;
          }
        }
        if (native && !e->name.empty()) {
          r.s = "(" + a.s + ")." + e->name;
        }
        else if (native && n == 1) {
          r.s = "(" + a.s + ")." + glsl_comp(e->member);
        }
        else {
          auto axis_s = [&](const int axis) -> std::string {
            if (src_n <= 1) {
              return (axis == 0) ? cast_to(a, Type::Float) : std::string("0.0");
            }
            if (axis >= src_n) {
              return "0.0";
            }
            static const char *sw = "xyzw";
            return "(" + a.s + ")." + sw[axis];
          };
          if (n <= 1) {
            r.s = axis_s(swizzle_axis(e->member, 0));
          }
          else {
            const char *ctor = (n == 2) ? "vec2" : (n == 3) ? "vec3" : "vec4";
            r.s = ctor;
            r.s += "(";
            for (int i = 0; i < n; i++) {
              if (i) {
                r.s += ", ";
              }
              r.s += axis_s(swizzle_axis(e->member, i));
            }
            r.s += ")";
          }
        }
        r.type = swizzle_result_type(n);
        r.lattice = a.lattice;
        r.intish = a.intish;
        return r;
      }
      case ExprKind::Index: {
        GpuVal a = em(e->a, Type::Void);
        GpuVal i = em(e->b, Type::Int);
        if (a.type == Type::VecArray) {
          r.s = "wr_arr_v(" + a.s + ", " + cast_to(i, Type::Int) + ")";
          r.type = Type::Vector;
        }
        else if (a.type == Type::IntArray) {
          r.s = "wr_arr_i(" + a.s + ", " + cast_to(i, Type::Int) + ")";
          r.type = Type::Int;
        }
        else if (type_is_array(a.type)) {
          r.s = "wr_arr_f(" + a.s + ", " + cast_to(i, Type::Int) + ")";
          r.type = Type::Float;
        }
        else {
          r.s = "((" + a.s + ")[" + cast_to(i, Type::Int) + "])";
          if (type_is_matrix(a.type)) {
            r.type = vec_type_for_dim(type_linear_dim(a.type));
          }
          else {
            r.type = Type::Float;
          }
        }
        return r;
      }
      case ExprKind::Cond: {
        GpuVal c = em(e->a, Type::Bool);
        GpuVal t = em(e->b, hint);
        GpuVal f = em(e->c, hint);
        r.type = t.type;
        r.s = "((" + c.s + ") ? (" + cast_to(t, r.type) + ") : (" + cast_to(f, r.type) + "))";
        /* `rank >= 3 ? 1.0 : 0.0` must stay an integer lattice offset so
         * `cell + i1` uses integer add, not a float add the backend can
         * reassociate inside `hash4(cell + i1)`. */
        if ((t.intish || t.lattice) && (f.intish || f.lattice)) {
          r.intish = true;
          r.lattice = true;
        }
        return r;
      }
      case ExprKind::VecLit: {
        if (type_is_matrix(hint) || e->args.size() > 4) {
          if (type_is_matrix(hint) && e->args.size() == 1) {
            r.s = cast_to(em(e->args[0], Type::Float), hint);
            r.type = hint;
            return r;
          }
          if (hint == Type::Matrix && e->args.size() == 3) {
            const std::string x = cast_to(em(e->args[0], Type::Float), Type::Float);
            const std::string y = cast_to(em(e->args[1], Type::Float), Type::Float);
            const std::string z = cast_to(em(e->args[2], Type::Float), Type::Float);
            r.s = "mat4(" + x + ",0.0,0.0,0.0, 0.0," + y + ",0.0,0.0, 0.0,0.0," + z +
                  ",0.0, 0.0,0.0,0.0,1.0)";
            r.type = Type::Matrix;
            return r;
          }
          if (hint == Type::Matrix || e->args.size() >= 16) {
            std::string s = "mat4(";
            const char *id16[16] = {"1.0",
                                    "0.0",
                                    "0.0",
                                    "0.0",
                                    "0.0",
                                    "1.0",
                                    "0.0",
                                    "0.0",
                                    "0.0",
                                    "0.0",
                                    "1.0",
                                    "0.0",
                                    "0.0",
                                    "0.0",
                                    "0.0",
                                    "1.0"};
            for (int i = 0; i < 16; i++) {
              if (i) {
                s += ", ";
              }
              if (i < int(e->args.size())) {
                s += cast_to(em(e->args[i], Type::Float), Type::Float);
              }
              else {
                s += id16[i];
              }
            }
            s += ")";
            r.s = s;
            r.type = Type::Matrix;
            return r;
          }
          std::string s = "wr_arr_new(1)";
          for (int i = 0; i < int(e->args.size()) && i < 120; i++) {
            s = "wr_arr_set_f(" + s + ", " + std::to_string(i) + ", " +
                cast_to(em(e->args[i], Type::Float), Type::Float) + ")";
          }
          r.s = s;
          r.type = Type::FloatArray;
          return r;
        }
        Vector<GpuVal> packed;
        int packed_n = 0;
        for (Expr *c : e->args) {
          const GpuVal a = em(c, Type::Void);
          packed.append(a);
          packed_n += type_pack_comp_count(a.type);
        }
        int want = 3;
        if (hint == Type::Vector2) {
          want = 2;
        }
        else if (hint == Type::Vector4 || hint == Type::Color || hint == Type::Rotation) {
          want = 4;
        }
        else if (hint == Type::Vector) {
          want = 3;
        }
        else if (packed_n == 2) {
          want = 2;
        }
        else if (packed_n >= 4) {
          want = 4;
        }
        r = gpu_vector_from_args(want, packed);
        if (hint == Type::Color || hint == Type::Rotation) {
          r.type = hint;
        }
        return r;
      }
      case ExprKind::Assign: {
        if (e->a && e->a->kind == ExprKind::Index && e->a->a && type_is_array(e->a->a->type)) {
          GpuVal arr = em(e->a->a, Type::Void);
          GpuVal ix = em(e->a->b, Type::Int);
          GpuVal v = em(e->b, array_elem_type(arr.type));
          std::string fn = "wr_arr_set_f";
          if (arr.type == Type::IntArray) {
            fn = "wr_arr_set_i";
          }
          else if (arr.type == Type::VecArray) {
            fn = "wr_arr_set_v";
          }
          const std::string lv = lvalue(e->a->a);
          r.s = "(" + lv + " = " + fn + "(" + arr.s + ", " + cast_to(ix, Type::Int) + ", " + v.s +
                "))";
          r.type = arr.type;
          return r;
        }
        Type dest = Type::Void;
        if (e->a) {
          if (e->a->kind == ExprKind::Member) {
            dest = swizzle_result_type(swizzle_count(e->a->member));
          }
          else if (e->a->kind == ExprKind::Ident || e->a->kind == ExprKind::Attr) {
            if (is_local(e->a->name)) {
              dest = local_type(e->a->name);
            }
            else {
              const int ai = attr_i(e->a->name);
              if (ai >= 0) {
                dest = prog.attrs[ai].type;
              }
              else if (e->a->type != Type::Void) {
                dest = e->a->type;
              }
            }
          }
        }
        GpuVal v = em(e->b, dest != Type::Void ? dest : hint);
        const std::string lv = lvalue(e->a);
        std::string op = e->op.empty() ? "=" : e->op;
        if (dest != Type::Void && dest != v.type) {
          r.s = "(" + lv + " " + op + " " + cast_to(v, dest) + ")";
          r.type = dest;
        }
        else {
          r.s = "(" + lv + " " + op + " " + v.s + ")";
          r.type = v.type;
        }
        if (e->a && e->a->kind == ExprKind::Ident) {
          if (GpuLocal *loc = find_local(e->a->name)) {
            loc->lattice = v.lattice;
            loc->intish = v.intish;
          }
        }
        r.lattice = v.lattice;
        r.intish = v.intish;
        return r;
      }
      case ExprKind::Postfix: {
        const std::string lv = lvalue(e->a);
        r.s = "(" + lv + e->op + ")";
        r.type = Type::Int;
        return r;
      }
      case ExprKind::Call: {
        const std::string &fn = e->name;
        auto arg = [&](const int i, Type h = Type::Void) {
          return (i < int(e->args.size())) ? em(e->args[i], h) : GpuVal{"0.0", Type::Float};
        };
        const int ufi = user_fn_index(fn);
        if (ufi >= 0) {
          const UserFn &uf = prog.user_fns[ufi];
          std::string s = glsl_ident(fn, uf_pfx);
          s += "(";
          for (int i = 0; i < int(e->args.size()); i++) {
            if (i) {
              s += ", ";
            }
            const Type pt = (i < int(uf.param_types.size())) ? uf.param_types[i] : Type::Float;
            const GpuVal av = arg(i, pt);
            const std::string as = cast_to(av, pt);
            if (type_is_vec_like(pt)) {
              const std::string copied = gpu_vec_copy(as, pt);
              s += (av.lattice || av.intish) ? gpu_lat_call(copied, pt) : copied;
            }
            else if ((av.lattice || av.intish) && pt == Type::Float) {
              /* wr_lat1 returns float. Never wrap `int` parameters: GLSL has no
               * implicit float→int conversion, and EEVEE goes magenta. */
              s += gpu_lat_call(as, Type::Float);
            }
            else {
              s += as;
            }
          }
          s += ")";
          r.s = std::move(s);
          r.type = uf.ret;
          return r;
        }
        if (fn == "rand" || fn == "random") {
          if (e->args.is_empty()) {
            r.s = "wr_rand0(elem_i, rand_seq)";
          }
          else if (arg(0).type == Type::Int) {
            r.s = "wr_hash_to_float(uint(" + arg(0).s + "))";
          }
          else if (arg(0).type == Type::Vector) {
            const GpuVal a = arg(0);
            r.s = "float(wr_hash3(floatBitsToUint((" + a.s + ").x), floatBitsToUint((" + a.s +
                  ").y), floatBitsToUint((" + a.s + ").z))) / 4294967295.0";
          }
          else {
            r.s = "wr_hash_to_float(floatBitsToUint(" + cast_to(arg(0), Type::Float) + "))";
          }
          r.type = Type::Float;
          return r;
        }
        if (fn == "npoints" || fn == "numpt") {
          r.s = "npoints";
          r.type = Type::Int;
          return r;
        }
        if (fn == "nedges") {
          r.s = "nedges";
          r.type = Type::Int;
          return r;
        }
        if (fn == "nfaces") {
          r.s = "nfaces";
          r.type = Type::Int;
          return r;
        }
        if (fn == "ncorners") {
          r.s = "ncorners";
          r.type = Type::Int;
          return r;
        }
        if (fn == "len") {
          r.s = "(" + arg(0).s + ").count";
          r.type = Type::Int;
          return r;
        }
        if (fn == "ident" || fn == "identity") {
          const Type t = type_is_matrix(hint) ? hint : Type::Matrix;
          if (t == Type::Matrix2) {
            r.s = "mat2(1.0)";
          }
          else if (t == Type::Matrix3) {
            r.s = "mat3(1.0)";
          }
          else {
            r.s = "mat4(1.0)";
          }
          r.type = t;
          return r;
        }
        if (fn == "invert" || fn == "invert_matrix") {
          const GpuVal a = arg(0);
          const Type t = type_is_matrix(a.type) ? a.type :
                         (type_is_matrix(hint) ? hint : Type::Matrix);
          r.s = "inverse(" + (type_is_matrix(a.type) ? a.s : cast_to(a, t)) + ")";
          r.type = t;
          return r;
        }
        if (fn == "transpose") {
          const GpuVal a = arg(0);
          const Type t = type_is_matrix(a.type) ? a.type :
                         (type_is_matrix(hint) ? hint : Type::Matrix);
          r.s = "transpose(" + (type_is_matrix(a.type) ? a.s : cast_to(a, t)) + ")";
          r.type = t;
          return r;
        }
        if (fn == "determinant" || fn == "det") {
          r.s = "determinant(" + arg(0).s + ")";
          r.type = Type::Float;
          return r;
        }
        if (fn == "transform_point") {
          r.s = "(" + arg(1, Type::Matrix).s + " * vec4(" +
                cast_to(arg(0, Type::Vector), Type::Vector) + ", 1.0)).xyz";
          r.type = Type::Vector;
          return r;
        }
        if (fn == "transform_direction") {
          r.s = "(" + arg(1, Type::Matrix).s + " * vec4(" +
                cast_to(arg(0, Type::Vector), Type::Vector) + ", 0.0)).xyz";
          r.type = Type::Vector;
          return r;
        }
        if (fn == "project_point") {
          r.s = "(" + arg(1, Type::Matrix).s + " * vec4(" +
                cast_to(arg(0, Type::Vector), Type::Vector) + ", 1.0)).xyz";
          r.type = Type::Vector;
          return r;
        }
        if (fn == "mix" || fn == "lerp") {
          const GpuVal a0 = arg(0);
          const GpuVal a1 = arg(1);
          const Type rt = type_is_vec_like(a0.type) ? a0.type :
                           (type_is_vec_like(a1.type) ? a1.type : Type::Float);
          r.s = gpu_lerp(cast_to(a0, rt), cast_to(a1, rt), cast_to(arg(2), Type::Float));
          r.type = rt;
          return r;
        }
        if (fn == "map") {
          const GpuVal v = arg(0);
          const Type rt = type_is_vec_like(v.type) ? v.type : Type::Float;
          const std::string vs = cast_to(v, rt);
          const std::string a = cast_to(arg(1), rt);
          const std::string b = cast_to(arg(2), rt);
          const std::string c = cast_to(arg(3), rt);
          const std::string d = cast_to(arg(4), rt);
          /* Match CPU: from_min==from_max → to_min. Guard the divide so mix() is not NaN. */
          r.s = gpu_lerp(c,
                         "(" + c + " + ((" + vs + " - " + a + ") * (" + d + " - " + c +
                             ") / ((" + b + " - " + a + ") + (1.0 - step(1e-8, abs(" + b +
                             " - " + a + "))))))",
                         "step(1e-8, abs(" + b + " - " + a + "))");
          r.type = rt;
          return r;
        }
        if (fn == "smooth") {
          if (e->args.size() >= 3) {
            const GpuVal x = arg(2);
            const Type rt = type_is_vec_like(x.type) ? x.type : Type::Float;
            r.s = "smoothstep(" + cast_to(arg(0), rt) + ", " + cast_to(arg(1), rt) + ", " +
                  cast_to(x, rt) + ")";
            r.type = rt;
            return r;
          }
          const GpuVal v = arg(0);
          const Type rt = type_is_vec_like(v.type) ? v.type : Type::Float;
          r.s = "smoothstep(0.0, 1.0, " + cast_to(v, rt) + ")";
          r.type = rt;
          return r;
        }
        if (is_vector_ctor_name(fn))
        {
          Vector<GpuVal> packed;
          Vector<Type, 4> arg_types;
          for (int i = 0; i < int(e->args.size()); i++) {
            const GpuVal a = arg(i);
            packed.append(a);
            arg_types.append(a.type);
          }
          int want = vector_ctor_want(fn);
          if (fn == "set") {
            const Type rt = set_fn_result_type(arg_types);
            want = (rt == Type::Vector2) ? 2 : (rt == Type::Vector4 ? 4 : 3);
          }
          r = gpu_vector_from_args(want, packed);
          return r;
        }
        if (fn == "rotation" || fn == "quaternion" || fn == "quat") {
          r.s = "vec4(" + cast_to(arg(0), Type::Float) + ", " +
                (e->args.size() >= 2 ? cast_to(arg(1), Type::Float) : std::string("0.0")) + ", " +
                (e->args.size() >= 3 ? cast_to(arg(2), Type::Float) : std::string("0.0")) + ", " +
                (e->args.size() >= 4 ? cast_to(arg(3), Type::Float) : std::string("1.0")) + ")";
          r.type = Type::Rotation;
          return r;
        }
        if (fn == "color") {
          r.s = "vec4(" + cast_to(arg(0), Type::Float) + ", " +
                cast_to(arg(1), Type::Float) + ", " + cast_to(arg(2), Type::Float) + ", " +
                (e->args.size() >= 4 ? cast_to(arg(3), Type::Float) : std::string("1.0")) + ")";
          r.type = Type::Color;
          return r;
        }
        if (fn == "float") {
          const GpuVal a0 = arg(0);
          r.s = cast_to(a0, Type::Float);
          r.type = Type::Float;
          r.lattice = a0.lattice;
          r.intish = a0.intish;
          return r;
        }
        if (fn == "int") {
          r.s = "int(" + arg(0).s + ")";
          r.type = Type::Int;
          r.lattice = true;
          r.intish = true;
          return r;
        }
        if (fn == "bool") {
          r.s = "bool(" + arg(0).s + ")";
          r.type = Type::Bool;
          return r;
        }
        if (fn == "matrix" || fn == "matrix4" || fn == "mat4") {
          if (e->args.size() == 1) {
            r.s = cast_to(arg(0), Type::Matrix);
            r.type = Type::Matrix;
            return r;
          }
          if (e->args.size() == 3) {
            const std::string x = cast_to(arg(0, Type::Float), Type::Float);
            const std::string y = cast_to(arg(1, Type::Float), Type::Float);
            const std::string z = cast_to(arg(2, Type::Float), Type::Float);
            r.s = "mat4(" + x + ",0.0,0.0,0.0, 0.0," + y + ",0.0,0.0, 0.0,0.0," + z +
                  ",0.0, 0.0,0.0,0.0,1.0)";
            r.type = Type::Matrix;
            return r;
          }
          std::string s = "mat4(";
          const char *id16[16] = {"1.0",
                                  "0.0",
                                  "0.0",
                                  "0.0",
                                  "0.0",
                                  "1.0",
                                  "0.0",
                                  "0.0",
                                  "0.0",
                                  "0.0",
                                  "1.0",
                                  "0.0",
                                  "0.0",
                                  "0.0",
                                  "0.0",
                                  "1.0"};
          for (int i = 0; i < 16; i++) {
            if (i) {
              s += ", ";
            }
            if (i < int(e->args.size())) {
              s += cast_to(arg(i, Type::Float), Type::Float);
            }
            else {
              s += id16[i];
            }
          }
          s += ")";
          r.s = s;
          r.type = Type::Matrix;
          return r;
        }
        if (fn == "array") {
          Type at = Type::FloatArray;
          if (!e->args.is_empty()) {
            GpuVal a0 = arg(0);
            at = type_is_array(a0.type) ? a0.type : array_type_of(a0.type);
            for (int i = 0; i < int(e->args.size()); i++) {
              GpuVal ai = arg(i);
              if (ai.type == Type::Vector) {
                at = Type::VecArray;
                break;
              }
              if (ai.type == Type::Float) {
                at = Type::FloatArray;
              }
            }
          }
          int kind = 1;
          if (at == Type::IntArray) {
            kind = 0;
          }
          if (at == Type::VecArray) {
            kind = 2;
          }
          std::string s = "wr_arr_new(" + std::to_string(kind) + ")";
          for (int i = 0; i < int(e->args.size()) && i < 40; i++) {
            GpuVal ai = arg(i);
            if (kind == 2) {
              s = "wr_arr_set_v(" + s + ", " + std::to_string(i) + ", " +
                  cast_to(ai, Type::Vector) + ")";
            }
            else if (kind == 0) {
              s = "wr_arr_set_i(" + s + ", " + std::to_string(i) + ", " +
                  cast_to(ai, Type::Int) + ")";
            }
            else {
              s = "wr_arr_set_f(" + s + ", " + std::to_string(i) + ", " +
                  cast_to(ai, Type::Float) + ")";
            }
          }
          r.s = s;
          r.type = at;
          return r;
        }
        if (fn == "chi" || fn == "chf" || fn == "chv" || fn == "chb" || fn == "chc" ||
            fn == "chm" || fn == "chq" || fn == "ch")
        {
          if (in_user_fn) {
            bad("GPU: chf/chi/... cannot be used inside user functions");
            r.s = "0.0";
            return r;
          }
          std::string nm;
          if (!e->args.is_empty()) {
            gpu_channel_arg_name(e->args[0], nm);
          }
          if (!nm.empty()) {
            r.s = glsl_ident(nm, "ch_");
            r.type = Type::Float;
            auto wrap_chf = [&]() {
              if (fn == "chf" || fn == "ch") {
                r.s = "float(" + r.s + ")";
                r.type = Type::Float;
              }
            };
            for (const int ci : prog.gpu_ch.index_range()) {
              if (prog.gpu_ch[ci].name == nm) {
                r.type = prog.gpu_ch[ci].type;
                wrap_chf();
                return r;
              }
            }
            const int ai = attr_i(nm);
            if (ai >= 0) {
              r.s = attr_local(ai);
              r.type = prog.attrs[ai].type;
              return r;
            }
            if (fn == "chi") {
              r.type = Type::Int;
            }
            else if (fn == "chb") {
              r.type = Type::Bool;
            }
            else if (fn == "chv") {
              r.type = Type::Vector;
            }
            else if (fn == "chc") {
              r.type = Type::Color;
            }
            else if (fn == "chm") {
              r.type = Type::Matrix;
            }
            else if (fn == "chq") {
              r.type = Type::Rotation;
            }
            wrap_chf();
            return r;
          }
          r.s = (fn == "chi" || fn == "chb") ? "0" : "0.0";
          r.type = (fn == "chi") ? Type::Int :
                   (fn == "chv" ? Type::Vector :
                    (fn == "chm" ? Type::Matrix :
                     (fn == "chq" ? Type::Rotation : Type::Float)));
          return r;
        }
        if (fn == "pointneighbours" || fn == "point_neighbours" || fn == "pointneighbors" ||
            fn == "point_neighbors" || fn == "neighbours" || fn == "neighbors")
        {
          GpuVal ix{"elem_i", Type::Int};
          if (int(e->args.size()) >= 2) {
            ix = arg(1, Type::Int);
          }
          else if (int(e->args.size()) == 1 && e->args[0] &&
                   e->args[0]->kind != ExprKind::LitInt)
          {
            ix = arg(0, Type::Int);
          }
          r.s = "wr_neighbours(" + cast_to(ix, Type::Int) + ")";
          r.type = Type::IntArray;
          return r;
        }
        if (fn == "point" || fn == "edge" || fn == "face" || fn == "prim" || fn == "corner" ||
            fn == "vertex" || fn == "curve" || fn == "instance" || fn == "instances")
        {
          int name_i = -1;
          int idx_i = -1;
          int geo_i = -1;
          const int n = int(e->args.size());
          if (n >= 3) {
            geo_i = 0;
            name_i = 1;
            idx_i = 2;
          }
          else if (n == 2) {
            if (e->args[0] &&
                ELEM(e->args[0]->kind, ExprKind::LitString, ExprKind::Ident, ExprKind::Attr))
            {
              name_i = 0;
              idx_i = 1;
            }
            else {
              geo_i = 0;
              name_i = 1;
            }
          }
          else if (n == 1) {
            if (e->args[0] &&
                ELEM(e->args[0]->kind, ExprKind::LitString, ExprKind::Ident, ExprKind::Attr))
            {
              name_i = 0;
            }
            else {
              idx_i = 0;
            }
          }
          if (geo_i >= 0 && e->args[geo_i] && e->args[geo_i]->kind == ExprKind::LitInt &&
              e->args[geo_i]->i != 0)
          {
            bad("GPU: sampling other geometry inputs is CPU-only");
            r.s = "0.0";
            return r;
          }
          std::string nm = "position";
          if (name_i >= 0 && e->args[name_i]) {
            nm = canonical_attr_name(e->args[name_i]->name);
          }
          const int ai = attr_i(nm);
          GpuVal ix = idx_i >= 0 ? arg(idx_i, Type::Int) : GpuVal{"elem_i", Type::Int};
          const std::string ixs = "wr_clampi(" + cast_to(ix, Type::Int) + ", n_elem)";
          if (ai < 0) {
            r.s = "0.0";
            r.type = Type::Float;
            return r;
          }
          /* Indexed sample is always the working array from the previous Jacobi pass
           * (`elems_in`), never the in-register value this thread may have already
           * written this pass. */
          r.s = load_field(ai, ixs);
          r.type = prog.attrs[ai].type;
          return r;
        }
        if (fn == "noise" || fn == "hash") {
          GpuVal p = arg(0, Type::Vector);
          r.s = "float(wr_hash3(floatBitsToUint((" + cast_to(p, Type::Vector) +
                ").x), floatBitsToUint((" + cast_to(p, Type::Vector) +
                ").y), floatBitsToUint((" + cast_to(p, Type::Vector) +
                ").z))) / 4294967295.0";
          r.type = Type::Float;
          return r;
        }
        if (fn == "atan2") {
          r.s = "atan(" + arg(0).s + ", " + arg(1).s + ")";
          r.type = Type::Float;
          return r;
        }
        if (fn == "clamp") {
          r.s = "clamp(" + arg(0).s + ", " + arg(1).s + ", " + arg(2).s + ")";
          r.type = arg(0).type;
          return r;
        }
        if (fn == "pow") {
          const GpuVal a0 = arg(0);
          const Type rt = type_is_vec_like(a0.type) ? a0.type : Type::Float;
          r.s = "pow(" + cast_to(arg(0), rt) + ", " + cast_to(arg(1), rt) + ")";
          r.type = rt;
          return r;
        }
        if (fn == "min" || fn == "max") {
          const GpuVal a0 = arg(0);
          const GpuVal a1 = arg(1);
          Type rt = dominant_vec_type(a0.type, a1.type);
          if (rt == Type::Void) {
            rt = (a0.type == Type::Int && a1.type == Type::Int) ? Type::Int : Type::Float;
          }
          r.s = fn + "(" + cast_to(a0, rt) + ", " + cast_to(a1, rt) + ")";
          r.type = rt;
          return r;
        }
        if (fn == "cross") {
          const GpuVal a0 = arg(0);
          const GpuVal a1 = arg(1);
          if (a0.type == Type::Vector2 || a1.type == Type::Vector2) {
            const std::string av = cast_to(a0, Type::Vector2);
            const std::string bv = cast_to(a1, Type::Vector2);
            r.s = "((" + av + ").x * (" + bv + ").y - (" + av + ").y * (" + bv + ").x)";
            r.type = Type::Float;
            return r;
          }
          if (a0.type == Type::Vector4 || a1.type == Type::Vector4) {
            r.s = "vec4(cross((" + cast_to(a0, Type::Vector4) + ").xyz, (" +
                  cast_to(a1, Type::Vector4) + ").xyz), 0.0)";
            r.type = Type::Vector4;
            return r;
          }
          r.s = "cross(" + cast_to(a0, Type::Vector) + ", " + cast_to(a1, Type::Vector) + ")";
          r.type = Type::Vector;
          return r;
        }
        if (fn == "dot") {
          const GpuVal a0 = arg(0);
          const GpuVal a1 = arg(1);
          Type vt = dominant_vec_type(a0.type, a1.type);
          if (vt == Type::Void) {
            vt = Type::Vector;
          }
          r.s = "dot(" + cast_to(a0, vt) + ", " + cast_to(a1, vt) + ")";
          r.type = Type::Float;
          return r;
        }
        if (fn == "distance") {
          const GpuVal a0 = arg(0);
          const GpuVal a1 = arg(1);
          Type vt = dominant_vec_type(a0.type, a1.type);
          if (vt == Type::Void) {
            vt = Type::Vector;
          }
          r.s = "distance(" + cast_to(a0, vt) + ", " + cast_to(a1, vt) + ")";
          r.type = Type::Float;
          return r;
        }
        if (fn == "length") {
          r.s = "length(" + arg(0).s + ")";
          r.type = Type::Float;
          return r;
        }
        if (fn == "normalize") {
          const GpuVal a0 = arg(0);
          const Type vt = type_is_vec_like(a0.type) ? a0.type : Type::Vector;
          r.s = "normalize(" + cast_to(a0, vt) + ")";
          r.type = vt;
          return r;
        }
        if (fn == "radians") {
          const GpuVal a0 = arg(0);
          const Type rt = type_is_vec_like(a0.type) ? a0.type : Type::Float;
          r.s = "radians(" + cast_to(a0, rt) + ")";
          r.type = rt;
          return r;
        }
        if (fn == "degrees") {
          const GpuVal a0 = arg(0);
          const Type rt = type_is_vec_like(a0.type) ? a0.type : Type::Float;
          r.s = "degrees(" + cast_to(a0, rt) + ")";
          r.type = rt;
          return r;
        }
        if (fn == "frac" || fn == "fract") {
          const GpuVal a0 = arg(0);
          Type rt = type_is_vec_like(a0.type) ? a0.type :
                    (e->type != Type::Void && type_is_vec_like(e->type) ? e->type : Type::Float);
          if (hint == Type::Vector2 && a0.type != Type::Vector2 && type_has_xyzw(a0.type)) {
            rt = Type::Vector2;
          }
          const std::string xs = cast_to(a0, rt);
          /* Component-wise helper: some backends' `floor(float2)` is scalar-only,
           * which turns value-noise lattice `i` into a splat and seams the grid. */
          r.s = shader_material ? gpu_fract_call(xs, rt) : ("(" + xs + " - floor(" + xs + "))");
          r.type = rt;
          return r;
        }
        if (fn == "floor" || fn == "ceil" || fn == "round" || fn == "trunc") {
          const GpuVal a0 = arg(0);
          Type rt = type_is_vec_like(a0.type) ? a0.type :
                    (e->type != Type::Void && type_is_vec_like(e->type) ? e->type : Type::Float);
          if (hint == Type::Vector2 && a0.type != Type::Vector2 && type_has_xyzw(a0.type)) {
            rt = Type::Vector2;
          }
          const std::string xs = cast_to(a0, rt);
          if (fn == "floor" && shader_material) {
            r.s = gpu_floor_call(xs, rt);
          }
          else {
            r.s = std::string(fn) + "(" + xs + ")";
          }
          r.type = rt;
          if (fn == "floor") {
            r.lattice = true;
            r.intish = true;
          }
          return r;
        }
        if (fn == "sign") {
          r.s = "sign(" + arg(0).s + ")";
          r.type = arg(0).type;
          return r;
        }
        if (fn == "ddx" || fn == "dFdx" || fn == "ddy" || fn == "dFdy" || fn == "fwidth") {
          const GpuVal a0 = arg(0);
          const Type rt = type_is_vec_like(a0.type) ? a0.type : Type::Float;
          if (shader_material) {
            const char *helper = "wr_ddx";
            if (fn == "ddy" || fn == "dFdy") {
              helper = "wr_ddy";
            }
            else if (fn == "fwidth") {
              helper = "wr_fwidth";
            }
            r.s = std::string(helper) + "(" + cast_to(a0, rt) + ", kg)";
          }
          else if (rt == Type::Vector2) {
            r.s = "vec2(0.0)";
          }
          else if (rt == Type::Vector) {
            r.s = "vec3(0.0)";
          }
          else if (ELEM(rt, Type::Vector4, Type::Color, Type::Rotation)) {
            r.s = "vec4(0.0)";
          }
          else {
            r.s = "0.0";
          }
          r.type = rt;
          return r;
        }
        if (fn == "matrix2" || fn == "mat2") {
          if (e->args.is_empty()) {
            r.s = "mat2(1.0)";
          }
          else if (e->args.size() == 1) {
            r.s = cast_to(arg(0), Type::Matrix2);
          }
          else if (e->args.size() == 2) {
            r.s = "mat2(" + cast_to(arg(0), Type::Vector2) + ", " +
                  cast_to(arg(1), Type::Vector2) + ")";
          }
          else {
            r.s = "mat2(" + cast_to(arg(0), Type::Float) + ", " +
                  (e->args.size() >= 2 ? cast_to(arg(1), Type::Float) : std::string("0.0")) +
                  ", " +
                  (e->args.size() >= 3 ? cast_to(arg(2), Type::Float) : std::string("0.0")) +
                  ", " +
                  (e->args.size() >= 4 ? cast_to(arg(3), Type::Float) : std::string("1.0")) + ")";
          }
          r.type = Type::Matrix2;
          return r;
        }
        if (fn == "matrix3" || fn == "mat3") {
          if (e->args.is_empty()) {
            r.s = "mat3(1.0)";
          }
          else if (e->args.size() == 1) {
            r.s = cast_to(arg(0), Type::Matrix3);
          }
          else {
            r.s = "mat3(" + arg(0).s + ")";
          }
          r.type = Type::Matrix3;
          return r;
        }
        /* Default: unary/binary GLSL builtin of the same name. */
        {
          std::string s = fn + "(";
          for (int i = 0; i < int(e->args.size()); i++) {
            if (i) {
              s += ", ";
            }
            s += arg(i).s;
          }
          s += ")";
          r.s = s;
          r.type = e->args.is_empty() ? Type::Float : arg(0).type;
          return r;
        }
      }
    }
    r.s = "0.0";
    return r;
  };

  ems = [&](Stmt *s) -> std::string {
    if (!s) {
      return "";
    }
    switch (s->kind) {
      case StmtKind::Empty:
        return "";
      case StmtKind::Expr:
        if (!s->expr) {
          return "";
        }
        return "  " + em(s->expr, Type::Void).s + ";\n";
      case StmtKind::Block: {
        const int mark = int(local_stack.size());
        std::string b = "  {\n";
        for (Stmt *in : s->body) {
          b += ems(in);
        }
        b += "  }\n";
        local_stack.resize(mark);
        return b;
      }
      case StmtKind::VarDecl: {
        Type t = s->var_type;
        GpuVal init;
        const bool has_init = s->expr != nullptr;
        if (has_init) {
          init = em(s->expr, t == Type::Void ? Type::Void : t);
          if (t == Type::Void) {
            t = init.type;
          }
        }
        else if (t == Type::Void) {
          t = Type::Float;
        }
        local_stack.append({s->var_name, t, init.lattice, init.intish});
        std::string line = "  " + std::string(gl_type(t)) + " " +
                           glsl_ident(s->var_name, "l_");
        if (has_init) {
          line += " = " + cast_to(init, t);
        }
        else if (t == Type::Matrix) {
          line += " = mat4(1.0)";
        }
        else if (t == Type::Matrix2) {
          line += " = mat2(1.0)";
        }
        else if (t == Type::Matrix3) {
          line += " = mat3(1.0)";
        }
        else if (type_is_array(t)) {
          int kind = 1;
          if (t == Type::IntArray) {
            kind = 0;
          }
          if (t == Type::VecArray) {
            kind = 2;
          }
          line += " = wr_arr_new(" + std::to_string(kind) + ")";
        }
        else if (t == Type::Vector2) {
          line += " = vec2(0.0)";
        }
        else if (t == Type::Vector) {
          line += " = vec3(0.0)";
        }
        else if (t == Type::Vector4) {
          line += " = vec4(0.0)";
        }
        else if (t == Type::Color) {
          line += " = vec4(0.0, 0.0, 0.0, 1.0)";
        }
        else if (t == Type::Rotation) {
          line += " = vec4(0.0, 0.0, 0.0, 1.0)";
        }
        else if (t == Type::Int) {
          line += " = 0";
        }
        else if (t == Type::Bool) {
          line += " = false";
        }
        else {
          line += " = 0.0";
        }
        line += ";\n";
        std::string extra;
        for (Stmt *b : s->body) {
          extra += ems(b);
        }
        return line + extra;
      }
      case StmtKind::If: {
        std::string b = "  if (" + em(s->expr, Type::Bool).s + ") {\n" + ems(s->s0) + "  }\n";
        if (s->s1) {
          b += "  else {\n" + ems(s->s1) + "  }\n";
        }
        return b;
      }
      case StmtKind::While:
        return "  while (" + em(s->expr, Type::Bool).s + ") {\n" + ems(s->s0) + "  }\n";
      case StmtKind::For: {
        const int mark = int(local_stack.size());
        std::string init, incr, cond = "true";
        if (s->s0 && s->s0->kind == StmtKind::VarDecl) {
          Type t = s->s0->var_type == Type::Void ? Type::Int : s->s0->var_type;
          local_stack.append({s->s0->var_name, t, t == Type::Int, t == Type::Int});
          init = std::string(gl_type(t)) + " " + glsl_ident(s->s0->var_name, "l_");
          if (s->s0->expr) {
            init += " = " + cast_to(em(s->s0->expr, t), t);
          }
          else {
            init += (t == Type::Int) ? " = 0" : " = 0.0";
          }
        }
        else if (s->s0 && s->s0->kind == StmtKind::Expr && s->s0->expr) {
          init = em(s->s0->expr, Type::Void).s;
        }
        if (s->expr) {
          cond = em(s->expr, Type::Bool).s;
        }
        if (s->s1 && s->s1->kind == StmtKind::Expr && s->s1->expr) {
          incr = em(s->s1->expr, Type::Void).s;
        }
        std::string b = "  for (" + init + "; " + cond + "; " + incr + ") {\n";
        for (Stmt *in : s->body) {
          b += ems(in);
        }
        b += "  }\n";
        local_stack.resize(mark);
        return b;
      }
      case StmtKind::Break:
        return "  break;\n";
      case StmtKind::Continue:
        return "  continue;\n";
      case StmtKind::Return:
        if (in_user_fn && s->expr && user_fn_ret != Type::Void) {
          return "  return " + cast_to(em(s->expr, user_fn_ret), user_fn_ret) + ";\n";
        }
        return "  return;\n";
      case StmtKind::FnDef:
        return "";
    }
    return "";
  };

  auto emit_fn_sig = [&](const Stmt *fn) -> std::string {
    std::string s;
    s += (fn->var_type == Type::Void) ? "void" : gl_type(fn->var_type);
    s += " ";
    s += glsl_ident(fn->var_name, uf_pfx);
    s += "(";
    for (int i = 0; i < int(fn->param_names.size()); i++) {
      if (i) {
        s += ", ";
      }
      const Type pt = (i < int(fn->param_types.size())) ? fn->param_types[i] : Type::Float;
      s += gl_type(pt);
      s += " ";
      s += glsl_ident(fn->param_names[i], "arg_");
    }
    s += ")";
    return s;
  };

  std::string helpers;
  for (Stmt *fn : user_fn_stmts) {
    if (fn == nullptr) {
      continue;
    }
    helpers += emit_fn_sig(fn);
    helpers += ";\n";
  }
  for (Stmt *fn : user_fn_stmts) {
    if (fn == nullptr) {
      continue;
    }
    in_user_fn = true;
    user_fn_ret = fn->var_type;
    local_stack.clear();
    for (int pi = 0; pi < int(fn->param_names.size()); pi++) {
      const Type pt = (pi < int(fn->param_types.size())) ? fn->param_types[pi] : Type::Float;
      local_stack.append({fn->param_names[pi], pt});
    }
    helpers += emit_fn_sig(fn);
    helpers += "\n{\n";
    /* Copy `arg_*` into `l_*` so `p = fract(p * …)` cannot mutate the caller's
     * cell index if the backend passes float2 by reference. `arg_` avoids MSL
     * `#define in`. */
    for (int pi = 0; pi < int(fn->param_names.size()); pi++) {
      const Type pt = (pi < int(fn->param_types.size())) ? fn->param_types[pi] : Type::Float;
      helpers += "  ";
      helpers += gl_type(pt);
      helpers += " ";
      helpers += glsl_ident(fn->param_names[pi], "l_");
      helpers += " = ";
      helpers += glsl_ident(fn->param_names[pi], "arg_");
      helpers += ";\n";
    }
    helpers += ems(fn->s0);
    helpers += "}\n";
    local_stack.clear();
  }
  in_user_fn = false;
  user_fn_ret = Type::Void;

  std::string body;
  for (Stmt *s : stmts) {
    body += ems(s);
  }
  if (!fail.empty()) {
    abort_gpu();
    return;
  }

  if (shader_material) {
    gpu_material_dialect(helpers);
    gpu_material_dialect(body);
    /* Jenkins hash.glsl `#define mix` and SMAA `#define lerp` must not apply.
     * Interpolation goes through `wr_interp` so `mix(` never appears in wrangle GLSL. */
    const char *interp =
        "float wr_interp(float wr_a, float wr_b, float wr_t) { return wr_a + wr_t * (wr_b - wr_a); }\n"
        "float2 wr_interp(float2 wr_a, float2 wr_b, float wr_t) { return wr_a + wr_t * (wr_b - wr_a); }\n"
        "float3 wr_interp(float3 wr_a, float3 wr_b, float wr_t) { return wr_a + wr_t * (wr_b - wr_a); }\n"
        "float4 wr_interp(float4 wr_a, float4 wr_b, float wr_t) { return wr_a + wr_t * (wr_b - wr_a); }\n"
        /* Snap through int so lattice points are exact integers. GPU `floor` can
         * return 3.0000002; hashing that vs `(2.0+1.0)` breaks value-noise seams. */
        "float wr_floor1(float v) { return float(int(floor(v))); }\n"
        "float2 wr_floor2(float2 v) { return float2(int2(floor(v))); }\n"
        "float3 wr_floor3(float3 v) { return float3(int3(floor(v))); }\n"
        "float4 wr_floor4(float4 v) { return float4(int4(floor(v))); }\n"
        "float wr_fract1(float v) { return v - wr_floor1(v); }\n"
        "float2 wr_fract2(float2 v) { return v - wr_floor2(v); }\n"
        "float3 wr_fract3(float3 v) { return v - wr_floor3(v); }\n"
        "float4 wr_fract4(float4 v) { return v - wr_floor4(v); }\n"
        /* Canonical integer keys for lattice hashes. Combined with wr_mul so
         * inlined `hash21(i+(1,0))` cannot CSE `(i+1)*c` into `i*c+c`.
         * Do not use GLSL `precise`: Blender's material preprocessor treats it
         * as a duplicated type qualifier and the whole EEVEE shader fails. */
        "float wr_lat1(float v) { return float(int(v)); }\n"
        "float2 wr_lat2(float2 v) { return float2(int2(v)); }\n"
        "float3 wr_lat3(float3 v) { return float3(int3(v)); }\n"
        "float4 wr_lat4(float4 v) { return float4(int4(v)); }\n"
        "float wr_mul(float a, float b) { return a * b; }\n"
        "float2 wr_mul(float2 a, float2 b) { return a * b; }\n"
        "float3 wr_mul(float3 a, float3 b) { return a * b; }\n"
        "float4 wr_mul(float4 a, float4 b) { return a * b; }\n";
    const char *deriv =
        "#ifdef GPU_FRAGMENT_SHADER\n"
        "float wr_ddx(float v, KernelGlobals kg) { return gpu_dfdx(v) * derivative_scale_get(kg); }\n"
        "float2 wr_ddx(float2 v, KernelGlobals kg) { return gpu_dfdx(v) * derivative_scale_get(kg); }\n"
        "float3 wr_ddx(float3 v, KernelGlobals kg) { return gpu_dfdx(v) * derivative_scale_get(kg); }\n"
        "float4 wr_ddx(float4 v, KernelGlobals kg) { return gpu_dfdx(v) * derivative_scale_get(kg); }\n"
        "float wr_ddy(float v, KernelGlobals kg) { return gpu_dfdy(v) * derivative_scale_get(kg); }\n"
        "float2 wr_ddy(float2 v, KernelGlobals kg) { return gpu_dfdy(v) * derivative_scale_get(kg); }\n"
        "float3 wr_ddy(float3 v, KernelGlobals kg) { return gpu_dfdy(v) * derivative_scale_get(kg); }\n"
        "float4 wr_ddy(float4 v, KernelGlobals kg) { return gpu_dfdy(v) * derivative_scale_get(kg); }\n"
        "float wr_fwidth(float v, KernelGlobals kg) { return gpu_fwidth(v) * derivative_scale_get(kg); }\n"
        "float2 wr_fwidth(float2 v, KernelGlobals kg) { return gpu_fwidth(v) * derivative_scale_get(kg); }\n"
        "float3 wr_fwidth(float3 v, KernelGlobals kg) { return gpu_fwidth(v) * derivative_scale_get(kg); }\n"
        "float4 wr_fwidth(float4 v, KernelGlobals kg) { return gpu_fwidth(v) * derivative_scale_get(kg); }\n"
        "#else\n"
        "float wr_ddx(float v, KernelGlobals kg) { return v * 0.0; }\n"
        "float2 wr_ddx(float2 v, KernelGlobals kg) { return v * 0.0; }\n"
        "float3 wr_ddx(float3 v, KernelGlobals kg) { return v * 0.0; }\n"
        "float4 wr_ddx(float4 v, KernelGlobals kg) { return v * 0.0; }\n"
        "float wr_ddy(float v, KernelGlobals kg) { return v * 0.0; }\n"
        "float2 wr_ddy(float2 v, KernelGlobals kg) { return v * 0.0; }\n"
        "float3 wr_ddy(float3 v, KernelGlobals kg) { return v * 0.0; }\n"
        "float4 wr_ddy(float4 v, KernelGlobals kg) { return v * 0.0; }\n"
        "float wr_fwidth(float v, KernelGlobals kg) { return v * 0.0; }\n"
        "float2 wr_fwidth(float2 v, KernelGlobals kg) { return v * 0.0; }\n"
        "float3 wr_fwidth(float3 v, KernelGlobals kg) { return v * 0.0; }\n"
        "float4 wr_fwidth(float4 v, KernelGlobals kg) { return v * 0.0; }\n"
        "#endif\n";
    prog.gpu_helpers = std::string("#undef mix\n#undef lerp\n") + interp + deriv + helpers;
    prog.gpu_src = body;
    prog.gpu_ok = true;
    return;
  }

  std::string src;
  if (!helpers.empty()) {
    src += helpers;
  }
  if (prog.gpu_neighbors) {
    src += "WrangleArr wr_neighbours(int v)\n{\n"
           "  WrangleArr a = wr_arr_new(0);\n"
           "  if (v < 0 || v >= n_elem) { return a; }\n"
           "  int s = nbr_off[v];\n"
           "  int e = nbr_off[v + 1];\n"
           "  int n = e - s;\n"
           "  if (n < 0) { n = 0; }\n"
           "  if (n > 120) { n = 120; }\n"
           "  a.count = n;\n"
           "  a.total = n;\n"
           "  a.kind = 0;\n"
           "  for (int i = 0; i < n; i++) { a.data[i] = nbr_idx[s + i]; }\n"
           "  return a;\n"
           "}\n";
  }
  src += "vec4 wr_qmul(vec4 a, vec4 b)\n{\n"
         "  return vec4(a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,\n"
         "              a.w*b.y + a.y*b.w + a.z*b.x - a.x*b.z,\n"
         "              a.w*b.z + a.z*b.w + a.x*b.y - a.y*b.x,\n"
         "              a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z);\n"
         "}\n"
         "vec3 wr_qrot(vec4 q, vec3 v)\n{\n"
         "  vec3 t = 2.0 * cross(q.xyz, v);\n"
         "  return v + q.w * t + cross(q.xyz, t);\n"
         "}\n";
  src += "void main()\n{\n";
  src += "  uint tid = gl_GlobalInvocationID.x + gl_GlobalInvocationID.y * "
         "(gl_NumWorkGroups.x * gl_WorkGroupSize.x);\n";
  src += "  int elem_i = int(tid);\n";
  src += "  if (elem_i < 0 || elem_i >= n_elem) { return; }\n";
  src += "  if (use_sel != 0 && sel[elem_i] == 0) {\n";
  for (const int i : prog.attrs.index_range()) {
    if (!prog.attrs[i].write) {
      continue;
    }
    const std::string in = "a" + std::to_string(i) + "_in";
    const std::string out = "a" + std::to_string(i) + "_out";
    if (prog.attrs[i].type == Type::Vector) {
      src += "    " + out + "[elem_i*3] = " + in + "[elem_i*3];\n";
      src += "    " + out + "[elem_i*3+1] = " + in + "[elem_i*3+1];\n";
      src += "    " + out + "[elem_i*3+2] = " + in + "[elem_i*3+2];\n";
    }
    else if (ELEM(prog.attrs[i].type, Type::Color, Type::Rotation)) {
      src += "    " + out + "[elem_i*4] = " + in + "[elem_i*4];\n";
      src += "    " + out + "[elem_i*4+1] = " + in + "[elem_i*4+1];\n";
      src += "    " + out + "[elem_i*4+2] = " + in + "[elem_i*4+2];\n";
      src += "    " + out + "[elem_i*4+3] = " + in + "[elem_i*4+3];\n";
    }
    else {
      src += "    " + out + "[elem_i] = " + in + "[elem_i];\n";
    }
  }
  src += "    return;\n";
  src += "  }\n";
  src += "  int rand_seq = 0;\n";
  for (const int ci : prog.gpu_ch.index_range()) {
    const Program::GpuCh &ch = prog.gpu_ch[ci];
    src += "  ";
    src += gl_type(ch.type);
    src += " ";
    src += glsl_ident(ch.name, "ch_");
    src += " = ";
    if (ch.type == Type::Vector) {
      src += "vec3(ch" + std::to_string(ci) + "[elem_i*3], ch" + std::to_string(ci) +
             "[elem_i*3+1], ch" + std::to_string(ci) + "[elem_i*3+2])";
    }
    else if (ch.type == Type::Bool) {
      src += "bool(ch" + std::to_string(ci) + "[elem_i])";
    }
    else if (ch.type == Type::Int) {
      src += "ch" + std::to_string(ci) + "[elem_i]";
    }
    else {
      src += "ch" + std::to_string(ci) + "[elem_i]";
    }
    src += ";\n";
  }
  for (const int i : prog.attrs.index_range()) {
    src += "  ";
    src += gl_type(prog.attrs[i].type);
    src += " ";
    src += attr_local(i);
    src += " = ";
    src += load_field(i, "elem_i");
    src += ";\n";
  }
  src += body;
  for (const int i : prog.attrs.index_range()) {
    if (!prog.attrs[i].write) {
      continue;
    }
    src += "  ";
    src += store_field(i, "elem_i", attr_local(i));
    src += ";\n";
  }
  src += "}\n";
  prog.gpu_src = src;
  prog.gpu_ok = true;
}

static bool expr_uses_ident(const Expr *e, const StringRef name)
{
  if (!e) {
    return false;
  }
  if (e->kind == ExprKind::Ident && e->name == name) {
    return true;
  }
  if (expr_uses_ident(e->a, name) || expr_uses_ident(e->b, name) || expr_uses_ident(e->c, name)) {
    return true;
  }
  for (const Expr *a : e->args) {
    if (expr_uses_ident(a, name)) {
      return true;
    }
  }
  return false;
}

static bool stmt_uses_ident(const Stmt *s, const StringRef name)
{
  if (!s) {
    return false;
  }
  if (s->kind == StmtKind::VarDecl && s->var_name == name) {
    return expr_uses_ident(s->expr, name);
  }
  if (expr_uses_ident(s->expr, name)) {
    return true;
  }
  if (stmt_uses_ident(s->s0, name) || stmt_uses_ident(s->s1, name)) {
    return true;
  }
  for (const Stmt *b : s->body) {
    if (stmt_uses_ident(b, name)) {
      return true;
    }
  }
  return false;
}

static bool is_sample_call(const StringRef name)
{
  return ELEM(name,
              "point",
              "edge",
              "face",
              "prim",
              "corner",
              "vertex",
              "curve",
              "instance",
              "instances");
}

static int expr_nearestpoints_mode(const Expr *e)
{
  if (!e) {
    return 0;
  }
  auto mode_of = [](const Expr *a) -> int {
    if (!a) {
      return -1;
    }
    if (a->kind == ExprKind::LitString) {
      return nearestpoints_mode_from_name(a->name);
    }
    if (a->kind == ExprKind::LitInt) {
      if (a->i == 2) {
        return 2;
      }
      return a->i != 0 ? 1 : 0;
    }
    return -1;
  };
  const int n = int(e->args.size());
  if (n >= 4) {
    if (e->args[3] && e->args[3]->kind == ExprKind::LitString) {
      return nearestpoints_mode_from_name(e->args[3]->name);
    }
    if (e->args[2] && e->args[2]->kind == ExprKind::LitString) {
      return nearestpoints_mode_from_name(e->args[2]->name);
    }
    if (e->args[3] && e->args[3]->kind == ExprKind::LitInt && e->args[3]->i == 2) {
      return 2;
    }
    if (e->args[2] && e->args[2]->kind == ExprKind::LitInt && e->args[2]->i == 2) {
      return 2;
    }
    const int m2 = mode_of(e->args[2]);
    return m2 < 0 ? 0 : m2;
  }
  if (n >= 3) {
    const int m = mode_of(e->args[2]);
    return m < 0 ? 0 : m;
  }
  if (n >= 2 && e->args[1] && e->args[1]->kind == ExprKind::LitFloat) {
    return 1;
  }
  return 0;
}

static bool expr_nearestpoints_k(const Expr *e, int &r_k, int &r_geo)
{
  if (!e) {
    return false;
  }
  if (e->kind == ExprKind::Call && (e->name == "nearestpoints" || e->name == "nearpoints")) {
    if (e->args.size() >= 1 && e->args[0] && e->args[0]->kind == ExprKind::LitInt) {
      r_geo = e->args[0]->i;
    }
    if (e->args.size() >= 2 && e->args[1] && e->args[1]->kind == ExprKind::LitInt) {
      r_k = e->args[1]->i;
    }
    return expr_nearestpoints_mode(e) == 0 && r_k > 0;
  }
  if (expr_nearestpoints_k(e->a, r_k, r_geo) || expr_nearestpoints_k(e->b, r_k, r_geo) ||
      expr_nearestpoints_k(e->c, r_k, r_geo))
  {
    return true;
  }
  for (const Expr *a : e->args) {
    if (expr_nearestpoints_k(a, r_k, r_geo)) {
      return true;
    }
  }
  return false;
}

static bool stmt_nearestpoints_k(const Stmt *s, int &r_k, int &r_geo)
{
  if (!s) {
    return false;
  }
  if (expr_nearestpoints_k(s->expr, r_k, r_geo)) {
    return true;
  }
  if (stmt_nearestpoints_k(s->s0, r_k, r_geo) || stmt_nearestpoints_k(s->s1, r_k, r_geo)) {
    return true;
  }
  for (const Stmt *b : s->body) {
    if (stmt_nearestpoints_k(b, r_k, r_geo)) {
      return true;
    }
  }
  return false;
}

static bool expr_nearestpoints_radius(const Expr *e)
{
  if (!e) {
    return false;
  }
  if (e->kind == ExprKind::Call && (e->name == "nearestpoints" || e->name == "nearpoints")) {
    const int mode = expr_nearestpoints_mode(e);
    return mode == 1 || mode == 2;
  }
  if (expr_nearestpoints_radius(e->a) || expr_nearestpoints_radius(e->b) ||
      expr_nearestpoints_radius(e->c))
  {
    return true;
  }
  for (const Expr *a : e->args) {
    if (expr_nearestpoints_radius(a)) {
      return true;
    }
  }
  return false;
}

static bool stmt_nearestpoints_radius(const Stmt *s)
{
  if (!s) {
    return false;
  }
  if (expr_nearestpoints_radius(s->expr)) {
    return true;
  }
  if (stmt_nearestpoints_radius(s->s0) || stmt_nearestpoints_radius(s->s1)) {
    return true;
  }
  for (const Stmt *b : s->body) {
    if (stmt_nearestpoints_radius(b)) {
      return true;
    }
  }
  return false;
}

static bool stmt_has_kind(const Stmt *s, const StmtKind kind)
{
  if (!s) {
    return false;
  }
  if (s->kind == kind) {
    return true;
  }
  if (stmt_has_kind(s->s0, kind) || stmt_has_kind(s->s1, kind)) {
    return true;
  }
  for (const Stmt *b : s->body) {
    if (stmt_has_kind(b, kind)) {
      return true;
    }
  }
  return false;
}

static bool sampled_is_only_position(const Vector<std::string> &sampled)
{
  for (const std::string &nm : sampled) {
    if (nm != "position" && nm != "P") {
      return false;
    }
  }
  return true;
}

static void expr_sample_write(const Expr *e, Vector<std::string> &sampled, Vector<std::string> &written)
{
  if (!e) {
    return;
  }
  if (e->kind == ExprKind::Call && is_sample_call(e->name)) {
    std::string nm = "position";
    const int n = int(e->args.size());
    int name_i = -1;
    if (n >= 3) {
      name_i = 1;
    }
    else if (n == 2 && e->args[0] &&
             ELEM(e->args[0]->kind, ExprKind::LitString, ExprKind::Ident, ExprKind::Attr))
    {
      name_i = 0;
    }
    else if (n == 1 && e->args[0] &&
             ELEM(e->args[0]->kind, ExprKind::LitString, ExprKind::Ident, ExprKind::Attr))
    {
      name_i = 0;
    }
    else if (n == 2) {
      name_i = 1;
    }
    if (name_i >= 0 && e->args[name_i] &&
        ELEM(e->args[name_i]->kind, ExprKind::LitString, ExprKind::Ident, ExprKind::Attr))
    {
      nm = canonical_attr_name(e->args[name_i]->name);
    }
    sampled.append(nm);
  }
  if (e->kind == ExprKind::Assign) {
    const Expr *lv = e->a;
    if (lv && lv->kind == ExprKind::Member) {
      lv = lv->a;
    }
    if (lv && lv->kind == ExprKind::Attr) {
      written.append(canonical_attr_name(lv->name));
    }
  }
  expr_sample_write(e->a, sampled, written);
  expr_sample_write(e->b, sampled, written);
  expr_sample_write(e->c, sampled, written);
  for (const Expr *a : e->args) {
    expr_sample_write(a, sampled, written);
  }
}

static void stmt_sample_write(const Stmt *s, Vector<std::string> &sampled, Vector<std::string> &written)
{
  if (!s) {
    return;
  }
  expr_sample_write(s->expr, sampled, written);
  stmt_sample_write(s->s0, sampled, written);
  stmt_sample_write(s->s1, sampled, written);
  for (const Stmt *b : s->body) {
    stmt_sample_write(b, sampled, written);
  }
}

static bool names_overlap(const Vector<std::string> &a, const Vector<std::string> &b)
{
  for (const std::string &x : a) {
    for (const std::string &y : b) {
      if (x == y) {
        return true;
      }
    }
  }
  return false;
}

static void expr_find_chf(const Expr *e, std::string &r_name)
{
  if (!e || !r_name.empty()) {
    return;
  }
  if (e->kind == ExprKind::Call && e->name == "chf" && !e->args.is_empty() && e->args[0] &&
      e->args[0]->kind == ExprKind::LitString)
  {
    r_name = e->args[0]->name;
    return;
  }
  expr_find_chf(e->a, r_name);
  expr_find_chf(e->b, r_name);
  expr_find_chf(e->c, r_name);
  for (const Expr *a : e->args) {
    expr_find_chf(a, r_name);
  }
}

static void stmt_find_chf(const Stmt *s, std::string &r_name)
{
  if (!s || !r_name.empty()) {
    return;
  }
  expr_find_chf(s->expr, r_name);
  stmt_find_chf(s->s0, r_name);
  stmt_find_chf(s->s1, r_name);
  for (const Stmt *b : s->body) {
    stmt_find_chf(b, r_name);
  }
}

static bool chi_name_of(const Expr *e, std::string &r_name)
{
  if (!e || e->kind != ExprKind::Call) {
    return false;
  }
  if (e->name != "chi" && e->name != "chf") {
    return false;
  }
  if (e->args.is_empty() || !e->args[0] || e->args[0]->kind != ExprKind::LitString) {
    return false;
  }
  r_name = e->args[0]->name;
  return true;
}

static bool counted_for_bound(const Stmt *s, std::string &r_ivar, std::string &r_ch, int &r_const)
{
  if (!s || s->kind != StmtKind::For || !s->s0 || s->s0->kind != StmtKind::VarDecl) {
    return false;
  }
  r_ivar = s->s0->var_name;
  if (!s->expr || s->expr->kind != ExprKind::Binary || s->expr->op != "<") {
    return false;
  }
  if (!s->expr->a || s->expr->a->kind != ExprKind::Ident || s->expr->a->name != r_ivar) {
    return false;
  }
  r_ch.clear();
  r_const = -1;
  if (s->expr->b && s->expr->b->kind == ExprKind::LitInt) {
    r_const = s->expr->b->i;
    return true;
  }
  return s->expr->b && chi_name_of(s->expr->b, r_ch);
}

/**
 * Return the exact trip count for the common small-loop form
 * `for (int i = C; i < K; i++)`. These loops are emitted as straight-line
 * bytecode so the runtime can use its jump-free tiled executor. Keep the
 * limit deliberately small to avoid bytecode growth; dynamic loops and loops
 * with control-flow exits retain the normal VM representation.
 */
static bool small_counted_for_trip_count(const Stmt *s, int &r_trip_count)
{
  r_trip_count = 0;
  if (!s || s->kind != StmtKind::For || !s->s0 || s->s0->kind != StmtKind::VarDecl ||
      !s->s0->expr || s->s0->expr->kind != ExprKind::LitInt || !s->expr ||
      s->expr->kind != ExprKind::Binary || s->expr->op != "<" || !s->expr->a ||
      s->expr->a->kind != ExprKind::Ident || s->expr->a->name != s->s0->var_name ||
      !s->expr->b || s->expr->b->kind != ExprKind::LitInt || !s->s1 ||
      s->s1->kind != StmtKind::Expr || !s->s1->expr ||
      s->s1->expr->kind != ExprKind::Postfix || s->s1->expr->op != "++" ||
      !s->s1->expr->a || s->s1->expr->a->kind != ExprKind::Ident ||
      s->s1->expr->a->name != s->s0->var_name)
  {
    return false;
  }
  const int64_t start = s->s0->expr->i;
  const int64_t end = s->expr->b->i;
  const int64_t trips = std::max<int64_t>(end - start, 0);
  constexpr int64_t max_unrolled_trips = 16;
  if (trips > max_unrolled_trips) {
    return false;
  }
  const Stmt *body = s->body.is_empty() ? nullptr : s->body[0];
  if (stmt_has_kind(body, StmtKind::Break) || stmt_has_kind(body, StmtKind::Continue)) {
    return false;
  }
  r_trip_count = int(trips);
  return true;
}

/* Peel a top-level counted for-loop that samples point() of attributes it writes.
 * Host/GPU then repeats the program as Jacobi passes on the working arrays. */
static void hoist_array_passes(Vector<Stmt *> &stmts, Program &prog)
{
  for (int i = 0; i < int(stmts.size()); i++) {
    Stmt *s = stmts[i];
    std::string ivar;
    std::string ch;
    int bound = -1;
    if (!counted_for_bound(s, ivar, ch, bound)) {
      continue;
    }
    Vector<std::string> sampled;
    Vector<std::string> written;
    stmt_sample_write(s, sampled, written);
    if (!names_overlap(sampled, written)) {
      continue;
    }
    bool uses_i = false;
    for (const Stmt *b : s->body) {
      if (stmt_uses_ident(b, ivar)) {
        uses_i = true;
        break;
      }
    }
    if (uses_i) {
      continue;
    }
    /* Neighbor-average Jacobi is only valid for `sum += point(P)` with no extra
     * logic. Radius search, `if`, or sampling other attrs (radius, N, …) must
     * stay on the VM — otherwise packing/push-apart loops produce no output. */
    const bool simple_jacobi = !stmt_nearestpoints_radius(s) && !stmt_has_kind(s, StmtKind::If) &&
                               sampled_is_only_position(sampled);
    if (simple_jacobi) {
      prog.jacobi_smooth = true;
      int knn_k = 0;
      int knn_geo = 0;
      if (stmt_nearestpoints_k(s, knn_k, knn_geo)) {
        prog.jacobi_knn = true;
        prog.jacobi_knn_k = std::max(knn_k, 1);
        prog.jacobi_knn_geo = knn_geo;
      }
    }
    /* kNN Jacobi keeps the original loop so a failed native path still writes
     * i[]@pts as an int array. Everything else peels to host passes. */
    if (!prog.jacobi_knn) {
      if (!s->body.is_empty() && s->body[0]) {
        stmts[i] = s->body[0];
      }
      else {
        stmts[i] = s->s0;
      }
      prog.array_pass_peeled = true;
    }
    for (const std::string &nm : sampled) {
      bool in_written = false;
      for (const std::string &w : written) {
        if (w == nm) {
          in_written = true;
          break;
        }
      }
      if (!in_written) {
        continue;
      }
      bool have = false;
      for (const std::string &x : prog.jacobi_sample_attrs) {
        if (x == nm) {
          have = true;
          break;
        }
      }
      if (!have) {
        prog.jacobi_sample_attrs.append(nm);
      }
    }
    for (const std::string &w : written) {
      bool sampled_w = false;
      for (const std::string &nm : sampled) {
        if (nm == w) {
          sampled_w = true;
          break;
        }
      }
      if (sampled_w) {
        continue;
      }
      bool have = false;
      for (const std::string &x : prog.jacobi_avg_attrs) {
        if (x == w) {
          have = true;
          break;
        }
      }
      if (!have) {
        prog.jacobi_avg_attrs.append(w);
      }
    }
    stmt_find_chf(s, prog.jacobi_fac_ch);
    if (!ch.empty()) {
      prog.array_passes_ch = ch;
      prog.array_passes = 1;
    }
    else {
      prog.array_passes = std::max(bound, 0);
    }
    return;
  }
}

static CompileOutput compile_impl(const StringRef source, const bool shader_material);

CompileOutput compile(const StringRef source)
{
  return compile_impl(source, false);
}

CompileOutput compile_shader_material(const StringRef source)
{
  return compile_impl(source, true);
}

static CompileOutput compile_impl(const StringRef source, const bool shader_material)
{
  CompileOutput out;
  const uint64_t key = get_default_hash(source) ^ (shader_material ? 0x9e3779b97f4a7c15ull : 0);
  struct CacheEntry {
    std::string src;
    bool shader_material = false;
    std::shared_ptr<Program> program;
    std::string error;
    std::string warning;
    Vector<Diagnostic> diags;
  };
  static std::mutex mutex;
  static Map<uint64_t, CacheEntry> cache;
  {
    std::lock_guard lock(mutex);
    if (CacheEntry *e = cache.lookup_ptr(key)) {
      if (e->src == source && e->shader_material == shader_material) {
        out.program = e->program;
        out.error = e->error;
        out.warning = e->warning;
        out.diags = e->diags;
        return out;
      }
    }
  }
  auto store_cache = [&]() {
    std::lock_guard lock(mutex);
    if (cache.size() > 256) {
      cache.clear();
    }
    CacheEntry e;
    e.src = std::string(source);
    e.shader_material = shader_material;
    e.program = out.program;
    e.error = out.error;
    e.warning = out.warning;
    e.diags = out.diags;
    cache.add_overwrite(key, std::move(e));
  };

  Parser parser(source);
  if (!parser.ok()) {
    out.error = parser.error;
    out.diags = parser.diags;
    store_cache();
    return out;
  }
  Vector<Stmt *> stmts = parser.parse_program();
  if (!parser.ok()) {
    out.error = parser.error;
    out.diags = parser.diags;
    store_cache();
    return out;
  }

  Compiler c;
  c.parser = &parser;
  c.shader_material = shader_material;
  hoist_array_passes(stmts, c.prog);

  struct LoopFrame {
    Vector<int> breaks;
    Vector<int> continues;
    int continue_pc = 0;
  };
  Vector<LoopFrame> f2;

  std::function<void(Stmt *)> cs;
  cs = [&](Stmt *s) {
    if (!s || !c.error.empty()) {
      return;
    }
    const Stmt *previous_error_stmt = c.error_stmt;
    c.error_stmt = s;
    struct RestoreErrorStmt {
      const Stmt *&slot;
      const Stmt *previous;
      ~RestoreErrorStmt()
      {
        slot = previous;
      }
    } restore_error_stmt{c.error_stmt, previous_error_stmt};
    switch (s->kind) {
      case StmtKind::Empty:
        return;
      case StmtKind::Expr:
        if (s->expr) {
          const bool leftover = s->expr->kind != ExprKind::Assign &&
                                s->expr->kind != ExprKind::Postfix;
          c.compile_expr(s->expr, true);
          if (leftover) {
            c.emit(Op::Pop);
          }
        }
        return;
      case StmtKind::Block:
        c.push_scope();
        for (Stmt *b : s->body) {
          cs(b);
        }
        c.pop_scope();
        return;
      case StmtKind::VarDecl:
        c.compile_stmt(s);
        return;
      case StmtKind::If: {
        c.compile_expr(s->expr);
        const int jmp_else = c.emit_jmp(Op::JmpIfFalse);
        cs(s->s0);
        if (s->s1) {
          const int jmp_end = c.emit_jmp(Op::Jmp);
          c.patch(jmp_else, int(c.prog.code.size()));
          cs(s->s1);
          c.patch(jmp_end, int(c.prog.code.size()));
        }
        else {
          c.patch(jmp_else, int(c.prog.code.size()));
        }
        return;
      }
      case StmtKind::While: {
        if (c.try_emit_while_cmp_add(s)) {
          return;
        }
        LoopFrame fr;
        fr.continue_pc = int(c.prog.code.size());
        f2.append(fr);
        c.compile_expr(s->expr);
        const int jmp_end = c.emit_jmp(Op::JmpIfFalse);
        cs(s->s0);
        c.emit(Op::Jmp, f2.last().continue_pc);
        const int end_pc = int(c.prog.code.size());
        c.patch(jmp_end, end_pc);
        for (const int br : f2.last().breaks) {
          c.patch(br, end_pc);
        }
        for (const int co : f2.last().continues) {
          c.patch(co, f2.last().continue_pc);
        }
        f2.pop_last();
        return;
      }
      case StmtKind::For: {
        int unrolled_trips = 0;
        if (small_counted_for_trip_count(s, unrolled_trips)) {
          c.push_scope();
          cs(s->s0);
          for (int i = 0; i < unrolled_trips; i++) {
            cs(s->body.is_empty() ? nullptr : s->body[0]);
            cs(s->s1);
          }
          c.pop_scope();
          return;
        }
        c.push_scope();
        if (s->s0) {
          cs(s->s0);
        }
        if (c.try_emit_gather_samples(s)) {
          c.pop_scope();
          return;
        }
        const int cond_pc = int(c.prog.code.size());
        if (s->expr) {
          c.compile_expr(s->expr);
        }
        else {
          c.emit(Op::PushB, 1);
        }
        const int jmp_end = c.emit_jmp(Op::JmpIfFalse);
        LoopFrame fr;
        f2.append(fr);
        cs(s->body.is_empty() ? nullptr : s->body[0]);
        const int incr_pc = int(c.prog.code.size());
        if (s->s1) {
          cs(s->s1);
        }
        c.emit(Op::Jmp, cond_pc);
        const int end_pc = int(c.prog.code.size());
        c.patch(jmp_end, end_pc);
        for (const int br : f2.last().breaks) {
          c.patch(br, end_pc);
        }
        for (const int co : f2.last().continues) {
          c.patch(co, incr_pc);
        }
        f2.pop_last();
        c.pop_scope();
        return;
      }
      case StmtKind::Break:
        if (f2.is_empty()) {
          c.fail("break outside loop");
          return;
        }
        f2.last().breaks.append(c.emit_jmp(Op::Jmp));
        return;
      case StmtKind::Continue:
        if (f2.is_empty()) {
          c.fail("continue outside loop");
          return;
        }
        f2.last().continues.append(c.emit_jmp(Op::Jmp));
        return;
      case StmtKind::Return:
        c.compile_stmt(s);
        return;
    }
  };

  if (c.parser) {
    for (Stmt *fn : c.parser->fns) {
      if (!fn) {
        continue;
      }
      UserFn u;
      u.name = fn->var_name;
      u.ret = fn->var_type;
      u.param_types = fn->param_types;
      c.prog.user_fns.append(std::move(u));
    }
  }

  for (Stmt *s : stmts) {
    cs(s);
  }
  /* Don't fall through into user-function bodies after the main script. */
  c.emit(Op::Return);

  if (c.parser) {
    /* Main script slots are independent of user-function frames. Reset the
     * allocator so parameters occupy 0..n-1 and body locals continue from n.
     * `prog.local_count` is a high-water mark: Block pop_scope drops names, not slots. */
    const int main_local_count = c.prog.local_count;
    for (int fi = 0; fi < int(c.parser->fns.size()); fi++) {
      Stmt *fn = c.parser->fns[fi];
      if (!fn || fi >= int(c.prog.user_fns.size())) {
        continue;
      }
      c.locals.clear();
      c.scope_stack.clear();
      c.attr_local.clear();
      c.loop_break.clear();
      c.loop_continue.clear();
      c.prog.user_fns[fi].entry = int(c.prog.code.size());
      c.prog.local_count = 0;
      for (int pi = 0; pi < int(fn->param_names.size()); pi++) {
        Local loc;
        loc.name = fn->param_names[pi];
        loc.type = fn->param_types[pi];
        loc.slot = c.prog.local_count++;
        c.locals.append(loc);
      }
      if (fn->s0) {
        cs(fn->s0);
      }
      c.emit(Op::PushI, c.add_const_i(0));
      c.emit(Op::Return);
      c.prog.user_fns[fi].local_count = std::max(c.prog.local_count, int(fn->param_names.size()));
    }
    c.prog.local_count = main_local_count;
  }

  if (!c.error.empty()) {
    out.error = c.error;
    out.warning = c.warning;
    out.diags = c.diags;
    store_cache();
    return out;
  }

  out.warning = c.warning;
  out.diags = c.diags;
  out.program = std::make_shared<Program>(std::move(c.prog));
  optimize_program(*out.program);
  gpu_emit_program(stmts, parser.fns, *out.program, shader_material, key);
  store_cache();
  return out;
}

}  // namespace blender::nodes::vex
