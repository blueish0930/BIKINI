/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_vex.hh"

namespace blender::nodes::vex {

StringRef function_param_types(const StringRef name)
{
  static const struct {
    const char *name;
    const char *types;
  } table[] = {
      {"sin", "num"},
      {"cos", "num"},
      {"tan", "num"},
      {"asin", "num"},
      {"acos", "num"},
      {"atan", "num"},
      {"atan2", "float,float"},
      {"abs", "num"},
      {"floor", "num"},
      {"ceil", "num"},
      {"round", "num"},
      {"trunc", "num"},
      {"sqrt", "num"},
      {"exp", "num"},
      {"log", "num"},
      {"pow", "float,float"},
      {"min", "num,num"},
      {"max", "num,num"},
      {"clamp", "num,num,num"},
      {"radians", "float"},
      {"degrees", "float"},
      {"sign", "num"},
      {"fract", "num"},
      {"ddx", "num"},
      {"ddy", "num"},
      {"fwidth", "num"},
      {"dFdx", "num"},
      {"dFdy", "num"},
      {"mix", "num,num,float"},
      {"lerp", "num,num,float"},
      {"map", "num,num,num,num,num"},
      {"smooth", "num"},
      {"noise", "vec,float,float,float,float,float,enum,enum"},
      {"hash", "num"},
      {"rand", "int"},
      {"float", "num"},
      {"int", "num"},
      {"bool", "num"},
      {"length", "vec"},
      {"distance", "vec,vec"},
      {"dot", "vec,vec"},
      {"cross", "vec,vec"},
      {"normalize", "vec"},
      {"vec3", "num,num,num,num"},
      {"vector", "num,num,num,num"},
      {"set", "num,num,num,num"},
      {"npoints", ""},
      {"nedges", ""},
      {"nfaces", ""},
      {"ncorners", ""},
      {"chf", "str"},
      {"ch", "str"},
      {"chi", "str"},
      {"chv", "str"},
      {"chb", "str"},
      {"chc", "str"},
      {"chm", "str"},
      {"chq", "str"},
      {"chs", "str"},
      {"cht", "str"},
      {"valuetostring", "num,int,int"},
      {"value_to_string", "num,int,int"},
      {"format", "str"},
      {"sprintf", "str"},
      {"len", "arr"},
      {"append", "arr,num"},
      {"insert", "arr,int,num"},
      {"removeindex", "arr,int"},
      {"removevalue", "arr,num"},
      {"sort", "arr"},
      {"invert", "mat"},
      {"invert_matrix", "mat"},
      {"transpose", "mat"},
      {"determinant", "mat"},
      {"det", "mat"},
      {"transform_point", "vec,mat"},
      {"svd", "mat,out,out,out"},
      {"svddecomp", "mat,out,out,out"},
      {"pd", "mat,out,out"},
      {"polardecomp", "mat,out,out"},
      {"polar_decomp", "mat,out,out"},
      {"eigen", "mat,out,out"},
      {"eigendecomp", "mat,out,out"},
      {"eigen_decomp", "mat,out,out"},
      {"transform_direction", "vec,mat"},
      {"project_point", "vec,mat"},
      {"translation", "mat"},
      {"rotation", "mat"},
      {"scale", "mat"},
      {"pointneighbours", "geo,int"},
      {"point_neighbours", "geo,int"},
      {"pointneighbors", "geo,int"},
      {"point_neighbors", "geo,int"},
      {"neighbours", "geo,int"},
      {"neighbors", "geo,int"},
      {"pointedges", "geo,int"},
      {"point_edges", "geo,int"},
      {"pointfaces", "geo,int"},
      {"point_faces", "geo,int"},
      {"pointcorners", "geo,int"},
      {"point_corners", "geo,int"},
      {"edgepoints", "geo,int"},
      {"edge_points", "geo,int"},
      {"edgefaces", "geo,int"},
      {"edge_faces", "geo,int"},
      {"edgecorners", "geo,int"},
      {"edge_corners", "geo,int"},
      {"corners_of_edge", "geo,int"},
      {"facepoints", "geo,int"},
      {"face_points", "geo,int"},
      {"faceedges", "geo,int"},
      {"face_edges", "geo,int"},
      {"faceneighbours", "geo,int"},
      {"face_neighbours", "geo,int"},
      {"facecorners", "geo,int"},
      {"face_corners", "geo,int"},
      {"cornerpoint", "geo,int"},
      {"corner_point", "geo,int"},
      {"cornervert", "geo,int"},
      {"corneredges", "geo,int"},
      {"corner_edges", "geo,int"},
      {"offsetcorner", "geo,int,int"},
      {"offset_corner", "geo,int,int"},
      {"offset_corner_in_face", "geo,int,int"},
      {"offsetcornerinface", "geo,int,int"},
      {"cornerface", "geo,int,out int,out int"},
      {"corner_face", "geo,int,out int,out int"},
      {"pointcurve", "geo,int,out int,out int"},
      {"point_curve", "geo,int,out int,out int"},
      {"curvepoints", "geo,int"},
      {"curve_points", "geo,int"},
      {"points_of_curve", "geo,int"},
      {"curve_of_point", "geo,int"},
      {"curveofpoint", "geo,int"},
      {"nearestpoints", "geo,num,enum"},
      {"nearpoints", "geo,num,enum"},
      {"point", "geo,str,int"},
      {"edge", "geo,str,int"},
      {"face", "geo,str,int"},
      {"corner", "geo,str,int"},
      {"curve", "geo,str,int"},
      {"instance", "geo,str,int"},
      {"addpoint", "geo,num"},
      {"addprim", "geo,enum,arr,bool"},
      {"add_prim", "geo,enum,arr,bool"},
      {"setattribute", "geo,enum,int,str,num,enum"},
      {"set_attribute", "geo,enum,int,str,num,enum"},
      {"setattr", "geo,enum,int,str,num,enum"},
      {"raycast", "geo,vec,vec,float,out int,out vec,out vec,out float"},
      {"raycastall", "geo,vec,vec,float,out arr,out arr,out arr,out arr"},
      {"raycast_all", "geo,vec,vec,float,out arr,out arr,out arr,out arr"},
      {"rayishit", "ray"},
      {"rayhitpos", "ray"},
      {"rayhitn", "ray"},
      {"rayhitdist", "ray"},
      {"bboxmin", "geo"},
      {"bboxmax", "geo"},
      {"boundingbox", "out vec,out vec"},
      {"bounding_box", "out vec,out vec"},
      {"geometry_proximity", "geo,enum,vec,out vec,out float"},
      {"proximity", "geo,enum,vec,out vec,out float"},
      {"sample_nearest_surface", "geo,str,vec,out"},
      {"delete_geometry", "enum,int,enum"},
      {"deletegeometry", "enum,int,enum"},
      {"deletegeo", "enum,int,enum"},
      {"ident", ""},
      {"identity", ""},
      {"combine_transform", "vec,quat,vec"},
      {"invert_rotation", "quat"},
      {"rotate_rotation", "quat,quat"},
      {"rotate", "quat,quat"},
      {"quaternion", "float,float,float,float"},
      {"rotation", "float,float,float,float"},
      {"vector2", "num,num"},
      {"vector4", "num,num,num,num"},
      {"matrix2", "num"},
      {"matrix3", "num"},
      {"accumulate", "num"},
      {"accumulate_field", "num"},
      {"field_average", "str"},
      {"fieldaverage", "str"},
      {"field_min", "str"},
      {"field_max", "str"},
      {"fieldminmax", "str"},
      {"field_minmax", "str"},
      {"edges_of_corner", "geo,int"},
      {"face_of_corner", "geo,int"},
      {"vertex_of_corner", "geo,int"},
      {"voronoi", "vec,float,float,float,float,float,float,enum,enum,enum,float"},
      {"voronoinoise", "vec,float,float,float,float,float,float,enum,enum,enum,float"},
      {nullptr, nullptr},
  };
  for (int i = 0; table[i].name; i++) {
    if (name == table[i].name) {
      return table[i].types;
    }
  }
  return {};
}

void gather_completions(Vector<std::string> &r_owned_names,
                        Vector<expression::CompletionItem> &r_items,
                        const bool shader_material)
{
  r_owned_names.clear();
  r_items.clear();

  using C = expression::CompletionCategory;
  using K = expression::CompletionKind;

  auto add = [&](const char *name,
                 const char *category_label,
                 const C category,
                 const K kind,
                 const char *insert,
                 const char *usage,
                 const bool parens) {
    r_items.append(expression::CompletionItem{
        name, category_label, category, kind, insert, usage, parens});
  };

  add("if", "Keyword", C::Variable, K::Keyword, "if", "if (cond) { }", false);
  add("else", "Keyword", C::Variable, K::Keyword, "else", "else { }", false);
  add("for", "Keyword", C::Variable, K::Keyword, "for", "for (int i = 0; i < n; i++)", false);
  add("foreach",
      "Keyword",
      C::Variable,
      K::Keyword,
      "foreach (int pt; pts) { }",
      "foreach (int pt; pts) { }",
      false);
  add("while", "Keyword", C::Variable, K::Keyword, "while", "while (cond) { }", false);
  add("break", "Keyword", C::Variable, K::Keyword, "break", "break;", false);
  add("continue", "Keyword", C::Variable, K::Keyword, "continue", "continue;", false);
  add("return", "Keyword", C::Variable, K::Keyword, "return", "return;", false);
  add("int", "Keyword", C::Variable, K::Keyword, "int", "int x = 0;", false);
  add("float", "Keyword", C::Variable, K::Keyword, "float", "float x = 0;", false);
  add("bool", "Keyword", C::Variable, K::Keyword, "bool", "bool x = true;", false);
  add("vector", "Keyword", C::Variable, K::Keyword, "vector", "vector v = {0, 0, 0};", false);
  add("vector2", "Keyword", C::Variable, K::Keyword, "vector2", "vector2 u = {0, 0};", false);
  add("vector4", "Keyword", C::Variable, K::Keyword, "vector4", "vector4 q = {0, 0, 0, 0};", false);
  add("matrix", "Keyword", C::Matrix, K::Keyword, "matrix", "matrix m = ident();", false);
  add("matrix2", "Keyword", C::Matrix, K::Keyword, "matrix2", "matrix2 m = ident();", false);
  add("matrix3", "Keyword", C::Matrix, K::Keyword, "matrix3", "matrix3 m = ident();", false);
  add("matrix",
      "Matrix",
      C::Matrix,
      K::Function,
      "matrix",
      "matrix(16 floats col-major) / 4 vectors",
      true);
  add("string", "Keyword", C::Variable, K::Keyword, "string", "string s = \"name\";", false);
  add("color", "Keyword", C::Color, K::Keyword, "color", "color c = {1, 0, 0};", false);

  add("rotation", "Keyword", C::Rotation, K::Keyword, "rotation", "rotation r = {0, 0, 0, 1};", false);
  add("quaternion", "Keyword", C::Rotation, K::Keyword, "quaternion", "quaternion q = {0, 0, 0, 1};", false);
  add("quat", "Keyword", C::Rotation, K::Keyword, "quat", "quat q = {0, 0, 0, 1};", false);

  add("P", "Attribute", C::Vector, K::Variable, "v@P", "v@P — position", false);
  add("N", "Attribute", C::Vector, K::Variable, "v@N", "v@N — normal", false);
  add("ptnum", "Attribute", C::Variable, K::Variable, "i@index", "i@index — element index", false);
  add("index", "Attribute", C::Variable, K::Variable, "i@index", "i@index — run-over element index", false);
  add("id", "Attribute", C::Variable, K::Variable, "i@id", "i@id — id attribute", false);
  add("f@", "Attribute", C::Float, K::Variable, "f@", "f@name — float attribute", false);
  add("u@", "Attribute", C::Vector, K::Variable, "u@", "u@name — vector2 attribute", false);
  add("v@", "Attribute", C::Vector, K::Variable, "v@", "v@name — vector attribute", false);
  add("q@", "Attribute", C::Vector, K::Variable, "q@", "q@name — vector4 attribute", false);
  add("i@", "Attribute", C::Variable, K::Variable, "i@", "i@name — int attribute", false);
  add("b@", "Attribute", C::Variable, K::Variable, "b@", "b@name — bool attribute", false);
  add("2@", "Attribute", C::Matrix, K::Variable, "2@", "2@name — 2x2 matrix (wrangle-only)", false);
  add("3@", "Attribute", C::Matrix, K::Variable, "3@", "3@name — 3x3 matrix (wrangle-only)", false);
  add("4@", "Attribute", C::Matrix, K::Variable, "4@", "4@name — 4x4 matrix attribute", false);
  add("m@", "Attribute", C::Matrix, K::Variable, "m@", "m@name — 4x4 matrix attribute", false);
  add("s@", "Attribute", C::Variable, K::Variable, "s@", "s@name — string attribute", false);
  add("r@", "Attribute", C::Rotation, K::Variable, "r@", "r@name — rotation attribute", false);
  add("i[]@", "Attribute", C::Variable, K::Variable, "i[]@", "i[]@name — int array attribute", false);
  add("f[]@", "Attribute", C::Float, K::Variable, "f[]@", "f[]@name — float array attribute", false);
  add("v[]@", "Attribute", C::Vector, K::Variable, "v[]@", "v[]@name — vector array attribute", false);
  add("c@", "Attribute", C::Color, K::Variable, "c@", "c@name — color attribute", false);

  add("sin", "Float", C::Float, K::Function, "sin", "sin(x)", true);
  add("cos", "Float", C::Float, K::Function, "cos", "cos(x)", true);
  add("tan", "Float", C::Float, K::Function, "tan", "tan(x)", true);
  add("asin", "Float", C::Float, K::Function, "asin", "asin(x)", true);
  add("acos", "Float", C::Float, K::Function, "acos", "acos(x)", true);
  add("atan", "Float", C::Float, K::Function, "atan", "atan(x)", true);
  add("atan2", "Float", C::Float, K::Function, "atan2", "atan2(y, x)", true);
  add("abs", "Float", C::Float, K::Function, "abs", "abs(x)", true);
  add("floor", "Float", C::Float, K::Function, "floor", "floor(x)", true);
  add("ceil", "Float", C::Float, K::Function, "ceil", "ceil(x)", true);
  add("round", "Float", C::Float, K::Function, "round", "round(x)", true);
  add("trunc", "Float", C::Float, K::Function, "trunc", "trunc(x)", true);
  add("sqrt", "Float", C::Float, K::Function, "sqrt", "sqrt(x)", true);
  add("exp", "Float", C::Float, K::Function, "exp", "exp(x)", true);
  add("log", "Float", C::Float, K::Function, "log", "log(x)", true);
  add("pow", "Float", C::Float, K::Function, "pow", "pow(x, y)", true);
  add("min", "Float", C::Float, K::Function, "min", "min(a, b)", true);
  add("max", "Float", C::Float, K::Function, "max", "max(a, b)", true);
  add("clamp", "Float", C::Float, K::Function, "clamp", "clamp(x, lo, hi)", true);
  add("radians", "Float", C::Float, K::Function, "radians", "radians(deg)", true);
  add("degrees", "Float", C::Float, K::Function, "degrees", "degrees(rad)", true);
  add("sign", "Float", C::Float, K::Function, "sign", "sign(x)", true);
  add("fract", "Float", C::Float, K::Function, "fract", "fract(x)", true);
  add("mix", "Float", C::Float, K::Function, "mix", "mix(a, b, t)", true);
  add("lerp", "Float", C::Float, K::Function, "lerp", "lerp(a, b, t)", true);
  add("map",
      "Float",
      C::Float,
      K::Function,
      "map",
      "map(value, from_min, from_max, to_min, to_max)",
      true);
  add("smooth",
      "Float",
      C::Float,
      K::Function,
      "smooth",
      "smooth(value)  or  smooth(min, max, value)",
      true);
  add("noise",
      "Float",
      C::Float,
      K::Function,
      "noise",
      "noise(P, scale, detail, roughness, lacunarity, distortion, dim, type)  type: fbm|...  dim 1-4",
      true);
  add("voronoi",
      "Float",
      C::Float,
      K::Function,
      "voronoi",
      "voronoi(P, scale, detail, roughness, lacunarity, smoothness, randomness, feature, metric, dim, distortion)",
      true);
  add("voronoinoise",
      "Float",
      C::Float,
      K::Function,
      "voronoinoise",
      "voronoinoise(P, scale, detail, roughness, lacunarity, smoothness, randomness, feature, metric, dim, distortion)",
      true);
  add("hash", "Float", C::Float, K::Function, "hash", "hash(P)", true);
  add("rand", "Float", C::Float, K::Function, "rand", "rand() / rand(seed)", true);
  add("float", "Float", C::Float, K::Function, "float", "float(x)", true);
  add("int", "Float", C::Float, K::Function, "int", "int(x)", true);
  add("bool", "Float", C::Float, K::Function, "bool", "bool(x)", true);

  add("length", "Vector", C::Vector, K::Function, "length", "length(v)", true);
  add("distance", "Vector", C::Vector, K::Function, "distance", "distance(a, b)", true);
  add("dot", "Vector", C::Vector, K::Function, "dot", "dot(a, b)", true);
  add("cross", "Vector", C::Vector, K::Function, "cross", "cross(a, b)  2D→float  3D→vector", true);
  add("normalize", "Vector", C::Vector, K::Function, "normalize", "normalize(v)", true);
  add("vec3", "Vector", C::Vector, K::Function, "vec3", "vec3(x, y, z)", true);
  add("vector", "Vector", C::Vector, K::Function, "vector", "vector(x, y, z)", true);
  add("set", "Vector", C::Vector, K::Function, "set", "set(x, y, z) / set(vector2, z)", true);

  add("x", "Member", C::Member, K::Member, "x", ".x", false);
  add("y", "Member", C::Member, K::Member, "y", ".y", false);
  add("z", "Member", C::Member, K::Member, "z", ".z", false);
  add("w", "Member", C::Member, K::Member, "w", ".w", false);
  add("xy", "Member", C::Member, K::Member, "xy", ".xy", false);
  add("zx", "Member", C::Member, K::Member, "zx", ".zx → vector2", false);
  add("zxy", "Member", C::Member, K::Member, "zxy", ".zxy → vector", false);

  add("npoints", "Geometry", C::Variable, K::Function, "npoints", "npoints()", true);
  add("nedges", "Geometry", C::Variable, K::Function, "nedges", "nedges()", true);
  add("nfaces", "Geometry", C::Variable, K::Function, "nfaces", "nfaces()", true);
  add("ncorners", "Geometry", C::Variable, K::Function, "ncorners", "ncorners()", true);
  add("point", "Geometry", C::Variable, K::Function, "point", "point(geo, \"P\", index)", true);
  add("edge", "Geometry", C::Variable, K::Function, "edge", "edge(geo, \"P\", index)", true);
  add("face", "Geometry", C::Variable, K::Function, "face", "face(geo, \"P\", index)", true);
  add("corner", "Geometry", C::Variable, K::Function, "corner", "corner(geo, \"P\", index)", true);
  add("curve", "Geometry", C::Variable, K::Function, "curve", "curve(geo, \"P\", index)", true);
  add("instance", "Geometry", C::Variable, K::Function, "instance", "instance(geo, \"P\", index)", true);
  add("addpoint", "Geometry", C::Variable, K::Function, "addpoint", "addpoint(geo, pos) / addpoint(geo, idx)", true);
  add("addprim", "Geometry", C::Variable, K::Function, "addprim", "addprim(geo, type, pts, close)", true);
  add("add_prim", "Geometry", C::Variable, K::Function, "add_prim", "addprim(geo, type, pts, close)", true);
  add("setattribute",
      "Geometry",
      C::Variable,
      K::Function,
      "setattribute",
      "setattribute(geo, domain, index, name, value, behavior)",
      true);
  add("set_attribute",
      "Geometry",
      C::Variable,
      K::Function,
      "set_attribute",
      "setattribute(geo, domain, index, name, value, behavior)",
      true);
  add("setattr",
      "Geometry",
      C::Variable,
      K::Function,
      "setattr",
      "setattribute(geo, domain, index, name, value, behavior)",
      true);

  add("invert", "Matrix", C::Matrix, K::Function, "invert", "invert(m) / invert(q)", true);
  add("invert_matrix", "Matrix", C::Matrix, K::Function, "invert_matrix", "invert_matrix(m)", true);
  add("invert_rotation", "Rotation", C::Rotation, K::Function, "invert_rotation", "invert_rotation(q)", true);
  add("rotate_rotation", "Rotation", C::Rotation, K::Function, "rotate_rotation", "rotate_rotation(a, b)", true);
  add("rotate", "Rotation", C::Rotation, K::Function, "rotate", "rotate(a, b)", true);
  add("rotation", "Rotation", C::Rotation, K::Function, "rotation", "rotation(x, y, z, w)", true);
  add("quaternion", "Rotation", C::Rotation, K::Function, "quaternion", "quaternion(x, y, z, w)", true);
  add("vector2", "Vector", C::Vector, K::Function, "vector2", "vector2(x, y)", true);
  add("vec2", "Vector", C::Vector, K::Function, "vec2", "vec2(x, y) alias of vector2", true);
  add("vec3", "Vector", C::Vector, K::Function, "vec3", "vec3(x, y, z) alias of vector", true);
  add("vector4", "Vector", C::Vector, K::Function, "vector4", "vector4(x, y, z, w)", true);
  add("vec4", "Vector", C::Vector, K::Function, "vec4", "vec4(...) alias of vector4", true);
  add("matrix2", "Matrix", C::Matrix, K::Function, "matrix2", "matrix2(...) 2x2", true);
  add("mat2", "Matrix", C::Matrix, K::Function, "mat2", "mat2(...) alias of matrix2", true);
  add("matrix3", "Matrix", C::Matrix, K::Function, "matrix3", "matrix3(...) 3x3", true);
  add("mat3", "Matrix", C::Matrix, K::Function, "mat3", "mat3(...) alias of matrix3", true);
  add("mat4", "Matrix", C::Matrix, K::Function, "mat4", "mat4(...) alias of matrix", true);
  add("transpose", "Matrix", C::Matrix, K::Function, "transpose", "transpose(m)", true);
  add("determinant", "Matrix", C::Matrix, K::Function, "determinant", "determinant(m)", true);
  add("det", "Matrix", C::Matrix, K::Function, "det", "det(m)", true);
  add("ident", "Matrix", C::Matrix, K::Function, "ident", "ident()", true);
  add("identity", "Matrix", C::Matrix, K::Function, "identity", "identity()", true);
  add("combine_transform",
      "Matrix",
      C::Matrix,
      K::Function,
      "combine_transform",
      "combine_transform(t, r, s)",
      true);
  add("transform_point",
      "Matrix",
      C::Matrix,
      K::Function,
      "transform_point",
      "transform_point(v, m)",
      true);
  add("svd", "Matrix", C::Matrix, K::Function, "svd", "svd(m, U, s, V)  —  m = U * diag(s) * Vᵀ", true);
  add("pd", "Matrix", C::Matrix, K::Function, "pd", "pd(m, R, S)  —  polar: m = R * S", true);
  add("polardecomp", "Matrix", C::Matrix, K::Function, "polardecomp", "polardecomp(m, R, S)", true);
  add("eigen", "Matrix", C::Matrix, K::Function, "eigen", "eigen(m, evals, evecs)  对称特征分解", true);
  add("transform_direction",
      "Matrix",
      C::Matrix,
      K::Function,
      "transform_direction",
      "transform_direction(v, m)",
      true);
  add("project_point", "Matrix", C::Matrix, K::Function, "project_point", "project_point(v, m)", true);
  add("translation", "Matrix", C::Matrix, K::Function, "translation", "translation(m)", true);
  add("rotation", "Matrix", C::Matrix, K::Function, "rotation", "rotation(m)", true);
  add("scale", "Matrix", C::Matrix, K::Function, "scale", "scale(m)", true);

  add("chf", "Channel", C::Float, K::Function, "chf", "chf(\"name\")", true);
  add("ch", "Channel", C::Float, K::Function, "ch", "ch(\"name\")", true);
  add("chi", "Channel", C::Variable, K::Function, "chi", "chi(\"name\")", true);
  add("chv", "Channel", C::Vector, K::Function, "chv", "chv(\"name\")", true);
  add("chb", "Channel", C::Variable, K::Function, "chb", "chb(\"name\")", true);
  add("chc", "Channel", C::Color, K::Function, "chc", "chc(\"name\")", true);
  add("chm", "Channel", C::Matrix, K::Function, "chm", "chm(\"name\")", true);
  add("chq", "Channel", C::Rotation, K::Function, "chq", "chq(\"name\")", true);
  add("chs", "Channel", C::Variable, K::Function, "chs", "chs(\"name\")", true);
  add("cht", "Channel", C::Variable, K::Function, "cht", "cht(\"code\")", true);
  add("valuetostring",
      "String",
      C::Variable,
      K::Function,
      "valuetostring",
      "valuetostring(v, before[, after])",
      true);
  add("format",
      "String",
      C::Variable,
      K::Function,
      "format",
      "format(\"{:.2f}\", x) / sprintf(\"%05d\", i)",
      true);
  add("sprintf", "String", C::Variable, K::Function, "sprintf", "sprintf(\"%05d\", i)", true);

  add("array", "Array", C::Variable, K::Function, "array", "array(1, 2, 3)", true);
  add("append", "Array", C::Variable, K::Function, "append", "append(arr, x)", true);
  add("insert", "Array", C::Variable, K::Function, "insert", "insert(arr, i, x)", true);
  add("removeindex", "Array", C::Variable, K::Function, "removeindex", "removeindex(arr, i)", true);
  add("removevalue", "Array", C::Variable, K::Function, "removevalue", "removevalue(arr, x)", true);
  add("sort", "Array", C::Variable, K::Function, "sort", "sort(arr)", true);
  add("len", "Array", C::Variable, K::Function, "len", "len(arr)", true);

  add("pointneighbours", "Topology", C::Variable, K::Function, "pointneighbours", "pointneighbours(0, i@index)", true);
  add("pointedges", "Topology", C::Variable, K::Function, "pointedges", "pointedges(0, i@index)", true);
  add("pointfaces", "Topology", C::Variable, K::Function, "pointfaces", "pointfaces(0, i@index)", true);
  add("pointcorners", "Topology", C::Variable, K::Function, "pointcorners", "pointcorners(0, i@index)", true);
  add("edgepoints", "Topology", C::Variable, K::Function, "edgepoints", "edgepoints(0, i@index)", true);
  add("edgefaces", "Topology", C::Variable, K::Function, "edgefaces", "edgefaces(0, i@index)", true);
  add("edgecorners", "Topology", C::Variable, K::Function, "edgecorners", "edgecorners(0, i@index)", true);
  add("facepoints", "Topology", C::Variable, K::Function, "facepoints", "facepoints(0, i@index)", true);
  add("faceedges", "Topology", C::Variable, K::Function, "faceedges", "faceedges(0, i@index)", true);
  add("faceneighbours", "Topology", C::Variable, K::Function, "faceneighbours", "faceneighbours(0, i@index)", true);
  add("facecorners", "Topology", C::Variable, K::Function, "facecorners", "facecorners(0, i@index)", true);
  add("cornerface", "Topology", C::Variable, K::Function, "cornerface", "cornerface(0, i@index, faceindex, order)", true);
  add("cornerpoint", "Topology", C::Variable, K::Function, "cornerpoint", "cornerpoint(0, i@index)", true);
  add("corneredges", "Topology", C::Variable, K::Function, "corneredges", "corneredges(0, i@index)", true);
  add("edges_of_corner", "Topology", C::Variable, K::Function, "edges_of_corner", "edges_of_corner()", true);
  add("face_of_corner", "Topology", C::Variable, K::Function, "face_of_corner", "face_of_corner()", true);
  add("vertex_of_corner", "Topology", C::Variable, K::Function, "vertex_of_corner", "vertex_of_corner()", true);
  add("offsetcorner",
      "Topology",
      C::Variable,
      K::Function,
      "offsetcorner",
      "offsetcorner(geo, index, offset)",
      true);
  add("pointcurve", "Topology", C::Variable, K::Function, "pointcurve", "pointcurve(0, i@index, curveindex, orderincurve)", true);
  add("curvepoints", "Topology", C::Variable, K::Function, "curvepoints", "curvepoints(0, i@index)", true);
  add("points_of_curve", "Topology", C::Variable, K::Function, "points_of_curve", "points_of_curve()", true);
  add("curve_of_point", "Topology", C::Variable, K::Function, "curve_of_point", "curve_of_point()", true);
  add("nearestpoints",
      "Topology",
      C::Variable,
      K::Function,
      "nearestpoints",
      "nearestpoints(geo, k_or_r, mode)  mode: k|r|rk",
      true);
  add("nearpoints",
      "Topology",
      C::Variable,
      K::Function,
      "nearpoints",
      "nearpoints(geo, k_or_r, mode)  mode: k|r|rk",
      true);

  add("raycast",
      "Geometry",
      C::Variable,
      K::Function,
      "raycast",
      "raycast(geo, origin, dir, length, is_hit, hit_pos, hit_n, hit_dist)",
      true);
  add("raycastall",
      "Geometry",
      C::Variable,
      K::Function,
      "raycastall",
      "ray hits[] = raycastall(geo, origin, dir, length)",
      true);
  add("rayishit", "Geometry", C::Variable, K::Function, "rayishit", "rayishit(r)", true);
  add("rayhitpos", "Geometry", C::Vector, K::Function, "rayhitpos", "rayhitpos(r)", true);
  add("rayhitn", "Geometry", C::Vector, K::Function, "rayhitn", "rayhitnormal(r)", true);
  add("rayhitdist", "Geometry", C::Float, K::Function, "rayhitdist", "rayhitdist(r)", true);
  add("bboxmin", "Geometry", C::Vector, K::Function, "bboxmin", "bboxmin()", true);
  add("bboxmax", "Geometry", C::Vector, K::Function, "bboxmax", "bboxmax()", true);
  add("boundingbox", "Geometry", C::Vector, K::Function, "boundingbox", "bounding_box(min, max)", true);
  add("bounding_box", "Geometry", C::Vector, K::Function, "bounding_box", "bounding_box(min, max)", true);
  add("geometry_proximity",
      "Geometry",
      C::Variable,
      K::Function,
      "geometry_proximity",
      "geometry_proximity(geo, domain, sample, pos, dist)  domain: point|edge|face",
      true);
  add("proximity",
      "Geometry",
      C::Variable,
      K::Function,
      "proximity",
      "geometry_proximity(geo, domain, sample, pos, dist)  domain: point|edge|face",
      true);
  add("sample_nearest_surface", "Geometry", C::Variable, K::Function, "sample_nearest_surface", "sample_nearest_surface(geo, data_type, sample_pos, value)", true);
  add("delete_geometry",
      "Geometry",
      C::Variable,
      K::Function,
      "delete_geometry",
      "delete_geometry(domain, index, behavior)  index=current; other domain maps",
      true);
  add("deletegeometry",
      "Geometry",
      C::Variable,
      K::Function,
      "deletegeometry",
      "delete_geometry(domain, index, behavior)  index=current; other domain maps",
      true);
  add("accumulate", "Field", C::Float, K::Function, "accumulate", "accumulate(value)", true);
  add("accumulate_field", "Field", C::Float, K::Function, "accumulate_field", "accumulate_field(value)", true);
  add("field_average", "Field", C::Float, K::Function, "field_average", "field_average(\"name\")", true);
  add("fieldaverage", "Field", C::Float, K::Function, "fieldaverage", "fieldaverage(\"name\")", true);
  add("field_min", "Field", C::Float, K::Function, "field_min", "field_min(\"name\")", true);
  add("field_max", "Field", C::Float, K::Function, "field_max", "field_max(\"name\")", true);
  add("fieldminmax", "Field", C::Vector, K::Function, "fieldminmax", "fieldminmax(\"name\")", true);
  add("field_minmax", "Field", C::Vector, K::Function, "field_minmax", "field_minmax(\"name\")", true);

  if (shader_material) {
    r_items.remove_if([](const expression::CompletionItem &item) {
      return item.category_label == "Geometry" || item.category_label == "Field";
    });
    add("ddx", "Float", C::Float, K::Function, "ddx", "ddx(x) — screen-space dFdx (EEVEE)", true);
    add("ddy", "Float", C::Float, K::Function, "ddy", "ddy(x) — screen-space dFdy (EEVEE)", true);
    add("fwidth",
        "Float",
        C::Float,
        K::Function,
        "fwidth",
        "fwidth(x) — abs(ddx)+abs(ddy) (EEVEE)",
        true);
    add("dFdx", "Float", C::Float, K::Function, "dFdx", "dFdx(x) — GLSL alias of ddx", true);
    add("dFdy", "Float", C::Float, K::Function, "dFdy", "dFdy(x) — GLSL alias of ddy", true);
  }
}

}  // namespace blender::nodes::vex
