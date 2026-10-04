/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "testing/testing.h"

#include <cmath>
#include <cstdio>

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_gtest_setup.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "BKE_wrangle_array.hh"

#include "BLI_cpp_type.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"

#include "DNA_mesh_types.h"
#include "DNA_curves_types.h"

#include "FN_field.hh"

#include "NOD_vex.hh"
#include "vex_program.hh"

namespace blender::nodes::vex::tests {

static void ensure_vex_runtime()
{
  static bool done = false;
  if (!done) {
    blender::bke::gtest_setup();
    done = true;
  }
}

static PureEvalOutput eval_source(const StringRef source)
{
  CompileOutput compiled = compile(source);
  if (!compiled.program) {
    PureEvalOutput out;
    out.ok = false;
    out.error = compiled.error.empty() ? "compile failed" : compiled.error;
    return out;
  }
  return execute_pure(*compiled.program);
}

TEST(nodes_vex, for_loop_sum)
{
  const PureEvalOutput out = eval_source(
      "int s = 0; for (int i = 0; i < 10000; i++) { s += 1; } return s;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 10000);
}

TEST(nodes_vex, for_break)
{
  const PureEvalOutput out = eval_source(
      "int s = 0; for (int i = 0; i < 100; i++) { if (i == 10) { break; } s += 1; } return s;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 10);
}

TEST(nodes_vex, for_continue)
{
  const PureEvalOutput out = eval_source(
      "int s = 0; for (int i = 0; i < 10; i++) { if (i == 5) { continue; } s += 1; } return s;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 9);
}

TEST(nodes_vex, small_fixed_for_loop_is_unrolled)
{
  const CompileOutput compiled = compile(
      "int s = 0; for (int i = 0; i < 3; i++) { s += i; } return s;");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  for (const Inst &in : compiled.program->code) {
    EXPECT_FALSE(ELEM(in.op, Op::Jmp, Op::JmpIfFalse, Op::JmpIfTrue));
  }
  const PureEvalOutput out = execute_pure(*compiled.program);
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 3);
}

TEST(nodes_vex, diagnostics_keep_the_primary_source_location)
{
  const std::string source = "float ok = 1;\nfloat bad = no_such_fn(1);\n";
  const CompileOutput compiled = compile(source);
  ASSERT_FALSE(bool(compiled.program));
  ASSERT_FALSE(compiled.diags.is_empty());
  EXPECT_EQ(compiled.diags[0].offset, int(source.find("no_such_fn")));
  EXPECT_NE(compiled.error.find("L2:"), std::string::npos);
}

TEST(nodes_vex, lexer_diagnostics_preserve_the_actual_error_kind)
{
  const CompileOutput string_error = compile("string s = \"unfinished;");
  ASSERT_FALSE(string_error.diags.is_empty());
  EXPECT_EQ(string_error.diags[0].message, "字符串少了结束引号");

  const CompileOutput comment_error = compile("float a = 1; /* unfinished");
  ASSERT_FALSE(comment_error.diags.is_empty());
  EXPECT_EQ(comment_error.diags[0].message, "块注释少了结束符 */");
}

TEST(nodes_vex, while_loop)
{
  const PureEvalOutput out = eval_source(
      "int s = 0; int i = 0; while (i < 7) { s += i; i++; } return s;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 21);
}

TEST(nodes_vex, user_fn_add)
{
  const PureEvalOutput out = eval_source(
      "int add(int a, int b) { return a + b; }\n"
      "return add(2, 3);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 5);
}

TEST(nodes_vex, user_fn_houdini_param_groups)
{
  const PureEvalOutput a = eval_source(
      "int add3(int a, b, c) { return a + b + c; }\n"
      "return add3(1, 2, 3);");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.return_int, 6);

  const PureEvalOutput b = eval_source(
      "float mix2(vector2 A, B; float t) {\n"
      "  vector2 r = A * (1.0 - t) + B * t;\n"
      "  return r.x + r.y;\n"
      "}\n"
      "return int(mix2({0, 0}, {4, 6}, 0.5));");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(b.return_int, 5);
}

TEST(nodes_vex, circumradius_vector2_cross)
{
  const PureEvalOutput out = eval_source(
      "float circumradius(vector2 A, B, C) {\n"
      "  float a = length(B - C);\n"
      "  float b = length(C - A);\n"
      "  float c = length(A - B);\n"
      "  float area2 = abs(cross(B - A, C - A));\n"
      "  return (a * b * c) / (2.0 * area2);\n"
      "}\n"
      "return circumradius({0, 0}, {2, 0}, {0, 2});");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_NEAR(out.return_float, 1.41421356f, 1e-5f);
}

TEST(nodes_vex, user_fn_locals_after_main_decls)
{
  /* Non-tail call + main locals: returning a function-local must not yield 0. */
  const PureEvalOutput area = eval_source(
      "float circumradius(vector A, B, C) {\n"
      "  float a = length(B - C);\n"
      "  float b = length(C - A);\n"
      "  float c = length(A - B);\n"
      "  float area2 = length(cross(B - A, C - A));\n"
      "  return area2;\n"
      "}\n"
      "vector p0 = {0, 0, 0};\n"
      "vector p1 = {2, 0, 0};\n"
      "vector p2 = {0, 2, 0};\n"
      "float r = circumradius(p0, p1, p2);\n"
      "return r;");
  ASSERT_TRUE(area.ok) << area.error;
  EXPECT_NEAR(area.return_float, 4.0f, 1e-5f);

  const PureEvalOutput radius = eval_source(
      "float circumradius(vector A, B, C) {\n"
      "  float a = length(B - C);\n"
      "  float b = length(C - A);\n"
      "  float c = length(A - B);\n"
      "  float area2 = length(cross(B - A, C - A));\n"
      "  return (a * b * c) / (2.0 * area2);\n"
      "}\n"
      "vector p0 = {0, 0, 0};\n"
      "vector p1 = {2, 0, 0};\n"
      "vector p2 = {0, 2, 0};\n"
      "float r = circumradius(p0, p1, p2);\n"
      "return r;");
  ASSERT_TRUE(radius.ok) << radius.error;
  EXPECT_NEAR(radius.return_float, 1.41421356f, 1e-5f);
}

TEST(nodes_vex, vector_matrix_ops_2_3_4d)
{
  const PureEvalOutput a = eval_source(
      "vector2 u = abs({-3, 4}); return int(length(u));");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.return_int, 5);

  const PureEvalOutput b = eval_source(
      "vector n = normalize({0, 3, 0}); return int(n.y);");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(b.return_int, 1);

  const PureEvalOutput c = eval_source(
      "vector4 q = {1, 0, 0, 0}; return int(length(q));");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.return_int, 1);

  const PureEvalOutput d = eval_source(
      "matrix2 m = ident(); return int(determinant(m) + determinant(invert(m)));");
  ASSERT_TRUE(d.ok) << d.error;
  EXPECT_EQ(d.return_int, 2);

  const PureEvalOutput e = eval_source(
      "matrix3 m = ident() * 2; return int(determinant(m));");
  ASSERT_TRUE(e.ok) << e.error;
  EXPECT_EQ(e.return_int, 8);

  const PureEvalOutput f = eval_source(
      "vector2 r = min({1, 5}, {2, 3}); return int(r.x + r.y);");
  ASSERT_TRUE(f.ok) << f.error;
  EXPECT_EQ(f.return_int, 4);
}

TEST(nodes_vex, user_fn_recursion_linear)
{
  /* Linear recursion must stay O(n), not explode per extra frame. */
  const PureEvalOutput out = eval_source(
      "int f(int n) {\n"
      "  if (n <= 0) { return 0; }\n"
      "  return f(n - 1) + 1;\n"
      "}\n"
      "return f(200);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 200);
}

TEST(nodes_vex, user_fn_recursion_tail)
{
  const PureEvalOutput out = eval_source(
      "int f(int n, int acc) {\n"
      "  if (n <= 0) { return acc; }\n"
      "  return f(n - 1, acc + 1);\n"
      "}\n"
      "return f(500, 0);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 500);
}

TEST(nodes_vex, user_fn_recursion_fib)
{
  const PureEvalOutput out = eval_source(
      "int fib(int n) {\n"
      "  if (n <= 1) { return n; }\n"
      "  return fib(n - 1) + fib(n - 2);\n"
      "}\n"
      "return fib(10);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 55);
}

TEST(nodes_vex, compile_error_unknown_function)
{
  const CompileOutput compiled = compile("foo(1);");
  EXPECT_FALSE(bool(compiled.program));
  EXPECT_FALSE(compiled.error.empty());
}

TEST(nodes_vex, string_literals_single_and_double)
{
  const PureEvalOutput a = eval_source("string s = \"hi\"; return 1;");
  ASSERT_TRUE(a.ok) << a.error;
  const PureEvalOutput b = eval_source("string s = 'hi'; return 2;");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(b.return_int, 2);
}

TEST(nodes_vex, matrix_identity_and_invert)
{
  const PureEvalOutput out = eval_source(
      "matrix m = ident(); matrix n = invert(m); return int(determinant(n));");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 1);
}

TEST(nodes_vex, matrix_brace_literal_not_identity)
{
  const PureEvalOutput out = eval_source(
      "matrix m = {2, 0, 0, 0, 0, 3, 0, 0, 0, 0, 4, 0, 0, 0, 0, 1}; return int(determinant(m));");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 24);
}

TEST(nodes_vex, matrix_constructor_not_identity)
{
  const PureEvalOutput out = eval_source(
      "matrix m = matrix(2, 0, 0, 0, 0, 3, 0, 0, 0, 0, 4, 0, 0, 0, 0, 1); return int(determinant(m));");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 24);
}

TEST(nodes_vex, matrix_default_is_identity)
{
  const PureEvalOutput out = eval_source("matrix m; return int(determinant(m));");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 1);
}

TEST(nodes_vex, matrix_from_float_is_diag_scale)
{
  /* One float → xyz diagonal, m44 stays 1. det = 2*2*2*1 = 8. */
  const PureEvalOutput a = eval_source("matrix m = 2; return int(determinant(m));");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.return_int, 8);
  const PureEvalOutput b = eval_source("matrix m = 2.0; return int(determinant(m));");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(b.return_int, 8);
  const PureEvalOutput c = eval_source("matrix m = {2}; return int(determinant(m));");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.return_int, 8);
  const PureEvalOutput d = eval_source("matrix m = matrix(2); return int(determinant(m));");
  ASSERT_TRUE(d.ok) << d.error;
  EXPECT_EQ(d.return_int, 8);
  const PureEvalOutput e = eval_source(
      "matrix m; m = 2; vector p = transform_point({1, 1, 1}, m); return int(p.x + p.y + p.z);");
  ASSERT_TRUE(e.ok) << e.error;
  EXPECT_EQ(e.return_int, 6);
  const PureEvalOutput f = eval_source("matrix m = ident() * 2; return int(determinant(m));");
  ASSERT_TRUE(f.ok) << f.error;
  EXPECT_EQ(f.return_int, 8);
  const PureEvalOutput g = eval_source(
      "matrix m = {2, 3, 4}; return int(determinant(m));");
  ASSERT_TRUE(g.ok) << g.error;
  EXPECT_EQ(g.return_int, 24);
}

TEST(nodes_vex, combine_transform_compiles)
{
  const CompileOutput compiled = compile(
      "matrix m = combine_transform({0, 0, 0}, {0, 0, 0}, {1, 1, 1}); v@P = transform_point(v@P, m);");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
}

TEST(nodes_vex, chf_channel_compiles)
{
  const CompileOutput compiled = compile("v@P += chv(\"v\") * chf(\"f\");");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
}

TEST(nodes_vex, counted_position_add_is_array_kernel)
{
  const CompileOutput compiled = compile(
      "vector offset = {0.1, 0, 0};\n"
      "for (int i = 0; i < 1000; i++) {\n"
      "  v@P += offset;\n"
      "}\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(program_can_array_exec(*compiled.program));
  bool has_muladd = false;
  bool has_loop_jmp = false;
  for (const Inst &in : compiled.program->code) {
    if (in.op == Op::AttrMulAdd) {
      has_muladd = true;
    }
    if (in.op == Op::Jmp || in.op == Op::JmpIfFalse) {
      has_loop_jmp = true;
    }
  }
  EXPECT_TRUE(has_muladd);
  EXPECT_FALSE(has_loop_jmp);
}

TEST(nodes_vex, while_attr_cmp_add_is_c_kernel)
{
  const CompileOutput compiled = compile("while (v@P.x < 2.0) { v@P.x += 0.1; }\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(program_can_array_exec(*compiled.program));
  bool has_while = false;
  bool has_jmp = false;
  for (const Inst &in : compiled.program->code) {
    if (in.op == Op::WhileCmpAdd) {
      has_while = true;
    }
    if (in.op == Op::Jmp || in.op == Op::JmpIfFalse) {
      has_jmp = true;
    }
  }
  EXPECT_TRUE(has_while);
  EXPECT_FALSE(has_jmp);
}

TEST(nodes_vex, while_attr_cmp_add_with_counter)
{
  const CompileOutput compiled = compile(
      "while (v@P.x < 2.0) { v@P.x += 0.001; i@counter++; }\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(program_can_array_exec(*compiled.program));
  bool has_while = false;
  bool has_jmp = false;
  for (const Inst &in : compiled.program->code) {
    if (in.op == Op::WhileCmpAdd) {
      has_while = true;
    }
    if (in.op == Op::Jmp || in.op == Op::JmpIfFalse) {
      has_jmp = true;
    }
  }
  EXPECT_TRUE(has_while);
  EXPECT_FALSE(has_jmp);
  ASSERT_FALSE(compiled.program->while_adds.is_empty());
  EXPECT_EQ(compiled.program->while_adds[0].counters.size(), 1);
}

TEST(nodes_vex, const_matrix_array_assign_is_array_exec)
{
  const CompileOutput compiled = compile(
      "m@m = {1,2,3,0, 3.1,2.1, 4.1,0,3,1, 3.3,0,0,1};\n"
      "m@ma = array(m@m, m@m, m@m);\n"
      "f[]@fa = {0.1, 0.2, 0.3, 0.4};\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(program_can_array_exec(*compiled.program));
  for (const Inst &in : compiled.program->code) {
    EXPECT_NE(in.op, Op::PushAttr);
    EXPECT_NE(in.op, Op::Jmp);
  }
}

TEST(nodes_vex, raycast_const_dir_is_batched)
{
  const CompileOutput compiled = compile(
      "int hit; vector hp, hn; float hd;\n"
      "raycast(0, v@P, {0, 0, -1}, 100, hit, hp, hn, hd);\n"
      "i@hit = hit; v@hp = hp; v@hn = hn; f@hd = hd;\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->ray_batch);
  EXPECT_TRUE(compiled.program->spatial_only);
  EXPECT_EQ(compiled.program->ray_geo_mask, 1u);
  EXPECT_FALSE(compiled.program->ray_geo_dynamic);
  EXPECT_GE(compiled.program->ray_hit_attr, 0);
  EXPECT_GE(compiled.program->ray_pos_attr, 0);
}

TEST(nodes_vex, spatial_literal_geometry_inputs_are_tracked)
{
  const CompileOutput compiled = compile(
      "int hit; vector hp; vector hn; float hd;\n"
      "raycast(2, v@P, {0, 0, -1}, 100, hit, hp, hn, hd);\n"
      "vector pp; float pd; geometry_proximity(1, \"face\", v@P, pp, pd);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_EQ(compiled.program->ray_geo_mask, uint8_t(1u << 2));
  EXPECT_EQ(compiled.program->prox_geo_mask, uint8_t(1u << 1));
  EXPECT_FALSE(compiled.program->ray_geo_dynamic);
  EXPECT_FALSE(compiled.program->prox_geo_dynamic);
}

TEST(nodes_vex, sampled_self_attributes_are_tracked)
{
  const CompileOutput compiled = compile(
      "vector a = corner(0, @UV, 1); vector b = point(1, \"P\", 2); "
      "string name = \"weight\"; float c = point(0, name, 3); f@result = a.x + b.x + c;");
  ASSERT_TRUE(compiled.program);
  EXPECT_TRUE(compiled.program->sampled_self_attrs.contains("UV"));
  EXPECT_FALSE(compiled.program->sampled_self_attrs.contains("position"));
  EXPECT_TRUE(compiled.program->sampled_self_attr_dynamic);
  ASSERT_EQ(compiled.program->element_samples.size(), 2);
  EXPECT_EQ(compiled.program->element_samples[0].geo, 0);
  EXPECT_EQ(compiled.program->element_samples[0].domain, 3);
  EXPECT_EQ(compiled.program->element_samples[0].name, "UV");
  EXPECT_EQ(compiled.program->element_samples[1].geo, 1);
  EXPECT_EQ(compiled.program->element_samples[1].domain, 0);
  EXPECT_EQ(compiled.program->element_samples[1].name, "position");
  int static_samples = 0;
  int dynamic_samples = 0;
  for (const Inst &in : compiled.program->code) {
    static_samples += in.op == Op::SampleElem;
    dynamic_samples += in.op == Op::Call && Builtin(in.imm) == Builtin::Point;
  }
  EXPECT_EQ(static_samples, 2);
  EXPECT_EQ(dynamic_samples, 1);
}

TEST(nodes_vex, static_sample_array_loop_is_fused)
{
  const CompileOutput compiled = compile(
      "int corners[] = facecorners(0, i@index);\n"
      "vector uv[] = array(); vector pos[] = array();\n"
      "for (int i = 0; i < len(corners); i++) {\n"
      "  uv[i] = corner(0, \"UVMap\", corners[i]);\n"
      "  pos[i] = corner(0, \"P\", corners[i]);\n"
      "}\n"
      "f@probe = uv[0].x + pos[1].y;\n"
      "v[]@uv_copy = uv;\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  ASSERT_EQ(compiled.program->gather_samples.size(), 1);
  EXPECT_EQ(compiled.program->gather_samples[0].output_locals.size(), 2);
  ASSERT_EQ(compiled.program->gather_samples[0].materialize.size(), 2);
  EXPECT_EQ(compiled.program->gather_samples[0].materialize[0], 1);
  EXPECT_EQ(compiled.program->gather_samples[0].materialize[1], 0);
  ASSERT_EQ(compiled.program->local_index_loads.size(), 2);
  for (const Program::LocalIndexLoad &load : compiled.program->local_index_loads) {
    EXPECT_GE(load.sample_slot, 0);
    EXPECT_GE(load.sample_index_array_local, 0);
  }
  int fused = 0;
  for (const Inst &in : compiled.program->code) {
    fused += in.op == Op::GatherSamples;
  }
  EXPECT_EQ(fused, 1);
}

TEST(nodes_vex, immutable_local_condition_removes_dead_branch)
{
  const CompileOutput compiled = compile(
      "int enabled = 0; float value = 2.0;\n"
      "if (enabled == 1) { value = sqrt(9.0) + length(set(1, 2, 3)); }\n"
      "return int(value);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  for (const Inst &in : compiled.program->code) {
    EXPECT_NE(in.op, Op::SqrtF);
    EXPECT_NE(in.op, Op::LengthV3);
  }
  const PureEvalOutput out = execute_pure(*compiled.program);
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 2);
}

TEST(nodes_vex, local_array_and_member_loads_are_fused)
{
  const CompileOutput compiled = compile(
      "vector values[] = array(set(1, 2, 3), set(4, 5, 6));\n"
      "int i = 1; vector plain = set(7, 8, 9);\n"
      "return int(values[i].y + values[0].z + plain.x);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  int indexed = 0;
  int members = 0;
  for (const Inst &in : compiled.program->code) {
    indexed += in.op == Op::PushLocalIndex;
    members += in.op == Op::PushLocalMember;
    EXPECT_NE(in.op, Op::Nop);
  }
  EXPECT_EQ(indexed, 2);
  EXPECT_EQ(members, 1);
  const PureEvalOutput out = execute_pure(*compiled.program);
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 15);
}

TEST(nodes_vex, common_typed_builtins_are_lowered_directly)
{
  const CompileOutput compiled = compile(
      "vector a = set(1, 2, 3); vector b = set(4, 5, 6); int ids[] = array(1, 2, 3);\n"
      "float x = length(a) + dot(a, b) + length(cross(a, b));\n"
      "x += sqrt(4.0) + log(2.0) + abs(-1.0) + float(x);\n"
      "return len(ids) + int(x);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  int direct = 0;
  int generic = 0;
  for (const Inst &in : compiled.program->code) {
    direct += ELEM(in.op,
                   Op::LengthV3,
                   Op::DotV3,
                   Op::CrossV3,
                   Op::SqrtF,
                   Op::LogF,
                   Op::AbsF,
                   Op::ArrayLen);
    generic += in.op == Op::Call &&
               ELEM(Builtin(in.imm),
                    Builtin::Length,
                    Builtin::Dot,
                    Builtin::Cross,
                    Builtin::Sqrt,
                    Builtin::Log,
                    Builtin::Abs,
                    Builtin::Len,
                    Builtin::FloatFn);
  }
  EXPECT_EQ(direct, 8);
  EXPECT_EQ(generic, 0);
}

TEST(nodes_vex, raycast_dir_attr_and_normalize_is_batched)
{
  const CompileOutput a = compile(
      "int hit; vector hp, hn; float hd;\n"
      "raycast(0, v@P, v@N, 100, hit, hp, hn, hd);\n"
      "i@hit = hit; v@hp = hp; v@hn = hn; f@hd = hd;\n");
  ASSERT_TRUE(bool(a.program)) << a.error;
  EXPECT_TRUE(a.program->ray_batch);
  EXPECT_TRUE(a.program->spatial_only);
  EXPECT_GE(a.program->ray_dir_attr, 0);
  EXPECT_FALSE(a.program->ray_dir_normalize);

  const CompileOutput b = compile(
      "int hit; vector hp, hn; float hd;\n"
      "raycast(0, v@P, normalize(v@N), 100, hit, hp, hn, hd);\n"
      "i@hit = hit; v@hp = hp; v@hn = hn; f@hd = hd;\n");
  ASSERT_TRUE(bool(b.program)) << b.error;
  EXPECT_TRUE(b.program->ray_batch);
  EXPECT_TRUE(b.program->spatial_only);
  EXPECT_TRUE(b.program->ray_dir_normalize);
  EXPECT_GE(b.program->ray_dir_attr, 0);
  EXPECT_GE(b.program->ray_hit_attr, 0);
  EXPECT_GE(b.program->ray_pos_attr, 0);
}

TEST(nodes_vex, nearestpoints_k_precomputes_table)
{
  const CompileOutput compiled = compile("i[]@knn = nearestpoints(0, 8, \"k\");\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->knn_precompute);
  EXPECT_EQ(compiled.program->knn_k, 8);
  EXPECT_EQ(compiled.program->knn_geo, 0);
}

TEST(nodes_vex, raycast_hitpos_stored_to_two_attrs)
{
  const CompileOutput compiled = compile(
      "bool ishit; vector hitpos, hitn; float hitd;\n"
      "ray r = raycast(1, v@P, {0, 0, 1}, 100, ishit, hitpos, hitn, hitd);\n"
      "v@temp = rayhitpos(r);\n"
      "v@hitpos = hitpos;\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->ray_batch);
  EXPECT_TRUE(compiled.program->spatial_only);
  EXPECT_GE(int(compiled.program->ray_pos_attrs.size()), 2);
  bool has_temp = false;
  bool has_hitpos = false;
  for (const int slot : compiled.program->ray_pos_attrs) {
    ASSERT_GE(slot, 0);
    ASSERT_LT(slot, int(compiled.program->attrs.size()));
    if (compiled.program->attrs[slot].name == "temp") {
      has_temp = true;
    }
    if (compiled.program->attrs[slot].name == "hitpos") {
      has_hitpos = true;
    }
  }
  EXPECT_TRUE(has_temp);
  EXPECT_TRUE(has_hitpos);
}

TEST(nodes_vex, raycastall_compiles_to_array_outs)
{
  const CompileOutput compiled = compile(
      "int hit[]; vector hp[], hn[]; float hd[];\n"
      "ray hits[] = raycastall(0, v@P, {0, 0, -1}, 100, hit, hp, hn, hd);\n"
      "i@n = len(hits);\n"
      "i[]@hit = hit; v[]@hp = hp; v[]@hn = hn; f[]@hd = hd;\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_FALSE(compiled.program->spatial_only);
  bool writes_hit = false;
  bool writes_hp = false;
  bool writes_hd = false;
  for (const AttrInfo &a : compiled.program->attrs) {
    if (a.name == "hit" && a.type == Type::IntArray && a.write) {
      writes_hit = true;
    }
    if (a.name == "hp" && a.type == Type::VecArray && a.write) {
      writes_hp = true;
    }
    if (a.name == "hd" && a.type == Type::FloatArray && a.write) {
      writes_hd = true;
    }
  }
  EXPECT_TRUE(writes_hit);
  EXPECT_TRUE(writes_hp);
  EXPECT_TRUE(writes_hd);
}

TEST(nodes_vex, proximity_is_spatial_kernel)
{
  const CompileOutput compiled = compile(
      "vector ppos; float pdist;\n"
      "geometry_proximity(0, \"face\", v@P, ppos, pdist);\n"
      "v@hit = ppos; f@dist = pdist;\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->prox_batch);
  EXPECT_TRUE(compiled.program->spatial_only);
  EXPECT_GE(compiled.program->prox_pos_attr, 0);
  EXPECT_GE(compiled.program->prox_dist_attr, 0);
}

TEST(nodes_vex, nearestpoints_unified_signature_compiles)
{
  const CompileOutput compiled = compile(
      "i[]@knn = nearestpoints(0, 8, \"k\");\n"
      "i[]@rnn = nearestpoints(1, 0.25, \"r\");\n"
      "i[]@knn2 = nearestpoints(0, 8, 0);\n"
      "i[]@rnn2 = nearestpoints(0, 0.5, 1, v@P);\n"
      "i[]@rk = nearestpoints(0, 0.25, 8, \"rk\");\n"
      "i[]@rk2 = nearestpoints(0, 0.25, \"rk\", 8);\n"
      "i[]@rk3 = nearestpoints(0, 0.5, 4, \"rk\", v@P);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
}

TEST(nodes_vex, nearestpoints_rk_does_not_precompute_knn_table)
{
  const CompileOutput compiled = compile("i[]@pts = nearestpoints(0, 0.25, 8, \"rk\");\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_FALSE(compiled.program->knn_precompute);
}

TEST(nodes_vex, knn_for_loop_keeps_int_array)
{
  const CompileOutput compiled = compile(
      "for (int i = 0; i < chi('ite'); i++) {\n"
      "  int pts[] = nearestpoints(0, 12, \"k\");\n"
      "  i[]@pts = pts;\n"
      "  vector o = {0, 0, 0};\n"
      "  foreach (int pt; pts) {\n"
      "    o += point(0, 'P', pt);\n"
      "  }\n"
      "  o /= len(pts);\n"
      "  v@P = o;\n"
      "}\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->jacobi_knn);
  EXPECT_EQ(compiled.program->jacobi_knn_k, 12);
  EXPECT_FALSE(compiled.program->array_pass_peeled);
  bool saw_pts = false;
  for (const AttrInfo &a : compiled.program->attrs) {
    if (a.name == "pts") {
      saw_pts = true;
      EXPECT_EQ(int(a.type), int(Type::IntArray));
      EXPECT_TRUE(a.write);
    }
  }
  EXPECT_TRUE(saw_pts);
}

TEST(nodes_vex, knn_jacobi_pointcloud_does_not_crash)
{
  ensure_vex_runtime();
  PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, 9);
  MutableSpan<float3> pos = pc->positions_for_write();
  for (int i = 0; i < 9; i++) {
    pos[i] = float3(float(i % 3), float(i / 3), 0.0f);
  }
  bke::GeometrySet geometry = bke::GeometrySet::from_pointcloud(pc);
  /* Keep a second owner so Jacobi `positions_for_write` copy-on-writes. */
  bke::GeometrySet shared = geometry;
  const CompileOutput compiled = compile(
      "for (int i = 0; i < 3; i++) {\n"
      "  int pts[] = nearestpoints(0, 4, \"k\");\n"
      "  vector sum = 0;\n"
      "  foreach (int pt; pts) {\n"
      "    sum += point(0, 'P', pt);\n"
      "  }\n"
      "  v@P = sum / len(pts);\n"
      "}\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->jacobi_knn);
  fn::Field<bool> selection(true);
  const ExecOutput out = execute(*compiled.program, geometry, {}, Domain::Point, selection);
  ASSERT_TRUE(out.ok) << out.error;
  const PointCloud *out_pc = geometry.get_pointcloud();
  ASSERT_NE(out_pc, nullptr);
  EXPECT_EQ(out_pc->totpoint, 9);
  const Span<float3> out_pos = out_pc->positions();
  ASSERT_EQ(out_pos.size(), 9);
  EXPECT_TRUE(std::isfinite(out_pos[0].x));
  EXPECT_TRUE(std::isfinite(out_pos[4].x));
}

TEST(nodes_vex, rk_for_loop_is_not_jacobi)
{
  const CompileOutput compiled = compile(
      "for (int i = 0; i < chi(\"ite\"); i++) {\n"
      "  int pts[] = nearestpoints(0, f@radius * 2, 8, \"rk\");\n"
      "  i[]@pts = pts;\n"
      "  vector sum = 0;\n"
      "  foreach (int pt; pts) {\n"
      "    sum += point(0, \"P\", pt);\n"
      "  }\n"
      "  v@P += sum / len(pts);\n"
      "}\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_FALSE(compiled.program->jacobi_smooth);
  EXPECT_FALSE(compiled.program->jacobi_knn);
  EXPECT_TRUE(compiled.program->array_pass_peeled);
}

TEST(nodes_vex, nearestpoints_rk_caps_at_k_inside_radius)
{
  ensure_vex_runtime();
  PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, 9);
  MutableSpan<float3> pos = pc->positions_for_write();
  for (int i = 0; i < 9; i++) {
    pos[i] = float3(float(i % 3), float(i / 3), 0.0f);
  }
  bke::GeometrySet geometry = bke::GeometrySet::from_pointcloud(pc);
  const CompileOutput compiled = compile(
      "int pts[] = nearestpoints(0, 1.01, 2, \"rk\");\n"
      "i[]@pts = pts;\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_FALSE(compiled.program->knn_precompute);
  fn::Field<bool> selection(true);
  const ExecOutput out = execute(*compiled.program, geometry, {}, Domain::Point, selection);
  ASSERT_TRUE(out.ok) << out.error;
  const PointCloud *out_pc = geometry.get_pointcloud();
  ASSERT_NE(out_pc, nullptr);
  const bke::AttributeAccessor attributes = out_pc->attributes();
  const bke::AttributeReader<bke::WrangleArrayValue> pts = attributes.lookup<bke::WrangleArrayValue>(
      "pts", bke::AttrDomain::Point);
  ASSERT_TRUE(pts);
  ASSERT_EQ(pts.varray.size(), 9);
  /* Center (1,1): 4 orthogonal neighbors at dist 1; cap k=2. Corner (0,0): only 2 in radius. */
  EXPECT_EQ(int(pts.varray[4].count), 2) << pts.varray[4].to_string();
  EXPECT_EQ(int(pts.varray[0].count), 2) << pts.varray[0].to_string();
  EXPECT_LE(int(pts.varray[0].count), 2);
  for (int v = 0; v < 9; v++) {
    EXPECT_LE(int(pts.varray[v].count), 2) << "vert " << v;
  }
}

TEST(nodes_vex, radius_overlap_loop_is_not_jacobi)
{
  const CompileOutput compiled = compile(
      "for (int i = 0; i < chi(\"ite\"); i++) {\n"
      "  int pts[] = nearestpoints(0, f@radius * 2, \"r\");\n"
      "  i[]@pts = pts;\n"
      "  vector sum = 0;\n"
      "  foreach (int pt; pts) {\n"
      "    vector np = point(0, \"P\", pt);\n"
      "    float nr = point(0, \"radius\", pt);\n"
      "    vector dir = normalize(v@P - np);\n"
      "    float dist = distance(v@P, np);\n"
      "    if (dist < (f@radius + nr)) {\n"
      "      sum += dir * (f@radius + nr - dist);\n"
      "    }\n"
      "  }\n"
      "  v@P += sum / len(pts);\n"
      "}\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_FALSE(compiled.program->jacobi_smooth);
  EXPECT_FALSE(compiled.program->jacobi_knn);
  EXPECT_TRUE(compiled.program->array_pass_peeled);
  EXPECT_EQ(compiled.program->array_passes_ch, "ite");
}

TEST(nodes_vex, nearestpoints_radius_foreach_does_not_crash)
{
  ensure_vex_runtime();
  PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, 64);
  MutableSpan<float3> pos = pc->positions_for_write();
  for (int i = 0; i < 64; i++) {
    pos[i] = float3(float(i % 8), float(i / 8), 0.0f);
  }
  bke::GeometrySet geometry = bke::GeometrySet::from_pointcloud(pc);
  const CompileOutput compiled = compile(
      "for (int i = 0; i < 3; i++) {\n"
      "  int pts[] = nearestpoints(0, 50.0, \"r\");\n"
      "  float s = 0;\n"
      "  foreach (int pt; pts) {\n"
      "    s += 1;\n"
      "  }\n"
      "  f@count = s;\n"
      "}\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  fn::Field<bool> selection(true);
  const ExecOutput out = execute(*compiled.program, geometry, {}, Domain::Point, selection);
  ASSERT_TRUE(out.ok) << out.error;
  const PointCloud *out_pc = geometry.get_pointcloud();
  ASSERT_NE(out_pc, nullptr);
  EXPECT_EQ(out_pc->totpoint, 64);
}

TEST(nodes_vex, index_and_rand_compile)
{
  const CompileOutput compiled = compile("v@P += v@N * rand(i@index); i@id = i@index;");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
}

TEST(nodes_vex, quaternion_literal_invert_and_rotate)
{
  const PureEvalOutput out = eval_source(
      "quaternion q = {0, 0, 0, 1}; "
      "quaternion n = invert(q); "
      "quaternion r = rotate_rotation(q, n); "
      "return int(r.w);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 1);
}

TEST(nodes_vex, rotation_keyword_is_quaternion)
{
  const PureEvalOutput out = eval_source(
      "rotation q = {0, 0, 0, 1}; "
      "rotation n = invert(q); "
      "return int(n.w);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 1);
}

TEST(nodes_vex, rotation_mul_is_rotate_rotation)
{
  const PureEvalOutput a = eval_source(
      "rotation q = {0, 0, 0, 1}; "
      "rotation n = invert(q); "
      "rotation r = q * n; "
      "return int(r.w);");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.return_int, 1);
  const PureEvalOutput b = eval_source(
      "rotation q = {0, 0, 0, 1}; "
      "vector v = q * {0, 1, 0}; "
      "return int(v.y);");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(b.return_int, 1);
}

TEST(nodes_vex, matrix_and_vector_subscript)
{
  const PureEvalOutput a = eval_source(
      "matrix3 m = matrix3(1, 0, 0, 0, 2, 0, 0, 0, 3); "
      "return int(m[1][1]);");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.return_int, 2);
  const PureEvalOutput b = eval_source(
      "matrix3 m = matrix3(); "
      "m[0][1] = 4; "
      "return int(m[0][1]);");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(b.return_int, 4);
  const PureEvalOutput c = eval_source("vector v = {1, 2, 3}; return int(v[1]);");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.return_int, 2);
}

TEST(nodes_vex, rand_pure)
{
  const PureEvalOutput out = eval_source("return int(rand(0) + rand(1) >= 0.0);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 1);
}

TEST(nodes_vex, array_len_append_removeindex)
{
  const PureEvalOutput a = eval_source("int[] xs = array(1, 2, 3); return len(xs);");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.return_int, 3);
  const PureEvalOutput b = eval_source(
      "int[] xs = array(1, 2, 3); removeindex(xs, -1); return len(xs);");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(b.return_int, 2);
  const PureEvalOutput c = eval_source("int[] xs = array(3, 1, 2); sort(xs); return xs[0];");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.return_int, 1);
}

TEST(nodes_vex, array_index_assign)
{
  const PureEvalOutput out = eval_source("int[] xs = array(4, 5, 6); xs[1] = 9; return xs[1];");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 9);
}

TEST(nodes_vex, array_index_grows_and_pads_zero)
{
  const PureEvalOutput len = eval_source("float abc[] = array(); abc[3] = 1.2; return len(abc);");
  ASSERT_TRUE(len.ok) << len.error;
  EXPECT_EQ(len.return_int, 4);
  const PureEvalOutput gap = eval_source(
      "float abc[] = array(); abc[3] = 1.2; return int(abc[0] + abc[1] + abc[2]);");
  ASSERT_TRUE(gap.ok) << gap.error;
  EXPECT_EQ(gap.return_int, 0);
  const PureEvalOutput wrote = eval_source(
      "float abc[] = array(); abc[3] = 1.2; return int(abc[3] > 1.0);");
  ASSERT_TRUE(wrote.ok) << wrote.error;
  EXPECT_EQ(wrote.return_int, 1);
  const PureEvalOutput mid = eval_source(
      "int xs[] = array(4, 5); xs[4] = 9; return xs[2] + xs[3] + xs[4];");
  ASSERT_TRUE(mid.ok) << mid.error;
  EXPECT_EQ(mid.return_int, 9);
  const PureEvalOutput neg = eval_source("int xs[] = array(); xs[-1] = 3; return len(xs);");
  ASSERT_TRUE(neg.ok) << neg.error;
  EXPECT_EQ(neg.return_int, 0);
}

TEST(nodes_vex, color_literal_and_members)
{
  const PureEvalOutput out = eval_source("color c = {1, 0, 0}; return int(c.r + c.a);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 2);
}

TEST(nodes_vex, color_rgba_literal)
{
  const PureEvalOutput out = eval_source("color c = {0, 1, 0, 1}; return int(c.g + c.a);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 2);
}

TEST(nodes_vex, chs_chq_compile)
{
  const CompileOutput compiled = compile("string s = chs(\"name\"); r@orient = chq(\"rot\");");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
}

TEST(nodes_vex, r_at_is_rotation)
{
  const CompileOutput compiled = compile("r@orient = {0, 0, 0, 1};");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  ASSERT_FALSE(compiled.program->attrs.is_empty());
  EXPECT_EQ(compiled.program->attrs[0].type, Type::Rotation);
}

TEST(nodes_vex, vector2_vector4_and_matrix23_locals)
{
  const PureEvalOutput a = eval_source("vector2 u = {2, 3}; return int(u.x + u.y);");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.return_int, 5);
  const PureEvalOutput b = eval_source("vector4 q = {1, 2, 3, 4}; return int(q.w);");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(b.return_int, 4);
  const PureEvalOutput c = eval_source("matrix2 m = 2; return int(determinant(m));");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.return_int, 4);
  const PureEvalOutput d = eval_source("matrix3 m = 2; return int(determinant(m));");
  ASSERT_TRUE(d.ok) << d.error;
  EXPECT_EQ(d.return_int, 8);
  const PureEvalOutput e = eval_source(
      "matrix2 m = {2, 0, 0, 3}; vector2 v = {1, 1}; vector2 r = m * v; return int(r.x + r.y);");
  ASSERT_TRUE(e.ok) << e.error;
  EXPECT_EQ(e.return_int, 5);
}

TEST(nodes_vex, vector_swizzle)
{
  const PureEvalOutput a = eval_source(
      "vector p = {1, 2, 3}; p = p.zxy; return int(p.x * 100 + p.y * 10 + p.z);");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.return_int, 312);

  const PureEvalOutput yzx = eval_source(
      "vector p = {1, 2, 3}; p = p.yzx; return int(p.x * 100 + p.y * 10 + p.z);");
  ASSERT_TRUE(yzx.ok) << yzx.error;
  EXPECT_EQ(yzx.return_int, 231);

  const PureEvalOutput b = eval_source(
      "vector p = {1, 2, 3}; vector2 u = p.zx; return int(u.x * 10 + u.y);");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(b.return_int, 31);

  const PureEvalOutput c = eval_source(
      "vector p = {1, 2, 3}; p.xy = {7, 8}; return int(p.x * 100 + p.y * 10 + p.z);");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.return_int, 783);
}

TEST(nodes_vex, attr_prefixes_u_q_r_234)
{
  const CompileOutput u = compile("u@uv = {1, 2};");
  ASSERT_TRUE(bool(u.program)) << u.error;
  EXPECT_EQ(u.program->attrs[0].type, Type::Vector2);
  const CompileOutput q = compile("q@tangent = {1, 0, 0, 1};");
  ASSERT_TRUE(bool(q.program)) << q.error;
  EXPECT_EQ(q.program->attrs[0].type, Type::Vector4);
  const CompileOutput m2 = compile("2@m = 1;");
  ASSERT_TRUE(bool(m2.program)) << m2.error;
  EXPECT_EQ(m2.program->attrs[0].type, Type::Matrix2);
  const CompileOutput m3 = compile("3@m = 1;");
  ASSERT_TRUE(bool(m3.program)) << m3.error;
  EXPECT_EQ(m3.program->attrs[0].type, Type::Matrix3);
  const CompileOutput m4 = compile("4@m = ident();");
  ASSERT_TRUE(bool(m4.program)) << m4.error;
  EXPECT_EQ(m4.program->attrs[0].type, Type::Matrix);
}

TEST(nodes_vex, compile_error_has_location)
{
  const CompileOutput compiled = compile("foo(1);");
  EXPECT_FALSE(bool(compiled.program));
  EXPECT_NE(compiled.error.find("L"), std::string::npos);
  EXPECT_NE(compiled.error.find("foo"), std::string::npos);
  ASSERT_FALSE(compiled.diags.is_empty());
  EXPECT_TRUE(compiled.diags[0].is_error);
}

TEST(nodes_vex, compile_error_missing_semicolon_is_specific)
{
  const CompileOutput compiled = compile("v@P += {0, 0, 1}");
  EXPECT_FALSE(bool(compiled.program));
  EXPECT_NE(compiled.error.find("v@P"), std::string::npos);
  EXPECT_TRUE(compiled.error.find(";") != std::string::npos ||
              compiled.error.find("分号") != std::string::npos);
}

TEST(nodes_vex, texture_and_geo_builtins_compile)
{
  const CompileOutput compiled = compile(
      "float n = noise(v@P, 5, 2, 0.5);\n"
      "float v = voronoi(v@P, 5);\n"
      "int hit = raycast({0, 0, -1});\n"
      "vector[] box = boundingbox();\n"
      "vector mn, mx;\n"
      "bounding_box(mn, mx);\n"
      "int ishit; vector hp, hn; float hd;\n"
      "raycast(0, v@P, {0, 0, -1}, 100, ishit, hp, hn, hd);\n"
      "int ishits[]; vector hps[], hns[]; float hds[];\n"
      "ray hits[] = raycastall(0, v@P, {0, 0, -1}, 100, ishits, hps, hns, hds);\n"
      "vector ppos; float pdist;\n"
      "geometry_proximity(0, \"face\", v@P, ppos, pdist);\n"
      "vector sval;\n"
      "sample_nearest_surface(0, \"position\", v@P, sval);\n"
      "delete_geometry(\"point\", i@index, \"all\");\n"
      "int f = addprim(0, \"face\", array(0, 1, 2));\n"
      "int e = add_prim(0, \"edge\", array(0, 1, 2), false);\n"
      "vector mm = fieldminmax(\"scale\");\n"
      "float avg = field_average(\"scale\");\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
}

TEST(nodes_vex, delete_geometry_domain_string_or_int)
{
  const CompileOutput a = compile("delete_geometry(\"point\", i@index, \"all\");\n");
  ASSERT_TRUE(bool(a.program)) << a.error;
  const CompileOutput b = compile("delete_geometry(0, i@index, 0);\n");
  ASSERT_TRUE(bool(b.program)) << b.error;
  const CompileOutput c = compile("deletegeometry(\"edge\", i@index, \"edge_face\");\n");
  ASSERT_TRUE(bool(c.program)) << c.error;
  const CompileOutput d = compile("delete_geometry(0, \"point\", i@index, \"all\");\n");
  ASSERT_TRUE(bool(d.program)) << d.error;
  const CompileOutput bad = compile("delete_geometry(v@P, i@index, 0);\n");
  EXPECT_FALSE(bool(bad.program));
  EXPECT_NE(bad.error.find("Type mismatch"), std::string::npos);
}

TEST(nodes_vex, geo_nonzero_is_forced_to_zero_with_warning)
{
  const CompileOutput a = compile("addpoint(2, {0, 1, 0});\n");
  ASSERT_TRUE(bool(a.program)) << a.error;
  EXPECT_NE(a.warning.find("geo"), std::string::npos);
  EXPECT_NE(a.warning.find("2"), std::string::npos);
  const CompileOutput b = compile(
      "setattribute(3, \"point\", i@index, \"P\", {0,0,0}, \"set\");\n");
  ASSERT_TRUE(bool(b.program)) << b.error;
  EXPECT_NE(b.warning.find("3"), std::string::npos);
  const CompileOutput c = compile("delete_geometry(4, \"point\", i@index, \"all\");\n");
  ASSERT_TRUE(bool(c.program)) << c.error;
  EXPECT_NE(c.warning.find("4"), std::string::npos);
}

TEST(nodes_vex, addpoint_and_setattribute_signatures_compile)
{
  const CompileOutput a = compile(
      "int p = addpoint(0, v@P);\n"
      "int q = addpoint(0, i@index);\n"
      "setattribute(0, \"point\", i@index, \"scale\", 1.5, \"set\");\n"
      "setattribute(0, \"point\", p, \"scale\", 2.0, \"add\");\n");
  ASSERT_TRUE(bool(a.program)) << a.error;
  EXPECT_TRUE(a.warning.empty()) << a.warning;
}

TEST(nodes_vex, addpoint_appends_and_setattr_writes)
{
  ensure_vex_runtime();
  PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, 4);
  MutableSpan<float3> pos = pc->positions_for_write();
  for (int i = 0; i < 4; i++) {
    pos[i] = float3(float(i), 0.0f, 0.0f);
  }
  bke::GeometrySet geometry = bke::GeometrySet::from_pointcloud(pc);
  const CompileOutput compiled = compile(
      "setattribute(0, \"point\", i@index, \"scale\", 2.0, \"set\");\n"
      "addpoint(0, v@P + {0, 1, 0});\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  fn::Field<bool> selection(true);
  const ExecOutput out = execute(*compiled.program, geometry, {}, Domain::Point, selection);
  ASSERT_TRUE(out.ok) << out.error;
  const PointCloud *out_pc = geometry.get_pointcloud();
  ASSERT_NE(out_pc, nullptr);
  EXPECT_EQ(out_pc->totpoint, 8);
  const bke::AttributeAccessor attributes = out_pc->attributes();
  const bke::AttributeReader<float> scale = attributes.lookup<float>("scale",
                                                                     bke::AttrDomain::Point);
  ASSERT_TRUE(scale);
  EXPECT_NEAR(scale.varray[0], 2.0f, 1e-5f);
}

TEST(nodes_vex, addpoint_without_if_appends_per_element)
{
  ensure_vex_runtime();
  PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, 4);
  MutableSpan<float3> pos = pc->positions_for_write();
  for (int i = 0; i < 4; i++) {
    pos[i] = float3(float(i), 0.0f, 0.0f);
  }
  bke::GeometrySet geometry = bke::GeometrySet::from_pointcloud(pc);
  const CompileOutput compiled = compile("addpoint(0, v@P + {0, 1, 0});\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  fn::Field<bool> selection(true);
  const ExecOutput out = execute(*compiled.program, geometry, {}, Domain::Point, selection);
  ASSERT_TRUE(out.ok) << out.error;
  const PointCloud *out_pc = geometry.get_pointcloud();
  ASSERT_NE(out_pc, nullptr);
  EXPECT_EQ(out_pc->totpoint, 8);
}

TEST(nodes_vex, delete_geometry_without_if_deletes_all_points)
{
  ensure_vex_runtime();
  PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, 3);
  MutableSpan<float3> pos = pc->positions_for_write();
  pos[0] = float3(0.0f, 0.0f, 0.0f);
  pos[1] = float3(1.0f, 0.0f, 0.0f);
  pos[2] = float3(0.0f, 1.0f, 0.0f);
  bke::GeometrySet geometry = bke::GeometrySet::from_pointcloud(pc);
  const CompileOutput compiled = compile("delete_geometry(\"point\", i@index, \"all\");\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  fn::Field<bool> selection(true);
  const ExecOutput out = execute(*compiled.program, geometry, {}, Domain::Point, selection);
  ASSERT_TRUE(out.ok) << out.error;
  const PointCloud *out_pc = geometry.get_pointcloud();
  if (out_pc) {
    EXPECT_EQ(out_pc->totpoint, 0);
  }
}

TEST(nodes_vex, addprim_face_and_edge)
{
  ensure_vex_runtime();
  Mesh *src = BKE_mesh_new_nomain(4, 0, 0, 0);
  MutableSpan<float3> pos = src->vert_positions_for_write();
  pos[0] = float3(0.0f, 0.0f, 0.0f);
  pos[1] = float3(1.0f, 0.0f, 0.0f);
  pos[2] = float3(1.0f, 1.0f, 0.0f);
  pos[3] = float3(0.0f, 1.0f, 0.0f);
  src->tag_positions_changed();
  bke::GeometrySet geometry = bke::GeometrySet::from_mesh(src);
  const CompileOutput compiled = compile(
      "if (i@index == 0) {\n"
      "  addprim(0, \"face\", array(0, 1, 2, 3));\n"
      "  addprim(0, \"edge\", array(0, 1, 2));\n"
      "}\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  fn::Field<bool> selection(true);
  const ExecOutput out = execute(*compiled.program, geometry, {}, Domain::Point, selection);
  ASSERT_TRUE(out.ok) << out.error;
  const Mesh *mesh = geometry.get_mesh();
  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->faces_num, 1);
  EXPECT_GE(mesh->edges_num, 2);
  EXPECT_EQ(mesh->faces()[0].size(), 4);
}

TEST(nodes_vex, addprim_curve_cyclic)
{
  ensure_vex_runtime();
  Mesh *src = BKE_mesh_new_nomain(3, 0, 0, 0);
  MutableSpan<float3> pos = src->vert_positions_for_write();
  pos[0] = float3(0.0f, 0.0f, 0.0f);
  pos[1] = float3(1.0f, 0.0f, 0.0f);
  pos[2] = float3(0.0f, 1.0f, 0.0f);
  src->tag_positions_changed();
  bke::GeometrySet geometry = bke::GeometrySet::from_mesh(src);
  const CompileOutput compiled = compile(
      "if (i@index == 0) {\n"
      "  addprim(0, \"curve\", array(0, 1, 2), true);\n"
      "}\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  fn::Field<bool> selection(true);
  const ExecOutput out = execute(*compiled.program, geometry, {}, Domain::Point, selection);
  ASSERT_TRUE(out.ok) << out.error;
  const Curves *curves_id = geometry.get_curves();
  ASSERT_NE(curves_id, nullptr);
  const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  EXPECT_EQ(curves.curves_num(), 1);
  EXPECT_EQ(curves.points_num(), 3);
  EXPECT_TRUE(curves.cyclic()[0]);
}

TEST(nodes_vex, noise_voronoi_full_args_compile)
{
  const CompileOutput n = compile(
      "f@a = noise(v@P, 5, 2, 0.5, 2.0, 0.1, 3, \"fbm\");\n"
      "f@b = noise(v@P, 5, 2, 0.5, 2.0, 0.1, \"2d\", \"ridged\");\n");
  ASSERT_TRUE(bool(n.program)) << n.error;
  const CompileOutput v = compile(
      "f@c = voronoi(v@P, 5, 2, 0.5, 2.0, 1, 1, \"f1\", \"euclidean\", 3, 0.1);\n"
      "f@d = voronoi(v@P, 5, 0, 0.5, 2.0, 1, 1, 0, 0, \"2d\", 0);\n");
  ASSERT_TRUE(bool(v.program)) << v.error;
}

TEST(nodes_vex, noise_full_args_eval)
{
  const PureEvalOutput out = eval_source(
      "float n = noise({0.2, 0.3, 0.4}, 5, 2, 0.5, 2.0, 0.1, 3, \"fbm\"); return n > -2 && n < 2;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 1);
  const PureEvalOutput v = eval_source(
      "float x = voronoi({0.2, 0.3, 0.4}, 5, 2, 0.5, 2.0, 1, 1, \"f1\", 0, 3, 0.1); "
      "return x >= 0;");
  ASSERT_TRUE(v.ok) << v.error;
  EXPECT_EQ(v.return_int, 1);
}

TEST(nodes_vex, int_array_brace_literal_keeps_length)
{
  const PureEvalOutput out = eval_source("int[] xs = {1, 2, 3, 4}; return len(xs);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 4);
}

TEST(nodes_vex, int_array_fn_keeps_length)
{
  const PureEvalOutput out = eval_source("int[] xs = array(1, 2, 3, 4, 5); return len(xs);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 5);
}

TEST(nodes_vex, c_style_array_name_suffix)
{
  const PureEvalOutput out = eval_source("int pts[] = {1, 2, 3, 4}; return len(pts);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 4);
}

TEST(nodes_vex, string_array_literal_compiles)
{
  const CompileOutput compiled = compile(
      "string[] names = array(\"a\", \"b\");\n"
      "s[]@tags = array(\"x\", \"y\", \"z\");\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
}

TEST(nodes_vex, comma_multi_var_decl)
{
  const PureEvalOutput out = eval_source(
      "float a = 0.1, b = 0.2; vector p, q, r; return int((a + b) * 10);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 3);
}

TEST(nodes_vex, foreach_value)
{
  const PureEvalOutput out = eval_source(
      "int pts[] = {10, 20, 30}; int s = 0; foreach (int pt; pts) { s += pt; } return s;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 60);
}

TEST(nodes_vex, foreach_index_value)
{
  const PureEvalOutput out = eval_source(
      "int pts[] = {10, 20, 30}; int s = 0; foreach (int idx; int pt; pts) { s += idx + pt; } "
      "return s;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 63);
}

TEST(nodes_vex, foreach_untyped)
{
  const PureEvalOutput out = eval_source(
      "int pts[] = {10, 20, 30}; int s = 0; foreach (pt; pts) { s += pt; } return s;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 60);
}

TEST(nodes_vex, foreach_untyped_index)
{
  const PureEvalOutput out = eval_source(
      "int pts[] = {10, 20, 30}; int s = 0; foreach (idx; pt; pts) { s += idx + pt; } return s;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 63);
}

TEST(nodes_vex, foreach_over_array_call)
{
  const PureEvalOutput out = eval_source(
      "int s = 0; foreach (int pt; array(10, 20, 30)) { s += pt; } return s;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 60);
}

TEST(nodes_vex, matrix_array_attr_compiles)
{
  const CompileOutput compiled = compile("m[]@xforms = array(ident(), ident());\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  bool writes_mat_array = false;
  if (compiled.program) {
    for (const AttrInfo &a : compiled.program->attrs) {
      if (a.name == "xforms" && a.type == Type::MatArray && a.write) {
        writes_mat_array = true;
      }
    }
  }
  EXPECT_TRUE(writes_mat_array);
}

TEST(nodes_vex, topology_geo_placeholder_ident_compiles)
{
  /* Usage docs say `fn(geo, index)`; `geo` is input 0, not a float attribute. */
  const CompileOutput compiled = compile(
      "int[] pn = pointneighbours(geo, index);\n"
      "int[] ec = edgecorners(geo, index);\n"
      "int cp = cornerpoint(geo, index);\n"
      "int[] ce = corneredges(geo, index);\n"
      "int oc = offsetcorner(geo, index, 1);\n"
      "int[] cps = curvepoints(geo, index);\n"
      "int cidx, corder;\n"
      "pointcurve(geo, index, cidx, corder);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
}

TEST(nodes_vex, topology_geo_index_builtins_compile)
{
  const CompileOutput compiled = compile(
      "int[] pn = pointneighbours(0, i@index);\n"
      "int[] pe = pointedges(0, i@index);\n"
      "int[] pf = pointfaces(0, i@index);\n"
      "int[] pc = pointcorners(0, i@index);\n"
      "int[] ep = edgepoints(0, 0);\n"
      "int[] ef = edgefaces(0, 0);\n"
      "int[] ec = edgecorners(0, 0);\n"
      "int[] fp = facepoints(0, 0);\n"
      "int[] fe = faceedges(0, 0);\n"
      "int[] fn = faceneighbours(0, 0);\n"
      "int[] fc = facecorners(0, 0);\n"
      "int cp = cornerpoint(0, 0);\n"
      "int[] ce = corneredges(0, 0);\n"
      "int oc = offsetcorner(0, 0, 1);\n"
      "int fi, ord;\n"
      "cornerface(0, 0, fi, ord);\n"
      "int[] cf = cornerface(0, 0);\n"
      "int cidx, corder;\n"
      "pointcurve(0, i@index, cidx, corder);\n"
      "int[] cps = curvepoints(0, 0);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
}

TEST(nodes_vex, topology_i_at_upgrades_to_array)
{
  const CompileOutput compiled = compile("i@nbr = pointneighbours(0, i@index);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  bool writes_int_array = false;
  if (compiled.program) {
    for (const AttrInfo &a : compiled.program->attrs) {
      if (a.name == "nbr" && a.type == Type::IntArray && a.write) {
        writes_int_array = true;
      }
    }
  }
  EXPECT_TRUE(writes_int_array);
}

TEST(nodes_vex, string_and_array_attr_store_compiles)
{
  const CompileOutput compiled = compile(
      "s@name = \"hello\";\n"
      "i[]@ids = array(1, 2, 3);\n"
      "f[]@ws = array(0.5, 1.0);\n"
      "v[]@dirs = array({1, 0, 0}, {0, 1, 0});\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  bool writes_string = false;
  bool writes_array = false;
  if (compiled.program) {
    for (const AttrInfo &a : compiled.program->attrs) {
      if (a.name == "name" && a.type == Type::String && a.write) {
        writes_string = true;
      }
      if (a.name == "ids" && a.type == Type::IntArray && a.write) {
        writes_array = true;
      }
    }
  }
  EXPECT_TRUE(writes_string);
  EXPECT_TRUE(writes_array);
}

TEST(nodes_vex, gpu_source_for_rand_while)
{
  const CompileOutput compiled = compile(
      "while (v@P.x < 2) {\n"
      "  v@P.x += rand() * 0.002 + 0.0002;\n"
      "  i@counter++;\n"
      "}\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok);
  EXPECT_TRUE(compiled.program->gpu_soa);
  EXPECT_FALSE(compiled.program->gpu_src.empty());
  EXPECT_NE(compiled.program->gpu_src.find("while"), std::string::npos);
  EXPECT_NE(compiled.program->gpu_src.find("wr_rand0"), std::string::npos);
}

TEST(nodes_vex, gpu_source_for_map_smooth)
{
  const CompileOutput compiled = compile(
      "for (int i = 0; i < 1; i++) {\n"
      "  f@a = map(f@a, 0.0, 1.0, 0.0, 2.0) + smooth(f@b);\n"
      "}\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.error;
  EXPECT_NE(compiled.program->gpu_src.find("smoothstep"), std::string::npos);
}

TEST(nodes_vex, gpu_source_for_matrix_and_array_rand)
{
  const CompileOutput compiled = compile(
      "m@m = matrix(rand(), 2, 3, 0, 3.1, 2.1, 4.1, 0, 3, 1, 3.3, 0, 0, 1);\n"
      "f[]@fa = array(rand(), 0.2, 0.3, 0.4);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  /* Straight-line ctor stores stay on the CPU tile kernel (GPU SSBO round-trip is slower). */
  EXPECT_FALSE(compiled.program->gpu_ok);
}

TEST(nodes_vex, neighbour_smooth_peels_loop_and_gpu)
{
  const CompileOutput compiled = compile(
      "int pts[] = pointneighbours(0, i@index);\n"
      "for (int i = 0; i < chi('ite'); i++) {\n"
      "  vector o = 0;\n"
      "  foreach (int pt; pts) {\n"
      "    o += point(0, 'P', pt);\n"
      "  }\n"
      "  v@o = o / len(pts);\n"
      "  v@P += (o / len(pts) - v@P) * chf('fac');\n"
      "}\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->array_pass_peeled);
  EXPECT_TRUE(compiled.program->jacobi_smooth);
  EXPECT_EQ(compiled.program->array_passes_ch, "ite");
  EXPECT_EQ(compiled.program->jacobi_fac_ch, "fac");
  EXPECT_TRUE(compiled.program->gpu_neighbors);
  EXPECT_TRUE(compiled.program->gpu_ok);
  EXPECT_FALSE(compiled.program->gpu_src.empty());
  EXPECT_TRUE(compiled.program->gpu_soa);
  EXPECT_NE(compiled.program->gpu_src.find("wr_neighbours"), std::string::npos);
  EXPECT_NE(compiled.program->gpu_src.find("a0_in"), std::string::npos);
}

TEST(nodes_vex, sample_point_p_compiles)
{
  const CompileOutput a = compile("v@P = point(0, \"P\", i@index);\n");
  ASSERT_TRUE(bool(a.program)) << a.error;
  EXPECT_EQ(a.program->code.is_empty(), false);
  bool has_unquoted_p = false;
  for (const std::string &s : a.program->const_s) {
    EXPECT_NE(s, "\"P\"");
    if (s == "P") {
      has_unquoted_p = true;
    }
  }
  EXPECT_TRUE(has_unquoted_p);
  const CompileOutput b = compile("vector q = point(0, P, 0);\n");
  ASSERT_TRUE(bool(b.program)) << b.error;
  const CompileOutput c = compile(
      "vector q = curve(0, \"P\", 0); vector r = instance(0, \"P\", 0);\n"
      "vector e = edge(0, \"P\", 0); vector f = face(0, \"P\", 0);\n"
      "vector k = corner(0, \"P\", 0);\n");
  ASSERT_TRUE(bool(c.program)) << c.error;
}

TEST(nodes_vex, type_mismatch_vector_where_int_expected)
{
  const CompileOutput compiled = compile("i[]@a = pointneighbours(0, v@P);\n");
  EXPECT_FALSE(bool(compiled.program));
  EXPECT_NE(compiled.error.find("int"), std::string::npos);
  EXPECT_NE(compiled.error.find("vector"), std::string::npos);
}

TEST(nodes_vex, matrix_mul_dim_mismatch_is_error)
{
  const CompileOutput a = compile("matrix2 a = 1; matrix3 b = 1; matrix2 c = a * b;");
  EXPECT_FALSE(bool(a.program));
  EXPECT_NE(a.error.find("维度"), std::string::npos);
  const CompileOutput b = compile("matrix m = ident(); vector p = m * v@P;");
  EXPECT_FALSE(bool(b.program));
  EXPECT_NE(b.error.find("维度"), std::string::npos);
}

TEST(nodes_vex, vec_times_mat_warns_not_error)
{
  const CompileOutput compiled = compile("matrix3 m = 1; vector v = {1, 0, 0}; vector r = v * m;");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_FALSE(compiled.warning.empty());
}

TEST(nodes_vex, svd_pd_eigen_compile)
{
  const CompileOutput a = compile(
      "matrix3 m = 2; matrix3 U, V; vector s; svd(m, U, s, V); "
      "matrix3 R, S; pd(m, R, S); vector e; matrix3 ev; eigen(m, e, ev);");
  ASSERT_TRUE(bool(a.program)) << a.error;
}

TEST(nodes_vex, svd_diag_singular_values)
{
  const PureEvalOutput out = eval_source(
      "matrix3 m = matrix3(2, 0, 0, 0, 3, 0, 0, 0, 4); "
      "matrix3 U, V; vector s; svd(m, U, s, V); "
      "return int(s.x + s.y + s.z + 0.1);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 9);
}

TEST(nodes_vex, function_param_types_lookup)
{
  EXPECT_EQ(function_param_types("pointneighbours"), "geo,int");
  EXPECT_EQ(function_param_types("sin"), "num");
  EXPECT_EQ(function_param_types("valuetostring"), "num,int,int");
  EXPECT_EQ(function_param_types("format"), "str");
  EXPECT_EQ(function_param_types("offsetcorner"), "geo,int,int");
  EXPECT_EQ(function_param_types("nearestpoints"), "geo,num,enum");
  EXPECT_EQ(function_param_types("raycast"), "geo,vec,vec,float,out int,out vec,out vec,out float");
  EXPECT_EQ(function_param_types("raycastall"), "geo,vec,vec,float,out arr,out arr,out arr,out arr");
  EXPECT_EQ(function_param_types("rayishit"), "ray");
  EXPECT_EQ(function_param_types("map"), "num,num,num,num,num");
  EXPECT_EQ(function_param_types("smooth"), "num");
  EXPECT_TRUE(function_param_types("not_a_fn").is_empty());
}

TEST(nodes_vex, map_float_linear)
{
  const PureEvalOutput a = eval_source("return map(0.5, 0.0, 1.0, 10.0, 20.0);");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_NEAR(a.return_float, 15.0f, 1e-5f);

  const PureEvalOutput b = eval_source("return map(-1.0, 0.0, 1.0, 0.0, 10.0);");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_NEAR(b.return_float, -10.0f, 1e-5f);

  const PureEvalOutput c = eval_source("return map(5.0, 1.0, 1.0, 3.0, 8.0);");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_NEAR(c.return_float, 3.0f, 1e-5f);
}

TEST(nodes_vex, map_vector_broadcast)
{
  const PureEvalOutput out = eval_source(
      "vector v = map({0.0, 1.0, 0.5}, 0.0, 1.0, 0.0, 2.0); return v.x + v.y + v.z;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_NEAR(out.return_float, 3.0f, 1e-5f);
}

TEST(nodes_vex, smooth_hermite)
{
  const PureEvalOutput a = eval_source("return smooth(0.0);");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_NEAR(a.return_float, 0.0f, 1e-5f);

  const PureEvalOutput b = eval_source("return smooth(1.0);");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_NEAR(b.return_float, 1.0f, 1e-5f);

  const PureEvalOutput c = eval_source("return smooth(0.5);");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_NEAR(c.return_float, 0.5f, 1e-5f);

  const PureEvalOutput d = eval_source("return smooth(-1.0);");
  ASSERT_TRUE(d.ok) << d.error;
  EXPECT_NEAR(d.return_float, 0.0f, 1e-5f);

  const PureEvalOutput e = eval_source("return smooth(2.0);");
  ASSERT_TRUE(e.ok) << e.error;
  EXPECT_NEAR(e.return_float, 1.0f, 1e-5f);

  const PureEvalOutput f = eval_source("return smooth(0.25);");
  ASSERT_TRUE(f.ok) << f.error;
  EXPECT_NEAR(f.return_float, 0.15625f, 1e-5f);
}

TEST(nodes_vex, smooth_range)
{
  const PureEvalOutput a = eval_source("return smooth(0.0, 1.0, 0.25);");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_NEAR(a.return_float, 0.15625f, 1e-5f);

  const PureEvalOutput b = eval_source("return smooth(10.0, 20.0, 15.0);");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_NEAR(b.return_float, 0.5f, 1e-5f);

  const PureEvalOutput c = eval_source("return smooth(0.0, 1.0, -1.0);");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_NEAR(c.return_float, 0.0f, 1e-5f);

  const PureEvalOutput d = eval_source("return smooth(0.0, 1.0, 2.0);");
  ASSERT_TRUE(d.ok) << d.error;
  EXPECT_NEAR(d.return_float, 1.0f, 1e-5f);

  const PureEvalOutput e = eval_source("return smooth(5.0, 5.0, 4.0);");
  ASSERT_TRUE(e.ok) << e.error;
  EXPECT_NEAR(e.return_float, 0.0f, 1e-5f);

  const PureEvalOutput f = eval_source("return smooth(5.0, 5.0, 5.0);");
  ASSERT_TRUE(f.ok) << f.error;
  EXPECT_NEAR(f.return_float, 1.0f, 1e-5f);
}

TEST(nodes_vex, smooth_vector_componentwise)
{
  const PureEvalOutput out = eval_source(
      "vector v = smooth({-1.0, 0.5, 2.0}); return v.x + v.y + v.z;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_NEAR(out.return_float, 1.5f, 1e-5f);
}

TEST(nodes_vex, valuetostring_int_pads_and_keeps_extra)
{
  const PureEvalOutput a = eval_source("return valuetostring(5, 3);");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.return_string, "005");
  const PureEvalOutput b = eval_source("return valuetostring(1234, 2);");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(b.return_string, "1234");
  const PureEvalOutput c = eval_source("return valuetostring(-7, 3);");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.return_string, "-007");
}

TEST(nodes_vex, valuetostring_float_before_after)
{
  const PureEvalOutput a = eval_source("return valuetostring(3.14159, 2, 2);");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.return_string, "03.14");
  const PureEvalOutput b = eval_source("return valuetostring(123.4, 2, 2);");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(b.return_string, "123.40");
  const PureEvalOutput c = eval_source("return valuetostring(-3.1, 2, 1);");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.return_string, "-03.1");
}

TEST(nodes_vex, format_python_and_printf)
{
  const PureEvalOutput a = eval_source("return format(\"pt_{:04d}\", 12);");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_EQ(a.return_string, "pt_0012");
  const PureEvalOutput b = eval_source("return format(\"{}-{}\", 1, 2);");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_EQ(b.return_string, "1-2");
  const PureEvalOutput c = eval_source("return format(\"{:.2f}\", 3.14159);");
  ASSERT_TRUE(c.ok) << c.error;
  EXPECT_EQ(c.return_string, "3.14");
  const PureEvalOutput d = eval_source("return sprintf(\"%s_%05d\", \"id\", 7);");
  ASSERT_TRUE(d.ok) << d.error;
  EXPECT_EQ(d.return_string, "id_00007");
}

TEST(nodes_vex, foreach_completion_inserts_statement)
{
  Vector<std::string> owned;
  Vector<expression::CompletionItem> items;
  gather_completions(owned, items);
  bool found = false;
  for (const expression::CompletionItem &it : items) {
    if (it.name == "foreach") {
      EXPECT_TRUE(it.insert_text.startswith("foreach ("));
      EXPECT_TRUE(it.usage.startswith("foreach ("));
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST(nodes_vex, shader_completions_include_length)
{
  Vector<std::string> owned;
  Vector<expression::CompletionItem> items;
  gather_completions(owned, items, true);
  bool has_length = false;
  bool has_npoints = false;
  for (const expression::CompletionItem &it : items) {
    if (it.name == "length") {
      has_length = true;
    }
    if (it.name == "npoints" || it.name == "point") {
      has_npoints = true;
    }
  }
  EXPECT_TRUE(has_length);
  EXPECT_FALSE(has_npoints);
}

static Mesh *make_triangle_mesh()
{
  Mesh *mesh = BKE_mesh_new_nomain(3, 3, 1, 3);
  MutableSpan<float3> pos = mesh->vert_positions_for_write();
  pos[0] = float3(0.0f, 0.0f, 0.0f);
  pos[1] = float3(1.0f, 0.0f, 0.0f);
  pos[2] = float3(0.0f, 1.0f, 0.0f);
  MutableSpan<int2> edges = mesh->edges_for_write();
  edges[0] = int2(0, 1);
  edges[1] = int2(1, 2);
  edges[2] = int2(2, 0);
  mesh->face_offsets_for_write()[0] = 0;
  mesh->face_offsets_for_write()[1] = 3;
  MutableSpan<int> corners = mesh->corner_verts_for_write();
  corners[0] = 0;
  corners[1] = 1;
  corners[2] = 2;
  MutableSpan<int> corner_edges = mesh->corner_edges_for_write();
  corner_edges[0] = 0;
  corner_edges[1] = 1;
  corner_edges[2] = 2;
  mesh->tag_positions_changed();
  mesh->tag_topology_changed();
  return mesh;
}

static Mesh *make_two_layer_tris_mesh()
{
  Mesh *mesh = BKE_mesh_new_nomain(6, 6, 2, 6);
  MutableSpan<float3> pos = mesh->vert_positions_for_write();
  pos[0] = float3(0.0f, 0.0f, 0.0f);
  pos[1] = float3(1.0f, 0.0f, 0.0f);
  pos[2] = float3(0.0f, 1.0f, 0.0f);
  pos[3] = float3(0.0f, 0.0f, -1.0f);
  pos[4] = float3(1.0f, 0.0f, -1.0f);
  pos[5] = float3(0.0f, 1.0f, -1.0f);
  MutableSpan<int2> edges = mesh->edges_for_write();
  edges[0] = int2(0, 1);
  edges[1] = int2(1, 2);
  edges[2] = int2(2, 0);
  edges[3] = int2(3, 4);
  edges[4] = int2(4, 5);
  edges[5] = int2(5, 3);
  MutableSpan<int> offsets = mesh->face_offsets_for_write();
  offsets[0] = 0;
  offsets[1] = 3;
  offsets[2] = 6;
  MutableSpan<int> corners = mesh->corner_verts_for_write();
  corners[0] = 0;
  corners[1] = 1;
  corners[2] = 2;
  corners[3] = 3;
  corners[4] = 4;
  corners[5] = 5;
  MutableSpan<int> corner_edges = mesh->corner_edges_for_write();
  corner_edges[0] = 0;
  corner_edges[1] = 1;
  corner_edges[2] = 2;
  corner_edges[3] = 3;
  corner_edges[4] = 4;
  corner_edges[5] = 5;
  mesh->tag_positions_changed();
  mesh->tag_topology_changed();
  return mesh;
}

static bool wrangle_array_contains(const bke::WrangleArrayValue &arr, const int value)
{
  for (int i = 0; i < int(arr.count); i++) {
    if (arr.d.i[i] == value) {
      return true;
    }
  }
  return false;
}

TEST(nodes_vex, pointneighbours_writes_pts_attr)
{
  ensure_vex_runtime();
  bke::GeometrySet geometry = bke::GeometrySet::from_mesh(make_triangle_mesh());
  const CompileOutput compiled = compile("i[]@pts = pointneighbours(0, i@index);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  bool writes_pts = false;
  for (const AttrInfo &a : compiled.program->attrs) {
    if (a.name == "pts" && a.type == Type::IntArray && a.write) {
      writes_pts = true;
    }
  }
  EXPECT_TRUE(writes_pts);

  ensure_vex_runtime();
  fn::Field<bool> selection(true);
  const ExecOutput out = execute(
      *compiled.program, geometry, {}, Domain::Point, selection);
  ASSERT_TRUE(out.ok) << out.error;

  const Mesh *mesh = geometry.get_mesh();
  ASSERT_NE(mesh, nullptr);
  const bke::AttributeAccessor attributes = mesh->attributes();
  const bke::AttributeReader<bke::WrangleArrayValue> pts = attributes.lookup<bke::WrangleArrayValue>(
      "pts", bke::AttrDomain::Point);
  ASSERT_TRUE(pts);
  ASSERT_EQ(pts.varray.size(), 3);
  for (int v = 0; v < 3; v++) {
    const bke::WrangleArrayValue &arr = pts.varray[v];
    EXPECT_EQ(int(arr.kind), int(bke::WrangleArrayKind::Int)) << "vert " << v;
    EXPECT_EQ(int(arr.count), 2) << "vert " << v << " " << arr.to_string();
    EXPECT_TRUE(wrangle_array_contains(arr, (v + 1) % 3)) << arr.to_string();
    EXPECT_TRUE(wrangle_array_contains(arr, (v + 2) % 3)) << arr.to_string();
  }
}

TEST(nodes_vex, raycastall_hits_both_layers)
{
  ensure_vex_runtime();
  bke::GeometrySet geometry = bke::GeometrySet::from_mesh(make_two_layer_tris_mesh());
  const CompileOutput compiled = compile(
      "int hit[]; vector hp[], hn[]; float hd[];\n"
      "ray hits[] = raycastall(0, {0.2, 0.2, 1}, {0, 0, -1}, 10, hit, hp, hn, hd);\n"
      "i@n = len(hits);\n"
      "i@h0 = rayishit(hits[0]);\n"
      "i@h1 = rayishit(hits[1]);\n"
      "v@p0 = rayhitpos(hits[0]);\n"
      "v@p1 = rayhitpos(hits[1]);\n"
      "f@d0 = rayhitdist(hits[0]);\n"
      "f@d1 = rayhitdist(hits[1]);\n"
      "int n2 = 0;\n"
      "foreach (ray r; hits) {\n"
      "  if (rayishit(r)) { n2 += 1; }\n"
      "}\n"
      "i@n2 = n2;\n"
      "i[]@hit = hit;\n"
      "v[]@hp = hp;\n"
      "f[]@hd = hd;\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  fn::Field<bool> selection(true);
  const ExecOutput out = execute(*compiled.program, geometry, {}, Domain::Point, selection);
  ASSERT_TRUE(out.ok) << out.error;
  const Mesh *mesh = geometry.get_mesh();
  ASSERT_NE(mesh, nullptr);
  const bke::AttributeAccessor attributes = mesh->attributes();
  const bke::AttributeReader<int> n = attributes.lookup<int>("n", bke::AttrDomain::Point);
  ASSERT_TRUE(n);
  EXPECT_EQ(n.varray[0], 2);
  const bke::AttributeReader<int> n2 = attributes.lookup<int>("n2", bke::AttrDomain::Point);
  ASSERT_TRUE(n2);
  EXPECT_EQ(n2.varray[0], 2);
  const bke::AttributeReader<int> h0 = attributes.lookup<int>("h0", bke::AttrDomain::Point);
  const bke::AttributeReader<int> h1 = attributes.lookup<int>("h1", bke::AttrDomain::Point);
  ASSERT_TRUE(h0);
  ASSERT_TRUE(h1);
  EXPECT_EQ(h0.varray[0], 1);
  EXPECT_EQ(h1.varray[0], 1);
  const bke::AttributeReader<float3> p0 = attributes.lookup<float3>("p0", bke::AttrDomain::Point);
  const bke::AttributeReader<float3> p1 = attributes.lookup<float3>("p1", bke::AttrDomain::Point);
  ASSERT_TRUE(p0);
  ASSERT_TRUE(p1);
  EXPECT_NEAR(p0.varray[0].z, 0.0f, 1e-4f);
  EXPECT_NEAR(p1.varray[0].z, -1.0f, 1e-4f);
  const bke::AttributeReader<float> d0 = attributes.lookup<float>("d0", bke::AttrDomain::Point);
  const bke::AttributeReader<float> d1 = attributes.lookup<float>("d1", bke::AttrDomain::Point);
  ASSERT_TRUE(d0);
  ASSERT_TRUE(d1);
  EXPECT_NEAR(d0.varray[0], 1.0f, 1e-4f);
  EXPECT_NEAR(d1.varray[0], 2.0f, 1e-4f);
  const bke::AttributeReader<bke::WrangleArrayValue> hit =
      attributes.lookup<bke::WrangleArrayValue>("hit", bke::AttrDomain::Point);
  ASSERT_TRUE(hit);
  EXPECT_EQ(int(hit.varray[0].count), 2);
  EXPECT_EQ(hit.varray[0].d.i[0], 1);
  EXPECT_EQ(hit.varray[0].d.i[1], 1);
  const bke::AttributeReader<bke::WrangleArrayValue> hp =
      attributes.lookup<bke::WrangleArrayValue>("hp", bke::AttrDomain::Point);
  ASSERT_TRUE(hp);
  EXPECT_EQ(int(hp.varray[0].kind), int(bke::WrangleArrayKind::Float3));
  EXPECT_EQ(int(hp.varray[0].count), 2);
  EXPECT_NEAR(hp.varray[0].d.f[2], 0.0f, 1e-4f);
  EXPECT_NEAR(hp.varray[0].d.f[5], -1.0f, 1e-4f);
  const bke::AttributeReader<bke::WrangleArrayValue> hd =
      attributes.lookup<bke::WrangleArrayValue>("hd", bke::AttrDomain::Point);
  ASSERT_TRUE(hd);
  EXPECT_EQ(int(hd.varray[0].kind), int(bke::WrangleArrayKind::Float));
  EXPECT_EQ(int(hd.varray[0].count), 2);
  EXPECT_NEAR(hd.varray[0].d.f[0], 1.0f, 1e-4f);
  EXPECT_NEAR(hd.varray[0].d.f[1], 2.0f, 1e-4f);
}

TEST(nodes_vex, wrangle_only_matrix2_matrix3_attrs)
{
  ensure_vex_runtime();
  bke::GeometrySet geometry = bke::GeometrySet::from_mesh(make_triangle_mesh());
  const CompileOutput compiled = compile(
      "2@m2 = 2;\n"
      "3@m3 = 3;\n"
      "4@m4 = 2;\n"
      "u@uv = {1, 2};\n"
      "q@t = {0, 1, 0, 1};\n"
      "r@orient = {0, 0, 0, 1};\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  fn::Field<bool> selection(true);
  const ExecOutput out = execute(*compiled.program, geometry, {}, Domain::Point, selection);
  ASSERT_TRUE(out.ok) << out.error;
  const Mesh *mesh = geometry.get_mesh();
  ASSERT_NE(mesh, nullptr);
  const bke::AttributeAccessor attributes = mesh->attributes();
  const bke::AttributeReader<bke::WrangleArrayValue> m2 =
      attributes.lookup<bke::WrangleArrayValue>("m2");
  ASSERT_TRUE(bool(m2));
  EXPECT_EQ(int(m2.varray[0].kind), int(bke::WrangleArrayKind::Matrix2));
  EXPECT_NEAR(m2.varray[0].as_matrix2()[0][0], 2.0f, 1e-5f);
  const bke::AttributeReader<bke::WrangleArrayValue> m3 =
      attributes.lookup<bke::WrangleArrayValue>("m3");
  ASSERT_TRUE(bool(m3));
  EXPECT_EQ(int(m3.varray[0].kind), int(bke::WrangleArrayKind::Matrix3));
  const bke::AttributeReader<float4x4> m4 = attributes.lookup<float4x4>("m4");
  ASSERT_TRUE(bool(m4));
  EXPECT_NEAR(m4.varray[0][0][0], 2.0f, 1e-5f);
  const bke::AttributeReader<float2> uv = attributes.lookup<float2>("uv");
  ASSERT_TRUE(bool(uv));
  EXPECT_NEAR(uv.varray[0].x, 1.0f, 1e-5f);
  const bke::AttributeReader<float4> t = attributes.lookup<float4>("t");
  ASSERT_TRUE(bool(t));
  EXPECT_NEAR(t.varray[0].w, 1.0f, 1e-5f);
  const bke::AttributeReader<math::Quaternion> orient =
      attributes.lookup<math::Quaternion>("orient");
  ASSERT_TRUE(bool(orient));
}

TEST(nodes_vex, mixed_uniform_matrix_list_with_varying_float)
{
  ensure_vex_runtime();
  bke::GeometrySet geometry = bke::GeometrySet::from_mesh(make_triangle_mesh());
  const CompileOutput compiled = compile(
      "m@m = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};\n"
      "f[]@fa = {1, 1.2, 1.3, 1.5, 1.7, f@f, f@f, f@f};\n"
      "m[]@ma = {m@m, m@m, m@m, m@m};\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_FALSE(program_can_array_exec(*compiled.program));
  fn::Field<bool> selection(true);
  const ExecOutput out = execute(*compiled.program, geometry, {}, Domain::Point, selection);
  ASSERT_TRUE(out.ok) << out.error;
  const Mesh *mesh = geometry.get_mesh();
  ASSERT_NE(mesh, nullptr);
  const bke::AttributeAccessor attributes = mesh->attributes();
  const bke::AttributeReader<float4x4> m = attributes.lookup<float4x4>("m");
  ASSERT_TRUE(bool(m));
  EXPECT_NEAR(m.varray[0][0][0], 1.0f, 1e-5f);
  const bke::AttributeReader<bke::WrangleArrayValue> ma =
      attributes.lookup<bke::WrangleArrayValue>("ma");
  ASSERT_TRUE(bool(ma));
  EXPECT_EQ(int(ma.varray[0].kind), int(bke::WrangleArrayKind::Matrix));
  EXPECT_EQ(int(ma.varray[0].count), 4);
  const bke::AttributeReader<bke::WrangleArrayValue> fa =
      attributes.lookup<bke::WrangleArrayValue>("fa");
  ASSERT_TRUE(bool(fa));
  EXPECT_EQ(int(fa.varray[0].kind), int(bke::WrangleArrayKind::Float));
  EXPECT_EQ(int(fa.varray[0].count), 8);
}

static Mesh *make_two_quad_mesh()
{
  Mesh *mesh = BKE_mesh_new_nomain(6, 7, 2, 8);
  MutableSpan<float3> pos = mesh->vert_positions_for_write();
  pos[0] = float3(0.0f, 0.0f, 0.0f);
  pos[1] = float3(1.0f, 0.0f, 0.0f);
  pos[2] = float3(2.0f, 0.0f, 0.0f);
  pos[3] = float3(0.0f, 1.0f, 0.0f);
  pos[4] = float3(1.0f, 1.0f, 0.0f);
  pos[5] = float3(2.0f, 1.0f, 0.0f);
  MutableSpan<int2> edges = mesh->edges_for_write();
  edges[0] = int2(0, 1);
  edges[1] = int2(1, 2);
  edges[2] = int2(0, 3);
  edges[3] = int2(1, 4);
  edges[4] = int2(2, 5);
  edges[5] = int2(3, 4);
  edges[6] = int2(4, 5);
  MutableSpan<int> offsets = mesh->face_offsets_for_write();
  offsets[0] = 0;
  offsets[1] = 4;
  offsets[2] = 8;
  MutableSpan<int> corners = mesh->corner_verts_for_write();
  corners[0] = 0;
  corners[1] = 1;
  corners[2] = 4;
  corners[3] = 3;
  corners[4] = 1;
  corners[5] = 2;
  corners[6] = 5;
  corners[7] = 4;
  MutableSpan<int> corner_edges = mesh->corner_edges_for_write();
  corner_edges[0] = 0;
  corner_edges[1] = 3;
  corner_edges[2] = 5;
  corner_edges[3] = 2;
  corner_edges[4] = 1;
  corner_edges[5] = 4;
  corner_edges[6] = 6;
  corner_edges[7] = 3;
  mesh->tag_positions_changed();
  mesh->tag_topology_changed();
  return mesh;
}

static ExecOutput wrangle_mesh(bke::GeometrySet &geometry, const StringRef source, const Domain domain)
{
  ensure_vex_runtime();
  const CompileOutput compiled = compile(source);
  if (!compiled.program) {
    ExecOutput out;
    out.ok = false;
    out.error = compiled.error.empty() ? "compile failed" : compiled.error;
    return out;
  }
  fn::Field<bool> selection(true);
  return execute(*compiled.program, geometry, {}, domain, selection);
}

TEST(nodes_vex, delete_geometry_point_removes_incident_face)
{
  ensure_vex_runtime();
  bke::GeometrySet geometry = bke::GeometrySet::from_mesh(make_two_quad_mesh());
  const ExecOutput out = wrangle_mesh(
      geometry, "if (i@index == 0) { delete_geometry(\"point\", i@index, \"all\"); }\n", Domain::Point);
  ASSERT_TRUE(out.ok) << out.error;
  const Mesh *mesh = geometry.get_mesh();
  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->faces_num, 1);
  EXPECT_EQ(mesh->verts_num, 5);
}

TEST(nodes_vex, delete_geometry_face_from_shared_point_expands)
{
  /* Vert 4 is used by both quads. Treating i@index as a face index would be a no-op. */
  ensure_vex_runtime();
  bke::GeometrySet geometry = bke::GeometrySet::from_mesh(make_two_quad_mesh());
  const ExecOutput out = wrangle_mesh(
      geometry, "if (i@index == 4) { delete_geometry(\"face\", i@index, \"all\"); }\n", Domain::Point);
  ASSERT_TRUE(out.ok) << out.error;
  const Mesh *mesh = geometry.get_mesh();
  EXPECT_TRUE(mesh == nullptr || mesh->faces_num == 0);
}

TEST(nodes_vex, delete_geometry_face_of_boundary_point)
{
  ensure_vex_runtime();
  bke::GeometrySet geometry = bke::GeometrySet::from_mesh(make_two_quad_mesh());
  const ExecOutput out = wrangle_mesh(
      geometry, "if (i@index == 0) { delete_geometry(\"face\"); }\n", Domain::Point);
  ASSERT_TRUE(out.ok) << out.error;
  const Mesh *mesh = geometry.get_mesh();
  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->faces_num, 1);
  EXPECT_EQ(mesh->verts_num, 4);
}

TEST(nodes_vex, delete_geometry_run_over_face)
{
  ensure_vex_runtime();
  bke::GeometrySet geometry = bke::GeometrySet::from_mesh(make_two_quad_mesh());
  const ExecOutput out = wrangle_mesh(
      geometry, "if (i@index == 0) { delete_geometry(\"face\", i@index, \"all\"); }\n", Domain::Face);
  ASSERT_TRUE(out.ok) << out.error;
  const Mesh *mesh = geometry.get_mesh();
  ASSERT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->faces_num, 1);
  EXPECT_EQ(mesh->verts_num, 4);
}

TEST(nodes_vex, set_two_args_is_vector2)
{
  const PureEvalOutput out = eval_source("vector2 u = set(1, 2); return int(u.x + u.y);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 3);
}

TEST(nodes_vex, set_vector2_and_float_is_vector)
{
  const PureEvalOutput out = eval_source(
      "vector2 p = set(1, 2); vector r = set(p, 4); return int(r.x + r.y + r.z);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 7);
}

TEST(nodes_vex, shader_material_position_and_set_gpu)
{
  const CompileOutput compiled = compile_shader_material(
      "vector2 p = v@Position;\n"
      "float strength = chf(\"strength\");\n"
      "float radius = chf(\"radius\");\n"
      "float r = length(p);\n"
      "if (r < radius) {\n"
      "  float falloff = 1.0 - r / radius;\n"
      "  float angle = strength * falloff;\n"
      "  float c = cos(angle);\n"
      "  float s = sin(angle);\n"
      "  p = set(p.x * c - p.y * s, p.x * s + p.y * c);\n"
      "  Output = set(p, 0);\n"
      "}\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.error.empty()) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.error;
  EXPECT_NE(compiled.program->gpu_src.find("a_position"), std::string::npos);
  EXPECT_EQ(compiled.program->gpu_src.find("float(l_"), std::string::npos);
  EXPECT_NE(compiled.program->gpu_src.find("float3"), std::string::npos);
}

TEST(nodes_vex, shader_material_custom_attr_and_set_gpu)
{
  const CompileOutput compiled = compile_shader_material(
      "vector2 p = v@abc;\n"
      "float strength = chf(\"strength\");\n"
      "float radius = chf(\"radius\");\n"
      "float r = length(p);\n"
      "if (r < radius) {\n"
      "  float falloff = 1.0 - r / radius;\n"
      "  float angle = strength * falloff;\n"
      "  float c = cos(angle);\n"
      "  float s = sin(angle);\n"
      "  p = set(p.x * c - p.y * s, p.x * s + p.y * c);\n"
      "}\n"
      "Output = set(p, 0);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.error.empty()) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.error;
  EXPECT_NE(compiled.program->gpu_src.find("a_abc"), std::string::npos);
  EXPECT_EQ(compiled.program->gpu_src.find("float(l_"), std::string::npos);
}

TEST(nodes_vex, shader_material_set_const_gpu)
{
  const CompileOutput compiled = compile_shader_material("Output = set(0,0,1);");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.error.empty()) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.error;
  EXPECT_NE(compiled.program->gpu_src.find("a_Output"), std::string::npos);
  EXPECT_NE(compiled.program->gpu_src.find("float3"), std::string::npos);
  EXPECT_EQ(compiled.program->gpu_src.find("//"), std::string::npos);
  EXPECT_EQ(compiled.program->gpu_src.find("/*"), std::string::npos);
  EXPECT_EQ(compiled.program->gpu_src.find("g_data"), std::string::npos);
}

TEST(nodes_vex, shader_material_set_const_with_comments_gpu)
{
  const CompileOutput compiled = compile_shader_material(
      "// vector2 p = v@abc;\n"
      "Output = set(0,0,1);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.error.empty()) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.error;
  EXPECT_NE(compiled.program->gpu_src.find("a_Output"), std::string::npos);
  EXPECT_EQ(compiled.program->gpu_src.find("a_abc"), std::string::npos);
  EXPECT_EQ(compiled.program->gpu_src.find("g_data"), std::string::npos);
}

TEST(nodes_vex, shader_material_set_const_eval)
{
  const CompileOutput compiled = compile(
      "vector Output = {0,0,0};\nOutput = set(0,0,1);\nreturn Output;");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  const PureEvalOutput ev = execute_pure(*compiled.program);
  ASSERT_TRUE(ev.ok) << ev.error;
  EXPECT_NEAR(ev.return_vec.x, 0.0f, 1e-5f);
  EXPECT_NEAR(ev.return_vec.y, 0.0f, 1e-5f);
  EXPECT_NEAR(ev.return_vec.z, 1.0f, 1e-5f);
}

TEST(nodes_vex, shader_material_gpu_map_smooth)
{
  const CompileOutput compiled = compile_shader_material(
      "Output = {map(0.5, 0.0, 1.0, 0.0, 2.0), smooth(0.5), 0.0};");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.error;
  EXPECT_NE(compiled.program->gpu_src.find("smoothstep"), std::string::npos);
}

TEST(nodes_vex, shader_material_swizzle_gpu)
{
  const CompileOutput compiled = compile_shader_material(
      "vector p = {1, 2, 3};\n"
      "p = p.yzx;\n"
      "vector2 u = p.zx;\n"
      "Output = set(p.x, u.x, u.y);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.program->gpu_error;
  EXPECT_NE(compiled.program->gpu_src.find(".yzx"), std::string::npos) << compiled.program->gpu_src;
  EXPECT_NE(compiled.program->gpu_src.find(".zx"), std::string::npos) << compiled.program->gpu_src;
}

TEST(nodes_vex, vector2_ctor_xy_not_splat)
{
  const PureEvalOutput out = eval_source(
      "vector2 u = vector2(1.0, 0.0); return int(u.x * 10 + u.y);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_EQ(out.return_int, 10);
}

TEST(nodes_vex, vector_scalar_ctor_splats)
{
  const PureEvalOutput v2 = eval_source(
      "vector2 u = vector2(1.0); return int(u.x * 10 + u.y);");
  ASSERT_TRUE(v2.ok) << v2.error;
  EXPECT_EQ(v2.return_int, 11);
  const PureEvalOutput v3 = eval_source(
      "vector v = vector(1.0); return int(v.x + v.y + v.z);");
  ASSERT_TRUE(v3.ok) << v3.error;
  EXPECT_EQ(v3.return_int, 3);
  const PureEvalOutput v4 = eval_source(
      "vector4 q = vector4(1.0); return int(q.x + q.y + q.z + q.w);");
  ASSERT_TRUE(v4.ok) << v4.error;
  EXPECT_EQ(v4.return_int, 4);

  const CompileOutput compiled = compile_shader_material(
      "vector4 q = vector4(1.0);\n"
      "Noise = q.x + q.y + q.z + q.w;\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.program->gpu_error;
  EXPECT_NE(compiled.program->gpu_src.find("1.0, 1.0, 1.0, 1.0"), std::string::npos)
      << compiled.program->gpu_src;
  EXPECT_EQ(compiled.program->gpu_src.find("1.0, 0.0, 0.0, 0.0"), std::string::npos)
      << compiled.program->gpu_src;
}

TEST(nodes_vex, vector_ctor_pack_and_aliases)
{
  const PureEvalOutput pack34 = eval_source(
      "vector4 q = vec4(vector(1, 2, 3), 4); return int(q.x * 1000 + q.y * 100 + q.z * 10 + q.w);");
  ASSERT_TRUE(pack34.ok) << pack34.error;
  EXPECT_EQ(pack34.return_int, 1234);

  const PureEvalOutput pack22 = eval_source(
      "vector4 q = vec4(vec2(1, 2), vec2(3, 4)); "
      "return int(q.x * 1000 + q.y * 100 + q.z * 10 + q.w);");
  ASSERT_TRUE(pack22.ok) << pack22.error;
  EXPECT_EQ(pack22.return_int, 1234);

  const PureEvalOutput pad = eval_source(
      "vector4 q = vec4(vec2(1, 2)); return int(q.x * 1000 + q.y * 100 + q.z * 10 + q.w);");
  ASSERT_TRUE(pad.ok) << pad.error;
  EXPECT_EQ(pad.return_int, 1200);

  const PureEvalOutput splat = eval_source(
      "vec4 q = vec4(2); return int(q.x + q.y + q.z + q.w);");
  ASSERT_TRUE(splat.ok) << splat.error;
  EXPECT_EQ(splat.return_int, 8);

  const PureEvalOutput paren = eval_source(
      "vector4 q = (vector(1, 2, 3), 4); return int(q.x * 1000 + q.y * 100 + q.z * 10 + q.w);");
  ASSERT_TRUE(paren.ok) << paren.error;
  EXPECT_EQ(paren.return_int, 1234);

  const PureEvalOutput v2alias = eval_source("vec2 u = vec2(9); return int(u.x * 10 + u.y);");
  ASSERT_TRUE(v2alias.ok) << v2alias.error;
  EXPECT_EQ(v2alias.return_int, 99);

  const PureEvalOutput matalias = eval_source(
      "mat2 m = mat2(vec2(1, 0), vec2(0, 1)); return 1;");
  ASSERT_TRUE(matalias.ok) << matalias.error;
  EXPECT_EQ(matalias.return_int, 1);

  const CompileOutput gpu = compile_shader_material(
      "vector4 q = vec4(vector(1.0, 2.0, 3.0), 4.0);\n"
      "Noise = q.x + q.y + q.z + q.w;\n");
  ASSERT_TRUE(bool(gpu.program)) << gpu.error;
  EXPECT_TRUE(gpu.program->gpu_ok) << gpu.program->gpu_error;
  EXPECT_NE(gpu.program->gpu_src.find("4.0"), std::string::npos) << gpu.program->gpu_src;
}

TEST(nodes_vex, shader_material_user_fn_fbm_gpu)
{
  const char *src =
      "float hash21(vector2 p)\n"
      "{\n"
      "    p = fract(p * vector2(123.34, 345.45));\n"
      "    p += dot(p, p + 34.345);\n"
      "    return fract(p.x * p.y);\n"
      "}\n"
      "float value_noise(vector2 p)\n"
      "{\n"
      "    vector2 i = floor(p);\n"
      "    vector2 f = fract(p);\n"
      "    f = f * f * (3.0 - 2.0 * f);\n"
      "    float a = hash21(i);\n"
      "    float b = hash21(i + vector2(1.0, 0.0));\n"
      "    float c = hash21(i + vector2(0.0, 1.0));\n"
      "    float d = hash21(i + vector2(1.0, 1.0));\n"
      "    float x1 = mix(a, b, f.x);\n"
      "    float x2 = mix(c, d, f.x);\n"
      "    return mix(x1, x2, f.y);\n"
      "}\n"
      "float fbm(vector2 p)\n"
      "{\n"
      "    float value = 0.0;\n"
      "    float amplitude = 0.5;\n"
      "    float frequency = 1.0;\n"
      "    for (int i = 0; i < 6; i++)\n"
      "    {\n"
      "        value += amplitude * value_noise(p * frequency);\n"
      "        frequency *= 2.0;\n"
      "        amplitude *= 0.5;\n"
      "    }\n"
      "    return value;\n"
      "}\n"
      "vector2 uv = set(@P.x, @P.y);\n"
      "float n = fbm(uv * chf(\"scale\"));\n"
      "Noise = set(n, n, n);\n";
  const CompileOutput compiled = compile_shader_material(src);
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.error.empty()) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.program->gpu_error;
  EXPECT_FALSE(compiled.program->gpu_helpers.empty());
  EXPECT_NE(compiled.program->gpu_helpers.find("hash21"), std::string::npos);
  EXPECT_NE(compiled.program->gpu_helpers.find("value_noise"), std::string::npos);
  EXPECT_NE(compiled.program->gpu_helpers.find("fbm"), std::string::npos);
  EXPECT_NE(compiled.program->gpu_src.find("ch_scale"), std::string::npos);
  EXPECT_EQ(compiled.program->gpu_src.find("//"), std::string::npos);
  EXPECT_EQ(compiled.program->gpu_helpers.find("//"), std::string::npos);
  EXPECT_NE(compiled.program->gpu_helpers.find("1.0, 0.0"), std::string::npos)
      << compiled.program->gpu_helpers;
  EXPECT_NE(compiled.program->gpu_helpers.find("0.0, 1.0"), std::string::npos)
      << compiled.program->gpu_helpers;
  EXPECT_EQ(compiled.program->gpu_helpers.find("mix("), std::string::npos)
      << compiled.program->gpu_helpers;

  const PureEvalOutput corners = eval_source(
      "float hash21(vector2 p)\n"
      "{\n"
      "    p = fract(p * vector2(123.34, 345.45));\n"
      "    p += dot(p, p + 34.345);\n"
      "    return fract(p.x * p.y);\n"
      "}\n"
      "return hash21(vector2(1.0, 0.0)) - "
      "hash21(vector2(0.0, 0.0) + vector2(1.0, 0.0));");
  ASSERT_TRUE(corners.ok) << corners.error;
  EXPECT_NEAR(corners.return_float, 0.0f, 1e-6f);
}

TEST(nodes_vex, lerp_b_x_plus_one_minus_x_times_b_is_b)
{
  /* `b*x + (1-x)*b` is identically `b`; it is not lerp(a, b, x). */
  const PureEvalOutput out = eval_source(
      "float a = 0.0; float b = 7.0; float x = 0.25; return b * x + (1.0 - x) * b;");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_NEAR(out.return_float, 7.0f, 1e-6f);
}

TEST(nodes_vex, lerp_manual_and_mix_match)
{
  const PureEvalOutput manual = eval_source(
      "float a = 2.0; float b = 8.0; float x = 0.25; return a * (1.0 - x) + b * x;");
  ASSERT_TRUE(manual.ok) << manual.error;
  EXPECT_NEAR(manual.return_float, 3.5f, 1e-6f);
  const PureEvalOutput builtin = eval_source("return mix(2.0, 8.0, 0.25);");
  ASSERT_TRUE(builtin.ok) << builtin.error;
  EXPECT_NEAR(builtin.return_float, 3.5f, 1e-6f);
}

TEST(nodes_vex, value_noise_cell_edge_continuous)
{
  const char *src =
      "float hash21(vector2 p)\n"
      "{\n"
      "    p = fract(p * vector2(123.34, 345.45));\n"
      "    p += dot(p, p + 34.345);\n"
      "    return fract(p.x * p.y);\n"
      "}\n"
      "float value_noise(vector2 p)\n"
      "{\n"
      "    vector2 i = floor(p);\n"
      "    vector2 f = fract(p);\n"
      "    f = f * f * (3.0 - 2.0 * f);\n"
      "    float a = hash21(i);\n"
      "    float b = hash21(i + vector2(1.0, 0.0));\n"
      "    float c = hash21(i + vector2(0.0, 1.0));\n"
      "    float d = hash21(i + vector2(1.0, 1.0));\n"
      "    float x1 = mix(a, b, f.x);\n"
      "    float x2 = mix(c, d, f.x);\n"
      "    return mix(x1, x2, f.y);\n"
      "}\n"
      "return value_noise(vector2(0.999, 0.4)) - value_noise(vector2(1.0, 0.4));";
  const PureEvalOutput out = eval_source(src);
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_NEAR(out.return_float, 0.0f, 2e-3f);
}

TEST(nodes_vex, ddx_cpu_is_zero)
{
  const PureEvalOutput out = eval_source(
      "return ddx(3.0) + ddy(set(1.0, 2.0, 3.0)).x + fwidth(0.5);");
  ASSERT_TRUE(out.ok) << out.error;
  EXPECT_NEAR(out.return_float, 0.0f, 1e-6f);
}

TEST(nodes_vex, shader_material_ddx_gpu)
{
  const CompileOutput compiled = compile_shader_material(
      "vector dx = ddx(v@P);\n"
      "vector dy = ddy(v@P);\n"
      "float w = fwidth(@P.x);\n"
      "Output = normalize(cross(dx, dy)) * w;\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.program->gpu_error;
  EXPECT_NE(compiled.program->gpu_src.find("wr_ddx"), std::string::npos)
      << compiled.program->gpu_src;
  EXPECT_NE(compiled.program->gpu_src.find("wr_ddy"), std::string::npos)
      << compiled.program->gpu_src;
  EXPECT_NE(compiled.program->gpu_src.find("wr_fwidth"), std::string::npos)
      << compiled.program->gpu_src;
  EXPECT_NE(compiled.program->gpu_helpers.find("gpu_dfdx"), std::string::npos)
      << compiled.program->gpu_helpers;
  EXPECT_NE(compiled.program->gpu_helpers.find("gpu_dfdy"), std::string::npos)
      << compiled.program->gpu_helpers;
}

TEST(nodes_vex, ddx_geometry_gpu_is_zero)
{
  const CompileOutput compiled = compile("f@d = ddx(v@P.x);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.program->gpu_error;
  EXPECT_EQ(compiled.program->gpu_src.find("gpu_dfdx"), std::string::npos)
      << compiled.program->gpu_src;
  EXPECT_EQ(compiled.program->gpu_src.find("wr_ddx"), std::string::npos)
      << compiled.program->gpu_src;
}

TEST(nodes_vex, shader_material_manual_lerp_gpu)
{
  const CompileOutput compiled = compile_shader_material(
      "float lerp2(float a, float b, float x) { return a * (1.0 - x) + b * x; }\n"
      "float n = lerp2(0.0, 1.0, 0.25);\n"
      "Output = set(n, n, n);\n");
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.program->gpu_error;
  EXPECT_NE(compiled.program->gpu_helpers.find("l_a"), std::string::npos)
      << compiled.program->gpu_helpers;
  EXPECT_NE(compiled.program->gpu_helpers.find("l_b"), std::string::npos)
      << compiled.program->gpu_helpers;
  EXPECT_NE(compiled.program->gpu_helpers.find("l_x"), std::string::npos)
      << compiled.program->gpu_helpers;
}

TEST(nodes_vex, shader_wrangle_1d_lerp_compiles)
{
  /* Same pattern as shader wrangle.blend Wrangle.002: lerp only in X. */
  const char *src =
      "float hash21(vector2 p)\n"
      "{\n"
      "    p = fract(p * vector2(123.34, 345.45));\n"
      "    p += dot(p, p + 34.345);\n"
      "    return fract(p.x * p.y);\n"
      "}\n"
      "vector2 uv = set(@P.x, @P.y);\n"
      "p = uv * chf(\"scale\");\n"
      "vector2 i = floor(p);\n"
      "vector2 f = fract(p);\n"
      "f = f * f * (3.0 - 2.0 * f);\n"
      "float a = hash21(i);\n"
      "float b = hash21(i + vector2(1.0, 0.0));\n"
      "float c = hash21(i + vector2(0.0, 1.0));\n"
      "float d = hash21(i + vector2(1.0, 1.0));\n"
      "Noise = f.x * b + (1.0 - f.x) * a;\n";
  const CompileOutput compiled = compile_shader_material(src);
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.program->gpu_error;
  EXPECT_NE(compiled.program->gpu_src.find("l_a"), std::string::npos) << compiled.program->gpu_src;
  EXPECT_NE(compiled.program->gpu_src.find("l_b"), std::string::npos) << compiled.program->gpu_src;
  EXPECT_NE(compiled.program->gpu_src.find("1.0, 0.0"), std::string::npos)
      << compiled.program->gpu_src;
  EXPECT_EQ(compiled.program->gpu_src.find("float3((("), std::string::npos)
      << compiled.program->gpu_src;
  EXPECT_NE(compiled.program->gpu_helpers.find("arg_p"), std::string::npos)
      << compiled.program->gpu_helpers;
  EXPECT_NE(compiled.program->gpu_helpers.find("l_p = arg_p"), std::string::npos)
      << compiled.program->gpu_helpers;
}

TEST(nodes_vex, shader_material_bilinear_mix_gpu_src)
{
  /* Same pattern as shader wrangle.blend Wrangle.002 with Noise = y1. */
  const char *src =
      "float hash21(vector2 p)\n"
      "{\n"
      "    vector2 newp = fract(p * vector2(123.34, 345.45));\n"
      "    newp += dot(newp, newp + 34.345);\n"
      "    return fract(newp.x * newp.y);\n"
      "}\n"
      "vector2 uv = set(@P.x, @P.y);\n"
      "vector2 p = uv * chf(\"scale\");\n"
      "vector2 f = fract(p);\n"
      "vector2 i = floor(p);\n"
      "f = f * f * (3.0 - 2.0 * f);\n"
      "float a = hash21(i);\n"
      "float b = hash21(i + vector2(1.0, 0.0));\n"
      "float c = hash21(i + vector2(0.0, 1.0));\n"
      "float d = hash21(i + vector2(1.0, 1.0));\n"
      "float x1 = mix(a, b, f.x);\n"
      "float x2 = mix(c, d, f.x);\n"
      "float y1 = mix(x1, x2, f.y);\n"
      "Noise = y1;\n";
  const CompileOutput compiled = compile_shader_material(src);
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.program->gpu_error;
  const std::string &body = compiled.program->gpu_src;
  const std::string &helpers = compiled.program->gpu_helpers;
  EXPECT_EQ(body.find("mix("), std::string::npos) << body;
  EXPECT_EQ(helpers.find("mix("), std::string::npos) << helpers;
  EXPECT_NE(body.find("wr_interp("), std::string::npos) << body;
  EXPECT_NE(helpers.find("wr_interp("), std::string::npos) << helpers;
  EXPECT_NE(helpers.find("wr_floor2("), std::string::npos) << helpers;
  EXPECT_NE(helpers.find("int2(floor("), std::string::npos) << helpers;
  EXPECT_NE(helpers.find("wr_fract2("), std::string::npos) << helpers;
  EXPECT_NE(body.find("wr_floor2("), std::string::npos) << body;
  EXPECT_NE(body.find("wr_fract2("), std::string::npos) << body;
  EXPECT_NE(body.find("float2(("), std::string::npos) << body;
  EXPECT_NE(body.find("wr_lat2("), std::string::npos) << body;
  EXPECT_NE(helpers.find("wr_lat2("), std::string::npos) << helpers;
  EXPECT_NE(helpers.find("wr_mul("), std::string::npos) << helpers;
  EXPECT_EQ(helpers.find("wr_precise"), std::string::npos) << helpers;
  EXPECT_EQ(helpers.find("precise float"), std::string::npos) << helpers;
  EXPECT_NE(body.find("int2("), std::string::npos) << body;
  EXPECT_NE(body.find("1.0, 0.0"), std::string::npos) << body;
  EXPECT_NE(body.find("0.0, 1.0"), std::string::npos) << body;
  EXPECT_NE(body.find("1.0, 1.0"), std::string::npos) << body;
  EXPECT_NE(body.find("(l_f).x"), std::string::npos) << body;
  EXPECT_NE(body.find("(l_f).y"), std::string::npos) << body;
  EXPECT_NE(body.find("l_x1"), std::string::npos) << body;
  EXPECT_NE(body.find("l_x2"), std::string::npos) << body;
  EXPECT_NE(body.find("l_y1"), std::string::npos) << body;
  EXPECT_EQ(body.find("float3"), std::string::npos) << body;
  EXPECT_EQ(body.find("1.0 -"), std::string::npos) << body;
  EXPECT_EQ(helpers.find("#undef mix"), 0) << helpers;
  auto count_sub = [](const std::string &s, const char *sub) {
    int n = 0;
    for (size_t p = 0; (p = s.find(sub, p)) != std::string::npos; p += 1) {
      n++;
    }
    return n;
  };
  /* One use of each fade component per mix, not `(1-t)*a + t*b` which mentions t twice. */
  EXPECT_EQ(count_sub(body, "(l_f).x"), 2) << body;
  EXPECT_EQ(count_sub(body, "(l_f).y"), 1) << body;
}

TEST(nodes_vex, shader_simplex_cell_offset_uses_int_add)
{
  /* 4D simplex ranks build 0/1 offsets with `?:`; those must stay lattice ints. */
  const char *src =
      "float hash4(vector4 p) { return fract(p.x * p.y); }\n"
      "vector4 p = {@P.x, @P.y, @P.z, 0.0};\n"
      "vector4 cell = floor(p);\n"
      "int rankX = 3;\n"
      "vector4 i1 = vector4(rankX >= 3 ? 1.0 : 0.0, 0.0, 0.0, 0.0);\n"
      "Noise = hash4(cell + i1);\n";
  const CompileOutput compiled = compile_shader_material(src);
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.program->gpu_error;
  const std::string &body = compiled.program->gpu_src;
  const std::string &helpers = compiled.program->gpu_helpers;
  EXPECT_NE(helpers.find("wr_lat4("), std::string::npos) << helpers;
  EXPECT_NE(body.find("wr_lat4("), std::string::npos) << body;
  EXPECT_NE(body.find("int4("), std::string::npos) << body;
  EXPECT_EQ(helpers.find("wr_precise"), std::string::npos) << helpers;
}

TEST(nodes_vex, shader_chf_single_quote_lerp_gpu)
{
  const char *src =
      "float hash21(vector2 p)\n"
      "{\n"
      "    vector2 newp = fract(p * vector2(123.34, 345.45));\n"
      "    newp += dot(newp, newp + 34.345);\n"
      "    return fract(newp.x * newp.y);\n"
      "}\n"
      "vector2 uv = set(@P.x, @P.y);\n"
      "vector2 p = uv * chf(\"scale\");\n"
      "vector2 i = floor(p);\n"
      "float a = hash21(i);\n"
      "float b = hash21(i + vector2(1.0, 0.0));\n"
      "Noise = a+((b-a)*chf('lerp'));\n";
  const CompileOutput compiled = compile_shader_material(src);
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.error.empty()) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.program->gpu_error;
  bool found_lerp = false;
  bool found_scale = false;
  for (const Program::GpuCh &ch : compiled.program->gpu_ch) {
    if (ch.name == "lerp") {
      found_lerp = true;
      EXPECT_EQ(int(ch.type), int(Type::Float));
    }
    if (ch.name == "scale") {
      found_scale = true;
    }
  }
  EXPECT_TRUE(found_lerp) << "gpu_ch missing lerp\n" << compiled.program->gpu_src;
  EXPECT_TRUE(found_scale) << "gpu_ch missing scale";
  EXPECT_NE(compiled.program->gpu_src.find("ch_p_lerp"), std::string::npos)
      << compiled.program->gpu_src;
  EXPECT_EQ(compiled.program->gpu_src.find("ch_lerp"), std::string::npos)
      << compiled.program->gpu_src;
  EXPECT_NE(compiled.program->gpu_src.find("ch_scale"), std::string::npos)
      << compiled.program->gpu_src;
  EXPECT_EQ(compiled.program->gpu_src.find("*(0.0)"), std::string::npos)
      << compiled.program->gpu_src;
  const CompileOutput dq = compile_shader_material(
      "float a = 2.0;\nfloat b = 8.0;\nNoise = a+((b-a)*chf(\"lerp\"));\n");
  const CompileOutput sq = compile_shader_material(
      "float a = 2.0;\nfloat b = 8.0;\nNoise = a+((b-a)*chf('lerp'));\n");
  ASSERT_TRUE(bool(dq.program)) << dq.error;
  ASSERT_TRUE(bool(sq.program)) << sq.error;
  EXPECT_EQ(dq.program->gpu_src, sq.program->gpu_src) << sq.program->gpu_src;
}

TEST(nodes_vex, shader_perlin4_fbm_gpu_ok)
{
  const char *src =
      "float hash4(vector4 p)\n"
      "{\n"
      "    p = fract(p * 0.1031);\n"
      "    p += dot(p, p.wzxy + 33.33);\n"
      "    return fract((p.x + p.y) * (p.z + p.w));\n"
      "}\n"
      "vector4 gradient4(vector4 p)\n"
      "{\n"
      "    float x = hash4(p + vector4(17.13, 31.71, 47.23, 59.41));\n"
      "    float y = hash4(p + vector4(73.19, 83.37, 97.11, 101.73));\n"
      "    float z = hash4(p + vector4(113.17, 127.31, 139.27, 151.43));\n"
      "    float w = hash4(p + vector4(163.11, 179.27, 191.43, 211.19));\n"
      "    vector4 g = vector4(x * 2.0 - 1.0, y * 2.0 - 1.0, z * 2.0 - 1.0, w * 2.0 - 1.0);\n"
      "    return normalize(g);\n"
      "}\n"
      "float fade(float t)\n"
      "{\n"
      "    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);\n"
      "}\n"
      "float perlinCorner4(vector4 x, vector4 gradient)\n"
      "{\n"
      "    return dot(gradient, x);\n"
      "}\n"
      "float lerpFloat(float a, float b, float t)\n"
      "{\n"
      "    return a + t * (b - a);\n"
      "}\n"
      "float perlin4(vector4 p)\n"
      "{\n"
      "    vector4 cell = floor(p);\n"
      "    vector4 f = fract(p);\n"
      "    float u = fade(f.x);\n"
      "    float v = fade(f.y);\n"
      "    float s = fade(f.z);\n"
      "    float t = fade(f.w);\n"
      "    vector4 p0000 = f - vector4(0.0, 0.0, 0.0, 0.0);\n"
      "    vector4 p1000 = f - vector4(1.0, 0.0, 0.0, 0.0);\n"
      "    vector4 p0100 = f - vector4(0.0, 1.0, 0.0, 0.0);\n"
      "    vector4 p1100 = f - vector4(1.0, 1.0, 0.0, 0.0);\n"
      "    vector4 p0010 = f - vector4(0.0, 0.0, 1.0, 0.0);\n"
      "    vector4 p1010 = f - vector4(1.0, 0.0, 1.0, 0.0);\n"
      "    vector4 p0110 = f - vector4(0.0, 1.0, 1.0, 0.0);\n"
      "    vector4 p1110 = f - vector4(1.0, 1.0, 1.0, 0.0);\n"
      "    vector4 p0001 = f - vector4(0.0, 0.0, 0.0, 1.0);\n"
      "    vector4 p1001 = f - vector4(1.0, 0.0, 0.0, 1.0);\n"
      "    vector4 p0101 = f - vector4(0.0, 1.0, 0.0, 1.0);\n"
      "    vector4 p1101 = f - vector4(1.0, 1.0, 0.0, 1.0);\n"
      "    vector4 p0011 = f - vector4(0.0, 0.0, 1.0, 1.0);\n"
      "    vector4 p1011 = f - vector4(1.0, 0.0, 1.0, 1.0);\n"
      "    vector4 p0111 = f - vector4(0.0, 1.0, 1.0, 1.0);\n"
      "    vector4 p1111 = f - vector4(1.0, 1.0, 1.0, 1.0);\n"
      "    vector4 g0000 = gradient4(cell + vector4(0.0, 0.0, 0.0, 0.0));\n"
      "    vector4 g1000 = gradient4(cell + vector4(1.0, 0.0, 0.0, 0.0));\n"
      "    vector4 g0100 = gradient4(cell + vector4(0.0, 1.0, 0.0, 0.0));\n"
      "    vector4 g1100 = gradient4(cell + vector4(1.0, 1.0, 0.0, 0.0));\n"
      "    vector4 g0010 = gradient4(cell + vector4(0.0, 0.0, 1.0, 0.0));\n"
      "    vector4 g1010 = gradient4(cell + vector4(1.0, 0.0, 1.0, 0.0));\n"
      "    vector4 g0110 = gradient4(cell + vector4(0.0, 1.0, 1.0, 0.0));\n"
      "    vector4 g1110 = gradient4(cell + vector4(1.0, 1.0, 1.0, 0.0));\n"
      "    vector4 g0001 = gradient4(cell + vector4(0.0, 0.0, 0.0, 1.0));\n"
      "    vector4 g1001 = gradient4(cell + vector4(1.0, 0.0, 0.0, 1.0));\n"
      "    vector4 g0101 = gradient4(cell + vector4(0.0, 1.0, 0.0, 1.0));\n"
      "    vector4 g1101 = gradient4(cell + vector4(1.0, 1.0, 0.0, 1.0));\n"
      "    vector4 g0011 = gradient4(cell + vector4(0.0, 0.0, 1.0, 1.0));\n"
      "    vector4 g1011 = gradient4(cell + vector4(1.0, 0.0, 1.0, 1.0));\n"
      "    vector4 g0111 = gradient4(cell + vector4(0.0, 1.0, 1.0, 1.0));\n"
      "    vector4 g1111 = gradient4(cell + vector4(1.0, 1.0, 1.0, 1.0));\n"
      "    float n0000 = perlinCorner4(p0000, g0000);\n"
      "    float n1000 = perlinCorner4(p1000, g1000);\n"
      "    float n0100 = perlinCorner4(p0100, g0100);\n"
      "    float n1100 = perlinCorner4(p1100, g1100);\n"
      "    float n0010 = perlinCorner4(p0010, g0010);\n"
      "    float n1010 = perlinCorner4(p1010, g1010);\n"
      "    float n0110 = perlinCorner4(p0110, g0110);\n"
      "    float n1110 = perlinCorner4(p1110, g1110);\n"
      "    float n0001 = perlinCorner4(p0001, g0001);\n"
      "    float n1001 = perlinCorner4(p1001, g1001);\n"
      "    float n0101 = perlinCorner4(p0101, g0101);\n"
      "    float n1101 = perlinCorner4(p1101, g1101);\n"
      "    float n0011 = perlinCorner4(p0011, g0011);\n"
      "    float n1011 = perlinCorner4(p1011, g1011);\n"
      "    float n0111 = perlinCorner4(p0111, g0111);\n"
      "    float n1111 = perlinCorner4(p1111, g1111);\n"
      "    float nx000 = lerpFloat(n0000, n1000, u);\n"
      "    float nx100 = lerpFloat(n0100, n1100, u);\n"
      "    float nx010 = lerpFloat(n0010, n1010, u);\n"
      "    float nx110 = lerpFloat(n0110, n1110, u);\n"
      "    float nx001 = lerpFloat(n0001, n1001, u);\n"
      "    float nx101 = lerpFloat(n0101, n1101, u);\n"
      "    float nx011 = lerpFloat(n0011, n1011, u);\n"
      "    float nx111 = lerpFloat(n0111, n1111, u);\n"
      "    float nxy00 = lerpFloat(nx000, nx100, v);\n"
      "    float nxy10 = lerpFloat(nx010, nx110, v);\n"
      "    float nxy01 = lerpFloat(nx001, nx101, v);\n"
      "    float nxy11 = lerpFloat(nx011, nx111, v);\n"
      "    float nxyz0 = lerpFloat(nxy00, nxy10, s);\n"
      "    float nxyz1 = lerpFloat(nxy01, nxy11, s);\n"
      "    return lerpFloat(nxyz0, nxyz1, t);\n"
      "}\n"
      "float fbm4(vector4 p, float scale, int detail, float roughness, float lacunarity)\n"
      "{\n"
      "    float value = 0.0;\n"
      "    float amplitude = 1.0;\n"
      "    float frequency = scale;\n"
      "    float amplitudeSum = 0.0;\n"
      "    for (int i = 0; i < detail; i++)\n"
      "    {\n"
      "        value += perlin4(p * frequency) * amplitude;\n"
      "        amplitudeSum += amplitude;\n"
      "        frequency *= lacunarity;\n"
      "        amplitude *= roughness;\n"
      "    }\n"
      "    if (amplitudeSum > 0.0)\n"
      "    {\n"
      "        value /= amplitudeSum;\n"
      "    }\n"
      "    return value;\n"
      "}\n"
      "float scale = 2.0;\n"
      "int detail = 5;\n"
      "float roughness = 0.5;\n"
      "float lacunarity = 2.0;\n"
      "vector4 p = vector4(v@a, chf('w'));\n"
      "float n = fbm4(p, scale, detail, roughness, lacunarity);\n"
      "n = n * 0.5 + 0.5;\n"
      "Noise = p.x;\n";

  const CompileOutput compiled = compile_shader_material(src);
  ASSERT_TRUE(bool(compiled.program)) << compiled.error;
  EXPECT_TRUE(compiled.error.empty()) << compiled.error;
  EXPECT_TRUE(compiled.program->gpu_ok) << compiled.program->gpu_error;
  EXPECT_FALSE(compiled.program->gpu_helpers.empty());
  EXPECT_EQ(compiled.program->gpu_helpers.find("mix("), std::string::npos)
      << compiled.program->gpu_helpers;
  EXPECT_EQ(compiled.program->gpu_src.find("mix("), std::string::npos) << compiled.program->gpu_src;
  EXPECT_EQ(compiled.program->gpu_helpers.find("//"), std::string::npos)
      << compiled.program->gpu_helpers;
  EXPECT_EQ(compiled.program->gpu_src.find("//"), std::string::npos) << compiled.program->gpu_src;
  EXPECT_NE(compiled.program->gpu_helpers.find("float4"), std::string::npos)
      << compiled.program->gpu_helpers;
  EXPECT_EQ(compiled.program->gpu_helpers.find("vec4"), std::string::npos)
      << compiled.program->gpu_helpers;
  EXPECT_NE(compiled.program->gpu_helpers.find("lerpFloat"), std::string::npos)
      << compiled.program->gpu_helpers;
  EXPECT_NE(compiled.program->gpu_src.find("ch_w"), std::string::npos) << compiled.program->gpu_src;
  /* `int detail` must stay an int argument. `wr_lat1` returns float and EEVEE
   * rejects implicit float→int at the fbm4 call (magenta material). */
  EXPECT_EQ(compiled.program->gpu_src.find("wr_lat1(l_detail)"), std::string::npos)
      << compiled.program->gpu_src;
  EXPECT_NE(compiled.program->gpu_src.find("l_detail"), std::string::npos)
      << compiled.program->gpu_src;

  FILE *hf = fopen("E:/tmp/perlin4_gpu_helpers.glsl", "wb");
  if (hf) {
    fwrite(compiled.program->gpu_helpers.data(), 1, compiled.program->gpu_helpers.size(), hf);
    fclose(hf);
  }
  FILE *bf = fopen("E:/tmp/perlin4_gpu_src.glsl", "wb");
  if (bf) {
    fwrite(compiled.program->gpu_src.data(), 1, compiled.program->gpu_src.size(), bf);
    fclose(bf);
  }
}

}  // namespace blender::nodes::vex::tests
