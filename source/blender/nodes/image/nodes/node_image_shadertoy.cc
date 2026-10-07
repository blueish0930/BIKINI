/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * GPU Texture Editor ShaderToy node.
 *
 * Paste Common / Buffer A–D / Image GLSL in the N-panel, Compile, ping-pong buffers,
 * output Color. iTime is Scene Time. #define scalars become N-panel sliders.
 */

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <initializer_list>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "MEM_guardedalloc.h"

#include "BLI_hash.hh"
#include "BLI_map.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_mutex.hh"
#include "BLI_set.hh"
#include "BLI_span.hh"
#include "BLI_string.hh"
#include "BLI_string_ref.hh"
#include "BLI_string_utf8.hh"
#include "BLI_task.hh"
#include "BLI_ustring.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"

#include "DNA_node_types.h"
#include "DNA_space_types.h"

#include "BKE_appdir.hh"
#include "BKE_context.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_report.hh"

#include "BLI_fileops.hh"
#include "BLI_path_utils.hh"

#include "BLO_read_write.hh"

#include "ED_node.hh"
#include "ED_screen.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "BLT_translation.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "GPU_batch.hh"
#include "GPU_framebuffer.hh"
#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"

#include "gpu_shader_create_info.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_shadertoy_cc {

NODE_STORAGE_FUNCS(NodeImageShaderToy)

static constexpr const char *DEFAULT_IMAGE_SHADER =
    "void mainImage(out vec4 fragColor, in vec2 fragCoord)\n"
    "{\n"
    "  vec2 uv = fragCoord / iResolution.xy;\n"
    "  vec3 col = 0.5 + 0.5 * cos(iTime + uv.xyx + vec3(0.0, 2.0, 4.0));\n"
    "  fragColor = vec4(col, 1.0);\n"
    "}\n";

static constexpr int MAX_SHADER_PARAMS = NODE_IMAGE_SHADERTOY_MAX_PARAMS;

static bool socket_is_linked(const bNode &node, const char *identifier)
{
  const bNodeSocket *sock = bke::node_find_socket(node, SOCK_IN, UString(identifier));
  return sock && sock->is_logically_linked();
}

static void set_owned_str(char **dst, const char *src)
{
  MEM_SAFE_DELETE(*dst);
  if (src && src[0] != '\0') {
    *dst = BLI_strdup(src);
  }
  else {
    *dst = nullptr;
  }
}

static const char *pass_rna_prop(const int pass_view)
{
  switch (pass_view) {
    case 1:
      return "code_common";
    case 2:
      return "code_buffer_a";
    case 3:
      return "code_buffer_b";
    case 4:
      return "code_buffer_c";
    case 5:
      return "code_buffer_d";
    default:
      return "code_image";
  }
}

/* -------------------------------------------------------------------- */
/** \name GLSL token stream
 *
 * User GLSL is only ever edited token-wise: a token is renamed, blanked, or gets text
 * appended. Whitespace, comments and line breaks are kept as they were pasted, so a
 * statement that spans several lines stays intact and compiler error lines still match
 * the ShaderToy source.
 * \{ */

static bool is_ident_start(const char c)
{
  return std::isalpha(static_cast<unsigned char>(c)) || c == '_';
}

static bool is_ident_char(const char c)
{
  return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
}

static StringRef ltrim(StringRef s)
{
  while (!s.is_empty() && std::isspace(static_cast<unsigned char>(s[0]))) {
    s = s.substr(1);
  }
  return s;
}

static StringRef rtrim(StringRef s)
{
  while (!s.is_empty() && std::isspace(static_cast<unsigned char>(s[s.size() - 1]))) {
    s = s.substr(0, s.size() - 1);
  }
  return s;
}

enum class TokKind : uint8_t { Ident, Number, Punct };

struct Tok {
  TokKind kind = TokKind::Punct;
  /** Part of a preprocessor directive, including its `#`. */
  bool pp = false;
  /** The `#` that starts a directive. */
  bool pp_start = false;
  /** Code token with a directive between it and the previous code token. */
  bool after_pp = false;
  bool replaced = false;
  /** Whitespace and comments in front of the token start here. */
  int64_t ws_begin = 0;
  int64_t begin = 0;
  int64_t end = 0;
  std::string repl;
  std::string append;
};

struct TokenStream {
  std::string src;
  Vector<Tok> toks;
  /** Indices into `toks` of everything outside preprocessor directives. */
  Vector<int> code;
  /** `//*` line comments: offsets of the `*`, see #str. */
  Vector<int64_t> comment_stars;
  int64_t tail_begin = 0;

  StringRef text(const int i) const
  {
    if (i < 0 || i >= toks.size()) {
      return StringRef();
    }
    const Tok &tok = toks[i];
    if (tok.replaced) {
      return StringRef(tok.repl);
    }
    return StringRef(src.data() + tok.begin, tok.end - tok.begin);
  }

  bool is(const int i, const StringRef s) const
  {
    return i >= 0 && i < toks.size() && this->text(i) == s;
  }

  bool is_ident(const int i) const
  {
    return i >= 0 && i < toks.size() && toks[i].kind == TokKind::Ident;
  }

  void replace(const int i, const StringRef s)
  {
    toks[i].replaced = true;
    toks[i].repl = std::string(s.data(), size_t(s.size()));
  }

  void blank(const int i)
  {
    this->replace(i, "");
  }

  /* Same accessors on the code view. */
  StringRef ctext(const int ci) const
  {
    return (ci >= 0 && ci < code.size()) ? this->text(code[ci]) : StringRef();
  }

  bool cis(const int ci, const StringRef s) const
  {
    return ci >= 0 && ci < code.size() && this->text(code[ci]) == s;
  }

  bool c_ident(const int ci) const
  {
    return ci >= 0 && ci < code.size() && toks[code[ci]].kind == TokKind::Ident;
  }

  bool c_after_pp(const int ci) const
  {
    return ci < 0 || ci >= code.size() || toks[code[ci]].after_pp;
  }

  /** Index of the bracket closing the one at `ci`, or -1. */
  int c_match(const int ci) const
  {
    int depth = 0;
    for (int i = ci; i < code.size(); i++) {
      const StringRef t = this->ctext(i);
      if (toks[code[i]].kind != TokKind::Punct) {
        continue;
      }
      if (t == "(" || t == "[" || t == "{") {
        depth++;
      }
      else if (t == ")" || t == "]" || t == "}") {
        depth--;
        if (depth == 0) {
          return i;
        }
      }
    }
    return -1;
  }

  /** One past the last token of the directive whose `#` is at `i`. */
  int directive_end(const int i) const
  {
    int end = i + 1;
    while (end < toks.size() && toks[end].pp && !toks[end].pp_start) {
      end++;
    }
    return end;
  }

  /** Source text from the end of token `i` to the end of that line (trailing comment). */
  StringRef rest_of_line(const int i) const
  {
    const int64_t from = toks[i].end;
    size_t to = src.find('\n', size_t(from));
    if (to == std::string::npos) {
      to = src.size();
    }
    return StringRef(src.data() + from, int64_t(to) - from);
  }

  std::string str() const
  {
    std::string out;
    out.reserve(src.size() + 256);
    int star = 0;
    auto trivia = [&](const int64_t from, const int64_t to) {
      int64_t at = from;
      /* `//*.7+.2` is a line comment, but scanners that look for block comments first
       * read it as an unclosed one. Write it as `// *`. */
      while (star < comment_stars.size() && comment_stars[star] < to) {
        const int64_t pos = comment_stars[star++];
        if (pos < at) {
          continue;
        }
        out.append(src.data() + at, size_t(pos - at));
        out.push_back(' ');
        at = pos;
      }
      out.append(src.data() + at, size_t(to - at));
    };
    for (const Tok &tok : toks) {
      trivia(tok.ws_begin, tok.begin);
      if (tok.replaced) {
        out += tok.repl;
      }
      else {
        out.append(src.data() + tok.begin, size_t(tok.end - tok.begin));
      }
      out += tok.append;
    }
    trivia(tail_begin, int64_t(src.size()));
    return out;
  }
};

static TokenStream tokenize(const StringRef source)
{
  TokenStream ts;
  ts.src.assign(source.data(), size_t(source.size()));
  const std::string &s = ts.src;
  const int64_t n = int64_t(s.size());
  auto newline_at = [&](const int64_t p) -> int {
    if (p < n && s[p] == '\n') {
      return 1;
    }
    if (p + 1 < n && s[p] == '\r' && s[p + 1] == '\n') {
      return 2;
    }
    return 0;
  };
  int64_t i = 0;
  int64_t ws_begin = 0;
  bool line_start = true;
  bool in_pp = false;
  while (i < n) {
    const char c = s[i];
    if (c == '\n') {
      in_pp = false;
      line_start = true;
      i++;
      continue;
    }
    if (c == '\\' && newline_at(i + 1)) {
      i += 1 + newline_at(i + 1);
      continue;
    }
    if (c == ' ' || c == '\t' || c == '\r' || c == '\f' || c == '\v') {
      i++;
      continue;
    }
    if (c == '/' && i + 1 < n && s[i + 1] == '/') {
      if (i + 2 < n && s[i + 2] == '*') {
        ts.comment_stars.append(i + 2);
      }
      while (i < n && s[i] != '\n') {
        if (s[i] == '\\' && newline_at(i + 1)) {
          i += 1 + newline_at(i + 1);
          continue;
        }
        i++;
      }
      continue;
    }
    if (c == '/' && i + 1 < n && s[i + 1] == '*') {
      i += 2;
      while (i + 1 < n && !(s[i] == '*' && s[i + 1] == '/')) {
        i++;
      }
      i = math::min(i + 2, n);
      continue;
    }

    Tok tok;
    tok.ws_begin = ws_begin;
    tok.begin = i;
    if (c == '#' && line_start) {
      in_pp = true;
      tok.pp_start = true;
      i++;
    }
    else if (is_ident_start(c)) {
      tok.kind = TokKind::Ident;
      while (i < n && is_ident_char(s[i])) {
        i++;
      }
    }
    else if (std::isdigit(static_cast<unsigned char>(c)) ||
             (c == '.' && i + 1 < n && std::isdigit(static_cast<unsigned char>(s[i + 1]))))
    {
      tok.kind = TokKind::Number;
      const bool hex = c == '0' && i + 1 < n && (s[i + 1] == 'x' || s[i + 1] == 'X');
      i++;
      while (i < n) {
        const char d = s[i];
        if (is_ident_char(d) || d == '.') {
          i++;
        }
        else if ((d == '+' || d == '-') && !hex && (s[i - 1] == 'e' || s[i - 1] == 'E')) {
          i++;
        }
        else {
          break;
        }
      }
    }
    else {
      static const char *ops[] = {"<<=", ">>=", "++", "--", "+=", "-=", "*=", "/=",
                                  "%=",  "&=",  "|=", "^=", "<<", ">>", "<=", ">=",
                                  "==",  "!=",  "&&", "||", "^^", "##"};
      int64_t len = 1;
      for (const char *op : ops) {
        const int64_t op_len = int64_t(strlen(op));
        if (i + op_len <= n && std::memcmp(s.data() + i, op, size_t(op_len)) == 0) {
          len = op_len;
          break;
        }
      }
      i += len;
    }
    tok.end = i;
    tok.pp = in_pp;
    line_start = false;
    ws_begin = i;
    ts.toks.append(std::move(tok));
  }
  ts.tail_begin = ws_begin;

  bool pp_seen = false;
  for (int t = 0; t < ts.toks.size(); t++) {
    Tok &tok = ts.toks[t];
    if (tok.pp) {
      pp_seen = true;
    }
    else {
      tok.after_pp = pp_seen;
      pp_seen = false;
      ts.code.append(t);
    }
  }
  return ts;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Shader parameter scan (#define / const / #iUniform)
 * \{ */

enum class ShaderParamKind { Float, Int, Bool, Vec2, Vec3, Color };

struct ShaderParam {
  std::string name;
  ShaderParamKind kind = ShaderParamKind::Float;
  float values[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float min_v = 0.0f;
  float max_v = 0.0f;
  bool has_range = false;
};

/** A parameter definition and the literal tokens (`first`..`last`) that hold its value. */
struct ParamDef {
  ShaderParam param;
  int first = -1;
  int last = -1;
};

static bool parse_number_token(StringRef &s, float &out)
{
  s = ltrim(s);
  if (s.is_empty()) {
    return false;
  }
  std::string tmp(s.data(), size_t(s.size()));
  char *end = nullptr;
  const float v = std::strtof(tmp.c_str(), &end);
  if (end == tmp.c_str()) {
    return false;
  }
  const int64_t eaten = int64_t(end - tmp.c_str());
  out = v;
  s = s.substr(eaten);
  return true;
}

/* Decimal int or float literal. Hex and unsigned literals are not sliders: rewriting
 * `0x7fffffff` or `1597334677U` as a float would change the type the shader hashes with. */
static bool parse_literal(const StringRef t, float &r_value, bool &r_is_int)
{
  if (t.is_empty()) {
    return false;
  }
  const std::string tmp(t.data(), size_t(t.size()));
  if (tmp.size() > 1 && tmp[0] == '0' && (tmp[1] == 'x' || tmp[1] == 'X')) {
    return false;
  }
  char *end = nullptr;
  const double v = std::strtod(tmp.c_str(), &end);
  if (end == tmp.c_str() || !std::isfinite(v)) {
    return false;
  }
  const std::string number(tmp.c_str(), size_t(end - tmp.c_str()));
  const bool is_float = number.find_first_of(".eE") != std::string::npos;
  const StringRef suffix(end);
  if (!suffix.is_empty() && !(is_float && (suffix == "f" || suffix == "F"))) {
    return false;
  }
  if (!is_float && std::fabs(v) > 1.0e7) {
    /* Not exactly representable in the float the slider stores. */
    return false;
  }
  r_value = float(v);
  r_is_int = !is_float;
  return true;
}

/* `[+-] NUMBER` at `i`. Returns the index after it, or -1. */
static int parse_signed_literal(
    const TokenStream &ts, int i, const int end, float &r_value, bool &r_is_int)
{
  float sign = 1.0f;
  if (i < end && (ts.is(i, "-") || ts.is(i, "+"))) {
    sign = ts.is(i, "-") ? -1.0f : 1.0f;
    i++;
  }
  if (i >= end || ts.toks[i].kind != TokKind::Number ||
      !parse_literal(ts.text(i), r_value, r_is_int))
  {
    return -1;
  }
  r_value *= sign;
  return i + 1;
}

/* `vec3(a, b, c)` with an optional `/ 255.` (byte palettes). Returns the index after it. */
static int parse_vec3_literal(const TokenStream &ts, int i, const int end, float r_values[3])
{
  if (!(i + 1 < end && (ts.is(i, "vec3") || ts.is(i, "color3") || ts.is(i, "float3")) &&
        ts.is(i + 1, "(")))
  {
    return -1;
  }
  i += 2;
  for (int c = 0; c < 3; c++) {
    bool is_int = false;
    i = parse_signed_literal(ts, i, end, r_values[c], is_int);
    if (i < 0 || i >= end || !ts.is(i, c < 2 ? "," : ")")) {
      return -1;
    }
    i++;
  }
  if (i + 1 < end && ts.is(i, "/")) {
    float div = 0.0f;
    bool is_int = false;
    if (ts.toks[i + 1].kind == TokKind::Number && parse_literal(ts.text(i + 1), div, is_int) &&
        std::fabs(div - 255.0f) < 0.01f)
    {
      for (int c = 0; c < 3; c++) {
        r_values[c] /= 255.0f;
      }
      i += 2;
    }
  }
  return i;
}

static void parse_range_from_rest(StringRef rest, ShaderParam &param)
{
  auto find_key = [&](const char *key, float &dst) -> bool {
    const int64_t hit = rest.find(key);
    if (hit == StringRef::not_found) {
      return false;
    }
    StringRef after = ltrim(rest.substr(hit + int64_t(strlen(key))));
    if (!after.is_empty() && after[0] == ':') {
      after = ltrim(after.substr(1));
    }
    return parse_number_token(after, dst);
  };
  float mn = 0.0f;
  float mx = 0.0f;
  if (find_key("min", mn) && find_key("max", mx) && mn < mx) {
    param.min_v = mn;
    param.max_v = mx;
    param.has_range = true;
    return;
  }
  /* `#iUniform float x = 1.0 in { 0.0, 10.0 }` */
  const int64_t in_pos = rest.find(" in ");
  if (in_pos == StringRef::not_found) {
    return;
  }
  StringRef after = ltrim(rest.substr(in_pos + 4));
  if (after.is_empty() || after[0] != '{') {
    return;
  }
  after = after.substr(1);
  if (!parse_number_token(after, mn)) {
    return;
  }
  after = ltrim(after);
  if (after.is_empty() || after[0] != ',') {
    return;
  }
  after = after.substr(1);
  if (!parse_number_token(after, mx)) {
    return;
  }
  if (mn < mx) {
    param.min_v = mn;
    param.max_v = mx;
    param.has_range = true;
  }
}

static bool is_skipped_param_name(const StringRef name)
{
  static const char *kSkip[] = {
      "PI",       "TAU",      "TWO_PI",    "TWOPI",     "PI2",      "PHI",         "E",
      "EPS",      "EPSILON",  "INFINITY",  "HUGE",      "M_PI",     "M_E",         "TRUE",
      "FALSE",    "NULL",     "FLT_MAX",   "FLT_MIN",   "iTime",    "iResolution", "iMouse",
      "iDate",    "iFrame",   "iTimeDelta", "iSampleRate", "iGlobalTime", "fragColor", "fragCoord",
      "mainImage", "Color",   "output_img", "texel",    "size",
      /* wdsGWS packing constants. Promoting EMPTY=1024 to an Int slider (min 0)
       * or keeping a leftover ZOOM=1 from another shader evaporates / rescales
       * the balls. These stay as the authored #defines. */
      "EMPTY",     "ZOOM",    "FTOV",       "VTOP",     "FRC1",        "FRC2",
  };
  for (const char *s : kSkip) {
    if (name == s) {
      return true;
    }
  }
  if (name.startswith("iChannel")) {
    return true;
  }
  return false;
}

static bool is_unit_range_vec3(const float v[3])
{
  return v[0] >= 0.0f && v[0] <= 1.0f && v[1] >= 0.0f && v[1] <= 1.0f && v[2] >= 0.0f &&
         v[2] <= 1.0f;
}

/* `#define NAME value` and `#iUniform type NAME = value`. Only an un-indented directive whose
 * whole value is one literal: `#define ZOOM (0.1 / iResolution.x)` is an expression. */
static void find_directive_param(const TokenStream &ts, const int hash, Vector<ParamDef> &r_defs)
{
  const int end = ts.directive_end(hash);
  const int64_t at = ts.toks[hash].begin;
  if (at > 0 && ts.src[size_t(at - 1)] != '\n') {
    return;
  }
  ParamDef def;
  ShaderParam &param = def.param;
  int value = -1;
  bool want_bool = false;
  bool want_int = false;
  bool want_vec3 = false;
  bool want_color = false;
  if (ts.is(hash + 1, "define")) {
    if (hash + 3 >= end || !ts.is_ident(hash + 2)) {
      return;
    }
    if (ts.toks[hash + 3].begin == ts.toks[hash + 2].end && ts.is(hash + 3, "(")) {
      return; /* Function-like macro. */
    }
    value = hash + 3;
  }
  else if (ts.is(hash + 1, "iUniform") || ts.is(hash + 1, "iuniform")) {
    if (hash + 5 >= end || !ts.is_ident(hash + 2) || !ts.is_ident(hash + 3) ||
        !ts.is(hash + 4, "="))
    {
      return;
    }
    const StringRef type = ts.text(hash + 2);
    want_bool = type == "bool";
    want_int = type == "int";
    want_color = type == "color3";
    want_vec3 = want_color || type == "vec3" || type == "float3";
    if (!(want_bool || want_int || want_vec3 || type == "float")) {
      return;
    }
    value = hash + 5;
  }
  else {
    return;
  }
  const bool is_define = ts.is(hash + 1, "define");
  const int name = is_define ? hash + 2 : hash + 3;
  param.name = std::string(ts.text(name));

  int after = -1;
  bool is_int = false;
  if (want_bool && (ts.is(value, "true") || ts.is(value, "false"))) {
    param.kind = ShaderParamKind::Bool;
    param.values[0] = ts.is(value, "true") ? 1.0f : 0.0f;
    after = value + 1;
  }
  else if (!want_vec3 &&
           (after = parse_signed_literal(ts, value, end, param.values[0], is_int)) >= 0)
  {
    if (want_bool) {
      param.kind = ShaderParamKind::Bool;
    }
    else if (is_define) {
      param.kind = is_int ? ShaderParamKind::Int : ShaderParamKind::Float;
    }
    else {
      param.kind = want_int ? ShaderParamKind::Int : ShaderParamKind::Float;
    }
  }
  else if ((is_define || want_vec3) &&
           (after = parse_vec3_literal(ts, value, end, param.values)) >= 0)
  {
    param.kind = (want_color || is_unit_range_vec3(param.values)) ? ShaderParamKind::Color :
                                                                    ShaderParamKind::Vec3;
  }
  else {
    return;
  }
  /* A define must be nothing but the literal. iUniform may carry `in { a, b }` / `step s`. */
  if (is_define && after != end) {
    return;
  }
  def.first = value;
  def.last = after - 1;
  parse_range_from_rest(ts.rest_of_line(def.last), param);
  r_defs.append(std::move(def));
}

/* File-scope `const float A = 1.5, B = 2.0;`. Each declarator whose initializer is exactly
 * one literal is a parameter; `const float C = 2.0 * PI;` is not. */
static void find_const_params(const TokenStream &ts, const int const_ci, Vector<ParamDef> &r_defs)
{
  int ci = const_ci + 1;
  const StringRef precision = ts.ctext(ci);
  if (precision == "highp" || precision == "mediump" || precision == "lowp") {
    ci++;
  }
  const StringRef type = ts.ctext(ci);
  const bool is_float = type == "float";
  const bool is_int = type == "int";
  const bool is_bool = type == "bool";
  const bool is_vec3 = type == "vec3";
  if (!(is_float || is_int || is_bool || is_vec3)) {
    return;
  }
  ci++;
  const int n = int(ts.code.size());
  while (ci + 2 < n) {
    if (!ts.c_ident(ci) || !ts.cis(ci + 1, "=") || ts.c_after_pp(ci) || ts.c_after_pp(ci + 2)) {
      return;
    }
    ParamDef def;
    ShaderParam &param = def.param;
    param.name = std::string(ts.ctext(ci));
    /* Literal tokens are adjacent in `toks` unless a directive splits them. */
    const int value = ts.code[ci + 2];
    int stop = value;
    while (stop < ts.toks.size() && !ts.toks[stop].pp) {
      stop++;
    }
    int after = -1;
    bool literal_is_int = false;
    if (is_bool && (ts.is(value, "true") || ts.is(value, "false"))) {
      param.kind = ShaderParamKind::Bool;
      param.values[0] = ts.is(value, "true") ? 1.0f : 0.0f;
      after = value + 1;
    }
    else if ((is_float || is_int) &&
             (after = parse_signed_literal(ts, value, stop, param.values[0], literal_is_int)) >= 0)
    {
      if (is_int && !literal_is_int) {
        after = -1;
      }
      param.kind = is_int ? ShaderParamKind::Int : ShaderParamKind::Float;
    }
    else if (is_vec3 && (after = parse_vec3_literal(ts, value, stop, param.values)) >= 0) {
      param.kind = is_unit_range_vec3(param.values) ? ShaderParamKind::Color :
                                                      ShaderParamKind::Vec3;
    }
    if (after >= 0 && after < stop && (ts.is(after, ",") || ts.is(after, ";"))) {
      def.first = value;
      def.last = after - 1;
      parse_range_from_rest(ts.rest_of_line(def.last), param);
      const bool done = ts.is(after, ";");
      r_defs.append(std::move(def));
      if (done) {
        return;
      }
      ci += 3 + (after - value);
      continue;
    }
    /* Expression initializer: skip to the next declarator. */
    int depth = 0;
    int k = ci + 2;
    for (; k < n; k++) {
      if (ts.c_after_pp(k)) {
        return;
      }
      const StringRef t = ts.ctext(k);
      if (t == "(" || t == "[" || t == "{") {
        depth++;
      }
      else if (t == ")" || t == "]" || t == "}") {
        if (depth == 0) {
          return;
        }
        depth--;
      }
      else if (depth == 0 && (t == "," || t == ";")) {
        break;
      }
    }
    if (k >= n || ts.cis(k, ";")) {
      return;
    }
    ci = k + 1;
  }
}

static Vector<ParamDef> find_param_defs(const TokenStream &ts)
{
  Vector<ParamDef> defs;
  for (int t = 0; t < ts.toks.size(); t++) {
    if (ts.toks[t].pp_start) {
      find_directive_param(ts, t, defs);
    }
  }
  /* A `const` outside braces and parentheses starts a file-scope declaration. Do not ask
   * for a `;` in front of it: `MAKE_OVERLOADS(iStep)` macro lines have none. */
  int depth = 0;
  int paren = 0;
  for (int ci = 0; ci < ts.code.size(); ci++) {
    const StringRef t = ts.ctext(ci);
    if (t == "{") {
      depth++;
    }
    else if (t == "}") {
      depth = math::max(depth - 1, 0);
    }
    else if (t == "(") {
      paren++;
    }
    else if (t == ")") {
      paren = math::max(paren - 1, 0);
    }
    else if (depth == 0 && paren == 0 && t == "const") {
      find_const_params(ts, ci, defs);
    }
  }
  return defs;
}

static void param_default_range(ShaderParam &param)
{
  if (param.has_range) {
    return;
  }
  const float v = param.values[0];
  if (param.kind == ShaderParamKind::Bool) {
    param.min_v = 0.0f;
    param.max_v = 1.0f;
  }
  else if (param.kind == ShaderParamKind::Int) {
    param.min_v = math::min(float(int(std::lround(v)) * 4), 0.0f);
    param.max_v = math::max(float(int(std::lround(v)) * 4), 16.0f);
  }
  else if (v >= 0.0f && v <= 1.0f) {
    param.min_v = 0.0f;
    param.max_v = 1.0f;
  }
  else {
    const float mag = math::max(math::abs(v), 1.0f);
    param.min_v = v - mag * 4.0f;
    param.max_v = v + mag * 4.0f;
  }
  param.has_range = true;
}

static Vector<ShaderParam> parse_shader_params(const NodeImageShaderToy &storage)
{
  Vector<ShaderParam> params;
  /* A name defined twice with different values (`#if`/`#else` quality presets, or the same
   * constant in two passes) has no single slider value. Leave those as authored. */
  Set<std::string> conflicts;
  for (const char *src : {storage.code_common,
                          storage.code_image,
                          storage.code_buffer_a,
                          storage.code_buffer_b,
                          storage.code_buffer_c,
                          storage.code_buffer_d})
  {
    if (src == nullptr || src[0] == '\0') {
      continue;
    }
    const TokenStream ts = tokenize(src);
    for (const ParamDef &def : find_param_defs(ts)) {
      const ShaderParam &param = def.param;
      if (param.name.empty() || is_skipped_param_name(param.name) ||
          conflicts.contains(param.name))
      {
        continue;
      }
      int existing = -1;
      for (int p = 0; p < params.size(); p++) {
        if (params[p].name == param.name) {
          existing = p;
          break;
        }
      }
      if (existing < 0) {
        params.append(param);
        continue;
      }
      const ShaderParam &other = params[existing];
      if (other.kind != param.kind || other.values[0] != param.values[0] ||
          other.values[1] != param.values[1] || other.values[2] != param.values[2])
      {
        conflicts.add(param.name);
        params.remove(existing);
      }
    }
  }
  for (ShaderParam &param : params) {
    param_default_range(param);
  }
  return params;
}

static bool param_is_scalar(const ShaderParamKind kind)
{
  return kind == ShaderParamKind::Float || kind == ShaderParamKind::Int ||
         kind == ShaderParamKind::Bool;
}

static Vector<ShaderParam> params_from_storage(const NodeImageShaderToy &storage)
{
  Vector<ShaderParam> params;
  const int count = math::clamp(int(storage.param_count), 0, MAX_SHADER_PARAMS);
  for (int i = 0; i < count; i++) {
    if (storage.param_name[i][0] == '\0') {
      continue;
    }
    ShaderParam param;
    param.name = storage.param_name[i];
    param.kind = ShaderParamKind(storage.param_kind[i]);
    if (!param_is_scalar(param.kind)) {
      param.kind = ShaderParamKind::Float;
    }
    param.values[0] = storage.param_value[i];
    param.min_v = storage.param_min[i];
    param.max_v = storage.param_max[i];
    param.has_range = true;
    params.append(std::move(param));
  }
  return params;
}

static void sync_params_from_code(NodeImageShaderToy &storage)
{
  const Vector<ShaderParam> scanned = parse_shader_params(storage);
  Map<std::string, float> old_values;
  const int old_n = math::clamp(int(storage.param_count), 0, MAX_SHADER_PARAMS);
  for (int i = 0; i < old_n; i++) {
    if (storage.param_name[i][0]) {
      old_values.add_overwrite(storage.param_name[i], storage.param_value[i]);
    }
  }
  memset(storage.param_name, 0, sizeof(storage.param_name));
  memset(storage.param_kind, 0, sizeof(storage.param_kind));
  storage.param_count = 0;
  auto push_scalar = [&](const std::string &name,
                         const ShaderParamKind kind,
                         const float value,
                         const float min_v,
                         const float max_v) {
    if (storage.param_count >= MAX_SHADER_PARAMS) {
      return;
    }
    const int i = storage.param_count;
    STRNCPY(storage.param_name[i], name.c_str());
    storage.param_kind[i] = char(kind);
    if (const float *old = old_values.lookup_ptr(name)) {
      storage.param_value[i] = *old;
    }
    else {
      storage.param_value[i] = value;
    }
    storage.param_min[i] = min_v;
    storage.param_max[i] = max_v;
    storage.param_count++;
  };

  for (const ShaderParam &param : scanned) {
    if (param.kind == ShaderParamKind::Vec3 || param.kind == ShaderParamKind::Color) {
      const bool as_color = (param.kind == ShaderParamKind::Color);
      const char *suf[3] = {as_color ? ".r" : ".x",
                            as_color ? ".g" : ".y",
                            as_color ? ".b" : ".z"};
      /* All three components fit, or none: a vec3 with a missing slider cannot be rebuilt. */
      if (storage.param_count + 3 > MAX_SHADER_PARAMS) {
        continue;
      }
      for (int c = 0; c < 3; c++) {
        ShaderParam comp;
        comp.kind = ShaderParamKind::Float;
        comp.values[0] = param.values[c];
        if (!as_color) {
          param_default_range(comp);
        }
        push_scalar(param.name + suf[c],
                    ShaderParamKind::Float,
                    param.values[c],
                    as_color ? 0.0f : comp.min_v,
                    as_color ? 1.0f : comp.max_v);
      }
      continue;
    }
    if (!param_is_scalar(param.kind)) {
      continue;
    }
    push_scalar(param.name, param.kind, param.values[0], param.min_v, param.max_v);
  }
}

static std::string glsl_number(const float v)
{
  char buf[64];
  SNPRINTF(buf, "%.8g", double(v));
  return buf;
}

/* Always a float literal: `#define SCALE 2.0` printed as `2` turns `SCALE / 3` into integer
 * division. */
static std::string glsl_float(const float v)
{
  if (!std::isfinite(v)) {
    return "0.0";
  }
  char buf[64];
  SNPRINTF(buf, "%.9g", double(v));
  std::string out = buf;
  if (out.find_first_of(".e") == std::string::npos) {
    out += ".0";
  }
  return out;
}

static std::string format_param_defines(const Span<ShaderParam> params)
{
  std::string out;
  out.reserve(params.size() * 48);
  for (const ShaderParam &param : params) {
    out += "#define ";
    out += param.name;
    out += " ";
    switch (param.kind) {
      case ShaderParamKind::Float:
        out += glsl_number(param.values[0]);
        break;
      case ShaderParamKind::Int:
        out += std::to_string(int(std::lround(param.values[0])));
        break;
      case ShaderParamKind::Bool:
        out += (param.values[0] > 0.5f) ? "1" : "0";
        break;
      case ShaderParamKind::Vec2:
        out += "vec2(";
        out += glsl_number(param.values[0]);
        out += ", ";
        out += glsl_number(param.values[1]);
        out += ")";
        break;
      case ShaderParamKind::Vec3:
      case ShaderParamKind::Color:
        out += "vec3(";
        out += glsl_number(param.values[0]);
        out += ", ";
        out += glsl_number(param.values[1]);
        out += ", ";
        out += glsl_number(param.values[2]);
        out += ")";
        break;
    }
    out += "\n";
  }
  out += "\n";
  return out;
}

static bool gather_vec3_param(const Span<ShaderParam> params, const StringRef base, float rgb[3])
{
  const char *sets[][3] = {{".r", ".g", ".b"}, {".x", ".y", ".z"}};
  for (const auto &suf : sets) {
    float v[3] = {0.0f, 0.0f, 0.0f};
    int found = 0;
    for (int c = 0; c < 3; c++) {
      const std::string n = std::string(base) + suf[c];
      for (const ShaderParam &param : params) {
        if (param.name == n) {
          v[c] = param.values[0];
          found++;
          break;
        }
      }
    }
    if (found == 3) {
      rgb[0] = v[0];
      rgb[1] = v[1];
      rgb[2] = v[2];
      return true;
    }
  }
  return false;
}

/* Write slider values back into the source. Only the literal is replaced, and only when the
 * slider was moved: an untouched parameter leaves the pasted line exactly as it is. */
static std::string apply_param_values(const StringRef src, const Span<ShaderParam> params)
{
  if (params.is_empty()) {
    return src;
  }
  TokenStream ts = tokenize(src);
  bool changed = false;
  for (const ParamDef &def : find_param_defs(ts)) {
    const ShaderParam &authored = def.param;
    std::string value;
    if (authored.kind == ShaderParamKind::Vec3 || authored.kind == ShaderParamKind::Color) {
      float rgb[3];
      if (!gather_vec3_param(params, authored.name, rgb) ||
          (rgb[0] == authored.values[0] && rgb[1] == authored.values[1] &&
           rgb[2] == authored.values[2]))
      {
        continue;
      }
      value = "vec3(" + glsl_float(rgb[0]) + ", " + glsl_float(rgb[1]) + ", " +
              glsl_float(rgb[2]) + ")";
    }
    else {
      const ShaderParam *slider = nullptr;
      for (const ShaderParam &param : params) {
        if (param.name == authored.name) {
          slider = &param;
          break;
        }
      }
      if (slider == nullptr) {
        continue;
      }
      const float v = slider->values[0];
      if (authored.kind == ShaderParamKind::Bool) {
        if ((v > 0.5f) == (authored.values[0] > 0.5f)) {
          continue;
        }
        /* A numeric literal stays numeric (`#iUniform bool x = 1`). */
        const bool numeric = ts.toks[def.first].kind == TokKind::Number;
        value = (v > 0.5f) ? (numeric ? "1" : "true") : (numeric ? "0" : "false");
      }
      else if (authored.kind == ShaderParamKind::Int) {
        if (std::lround(v) == std::lround(authored.values[0])) {
          continue;
        }
        value = std::to_string(int(std::lround(v)));
      }
      else {
        if (v == authored.values[0]) {
          continue;
        }
        value = glsl_float(v);
      }
    }
    ts.replace(def.first, value);
    for (int t = def.first + 1; t <= def.last; t++) {
      ts.blank(t);
    }
    changed = true;
  }
  return changed ? ts.str() : std::string(src.data(), size_t(src.size()));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Declare / UI / DNA
 * \{ */

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("Time Scale"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .max(1000.0f)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Multiplier on Scene Time for iTime");
  b.add_input<decl::Bool>("Pause"_ustr)
      .default_value(false)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Freeze iTime and Buffer A–D");
  /* Keep native resolution: ShaderToy iChannels are not resized to the canvas.
   * These sockets are site media (noise / organic / photo). Buffer A–D still
   * auto-wire to each other and to Image; they do not consume a Channel slot. */
  b.add_input<decl::Color>("Channel 0"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Texture for iChannel0 (Buffer and Image can both sample this)");
  b.add_input<decl::Color>("Channel 1"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Texture for iChannel1 (Buffer and Image can both sample this)");
  b.add_input<decl::Color>("Channel 2"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Texture for iChannel2 (Buffer and Image can both sample this)");
  b.add_input<decl::Color>("Channel 3"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Texture for iChannel3 (Buffer and Image can both sample this)");

  b.add_output<decl::Color>("Color"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("ShaderToy Image pass output");
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA * /*ptr*/)
{
  layout.op("node.shadertoy_compile", IFACE_("Compile"), ICON_FILE_REFRESH);
}

static void node_layout_ex(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);

  layout.op("node.shadertoy_compile", IFACE_("Compile"), ICON_FILE_REFRESH);

  bNode &node = *static_cast<bNode *>(ptr->data);
  NodeImageShaderToy &storage = node_storage(node);

  layout.separator();
  layout.prop(ptr, "mouse_pos", UI_ITEM_NONE, IFACE_("Mouse"), ICON_NONE);
  layout.separator();
  layout.label(IFACE_("iChannel  A–D Buffer, T texture, K black, U cubemap, V volume"), ICON_NONE);
  layout.label(IFACE_("Cubemap A (mainCubemap) pastes into Buffer A"), ICON_NONE);
  layout.prop(ptr, "channels_image", UI_ITEM_NONE, IFACE_("Image"), ICON_NONE);
  layout.prop(ptr, "channels_buffer_a", UI_ITEM_NONE, IFACE_("Buffer A"), ICON_NONE);
  layout.prop(ptr, "channels_buffer_b", UI_ITEM_NONE, IFACE_("Buffer B"), ICON_NONE);
  layout.prop(ptr, "channels_buffer_c", UI_ITEM_NONE, IFACE_("Buffer C"), ICON_NONE);
  layout.prop(ptr, "channels_buffer_d", UI_ITEM_NONE, IFACE_("Buffer D"), ICON_NONE);

  if (storage.param_count > 0) {
    layout.separator();
    PropertyRNA *value_prop = RNA_struct_find_property(ptr, "param_value");
    const int count = math::clamp(int(storage.param_count), 0, MAX_SHADER_PARAMS);
    for (int i = 0; i < count; i++) {
      if (storage.param_name[i][0] == '\0' || value_prop == nullptr) {
        continue;
      }
      layout.prop(ptr, value_prop, i, 0, UI_ITEM_NONE, storage.param_name[i], ICON_NONE);
    }
  }

  layout.separator();
  layout.prop(ptr, "pass_view", UI_ITEM_NONE, IFACE_("Pass"), ICON_NONE);
  const char *prop = pass_rna_prop(storage.pass_view);
  layout.textbox_with_state(ptr, prop, &storage.textbox_state, IFACE_("GLSL"));

  if (storage.error_log && storage.error_log[0]) {
    layout.separator();
    layout.label(IFACE_("Compile error"), ICON_ERROR);
    layout.prop(ptr, "error_log", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  }
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  NodeImageShaderToy *storage = MEM_new<NodeImageShaderToy>(__func__);
  storage->textbox_state.visible_lines = 14;
  storage->code_image = BLI_strdup(DEFAULT_IMAGE_SHADER);
  node->storage = storage;
  node->width = 200.0f;
}

static void node_free_storage(bNode *node)
{
  NodeImageShaderToy *storage = static_cast<NodeImageShaderToy *>(node->storage);
  if (storage == nullptr) {
    return;
  }
  MEM_SAFE_DELETE(storage->code_common);
  MEM_SAFE_DELETE(storage->code_buffer_a);
  MEM_SAFE_DELETE(storage->code_buffer_b);
  MEM_SAFE_DELETE(storage->code_buffer_c);
  MEM_SAFE_DELETE(storage->code_buffer_d);
  MEM_SAFE_DELETE(storage->code_image);
  MEM_SAFE_DELETE(storage->error_log);
  MEM_delete(storage);
  node->storage = nullptr;
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeImageShaderToy &src = node_storage(*src_node);
  NodeImageShaderToy *dst = MEM_new<NodeImageShaderToy>(__func__, dna::shallow_copy(src));
  dst_node->storage = dst;
  dst->code_common = BLI_strdup_null(src.code_common);
  dst->code_buffer_a = BLI_strdup_null(src.code_buffer_a);
  dst->code_buffer_b = BLI_strdup_null(src.code_buffer_b);
  dst->code_buffer_c = BLI_strdup_null(src.code_buffer_c);
  dst->code_buffer_d = BLI_strdup_null(src.code_buffer_d);
  dst->code_image = BLI_strdup_null(src.code_image);
  dst->error_log = BLI_strdup_null(src.error_log);
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  const NodeImageShaderToy &storage = node_storage(node);
  writer.write_string(storage.code_common);
  writer.write_string(storage.code_buffer_a);
  writer.write_string(storage.code_buffer_b);
  writer.write_string(storage.code_buffer_c);
  writer.write_string(storage.code_buffer_d);
  writer.write_string(storage.code_image);
  writer.write_string(storage.error_log);
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  NodeImageShaderToy &storage = node_storage(node);
  BLO_read_string(&reader, &storage.code_common);
  BLO_read_string(&reader, &storage.code_buffer_a);
  BLO_read_string(&reader, &storage.code_buffer_b);
  BLO_read_string(&reader, &storage.code_buffer_c);
  BLO_read_string(&reader, &storage.code_buffer_d);
  BLO_read_string(&reader, &storage.code_image);
  BLO_read_string(&reader, &storage.error_log);
  if (storage.time_scale <= 0.0f) {
    storage.time_scale = 1.0f;
  }
}

static void node_label(const bNodeTree * /*ntree*/,
                       const bNode * /*node*/,
                       char *label,
                       int label_maxncpy)
{
  BLI_strncpy(label, IFACE_("ShaderToy"), label_maxncpy);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name GLSL wrapper + runtime compile (fullscreen fragment, so dFdx works)
 *
 * ShaderToy sources already compile as desktop GLSL when they are wrapped the way the site
 * wraps them. Everything here is therefore a token edit that leaves the pasted code in
 * place: names that collide with desktop keywords, WebGL's zero-initialized variables, and
 * a few sampling fixes. Never split, move or re-emit user statements.
 * \{ */

static bool is_ident_char_st(const char c)
{
  return is_ident_char(c);
}

static bool is_negative_mip_bias(StringRef s)
{
  s = rtrim(ltrim(s));
  while (s.size() >= 2 && s[0] == '(' && s[s.size() - 1] == ')') {
    s = rtrim(ltrim(s.substr(1, s.size() - 2)));
  }
  if (s.is_empty() || s[0] != '-') {
    return false;
  }
  std::string tmp(s.data(), size_t(s.size()));
  char *end = nullptr;
  const double v = std::strtod(tmp.c_str(), &end);
  return end != tmp.c_str() && v < -0.5;
}

static bool looks_like_iq_noise_uv(StringRef uv)
{
  /* Classic IQ packing: `(uv+0.5)/256.0` — bilinear between texel centers.
   * Must NOT match ShaderToy keyboard `vec2(32.5/256.0, 0.5)` (Xst3Dj reset).
   * That UV also contains 256 and 0.5; rewriting it to lod0-fetch of a noise
   * dummy makes reset() stick true and freezes the sim. */
  if (uv.find("256") == StringRef::not_found) {
    return false;
  }
  return uv.find("+0.5") != StringRef::not_found || uv.find("+ 0.5") != StringRef::not_found ||
         uv.find("+.5") != StringRef::not_found || uv.find("0.5+") != StringRef::not_found ||
         uv.find("0.5 +") != StringRef::not_found || uv.find(".5+") != StringRef::not_found;
}

static bool looks_like_keyboard_uv(StringRef uv)
{
  /* ShaderToy keyboard: `texture(iChannelN, vec2(32.5/256.0, 0.5))`. */
  if (uv.find("256") == StringRef::not_found || uv.find("vec2") == StringRef::not_found) {
    return false;
  }
  if (looks_like_iq_noise_uv(uv)) {
    return false;
  }
  return uv.find("0.5") != StringRef::not_found || uv.find(".5") != StringRef::not_found;
}

static std::string glsl_zero_for_type(const StringRef type,
                                     const Map<std::string, std::string> &struct_ctors)
{
  static const char *zeros[][2] = {
      {"float", "0.0"},          {"int", "0"},           {"uint", "0u"},
      {"bool", "false"},         {"vec2", "vec2(0.0)"},  {"vec3", "vec3(0.0)"},
      {"vec4", "vec4(0.0)"},     {"ivec2", "ivec2(0)"},  {"ivec3", "ivec3(0)"},
      {"ivec4", "ivec4(0)"},     {"uvec2", "uvec2(0u)"}, {"uvec3", "uvec3(0u)"},
      {"uvec4", "uvec4(0u)"},    {"bvec2", "bvec2(false)"}, {"bvec3", "bvec3(false)"},
      {"bvec4", "bvec4(false)"}, {"mat2", "mat2(0.0)"},  {"mat3", "mat3(0.0)"},
      {"mat4", "mat4(0.0)"},
  };
  for (const auto &zero : zeros) {
    if (type == zero[0]) {
      return zero[1];
    }
  }
  if (const std::string *ctor = struct_ctors.lookup_ptr(std::string(type))) {
    return *ctor;
  }
  return "";
}

static bool is_precision_qualifier(const StringRef t)
{
  return t == "highp" || t == "mediump" || t == "lowp";
}

/* Words that are never a type or a variable name. */
static bool is_statement_keyword(const StringRef t)
{
  static const char *words[] = {
      "return",  "if",       "else",    "for",       "while",     "do",      "switch",
      "case",    "default",  "break",   "continue",  "discard",   "const",   "in",
      "out",     "inout",    "uniform", "layout",    "precision", "highp",   "mediump",
      "lowp",    "struct",   "flat",    "smooth",    "centroid",  "invariant",
  };
  for (const char *word : words) {
    if (t == word) {
      return true;
    }
  }
  return false;
}

/* WebGL lets shaders use names that desktop and Vulkan GLSL reserve.
 * Dsf3WH: `float drawFont(vec2 p, int char)`, csc3RS: `SampleDepth(sampler2D sampler, ...)`. */
static bool is_reserved_ident(const StringRef t)
{
  static const char *words[] = {
      "char",           "class",          "union",        "template",      "this",
      "packed",         "input",          "output",       "filter",        "partition",
      "public",         "private",        "short",        "long",          "half",
      "fixed",          "unsigned",       "byte",         "goto",          "inline",
      "typename",       "using",          "namespace",    "volatile",      "abstract",
      "active",         "asm",            "cast",         "extern",        "interface",
      "sampler",        "sample",         "buffer",       "shared",        "subroutine",
      "patch",          "coherent",       "restrict",     "readonly",      "writeonly",
      "precise",        "resource",       "common",       "noperspective", "samplerShadow",
      "texture1D",      "texture1DArray", "texture2DArray", "texture2DMS", "texture2DMSArray",
      "texture2DRect",  "textureBuffer",  "textureCubeArray", "subpassInput", "subpassInputMS",
  };
  for (const char *word : words) {
    if (t == word) {
      return true;
    }
  }
  return false;
}

/* WebGL 1 sampling functions. Mapped to the modern builtin unless the shader brings its own
 * `vec4 texture2D(sampler2D s, vec2 uv)` wrapper, which is then renamed instead. */
static const char *legacy_texture_names[][2] = {
    {"texture2D", "texture"},           {"textureCube", "texture"},
    {"texture3D", "texture"},           {"texture2DLod", "textureLod"},
    {"textureCubeLod", "textureLod"},   {"texture2DLodEXT", "textureLod"},
    {"textureCubeLodEXT", "textureLod"}, {"texture2DGrad", "textureGrad"},
    {"texture2DGradEXT", "textureGrad"}, {"textureCubeGradEXT", "textureGrad"},
    {"texture2DProj", "textureProj"},   {"texture2DProjLod", "textureProjLod"},
};

static bool is_legacy_texture_name(const StringRef t)
{
  for (const auto &legacy : legacy_texture_names) {
    if (t == legacy[0]) {
      return true;
    }
  }
  return false;
}

/* Builtins that exist on desktop but not in WebGL 2, so shaders write their own:
 * `int bitCount(int x)`, `float max3(float a, float b, float c)`. With identical parameter
 * types that is a redeclaration of the builtin, which glslang rejects ("must have the same
 * parameter precision qualifiers"). A shader that declares one gets its own name. */
static bool is_desktop_builtin_name(const StringRef t)
{
  static const char *names[] = {
      "fma",            "frexp",          "ldexp",           "bitCount",        "bitfieldExtract",
      "bitfieldInsert", "bitfieldReverse", "findLSB",        "findMSB",         "uaddCarry",
      "usubBorrow",     "umulExtended",   "imulExtended",    "noise1",          "noise2",
      "noise3",         "noise4",         "min3",            "max3",            "mid3",
      "packUnorm4x8",   "packSnorm4x8",   "unpackUnorm4x8",  "unpackSnorm4x8",  "packDouble2x32",
      "unpackDouble2x32", "textureQueryLod", "textureQueryLevels", "textureGather",
      "textureGatherOffset", "textureSamples", "dFdxFine",   "dFdyFine",        "fwidthFine",
      "dFdxCoarse",     "dFdyCoarse",     "fwidthCoarse",    "interpolateAtCentroid",
      "interpolateAtSample", "interpolateAtOffset",
  };
  for (const char *name : names) {
    if (t == name) {
      return true;
    }
  }
  return false;
}

/* Builtins glslang does not fold, see #demote_non_constant_consts. */
static bool is_unfoldable_builtin_name(const StringRef t)
{
  static const char *names[] = {
      "uintBitsToFloat", "intBitsToFloat", "floatBitsToInt", "floatBitsToUint", "inverse",
      "determinant",     "transpose",      "texture",        "textureLod",      "texelFetch",
      "textureSize",
  };
  for (const char *name : names) {
    if (t == name) {
      return true;
    }
  }
  return false;
}

static bool is_shadertoy_uniform_name(const StringRef t)
{
  static const char *names[] = {
      "iResolution", "iTime",     "iTimeDelta", "iFrame",    "iFrameRate",
      "iMouse",      "iDate",     "iSampleRate", "iChannel0", "iChannel1",
      "iChannel2",   "iChannel3", "iChannelResolution", "iChannelTime", "iGlobalTime",
  };
  for (const char *name : names) {
    if (t == name) {
      return true;
    }
  }
  return false;
}

/** What Common and the pass declare. Gathered before any edit so both halves agree. */
struct SourceInfo {
  Set<std::string> macros;
  /** Function-like macros that take at least one parameter. */
  Set<std::string> macros_with_params;
  /** Replacement tokens of every macro. */
  Map<std::string, Vector<std::string>> macro_bodies;
  /** Functions and variables declared at file scope. */
  Set<std::string> globals;
  Set<std::string> functions;
  /** Every `name(`. */
  Set<std::string> calls;
  Set<std::string> struct_names;
  Map<std::string, std::string> struct_zero;
  /** File-scope `const` that had to become a plain global. */
  Set<std::string> demoted_consts;

  bool declares(const StringRef name) const
  {
    const std::string key(name.data(), size_t(name.size()));
    return macros.contains(key) || globals.contains(key);
  }
};

/* `[N]` at `ci` with a small literal size. Returns N, or 0. */
static int literal_array_size(const TokenStream &ts, const int ci)
{
  if (!ts.cis(ci, "[") || !ts.cis(ci + 2, "]")) {
    return 0;
  }
  const std::string text(ts.ctext(ci + 1));
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos ||
      text.size() > 2)
  {
    return 0;
  }
  const int count = std::atoi(text.c_str());
  return (count >= 1 && count <= 64) ? count : 0;
}

/* `Particle(vec3(0.0), 0.0, ...)` for each struct whose members are all known. */
static void collect_struct_zero_ctors(const TokenStream &ts, Map<std::string, std::string> &ctors)
{
  const int n = int(ts.code.size());
  for (int ci = 0; ci + 2 < n; ci++) {
    if (!ts.cis(ci, "struct") || !ts.c_ident(ci + 1) || !ts.cis(ci + 2, "{")) {
      continue;
    }
    const int close = ts.c_match(ci + 2);
    if (close < 0) {
      return;
    }
    const std::string name(ts.ctext(ci + 1));
    std::string args;
    bool ok = true;
    int m = ci + 3;
    while (ok && m < close) {
      if (ts.c_after_pp(m)) {
        /* `#if` inside the struct: the member list depends on the preprocessor. */
        ok = false;
        break;
      }
      if (is_precision_qualifier(ts.ctext(m))) {
        m++;
      }
      const std::string type(ts.ctext(m));
      const std::string zero = glsl_zero_for_type(type, ctors);
      m++;
      /* `float[3] a;` and `float a[3];` both declare an array member. */
      const int type_count = literal_array_size(ts, m);
      if (zero.empty() || (ts.cis(m, "[") && type_count == 0)) {
        ok = false;
        break;
      }
      if (type_count > 0) {
        m += 3;
      }
      while (true) {
        if (!ts.c_ident(m) || ts.c_after_pp(m)) {
          ok = false;
          break;
        }
        m++;
        int count = type_count;
        if (ts.cis(m, "[")) {
          count = literal_array_size(ts, m);
          if (count == 0 || type_count > 0) {
            ok = false;
            break;
          }
          m += 3;
        }
        if (!args.empty()) {
          args += ", ";
        }
        if (count > 0) {
          args += type + "[" + std::to_string(count) + "](";
          for (int k = 0; k < count; k++) {
            args += (k > 0) ? ", " : "";
            args += zero;
          }
          args += ")";
        }
        else {
          args += zero;
        }
        if (ts.cis(m, ",")) {
          m++;
          continue;
        }
        if (ts.cis(m, ";")) {
          m++;
        }
        else {
          ok = false;
        }
        break;
      }
    }
    if (ok && !args.empty()) {
      ctors.add_overwrite(name, name + "(" + args + ")");
    }
    ci = close;
  }
}

static void analyze_source(const TokenStream &ts, SourceInfo &info)
{
  const int n = int(ts.toks.size());
  for (int t = 0; t < n; t++) {
    if (ts.is_ident(t) && ts.is(t + 1, "(")) {
      info.calls.add(std::string(ts.text(t)));
    }
    if (!(ts.toks[t].pp_start && ts.is(t + 1, "define") && ts.is_ident(t + 2))) {
      continue;
    }
    const int end = ts.directive_end(t);
    const std::string name(ts.text(t + 2));
    info.macros.add(name);
    int body = t + 3;
    if (body < end && ts.is(body, "(") && ts.toks[body].begin == ts.toks[t + 2].end) {
      if (!ts.is(body + 1, ")")) {
        info.macros_with_params.add(name);
      }
      while (body < end && !ts.is(body, ")")) {
        body++;
      }
      body++;
    }
    Vector<std::string> tokens;
    for (int k = body; k < end; k++) {
      tokens.append(std::string(ts.text(k)));
      /* `#define FUNC_LERP(T) T lerp(T a, T b, float t) { ... }` declares `lerp`. */
      if (k > body && ts.is_ident(k) && ts.is_ident(k - 1) && ts.is(k + 1, "(") &&
          !is_statement_keyword(ts.text(k - 1)))
      {
        info.globals.add(std::string(ts.text(k)));
        info.functions.add(std::string(ts.text(k)));
      }
    }
    info.macro_bodies.add_overwrite(name, std::move(tokens));
  }
  int depth = 0;
  for (int ci = 0; ci < ts.code.size(); ci++) {
    const StringRef t = ts.ctext(ci);
    if (t == "{") {
      depth++;
      /* `struct S {`, and `ST S {` with `#define ST struct`. */
      if (ci >= 2 && ts.c_ident(ci - 1) && ts.c_ident(ci - 2) &&
          (ts.cis(ci - 2, "struct") || !is_statement_keyword(ts.ctext(ci - 2))))
      {
        info.struct_names.add(std::string(ts.ctext(ci - 1)));
      }
    }
    else if (t == "}") {
      depth = math::max(depth - 1, 0);
    }
    else if (depth == 0 && ci > 0 && ts.c_ident(ci) && ts.c_ident(ci - 1) &&
             !is_statement_keyword(ts.ctext(ci - 1)) && !ts.c_after_pp(ci))
    {
      const StringRef next = ts.ctext(ci + 1);
      if (next == "(" || next == ";" || next == "=" || next == "," || next == "[") {
        info.globals.add(std::string(t));
      }
      if (next == "(") {
        info.functions.add(std::string(t));
      }
    }
  }
  collect_struct_zero_ctors(ts, info.struct_zero);
}

/* `#iUniform float speed = 1.0 in { 0.0, 4.0 }` is an editor extension, not GLSL. */
static void convert_iuniform(TokenStream &ts, const int hash, const int end)
{
  if (hash + 3 >= end) {
    for (int t = hash; t < end; t++) {
      ts.blank(t);
    }
    return;
  }
  ts.replace(hash + 1, "define");
  ts.blank(hash + 2);
  if (hash + 4 < end && ts.is(hash + 4, "=")) {
    ts.blank(hash + 4);
  }
  for (int t = hash + 5; t < end; t++) {
    if (ts.is(t, "in") || ts.is(t, "step")) {
      ts.replace(t, "// " + std::string(ts.text(t)));
      break;
    }
    if (ts.is(t, "color3") || ts.is(t, "float3")) {
      ts.replace(t, "vec3");
    }
  }
}

/* True when `name` cannot be part of a constant expression for glslang: a call to a user
 * function or an unfolded builtin, a uniform, or a macro / demoted const that leads to one. */
static bool is_non_constant_name(const SourceInfo &info,
                                 const StringRef name,
                                 const bool is_call,
                                 const int rec = 0)
{
  const std::string key(name.data(), size_t(name.size()));
  if (info.demoted_consts.contains(key) || is_shadertoy_uniform_name(name)) {
    return true;
  }
  if (const Vector<std::string> *body = info.macro_bodies.lookup_ptr(key)) {
    if (rec > 6) {
      return false;
    }
    for (int k = 0; k < body->size(); k++) {
      const StringRef token = (*body)[k];
      if (!token.is_empty() && is_ident_start(token[0]) &&
          is_non_constant_name(info, token, k + 1 < body->size() && (*body)[k + 1] == "(", rec + 1))
      {
        return true;
      }
    }
    return false;
  }
  return is_call && (is_unfoldable_builtin_name(name) || info.functions.contains(key));
}

/* `const float MaxFloat = uintBitsToFloat(0x7F800000u);` and `const mat3 toCam =
 * inverse(diag(...));` are accepted by browsers, but glslang wants a file-scope `const`
 * initializer it can fold ("global const initializers must be constant"). Drop the `const`:
 * the value is the same, and constants built from it follow in declaration order. */
static void demote_non_constant_consts(TokenStream &ts, SourceInfo &info)
{
  const int n = int(ts.code.size());
  int depth = 0;
  int paren = 0;
  for (int ci = 0; ci < n; ci++) {
    const StringRef t = ts.ctext(ci);
    if (t == "{") {
      depth++;
    }
    else if (t == "}") {
      depth = math::max(depth - 1, 0);
    }
    else if (t == "(") {
      paren++;
    }
    else if (t == ")") {
      paren = math::max(paren - 1, 0);
    }
    if (depth != 0 || paren != 0 || t != "const") {
      continue;
    }
    Vector<std::string> names;
    bool non_constant = false;
    bool in_init = false;
    int nest = 0;
    int end = ci + 1;
    for (; end < n; end++) {
      if (ts.c_after_pp(end)) {
        break;
      }
      const StringRef s = ts.ctext(end);
      if (s == "(" || s == "[" || s == "{") {
        nest++;
      }
      else if (s == ")" || s == "]" || s == "}") {
        nest--;
      }
      else if (nest == 0 && s == ";") {
        break;
      }
      else if (nest == 0 && s == "=") {
        in_init = true;
        if (ts.c_ident(end - 1)) {
          names.append(std::string(ts.ctext(end - 1)));
        }
        else if (ts.cis(end - 1, "]")) {
          /* `const float a[2] = ...`: the name is in front of the brackets. */
          int k = end - 1;
          while (k > ci && !ts.cis(k, "[")) {
            k--;
          }
          if (ts.c_ident(k - 1)) {
            names.append(std::string(ts.ctext(k - 1)));
          }
        }
      }
      else if (nest == 0 && s == ",") {
        in_init = false;
      }
      else if (in_init && ts.c_ident(end) && !ts.cis(end - 1, ".")) {
        non_constant |= is_non_constant_name(info, s, ts.cis(end + 1, "("));
      }
    }
    if (end >= n || !ts.cis(end, ";") || !non_constant) {
      continue;
    }
    ts.blank(ts.code[ci]);
    for (const std::string &name : names) {
      info.demoted_consts.add(name);
    }
    ci = end;
  }
}

/* Declarations the site (or a converter) adds around the shader, and names that mean
 * something else on desktop. */
static void rewrite_names_and_preamble(TokenStream &ts, const SourceInfo &info)
{
  const int n = int(ts.toks.size());
  for (int t = 0; t < n; t++) {
    /* `H()` for `#define H(s) ...` passes one empty argument. glslang calls that "too few
     * args"; a macro that expands to nothing is the same argument with a token in it. */
    if (ts.is_ident(t) && t + 2 < n && ts.is(t + 1, "(") && ts.is(t + 2, ")") &&
        ts.toks[t].pp == ts.toks[t + 2].pp && !(t > 0 && ts.toks[t - 1].pp_start) &&
        info.macros_with_params.contains(std::string(ts.text(t))) &&
        !(t >= 2 && ts.toks[t - 2].pp_start && ts.is(t - 1, "define")))
    {
      ts.toks[t + 1].append = "_st_empty";
    }
    if (!ts.toks[t].pp_start) {
      continue;
    }
    const int end = ts.directive_end(t);
    if (t + 1 >= end) {
      continue;
    }
    if (ts.is(t + 1, "version") || ts.is(t + 1, "extension")) {
      for (int k = t; k < end; k++) {
        ts.blank(k);
      }
    }
    else if (ts.is(t + 1, "iUniform") || ts.is(t + 1, "iuniform")) {
      convert_iuniform(ts, t, end);
    }
    else if (ts.is(t + 1, "if") || ts.is(t + 1, "elif")) {
      /* `#if VOLUME_RES == 64u`: glslang's preprocessor has no unsigned literals. */
      for (int k = t + 2; k < end; k++) {
        const StringRef number = ts.text(k);
        if (ts.toks[k].kind == TokKind::Number && number.size() > 1 &&
            (number.endswith("u") || number.endswith("U")))
        {
          ts.replace(k, std::string(number.substr(0, number.size() - 1)));
        }
      }
    }
    else if (ts.is(t + 1, "define") && t + 3 < end && !ts.is(t + 3, "(") &&
             ts.toks[t + 3].begin == ts.toks[t + 2].end)
    {
      /* `#define update; if (x) {...}`: glslang needs the space after the name. */
      ts.toks[t + 2].append = " ";
    }
  }

  /* File-scope `precision ...;`, `uniform ...;` and `out vec4 fragColor;`. Statements, not
   * lines: a `mainImage(` signature broken over lines also has a line starting with `out`. */
  const int cn = int(ts.code.size());
  int depth = 0;
  int paren = 0;
  bool stmt_start = true;
  for (int ci = 0; ci < cn; ci++) {
    const StringRef t = ts.ctext(ci);
    const bool start = stmt_start;
    stmt_start = false;
    if (t == "{") {
      depth++;
      stmt_start = true;
      continue;
    }
    if (t == "}") {
      depth = math::max(depth - 1, 0);
      stmt_start = true;
      continue;
    }
    if (t == "(") {
      paren++;
    }
    else if (t == ")") {
      paren = math::max(paren - 1, 0);
    }
    else if (t == ";") {
      stmt_start = (paren == 0);
    }
    if (!start || depth != 0 || paren != 0) {
      continue;
    }
    if (!(t == "precision" || t == "uniform" || t == "in" || t == "out" || t == "layout")) {
      continue;
    }
    int end = ci;
    int nest = 0;
    int uniform_at = -1;
    bool ok = true;
    for (; end < cn; end++) {
      const StringRef s = ts.ctext(end);
      if ((end > ci && ts.c_after_pp(end)) || s == "{" || s == "}") {
        ok = false;
        break;
      }
      if (s == "(" || s == "[") {
        nest++;
      }
      else if (s == ")" || s == "]") {
        nest--;
      }
      else if (s == "uniform") {
        uniform_at = end;
      }
      else if (s == ";" && nest == 0) {
        break;
      }
    }
    if (!ok || end >= cn) {
      continue;
    }
    int name = end - 1;
    while (name > ci && ts.cis(name, "]")) {
      while (name > ci && !ts.cis(name, "[")) {
        name--;
      }
      name--;
    }
    const StringRef declared = ts.c_ident(name) ? ts.ctext(name) : StringRef();
    int blank_to = ci - 1;
    if (t == "precision") {
      blank_to = end;
    }
    else if (uniform_at >= 0) {
      const StringRef type = ts.ctext(uniform_at + 1);
      if (is_shadertoy_uniform_name(declared)) {
        blank_to = end;
      }
      else if (type.find("sampler") == StringRef::not_found) {
        /* `uniform float u_time;` from another host. Loose uniforms are not allowed on
         * Vulkan; the site leaves them at zero, so a plain global behaves the same. */
        blank_to = uniform_at;
      }
    }
    else if (declared == "fragColor" || declared == "fragCoord" || declared == "gl_FragColor") {
      blank_to = end;
    }
    for (int k = ci; k <= blank_to; k++) {
      ts.blank(ts.code[k]);
    }
    ci = end;
    stmt_start = true;
  }

  for (int t = 0; t < n; t++) {
    if (!ts.is_ident(t) || ts.toks[t].replaced || (t > 0 && ts.toks[t - 1].pp_start)) {
      continue;
    }
    const StringRef name = ts.text(t);
    if (name == "main" && ts.is(t + 1, "(")) {
      ts.replace(t, "_st_unused_main");
    }
    else if (is_reserved_ident(name)) {
      ts.replace(t, "_st_" + std::string(name));
    }
    else if ((is_legacy_texture_name(name) || is_desktop_builtin_name(name)) &&
             info.declares(name))
    {
      ts.replace(t, "_st_u_" + std::string(name));
    }
  }
}

struct ArgRange {
  /** First token of the argument, and the `,` or `)` that ends it. */
  int begin;
  int end;
};

/* Arguments of the call whose `(` is at `open`. Returns the index of `)`, or -1. A call
 * inside a `#define` must end inside it. */
static int parse_call_tokens(const TokenStream &ts, const int open, Vector<ArgRange> &r_args)
{
  r_args.clear();
  const bool pp = ts.toks[open].pp;
  int depth = 0;
  int start = open + 1;
  for (int t = open; t < ts.toks.size(); t++) {
    const Tok &tok = ts.toks[t];
    if (tok.pp != pp || (t > open && tok.pp_start)) {
      return -1;
    }
    if (tok.kind != TokKind::Punct) {
      continue;
    }
    const StringRef s = ts.text(t);
    if (s == "(" || s == "[" || s == "{") {
      depth++;
    }
    else if (s == ")" || s == "]" || s == "}") {
      depth--;
      if (depth == 0) {
        r_args.append({start, t});
        return t;
      }
    }
    else if (s == "," && depth == 1) {
      r_args.append({start, t});
      start = t + 1;
    }
  }
  return -1;
}

static std::string arg_text(const TokenStream &ts, const ArgRange &arg)
{
  std::string out;
  for (int t = arg.begin; t < arg.end; t++) {
    const StringRef s = ts.text(t);
    out.append(s.data(), size_t(s.size()));
  }
  return out;
}

static bool arg_is_zero_literal(const TokenStream &ts, const ArgRange &arg)
{
  if (arg.end - arg.begin != 1 || ts.toks[arg.begin].kind != TokKind::Number) {
    return false;
  }
  float v = 1.0f;
  bool is_int = false;
  return parse_literal(ts.text(arg.begin), v, is_int) && v == 0.0f;
}

/* Sampling fixes, all on the call's own tokens:
 *
 * - `textureLod(s, uv, 0.0)` -> `texture(s, uv)`. Vulkan `textureLod(..., 0.0)` on a 1-mip
 *   ping-pong buffer is often black. Chimera's Breath (4tGfDW) samples all 4 neighbors that
 *   way; if those return 0, pressure projection dies.
 * - IQ / Dave Hoskins 3D noise `texture(iChannel0, (uv+0.5)/256.0, -99.0)`: `uv` contains
 *   `floor(x)`, so implicit LOD spikes at every lattice face and samples blurry mips (noise
 *   grid seams in Remnant X, 4sjSW1). `_st_tex2d_lod0` is a wrap-REPEAT texelFetch bilinear.
 * - The `.yx` of that fetch lerps two channels by f.z, which is only continuous if
 *   G(x,y) == R(x-37,y-17). `_st_iq_yx` takes both taps from one channel instead.
 * - Nearest channels: MlVfDR stores voronoi particle IDs as pixel positions and samples
 *   `texture(iChannel, U/R)`. Linear blends two IDs; texelFetch is filter- and lod-proof.
 *   wdsGWS uses `(u+m)*r` with r=1/res and EMPTY==1024 — same idea.
 *
 * Do NOT turn 2-arg `texture()` into `textureLod` globally: fljBWc samples a volume with
 * `texture(iChannel0, p/amplitude)`. */
static void rewrite_texture_calls(TokenStream &ts,
                                  const uint8_t nearest_mask,
                                  const uint8_t wrap_mask,
                                  const uint8_t non_2d_mask)
{
  Vector<ArgRange> args;
  for (int t = 0; t + 1 < ts.toks.size(); t++) {
    if (!ts.is_ident(t) || ts.toks[t].replaced || !ts.is(t + 1, "(") ||
        ts.toks[t].pp != ts.toks[t + 1].pp)
    {
      continue;
    }
    const StringRef name = ts.text(t);
    const bool is_lod = name == "textureLod";
    if (!is_lod && name != "texture" && name != "texture2D") {
      continue;
    }
    if (!ts.toks[t].pp && t > 0 && ts.is_ident(t - 1) && !is_statement_keyword(ts.text(t - 1)))
    {
      continue; /* A declaration of that name, not a call. */
    }
    const int close = parse_call_tokens(ts, t + 1, args);
    if (close < 0) {
      continue;
    }
    int argc = int(args.size());
    if (is_lod) {
      if (argc != 3 || !arg_is_zero_literal(ts, args[2])) {
        continue;
      }
      for (int k = args[1].end; k < close; k++) {
        ts.blank(k);
      }
      ts.replace(t, "texture");
      argc = 2;
    }
    if (argc < 2 || argc > 3) {
      continue;
    }
    const std::string coord = arg_text(ts, args[1]);
    if (looks_like_iq_noise_uv(coord) ||
        (argc == 3 && is_negative_mip_bias(arg_text(ts, args[2]))))
    {
      for (int k = args[1].end; k < close; k++) {
        ts.blank(k);
      }
      const bool yx = close + 2 < ts.toks.size() && ts.is(close + 1, ".") &&
                      ts.is(close + 2, "yx") && ts.toks[close + 2].pp == ts.toks[t].pp &&
                      !ts.toks[close + 1].pp_start;
      if (yx) {
        ts.blank(close + 1);
        ts.blank(close + 2);
      }
      ts.replace(t, yx ? "_st_iq_yx" : "_st_tex2d_lod0");
      continue;
    }
    if (argc != 2 || args[0].end - args[0].begin != 1) {
      continue;
    }
    const StringRef sampler = ts.text(args[0].begin);
    if (!(sampler.size() == 9 && sampler.startswith("iChannel") && sampler[8] >= '0' &&
          sampler[8] <= '3'))
    {
      continue;
    }
    const uint8_t bit = uint8_t(1u << (sampler[8] - '0'));
    if ((nearest_mask & bit) == 0 || (non_2d_mask & bit) != 0) {
      continue;
    }
    const bool repeat = (wrap_mask & bit) != 0;
    /* `expr / R`: fetch at `expr` directly. Only when the division covers the whole
     * coordinate (`a + b/R` does not). */
    bool div_r = args[1].end - args[1].begin >= 3 && ts.is(args[1].end - 1, "R") &&
                 ts.is(args[1].end - 2, "/");
    int nest = 0;
    for (int k = args[1].begin; div_r && k < args[1].end - 2; k++) {
      const StringRef s = ts.text(k);
      if (s == "(" || s == "[") {
        nest++;
      }
      else if (s == ")" || s == "]") {
        nest--;
      }
      else if (nest == 0 && (s == "+" || s == "-")) {
        div_r = false;
      }
    }
    const StringRef coord_ref(coord);
    const bool looks_vec3 = coord_ref.startswith("normalize") || coord_ref.startswith("vec3") ||
                            coord_ref.startswith("reflect") || coord_ref.startswith("refract");
    if (div_r) {
      ts.blank(args[1].end - 2);
      ts.blank(args[1].end - 1);
      ts.replace(t, repeat ? "_st_fetch_px_rep" : "_st_fetch_px");
    }
    else if (!looks_vec3) {
      ts.replace(t, repeat ? "_st_fetch_uv_rep" : "_st_fetch_uv");
    }
  }
}

/* Desktop drivers unroll `for (int i = 0; i < 80; i++)` when 80 is a constant. Nested
 * raymarch + shadow (Saturday cubism, Shane, IQ) then hangs the compiler for minutes or
 * TDR-kills the GPU. ShaderToy/WebGL keeps these as loops; IQ's `ZERO = min(iFrame,0)` is
 * the same idea. `_st_loop_zero()` reads iFrame, so the trip count stays dynamic. */
static void rewrite_loop_zero_starts(TokenStream &ts)
{
  for (int ci = 0; ci + 6 < ts.code.size(); ci++) {
    if (!ts.cis(ci, "for") || !ts.cis(ci + 1, "(") || !ts.c_ident(ci + 3) ||
        !ts.cis(ci + 4, "=") || !ts.cis(ci + 6, ";"))
    {
      continue;
    }
    const bool is_int = ts.cis(ci + 2, "int");
    const bool is_uint = ts.cis(ci + 2, "uint");
    const StringRef zero = ts.ctext(ci + 5);
    if (is_int && zero == "0") {
      ts.replace(ts.code[ci + 5], "_st_loop_zero()");
    }
    else if (is_uint && (zero == "0" || zero == "0u" || zero == "0U")) {
      ts.replace(ts.code[ci + 5], "uint(_st_loop_zero())");
    }
  }
}

/* WebGL zero-fills every variable; desktop Vulkan does not.
 *
 * - Locals: mstfzS does `Particle p0; ApplyForce` / `density +=` with no init. Golf shaders
 *   rely on it everywhere: `vec4 o,p,P,U=vec4(1,2,3,0);`, `for(float i,z,d; ++i<77.;)`
 *   (4D Beats, tfK3Dy) is a billion-iteration TDR when `i` starts as garbage.
 * - Globals: `float light;` then `light +=` (3ccyD7) HDR-blows the image.
 * - `out` parameters: `void mainImage(out vec4 O, vec2 U) { O += ...; }`.
 *
 * Only ` = zero` is added after a declarator that has no initializer. Initialized
 * declarators, arrays and anything a preprocessor branch cuts through are left alone. */
struct ZeroInit {
  TokenStream &ts;
  const SourceInfo &info;

  void append(const int ci, const std::string &text)
  {
    ts.toks[ts.code[ci]].append += text;
  }

  /* `ret name(params) {` at `ci`: zero the `out` parameters at the top of the body. */
  void function_definition(const int ci)
  {
    const int close = ts.c_match(ci + 2);
    if (close < 0 || !ts.cis(close + 1, "{")) {
      return;
    }
    for (int k = ci + 1; k <= close + 1; k++) {
      if (ts.c_after_pp(k)) {
        return;
      }
    }
    std::string inits;
    int a = ci + 3;
    while (a < close) {
      int b = a;
      int nest = 0;
      for (; b < close; b++) {
        const StringRef s = ts.ctext(b);
        if (s == "(" || s == "[") {
          nest++;
        }
        else if (s == ")" || s == "]") {
          nest--;
        }
        else if (s == "," && nest == 0) {
          break;
        }
      }
      int p = a;
      bool is_out = false;
      for (; p < b; p++) {
        const StringRef s = ts.ctext(p);
        if (s == "out") {
          is_out = true;
        }
        else if (!(s == "const" || s == "in" || s == "inout" || is_precision_qualifier(s))) {
          break;
        }
      }
      if (is_out && p + 2 == b && ts.c_ident(p) && ts.c_ident(p + 1)) {
        const std::string name(ts.ctext(p + 1));
        const std::string zero = glsl_zero_for_type(ts.ctext(p), info.struct_zero);
        if (!zero.empty() && !info.macros.contains(name)) {
          inits += " " + name + " = " + zero + ";";
        }
      }
      a = b + 1;
    }
    if (!inits.empty()) {
      this->append(close + 1, inits);
    }
  }

  /* A statement starts at `ci`. */
  void statement(const int ci, const bool file_scope)
  {
    int j = ci;
    if (is_precision_qualifier(ts.ctext(j))) {
      j++;
    }
    if (!ts.c_ident(j)) {
      return;
    }
    if (file_scope && ts.c_ident(j + 1) && ts.cis(j + 2, "(")) {
      this->function_definition(j);
      return;
    }
    const StringRef type = ts.ctext(j);
    if (is_statement_keyword(type)) {
      return;
    }
    const std::string zero = glsl_zero_for_type(type, info.struct_zero);
    if (zero.empty()) {
      return;
    }
    j++;
    const int n = int(ts.code.size());
    Vector<int> names;
    while (true) {
      if (!ts.c_ident(j) || ts.c_after_pp(j)) {
        return;
      }
      const int name = j;
      int k = j + 1;
      bool plain = true;
      /* `vec2 d0, d1;` may reuse a struct's name for a variable, but glslang only parses
       * that while no declarator in the list has an initializer (3tffRH). */
      if (ts.cis(k, "(") || info.struct_names.contains(std::string(ts.ctext(name)))) {
        return;
      }
      while (ts.cis(k, "[")) {
        k = ts.c_match(k);
        if (k < 0) {
          return;
        }
        k++;
        plain = false;
      }
      if (ts.cis(k, "=")) {
        plain = false;
        int nest = 0;
        for (k++;; k++) {
          if (k >= n || ts.c_after_pp(k)) {
            return;
          }
          const StringRef s = ts.ctext(k);
          if (s == "(" || s == "[" || s == "{") {
            nest++;
          }
          else if (s == ")" || s == "]" || s == "}") {
            if (nest == 0) {
              return;
            }
            nest--;
          }
          else if (nest == 0 && (s == "," || s == ";")) {
            break;
          }
        }
      }
      if (k >= n || ts.c_after_pp(k)) {
        return;
      }
      if (plain && !info.macros.contains(std::string(ts.ctext(name)))) {
        names.append(name);
      }
      if (ts.cis(k, ",")) {
        j = k + 1;
        continue;
      }
      if (ts.cis(k, ";")) {
        break;
      }
      return;
    }
    for (const int name : names) {
      this->append(name, " = " + zero);
    }
  }

  void run()
  {
    /* True for a struct body: members cannot have initializers. */
    Vector<bool> braces;
    bool pending_struct = false;
    bool stmt_start = true;
    int paren = 0;
    for (int ci = 0; ci < ts.code.size(); ci++) {
      const StringRef t = ts.ctext(ci);
      const bool start = stmt_start;
      stmt_start = false;
      if (t == "{") {
        braces.append(pending_struct);
        pending_struct = false;
        stmt_start = true;
        continue;
      }
      if (t == "}") {
        if (!braces.is_empty()) {
          braces.pop_last();
        }
        stmt_start = true;
        continue;
      }
      if (t == "(") {
        paren++;
        /* `for (float i, d; ...)`. */
        stmt_start = ts.cis(ci - 1, "for");
        continue;
      }
      if (t == ")") {
        paren = math::max(paren - 1, 0);
        continue;
      }
      if (t == ";") {
        pending_struct = false;
        stmt_start = (paren == 0);
        continue;
      }
      if (t == "struct") {
        pending_struct = true;
        continue;
      }
      if (!start || (!braces.is_empty() && braces.last())) {
        continue;
      }
      this->statement(ci, braces.is_empty() && paren == 0);
    }
  }
};

static const char *fullscreen_vertex_src()
{
  /* Two-triangle clip-space quad. A single oversized triangle (the old -1/-1/3 trick)
   * interpolates across a huge hypoteneuse and shows up as a diagonal seam on Seascape,
   * warp-fBM, and other smooth fields. */
  return "void main()\n"
         "{\n"
         "  int v = gl_VertexID;\n"
         "  vec2 pos;\n"
         "  if (v == 0 || v == 3) {\n"
         "    pos = vec2(-1.0, -1.0);\n"
         "  }\n"
         "  else if (v == 1) {\n"
         "    pos = vec2(1.0, -1.0);\n"
         "  }\n"
         "  else if (v == 2 || v == 4) {\n"
         "    pos = vec2(1.0, 1.0);\n"
         "  }\n"
         "  else {\n"
         "    pos = vec2(-1.0, 1.0);\n"
         "  }\n"
         "  gl_Position = vec4(pos, 0.0, 1.0);\n"
         "}\n";
}

static void replace_all_str(std::string &s, const StringRef from, const StringRef to)
{
  if (from.is_empty()) {
    return;
  }
  const std::string src(from.data(), size_t(from.size()));
  const std::string dst(to.data(), size_t(to.size()));
  size_t pos = 0;
  while ((pos = s.find(src, pos)) != std::string::npos) {
    s.replace(pos, src.size(), dst);
    pos += dst.size();
  }
}

/* Grid SPH (mstfzS) and similar shaders assume WebGL defaults. */
static std::string rewrite_shadertoy_grid_compat(std::string src)
{
  /* Exact float cell tests drop particles on desktop (`10.0` vs `10.000001`). */
  replace_all_str(src,
                  "all(equal(pos, floor(incoming.pos)))",
                  "all(equal(ivec3(floor(pos + 0.5)), ivec3(floor(incoming.pos))))");
  /* unpackParticles never writes force/density; ApplyForce/AddDensity do `+=`. */
  replace_all_str(src,
                  "p1.vel = unpackvec3(data4);",
                  "p1.vel = unpackvec3(data4);\n"
                  "    p0.force = vec3(0.0);\n"
                  "    p0.density = 0.0;\n"
                  "    p1.force = vec3(0.0);\n"
                  "    p1.density = 0.0;");
  /* SCALE.y floors to 0 on small canvases → size3d NaN, particles vanish. */
  replace_all_str(src,
                  "SCALE = floor(ar*pow(iR.x*iR.y,0.1666666));",
                  "SCALE = max(floor(ar*pow(max(iR.x*iR.y, 1.0),0.1666666)), vec2(1.0));");
  replace_all_str(src,
                  "size3d = vec3(floor(iR.xy/SCALE), SCALE.x*SCALE.y);",
                  "size3d = max(floor(vec3(floor(iR.xy/SCALE), SCALE.x*SCALE.y) + 0.5), vec3(1.0));");
  /* If a particle still overshoots, keep it on a valid cell so Clusterize can find it. */
  replace_all_str(src,
                  "p0_.pos += p0_.vel*dt;",
                  "p0_.pos += p0_.vel*dt;\n"
                  "            p0_.pos = clamp(p0_.pos, vec3(0.0), max(size3d - vec3(1.001), vec3(0.0)));");
  replace_all_str(src,
                  "p1_.pos += p1_.vel*dt;",
                  "p1_.pos += p1_.vel*dt;\n"
                  "            p1_.pos = clamp(p1_.pos, vec3(0.0), max(size3d - vec3(1.001), vec3(0.0)));");
  return src;
}

/* Blender prepends `gpu_shader_compat_glsl.glsl` to every shader. Its macros are ordinary
 * ShaderToy names: `#define FLT_MAX 3.4e38` is a "macro redefined" error, `const uint
 * UINT_MAX = ...` and `float select(...)` expand into nonsense, and a variable called
 * `device` or `thread` disappears. None of them are used by the wrapper. */
static void append_blender_macro_undefs(std::string &src)
{
  static const char *names[] = {
      "constexpr",      "bool32_t",       "bool2",          "bool3",          "bool4",
      "float2",         "float3",         "float4",         "int2",           "int3",
      "int4",           "uint2",          "uint3",          "uint4",          "packed_float2",
      "packed_int2",    "packed_uint2",   "packed_float3",  "packed_int3",    "packed_uint3",
      "packed_float4",  "packed_int4",    "packed_uint4",   "float2x2",       "float3x2",
      "float4x2",       "float2x3",       "float3x3",       "float4x3",       "float2x4",
      "float3x4",       "float4x4",       "char",           "char2",          "char3",
      "char4",          "short",          "short2",         "short3",         "short4",
      "uchar",          "uchar2",         "uchar3",         "uchar4",         "ushort",
      "ushort2",        "ushort3",        "ushort4",        "half",           "half2",
      "half3",          "half4",          "int32_t",        "uint32_t",       "imageStoreFast",
      "imageLoadFast",  "sampler2DDepth", "sampler2DArrayDepth", "samplerCubeDepth",
      "samplerCubeArrayDepth", "usampler2DArrayAtomic", "usampler2DAtomic", "usampler3DAtomic",
      "isampler2DArrayAtomic", "isampler2DAtomic", "isampler3DAtomic", "bitfieldInsertAssign",
      "imageFence",     "select",         "float_array",    "float2_array",   "float3_array",
      "float4_array",   "int_array",      "int2_array",     "int3_array",     "int4_array",
      "uint_array",     "uint2_array",    "uint3_array",    "uint4_array",    "bool_array",
      "bool2_array",    "bool3_array",    "bool4_array",    "ARRAY_T",        "ARRAY_V",
      "ATTR_FALLTHROUGH", "static",       "constant",       "device",         "thread",
      "threadgroup",    "textureGather0", "textureGather1", "textureGather2", "textureGather3",
      "saturate",       "GPU_THREAD",     "FLT_MAX",        "FLT_MIN",        "FLT_EPSILON",
      "SHRT_MAX",       "INT_MAX",        "USHRT_MAX",      "UINT_MAX",       "NAN_FLT",
  };
  for (const char *name : names) {
    src += "#undef ";
    src += name;
    src += "\n";
  }
}

/* HLSL-style helpers for pasted code that is not from the site. Only when the shader calls
 * one and does not bring its own: `vec3 saturate(vec3 x)` or a global named `frac` must not
 * meet a second definition. */
static void append_hlsl_helpers(std::string &src, const SourceInfo &info)
{
  auto wanted = [&](const char *name) { return info.calls.contains(name) && !info.declares(name); };
  if (wanted("saturate")) {
    src += "float saturate(float x) { return clamp(x, 0.0, 1.0); }\n";
    src += "vec2 saturate(vec2 x) { return clamp(x, 0.0, 1.0); }\n";
    src += "vec3 saturate(vec3 x) { return clamp(x, 0.0, 1.0); }\n";
    src += "vec4 saturate(vec4 x) { return clamp(x, 0.0, 1.0); }\n";
  }
  if (wanted("lerp")) {
    src += "float lerp(float a, float b, float t) { return mix(a, b, t); }\n";
    src += "vec2 lerp(vec2 a, vec2 b, float t) { return mix(a, b, t); }\n";
    src += "vec2 lerp(vec2 a, vec2 b, vec2 t) { return mix(a, b, t); }\n";
    src += "vec3 lerp(vec3 a, vec3 b, float t) { return mix(a, b, t); }\n";
    src += "vec3 lerp(vec3 a, vec3 b, vec3 t) { return mix(a, b, t); }\n";
    src += "vec4 lerp(vec4 a, vec4 b, float t) { return mix(a, b, t); }\n";
    src += "vec4 lerp(vec4 a, vec4 b, vec4 t) { return mix(a, b, t); }\n";
  }
  if (wanted("frac")) {
    src += "float frac(float x) { return fract(x); }\n";
    src += "vec2 frac(vec2 x) { return fract(x); }\n";
    src += "vec3 frac(vec3 x) { return fract(x); }\n";
    src += "vec4 frac(vec4 x) { return fract(x); }\n";
  }
  if (wanted("atan2")) {
    src += "float atan2(float y, float x) { return atan(y, x); }\n";
    src += "vec2 atan2(vec2 y, vec2 x) { return atan(y, x); }\n";
    src += "vec3 atan2(vec3 y, vec3 x) { return atan(y, x); }\n";
    src += "vec4 atan2(vec4 y, vec4 x) { return atan(y, x); }\n";
  }
  if (wanted("fmod")) {
    src += "float fmod(float a, float b) { return mod(a, b); }\n";
    src += "vec2 fmod(vec2 a, vec2 b) { return mod(a, b); }\n";
    src += "vec3 fmod(vec3 a, vec3 b) { return mod(a, b); }\n";
    src += "vec4 fmod(vec4 a, vec4 b) { return mod(a, b); }\n";
  }
  if (wanted("st_assert")) {
    src += "void st_assert(bool cond) {}\n";
    src += "void st_assert(bool cond, int v) {}\n";
  }
}

static std::string build_fragment(const char *common,
                                  const char *body,
                                  Span<ShaderParam> params,
                                  const bool force_opaque,
                                  const bool cubemap_pass,
                                  const uint8_t cube_mask,
                                  const uint8_t vol_mask,
                                  const uint8_t nearest_mask,
                                  const uint8_t wrap_mask)
{
  std::string common_src = (common && common[0]) ?
                               apply_param_values(rewrite_shadertoy_grid_compat(common), params) :
                               "";
  std::string body_src = apply_param_values(rewrite_shadertoy_grid_compat(body ? body : ""),
                                            params);
  SourceInfo info;
  analyze_source(tokenize(common_src), info);
  analyze_source(tokenize(body_src), info);
  auto translate = [&](std::string &source) {
    if (source.empty()) {
      return;
    }
    {
      TokenStream ts = tokenize(source);
      demote_non_constant_consts(ts, info);
      rewrite_names_and_preamble(ts, info);
      source = ts.str();
    }
    {
      TokenStream ts = tokenize(source);
      rewrite_texture_calls(ts, nearest_mask, wrap_mask, cube_mask | vol_mask);
      source = ts.str();
    }
    {
      TokenStream ts = tokenize(source);
      rewrite_loop_zero_starts(ts);
      ZeroInit{ts, info}.run();
      source = ts.str();
    }
  };
  translate(common_src);
  translate(body_src);

  std::string src;
  src.reserve(8192 + common_src.size() + body_src.size());
  /* GPU_shader_create_from_info_python only injects CREATE_INFO_RES_PASS_pyGPU_Shader,
   * so the ShaderCreateInfo must be named pyGPU_Shader (see gpu_shader.cc). */
  append_blender_macro_undefs(src);
  src += "#define _st_empty\n";
  /* A shader may define these itself (`#define iGlobalTime (iTime / SPEED)`, NdGfzG). */
  auto define = [&](const char *name, const char *value) {
    if (!info.macros.contains(name)) {
      src += "#define ";
      src += name;
      src += " ";
      src += value;
      src += "\n";
    }
  };
  define("iGlobalTime", "iTime");
  for (const auto &legacy : legacy_texture_names) {
    define(legacy[0], legacy[1]);
  }
  define("highp", "");
  define("mediump", "");
  define("lowp", "");
  append_hlsl_helpers(src, info);
  src += "const float iSampleRate = 44100.0;\n";
  /* Push constant is vec4 (std430). ShaderToy iResolution is vec3, and shaders pass it
   * around by that name (`float cone(vec2 uv, vec3 iResolution)`, 3sXSzl), so it has to be
   * a real variable rather than a macro. Globals initialize in order: this one is first. */
  src += "vec3 iResolution = _st_ires.xyz;\n";
  src += "float iChannelTime[4] = float[4](iTime, iTime, iTime, iTime);\n";
  /* ShaderToy canvas is display-referred sRGB. Convert the Image pass so the Texture
   * Editor can apply scene color management (Standard ≈ website, AgX = filmic). */
  src += "vec3 _st_srgb_to_linear(vec3 c)\n";
  src += "{\n";
  src += "  vec3 x = max(c, vec3(0.0));\n";
  src += "  vec3 lo = x / 12.92;\n";
  src += "  vec3 hi = pow((x + 0.055) / 1.055, vec3(2.4));\n";
  src += "  return mix(hi, lo, vec3(lessThanEqual(x, vec3(0.04045))));\n";
  src += "}\n";
  /* Must NOT fold to a literal 0: desktop compilers then unroll 100+ iteration
   * fractals and TDR-kill the process (4s3SRN). iFrame is a push-constant, so
   * the trip count stays dynamic. */
  src += "int _st_loop_zero()\n";
  src += "{\n";
  src += "  return min(iFrame, 0);\n";
  src += "}\n";
  /* Wrap-REPEAT bilinear of mip 0. IQ 3D noise uses floor() in the UV, which spikes
   * implicit LOD; textureLod(0) still picks blurry mips on some Vulkan drivers. */
  src += "vec4 _st_tex2d_lod0(sampler2D samp, vec2 uv)\n";
  src += "{\n";
  src += "  ivec2 sz = textureSize(samp, 0);\n";
  src += "  sz = max(sz, ivec2(1));\n";
  src += "  vec2 dim = vec2(sz);\n";
  src += "  vec2 p = uv * dim - vec2(0.5);\n";
  src += "  vec2 pf = floor(p);\n";
  src += "  vec2 f = p - pf;\n";
  src += "  vec2 s0 = mod(pf, dim);\n";
  src += "  vec2 s1 = mod(pf + vec2(1.0), dim);\n";
  src += "  ivec2 i0 = ivec2(s0);\n";
  src += "  ivec2 i1 = ivec2(s1);\n";
  src += "  vec4 a = texelFetch(samp, i0, 0);\n";
  src += "  vec4 b = texelFetch(samp, ivec2(i1.x, i0.y), 0);\n";
  src += "  vec4 c = texelFetch(samp, ivec2(i0.x, i1.y), 0);\n";
  src += "  vec4 d = texelFetch(samp, i1, 0);\n";
  src += "  return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);\n";
  src += "}\n";
  /* The sampler behind a rewritten call is not always 2D (a helper taking `samplerCube`). */
  src += "vec4 _st_tex2d_lod0(samplerCube samp, vec3 p) { return textureLod(samp, p, 0.0); }\n";
  src += "vec4 _st_tex2d_lod0(sampler3D samp, vec3 p) { return textureLod(samp, p, 0.0); }\n";
  /* IQ packed 3D noise samples ONE bilinear tap and lerps .yx by f.z. Independent RGBA
   * (many user textures) makes every integer-Z plane a hard seam — cube grid on
   * Remnant X's sky. Two taps of the same channel at uv and uv+(37,17) are C0. */
  src += "vec2 _st_iq_yx(sampler2D samp, vec2 uv)\n";
  src += "{\n";
  src += "  return vec2(_st_tex2d_lod0(samp, uv).x,\n";
  src += "              _st_tex2d_lod0(samp, uv + vec2(37.0, 17.0) / 256.0).x);\n";
  src += "}\n";
  src += "vec2 _st_iq_yx(samplerCube samp, vec3 p) { return textureLod(samp, p, 0.0).yx; }\n";
  src += "vec2 _st_iq_yx(sampler3D samp, vec3 p) { return textureLod(samp, p, 0.0).yx; }\n";
  /* Pixel-space nearest (ShaderToy `filter: nearest` on particle buffers). */
  src += "vec4 _st_fetch_px(sampler2D samp, vec2 p)\n";
  src += "{\n";
  src += "  ivec2 sz = textureSize(samp, 0);\n";
  src += "  sz = max(sz, ivec2(1));\n";
  src += "  ivec2 t = ivec2(floor(p));\n";
  src += "  t = clamp(t, ivec2(0), sz - ivec2(1));\n";
  src += "  return texelFetch(samp, t, 0);\n";
  src += "}\n";
  src += "vec4 _st_fetch_px_rep(sampler2D samp, vec2 p)\n";
  src += "{\n";
  src += "  ivec2 sz = textureSize(samp, 0);\n";
  src += "  sz = max(sz, ivec2(1));\n";
  src += "  vec2 q = p - vec2(sz) * floor(p / vec2(sz));\n";
  src += "  ivec2 t = ivec2(q);\n";
  src += "  t = clamp(t, ivec2(0), sz - ivec2(1));\n";
  src += "  return texelFetch(samp, t, 0);\n";
  src += "}\n";
  src += "vec4 _st_fetch_uv(sampler2D samp, vec2 uv)\n";
  src += "{\n";
  src += "  ivec2 sz = textureSize(samp, 0);\n";
  src += "  sz = max(sz, ivec2(1));\n";
  src += "  ivec2 t = ivec2(floor(uv * vec2(sz)));\n";
  src += "  t = clamp(t, ivec2(0), sz - ivec2(1));\n";
  src += "  return texelFetch(samp, t, 0);\n";
  src += "}\n";
  src += "vec4 _st_fetch_uv_rep(sampler2D samp, vec2 uv)\n";
  src += "{\n";
  src += "  ivec2 sz = textureSize(samp, 0);\n";
  src += "  sz = max(sz, ivec2(1));\n";
  src += "  vec2 p = uv * vec2(sz);\n";
  src += "  p -= vec2(sz) * floor(p / vec2(sz));\n";
  src += "  ivec2 t = ivec2(p);\n";
  src += "  t = clamp(t, ivec2(0), sz - ivec2(1));\n";
  src += "  return texelFetch(samp, t, 0);\n";
  src += "}\n";
  /* Declared BEFORE the user shader. Common tabs often
   * `#define textureSize iResolution.xy` (7tSSDD); that turns a later
   * `textureSize(iChannel0, 0)` into `iResolution.xy(...)`. */
  src += "vec3 iChannelResolution[4] = vec3[4](\n";
  for (int ch = 0; ch < 4; ch++) {
    char line[192];
    if (cube_mask & (1u << ch)) {
      SNPRINTF(line,
               "  vec3(vec2(textureSize(iChannel%d, 0)), float(textureSize(iChannel%d, 0).x))",
               ch,
               ch);
    }
    else if (vol_mask & (1u << ch)) {
      SNPRINTF(line, "  vec3(textureSize(iChannel%d, 0))", ch);
    }
    else {
      SNPRINTF(line, "  vec3(vec2(textureSize(iChannel%d, 0)), 1.0)", ch);
    }
    src += line;
    src += (ch < 3) ? ",\n" : ");\n";
  }
  /* Like the site, `main` goes in front of the user code and reaches the entry point
   * through a prototype. User macros cannot touch it there: golfed shaders `#define a 2.0`
   * (M3B3WK), and a common anti-aliasing wrapper is `#define mainImage mainImage0(...);
   * void mainImage(...) {...} void mainImage0` (DsByzG), which would also expand a call
   * placed after it. */
  if (cubemap_pass) {
    src += "void mainCubemap(out vec4 c, in vec2 f, in vec3 ro, in vec3 rd);\n";
  }
  else {
    src += "void mainImage(out vec4 c, in vec2 f);\n";
  }
  src += "void main()\n";
  src += "{\n";
  src += "  vec4 color = vec4(0.0, 0.0, 0.0, 1.0);\n";
  if (cubemap_pass) {
    /* ShaderToy Cubemap A: mainCubemap(..., rayOri, rayDir). Face layout matches GL/Vulkan. */
    src += "  vec2 uv = gl_FragCoord.xy / _st_ires.xy * 2.0 - 1.0;\n";
    src += "  vec3 rd;\n";
    src += "  if (iCubeFace == 0) { rd = vec3(1.0, -uv.y, -uv.x); }\n";
    src += "  else if (iCubeFace == 1) { rd = vec3(-1.0, -uv.y, uv.x); }\n";
    src += "  else if (iCubeFace == 2) { rd = vec3(uv.x, 1.0, uv.y); }\n";
    src += "  else if (iCubeFace == 3) { rd = vec3(uv.x, -1.0, -uv.y); }\n";
    src += "  else if (iCubeFace == 4) { rd = vec3(uv.x, -uv.y, 1.0); }\n";
    src += "  else { rd = vec3(-uv.x, -uv.y, -1.0); }\n";
    src += "  mainCubemap(color, gl_FragCoord.xy, vec3(0.0), normalize(rd));\n";
  }
  else {
    src += "  mainImage(color, gl_FragCoord.xy);\n";
  }
  /* Image pass: ShaderToy writes display-referred sRGB (browser canvas). Convert
   * to scene-linear so the *blend file* view transform applies (Standard ≈ site,
   * AgX/Filmic = that file's look). Do not tag Non-Color — that locks CM.
   * Buffer passes MUST keep raw bits (camera packing / uintBitsToFloat). */
  if (force_opaque) {
    src += "  if (any(isnan(color))) {\n";
    src += "    color = vec4(0.0, 0.0, 0.0, 1.0);\n";
    src += "  }\n";
    src += "  color.rgb = _st_srgb_to_linear(color.rgb);\n";
    src += "  color.a = 1.0;\n";
  }
  src += "  out_fragColor = color;\n";
  src += "}\n";
  src += "\n";
  if (!common_src.empty()) {
    src += common_src;
    src += "\n";
  }
  src += body_src;
  src += "\n";
  /* Older golfed shaders have no function at all: `#define mainImage(O,U) <statements>`,
   * expanded by the call the site used to place after the code (Md2BWh). Expand it into a
   * real entry point. Decided by the preprocessor, as such a macro often sits in an `#if`
   * next to a regular `void mainImage`. */
  if (!cubemap_pass && info.macros_with_params.contains("mainImage")) {
    src += "#ifdef mainImage\n";
    src += "void _st_macro_main(inout vec4 _st_o, in vec2 _st_u)\n";
    src += "{\n";
    src += "  mainImage(_st_o, _st_u);\n";
    src += "}\n";
    src += "#undef mainImage\n";
    src += "void mainImage(out vec4 c, in vec2 f)\n";
    src += "{\n";
    src += "  c = vec4(0.0, 0.0, 0.0, 1.0);\n";
    src += "  _st_macro_main(c, f);\n";
    src += "}\n";
    src += "#endif\n";
  }
  return src;
}

/* -------------------------------------------------------------------- */
/** \name Cubemap iChannel detection
 *
 * ShaderToy cubemaps use `texture(iChannelN, vec3)` / `textureCube`. Declaring those
 * as sampler2D fails to compile (`no matching overloaded function found`).
 * \{ */

static bool match_ident_at(const StringRef src, const int64_t pos, const StringRef ident)
{
  if (pos < 0 || pos + ident.size() > src.size()) {
    return false;
  }
  if (std::memcmp(src.data() + pos, ident.data(), size_t(ident.size())) != 0) {
    return false;
  }
  const char before = (pos > 0) ? src[pos - 1] : ' ';
  const int64_t after_i = pos + ident.size();
  const char after = (after_i < src.size()) ? src[after_i] : ' ';
  return !is_ident_char_st(before) && !is_ident_char_st(after);
}

static int64_t skip_ws_st(const StringRef src, int64_t i)
{
  while (i < src.size() && std::isspace(static_cast<unsigned char>(src[i]))) {
    i++;
  }
  return i;
}

static bool type_starts_at(const StringRef expr, const char *type)
{
  const int64_t n = int64_t(strlen(type));
  if (expr.size() < n) {
    return false;
  }
  if (std::memcmp(expr.data(), type, size_t(n)) != 0) {
    return false;
  }
  return expr.size() == n || !is_ident_char_st(expr[n]);
}

static bool parse_call_args(const StringRef src, const int64_t open_paren, Vector<StringRef> &args)
{
  args.clear();
  if (open_paren < 0 || open_paren >= src.size() || src[open_paren] != '(') {
    return false;
  }
  int depth = 1;
  int64_t start = open_paren + 1;
  for (int64_t i = open_paren + 1; i < src.size() && depth > 0; i++) {
    const char c = src[i];
    if (c == '/' && i + 1 < src.size() && src[i + 1] == '/') {
      while (i < src.size() && src[i] != '\n') {
        i++;
      }
      continue;
    }
    if (c == '(') {
      depth++;
    }
    else if (c == ')') {
      depth--;
      if (depth == 0) {
        args.append(rtrim(ltrim(src.substr(start, i - start))));
        return true;
      }
    }
    else if (c == ',' && depth == 1) {
      args.append(rtrim(ltrim(src.substr(start, i - start))));
      start = i + 1;
    }
  }
  return false;
}

static int64_t find_call_paren(const StringRef src, const StringRef func, int64_t from)
{
  while (from < src.size()) {
    const int64_t hit = src.find(func, from);
    if (hit == StringRef::not_found) {
      return StringRef::not_found;
    }
    if (match_ident_at(src, hit, func)) {
      const int64_t q = skip_ws_st(src, hit + func.size());
      if (q < src.size() && src[q] == '(') {
        return q;
      }
    }
    from = hit + 1;
  }
  return StringRef::not_found;
}

static bool arg_is_ichannel(StringRef arg, const int channel)
{
  arg = rtrim(ltrim(arg));
  while (arg.size() >= 2 && arg[0] == '(' && arg[arg.size() - 1] == ')') {
    arg = rtrim(ltrim(arg.substr(1, arg.size() - 2)));
  }
  char name[16];
  SNPRINTF(name, "iChannel%d", channel);
  if (arg == StringRef(name)) {
    return true;
  }
  /* Common tab often `#define ch0 iChannel0` (mstfzS LOAD3D). */
  SNPRINTF(name, "ch%d", channel);
  return arg == StringRef(name);
}

static bool ident_declared_as(const StringRef src, const StringRef name, const char *type)
{
  const StringRef t(type);
  int64_t p = 0;
  while (p < src.size()) {
    const int64_t hit = src.find(t, p);
    if (hit == StringRef::not_found) {
      return false;
    }
    if (!match_ident_at(src, hit, t)) {
      p = hit + 1;
      continue;
    }
    int64_t q = skip_ws_st(src, hit + t.size());
    while (q < src.size()) {
      q = skip_ws_st(src, q);
      if (q >= src.size() || !is_ident_start(src[q])) {
        break;
      }
      int64_t e = q + 1;
      while (e < src.size() && is_ident_char_st(src[e])) {
        e++;
      }
      if (src.substr(q, e - q) == name) {
        return true;
      }
      q = skip_ws_st(src, e);
      if (q < src.size() && src[q] == ',') {
        q++;
        continue;
      }
      break;
    }
    p = hit + 1;
  }
  return false;
}

static int infer_coord_dim(const StringRef src, StringRef expr, const int rec = 0)
{
  if (rec > 24) {
    return 0;
  }
  expr = rtrim(ltrim(expr));
  if (expr.is_empty()) {
    return 0;
  }
  while (expr.size() >= 2 && expr[0] == '(') {
    int depth = 0;
    bool wraps = true;
    for (int64_t i = 0; i < expr.size(); i++) {
      if (expr[i] == '(') {
        depth++;
      }
      else if (expr[i] == ')') {
        depth--;
        if (depth == 0 && i + 1 < expr.size()) {
          wraps = false;
          break;
        }
      }
    }
    if (wraps && depth == 0) {
      expr = rtrim(ltrim(expr.substr(1, expr.size() - 2)));
    }
    else {
      break;
    }
  }
  if (expr.is_empty()) {
    return 0;
  }
  /* `p/amplitude` — use the left operand's dimension (Shane 4scXzn). */
  {
    int depth = 0;
    int64_t op = -1;
    for (int64_t i = 0; i < expr.size(); i++) {
      if (expr[i] == '(') {
        depth++;
      }
      else if (expr[i] == ')') {
        depth--;
      }
      else if (depth == 0 && (expr[i] == '/' || expr[i] == '*' || expr[i] == '+' || expr[i] == '-'))
      {
        if (i > 0) {
          op = i;
          break;
        }
      }
    }
    if (op > 0) {
      const int left = infer_coord_dim(src, expr.substr(0, op), rec + 1);
      if (left != 0) {
        return left;
      }
    }
  }
  if (type_starts_at(expr, "vec2") || type_starts_at(expr, "ivec2")) {
    return 2;
  }
  if (type_starts_at(expr, "vec3") || type_starts_at(expr, "ivec3")) {
    return 3;
  }
  if (type_starts_at(expr, "reflect") || type_starts_at(expr, "refract") ||
      type_starts_at(expr, "cross"))
  {
    return 3;
  }
  if (type_starts_at(expr, "normalize")) {
    const int64_t paren = expr.find('(');
    if (paren != StringRef::not_found) {
      Vector<StringRef> inner;
      if (parse_call_args(expr, paren, inner) && !inner.is_empty()) {
        return infer_coord_dim(src, inner[0], rec + 1);
      }
    }
  }

  int depth = 0;
  int64_t last_dot = -1;
  for (int64_t i = 0; i < expr.size(); i++) {
    if (expr[i] == '(') {
      depth++;
    }
    else if (expr[i] == ')') {
      depth--;
    }
    else if (expr[i] == '.' && depth == 0) {
      last_dot = i;
    }
  }
  if (last_dot >= 0) {
    const StringRef sw = expr.substr(last_dot + 1);
    auto is_swizzle = [](const char c) {
      return c == 'x' || c == 'y' || c == 'z' || c == 'w' || c == 'r' || c == 'g' || c == 'b' ||
             c == 'a' || c == 's' || c == 't' || c == 'p' || c == 'q';
    };
    int n = 0;
    while (n < sw.size() && is_swizzle(sw[n])) {
      n++;
    }
    if (n >= 1 && n <= 4 && rtrim(ltrim(sw.substr(n))).is_empty()) {
      return n;
    }
  }

  if (is_ident_start(expr[0])) {
    int64_t n = 1;
    while (n < expr.size() && is_ident_char_st(expr[n])) {
      n++;
    }
    if (n == expr.size()) {
      const StringRef name = expr.substr(0, n);
      const bool as2 = ident_declared_as(src, name, "vec2") || ident_declared_as(src, name, "ivec2");
      const bool as3 = ident_declared_as(src, name, "vec3") || ident_declared_as(src, name, "ivec3");
      if (as3 && !as2) {
        return 3;
      }
      if (as2 && !as3) {
        return 2;
      }
      /* `rf = reflect(rd, n); texture(iChannel1, rf)` without a nearby vec3 decl. */
      int64_t p = 0;
      int hits = 0;
      while (p < src.size() && hits++ < 256) {
        const int64_t hit = src.find(name, p);
        if (hit == StringRef::not_found) {
          break;
        }
        if (match_ident_at(src, hit, name)) {
          int64_t q = skip_ws_st(src, hit + name.size());
          if (q < src.size() && src[q] == '=' && (q + 1 >= src.size() || src[q + 1] != '=')) {
            q = skip_ws_st(src, q + 1);
            const StringRef rhs = src.substr(q);
            if (type_starts_at(rhs, "reflect") || type_starts_at(rhs, "refract") ||
                type_starts_at(rhs, "cross"))
            {
              return 3;
            }
          }
        }
        p = hit + 1;
      }
    }
  }
  return 0;
}

static bool ident_looks_like_dir(const StringRef name)
{
  if (name == "rd" || name == "ray" || name == "rayDir" || name == "raydir" || name == "rdir" ||
      name == "vdir" || name == "dir" || name == "rf" || name == "n" || name == "nor" ||
      name == "sn" || name == "refl" || name == "refl_d" || name == "view" || name == "eye")
  {
    return true;
  }
  if (name.size() >= 4 && (name.endswith("Dir") || name.endswith("dir") || name.endswith("_rd"))) {
    return true;
  }
  return name.startswith("refl");
}

static StringRef strip_trailing_swizzle(StringRef expr)
{
  expr = rtrim(ltrim(expr));
  int depth = 0;
  int64_t last_dot = -1;
  for (int64_t i = 0; i < expr.size(); i++) {
    if (expr[i] == '(') {
      depth++;
    }
    else if (expr[i] == ')') {
      depth--;
    }
    else if (expr[i] == '.' && depth == 0) {
      last_dot = i;
    }
  }
  if (last_dot <= 0) {
    return expr;
  }
  const StringRef sw = expr.substr(last_dot + 1);
  auto is_swizzle = [](const char c) {
    return c == 'x' || c == 'y' || c == 'z' || c == 'w' || c == 'r' || c == 'g' || c == 'b' ||
           c == 'a' || c == 's' || c == 't' || c == 'p' || c == 'q';
  };
  int n = 0;
  while (n < sw.size() && is_swizzle(sw[n])) {
    n++;
  }
  /* `rd.yzx` (mstfzS cubemap) — permutation of a vec3 direction. */
  if (n == 3 && rtrim(ltrim(sw.substr(n))).is_empty()) {
    return rtrim(expr.substr(0, last_dot));
  }
  return expr;
}

static bool coord_looks_like_direction(const StringRef src, StringRef expr)
{
  expr = rtrim(ltrim(expr));
  expr = strip_trailing_swizzle(expr);
  if (type_starts_at(expr, "reflect") || type_starts_at(expr, "refract") ||
      type_starts_at(expr, "cross"))
  {
    return true;
  }
  if (type_starts_at(expr, "normalize") || type_starts_at(expr, "getRay") ||
      type_starts_at(expr, "get_ray"))
  {
    const int64_t paren = expr.find('(');
    if (paren != StringRef::not_found) {
      Vector<StringRef> inner;
      if (parse_call_args(expr, paren, inner) && !inner.is_empty()) {
        if (type_starts_at(expr, "getRay") || type_starts_at(expr, "get_ray")) {
          return true;
        }
        if (coord_looks_like_direction(src, inner[0])) {
          return true;
        }
        /* 4s3GDB: `texture(iChannel1, normalize(vec3(sin,cos,sin)))` is a
         * cubemap lookup. Treating it as volume sampled 3D noise. */
        if (type_starts_at(expr, "normalize") && infer_coord_dim(src, inner[0]) == 3) {
          return true;
        }
        return false;
      }
    }
    if (type_starts_at(expr, "getRay") || type_starts_at(expr, "get_ray")) {
      return true;
    }
  }
  if (ident_looks_like_dir(expr)) {
    return true;
  }
  if (!expr.is_empty() && is_ident_start(expr[0])) {
    int64_t n = 1;
    while (n < expr.size() && is_ident_char_st(expr[n])) {
      n++;
    }
    if (n == expr.size()) {
      const StringRef name = expr.substr(0, n);
      int64_t p = 0;
      int hits = 0;
      while (p < src.size() && hits++ < 256) {
        const int64_t hit = src.find(name, p);
        if (hit == StringRef::not_found) {
          break;
        }
        if (match_ident_at(src, hit, name)) {
          int64_t q = skip_ws_st(src, hit + name.size());
          if (q < src.size() && src[q] == '=' && (q + 1 >= src.size() || src[q + 1] != '=')) {
            q = skip_ws_st(src, q + 1);
            const StringRef rhs = src.substr(q);
            if (type_starts_at(rhs, "reflect") || type_starts_at(rhs, "refract") ||
                type_starts_at(rhs, "cross") || type_starts_at(rhs, "getRay") ||
                type_starts_at(rhs, "get_ray"))
            {
              return true;
            }
          }
        }
        p = hit + 1;
      }
    }
  }
  return infer_coord_dim(src, expr) == 3 &&
         (expr.find("reflect") != StringRef::not_found ||
          expr.find("refract") != StringRef::not_found);
}

static std::string function_name_before_open_paren(StringRef src, const int64_t open_paren)
{
  int64_t q = open_paren;
  while (q > 0 && std::isspace(static_cast<unsigned char>(src[q - 1]))) {
    q--;
  }
  const int64_t end = q;
  while (q > 0 && is_ident_char_st(src[q - 1])) {
    q--;
  }
  if (q >= end || !is_ident_start(src[q])) {
    return {};
  }
  return std::string(src.data() + q, size_t(end - q));
}

/* WdGcDt wraps cubemap lookups in `tx(samplerCube, vec2, int)` / `txFace1(...)`.
 * Those never call `texture(iChannelN, vec3)` directly, so the old detector
 * left iChannel0 as sampler2D and the helper overloads failed to match. */
static void collect_funcs_with_typed_sampler_param(StringRef src,
                                                   Set<std::string> &cube_funcs,
                                                   Set<std::string> &vol_funcs)
{
  auto harvest = [&](const char *stype, Set<std::string> &out) {
    const StringRef key(stype);
    int64_t from = 0;
    while (from < src.size()) {
      const int64_t hit = src.find(key, from);
      if (hit == StringRef::not_found) {
        break;
      }
      from = hit + key.size();
      const char before = (hit > 0) ? src[hit - 1] : ' ';
      const char after = (from < src.size()) ? src[from] : ' ';
      if (is_ident_char_st(before) || is_ident_char_st(after)) {
        continue;
      }
      int64_t q = hit;
      while (q > 0 && std::isspace(static_cast<unsigned char>(src[q - 1]))) {
        q--;
      }
      auto skip_qual = [&](const char *w) {
        const int n = int(strlen(w));
        if (q < n) {
          return;
        }
        if (src.substr(q - n, n) != StringRef(w)) {
          return;
        }
        if (q > n && is_ident_char_st(src[q - n - 1])) {
          return;
        }
        q -= n;
        while (q > 0 && std::isspace(static_cast<unsigned char>(src[q - 1]))) {
          q--;
        }
      };
      skip_qual("inout");
      skip_qual("in");
      if (q <= 0 || src[q - 1] != '(') {
        continue;
      }
      const std::string fname = function_name_before_open_paren(src, q - 1);
      if (fname.empty() || fname == "texture" || fname == "textureLod" || fname == "textureGrad" ||
          fname == "textureCube" || fname == "texelFetch" || fname == "textureSize")
      {
        continue;
      }
      out.add(fname);
    }
  };
  harvest("samplerCube", cube_funcs);
  harvest("sampler3D", vol_funcs);
}

static void detect_sampler_masks(const char *common,
                                 const char *body,
                                 uint8_t &cube_mask,
                                 uint8_t &vol_mask)
{
  cube_mask = 0;
  vol_mask = 0;
  std::string joined;
  joined.reserve((common ? strlen(common) : 0) + (body ? strlen(body) : 0) + 2);
  if (common && common[0]) {
    joined += common;
    joined += '\n';
  }
  if (body && body[0]) {
    joined += body;
  }
  const StringRef src(joined);
  Set<std::string> cube_funcs;
  Set<std::string> vol_funcs;
  collect_funcs_with_typed_sampler_param(src, cube_funcs, vol_funcs);
  const char *funcs[] = {"textureCube", "textureLod", "textureGrad", "texture2D", "texture"};
  for (int ch = 0; ch < 4; ch++) {
    bool has_2d = false;
    bool has_dir = false;
    bool has_pos3 = false;
    bool has_cube_fn = false;
    bool helper_cube = false;
    bool helper_vol = false;
    auto scan_helpers = [&](const Set<std::string> &names, bool &flag) {
      for (const std::string &fname : names) {
        int64_t from = 0;
        int guard = 0;
        while (guard++ < 4096) {
          const int64_t paren = find_call_paren(src, fname, from);
          if (paren == StringRef::not_found) {
            break;
          }
          Vector<StringRef> args;
          if (parse_call_args(src, paren, args) && !args.is_empty() &&
              arg_is_ichannel(args[0], ch))
          {
            flag = true;
            return;
          }
          from = paren + 1;
        }
      }
    };
    scan_helpers(cube_funcs, helper_cube);
    scan_helpers(vol_funcs, helper_vol);
    for (const char *func : funcs) {
      int64_t from = 0;
      int guard = 0;
      while (guard++ < 4096) {
        const int64_t paren = find_call_paren(src, func, from);
        if (paren == StringRef::not_found) {
          break;
        }
        Vector<StringRef> args;
        if (parse_call_args(src, paren, args) && args.size() >= 2 && arg_is_ichannel(args[0], ch))
        {
          /* `#define TEX(uv) texture(iChannel0, uv)` in Common is a macro, not a
           * real 2D sample. Counting it as 2D blocked volume (fljBWc fbm). */
          int64_t line = paren;
          while (line > 0 && src[line - 1] != '\n') {
            line--;
          }
          while (line < src.size() && (src[line] == ' ' || src[line] == '\t')) {
            line++;
          }
          if (line + 7 <= src.size() && src.substr(line, 7) == "#define") {
            from = paren + 1;
            continue;
          }
          if (std::strcmp(func, "textureCube") == 0) {
            has_cube_fn = true;
          }
          else {
            const int dim = infer_coord_dim(src, args[1]);
            if (dim == 2) {
              has_2d = true;
            }
            else if (dim == 3) {
              if (coord_looks_like_direction(src, args[1])) {
                has_dir = true;
              }
              else {
                has_pos3 = true;
              }
            }
            else {
              /* Unknown (`p/amplitude` after failed infer): treat as 2D so we don't
               * promote the sampler to cube and break vec2 overloads (4scXzn). */
              has_2d = true;
            }
          }
        }
        from = paren + 1;
      }
    }
    if (helper_cube || ((has_cube_fn || has_dir) && !has_2d)) {
      cube_mask |= uint8_t(1u << ch);
    }
    else if ((helper_vol || has_pos3) && !has_2d && !has_cube_fn && !has_dir) {
      vol_mask |= uint8_t(1u << ch);
    }
  }
}

static uint8_t detect_texelfetch_mask(const char *common, const char *body)
{
  std::string joined;
  if (common && common[0]) {
    joined += common;
    joined += '\n';
  }
  if (body && body[0]) {
    joined += body;
  }
  const StringRef src(joined);
  uint8_t mask = 0;
  int64_t from = 0;
  while (from < src.size()) {
    const int64_t paren = find_call_paren(src, "texelFetch", from);
    if (paren == StringRef::not_found) {
      break;
    }
    Vector<StringRef> args;
    if (parse_call_args(src, paren, args) && !args.is_empty()) {
      for (int ch = 0; ch < 4; ch++) {
        if (arg_is_ichannel(args[0], ch)) {
          mask |= uint8_t(1u << ch);
        }
      }
    }
    from = paren + 1;
  }
  return mask;
}

/* Comments blanked out, so `// iChannel1: keyboard` does not count as a use. */
static std::string blank_glsl_comments(const StringRef src)
{
  std::string out(src.data(), size_t(src.size()));
  const size_t n = out.size();
  for (size_t i = 0; i + 1 < n; i++) {
    if (out[i] == '/' && out[i + 1] == '/') {
      while (i < n && out[i] != '\n') {
        out[i++] = ' ';
      }
    }
    else if (out[i] == '/' && out[i + 1] == '*') {
      while (i + 1 < n && !(out[i] == '*' && out[i + 1] == '/')) {
        if (out[i] != '\n') {
          out[i] = ' ';
        }
        i++;
      }
      if (i + 1 < n) {
        out[i] = ' ';
        out[i + 1] = ' ';
        i++;
      }
    }
  }
  return out;
}

/* `ivec2(KEY, row)`: a key code and row 0-2 (down / pressed / toggle) of the keyboard
 * texture. Small literals are left alone: `ivec2(0, 0)` is how shaders read their own state. */
static bool is_keyboard_texel(StringRef coord)
{
  coord = rtrim(ltrim(coord));
  if (!type_starts_at(coord, "ivec2") || !coord.endswith(")")) {
    return false;
  }
  const int64_t paren = coord.find('(');
  Vector<StringRef> xy;
  if (paren == StringRef::not_found || coord.find('(', paren + 1) != StringRef::not_found ||
      !parse_call_args(coord, paren, xy) || xy.size() != 2)
  {
    return false;
  }
  const StringRef key = xy[0];
  const StringRef row = xy[1];
  if (key.is_empty() || !(row == "0" || row == "1" || row == "2")) {
    return false;
  }
  bool digits = true;
  bool ident = is_ident_start(key[0]);
  for (const char c : key) {
    digits &= std::isdigit(static_cast<unsigned char>(c)) != 0;
    ident &= is_ident_char_st(c);
  }
  if (digits) {
    const int code = (key.size() <= 3) ? std::atoi(std::string(key).c_str()) : 0;
    return code >= 8 && code <= 255;
  }
  if (!ident) {
    return false;
  }
  std::string lower(key.data(), size_t(key.size()));
  for (char &c : lower) {
    c = char(std::tolower(static_cast<unsigned char>(c)));
  }
  return lower.find("key") != std::string::npos;
}

static uint8_t detect_keyboard_mask(const char *common, const char *body)
{
  std::string joined;
  if (common && common[0]) {
    joined += common;
    joined += '\n';
  }
  if (body && body[0]) {
    joined += body;
  }
  const StringRef src(joined);
  uint8_t mask = 0;
  const char *funcs[] = {"texture", "texture2D"};
  for (const char *func : funcs) {
    int64_t from = 0;
    while (from < src.size()) {
      const int64_t paren = find_call_paren(src, func, from);
      if (paren == StringRef::not_found) {
        break;
      }
      Vector<StringRef> args;
      if (parse_call_args(src, paren, args) && args.size() >= 2) {
        if (looks_like_keyboard_uv(args[1])) {
          for (int ch = 0; ch < 4; ch++) {
            if (arg_is_ichannel(args[0], ch)) {
              mask |= uint8_t(1u << ch);
            }
          }
        }
      }
      from = paren + 1;
    }
  }

  /* Modern shaders read keys with `texelFetch(iChannelN, ivec2(KEY, row), 0)`. When that is
   * the only thing a pass does with a channel, the channel is the keyboard. Pasted code has
   * no input list, and auto-wiring such a channel to Buffer N reads that buffer as key
   * state: a terrain demo's "Enter toggles erosion" flipped with Buffer B's scrolling noise. */
  const std::string code = blank_glsl_comments(src);
  const StringRef code_ref(code);
  for (int ch = 0; ch < 4; ch++) {
    char name[16];
    SNPRINTF(name, "iChannel%d", ch);
    int uses = 0;
    for (int64_t at = code_ref.find(name); at != StringRef::not_found;
         at = code_ref.find(name, at + 1))
    {
      uses += match_ident_at(code_ref, at, name) ? 1 : 0;
    }
    int key_reads = 0;
    int64_t from = 0;
    while (uses > 0 && from < code_ref.size()) {
      const int64_t paren = find_call_paren(code_ref, "texelFetch", from);
      if (paren == StringRef::not_found) {
        break;
      }
      Vector<StringRef> args;
      if (parse_call_args(code_ref, paren, args) && args.size() == 3 &&
          args[0] == StringRef(name) && args[2] == "0" && is_keyboard_texel(args[1]))
      {
        key_reads++;
      }
      from = paren + 1;
    }
    if (key_reads > 0 && key_reads == uses) {
      mask |= uint8_t(1u << ch);
    }
  }
  return mask;
}

static uint8_t detect_texture_like_mask(const char *common, const char *body)
{
  /* Site media (organic / RGBA noise / triplanar), as opposed to a Buffer pass.
   * Image should keep using Buffer A for `texture(iChannel0, fragCoord/res)`. */
  std::string joined;
  if (common && common[0]) {
    joined += common;
    joined += '\n';
  }
  if (body && body[0]) {
    joined += body;
  }
  const StringRef src(joined);
  uint8_t mask = 0;
  const char *helpers[] = {"tex3D",
                           "texcube",
                           "texCube",
                           "triplanar",
                           "tx",
                           "txFace0",
                           "txFace1",
                           "texMapCh",
                           "texMapSmoothCh",
                           "txChSm"};
  for (const char *func : helpers) {
    int64_t from = 0;
    while (from < src.size()) {
      const int64_t paren = find_call_paren(src, func, from);
      if (paren == StringRef::not_found) {
        break;
      }
      Vector<StringRef> args;
      if (parse_call_args(src, paren, args) && !args.is_empty()) {
        for (int ch = 0; ch < 4; ch++) {
          if (arg_is_ichannel(args[0], ch)) {
            mask |= uint8_t(1u << ch);
          }
        }
      }
      from = paren + 1;
    }
  }
  const char *funcs[] = {"texture", "textureLod", "texture2D", "textureGrad", "texture2DLod"};
  for (int ch = 0; ch < 4; ch++) {
    if (mask & (1u << ch)) {
      continue;
    }
    for (const char *func : funcs) {
      int64_t from = 0;
      bool hit = false;
      while (!hit) {
        const int64_t paren = find_call_paren(src, func, from);
        if (paren == StringRef::not_found) {
          break;
        }
        Vector<StringRef> args;
        if (parse_call_args(src, paren, args) && args.size() >= 2 && arg_is_ichannel(args[0], ch))
        {
          const StringRef coord = args[1];
          const bool noise256 = coord.find("256") != StringRef::not_found;
          const bool ch_res = coord.find("iChannelResolution") != StringRef::not_found;
          const bool triplanar = coord.find(".yz") != StringRef::not_found ||
                                 coord.find(".zx") != StringRef::not_found ||
                                 coord.find(".xz") != StringRef::not_found ||
                                 coord.find(".zy") != StringRef::not_found;
          if (noise256 || ch_res || triplanar) {
            mask |= uint8_t(1u << ch);
            hit = true;
          }
        }
        from = paren + 1;
      }
      if (hit) {
        break;
      }
    }
  }
  return mask;
}

static bool is_cubemap_only_pass(const char *body)
{
  if (body == nullptr || body[0] == '\0') {
    return false;
  }
  const StringRef s(body);
  if (s.find("mainCubemap") == StringRef::not_found) {
    return false;
  }
  if (s.find("mainImage") == StringRef::not_found) {
    return true;
  }
  /* Both words appear. A comment that mentions the other entry point does not count. */
  const TokenStream ts = tokenize(s);
  bool cubemap = false;
  for (int ci = 0; ci + 1 < ts.code.size(); ci++) {
    if (!ts.cis(ci + 1, "(")) {
      continue;
    }
    if (ts.cis(ci, "mainImage")) {
      return false;
    }
    cubemap |= ts.cis(ci, "mainCubemap");
  }
  return cubemap;
}

/* Cube-map passes (Dry Ice 2 Cubemap A) use mainCubemap. 2D Image/Buffer keep mainImage. */
static std::string ensure_main_image(const char *body)
{
  std::string s = body ? body : "";
  if (s.find("mainImage") != std::string::npos || s.find("mainCubemap") != std::string::npos) {
    return s;
  }
  return s;
}

/** \} */

struct CompiledPass {
  gpu::Shader *shader = nullptr;
  std::string error;
  uint8_t cube_mask = 0;
  uint8_t vol_mask = 0;
  uint8_t fetch_mask = 0;
  uint8_t tex_like_mask = 0;
  bool cubemap_pass = false;
};

static Mutex &shader_cache_mutex()
{
  static Mutex mutex;
  return mutex;
}

static Map<uint64_t, CompiledPass> &shader_cache()
{
  static Map<uint64_t, CompiledPass> map;
  return map;
}

static gpu::Texture *&noise_texture_cache()
{
  static gpu::Texture *tex = nullptr;
  return tex;
}

static gpu::Texture *&cube_texture_cache()
{
  static gpu::Texture *tex = nullptr;
  return tex;
}

static gpu::Texture *&site_cube_texture_cache()
{
  static gpu::Texture *tex = nullptr;
  return tex;
}

static bool &site_cube_texture_tried()
{
  static bool tried = false;
  return tried;
}

static gpu::Texture *&black_texture_cache()
{
  static gpu::Texture *tex = nullptr;
  return tex;
}

static gpu::Texture *&volume_texture_cache()
{
  static gpu::Texture *tex = nullptr;
  return tex;
}

static gpu::FrameBuffer *&framebuffer_cache()
{
  static gpu::FrameBuffer *fb = nullptr;
  return fb;
}

static gpu::Batch *&triangle_cache()
{
  static gpu::Batch *batch = nullptr;
  return batch;
}

static uint64_t hash_source(const char *common,
                            const char *body,
                            const Span<ShaderParam> params,
                            const gpu::TextureFormat fmt,
                            const bool force_opaque,
                            const uint8_t cube_mask,
                            const uint8_t vol_mask,
                            const bool cubemap_pass,
                            const uint8_t nearest_mask,
                            const uint8_t wrap_mask)
{
  DefaultHash<StringRef> hasher;
  uint64_t h = hasher("shadertoy_v59_token_rewrite");
  h = h * 33 + hasher(common ? common : "");
  h = h * 33 + hasher(body ? body : "");
  h = h * 33 + hasher(format_param_defines(params));
  h = h * 33 + uint64_t(fmt);
  h = h * 33 + uint64_t(force_opaque);
  h = h * 33 + uint64_t(cube_mask);
  h = h * 33 + uint64_t(vol_mask);
  h = h * 33 + uint64_t(nearest_mask);
  h = h * 33 + uint64_t(wrap_mask);
  h = h * 33 + uint64_t(cubemap_pass);
  return h;
}

static void apply_buffer_channel_sampler_types(const NodeImageShaderToy &storage,
                                               char pass_tag,
                                               uint8_t &cube_mask,
                                               uint8_t &vol_mask);
static uint8_t detect_nearest_channel_mask(const NodeImageShaderToy &storage, const char pass_tag);
static uint8_t detect_wrap_channel_mask(const NodeImageShaderToy &storage, const char pass_tag);
static bool pass_has_code(const char *common, const char *code);
static bool buffer_is_cubemap_pass(const NodeImageShaderToy &storage, int buffer);

static CompiledPass &compile_pass(const char * /*debug_name*/,
                                  const char *common,
                                  const char *body,
                                  const Span<ShaderParam> params,
                                  const gpu::TextureFormat fmt,
                                  const bool force_opaque,
                                  const NodeImageShaderToy *channel_storage = nullptr,
                                  const char pass_tag = 0)
{
  using namespace gpu::shader;
  /* Cubemap A is a Buffer pass. If the user pasted it into Image, fall back to a 2D view. */
  const bool is_buffer = channel_storage && pass_tag >= 'A' && pass_tag <= 'D';
  bool cubemap_pass = !force_opaque &&
                      (is_buffer ? buffer_is_cubemap_pass(*channel_storage, pass_tag - 'A') :
                                   is_cubemap_only_pass(body));
  std::string body_owned;
  if (cubemap_pass) {
    body_owned = body ? body : "";
  }
  else if (is_cubemap_only_pass(body)) {
    body_owned = body ? body : "";
    body_owned +=
        "\nvoid mainImage(out vec4 fragColor, in vec2 fragCoord)\n"
        "{\n"
        "  vec2 p = (2.0 * fragCoord - iResolution.xy) / iResolution.y;\n"
        "  vec3 rd = normalize(vec3(p, 1.2));\n"
        "  mainCubemap(fragColor, fragCoord, vec3(0.0), rd);\n"
        "}\n";
  }
  else {
    body_owned = ensure_main_image(body);
  }
  const char *body_use = body_owned.c_str();
  uint8_t cube_mask = 0;
  uint8_t vol_mask = 0;
  detect_sampler_masks(common, body_use, cube_mask, vol_mask);
  if (channel_storage && pass_tag) {
    apply_buffer_channel_sampler_types(*channel_storage, pass_tag, cube_mask, vol_mask);
  }
  const uint8_t fetch_mask = detect_texelfetch_mask(common, body_use);
  const uint8_t tex_like_mask = detect_texture_like_mask(common, body_use);
  const uint8_t nearest_mask = (channel_storage && pass_tag) ?
                                  detect_nearest_channel_mask(*channel_storage, pass_tag) :
                                  0;
  const uint8_t wrap_mask = (channel_storage && pass_tag) ?
                               detect_wrap_channel_mask(*channel_storage, pass_tag) :
                               0;
  const uint64_t key = hash_source(common,
                                   body_use,
                                   params,
                                   fmt,
                                   force_opaque,
                                   cube_mask,
                                   vol_mask,
                                   cubemap_pass,
                                   nearest_mask,
                                   wrap_mask);
  {
    std::lock_guard lock(shader_cache_mutex());
    if (CompiledPass *existing = shader_cache().lookup_ptr(key)) {
      return *existing;
    }
  }

  CompiledPass compiled;
  compiled.cube_mask = cube_mask;
  compiled.vol_mask = vol_mask;
  compiled.fetch_mask = fetch_mask;
  compiled.tex_like_mask = tex_like_mask;
  compiled.cubemap_pass = cubemap_pass;
  const bool has_entry = cubemap_pass || pass_has_code(common, body_use);
  if (body_owned.empty() || !has_entry) {
    compiled.error = "Pass has no mainImage() / mainCubemap()";
    std::lock_guard lock(shader_cache_mutex());
    if (CompiledPass *existing = shader_cache().lookup_ptr(key)) {
      return *existing;
    }
    shader_cache().add(key, compiled);
    return shader_cache().lookup(key);
  }

  /* Name MUST be pyGPU_Shader: GPU_shader_create_from_info_python only expands
   * CREATE_INFO_RES_PASS_pyGPU_Shader (uniforms / iChannel). */
  ShaderCreateInfo info("pyGPU_Shader");
  info.builtins(BuiltinBits::VERTEX_ID | BuiltinBits::FRAG_COORD);
  /* float4: vec3 push-constants are 16-byte aligned (std430) and iResolution.y can be garbage.
   * Exposed as `_st_ires`; `#define iResolution _st_ires.xyz`. */
  info.push_constant(Type::float4_t, "_st_ires");
  info.push_constant(Type::float_t, "iTime");
  info.push_constant(Type::float_t, "iTimeDelta");
  info.push_constant(Type::int_t, "iFrame");
  info.push_constant(Type::float_t, "iFrameRate");
  info.push_constant(Type::float4_t, "iMouse");
  info.push_constant(Type::float4_t, "iDate");
  info.push_constant(Type::int_t, "iCubeFace");
  auto sampler_type = [&](const int ch) {
    if (cube_mask & (1u << ch)) {
      return ImageType::FloatCube;
    }
    if (vol_mask & (1u << ch)) {
      return ImageType::Float3D;
    }
    return ImageType::Float2D;
  };
  info.sampler(0, sampler_type(0), "iChannel0");
  info.sampler(1, sampler_type(1), "iChannel1");
  info.sampler(2, sampler_type(2), "iChannel2");
  info.sampler(3, sampler_type(3), "iChannel3");
  info.fragment_out(0, Type::float4_t, "out_fragColor");
  info.vertex_source_generated = fullscreen_vertex_src();
  info.fragment_source_generated = build_fragment(common,
                                                  body_use,
                                                  params,
                                                  force_opaque,
                                                  cubemap_pass,
                                                  cube_mask,
                                                  vol_mask,
                                                  nearest_mask,
                                                  wrap_mask);

  /* If Compile TDR-kills the process, this file is the last GLSL the driver saw. */
  {
    FILE *dump = fopen("E:/MySoftwares/Grok/_st_dump/_last_st.glsl", "wb");
    if (dump) {
      fwrite(info.fragment_source_generated.data(),
             1,
             info.fragment_source_generated.size(),
             dump);
      fclose(dump);
    }
    if (pass_tag) {
      char pass_path[256];
      SNPRINTF(pass_path, "E:/MySoftwares/Grok/_st_dump/_last_st_%c.glsl", pass_tag);
      FILE *pdump = fopen(pass_path, "wb");
      if (pdump) {
        fwrite(info.fragment_source_generated.data(),
               1,
               info.fragment_source_generated.size(),
               pdump);
        fclose(pdump);
      }
    }
  }

  GPU_shader_clear_last_compile_error();
  try {
    compiled.shader = GPU_shader_create_from_info_python(
        reinterpret_cast<const GPUShaderCreateInfo *>(&info));
  }
  catch (const std::exception &) {
    compiled.shader = nullptr;
    compiled.error =
        "GPU preprocessor crashed (often a `//*` line comment). "
        "This should no longer abort Blender; try Compile again after restart.";
  }
  if (compiled.shader == nullptr && compiled.error.empty()) {
    std::string log = GPU_shader_get_last_compile_error();
    std::string clean;
    clean.reserve(log.size());
    for (size_t i = 0; i < log.size(); i++) {
      if (log[i] == '\033') {
        while (i < log.size() && log[i] != 'm') {
          i++;
        }
        continue;
      }
      clean.push_back(log[i]);
    }
    while (!clean.empty() && (clean.back() == '\n' || clean.back() == ' ' || clean.back() == '\r'))
    {
      clean.pop_back();
    }
    if (clean.size() > 1800) {
      clean.resize(1800);
      clean += "\n...";
    }
    if (!clean.empty()) {
      compiled.error = "GPU compile failed:\n" + clean;
    }
    else {
      compiled.error =
          "GPU compile failed (see Blender System Console). "
          "Typical: IQ loops using min(iFrame,0), cubemaps, or WebGL-only builtins.";
    }
    const bool missing_common = (common == nullptr || common[0] == '\0');
    if (missing_common &&
        (compiled.error.find("hash21") != std::string::npos ||
         compiled.error.find("hash22") != std::string::npos ||
         compiled.error.find("rot2") != std::string::npos ||
         compiled.error.find("'TAU'") != std::string::npos))
    {
      compiled.error +=
          "\nHint: This shader has a Common tab. In the N-panel set Pass = Common, "
          "paste that GLSL, then Compile again. Helpers like hash21 / rot2 / TAU live there.";
    }
  }
  std::lock_guard lock(shader_cache_mutex());
  if (CompiledPass *existing = shader_cache().lookup_ptr(key)) {
    if (compiled.shader) {
      GPU_shader_free(compiled.shader);
      compiled.shader = nullptr;
    }
    return *existing;
  }
  shader_cache().add(key, compiled);
  return shader_cache().lookup(key);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Persistent Buffer A–D ping-pong
 * \{ */

struct BufferState {
  gpu::Texture *tex[4][2] = {};
  gpu::Texture *cube[4][2] = {};
  int cube_size = 256;
  /* Mip chain required: LINEAR_MIPMAP on a 1-level cube is black on Vulkan. */
  static constexpr int cube_mips = 8;
  int ping = 0;
  int2 size = int2(0);
  gpu::TextureFormat format = gpu::TextureFormat::Invalid;
  int last_frame = std::numeric_limits<int>::min();
  float last_time = 0.0f;
  /* ShaderToy iFrame is 0-based and counts buffer ticks since Compile/reset, not Scene cfra. */
  int shader_frame = 0;
  gpu::Texture *image_cache = nullptr;
  int2 image_cache_size = int2(0);
  uint64_t image_cook_key = 0;

  void reset_temporal()
  {
    ping = 0;
    last_frame = std::numeric_limits<int>::min();
    last_time = 0.0f;
    shader_frame = 0;
    image_cook_key = 0;
  }

  void clear_contents(const float fill = 0.0f)
  {
    const float col[4] = {fill, fill, fill, 1.0f};
    for (int p = 0; p < 4; p++) {
      for (int i = 0; i < 2; i++) {
        if (tex[p][i]) {
          GPU_texture_clear(tex[p][i], GPU_DATA_FLOAT, col);
        }
        if (cube[p][i]) {
          GPU_texture_clear(cube[p][i], GPU_DATA_FLOAT, col);
        }
      }
    }
    reset_temporal();
  }

  void free()
  {
    for (int p = 0; p < 4; p++) {
      for (int i = 0; i < 2; i++) {
        if (tex[p][i]) {
          GPU_texture_free(tex[p][i]);
          tex[p][i] = nullptr;
        }
        if (cube[p][i]) {
          GPU_texture_free(cube[p][i]);
          cube[p][i] = nullptr;
        }
      }
    }
    if (image_cache) {
      GPU_texture_free(image_cache);
      image_cache = nullptr;
    }
    image_cache_size = int2(0);
    size = int2(0);
    format = gpu::TextureFormat::Invalid;
    reset_temporal();
  }

  gpu::Texture *ensure_image_cache(const int2 want, const gpu::TextureFormat fmt)
  {
    if (image_cache && image_cache_size == want) {
      return image_cache;
    }
    if (image_cache) {
      GPU_texture_free(image_cache);
      image_cache = nullptr;
    }
    image_cache = GPU_texture_create_2d(
        "shadertoy_image_cache", want.x, want.y, 1, fmt, GPU_TEXTURE_USAGE_GENERAL, nullptr);
    image_cache_size = want;
    image_cook_key = 0;
    return image_cache;
  }

  gpu::Texture *ensure_cube(const int buffer, const int ping_i, const gpu::TextureFormat fmt)
  {
    if (buffer < 0 || buffer >= 4 || ping_i < 0 || ping_i > 1) {
      return nullptr;
    }
    if (cube[buffer][ping_i] == nullptr) {
      char name[64];
      SNPRINTF(name, "shadertoy_cube%c_%d", char('A' + buffer), ping_i);
      cube[buffer][ping_i] = GPU_texture_create_cube(
          name, cube_size, cube_mips, fmt, GPU_TEXTURE_USAGE_GENERAL, nullptr);
      if (cube[buffer][ping_i]) {
        const float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        GPU_texture_clear(cube[buffer][ping_i], GPU_DATA_FLOAT, black);
      }
    }
    return cube[buffer][ping_i];
  }

  void ensure(const int2 want, const gpu::TextureFormat fmt)
  {
    if (size == want && format == fmt && tex[0][0] != nullptr) {
      return;
    }
    free();
    size = want;
    format = fmt;
    reset_temporal();
    const float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    for (int p = 0; p < 4; p++) {
      for (int i = 0; i < 2; i++) {
        char name[64];
        SNPRINTF(name, "shadertoy_buf%c_%d", char('A' + p), i);
        tex[p][i] = GPU_texture_create_2d(
            name, want.x, want.y, 1, fmt, GPU_TEXTURE_USAGE_GENERAL, nullptr);
        if (tex[p][i]) {
          GPU_texture_clear(tex[p][i], GPU_DATA_FLOAT, black);
        }
      }
    }
  }
};

static Mutex &buffer_cache_mutex()
{
  static Mutex mutex;
  return mutex;
}

static Map<uint64_t, std::unique_ptr<BufferState>> &buffer_cache()
{
  static Map<uint64_t, std::unique_ptr<BufferState>> map;
  return map;
}

static uint64_t buffer_key(const bNode &node)
{
  return get_default_hash(uint64_t(node.owner_tree().id.session_uid), uint64_t(node.identifier));
}

static BufferState &get_buffer_state(const bNode &node,
                                     const int2 size,
                                     const gpu::TextureFormat fmt,
                                     const bool reset,
                                     const float fill = 0.0f)
{
  std::lock_guard lock(buffer_cache_mutex());
  const uint64_t key = buffer_key(node);
  std::unique_ptr<BufferState> &slot = buffer_cache().lookup_or_add_cb(
      key, []() { return std::make_unique<BufferState>(); });
  slot->ensure(size, fmt);
  if (reset) {
    slot->clear_contents(fill);
  }
  return *slot;
}

static gpu::Texture *dummy_noise_texture()
{
  /* ShaderToy Image-only shaders almost always sample iChannel0 as the site's RGBA noise
   * texture. A 1×1 black dummy makes those look completely wrong. */
  gpu::Texture *&tex = noise_texture_cache();
  if (tex == nullptr) {
    constexpr int n = 256;
    Vector<float> pixels(n * n * 4);
    /* ShaderToy RGBA noise is an 8-bit white texture. sin-hash is low-frequency and
     * lattice-aligned, which shows up as fog "格子" and missing octaves in Remnant X. */
    auto uhash = [](uint32_t x) {
      x ^= x >> 16;
      x *= 0x7feb352du;
      x ^= x >> 15;
      x *= 0x846ca68bu;
      x ^= x >> 16;
      return x;
    };
    for (int y = 0; y < n; y++) {
      for (int x = 0; x < n; x++) {
        const uint32_t px = uint32_t(x);
        const uint32_t py = uint32_t(y);
        const int i = (y * n + x) * 4;
        /* R = white. G/B are R shifted by IQ's (37,17) / (19,47) so
         * `mix(tex.yx, f.z)` stays continuous across integer Z planes. */
        /* Store ~sRGB (sqrt) so Shane/IQ `tx*tx` linearization matches the site. */
        auto enc = [](float lin) {
          return std::sqrt(math::clamp(lin, 0.0f, 1.0f));
        };
        pixels[i + 0] = enc(float(uhash(px + 374761393u * py + 1u) & 255u) / 255.0f);
        pixels[i + 2] = enc(float(uhash(px + 362437u * py + 1009u) & 255u) / 255.0f);
        pixels[i + 3] = float(uhash(px + 2246822519u * py + 17u) & 255u) / 255.0f;
      }
    }
    for (int y = 0; y < n; y++) {
      for (int x = 0; x < n; x++) {
        const int i = (y * n + x) * 4;
        const int gx = (x - 37 + n) % n;
        const int gy = (y - 17 + n) % n;
        const int bx = (x - 19 + n) % n;
        const int by = (y - 47 + n) % n;
        pixels[i + 1] = pixels[(gy * n + gx) * 4 + 0];
        pixels[i + 2] = pixels[(by * n + bx) * 4 + 0];
      }
    }
    /* Full mip chain: IQ's Elevated/Volcanic and many others call texture() / textureLod
     * on the 256² noise. A 1-level texture returns black/undefined for lod>0. */
    const int mips = 9;
    tex = GPU_texture_create_2d("shadertoy_noise",
                                n,
                                n,
                                mips,
                                gpu::TextureFormat::SFLOAT_16_16_16_16,
                                GPU_TEXTURE_USAGE_GENERAL,
                                pixels.data());
    if (tex) {
      GPU_texture_update_mipmap_chain(tex);
    }
  }
  return tex;
}

static gpu::Texture *dummy_cube_texture()
{
  /* ShaderToy cubemap iChannels (forest / studio HDRI / sky). Unbound cube samplers
   * need a real cube texture — a 2D dummy will not bind to samplerCube. */
  gpu::Texture *&tex = cube_texture_cache();
  if (tex == nullptr) {
    constexpr int n = 64;
    Vector<float> pixels(6 * n * n * 4);
    for (int face = 0; face < 6; face++) {
      for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
          const float u = (2.0f * (float(x) + 0.5f) / float(n)) - 1.0f;
          const float v = (2.0f * (float(y) + 0.5f) / float(n)) - 1.0f;
          float dx;
          float dy;
          float dz;
          switch (face) {
            case 0:
              dx = 1.0f;
              dy = -v;
              dz = -u;
              break;
            case 1:
              dx = -1.0f;
              dy = -v;
              dz = u;
              break;
            case 2:
              dx = u;
              dy = 1.0f;
              dz = v;
              break;
            case 3:
              dx = u;
              dy = -1.0f;
              dz = -v;
              break;
            case 4:
              dx = u;
              dy = -v;
              dz = 1.0f;
              break;
            default:
              dx = -u;
              dy = -v;
              dz = -1.0f;
              break;
          }
          const float inv = 1.0f / std::sqrt(dx * dx + dy * dy + dz * dz);
          dx *= inv;
          dy *= inv;
          dz *= inv;
          /* Default site cubemap is Uffizi Gallery (indoor, warm, bright
           * windows) — not a sky. 4s3GDB samples it as the whole image. */
          float r;
          float g;
          float b;
          if (dy > 0.45f) {
            const float sky = (dy - 0.45f) / 0.55f;
            const float win = std::pow(std::max(1.0f - (dx * dx + dz * dz) * 1.8f, 0.0f), 2.0f);
            r = 0.42f + 0.50f * sky + 0.38f * win;
            g = 0.38f + 0.42f * sky + 0.32f * win;
            b = 0.30f + 0.28f * sky + 0.22f * win;
          }
          else if (dy < -0.35f) {
            const float tile = std::fmod(std::floor(dx * 6.0f) + std::floor(dz * 6.0f), 2.0f);
            const float fl = (tile > 0.5f) ? 0.16f : 0.11f;
            r = fl * 1.35f;
            g = fl * 0.95f;
            b = fl * 0.70f;
          }
          else {
            r = 0.58f;
            g = 0.50f;
            b = 0.38f;
            const float win = std::max(dz, 0.0f);
            if (win > 0.55f) {
              const float w = std::pow((win - 0.55f) / 0.45f, 1.4f);
              r = 0.58f + 0.40f * w;
              g = 0.50f + 0.38f * w;
              b = 0.38f + 0.28f * w;
            }
            const float frame = std::min(std::min(std::fabs(dx), std::fabs(dy + 0.05f)), 0.35f);
            if (frame < 0.04f) {
              r *= 0.45f;
              g *= 0.42f;
              b *= 0.40f;
            }
          }
          r = std::min(std::max(r, 0.0f), 1.0f);
          g = std::min(std::max(g, 0.0f), 1.0f);
          b = std::min(std::max(b, 0.0f), 1.0f);
          /* Store display-referred sRGB like ShaderToy `srgb:false` 8-bit cubes.
           * Image pass then converts sRGB→linear so Standard ≈ the website. */
          auto to_srgb = [](const float lin) {
            if (lin <= 0.0031308f) {
              return 12.92f * lin;
            }
            return 1.055f * std::pow(lin, 1.0f / 2.4f) - 0.055f;
          };
          const int i = ((face * n + y) * n + x) * 4;
          pixels[i + 0] = to_srgb(r);
          pixels[i + 1] = to_srgb(g);
          pixels[i + 2] = to_srgb(b);
          pixels[i + 3] = 1.0f;
        }
      }
    }
    constexpr int mips = 7;
    tex = GPU_texture_create_cube("shadertoy_cube",
                                  n,
                                  mips,
                                  gpu::TextureFormat::SFLOAT_16_16_16_16,
                                  GPU_TEXTURE_USAGE_GENERAL,
                                  pixels.data());
    if (tex) {
      GPU_texture_update_mipmap_chain(tex);
    }
  }
  return tex;
}

static gpu::Texture *load_stcube_file(const char *path)
{
  FILE *fp = BLI_fopen(path, "rb");
  if (fp == nullptr) {
    return nullptr;
  }
  char magic[8];
  if (fread(magic, 1, 8, fp) != 8 || std::memcmp(magic, "STCUBE01", 8) != 0) {
    fclose(fp);
    return nullptr;
  }
  uint32_t n = 0;
  if (fread(&n, sizeof(n), 1, fp) != 1 || n < 8 || n > 2048) {
    fclose(fp);
    return nullptr;
  }
  const size_t face = size_t(n) * size_t(n) * 4;
  Vector<unsigned char> bytes(6 * face);
  if (fread(bytes.data(), 1, bytes.size(), fp) != bytes.size()) {
    fclose(fp);
    return nullptr;
  }
  fclose(fp);
  Vector<float> pixels(int64_t(6 * face));
  for (size_t i = 0; i < pixels.size(); i++) {
    pixels[i] = float(bytes[i]) * (1.0f / 255.0f);
  }
  const int mips = 1 + int(std::floor(std::log2(float(n))));
  gpu::Texture *tex = GPU_texture_create_cube("shadertoy_uffizi",
                                              int(n),
                                              mips,
                                              gpu::TextureFormat::SFLOAT_16_16_16_16,
                                              GPU_TEXTURE_USAGE_GENERAL,
                                              pixels.data());
  if (tex) {
    GPU_texture_update_mipmap_chain(tex);
  }
  return tex;
}

static gpu::Texture *site_cubemap_texture()
{
  /* ShaderToy default cubemap is Uffizi Gallery (4s3GDB rain refraction, etc.).
   * A procedural indoor box is not website shading. */
  gpu::Texture *&tex = site_cube_texture_cache();
  bool &tried = site_cube_texture_tried();
  if (tried) {
    return tex;
  }
  tried = true;

  Vector<std::string> dirs;
  if (std::optional<std::string> p = BKE_appdir_folder_id(BLENDER_SYSTEM_SCRIPTS,
                                                         "startup/shadertoy_media"))
  {
    dirs.append(*p);
  }
  if (std::optional<std::string> p = BKE_appdir_folder_id(BLENDER_USER_SCRIPTS,
                                                         "startup/shadertoy_media"))
  {
    dirs.append(*p);
  }
  if (std::optional<std::string> p = BKE_appdir_folder_id(BLENDER_USER_DATAFILES, "shadertoy")) {
    dirs.append(*p);
  }
  char prog[1024];
  BLI_path_join(prog,
                sizeof(prog),
                BKE_appdir_program_dir(),
                "5.3",
                "scripts",
                "startup",
                "shadertoy_media");
  dirs.append(prog);

  for (const std::string &dir : dirs) {
    char path[1024];
    BLI_path_join(path, sizeof(path), dir.c_str(), "uffizi.stcube");
    if (!BLI_exists(path)) {
      continue;
    }
    tex = load_stcube_file(path);
    if (tex) {
      break;
    }
  }
  return tex;
}

static gpu::Texture *dummy_black_texture()
{
  gpu::Texture *&tex = black_texture_cache();
  if (tex == nullptr) {
    const float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    tex = GPU_texture_create_2d(
        "shadertoy_black", 1, 1, 1, gpu::TextureFormat::SFLOAT_16_16_16_16, GPU_TEXTURE_USAGE_GENERAL, black);
  }
  return tex;
}

static gpu::Texture *dummy_volume_texture()
{
  /* 32³ RGBA noise for ShaderToy volume iChannels (`texture(iChannel0, vec3)`). */
  gpu::Texture *&tex = volume_texture_cache();
  if (tex == nullptr) {
    constexpr int n = 32;
    Vector<float> pixels(n * n * n * 4);
    for (int z = 0; z < n; z++) {
      for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
          auto hash = [](float a, float b, float c) {
            const float v = std::sin(a * 127.1f + b * 311.7f + c * 74.7f) * 43758.5453f;
            return v - std::floor(v);
          };
          const float fx = float(x) + 0.5f;
          const float fy = float(y) + 0.5f;
          const float fz = float(z) + 0.5f;
          const int i = ((z * n + y) * n + x) * 4;
          pixels[i + 0] = hash(fx, fy, fz);
          pixels[i + 1] = hash(fx, fy, fz + 1.7f);
          pixels[i + 2] = hash(fx, fy, fz + 3.1f);
          pixels[i + 3] = hash(fx, fy, fz + 5.3f);
        }
      }
    }
    tex = GPU_texture_create_3d("shadertoy_volume",
                                n,
                                n,
                                n,
                                1,
                                gpu::TextureFormat::SFLOAT_16_16_16_16,
                                GPU_TEXTURE_USAGE_GENERAL,
                                pixels.data());
  }
  return tex;
}

static gpu::FrameBuffer *shadertoy_fb()
{
  gpu::FrameBuffer *&fb = framebuffer_cache();
  if (fb == nullptr) {
    fb = GPU_framebuffer_create("shadertoy_fb");
  }
  return fb;
}

static gpu::Batch *shadertoy_tri()
{
  gpu::Batch *&batch = triangle_cache();
  if (batch == nullptr) {
    batch = GPU_batch_create_procedural(GPU_PRIM_TRIS, 6);
  }
  return batch;
}

void free_session_caches()
{
  {
    std::lock_guard lock(shader_cache_mutex());
    for (CompiledPass &pass : shader_cache().values()) {
      if (pass.shader != nullptr) {
        GPU_shader_free(pass.shader);
      }
    }
    shader_cache().clear();
  }
  {
    std::lock_guard lock(buffer_cache_mutex());
    for (std::unique_ptr<BufferState> &state : buffer_cache().values()) {
      state->free();
    }
    buffer_cache().clear();
  }
  for (gpu::Texture **tex : {&noise_texture_cache(),
                             &cube_texture_cache(),
                             &site_cube_texture_cache(),
                             &black_texture_cache(),
                             &volume_texture_cache()})
  {
    if (*tex != nullptr) {
      GPU_texture_free(*tex);
      *tex = nullptr;
    }
  }
  site_cube_texture_tried() = false;
  if (gpu::FrameBuffer *&fb = framebuffer_cache()) {
    GPU_framebuffer_free(fb);
    fb = nullptr;
  }
  if (gpu::Batch *&batch = triangle_cache()) {
    GPU_batch_discard(batch);
    batch = nullptr;
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Channel map parser
 * \{ */

enum class ChannelKind { None, Buffer, Texture, Keyboard, Skip, Cubemap, Volume };

struct ChannelBind {
  ChannelKind kind = ChannelKind::None;
  int buffer = 0; /* 0–3 = A–D */
  bool nearest = false;
  bool repeat = false;
  std::string media;
};

static bool consume_channel_sep(const StringRef s, int64_t &i)
{
  if (i >= s.size()) {
    return false;
  }
  const unsigned char c = static_cast<unsigned char>(s[i]);
  if (c == ',' || c == ';' || c == ' ' || c == '\t' || c == '\n' || c == '\r') {
    i++;
    return true;
  }
  /* UTF-8 fullwidth comma U+FF0C. */
  if (i + 2 < s.size() && c == 0xEF && static_cast<unsigned char>(s[i + 1]) == 0xBC &&
      static_cast<unsigned char>(s[i + 2]) == 0x8C)
  {
    i += 3;
    return true;
  }
  /* UTF-8 ideographic comma U+3001. */
  if (i + 2 < s.size() && c == 0xE3 && static_cast<unsigned char>(s[i + 1]) == 0x80 &&
      static_cast<unsigned char>(s[i + 2]) == 0x81)
  {
    i += 3;
    return true;
  }
  return false;
}

static int buffer_index_from_letter(const char letter)
{
  const char u = char(std::toupper(static_cast<unsigned char>(letter)));
  if (u >= 'A' && u <= 'D') {
    return u - 'A';
  }
  return -1;
}

/* Fullwidth Latin Ａ–Ｄ / Ｔ / Ｋ (U+FF21–FF24, FF34, FF2B) as a 3-byte UTF-8 cluster. */
static int fullwidth_channel_letter(StringRef tok, const int64_t i)
{
  if (i + 2 >= tok.size()) {
    return 0;
  }
  const unsigned char a = static_cast<unsigned char>(tok[i]);
  const unsigned char b = static_cast<unsigned char>(tok[i + 1]);
  const unsigned char c = static_cast<unsigned char>(tok[i + 2]);
  if (a != 0xEF || b != 0xBC) {
    return 0;
  }
  if (c >= 0xA1 && c <= 0xA4) {
    return 'A' + (c - 0xA1);
  }
  if (c == 0xB4) {
    return 'T';
  }
  if (c == 0xAB) {
    return 'K';
  }
  return 0;
}

static ChannelBind parse_channel_token(StringRef tok)
{
  ChannelBind bind;
  bind.kind = ChannelKind::Texture;
  tok = rtrim(ltrim(tok));
  if (tok.is_empty()) {
    return bind;
  }
  if (tok.startswith("buffer:") && tok.size() > 7) {
    const int idx = buffer_index_from_letter(tok.data()[7]);
    if (idx >= 0) {
      bind.kind = ChannelKind::Buffer;
      bind.buffer = idx;
    }
    if (tok.find(":n") != StringRef::not_found || tok.find("nearest") != StringRef::not_found) {
      bind.nearest = true;
    }
    if (tok.find(":r") != StringRef::not_found || tok.find("repeat") != StringRef::not_found) {
      bind.repeat = true;
    }
    return bind;
  }
  if (tok.startswith("texture:")) {
    bind.kind = ChannelKind::Texture;
    return bind;
  }
  if (tok.startswith("keyboard:")) {
    bind.kind = ChannelKind::Keyboard;
    return bind;
  }
  if (tok.startswith("skip:")) {
    if (tok.find("cube") != StringRef::not_found) {
      bind.kind = ChannelKind::Cubemap;
    }
    else if (tok.find("volume") != StringRef::not_found || tok.find("vol") != StringRef::not_found)
    {
      bind.kind = ChannelKind::Volume;
    }
    else {
      bind.kind = ChannelKind::Skip;
    }
    return bind;
  }
  if (tok.startswith("cubemap:")) {
    bind.kind = ChannelKind::Cubemap;
    const StringRef rest = rtrim(ltrim(tok.substr(8)));
    if (!rest.is_empty()) {
      bind.media = std::string(rest.data(), size_t(rest.size()));
    }
    return bind;
  }
  if (tok.startswith("volume:")) {
    bind.kind = ChannelKind::Volume;
    return bind;
  }
  std::string lower(tok.data(), size_t(tok.size()));
  for (char &c : lower) {
    c = char(std::tolower(static_cast<unsigned char>(c)));
  }
  if (lower == "#" || lower == "black" || lower == "-") {
    bind.kind = ChannelKind::Skip;
    return bind;
  }
  if (lower == "k" || lower == "keyboard") {
    bind.kind = ChannelKind::Keyboard;
    return bind;
  }
  if (lower == "u" || lower == "cube" || lower == "cubemap") {
    bind.kind = ChannelKind::Cubemap;
    return bind;
  }
  if (lower == "v" || lower == "vol" || lower == "volume") {
    bind.kind = ChannelKind::Volume;
    return bind;
  }
  if (lower == "t" || lower == "tex" || lower == "texture" || lower == "n" || lower == "noise") {
    bind.kind = ChannelKind::Texture;
    return bind;
  }

  auto apply_letter = [&](const int letter) {
    if (letter == 'K') {
      bind.kind = ChannelKind::Keyboard;
      return true;
    }
    if (letter == 'T') {
      bind.kind = ChannelKind::Texture;
      return true;
    }
    if (letter == 'U') {
      bind.kind = ChannelKind::Cubemap;
      return true;
    }
    if (letter == 'V') {
      bind.kind = ChannelKind::Volume;
      return true;
    }
    if (letter >= 'A' && letter <= 'D') {
      bind.kind = ChannelKind::Buffer;
      bind.buffer = letter - 'A';
      return true;
    }
    return false;
  };

  if (!lower.empty() &&
      apply_letter(char(std::toupper(static_cast<unsigned char>(lower[0])))))
  {
    bool flags_ok = true;
    for (size_t i = 1; i < lower.size(); i++) {
      if (lower[i] == 'n') {
        bind.nearest = true;
      }
      else if (lower[i] == 'r') {
        bind.repeat = true;
      }
      else if (lower[i] == 'l') {
        bind.nearest = false;
      }
      else {
        flags_ok = false;
        break;
      }
    }
    if (flags_ok) {
      return bind;
    }
    bind = ChannelBind{};
    bind.kind = ChannelKind::Texture;
  }
  if (const int fw = fullwidth_channel_letter(tok, 0)) {
    if (tok.size() == 3 && apply_letter(fw)) {
      return bind;
    }
  }

  /* "BufferA" / "buf a" / "layerA" — last A–D wins. Do not scan arbitrary words (black has 'a'). */
  if (lower.find("buffer") != std::string::npos || lower.find("buf") == 0 ||
      lower.find("layer") != std::string::npos)
  {
    for (int64_t i = tok.size() - 1; i >= 0; i--) {
      const int idx = buffer_index_from_letter(tok[i]);
      if (idx >= 0) {
        bind.kind = ChannelKind::Buffer;
        bind.buffer = idx;
        return bind;
      }
      const int fw = fullwidth_channel_letter(tok, i);
      if (fw >= 'A' && fw <= 'D') {
        bind.kind = ChannelKind::Buffer;
        bind.buffer = fw - 'A';
        return bind;
      }
    }
  }
  return bind;
}

static const char *pass_channels_spec(const NodeImageShaderToy &storage, const char pass_tag)
{
  switch (pass_tag) {
    case 'I':
      return storage.channels_image;
    case 'A':
      return storage.channels_buffer_a;
    case 'B':
      return storage.channels_buffer_b;
    case 'C':
      return storage.channels_buffer_c;
    case 'D':
      return storage.channels_buffer_d;
    default:
      return "";
  }
}

static StringRef channel_map_slice(const NodeImageShaderToy &storage, const char pass_tag)
{
  const char *map = storage.channel_map;
  if (map == nullptr || map[0] == '\0') {
    return {};
  }
  const StringRef s(map);
  auto find_key = [&](const StringRef key) -> int64_t {
    int64_t from = 0;
    while (from < s.size()) {
      const int64_t hit = s.find(key, from);
      if (hit == StringRef::not_found) {
        return StringRef::not_found;
      }
      if (hit == 0 || s[hit - 1] == '|') {
        return hit + key.size();
      }
      from = hit + 1;
    }
    return StringRef::not_found;
  };
  int64_t start = StringRef::not_found;
  if (pass_tag == 'I') {
    start = find_key("IMAGE:");
    if (start == StringRef::not_found) {
      start = find_key("I:");
    }
  }
  else {
    char key[3] = {pass_tag, ':', '\0'};
    start = find_key(key);
  }
  if (start == StringRef::not_found) {
    return {};
  }
  int64_t end = s.find('|', start);
  if (end == StringRef::not_found) {
    end = s.size();
  }
  return s.substr(start, end - start);
}

static bool skip_one_list_sep(const StringRef s, int64_t &i)
{
  if (i >= s.size()) {
    return false;
  }
  const unsigned char c = static_cast<unsigned char>(s[i]);
  if (c == ',' || c == ';') {
    i++;
    return true;
  }
  if (i + 2 < s.size() && c == 0xEF && static_cast<unsigned char>(s[i + 1]) == 0xBC &&
      static_cast<unsigned char>(s[i + 2]) == 0x8C)
  {
    i += 3;
    return true;
  }
  if (i + 2 < s.size() && c == 0xE3 && static_cast<unsigned char>(s[i + 1]) == 0x80 &&
      static_cast<unsigned char>(s[i + 2]) == 0x81)
  {
    i += 3;
    return true;
  }
  return false;
}

static void parse_channel_spec(const StringRef s, ChannelBind out[4])
{
  for (int t = 0; t < 4; t++) {
    out[t].kind = ChannelKind::Texture;
  }
  if (s.is_empty()) {
    return;
  }
  bool indexed = false;
  {
    int64_t i = 0;
    while (i < s.size() && consume_channel_sep(s, i)) {
    }
    if (i + 1 < s.size() && s[i] >= '0' && s[i] <= '3' && s[i + 1] == '=') {
      indexed = true;
    }
  }
  if (indexed) {
    int64_t i = 0;
    while (i < s.size()) {
      while (consume_channel_sep(s, i)) {
      }
      if (i >= s.size()) {
        break;
      }
      const int64_t start = i;
      while (i < s.size()) {
        const int64_t save = i;
        if (consume_channel_sep(s, i)) {
          i = save;
          break;
        }
        i++;
      }
      if (i <= start) {
        continue;
      }
      StringRef tok = s.substr(start, i - start);
      if (tok.size() >= 2 && tok[0] >= '0' && tok[0] <= '3' && tok[1] == '=') {
        const int slot = tok[0] - '0';
        out[slot] = parse_channel_token(tok.substr(2));
      }
    }
    return;
  }

  /* Compact letters. `A,T,,K` (Xst3Dj) must keep the empty iChannel2 slot so K
   * stays on iChannel3. Collapsing consecutive commas bound keyboard to noise
   * and froze `reset()` on a bright texel. */
  bool has_list_sep = s.find(',') != StringRef::not_found || s.find(';') != StringRef::not_found;
  if (!has_list_sep) {
    for (int64_t p = 0; p < s.size();) {
      int64_t q = p;
      if (skip_one_list_sep(s, q)) {
        has_list_sep = true;
        break;
      }
      p++;
    }
  }
  if (has_list_sep) {
    int slot = 0;
    int64_t i = 0;
    while (slot < 4) {
      const int64_t start = i;
      while (i < s.size()) {
        int64_t q = i;
        if (skip_one_list_sep(s, q)) {
          break;
        }
        i++;
      }
      StringRef tok = rtrim(ltrim(s.substr(start, i - start)));
      if (!tok.is_empty()) {
        out[slot] = parse_channel_token(tok);
      }
      slot++;
      if (i >= s.size()) {
        break;
      }
      skip_one_list_sep(s, i);
    }
    return;
  }

  int n = 0;
  int64_t i = 0;
  while (i < s.size() && n < 4) {
    while (consume_channel_sep(s, i)) {
    }
    if (i >= s.size()) {
      break;
    }
    const int64_t start = i;
    while (i < s.size()) {
      const int64_t save = i;
      if (consume_channel_sep(s, i)) {
        i = save;
        break;
      }
      i++;
    }
    if (i <= start) {
      break;
    }
    out[n] = parse_channel_token(s.substr(start, i - start));
    n++;
  }
}

static ChannelBind parse_channel(const NodeImageShaderToy &storage,
                                 const char pass_tag,
                                 const int channel)
{
  ChannelBind bind;
  bind.kind = ChannelKind::None;
  if (channel < 0 || channel > 3) {
    bind.kind = ChannelKind::Texture;
    return bind;
  }
  const char *spec = pass_channels_spec(storage, pass_tag);
  const StringRef letters = (spec && spec[0]) ? StringRef(spec) : StringRef();
  const StringRef map = channel_map_slice(storage, pass_tag);
  if (letters.is_empty() && map.is_empty()) {
    /* Empty wiring: auto-bind iChannelN → Buffer N (self-feedback), not the dummy
     * noise texture. Dummy noise makes NS fluids (4tGfDW) look scattered / not
     * divergence-free because every frame samples static hash instead of the sim. */
    return bind;
  }
  ChannelBind from_letters[4];
  ChannelBind from_map[4];
  if (!letters.is_empty()) {
    parse_channel_spec(letters, from_letters);
    bind = from_letters[channel];
  }
  if (!map.is_empty()) {
    parse_channel_spec(map, from_map);
    if (letters.is_empty()) {
      bind = from_map[channel];
    }
    else {
      /* Compact letters can drop empty slots (`A,T,K` vs `A,T,,K`). Indexed
       * channel_map still has `3=keyboard:` — never let that become dummy noise
       * or Xst3Dj reset() sticks true and the sim is frozen on frame 0. */
      const ChannelBind &m = from_map[channel];
      if (m.kind == ChannelKind::Keyboard) {
        bind = m;
      }
      else if (m.kind == ChannelKind::Buffer &&
               (bind.kind == ChannelKind::Texture || bind.kind == ChannelKind::None))
      {
        bind = m;
      }
      if (m.nearest) {
        bind.nearest = true;
      }
      if (m.repeat) {
        bind.repeat = true;
      }
      if (m.kind == ChannelKind::Cubemap) {
        if (bind.kind == ChannelKind::Cubemap || bind.kind == ChannelKind::Texture ||
            bind.kind == ChannelKind::None)
        {
          bind.kind = ChannelKind::Cubemap;
        }
        if (!m.media.empty()) {
          bind.media = m.media;
        }
      }
    }
  }
  return bind;
}

static bool pass_has_code(const char *common, const char *code)
{
  if (code == nullptr || code[0] == '\0') {
    return false;
  }
  const StringRef s(code);
  if (s.find("mainImage") != StringRef::not_found ||
      s.find("mainCubemap") != StringRef::not_found)
  {
    return true;
  }
  if (common == nullptr || common[0] == '\0') {
    return false;
  }
  /* `#define Main void mainImage(out vec4 Q, in vec2 U)` in Common and `Main { ... }` in
   * every pass (WcsSDl). The pass text alone never mentions mainImage. */
  const TokenStream ts = tokenize(common);
  for (int t = 0; t < ts.toks.size(); t++) {
    if (!ts.toks[t].pp_start || !ts.is(t + 1, "define") || !ts.is_ident(t + 2)) {
      continue;
    }
    bool is_entry = false;
    for (int k = t + 3; k < ts.directive_end(t); k++) {
      is_entry |= ts.is(k, "mainImage") || ts.is(k, "mainCubemap");
    }
    if (!is_entry) {
      continue;
    }
    const StringRef name = ts.text(t + 2);
    for (int64_t at = s.find(name); at != StringRef::not_found; at = s.find(name, at + 1)) {
      if (match_ident_at(s, at, name)) {
        return true;
      }
    }
  }
  return false;
}

/* NS pressure Jacobi: 4-neighbor average, often `* 0.25`, reading the same
 * buffer as last frame so the solve accumulates. ShaderToy does one step per
 * 60Hz frame; the Texture Editor ticks with the scene (often 24fps) so Chimera
 * (4tGfDW) under-solves and the two emitters collapse into a center seam.
 * Extra even-count self-iterations keep ping-pong on the same write index. */
static bool looks_like_pressure_jacobi(const char *code)
{
  if (code == nullptr || code[0] == '\0') {
    return false;
  }
  const StringRef s(code);
  if (s.find("uintBitsToFloat") != StringRef::not_found ||
      s.find("Clusterize") != StringRef::not_found ||
      s.find("unpackParticle") != StringRef::not_found ||
      s.find("reproject") != StringRef::not_found ||
      s.find("*dt") != StringRef::not_found ||
      s.find("* dt") != StringRef::not_found ||
      s.find("*iTimeDelta") != StringRef::not_found)
  {
    return false;
  }
  const bool avg = s.find("0.25") != StringRef::not_found ||
                   s.find("*.25") != StringRef::not_found ||
                   s.find("* .25") != StringRef::not_found ||
                   s.find("/4.0") != StringRef::not_found ||
                   s.find("/ 4.0") != StringRef::not_found ||
                   s.find("/4.") != StringRef::not_found;
  if (!avg) {
    return false;
  }
  const bool dx = s.find("vec2(1") != StringRef::not_found ||
                  s.find("vec2(-1") != StringRef::not_found ||
                  s.find("vec2( 1") != StringRef::not_found ||
                  s.find("vec2(1.0") != StringRef::not_found;
  const bool dy = s.find("vec2(0,1") != StringRef::not_found ||
                  s.find("vec2(0, 1") != StringRef::not_found ||
                  s.find("vec2(0,-1") != StringRef::not_found ||
                  s.find("vec2(0, -1") != StringRef::not_found ||
                  s.find("vec2(0.0,1") != StringRef::not_found;
  int lods = 0;
  for (int64_t p = 0; p < s.size();) {
    const int64_t hit = s.find("textureLod", p);
    if (hit == StringRef::not_found) {
      break;
    }
    lods++;
    p = hit + 10;
    if (lods >= 4) {
      break;
    }
  }
  return (lods >= 4) || (dx && dy);
}

static bool buffer_stores_particle_ids(const char *code)
{
  /* MlVfDR Buffer B: voronoi particles live in .xy as pixel positions. Linear
   * filtering after `U = U-T(U).xy` blends two IDs and the mosaic washes out
   * in a few frames. Site sampler is nearest. */
  if (code == nullptr || code[0] == '\0') {
    return false;
  }
  const StringRef s(code);
  if (s.find("floor(U/10") != StringRef::not_found ||
      s.find("floor(U / 10") != StringRef::not_found)
  {
    return true;
  }
  /* wdsGWS: pixel is either a ball or EMPTY=1024. Linear filtering makes
   * `t2.x == EMPTY` never true and the fluid evaporates. */
  if (s.find("EMPTY") != StringRef::not_found) {
    return true;
  }
  return s.find("1e5") != StringRef::not_found && s.find("swap") != StringRef::not_found;
}

static const char *storage_buffer_code(const NodeImageShaderToy &storage, const int buffer)
{
  switch (buffer) {
    case 0:
      return storage.code_buffer_a;
    case 1:
      return storage.code_buffer_b;
    case 2:
      return storage.code_buffer_c;
    case 3:
      return storage.code_buffer_d;
    default:
      return nullptr;
  }
}

/* Whether Buffer A–D slot `buffer` holds a ShaderToy Cubemap pass. The importer records it
 * as `CUBE:<letters>` in the channel map, because the code cannot tell: a Cubemap tab may
 * keep a `mainImage` next to `mainCubemap` (NtjSzd), and a Buffer tab may have a helper
 * called `mainCubemap` (3s33Rn). Hand-pasted code has no record and goes by its content. */
static bool buffer_is_cubemap_pass(const NodeImageShaderToy &storage, const int buffer)
{
  const char *code = storage_buffer_code(storage, buffer);
  const StringRef map(storage.channel_map);
  for (int64_t at = map.find("CUBE:"); at != StringRef::not_found; at = map.find("CUBE:", at + 1))
  {
    if (at > 0 && map[at - 1] != '|') {
      continue;
    }
    StringRef letters = map.substr(at + 5);
    const int64_t end = letters.find('|');
    if (end != StringRef::not_found) {
      letters = letters.substr(0, end);
    }
    return code != nullptr && code[0] != '\0' &&
           letters.find(char('A' + buffer)) != StringRef::not_found;
  }
  return is_cubemap_only_pass(code);
}

static bool shadertoy_uses_empty_id(const NodeImageShaderToy &st)
{
  auto has = [](const char *c) -> bool {
    return c != nullptr && c[0] != '\0' && StringRef(c).find("EMPTY") != StringRef::not_found;
  };
  return has(st.code_common) || has(st.code_buffer_a) || has(st.code_buffer_b) ||
         has(st.code_buffer_c) || has(st.code_buffer_d) || has(st.code_image);
}

static uint8_t detect_nearest_channel_mask(const NodeImageShaderToy &storage, const char pass_tag)
{
  uint8_t mask = 0;
  const bool empty_id = shadertoy_uses_empty_id(storage);
  for (int c = 0; c < 4; c++) {
    const ChannelBind b = parse_channel(storage, pass_tag, c);
    if (b.nearest) {
      mask |= uint8_t(1u << c);
    }
    int buf = -1;
    if (b.kind == ChannelKind::Buffer) {
      buf = b.buffer;
    }
    else if (b.kind == ChannelKind::None) {
      buf = c;
    }
    if (buf >= 0 && buffer_stores_particle_ids(storage_buffer_code(storage, buf))) {
      mask |= uint8_t(1u << c);
    }
    if (empty_id && (b.kind == ChannelKind::Buffer || b.kind == ChannelKind::None)) {
      mask |= uint8_t(1u << c);
    }
  }
  return mask;
}

static uint8_t detect_wrap_channel_mask(const NodeImageShaderToy &storage, const char pass_tag)
{
  uint8_t mask = 0;
  const bool empty_id = shadertoy_uses_empty_id(storage);
  for (int c = 0; c < 4; c++) {
    const ChannelBind b = parse_channel(storage, pass_tag, c);
    if (b.repeat) {
      mask |= uint8_t(1u << c);
    }
    int buf = -1;
    if (b.kind == ChannelKind::Buffer) {
      buf = b.buffer;
    }
    else if (b.kind == ChannelKind::None) {
      buf = c;
    }
    const char *code = (buf >= 0) ? storage_buffer_code(storage, buf) : nullptr;
    /* wdsGWS Buffer A/B are REPEAT so balls wrap grids. Image is CLAMP — wrapping
     * the zoomed corner samples the opposite edge and the cells look wrong. */
    const bool image_pass = (pass_tag == 'I');
    if (!image_pass && code && StringRef(code).find("EMPTY") != StringRef::not_found) {
      mask |= uint8_t(1u << c);
    }
    if (!image_pass && empty_id &&
        (b.kind == ChannelKind::Buffer || b.kind == ChannelKind::None))
    {
      mask |= uint8_t(1u << c);
    }
  }
  return mask;
}

static void apply_buffer_channel_sampler_types(const NodeImageShaderToy &storage,
                                               const char pass_tag,
                                               uint8_t &cube_mask,
                                               uint8_t &vol_mask)
{
  /* Buffer A–D are 2D ping-pong (unless that pass is Cubemap A). Vec3 samples in Common
   * must not promote a mapped Buffer channel to samplerCube/sampler3D — that binds the
   * dummy sky/volume instead of the sim, which looks like noise (mstfzS). */
  /* A pass loaded from ShaderToy JSON lists its inputs, and the site declares every one
   * that is not a cubemap or volume as sampler2D. Guessing from the code can only be wrong
   * there: `texture(iChannel0, reflect(rd, n).xy)` is not a cubemap lookup. */
  const bool from_site = !channel_map_slice(storage, pass_tag).is_empty();
  for (int c = 0; c < 4; c++) {
    const ChannelBind bind = parse_channel(storage, pass_tag, c);
    const uint8_t bit = uint8_t(1u << c);
    if (bind.kind == ChannelKind::Cubemap) {
      cube_mask |= bit;
      vol_mask &= uint8_t(~bit);
      continue;
    }
    if (bind.kind == ChannelKind::Volume) {
      vol_mask |= bit;
      cube_mask &= uint8_t(~bit);
      continue;
    }
    if (bind.kind == ChannelKind::None) {
      /* Empty spec auto-wires iChannelN -> Buffer N. Cubemap A must stay cube. */
      if (buffer_is_cubemap_pass(storage, c)) {
        cube_mask |= bit;
        vol_mask &= uint8_t(~bit);
      }
      continue;
    }
    if (bind.kind != ChannelKind::Buffer) {
      if (from_site) {
        cube_mask &= uint8_t(~bit);
        vol_mask &= uint8_t(~bit);
      }
      continue;
    }
    if (buffer_is_cubemap_pass(storage, bind.buffer)) {
      cube_mask |= bit;
      vol_mask &= uint8_t(~bit);
    }
    else {
      cube_mask &= uint8_t(~bit);
      vol_mask &= uint8_t(~bit);
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Compositor operation
 * \{ */

using namespace blender::compositor;

class ShaderToyOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    return this->context().get_compositing_domain();
  }

  static float read_float_param(const Result &r, const float fallback)
  {
    if (r.is_single_value()) {
      return r.get_single_value_default<float>();
    }
    if (r.is_allocated()) {
      return r.load_pixel<float, true>(int2(0));
    }
    return fallback;
  }

  static bool read_bool_param(const Result &r, const bool fallback)
  {
    if (r.is_single_value()) {
      return r.get_single_value_default<bool>();
    }
    if (r.is_allocated()) {
      return r.load_pixel<bool, true>(int2(0));
    }
    return fallback;
  }

  void fill_error_color(Result &color_out, const Color color)
  {
    const Domain domain = this->compute_domain();
    color_out.allocate_texture(domain);
    if (this->context().use_gpu() && color_out.is_stored_on_gpu()) {
      const float rgba[4] = {color.r, color.g, color.b, color.a};
      GPU_texture_clear(color_out, GPU_DATA_FLOAT, rgba);
      return;
    }
    color_out.allocate_texture(domain, false, ResultStorageType::CPUImage);
    const int2 size = domain.data_size;
    parallel_for(size, [&](const int2 texel) { color_out.store_pixel(texel, color); });
    if (this->context().use_gpu()) {
      Result gpu = color_out.upload_to_gpu(true);
      color_out.release();
      color_out.share_data(gpu);
      gpu.release();
    }
  }

  void bind_sampler(gpu::Shader *shader,
                    const char *name,
                    gpu::Texture *tex,
                    const bool repeat,
                    const bool cube,
                    const bool volume,
                    const bool nearest,
                    const bool mipmap)
  {
    if (tex == nullptr) {
      gpu::Texture *site = cube ? site_cubemap_texture() : nullptr;
      tex = cube ? (site ? site : dummy_cube_texture()) :
                   (volume ? dummy_volume_texture() : dummy_noise_texture());
    }
    if (tex == nullptr) {
      return;
    }
    const int unit = GPU_shader_get_sampler_binding(shader, name);
    GPUSamplerState state = GPUSamplerState::default_sampler();
    /* texelFetch ignores filter. Fluids need bilinear (Dry Ice 2 became a
     * pixel grid on nearest). Particle IDs use `_st_fetch_px` / texelFetch so
     * this sampler stays linear — Vulkan nearest+implicit LOD collapsed
     * MlVfDR to a flat jet-green. */
    (void)nearest;
    if (mipmap) {
      state.filtering = GPUSamplerFiltering(int(GPU_SAMPLER_FILTERING_LINEAR) |
                                            int(GPU_SAMPLER_FILTERING_MIPMAP));
    }
    else {
      /* 1-mip ping-pong buffers: LINEAR_MIPMAP is undefined / black on Vulkan. */
      state.filtering = GPU_SAMPLER_FILTERING_LINEAR;
    }
    /* Cubemaps must clamp — REPEAT seams the edges. */
    const GPUSamplerExtendMode wrap = (cube || !repeat) ? GPU_SAMPLER_EXTEND_MODE_EXTEND :
                                                          GPU_SAMPLER_EXTEND_MODE_REPEAT;
    state.extend_x = wrap;
    state.extend_yz = wrap;
    GPU_texture_bind_ex(tex, state, unit);
  }

  void unbind_sampler(gpu::Texture *tex)
  {
    if (tex) {
      GPU_texture_unbind(tex);
    }
  }

  void dispatch_pass(gpu::Shader *shader,
                     gpu::Texture *out_tex,
                     gpu::Texture *ch[4],
                     const bool ch_repeat[4],
                     const bool ch_nearest[4],
                     const uint8_t cube_mask,
                     const uint8_t vol_mask,
                     const uint8_t fetch_mask,
                     const bool cubemap_out,
                     const int2 size,
                     const float i_time,
                     const float i_dt,
                     const int i_frame,
                     const float i_frame_rate,
                     const NodeImageShaderToy &st)
  {
    if (out_tex == nullptr || shader == nullptr) {
      return;
    }

    GPU_shader_bind(shader);
    const float i_res[4] = {float(size.x), float(size.y), 1.0f, 0.0f};
    GPU_shader_uniform_4fv(shader, "_st_ires", i_res);
    GPU_shader_uniform_1f(shader, "iTime", i_time);
    GPU_shader_uniform_1f(shader, "iTimeDelta", i_dt);
    GPU_shader_uniform_1i(shader, "iFrame", i_frame);
    GPU_shader_uniform_1f(shader, "iFrameRate", i_frame_rate);
    /* ShaderToy iMouse: (0,0,0,0) until the first click. xy = pointer in pixels,
     * zw = click origin with z>0 while held. One N-panel Mouse slider is enough:
     * (0,0) keeps the site default; any other value is a held click at that point. */
    float i_mouse[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    const float nx = math::clamp(st.mouse_pos[0], 0.0f, 1.0f);
    const float ny = math::clamp(st.mouse_pos[1], 0.0f, 1.0f);
    if (nx > 1.0e-6f || ny > 1.0e-6f) {
      i_mouse[0] = nx * i_res[0];
      i_mouse[1] = ny * i_res[1];
      i_mouse[2] = i_mouse[0];
      i_mouse[3] = i_mouse[1];
    }
    GPU_shader_uniform_4fv(shader, "iMouse", i_mouse);
    const float i_date[4] = {0.0f, 0.0f, 0.0f, i_time};
    GPU_shader_uniform_4fv(shader, "iDate", i_date);

    gpu::Texture *bound[4] = {nullptr, nullptr, nullptr, nullptr};
    auto bind_ch = [&](const int c, const char *name) {
      const bool cube = (cube_mask & (1u << c)) != 0;
      const bool volume = (vol_mask & (1u << c)) != 0;
      const bool fetch = (fetch_mask & (1u << c)) != 0;
      gpu::Texture *dummy2d = dummy_noise_texture();
      gpu::Texture *tex = ch[c];
      /* Dummy 2D keeps a mip chain for `textureLod`, but world-space `texture()`
       * (XsffWj tex3D) has huge derivatives and would sample the 1×1 mip (mud). */
      const bool mipmap = cube || volume;
      if (tex == nullptr) {
        gpu::Texture *site = cube ? site_cubemap_texture() : nullptr;
        tex = cube ? (site ? site : dummy_cube_texture()) :
                     (volume ? dummy_volume_texture() : dummy2d);
      }
      const bool nearest = fetch || (ch_nearest && ch_nearest[c]);
      this->bind_sampler(
          shader, name, tex, ch_repeat[c], cube, volume, nearest && !cube && !volume, mipmap);
      bound[c] = tex;
    };
    bind_ch(0, "iChannel0");
    bind_ch(1, "iChannel1");
    bind_ch(2, "iChannel2");
    bind_ch(3, "iChannel3");

    gpu::FrameBuffer *fb = shadertoy_fb();
    gpu::FrameBuffer *prev = GPU_framebuffer_active_get();
    GPU_scissor_test(false);
    GPU_depth_test(GPU_DEPTH_NONE);
    GPU_blend(GPU_BLEND_NONE);
    GPU_face_culling(GPU_CULL_NONE);

    gpu::Batch *tri = shadertoy_tri();
    GPU_batch_set_shader(tri, shader);

    const int faces = cubemap_out ? 6 : 1;
    for (int face = 0; face < faces; face++) {
      GPU_shader_uniform_1i(shader, "iCubeFace", face);
      if (cubemap_out) {
        GPU_framebuffer_ensure_config(
            &fb, {GPU_ATTACHMENT_NONE, GPU_ATTACHMENT_TEXTURE_CUBEFACE(out_tex, face)});
      }
      else {
        GPU_framebuffer_ensure_config(&fb,
                                      {GPU_ATTACHMENT_NONE, GPU_ATTACHMENT_TEXTURE(out_tex)});
      }
      GPU_framebuffer_bind_no_srgb(fb);
      const bool empty_id = shadertoy_uses_empty_id(st);
      GPU_framebuffer_clear_color(fb,
                                  empty_id ? double4(1024.0, 1024.0, 1024.0, 1.0) :
                                             double4(0.0, 0.0, 0.0, 1.0));
      GPU_viewport(0, 0, size.x, size.y);
      GPU_batch_draw(tri);
    }

    GPU_memory_barrier(GPU_BARRIER_FRAMEBUFFER | GPU_BARRIER_TEXTURE_FETCH |
                       GPU_BARRIER_SHADER_IMAGE_ACCESS);
    if (cubemap_out && out_tex) {
      GPU_texture_update_mipmap_chain(out_tex);
    }

    if (prev) {
      GPU_framebuffer_bind(prev);
    }
    else {
      GPU_framebuffer_restore();
    }

    this->unbind_sampler(bound[0]);
    this->unbind_sampler(bound[1]);
    this->unbind_sampler(bound[2]);
    this->unbind_sampler(bound[3]);
    GPU_shader_unbind();
  }

  void resolve_channels(const char pass_tag,
                        const NodeImageShaderToy &storage,
                        BufferState &bufs,
                        const int pass_buffer,
                        const int prev_index,
                        const int latest_index,
                        const uint8_t cube_mask,
                        const uint8_t vol_mask,
                        const uint8_t tex_like_mask,
                        gpu::Texture *out_ch[4],
                        bool out_repeat[4],
                        bool out_nearest[4])
  {
    gpu::Texture *dummy = dummy_noise_texture();
    gpu::Texture *dummy_cube = dummy_cube_texture();
    gpu::Texture *dummy_black = dummy_black_texture();
    gpu::Texture *dummy_vol = dummy_volume_texture();
    const bool image_pass = pass_buffer < 0;
    const char *pass_code = image_pass ? storage.code_image :
                                         storage_buffer_code(storage, pass_buffer);
    const uint8_t kb_mask = detect_keyboard_mask(storage.code_common, pass_code);
    const uint8_t wrap_mask = detect_wrap_channel_mask(storage, pass_tag);
    auto linked_channel_tex = [&](const int c) -> gpu::Texture * {
      char sock_name[16];
      SNPRINTF(sock_name, "Channel %d", c);
      if (!socket_is_linked(this->node(), sock_name)) {
        return nullptr;
      }
      Result &in = this->get_input(sock_name);
      if (in.is_allocated() && in.is_stored_on_gpu()) {
        return in.gpu_texture();
      }
      return nullptr;
    };

    auto find_cube_buffer = [&](const int hint) -> int {
      if (hint >= 0 && hint < 4 && buffer_is_cubemap_pass(storage, hint)) {
        return hint;
      }
      for (int b = 0; b < 4; b++) {
        if (buffer_is_cubemap_pass(storage, b)) {
          return b;
        }
      }
      return -1;
    };

    for (int c = 0; c < 4; c++) {
      const ChannelBind bind = parse_channel(storage, pass_tag, c);
      const bool cube = (cube_mask & (1u << c)) != 0;
      const bool volume = (vol_mask & (1u << c)) != 0;
      out_repeat[c] = false;
      out_nearest[c] = bind.nearest;

      auto bind_buffer = [&](const int buffer) {
        const bool src_cube = buffer_is_cubemap_pass(storage, buffer);
        if (src_cube) {
          const int idx = image_pass ? latest_index :
                                       ((buffer < pass_buffer) ? latest_index : prev_index);
          gpu::Texture *t = bufs.cube[buffer][idx];
          out_ch[c] = t ? t : dummy_cube;
          out_repeat[c] = false;
          return;
        }
        if (bind.nearest || buffer_stores_particle_ids(storage_buffer_code(storage, buffer))) {
          out_nearest[c] = true;
        }
        if (image_pass) {
          gpu::Texture *t = bufs.tex[buffer][latest_index];
          out_ch[c] = t ? t : dummy;
          out_repeat[c] = (wrap_mask & uint8_t(1u << c)) != 0;
        }
        else {
          const bool use_latest = (buffer < pass_buffer);
          const int idx = use_latest ? latest_index : prev_index;
          gpu::Texture *t = bufs.tex[buffer][idx];
          out_ch[c] = t ? t : dummy;
          out_repeat[c] = (wrap_mask & uint8_t(1u << c)) != 0;
        }
      };

      /* Keyboard UV on this channel: never dummy noise. Xst3Dj reset() reads
       * `vec2(32.5/256.0, 0.5)` and our RGBA noise at that texel is ~0.55, so
       * reset stays true and the CA never leaves the init frame. */
      if ((kb_mask & uint8_t(1u << c)) != 0 && bind.kind != ChannelKind::Buffer) {
        out_ch[c] = dummy_black;
        out_repeat[c] = false;
        continue;
      }

      if (bind.kind == ChannelKind::Cubemap) {
        gpu::Texture *site = site_cubemap_texture();
        out_ch[c] = site ? site : dummy_cube;
        out_repeat[c] = false;
        continue;
      }
      if (bind.kind == ChannelKind::Volume) {
        out_ch[c] = dummy_vol;
        out_repeat[c] = true;
        continue;
      }
      if (bind.kind == ChannelKind::Skip || bind.kind == ChannelKind::Keyboard) {
        gpu::Texture *site = cube ? site_cubemap_texture() : nullptr;
        out_ch[c] = cube ? (site ? site : dummy_cube) : (volume ? dummy_vol : dummy_black);
        out_repeat[c] = false;
        continue;
      }
      if (bind.kind == ChannelKind::Texture) {
        if (cube) {
          gpu::Texture *site = site_cubemap_texture();
          out_ch[c] = site ? site : dummy_cube;
          out_repeat[c] = false;
        }
        else if (volume) {
          out_ch[c] = dummy_vol;
          out_repeat[c] = true;
        }
        else if (gpu::Texture *tex = linked_channel_tex(c)) {
          out_ch[c] = tex;
          out_repeat[c] = true;
        }
        else {
          out_ch[c] = dummy;
          out_repeat[c] = true;
        }
        continue;
      }
      if (bind.kind == ChannelKind::Buffer) {
        bind_buffer(bind.buffer);
        continue;
      }

      if (cube) {
        const int cube_buf = find_cube_buffer(c);
        if (cube_buf >= 0) {
          bind_buffer(cube_buf);
        }
        else {
          gpu::Texture *site = site_cubemap_texture();
          out_ch[c] = site ? site : dummy_cube;
          out_repeat[c] = false;
        }
        continue;
      }
      if (volume) {
        out_ch[c] = dummy_vol;
        out_repeat[c] = true;
        continue;
      }

      out_ch[c] = dummy;
      out_repeat[c] = true;

      const bool want_site_tex = (tex_like_mask & (1u << c)) != 0;
      int buffer = -1;
      if (pass_has_code(storage.code_common, storage_buffer_code(storage, c)) &&
          !buffer_is_cubemap_pass(storage, c))
      {
        buffer = c;
      }

      if (image_pass) {
        if (want_site_tex) {
          if (gpu::Texture *tex = linked_channel_tex(c)) {
            out_ch[c] = tex;
            out_repeat[c] = true;
          }
          continue;
        }
        if (buffer >= 0) {
          bind_buffer(buffer);
          continue;
        }
        if (gpu::Texture *tex = linked_channel_tex(c)) {
          out_ch[c] = tex;
          out_repeat[c] = true;
        }
      }
      else {
        if (gpu::Texture *tex = linked_channel_tex(c)) {
          out_ch[c] = tex;
          out_repeat[c] = true;
          continue;
        }
        if (buffer >= 0) {
          bind_buffer(buffer);
        }
      }
    }
  }

  void execute() override
  {
    Result &color_out = this->get_result("Color");
    if (!color_out.should_compute()) {
      return;
    }

    const NodeImageShaderToy &storage = node_storage(this->node());
    if (!this->context().use_gpu()) {
      this->add_warning(NodeWarningType::Error, "ShaderToy requires GPU evaluation");
      this->fill_error_color(color_out, Color(1.0f, 0.0f, 0.4f, 1.0f));
      return;
    }

    const Domain domain = this->compute_domain();
    const int2 size = domain.data_size;
    if (size.x < 1 || size.y < 1) {
      this->fill_error_color(color_out, Color(0.0f, 0.0f, 0.0f, 1.0f));
      return;
    }

    const Vector<ShaderParam> params = params_from_storage(storage);

    /* ShaderToy physics/camera are authored around ~60Hz. iTime follows tick count, not
     * Blender fps, so 24fps files don't run 12 SPH substeps per timeline frame. */
    const float st_fps = 60.0f;
    const float i_frame_rate = st_fps;
    const int scene_frame = this->context().get_frame_number();
    const float scale = math::max(read_float_param(this->get_input("Time Scale"), 1.0f), 0.0f);
    const float time_scale = (scale > 0.0f) ? scale : 1.0f;
    const bool paused = read_bool_param(this->get_input("Pause"), false);

    /* ShaderToy buffers are float (Rainforest stores a camera matrix in 3 texels). */
    const gpu::TextureFormat fmt = gpu::TextureFormat::SFLOAT_32_32_32_32;
    /* wdsGWS: empty cells are EMPTY=1024, not 0. Clearing to 0 makes packing treat
     * every texel as a ball at the origin and they vanish after a few frames. */
    const bool empty_id = shadertoy_uses_empty_id(storage);
    const float buf_fill = empty_id ? 1024.0f : 0.0f;
    BufferState &bufs = get_buffer_state(
        this->node(), size, fmt, storage.reset_buffers != 0, buf_fill);
    if (storage.reset_buffers) {
      const_cast<NodeImageShaderToy &>(storage).reset_buffers = 0;
    }
    if (empty_id && bufs.shader_frame == 0) {
      bufs.clear_contents(buf_fill);
    }

    /* Scrubbing backward is ShaderToy rewind: clear history so TAA/reproject doesn't explode. */
    if (!paused && bufs.last_frame != std::numeric_limits<int>::min() &&
        scene_frame < bufs.last_frame)
    {
      bufs.clear_contents(buf_fill);
    }

    /* Only tick buffers when the scene frame advances (or first cook after Compile).
     * Same-frame recooks must NOT flip ping-pong — that was reading the wrong target
     * and feeding Rainforest's reprojection garbage camera data. */
    const bool simulate = !paused && (bufs.last_frame != scene_frame);

    const char *common = storage.code_common;
    struct PassDesc {
      const char tag;
      const char *code;
      const char *debug;
      int buffer; /* -1 = Image */
    };
    const PassDesc passes[5] = {
        {'A', storage.code_buffer_a, "shadertoy_A", 0},
        {'B', storage.code_buffer_b, "shadertoy_B", 1},
        {'C', storage.code_buffer_c, "shadertoy_C", 2},
        {'D', storage.code_buffer_d, "shadertoy_D", 3},
        {'I', storage.code_image, "shadertoy_Image", -1},
    };

    std::string errors;
    bool buffer_failed = false;

    /* One physics tick per displayed Blender frame — same as ShaderToy (one Buffer
     * pass per frame). Catching up to 60Hz with 12 substeps made mstfzS eject every
     * particle in a few timeline frames (`dt` is 0.7 cells/tick).
     * Raymarch buffers (dl2fzz) are not a sim: they must still run once after
     * Compile/reset, otherwise Image DOF-samples a cleared target and is black. */
    int ticks = simulate ? 1 : 0;
    if (ticks == 0 && bufs.shader_frame == 0) {
      for (int p = 0; p < 4; p++) {
        if (pass_has_code(common, passes[p].code)) {
          ticks = 1;
          break;
        }
      }
    }

    int i_frame = math::max(bufs.shader_frame - 1, 0);
    float i_time = (bufs.shader_frame > 0) ? bufs.last_time : 0.0f;
    float i_dt = 1.0f / st_fps;

    for (int tick = 0; tick < ticks; tick++) {
      const int read_i = bufs.ping;
      const int write_i = 1 - bufs.ping;
      i_frame = bufs.shader_frame;
      i_time = float(i_frame) / st_fps * time_scale;
      i_dt = 1.0f / st_fps;
      for (int p = 0; p < 4; p++) {
        if (!pass_has_code(common, passes[p].code)) {
          continue;
        }
        CompiledPass &compiled = compile_pass(
            passes[p].debug, common, passes[p].code, params, fmt, false, &storage, passes[p].tag);
        if (compiled.shader == nullptr) {
          if (!errors.empty()) {
            errors += " | ";
          }
          errors += passes[p].debug;
          errors += ": ";
          errors += compiled.error;
          buffer_failed = true;
          continue;
        }
        gpu::Texture *ch[4];
        bool ch_repeat[4];
        bool ch_nearest[4] = {false, false, false, false};
        this->resolve_channels(passes[p].tag,
                               storage,
                               bufs,
                               p,
                               read_i,
                               write_i,
                               compiled.cube_mask,
                               compiled.vol_mask,
                               compiled.tex_like_mask,
                               ch,
                               ch_repeat,
                               ch_nearest);
        gpu::Texture *out_tex = bufs.tex[p][write_i];
        int2 pass_size = size;
        if (compiled.cubemap_pass) {
          out_tex = bufs.ensure_cube(p, write_i, fmt);
          pass_size = int2(bufs.cube_size, bufs.cube_size);
        }
        this->dispatch_pass(compiled.shader,
                            out_tex,
                            ch,
                            ch_repeat,
                            ch_nearest,
                            compiled.cube_mask,
                            compiled.vol_mask,
                            compiled.fetch_mask,
                            compiled.cubemap_pass,
                            pass_size,
                            i_time,
                            i_dt,
                            i_frame,
                            i_frame_rate,
                            storage);

        /* Extra Jacobi on self-reading pressure buffers. Even extra count so
         * the last write stays at write_i and later passes still see it. */
        bool reads_self = false;
        for (int c = 0; c < 4; c++) {
          if (ch[c] == bufs.tex[p][read_i] || ch[c] == bufs.tex[p][write_i]) {
            reads_self = true;
            break;
          }
        }
        if (!compiled.cubemap_pass && reads_self &&
            looks_like_pressure_jacobi(passes[p].code))
        {
          constexpr int extra = 2;
          int local_r = read_i;
          int local_w = write_i;
          for (int e = 0; e < extra; e++) {
            const int tmp = local_r;
            local_r = local_w;
            local_w = tmp;
            gpu::Texture *ch_j[4] = {ch[0], ch[1], ch[2], ch[3]};
            for (int c = 0; c < 4; c++) {
              if (ch[c] == bufs.tex[p][read_i] || ch[c] == bufs.tex[p][write_i]) {
                ch_j[c] = bufs.tex[p][local_r];
              }
            }
            this->dispatch_pass(compiled.shader,
                                bufs.tex[p][local_w],
                                ch_j,
                                ch_repeat,
                                ch_nearest,
                                compiled.cube_mask,
                                compiled.vol_mask,
                                compiled.fetch_mask,
                                false,
                                pass_size,
                                i_time,
                                i_dt,
                                i_frame,
                                i_frame_rate,
                                storage);
          }
        }
      }
      bufs.ping = write_i;
      bufs.last_frame = scene_frame;
      bufs.last_time = i_time;
      bufs.shader_frame += 1;
    }

    i_frame = math::max(bufs.shader_frame - 1, 0);
    i_time = (bufs.shader_frame > 0) ? bufs.last_time : 0.0f;
    i_dt = 1.0f / st_fps;

    color_out.allocate_texture(domain);
    /* Scene-linear: Texture Editor uses the blend file view transform. */
    color_out.meta_data.is_non_color_data = false;
    if (buffer_failed) {
      this->fill_error_color(color_out, Color(1.0f, 0.15f, 0.45f, 1.0f));
    }
    else if (pass_has_code(common, storage.code_image)) {
      gpu::Texture *out_gpu = color_out.gpu_texture();
      gpu::Texture *cache = bufs.ensure_image_cache(size, fmt);
      DefaultHash<StringRef> key_hash;
      uint64_t cook = key_hash(format_param_defines(params));
      cook = cook * 33 + uint64_t(i_frame);
      cook = cook * 33 + uint64_t(size.x) + (uint64_t(size.y) << 20);
      cook = cook * 33 + uint64_t(int(i_time * 240.0f));
      cook = cook * 33 + uint64_t(int(storage.mouse_pos[0] * 10000.0f) & 0xffff);
      cook = cook * 33 + uint64_t(int(storage.mouse_pos[1] * 10000.0f) & 0xffff);
      /* Texture Editor recooks the same frame many times per mouse move. Skip
       * compile + raymarch; only blit the cached Image. */
      const bool reuse = (cache != nullptr && cook == bufs.image_cook_key && !simulate);
      if (reuse) {
        if (cache && out_gpu && cache != out_gpu) {
          GPU_texture_copy(out_gpu, cache);
        }
      }
      else {
        CompiledPass &compiled = compile_pass(
            "shadertoy_Image", common, storage.code_image, params, fmt, true, &storage, 'I');
        if (compiled.shader == nullptr) {
          if (!errors.empty()) {
            errors += " | ";
          }
          errors += compiled.error;
          this->fill_error_color(color_out, Color(1.0f, 0.15f, 0.45f, 1.0f));
        }
        else {
          gpu::Texture *ch[4];
          bool ch_repeat[4];
          bool ch_nearest[4] = {false, false, false, false};
          /* Image always samples the latest completed buffers (ping after a tick). */
          this->resolve_channels('I',
                                 storage,
                                 bufs,
                                 -1,
                                 1 - bufs.ping,
                                 bufs.ping,
                                 compiled.cube_mask,
                                 compiled.vol_mask,
                                 compiled.tex_like_mask,
                                 ch,
                                 ch_repeat,
                                 ch_nearest);
          this->dispatch_pass(compiled.shader,
                              cache ? cache : out_gpu,
                              ch,
                              ch_repeat,
                              ch_nearest,
                              compiled.cube_mask,
                              compiled.vol_mask,
                              compiled.fetch_mask,
                              false,
                              size,
                              i_time,
                              i_dt,
                              i_frame,
                              i_frame_rate,
                              storage);
          bufs.image_cook_key = cook;
          if (cache && out_gpu && cache != out_gpu) {
            GPU_texture_copy(out_gpu, cache);
          }
        }
      }
    }
    else {
      this->fill_error_color(color_out, Color(0.1f, 0.1f, 0.12f, 1.0f));
      errors = "No Image pass (mainImage / mainCubemap missing)";
    }

    if (!errors.empty()) {
      NodeImageShaderToy &mut = const_cast<NodeImageShaderToy &>(storage);
      set_owned_str(&mut.error_log, errors.c_str());
    }
    else if (storage.error_log) {
      set_owned_str(&const_cast<NodeImageShaderToy &>(storage).error_log, nullptr);
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new ShaderToyOperation(context, node);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Operators
 * \{ */

static bool shadertoy_node_poll(bContext *C)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree || snode->edittree->type != NTREE_IMAGE) {
    return false;
  }
  const bNode *node = blender::bke::node_get_active(*snode->edittree);
  return node && node->is_type("ImageNodeShaderToy"_ustr);
}

static wmOperatorStatus shadertoy_reset_buffers_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return OPERATOR_CANCELLED;
  }
  bNode *node = blender::bke::node_get_active(*snode->edittree);
  if (!node || !node->is_type("ImageNodeShaderToy"_ustr)) {
    BKE_report(op->reports, RPT_ERROR, "Select a ShaderToy node");
    return OPERATOR_CANCELLED;
  }
  NodeImageShaderToy &storage = node_storage(*node);
  storage.reset_buffers = 1;
  {
    std::lock_guard lock(buffer_cache_mutex());
    buffer_cache().remove(buffer_key(*node));
  }
  WM_event_add_notifier(C, NC_NODE | NA_EDITED, snode->edittree);
  ED_area_tag_refresh(CTX_wm_area(C));
  BKE_report(op->reports, RPT_INFO, "ShaderToy buffers cleared");
  return OPERATOR_FINISHED;
}

static void NODE_OT_shadertoy_reset_buffers(wmOperatorType *ot)
{
  ot->name = "Reset ShaderToy Buffers";
  ot->idname = "NODE_OT_shadertoy_reset_buffers";
  ot->description = "Clear Buffer A–D ping-pong textures for the active ShaderToy node";
  ot->exec = shadertoy_reset_buffers_exec;
  ot->poll = shadertoy_node_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static std::string compile_all_passes(const NodeImageShaderToy &storage)
{
  if (!pass_has_code(storage.code_common, storage.code_image)) {
    return "Image pass has no mainImage() / mainCubemap()";
  }
  const Vector<ShaderParam> params = params_from_storage(storage);
  const gpu::TextureFormat fmt = gpu::TextureFormat::SFLOAT_32_32_32_32;
  const char *common = storage.code_common;
  struct PassSpec {
    const char *code;
    const char *debug;
    char tag;
  };
  const PassSpec passes[] = {
      {storage.code_buffer_a, "Buffer A", 'A'},
      {storage.code_buffer_b, "Buffer B", 'B'},
      {storage.code_buffer_c, "Buffer C", 'C'},
      {storage.code_buffer_d, "Buffer D", 'D'},
      {storage.code_image, "Image", 'I'},
  };
  std::string errors;
  for (const PassSpec &pass : passes) {
    if (pass.code != storage.code_image && !pass_has_code(common, pass.code)) {
      continue;
    }
    const bool force_opaque = (pass.code == storage.code_image);
    CompiledPass &compiled = compile_pass(
        pass.debug, common, pass.code, params, fmt, force_opaque, &storage, pass.tag);
    if (compiled.shader == nullptr) {
      if (!errors.empty()) {
        errors += " | ";
      }
      errors += pass.debug;
      errors += ": ";
      errors += compiled.error;
    }
  }
  return errors;
}

static wmOperatorStatus shadertoy_compile_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return OPERATOR_CANCELLED;
  }
  bNode *node = blender::bke::node_get_active(*snode->edittree);
  if (!node || !node->is_type("ImageNodeShaderToy"_ustr)) {
    BKE_report(op->reports, RPT_ERROR, "Select a ShaderToy node");
    return OPERATOR_CANCELLED;
  }
  NodeImageShaderToy &storage = node_storage(*node);
  sync_params_from_code(storage);
  storage.reset_buffers = 1;
  {
    std::lock_guard lock(buffer_cache_mutex());
    buffer_cache().remove(buffer_key(*node));
  }

  std::string errors = compile_all_passes(storage);
  if (errors.empty()) {
    set_owned_str(&storage.error_log, nullptr);
    BKE_reportf(op->reports,
                RPT_INFO,
                "Compiled (%d parameter%s)",
                int(storage.param_count),
                storage.param_count == 1 ? "" : "s");
  }
  else {
    set_owned_str(&storage.error_log, errors.c_str());
    BKE_report(op->reports, RPT_ERROR, errors.c_str());
  }

  BKE_ntree_update_tag_node_property(snode->edittree, node);
  WM_event_add_notifier(C, NC_NODE | NA_EDITED, snode->edittree);
  ED_area_tag_refresh(CTX_wm_area(C));
  return errors.empty() ? OPERATOR_FINISHED : OPERATOR_FINISHED;
}

static void NODE_OT_shadertoy_compile(wmOperatorType *ot)
{
  ot->name = "Compile ShaderToy";
  ot->idname = "NODE_OT_shadertoy_compile";
  ot->description =
      "Scan #define parameters into the sidebar and compile Common / Buffer / Image GLSL";
  ot->exec = shadertoy_compile_exec;
  ot->poll = shadertoy_node_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static void node_operators()
{
  WM_operatortype_append(NODE_OT_shadertoy_reset_buffers);
  WM_operatortype_append(NODE_OT_shadertoy_compile);
}

/** \} */

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeShaderToy"_ustr, IMG_NODE_SHADERTOY);
  ntype.ui_name = "ShaderToy";
  ntype.ui_description =
      "Paste ShaderToy GLSL (Image / Common / Buffer A–D) in the sidebar, Compile, output Color";
  ntype.enum_name_legacy = "SHADERTOY";
  ntype.nclass = NODE_CLASS_TEXTURE;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.draw_buttons_ex = node_layout_ex;
  ntype.labelfunc = node_label;
  ntype.get_compositor_operation = get_compositor_operation;
  ntype.register_operators = node_operators;
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  ntype.default_width = bke::NodeWidth::_240;
  bke::node_type_storage(ntype, "NodeImageShaderToy", node_free_storage, node_copy_storage);

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_shadertoy_cc
