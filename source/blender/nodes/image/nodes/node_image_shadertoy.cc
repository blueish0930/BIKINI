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

static bool skip_ident_name(StringRef &s, std::string &name)
{
  s = ltrim(s);
  if (s.is_empty() || !is_ident_start(s[0])) {
    return false;
  }
  int64_t n = 1;
  while (n < s.size() && is_ident_char(s[n])) {
    n++;
  }
  name.assign(s.data(), size_t(n));
  s = s.substr(n);
  return true;
}

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

static bool parse_bool_token(StringRef &s, float &out)
{
  s = ltrim(s);
  if (s.startswith("true") || s.startswith("TRUE")) {
    out = 1.0f;
    s = s.substr(4);
    return true;
  }
  if (s.startswith("false") || s.startswith("FALSE")) {
    out = 0.0f;
    s = s.substr(5);
    return true;
  }
  return parse_number_token(s, out);
}

static bool parse_vec_literal(StringRef &s, const int n, float out[4])
{
  s = ltrim(s);
  const char *tag = (n == 2) ? "vec2" : (n == 3) ? "vec3" : "vec4";
  if (!s.startswith(tag)) {
    return false;
  }
  s = ltrim(s.substr(int64_t(strlen(tag))));
  if (s.is_empty() || s[0] != '(') {
    return false;
  }
  s = s.substr(1);
  for (int i = 0; i < n; i++) {
    if (i > 0) {
      s = ltrim(s);
      if (s.is_empty() || s[0] != ',') {
        return false;
      }
      s = s.substr(1);
    }
    if (!parse_number_token(s, out[i])) {
      return false;
    }
  }
  s = ltrim(s);
  if (s.is_empty() || s[0] != ')') {
    return false;
  }
  s = s.substr(1);
  return true;
}

static bool consume_div_255(StringRef &s)
{
  StringRef t = ltrim(s);
  if (t.is_empty() || t[0] != '/') {
    return false;
  }
  t = ltrim(t.substr(1));
  float v = 0.0f;
  StringRef n = t;
  if (!parse_number_token(n, v)) {
    return false;
  }
  if (std::fabs(v - 255.0f) > 0.01f) {
    return false;
  }
  s = n;
  return true;
}

static bool looks_like_srgb_bytes(const float rgb[3])
{
  float mx = 0.0f;
  for (int i = 0; i < 3; i++) {
    if (rgb[i] < -0.01f || rgb[i] > 255.01f) {
      return false;
    }
    const float nearest = std::round(rgb[i]);
    if (std::fabs(rgb[i] - nearest) > 0.01f) {
      return false;
    }
    mx = math::max(mx, rgb[i]);
  }
  /* 0/1/2 as a basis vector is not a palette. Palette bytes always have a
   * channel well above 1 (dtSfzR skyColor vec3(101,164,208)/255.). */
  return mx >= 8.0f;
}

static std::string glsl_number(const float v);

/* dtSfzR / many ShaderToy palettes: `#define skyColor vec3(101, 164, 208)/255.`.
 * Dropping `/255` makes far mountains and clouds HDR-white against the sky. */
static std::string fold_srgb_byte_vec3_defines(StringRef src)
{
  std::string out;
  out.reserve(size_t(src.size()) + 32);
  int64_t i = 0;
  while (i < src.size()) {
    const int64_t line_end = src.find('\n', i);
    const int64_t end = (line_end == StringRef::not_found) ? src.size() : line_end;
    StringRef line = src.substr(i, end - i);
    StringRef t = ltrim(line);
    bool replaced = false;
    if (t.startswith("#")) {
      StringRef rest = ltrim(t.substr(1));
      if (rest.startswith("define")) {
        rest = ltrim(rest.substr(6));
        std::string name;
        if (skip_ident_name(rest, name)) {
          float rgb[4] = {0.0f, 0.0f, 0.0f, 0.0f};
          StringRef lit = ltrim(rest);
          if (parse_vec_literal(lit, 3, rgb) && looks_like_srgb_bytes(rgb)) {
            StringRef after = ltrim(lit);
            const bool already = consume_div_255(after);
            after = ltrim(after);
            const bool rest_ok = after.is_empty() || after.startswith("//") ||
                                 after.startswith("/*");
            if (rest_ok && !already) {
              const int64_t lead = line.size() - ltrim(line).size();
              out.append(line.data(), size_t(lead));
              out += "#define ";
              out += name;
              out += " vec3(";
              out += glsl_number(rgb[0] / 255.0f);
              out += ", ";
              out += glsl_number(rgb[1] / 255.0f);
              out += ", ";
              out += glsl_number(rgb[2] / 255.0f);
              out += ")";
              replaced = true;
            }
          }
        }
      }
    }
    if (!replaced) {
      out.append(line.data(), size_t(line.size()));
    }
    if (end < src.size()) {
      out.push_back('\n');
    }
    i = (end < src.size()) ? end + 1 : src.size();
  }
  return out;
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

static bool add_param(Vector<ShaderParam> &params, Set<std::string> &seen, ShaderParam param)
{
  if (param.name.empty() || is_skipped_param_name(param.name)) {
    return false;
  }
  if (params.size() >= MAX_SHADER_PARAMS) {
    return false;
  }
  if (!seen.add(param.name)) {
    return false;
  }
  if (param.kind == ShaderParamKind::Vec3 && param.values[0] >= 0.0f && param.values[0] <= 1.0f &&
      param.values[1] >= 0.0f && param.values[1] <= 1.0f && param.values[2] >= 0.0f &&
      param.values[2] <= 1.0f)
  {
    param.kind = ShaderParamKind::Color;
  }
  if (!param.has_range) {
    const float v = param.values[0];
    if (param.kind == ShaderParamKind::Bool) {
      param.min_v = 0.0f;
      param.max_v = 1.0f;
      param.has_range = true;
    }
    else if (param.kind == ShaderParamKind::Int) {
      param.min_v = 0.0f;
      param.max_v = math::max(float(int(std::lround(v)) * 4), 16.0f);
      param.has_range = true;
    }
    else if (v >= 0.0f && v <= 1.0f) {
      param.min_v = 0.0f;
      param.max_v = 1.0f;
      param.has_range = true;
    }
    else {
      const float mag = math::max(math::abs(v), 1.0f);
      param.min_v = v - mag * 4.0f;
      param.max_v = v + mag * 4.0f;
      param.has_range = true;
    }
  }
  params.append(std::move(param));
  return true;
}

static void parse_params_from_source(const char *src, Vector<ShaderParam> &params, Set<std::string> &seen)
{
  if (src == nullptr || src[0] == '\0') {
    return;
  }
  StringRef text(src);
  int64_t i = 0;
  bool in_block = false;
  while (i < text.size()) {
    const int64_t line_end = text.find('\n', i);
    const int64_t end = (line_end == StringRef::not_found) ? text.size() : line_end;
    StringRef line = text.substr(i, end - i);
    i = (end < text.size()) ? end + 1 : text.size();

    /* Crude block-comment skip. */
    if (in_block) {
      const int64_t close = line.find("*/");
      if (close == StringRef::not_found) {
        continue;
      }
      in_block = false;
      line = line.substr(close + 2);
    }
    const int64_t block_open = line.find("/*");
    if (block_open != StringRef::not_found && line.find("*/") == StringRef::not_found) {
      in_block = true;
      line = line.substr(0, block_open);
    }

    /* Only file-scope. Indented `const float tmax = 2000` inside mainImage must not
     * become `#define tmax 2000` or function args named tmax get macro-replaced
     * (Rainforest renderClouds → unexpected INTCONSTANT). */
    if (!line.is_empty() && (line[0] == ' ' || line[0] == '\t')) {
      continue;
    }

    StringRef t = ltrim(line);
    if (t.startswith("//")) {
      continue;
    }

    ShaderParam param;

    if (t.startswith("#")) {
      t = ltrim(t.substr(1));
      const bool is_define = t.startswith("define");
      const bool is_iuniform = t.startswith("iUniform") || t.startswith("iuniform");
      if (!is_define && !is_iuniform) {
        continue;
      }
      t = ltrim(t.substr(is_define ? 6 : 8));
      if (is_iuniform) {
        std::string type;
        if (!skip_ident_name(t, type)) {
          continue;
        }
        if (!skip_ident_name(t, param.name)) {
          continue;
        }
        t = ltrim(t);
        if (!t.is_empty() && t[0] == '=') {
          t = t.substr(1);
        }
        if (type == "bool") {
          param.kind = ShaderParamKind::Bool;
          if (!parse_bool_token(t, param.values[0])) {
            continue;
          }
        }
        else if (type == "int") {
          param.kind = ShaderParamKind::Int;
          if (!parse_number_token(t, param.values[0])) {
            continue;
          }
        }
        else if (type == "vec2" || type == "float2") {
          param.kind = ShaderParamKind::Vec2;
          if (!parse_vec_literal(t, 2, param.values)) {
            continue;
          }
        }
        else if (type == "vec3" || type == "float3") {
          param.kind = ShaderParamKind::Vec3;
          if (!parse_vec_literal(t, 3, param.values)) {
            continue;
          }
        }
        else if (type == "color3") {
          param.kind = ShaderParamKind::Color;
          if (!parse_vec_literal(t, 3, param.values)) {
            continue;
          }
          param.values[3] = 1.0f;
        }
        else if (type == "float") {
          param.kind = ShaderParamKind::Float;
          if (!parse_number_token(t, param.values[0])) {
            continue;
          }
        }
        else {
          continue;
        }
        parse_range_from_rest(t, param);
        add_param(params, seen, std::move(param));
        continue;
      }

      /* #define NAME value */
      if (!skip_ident_name(t, param.name)) {
        continue;
      }
      t = ltrim(t);
      if (!t.is_empty() && t[0] == '(') {
        continue; /* Function-like macro. */
      }
      if (parse_vec_literal(t, 3, param.values)) {
        param.kind = ShaderParamKind::Vec3;
        if (consume_div_255(t) || looks_like_srgb_bytes(param.values)) {
          param.values[0] /= 255.0f;
          param.values[1] /= 255.0f;
          param.values[2] /= 255.0f;
        }
      }
      else if (parse_vec_literal(t, 2, param.values)) {
        param.kind = ShaderParamKind::Vec2;
      }
      else if (parse_number_token(t, param.values[0])) {
        const StringRef rest = ltrim(t);
        const bool rest_ok = rest.is_empty() || rest.startswith("//") || rest.startswith("/*") ||
                             rest.startswith("{") || rest.startswith("in ");
        if (!rest_ok) {
          /* Expression macro such as `(0.1 / iResolution.x)` — do not rewrite. */
          continue;
        }
        /* Keep 1024-class sentinels (EMPTY) as the authored token, not Int 0–max. */
        const bool looks_int = std::floor(param.values[0]) == param.values[0] &&
                               std::fabs(param.values[0]) < 256.0f;
        param.kind = looks_int ? ShaderParamKind::Int : ShaderParamKind::Float;
      }
      else {
        continue;
      }
      parse_range_from_rest(line, param);
      add_param(params, seen, std::move(param));
      continue;
    }

    /* File-scope `const float NAME = 1.5;` */
    if (t.startswith("const ")) {
      t = ltrim(t.substr(6));
      std::string type;
      if (!skip_ident_name(t, type)) {
        continue;
      }
      if (!skip_ident_name(t, param.name)) {
        continue;
      }
      t = ltrim(t);
      if (t.is_empty() || t[0] != '=') {
        continue;
      }
      t = t.substr(1);
      if (type == "float") {
        param.kind = ShaderParamKind::Float;
        if (!parse_number_token(t, param.values[0])) {
          continue;
        }
      }
      else if (type == "int") {
        param.kind = ShaderParamKind::Int;
        if (!parse_number_token(t, param.values[0])) {
          continue;
        }
      }
      else if (type == "bool") {
        param.kind = ShaderParamKind::Bool;
        if (!parse_bool_token(t, param.values[0])) {
          continue;
        }
      }
      else if (type == "vec2") {
        param.kind = ShaderParamKind::Vec2;
        if (!parse_vec_literal(t, 2, param.values)) {
          continue;
        }
      }
      else if (type == "vec3") {
        param.kind = ShaderParamKind::Vec3;
        if (!parse_vec_literal(t, 3, param.values)) {
          continue;
        }
        if (consume_div_255(t) || looks_like_srgb_bytes(param.values)) {
          param.values[0] /= 255.0f;
          param.values[1] /= 255.0f;
          param.values[2] /= 255.0f;
        }
        if (param.values[0] >= 0.0f && param.values[0] <= 1.0f &&
            param.values[1] >= 0.0f && param.values[1] <= 1.0f &&
            param.values[2] >= 0.0f && param.values[2] <= 1.0f)
        {
          param.kind = ShaderParamKind::Color;
        }
      }
      else {
        continue;
      }
      parse_range_from_rest(line, param);
      add_param(params, seen, std::move(param));
    }
  }
}

static Vector<ShaderParam> parse_shader_params(const NodeImageShaderToy &storage)
{
  Vector<ShaderParam> params;
  Set<std::string> seen;
  parse_params_from_source(storage.code_common, params, seen);
  parse_params_from_source(storage.code_image, params, seen);
  parse_params_from_source(storage.code_buffer_a, params, seen);
  parse_params_from_source(storage.code_buffer_b, params, seen);
  parse_params_from_source(storage.code_buffer_c, params, seen);
  parse_params_from_source(storage.code_buffer_d, params, seen);
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
      const float mn = as_color ? 0.0f : param.min_v;
      const float mx = as_color ? 1.0f : param.max_v;
      for (int c = 0; c < 3; c++) {
        push_scalar(param.name + suf[c], ShaderParamKind::Float, param.values[c], mn, mx);
      }
      continue;
    }
    if (!param_is_scalar(param.kind)) {
      continue;
    }
    push_scalar(param.name, param.kind, param.values[0], param.min_v, param.max_v);
  }
}

static bool line_defines_param(StringRef trimmed, const StringRef name)
{
  auto after_keyword = [&](StringRef t) {
    t = ltrim(t);
    /* Optional type for iUniform / const. */
    if (t.startswith(name) && (t.size() == name.size() || !is_ident_char(t[name.size()]))) {
      return true;
    }
    std::string type;
    if (!skip_ident_name(t, type)) {
      return false;
    }
    t = ltrim(t);
    return t.startswith(name) && (t.size() == name.size() || !is_ident_char(t[name.size()]));
  };

  if (trimmed.startswith("#")) {
    StringRef t = ltrim(trimmed.substr(1));
    if (t.startswith("define")) {
      t = ltrim(t.substr(6));
      return t.startswith(name) && (t.size() == name.size() || !is_ident_char(t[name.size()]));
    }
    if (t.startswith("iUniform") || t.startswith("iuniform")) {
      t = ltrim(t.substr(8));
      return after_keyword(t);
    }
  }
  if (trimmed.startswith("const ")) {
    return after_keyword(ltrim(trimmed.substr(6)));
  }
  return false;
}

static std::string glsl_number(const float v)
{
  char buf[64];
  SNPRINTF(buf, "%.8g", double(v));
  return buf;
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

static std::string rewrite_param_line(const ShaderParam &param, const bool as_define)
{
  std::string out;
  if (as_define) {
    out += "#define ";
    out += param.name;
    out += " ";
  }
  else {
    out += "const ";
    out += (param.kind == ShaderParamKind::Int || param.kind == ShaderParamKind::Bool) ? "int " :
                                                                                         "float ";
    out += param.name;
    out += " = ";
  }
  switch (param.kind) {
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
    default:
      out += glsl_number(param.values[0]);
      break;
  }
  if (!as_define) {
    out += ";";
  }
  return out;
}

static bool extract_defined_name(StringRef trimmed, std::string &name)
{
  if (trimmed.startswith("#")) {
    StringRef t = ltrim(trimmed.substr(1));
    if (!t.startswith("define")) {
      return false;
    }
    t = ltrim(t.substr(6));
    return skip_ident_name(t, name);
  }
  if (trimmed.startswith("const ")) {
    StringRef t = ltrim(trimmed.substr(6));
    std::string type;
    if (!skip_ident_name(t, type)) {
      return false;
    }
    t = ltrim(t);
    return skip_ident_name(t, name);
  }
  return false;
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

static std::string apply_param_values(StringRef src, Span<ShaderParam> params)
{
  if (params.is_empty()) {
    return src;
  }
  std::string out;
  out.reserve(size_t(src.size()) + 64);
  int64_t i = 0;
  while (i < src.size()) {
    const int64_t line_end = src.find('\n', i);
    const int64_t end = (line_end == StringRef::not_found) ? src.size() : line_end;
    StringRef line = src.substr(i, end - i);
    const StringRef trimmed = ltrim(line);
    bool replaced = false;
    /* Only rewrite un-indented definitions — never a function signature. */
    if (!line.is_empty() && line[0] != ' ' && line[0] != '\t') {
      std::string defined;
      if (extract_defined_name(trimmed, defined)) {
        float rgb[3];
        if (gather_vec3_param(params, defined, rgb)) {
          if (trimmed.startswith("#")) {
            out += "#define ";
            out += defined;
            out += " vec3(";
          }
          else {
            out += "const vec3 ";
            out += defined;
            out += " = vec3(";
          }
          out += glsl_number(rgb[0]);
          out += ", ";
          out += glsl_number(rgb[1]);
          out += ", ";
          out += glsl_number(rgb[2]);
          out += ")";
          if (!trimmed.startswith("#")) {
            out += ";";
          }
          if (end < src.size()) {
            out.push_back('\n');
          }
          replaced = true;
        }
      }
      if (!replaced) {
        for (const ShaderParam &param : params) {
          if (param.name.find('.') != std::string::npos) {
            continue;
          }
          if (!line_defines_param(trimmed, param.name)) {
            continue;
          }
          out += rewrite_param_line(param, trimmed.startswith("#"));
          if (end < src.size()) {
            out.push_back('\n');
          }
          replaced = true;
          break;
        }
      }
    }
    if (!replaced) {
      out.append(line.data(), size_t(line.size()));
      if (end < src.size()) {
        out.push_back('\n');
      }
    }
    i = (end < src.size()) ? end + 1 : src.size();
  }
  return out;
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
 * \{ */

static bool is_ident_char_st(const char c)
{
  return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
}

static bool line_is_shadertoy_builtin_decl(StringRef trimmed)
{
  if (trimmed.startswith("#version") || trimmed.startswith("precision ") ||
      trimmed.startswith("#extension"))
  {
    return true;
  }
  auto has_word = [&](const char *word) -> bool {
    const int64_t p = trimmed.find(word);
    if (p == StringRef::not_found) {
      return false;
    }
    const char before = (p > 0) ? trimmed[p - 1] : ' ';
    const int64_t after_i = p + int64_t(strlen(word));
    const char after = (after_i < trimmed.size()) ? trimmed[after_i] : ' ';
    return !is_ident_char_st(before) && !is_ident_char_st(after);
  };
  const bool builtin = has_word("iResolution") || has_word("iTime") || has_word("iTimeDelta") ||
                       has_word("iFrame") || has_word("iFrameRate") || has_word("iMouse") ||
                       has_word("iDate") || has_word("iSampleRate") || has_word("iChannel0") ||
                       has_word("iChannel1") || has_word("iChannel2") || has_word("iChannel3") ||
                       has_word("iChannelResolution") || has_word("iChannelTime") ||
                       has_word("iGlobalTime");
  if (trimmed.startswith("uniform") && builtin) {
    return true;
  }
  if (trimmed.startswith("layout") && (has_word("fragColor") || has_word("fragCoord"))) {
    return true;
  }
  /* Converted shaders often declare these at file scope. Do not strip mainImage's `out vec4`. */
  if ((trimmed.startswith("out ") || trimmed.startswith("in ")) && trimmed.find('(') == StringRef::not_found)
  {
    if (has_word("fragColor") || has_word("fragCoord") || has_word("gl_FragColor")) {
      return true;
    }
  }
  return false;
}

static std::string rename_user_main(StringRef src)
{
  std::string out;
  out.reserve(size_t(src.size()) + 32);
  int64_t i = 0;
  while (i < src.size()) {
    const int64_t p = src.find("main", i);
    if (p == StringRef::not_found) {
      out.append(src.data() + i, size_t(src.size() - i));
      break;
    }
    out.append(src.data() + i, size_t(p - i));
    const char before = (p > 0) ? src[p - 1] : ' ';
    const char after = (p + 4 < src.size()) ? src[p + 4] : ' ';
    if (is_ident_char_st(before) || is_ident_char_st(after)) {
      out.append("main", 4);
      i = p + 4;
      continue;
    }
    int64_t q = p + 4;
    while (q < src.size() && std::isspace(static_cast<unsigned char>(src[q]))) {
      q++;
    }
    if (q < src.size() && src[q] == '(') {
      out.append("_st_unused_main");
      i = p + 4;
      continue;
    }
    out.append("main", 4);
    i = p + 4;
  }
  return out;
}

/* WebGL/ShaderToy allows identifiers that desktop GLSL reserves.
 * Dsf3WH: `float drawFont(vec2 p, int char)` — `char` is a keyword. */
static std::string rewrite_reserved_idents(StringRef src)
{
  static const char *words[] = {
      "char",     "class",     "union",     "template", "this",     "packed",
      "input",    "output",    "filter",    "partition","public",   "private",
      "short",    "long",      "half",      "fixed",    "unsigned", "byte",
      "goto",     "inline",    "typename",  "using",    "namespace","volatile",
      "abstract", "active",    "asm",       "cast",     "extern",   "interface",
      /* csc3RS: `SampleDepth(sampler2D sampler, ...)` — `sampler` is a keyword. */
      "sampler",  "sample",    "buffer",    "shared",   "subroutine","patch",
      "coherent", "restrict",  "readonly",  "writeonly","precise",  "invariant",
  };
  std::string s(src.data(), size_t(src.size()));
  for (const char *word : words) {
    const std::string from(word);
    const std::string to = std::string("_st_") + word;
    size_t pos = 0;
    while ((pos = s.find(from, pos)) != std::string::npos) {
      const char before = (pos > 0) ? s[pos - 1] : ' ';
      const char after = (pos + from.size() < s.size()) ? s[pos + from.size()] : ' ';
      if (is_ident_char_st(before) || is_ident_char_st(after)) {
        pos += from.size();
        continue;
      }
      s.replace(pos, from.size(), to);
      pos += to.size();
    }
  }
  return s;
}

/* Blender's GLSL preprocessor scans for `/*` before `//`. A line comment like
 * `//*.7+.2` (Shane Fractal Flythrough) contains the two-char `/*` start, is
 * treated as an unclosed block comment, and throws ParserException (闪退). */
static std::string sanitize_line_comment_block_starts(StringRef src)
{
  std::string s(src.data(), size_t(src.size()));
  size_t pos = 0;
  while ((pos = s.find("//*", pos)) != std::string::npos) {
    s.replace(pos, 3, "// *");
    pos += 4;
  }
  return s;
}

static std::string strip_shadertoy_preamble(StringRef src)
{
  const std::string sanitized = sanitize_line_comment_block_starts(src);
  src = sanitized;
  std::string out;
  out.reserve(size_t(src.size()));
  int64_t i = 0;
  while (i < src.size()) {
    const int64_t line_end = src.find('\n', i);
    const int64_t end = (line_end == StringRef::not_found) ? src.size() : line_end;
    StringRef line = src.substr(i, end - i);
    StringRef trimmed = line.trim();
    /* IQ's `ZERO = min(iFrame,0)` stops WebGL from unrolling. Replacing it with the
     * literal `0` lets desktop compilers unroll huge raymarch loops and TDR/crash
     * (Shane 4s3SRN). A function call is a valid loop start and blocks unroll. */
    if (trimmed.startswith("#define ZERO") && trimmed.find("iFrame") != StringRef::not_found) {
      out += "#define ZERO _st_loop_zero()";
      if (end < src.size()) {
        out.push_back('\n');
      }
    }
    else if (!line_is_shadertoy_builtin_decl(trimmed)) {
      out.append(line.data(), size_t(line.size()));
      if (end < src.size()) {
        out.push_back('\n');
      }
    }
    i = (end < src.size()) ? end + 1 : src.size();
  }
  return rewrite_reserved_idents(rename_user_main(out));
}

static int64_t matching_paren(StringRef s, const int64_t open)
{
  if (open < 0 || open >= s.size() || s[open] != '(') {
    return StringRef::not_found;
  }
  int depth = 1;
  for (int64_t i = open + 1; i < s.size(); i++) {
    if (s[i] == '(') {
      depth++;
    }
    else if (s[i] == ')') {
      depth--;
      if (depth == 0) {
        return i;
      }
    }
  }
  return StringRef::not_found;
}

/* Vulkan `textureLod(..., 0.0)` on a 1-mip ping-pong buffer is often black / undefined.
 * Chimera's Breath (4tGfDW) samples all 4 neighbors with textureLod — if those return 0,
 * pressure projection dies and the two emitters stay split with a noisy middle.
 * Do NOT rewrite 2-arg `texture()` into `textureLod` globally: fljBWc samples a
 * volume with `texture(iChannel0, p/amplitude)` (vec3). Forcing lod 0 then
 * mismatches sampler2D and fails to compile. XsffWj world-space mips are handled
 * by binding 2D dummies without MIPMAP filtering. */
static std::string rewrite_texturelod_zero(StringRef src)
{
  std::string out;
  out.reserve(size_t(src.size()));
  const StringRef key("textureLod");
  int64_t i = 0;
  while (i < src.size()) {
    const int64_t hit = src.find(key, i);
    if (hit == StringRef::not_found) {
      out.append(src.data() + i, size_t(src.size() - i));
      break;
    }
    const char before = (hit > 0) ? src[hit - 1] : ' ';
    const int64_t after_i = hit + key.size();
    const char after = (after_i < src.size()) ? src[after_i] : ' ';
    if (is_ident_char_st(before) || is_ident_char_st(after)) {
      out.append(src.data() + i, size_t(hit + 1 - i));
      i = hit + 1;
      continue;
    }
    int64_t q = after_i;
    while (q < src.size() && std::isspace(static_cast<unsigned char>(src[q]))) {
      q++;
    }
    if (q >= src.size() || src[q] != '(') {
      out.append(src.data() + i, size_t(q - i));
      i = q;
      continue;
    }
    const int64_t close = matching_paren(src, q);
    if (close == StringRef::not_found) {
      out.append(src.data() + i, size_t(src.size() - i));
      break;
    }
    StringRef inside = src.substr(q + 1, close - (q + 1));
    int depth = 0;
    int64_t c0 = StringRef::not_found;
    int64_t c1 = StringRef::not_found;
    for (int64_t k = 0; k < inside.size(); k++) {
      const char ch = inside[k];
      if (ch == '(') {
        depth++;
      }
      else if (ch == ')') {
        depth--;
      }
      else if (ch == ',' && depth == 0) {
        if (c0 == StringRef::not_found) {
          c0 = k;
        }
        else if (c1 == StringRef::not_found) {
          c1 = k;
          break;
        }
      }
    }
    bool replaced = false;
    if (c0 != StringRef::not_found && c1 != StringRef::not_found) {
      StringRef lod = rtrim(ltrim(inside.substr(c1 + 1)));
      if (lod == "0" || lod == "0." || lod == "0.0" || lod == "0.00" || lod == "0.000") {
        StringRef a0 = rtrim(ltrim(inside.substr(0, c0)));
        StringRef a1 = rtrim(ltrim(inside.substr(c0 + 1, c1 - (c0 + 1))));
        out.append(src.data() + i, size_t(hit - i));
        out += "texture(";
        out.append(a0.data(), size_t(a0.size()));
        out += ", ";
        out.append(a1.data(), size_t(a1.size()));
        out += ")";
        replaced = true;
      }
    }
    if (!replaced) {
      out.append(src.data() + i, size_t(close + 1 - i));
    }
    i = close + 1;
  }
  return out;
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

static bool is_zero_literal_st(StringRef s)
{
  s = rtrim(ltrim(s));
  if (s.is_empty()) {
    return false;
  }
  std::string tmp(s.data(), size_t(s.size()));
  char *end = nullptr;
  const double v = std::strtod(tmp.c_str(), &end);
  return end != tmp.c_str() && *end == '\0' && v == 0.0;
}

/* `for (int i = min(iFrame, 0); i < N; i++)` is IQ/Shane's anti-unroll trick.
 * Leaving it as-is crashes some desktop compilers; folding to a literal `0`
 * unrolls 100+ iteration fractals and TDR-kills the process (4s3SRN). */
static std::string rewrite_iframe_zero_calls(StringRef src)
{
  std::string out;
  out.reserve(size_t(src.size()));
  const StringRef key("min");
  int64_t i = 0;
  while (i < src.size()) {
    const int64_t hit = src.find(key, i);
    if (hit == StringRef::not_found) {
      out.append(src.data() + i, size_t(src.size() - i));
      break;
    }
    const char before = (hit > 0) ? src[hit - 1] : ' ';
    const char after = (hit + key.size() < src.size()) ? src[hit + key.size()] : ' ';
    if (is_ident_char_st(before) || is_ident_char_st(after)) {
      out.append(src.data() + i, size_t(hit + key.size() - i));
      i = hit + key.size();
      continue;
    }
    int64_t q = hit + key.size();
    while (q < src.size() && std::isspace(static_cast<unsigned char>(src[q]))) {
      q++;
    }
    if (q >= src.size() || src[q] != '(') {
      out.append(src.data() + i, size_t(q - i));
      i = q;
      continue;
    }
    const int64_t close = matching_paren(src, q);
    if (close == StringRef::not_found) {
      out.append(src.data() + i, size_t(src.size() - i));
      break;
    }
    StringRef inside = src.substr(q + 1, close - (q + 1));
    int depth = 0;
    int64_t comma = StringRef::not_found;
    for (int64_t k = 0; k < inside.size(); k++) {
      const char ch = inside[k];
      if (ch == '(') {
        depth++;
      }
      else if (ch == ')') {
        depth--;
      }
      else if (ch == ',' && depth == 0) {
        comma = k;
        break;
      }
    }
    bool replaced = false;
    if (comma != StringRef::not_found) {
      StringRef a0 = rtrim(ltrim(inside.substr(0, comma)));
      StringRef a1 = rtrim(ltrim(inside.substr(comma + 1)));
      const bool iframe_zero = (a0 == "iFrame" && is_zero_literal_st(a1)) ||
                               (a1 == "iFrame" && is_zero_literal_st(a0));
      if (iframe_zero) {
        out.append(src.data() + i, size_t(hit - i));
        out += "_st_loop_zero()";
        replaced = true;
      }
    }
    if (!replaced) {
      out.append(src.data() + i, size_t(close + 1 - i));
    }
    i = close + 1;
  }
  return out;
}

/* Desktop GLSL/SPIR-V will unroll `for (int i = 0; i < 80; i++)` when 80 is a
 * #define constant. Nested raymarch + shadow (Saturday cubism, Shane, IQ)
 * then either hangs shaderc for minutes or TDR-kills the GPU. ShaderToy/WebGL
 * keeps these as loops; IQ's `ZERO = min(iFrame,0)` trick is the same idea. */
static std::string rewrite_loop_zero_starts(StringRef src)
{
  std::string out;
  out.reserve(size_t(src.size()) + 128);
  auto skip_ws = [&](int64_t p) {
    while (p < src.size() && std::isspace(static_cast<unsigned char>(src[p]))) {
      p++;
    }
    return p;
  };
  auto starts_type = [&](int64_t p, const char *type) -> int64_t {
    const int64_t n = int64_t(strlen(type));
    if (p + n > src.size()) {
      return -1;
    }
    if (std::memcmp(src.data() + p, type, size_t(n)) != 0) {
      return -1;
    }
    if (p + n < src.size() && is_ident_char_st(src[p + n])) {
      return -1;
    }
    return p + n;
  };
  int64_t i = 0;
  while (i < src.size()) {
    const int64_t hit = src.find("for", i);
    if (hit == StringRef::not_found) {
      out.append(src.data() + i, size_t(src.size() - i));
      break;
    }
    const char before = (hit > 0) ? src[hit - 1] : ' ';
    const char after = (hit + 3 < src.size()) ? src[hit + 3] : ' ';
    if (is_ident_char_st(before) || is_ident_char_st(after)) {
      out.append(src.data() + i, size_t(hit + 3 - i));
      i = hit + 3;
      continue;
    }
    int64_t p = skip_ws(hit + 3);
    if (p >= src.size() || src[p] != '(') {
      out.append(src.data() + i, size_t(p - i));
      i = p;
      continue;
    }
    p = skip_ws(p + 1);
    int64_t t = starts_type(p, "int");
    if (t < 0) {
      t = starts_type(p, "uint");
    }
    if (t < 0) {
      out.append(src.data() + i, size_t(hit + 3 - i));
      i = hit + 3;
      continue;
    }
    p = skip_ws(t);
    if (p >= src.size() || !(std::isalpha(static_cast<unsigned char>(src[p])) || src[p] == '_')) {
      out.append(src.data() + i, size_t(hit + 3 - i));
      i = hit + 3;
      continue;
    }
    while (p < src.size() && is_ident_char_st(src[p])) {
      p++;
    }
    p = skip_ws(p);
    if (p >= src.size() || src[p] != '=') {
      out.append(src.data() + i, size_t(hit + 3 - i));
      i = hit + 3;
      continue;
    }
    p = skip_ws(p + 1);
    if (p >= src.size() || src[p] != '0') {
      out.append(src.data() + i, size_t(hit + 3 - i));
      i = hit + 3;
      continue;
    }
    const int64_t after0 = p + 1;
    if (after0 < src.size() &&
        (std::isdigit(static_cast<unsigned char>(src[after0])) || src[after0] == '.' ||
         src[after0] == 'x' || src[after0] == 'X' || is_ident_char_st(src[after0])))
    {
      out.append(src.data() + i, size_t(hit + 3 - i));
      i = hit + 3;
      continue;
    }
    const int64_t semi = skip_ws(after0);
    if (semi >= src.size() || src[semi] != ';') {
      out.append(src.data() + i, size_t(hit + 3 - i));
      i = hit + 3;
      continue;
    }
    out.append(src.data() + i, size_t(p - i));
    out += "_st_loop_zero()";
    i = after0;
  }
  return out;
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

/* IQ / Dave Hoskins 3D noise: `texture(iChannel0, (uv+0.5)/256.0, -99.0)`.
 * `uv` contains `floor(x)`, so implicit LOD spikes at every lattice face and
 * samples blurry mips — visible noise-grid seams (Remnant X 4sjSW1).
 * `textureLod(..., 0)` is still trilinear/aniso on some Vulkan drivers.
 * Manual wrap-REPEAT texelFetch bilinear is lod-proof. Do this AFTER
 * rewriting textureLod(...,0) → texture(), or that pass would undo us. */
static std::string rewrite_texture_neg_bias(StringRef src)
{
  std::string out;
  out.reserve(size_t(src.size()) + 64);
  const char *funcs[] = {"texture2D", "texture"};
  int64_t i = 0;
  while (i < src.size()) {
    int64_t best = StringRef::not_found;
    const char *best_fn = nullptr;
    for (const char *fn : funcs) {
      const StringRef key(fn);
      const int64_t hit = src.find(key, i);
      if (hit == StringRef::not_found) {
        continue;
      }
      const char before = (hit > 0) ? src[hit - 1] : ' ';
      const int64_t after_i = hit + key.size();
      const char after = (after_i < src.size()) ? src[after_i] : ' ';
      if (is_ident_char_st(before) || is_ident_char_st(after)) {
        continue;
      }
      if (best == StringRef::not_found || hit < best) {
        best = hit;
        best_fn = fn;
      }
    }
    if (best == StringRef::not_found) {
      out.append(src.data() + i, size_t(src.size() - i));
      break;
    }
    const StringRef key(best_fn);
    int64_t q = best + key.size();
    while (q < src.size() && std::isspace(static_cast<unsigned char>(src[q]))) {
      q++;
    }
    if (q >= src.size() || src[q] != '(') {
      out.append(src.data() + i, size_t(q - i));
      i = q;
      continue;
    }
    const int64_t close = matching_paren(src, q);
    if (close == StringRef::not_found) {
      out.append(src.data() + i, size_t(src.size() - i));
      break;
    }
    StringRef inside = src.substr(q + 1, close - (q + 1));
    int depth = 0;
    int64_t c0 = StringRef::not_found;
    int64_t c1 = StringRef::not_found;
    for (int64_t k = 0; k < inside.size(); k++) {
      const char ch = inside[k];
      if (ch == '(') {
        depth++;
      }
      else if (ch == ')') {
        depth--;
      }
      else if (ch == ',' && depth == 0) {
        if (c0 == StringRef::not_found) {
          c0 = k;
        }
        else if (c1 == StringRef::not_found) {
          c1 = k;
          break;
        }
      }
    }
    bool replaced = false;
    if (c0 != StringRef::not_found) {
      StringRef a0 = rtrim(ltrim(inside.substr(0, c0)));
      StringRef a1;
      bool use_fetch = false;
      if (c1 != StringRef::not_found) {
        a1 = rtrim(ltrim(inside.substr(c0 + 1, c1 - (c0 + 1))));
        use_fetch = is_negative_mip_bias(inside.substr(c1 + 1)) || looks_like_iq_noise_uv(a1);
      }
      else {
        a1 = rtrim(ltrim(inside.substr(c0 + 1)));
        use_fetch = looks_like_iq_noise_uv(a1);
      }
      if (use_fetch) {
        out.append(src.data() + i, size_t(best - i));
        out += "_st_tex2d_lod0(";
        out.append(a0.data(), size_t(a0.size()));
        out += ", ";
        out.append(a1.data(), size_t(a1.size()));
        out += ")";
        replaced = true;
      }
    }
    if (!replaced) {
      out.append(src.data() + i, size_t(close + 1 - i));
    }
    i = close + 1;
  }
  return out;
}

static std::string rewrite_pixel_texel_fetch(StringRef src,
                                            const uint8_t nearest_mask,
                                            const uint8_t wrap_mask)
{
  /* MlVfDR stores voronoi particle IDs as pixel positions and samples
   * `texture(iChannel, U/R)` (nearest on the site). Linear blends two IDs and
   * the mosaic washes out; a nearest sampler + implicit LOD collapsed every
   * pixel onto one jet (flat green). texelFetch is filter- and lod-proof.
   * wdsGWS uses `(u+m)*r` with r=1/res and EMPTY==1024 — same idea. */
  if (nearest_mask == 0) {
    return std::string(src.data(), size_t(src.size()));
  }
  std::string out;
  out.reserve(size_t(src.size()) + 32);
  const char *funcs[] = {"texture2D", "texture"};
  int64_t i = 0;
  while (i < src.size()) {
    int64_t best = StringRef::not_found;
    const char *best_fn = nullptr;
    for (const char *fn : funcs) {
      const StringRef key(fn);
      const int64_t hit = src.find(key, i);
      if (hit == StringRef::not_found) {
        continue;
      }
      const char before = (hit > 0) ? src[hit - 1] : ' ';
      const int64_t after_i = hit + key.size();
      const char after = (after_i < src.size()) ? src[after_i] : ' ';
      if (is_ident_char_st(before) || is_ident_char_st(after)) {
        continue;
      }
      if (best == StringRef::not_found || hit < best) {
        best = hit;
        best_fn = fn;
      }
    }
    if (best == StringRef::not_found) {
      out.append(src.data() + i, size_t(src.size() - i));
      break;
    }
    const StringRef key(best_fn);
    int64_t q = best + key.size();
    while (q < src.size() && std::isspace(static_cast<unsigned char>(src[q]))) {
      q++;
    }
    if (q >= src.size() || src[q] != '(') {
      out.append(src.data() + i, size_t(q - i));
      i = q;
      continue;
    }
    const int64_t close = matching_paren(src, q);
    if (close == StringRef::not_found) {
      out.append(src.data() + i, size_t(src.size() - i));
      break;
    }
    StringRef inside = src.substr(q + 1, close - (q + 1));
    int depth = 0;
    int64_t c0 = StringRef::not_found;
    for (int64_t k = 0; k < inside.size(); k++) {
      const char ch = inside[k];
      if (ch == '(') {
        depth++;
      }
      else if (ch == ')') {
        depth--;
      }
      else if (ch == ',' && depth == 0) {
        c0 = k;
        break;
      }
    }
    bool replaced = false;
    if (c0 != StringRef::not_found) {
      StringRef a0 = rtrim(ltrim(inside.substr(0, c0)));
      StringRef a1 = rtrim(ltrim(inside.substr(c0 + 1)));
      int ch = -1;
      if (a0.size() == 9 && a0.startswith("iChannel") && a0[8] >= '0' && a0[8] <= '3') {
        ch = a0[8] - '0';
      }
      int64_t c1 = StringRef::not_found;
      int d1 = 0;
      for (int64_t k = c0 + 1; k < inside.size(); k++) {
        const char ch2 = inside[k];
        if (ch2 == '(') {
          d1++;
        }
        else if (ch2 == ')') {
          d1--;
        }
        else if (ch2 == ',' && d1 == 0) {
          c1 = k;
          break;
        }
      }
      if (c1 != StringRef::not_found) {
        /* 3-arg texture(samp, uv, lod) — leave it. */
      }
      else if (ch >= 0 && (nearest_mask & uint8_t(1u << ch)) != 0) {
        StringRef expr;
        bool div_r = false;
        if (a1.size() >= 3 && a1[a1.size() - 2] == '/' && a1[a1.size() - 1] == 'R') {
          expr = rtrim(a1.substr(0, a1.size() - 2));
          div_r = !expr.is_empty();
        }
        else if (a1.size() >= 4 && a1[a1.size() - 3] == '/' && a1[a1.size() - 2] == ' ' &&
                 a1[a1.size() - 1] == 'R')
        {
          expr = rtrim(a1.substr(0, a1.size() - 3));
          div_r = !expr.is_empty();
        }
        const bool looks_vec3 = a1.startswith("normalize") || a1.startswith("vec3") ||
                                a1.startswith("reflect") || a1.startswith("refract");
        if (div_r) {
          out.append(src.data() + i, size_t(best - i));
          if (wrap_mask & uint8_t(1u << ch)) {
            out += "_st_fetch_px_rep(";
          }
          else {
            out += "_st_fetch_px(";
          }
          out.append(a0.data(), size_t(a0.size()));
          out += ", ";
          out.append(expr.data(), size_t(expr.size()));
          out += ")";
          replaced = true;
        }
        else if (!looks_vec3) {
          /* wdsGWS: `texture(iChannel0, (u+m.xy)*r)` with r=1/res. Linear
           * blends EMPTY=1024 into ball xy and `== EMPTY` never hits. */
          out.append(src.data() + i, size_t(best - i));
          if (wrap_mask & uint8_t(1u << ch)) {
            out += "_st_fetch_uv_rep(";
          }
          else {
            out += "_st_fetch_uv(";
          }
          out.append(a0.data(), size_t(a0.size()));
          out += ", ";
          out.append(a1.data(), size_t(a1.size()));
          out += ")";
          replaced = true;
        }
      }
    }
    if (!replaced) {
      out.append(src.data() + i, size_t(close + 1 - i));
    }
    i = close + 1;
  }
  return out;
}

/* IQ packed 3D noise samples ONE bilinear tap and lerps .yx by f.z.
 * That is only continuous in Z if G(x,y) == R(x-37,y-17). Independent RGBA
 * (our dummy, and many user textures) makes every integer-Z plane a hard
 * seam — cube grid on Remnant X's sky, which is GetSky(dir) on a sphere.
 * Two taps of the same channel at uv and uv+(37,17) are actually C0. */
static std::string rewrite_iq_z_slices(std::string s)
{
  const std::string mark = "_st_tex2d_lod0(";
  size_t pos = 0;
  while ((pos = s.find(mark, pos)) != std::string::npos) {
    const size_t call = pos;
    int depth = 0;
    size_t close = std::string::npos;
    for (size_t p = call + mark.size() - 1; p < s.size(); p++) {
      if (s[p] == '(') {
        depth++;
      }
      else if (s[p] == ')') {
        depth--;
        if (depth == 0) {
          close = p;
          break;
        }
      }
    }
    if (close == std::string::npos) {
      break;
    }
    size_t sw = close + 1;
    while (sw < s.size() && std::isspace(static_cast<unsigned char>(s[sw]))) {
      sw++;
    }
    if (sw + 3 > s.size() || s.compare(sw, 3, ".yx") != 0) {
      pos = close + 1;
      continue;
    }
    size_t stmt = call;
    while (stmt > 0 && s[stmt - 1] != ';' && s[stmt - 1] != '{' && s[stmt - 1] != '\n') {
      stmt--;
    }
    if (stmt > 0 && s[stmt - 1] == '\n') {
      /* keep indent; start after previous newline already */
    }
    while (stmt < call && (s[stmt] == ' ' || s[stmt] == '\t')) {
      /* include indent in the replaced range */
      break;
    }
    size_t ident_start = stmt;
    while (ident_start < call && (s[ident_start] == ' ' || s[ident_start] == '\t')) {
      ident_start++;
    }
    if (s.compare(ident_start, 4, "vec2") != 0) {
      pos = close + 1;
      continue;
    }
    size_t name_s = ident_start + 4;
    while (name_s < call && std::isspace(static_cast<unsigned char>(s[name_s]))) {
      name_s++;
    }
    size_t name_e = name_s;
    while (name_e < call && is_ident_char_st(s[name_e])) {
      name_e++;
    }
    if (name_e == name_s) {
      pos = close + 1;
      continue;
    }
    const std::string var = s.substr(name_s, name_e - name_s);
    size_t semi = s.find(';', sw);
    if (semi == std::string::npos || semi > sw + 8) {
      pos = close + 1;
      continue;
    }
    /* Mix is usually the next statement: return mix(rg.x, rg.y, f.z); */
    size_t mix_at = semi + 1;
    while (mix_at < s.size() && std::isspace(static_cast<unsigned char>(s[mix_at]))) {
      mix_at++;
    }
    const bool ret = (s.compare(mix_at, 6, "return") == 0);
    size_t mix_kw = mix_at;
    if (ret) {
      mix_kw = mix_at + 6;
      while (mix_kw < s.size() && std::isspace(static_cast<unsigned char>(s[mix_kw]))) {
        mix_kw++;
      }
    }
    if (s.compare(mix_kw, 4, "mix(") != 0) {
      pos = close + 1;
      continue;
    }
    const size_t mix_paren = mix_kw + 3;
    const int64_t mix_close_i = matching_paren(StringRef(s.c_str(), int64_t(s.size())),
                                               int64_t(mix_paren));
    if (mix_close_i == StringRef::not_found) {
      pos = close + 1;
      continue;
    }
    const size_t mix_close = size_t(mix_close_i);
    const std::string inside = s.substr(mix_paren + 1, mix_close - (mix_paren + 1));
    const std::string xname = var + ".x";
    const std::string yname = var + ".y";
    if (inside.find(xname) == std::string::npos || inside.find(yname) == std::string::npos) {
      pos = close + 1;
      continue;
    }
    /* Extract sampler + uv from _st_tex2d_lod0(samp, uv). */
    const std::string args = s.substr(call + mark.size(), close - (call + mark.size()));
    int ad = 0;
    size_t comma = std::string::npos;
    for (size_t k = 0; k < args.size(); k++) {
      if (args[k] == '(') {
        ad++;
      }
      else if (args[k] == ')') {
        ad--;
      }
      else if (args[k] == ',' && ad == 0) {
        comma = k;
        break;
      }
    }
    if (comma == std::string::npos) {
      pos = close + 1;
      continue;
    }
    std::string samp = args.substr(0, comma);
    std::string uv = args.substr(comma + 1);
    auto trim = [](std::string &t) {
      size_t a = 0;
      while (a < t.size() && std::isspace(static_cast<unsigned char>(t[a]))) {
        a++;
      }
      size_t b = t.size();
      while (b > a && std::isspace(static_cast<unsigned char>(t[b - 1]))) {
        b--;
      }
      t = t.substr(a, b - a);
    };
    trim(samp);
    trim(uv);
    /* `(uv+0.5)/256` → extra slice is `(uv+vec2(37,17)+0.5)/256`. */
    std::string uv1 = uv;
    const size_t plus = uv.find("+");
    if (plus != std::string::npos && uv.find("256") != std::string::npos) {
      uv1 = uv.substr(0, plus) + "+vec2(37.0,17.0)" + uv.substr(plus);
    }
    else {
      uv1 = "(" + uv + "+vec2(37.0,17.0)/256.0)";
    }
    std::string repl;
    const std::string indent = s.substr(stmt, ident_start - stmt);
    repl += indent;
    repl += "float _st_n0 = _st_tex2d_lod0(";
    repl += samp;
    repl += ", ";
    repl += uv;
    repl += ").x;\n";
    repl += indent;
    repl += "float _st_n1 = _st_tex2d_lod0(";
    repl += samp;
    repl += ", ";
    repl += uv1;
    repl += ").x;\n";
    repl += indent;
    if (ret) {
      repl += "return mix(_st_n0, _st_n1, f.z);";
    }
    else {
      repl += "mix(_st_n0, _st_n1, f.z);";
    }
    size_t mix_semi = mix_close;
    while (mix_semi < s.size() && s[mix_semi] != ';') {
      mix_semi++;
    }
    if (mix_semi < s.size()) {
      mix_semi++;
    }
    s.replace(stmt, mix_semi - stmt, repl);
    pos = stmt + repl.size();
  }
  return s;
}

/* `fract(sin(x)*43758.5453)` aliases on desktop highp (tdG3Rd banding). Replacing it
 * also *rebuilds* Shane fractals (4s3SRN / 4scXzn) because the hash *is* the geometry.
 * Leave the original expression so ShaderToy matches; accept mediump vs highp drift. */
static std::string rewrite_unstable_sin_hash(StringRef src)
{
  /* Identity: see comment above. */
  return std::string(src.data(), size_t(src.size()));
}

static bool is_glsl_value_type(const StringRef t)
{
  return t == "float" || t == "int" || t == "uint" || t == "bool" || t == "vec2" || t == "vec3" ||
         t == "vec4" || t == "ivec2" || t == "ivec3" || t == "ivec4" || t == "uvec2" ||
         t == "uvec3" || t == "uvec4" || t == "mat2" || t == "mat3" || t == "mat4" ||
         t == "bvec2" || t == "bvec3" || t == "bvec4";
}

/* WebGL2 allows `vec3[3] a` / `inout vec2[3] id` (prefix size). Desktop GLSL wants
 * `vec3 a[3]` / `inout vec2 id[3]`. dl2fzz Buffer A `tr()`/`nr()` used the prefix
 * form; if that pass fails, Image's DOF samples a cleared buffer → solid black.
 *
 * Keep sized constructors `vec2[3](...)`. Rewriting them to unsized `vec2[](...)`
 * is what some NVIDIA/OpenGL parsers reject even after the declaration rewrite. */
static std::string rewrite_c_style_array_types(StringRef src)
{
  std::string s(src.data(), size_t(src.size()));
  std::string out;
  out.reserve(s.size() + 16);
  int64_t i = 0;
  const StringRef in(s.c_str(), int64_t(s.size()));
  while (i < in.size()) {
    std::string type;
    StringRef rest = in.substr(i);
    const int64_t type_at = i;
    if (!((i == 0 || !is_ident_char_st(in[i - 1])) && skip_ident_name(rest, type) &&
          is_glsl_value_type(type)))
    {
      out.push_back(in[i]);
      i++;
      continue;
    }
    i = in.size() - rest.size();
    StringRef after = ltrim(rest);
    if (after.size() < 3 || after[0] != '[') {
      out.append(in.data() + type_at, size_t(i - type_at));
      continue;
    }
    int64_t n = 1;
    while (n < after.size() && std::isdigit(static_cast<unsigned char>(after[n]))) {
      n++;
    }
    if (n == 1 || n >= after.size() || after[n] != ']') {
      out.append(in.data() + type_at, size_t(i - type_at));
      continue;
    }
    StringRef tail = ltrim(after.substr(n + 1));
    /* Constructor `vec2[3](...)` — leave the sized form alone. */
    if (!tail.is_empty() && tail[0] == '(') {
      out.append(in.data() + type_at, size_t(i - type_at));
      continue;
    }
    std::string name;
    StringRef namest = tail;
    if (!skip_ident_name(namest, name)) {
      out.append(in.data() + type_at, size_t(i - type_at));
      continue;
    }
    out += type;
    out += ' ';
    out += name;
    out.append(after.data(), size_t(n + 1));
    i = in.size() - namest.size();
  }
  return out;
}

static std::string glsl_zero_for_type(const StringRef type,
                                     const Map<std::string, std::string> &struct_ctors);

/* Vulkan GLSL forbids initializers on non-const file-scope variables
 * (`vec2 res = vec2(0);` in XsK3RR, `float tau = atan(1.0)*8.0;`).
 * Declare the variable, then assign it from `_st_init_globals()` in main.
 * Promoting `atan(...)` to `const` is also illegal (not a constant expression). */
struct VulkanRewrite {
  std::string source;
  std::string inits;
  /* File-scope `type name;` emitted outside `#if` so a `#if AA>1` dead branch
   * cannot drop the declaration while `_st_init_globals` still assigns it
   * (3dlSzs `once_AAlgs` / `retv_AA`). */
  std::string decls;
};

static StringRef strip_expr_tail(StringRef s)
{
  s = rtrim(s);
  const int64_t cmt = s.find("//");
  if (cmt != StringRef::not_found) {
    s = rtrim(s.substr(0, cmt));
  }
  const int64_t blk = s.find("/*");
  if (blk != StringRef::not_found) {
    s = rtrim(s.substr(0, blk));
  }
  if (!s.is_empty() && s[s.size() - 1] == ';') {
    s = rtrim(s.substr(0, s.size() - 1));
  }
  return s;
}

static VulkanRewrite rewrite_vulkan_globals(StringRef src)
{
  VulkanRewrite result;
  result.source.reserve(size_t(src.size()) + 32);
  int depth = 0;
  int64_t i = 0;
  static Map<std::string, std::string> no_ctors;
  while (i < src.size()) {
    const int64_t line_end = src.find('\n', i);
    const int64_t end = (line_end == StringRef::not_found) ? src.size() : line_end;
    StringRef line = src.substr(i, end - i);

    bool rewritten = false;
    if (depth == 0) {
      StringRef t = ltrim(line);
      if (!(t.startswith("#") || t.startswith("//") || t.startswith("const ") ||
            t.startswith("uniform ") || t.startswith("struct") || t.startswith("void ") ||
            t.startswith("layout") || t.startswith("precision") || t.startswith("in ") ||
            t.startswith("out ") || t.startswith("inout ")))
      {
        std::string type;
        StringRef parse = t;
        if (skip_ident_name(parse, type) && is_glsl_value_type(type)) {
          /* Vulkan forbids *initializers* on non-const file-scope vars
           * (`vec2 res = vec2(0);`). Bare `float light;` is legal but not
           * zero on desktop; hoist + assign in `_st_init_globals`. Keep any
           * unparsed tail (MlVfDR `float N` after `vec2 R;`). */
          struct Item {
            std::string type;
            std::string name;
            std::string expr;
          };
          Vector<Item> items;
          bool abort_line = false;
          StringRef leftover;
          while (!abort_line) {
            parse = ltrim(parse);
            leftover = parse;
            std::string name;
            if (!skip_ident_name(parse, name)) {
              break;
            }
            parse = ltrim(parse);
            if (!parse.is_empty() && (parse[0] == '(' || parse[0] == '[')) {
              items.clear();
              abort_line = true;
              break;
            }
            std::string expr;
            if (!parse.is_empty() && parse[0] == '=') {
              parse = ltrim(parse.substr(1));
              int pd = 0;
              int64_t k = 0;
              for (; k < parse.size(); k++) {
                const char c = parse[k];
                if (c == '/' && k + 1 < parse.size() &&
                    (parse[k + 1] == '/' || parse[k + 1] == '*'))
                {
                  break;
                }
                if (c == '(' || c == '[' || c == '{') {
                  pd++;
                }
                else if (c == ')' || c == ']' || c == '}') {
                  pd--;
                }
                else if ((c == ',' || c == ';') && pd == 0) {
                  break;
                }
              }
              StringRef e = rtrim(parse.substr(0, k));
              expr.assign(e.data(), size_t(e.size()));
              parse = parse.substr(k);
            }
            items.append({type, name, expr});
            leftover = parse;
            parse = ltrim(parse);
            if (parse.is_empty() || parse.startswith("//")) {
              leftover = StringRef();
              break;
            }
            if (parse[0] == ',') {
              parse = parse.substr(1);
              continue;
            }
            if (parse[0] == ';') {
              parse = ltrim(parse.substr(1));
              leftover = parse;
              if (parse.is_empty() || parse.startswith("//")) {
                leftover = StringRef();
                break;
              }
              std::string next_type;
              StringRef peek = parse;
              if (skip_ident_name(peek, next_type) && is_glsl_value_type(next_type)) {
                type = next_type;
                parse = peek;
                continue;
              }
              /* Keep the unparsed tail in the shader (do not drop `float N`). */
              break;
            }
            items.clear();
            abort_line = true;
            break;
          }
          if (!abort_line && !items.is_empty()) {
            /* Hoist every file-scope value decl, with or without `=`.
             * Vulkan forbids `float light = 0.0;` at global scope, so we
             * emit a bare `type name;` and assign from `_st_init_globals`.
             * Uninitialized globals (`float light;` in 3ccyD7) are 0 on
             * WebGL and garbage on desktop; `light +=` then HDR-blows the
             * image. Do not drop an unparsed tail on the same line. */
            for (const Item &it : items) {
              result.decls += it.type;
              result.decls += " ";
              result.decls += it.name;
              result.decls += ";\n";
              std::string rhs = it.expr;
              if (rhs.empty()) {
                rhs = glsl_zero_for_type(it.type, no_ctors);
              }
              if (!rhs.empty()) {
                result.inits += "  ";
                result.inits += it.name;
                result.inits += " = ";
                result.inits += rhs;
                result.inits += ";\n";
              }
            }
            rewritten = true;
            leftover = ltrim(leftover);
            if (!leftover.is_empty() && !leftover.startswith("//")) {
              result.source.append(leftover.data(), size_t(leftover.size()));
            }
          }
        }
      }
    }

    if (!rewritten) {
      result.source.append(line.data(), size_t(line.size()));
    }
    if (end < src.size()) {
      result.source.push_back('\n');
    }

    for (int64_t c = 0; c < line.size(); c++) {
      if (c + 1 < line.size() && line[c] == '/' && line[c + 1] == '/') {
        break;
      }
      if (line[c] == '{') {
        depth++;
      }
      else if (line[c] == '}' && depth > 0) {
        depth--;
      }
    }
    i = (end < src.size()) ? end + 1 : src.size();
  }
  return result;
}

static int64_t matching_brace(StringRef s, const int64_t open)
{
  if (open < 0 || open >= s.size() || s[open] != '{') {
    return StringRef::not_found;
  }
  int depth = 1;
  for (int64_t i = open + 1; i < s.size(); i++) {
    if (i + 1 < s.size() && s[i] == '/' && s[i + 1] == '/') {
      while (i < s.size() && s[i] != '\n') {
        i++;
      }
      continue;
    }
    if (s[i] == '{') {
      depth++;
    }
    else if (s[i] == '}') {
      depth--;
      if (depth == 0) {
        return i;
      }
    }
  }
  return StringRef::not_found;
}

static std::string glsl_zero_for_type(const StringRef type, const Map<std::string, std::string> &struct_ctors)
{
  if (type == "float") {
    return "0.0";
  }
  if (type == "int") {
    return "0";
  }
  if (type == "uint") {
    return "0u";
  }
  if (type == "bool") {
    return "false";
  }
  if (type == "vec2") {
    return "vec2(0.0)";
  }
  if (type == "vec3") {
    return "vec3(0.0)";
  }
  if (type == "vec4") {
    return "vec4(0.0)";
  }
  if (type == "ivec2") {
    return "ivec2(0)";
  }
  if (type == "ivec3") {
    return "ivec3(0)";
  }
  if (type == "ivec4") {
    return "ivec4(0)";
  }
  if (type == "uvec2") {
    return "uvec2(0u)";
  }
  if (type == "uvec3") {
    return "uvec3(0u)";
  }
  if (type == "uvec4") {
    return "uvec4(0u)";
  }
  if (type == "bvec2") {
    return "bvec2(false)";
  }
  if (type == "bvec3") {
    return "bvec3(false)";
  }
  if (type == "bvec4") {
    return "bvec4(false)";
  }
  if (type == "mat2") {
    return "mat2(0.0)";
  }
  if (type == "mat3") {
    return "mat3(0.0)";
  }
  if (type == "mat4") {
    return "mat4(0.0)";
  }
  if (const std::string *ctor = struct_ctors.lookup_ptr(std::string(type))) {
    return *ctor;
  }
  return "";
}

static Map<std::string, std::string> collect_struct_zero_ctors(const StringRef src)
{
  Map<std::string, std::string> ctors;
  int64_t i = 0;
  while (i < src.size()) {
    const int64_t hit = src.find("struct", i);
    if (hit == StringRef::not_found) {
      break;
    }
    const char before = (hit > 0) ? src[hit - 1] : ' ';
    const char after = (hit + 6 < src.size()) ? src[hit + 6] : ' ';
    if (is_ident_char_st(before) || is_ident_char_st(after)) {
      i = hit + 1;
      continue;
    }
    int64_t q = hit + 6;
    while (q < src.size() && std::isspace(static_cast<unsigned char>(src[q]))) {
      q++;
    }
    std::string sname;
    StringRef rest = src.substr(q);
    if (!skip_ident_name(rest, sname)) {
      i = hit + 1;
      continue;
    }
    q = src.size() - rest.size();
    while (q < src.size() && std::isspace(static_cast<unsigned char>(src[q]))) {
      q++;
    }
    if (q >= src.size() || src[q] != '{') {
      i = hit + 1;
      continue;
    }
    const int64_t close = matching_brace(src, q);
    if (close == StringRef::not_found) {
      break;
    }
    Vector<std::string> member_zeros;
    bool ok = true;
    StringRef body = src.substr(q + 1, close - (q + 1));
    int64_t b = 0;
    while (b < body.size() && ok) {
      const int64_t line_end = body.find('\n', b);
      const int64_t end = (line_end == StringRef::not_found) ? body.size() : line_end;
      StringRef line = ltrim(rtrim(body.substr(b, end - b)));
      const int64_t cmt = line.find("//");
      if (cmt != StringRef::not_found) {
        line = rtrim(line.substr(0, cmt));
      }
      b = (end < body.size()) ? end + 1 : body.size();
      if (line.is_empty() || line.startswith("/*")) {
        continue;
      }
      std::string mtype;
      StringRef parse = line;
      if (!skip_ident_name(parse, mtype)) {
        continue;
      }
      std::string mname;
      parse = ltrim(parse);
      if (!skip_ident_name(parse, mname)) {
        continue;
      }
      const std::string z = glsl_zero_for_type(mtype, ctors);
      if (z.empty()) {
        ok = false;
        break;
      }
      member_zeros.append(z);
    }
    if (ok && !member_zeros.is_empty()) {
      std::string ctor = sname;
      ctor += "(";
      for (int mi = 0; mi < member_zeros.size(); mi++) {
        if (mi > 0) {
          ctor += ", ";
        }
        ctor += member_zeros[mi];
      }
      ctor += ")";
      ctors.add_overwrite(sname, ctor);
    }
    i = close + 1;
  }
  return ctors;
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

/* WebGL/ANGLE typically zero-fills locals; desktop Vulkan does not. mstfzS
 * does `Particle p0; ApplyForce` / `density +=` with no init, so pressure and
 * normals become garbage and the liquid both moves and shades wrong. */
static std::string rewrite_zero_init_locals(StringRef src,
                                           const Map<std::string, std::string> &struct_ctors)
{
  std::string out;
  out.reserve(size_t(src.size()) + 256);
  int depth = 0;
  int struct_depth = -1;
  int64_t i = 0;
  while (i < src.size()) {
    const int64_t line_end = src.find('\n', i);
    const int64_t end = (line_end == StringRef::not_found) ? src.size() : line_end;
    StringRef line = src.substr(i, end - i);
    StringRef t = ltrim(line);
    bool rewritten = false;
    if (struct_depth < 0 && t.startswith("struct") &&
        (t.size() == 6 || !is_ident_char_st(t[6])))
    {
      struct_depth = depth;
    }
    if (depth >= 1 && struct_depth < 0) {
      StringRef parse = t;
      std::string type;
      if (!parse.startswith("const ") && !parse.startswith("uniform ") &&
          !parse.startswith("in ") && !parse.startswith("out ") &&
          !parse.startswith("inout ") && !parse.startswith("layout") &&
          !parse.startswith("precision") && !parse.startswith("return") &&
          !parse.startswith("if") && !parse.startswith("for") &&
          !parse.startswith("while") && !parse.startswith("switch") &&
          !parse.startswith("discard") && !parse.startswith("#") &&
          !parse.startswith("//") && skip_ident_name(parse, type))
      {
        const std::string zero = glsl_zero_for_type(type, struct_ctors);
        if (!zero.empty()) {
          /* WebGL zeros every local. Golf shaders mix bare and initialized
           * names on one line: `vec4 o,p,P,U=vec4(1,2,3,0);` and
           * `float i,z,d,k,T=iChannelTime[0]*1.9,...` (4D Beats / tfK3Dy).
           * The old rewriter bailed on `=`, left `i`/`o` garbage, then
           * `++i<77.` became a billion-iteration TDR and NaN pixels went
           * black. Split the list; zero only the names with no initializer. */
          struct LocalDecl {
            std::string name;
            std::string init;
          };
          Vector<LocalDecl> decls;
          bool ok = true;
          parse = ltrim(parse);
          while (ok) {
            parse = ltrim(parse);
            std::string name;
            if (!skip_ident_name(parse, name)) {
              ok = false;
              break;
            }
            parse = ltrim(parse);
            if (!parse.is_empty() && (parse[0] == '(' || parse[0] == '[')) {
              /* Function signature or array — leave the line alone. */
              ok = false;
              break;
            }
            std::string init;
            if (!parse.is_empty() && parse[0] == '=') {
              parse = ltrim(parse.substr(1));
              int pd = 0;
              int64_t k = 0;
              for (; k < parse.size(); k++) {
                const char c = parse[k];
                if (c == '/' && k + 1 < parse.size() &&
                    (parse[k + 1] == '/' || parse[k + 1] == '*'))
                {
                  break;
                }
                if (c == '(' || c == '[' || c == '{') {
                  pd++;
                }
                else if (c == ')' || c == ']' || c == '}') {
                  pd--;
                }
                else if ((c == ',' || c == ';') && pd == 0) {
                  break;
                }
              }
              if (pd != 0) {
                ok = false;
                break;
              }
              StringRef expr = rtrim(parse.substr(0, k));
              init.assign(expr.data(), size_t(expr.size()));
              if (init.empty()) {
                ok = false;
                break;
              }
              parse = parse.substr(k);
            }
            decls.append({std::move(name), std::move(init)});
            parse = ltrim(parse);
            if (parse.is_empty() || parse[0] == ';' || parse.startswith("//")) {
              break;
            }
            if (parse[0] == ',') {
              parse = parse.substr(1);
              continue;
            }
            ok = false;
            break;
          }
          if (ok && !decls.is_empty()) {
            /* Shane packs a second statement after the decl (XsffWj):
             * `vec3 i = floor(...);  p -= i - dot(i, vec3(1./6.));`
             * Dropping the tail deletes the simplex unskew, rays miss, fog
             * is `vec3(0)`, and the image is pure black.
             * dl2fzz: `vec3 d; vec2 id[3]; int oID;` — WebGL zeros oID. */
            std::string leftover;
            if (!parse.is_empty() && parse[0] == ';') {
              leftover.assign(parse.data() + 1, size_t(parse.size() - 1));
            }
            const int64_t lead = line.size() - ltrim(line).size();
            for (int n = 0; n < decls.size(); n++) {
              if (n > 0) {
                out.push_back('\n');
              }
              out.append(line.data(), size_t(lead));
              out += type;
              out += ' ';
              out += decls[n].name;
              out += " = ";
              out += decls[n].init.empty() ? zero : decls[n].init;
              out += ';';
            }
            StringRef more(leftover.c_str(), int64_t(leftover.size()));
            while (!more.is_empty()) {
              more = ltrim(more);
              if (more.is_empty() || more.startswith("//")) {
                out.append(more.data(), size_t(more.size()));
                break;
              }
              std::string t2;
              StringRef peek = more;
              if (!skip_ident_name(peek, t2) || glsl_zero_for_type(t2, struct_ctors).empty()) {
                out.append(more.data(), size_t(more.size()));
                break;
              }
              peek = ltrim(peek);
              std::string n2;
              if (!skip_ident_name(peek, n2)) {
                out.append(more.data(), size_t(more.size()));
                break;
              }
              peek = ltrim(peek);
              if (!peek.is_empty() && peek[0] == '[') {
                int64_t end_decl = 0;
                while (end_decl < more.size() && more[end_decl] != ';') {
                  end_decl++;
                }
                if (end_decl < more.size()) {
                  end_decl++;
                }
                out.push_back('\n');
                out.append(line.data(), size_t(lead));
                out.append(more.data(), size_t(end_decl));
                more = more.substr(end_decl);
                continue;
              }
              if (!peek.is_empty() && peek[0] == '=') {
                int64_t end_decl = 0;
                int pd = 0;
                for (; end_decl < more.size(); end_decl++) {
                  const char c = more[end_decl];
                  if (c == '(' || c == '[' || c == '{') {
                    pd++;
                  }
                  else if (c == ')' || c == ']' || c == '}') {
                    pd--;
                  }
                  else if (c == ';' && pd == 0) {
                    end_decl++;
                    break;
                  }
                }
                out.push_back('\n');
                out.append(line.data(), size_t(lead));
                out.append(more.data(), size_t(end_decl));
                more = more.substr(end_decl);
                continue;
              }
              out.push_back('\n');
              out.append(line.data(), size_t(lead));
              out += t2;
              out += ' ';
              out += n2;
              out += " = ";
              out += glsl_zero_for_type(t2, struct_ctors);
              out += ';';
              more = ltrim(peek);
              if (!more.is_empty() && more[0] == ';') {
                more = more.substr(1);
              }
            }
            rewritten = true;
          }
        }
      }
    }

    if (!rewritten) {
      out.append(line.data(), size_t(line.size()));
    }
    if (end < src.size()) {
      out.push_back('\n');
    }

    for (int64_t c = 0; c < line.size(); c++) {
      if (c + 1 < line.size() && line[c] == '/' && line[c + 1] == '/') {
        break;
      }
      if (line[c] == '{') {
        depth++;
      }
      else if (line[c] == '}' && depth > 0) {
        depth--;
        if (struct_depth >= 0 && depth <= struct_depth) {
          struct_depth = -1;
        }
      }
    }
    i = (end < src.size()) ? end + 1 : src.size();
  }
  return out;
}

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

/* True if `name` is declared as a function (or `#define name(`) in user GLSL.
 * HLSL-style `#define saturate(a) clamp(...)` must not wrap such a definition:
 * `vec3 saturate(vec3 x)` becomes `vec3 clamp((vec3 x), ...)` and the GPU
 * compiler reports a paren error (often mis-attributed to an include). */
static bool source_has_fn_or_macro_def(StringRef src, const char *name)
{
  const int64_t n = int64_t(strlen(name));
  auto is_call_kw = [](StringRef t) {
    return t == "return" || t == "if" || t == "while" || t == "for" || t == "switch" ||
           t == "else" || t == "discard" || t == "break" || t == "continue" || t == "case";
  };
  int64_t i = 0;
  while (i < src.size()) {
    if (src[i] == '/' && i + 1 < src.size()) {
      if (src[i + 1] == '/') {
        i += 2;
        while (i < src.size() && src[i] != '\n') {
          i++;
        }
        continue;
      }
      if (src[i + 1] == '*') {
        i += 2;
        while (i + 1 < src.size() && !(src[i] == '*' && src[i + 1] == '/')) {
          i++;
        }
        i = (i + 1 < src.size()) ? i + 2 : src.size();
        continue;
      }
    }
    if (src[i] == '"' || src[i] == '\'') {
      const char q = src[i++];
      while (i < src.size() && src[i] != q) {
        if (src[i] == '\\' && i + 1 < src.size()) {
          i += 2;
        }
        else {
          i++;
        }
      }
      if (i < src.size()) {
        i++;
      }
      continue;
    }
    if (i + n <= src.size() && std::memcmp(src.data() + i, name, size_t(n)) == 0 &&
        (i == 0 || !is_ident_char_st(src[i - 1])) &&
        (i + n == src.size() || !is_ident_char_st(src[i + n])))
    {
      int64_t q = i + n;
      while (q < src.size() && std::isspace(static_cast<unsigned char>(src[q]))) {
        q++;
      }
      if (q < src.size() && src[q] == '(') {
        int64_t p = i;
        while (p > 0 && std::isspace(static_cast<unsigned char>(src[p - 1]))) {
          p--;
        }
        int64_t d = p;
        while (d > 0 && is_ident_char_st(src[d - 1])) {
          d--;
        }
        const StringRef prev = src.substr(d, p - d);
        if (prev == "define") {
          int64_t h = d;
          while (h > 0 && std::isspace(static_cast<unsigned char>(src[h - 1]))) {
            h--;
          }
          if (h > 0 && src[h - 1] == '#') {
            return true;
          }
        }
        if (!prev.is_empty() && is_ident_char_st(prev[0]) && !is_call_kw(prev)) {
          return true;
        }
      }
      i += n;
      continue;
    }
    i++;
  }
  return false;
}

static void append_hlsl_helpers(std::string &src, const char *common, const char *body)
{
  const StringRef common_ref = common ? StringRef(common) : StringRef();
  const StringRef body_ref = body ? StringRef(body) : StringRef();
  auto user_has = [&](const char *name) {
    return source_has_fn_or_macro_def(common_ref, name) ||
           source_has_fn_or_macro_def(body_ref, name);
  };
  /* Includes (or leftover driver macros) may still `#define saturate`. Undef
   * first so `vec3 saturate(vec3 x)` in user code stays a function. */
  src += "#undef saturate\n";
  src += "#undef lerp\n";
  src += "#undef frac\n";
  src += "#undef atan2\n";
  src += "#undef fmod\n";
  if (!user_has("saturate")) {
    src += "float saturate(float x) { return clamp(x, 0.0, 1.0); }\n";
    src += "vec2 saturate(vec2 x) { return clamp(x, 0.0, 1.0); }\n";
    src += "vec3 saturate(vec3 x) { return clamp(x, 0.0, 1.0); }\n";
    src += "vec4 saturate(vec4 x) { return clamp(x, 0.0, 1.0); }\n";
  }
  if (!user_has("lerp")) {
    src += "float lerp(float a, float b, float t) { return mix(a, b, t); }\n";
    src += "vec2 lerp(vec2 a, vec2 b, float t) { return mix(a, b, t); }\n";
    src += "vec2 lerp(vec2 a, vec2 b, vec2 t) { return mix(a, b, t); }\n";
    src += "vec3 lerp(vec3 a, vec3 b, float t) { return mix(a, b, t); }\n";
    src += "vec3 lerp(vec3 a, vec3 b, vec3 t) { return mix(a, b, t); }\n";
    src += "vec4 lerp(vec4 a, vec4 b, float t) { return mix(a, b, t); }\n";
    src += "vec4 lerp(vec4 a, vec4 b, vec4 t) { return mix(a, b, t); }\n";
  }
  if (!user_has("frac")) {
    src += "float frac(float x) { return fract(x); }\n";
    src += "vec2 frac(vec2 x) { return fract(x); }\n";
    src += "vec3 frac(vec3 x) { return fract(x); }\n";
    src += "vec4 frac(vec4 x) { return fract(x); }\n";
  }
  if (!user_has("atan2")) {
    src += "float atan2(float y, float x) { return atan(y, x); }\n";
    src += "vec2 atan2(vec2 y, vec2 x) { return atan(y, x); }\n";
    src += "vec3 atan2(vec3 y, vec3 x) { return atan(y, x); }\n";
    src += "vec4 atan2(vec4 y, vec4 x) { return atan(y, x); }\n";
  }
  if (!user_has("fmod")) {
    src += "float fmod(float a, float b) { return mod(a, b); }\n";
    src += "vec2 fmod(vec2 a, vec2 b) { return mod(a, b); }\n";
    src += "vec3 fmod(vec3 a, vec3 b) { return mod(a, b); }\n";
    src += "vec4 fmod(vec4 a, vec4 b) { return mod(a, b); }\n";
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
  std::string src;
  src.reserve(8192 + (common ? strlen(common) : 0) + (body ? strlen(body) : 0));
  /* GPU_shader_create_from_info_python only injects CREATE_INFO_RES_PASS_pyGPU_Shader,
   * so the ShaderCreateInfo must be named pyGPU_Shader (see gpu_shader.cc). */
  src += "#define iGlobalTime iTime\n";
  /* Push constant is vec4 (std430). ShaderToy iResolution is vec3; `vec3 r=iResolution`
   * (tfK3Dy) fails if the uniform stays float4. Name must not contain the token
   * iResolution or a string-replace preprocessor can recurse forever. */
  src += "#define iResolution _st_ires.xyz\n";
  src += "#define texture2D texture\n";
  src += "#define textureCube texture\n";
  src += "#define texture2DLod textureLod\n";
  src += "#define textureCubeLod textureLod\n";
  src += "#define texture2DGrad textureGrad\n";
  src += "#define texture2DProj textureProj\n";
  src += "#define highp\n";
  src += "#define mediump\n";
  src += "#define lowp\n";
  append_hlsl_helpers(src, common, body);
  src += "const float iSampleRate = 44100.0;\n";
  src += "float iChannelTime[4];\n";
  src += "vec3 iChannelResolution[4];\n";
  src += "float _st_hash11(float n)\n";
  src += "{\n";
  src += "  n = fract(n * 0.1031);\n";
  src += "  n *= n + 33.33;\n";
  src += "  n *= n + n;\n";
  src += "  return fract(n);\n";
  src += "}\n";
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
  /* Fill iChannelResolution BEFORE the user shader. Common tabs often
   * `#define textureSize iResolution.xy` (7tSSDD); that turns later
   * `textureSize(iChannel0, 0)` into `iResolution.xy(...)` which is
   * "function call expected". This helper is parsed above that define. */
  src += "void _st_fill_ichannel_res()\n";
  src += "{\n";
  for (int ch = 0; ch < 4; ch++) {
    char line[192];
    if (cube_mask & (1u << ch)) {
      SNPRINTF(line,
               "  { ivec2 _ts = textureSize(iChannel%d, 0); "
               "iChannelResolution[%d] = vec3(vec2(_ts), float(_ts.x)); }\n",
               ch,
               ch);
    }
    else if (vol_mask & (1u << ch)) {
      SNPRINTF(line,
               "  iChannelResolution[%d] = vec3(textureSize(iChannel%d, 0));\n",
               ch,
               ch);
    }
    else {
      SNPRINTF(line,
               "  iChannelResolution[%d] = vec3(vec2(textureSize(iChannel%d, 0)), 1.0);\n",
               ch,
               ch);
    }
    src += line;
  }
  src += "}\n";
  src += "\n";
  /* Rewrite original #define/const lines in place. Do NOT inject a global #define NAME
   * block — that hijacks function parameters (tmax, px, ro, …). */
  std::string common_src = (common && common[0]) ?
                               rewrite_loop_zero_starts(rewrite_iq_z_slices(
                                   rewrite_texture_neg_bias(rewrite_texturelod_zero(
                                       rewrite_iframe_zero_calls(rewrite_shadertoy_grid_compat(
                                           rewrite_unstable_sin_hash(apply_param_values(
                                               rewrite_c_style_array_types(
                                                   fold_srgb_byte_vec3_defines(
                                                       strip_shadertoy_preamble(common))),
                                               params)))))))) :
                               "";
  std::string body_src = rewrite_loop_zero_starts(rewrite_iq_z_slices(
      rewrite_texture_neg_bias(rewrite_texturelod_zero(rewrite_iframe_zero_calls(
          rewrite_shadertoy_grid_compat(rewrite_unstable_sin_hash(apply_param_values(
              rewrite_c_style_array_types(fold_srgb_byte_vec3_defines(
                  strip_shadertoy_preamble(body ? body : ""))),
              params))))))));
  if (nearest_mask != 0) {
    if (!common_src.empty()) {
      common_src = rewrite_pixel_texel_fetch(common_src, nearest_mask, wrap_mask);
    }
    body_src = rewrite_pixel_texel_fetch(body_src, nearest_mask, wrap_mask);
  }
  const std::string joined_for_structs = common_src + "\n" + body_src;
  const Map<std::string, std::string> struct_ctors = collect_struct_zero_ctors(joined_for_structs);

  VulkanRewrite common_rw;
  if (!common_src.empty()) {
    common_rw = rewrite_vulkan_globals(rewrite_zero_init_locals(common_src, struct_ctors));
  }
  VulkanRewrite body_rw = rewrite_vulkan_globals(rewrite_zero_init_locals(body_src, struct_ctors));
  /* Hoisted globals MUST sit before any user `#if` / `#define`. 3dlSzs keeps
   * `bool once_AAlgs = false;` inside `#if AA>1`; leaving the decl there makes
   * `_st_init_globals` assign a name the preprocessor deleted. */
  src += common_rw.decls;
  src += body_rw.decls;
  if (!common_rw.source.empty()) {
    src += common_rw.source;
    src += "\n";
  }
  src += body_rw.source;
  src += "\nvoid _st_init_globals()\n{\n";
  src += common_rw.inits;
  src += body_rw.inits;
  src += "}\n";
  src += "void main()\n";
  src += "{\n";
  src += "  iChannelTime[0] = iTime;\n";
  src += "  iChannelTime[1] = iTime;\n";
  src += "  iChannelTime[2] = iTime;\n";
  src += "  iChannelTime[3] = iTime;\n";
  src += "  _st_fill_ichannel_res();\n";
  src += "  _st_init_globals();\n";
  src += "  vec4 fragColor = vec4(0.0, 0.0, 0.0, 1.0);\n";
  if (cubemap_pass) {
    /* ShaderToy Cubemap A: mainCubemap(..., rayOri, rayDir). Face layout matches GL/Vulkan. */
    src += "  vec2 _st = gl_FragCoord.xy / iResolution.xy * 2.0 - 1.0;\n";
    src += "  vec3 rd;\n";
    src += "  if (iCubeFace == 0) { rd = vec3(1.0, -_st.y, -_st.x); }\n";
    src += "  else if (iCubeFace == 1) { rd = vec3(-1.0, -_st.y, _st.x); }\n";
    src += "  else if (iCubeFace == 2) { rd = vec3(_st.x, 1.0, _st.y); }\n";
    src += "  else if (iCubeFace == 3) { rd = vec3(_st.x, -1.0, -_st.y); }\n";
    src += "  else if (iCubeFace == 4) { rd = vec3(_st.x, -_st.y, 1.0); }\n";
    src += "  else { rd = vec3(-_st.x, -_st.y, -1.0); }\n";
    src += "  rd = normalize(rd);\n";
    src += "  mainCubemap(fragColor, gl_FragCoord.xy, vec3(0.0), rd);\n";
  }
  else {
    src += "  mainImage(fragColor, gl_FragCoord.xy);\n";
  }
  /* Image pass: ShaderToy writes display-referred sRGB (browser canvas). Convert
   * to scene-linear so the *blend file* view transform applies (Standard ≈ site,
   * AgX/Filmic = that file's look). Do not tag Non-Color — that locks CM.
   * Buffer passes MUST keep raw bits (camera packing / uintBitsToFloat). */
  if (force_opaque) {
    src += "  if (any(isnan(fragColor))) {\n";
    src += "    fragColor = vec4(0.0, 0.0, 0.0, 1.0);\n";
    src += "  }\n";
    src += "  fragColor.rgb = _st_srgb_to_linear(fragColor.rgb);\n";
    src += "  fragColor.a = 1.0;\n";
  }
  src += "  out_fragColor = fragColor;\n";
  src += "}\n";
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
  return s.find("mainCubemap") != StringRef::not_found &&
         s.find("mainImage") == StringRef::not_found;
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
  uint64_t h = hasher("shadertoy_v58_zero_file_globals");
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
  bool cubemap_pass = is_cubemap_only_pass(body) && !force_opaque;
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
  const bool has_entry = cubemap_pass ||
                         StringRef(body_use).find("mainImage") != StringRef::not_found;
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
  static gpu::Texture *tex = nullptr;
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
  static gpu::Texture *tex = nullptr;
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
  static gpu::Texture *tex = nullptr;
  static bool tried = false;
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
  static gpu::Texture *tex = nullptr;
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
  static gpu::Texture *tex = nullptr;
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
  static gpu::FrameBuffer *fb = nullptr;
  if (fb == nullptr) {
    fb = GPU_framebuffer_create("shadertoy_fb");
  }
  return fb;
}

static gpu::Batch *shadertoy_tri()
{
  static gpu::Batch *batch = nullptr;
  if (batch == nullptr) {
    batch = GPU_batch_create_procedural(GPU_PRIM_TRIS, 6);
  }
  return batch;
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

static bool pass_has_code(const char *code)
{
  if (code == nullptr || code[0] == '\0') {
    return false;
  }
  const StringRef s(code);
  return s.find("mainImage") != StringRef::not_found ||
         s.find("mainCubemap") != StringRef::not_found;
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
      if (is_cubemap_only_pass(storage_buffer_code(storage, c))) {
        cube_mask |= bit;
        vol_mask &= uint8_t(~bit);
      }
      continue;
    }
    if (bind.kind != ChannelKind::Buffer) {
      continue;
    }
    if (is_cubemap_only_pass(storage_buffer_code(storage, bind.buffer))) {
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
      if (hint >= 0 && hint < 4 && is_cubemap_only_pass(storage_buffer_code(storage, hint))) {
        return hint;
      }
      for (int b = 0; b < 4; b++) {
        if (is_cubemap_only_pass(storage_buffer_code(storage, b))) {
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
        const bool src_cube = is_cubemap_only_pass(storage_buffer_code(storage, buffer));
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
      if (pass_has_code(storage_buffer_code(storage, c)) &&
          !is_cubemap_only_pass(storage_buffer_code(storage, c)))
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
        if (pass_has_code(passes[p].code)) {
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
        if (!pass_has_code(passes[p].code)) {
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
    else if (pass_has_code(storage.code_image)) {
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
  if (!pass_has_code(storage.code_image)) {
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
    if (pass.code != storage.code_image && !pass_has_code(pass.code)) {
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
