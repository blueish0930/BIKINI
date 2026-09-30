/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup shdnodes
 *
 * Shader Wrangle: VEX body compiled to an EEVEE material function.
 * Field sockets are `@name`; Compile creates Constant sockets from chf/chi/….
 */

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string>
#include <variant>

#include "MEM_guardedalloc.h"

#include "BLI_hash.hh"
#include "BLI_map.hh"
#include "BLI_set.hh"
#include "BLI_span.hh"
#include "BLI_string.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "BKE_context.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_report.hh"
#include "BKE_scene.hh"

#include "BLO_read_write.hh"

#include "DNA_array_utils.hh"
#include "DNA_node_types.h"
#include "DNA_scene_types.h"
#include "DNA_userdef_types.h"

#include "ED_screen.hh"

#include "GPU_material.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "NOD_node_extra_info.hh"
#include "NOD_shader.h"
#include "NOD_shader_wrangle.hh"
#include "NOD_socket.hh"
#include "NOD_socket_items_blend.hh"
#include "NOD_socket_items_ops.hh"
#include "NOD_socket_items_ui.hh"
#include "NOD_socket_search_link.hh"
#include "NOD_vex.hh"

#include "node_shader_util.hh"
#include "node_util.hh"

#include "vex/vex_program.hh"

#include "../../../editors/interface/interface_intern.hh"

namespace blender::nodes::node_shader_wrangle_cc {

NODE_STORAGE_FUNCS(NodeShaderWrangle)

static const char *default_wrangle_code()
{
  return "Output = {0, 0, 0};";
}

/* -------------------------------------------------------------------- */
/** \name Tab autocomplete (TextBox is not a SearchMenu; do not call button_func_search_set)
 * \{ */

static constexpr int WRANGLE_STR_MAX = 65536;

static bool item_matches_token(const expression::CompletionItem &item,
                               const StringRef token,
                               const bool is_member,
                               const bool show_all)
{
  if (item.kind == expression::CompletionKind::Header) {
    return false;
  }
  if (is_member) {
    return item.kind == expression::CompletionKind::Member;
  }
  if (item.kind == expression::CompletionKind::Member) {
    return false;
  }
  if (show_all) {
    return true;
  }
  if (token.is_empty()) {
    return false;
  }
  const StringRef key = item.insert_text.is_empty() ? item.name : item.insert_text;
  if (key.size() < token.size()) {
    return false;
  }
  for (const int64_t i : token.index_range()) {
    const char a = key[i];
    const char b = token[i];
    const char al = (a >= 'A' && a <= 'Z') ? char(a + 32) : a;
    const char bl = (b >= 'A' && b <= 'Z') ? char(b + 32) : b;
    if (al != bl) {
      return false;
    }
  }
  return true;
}

static StringRef completion_line(const StringRef text)
{
  int end = int(text.size());
  while (end > 0 && ELEM(text[end - 1], ' ', '\t', '\r')) {
    end--;
  }
  int start = end;
  while (start > 0 && text[start - 1] != '\n') {
    start--;
  }
  return text.substr(start, end - start);
}

static std::string tab_complete_result(const StringRef expression,
                                       const int token_start,
                                       const int token_len,
                                       const Span<const expression::CompletionItem *> matches)
{
  BLI_assert(!matches.is_empty());
  if (matches.size() == 1) {
    return expression::apply_completion(expression,
                                        token_start,
                                        token_len,
                                        matches[0]->insert_text,
                                        matches[0]->insert_call_parens);
  }
  StringRef lcp = matches[0]->insert_text;
  for (const expression::CompletionItem *item : matches.drop_front(1)) {
    const StringRef t = item->insert_text;
    int64_t n = 0;
    while (n < lcp.size() && n < t.size() && lcp[n] == t[n]) {
      n++;
    }
    lcp = lcp.substr(0, n);
  }
  const StringRef token = expression.substr(token_start, token_len);
  if (lcp.size() <= token.size()) {
    return expression::apply_completion(expression,
                                        token_start,
                                        token_len,
                                        matches[0]->insert_text,
                                        matches[0]->insert_call_parens);
  }
  return expression::apply_completion(expression, token_start, token_len, lcp, false);
}

static void collect_matches(const StringRef token,
                            const bool is_member,
                            const bool show_all,
                            Vector<std::string> &r_owned,
                            Vector<expression::CompletionItem> &r_all,
                            Vector<const expression::CompletionItem *> &r_matches)
{
  r_owned.clear();
  r_all.clear();
  r_matches.clear();
  vex::gather_completions(r_owned, r_all, true);
  for (const expression::CompletionItem &item : r_all) {
    if (item_matches_token(item, token, is_member, show_all)) {
      r_matches.append(&item);
    }
  }
}

static int wrangle_autocomplete_fn(bContext * /*C*/, char *str, void * /*arg_v*/)
{
  if (!str) {
    return AUTOCOMPLETE_NO_MATCH;
  }
  const StringRef expression = str;
  const StringRef line = completion_line(expression);
  int token_start = 0;
  int token_len = 0;
  bool is_member = false;
  if (!expression::find_completion_token(line, token_start, token_len, is_member)) {
    return AUTOCOMPLETE_NO_MATCH;
  }
  if (token_len == 0) {
    return AUTOCOMPLETE_NO_MATCH;
  }
  const int line_off = int(line.data() - expression.data());
  token_start += line_off;

  Vector<std::string> owned;
  Vector<expression::CompletionItem> all_items;
  Vector<const expression::CompletionItem *> matches;
  collect_matches(
      expression.substr(token_start, token_len), is_member, false, owned, all_items, matches);
  if (matches.is_empty()) {
    return AUTOCOMPLETE_NO_MATCH;
  }
  Vector<const expression::CompletionItem *> unique;
  for (const expression::CompletionItem *item : matches) {
    bool seen = false;
    for (const expression::CompletionItem *u : unique) {
      if (u->insert_text == item->insert_text) {
        seen = true;
        break;
      }
    }
    if (!seen) {
      unique.append(item);
    }
  }
  const std::string result = tab_complete_result(expression, token_start, token_len, unique);
  if (result.size() >= WRANGLE_STR_MAX) {
    return AUTOCOMPLETE_NO_MATCH;
  }
  BLI_strncpy(str, result.c_str(), WRANGLE_STR_MAX);
  return AUTOCOMPLETE_PARTIAL_MATCH;
}

static void wrangle_suggest_fn(const char *str,
                               int cursor,
                               void * /*arg*/,
                               char (*out)[192],
                               int *r_num,
                               const int max_num)
{
  *r_num = 0;
  if (!str || max_num <= 0) {
    return;
  }
  const bool show_all = cursor < 0;
  const int len = int(strlen(str));
  cursor = std::clamp(show_all ? len : cursor, 0, len);
  bool is_member = false;
  int start = cursor;
  while (start > 0) {
    const char c = str[start - 1];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' ||
        c == '@')
    {
      start--;
    }
    else {
      if (c == '.') {
        is_member = true;
      }
      break;
    }
  }
  const StringRef match_token(str + start, std::max(cursor - start, 0));
  if (match_token.is_empty() && !is_member && !show_all) {
    return;
  }
  Vector<std::string> owned;
  Vector<expression::CompletionItem> all_items;
  Vector<const expression::CompletionItem *> matches;
  collect_matches(match_token, is_member, show_all, owned, all_items, matches);
  int n = 0;
  for (const expression::CompletionItem *item : matches) {
    if (n >= max_num) {
      break;
    }
    const StringRef shown = item->usage.is_empty() ?
                                (item->insert_text.is_empty() ? item->name : item->insert_text) :
                                item->usage;
    const StringRef ins = item->insert_text.is_empty() ? item->name : item->insert_text;
    std::string packed = std::string(shown);
    if (item->kind == expression::CompletionKind::Function) {
      const StringRef types = vex::function_param_types(item->name);
      if (!types.is_empty()) {
        packed.push_back('\x1f');
        packed += types;
      }
    }
    bool dup = false;
    for (int i = 0; i < n; i++) {
      if (StringRef(packed) == StringRef(out[i])) {
        dup = true;
        break;
      }
    }
    if (dup) {
      continue;
    }
    StringRef(packed).copy_utf8_truncated(out[n], 192);
    n++;
    (void)ins;
  }
  *r_num = n;
}

static void wrangle_code_diag_fn(const char *str,
                                 void * /*arg*/,
                                 ui::uiCodeDiag *out,
                                 int *r_num,
                                 const int max_num)
{
  *r_num = 0;
  if (!str || max_num <= 0) {
    return;
  }
  const vex::CompileOutput compiled = vex::compile_shader_material(str);
  const int n = std::min(max_num, int(compiled.diags.size()));
  for (int i = 0; i < n; i++) {
    out[i].offset = compiled.diags[i].offset;
    out[i].length = std::max(1, compiled.diags[i].length);
    out[i].is_error = compiled.diags[i].is_error ? 1 : 0;
  }
  *r_num = n;
}

static void wrangle_apply_code_editor_flags(ui::Block *block, const int64_t buttons_before)
{
  if (!block) {
    return;
  }
  for (int64_t i = buttons_before; i < block->buttons_ptrs.size(); i++) {
    ui::Button *but = block->buttons_ptrs[i].get();
    if (but && but->type == ui::ButtonType::TextBox) {
      button_flag_enable(but, ui::BUT_CODE_EDITOR);
      button_drawflag_enable(but, ui::BUT_TEXT_LEFT);
      button_drawflag_disable(but, ui::BUT_NO_TEXT_PADDING);
      button_func_complete_set(but, wrangle_autocomplete_fn, nullptr);
      button_func_suggest_set(but, wrangle_suggest_fn, nullptr);
      button_func_code_diag_set(but, wrangle_code_diag_fn, nullptr);
    }
  }
}

/** \} */

static std::string field_attr_name(const StringRef name)
{
  if (ELEM(name, "P", "p", "Position")) {
    return "position";
  }
  if (ELEM(name, "N", "n", "Normal")) {
    return "normal";
  }
  return std::string(name);
}

static std::string glsl_ident(const StringRef name, const char *prefix)
{
  std::string s = prefix;
  /* Same reserved-name list as VEX GPU emit. `ch_lerp` is not safe in EEVEE GLSL. */
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

static GPUType socket_type_to_gpu(const eNodeSocketDatatype type)
{
  switch (type) {
    case SOCK_FLOAT:
    case SOCK_INT:
    case SOCK_BOOLEAN:
      return GPU_FLOAT;
    case SOCK_VECTOR:
      return GPU_VEC3;
    case SOCK_RGBA:
    case SOCK_ROTATION:
      return GPU_VEC4;
    case SOCK_MATRIX:
      return GPU_MAT4;
    default:
      return GPU_FLOAT;
  }
}

static const char *gpu_type_glsl_name(const GPUType type)
{
  switch (type) {
    case GPU_VEC2:
      return "float2";
    case GPU_VEC3:
      return "float3";
    case GPU_VEC4:
      return "float4";
    case GPU_MAT3:
      return "float3x3";
    case GPU_MAT4:
      return "float4x4";
    default:
      return "float";
  }
}

/* GPU material sources must not contain line or block comments. */
static std::string wrangle_strip_gpu_comments(const StringRef input)
{
  std::string out;
  out.reserve(input.size());
  enum class Mode { Normal, LineComment, BlockComment, String } mode = Mode::Normal;
  for (size_t i = 0; i < input.size(); i++) {
    const char c = input[i];
    const char n = (i + 1 < input.size()) ? input[i + 1] : '\0';
    switch (mode) {
      case Mode::Normal:
        if (c == '/' && n == '/') {
          mode = Mode::LineComment;
          i++;
        }
        else if (c == '/' && n == '*') {
          mode = Mode::BlockComment;
          i++;
        }
        else if (c == '"') {
          mode = Mode::String;
          out.push_back(c);
        }
        else {
          out.push_back(c);
        }
        break;
      case Mode::LineComment:
        if (c == '\n') {
          mode = Mode::Normal;
          out.push_back(c);
        }
        break;
      case Mode::BlockComment:
        if (c == '*' && n == '/') {
          mode = Mode::Normal;
          i++;
          out.push_back(' ');
        }
        else if (c == '\n') {
          out.push_back(c);
        }
        break;
      case Mode::String:
        out.push_back(c);
        if (c == '\\' && n != '\0') {
          out.push_back(n);
          i++;
        }
        else if (c == '"') {
          mode = Mode::Normal;
        }
        break;
    }
  }
  return out;
}

static const char *vex_type_glsl(const vex::Type type)
{
  switch (type) {
    case vex::Type::Bool:
      return "bool";
    case vex::Type::Int:
      return "int";
    case vex::Type::Vector2:
      return "float2";
    case vex::Type::Vector:
      return "float3";
    case vex::Type::Vector4:
    case vex::Type::Color:
    case vex::Type::Rotation:
      return "float4";
    case vex::Type::Matrix2:
      return "float2x2";
    case vex::Type::Matrix3:
      return "float3x3";
    case vex::Type::Matrix:
      return "float4x4";
    default:
      return "float";
  }
}

static vex::Type socket_to_vex_type(const eNodeSocketDatatype type)
{
  switch (type) {
    case SOCK_INT:
      return vex::Type::Int;
    case SOCK_BOOLEAN:
      return vex::Type::Bool;
    case SOCK_VECTOR:
      return vex::Type::Vector;
    case SOCK_RGBA:
      return vex::Type::Color;
    case SOCK_ROTATION:
      return vex::Type::Rotation;
    case SOCK_MATRIX:
      return vex::Type::Matrix;
    default:
      return vex::Type::Float;
  }
}

static std::string vex_type_default(const vex::Type type)
{
  switch (type) {
    case vex::Type::Bool:
      return "false";
    case vex::Type::Int:
      return "0";
    case vex::Type::Vector2:
      return "float2(0.0)";
    case vex::Type::Vector:
      return "float3(0.0)";
    case vex::Type::Vector4:
    case vex::Type::Color:
      return "float4(0.0)";
    case vex::Type::Rotation:
      return "float4(0.0, 0.0, 0.0, 1.0)";
    case vex::Type::Matrix2:
      return "float2x2(1.0)";
    case vex::Type::Matrix3:
      return "float3x3(1.0)";
    case vex::Type::Matrix:
      return "float4x4(1.0)";
    default:
      return "0.0";
  }
}

static std::string wrangle_glsl_float_lit(const float f)
{
  uint32_t bits = 0;
  memcpy(&bits, &f, sizeof(bits));
  char buf[64];
  snprintf(buf, sizeof(buf), "uintBitsToFloat(%uu)", bits);
  return buf;
}

static std::string wrangle_socket_default_glsl(bNodeTree &ntree,
                                              bNode &node,
                                              bNodeSocket &sock,
                                              const vex::Type type)
{
  switch (sock.type) {
    case SOCK_FLOAT: {
      return wrangle_glsl_float_lit(node_socket_get_float(&ntree, &node, &sock));
    }
    case SOCK_INT: {
      const int iv = node_socket_get_int(&ntree, &node, &sock);
      if (type == vex::Type::Int) {
        return std::to_string(iv);
      }
      return wrangle_glsl_float_lit(float(iv));
    }
    case SOCK_BOOLEAN: {
      const bool bv = node_socket_get_bool(&ntree, &node, &sock);
      if (type == vex::Type::Bool) {
        return bv ? "true" : "false";
      }
      return bv ? "1.0" : "0.0";
    }
    case SOCK_VECTOR: {
      const bNodeSocketValueVector *v = sock.default_value_typed<bNodeSocketValueVector>();
      const float x = v ? v->value[0] : 0.0f;
      const float y = v ? v->value[1] : 0.0f;
      const float z = v ? v->value[2] : 0.0f;
      if (type == vex::Type::Vector2) {
        return "float2(" + wrangle_glsl_float_lit(x) + ", " + wrangle_glsl_float_lit(y) + ")";
      }
      return "float3(" + wrangle_glsl_float_lit(x) + ", " + wrangle_glsl_float_lit(y) + ", " +
             wrangle_glsl_float_lit(z) + ")";
    }
    case SOCK_RGBA: {
      const bNodeSocketValueRGBA *v = sock.default_value_typed<bNodeSocketValueRGBA>();
      const float x = v ? v->value[0] : 0.0f;
      const float y = v ? v->value[1] : 0.0f;
      const float z = v ? v->value[2] : 0.0f;
      const float w = v ? v->value[3] : 1.0f;
      return "float4(" + wrangle_glsl_float_lit(x) + ", " + wrangle_glsl_float_lit(y) + ", " +
             wrangle_glsl_float_lit(z) + ", " + wrangle_glsl_float_lit(w) + ")";
    }
    default:
      return vex_type_default(type);
  }
}

static std::string cast_gpu_to_vex(const std::string &src,
                                   const GPUType from,
                                   const vex::Type to)
{
  if (to == vex::Type::Int) {
    return "int(" + src + ")";
  }
  if (to == vex::Type::Bool) {
    return "(" + src + " != 0.0)";
  }
  if (to == vex::Type::Float && from != GPU_FLOAT) {
    return "float(" + src + ")";
  }
  if (to == vex::Type::Vector && from == GPU_VEC4) {
    return "(" + src + ").xyz";
  }
  if (ELEM(to, vex::Type::Vector4, vex::Type::Color, vex::Type::Rotation) && from == GPU_VEC3)
  {
    return "float4(" + src + ", " + (to == vex::Type::Rotation ? "1.0" : "0.0") + ")";
  }
  (void)from;
  return src;
}

static std::string vex_attr_from_output_name(const StringRef name)
{
  return glsl_ident(name, "a_");
}

static void append_output_from_vex(std::string &body,
                                     const std::string &dst,
                                     const eNodeSocketDatatype sock_type,
                                     const std::string &src,
                                     const vex::Type src_type)
{
  body += "  ";
  body += dst;
  body += " = ";
  const GPUType to = socket_type_to_gpu(sock_type);
  if (to == GPU_FLOAT) {
    if (vex::type_is_vec_like(src_type) || src_type == vex::Type::Rotation) {
      body += "(" + src + ").x";
    }
    else {
      body += src;
    }
  }
  else if (to == GPU_VEC3) {
    if (src_type == vex::Type::Vector2) {
      body += "float3(" + src + ", 0.0)";
    }
    else if (ELEM(src_type, vex::Type::Vector4, vex::Type::Color, vex::Type::Rotation)) {
      body += "(" + src + ").xyz";
    }
    else if (src_type == vex::Type::Vector) {
      body += src;
    }
    else {
      body += "float3(" + src + ")";
    }
  }
  else if (to == GPU_VEC4) {
    if (src_type == vex::Type::Vector2) {
      body += "float4(" + src + ", 0.0, " + (sock_type == SOCK_ROTATION ? "1.0" : "1.0") + ")";
    }
    else if (src_type == vex::Type::Vector) {
      body += "float4(" + src + ", " + (sock_type == SOCK_ROTATION ? "1.0" : "1.0") + ")";
    }
    else {
      body += src;
    }
  }
  else if (to == GPU_MAT4) {
    if (src_type == vex::Type::Matrix) {
      body += src;
    }
    else {
      body += "float4x4(1.0)";
    }
  }
  else {
    body += src;
  }
  body += ";\n";
}

static void wrangle_sanitize_output_types(bNode &node)
{
  NodeShaderWrangle &storage = node_storage(node);
  for (const int i : IndexRange(storage.output_items.items_num)) {
    eNodeSocketDatatype type = eNodeSocketDatatype(storage.output_items.items[i].socket_type);
    if (!ShaderWrangleOutputItemsAccessor::supports_socket_type(type, 0)) {
      storage.output_items.items[i].socket_type = SOCK_VECTOR;
    }
  }
}

static void wrangle_fix_vector_socket_dimensions(bNode &node)
{
  auto fix = [](bNodeSocket &sock) {
    if (sock.type != SOCK_VECTOR || sock.default_value == nullptr) {
      return;
    }
    bNodeSocketValueVector *val = sock.default_value_typed<bNodeSocketValueVector>();
    if (val->dimensions < 2 || val->dimensions > 4) {
      val->dimensions = 3;
    }
  };
  for (bNodeSocket &sock : node.inputs) {
    fix(sock);
  }
  for (bNodeSocket &sock : node.outputs) {
    fix(sock);
  }
}

static void wrangle_ensure_default_output(bNode &node)
{
  NodeShaderWrangle &storage = node_storage(node);
  if (storage.output_items.items_num > 0) {
    wrangle_sanitize_output_types(node);
    return;
  }
  NodeShaderWrangleItem *items = MEM_new_array<NodeShaderWrangleItem>(1, __func__);
  items[0].name = BLI_strdup("Output");
  items[0].socket_type = SOCK_VECTOR;
  items[0].identifier = 0;
  items[0].kind = 0;
  storage.output_items.items = items;
  storage.output_items.items_num = 1;
  storage.output_items.next_identifier = 1;
}

/* -------------------------------------------------------------------- */
/** \name Channel scan / Compile
 * \{ */

struct ChannelParmRef {
  std::string name;
  eNodeSocketDatatype type = SOCK_FLOAT;
};

static bool wrangle_is_ident_char(const char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
}

static std::optional<eNodeSocketDatatype> wrangle_channel_fn_type(const StringRef ident)
{
  if (ident == "chf" || ident == "ch") {
    return SOCK_FLOAT;
  }
  if (ident == "chi") {
    return SOCK_INT;
  }
  if (ident == "chv") {
    return SOCK_VECTOR;
  }
  if (ident == "chb") {
    return SOCK_BOOLEAN;
  }
  if (ident == "chc") {
    return SOCK_RGBA;
  }
  if (ident == "chm") {
    return SOCK_MATRIX;
  }
  if (ident == "chq") {
    return SOCK_ROTATION;
  }
  return std::nullopt;
}

static Vector<ChannelParmRef> wrangle_scan_channel_parms(const StringRef src)
{
  Vector<ChannelParmRef> out;
  const int n = int(src.size());
  int i = 0;
  while (i < n) {
    if (src[i] == '/' && i + 1 < n && src[i + 1] == '/') {
      i += 2;
      while (i < n && src[i] != '\n') {
        i++;
      }
      continue;
    }
    if (src[i] == '/' && i + 1 < n && src[i + 1] == '*') {
      i += 2;
      while (i + 1 < n && !(src[i] == '*' && src[i + 1] == '/')) {
        i++;
      }
      i = std::min(i + 2, n);
      continue;
    }
    if (src[i] == '"' || src[i] == '\'') {
      const char q = src[i];
      i++;
      while (i < n && src[i] != q) {
        if (src[i] == '\\' && i + 1 < n) {
          i += 2;
        }
        else {
          i++;
        }
      }
      if (i < n) {
        i++;
      }
      continue;
    }
    if (wrangle_is_ident_char(src[i]) && (i == 0 || !wrangle_is_ident_char(src[i - 1]))) {
      const int ident_start = i;
      while (i < n && wrangle_is_ident_char(src[i])) {
        i++;
      }
      const StringRef ident = src.substr(ident_start, i - ident_start);
      const std::optional<eNodeSocketDatatype> type = wrangle_channel_fn_type(ident);
      if (!type) {
        continue;
      }
      int j = i;
      while (j < n && ELEM(src[j], ' ', '\t')) {
        j++;
      }
      if (j >= n || src[j] != '(') {
        continue;
      }
      j++;
      while (j < n && ELEM(src[j], ' ', '\t')) {
        j++;
      }
      if (j >= n || (src[j] != '"' && src[j] != '\'')) {
        continue;
      }
      const char q = src[j];
      j++;
      std::string inner;
      while (j < n && src[j] != q) {
        if (src[j] == '\\' && j + 1 < n) {
          inner += src[j + 1];
          j += 2;
        }
        else {
          inner += src[j];
          j++;
        }
      }
      if (j >= n || inner.empty()) {
        continue;
      }
      i = j + 1;
      bool seen = false;
      for (const ChannelParmRef &prev : out) {
        if (prev.name == inner) {
          seen = true;
          break;
        }
      }
      if (!seen) {
        out.append({std::move(inner), *type});
      }
      continue;
    }
    i++;
  }
  return out;
}

static void wrangle_commit_run_code(NodeShaderWrangle &storage)
{
  const char *code = storage.code ? storage.code : "";
  MEM_SAFE_DELETE(storage.code_run);
  storage.code_run = BLI_strdup(code);
}

static wmOperatorStatus wrangle_compile_exec(bContext *C, wmOperator *op)
{
  PointerRNA node_ptr = socket_items::ops::get_active_node_to_operate_on(
      C, op, ShaderWrangleInputItemsAccessor::node_idname);
  if (!node_ptr) {
    return OPERATOR_CANCELLED;
  }
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(node_ptr.owner_id);
  bNode &node = *static_cast<bNode *>(node_ptr.data);
  NodeShaderWrangle &storage = node_storage(node);
  wrangle_commit_run_code(storage);
  const char *code = storage.code ? storage.code : "";
  const Vector<ChannelParmRef> wanted = wrangle_scan_channel_parms(code);

  socket_items::SocketItemsRef ref = ShaderWrangleInputItemsAccessor::get_items_from_node(node);
  if (*ref.items_num > 0) {
    dna::array::remove_if<NodeShaderWrangleItem>(
        ref.items,
        ref.items_num,
        [&](const NodeShaderWrangleItem &item) -> bool {
          if (shader_wrangle_item_kind(item) != SH_NODE_WRANGLE_ITEM_CONSTANT) {
            return false;
          }
          if (item.name == nullptr || item.name[0] == '\0') {
            return true;
          }
          for (const ChannelParmRef &parm : wanted) {
            if (parm.name == item.name) {
              return false;
            }
          }
          return true;
        },
        ShaderWrangleInputItemsAccessor::destruct_item);
  }
  if (ref.active_index) {
    *ref.active_index = std::clamp(*ref.active_index, 0, std::max(*ref.items_num - 1, 0));
  }

  for (const ChannelParmRef &parm : wanted) {
    socket_items::SocketItemsRef live = ShaderWrangleInputItemsAccessor::get_items_from_node(node);
    NodeShaderWrangleItem *existing = nullptr;
    bool name_taken = false;
    for (const int i : IndexRange(*live.items_num)) {
      NodeShaderWrangleItem &item = (*live.items)[i];
      if (!(item.name && parm.name == item.name)) {
        continue;
      }
      if (shader_wrangle_item_kind(item) == SH_NODE_WRANGLE_ITEM_CONSTANT) {
        existing = &item;
      }
      else {
        name_taken = true;
      }
      break;
    }
    if (existing) {
      existing->socket_type = parm.type;
      existing->kind = SH_NODE_WRANGLE_ITEM_CONSTANT;
      continue;
    }
    if (name_taken) {
      continue;
    }
    NodeShaderWrangleItem *item =
        socket_items::add_item_with_socket_type_and_name<ShaderWrangleInputItemsAccessor>(
            *ntree, node, parm.type, parm.name.c_str());
    if (item) {
      MEM_delete(item->name);
      item->name = BLI_strdup(parm.name.c_str());
      item->kind = SH_NODE_WRANGLE_ITEM_CONSTANT;
    }
  }

  nodes::remove_unlinked_shader_wrangle_field_items(*ntree, node);

  socket_items::ops::update_after_node_change(C, node_ptr);
  return OPERATOR_FINISHED;
}

/** \} */

static void wrangle_draw_message(ui::Layout &layout, const StringRef text, const int icon, const bool red)
{
  if (red) {
    layout.red_alert_set(true);
  }
  const char *p = text.data();
  const char *end = p + text.size();
  while (p < end) {
    const char *nl = static_cast<const char *>(memchr(p, '\n', size_t(end - p)));
    const StringRef line = nl ? StringRef(p, nl) : StringRef(p, end);
    if (!line.is_empty()) {
      layout.label(line, icon);
    }
    if (!nl) {
      break;
    }
    p = nl + 1;
  }
  if (red) {
    layout.red_alert_set(false);
  }
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  bNode &node = *static_cast<bNode *>(ptr->data);
  NodeShaderWrangle &storage = node_storage(node);

  layout.use_property_split_set(false);
  layout.alignment_set(ui::LayoutAlign::Expand);

  ui::Layout &compile_row = layout.row(true);
  compile_row.alignment_set(ui::LayoutAlign::Right);
  PointerRNA edit_ptr = compile_row.op("NODE_OT_shader_wrangle_edit", IFACE_("Edit"), ICON_TEXT);
  RNA_int_set(&edit_ptr, "node_identifier", node.identifier);
  PointerRNA op_ptr = compile_row.op(
      "NODE_OT_shader_wrangle_compile", IFACE_("Compile"), ICON_FILE_REFRESH);
  RNA_int_set(&op_ptr, "node_identifier", node.identifier);

  ui::Block *block = layout.block();
  const int64_t buttons_before = block ? block->buttons_ptrs.size() : 0;
  layout.textbox_with_state(ptr, "code", &storage.textbox_state, std::nullopt);
  wrangle_apply_code_editor_flags(block, buttons_before);
  const char *code = storage.code ? storage.code : "";
  if (code[0] != '\0') {
    const vex::CompileOutput compiled = vex::compile_shader_material(code);
    if (!compiled.program && !compiled.error.empty()) {
      wrangle_draw_message(layout, compiled.error, ICON_ERROR, true);
    }
    else if (!compiled.warning.empty()) {
      wrangle_draw_message(layout, compiled.warning, ICON_INFO, false);
    }
  }
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_default_layout();

  auto &constant_panel = b.add_panel("Constant"_ustr)
                             .description("Single values from chi/chf/chv/... created by Compile");
  auto &field_panel = b.add_panel("Field"_ustr).description(
      "Inputs captured as named attributes; read in VEX with i@/f@/v@ using the socket name");

  const bNodeTree *tree = b.tree_or_null();
  if (bNode *node_mut = const_cast<bNode *>(b.node_or_null())) {
    if (node_mut->storage) {
      wrangle_sanitize_output_types(*node_mut);
      wrangle_fix_vector_socket_dimensions(*node_mut);
    }
  }
  const bNode *node = b.node_or_null();
  if (node && tree && node->storage) {
    const NodeShaderWrangle &storage = node_storage(*node);

    for (const int i : IndexRange(storage.output_items.items_num)) {
      const NodeShaderWrangleItem &item = storage.output_items.items[i];
      const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
      const UString identifier{ShaderWrangleOutputItemsAccessor::socket_identifier_for_item(item)};
      const UString name{item.name ? item.name : ""};
      b.add_output(socket_type, name, identifier)
          .description("Assign this name in VEX (`Output = set(p, 0)`). Float/Vector/Color")
          .socket_name_ptr(&tree->id, *ShaderWrangleOutputItemsAccessor::item_srna, &item, "name")
          .custom_draw([i](CustomSocketDrawParams &params) {
            socket_items::ui::draw_item_socket_with_remove<ShaderWrangleOutputItemsAccessor>(
                params, i);
          });
    }
  }
  b.add_output<decl::Extend>(""_ustr, "__extend__wrangle_out"_ustr)
      .custom_draw(socket_items::ui::draw_extend_socket_fn<ShaderWrangleOutputItemsAccessor>());

  if (node && tree && node->storage) {
    const NodeShaderWrangle &storage = node_storage(*node);

    for (const int i : IndexRange(storage.input_items.items_num)) {
      const NodeShaderWrangleItem &item = storage.input_items.items[i];
      if (shader_wrangle_item_kind(item) != SH_NODE_WRANGLE_ITEM_CONSTANT) {
        continue;
      }
      const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
      const UString identifier{ShaderWrangleInputItemsAccessor::socket_identifier_for_item(item)};
      const UString name{item.name ? item.name : ""};
      constant_panel.add_input(socket_type, name, identifier)
          .socket_name_ptr(&tree->id, *ShaderWrangleInputItemsAccessor::item_srna, &item, "name")
          .structure_type(StructureType::Single);
    }

    for (const int i : IndexRange(storage.input_items.items_num)) {
      const NodeShaderWrangleItem &item = storage.input_items.items[i];
      if (shader_wrangle_item_kind(item) != SH_NODE_WRANGLE_ITEM_FIELD) {
        continue;
      }
      const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
      const UString identifier{ShaderWrangleFieldItemsAccessor::socket_identifier_for_item(item)};
      const UString name{item.name ? item.name : ""};
      field_panel.add_input(socket_type, name, identifier)
          .socket_name_ptr(&tree->id, *ShaderWrangleFieldItemsAccessor::item_srna, &item, "name")
          .custom_draw([i](CustomSocketDrawParams &params) {
            socket_items::ui::draw_item_socket_with_remove<ShaderWrangleFieldItemsAccessor>(params, i);
          });
    }
  }
  field_panel.add_input<decl::Extend>(""_ustr, "__extend__wrangle_field"_ustr);
}

static void node_init(bNodeTree *tree, bNode *node)
{
  NodeShaderWrangle *storage = MEM_new<NodeShaderWrangle>(__func__);
  storage->textbox_state.visible_lines = 14;
  storage->code = BLI_strdup(default_wrangle_code());
  storage->code_run = BLI_strdup(default_wrangle_code());
  node->storage = storage;
  node->width = 300.0f;
  if (tree) {
    socket_items::add_item_with_socket_type_and_name<ShaderWrangleOutputItemsAccessor>(
        *tree, *node, SOCK_VECTOR, "Output");
  }
  else {
    wrangle_ensure_default_output(*node);
  }
}

static void node_free_storage(bNode *node)
{
  NodeShaderWrangle *storage = static_cast<NodeShaderWrangle *>(node->storage);
  if (storage == nullptr) {
    return;
  }
  socket_items::destruct_array<ShaderWrangleInputItemsAccessor>(*node);
  socket_items::destruct_array<ShaderWrangleOutputItemsAccessor>(*node);
  MEM_SAFE_DELETE(storage->code);
  MEM_SAFE_DELETE(storage->code_run);
  MEM_delete(storage);
  node->storage = nullptr;
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeShaderWrangle &src_storage = node_storage(*src_node);
  NodeShaderWrangle *dst_storage = MEM_new<NodeShaderWrangle>(__func__,
                                                            dna::shallow_copy(src_storage));
  dst_node->storage = dst_storage;
  if (src_storage.code) {
    dst_storage->code = BLI_strdup(src_storage.code);
  }
  if (src_storage.code_run) {
    dst_storage->code_run = BLI_strdup(src_storage.code_run);
  }
  socket_items::copy_array<ShaderWrangleInputItemsAccessor>(*src_node, *dst_node);
  socket_items::copy_array<ShaderWrangleOutputItemsAccessor>(*src_node, *dst_node);
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  const NodeShaderWrangle &storage = node_storage(node);
  writer.write_string(storage.code);
  writer.write_string(storage.code_run);
  socket_items::blend_write<ShaderWrangleInputItemsAccessor>(&writer, node);
  socket_items::blend_write<ShaderWrangleOutputItemsAccessor>(&writer, node);
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  NodeShaderWrangle &storage = node_storage(node);
  BLO_read_string(&reader, &storage.code);
  BLO_read_string(&reader, &storage.code_run);
  if (storage.code_run == nullptr && storage.code) {
    storage.code_run = BLI_strdup(storage.code);
  }
  socket_items::blend_read_data<ShaderWrangleInputItemsAccessor>(&reader, node);
  socket_items::blend_read_data<ShaderWrangleOutputItemsAccessor>(&reader, node);
  wrangle_ensure_default_output(node);
}

static void node_extra_info(NodeExtraInfoParams &parameters)
{
  const NodeShaderWrangle &storage = node_storage(parameters.node);
  const char *code = storage.code ? storage.code : "";
  if (code[0] != '\0') {
    const vex::CompileOutput compiled = vex::compile_shader_material(code);
    if (!compiled.program && !compiled.error.empty()) {
      NodeExtraInfoRow row;
      const size_t nl = compiled.error.find('\n');
      row.text = (nl == std::string::npos) ? compiled.error : compiled.error.substr(0, nl);
      row.icon = ICON_ERROR;
      parameters.rows.append(std::move(row));
    }
    else if (compiled.program && !compiled.program->gpu_ok) {
      NodeExtraInfoRow row;
      if (!compiled.program->gpu_error.empty()) {
        const std::string &err = compiled.program->gpu_error;
        const size_t nl = err.find('\n');
        row.text = (nl == std::string::npos) ? err : err.substr(0, nl);
      }
      else {
        row.text = IFACE_("GPU compile failed");
      }
      row.tooltip = TIP_("This VEX cannot be emitted as EEVEE GLSL");
      row.icon = ICON_ERROR;
      parameters.rows.append(std::move(row));
    }
  }
  const Scene *scene = CTX_data_scene(&parameters.C);
  if (scene && BKE_scene_uses_blender_eevee(scene)) {
    return;
  }
  NodeExtraInfoRow row;
  row.text = IFACE_("EEVEE Only");
  row.tooltip = TIP_(
      "VEX compiles to EEVEE GLSL. Cycles can evaluate a constant Output as a color");
  row.icon = ICON_STATUS_INFO;
  parameters.rows.append(std::move(row));
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  if (ntree == nullptr || node == nullptr || node->storage == nullptr) {
    return;
  }
  const int before = node_storage(*node).input_items.items_num;
  nodes::remove_unlinked_shader_wrangle_field_items(*ntree, *node);
  if (node_storage(*node).input_items.items_num != before) {
    nodes::update_node_declaration_and_sockets(*ntree, *node);
  }
}

static bool node_insert_link(bke::NodeInsertLinkParams &params)
{
  bNodeSocket *sock = nullptr;
  if (params.link.tonode == &params.node) {
    sock = params.link.tosock;
  }
  else if (params.link.fromnode == &params.node) {
    sock = params.link.fromsock;
  }
  if (sock == nullptr || !STREQ(sock->idname, "NodeSocketVirtual")) {
    return true;
  }
  if (StringRef(sock->identifier) == "__extend__wrangle_out") {
    const bool ok = socket_items::try_add_item_via_any_extend_socket<
        ShaderWrangleOutputItemsAccessor>(
        params.ntree, params.node, params.node, params.link, "__extend__wrangle_out");
    if (!ok && params.C) {
      BKE_report(CTX_wm_reports(params.C),
                 RPT_ERROR,
                 "Wrangle output accepts float, int, boolean, vector, or color");
    }
    return ok;
  }
  if (StringRef(sock->identifier) == "__extend__wrangle_field") {
    const bool ok = socket_items::try_add_item_via_any_extend_socket<ShaderWrangleFieldItemsAccessor>(
        params.ntree, params.node, params.node, params.link, "__extend__wrangle_field");
    if (!ok && params.C) {
      BKE_report(CTX_wm_reports(params.C),
                 RPT_ERROR,
                 "Wrangle field input only accepts float, int, vector, color, rotation, or matrix");
    }
    return ok;
  }
  return false;
}

static const char *wrangle_live_code(const NodeShaderWrangle &storage)
{
  /* Draft is `code`. GPU / Cycles cook `code_run` until Ctrl+Enter, click-outside, or Compile. */
  if (storage.code_run && storage.code_run[0] != '\0') {
    return storage.code_run;
  }
  if (storage.code && storage.code[0] != '\0') {
    return storage.code;
  }
  return "";
}

static const char *wrangle_intern_gpu_name(const char *name)
{
  /* GPUNode stores this pointer and uses it after gpu_fn returns. Stack buffers dangle. */
  static Map<std::string, const char *> intern;
  const std::string key(name);
  if (const char **existing = intern.lookup_ptr(key)) {
    return *existing;
  }
  const char *copy = BLI_strdup(name);
  intern.add(name, copy);
  return copy;
}

static uint64_t hash_wrangle_signature(const NodeShaderWrangle &storage, const bNode & /*node*/)
{
  uint64_t h = get_default_hash(StringRef(wrangle_live_code(storage)));
  for (const int i : IndexRange(storage.input_items.items_num)) {
    const NodeShaderWrangleItem &item = storage.input_items.items[i];
    h = h * 33 + get_default_hash(StringRef(item.name ? item.name : ""));
    h = h * 33 + uint64_t(item.socket_type);
    h = h * 33 + uint64_t(item.kind);
    h = h * 33 + uint64_t(item.identifier);
  }
  for (const int i : IndexRange(storage.output_items.items_num)) {
    const NodeShaderWrangleItem &item = storage.output_items.items[i];
    h = h * 33 + get_default_hash(StringRef(item.name ? item.name : ""));
    h = h * 33 + uint64_t(item.socket_type);
    h = h * 33 + uint64_t(item.identifier);
  }
  return h;
}

static void wrangle_fix_gpu_stack_none(GPUNodeStack *stack)
{
  if (stack == nullptr) {
    return;
  }
  for (int i = 0; !stack[i].end; i++) {
    if (stack[i].type != GPU_NONE) {
      continue;
    }
    switch (stack[i].sockettype) {
      case SOCK_FLOAT:
      case SOCK_INT:
      case SOCK_BOOLEAN:
        stack[i].type = GPU_FLOAT;
        break;
      case SOCK_VECTOR:
        stack[i].type = GPU_VEC3;
        break;
      case SOCK_RGBA:
      case SOCK_ROTATION:
        stack[i].type = GPU_VEC4;
        break;
      default:
        break;
    }
  }
}

static const NodeShaderWrangleItem *wrangle_find_item_for_socket(const NodeShaderWrangle &storage,
                                                                  const bNodeSocket &sock,
                                                                  const bool is_output)
{
  if (is_output) {
    for (const int i : IndexRange(storage.output_items.items_num)) {
      const NodeShaderWrangleItem &item = storage.output_items.items[i];
      if (StringRef(sock.identifier) ==
          ShaderWrangleOutputItemsAccessor::socket_identifier_for_item(item))
      {
        return &item;
      }
    }
    return nullptr;
  }
  for (const int i : IndexRange(storage.input_items.items_num)) {
    const NodeShaderWrangleItem &item = storage.input_items.items[i];
    const std::string ident = (shader_wrangle_item_kind(item) == SH_NODE_WRANGLE_ITEM_FIELD) ?
                                  ShaderWrangleFieldItemsAccessor::socket_identifier_for_item(item) :
                                  ShaderWrangleInputItemsAccessor::socket_identifier_for_item(item);
    if (StringRef(sock.identifier) == ident) {
      return &item;
    }
  }
  return nullptr;
}

static void wrangle_fill_gpu_stack_from_sockets(GPUNodeStack *stack,
                                                bNode &node,
                                                const bool is_output)
{
  if (stack == nullptr) {
    return;
  }
  int i = 0;
  for (bNodeSocket &sock : (is_output ? node.outputs : node.inputs)) {
    if (stack[i].end) {
      break;
    }
    if (stack[i].type == GPU_NONE && sock.type != SOCK_CUSTOM && sock.type != SOCK_SHADER) {
      stack[i].type = socket_type_to_gpu(eNodeSocketDatatype(sock.type));
    }
    if (std::holds_alternative<std::monostate>(stack[i].value)) {
      bNodeTree &ntree = node.owner_tree();
      switch (sock.type) {
        case SOCK_FLOAT:
          stack[i].value = node_socket_get_float(&ntree, &node, &sock);
          break;
        case SOCK_INT:
          stack[i].value = float(node_socket_get_int(&ntree, &node, &sock));
          break;
        case SOCK_BOOLEAN:
          stack[i].value = node_socket_get_bool(&ntree, &node, &sock) ? 1.0f : 0.0f;
          break;
        case SOCK_VECTOR: {
          float vec[3] = {0.0f, 0.0f, 0.0f};
          node_socket_get_vector(&ntree, &node, &sock, vec);
          stack[i].value = float3(vec[0], vec[1], vec[2]);
          break;
        }
        case SOCK_RGBA: {
          float col[4] = {0.0f, 0.0f, 0.0f, 1.0f};
          node_socket_get_color(&ntree, &node, &sock, col);
          stack[i].value = float4(col[0], col[1], col[2], col[3]);
          break;
        }
        default:
          break;
      }
    }
    i++;
  }
}

static const bNodeSocket *wrangle_find_input_socket_named(const bNode &node,
                                                            const NodeShaderWrangle &storage,
                                                            const StringRef name)
{
  for (const bNodeSocket &sock : node.inputs) {
    const NodeShaderWrangleItem *item = wrangle_find_item_for_socket(storage, sock, false);
    if (item && item->name && name == StringRef(item->name)) {
      return &sock;
    }
  }
  for (const bNodeSocket &sock : node.inputs) {
    if (name == sock.name) {
      return &sock;
    }
  }
  return nullptr;
}

static int node_shader_gpu_wrangle(GPUMaterial *mat,
                                    bNode *node,
                                    bNodeExecData * /*execdata*/,
                                    GPUNodeStack *in,
                                    GPUNodeStack *out)
{
  wrangle_fix_vector_socket_dimensions(*node);
  wrangle_fix_gpu_stack_none(in);
  wrangle_fix_gpu_stack_none(out);
  wrangle_fill_gpu_stack_from_sockets(in, *node, false);
  wrangle_fill_gpu_stack_from_sockets(out, *node, true);

  const NodeShaderWrangle &storage = node_storage(*node);
  const char *code = wrangle_live_code(storage);
  const vex::CompileOutput compiled = vex::compile_shader_material(code);

  const uint64_t content_hash = hash_wrangle_signature(storage, *node);
  char func_name[64];
  SNPRINTF(func_name, "node_wrangle_%08x", uint32_t(content_hash));

  bool need_wp = false;
  bool need_wn = false;
  if (compiled.program) {
    for (const vex::AttrInfo &a : compiled.program->attrs) {
      if (a.name == "position") {
        need_wp = true;
      }
      else if (a.name == "normal") {
        need_wn = true;
      }
    }
  }

  Vector<GPUType> param_types;
  Vector<int> param_quals;
  std::string sig = "void ";
  sig += func_name;
  sig += "(";
  bool first = true;
  auto append_in = [&](const GPUType type, const std::string &ident) {
    if (!first) {
      sig += ", ";
    }
    first = false;
    sig += gpu_type_glsl_name(type);
    sig += " ";
    sig += ident;
    param_types.append(type);
    param_quals.append(0);
  };
  auto append_out = [&](const GPUType type, const std::string &ident) {
    if (!first) {
      sig += ", ";
    }
    first = false;
    sig += "_ref(";
    sig += gpu_type_glsl_name(type);
    sig += " ,";
    sig += ident;
    sig += ")";
    param_types.append(type);
    param_quals.append(1);
  };

  struct InParam {
    GPUType gpu_type = GPU_FLOAT;
    vex::Type vex_type = vex::Type::Float;
    std::string ident;
    std::string item_name;
    int8_t kind = 0;
  };
  Vector<InParam> in_params;
  /* Match GPU_stack_link: only non-NONE stack sockets, in node socket order. */
  if (in) {
    int si = 0;
    for (bNodeSocket &sock : node->inputs) {
      if (in[si].end) {
        break;
      }
      if (in[si].type != GPU_NONE) {
        const NodeShaderWrangleItem *item = wrangle_find_item_for_socket(storage, sock, false);
        InParam p;
        p.gpu_type = in[si].type;
        p.ident = "in_" + std::to_string(si);
        if (item) {
          p.vex_type = socket_to_vex_type(eNodeSocketDatatype(item->socket_type));
          p.ident = "in_" + std::to_string(item->identifier);
          p.item_name = item->name ? item->name : "";
          p.kind = item->kind;
        }
        else {
          p.vex_type = socket_to_vex_type(eNodeSocketDatatype(sock.type));
          p.item_name = sock.name;
        }
        append_in(p.gpu_type, p.ident);
        in_params.append(std::move(p));
      }
      si++;
    }
  }
  /* Extra GPU_link inputs after sockets so GPU_stack_link matches totin then va_arg. */
  if (need_wp) {
    append_in(GPU_VEC3, "_wp");
  }
  if (need_wn) {
    append_in(GPU_VEC3, "_wn");
  }
  /* Screen-space ddx/ddy and any future shading lookup. Not a socket. */
  if (!first) {
    sig += ", ";
  }
  first = false;
  sig += "KernelGlobals kg";
  param_types.append(GPU_KERNEL_GLOBALS);
  param_quals.append(0);

  struct OutParam {
    GPUType gpu_type = GPU_VEC3;
    eNodeSocketDatatype sock_type = SOCK_VECTOR;
    std::string ident;
    std::string item_name = "Output";
  };
  Vector<OutParam> out_params;
  if (out) {
    int si = 0;
    for (bNodeSocket &sock : node->outputs) {
      if (out[si].end) {
        break;
      }
      if (out[si].type != GPU_NONE) {
        const NodeShaderWrangleItem *item = wrangle_find_item_for_socket(storage, sock, true);
        OutParam p;
        p.gpu_type = out[si].type;
        p.sock_type = eNodeSocketDatatype(sock.type);
        p.ident = "out_" + std::to_string(si);
        p.item_name = sock.name;
        if (item) {
          p.ident = "out_" + std::to_string(item->identifier);
          p.item_name = item->name ? item->name : "Output";
          p.sock_type = eNodeSocketDatatype(item->socket_type);
        }
        append_out(p.gpu_type, p.ident);
        out_params.append(std::move(p));
      }
      si++;
    }
  }
  sig += ")\n{\n";

  std::string body;
  bool need_qmul = false;
  bool need_qrot = false;
  std::string gpu_body;
  if (compiled.program && compiled.program->gpu_ok && !compiled.program->gpu_src.empty()) {
    gpu_body = wrangle_strip_gpu_comments(compiled.program->gpu_src);
    need_qmul = gpu_body.find("wr_qmul") != std::string::npos;
    need_qrot = gpu_body.find("wr_qrot") != std::string::npos;
  }
  std::string helpers;
  if (compiled.program && compiled.program->gpu_ok && !compiled.program->gpu_helpers.empty()) {
    helpers = wrangle_strip_gpu_comments(compiled.program->gpu_helpers);
    if (!helpers.empty() && helpers.back() != '\n') {
      helpers += '\n';
    }
    need_qmul = need_qmul || helpers.find("wr_qmul") != std::string::npos;
    need_qrot = need_qrot || helpers.find("wr_qrot") != std::string::npos;
    if (helpers.find("wr_qmul") != std::string::npos) {
      helpers =
          "#define wr_qmul(a,b) float4((a).w*(b).x+(a).x*(b).w+(a).y*(b).z-(a).z*(b).y,"
          "(a).w*(b).y+(a).y*(b).w+(a).z*(b).x-(a).x*(b).z,"
          "(a).w*(b).z+(a).z*(b).w+(a).x*(b).y-(a).y*(b).x,"
          "(a).w*(b).w-(a).x*(b).x-(a).y*(b).y-(a).z*(b).z)\n" +
          helpers + "#undef wr_qmul\n";
    }
    if (helpers.find("wr_qrot") != std::string::npos) {
      helpers =
          "#define wr_qrot(q,v) "
          "((v)+((q).w*(2.0*cross((q).xyz,(v))))+cross((q).xyz,(2.0*cross((q).xyz,(v)))))\n" +
          helpers + "#undef wr_qrot\n";
    }
  }
  if (gpu_body.empty()) {
    float rgb[3] = {0.0f, 0.0f, 0.0f};
    if (ntreeShaderWrangleEvalConstantRGB(code, rgb)) {
      char assign[160];
      SNPRINTF(assign,
               "  a_Output = float3(%ff, %ff, %ff);\n",
               rgb[0],
               rgb[1],
               rgb[2]);
      gpu_body = assign;
    }
  }
  if (need_qmul) {
    body +=
        "#define wr_qmul(a,b) float4((a).w*(b).x+(a).x*(b).w+(a).y*(b).z-(a).z*(b).y,"
        "(a).w*(b).y+(a).y*(b).w+(a).z*(b).x-(a).x*(b).z,"
        "(a).w*(b).z+(a).z*(b).w+(a).x*(b).y-(a).y*(b).x,"
        "(a).w*(b).w-(a).x*(b).x-(a).y*(b).y-(a).z*(b).z)\n";
  }
  if (need_qrot) {
    body +=
        "#define wr_qrot(q,v) "
        "((v)+((q).w*(2.0*cross((q).xyz,(v))))+cross((q).xyz,(2.0*cross((q).xyz,(v)))))\n";
  }

  for (const OutParam &op : out_params) {
    body += "  ";
    body += op.ident;
    body += " = ";
    body += gpu_type_glsl_name(op.gpu_type);
    body += "(0.0);\n";
  }

  Set<std::string> declared;
  auto declare = [&](const std::string &ident, const vex::Type type, const std::string &init) {
    if (!declared.add(ident)) {
      return;
    }
    body += "  ";
    body += vex_type_glsl(type);
    body += " ";
    body += ident;
    body += " = ";
    body += init;
    body += ";\n";
  };

  for (const InParam &p : in_params) {
    if (p.item_name.empty()) {
      continue;
    }
    const std::string init = cast_gpu_to_vex(p.ident, p.gpu_type, p.vex_type);
    if (p.kind == SH_NODE_WRANGLE_ITEM_FIELD) {
      declare(glsl_ident(field_attr_name(p.item_name), "a_"), p.vex_type, init);
    }
    declare(glsl_ident(p.item_name, "ch_"), p.vex_type, init);
  }

  bool wrote_position = false;
  if (compiled.program) {
    for (const vex::AttrInfo &a : compiled.program->attrs) {
      const std::string ident = glsl_ident(a.name, "a_");
      std::string init = vex_type_default(a.type);
      if (a.name == "position") {
        init = cast_gpu_to_vex("_wp", GPU_VEC3, a.type);
        wrote_position = a.write;
      }
      else if (a.name == "normal") {
        init = cast_gpu_to_vex("_wn", GPU_VEC3, a.type);
      }
      declare(ident, a.type, init);
    }
    for (const vex::Program::GpuCh &ch : compiled.program->gpu_ch) {
      const std::string ident = glsl_ident(ch.name, "ch_");
      std::string init = vex_type_default(ch.type);
      bool bound = false;
      for (const InParam &p : in_params) {
        if (p.item_name == ch.name) {
          init = cast_gpu_to_vex(p.ident, p.gpu_type, ch.type);
          bound = true;
          break;
        }
      }
      if (!bound) {
        if (bNodeSocket *sock = const_cast<bNodeSocket *>(
                wrangle_find_input_socket_named(*node, storage, ch.name)))
        {
          init = wrangle_socket_default_glsl(node->owner_tree(), *node, *sock, ch.type);
        }
      }
      declare(ident, ch.type, init);
    }
  }
  for (const OutParam &op : out_params) {
    declare(vex_attr_from_output_name(op.item_name),
            socket_to_vex_type(op.sock_type),
            vex_type_default(socket_to_vex_type(op.sock_type)));
  }
  declare("a_Output", vex::Type::Vector, "float3(0.0)");

  if (!gpu_body.empty()) {
    body += gpu_body;
    if (gpu_body.back() != '\n') {
      body += "\n";
    }
  }

  auto vex_src_type = [&](const StringRef name) -> vex::Type {
    if (compiled.program) {
      for (const vex::AttrInfo &a : compiled.program->attrs) {
        if (a.name == name) {
          return a.type;
        }
      }
    }
    return vex::Type::Vector;
  };
  Set<std::string> written_attrs;
  if (compiled.program) {
    for (const vex::AttrInfo &a : compiled.program->attrs) {
      if (a.write) {
        written_attrs.add(a.name);
      }
    }
  }
  for (const OutParam &op : out_params) {
    const std::string nm = op.item_name.empty() ? "Output" : op.item_name;
    std::string src = vex_attr_from_output_name(nm);
    vex::Type src_type = socket_to_vex_type(op.sock_type);
    if (written_attrs.contains(nm)) {
      src_type = vex_src_type(nm);
    }
    else if ((nm == "Output" || nm == "output") && wrote_position) {
      src = "a_position";
      src_type = vex::Type::Vector;
    }
    append_output_from_vex(body, op.ident, op.sock_type, src, src_type);
  }
  if (out_params.is_empty()) {
    /* No GPU outputs: keep signature matched to the stack (do not invent out params). */
  }
  if (need_qmul) {
    body += "#undef wr_qmul\n";
  }
  if (need_qrot) {
    body += "#undef wr_qrot\n";
  }

  std::string glsl = wrangle_strip_gpu_comments(
      std::string("#undef mix\n#undef lerp\n") + helpers +
      "#define mix wr_interp\n#define lerp wr_interp\n" + sig + body +
      "}\n#undef mix\n#undef lerp\n");

  if (param_types.is_empty()) {
    return 0;
  }

  const char *stable_name = wrangle_intern_gpu_name(func_name);

  if (!GPU_material_library_ensure_function(func_name,
                                              glsl.c_str(),
                                              param_types.data(),
                                              param_quals.data(),
                                              param_types.size()))
  {
    /* Keep a GPU link so EEVEE does not fall back to the magenta error shader. */
    for (int i = 0; out && !out[i].end; i++) {
      if (out[i].type == GPU_VEC3) {
        GPU_link(mat, "set_rgb_zero", &out[i].link);
      }
      else if (out[i].type == GPU_FLOAT) {
        GPU_link(mat, "set_value_zero", &out[i].link);
      }
      else if (out[i].type == GPU_VEC4) {
        GPU_link(mat, "set_rgba_zero", &out[i].link);
      }
    }
    return 1;
  }

  GPUNodeLink *wp = nullptr;
  GPUNodeLink *wn = nullptr;
  if (need_wp) {
    GPU_link(mat, "world_position_get", GPU_shading_data(), &wp);
  }
  if (need_wn) {
    GPU_link(mat, "world_normals_get", GPU_shading_data(), &wn);
  }
  GPUNodeLink *kg = GPU_kernel_globals();
  if (need_wp && need_wn) {
    return GPU_stack_link(mat, node, stable_name, in, out, wp, wn, kg) ? 1 : 0;
  }
  if (need_wp) {
    return GPU_stack_link(mat, node, stable_name, in, out, wp, kg) ? 1 : 0;
  }
  if (need_wn) {
    return GPU_stack_link(mat, node, stable_name, in, out, wn, kg) ? 1 : 0;
  }
  return GPU_stack_link(mat, node, stable_name, in, out, kg) ? 1 : 0;
}

struct WrangleEditPopupArg {
  bNode *node = nullptr;
  bNodeTree *ntree = nullptr;
  int32_t node_identifier = 0;
  TextboxState editor_state;
  char status[256] = {};
  bool pinned = false;
  bool activate_editor = true;
  float font_pt = 0.0f;
};

static bNode *wrangle_live_edit_node(WrangleEditPopupArg *arg)
{
  if (!arg || !arg->ntree || arg->node_identifier <= 0) {
    return nullptr;
  }
  /* Node delete frees storage; the popup keeps `arg->node` across redraws. */
  bNode *node = arg->ntree->node_by_id(arg->node_identifier);
  arg->node = node;
  return node;
}

static ui::Block *wrangle_edit_popup(bContext *C, ARegion *region, void *arg_v)
{
  WrangleEditPopupArg *arg = static_cast<WrangleEditPopupArg *>(arg_v);
  if (!arg || !arg->ntree) {
    return nullptr;
  }
  bNode *live_node = wrangle_live_edit_node(arg);
  if (!live_node || !live_node->storage) {
    ui::Block *gone = ui::block_begin(
        C, region, "shader_wrangle_code_editor", ui::EmbossType::Emboss);
    ui::block_flag_enable(gone, ui::BLOCK_KEEP_OPEN | ui::BLOCK_LOOP);
    const uiStyle *style = ui::style_get_dpi();
    ui::Layout &layout = ui::block_layout(gone,
                                          ui::LayoutDirection::Vertical,
                                          ui::LayoutType::Panel,
                                          0,
                                          0,
                                          int(20 * UI_UNIT_X),
                                          int(2 * UI_UNIT_Y),
                                          4,
                                          style);
    layout.label(IFACE_("Wrangle node was deleted"), ICON_ERROR);
    const int gone_bounds[2] = {0, 0};
    ui::block_bounds_set_popup(gone, int(0.3f * U.widget_unit), gone_bounds);
    return gone;
  }
  NodeShaderWrangle &storage = node_storage(*live_node);
  if (arg->editor_state.visible_lines < 20) {
    arg->editor_state.visible_lines = 26;
  }
  if (arg->editor_state.font_scale > 0.05f) {
    storage.textbox_state.font_scale = arg->editor_state.font_scale;
  }
  else if (storage.textbox_state.font_scale > 0.05f) {
    arg->editor_state.font_scale = storage.textbox_state.font_scale;
  }
  if (arg->editor_state.width > 0) {
    storage.textbox_state.width = arg->editor_state.width;
  }
  else if (storage.textbox_state.width > 0) {
    arg->editor_state.width = storage.textbox_state.width;
  }

  const uiStyle *style = ui::style_get_dpi();
  ui::Block *block = ui::block_begin(C, region, "shader_wrangle_code_editor", ui::EmbossType::Emboss);
  ui::block_flag_enable(block,
                        ui::BLOCK_KEEP_OPEN | ui::BLOCK_LOOP | ui::BLOCK_NO_ACCELERATOR_KEYS);
  if (arg->pinned) {
    ui::block_flag_enable(block, ui::BLOCK_PINNED);
  }
  ui::block_theme_style_set(block, ui::BLOCK_THEME_STYLE_REGULAR);

  const int layout_w = std::clamp(arg->editor_state.width > 0 ? arg->editor_state.width :
                                                                int(50 * UI_UNIT_X),
                                  int(22 * UI_UNIT_X),
                                  int(140 * UI_UNIT_X));
  ui::Layout &layout = ui::block_layout(block,
                                        ui::LayoutDirection::Vertical,
                                        ui::LayoutType::Panel,
                                        0,
                                        0,
                                        layout_w,
                                        int(2 * UI_UNIT_Y),
                                        4,
                                        style);

  ui::Layout &header = layout.row(true);
  header.label(IFACE_("Wrangle"), ICON_NODETREE);
  {
    const float base_pt = std::max(1.0f, style->widget.points);
    if (arg->font_pt < 1.0f) {
      const float scale = arg->editor_state.font_scale > 0.05f ? arg->editor_state.font_scale :
                                                                1.0f;
      arg->font_pt = std::clamp(base_pt * scale, 8.0f, 36.0f);
    }
    arg->editor_state.font_scale = std::clamp(arg->font_pt / base_pt, 0.5f, 4.0f);
    storage.textbox_state.font_scale = arg->editor_state.font_scale;
    ui::Button *size_but = ui::uiDefButV(block,
                                        ui::ButtonType::Num,
                                        IFACE_("Size"),
                                        0,
                                        0,
                                        short(6 * UI_UNIT_X),
                                        UI_UNIT_Y,
                                        &arg->font_pt,
                                        8.0f,
                                        36.0f,
                                        TIP_("Code editor font size"));
    ui::button_number_step_size_set(size_but, 1.0f);
    ui::button_number_precision_set(size_but, 0.0f);
    ui::button_func_set(size_but, [arg, base_pt](bContext &C) {
      arg->editor_state.font_scale = std::clamp(arg->font_pt / base_pt, 0.5f, 4.0f);
      if (bNode *n = wrangle_live_edit_node(arg)) {
        node_storage(*n).textbox_state.font_scale = arg->editor_state.font_scale;
      }
      if (ARegion *popup = CTX_wm_region_popup(&C)) {
        ED_region_tag_refresh_ui(popup);
        ED_region_tag_redraw(popup);
      }
    });
  }
  header.separator();
  PointerRNA compile_hdr = header.op(
      "NODE_OT_shader_wrangle_compile", IFACE_("Compile"), ICON_FILE_REFRESH);
  RNA_int_set(&compile_hdr, "node_identifier", live_node->identifier);
  header.separator_spacer();
  header.button("",
                arg->pinned ? ICON_PINNED : ICON_UNPINNED,
                [arg](bContext &C) {
                  arg->pinned = !arg->pinned;
                  if (ARegion *popup = CTX_wm_region_popup(&C)) {
                    ED_region_tag_refresh_ui(popup);
                    ED_region_tag_redraw(popup);
                  }
                },
                arg->pinned ? TIP_("Unpin") : TIP_("Keep this window open"));
  header.button("", ICON_X, [block](bContext & /*C*/) { ui::popup_menu_close(block, false); }, IFACE_("Close"));

  const char *code = storage.code ? storage.code : "";
  const vex::CompileOutput compiled = vex::compile_shader_material(code);
  if (!compiled.program && !compiled.error.empty()) {
    wrangle_draw_message(layout, compiled.error, ICON_ERROR, true);
  }
  else if (!compiled.warning.empty()) {
    wrangle_draw_message(layout, compiled.warning, ICON_INFO, false);
  }

  PointerRNA node_ptr = RNA_pointer_create_discrete(&arg->ntree->id, RNA_Node, live_node);
  const int64_t buttons_before = block->buttons_ptrs.size();
  layout.textbox_with_state(&node_ptr, "code", &arg->editor_state, std::nullopt);
  wrangle_apply_code_editor_flags(block, buttons_before);
  if (arg->activate_editor) {
    for (int64_t i = buttons_before; i < block->buttons_ptrs.size(); i++) {
      ui::Button *but = block->buttons_ptrs[i].get();
      if (but && but->type == ui::ButtonType::TextBox) {
        button_flag_enable(but, ui::BUT_ACTIVATE_ON_INIT);
        button_flag_enable(but, ui::BUT_ACTIVATE_ON_INIT_NO_SELECT);
      }
    }
    arg->activate_editor = false;
  }

  const int bounds_offset[2] = {0, -UI_UNIT_Y};
  ui::block_bounds_set_popup(block, int(0.3f * U.widget_unit), bounds_offset);
  return block;
}

static void wrangle_edit_popup_free(void *arg_v)
{
  WrangleEditPopupArg *arg = static_cast<WrangleEditPopupArg *>(arg_v);
  if (arg) {
    if (bNode *n = wrangle_live_edit_node(arg)) {
      if (arg->editor_state.font_scale > 0.05f) {
        node_storage(*n).textbox_state.font_scale = arg->editor_state.font_scale;
      }
      if (arg->editor_state.width > 0) {
        node_storage(*n).textbox_state.width = arg->editor_state.width;
      }
    }
  }
  MEM_delete(arg);
}

static wmOperatorStatus wrangle_edit_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  PointerRNA node_ptr = socket_items::ops::get_active_node_to_operate_on(
      C, op, ShaderWrangleInputItemsAccessor::node_idname);
  if (!node_ptr) {
    return OPERATOR_CANCELLED;
  }
  WrangleEditPopupArg *arg = MEM_new<WrangleEditPopupArg>(__func__);
  arg->node = static_cast<bNode *>(node_ptr.data);
  arg->ntree = reinterpret_cast<bNodeTree *>(node_ptr.owner_id);
  arg->node_identifier = arg->node ? arg->node->identifier : 0;
  arg->editor_state.visible_lines = 26;
  const float stored = node_storage(*arg->node).textbox_state.font_scale;
  arg->editor_state.font_scale = stored > 0.05f ? std::clamp(stored, 0.5f, 4.0f) : 1.0f;
  arg->editor_state.width = node_storage(*arg->node).textbox_state.width;
  ui::popup_block_invoke(C, wrangle_edit_popup, arg, wrangle_edit_popup_free);
  return OPERATOR_CANCELLED;
}

static void node_operators()
{
  socket_items::ops::make_common_operators<ShaderWrangleInputItemsAccessor>();
  socket_items::ops::make_add_item_operator<ShaderWrangleFieldItemsAccessor>();
  socket_items::ops::make_remove_item_by_index_operator<ShaderWrangleFieldItemsAccessor>();
  socket_items::ops::make_move_item_operator<ShaderWrangleFieldItemsAccessor>();
  socket_items::ops::make_add_item_operator<ShaderWrangleOutputItemsAccessor>();
  socket_items::ops::make_remove_item_by_index_operator<ShaderWrangleOutputItemsAccessor>();
  socket_items::ops::make_move_item_operator<ShaderWrangleOutputItemsAccessor>();
  WM_operatortype_append([](wmOperatorType *ot) {
    ot->name = "Compile";
    ot->idname = "NODE_OT_shader_wrangle_compile";
    ot->description =
        "Commit the editor draft and evaluate. Creates Constant panel inputs from "
        "chf/chi/chv/chb/chc/chm/chq and removes unused channel sockets";
    ot->exec = wrangle_compile_exec;
    ot->poll = socket_items::ops::editable_node_active_poll<ShaderWrangleInputItemsAccessor>;
    ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
    socket_items::ops::add_node_identifier_property(ot);
  });
  WM_operatortype_append([](wmOperatorType *ot) {
    ot->name = "Edit Wrangle Code";
    ot->idname = "NODE_OT_shader_wrangle_edit";
    ot->description = "Open a full wrangle code editor";
    ot->invoke = wrangle_edit_invoke;
    ot->poll = socket_items::ops::editable_node_active_poll<ShaderWrangleInputItemsAccessor>;
    ot->flag = OPTYPE_REGISTER;
    socket_items::ops::add_node_identifier_property(ot);
  });
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const bNodeSocket &other = params.other_socket();
  const eNodeSocketDatatype type = other.typeinfo->type;
  if (other.in_out == SOCK_IN) {
    if (!ShaderWrangleOutputItemsAccessor::supports_socket_type(type, params.node_tree().type)) {
      return;
    }
    params.add_item(IFACE_("Output"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("ShaderNodeWrangle"_ustr);
      wrangle_ensure_default_output(node);
      NodeShaderWrangle &storage = node_storage(node);
      if (storage.output_items.items_num == 1 && storage.output_items.items[0].name &&
          STREQ(storage.output_items.items[0].name, "Output"))
      {
        storage.output_items.items[0].socket_type = params.socket.typeinfo->type;
        params.update_and_connect_available_socket(node, "Output"_ustr);
      }
      else {
        const auto *item =
            socket_items::add_item_with_socket_type_and_name<ShaderWrangleOutputItemsAccessor>(
                params.node_tree, node, params.socket.typeinfo->type, "Output");
        params.update_and_connect_available_socket(node, UString(item->name));
      }
    });
  }
  else if (ShaderWrangleFieldItemsAccessor::supports_socket_type(type, params.node_tree().type)) {
    params.add_item(IFACE_("Field"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("ShaderNodeWrangle"_ustr);
      wrangle_ensure_default_output(node);
      const auto *item =
          socket_items::add_item_with_socket_type_and_name<ShaderWrangleFieldItemsAccessor>(
              params.node_tree, node, params.socket.typeinfo->type, params.socket.name);
      nodes::update_node_declaration_and_sockets(params.node_tree, node);
      params.connect_available_socket_by_identifier(
          node, UString(ShaderWrangleFieldItemsAccessor::socket_identifier_for_item(*item)));
    });
  }
}

}  // namespace blender::nodes::node_shader_wrangle_cc

namespace blender {

void register_node_type_sh_wrangle()
{
  namespace file_ns = nodes::node_shader_wrangle_cc;

  static bke::bNodeType ntype;

  sh_node_type_base(&ntype, "ShaderNodeWrangle"_ustr, SH_NODE_WRANGLE);
  ntype.ui_name = "Wrangle";
  ntype.ui_description =
      "VEX material function (like GLSL / Shadertoy). Outputs are float/vector/color "
      "you plug into other sockets. Field sockets are @name; Compile creates "
      "chf/chi/chv constants";
  ntype.enum_name_legacy = "WRANGLE";
  ntype.nclass = NODE_CLASS_SCRIPT;
  ntype.declare = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  ntype.get_extra_info = file_ns::node_extra_info;
  ntype.insert_link = file_ns::node_insert_link;
  ntype.register_operators = file_ns::node_operators;
  ntype.gather_link_search_ops = file_ns::node_gather_link_searches;
  ntype.blend_write_storage_content = file_ns::node_blend_write;
  ntype.blend_data_read_storage_content = file_ns::node_blend_read;
  ntype.gpu_fn = file_ns::node_shader_gpu_wrangle;
  ntype.add_ui_poll = object_shader_nodes_poll;
  bke::node_type_storage(
      ntype, "NodeShaderWrangle", file_ns::node_free_storage, file_ns::node_copy_storage);

  bke::node_register_type(ntype);
}

bool ntreeShaderWrangleEvalConstantRGB(const char *code, float rgb[3])
{
  rgb[0] = rgb[1] = rgb[2] = 0.0f;
  if (code == nullptr || code[0] == '\0') {
    return false;
  }
  std::string wrapped = "vector Output = {0,0,0};\n";
  wrapped += code;
  wrapped += "\nreturn Output;";
  const nodes::vex::CompileOutput compiled = nodes::vex::compile(wrapped);
  if (!compiled.program) {
    return false;
  }
  const nodes::vex::PureEvalOutput ev = nodes::vex::execute_pure(*compiled.program);
  if (!ev.ok) {
    return false;
  }
  rgb[0] = ev.return_vec.x;
  rgb[1] = ev.return_vec.y;
  rgb[2] = ev.return_vec.z;
  return true;
}

}  // namespace blender

namespace blender::nodes {

StructRNA **ShaderWrangleInputItemsAccessor::item_srna = &RNA_NodeShaderWrangleInputItem;
int ShaderWrangleInputItemsAccessor::node_type = SH_NODE_WRANGLE;
StructRNA **ShaderWrangleFieldItemsAccessor::item_srna = &RNA_NodeShaderWrangleInputItem;
int ShaderWrangleFieldItemsAccessor::node_type = SH_NODE_WRANGLE;
StructRNA **ShaderWrangleOutputItemsAccessor::item_srna = &RNA_NodeShaderWrangleOutputItem;
int ShaderWrangleOutputItemsAccessor::node_type = SH_NODE_WRANGLE;

void ShaderWrangleInputItemsAccessor::blend_write_item(BlendWriter *writer, const ItemT &item)
{
  writer->write_string(item.name);
}

void ShaderWrangleInputItemsAccessor::blend_read_data_item(BlendDataReader *reader, ItemT &item)
{
  BLO_read_string(reader, &item.name);
}

void ShaderWrangleFieldItemsAccessor::blend_write_item(BlendWriter *writer, const ItemT &item)
{
  writer->write_string(item.name);
}

void ShaderWrangleFieldItemsAccessor::blend_read_data_item(BlendDataReader *reader, ItemT &item)
{
  BLO_read_string(reader, &item.name);
}

void ShaderWrangleOutputItemsAccessor::blend_write_item(BlendWriter *writer, const ItemT &item)
{
  writer->write_string(item.name);
}

void ShaderWrangleOutputItemsAccessor::blend_read_data_item(BlendDataReader *reader, ItemT &item)
{
  BLO_read_string(reader, &item.name);
}

}  // namespace blender::nodes
