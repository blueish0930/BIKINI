/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 *
 * Image Wrangle (GPU Texture Editor): VEX code that runs once per pixel as a compute shader.
 *
 * Used like the Shader Wrangle: the sockets of the Field panel are images that the code reads
 * like variables, by the name of the socket, an output is written by assigning its name, and
 * Compile creates the single values of the Constant panel from chf/chi/chv/chb/chc.
 *
 * The values of the pixel the code runs on are read and written with `@`, like attributes:
 * `@ix`, `@iy`, `@X`, `@Y`, `@uv` and `@P`, and the size of the image `@resx` and `@resy`
 * (see #builtin_names). There is no geometry, so any other `@name` is an error, and so is a
 * name without `@` that is neither a socket nor a declared variable.
 *
 * The name of an image input is its value at the pixel the code runs on. Any other pixel is
 * `pixel(name, x, y)`, with integer coordinates that are clamped to the image.
 */

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include "MEM_guardedalloc.h"

#include "BLI_hash.hh"
#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_math_vector_types.hh"
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

#include "BLO_read_write.hh"

#include "BLT_translation.hh"

#include "DNA_array_utils.hh"
#include "DNA_node_types.h"
#include "DNA_userdef_types.h"

#include "ED_screen.hh"

#include "GPU_shader.hh"
#include "GPU_texture.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "UI_code_editor.hh"
#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "NOD_image_wrangle.hh"
#include "NOD_node_extra_info.hh"
#include "NOD_socket.hh"
#include "NOD_socket_items_blend.hh"
#include "NOD_socket_items_ops.hh"
#include "NOD_socket_items_ui.hh"
#include "NOD_socket_search_link.hh"
#include "NOD_vex.hh"

#include "gpu_shader_create_info.hh"

#include "node_image_util.hh"

#include "vex/vex_gpu.hh"
#include "vex/vex_program.hh"

#include "../../../editors/interface/interface_intern.hh"

namespace blender::nodes::node_image_wrangle_cc {

NODE_STORAGE_FUNCS(NodeShaderWrangle)

static const char *default_wrangle_code()
{
  return "Output = set(@X, @Y, 0, 1);";
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
  /* Tab inserts the first row: an exact match first, then the shortest names, which are the
   * closest to what is typed, then alphabetically. Ctrl-Space on nothing keeps the table order,
   * which is grouped by topic. */
  if (!match_token.is_empty()) {
    auto key = [](const expression::CompletionItem *item) {
      return item->insert_text.is_empty() ? item->name : item->insert_text;
    };
    std::stable_sort(matches.begin(),
                     matches.end(),
                     [&](const expression::CompletionItem *a, const expression::CompletionItem *b) {
                       const StringRef key_a = key(a);
                       const StringRef key_b = key(b);
                       if (key_a.size() != key_b.size()) {
                         return key_a.size() < key_b.size();
                       }
                       return key_a < key_b;
                     });
  }
  int n = 0;
  for (const expression::CompletionItem *item : matches) {
    if (n >= max_num) {
      break;
    }
    const StringRef shown = item->usage.is_empty() ?
                                (item->insert_text.is_empty() ? item->name : item->insert_text) :
                                item->usage;
    const StringRef ins = item->insert_text.is_empty() ? item->name : item->insert_text;
    /* `text \x1f parameter types \x1f category`, the list draws the three apart. */
    std::string packed = std::string(shown);
    packed.push_back('\x1f');
    if (item->kind == expression::CompletionKind::Function) {
      packed += vex::function_param_types(item->name);
    }
    packed.push_back('\x1f');
    packed += item->category_label;
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
    StringRef(packed).copy_utf8_truncated(out[n]);
    n++;
    (void)ins;
  }
  *r_num = n;
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
    default:
      return vex::Type::Float;
  }
}

/**
 * What the code can read about the pixel it runs on, named like in a Houdini COP wrangle. A socket
 * with one of these names replaces the value.
 */
struct BuiltinName {
  const char *name;
  vex::Type type;
  /** GLSL value, `texel` is the pixel and `wr_size` the size of the image. */
  const char *glsl;
};

static const BuiltinName builtin_names[] = {
    {"ix", vex::Type::Int, "texel.x"},
    {"iy", vex::Type::Int, "texel.y"},
    {"X", vex::Type::Float, "((float(texel.x) + 0.5) / float(wr_size.x))"},
    {"Y", vex::Type::Float, "((float(texel.y) + 0.5) / float(wr_size.y))"},
    {"resx", vex::Type::Int, "wr_size.x"},
    {"resy", vex::Type::Int, "wr_size.y"},
    /* Houdini's names. */
    {"xres", vex::Type::Int, "wr_size.x"},
    {"yres", vex::Type::Int, "wr_size.y"},
    {"uv", vex::Type::Vector, "vec3((vec2(texel) + 0.5) / vec2(wr_size), 0.0)"},
    {"P", vex::Type::Vector, "vec3(vec2(texel) + 0.5, 0.0)"},
};

/** The name the compiler stores for \a name, see `canonical_attr_name` in `vex_front.cc`. */
static std::string wrangle_canonical_name(const StringRef name)
{
  auto equals_ignoring_case = [&](const StringRef other) {
    if (name.size() != other.size()) {
      return false;
    }
    for (const int64_t i : name.index_range()) {
      const char a = name[i];
      const char lower = (a >= 'A' && a <= 'Z') ? char(a + 32) : a;
      if (lower != other[i]) {
        return false;
      }
    }
    return true;
  };
  if (name == "P" || equals_ignoring_case("position")) {
    return "position";
  }
  if (equals_ignoring_case("normal")) {
    return "normal";
  }
  if (equals_ignoring_case("output")) {
    return "Output";
  }
  return name;
}

/** Sockets first: the compiler keeps the first entry of a name, so they win over the builtins. */
static Vector<vex::PredeclaredName> wrangle_names(const bNode &node)
{
  Vector<vex::PredeclaredName> names;
  if (node.storage != nullptr) {
    const NodeShaderWrangle &storage = node_storage(node);
    for (const int i : IndexRange(storage.input_items.items_num)) {
      const NodeShaderWrangleItem &item = storage.input_items.items[i];
      if (shader_wrangle_item_kind(item) != SH_NODE_WRANGLE_ITEM_FIELD || item.name == nullptr) {
        continue;
      }
      names.append(
          {item.name, socket_to_vex_type(eNodeSocketDatatype(item.socket_type)), false});
    }
    for (const int i : IndexRange(storage.output_items.items_num)) {
      const NodeShaderWrangleItem &item = storage.output_items.items[i];
      if (item.name == nullptr) {
        continue;
      }
      names.append(
          {item.name, socket_to_vex_type(eNodeSocketDatatype(item.socket_type)), false});
    }
  }
  for (const BuiltinName &builtin : builtin_names) {
    names.append({builtin.name, builtin.type, true});
  }
  return names;
}

static vex::CompileOutput wrangle_compile(const bNode &node, const char *code)
{
  const Vector<vex::PredeclaredName> names = wrangle_names(node);
  return vex::compile_image_wrangle(code ? code : "", names);
}

/** \a arg is the node, the types of its sockets decide what the code means. */
static void wrangle_code_diag_fn(
    const char *str, void *arg, ui::uiCodeDiag *out, int *r_num, const int max_num)
{
  *r_num = 0;
  if (!str || max_num <= 0) {
    return;
  }
  const bNode *node = static_cast<const bNode *>(arg);
  const vex::CompileOutput compiled = node ? wrangle_compile(*node, str) :
                                             vex::compile_shader_material(str);
  const int n = std::min(max_num, int(compiled.diags.size()));
  for (int i = 0; i < n; i++) {
    out[i].offset = compiled.diags[i].offset;
    out[i].length = std::max(1, compiled.diags[i].length);
    out[i].is_error = compiled.diags[i].is_error ? 1 : 0;
  }
  *r_num = n;
}

static void wrangle_apply_code_editor_flags(ui::Block *block,
                                            const int64_t buttons_before,
                                            bNode *node,
                                            ui::CodeSearchState *search = nullptr)
{
  if (!block) {
    return;
  }
  for (int64_t i = buttons_before; i < block->buttons_ptrs.size(); i++) {
    ui::Button *but = block->buttons_ptrs[i].get();
    if (but && but->type == ui::ButtonType::TextBox) {
      button_flag_enable(but, ui::BUT_CODE_EDITOR);
      ui::button_code_search_set(but, search);
      button_drawflag_enable(but, ui::BUT_TEXT_LEFT);
      button_drawflag_disable(but, ui::BUT_NO_TEXT_PADDING);
      button_func_complete_set(but, wrangle_autocomplete_fn, nullptr);
      button_func_suggest_set(but, wrangle_suggest_fn, nullptr);
      button_func_code_diag_set(but, wrangle_code_diag_fn, node);
    }
  }
}

/** \} */

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
      C, op, ImageWrangleInputItemsAccessor::node_idname);
  if (!node_ptr) {
    return OPERATOR_CANCELLED;
  }
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(node_ptr.owner_id);
  bNode &node = *static_cast<bNode *>(node_ptr.data);
  NodeShaderWrangle &storage = node_storage(node);
  wrangle_commit_run_code(storage);
  const char *code = storage.code ? storage.code : "";
  const Vector<ChannelParmRef> wanted = wrangle_scan_channel_parms(code);

  socket_items::SocketItemsRef ref = ImageWrangleInputItemsAccessor::get_items_from_node(node);
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
        ImageWrangleInputItemsAccessor::destruct_item);
  }
  if (ref.active_index) {
    *ref.active_index = std::clamp(*ref.active_index, 0, std::max(*ref.items_num - 1, 0));
  }

  for (const ChannelParmRef &parm : wanted) {
    socket_items::SocketItemsRef live = ImageWrangleInputItemsAccessor::get_items_from_node(node);
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
        socket_items::add_item_with_socket_type_and_name<ImageWrangleInputItemsAccessor>(
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

static void wrangle_sanitize_item_types(bNode &node)
{
  NodeShaderWrangle &storage = node_storage(node);
  for (const int i : IndexRange(storage.output_items.items_num)) {
    NodeShaderWrangleItem &item = storage.output_items.items[i];
    if (!image_wrangle_supports_socket_type(eNodeSocketDatatype(item.socket_type))) {
      item.socket_type = SOCK_RGBA;
    }
  }
  for (const int i : IndexRange(storage.input_items.items_num)) {
    NodeShaderWrangleItem &item = storage.input_items.items[i];
    if (!image_wrangle_supports_socket_type(eNodeSocketDatatype(item.socket_type))) {
      item.socket_type = SOCK_RGBA;
    }
  }
}

static void wrangle_ensure_default_output(bNode &node)
{
  NodeShaderWrangle &storage = node_storage(node);
  if (storage.output_items.items_num > 0) {
    wrangle_sanitize_item_types(node);
    return;
  }
  NodeShaderWrangleItem *items = MEM_new_array<NodeShaderWrangleItem>(1, __func__);
  items[0].name = BLI_strdup("Output");
  items[0].socket_type = SOCK_RGBA;
  items[0].identifier = 0;
  items[0].kind = 0;
  storage.output_items.items = items;
  storage.output_items.items_num = 1;
  storage.output_items.next_identifier = 1;
}

static const char *wrangle_live_code(const NodeShaderWrangle &storage)
{
  /* The draft is `code`. Evaluation uses `code_run` until Ctrl+Enter, a click outside or Compile. */
  if (storage.code_run && storage.code_run[0] != '\0') {
    return storage.code_run;
  }
  if (storage.code && storage.code[0] != '\0') {
    return storage.code;
  }
  return "";
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  bNode &node = *static_cast<bNode *>(ptr->data);
  NodeShaderWrangle &storage = node_storage(node);

  layout.use_property_split_set(false);
  layout.alignment_set(ui::LayoutAlign::Expand);

  ui::Layout &compile_row = layout.row(true);
  compile_row.alignment_set(ui::LayoutAlign::Right);
  PointerRNA edit_ptr = compile_row.op("NODE_OT_image_wrangle_edit", IFACE_("Edit"), ICON_TEXT);
  RNA_int_set(&edit_ptr, "node_identifier", node.identifier);
  PointerRNA op_ptr = compile_row.op(
      "NODE_OT_image_wrangle_compile", IFACE_("Compile"), ICON_FILE_REFRESH);
  RNA_int_set(&op_ptr, "node_identifier", node.identifier);
  PointerRNA docs_ptr = compile_row.op("WM_OT_url_open", IFACE_("Docs"), ICON_HELP);
  RNA_string_set(&docs_ptr, "url", vex::docs_url);

  ui::Block *block = layout.block();
  const int64_t buttons_before = block ? block->buttons_ptrs.size() : 0;
  /* Same code size as in every other Wrangle node and in the editor window. */
  storage.textbox_state.font_scale = ui::wrangle_editor_font_scale();
  layout.textbox_with_state(ptr, "code", &storage.textbox_state, std::nullopt);
  wrangle_apply_code_editor_flags(block, buttons_before, &node);
  const char *code = storage.code ? storage.code : "";
  if (code[0] != '\0') {
    const vex::CompileOutput compiled = wrangle_compile(node, code);
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
                             .description("Single values from chf/chi/chv/chb/chc created by Compile");
  auto &field_panel = b.add_panel("Field"_ustr).description(
      "Images read per pixel; the code uses the name of the socket");

  const bNodeTree *tree = b.tree_or_null();
  if (bNode *node_mut = const_cast<bNode *>(b.node_or_null())) {
    if (node_mut->storage) {
      wrangle_sanitize_item_types(*node_mut);
      wrangle_fix_vector_socket_dimensions(*node_mut);
    }
  }
  const bNode *node = b.node_or_null();
  if (node && tree && node->storage) {
    const NodeShaderWrangle &storage = node_storage(*node);

    for (const int i : IndexRange(storage.output_items.items_num)) {
      const NodeShaderWrangleItem &item = storage.output_items.items[i];
      const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
      const UString identifier{ImageWrangleOutputItemsAccessor::socket_identifier_for_item(item)};
      const UString name{item.name ? item.name : ""};
      b.add_output(socket_type, name, identifier)
          .description("Image written per pixel: assign this name in the code")
          .structure_type(StructureType::Dynamic)
          .socket_name_ptr(&tree->id, *ImageWrangleOutputItemsAccessor::item_srna, &item, "name")
          .custom_draw([i](CustomSocketDrawParams &params) {
            socket_items::ui::draw_item_socket_with_remove<ImageWrangleOutputItemsAccessor>(
                params, i);
          });
    }
  }
  b.add_output<decl::Extend>(""_ustr, "__extend__wrangle_out"_ustr)
      .custom_draw(socket_items::ui::draw_extend_socket_fn<ImageWrangleOutputItemsAccessor>());

  if (node && tree && node->storage) {
    const NodeShaderWrangle &storage = node_storage(*node);

    for (const int i : IndexRange(storage.input_items.items_num)) {
      const NodeShaderWrangleItem &item = storage.input_items.items[i];
      if (shader_wrangle_item_kind(item) != SH_NODE_WRANGLE_ITEM_CONSTANT) {
        continue;
      }
      const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
      const UString identifier{ImageWrangleInputItemsAccessor::socket_identifier_for_item(item)};
      const UString name{item.name ? item.name : ""};
      constant_panel.add_input(socket_type, name, identifier)
          .socket_name_ptr(&tree->id, *ImageWrangleInputItemsAccessor::item_srna, &item, "name")
          .structure_type(StructureType::Single);
    }

    for (const int i : IndexRange(storage.input_items.items_num)) {
      const NodeShaderWrangleItem &item = storage.input_items.items[i];
      if (shader_wrangle_item_kind(item) != SH_NODE_WRANGLE_ITEM_FIELD) {
        continue;
      }
      const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
      const UString identifier{ImageWrangleFieldItemsAccessor::socket_identifier_for_item(item)};
      const UString name{item.name ? item.name : ""};
      field_panel.add_input(socket_type, name, identifier)
          .structure_type(StructureType::Dynamic)
          .socket_name_ptr(&tree->id, *ImageWrangleFieldItemsAccessor::item_srna, &item, "name")
          .custom_draw([i](CustomSocketDrawParams &params) {
            socket_items::ui::draw_item_socket_with_remove<ImageWrangleFieldItemsAccessor>(params,
                                                                                         i);
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
    socket_items::add_item_with_socket_type_and_name<ImageWrangleOutputItemsAccessor>(
        *tree, *node, SOCK_RGBA, "Output");
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
  socket_items::destruct_array<ImageWrangleInputItemsAccessor>(*node);
  socket_items::destruct_array<ImageWrangleOutputItemsAccessor>(*node);
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
  socket_items::copy_array<ImageWrangleInputItemsAccessor>(*src_node, *dst_node);
  socket_items::copy_array<ImageWrangleOutputItemsAccessor>(*src_node, *dst_node);
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  const NodeShaderWrangle &storage = node_storage(node);
  writer.write_string(storage.code);
  writer.write_string(storage.code_run);
  socket_items::blend_write<ImageWrangleInputItemsAccessor>(&writer, node);
  socket_items::blend_write<ImageWrangleOutputItemsAccessor>(&writer, node);
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  NodeShaderWrangle &storage = node_storage(node);
  BLO_read_string(&reader, &storage.code);
  BLO_read_string(&reader, &storage.code_run);
  if (storage.code_run == nullptr && storage.code) {
    storage.code_run = BLI_strdup(storage.code);
  }
  socket_items::blend_read_data<ImageWrangleInputItemsAccessor>(&reader, node);
  socket_items::blend_read_data<ImageWrangleOutputItemsAccessor>(&reader, node);
  wrangle_ensure_default_output(node);
}

static void node_extra_info(NodeExtraInfoParams &parameters)
{
  const NodeShaderWrangle &storage = node_storage(parameters.node);
  const char *code = storage.code ? storage.code : "";
  if (code[0] == '\0') {
    return;
  }
  const vex::CompileOutput compiled = wrangle_compile(parameters.node, code);
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
    row.tooltip = TIP_("This code cannot run as a shader, and the node only runs on the GPU");
    row.icon = ICON_ERROR;
    parameters.rows.append(std::move(row));
  }
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
        ImageWrangleOutputItemsAccessor>(
        params.ntree, params.node, params.node, params.link, "__extend__wrangle_out");
    if (!ok && params.C) {
      BKE_report(CTX_wm_reports(params.C),
                 RPT_ERROR,
                 "Wrangle output accepts float, integer, boolean, vector or color");
    }
    return ok;
  }
  if (StringRef(sock->identifier) == "__extend__wrangle_field") {
    const bool ok = socket_items::try_add_item_via_any_extend_socket<ImageWrangleFieldItemsAccessor>(
        params.ntree, params.node, params.node, params.link, "__extend__wrangle_field");
    if (!ok && params.C) {
      BKE_report(CTX_wm_reports(params.C),
                 RPT_ERROR,
                 "Wrangle field input accepts float, integer, boolean, vector or color");
    }
    return ok;
  }
  return false;
}

struct WrangleEditPopupArg {
  bNode *node = nullptr;
  bNodeTree *ntree = nullptr;
  int32_t node_identifier = 0;
  TextboxState editor_state;
  ui::CodeSearchState search;
  char status[256] = {};
  bool pinned = false;
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
        C, region, "image_wrangle_code_editor", ui::EmbossType::Emboss);
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
  /* Font preference is shared; an open editor keeps its own size and scroll state. */
  arg->editor_state.font_scale = ui::wrangle_editor_font_scale();

  const uiStyle *style = ui::style_get_dpi();
  ui::Block *block = ui::block_begin(C, region, "image_wrangle_code_editor", ui::EmbossType::Emboss);
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
    arg->font_pt = std::clamp(base_pt * arg->editor_state.font_scale, 8.0f, 36.0f);
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
      ui::wrangle_editor_font_scale_set(arg->font_pt / base_pt);
      if (ARegion *popup = CTX_wm_region_popup(&C)) {
        ED_region_tag_refresh_ui(popup);
        ED_region_tag_redraw(popup);
      }
    });
  }
  header.separator();
  PointerRNA compile_hdr = header.op(
      "NODE_OT_image_wrangle_compile", IFACE_("Compile"), ICON_FILE_REFRESH);
  RNA_int_set(&compile_hdr, "node_identifier", live_node->identifier);
  PointerRNA docs_hdr = header.op("WM_OT_url_open", IFACE_("Docs"), ICON_HELP);
  RNA_string_set(&docs_hdr, "url", vex::docs_url);
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

  PointerRNA tools_ptr = RNA_pointer_create_discrete(&arg->ntree->id, RNA_Node, arg->node);
  ui::code_editor_draw_tools(layout, tools_ptr, arg->search, arg->editor_state);

  const char *code = storage.code ? storage.code : "";
  const vex::CompileOutput compiled = wrangle_compile(*live_node, code);
  if (!compiled.program && !compiled.error.empty()) {
    wrangle_draw_message(layout, compiled.error, ICON_ERROR, true);
  }
  else if (!compiled.warning.empty()) {
    wrangle_draw_message(layout, compiled.warning, ICON_INFO, false);
  }

  PointerRNA node_ptr = RNA_pointer_create_discrete(&arg->ntree->id, RNA_Node, live_node);
  const int64_t buttons_before = block->buttons_ptrs.size();
  layout.textbox_with_state(&node_ptr, "code", &arg->editor_state, std::nullopt);
  wrangle_apply_code_editor_flags(block, buttons_before, live_node, &arg->search);

  const int bounds_offset[2] = {0, -UI_UNIT_Y};
  ui::block_bounds_set_popup(block, int(0.3f * U.widget_unit), bounds_offset);
  return block;
}

static void wrangle_edit_popup_free(void *arg_v)
{
  MEM_delete(static_cast<WrangleEditPopupArg *>(arg_v));
}

static wmOperatorStatus wrangle_edit_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  PointerRNA node_ptr = socket_items::ops::get_active_node_to_operate_on(
      C, op, ImageWrangleInputItemsAccessor::node_idname);
  if (!node_ptr) {
    return OPERATOR_CANCELLED;
  }
  WrangleEditPopupArg *arg = MEM_new<WrangleEditPopupArg>(__func__);
  arg->node = static_cast<bNode *>(node_ptr.data);
  arg->ntree = reinterpret_cast<bNodeTree *>(node_ptr.owner_id);
  arg->node_identifier = arg->node ? arg->node->identifier : 0;
  ui::wrangle_editor_state_load(arg->editor_state);
  ui::popup_block_invoke(C, wrangle_edit_popup, arg, wrangle_edit_popup_free);
  return OPERATOR_CANCELLED;
}

/* -------------------------------------------------------------------- */
/** \name Evaluation
 * \{ */

using namespace blender::compositor;

/** Type of a shader global, in the dialect the compiler emits. Null for what an image cannot hold. */
static const char *vex_type_glsl(const vex::Type type)
{
  switch (type) {
    case vex::Type::Bool:
      return "bool";
    case vex::Type::Int:
      return "int";
    case vex::Type::Float:
      return "float";
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
      return nullptr;
  }
}

static const char *vex_type_default(const vex::Type type)
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

static bool vex_type_is_pixel_value(const vex::Type type)
{
  return ELEM(type,
              vex::Type::Bool,
              vex::Type::Int,
              vex::Type::Float,
              vex::Type::Vector2,
              vex::Type::Vector,
              vex::Type::Vector4,
              vex::Type::Color,
              vex::Type::Rotation);
}

static bool result_type_is_pixel_value(const ResultType type)
{
  return ELEM(type,
              ResultType::Float,
              ResultType::Float2,
              ResultType::Float3,
              ResultType::Float4,
              ResultType::Color,
              ResultType::Int,
              ResultType::Bool);
}

/**
 * Values pass between images and the code as an RGBA value: a single number fills RGB, a 2D vector
 * has zero in B, and alpha is 1 for everything that has none.
 */
static std::string glsl_value_from_rgba(const std::string &rgba, const vex::Type type)
{
  switch (type) {
    case vex::Type::Float:
      return "(" + rgba + ").x";
    case vex::Type::Int:
      return "int((" + rgba + ").x)";
    case vex::Type::Bool:
      return "((" + rgba + ").x != 0.0)";
    case vex::Type::Vector2:
      return "(" + rgba + ").xy";
    case vex::Type::Vector:
      return "(" + rgba + ").xyz";
    default:
      return rgba;
  }
}

static std::string glsl_rgba_from_value(const std::string &value, const vex::Type type)
{
  switch (type) {
    case vex::Type::Float:
      return "vec4(vec3(" + value + "), 1.0)";
    case vex::Type::Int:
    case vex::Type::Bool:
      return "vec4(vec3(float(" + value + ")), 1.0)";
    case vex::Type::Vector2:
      return "vec4(" + value + ", 0.0, 1.0)";
    case vex::Type::Vector:
      return "vec4(" + value + ", 1.0)";
    default:
      return value;
  }
}

struct InputBinding {
  const Result *result = nullptr;
  /** Shader global that gets the value: `a_name` of a Field socket, `ch_name` of a Constant. */
  std::string target;
  vex::Type target_type = vex::Type::Float;
  /** Name of the sampler when the input is an image, of the push constant when it is a value. */
  std::string resource;
  ResultType type = ResultType::Float;
  bool is_texture = false;
  /** Integers and booleans are read from integer textures and constants. */
  bool is_int = false;
};

struct OutputBinding {
  Result *result = nullptr;
  std::string name;
  /** Shader global that is stored, empty when the code has no such name. */
  std::string source;
  vex::Type source_type = vex::Type::Float;
  std::string resource;
  bool is_int = false;
};

static std::string glsl_input_value(const InputBinding &binding)
{
  const std::string fetch = "texelFetch(" + binding.resource + ", texel, 0)";
  if (binding.is_int) {
    const std::string value = binding.is_texture ? fetch + ".x" : binding.resource;
    if (binding.target_type == vex::Type::Int) {
      return value;
    }
    if (binding.target_type == vex::Type::Bool) {
      return "(" + value + " != 0)";
    }
    return glsl_value_from_rgba("vec4(vec3(float(" + value + ")), 1.0)", binding.target_type);
  }
  /* A push constant already holds the RGBA form, see #single_value_rgba. */
  std::string rgba = binding.resource;
  if (binding.is_texture) {
    switch (binding.type) {
      case ResultType::Float:
        rgba = "vec4(vec3(" + fetch + ".x), 1.0)";
        break;
      case ResultType::Float2:
        rgba = "vec4(" + fetch + ".xy, 0.0, 1.0)";
        break;
      case ResultType::Float3:
        rgba = "vec4(" + fetch + ".xyz, 1.0)";
        break;
      default:
        rgba = fetch;
        break;
    }
  }
  return glsl_value_from_rgba(rgba, binding.target_type);
}

static std::string glsl_output_store(const OutputBinding &binding)
{
  const std::string rgba = binding.source.empty() ?
                               "vec4(0.0, 0.0, 0.0, 1.0)" :
                               glsl_rgba_from_value(binding.source, binding.source_type);
  if (!binding.is_int) {
    return "  imageStore(" + binding.resource + ", texel, " + rgba + ");\n";
  }
  std::string value = "int((" + rgba + ").x)";
  if (!binding.source.empty() && binding.source_type == vex::Type::Int) {
    value = binding.source;
  }
  else if (!binding.source.empty() && binding.source_type == vex::Type::Bool) {
    value = "int(" + binding.source + ")";
  }
  return "  imageStore(" + binding.resource + ", texel, ivec4(" + value + "));\n";
}

static float4 single_value_rgba(const Result &result)
{
  switch (result.type()) {
    case ResultType::Float: {
      const float value = result.get_single_value_default<float>();
      return float4(value, value, value, 1.0f);
    }
    case ResultType::Float2: {
      const float2 value = result.get_single_value_default<float2>();
      return float4(value.x, value.y, 0.0f, 1.0f);
    }
    case ResultType::Float3: {
      const float3 value = result.get_single_value_default<float3>();
      return float4(value.x, value.y, value.z, 1.0f);
    }
    case ResultType::Float4:
      return result.get_single_value_default<float4>();
    case ResultType::Color: {
      const Color value = result.get_single_value_default<Color>();
      return float4(value.r, value.g, value.b, value.a);
    }
    default:
      return float4(0.0f, 0.0f, 0.0f, 1.0f);
  }
}

static int single_value_int(const Result &result)
{
  if (result.type() == ResultType::Bool) {
    return result.get_single_value_default<bool>() ? 1 : 0;
  }
  return result.get_single_value_default<int32_t>();
}

struct CachedShader {
  gpu::Shader *shader = nullptr;
  std::string error;
};

static std::mutex &shader_cache_mutex()
{
  static std::mutex mutex;
  return mutex;
}

static Map<uint64_t, CachedShader> &shader_cache()
{
  static Map<uint64_t, CachedShader> cache;
  return cache;
}

void free_session_caches()
{
  std::lock_guard lock(shader_cache_mutex());
  for (CachedShader &cached : shader_cache().values()) {
    if (cached.shader != nullptr) {
      GPU_shader_free(cached.shader);
    }
  }
  shader_cache().clear();
}

static std::string clean_compile_log(const std::string &log)
{
  /* Without the terminal color codes, and short enough for a node warning. */
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
  while (!clean.empty() && ELEM(clean.back(), '\n', '\r', ' ')) {
    clean.pop_back();
  }
  if (clean.size() > 1200) {
    clean.resize(1200);
    clean += "\n...";
  }
  return clean;
}

/**
 * The shader of one program with one set of resources. What is an image and what a single value
 * is only known when the node runs, so the same code can have more than one shader.
 */
static const CachedShader &wrangle_shader_ensure(const std::string &source,
                                                 const Span<InputBinding> inputs,
                                                 const Span<OutputBinding> outputs)
{
  std::string signature = source;
  for (const InputBinding &input : inputs) {
    signature += input.resource + (input.is_texture ? ":t" : ":c") + (input.is_int ? "i;" : "f;");
  }
  for (const OutputBinding &output : outputs) {
    signature += output.resource + ":" +
                 std::to_string(int(Result::gpu_texture_format(output.result->type(),
                                                               output.result->precision()))) +
                 ";";
  }
  const uint64_t key = get_default_hash(StringRef(signature));
  std::lock_guard lock(shader_cache_mutex());
  if (const CachedShader *cached = shader_cache().lookup_ptr(key)) {
    return *cached;
  }

  using namespace blender::gpu::shader;
  /* The name has to be this one: only then are the resources of a generated shader declared. */
  ShaderCreateInfo info("pyGPU_Shader");
  info.local_group_size(16, 16);
  info.push_constant(Type::int2_t, "wr_size");
  /* The info keeps references to the names. */
  std::deque<std::string> names;
  int sampler_slot = 0;
  for (const InputBinding &input : inputs) {
    names.push_back(input.resource);
    if (input.is_texture) {
      info.sampler(
          sampler_slot++, input.is_int ? ImageType::Int2D : ImageType::Float2D, names.back());
    }
    else {
      info.push_constant(input.is_int ? Type::int_t : Type::float4_t, names.back());
    }
  }
  int image_slot = 0;
  for (const OutputBinding &output : outputs) {
    names.push_back(output.resource);
    info.image(image_slot++,
               Result::gpu_texture_format(output.result->type(), output.result->precision()),
               Qualifier::write,
               output.is_int ? ImageReadWriteType::Int2D : ImageReadWriteType::Float2D,
               names.back());
  }
  info.typedef_source_generated = vex::gpu_glsl_array_struct();
  info.compute_source_generated = source;

  CachedShader cached;
  GPU_shader_clear_last_compile_error();
  try {
    cached.shader = GPU_shader_create_from_info_python(
        reinterpret_cast<const GPUShaderCreateInfo *>(&info));
  }
  catch (const std::exception &) {
    cached.shader = nullptr;
  }
  if (cached.shader == nullptr) {
    const std::string log = clean_compile_log(GPU_shader_get_last_compile_error());
    cached.error = log.empty() ? "The wrangle shader failed to compile" :
                                 "The wrangle shader failed to compile:\n" + log;
  }
  shader_cache().add(key, std::move(cached));
  return shader_cache().lookup(key);
}

static const NodeShaderWrangleItem *find_item(const NodeShaderWrangleItems &items,
                                              const StringRef socket_identifier,
                                              const bool is_output)
{
  for (const int i : IndexRange(items.items_num)) {
    const NodeShaderWrangleItem &item = items.items[i];
    const std::string identifier =
        is_output ? ImageWrangleOutputItemsAccessor::socket_identifier_for_item(item) :
                    ImageWrangleInputItemsAccessor::socket_identifier_for_item(item);
    if (socket_identifier == identifier) {
      return &item;
    }
  }
  return nullptr;
}

class WrangleOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  /** Without an image to take the size from, the node fills the resolution of the editor. */
  Domain compute_domain() override
  {
    for (const bNodeSocket &socket : this->node().inputs) {
      if (!is_socket_available(&socket)) {
        continue;
      }
      const Result &input = this->get_input(socket.identifier);
      if (input.is_allocated() && !input.is_single_value()) {
        return NodeOperation::compute_domain();
      }
    }
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    const bNode &node = this->node();
    const NodeShaderWrangle &storage = node_storage(node);

    Vector<OutputBinding> outputs;
    for (const bNodeSocket &socket : node.outputs) {
      if (!is_socket_available(&socket)) {
        continue;
      }
      Result &result = this->get_result(socket.identifier);
      if (!result.should_compute()) {
        continue;
      }
      const NodeShaderWrangleItem *item = find_item(storage.output_items, socket.identifier, true);
      OutputBinding binding;
      binding.result = &result;
      binding.name = (item && item->name) ? item->name : socket.name;
      binding.is_int = ELEM(result.type(), ResultType::Int, ResultType::Bool);
      binding.resource = "wr_out_" + std::to_string(outputs.size());
      outputs.append(std::move(binding));
    }
    if (outputs.is_empty()) {
      return;
    }

    auto fail = [&](std::string message) {
      this->add_warning(NodeWarningType::Error, std::move(message));
      for (OutputBinding &output : outputs) {
        output.result->allocate_invalid();
      }
    };

    if (!this->context().use_gpu()) {
      fail("The Wrangle node runs as a shader and needs a GPU");
      return;
    }
    for (const OutputBinding &output : outputs) {
      if (!result_type_is_pixel_value(output.result->type())) {
        fail("Output \"" + output.name + "\" has a type an image cannot hold");
        return;
      }
    }
    /* Enough for every graphics driver: eight images to write. */
    if (outputs.size() > 8) {
      fail("At most 8 outputs can be written by one Wrangle node");
      return;
    }

    const vex::CompileOutput compiled = wrangle_compile(node, wrangle_live_code(storage));
    if (!compiled.program) {
      fail(compiled.error.empty() ? "The code did not compile" : compiled.error);
      return;
    }
    const vex::Program &program = *compiled.program;
    if (!program.gpu_ok) {
      fail(program.gpu_error.empty() ? "The code cannot run as a shader" : program.gpu_error);
      return;
    }

    auto attr_index = [&](const StringRef name) -> int {
      const std::string attr_name = wrangle_canonical_name(name);
      for (const int i : program.attrs.index_range()) {
        if (program.attrs[i].name == attr_name) {
          return i;
        }
      }
      return -1;
    };

    /* Names of sockets: a builtin of the same name is not available. */
    Set<std::string> socket_names;
    for (const int i : IndexRange(storage.output_items.items_num)) {
      if (const char *name = storage.output_items.items[i].name) {
        socket_names.add(wrangle_canonical_name(name));
      }
    }

    Vector<InputBinding> inputs;
    /* Kept alive until the shader has run. */
    std::deque<Result> uploaded;
    int textures_num = 0;
    for (const bNodeSocket &socket : node.inputs) {
      if (!is_socket_available(&socket)) {
        continue;
      }
      const NodeShaderWrangleItem *item = find_item(storage.input_items, socket.identifier, false);
      if (item == nullptr || item->name == nullptr) {
        continue;
      }
      InputBinding binding;
      if (shader_wrangle_item_kind(*item) == SH_NODE_WRANGLE_ITEM_FIELD) {
        socket_names.add(wrangle_canonical_name(item->name));
        const int attr = attr_index(item->name);
        if (attr < 0 || !program.attrs[attr].read) {
          continue;
        }
        binding.target = glsl_ident(program.attrs[attr].name, "a_");
        binding.target_type = program.attrs[attr].type;
      }
      else {
        bool found = false;
        for (const vex::Program::GpuCh &channel : program.gpu_ch) {
          if (channel.name == item->name) {
            binding.target = glsl_ident(channel.name, "ch_");
            binding.target_type = channel.type;
            found = true;
            break;
          }
        }
        if (!found) {
          continue;
        }
      }
      if (!vex_type_is_pixel_value(binding.target_type)) {
        continue;
      }
      const Result &result = this->get_input(socket.identifier);
      if (!result_type_is_pixel_value(result.type())) {
        fail("Input \"" + std::string(item->name) + "\" has a type the code cannot read");
        return;
      }
      binding.result = &result;
      binding.type = result.type();
      binding.is_int = ELEM(result.type(), ResultType::Int, ResultType::Bool);
      binding.is_texture = result.is_allocated() && !result.is_single_value();
      if (binding.is_texture) {
        if (!result.is_stored_on_gpu()) {
          /* A node before this one ran on the CPU. */
          uploaded.push_back(result.upload_to_gpu(true));
          binding.result = &uploaded.back();
        }
        textures_num++;
      }
      binding.resource = (binding.is_texture ? "wr_tx_" : "wr_u_") +
                         std::to_string(inputs.size());
      inputs.append(std::move(binding));
    }
    auto release_uploaded = [&]() {
      for (Result &result : uploaded) {
        result.release();
      }
    };
    if (textures_num > 16) {
      release_uploaded();
      fail("At most 16 images can be read by one Wrangle node");
      return;
    }

    std::string globals = "int elem_i;\nint rand_seq;\n";
    std::string init;
    for (const vex::AttrInfo &attr : program.attrs) {
      const char *glsl_type = vex_type_glsl(attr.type);
      if (glsl_type == nullptr) {
        release_uploaded();
        fail("\"" + attr.name + "\": arrays and strings cannot be stored per pixel");
        return;
      }
      const std::string ident = glsl_ident(attr.name, "a_");
      globals += std::string(glsl_type) + " " + ident + ";\n";
      init += "  " + ident + " = " + vex_type_default(attr.type) + ";\n";
    }
    for (const vex::Program::GpuCh &channel : program.gpu_ch) {
      const char *glsl_type = vex_type_glsl(channel.type);
      if (glsl_type == nullptr) {
        release_uploaded();
        fail("\"" + channel.name + "\": this channel type is not available here");
        return;
      }
      const std::string ident = glsl_ident(channel.name, "ch_");
      globals += std::string(glsl_type) + " " + ident + ";\n";
      init += "  " + ident + " = " + vex_type_default(channel.type) + ";\n";
    }
    for (const BuiltinName &builtin : builtin_names) {
      const int attr = attr_index(builtin.name);
      if (attr < 0 || !program.attrs[attr].read || program.attrs[attr].type != builtin.type ||
          socket_names.contains(wrangle_canonical_name(builtin.name)))
      {
        continue;
      }
      init += "  " + glsl_ident(program.attrs[attr].name, "a_") + " = " + builtin.glsl + ";\n";
    }
    for (const InputBinding &input : inputs) {
      init += "  " + input.target + " = " + glsl_input_value(input) + ";\n";
    }

    /* `pixel(name, x, y)` in the code is a call of one of these. */
    std::string pixel_functions;
    for (const std::string &name : program.gpu_pixel_inputs) {
      const std::string target = glsl_ident(name, "a_");
      const InputBinding *binding = nullptr;
      for (const InputBinding &input : inputs) {
        if (input.target == target) {
          binding = &input;
          break;
        }
      }
      if (binding == nullptr) {
        release_uploaded();
        fail("pixel(): \"" + name + "\" is not an image input of the node");
        return;
      }
      /* Inputs have the size of the result, see #compute_domain. A single value is the same
       * everywhere. */
      pixel_functions += std::string(vex_type_glsl(binding->target_type)) + " wr_pixel_" +
                         target +
                         "(int x, int y)\n{\n"
                         "  ivec2 texel = clamp(ivec2(x, y), ivec2(0), wr_size - 1);\n"
                         "  return " +
                         glsl_input_value(*binding) + ";\n}\n";
    }

    std::string stores;
    for (OutputBinding &output : outputs) {
      const int attr = attr_index(output.name);
      if (attr >= 0 && vex_type_is_pixel_value(program.attrs[attr].type)) {
        output.source = glsl_ident(program.attrs[attr].name, "a_");
        output.source_type = program.attrs[attr].type;
      }
      stores += glsl_output_store(output);
    }

    /* The array functions report an overflow in `wr_ovf`. An image has no interpreter to fall
     * back to, so the flag is only there to make them compile. */
    std::string source = "int wr_ovf[1];\n";
    source += vex::gpu_glsl_preamble();
    source += "vec4 wr_qmul(vec4 a, vec4 b)\n{\n"
              "  return vec4(a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,\n"
              "              a.w*b.y + a.y*b.w + a.z*b.x - a.x*b.z,\n"
              "              a.w*b.z + a.z*b.w + a.x*b.y - a.y*b.x,\n"
              "              a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z);\n"
              "}\n"
              "vec3 wr_qrot(vec4 q, vec3 v)\n{\n"
              "  vec3 t = 2.0 * cross(q.xyz, v);\n"
              "  return v + q.w * t + cross(q.xyz, t);\n"
              "}\n";
    /* The derivative functions of a material take this argument. A compute shader has no
     * derivatives, they are zero. */
    source += pixel_functions;
    source += "#define KernelGlobals int\nint kg = 0;\n";
    source += program.gpu_helpers;
    if (!source.empty() && source.back() != '\n') {
      source += "\n";
    }
    source += "#define mix wr_interp\n#define lerp wr_interp\n";
    source += globals;
    /* In a function of its own, so that a `return` in the code still stores the outputs. */
    source += "void wr_body()\n{\n";
    source += program.gpu_src;
    if (!program.gpu_src.empty() && program.gpu_src.back() != '\n') {
      source += "\n";
    }
    source += "}\n#undef mix\n#undef lerp\n";
    source += "void main()\n{\n"
              "  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);\n"
              "  if (texel.x >= wr_size.x || texel.y >= wr_size.y) {\n"
              "    return;\n"
              "  }\n"
              "  elem_i = texel.y * wr_size.x + texel.x;\n"
              "  rand_seq = 0;\n";
    source += init;
    source += "  wr_body();\n";
    source += stores;
    source += "}\n";

    const CachedShader &cached = wrangle_shader_ensure(source, inputs, outputs);
    if (cached.shader == nullptr) {
      release_uploaded();
      fail(cached.error);
      return;
    }
    gpu::Shader *shader = cached.shader;

    const Domain domain = this->compute_domain();
    const int2 size = domain.data_size;

    GPU_shader_bind(shader);
    GPU_shader_uniform_2iv(shader, "wr_size", size);
    for (const InputBinding &input : inputs) {
      if (input.is_texture) {
        input.result->bind_as_texture(shader, input.resource.c_str());
      }
      else if (input.is_int) {
        GPU_shader_uniform_1i(shader, input.resource.c_str(), single_value_int(*input.result));
      }
      else {
        const float4 value = single_value_rgba(*input.result);
        GPU_shader_uniform_4fv(shader, input.resource.c_str(), value);
      }
    }
    for (const OutputBinding &output : outputs) {
      output.result->allocate_texture(domain);
      output.result->bind_as_image(shader, output.resource.c_str());
    }

    compute_dispatch_threads_at_least(shader, size);

    for (const InputBinding &input : inputs) {
      if (input.is_texture) {
        input.result->unbind_as_texture();
      }
    }
    for (const OutputBinding &output : outputs) {
      output.result->unbind_as_image();
    }
    GPU_shader_unbind();
    release_uploaded();
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new WrangleOperation(context, node);
}

/** \} */

static void node_operators()
{
  socket_items::ops::make_common_operators<ImageWrangleInputItemsAccessor>();
  socket_items::ops::make_add_item_operator<ImageWrangleFieldItemsAccessor>();
  socket_items::ops::make_remove_item_by_index_operator<ImageWrangleFieldItemsAccessor>();
  socket_items::ops::make_move_item_operator<ImageWrangleFieldItemsAccessor>();
  socket_items::ops::make_add_item_operator<ImageWrangleOutputItemsAccessor>();
  socket_items::ops::make_remove_item_by_index_operator<ImageWrangleOutputItemsAccessor>();
  socket_items::ops::make_move_item_operator<ImageWrangleOutputItemsAccessor>();
  WM_operatortype_append([](wmOperatorType *ot) {
    ot->name = "Compile";
    ot->idname = "NODE_OT_image_wrangle_compile";
    ot->description =
        "Commit the editor draft and evaluate. Creates Constant panel inputs from "
        "chf/chi/chv/chb/chc and removes unused channel sockets";
    ot->exec = wrangle_compile_exec;
    ot->poll = socket_items::ops::editable_node_active_poll<ImageWrangleInputItemsAccessor>;
    ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
    socket_items::ops::add_node_identifier_property(ot);
  });
  WM_operatortype_append([](wmOperatorType *ot) {
    ot->name = "Edit Wrangle Code";
    ot->idname = "NODE_OT_image_wrangle_edit";
    ot->description = "Open a full wrangle code editor";
    ot->invoke = wrangle_edit_invoke;
    ot->poll = socket_items::ops::editable_node_active_poll<ImageWrangleInputItemsAccessor>;
    ot->flag = OPTYPE_REGISTER;
    socket_items::ops::add_node_identifier_property(ot);
  });
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const bNodeSocket &other = params.other_socket();
  const eNodeSocketDatatype type = other.typeinfo->type;
  if (!image_wrangle_supports_socket_type(type)) {
    return;
  }
  if (other.in_out == SOCK_IN) {
    params.add_item(IFACE_("Output"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("ImageNodeWrangle"_ustr);
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
            socket_items::add_item_with_socket_type_and_name<ImageWrangleOutputItemsAccessor>(
                params.node_tree, node, params.socket.typeinfo->type, "Output");
        params.update_and_connect_available_socket(node, UString(item->name));
      }
    });
  }
  else {
    params.add_item(IFACE_("Field"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("ImageNodeWrangle"_ustr);
      wrangle_ensure_default_output(node);
      const auto *item =
          socket_items::add_item_with_socket_type_and_name<ImageWrangleFieldItemsAccessor>(
              params.node_tree, node, params.socket.typeinfo->type, params.socket.name);
      nodes::update_node_declaration_and_sockets(params.node_tree, node);
      params.connect_available_socket_by_identifier(
          node, UString(ImageWrangleFieldItemsAccessor::socket_identifier_for_item(*item)));
    });
  }
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeWrangle"_ustr, IMG_NODE_WRANGLE);
  ntype.ui_name = "Wrangle";
  ntype.ui_description =
      "Code that runs for every pixel on the GPU. Field inputs are images read by the name of "
      "their socket (pixel(name, x, y) reads another pixel), an output is written by assigning "
      "its name. The pixel is @ix, @iy, @P";
  ntype.enum_name_legacy = "WRANGLE";
  ntype.nclass = NODE_CLASS_SCRIPT;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.updatefunc = node_update;
  ntype.get_extra_info = node_extra_info;
  ntype.insert_link = node_insert_link;
  ntype.register_operators = node_operators;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  ntype.get_compositor_operation = get_compositor_operation;
  bke::node_type_storage(ntype, "NodeShaderWrangle", node_free_storage, node_copy_storage);

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_wrangle_cc

namespace blender::nodes {

StructRNA **ImageWrangleInputItemsAccessor::item_srna = &RNA_NodeImageWrangleInputItem;
int ImageWrangleInputItemsAccessor::node_type = IMG_NODE_WRANGLE;
StructRNA **ImageWrangleFieldItemsAccessor::item_srna = &RNA_NodeImageWrangleInputItem;
int ImageWrangleFieldItemsAccessor::node_type = IMG_NODE_WRANGLE;
StructRNA **ImageWrangleOutputItemsAccessor::item_srna = &RNA_NodeImageWrangleOutputItem;
int ImageWrangleOutputItemsAccessor::node_type = IMG_NODE_WRANGLE;

}  // namespace blender::nodes
