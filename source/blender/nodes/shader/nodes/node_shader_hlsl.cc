/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup shdnodes
 *
 * GLSL shader node: multi-line GLSL body with dynamic sockets, evaluated in EEVEE.
 * A small HLSL-compat layer (lerp/saturate/frac/ddx/ddy) is rewritten to GLSL.
 */

#include <cctype>
#include <string>

#include "BLI_function_ref.hh"
#include "BLI_hash.hh"
#include "BLI_map.hh"
#include "BLI_set.hh"
#include "BLI_string.hh"
#include "BLI_string_ref.hh"
#include "BLI_string_utils.hh"
#include "BLI_vector.hh"

#include "GPU_material.hh"

#include "node_shader_util.hh"

#include "BLO_read_write.hh"

#include "NOD_shader_hlsl.hh"
#include "NOD_socket.hh"
#include "NOD_socket_items_blend.hh"
#include "NOD_socket_items_ops.hh"
#include "NOD_socket_items_ui.hh"
#include "NOD_socket_search_link.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_prototypes.hh"

#include "BKE_context.hh"
#include "BKE_scene.hh"
#include "DNA_scene_types.h"
#include "NOD_node_extra_info.hh"

namespace blender {

namespace nodes::node_shader_hlsl_cc {

NODE_STORAGE_FUNCS(NodeShaderHLSL)

/* -------------------------------------------------------------------- */
/** \name Optional HLSL aliases → GLSL (body is otherwise GLSL)
 * \{ */

static bool is_ident_char(const char c)
{
  return std::isalnum(uchar(c)) || c == '_';
}

/* Strip C/HLSL comments. GPU material sources must not contain line or block comments
 * (see GPUSource constructor asserts). */
static std::string strip_comments(const StringRef input)
{
  std::string out;
  out.reserve(input.size());
  enum class Mode { Normal, LineComment, BlockComment, String, Char } mode = Mode::Normal;

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
        else if (c == '\'') {
          mode = Mode::Char;
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
          /* Keep a space so tokens do not glue together. */
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
      case Mode::Char:
        out.push_back(c);
        if (c == '\\' && n != '\0') {
          out.push_back(n);
          i++;
        }
        else if (c == '\'') {
          mode = Mode::Normal;
        }
        break;
    }
  }
  return out;
}

/** Replace whole-word `from` with `to` (identifier boundaries). */
static void replace_word(std::string &code, const StringRef from, const StringRef to)
{
  size_t pos = 0;
  while ((pos = code.find(from.data(), pos, from.size())) != std::string::npos) {
    const bool left_ok = (pos == 0) || !is_ident_char(code[pos - 1]);
    const bool right_ok = (pos + from.size() >= code.size()) ||
                          !is_ident_char(code[pos + from.size()]);
    if (left_ok && right_ok) {
      code.replace(pos, from.size(), to.data(), to.size());
      pos += to.size();
    }
    else {
      pos += from.size();
    }
  }
}

/** Rewrite `call_name(arg)` using #make_replacement on the top-level argument list. */
static void replace_call(std::string &code,
                         const StringRef call_name,
                         const FunctionRef<std::string(StringRef arg)> &make_replacement)
{
  const size_t name_len = call_name.size();
  size_t pos = 0;
  while ((pos = code.find(call_name.data(), pos, name_len)) != std::string::npos) {
    const bool left_ok = (pos == 0) || !is_ident_char(code[pos - 1]);
    const bool right_ok = (pos + name_len >= code.size()) ||
                          !is_ident_char(code[pos + name_len]);
    if (!left_ok || !right_ok) {
      pos += name_len;
      continue;
    }
    const size_t open = code.find_first_not_of(" \t\r\n", pos + name_len);
    if (open == std::string::npos || code[open] != '(') {
      pos += name_len;
      continue;
    }
    int depth = 0;
    size_t close = std::string::npos;
    for (size_t i = open; i < code.size(); i++) {
      if (code[i] == '(') {
        depth++;
      }
      else if (code[i] == ')') {
        depth--;
        if (depth == 0) {
          close = i;
          break;
        }
      }
    }
    if (close == std::string::npos) {
      break;
    }
    const StringRef arg = StringRef(code).substr(open + 1, close - open - 1);
    const std::string replacement = make_replacement(arg);
    code.replace(pos, close - pos + 1, replacement);
    pos += replacement.size();
  }
}

static std::string translate_hlsl_subset_to_glsl(std::string code)
{
  code = strip_comments(code);

  /* Only rewrite HLSL-specific names. Leave real GLSL (vecN, mix, fract, dFdx) alone.
   * Do not rewrite types like half/matrix — those collide with identifiers. */
  replace_word(code, "lerp", "mix");
  replace_word(code, "frac", "fract");
  replace_word(code, "rsqrt", "inversesqrt");
  replace_word(code, "ddx", "dFdx");
  replace_word(code, "ddy", "dFdy");
  replace_word(code, "atan2", "atan");
  replace_word(code, "ddx_fine", "dFdx");
  replace_word(code, "ddy_fine", "dFdy");
  replace_word(code, "ddx_coarse", "dFdx");
  replace_word(code, "ddy_coarse", "dFdy");

  /* saturate(x) → clamp(x, 0.0, 1.0) */
  replace_call(code, "saturate", [](const StringRef arg) -> std::string {
    return "clamp(" + std::string(arg) + ", 0.0, 1.0)";
  });

  /* mad(a, b, c) → ((a) * (b) + (c)) */
  replace_call(code, "mad", [](const StringRef arg) -> std::string {
    /* Split on top-level commas. */
    Vector<std::string> parts;
    int depth = 0;
    size_t start = 0;
    for (size_t i = 0; i < arg.size(); i++) {
      if (arg[i] == '(') {
        depth++;
      }
      else if (arg[i] == ')') {
        depth--;
      }
      else if (arg[i] == ',' && depth == 0) {
        parts.append(std::string(arg.substr(start, i - start)));
        start = i + 1;
      }
    }
    parts.append(std::string(arg.substr(start)));
    if (parts.size() == 3) {
      return "((" + parts[0] + ") * (" + parts[1] + ") + (" + parts[2] + "))";
    }
    return "mad(" + std::string(arg) + ")";
  });

  /* rcp(x) → (1.0 / (x)) */
  replace_call(code, "rcp", [](const StringRef arg) -> std::string {
    return "(1.0 / (" + std::string(arg) + "))";
  });

  return code;
}

static const char *hlsl_intern_gpu_name(const char *name)
{
  static Map<std::string, const char *> intern;
  const std::string key(name);
  if (const char **existing = intern.lookup_ptr(key)) {
    return *existing;
  }
  const char *copy = BLI_strdup(name);
  intern.add(name, copy);
  return copy;
}

/** Match #node_gpu_stack_from_data: int/bool ride as float. */
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
      return GPU_VEC4;
    default:
      return GPU_FLOAT;
  }
}

static const char *gpu_type_glsl_name(const GPUType type)
{
  switch (type) {
    case GPU_FLOAT:
      return "float";
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
    case GPU_INT:
      return "int";
    case GPU_INT2:
      return "int2";
    case GPU_INT3:
      return "int3";
    case GPU_INT4:
      return "int4";
    case GPU_BOOL:
      return "bool";
    case GPU_CLOSURE:
      return "Closure";
    default:
      return "float";
  }
}

/** Sanitize socket name to a valid GLSL identifier (not yet uniquified). */
static std::string glsl_identifier_from_name(const char *name, const char *fallback_prefix, int index)
{
  std::string id;
  if (name && name[0] != '\0') {
    if (std::isalpha(uchar(name[0])) || name[0] == '_') {
      id.push_back(name[0]);
    }
    else {
      id.push_back('_');
    }
    for (const char *p = name + 1; *p; p++) {
      if (is_ident_char(*p)) {
        id.push_back(*p);
      }
      else {
        id.push_back('_');
      }
    }
  }
  if (id.empty()) {
    id = std::string(fallback_prefix) + std::to_string(index);
  }
  return id;
}

/** Ensure parameter identifiers are unique across inputs and outputs. */
static std::string uniquify_identifier(std::string id, Set<std::string> &used)
{
  if (used.add(id)) {
    return id;
  }
  for (int suffix = 2;; suffix++) {
    std::string candidate = id + "_" + std::to_string(suffix);
    if (used.add(candidate)) {
      return candidate;
    }
  }
}

static uint64_t hash_hlsl_signature(const NodeShaderHLSL &storage, const bNode &node)
{
  DefaultHash<StringRef> hasher;
  uint64_t h = 0;
  h = h * 33 + hasher(storage.code ? storage.code : "");
  for (const int i : IndexRange(storage.input_items.items_num)) {
    const NodeShaderHLSLItem &item = storage.input_items.items[i];
    h = h * 33 + hasher(item.name ? item.name : "");
    h = h * 33 + uint64_t(item.socket_type);
    h = h * 33 + uint64_t(item.identifier);
  }
  for (const int i : IndexRange(storage.output_items.items_num)) {
    const NodeShaderHLSLItem &item = storage.output_items.items[i];
    h = h * 33 + hasher(item.name ? item.name : "");
    h = h * 33 + uint64_t(item.socket_type);
    h = h * 33 + uint64_t(item.identifier);
  }
  h = h * 33 + uint64_t(node.identifier);
  return h;
}

/**
 * Build a material-library function in Blender's post-processed GLSL dialect.
 * Out parameters use `_ref(type, name)` (see gpu_shader_compat_glsl.glsl) so the source
 * can be injected without running the shader_tool preprocessor.
 */
static std::string build_glsl_function(const char *func_name,
                                       const NodeShaderHLSL &storage,
                                       const std::string &body_glsl,
                                       const Span<std::string> input_idents,
                                       const Span<std::string> output_idents,
                                       Vector<GPUType> &r_types,
                                       Vector<int> &r_quals)
{
  r_types.clear();
  r_quals.clear();

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
    r_types.append(type);
    r_quals.append(0); /* in */
  };
  auto append_out = [&](const GPUType type, const std::string &ident) {
    if (!first) {
      sig += ", ";
    }
    first = false;
    /* Match preprocessed material dialect: `_ref(float4 ,outcol)`. */
    sig += "_ref(";
    sig += gpu_type_glsl_name(type);
    sig += " ,";
    sig += ident;
    sig += ")";
    r_types.append(type);
    r_quals.append(1); /* out */
  };

  for (const int i : IndexRange(storage.input_items.items_num)) {
    const NodeShaderHLSLItem &item = storage.input_items.items[i];
    append_in(socket_type_to_gpu(item.socket_type), input_idents[i]);
  }
  for (const int i : IndexRange(storage.output_items.items_num)) {
    const NodeShaderHLSLItem &item = storage.output_items.items[i];
    append_out(socket_type_to_gpu(item.socket_type), output_idents[i]);
  }

  sig += ")\n{\n";
  sig += body_glsl;
  if (!body_glsl.empty() && body_glsl.back() != '\n') {
    sig += "\n";
  }
  sig += "}\n";
  return sig;
}

static void collect_param_identifiers(const NodeShaderHLSL &storage,
                                      Vector<std::string> &r_input_idents,
                                      Vector<std::string> &r_output_idents)
{
  r_input_idents.clear();
  r_output_idents.clear();
  Set<std::string> used;
  for (const int i : IndexRange(storage.input_items.items_num)) {
    const NodeShaderHLSLItem &item = storage.input_items.items[i];
    r_input_idents.append(
        uniquify_identifier(glsl_identifier_from_name(item.name, "In", i), used));
  }
  for (const int i : IndexRange(storage.output_items.items_num)) {
    const NodeShaderHLSLItem &item = storage.output_items.items[i];
    r_output_idents.append(
        uniquify_identifier(glsl_identifier_from_name(item.name, "Out", i), used));
  }
}

static int node_shader_gpu_hlsl(GPUMaterial *mat,
                                bNode *node,
                                bNodeExecData * /*execdata*/,
                                GPUNodeStack *in,
                                GPUNodeStack *out)
{
  const NodeShaderHLSL &storage = node_storage(*node);

  if (storage.output_items.items_num == 0) {
    return 0;
  }

  Vector<std::string> input_idents;
  Vector<std::string> output_idents;
  collect_param_identifiers(storage, input_idents, output_idents);

  const uint64_t content_hash = hash_hlsl_signature(storage, *node);
  char func_name[64];
  SNPRINTF(func_name, "node_hlsl_%08x", uint32_t(content_hash));
  const char *stable_name = hlsl_intern_gpu_name(func_name);

  std::string body = storage.code ? storage.code : "";
  if (StringRef(body).trim().is_empty()) {
    /* Safe default when code is empty: zero all outputs. */
    body.clear();
    for (const int i : IndexRange(storage.output_items.items_num)) {
      const GPUType type = socket_type_to_gpu(storage.output_items.items[i].socket_type);
      body += "  ";
      body += output_idents[i];
      body += " = ";
      body += gpu_type_glsl_name(type);
      body += "(0.0);\n";
    }
  }
  else {
    body = translate_hlsl_subset_to_glsl(std::move(body));
    /* Indent body for readability in generated source. */
    if (!body.empty() && body[0] != ' ' && body[0] != '\t') {
      std::string indented = "  ";
      for (const char c : body) {
        indented.push_back(c);
        if (c == '\n') {
          indented += "  ";
        }
      }
      /* Drop trailing indent after final newline. */
      if (indented.size() >= 2 && indented.substr(indented.size() - 2) == "  ") {
        indented.resize(indented.size() - 2);
      }
      body = std::move(indented);
    }
  }

  /* Zero outputs first so missing assignments don't leave garbage (pink / flicker). */
  {
    std::string init;
    for (const int i : IndexRange(storage.output_items.items_num)) {
      const GPUType type = socket_type_to_gpu(storage.output_items.items[i].socket_type);
      init += "  ";
      init += output_idents[i];
      init += " = ";
      init += gpu_type_glsl_name(type);
      init += "(0.0);\n";
    }
    body = init + body;
  }

  Vector<GPUType> param_types;
  Vector<int> param_quals;
  const std::string glsl = build_glsl_function(func_name,
                                               storage,
                                               body,
                                               input_idents,
                                               output_idents,
                                               param_types,
                                               param_quals);

  if (param_types.is_empty()) {
    return 0;
  }

  if (!GPU_material_library_ensure_function(func_name,
                                            glsl.c_str(),
                                            param_types.data(),
                                            param_quals.data(),
                                            param_types.size()))
  {
    return 0;
  }

  /* #GPU_stack_link skips GPU_NONE (Extend sockets) and matches remaining sockets to params. */
  return GPU_stack_link(mat, node, stable_name, in, out) ? 1 : 0;
}

/** \} */

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  /* Multi-line HLSL code box occupies the node body (Geometry Nodes String-style textbox). */
  b.add_default_layout();

  const bNode *node = b.node_or_null();
  const bNodeTree *tree = b.tree_or_null();
  if (!node || !tree) {
    return;
  }

  const NodeShaderHLSL &storage = node_storage(*node);

  /* Right side: user-created output parameters (top to bottom). */
  for (const int i : IndexRange(storage.output_items.items_num)) {
    const NodeShaderHLSLItem &item = storage.output_items.items[i];
    const UString identifier(HLSLOutputItemsAccessor::socket_identifier_for_item(item));
    b.add_output(item.socket_type, UString(item.name), identifier)
        .socket_name_ptr(&tree->id, *HLSLOutputItemsAccessor::item_srna, &item, "name")
        .custom_draw([i](CustomSocketDrawParams &params) {
          socket_items::ui::draw_item_socket_with_remove<HLSLOutputItemsAccessor>(
              params, i, std::nullopt, socket_items::ui::ItemRemoveButtonSide::Left);
        });
  }
  b.add_output<decl::Extend>(""_ustr, "__extend_out__"_ustr)
      .custom_draw(socket_items::ui::draw_extend_socket_fn<HLSLOutputItemsAccessor>());

  /* Left side: user-created input parameters (top to bottom). */
  for (const int i : IndexRange(storage.input_items.items_num)) {
    const NodeShaderHLSLItem &item = storage.input_items.items[i];
    const UString identifier(HLSLInputItemsAccessor::socket_identifier_for_item(item));
    b.add_input(item.socket_type, UString(item.name), identifier)
        .socket_name_ptr(&tree->id, *HLSLInputItemsAccessor::item_srna, &item, "name")
        .custom_draw([i](CustomSocketDrawParams &params) {
          socket_items::ui::draw_item_socket_with_remove<HLSLInputItemsAccessor>(params, i);
        });
  }
  b.add_input<decl::Extend>(""_ustr, "__extend_in__"_ustr)
      .custom_draw(socket_items::ui::draw_extend_socket_fn<HLSLInputItemsAccessor>());
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  bNode &node = *static_cast<bNode *>(ptr->data);
  NodeShaderHLSL &storage = node_storage(node);

  layout.alignment_set(ui::LayoutAlign::Expand);
  layout.textbox_with_state(ptr, "code", &storage.textbox_state, IFACE_("GLSL"));
}

static void node_extra_info(NodeExtraInfoParams &parameters)
{
  const Scene *scene = CTX_data_scene(&parameters.C);
  if (scene && BKE_scene_uses_blender_eevee(scene)) {
    return;
  }
  NodeExtraInfoRow row;
  row.text = IFACE_("EEVEE Only");
  row.tooltip = TIP_("GLSL runs in EEVEE as a material function. Cycles cannot execute GLSL");
  row.icon = ICON_STATUS_INFO;
  parameters.rows.append(std::move(row));
}

static void node_layout_ex(ui::Layout &layout, bContext *C, PointerRNA *ptr)
{
  bNodeTree &tree = *reinterpret_cast<bNodeTree *>(ptr->owner_id);
  bNode &node = *static_cast<bNode *>(ptr->data);

  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);

  if (ui::Layout *panel = layout.panel(C, "input_items", false, IFACE_("Input Parameters"))) {
    socket_items::ui::draw_items_list_with_operators<HLSLInputItemsAccessor>(C, panel, tree, node);
    socket_items::ui::draw_active_item_props<HLSLInputItemsAccessor>(
        tree, node, [&](PointerRNA *item_ptr) {
          panel->use_property_split_set(true);
          panel->use_property_decorate_set(false);
          panel->prop(item_ptr, "socket_type", UI_ITEM_NONE, IFACE_("Type"), ICON_NONE);
        });
  }
  if (ui::Layout *panel = layout.panel(C, "output_items", false, IFACE_("Output Parameters"))) {
    socket_items::ui::draw_items_list_with_operators<HLSLOutputItemsAccessor>(
        C, panel, tree, node);
    socket_items::ui::draw_active_item_props<HLSLOutputItemsAccessor>(
        tree, node, [&](PointerRNA *item_ptr) {
          panel->use_property_split_set(true);
          panel->use_property_decorate_set(false);
          panel->prop(item_ptr, "socket_type", UI_ITEM_NONE, IFACE_("Type"), ICON_NONE);
        });
  }
}

static void node_init(bNodeTree *tree, bNode *node)
{
  NodeShaderHLSL *storage = MEM_new<NodeShaderHLSL>(__func__);
  storage->textbox_state.visible_lines = 8;
  /* Starter: pass Color through. Socket names are GLSL variables.
   * Fac is unused on purpose (unconnected Fac defaults to 0 and would black out Color). */
  storage->code = BLI_strdup("Result = Color;\n");
  node->storage = storage;
  /* Wide enough for a multi-line code body. */
  node->width = 280.0f;

  /* Default sockets for the starter snippet. */
  socket_items::add_item_with_socket_type_and_name<HLSLInputItemsAccessor>(
      *tree, *node, SOCK_RGBA, "Color");
  socket_items::add_item_with_socket_type_and_name<HLSLInputItemsAccessor>(
      *tree, *node, SOCK_FLOAT, "Fac");
  socket_items::add_item_with_socket_type_and_name<HLSLOutputItemsAccessor>(
      *tree, *node, SOCK_RGBA, "Result");
}

static void node_free_storage(bNode *node)
{
  NodeShaderHLSL *storage = static_cast<NodeShaderHLSL *>(node->storage);
  if (storage == nullptr) {
    return;
  }
  socket_items::destruct_array<HLSLInputItemsAccessor>(*node);
  socket_items::destruct_array<HLSLOutputItemsAccessor>(*node);
  MEM_SAFE_DELETE(storage->code);
  MEM_delete(storage);
  node->storage = nullptr;
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeShaderHLSL &src_storage = node_storage(*src_node);
  NodeShaderHLSL *dst_storage = MEM_new<NodeShaderHLSL>(__func__, dna::shallow_copy(src_storage));
  dst_node->storage = dst_storage;

  if (src_storage.code) {
    dst_storage->code = BLI_strdup(src_storage.code);
  }

  socket_items::copy_array<HLSLInputItemsAccessor>(*src_node, *dst_node);
  socket_items::copy_array<HLSLOutputItemsAccessor>(*src_node, *dst_node);
}

static bool node_insert_link(bke::NodeInsertLinkParams &params)
{
  if (params.link.tonode == &params.node) {
    return socket_items::try_add_item_via_any_extend_socket<HLSLInputItemsAccessor>(
        params.ntree, params.node, params.node, params.link);
  }
  return socket_items::try_add_item_via_any_extend_socket<HLSLOutputItemsAccessor>(
      params.ntree, params.node, params.node, params.link);
}

static void node_operators()
{
  socket_items::ops::make_common_operators<HLSLInputItemsAccessor>();
  socket_items::ops::make_common_operators<HLSLOutputItemsAccessor>();
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const bNodeSocket &other_socket = params.other_socket();
  if (!HLSLInputItemsAccessor::supports_socket_type(other_socket.typeinfo->type,
                                                    params.node_tree().type))
  {
    return;
  }

  if (other_socket.in_out == SOCK_IN) {
    params.add_item(IFACE_("Output"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("ShaderNodeHLSL"_ustr);
      const auto *item =
          socket_items::add_item_with_socket_type_and_name<HLSLOutputItemsAccessor>(
              params.node_tree, node, params.socket.typeinfo->type, params.socket.name);
      params.update_and_connect_available_socket(node, UString(item->name));
    });
  }
  else {
    params.add_item(IFACE_("Input"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("ShaderNodeHLSL"_ustr);
      const auto *item = socket_items::add_item_with_socket_type_and_name<HLSLInputItemsAccessor>(
          params.node_tree, node, params.socket.typeinfo->type, params.socket.name);
      nodes::update_node_declaration_and_sockets(params.node_tree, node);
      params.connect_available_socket_by_identifier(
          node, UString(HLSLInputItemsAccessor::socket_identifier_for_item(*item)));
    });
  }
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  const NodeShaderHLSL &storage = node_storage(node);
  writer.write_string(storage.code);
  socket_items::blend_write<HLSLInputItemsAccessor>(&writer, node);
  socket_items::blend_write<HLSLOutputItemsAccessor>(&writer, node);
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  NodeShaderHLSL &storage = node_storage(node);
  BLO_read_string(&reader, &storage.code);
  socket_items::blend_read_data<HLSLInputItemsAccessor>(&reader, node);
  socket_items::blend_read_data<HLSLOutputItemsAccessor>(&reader, node);
}

}  // namespace nodes::node_shader_hlsl_cc

void register_node_type_sh_hlsl()
{
  namespace file_ns = nodes::node_shader_hlsl_cc;

  static bke::bNodeType ntype;

  sh_node_type_base(&ntype, "ShaderNodeHLSL"_ustr, SH_NODE_HLSL);
  ntype.ui_name = "GLSL";
  ntype.ui_description =
      "Write EEVEE GLSL using socket names as variables (float/float3/float4 or vecN). "
      "This is a function body, not a full shader: no main(), no texture() unless you add "
      "a sampler. lerp/saturate/frac/ddx are accepted as HLSL aliases. Cycles ignores this node";
  ntype.enum_name_legacy = "HLSL";
  ntype.nclass = NODE_CLASS_SCRIPT;
  ntype.declare = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.draw_buttons_ex = file_ns::node_layout_ex;
  ntype.initfunc = file_ns::node_init;
  ntype.get_extra_info = file_ns::node_extra_info;
  ntype.insert_link = file_ns::node_insert_link;
  ntype.register_operators = file_ns::node_operators;
  ntype.gather_link_search_ops = file_ns::node_gather_link_searches;
  ntype.blend_write_storage_content = file_ns::node_blend_write;
  ntype.blend_data_read_storage_content = file_ns::node_blend_read;
  ntype.gpu_fn = file_ns::node_shader_gpu_hlsl;
  ntype.add_ui_poll = object_shader_nodes_poll;
  bke::node_type_storage(
      ntype, "NodeShaderHLSL", file_ns::node_free_storage, file_ns::node_copy_storage);

  bke::node_register_type(ntype);
}

namespace nodes {

StructRNA **HLSLInputItemsAccessor::item_srna = &RNA_NodeShaderHLSLInputItem;
StructRNA **HLSLOutputItemsAccessor::item_srna = &RNA_NodeShaderHLSLOutputItem;

void HLSLInputItemsAccessor::blend_write_item(BlendWriter *writer, const ItemT &item)
{
  writer->write_string(item.name);
}

void HLSLInputItemsAccessor::blend_read_data_item(BlendDataReader *reader, ItemT &item)
{
  BLO_read_string(reader, &item.name);
}

void HLSLOutputItemsAccessor::blend_write_item(BlendWriter *writer, const ItemT &item)
{
  writer->write_string(item.name);
}

void HLSLOutputItemsAccessor::blend_read_data_item(BlendDataReader *reader, ItemT &item)
{
  BLO_read_string(reader, &item.name);
}

}  // namespace nodes
}  // namespace blender
