/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <iostream>
#include <sstream>

#include <fmt/format.h>

#include "BLI_dot_export.hh"
#include "BLI_string.hh"
#include "BLI_string_ref.hh"

#include "BLO_read_write.hh"

#include "DNA_node_types.h"

#include "BKE_compute_contexts.hh"
#include "BKE_context.hh"
#include "BKE_node_runtime.hh"
#include "BKE_screen.hh"

#include "ED_node.hh"
#include "ED_undo.hh"

#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"
#include "UI_string_search.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "NOD_expression_complete.hh"
#include "NOD_expression_parse.hh"
#include "NOD_expression_to_nodes.hh"
#include "NOD_geo_expression.hh"
#include "NOD_socket_items_blend.hh"
#include "NOD_socket_items_ops.hh"
#include "NOD_socket_items_ui.hh"

#include "COM_cached_expression_node_group.hh"
#include "COM_node_group_operation.hh"
#include "COM_node_operation.hh"

#include "node_geometry_util.hh"
#include "shader/node_shader_util.hh"

namespace blender::nodes::node_geo_expression_cc {

NODE_STORAGE_FUNCS(NodeExpression)

/* -------------------------------------------------------------------- */
/** \name Expression autocomplete (Houdini VEX-style suggestions)
 * \{ */

/**
 * Must stay POD-friendly: attribute search notes that a custom free function
 * currently breaks SearchMenu activation (cursor never enters the field).
 * free_arg=true + MEM_delete_void — no Vector members.
 *
 * Search list can scroll past 10 visible rows; keep enough slots for a full catalog.
 */
static constexpr int EXPRESSION_SEARCH_ITEMS = 96;
static constexpr int EXPRESSION_STR_MAX = 1024; /* bNodeSocketValueString */

struct ExpressionCompleteData {
  const bNode *node = nullptr;
  bNodeSocket *socket = nullptr;
  ID *owner_id = nullptr;

  /** Expression text at last update (before SearchMenu overwrites editstr on Enter). */
  char expression_snapshot[EXPRESSION_STR_MAX] = {};
  int token_start = 0;
  int token_len = 0;
  bool is_member = false;

  /** What to insert for each list row (index = search poin). Function name only, e.g. "pow". */
  char insert_texts[EXPRESSION_SEARCH_ITEMS][64] = {};
  bool insert_parens[EXPRESSION_SEARCH_ITEMS] = {};
  bool is_header[EXPRESSION_SEARCH_ITEMS] = {};
  /**
   * Full field value after applying this row (snapshot with token → insert_text + optional '(').
   * Never includes usage args like "(a, b)".
   */
  char result_texts[EXPRESSION_SEARCH_ITEMS][EXPRESSION_STR_MAX] = {};
  int insert_num = 0;
};

static int completion_icon(const expression::CompletionKind kind)
{
  switch (kind) {
    case expression::CompletionKind::Variable:
      return ICON_NODE_SOCKET_FLOAT;
    case expression::CompletionKind::Function:
      return ICON_SCRIPT;
    case expression::CompletionKind::Operator:
      return ICON_DRIVER_TRANSFORM;
    case expression::CompletionKind::Member:
      return ICON_DOT;
    case expression::CompletionKind::Keyword:
      return ICON_SYNTAX_ON;
    case expression::CompletionKind::Header:
      return ICON_NONE;
  }
  return ICON_NONE;
}

static int category_icon(const expression::CompletionCategory category)
{
  switch (category) {
    case expression::CompletionCategory::Variable:
      return ICON_NODE_SOCKET_FLOAT;
    case expression::CompletionCategory::Operator:
      return ICON_DRIVER_TRANSFORM;
    case expression::CompletionCategory::Float:
      return ICON_NODE_SOCKET_FLOAT;
    case expression::CompletionCategory::Vector:
      return ICON_NODE_SOCKET_VECTOR;
    case expression::CompletionCategory::Color:
      return ICON_NODE_SOCKET_RGBA;
    case expression::CompletionCategory::Matrix:
      return ICON_NODE_SOCKET_MATRIX;
    case expression::CompletionCategory::Rotation:
      return ICON_NODE_SOCKET_ROTATION;
    case expression::CompletionCategory::Member:
      return ICON_DOT;
  }
  return ICON_NONE;
}

static PointerRNA expression_socket_rna(const ExpressionCompleteData &data)
{
  /* Must be NodeSocketString — default_value lives on the string subtype, not NodeSocket. */
  return RNA_pointer_create_discrete(data.owner_id, RNA_NodeSocketString, data.socket);
}

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
  if (show_all || token.is_empty()) {
    return true;
  }
  /* Case-insensitive prefix match on insert text / name. */
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

/** Search poin cannot be 0 (nullptr); store index+1. */
static void *index_to_poin(const int index)
{
  return POINTER_FROM_INT(index + 1);
}
static int poin_to_index(void *poin)
{
  return poin ? POINTER_AS_INT(poin) - 1 : -1;
}

/**
 * Private SearchItems layout in interface_region_search.cc — used only to detect Tab mode
 * (autocpl != null). Must stay in sync with Blender's SearchItems.
 */
struct SearchItemsAutocplPeek {
  int maxitem, totitem, maxstrlen;
  int offset, offset_i;
  int more;
  char **names;
  void **pointers;
  int *icons;
  int64_t *but_flags;
  uint8_t *name_prefix_offsets;
  bool has_icon;
  void *autocpl;
  void *active;
};

static bool search_items_is_tab_mode(const ui::SearchItems *items)
{
  return reinterpret_cast<const SearchItemsAutocplPeek *>(items)->autocpl != nullptr;
}

static bool add_search_row(ExpressionCompleteData &data,
                           ui::SearchItems *items,
                           const StringRef display,
                           const StringRef insert_text,
                           const bool insert_parens,
                           const bool is_header,
                           const int icon,
                           const int64_t but_flag,
                           const StringRef result_text = {})
{
  if (data.insert_num >= EXPRESSION_SEARCH_ITEMS) {
    return false;
  }
  const int index = data.insert_num++;
  insert_text.copy_utf8_truncated(data.insert_texts[index], sizeof(data.insert_texts[index]));
  data.insert_parens[index] = insert_parens;
  data.is_header[index] = is_header;
  if (!result_text.is_empty()) {
    result_text.copy_utf8_truncated(data.result_texts[index], sizeof(data.result_texts[index]));
  }
  else {
    data.result_texts[index][0] = '\0';
  }

  /* Display = usage only (min(a, b)). Enter applies #result_texts via exec_fn (stops at '('). */
  return search_item_add(items,
                         display,
                         is_header ? nullptr : index_to_poin(index),
                         icon,
                         but_flag,
                         0);
}

static void collect_token_matches(const bNode &node,
                                  const StringRef token,
                                  const bool is_member,
                                  const bool show_all,
                                  Vector<std::string> &r_owned_names,
                                  Vector<expression::CompletionItem> &r_all_items,
                                  Vector<const expression::CompletionItem *> &r_matches)
{
  r_owned_names.clear();
  r_all_items.clear();
  r_matches.clear();
  expression::gather_completions(node, r_owned_names, r_all_items);
  for (const expression::CompletionItem &item : r_all_items) {
    if (item_matches_token(item, token, is_member, show_all)) {
      r_matches.append(&item);
    }
  }
}

/**
 * Build the string Tab should write: expand token to insert_text (+ optional '(').
 * Multi-match → longest common prefix of insert names; if that does not advance past the
 * token (e.g. transform_point vs transform_direction → "transform_"), take the first match
 * fully so Tab is not stuck on '_'.
 */
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

  /* LCP of insert_text among matches. */
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
  /* No progress (or stuck at '_') → complete first match fully. */
  if (lcp.size() <= token.size()) {
    return expression::apply_completion(expression,
                                        token_start,
                                        token_len,
                                        matches[0]->insert_text,
                                        matches[0]->insert_call_parens);
  }

  /* Partial unique prefix (no paren yet — more than one name still shares it). */
  return expression::apply_completion(expression, token_start, token_len, lcp, false);
}

static void expression_search_update_fn(
    const bContext * /*C*/, void *arg, const char *str, ui::SearchItems *items, const bool is_first)
{
  ExpressionCompleteData &data = *static_cast<ExpressionCompleteData *>(arg);
  data.insert_num = 0;

  if (!data.node || !data.socket) {
    return;
  }

  const StringRef expression = str ? str : "";
  BLI_strncpy(data.expression_snapshot, expression.data(), sizeof(data.expression_snapshot));
  if (expression.size() >= sizeof(data.expression_snapshot)) {
    data.expression_snapshot[sizeof(data.expression_snapshot) - 1] = '\0';
  }

  data.token_start = 0;
  data.token_len = 0;
  data.is_member = false;
  if (!expression::find_completion_token(
          expression, data.token_start, data.token_len, data.is_member))
  {
    return;
  }
  const StringRef token = expression.substr(data.token_start, data.token_len);
  const bool show_all = is_first || token.is_empty();

  Vector<std::string> owned_names;
  Vector<expression::CompletionItem> all_items;
  Vector<const expression::CompletionItem *> matches;
  collect_token_matches(
      *data.node, token, data.is_member, show_all, owned_names, all_items, matches);

  /* ---- Tab mode: SearchMenu hijacks update to feed AutoComplete (not the visible list). ---- */
  if (search_items_is_tab_mode(items)) {
    if (token.is_empty() || matches.is_empty()) {
      return;
    }
    /* Dedupe by insert_text so * / m*v don't spam. */
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
    const std::string result = tab_complete_result(
        expression, data.token_start, data.token_len, unique);
    /* Register twice → matches>=2 → PARTIAL_MATCH so UI does not exit text edit (pow bug). */
    search_item_add(items, result, nullptr, ICON_NONE, 0, 0);
    search_item_add(items, result, nullptr, ICON_NONE, 0, 0);
    return;
  }

  /* ---- Normal list: category panels, usage-only labels. ---- */
  const expression::CompletionCategory category_order[] = {
      expression::CompletionCategory::Variable,
      expression::CompletionCategory::Operator,
      expression::CompletionCategory::Float,
      expression::CompletionCategory::Vector,
      expression::CompletionCategory::Color,
      expression::CompletionCategory::Matrix,
      expression::CompletionCategory::Rotation,
      expression::CompletionCategory::Member,
  };

  for (const expression::CompletionCategory cat : category_order) {
    if (data.is_member && cat != expression::CompletionCategory::Member) {
      continue;
    }
    if (!data.is_member && cat == expression::CompletionCategory::Member) {
      continue;
    }

    Vector<const expression::CompletionItem *> in_cat;
    for (const expression::CompletionItem *item : matches) {
      if (item->category == cat) {
        in_cat.append(item);
      }
    }
    if (in_cat.is_empty()) {
      continue;
    }

    const std::string header = fmt::format("── {} ──", expression::category_label(cat));
    if (!add_search_row(data,
                        items,
                        header,
                        "",
                        false,
                        true,
                        category_icon(cat),
                        ui::BUT_DISABLED | ui::BUT_INACTIVE))
    {
      return;
    }

    for (const expression::CompletionItem *item : in_cat) {
      /* List shows usage only: min(a, b), inv(m). Field gets insert up to '('. */
      const StringRef display = item->usage.is_empty() ? item->name : item->usage;
      const std::string result = expression::apply_completion(expression,
                                                              data.token_start,
                                                              data.token_len,
                                                              item->insert_text,
                                                              item->insert_call_parens);
      if (!add_search_row(data,
                          items,
                          display,
                          item->insert_text,
                          item->insert_call_parens,
                          false,
                          completion_icon(item->kind),
                          0,
                          result))
      {
        return;
      }
    }
  }
}

static void expression_search_exec_fn(bContext *C, void *arg_v, void *item_v)
{
  ExpressionCompleteData &data = *static_cast<ExpressionCompleteData *>(arg_v);
  PointerRNA socket_ptr = expression_socket_rna(data);

  const int index = poin_to_index(item_v);
  const bool valid = index >= 0 && index < data.insert_num && !data.is_header[index] &&
                     data.insert_texts[index][0] != '\0';

  if (!valid) {
    /* Keep free-typed text from button_string_set. */
    return;
  }

  /* Prefer precomputed result (stops at '(' — never "pow(a, b)"). */
  const char *result = data.result_texts[index];
  std::string result_fallback;
  if (result[0] == '\0') {
    result_fallback = expression::apply_completion(data.expression_snapshot,
                                                   data.token_start,
                                                   data.token_len,
                                                   data.insert_texts[index],
                                                   data.insert_parens[index]);
    result = result_fallback.c_str();
  }

  /* Write DNA first so button_string_get can reload even if RNA type resolution fails. */
  if (data.socket && data.socket->default_value) {
    auto *str_val = static_cast<bNodeSocketValueString *>(data.socket->default_value);
    BLI_strncpy(str_val->value, result, sizeof(str_val->value));
  }
  /* Also set via RNA for consistency with the button property. Avoid notifiers/undo here:
   * Enter stays in text-edit — NC_NODE / undo would rebuild UI and kill the edit session. */
  RNA_string_set(&socket_ptr, "default_value", result);
  (void)C; /* No notifier/undo — caller stays in text-edit. */
}

/** Tab when searchbox is not open (fallback). Never returns FULL_MATCH so edit mode stays open. */
static int expression_autocomplete_fn(bContext * /*C*/, char *str, void *arg_v)
{
  ExpressionCompleteData &data = *static_cast<ExpressionCompleteData *>(arg_v);
  if (!data.node || !str) {
    return AUTOCOMPLETE_NO_MATCH;
  }

  const StringRef expression = str;
  int token_start = 0;
  int token_len = 0;
  bool is_member = false;
  if (!expression::find_completion_token(expression, token_start, token_len, is_member)) {
    return AUTOCOMPLETE_NO_MATCH;
  }
  if (token_len == 0) {
    return AUTOCOMPLETE_NO_MATCH;
  }

  Vector<std::string> owned_names;
  Vector<expression::CompletionItem> all_items;
  Vector<const expression::CompletionItem *> matches;
  const StringRef token = expression.substr(token_start, token_len);
  collect_token_matches(
      *data.node, token, is_member, false, owned_names, all_items, matches);
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
  BLI_strncpy(str, result.c_str(), EXPRESSION_STR_MAX);
  /* Always PARTIAL so Blender does not exit text-edit after a unique match (pow bug). */
  return AUTOCOMPLETE_PARTIAL_MATCH;
}

static void draw_expression_string_socket(CustomSocketDrawParams &params)
{
  ui::Layout &layout = params.layout;
  ui::Block *block = layout.block();

  ui::Button *but = uiDefIconTextButR(block,
                                      ui::ButtonType::SearchMenu,
                                      ICON_NONE,
                                      "",
                                      0,
                                      0,
                                      10 * UI_UNIT_X,
                                      UI_UNIT_Y,
                                      &params.socket_ptr,
                                      "default_value",
                                      0,
                                      TIP_("Expression — browse ops by category; Enter inserts syntax only"));

  /* Same allocation pattern as attribute search: free_arg=true, free_fn=nullptr. */
  ExpressionCompleteData *data = static_cast<ExpressionCompleteData *>(
      MEM_new_uninitialized(sizeof(ExpressionCompleteData), __func__));
  *data = ExpressionCompleteData{};
  data->node = &params.node;
  data->socket = &params.socket;
  data->owner_id = params.socket_ptr.owner_id;

  button_func_search_set_results_are_suggestions(but, true);
  /* Split display name at UI_SEP_CHAR so category hint is not written into the field. */
  /* (use_shortcut_sep is operator-only; exec_fn still rebuilds from insert_texts + snapshot.) */
  button_func_search_set_sep_string(but, UI_MENU_ARROW_SEP);
  button_func_search_set(but,
                         nullptr,
                         expression_search_update_fn,
                         data,
                         true,
                         nullptr,
                         expression_search_exec_fn,
                         nullptr);
  button_func_complete_set(but, expression_autocomplete_fn, data);
}

/** \} */

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  const bNode *node = b.node_or_null();
  const bNodeTree *tree = b.tree_or_null();
  if (!node || !tree) {
    return;
  }
  const NodeExpression &storage = node_storage(*node);

  for (const int i : IndexRange(storage.expression_items.items_num)) {
    const NodeExpressionItem &item = storage.expression_items.items[i];
    const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
    const UString identifier{ExpressionItemsAccessor::socket_identifier_for_item(item)};
    const UString name{item.name};
    b.add_input<decl::String>(name, identifier)
        .optional_label()
        .description("Expression to be evaluated (Tab / type for autocomplete)")
        .custom_draw([i](CustomSocketDrawParams &params) {
          socket_items::ui::draw_item_socket_with_remove<ExpressionItemsAccessor>(
              params, i, socket_items::ui::ItemRemoveButtonSide::Right, draw_expression_string_socket);
        });
    b.add_output(socket_type, name, identifier)
        .align_with_previous()
        .propagate_all()
        .structure_type(StructureType::Dynamic);
  }
  b.add_input<decl::Extend>(""_ustr, "__extend__expression_input"_ustr)
      .structure_type(StructureType::Dynamic)
      .custom_draw(socket_items::ui::draw_extend_socket_fn<ExpressionItemsAccessor>());
  b.add_output<decl::Extend>(""_ustr, "__extend__expression_output"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous();

  auto &inputs_panel = b.add_panel("Inputs"_ustr);
  for (const int i : IndexRange(storage.input_items.items_num)) {
    const NodeExpressionInputItem &item = storage.input_items.items[i];
    const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
    const UString identifier{ExpressionInputItemsAccessor::socket_identifier_for_item(item)};
    const UString name{item.name};
    inputs_panel.add_input(socket_type, name, identifier)
        .socket_name_ptr(&tree->id, *ExpressionInputItemsAccessor::item_srna, &item, "name")
        .structure_type(StructureType::Dynamic)
        .custom_draw([i](CustomSocketDrawParams &params) {
          socket_items::ui::draw_item_socket_with_remove<ExpressionInputItemsAccessor>(params, i);
        });
  }
  inputs_panel.add_input<decl::Extend>(""_ustr, "__extend__input"_ustr)
      .structure_type(StructureType::Dynamic)
      .custom_draw(socket_items::ui::draw_extend_socket_fn<ExpressionInputItemsAccessor>());
}

static void node_layout_ex(ui::Layout &layout, bContext *C, PointerRNA *ptr)
{
  bNodeTree &ntree = *id_cast<bNodeTree *>(ptr->owner_id);
  bNode &node = *ptr->data_as<bNode>();
  if (ui::Layout *panel = layout.panel(C, "expression_items", false, IFACE_("Expression Items"))) {
    socket_items::ui::draw_items_list_with_operators<ExpressionItemsAccessor>(
        C, panel, ntree, node);
    socket_items::ui::draw_active_item_props<ExpressionItemsAccessor>(
        ntree, node, [&](PointerRNA *item_ptr) {
          panel->use_property_split_set(true);
          panel->use_property_decorate_set(false);
          panel->prop(item_ptr, "socket_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
        });
  }
  if (ui::Layout *panel = layout.panel(C, "input_items", false, IFACE_("Input Items"))) {
    socket_items::ui::draw_items_list_with_operators<ExpressionInputItemsAccessor>(
        C, panel, ntree, node);
    socket_items::ui::draw_active_item_props<ExpressionInputItemsAccessor>(
        ntree, node, [&](PointerRNA *item_ptr) {
          panel->use_property_split_set(true);
          panel->use_property_decorate_set(false);
          panel->prop(item_ptr, "socket_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
        });
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  auto *storage = MEM_new<NodeExpression>(__func__);
  node->storage = storage;

  storage->expression_items.items = MEM_new_array<NodeExpressionItem>(1, __func__);
  NodeExpressionItem &item = storage->expression_items.items[0];
  item.name = BLI_strdup(DATA_("Expression"));
  item.socket_type = SOCK_RGBA;
  item.identifier = storage->expression_items.next_identifier++;
  storage->expression_items.items_num = 1;
}

static void node_free_storage(bNode *node)
{
  socket_items::destruct_array<ExpressionInputItemsAccessor>(*node);
  socket_items::destruct_array<ExpressionItemsAccessor>(*node);
  NodeExpression &storage = node_storage(*node);
  MEM_delete(&storage);
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeExpression &src_storage = node_storage(*src_node);
  auto *dst_storage = MEM_new<NodeExpression>(__func__, dna::shallow_copy(src_storage));
  dst_node->storage = dst_storage;

  socket_items::copy_array<ExpressionInputItemsAccessor>(*src_node, *dst_node);
  socket_items::copy_array<ExpressionItemsAccessor>(*src_node, *dst_node);
}

static void node_operators()
{
  socket_items::ops::make_common_operators<ExpressionInputItemsAccessor>();
  socket_items::ops::make_common_operators<ExpressionItemsAccessor>();
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  socket_items::blend_write<ExpressionInputItemsAccessor>(&writer, node);
  socket_items::blend_write<ExpressionItemsAccessor>(&writer, node);
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  socket_items::blend_read_data<ExpressionInputItemsAccessor>(&reader, node);
  socket_items::blend_read_data<ExpressionItemsAccessor>(&reader, node);
}

static bool node_insert_link(bke::NodeInsertLinkParams &params)
{
  if (!socket_items::try_add_item_via_any_extend_socket<ExpressionItemsAccessor>(
          params.ntree, params.node, params.node, params.link, "__extend__expression_input"))
  {
    return false;
  }
  if (!socket_items::try_add_item_via_any_extend_socket<ExpressionItemsAccessor>(
          params.ntree, params.node, params.node, params.link, "__extend__expression_output"))
  {
    return false;
  }
  return socket_items::try_add_item_via_any_extend_socket<ExpressionInputItemsAccessor>(
      params.ntree, params.node, params.node, params.link);
}

using namespace blender::compositor;

class ExpressionOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    const expression::ExpressionNodeGroup &expression_node_group =
        this->get_expression_node_group();
    if (!expression_node_group.tree) {
      /* TODO: Error message. */
      this->allocate_default_remaining_outputs();
      return;
    }

    const bNodeTree &node_group = *expression_node_group.tree;

    const bke::GroupNodeComputeContext compute_context(
        &this->get_compute_context(), this->node().identifier, &this->node().owner_tree());
    NodeGroupOperation operation(
        this->context(), node_group, NodeGroupOutputTypes::None, compute_context);

    this->set_reference_counts(operation, node_group);
    Vector<std::unique_ptr<Result>> temporary_inputs = this->map_inputs(operation, node_group);
    operation.evaluate();
    this->write_outputs(operation, node_group);
  }

  /* Sets the reference counts of the node group operation according to the needed status of the
   * outputs of the expression node. */
  void set_reference_counts(Operation &operation, const bNodeTree &node_group)
  {
    const NodeExpression &storage = node_storage(this->node());
    node_group.ensure_interface_cache();

    for (const int i : IndexRange(storage.expression_items.items_num)) {
      const NodeExpressionItem &item = storage.expression_items.items[i];
      const std::string identifier = ExpressionItemsAccessor::socket_identifier_for_item(item);

      const bNodeTreeInterfaceSocket *interface_socket = node_group.interface_outputs()[i];

      Result &node_group_result = operation.get_result(interface_socket->identifier);
      Result &expression_node_result = this->get_result(identifier);
      node_group_result.set_reference_count(expression_node_result.should_compute() ? 1 : 0);
    }
  }

  /* Maps the input results of the node group operation to this expression node's inputs through
   * temporary results that share the data of the this group's inputs. */
  Vector<std::unique_ptr<Result>> map_inputs(Operation &operation, const bNodeTree &node_group)
  {
    const NodeExpression &storage = node_storage(this->node());
    node_group.ensure_interface_cache();

    Vector<std::unique_ptr<Result>> temporary_inputs;
    for (const int i : IndexRange(storage.input_items.items_num)) {
      const NodeExpressionInputItem &item = storage.input_items.items[i];
      const std::string identifier = ExpressionInputItemsAccessor::socket_identifier_for_item(
          item);

      const bNodeTreeInterfaceSocket *interface_socket = node_group.interface_inputs()[i];

      const Result &input_result = this->get_input(identifier);
      std::unique_ptr<Result> temporary_input = std::make_unique<Result>(
          this->context().create_result(input_result.type(), input_result.precision()));
      temporary_input->share_data(input_result);
      temporary_inputs.append(std::move(temporary_input));
      operation.map_input_to_result(interface_socket->identifier, temporary_inputs.last().get());
    }

    return temporary_inputs;
  }

  /* Writes the output results of the node group operation to this expression node operation by
   * sharing its data and freeing the results. */
  void write_outputs(Operation &operation, const bNodeTree &node_group)
  {
    const NodeExpression &storage = node_storage(this->node());
    node_group.ensure_interface_cache();

    for (const int i : IndexRange(storage.expression_items.items_num)) {
      const NodeExpressionItem &item = storage.expression_items.items[i];
      const std::string identifier = ExpressionItemsAccessor::socket_identifier_for_item(item);

      const bNodeTreeInterfaceSocket *interface_socket = node_group.interface_outputs()[i];

      Result &node_group_result = operation.get_result(interface_socket->identifier);
      Result &expression_node_result = this->get_result(identifier);
      if (expression_node_result.should_compute()) {
        expression_node_result.share_data(node_group_result);
        node_group_result.release();
      }
    }
  }

  const expression::ExpressionNodeGroup &get_expression_node_group()
  {
    const NodeExpression &storage = node_storage(this->node());

    Array<std::string> expressions(storage.expression_items.items_num);
    for (const int i : IndexRange(storage.expression_items.items_num)) {
      const NodeExpressionItem &item = storage.expression_items.items[i];
      const std::string identifier = ExpressionItemsAccessor::socket_identifier_for_item(item);
      expressions[i] = this->get_input(identifier).get_single_value_default<std::string>();
    }

    Array<StringRef> expressions_references(storage.expression_items.items_num);
    for (const int i : IndexRange(storage.expression_items.items_num)) {
      expressions_references[i] = expressions[i];
    }

    return this->context().cache_manager().expression_node_groups.get(this->node(),
                                                                      expressions_references);
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new ExpressionOperation(context, node);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  common_node_type_base(&ntype, "NodeExpression"_ustr);
  ntype.ui_name = "Expression";
  ntype.ui_description = "Evaluate an expression on inputs";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  blender::bke::node_type_storage(ntype, "NodeExpression", node_free_storage, node_copy_storage);
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  ntype.register_operators = node_operators;
  ntype.draw_buttons_ex = node_layout_ex;
  ntype.insert_link = node_insert_link;
  ntype.get_compositor_operation = get_compositor_operation;
  blender::bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_expression_cc

namespace blender::nodes {

StructRNA **ExpressionInputItemsAccessor::item_srna = &RNA_NodeExpressionInputItem;
StructRNA **ExpressionItemsAccessor::item_srna = &RNA_NodeExpressionItem;

void ExpressionInputItemsAccessor::blend_write_item(BlendWriter *writer, const ItemT &item)
{
  writer->write_string(item.name);
}

void ExpressionInputItemsAccessor::blend_read_data_item(BlendDataReader *reader, ItemT &item)
{
  BLO_read_string(reader, &item.name);
}

void ExpressionItemsAccessor::blend_write_item(BlendWriter *writer, const ItemT &item)
{
  writer->write_string(item.name);
}

void ExpressionItemsAccessor::blend_read_data_item(BlendDataReader *reader, ItemT &item)
{
  BLO_read_string(reader, &item.name);
}

}  // namespace blender::nodes
