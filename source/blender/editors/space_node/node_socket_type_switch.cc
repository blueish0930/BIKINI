/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 *
 * Shift+LMB on a socket shape: data-type popup (group interface, Capture Attribute,
 * Expression, Format String, Bundle, Closure, HLSL, etc.).
 * Alt+LMB on a socket shape: structure/shape popup (Auto / Single / Field / Grid / List).
 *
 * Both use hold-time (not drag distance): short press (<0.05s) → sticky panel;
 * longer press → hold/release menu (slide to select).
 */

#include <climits>
#include <optional>

#include "MEM_guardedalloc.h"

#include "DNA_node_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_userdef_types.h"
#include "DNA_windowmanager_types.h"

#include "BLI_listbase.hh"
#include "BLI_string.hh"
#include "BLI_string_ref.hh"
#include "BLI_time.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"

#include "BKE_context.hh"
#include "BKE_main_invariants.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_interface.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_report.hh"
#include "BKE_wm_runtime.hh"

#include "BLT_translation.hh"

#include "ED_node.hh"
#include "ED_screen.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"
#include "RNA_prototypes.hh"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"
#include "UI_view2d.hh"

#include "WM_api.hh"
#include "WM_keymap.hh"
#include "WM_types.hh"

#include "NOD_fn_format_string.hh"
#include "NOD_geo_bundle.hh"
#include "NOD_geo_capture_attribute.hh"
#include "NOD_geo_closure.hh"
#include "NOD_geo_expression.hh"
#include "NOD_shader_hlsl.hh"
#include "NOD_shader_wrangle.hh"
#include "NOD_socket.hh"
#include "NOD_socket_items.hh"

#include "node_intern.hh"

namespace blender::ed::space_node {

/* -------------------------------------------------------------------- */
/** \name Resolve socket under cursor to a type-editable RNA target
 * \{ */

enum class SocketTypeTargetKind {
  /** #bNodeTreeInterfaceSocket::socket_type (idname-backed RNA enum). */
  Interface,
  /** Capture item #data_type (CustomDataType). */
  CaptureDataType,
  /** Generic item / storage #socket_type (eNodeSocketDatatype). */
  SocketDatatype,
};

struct SocketTypeTarget {
  SocketTypeTargetKind kind = SocketTypeTargetKind::SocketDatatype;
  PointerRNA ptr = {};
  /** RNA property name on #ptr. */
  const char *prop_name = nullptr;
  /** Tree that owns the data (group tree for group sockets, edit tree otherwise). */
  bNodeTree *owner_tree = nullptr;
  /** Node that needs a property-tag update for item arrays; null for interface. */
  bNode *owner_node = nullptr;
  /** Current type as #eNodeSocketDatatype for menu default / filtering. */
  eNodeSocketDatatype current_socket_type = SOCK_FLOAT;
  int ntree_type = 0;
  /** Optional supports filter; when set, menu items must pass. */
  bool (*supports_type)(eNodeSocketDatatype type, int ntree_type) = nullptr;
};

static bool interface_supports_socket_datatype(const bNodeTree &tree,
                                               const eNodeSocketDatatype type)
{
  return bke::node_tree_type_supports_socket_type_static(tree.type, type);
}

static std::optional<SocketTypeTarget> resolve_group_interface(bNodeTree &edit_tree,
                                                               bNode &node,
                                                               bNodeSocket &socket)
{
  bNodeTree *interface_tree = nullptr;
  const bNodeTreeInterfaceSocket *io_socket = nullptr;

  if (node.is_group()) {
    if (node.id == nullptr || GS(node.id->name) != ID_NT) {
      return std::nullopt;
    }
    interface_tree = reinterpret_cast<bNodeTree *>(node.id);
    if (socket.is_input()) {
      io_socket = bke::node_find_interface_input_by_identifier(*interface_tree, socket.identifier);
    }
    else {
      io_socket = bke::node_find_interface_output_by_identifier(*interface_tree, socket.identifier);
    }
  }
  else if (node.is_group_input()) {
    interface_tree = &edit_tree;
    /* Group Input exposes interface inputs as outputs. */
    io_socket = bke::node_find_interface_input_by_identifier(*interface_tree, socket.identifier);
  }
  else if (node.is_group_output()) {
    interface_tree = &edit_tree;
    /* Group Output exposes interface outputs as inputs. */
    io_socket = bke::node_find_interface_output_by_identifier(*interface_tree, socket.identifier);
  }
  else {
    return std::nullopt;
  }

  if (interface_tree == nullptr || io_socket == nullptr) {
    return std::nullopt;
  }

  const bke::bNodeSocketType *stype = io_socket->socket_typeinfo();
  if (stype == nullptr) {
    return std::nullopt;
  }

  SocketTypeTarget target;
  target.kind = SocketTypeTargetKind::Interface;
  target.owner_tree = interface_tree;
  target.owner_node = nullptr;
  target.ptr = RNA_pointer_create_discrete(
      &interface_tree->id,
      RNA_NodeTreeInterfaceSocket,
      const_cast<bNodeTreeInterfaceSocket *>(io_socket));
  target.prop_name = "socket_type";
  target.current_socket_type = stype->type;
  target.ntree_type = interface_tree->type;
  target.supports_type = [](eNodeSocketDatatype type, int ntree_type) {
    return bke::node_tree_type_supports_socket_type_static(ntree_type, type);
  };
  return target;
}

template<typename Accessor>
static typename Accessor::ItemT *find_item_for_socket_identifier(bNode &node,
                                                                const StringRef identifier)
{
  using ItemT = typename Accessor::ItemT;
  nodes::socket_items::SocketItemsRef<ItemT> ref = Accessor::get_items_from_node(node);
  for (const int i : IndexRange(*ref.items_num)) {
    ItemT &item = (*ref.items)[i];
    if constexpr (Accessor::has_single_identifier_str) {
      if (Accessor::socket_identifier_for_item(item) == identifier) {
        return &item;
      }
    }
    else {
      if (Accessor::input_socket_identifier_for_item(item) == identifier ||
          Accessor::output_socket_identifier_for_item(item) == identifier)
      {
        return &item;
      }
    }
  }
  return nullptr;
}

template<typename Accessor>
static std::optional<SocketTypeTarget> resolve_item_accessor(bNodeTree &tree,
                                                             bNode &storage_node,
                                                             bNodeSocket &socket,
                                                             const char *prop_name,
                                                             const SocketTypeTargetKind kind)
{
  using ItemT = typename Accessor::ItemT;
  ItemT *item = find_item_for_socket_identifier<Accessor>(storage_node, socket.identifier);
  if (item == nullptr) {
    return std::nullopt;
  }
  if (Accessor::item_srna == nullptr || *Accessor::item_srna == nullptr) {
    return std::nullopt;
  }

  SocketTypeTarget target;
  target.kind = kind;
  target.owner_tree = &tree;
  target.owner_node = &storage_node;
  target.ptr = RNA_pointer_create_discrete(&tree.id, *Accessor::item_srna, item);
  target.prop_name = prop_name;
  target.current_socket_type = Accessor::get_socket_type(*item);
  target.ntree_type = tree.type;
  target.supports_type = [](eNodeSocketDatatype type, int ntree_type) {
    return Accessor::supports_socket_type(type, ntree_type);
  };
  return target;
}

static std::optional<SocketTypeTarget> resolve_socket_items(bNodeTree &tree,
                                                            bNode &node,
                                                            bNodeSocket &socket)
{
  using namespace nodes;

  /* Capture Attribute: input Value_* and output Attribute_* share one item. */
  if (node.is_type("GeometryNodeCaptureAttribute"_ustr)) {
    return resolve_item_accessor<CaptureAttributeItemsAccessor>(
        tree, node, socket, "data_type", SocketTypeTargetKind::CaptureDataType);
  }

  /* Expression inputs + outputs. */
  if (node.is_type("NodeExpression"_ustr)) {
    if (auto t = resolve_item_accessor<ExpressionInputItemsAccessor>(
            tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype))
    {
      return t;
    }
    return resolve_item_accessor<ExpressionItemsAccessor>(
        tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype);
  }

  /* Format String items (Float / Integer / String only). */
  if (node.is_type("FunctionNodeFormatString"_ustr)) {
    return resolve_item_accessor<FormatStringItemsAccessor>(
        tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype);
  }

  /* Bundle. */
  if (node.is_type("NodeCombineBundle"_ustr)) {
    return resolve_item_accessor<CombineBundleItemsAccessor>(
        tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype);
  }
  if (node.is_type("NodeSeparateBundle"_ustr)) {
    return resolve_item_accessor<SeparateBundleItemsAccessor>(
        tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype);
  }

  /* Evaluate Closure (single node, items on self). */
  if (node.is_type("NodeEvaluateClosure"_ustr)) {
    if (auto t = resolve_item_accessor<EvaluateClosureInputItemsAccessor>(
            tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype))
    {
      return t;
    }
    return resolve_item_accessor<EvaluateClosureOutputItemsAccessor>(
        tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype);
  }

  /* Closure Input: items live on paired Closure Output. */
  if (node.is_type("NodeClosureInput"_ustr)) {
    const auto &storage = *static_cast<const NodeClosureInput *>(node.storage);
    bNode *output_node = tree.node_by_id(storage.output_node_id);
    if (output_node == nullptr) {
      return std::nullopt;
    }
    return resolve_item_accessor<ClosureInputItemsAccessor>(
        tree, *output_node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype);
  }

  /* Closure Output: output-items on self (zone border inputs). */
  if (node.is_type("NodeClosureOutput"_ustr)) {
    if (auto t = resolve_item_accessor<ClosureOutputItemsAccessor>(
            tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype))
    {
      return t;
    }
    /* Also allow changing input-item types when those sockets appear on the output node. */
    return resolve_item_accessor<ClosureInputItemsAccessor>(
        tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype);
  }

  /* Shader HLSL: user-defined inputs / outputs (Float/Int/Bool/Vector/Color). */
  if (node.is_type("ShaderNodeHLSL"_ustr)) {
    if (auto t = resolve_item_accessor<HLSLInputItemsAccessor>(
            tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype))
    {
      return t;
    }
    return resolve_item_accessor<HLSLOutputItemsAccessor>(
        tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype);
  }

  if (node.is_type("ShaderNodeWrangle"_ustr)) {
    if (auto t = resolve_item_accessor<ShaderWrangleOutputItemsAccessor>(
            tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype))
    {
      return t;
    }
    if (auto t = resolve_item_accessor<ShaderWrangleFieldItemsAccessor>(
            tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype))
    {
      return t;
    }
    return resolve_item_accessor<ShaderWrangleInputItemsAccessor>(
        tree, node, socket, "socket_type", SocketTypeTargetKind::SocketDatatype);
  }

  return std::nullopt;
}

static bool socket_is_extend_identifier(const char *identifier)
{
  return STREQ(identifier, "__extend__") || STREQ(identifier, "__extend__out") ||
         STREQ(identifier, "__extend_in__") || STREQ(identifier, "__extend_out__") ||
         STREQ(identifier, "__extend__wrangle_field") ||
         STREQ(identifier, "__extend__wrangle_out");
}

static std::optional<SocketTypeTarget> resolve_socket_type_target(bNodeTree &tree,
                                                                  bNode &node,
                                                                  bNodeSocket &socket)
{
  /* Skip virtual / extend sockets. */
  if (socket_is_extend_identifier(socket.identifier)) {
    return std::nullopt;
  }

  if (auto t = resolve_group_interface(tree, node, socket)) {
    return t;
  }
  return resolve_socket_items(tree, node, socket);
}

static std::optional<SocketTypeTarget> resolve_from_cursor(bContext *C, const int2 mval)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  if (snode == nullptr || snode->edittree == nullptr || region == nullptr) {
    return std::nullopt;
  }
  if (!ED_operator_node_editable(C)) {
    return std::nullopt;
  }

  bNodeTree &tree = *snode->edittree;
  tree.ensure_topology_cache();

  float2 cursor;
  ui::view2d_region_to_view(&region->v2d, mval.x, mval.y, &cursor.x, &cursor.y);

  bNodeSocket *sock = node_find_indicated_socket(*snode, *region, cursor, SOCK_IN | SOCK_OUT);
  if (sock == nullptr || sock->runtime == nullptr || sock->runtime->owner_node == nullptr) {
    return std::nullopt;
  }
  bNode &node = *sock->runtime->owner_node;
  return resolve_socket_type_target(tree, node, *sock);
}

static std::optional<SocketTypeTarget> resolve_from_op_props(bContext *C, PointerRNA *op_ptr)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (snode == nullptr || snode->edittree == nullptr) {
    return std::nullopt;
  }
  bNodeTree &tree = *snode->edittree;
  tree.ensure_topology_cache();

  const int node_identifier = RNA_int_get(op_ptr, "node_identifier");
  char socket_identifier[MAX_NAME];
  RNA_string_get(op_ptr, "socket_identifier", socket_identifier);

  bNode *node = tree.node_by_id(node_identifier);
  if (node == nullptr) {
    return std::nullopt;
  }
  const UString sock_id(socket_identifier);
  bNodeSocket *sock = bke::node_find_socket(*node, SOCK_IN, sock_id);
  if (sock == nullptr) {
    sock = bke::node_find_socket(*node, SOCK_OUT, sock_id);
  }
  if (sock == nullptr) {
    return std::nullopt;
  }
  return resolve_socket_type_target(tree, *node, *sock);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Apply type change via existing RNA / DNA paths
 * \{ */

static bool apply_socket_type(bContext *C, SocketTypeTarget &target, const eNodeSocketDatatype type)
{
  Main *bmain = CTX_data_main(C);
  if (target.supports_type && !target.supports_type(type, target.ntree_type)) {
    return false;
  }

  switch (target.kind) {
    case SocketTypeTargetKind::Interface: {
      /* Must use NodeSocket* idnames (e.g. NodeSocketFloat), same as RNA/Python.
       * NodeTreeInterfaceSocket* idnames from node_static_socket_interface_type_new()
       * are rejected / do not update group instances correctly. */
      const std::optional<StringRefNull> idname = bke::node_static_socket_type(type, PROP_NONE);
      if (!idname) {
        return false;
      }
      auto *io = static_cast<bNodeTreeInterfaceSocket *>(target.ptr.data);
      if (!io->set_socket_type(*idname)) {
        return false;
      }
      target.owner_tree->tree_interface.tag_item_property_changed();
      BKE_main_ensure_invariants(*bmain, target.owner_tree->id);
      /* Also refresh the currently edited tree (outer graph with group instances). */
      if (SpaceNode *snode = CTX_wm_space_node(C)) {
        if (snode->edittree && snode->edittree != target.owner_tree) {
          BKE_main_ensure_invariants(*bmain, snode->edittree->id);
          WM_main_add_notifier(NC_NODE | NA_EDITED, snode->edittree);
        }
      }
      WM_main_add_notifier(NC_NODE | NA_EDITED, target.owner_tree);
      return true;
    }
    case SocketTypeTargetKind::CaptureDataType: {
      const std::optional<eCustomDataType> cd = bke::socket_type_to_custom_data_type(type);
      if (!cd) {
        return false;
      }
      PropertyRNA *prop = RNA_struct_find_property(&target.ptr, target.prop_name);
      if (prop == nullptr) {
        return false;
      }
      RNA_property_enum_set(&target.ptr, prop, int(*cd));
      RNA_property_update(C, &target.ptr, prop);
      return true;
    }
    case SocketTypeTargetKind::SocketDatatype: {
      PropertyRNA *prop = RNA_struct_find_property(&target.ptr, target.prop_name);
      if (prop == nullptr) {
        return false;
      }
      RNA_property_enum_set(&target.ptr, prop, int(type));
      RNA_property_update(C, &target.ptr, prop);
      return true;
    }
  }
  return false;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Operator
 * \{ */

/**
 * Region-space mouse from the window event state (used by poll, which has no #wmEvent).
 */
static bool region_mval_from_window(const bContext *C, int2 &r_mval)
{
  const wmWindow *win = CTX_wm_window(C);
  const ARegion *region = CTX_wm_region(C);
  if (win == nullptr || win->runtime == nullptr || win->runtime->eventstate == nullptr ||
      region == nullptr)
  {
    return false;
  }
  r_mval.x = win->runtime->eventstate->xy[0] - region->winrct.xmin;
  r_mval.y = win->runtime->eventstate->xy[1] - region->winrct.ymin;
  return true;
}

bool node_socket_type_menu_can_invoke_at(bContext *C, const int2 mval)
{
  if (!ED_operator_node_editable(C)) {
    return false;
  }
  return resolve_from_cursor(C, mval).has_value();
}

/**
 * Loose poll: editable node tree only.
 * Cursor hit-testing is done in invoke so #WM_operator_name_call from #NODE_OT_select
 * cannot fail poll after a successful pre-check with a different mouse sample.
 */
static bool socket_type_menu_poll(bContext *C)
{
  return ED_operator_node_editable(C);
}

static bool type_allowed_for_target(const SocketTypeTarget &target, const eNodeSocketDatatype type)
{
  if (target.supports_type) {
    return target.supports_type(type, target.ntree_type);
  }
  if (target.owner_tree) {
    return interface_supports_socket_datatype(*target.owner_tree, type);
  }
  return true;
}

static bool allowed_types_has(const Span<int> allowed_types, const int value)
{
  for (const int t : allowed_types) {
    if (t == value) {
      return true;
    }
  }
  return false;
}

static bool fill_allowed_types(const SocketTypeTarget &target, Vector<int> &r_types)
{
  r_types.clear();
  for (const EnumPropertyItem *item = rna_enum_node_socket_data_type_items;
       item->identifier != nullptr;
       item++)
  {
    if (item->identifier[0] == '\0') {
      continue;
    }
    const eNodeSocketDatatype type = eNodeSocketDatatype(item->value);
    if (type_allowed_for_target(target, type)) {
      r_types.append(item->value);
    }
  }
  return !r_types.is_empty();
}

/**
 * Hold duration threshold (seconds).
 * Below → click (sticky panel). At/above → hold/release menu (slide to select).
 * Kept short so a brief drag already opens the release-to-apply menu.
 */
static constexpr double socket_type_hold_threshold_sec = 0.05;

/** Shared UI config for data-type (Shift) and structure/shape (Alt) menus. */
struct SocketShapeMenuSpec {
  const char *op_idname = nullptr;
  const char *enum_prop = nullptr;
  const char *title = nullptr;
  const EnumPropertyItem *enum_items = nullptr;
};

struct StickyShapePanelArg {
  int node_identifier = 0;
  char socket_identifier[MAX_NAME] = {};
  Vector<int> allowed_values;
  SocketShapeMenuSpec spec = {};
};

struct SocketShapeMenuModalData {
  int node_identifier = 0;
  char socket_identifier[MAX_NAME] = {};
  Vector<int> allowed_values;
  SocketShapeMenuSpec spec = {};
  double press_time = 0.0;
  wmTimer *timer = nullptr;
  bool hold_menu_opened = false;
};

static ui::Block *socket_shape_sticky_panel_block(bContext *C, ARegion *region, void *arg_v)
{
  StickyShapePanelArg *arg = static_cast<StickyShapePanelArg *>(arg_v);
  const uiStyle *style = ui::style_get_dpi();

  ui::Block *block = ui::block_begin(C, region, "node_socket_shape_sticky", ui::EmbossType::Emboss);
  ui::block_flag_enable(block,
                        ui::BLOCK_LOOP | ui::BLOCK_KEEP_OPEN | ui::BLOCK_NUMSELECT |
                            ui::BLOCK_POPUP);
  ui::block_theme_style_set(block, ui::BLOCK_THEME_STYLE_POPUP);

  ui::Layout &layout = ui::block_layout(block,
                                        ui::LayoutDirection::Vertical,
                                        ui::LayoutType::Panel,
                                        0,
                                        0,
                                        int(10 * U.widget_unit),
                                        int(2 * U.widget_unit),
                                        0,
                                        style);
  layout.operator_context_set(wm::OpCallContext::ExecDefault);

  layout.label(IFACE_(arg->spec.title), ICON_NONE);

  for (const EnumPropertyItem *item = arg->spec.enum_items; item->identifier != nullptr; item++) {
    if (!allowed_types_has(arg->allowed_values, item->value)) {
      continue;
    }
    PointerRNA op_ptr = layout.op(arg->spec.op_idname, IFACE_(item->name), item->icon);
    RNA_int_set(&op_ptr, "node_identifier", arg->node_identifier);
    RNA_string_set(&op_ptr, "socket_identifier", arg->socket_identifier);
    RNA_enum_set(&op_ptr, arg->spec.enum_prop, item->value);
  }

  const int bounds_offset[2] = {0, -UI_UNIT_Y};
  ui::block_bounds_set_popup(block, int(0.3f * U.widget_unit), bounds_offset);
  return block;
}

static void sticky_shape_panel_arg_free(void *arg_v)
{
  MEM_delete(static_cast<StickyShapePanelArg *>(arg_v));
}

static int open_sticky_shape_panel(bContext *C,
                                   const int node_identifier,
                                   const char *socket_identifier,
                                   const Span<int> allowed_values,
                                   const SocketShapeMenuSpec &spec)
{
  StickyShapePanelArg *arg = MEM_new<StickyShapePanelArg>(__func__);
  arg->node_identifier = node_identifier;
  BLI_strncpy(arg->socket_identifier, socket_identifier, sizeof(arg->socket_identifier));
  arg->allowed_values = allowed_values;
  arg->spec = spec;
  ui::popup_block_invoke(C, socket_shape_sticky_panel_block, arg, sticky_shape_panel_arg_free);
  return int(allowed_values.size());
}

static int open_hold_release_shape_menu(bContext *C,
                                        const int node_identifier,
                                        const char *socket_identifier,
                                        const Span<int> allowed_values,
                                        const SocketShapeMenuSpec &spec)
{
  ui::PopupMenu *pup = ui::popup_menu_begin(
      C, CTX_IFACE_(BLT_I18NCONTEXT_OPERATOR_DEFAULT, spec.title), ICON_NONE);
  ui::Layout *layout = ui::popup_menu_layout(pup);
  layout->operator_context_set(wm::OpCallContext::ExecDefault);

  int items_added = 0;
  for (const EnumPropertyItem *item = spec.enum_items; item->identifier != nullptr; item++) {
    if (!allowed_types_has(allowed_values, item->value)) {
      continue;
    }
    PointerRNA op_ptr = layout->op(spec.op_idname, IFACE_(item->name), item->icon);
    RNA_int_set(&op_ptr, "node_identifier", node_identifier);
    RNA_string_set(&op_ptr, "socket_identifier", socket_identifier);
    RNA_enum_set(&op_ptr, spec.enum_prop, item->value);
    items_added++;
  }
  ui::popup_menu_end(C, pup);
  return items_added;
}

/* Backward-compatible wrappers for data-type menus. */
static const SocketShapeMenuSpec data_type_menu_spec()
{
  return {"NODE_OT_socket_type_menu",
          "socket_type",
          "Socket Type",
          rna_enum_node_socket_data_type_items};
}

static int open_sticky_type_panel(bContext *C,
                                  const int node_identifier,
                                  const char *socket_identifier,
                                  const Span<int> allowed_types)
{
  return open_sticky_shape_panel(
      C, node_identifier, socket_identifier, allowed_types, data_type_menu_spec());
}

static int open_hold_release_type_menu(bContext *C,
                                       const int node_identifier,
                                       const char *socket_identifier,
                                       const Span<int> allowed_types)
{
  return open_hold_release_shape_menu(
      C, node_identifier, socket_identifier, allowed_types, data_type_menu_spec());
}

static bool resolve_invoke_ids(bContext *C,
                               const int2 mval,
                               int &r_node_id,
                               char r_socket_id[MAX_NAME],
                               Vector<int> &r_allowed_types)
{
  std::optional<SocketTypeTarget> target = resolve_from_cursor(C, mval);
  if (!target) {
    return false;
  }
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  if (snode == nullptr || region == nullptr) {
    return false;
  }
  float2 cursor;
  ui::view2d_region_to_view(&region->v2d, mval.x, mval.y, &cursor.x, &cursor.y);
  bNodeSocket *sock = node_find_indicated_socket(*snode, *region, cursor, SOCK_IN | SOCK_OUT);
  if (sock == nullptr || sock->runtime == nullptr || sock->runtime->owner_node == nullptr) {
    return false;
  }
  r_node_id = sock->runtime->owner_node->identifier;
  BLI_strncpy(r_socket_id, sock->identifier, MAX_NAME);
  return fill_allowed_types(*target, r_allowed_types);
}

static void modal_data_free(bContext *C, wmOperator *op)
{
  SocketShapeMenuModalData *data = static_cast<SocketShapeMenuModalData *>(op->customdata);
  if (data == nullptr) {
    return;
  }
  if (data->timer) {
    WM_event_timer_remove(CTX_wm_manager(C), CTX_wm_window(C), data->timer);
    data->timer = nullptr;
  }
  MEM_delete(data);
  op->customdata = nullptr;
}

/**
 * Shared invoke: CLICK → sticky panel; PRESS → timed modal (hold duration).
 * #resolve_allowed fills allowed enum values for the socket under the cursor.
 */
using ResolveAllowedFn = bool (*)(
    bContext *C, const int2 mval, int &r_node_id, char r_socket_id[MAX_NAME], Vector<int> &r_allowed);

static wmOperatorStatus socket_shape_menu_invoke_common(bContext *C,
                                                        wmOperator *op,
                                                        const wmEvent *event,
                                                        const SocketShapeMenuSpec &spec,
                                                        ResolveAllowedFn resolve_allowed,
                                                        const char *empty_warning)
{
  int2 mval(event->mval[0], event->mval[1]);
  if (mval.x < 0 || mval.y < 0) {
    if (!region_mval_from_window(C, mval)) {
      return OPERATOR_PASS_THROUGH | OPERATOR_CANCELLED;
    }
  }

  int node_id = 0;
  char socket_id[MAX_NAME] = {};
  Vector<int> allowed;
  if (!resolve_allowed(C, mval, node_id, socket_id, allowed)) {
    return OPERATOR_PASS_THROUGH | OPERATOR_CANCELLED;
  }

  RNA_int_set(op->ptr, "node_identifier", node_id);
  RNA_string_set(op->ptr, "socket_identifier", socket_id);

  if (event->val == KM_CLICK) {
    if (open_sticky_shape_panel(C, node_id, socket_id, allowed, spec) == 0) {
      BKE_report(op->reports, RPT_WARNING, empty_warning);
      return OPERATOR_CANCELLED;
    }
    return OPERATOR_FINISHED;
  }

  if (event->val != KM_PRESS) {
    return OPERATOR_PASS_THROUGH | OPERATOR_CANCELLED;
  }

  SocketShapeMenuModalData *data = MEM_new<SocketShapeMenuModalData>(__func__);
  data->node_identifier = node_id;
  BLI_strncpy(data->socket_identifier, socket_id, sizeof(data->socket_identifier));
  data->allowed_values = std::move(allowed);
  data->spec = spec;
  data->press_time = BLI_time_now_seconds();
  data->hold_menu_opened = false;
  data->timer = WM_event_timer_add(
      CTX_wm_manager(C), CTX_wm_window(C), TIMER, socket_type_hold_threshold_sec);
  op->customdata = data;

  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus socket_shape_menu_modal_common(bContext *C,
                                                       wmOperator *op,
                                                       const wmEvent *event)
{
  SocketShapeMenuModalData *data = static_cast<SocketShapeMenuModalData *>(op->customdata);
  if (data == nullptr) {
    return OPERATOR_CANCELLED;
  }

  auto finish = [&](wmOperatorStatus status) -> wmOperatorStatus {
    modal_data_free(C, op);
    return status;
  };

  if (event->type == EVT_ESCKEY && event->val == KM_PRESS) {
    return finish(OPERATOR_CANCELLED);
  }

  const double held = BLI_time_now_seconds() - data->press_time;
  const bool is_hold = held >= socket_type_hold_threshold_sec;

  if (!data->hold_menu_opened && is_hold &&
      (event->type == TIMER || ISMOUSE_MOTION(event->type) || event->type == LEFTMOUSE))
  {
    if (event->type == TIMER && event->customdata != data->timer) {
      return OPERATOR_RUNNING_MODAL;
    }
    open_hold_release_shape_menu(C,
                                 data->node_identifier,
                                 data->socket_identifier,
                                 data->allowed_values,
                                 data->spec);
    data->hold_menu_opened = true;
    return finish(OPERATOR_FINISHED);
  }

  if (event->type == LEFTMOUSE && event->val == KM_RELEASE) {
    if (data->hold_menu_opened) {
      return finish(OPERATOR_FINISHED);
    }
    open_sticky_shape_panel(C,
                            data->node_identifier,
                            data->socket_identifier,
                            data->allowed_values,
                            data->spec);
    return finish(OPERATOR_FINISHED);
  }

  return OPERATOR_RUNNING_MODAL;
}

static void socket_shape_menu_cancel_common(bContext *C, wmOperator *op)
{
  modal_data_free(C, op);
}

static wmOperatorStatus socket_type_menu_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  return socket_shape_menu_invoke_common(C,
                                         op,
                                         event,
                                         data_type_menu_spec(),
                                         resolve_invoke_ids,
                                         "No compatible socket types for this item");
}

static wmOperatorStatus socket_type_menu_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  return socket_shape_menu_modal_common(C, op, event);
}

static void socket_type_menu_cancel(bContext *C, wmOperator *op)
{
  socket_shape_menu_cancel_common(C, op);
}

static wmOperatorStatus socket_type_menu_exec(bContext *C, wmOperator *op)
{
  std::optional<SocketTypeTarget> target = resolve_from_op_props(C, op->ptr);
  if (!target) {
    return OPERATOR_CANCELLED;
  }

  const eNodeSocketDatatype type = eNodeSocketDatatype(RNA_enum_get(op->ptr, "socket_type"));
  if (!apply_socket_type(C, *target, type)) {
    BKE_report(op->reports, RPT_ERROR, "Cannot change socket to this type");
    return OPERATOR_CANCELLED;
  }

  return OPERATOR_FINISHED;
}

void NODE_OT_socket_type_menu(wmOperatorType *ot)
{
  ot->name = "Set Socket Type";
  ot->idname = "NODE_OT_socket_type_menu";
  ot->description =
      "Change the data type of a group interface socket or dynamic item socket under the cursor";

  ot->invoke = socket_type_menu_invoke;
  ot->modal = socket_type_menu_modal;
  ot->cancel = socket_type_menu_cancel;
  ot->exec = socket_type_menu_exec;
  ot->poll = socket_type_menu_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_BLOCKING;

  PropertyRNA *prop;

  prop = RNA_def_int(ot->srna,
                     "node_identifier",
                     0,
                     INT_MIN,
                     INT_MAX,
                     "Node",
                     "Identifier of the node that owns the socket",
                     INT_MIN,
                     INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_string(ot->srna,
                        "socket_identifier",
                        nullptr,
                        MAX_NAME,
                        "Socket",
                        "Identifier of the socket under the cursor");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_enum(ot->srna,
                      "socket_type",
                      rna_enum_node_socket_data_type_items,
                      SOCK_FLOAT,
                      "Socket Type",
                      "New data type for the socket item");
  ot->prop = prop;
}

/* -------------------------------------------------------------------- */
/** \name Alt+LMB: structure / shape (Auto, Single, Field, Grid, List, Dynamic)
 * \{ */

struct StructureTarget {
  PointerRNA ptr = {};
  bNodeTree *owner_tree = nullptr;
  eNodeSocketDatatype socket_data_type = SOCK_FLOAT;
};

static bool structure_value_supported(const bNodeTree &ntree,
                                      const eNodeSocketDatatype socket_type,
                                      const NodeSocketInterfaceStructureType structure)
{
  const bool is_geometry_nodes = ntree.type == NTREE_GEOMETRY;
  const bool supports_fields = is_geometry_nodes && nodes::socket_type_supports_fields(socket_type);
  const bool supports_grids = is_geometry_nodes && nodes::socket_type_supports_grids(socket_type);
  const bool supports_lists = is_geometry_nodes;

  switch (structure) {
    case NodeSocketInterfaceStructureType::Auto:
    case NodeSocketInterfaceStructureType::Single:
      return true;
    case NodeSocketInterfaceStructureType::List:
      return supports_lists;
    case NodeSocketInterfaceStructureType::Field:
      return supports_fields;
    case NodeSocketInterfaceStructureType::Grid:
      return supports_grids;
    case NodeSocketInterfaceStructureType::Dynamic:
      return supports_fields || supports_grids;
  }
  return false;
}

static bool fill_allowed_structures(const bNodeTree &ntree,
                                    const eNodeSocketDatatype socket_type,
                                    Vector<int> &r_values)
{
  r_values.clear();
  for (const EnumPropertyItem *item = rna_enum_node_socket_structure_type_items;
       item->identifier != nullptr;
       item++)
  {
    if (item->identifier[0] == '\0') {
      continue;
    }
    if (structure_value_supported(
            ntree, socket_type, NodeSocketInterfaceStructureType(item->value)))
    {
      r_values.append(item->value);
    }
  }
  return !r_values.is_empty();
}

static std::optional<StructureTarget> resolve_structure_group(bNodeTree &edit_tree,
                                                              bNode &node,
                                                              bNodeSocket &socket)
{
  bNodeTree *interface_tree = nullptr;
  const bNodeTreeInterfaceSocket *io_socket = nullptr;

  if (node.is_group()) {
    if (node.id == nullptr || GS(node.id->name) != ID_NT) {
      return std::nullopt;
    }
    interface_tree = reinterpret_cast<bNodeTree *>(node.id);
    if (socket.is_input()) {
      io_socket = bke::node_find_interface_input_by_identifier(*interface_tree, socket.identifier);
    }
    else {
      io_socket = bke::node_find_interface_output_by_identifier(*interface_tree, socket.identifier);
    }
  }
  else if (node.is_group_input()) {
    interface_tree = &edit_tree;
    io_socket = bke::node_find_interface_input_by_identifier(*interface_tree, socket.identifier);
  }
  else if (node.is_group_output()) {
    interface_tree = &edit_tree;
    io_socket = bke::node_find_interface_output_by_identifier(*interface_tree, socket.identifier);
  }
  else {
    return std::nullopt;
  }

  if (interface_tree == nullptr || io_socket == nullptr) {
    return std::nullopt;
  }
  const bke::bNodeSocketType *stype = io_socket->socket_typeinfo();
  if (stype == nullptr) {
    return std::nullopt;
  }

  StructureTarget target;
  target.owner_tree = interface_tree;
  target.socket_data_type = stype->type;
  target.ptr = RNA_pointer_create_discrete(
      &interface_tree->id,
      RNA_NodeTreeInterfaceSocket,
      const_cast<bNodeTreeInterfaceSocket *>(io_socket));
  return target;
}

template<typename Accessor>
static std::optional<StructureTarget> resolve_structure_item(bNodeTree &tree,
                                                             bNode &storage_node,
                                                             bNodeSocket &socket)
{
  using ItemT = typename Accessor::ItemT;
  ItemT *item = find_item_for_socket_identifier<Accessor>(storage_node, socket.identifier);
  if (item == nullptr || Accessor::item_srna == nullptr || *Accessor::item_srna == nullptr) {
    return std::nullopt;
  }
  /* Only items that expose RNA structure_type. */
  PointerRNA ptr = RNA_pointer_create_discrete(&tree.id, *Accessor::item_srna, item);
  if (RNA_struct_find_property(&ptr, "structure_type") == nullptr) {
    return std::nullopt;
  }

  StructureTarget target;
  target.owner_tree = &tree;
  target.socket_data_type = Accessor::get_socket_type(*item);
  target.ptr = ptr;
  return target;
}

static std::optional<StructureTarget> resolve_structure_items(bNodeTree &tree,
                                                              bNode &node,
                                                              bNodeSocket &socket)
{
  using namespace nodes;

  if (node.is_type("NodeCombineBundle"_ustr)) {
    return resolve_structure_item<CombineBundleItemsAccessor>(tree, node, socket);
  }
  if (node.is_type("NodeSeparateBundle"_ustr)) {
    return resolve_structure_item<SeparateBundleItemsAccessor>(tree, node, socket);
  }
  if (node.is_type("NodeEvaluateClosure"_ustr)) {
    if (auto t = resolve_structure_item<EvaluateClosureInputItemsAccessor>(tree, node, socket)) {
      return t;
    }
    return resolve_structure_item<EvaluateClosureOutputItemsAccessor>(tree, node, socket);
  }
  if (node.is_type("NodeClosureInput"_ustr)) {
    const auto &storage = *static_cast<const NodeClosureInput *>(node.storage);
    bNode *output_node = tree.node_by_id(storage.output_node_id);
    if (output_node == nullptr) {
      return std::nullopt;
    }
    return resolve_structure_item<ClosureInputItemsAccessor>(tree, *output_node, socket);
  }
  if (node.is_type("NodeClosureOutput"_ustr)) {
    if (auto t = resolve_structure_item<ClosureOutputItemsAccessor>(tree, node, socket)) {
      return t;
    }
    return resolve_structure_item<ClosureInputItemsAccessor>(tree, node, socket);
  }
  return std::nullopt;
}

static std::optional<StructureTarget> resolve_structure_target(bNodeTree &tree,
                                                               bNode &node,
                                                               bNodeSocket &socket)
{
  if (socket_is_extend_identifier(socket.identifier)) {
    return std::nullopt;
  }
  if (auto t = resolve_structure_group(tree, node, socket)) {
    return t;
  }
  return resolve_structure_items(tree, node, socket);
}

static std::optional<StructureTarget> resolve_structure_from_cursor(bContext *C, const int2 mval)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  if (snode == nullptr || snode->edittree == nullptr || region == nullptr) {
    return std::nullopt;
  }
  if (!ED_operator_node_editable(C)) {
    return std::nullopt;
  }

  bNodeTree &tree = *snode->edittree;
  tree.ensure_topology_cache();

  float2 cursor;
  ui::view2d_region_to_view(&region->v2d, mval.x, mval.y, &cursor.x, &cursor.y);

  bNodeSocket *sock = node_find_indicated_socket(*snode, *region, cursor, SOCK_IN | SOCK_OUT);
  if (sock == nullptr || sock->runtime == nullptr || sock->runtime->owner_node == nullptr) {
    return std::nullopt;
  }
  return resolve_structure_target(tree, *sock->runtime->owner_node, *sock);
}

static std::optional<StructureTarget> resolve_structure_from_op_props(bContext *C, PointerRNA *op_ptr)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (snode == nullptr || snode->edittree == nullptr) {
    return std::nullopt;
  }
  bNodeTree &tree = *snode->edittree;
  tree.ensure_topology_cache();

  const int node_identifier = RNA_int_get(op_ptr, "node_identifier");
  char socket_identifier[MAX_NAME];
  RNA_string_get(op_ptr, "socket_identifier", socket_identifier);

  bNode *node = tree.node_by_id(node_identifier);
  if (node == nullptr) {
    return std::nullopt;
  }
  const UString sock_id(socket_identifier);
  bNodeSocket *sock = bke::node_find_socket(*node, SOCK_IN, sock_id);
  if (sock == nullptr) {
    sock = bke::node_find_socket(*node, SOCK_OUT, sock_id);
  }
  if (sock == nullptr) {
    return std::nullopt;
  }
  return resolve_structure_target(tree, *node, *sock);
}

static bool apply_structure_type(bContext *C,
                                 StructureTarget &target,
                                 const NodeSocketInterfaceStructureType structure)
{
  if (!structure_value_supported(*target.owner_tree, target.socket_data_type, structure)) {
    return false;
  }
  PropertyRNA *prop = RNA_struct_find_property(&target.ptr, "structure_type");
  if (prop == nullptr) {
    return false;
  }
  RNA_property_enum_set(&target.ptr, prop, int(structure));
  RNA_property_update(C, &target.ptr, prop);

  Main *bmain = CTX_data_main(C);
  BKE_main_ensure_invariants(*bmain, target.owner_tree->id);
  if (SpaceNode *snode = CTX_wm_space_node(C)) {
    if (snode->edittree && snode->edittree != target.owner_tree) {
      BKE_main_ensure_invariants(*bmain, snode->edittree->id);
      WM_main_add_notifier(NC_NODE | NA_EDITED, snode->edittree);
    }
  }
  WM_main_add_notifier(NC_NODE | NA_EDITED, target.owner_tree);
  return true;
}

static bool resolve_structure_invoke_ids(bContext *C,
                                         const int2 mval,
                                         int &r_node_id,
                                         char r_socket_id[MAX_NAME],
                                         Vector<int> &r_allowed)
{
  std::optional<StructureTarget> target = resolve_structure_from_cursor(C, mval);
  if (!target) {
    return false;
  }
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  if (snode == nullptr || region == nullptr) {
    return false;
  }
  float2 cursor;
  ui::view2d_region_to_view(&region->v2d, mval.x, mval.y, &cursor.x, &cursor.y);
  bNodeSocket *sock = node_find_indicated_socket(*snode, *region, cursor, SOCK_IN | SOCK_OUT);
  if (sock == nullptr || sock->runtime == nullptr || sock->runtime->owner_node == nullptr) {
    return false;
  }
  r_node_id = sock->runtime->owner_node->identifier;
  BLI_strncpy(r_socket_id, sock->identifier, MAX_NAME);
  return fill_allowed_structures(*target->owner_tree, target->socket_data_type, r_allowed);
}

static const SocketShapeMenuSpec structure_menu_spec()
{
  return {"NODE_OT_socket_structure_menu",
          "structure_type",
          "Socket Shape",
          rna_enum_node_socket_structure_type_items};
}

bool node_socket_structure_menu_can_invoke_at(bContext *C, const int2 mval)
{
  if (!ED_operator_node_editable(C)) {
    return false;
  }
  return resolve_structure_from_cursor(C, mval).has_value();
}

static bool socket_structure_menu_poll(bContext *C)
{
  return ED_operator_node_editable(C);
}

static wmOperatorStatus socket_structure_menu_invoke(bContext *C,
                                                     wmOperator *op,
                                                     const wmEvent *event)
{
  return socket_shape_menu_invoke_common(C,
                                         op,
                                         event,
                                         structure_menu_spec(),
                                         resolve_structure_invoke_ids,
                                         "No compatible structure types for this socket");
}

static wmOperatorStatus socket_structure_menu_modal(bContext *C,
                                                    wmOperator *op,
                                                    const wmEvent *event)
{
  return socket_shape_menu_modal_common(C, op, event);
}

static void socket_structure_menu_cancel(bContext *C, wmOperator *op)
{
  socket_shape_menu_cancel_common(C, op);
}

static wmOperatorStatus socket_structure_menu_exec(bContext *C, wmOperator *op)
{
  std::optional<StructureTarget> target = resolve_structure_from_op_props(C, op->ptr);
  if (!target) {
    return OPERATOR_CANCELLED;
  }
  const auto structure = NodeSocketInterfaceStructureType(
      RNA_enum_get(op->ptr, "structure_type"));
  if (!apply_structure_type(C, *target, structure)) {
    BKE_report(op->reports, RPT_ERROR, "Cannot change socket to this structure type");
    return OPERATOR_CANCELLED;
  }
  return OPERATOR_FINISHED;
}

void NODE_OT_socket_structure_menu(wmOperatorType *ot)
{
  ot->name = "Set Socket Shape";
  ot->idname = "NODE_OT_socket_structure_menu";
  ot->description =
      "Change the structure type (shape) of a group interface or item socket under the cursor";

  ot->invoke = socket_structure_menu_invoke;
  ot->modal = socket_structure_menu_modal;
  ot->cancel = socket_structure_menu_cancel;
  ot->exec = socket_structure_menu_exec;
  ot->poll = socket_structure_menu_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_BLOCKING;

  PropertyRNA *prop;

  prop = RNA_def_int(ot->srna,
                     "node_identifier",
                     0,
                     INT_MIN,
                     INT_MAX,
                     "Node",
                     "Identifier of the node that owns the socket",
                     INT_MIN,
                     INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_string(ot->srna,
                        "socket_identifier",
                        nullptr,
                        MAX_NAME,
                        "Socket",
                        "Identifier of the socket under the cursor");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_enum(ot->srna,
                      "structure_type",
                      rna_enum_node_socket_structure_type_items,
                      int(NodeSocketInterfaceStructureType::Auto),
                      "Structure Type",
                      "Auto / Single / Field / Grid / List / Dynamic");
  ot->prop = prop;
}

/** \} */

/**
 * #NODE_OT_select strips #OPERATOR_PASS_THROUGH on non-PRESS events (CLICK).
 * That becomes #WM_HANDLER_BREAK, so any later Shift+CLICK binding never runs.
 * Force our item to the head of every Node Editor keymap (including "Blender user").
 */
void node_socket_type_menu_ensure_keymap_priority(wmWindowManager *wm)
{
  if (wm == nullptr) {
    return;
  }

  auto ensure_head = [](wmKeyMap *keymap,
                        const char *idname,
                        const int16_t modifier_flag) {
    if (keymap == nullptr) {
      return;
    }
    wmKeyMapItem *found = nullptr;
    for (wmKeyMapItem &kmi : keymap->items) {
      if (STREQ(kmi.idname, idname) && kmi.type == LEFTMOUSE && kmi.val == KM_CLICK) {
        const bool want_ctrl = (modifier_flag & KM_CTRL) != 0;
        const bool want_alt = (modifier_flag & KM_ALT) != 0;
        const bool want_shift = (modifier_flag & KM_SHIFT) != 0;
        const bool has_ctrl = (kmi.ctrl == KM_MOD_HELD || kmi.ctrl == KM_ANY);
        const bool has_alt = (kmi.alt == KM_MOD_HELD || kmi.alt == KM_ANY);
        const bool has_shift = (kmi.shift == KM_MOD_HELD || kmi.shift == KM_ANY);
        if (want_ctrl == has_ctrl && want_alt == has_alt && want_shift == has_shift) {
          found = &kmi;
          break;
        }
      }
    }
    if (found == nullptr) {
      KeyMapItem_Params params{};
      params.type = LEFTMOUSE;
      params.value = KM_CLICK;
      params.modifier = modifier_flag;
      params.keymodifier = 0;
      params.direction = KM_ANY;
      found = WM_keymap_add_item(keymap, idname, &params);
    }
    if (found != nullptr) {
      BLI_remlink(&keymap->items, found);
      BLI_addhead(&keymap->items, found);
    }
  };

  auto fix_keymap = [&](wmKeyMap *keymap) {
    if (keymap == nullptr) {
      return;
    }
    /* Drop leftover Ctrl+CLICK type-menu items after switching to Shift. */
    for (wmKeyMapItem *kmi = static_cast<wmKeyMapItem *>(keymap->items.first()); kmi != nullptr;) {
      wmKeyMapItem *next = kmi->next;
      if (STREQ(kmi->idname, "NODE_OT_socket_type_menu") && kmi->type == LEFTMOUSE &&
          kmi->val == KM_CLICK && kmi->shift == KM_NOTHING &&
          (kmi->ctrl == KM_MOD_HELD || kmi->ctrl == KM_ANY) && (kmi->alt == KM_NOTHING))
      {
        WM_keymap_remove_item(keymap, kmi);
      }
      kmi = next;
    }
    /* Alt structure first, then Shift type — both ahead of node.select. */
    ensure_head(keymap, "NODE_OT_socket_structure_menu", KM_ALT);
    ensure_head(keymap, "NODE_OT_socket_type_menu", KM_SHIFT);
  };

  for (wmKeyConfig &keyconf : wm->runtime->keyconfigs) {
    fix_keymap(WM_keymap_list_find(&keyconf.keymaps, "Node Editor", SPACE_NODE, RGN_TYPE_WINDOW));
  }
  if (wm->runtime->defaultconf) {
    fix_keymap(WM_keymap_list_find(
        &wm->runtime->defaultconf->keymaps, "Node Editor", SPACE_NODE, RGN_TYPE_WINDOW));
  }
  if (wm->runtime->userconf) {
    fix_keymap(WM_keymap_list_find(
        &wm->runtime->userconf->keymaps, "Node Editor", SPACE_NODE, RGN_TYPE_WINDOW));
  }
}

/** \} */

}  // namespace blender::ed::space_node
