/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 */

#include <fmt/format.h>

#include "DNA_node_tree_interface_types.h"

#include "BLI_vector.hh"

#include "NOD_caller_ui.hh"
#include "NOD_socket_usage_inference.hh"

#include "RNA_access.hh"
#include "RNA_types.hh"

#include "BLT_translation.hh"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"

namespace blender::nodes {

static bool interface_panel_has_socket(
    const bNodeTreeInterfacePanel &interface_panel,
    FunctionRef<bool(const bNodeTreeInterfaceSocket &)> fn_input_is_visible)
{
  for (const bNodeTreeInterfaceItem *item : interface_panel.items()) {
    if (ELEM(item->item_type,
             NodeTreeInterfaceItemType::Separator,
             NodeTreeInterfaceItemType::Message))
    {
      return true;
    }
    if (item->item_type == NodeTreeInterfaceItemType::Socket) {
      const bNodeTreeInterfaceSocket &socket = *reinterpret_cast<const bNodeTreeInterfaceSocket *>(
          item);
      if (socket.flag & NODE_INTERFACE_SOCKET_HIDE_IN_MODIFIER) {
        continue;
      }
      if (socket.flag & NODE_INTERFACE_SOCKET_INPUT) {
        if (fn_input_is_visible(socket)) {
          return true;
        }
      }
    }
    else if (item->item_type == NodeTreeInterfaceItemType::Panel) {
      const auto &panel_item = *reinterpret_cast<const bNodeTreeInterfacePanel *>(item);
      if (interface_panel_has_socket(panel_item, fn_input_is_visible)) {
        return true;
      }
    }
  }
  return false;
}

static bool interface_panel_affects_output(
    const bNodeTreeInterfacePanel &panel,
    FunctionRef<bool(const bNodeTreeInterfaceSocket &)> fn_input_is_active)
{
  for (const bNodeTreeInterfaceItem *item : panel.items()) {
    if (item->item_type == NodeTreeInterfaceItemType::Socket) {
      const auto &socket = *reinterpret_cast<const bNodeTreeInterfaceSocket *>(item);
      if (socket.flag & NODE_INTERFACE_SOCKET_HIDE_IN_MODIFIER) {
        continue;
      }
      if (!(socket.flag & NODE_INTERFACE_SOCKET_INPUT)) {
        continue;
      }
      if (fn_input_is_active(socket)) {
        return true;
      }
    }
    else if (item->item_type == NodeTreeInterfaceItemType::Panel) {
      const auto &sub_interface_panel = *reinterpret_cast<const bNodeTreeInterfacePanel *>(item);
      if (interface_panel_affects_output(sub_interface_panel, fn_input_is_active)) {
        return true;
      }
    }
  }
  return false;
}

void draw_interface_panel_as_panel(
    const bContext &C,
    ui::Layout &layout,
    PointerRNA *properties_ptr,
    const bNodeTreeInterfacePanel &interface_panel,
    FunctionRef<bool(const bNodeTreeInterfaceSocket &)> fn_input_is_visible,
    FunctionRef<bool(const bNodeTreeInterfaceSocket &)> fn_input_is_active,
    FunctionRef<void(ui::Layout &,
                     const bNodeTreeInterfaceSocket &,
                     PointerRNA *,
                     const std::optional<StringRef>)> fn_draw_property_for_socket)
{
  if (!interface_panel_has_socket(interface_panel, fn_input_is_visible)) {
    return;
  }
  PointerRNA panels_ptr = RNA_pointer_get(properties_ptr, "panels");
  const std::string panel_open_name = fmt::format("open_{}", interface_panel.identifier);
  ui::PanelLayout panel_layout;
  bool skip_first = false;
  /* Check if the panel should have a toggle in the header. */
  const bNodeTreeInterfaceSocket *toggle_socket = interface_panel.header_toggle_socket();
  const StringRef panel_name = interface_panel.name();
  if (toggle_socket && !(toggle_socket->flag & NODE_INTERFACE_SOCKET_HIDE_IN_MODIFIER)) {
    PointerRNA inputs_ptr = RNA_pointer_get(properties_ptr, "inputs");
    PointerRNA toggle_ptr = RNA_pointer_get(&inputs_ptr, toggle_socket->identifier);
    panel_layout = layout.panel_prop_with_bool_header(
        &C, &panels_ptr, panel_open_name, &toggle_ptr, "value", IFACE_(panel_name));
    skip_first = true;
  }
  else {
    panel_layout = layout.panel_prop(&C, &panels_ptr, panel_open_name);
    panel_layout.header->label(IFACE_(panel_name), ICON_NONE);
  }
  if (!interface_panel_affects_output(interface_panel, fn_input_is_active)) {
    panel_layout.header->active_set(false);
  }
  uiLayoutSetTooltipFunc(
      panel_layout.header,
      [](bContext * /*C*/, void *panel_arg, const StringRef /*tip*/) -> std::string {
        const auto *panel = static_cast<bNodeTreeInterfacePanel *>(panel_arg);
        return panel->description();
      },
      const_cast<bNodeTreeInterfacePanel *>(&interface_panel),
      nullptr,
      nullptr);
  if (panel_layout.body) {
    draw_interface_panel_content(C,
                                 *panel_layout.body,
                                 properties_ptr,
                                 interface_panel,
                                 fn_input_is_visible,
                                 fn_input_is_active,
                                 fn_draw_property_for_socket,
                                 skip_first,
                                 panel_name);
  }
}

static bool interface_socket_should_draw_property(
    const bNodeTreeInterfaceSocket &interface_socket,
    FunctionRef<bool(const bNodeTreeInterfaceSocket &)> fn_input_is_visible)
{
  if (!(interface_socket.flag & NODE_INTERFACE_SOCKET_INPUT)) {
    return false;
  }
  if (interface_socket.flag & NODE_INTERFACE_SOCKET_HIDE_IN_MODIFIER) {
    return false;
  }
  return fn_input_is_visible(interface_socket);
}

void draw_interface_panel_content(
    const bContext &C,
    ui::Layout &layout,
    PointerRNA *properties_ptr,
    const bNodeTreeInterfacePanel &interface_panel,
    FunctionRef<bool(const bNodeTreeInterfaceSocket &)> fn_input_is_visible,
    FunctionRef<bool(const bNodeTreeInterfaceSocket &)> fn_input_is_active,
    FunctionRef<void(ui::Layout &,
                     const bNodeTreeInterfaceSocket &,
                     PointerRNA *,
                     const std::optional<StringRef>)> fn_draw_property_for_socket,
    const bool skip_first,
    const std::optional<StringRef> parent_name)
{
  const Span<const bNodeTreeInterfaceItem *> items = interface_panel.items().drop_front(
      skip_first ? 1 : 0);

  for (int item_i = 0; item_i < items.size(); item_i++) {
    const bNodeTreeInterfaceItem *item = items[item_i];
    switch (item->item_type) {
      case NodeTreeInterfaceItemType::Panel: {
        const auto &sub_interface_panel = *reinterpret_cast<const bNodeTreeInterfacePanel *>(item);
        draw_interface_panel_as_panel(C,
                                      layout,
                                      properties_ptr,
                                      sub_interface_panel,
                                      fn_input_is_visible,
                                      fn_input_is_active,
                                      fn_draw_property_for_socket);
        break;
      }
      case NodeTreeInterfaceItemType::Socket: {
        const auto &interface_socket = *reinterpret_cast<const bNodeTreeInterfaceSocket *>(item);
        if (!interface_socket_should_draw_property(interface_socket, fn_input_is_visible)) {
          break;
        }

        /* Collect consecutive sockets that are horizontally joined via JOIN_TO_NEXT. */
        Vector<const bNodeTreeInterfaceSocket *> joined_sockets;
        joined_sockets.append(&interface_socket);
        while (true) {
          const bNodeTreeInterfaceSocket &last = *joined_sockets.last();
          if (!(last.flag & NODE_INTERFACE_SOCKET_JOIN_TO_NEXT)) {
            break;
          }
          const int next_i = item_i + joined_sockets.size();
          if (next_i >= items.size()) {
            break;
          }
          const bNodeTreeInterfaceItem *next_item = items[next_i];
          if (next_item->item_type != NodeTreeInterfaceItemType::Socket) {
            break;
          }
          const auto &next_socket = *reinterpret_cast<const bNodeTreeInterfaceSocket *>(next_item);
          if (!interface_socket_should_draw_property(next_socket, fn_input_is_visible)) {
            break;
          }
          joined_sockets.append(&next_socket);
        }

        auto draw_one = [&](ui::Layout &target, const bNodeTreeInterfaceSocket &socket) {
          PointerRNA inputs_ptr = RNA_pointer_get(properties_ptr, "inputs");
          PointerRNA socket_props_ptr = RNA_pointer_get(&inputs_ptr, socket.identifier);
          fn_draw_property_for_socket(target, socket, &socket_props_ptr, parent_name);
        };

        if (joined_sockets.size() == 1) {
          draw_one(layout, *joined_sockets[0]);
        }
        else {
          /* Side-by-side: disable property-split so each cell is label+value in one line,
           * otherwise each socket still expands to a full-width stacked row. */
          ui::Layout &row = layout.row(true);
          row.use_property_split_set(false);
          row.use_property_decorate_set(true);
          for (const bNodeTreeInterfaceSocket *socket : joined_sockets) {
            ui::Layout &cell = row.column(true);
            cell.use_property_split_set(false);
            cell.use_property_decorate_set(true);
            draw_one(cell, *socket);
          }
        }

        /* Skip sockets already drawn as part of this horizontal group. */
        item_i += joined_sockets.size() - 1;
        break;
      }
      case NodeTreeInterfaceItemType::Separator: {
        /* Full-width rule with ~1–2px padding. */
        layout.separator(0.3f, ui::LayoutSeparatorType::Space);
        layout.separator(1.0f, ui::LayoutSeparatorType::Line);
        layout.separator(0.3f, ui::LayoutSeparatorType::Space);
        break;
      }
      case NodeTreeInterfaceItemType::Message: {
        const auto &message = *reinterpret_cast<const bNodeTreeInterfaceMessage *>(item);
        layout.label_multiline(message.message ? message.message : "", ICON_NONE);
        break;
      }
    }
  }
}

}  // namespace blender::nodes
