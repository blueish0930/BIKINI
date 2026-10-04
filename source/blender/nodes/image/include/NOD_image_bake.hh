/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>
#include <string>

#include "BLI_math_base.hh"
#include "BLI_string.hh"
#include "BLI_string_ref.hh"

#include "MEM_guardedalloc.h"

#include "DNA_node_types.h"

#include "NOD_socket_items.hh"

struct BlendDataReader;
struct BlendWriter;
struct StructRNA;

namespace blender::nodes {

struct ImageBakeItemsAccessor : public socket_items::SocketItemsAccessorDefaults {
  using ItemT = NodeImageBakeItem;
  static StructRNA **item_srna;
  static constexpr StringRefNull node_idname = "ImageNodeBakeImage";
  static constexpr bool has_type = false;
  static constexpr bool has_name = true;
  static constexpr bool has_name_validation = true;
  static constexpr bool can_have_empty_name = false;
  static constexpr char unique_name_separator = '_';
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_image_bake_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_image_bake_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_image_bake_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_image_bake_items";
  };
  struct rna_names {
    static constexpr StringRefNull items = "bake_items";
    static constexpr StringRefNull active_index = "active_item_index";
  };

  static socket_items::SocketItemsRef<NodeImageBakeItem> get_items_from_node(bNode &node)
  {
    auto *storage = static_cast<NodeImageBakeImage *>(node.storage);
    return {&storage->items, &storage->items_num, &storage->active_index};
  }

  static void copy_item(const NodeImageBakeItem &source, NodeImageBakeItem &destination)
  {
    destination = source;
    destination.name = BLI_strdup_null(destination.name);
  }

  static void destruct_item(NodeImageBakeItem *item)
  {
    MEM_SAFE_DELETE(item->name);
  }

  static void blend_write_item(BlendWriter *writer, const ItemT &item);
  static void blend_read_data_item(BlendDataReader *reader, ItemT &item);

  static eNodeSocketDatatype get_socket_type(const NodeImageBakeItem & /*item*/)
  {
    return SOCK_RGBA;
  }

  static char **get_name(NodeImageBakeItem &item)
  {
    return &item.name;
  }

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int /*ntree_type*/)
  {
    /* Bake stores everything as Color images on disk; accept Color links for extend. */
    return socket_type == SOCK_RGBA;
  }

  static int find_available_identifier(const NodeImageBakeItem *items, const int items_num)
  {
    int max_identifier = -1;
    for (int i = 0; i < items_num; i++) {
      max_identifier = math::max(items[i].identifier, max_identifier);
    }
    return max_identifier + 1;
  }

  static void init_with_name(bNode &node, NodeImageBakeItem &item, const char *name)
  {
    auto *storage = static_cast<NodeImageBakeImage *>(node.storage);
    item.identifier = find_available_identifier(storage->items, storage->items_num);
    socket_items::set_item_name_and_make_unique<ImageBakeItemsAccessor>(node, item, name);
  }

  static void init_with_socket_type_and_name(bNode &node,
                                             NodeImageBakeItem &item,
                                             const eNodeSocketDatatype /*socket_type*/,
                                             const char *name,
                                             std::optional<int> /*dimensions*/ = std::nullopt)
  {
    init_with_name(node, item, name);
  }

  static std::string validate_name(const StringRef name);

  static std::string socket_identifier_for_item(const NodeImageBakeItem &item)
  {
    return "Item_" + std::to_string(item.identifier);
  }
};

}  // namespace blender::nodes
