/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_node_types.h"

#include "NOD_socket_items.hh"

namespace blender::nodes {

inline bool socket_type_supported_in_hlsl(const eNodeSocketDatatype socket_type,
                                          const int /*ntree_type*/)
{
  /* Closures (Shader sockets) are not supported as HLSL body parameters in EEVEE. */
  return ELEM(socket_type, SOCK_FLOAT, SOCK_INT, SOCK_BOOLEAN, SOCK_VECTOR, SOCK_RGBA);
}

struct HLSLInputItemsAccessor : public socket_items::SocketItemsAccessorDefaults {
  using ItemT = NodeShaderHLSLItem;
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "ShaderNodeHLSL";
  static constexpr bool has_type = true;
  static constexpr bool has_name = true;
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_hlsl_input_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_hlsl_input_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_hlsl_input_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_hlsl_input_items";
  };
  struct rna_names {
    static constexpr StringRefNull items = "input_items";
    static constexpr StringRefNull active_index = "active_input_index";
  };

  static socket_items::SocketItemsRef<ItemT> get_items_from_node(bNode &node)
  {
    auto *storage = static_cast<NodeShaderHLSL *>(node.storage);
    return {&storage->input_items.items,
            &storage->input_items.items_num,
            &storage->input_items.active_index};
  }

  static void copy_item(const ItemT &src, ItemT &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }

  static void destruct_item(ItemT *item)
  {
    MEM_SAFE_DELETE(item->name);
  }

  static void blend_write_item(BlendWriter *writer, const ItemT &item);
  static void blend_read_data_item(BlendDataReader *reader, ItemT &item);

  static eNodeSocketDatatype get_socket_type(const ItemT &item)
  {
    return item.socket_type;
  }

  static char **get_name(ItemT &item)
  {
    return &item.name;
  }

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int ntree_type)
  {
    return socket_type_supported_in_hlsl(socket_type, ntree_type);
  }

  static void init_with_socket_type_and_name(bNode &node,
                                             ItemT &item,
                                             const eNodeSocketDatatype socket_type,
                                             const char *name)
  {
    auto *storage = static_cast<NodeShaderHLSL *>(node.storage);
    item.socket_type = socket_type;
    item.identifier = storage->input_items.next_identifier++;
    socket_items::set_item_name_and_make_unique<HLSLInputItemsAccessor>(node, item, name);
  }

  static std::string socket_identifier_for_item(const ItemT &item)
  {
    return "In_" + std::to_string(item.identifier);
  }
};

struct HLSLOutputItemsAccessor : public socket_items::SocketItemsAccessorDefaults {
  using ItemT = NodeShaderHLSLItem;
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "ShaderNodeHLSL";
  static constexpr bool has_type = true;
  static constexpr bool has_name = true;
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_hlsl_output_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_hlsl_output_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_hlsl_output_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_hlsl_output_items";
  };
  struct rna_names {
    static constexpr StringRefNull items = "output_items";
    static constexpr StringRefNull active_index = "active_output_index";
  };

  static socket_items::SocketItemsRef<ItemT> get_items_from_node(bNode &node)
  {
    auto *storage = static_cast<NodeShaderHLSL *>(node.storage);
    return {&storage->output_items.items,
            &storage->output_items.items_num,
            &storage->output_items.active_index};
  }

  static void copy_item(const ItemT &src, ItemT &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }

  static void destruct_item(ItemT *item)
  {
    MEM_SAFE_DELETE(item->name);
  }

  static void blend_write_item(BlendWriter *writer, const ItemT &item);
  static void blend_read_data_item(BlendDataReader *reader, ItemT &item);

  static eNodeSocketDatatype get_socket_type(const ItemT &item)
  {
    return item.socket_type;
  }

  static char **get_name(ItemT &item)
  {
    return &item.name;
  }

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int ntree_type)
  {
    return socket_type_supported_in_hlsl(socket_type, ntree_type);
  }

  static void init_with_socket_type_and_name(bNode &node,
                                             ItemT &item,
                                             const eNodeSocketDatatype socket_type,
                                             const char *name)
  {
    auto *storage = static_cast<NodeShaderHLSL *>(node.storage);
    item.socket_type = socket_type;
    item.identifier = storage->output_items.next_identifier++;
    socket_items::set_item_name_and_make_unique<HLSLOutputItemsAccessor>(node, item, name);
  }

  static std::string socket_identifier_for_item(const ItemT &item)
  {
    return "Out_" + std::to_string(item.identifier);
  }
};

}  // namespace blender::nodes
