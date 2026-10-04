/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>

#include "DNA_node_types.h"

#include "NOD_socket_items.hh"
#include "NOD_socket_items_name_util.hh"

#include "RNA_access.hh"

namespace blender::nodes {

struct ExpressionInputItemsAccessor : public socket_items::SocketItemsAccessorDefaults {
  using ItemT = NodeExpressionInputItem;
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "NodeExpression";
  static constexpr bool has_type = true;
  static constexpr bool has_name = true;
  static constexpr bool has_custom_initial_name = true;
  static constexpr char unique_name_separator = '_';
  static constexpr bool has_name_validation = true;
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_expression_input_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_expression_input_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_expression_input_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_expression_input_state";
  };
  struct rna_names {
    static constexpr StringRefNull items = "input_items";
    static constexpr StringRefNull active_index = "active_input_index";
  };

  static socket_items::SocketItemsRef<NodeExpressionInputItem> get_items_from_node(bNode &node)
  {
    auto *storage = static_cast<NodeExpression *>(node.storage);
    return {&storage->input_items.items,
            &storage->input_items.items_num,
            &storage->input_items.active_index};
  }

  static void copy_item(const NodeExpressionInputItem &src, NodeExpressionInputItem &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }

  static void destruct_item(NodeExpressionInputItem *item)
  {
    MEM_delete(item->name);
  }

  static void blend_write_item(BlendWriter *writer, const ItemT &item);
  static void blend_read_data_item(BlendDataReader *reader, ItemT &item);

  static eNodeSocketDatatype get_socket_type(const NodeExpressionInputItem &item)
  {
    return eNodeSocketDatatype(item.socket_type);
  }

  static char **get_name(NodeExpressionInputItem &item)
  {
    return &item.name;
  }

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int ntree_type)
  {
    switch (ntree_type) {
      case NTREE_GEOMETRY:
      case NTREE_COMPOSIT:
        return ELEM(socket_type,
                    SOCK_FLOAT,
                    SOCK_VECTOR,
                    SOCK_RGBA,
                    SOCK_BOOLEAN,
                    SOCK_ROTATION,
                    SOCK_MATRIX,
                    SOCK_INT,
                    SOCK_STRING);
      case NTREE_SHADER:
        return ELEM(
            socket_type, SOCK_FLOAT, SOCK_VECTOR, SOCK_RGBA, SOCK_BOOLEAN, SOCK_INT, SOCK_STRING);
      default:
        return false;
    }
  }

  static void init_with_socket_type_and_name(bNode &node,
                                             NodeExpressionInputItem &item,
                                             const eNodeSocketDatatype socket_type,
                                             const char *name)
  {
    auto *storage = static_cast<NodeExpression *>(node.storage);
    item.socket_type = socket_type;
    item.identifier = storage->input_items.next_identifier++;
    socket_items::set_item_name_and_make_unique<ExpressionInputItemsAccessor>(node, item, name);
  }

  static std::string socket_identifier_for_item(const NodeExpressionInputItem &item)
  {
    return "InputItem_" + std::to_string(item.identifier);
  }

  static std::string custom_initial_name(const bNode &node, StringRef src_name)
  {
    return socket_items::variable_name_find_short<ExpressionInputItemsAccessor>(node, src_name);
  }

  static std::string validate_name(const StringRef name)
  {
    return socket_items::variable_name_validate(name);
  }
};

struct ExpressionItemsAccessor : public socket_items::SocketItemsAccessorDefaults {
  using ItemT = NodeExpressionItem;
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "NodeExpression";
  static constexpr bool has_type = true;
  static constexpr bool has_name = true;
  static constexpr bool has_custom_initial_name = true;
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_expression_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_expression_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_expression_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_expression_items";
  };
  struct rna_names {
    static constexpr StringRefNull items = "expression_items";
    static constexpr StringRefNull active_index = "active_expression_index";
  };

  static socket_items::SocketItemsRef<NodeExpressionItem> get_items_from_node(bNode &node)
  {
    auto *storage = static_cast<NodeExpression *>(node.storage);
    return {&storage->expression_items.items,
            &storage->expression_items.items_num,
            &storage->expression_items.active_index};
  }

  static void copy_item(const NodeExpressionItem &src, NodeExpressionItem &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }

  static void destruct_item(NodeExpressionItem *item)
  {
    MEM_delete(item->name);
  }

  static void blend_write_item(BlendWriter *writer, const ItemT &item);
  static void blend_read_data_item(BlendDataReader *reader, ItemT &item);

  static eNodeSocketDatatype get_socket_type(const NodeExpressionItem &item)
  {
    return eNodeSocketDatatype(item.socket_type);
  }

  static char **get_name(NodeExpressionItem &item)
  {
    return &item.name;
  }

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int ntree_type)
  {
    return ExpressionInputItemsAccessor::supports_socket_type(socket_type, ntree_type);
  }

  static void init_with_socket_type_and_name(bNode &node,
                                             NodeExpressionItem &item,
                                             const eNodeSocketDatatype socket_type,
                                             const char *name)
  {
    auto *storage = static_cast<NodeExpression *>(node.storage);
    item.socket_type = socket_type;
    item.identifier = storage->expression_items.next_identifier++;
    socket_items::set_item_name_and_make_unique<ExpressionItemsAccessor>(node, item, name);
  }

  static std::string socket_identifier_for_item(const NodeExpressionItem &item)
  {
    return "ExpressionItem_" + std::to_string(item.identifier);
  }

  static std::string custom_initial_name(const bNode & /*node*/, StringRef /*src_name*/)
  {
    return "Expression";
  }
};

}  // namespace blender::nodes
