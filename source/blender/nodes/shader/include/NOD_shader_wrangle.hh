/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <algorithm>
#include <string>

#include "BLI_string_ref.hh"

#include "DNA_node_types.h"
#include "DNA_array_utils.hh"

#include "NOD_socket_items.hh"
#include "NOD_socket_items_name_util.hh"

#include "RNA_access.hh"

namespace blender::nodes {

inline NodeShaderWrangleItemKind shader_wrangle_item_kind(const NodeShaderWrangleItem &item)
{
  return NodeShaderWrangleItemKind(item.kind);
}

inline socket_items::SocketItemsRef<NodeShaderWrangleItem> shader_wrangle_items_ref(bNode &node)
{
  auto *storage = static_cast<NodeShaderWrangle *>(node.storage);
  return {&storage->input_items.items,
          &storage->input_items.items_num,
          &storage->input_items.active_index};
}

inline socket_items::SocketItemsRef<NodeShaderWrangleItem> shader_wrangle_output_items_ref(
    bNode &node)
{
  auto *storage = static_cast<NodeShaderWrangle *>(node.storage);
  return {&storage->output_items.items,
          &storage->output_items.items_num,
          &storage->output_items.active_index};
}

/**
 * Compiled `chi`/`chf`/`chv`/… channel parameters. No + socket; Compile creates and
 * removes these. Single values, not fields.
 */
struct ShaderWrangleInputItemsAccessor : public socket_items::SocketItemsAccessorDefaults {
  using ItemT = NodeShaderWrangleItem;
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "ShaderNodeWrangle";
  static constexpr bool has_type = true;
  static constexpr bool has_name = true;
  static constexpr bool has_custom_initial_name = true;
  static constexpr char unique_name_separator = '_';
  static constexpr bool has_name_validation = false;
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_shader_wrangle_input_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_shader_wrangle_input_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_shader_wrangle_input_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_shader_wrangle_input_items";
  };
  struct rna_names {
    static constexpr StringRefNull items = "input_items";
    static constexpr StringRefNull active_index = "active_input_index";
  };

  static socket_items::SocketItemsRef<ItemT> get_items_from_node(bNode &node)
  {
    return shader_wrangle_items_ref(node);
  }

  static void copy_item(const ItemT &src, ItemT &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }

  static void destruct_item(ItemT *item)
  {
    MEM_delete(item->name);
  }

  static void blend_write_item(BlendWriter *writer, const ItemT &item);
  static void blend_read_data_item(BlendDataReader *reader, ItemT &item);

  static eNodeSocketDatatype get_socket_type(const ItemT &item)
  {
    return eNodeSocketDatatype(item.socket_type);
  }

  static char **get_name(ItemT &item)
  {
    return &item.name;
  }

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int /*ntree_type*/)
  {
    return ELEM(socket_type,
                SOCK_FLOAT,
                SOCK_INT,
                SOCK_BOOLEAN,
                SOCK_VECTOR,
                SOCK_RGBA,
                SOCK_MATRIX,
                SOCK_ROTATION);
  }

  static void init_with_socket_type_and_name(bNode &node,
                                             ItemT &item,
                                             const eNodeSocketDatatype socket_type,
                                             const char *name)
  {
    auto *storage = static_cast<NodeShaderWrangle *>(node.storage);
    item.socket_type = socket_type;
    item.kind = SH_NODE_WRANGLE_ITEM_CONSTANT;
    item.identifier = storage->input_items.next_identifier++;
    socket_items::set_item_name_and_make_unique<ShaderWrangleInputItemsAccessor>(node, item, name);
  }

  static std::string socket_identifier_for_item(const ItemT &item)
  {
    return "InputItem_" + std::to_string(item.identifier);
  }

  static std::string custom_initial_name(const bNode & /*node*/, StringRef src_name)
  {
    if (!src_name.is_empty()) {
      return src_name;
    }
    return "parm";
  }
};

/**
 * Field inputs captured as named attributes. Socket name is the attribute name
 * (`i@name`, `f@name`, `v@name`, …).
 */
struct ShaderWrangleFieldItemsAccessor : public socket_items::SocketItemsAccessorDefaults {
  using ItemT = NodeShaderWrangleItem;
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "ShaderNodeWrangle";
  static constexpr bool has_type = true;
  static constexpr bool has_name = true;
  static constexpr bool has_custom_initial_name = true;
  static constexpr char unique_name_separator = '_';
  static constexpr bool has_name_validation = true;
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_shader_wrangle_field_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_shader_wrangle_field_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_shader_wrangle_field_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_shader_wrangle_field_items";
  };
  struct rna_names {
    static constexpr StringRefNull items = "input_items";
    static constexpr StringRefNull active_index = "active_input_index";
  };

  static socket_items::SocketItemsRef<ItemT> get_items_from_node(bNode &node)
  {
    return shader_wrangle_items_ref(node);
  }

  static void copy_item(const ItemT &src, ItemT &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }

  static void destruct_item(ItemT *item)
  {
    MEM_delete(item->name);
  }

  static void blend_write_item(BlendWriter *writer, const ItemT &item);
  static void blend_read_data_item(BlendDataReader *reader, ItemT &item);

  static eNodeSocketDatatype get_socket_type(const ItemT &item)
  {
    return eNodeSocketDatatype(item.socket_type);
  }

  static char **get_name(ItemT &item)
  {
    return &item.name;
  }

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int /*ntree_type*/)
  {
    return ELEM(socket_type,
                SOCK_FLOAT,
                SOCK_INT,
                SOCK_VECTOR,
                SOCK_RGBA,
                SOCK_ROTATION,
                SOCK_MATRIX);
  }

  static void init_with_socket_type_and_name(bNode &node,
                                             ItemT &item,
                                             const eNodeSocketDatatype socket_type,
                                             const char *name)
  {
    auto *storage = static_cast<NodeShaderWrangle *>(node.storage);
    item.socket_type = socket_type;
    item.kind = SH_NODE_WRANGLE_ITEM_FIELD;
    item.identifier = storage->input_items.next_identifier++;
    socket_items::set_item_name_and_make_unique<ShaderWrangleFieldItemsAccessor>(node, item, name);
  }

  static std::string socket_identifier_for_item(const ItemT &item)
  {
    return "InputItem_" + std::to_string(item.identifier);
  }

  static std::string custom_initial_name(const bNode & /*node*/, StringRef src_name)
  {
    if (!src_name.is_empty()) {
      return src_name;
    }
    return "attr";
  }

  static std::string validate_name(const StringRef name)
  {
    return socket_items::variable_name_validate(name);
  }
};

/** Drop Field-panel items that have no incoming link. */
inline const bNodeSocket *shader_wrangle_find_input(const bNode &node, const StringRef identifier)
{
  for (const bNodeSocket &socket : node.inputs) {
    if (StringRef(socket.identifier) == identifier) {
      return &socket;
    }
  }
  return nullptr;
}

inline bool shader_wrangle_field_socket_is_connected(const bNodeSocket &socket)
{
  if (socket.link != nullptr && !(socket.link->flag & NODE_LINK_MUTED)) {
    return true;
  }
  if ((socket.flag & SOCK_IS_LINKED) == 0) {
    return false;
  }
  return socket.link != nullptr;
}

inline void remove_unlinked_shader_wrangle_field_items(bNodeTree &ntree, bNode &node)
{
  if (node.storage == nullptr) {
    return;
  }
  ntree.ensure_topology_cache();
  socket_items::SocketItemsRef ref = shader_wrangle_items_ref(node);
  if (*ref.items_num <= 0) {
    return;
  }
  dna::array::remove_if<NodeShaderWrangleItem>(
      ref.items,
      ref.items_num,
      [&](const NodeShaderWrangleItem &item) -> bool {
        if (shader_wrangle_item_kind(item) != SH_NODE_WRANGLE_ITEM_FIELD) {
          return false;
        }
        const std::string identifier = ShaderWrangleFieldItemsAccessor::socket_identifier_for_item(
            item);
        const bNodeSocket *socket = shader_wrangle_find_input(node, identifier);
        if (socket == nullptr) {
          /* Item was just added; sockets are rebuilt after this pass. */
          return false;
        }
        return !shader_wrangle_field_socket_is_connected(*socket);
      },
      ShaderWrangleFieldItemsAccessor::destruct_item);
  if (ref.active_index) {
    *ref.active_index = std::clamp(*ref.active_index, 0, std::max(*ref.items_num - 1, 0));
  }
}

/**
 * Outputs on the right. Socket name is the VEX identifier (`Output = …`).
 * Identifier 0 stays `"Output"` so older files keep their links.
 */
struct ShaderWrangleOutputItemsAccessor : public socket_items::SocketItemsAccessorDefaults {
  using ItemT = NodeShaderWrangleItem;
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "ShaderNodeWrangle";
  static constexpr bool has_type = true;
  static constexpr bool has_name = true;
  static constexpr bool has_custom_initial_name = true;
  static constexpr bool has_name_validation = true;
  static constexpr char unique_name_separator = '_';
  static constexpr int min_items_num = 0;
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_shader_wrangle_output_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_shader_wrangle_output_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_shader_wrangle_output_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_shader_wrangle_output_items";
  };
  struct rna_names {
    static constexpr StringRefNull items = "output_items";
    static constexpr StringRefNull active_index = "active_output_index";
  };

  static socket_items::SocketItemsRef<ItemT> get_items_from_node(bNode &node)
  {
    return shader_wrangle_output_items_ref(node);
  }

  static void copy_item(const ItemT &src, ItemT &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }

  static void destruct_item(ItemT *item)
  {
    MEM_delete(item->name);
  }

  static void blend_write_item(BlendWriter *writer, const ItemT &item);
  static void blend_read_data_item(BlendDataReader *reader, ItemT &item);

  static eNodeSocketDatatype get_socket_type(const ItemT &item)
  {
    return eNodeSocketDatatype(item.socket_type);
  }

  static char **get_name(ItemT &item)
  {
    return &item.name;
  }

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int /*ntree_type*/)
  {
    /* Same as the GLSL node: values, not closures. Plug into Emission Color / Displacement. */
    return ELEM(socket_type, SOCK_FLOAT, SOCK_INT, SOCK_BOOLEAN, SOCK_VECTOR, SOCK_RGBA);
  }

  static void init_with_socket_type_and_name(bNode &node,
                                             ItemT &item,
                                             const eNodeSocketDatatype socket_type,
                                             const char *name)
  {
    auto *storage = static_cast<NodeShaderWrangle *>(node.storage);
    item.socket_type = socket_type;
    item.kind = 0;
    item.identifier = storage->output_items.next_identifier++;
    socket_items::set_item_name_and_make_unique<ShaderWrangleOutputItemsAccessor>(node, item, name);
  }

  static std::string socket_identifier_for_item(const ItemT &item)
  {
    if (item.identifier == 0) {
      return "Output";
    }
    return "OutputItem_" + std::to_string(item.identifier);
  }

  static std::string custom_initial_name(const bNode & /*node*/, StringRef /*src_name*/)
  {
    /* Keep VEX names stable (`Output = …`). Type comes from the linked socket. */
    return "Output";
  }

  static std::string validate_name(const StringRef name)
  {
    return socket_items::variable_name_validate(name);
  }
};

}  // namespace blender::nodes
