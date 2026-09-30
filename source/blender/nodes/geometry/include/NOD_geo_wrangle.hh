/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <algorithm>
#include <string>

#include "DNA_node_types.h"
#include "DNA_array_utils.hh"

#include "NOD_socket_items.hh"
#include "NOD_socket_items_name_util.hh"

#include "RNA_access.hh"

namespace blender::nodes {

inline NodeGeometryWrangleItemKind wrangle_item_kind(const NodeGeometryWrangleInputItem &item)
{
  if (item.kind != GEO_NODE_WRANGLE_ITEM_AUTO) {
    return NodeGeometryWrangleItemKind(item.kind);
  }
  return item.socket_type == SOCK_GEOMETRY ? GEO_NODE_WRANGLE_ITEM_GEO :
                                             GEO_NODE_WRANGLE_ITEM_CONSTANT;
}

inline socket_items::SocketItemsRef<NodeGeometryWrangleInputItem> wrangle_items_ref(bNode &node)
{
  auto *storage = static_cast<NodeGeometryWrangle *>(node.storage);
  return {&storage->input_items.items,
          &storage->input_items.items_num,
          &storage->input_items.active_index};
}

/**
 * Compiled `chi`/`chf`/`chv`/… channel parameters. No + socket; Compile creates and
 * removes these. Single values, not fields.
 */
struct WrangleInputItemsAccessor : public socket_items::SocketItemsAccessorDefaults {
  using ItemT = NodeGeometryWrangleInputItem;
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "GeometryNodeWrangle";
  static constexpr bool has_type = true;
  static constexpr bool has_name = true;
  static constexpr bool has_custom_initial_name = true;
  static constexpr char unique_name_separator = '_';
  static constexpr bool has_name_validation = false;
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_wrangle_input_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_wrangle_input_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_wrangle_input_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_wrangle_input_items";
  };
  struct rna_names {
    static constexpr StringRefNull items = "input_items";
    static constexpr StringRefNull active_index = "active_input_index";
  };

  static socket_items::SocketItemsRef<NodeGeometryWrangleInputItem> get_items_from_node(bNode &node)
  {
    return wrangle_items_ref(node);
  }

  static void copy_item(const NodeGeometryWrangleInputItem &src, NodeGeometryWrangleInputItem &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }

  static void destruct_item(NodeGeometryWrangleInputItem *item)
  {
    MEM_delete(item->name);
  }

  static void blend_write_item(BlendWriter *writer, const ItemT &item);
  static void blend_read_data_item(BlendDataReader *reader, ItemT &item);

  static eNodeSocketDatatype get_socket_type(const NodeGeometryWrangleInputItem &item)
  {
    return eNodeSocketDatatype(item.socket_type);
  }

  static char **get_name(NodeGeometryWrangleInputItem &item)
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
                SOCK_STRING,
                SOCK_ROTATION);
  }

  static void init_with_socket_type_and_name(bNode &node,
                                             NodeGeometryWrangleInputItem &item,
                                             const eNodeSocketDatatype socket_type,
                                             const char *name)
  {
    auto *storage = static_cast<NodeGeometryWrangle *>(node.storage);
    item.socket_type = socket_type;
    item.kind = GEO_NODE_WRANGLE_ITEM_CONSTANT;
    item.identifier = storage->input_items.next_identifier++;
    socket_items::set_item_name_and_make_unique<WrangleInputItemsAccessor>(node, item, name);
  }

  static std::string socket_identifier_for_item(const NodeGeometryWrangleInputItem &item)
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

/** Extra geometry inputs (geo 1, 2, 3, …). The + only accepts Geometry. */
struct WrangleGeoItemsAccessor : public socket_items::SocketItemsAccessorDefaults {
  using ItemT = NodeGeometryWrangleInputItem;
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "GeometryNodeWrangle";
  static constexpr bool has_type = true;
  static constexpr bool has_name = true;
  static constexpr bool has_custom_initial_name = true;
  static constexpr char unique_name_separator = '_';
  static constexpr bool has_name_validation = false;
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_wrangle_geo_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_wrangle_geo_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_wrangle_geo_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_wrangle_geo_items";
  };
  struct rna_names {
    static constexpr StringRefNull items = "input_items";
    static constexpr StringRefNull active_index = "active_input_index";
  };

  static socket_items::SocketItemsRef<NodeGeometryWrangleInputItem> get_items_from_node(bNode &node)
  {
    return wrangle_items_ref(node);
  }

  static void copy_item(const NodeGeometryWrangleInputItem &src, NodeGeometryWrangleInputItem &dst)
  {
    dst = src;
    dst.name = BLI_strdup_null(dst.name);
  }

  static void destruct_item(NodeGeometryWrangleInputItem *item)
  {
    MEM_delete(item->name);
  }

  static eNodeSocketDatatype get_socket_type(const NodeGeometryWrangleInputItem &item)
  {
    return eNodeSocketDatatype(item.socket_type);
  }

  static char **get_name(NodeGeometryWrangleInputItem &item)
  {
    return &item.name;
  }

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int /*ntree_type*/)
  {
    return socket_type == SOCK_GEOMETRY;
  }

  static void init_with_socket_type_and_name(bNode &node,
                                             NodeGeometryWrangleInputItem &item,
                                             const eNodeSocketDatatype /*socket_type*/,
                                             const char * /*name*/)
  {
    auto *storage = static_cast<NodeGeometryWrangle *>(node.storage);
    item.socket_type = SOCK_GEOMETRY;
    item.kind = GEO_NODE_WRANGLE_ITEM_GEO;
    item.identifier = storage->input_items.next_identifier++;
    /* Hardcoded input is geo 0; extras are 1, 2, 3, … */
    int geo_n = 1;
    for (int i = 0; i < storage->input_items.items_num; i++) {
      if (&storage->input_items.items[i] == &item) {
        continue;
      }
      if (wrangle_item_kind(storage->input_items.items[i]) == GEO_NODE_WRANGLE_ITEM_GEO) {
        geo_n++;
      }
    }
    const std::string numbered = std::to_string(geo_n);
    socket_items::set_item_name_and_make_unique<WrangleGeoItemsAccessor>(
        node, item, numbered.c_str());
  }

  static std::string socket_identifier_for_item(const NodeGeometryWrangleInputItem &item)
  {
    return "InputItem_" + std::to_string(item.identifier);
  }

  static std::string custom_initial_name(const bNode & /*node*/, StringRef /*src_name*/)
  {
    return "";
  }
};

}  // namespace blender::nodes
