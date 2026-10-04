/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "MEM_guardedalloc.h"

#include "BLI_string.hh"

#include "DNA_node_types.h"

#include "NOD_socket_items.hh"

namespace blender::nodes {

/**
 * Color field items on the Fluid Simulation zone (single sockets + extend),
 * matching Simulation / Repeat zone item UX.
 */
struct FluidColorItemsAccessor : public socket_items::SocketItemsAccessorDefaults {
  using ItemT = NodeImageFluidColorItem;
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "ImageNodeFluidSimOutput";
  static constexpr bool has_type = false;
  static constexpr bool has_name = true;
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_fluid_zone_color_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_fluid_zone_color_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_fluid_zone_color_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_fluid_zone_color";
  };
  struct rna_names {
    static constexpr StringRefNull items = "color_items";
    static constexpr StringRefNull active_index = "color_active_index";
  };

  static socket_items::SocketItemsRef<NodeImageFluidColorItem> get_items_from_node(bNode &node)
  {
    auto *storage = static_cast<NodeImageFluidSimOutput *>(node.storage);
    return {&storage->color_items, &storage->color_items_num, &storage->color_active_index};
  }

  static void copy_item(const NodeImageFluidColorItem &src, NodeImageFluidColorItem &dst)
  {
    dst.identifier = src.identifier;
    dst.name = BLI_strdup_null(src.name);
  }

  static void destruct_item(NodeImageFluidColorItem *item)
  {
    MEM_SAFE_DELETE(item->name);
  }

  static void blend_write_item(BlendWriter *writer, const ItemT &item);
  static void blend_read_data_item(BlendDataReader *reader, ItemT &item);

  static eNodeSocketDatatype get_socket_type(const NodeImageFluidColorItem & /*item*/)
  {
    return SOCK_RGBA;
  }

  static char **get_name(NodeImageFluidColorItem &item)
  {
    return &item.name;
  }

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int /*ntree_type*/)
  {
    return socket_type == SOCK_RGBA;
  }

  static void init_with_name(bNode &node, NodeImageFluidColorItem &item, const char *name)
  {
    auto *storage = static_cast<NodeImageFluidSimOutput *>(node.storage);
    item.identifier = storage->color_next_identifier++;
    socket_items::set_item_name_and_make_unique<FluidColorItemsAccessor>(node, item, name);
  }

  static void init_with_socket_type_and_name(bNode &node,
                                             NodeImageFluidColorItem &item,
                                             const eNodeSocketDatatype /*socket_type*/,
                                             const char *name)
  {
    init_with_name(node, item, name);
  }

  static std::string socket_identifier_for_item(const NodeImageFluidColorItem &item)
  {
    return "Color_" + std::to_string(item.identifier);
  }
};

/** Multi-frame fluid cache control used by evaluation + RNA param updates. */
namespace image_fluid_zone {
int resume_frame(const bNodeTree &ntree, int target_frame);
void invalidate_after(const bNodeTree &ntree, int keep_frame);
/** Force-clear GPU + dense cache for one Fluid Output zone (N-panel Reset). */
void reset(const bNodeTree &ntree, int32_t output_node_id);
}  // namespace image_fluid_zone

}  // namespace blender::nodes
