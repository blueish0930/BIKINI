/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup nodes
 *
 * Socket items of the Image Wrangle node (GPU Texture Editor). The node stores the same
 * #NodeShaderWrangle data as the Shader Wrangle and is used the same way, so the accessors only
 * replace what names the node: its type, its operators and the socket types an image can carry.
 */

#include "BLI_listbase.hh"

#include "NOD_shader_wrangle.hh"

namespace blender::nodes {

/** Values an Image Wrangle can read from or write to an image, or take as a single value. */
inline bool image_wrangle_supports_socket_type(const eNodeSocketDatatype socket_type)
{
  return ELEM(socket_type, SOCK_FLOAT, SOCK_INT, SOCK_BOOLEAN, SOCK_VECTOR, SOCK_RGBA);
}

/** Single values created by Compile from `chf("name")`, `chi`, `chv`, `chb` and `chc`. */
struct ImageWrangleInputItemsAccessor : public ShaderWrangleInputItemsAccessor {
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "ImageNodeWrangle";
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_image_wrangle_input_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_image_wrangle_input_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_image_wrangle_input_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_image_wrangle_input_items";
  };

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int /*ntree_type*/)
  {
    return image_wrangle_supports_socket_type(socket_type);
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
    socket_items::set_item_name_and_make_unique<ImageWrangleInputItemsAccessor>(node, item, name);
  }
};

/** Images (or single values) read per pixel. The socket name is the name used in the code. */
struct ImageWrangleFieldItemsAccessor : public ShaderWrangleFieldItemsAccessor {
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "ImageNodeWrangle";
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_image_wrangle_field_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_image_wrangle_field_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_image_wrangle_field_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_image_wrangle_field_items";
  };

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int /*ntree_type*/)
  {
    return image_wrangle_supports_socket_type(socket_type);
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
    socket_items::set_item_name_and_make_unique<ImageWrangleFieldItemsAccessor>(node, item, name);
  }

  static std::string custom_initial_name(const bNode & /*node*/, StringRef src_name)
  {
    if (!src_name.is_empty()) {
      return src_name;
    }
    return "image";
  }
};

/** Images written per pixel. The socket name is the name assigned in the code. */
struct ImageWrangleOutputItemsAccessor : public ShaderWrangleOutputItemsAccessor {
  static StructRNA **item_srna;
  static int node_type;
  static constexpr StringRefNull node_idname = "ImageNodeWrangle";
  struct operator_idnames {
    static constexpr StringRefNull add_item = "NODE_OT_image_wrangle_output_item_add";
    static constexpr StringRefNull remove_item = "NODE_OT_image_wrangle_output_item_remove";
    static constexpr StringRefNull move_item = "NODE_OT_image_wrangle_output_item_move";
  };
  struct ui_idnames {
    static constexpr StringRefNull list = "DATA_UL_image_wrangle_output_items";
  };

  static bool supports_socket_type(const eNodeSocketDatatype socket_type, const int /*ntree_type*/)
  {
    return image_wrangle_supports_socket_type(socket_type);
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
    socket_items::set_item_name_and_make_unique<ImageWrangleOutputItemsAccessor>(node, item, name);
  }
};

}  // namespace blender::nodes
