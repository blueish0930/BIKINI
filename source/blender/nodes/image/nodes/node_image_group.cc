/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Image Process: node group instance type (ImageNodeGroup).
 * Without this registration, group instances show as Undefined (red). */

#include "DNA_node_types.h"

#include "BKE_node.hh"

#include "NOD_common.hh"
#include "node_common.h"

#include "node_image_util.hh"

#include "RNA_access.hh"

#include "UI_resources.hh"

namespace blender::nodes::node_image_group_cc {

static void node_register()
{
  static bke::bNodeType ntype;

  /* Cannot use img_node_type_base — that would map to a shared NODE_GROUP type id. */
  bke::node_type_base_custom(ntype, "ImageNodeGroup", "Group", "GROUP", NODE_CLASS_GROUP);
  ntype.enum_name_legacy = "GROUP";
  ntype.type_legacy = NODE_GROUP;
  ntype.ui_icon = ICON_NODETREE;
  ntype.poll = img_node_poll_default;
  ntype.poll_instance = node_group_poll_instance;
  ntype.insert_link = node_insert_link_default;
  ntype.ui_class = node_group_ui_class;
  ntype.ui_description_fn = node_group_ui_description;
  ntype.rna_ext.srna = RNA_struct_find("ImageNodeGroup");
  BLI_assert(ntype.rna_ext.srna != nullptr);
  RNA_struct_blender_type_set(ntype.rna_ext.srna, &ntype);

  ntype.minwidth = bke::NodeWidth::GroupMin;
  ntype.labelfunc = node_group_label;
  ntype.declare = nodes::node_group_declare;
  /* Evaluation: NodeGroupOperation detects is_group() before get_compositor_operation. */

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_group_cc

namespace blender {

void register_node_type_img_custom_group(bke::bNodeType *ntype)
{
  if (ntype->poll == nullptr) {
    ntype->poll = img_node_poll_default;
  }
  if (ntype->insert_link == nullptr) {
    ntype->insert_link = node_insert_link_default;
  }
  ntype->declare = nodes::node_group_declare;
}

}  // namespace blender
