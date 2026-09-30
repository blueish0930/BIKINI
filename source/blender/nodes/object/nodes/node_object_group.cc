/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_node_types.h"

#include "BKE_node.hh"

#include "NOD_common.hh"
#include "node_common.h"

#include "node_object_util.hh"

#include "RNA_access.hh"

namespace blender::nodes::node_object_group_cc {

static void node_register()
{
  static bke::bNodeType ntype;

  bke::node_type_base_custom(ntype, "ObjectNodeGroup", "Group", "GROUP", NODE_CLASS_GROUP);
  ntype.enum_name_legacy = "GROUP";
  ntype.type_legacy = NODE_GROUP;
  ntype.poll = obj_node_poll_default;
  ntype.poll_instance = node_group_poll_instance;
  ntype.insert_link = node_insert_link_default;
  ntype.ui_class = node_group_ui_class;
  ntype.ui_description_fn = node_group_ui_description;
  ntype.rna_ext.srna = RNA_struct_find("ObjectNodeGroup");
  BLI_assert(ntype.rna_ext.srna != nullptr);
  RNA_struct_blender_type_set(ntype.rna_ext.srna, &ntype);

  ntype.minwidth = bke::NodeWidth::GroupMin;
  ntype.labelfunc = node_group_label;
  ntype.declare = nodes::node_group_declare;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_object_group_cc

namespace blender {

void register_node_type_obj_custom_group(bke::bNodeType *ntype)
{
  if (ntype->poll == nullptr) {
    ntype->poll = obj_node_poll_default;
  }
  if (ntype->insert_link == nullptr) {
    ntype->insert_link = node_insert_link_default;
  }
  ntype->declare = nodes::node_group_declare;
}

}  // namespace blender
