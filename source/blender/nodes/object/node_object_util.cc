/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <optional>

#include "BKE_node_runtime.hh"

#include "NOD_socket_search_link.hh"

#include "node_object_util.hh"

namespace blender {

bool obj_node_poll_default(const bke::bNodeType * /*ntype*/,
                           const bNodeTree *ntree,
                           const char **r_disabled_hint)
{
  if (!STREQ(ntree->idname, "ObjectNodeTree")) {
    *r_disabled_hint = RPT_("Not an object node tree");
    return false;
  }
  return true;
}

void obj_node_type_base(bke::bNodeType *ntype,
                        UString idname,
                        const std::optional<int16_t> legacy_type)
{
  bke::node_type_base(*ntype, idname, legacy_type);

  ntype->poll = obj_node_poll_default;
  ntype->insert_link = node_insert_link_default;
  ntype->gather_link_search_ops = nodes::search_link_ops_for_basic_node;
}

}  // namespace blender
