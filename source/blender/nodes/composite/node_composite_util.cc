/* SPDX-FileCopyrightText: 2006 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 */

#include <optional>

#include "BKE_node_runtime.hh"

#include "NOD_socket_search_link.hh"

#include "node_composite_util.hh"

namespace blender {

bool cmp_node_poll_default(const bke::bNodeType * /*ntype*/,
                           const bNodeTree *ntree,
                           const char **r_disabled_hint)
{
  /* ImageNodeTree reuses compositor ops for Color/Filter/Texture/Transform/Utilities. */
  if (!STREQ(ntree->idname, "CompositorNodeTree") && !STREQ(ntree->idname, "ImageNodeTree")) {
    *r_disabled_hint = RPT_("Not a compositor or image node tree");
    return false;
  }
  return true;
}

bool cmp_node_poll_compositor_only(const bke::bNodeType *ntype,
                                   const bNodeTree *ntree,
                                   const char **r_disabled_hint)
{
  if (STREQ(ntree->idname, "ImageNodeTree")) {
    *r_disabled_hint = RPT_("Not available in GPU Texture Editor");
    return false;
  }
  return cmp_node_poll_default(ntype, ntree, r_disabled_hint);
}

void cmp_node_type_base(bke::bNodeType *ntype,
                        UString idname,
                        const std::optional<int16_t> legacy_type)
{
  bke::node_type_base(*ntype, idname, legacy_type);

  ntype->poll = cmp_node_poll_default;
  ntype->insert_link = node_insert_link_default;
  ntype->gather_link_search_ops = nodes::search_link_ops_for_basic_node;
}

}  // namespace blender
