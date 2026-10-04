/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>

#include "DNA_node_types.h"

#include "BKE_node_legacy_types.hh"

#include "node_util.hh"

#include "NOD_object.hh"
#include "NOD_register.hh"
#include "NOD_socket.hh"
#include "NOD_socket_declarations.hh"
#include "NOD_socket_search_link.hh"

namespace blender {

bool obj_node_poll_default(const bke::bNodeType *ntype,
                           const bNodeTree *ntree,
                           const char **r_disabled_hint);

void obj_node_type_base(bke::bNodeType *ntype,
                        UString idname,
                        std::optional<int16_t> legacy_type = std::nullopt);

}  // namespace blender
