/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup geo
 *
 * SPOM Prisms node removed. GEO_NODE_SPOM_PRISMS remains reserved for ABI stability.
 */

#include "NOD_register.hh"

namespace blender::nodes::node_geo_spom_prisms_cc {

static void node_register() {}

NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_spom_prisms_cc
