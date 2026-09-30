/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>

#include "DEG_depsgraph_query.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_time_shift_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Geometry"_ustr)
      .is_default_link_socket()
      .description("Geometry to cache at the current scene frame");
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .propagate_all()
      .align_with_previous()
      .description("Geometry cooked at the absolute Frame (Houdini-style Time Shift)");
  b.add_input<decl::Int>("Frame"_ustr)
      .default_value(1)
      .min(1)
      .max(100000)
      .description(
          "Absolute scene frame to output (minimum 1). Uses Time Shift's own cache; cooks on "
          "demand when missing");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry"_ustr);
  /* Clamp hard: negative Frame values used to poison the absolute-frame map and sim resume. */
  const int frame = std::max(1, params.extract_input<int>("Frame"_ustr));

  GeoNodesUserData *user_data = params.user_data();
  if (!user_data || !user_data->call_data->time_shift_params) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Time Shift requires a Geometry Nodes modifier context"));
    params.set_output("Geometry"_ustr, std::move(geometry));
    return;
  }

  const std::optional<FoundNestedNodeID> found_id = find_nested_node_id(
      *user_data, params.node().identifier);
  if (!found_id) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Time Shift is not registered in the root node tree"));
    params.set_output("Geometry"_ustr, std::move(geometry));
    return;
  }
  if (found_id->is_in_loop || found_id->is_in_closure || found_id->is_in_simulation) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("Time Shift cannot be used in a loop, closure, or simulation zone"));
    params.set_output("Geometry"_ustr, std::move(geometry));
    return;
  }

  GeometrySet shifted_geometry;
  int missing_frame = frame;
  /* On miss, a silent background filler computes up to this absolute frame (reusing simulation
   * cache when possible). Do not spam "not in cache" — that is the expected fill path. */
  user_data->call_data->time_shift_params->evaluate(
      found_id->id, frame, std::move(geometry), shifted_geometry, missing_frame);
  UNUSED_VARS(missing_frame);
  params.set_output("Geometry"_ustr, std::move(shifted_geometry));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeTimeShift"_ustr, GEO_NODE_TIME_SHIFT);
  ntype.ui_name = "Time Shift";
  ntype.ui_description =
      "Output geometry at an absolute scene frame. Scene Time ± offset is sampled on the current "
      "playhead; animated inputs cook that frame directly; unbaked simulations play forward and "
      "block until the frame is ready";
  ntype.enum_name_legacy = "TIME_SHIFT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_time_shift_cc
