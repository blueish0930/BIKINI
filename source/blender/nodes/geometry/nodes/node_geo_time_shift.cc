/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_time_shift_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Geometry"_ustr)
      .is_default_link_socket()
      .description("Geometry to output at another frame");
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .propagate_all()
      .align_with_previous()
      .description("The input geometry as it is at the given frame");
  b.add_input<decl::Float>("Frame"_ustr)
      .default_value(1.0f)
      .description(
          "Scene frame to output the geometry of, in the units of the Frame output of the Scene "
          "Time node. Subframes are supported");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry"_ustr);
  const float frame = params.extract_input<float>("Frame"_ustr);

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
  /* A frame that is not known yet is computed right after this evaluation, followed by another
   * evaluation of this node. The message only stays when that was not possible. */
  if (!user_data->call_data->time_shift_params->evaluate(
          found_id->id, frame, std::move(geometry), shifted_geometry))
  {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("The frame is not available, the input is passed through"));
  }
  params.set_output("Geometry"_ustr, std::move(shifted_geometry));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeTimeShift"_ustr, GEO_NODE_TIME_SHIFT);
  ntype.ui_name = "Time Shift";
  ntype.ui_description =
      "Output the geometry as it is at another scene frame. Animation in front of the node is "
      "evaluated at that frame, simulations are stepped up to it";
  ntype.enum_name_legacy = "TIME_SHIFT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_time_shift_cc
