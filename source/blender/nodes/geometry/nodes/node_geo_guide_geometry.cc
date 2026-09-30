/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"

#include "DNA_modifier_types.h"
#include "DNA_space_types.h"

#include "ED_node.hh"

#include "NOD_node_extra_info.hh"
#include "NOD_socket_search_link.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_guide_geometry_cc {

static const EnumPropertyItem display_items[] = {
    {GEO_NODE_GUIDE_DISPLAY_WIRE, "WIRE", 0, "Wire", "Draw edges and curves as wires"},
    {GEO_NODE_GUIDE_DISPLAY_SOLID, "SOLID", 0, "Solid", "Draw mesh faces as a translucent overlay"},
    {GEO_NODE_GUIDE_DISPLAY_WIRE_AND_SOLID,
     "WIRE_AND_SOLID",
     0,
     "Wire and Solid",
     "Draw both faces and wires"},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Bool>("Show"_ustr)
      .default_value(true)
      .hide_value()
      .description("Turn this guide overlay on or off");
  b.add_input<decl::Geometry>("Geometry"_ustr)
      .description(
          "Geometry drawn as a viewport guide. It is not part of the modifier output, is not "
          "rendered, and does not appear in the Spreadsheet");
  b.add_input<decl::Color>("Color"_ustr)
      .default_value({1.0f, 0.55f, 0.15f, 0.85f})
      .description("Overlay color used when drawing the guide in the 3D Viewport");
  b.add_input<decl::Menu>("Display"_ustr)
      .static_items(display_items)
      .default_value(GEO_NODE_GUIDE_DISPLAY_WIRE)
      .optional_label()
      .description("How the guide geometry is drawn in the 3D Viewport");
  b.add_input<decl::Bool>("X-Ray"_ustr)
      .default_value(true)
      .description("Draw the guide in front of other geometry so it stays visible");
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const bNodeSocket &other_socket = params.other_socket();
  if (other_socket.in_out != SOCK_OUT) {
    return;
  }
  if (other_socket.type != SOCK_GEOMETRY) {
    return;
  }
  params.add_item(IFACE_("Geometry"), [](LinkSearchOpParams &params) {
    bNode &node = params.add_node("GeometryNodeGuideGeometry"_ustr);
    params.update_and_connect_available_socket(node, "Geometry"_ustr);
  });
}

static void node_extra_info(NodeExtraInfoParams &params)
{
  SpaceNode *snode = CTX_wm_space_node(&params.C);
  if (!snode) {
    return;
  }
  if (std::optional<ed::space_node::ObjectAndModifier> object_and_modifier =
          ed::space_node::get_geometry_nodes_modifier_for_node_editor(*snode))
  {
    const NodesModifierData &nmd = *object_and_modifier->nmd;
    if (!(nmd.modifier.mode & eModifierMode_Realtime)) {
      NodeExtraInfoRow row;
      row.icon = ICON_STATUS_ERROR;
      row.text = TIP_("Modifier disabled");
      row.tooltip = TIP_("The guide is not shown because the modifier is disabled");
      params.rows.append(std::move(row));
    }
  }
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeGuideGeometry"_ustr, GEO_NODE_GUIDE_GEOMETRY);
  ntype.ui_name = "Guide Geometry";
  ntype.ui_description =
      "Draw extra geometry in the 3D Viewport while the object is selected. Hidden when the "
      "modifier is disabled. Does not affect render or the modifier output, and is not shown in "
      "the Spreadsheet";
  ntype.enum_name_legacy = "GUIDE_GEOMETRY";
  ntype.nclass = NODE_CLASS_OUTPUT;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.get_extra_info = node_extra_info;
  ntype.default_width = bke::NodeWidth::_160;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_guide_geometry_cc
