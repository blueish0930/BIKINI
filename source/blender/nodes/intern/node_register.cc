/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 */

#include "NOD_composite.hh"
#include "NOD_geometry.hh"
#include "NOD_image.hh" /* IMAGE_NODES_MVP */
#include "NOD_object.hh"
#include "NOD_register.hh"
#include "NOD_socket.hh"

#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"

#include "BLT_translation.hh"

#include "UI_resources.hh"

namespace blender {

static bool node_undefined_poll(const bke::bNodeType * /*ntype*/,
                                const bNodeTree * /*nodetree*/,
                                const char ** /*r_disabled_hint*/)
{
  /* this type can not be added deliberately, it's just a placeholder */
  return false;
}

/* register fallback types used for undefined tree, nodes, sockets */
static void register_undefined_types()
{
  /* NOTE: these types are not registered in the type hashes,
   * they are just used as placeholders in case the actual types are not registered.
   */

  bke::NodeTreeTypeUndefined.type = NTREE_UNDEFINED;
  bke::NodeTreeTypeUndefined.idname = "NodeTreeUndefined"_ustr;
  bke::NodeTreeTypeUndefined.ui_name = N_("Undefined");
  bke::NodeTreeTypeUndefined.ui_description = N_("Undefined Node Tree Type");

  bke::node_type_base_custom(bke::NodeTypeUndefined, "NodeUndefined", "Undefined", "UNDEFINED", 0);
  bke::NodeTypeUndefined.poll = node_undefined_poll;

  bke::NodeSocketTypeUndefined.idname = "NodeSocketUndefined"_ustr;
  /* extra type info for standard socket types */
  bke::NodeSocketTypeUndefined.type = SOCK_CUSTOM;
  bke::NodeSocketTypeUndefined.subtype = PROP_NONE;

  bke::NodeSocketTypeUndefined.use_link_limits_of_type = true;
  bke::NodeSocketTypeUndefined.input_link_limit = 0xFFF;
  bke::NodeSocketTypeUndefined.output_link_limit = 0xFFF;
}

template<typename StorageT> static const int &output_node_id_from_storage(const bNode &input_bnode)
{
  static int none = 0;
  if (input_bnode.storage == nullptr) {
    none = 0;
    return none;
  }
  return static_cast<const StorageT *>(input_bnode.storage)->output_node_id;
}

class SimulationZoneType : public bke::bNodeZoneType {
 public:
  SimulationZoneType()
  {
    this->input_idname = "GeometryNodeSimulationInput"_ustr;
    this->output_idname = "GeometryNodeSimulationOutput"_ustr;
    this->input_type = GEO_NODE_SIMULATION_INPUT;
    this->output_type = GEO_NODE_SIMULATION_OUTPUT;
    this->theme_id = TH_NODE_ZONE_SIMULATION;
  }

  const int &get_corresponding_output_id(const bNode &input_bnode) const override
  {
    BLI_assert(input_bnode.type_legacy == this->input_type);
    return output_node_id_from_storage<NodeGeometrySimulationInput>(input_bnode);
  }
};

class RepeatZoneType : public bke::bNodeZoneType {
 public:
  RepeatZoneType()
  {
    this->input_idname = "GeometryNodeRepeatInput"_ustr;
    this->output_idname = "GeometryNodeRepeatOutput"_ustr;
    this->input_type = GEO_NODE_REPEAT_INPUT;
    this->output_type = GEO_NODE_REPEAT_OUTPUT;
    this->theme_id = TH_NODE_ZONE_REPEAT;
  }

  const int &get_corresponding_output_id(const bNode &input_bnode) const override
  {
    BLI_assert(input_bnode.type_legacy == this->input_type);
    return output_node_id_from_storage<NodeGeometryRepeatInput>(input_bnode);
  }
};

class ForeachGeometryElementZoneType : public bke::bNodeZoneType {
 public:
  ForeachGeometryElementZoneType()
  {
    this->input_idname = "GeometryNodeForeachGeometryElementInput"_ustr;
    this->output_idname = "GeometryNodeForeachGeometryElementOutput"_ustr;
    this->input_type = GEO_NODE_FOREACH_GEOMETRY_ELEMENT_INPUT;
    this->output_type = GEO_NODE_FOREACH_GEOMETRY_ELEMENT_OUTPUT;
    this->theme_id = TH_NODE_ZONE_FOREACH_GEOMETRY_ELEMENT;
  }

  const int &get_corresponding_output_id(const bNode &input_bnode) const override
  {
    BLI_assert(input_bnode.type_legacy == this->input_type);
    return output_node_id_from_storage<NodeGeometryForeachGeometryElementInput>(input_bnode);
  }
};

class ClosureZoneType : public bke::bNodeZoneType {
 public:
  ClosureZoneType()
  {
    this->input_idname = "NodeClosureInput"_ustr;
    this->output_idname = "NodeClosureOutput"_ustr;
    this->input_type = NODE_CLOSURE_INPUT;
    this->output_type = NODE_CLOSURE_OUTPUT;
    this->theme_id = TH_NODE_ZONE_CLOSURE;
  }

  const int &get_corresponding_output_id(const bNode &input_bnode) const override
  {
    BLI_assert(input_bnode.type_legacy == this->input_type);
    return output_node_id_from_storage<NodeClosureInput>(input_bnode);
  }
};

class FluidSimZoneType : public bke::bNodeZoneType {
 public:
  FluidSimZoneType()
  {
    this->input_idname = "ImageNodeFluidSimInput"_ustr;
    this->output_idname = "ImageNodeFluidSimOutput"_ustr;
    this->input_type = IMG_NODE_FLUID_SIM_INPUT;
    this->output_type = IMG_NODE_FLUID_SIM_OUTPUT;
    this->theme_id = TH_NODE_ZONE_FLUID;
  }

  const int &get_corresponding_output_id(const bNode &input_bnode) const override
  {
    BLI_assert(input_bnode.type_legacy == this->input_type);
    return output_node_id_from_storage<NodeImageFluidSimInput>(input_bnode);
  }
};

class LightIterInternalZoneType : public bke::bNodeZoneType {
 public:
  LightIterInternalZoneType()
  {
    this->input_idname = "ShaderNodeLightIterInternalInput"_ustr;
    this->output_idname = "ShaderNodeLightIterInternalOutput"_ustr;
    this->input_type = SH_NODE_LIGHT_ITER_INTERNAL_INPUT;
    this->output_type = SH_NODE_LIGHT_ITER_INTERNAL_OUTPUT;
    this->theme_id = TH_NODE_ZONE_REPEAT;
  }

  bool output_id_requires_storage() const override
  {
    return false;
  }

  const int &get_corresponding_output_id(const bNode &input_bnode) const override
  {
    BLI_assert(input_bnode.type_legacy == this->input_type);
    return *reinterpret_cast<const int *>(&input_bnode.custom3);
  }
};

static void register_zone_types()
{
  static SimulationZoneType simulation_zone_type;
  static RepeatZoneType repeat_zone_type;
  static ForeachGeometryElementZoneType foreach_geometry_element_zone_type;
  static ClosureZoneType closure_zone_type;
  static FluidSimZoneType fluid_sim_zone_type;
  static LightIterInternalZoneType light_iter_internal_zone_type;
  bke::register_node_zone_type(simulation_zone_type);
  bke::register_node_zone_type(repeat_zone_type);
  bke::register_node_zone_type(foreach_geometry_element_zone_type);
  bke::register_node_zone_type(closure_zone_type);
  bke::register_node_zone_type(fluid_sim_zone_type);
  bke::register_node_zone_type(light_iter_internal_zone_type);
}

void register_nodes()
{
  register_zone_types();

  register_undefined_types();

  register_standard_node_socket_types();

  register_node_tree_type_geo();
  register_node_tree_type_cmp();
  register_node_tree_type_img(); /* IMAGE_NODES_MVP */
  register_node_tree_type_obj();

  register_node_type_frame();
  register_node_type_reroute();
  register_node_type_implicit_conversion();
  register_node_type_comment();
  register_node_type_group_input();
  register_node_type_group_output();

  register_compositor_nodes();
  register_shader_nodes();
  register_texture_nodes();
  register_geometry_nodes();
  register_function_nodes();
  register_image_nodes(); /* IMAGE_NODES_MVP */
  register_object_nodes();
  register_named_portal_nodes();
  /* After NodeCombineBundle exists: Image Process maps cache publisher for Bake Image. */
  register_image_combine_bundle_compositor();
}

}  // namespace blender
