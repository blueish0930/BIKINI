/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup RNA
 */

#include "BLT_translation.hh"

#include "BKE_attribute.hh"
#include "BKE_node.hh"

#include "RNA_enum_types.hh"

#include "UI_interface_c.hh"

#include "rna_internal.hh"
#include "rna_nodetree_common.hh"

namespace blender {

static const EnumPropertyItem rna_node_geometry_curve_handle_type_items[] = {
    {GEO_NODE_CURVE_HANDLE_FREE,
     "FREE",
     ICON_HANDLE_FREE,
     "Free",
     "The handle can be moved anywhere, and does not influence the point's other handle"},
    {GEO_NODE_CURVE_HANDLE_AUTO,
     "AUTO",
     ICON_HANDLE_AUTO,
     "Auto",
     "The location is automatically calculated to be smooth"},
    {GEO_NODE_CURVE_HANDLE_VECTOR,
     "VECTOR",
     ICON_HANDLE_VECTOR,
     "Vector",
     "The location is calculated to point to the next/previous control point"},
    {GEO_NODE_CURVE_HANDLE_ALIGN,
     "ALIGN",
     ICON_HANDLE_ALIGNED,
     "Align",
     "The location is constrained to point in the opposite direction as the other handle"},
    {0, nullptr, 0, nullptr, nullptr}};

const EnumPropertyItem rna_enum_node_geometry_curve_handle_side_items[] = {
    {GEO_NODE_CURVE_HANDLE_LEFT, "LEFT", ICON_NONE, "Left", "Use the left handles"},
    {GEO_NODE_CURVE_HANDLE_RIGHT, "RIGHT", ICON_NONE, "Right", "Use the right handles"},
    {0, nullptr, 0, nullptr, nullptr}};

const EnumPropertyItem rna_enum_node_grease_pencil_merge_mode_items[] = {
    {GEO_NODE_MERGE_LAYERS_BY_NAME,
     "MERGE_BY_NAME",
     0,
     "By Name",
     "Combine all layers which have the same name"},
    {GEO_NODE_MERGE_LAYERS_BY_ID,
     "MERGE_BY_ID",
     0,
     "By Group ID",
     "Provide a custom group ID for each layer and all layers with the same ID will be merged "
     "into one"},
    {0, nullptr, 0, nullptr, nullptr},
};

const EnumPropertyItem rna_enum_node_grease_pencil_stroke_type_items[] = {
    {GEO_NODE_GREASE_PENCIL_STROKE,
     "STROKE",
     ICON_GP_DRAW_STROKE,
     "Stroke",
     "Set the color and opacity for the points of the stroke"},
    {GEO_NODE_GREASE_PENCIL_FILL,
     "FILL",
     ICON_GP_DRAW_FILL,
     "Fill",
     "Set the color and opacity for the stroke fills"},
    {GEO_NODE_GREASE_PENCIL_BOTH,
     "BOTH",
     ICON_GP_DRAW_BOTH,
     "Both",
     "Set the color and opacity for stroke points and fills together"},
    {0, nullptr, 0, nullptr, nullptr},
};

}  // namespace blender

#ifdef RNA_RUNTIME

#  include "NOD_geo_bake.hh"
#  include "NOD_geo_capture_attribute.hh"
#  include "NOD_geo_closure_to_list.hh"
#  include "NOD_geo_combine_list.hh"
#  include "NOD_geo_expression.hh"
#  include "NOD_geo_wrangle.hh"
#  include "NOD_geo_field_to_grid.hh"
#  include "NOD_geo_field_to_list.hh"
#  include "NOD_geo_foreach_geometry_element.hh"
#  include "NOD_geo_index_switch.hh"
#  include "NOD_geo_menu_switch.hh"
#  include "NOD_geo_rasterize_points.hh"
#  include "NOD_geo_repeat.hh"
#  include "NOD_geo_viewer.hh"

namespace blender {

using nodes::BakeItemsAccessor;
using nodes::CaptureAttributeItemsAccessor;
using nodes::ImageBakeItemsAccessor;
using nodes::FluidColorItemsAccessor;
using nodes::ClosureToListItemsAccessor;
using nodes::CombineListItemsAccessor;
using nodes::FieldToGridItemsAccessor;
using nodes::FieldToListItemsAccessor;
using nodes::ForeachGeometryElementGenerationItemsAccessor;
using nodes::ForeachGeometryElementInputItemsAccessor;
using nodes::ForeachGeometryElementMainItemsAccessor;
using nodes::GeoViewerItemsAccessor;
using nodes::HLSLInputItemsAccessor;
using nodes::HLSLOutputItemsAccessor;
using nodes::ShaderWrangleInputItemsAccessor;
using nodes::ShaderWrangleFieldItemsAccessor;
using nodes::ShaderWrangleOutputItemsAccessor;
using nodes::IndexSwitchItemsAccessor;
using nodes::MenuSwitchItemsAccessor;
using nodes::RasterizePointsItemsAccessor;
using nodes::RepeatItemsAccessor;
using nodes::SimulationItemsAccessor;

static bool generic_attribute_type_supported(const EnumPropertyItem *item)
{
  return ELEM(item->value,
              CD_PROP_FLOAT,
              CD_PROP_FLOAT2,
              CD_PROP_FLOAT3,
              CD_PROP_COLOR,
              CD_PROP_BOOL,
              CD_PROP_INT32,
              CD_PROP_BYTE_COLOR,
              CD_PROP_QUATERNION,
              CD_PROP_FLOAT4X4);
}

static bool generic_attribute_type_supported_with_socket(const EnumPropertyItem *item)
{
  return generic_attribute_type_supported(item) &&
         !ELEM(item->value, CD_PROP_BYTE_COLOR, CD_PROP_FLOAT2);
}

static const EnumPropertyItem *rna_GeometryNodeAttributeType_type_with_socket_itemf(
    bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free)
{
  *r_free = true;
  return itemf_function_check(rna_enum_attribute_type_items,
                              generic_attribute_type_supported_with_socket);
}

static const EnumPropertyItem *rna_GeometryNodeAttributeDomain_attribute_domain_itemf(
    bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free)
{
  EnumPropertyItem *item_array = nullptr;
  int items_len = 0;

  for (const EnumPropertyItem *item = rna_enum_attribute_domain_items; item->identifier != nullptr;
       item++)
  {
    RNA_enum_item_add(&item_array, &items_len, item);
  }
  RNA_enum_item_end(&item_array, &items_len);

  *r_free = true;
  return item_array;
}

/** Similar to #rna_NodeSocket_structure_type_item_filter. */
static const EnumPropertyItem *rna_GeometryNodeClosureToListItem_structure_type_itemf(
    bContext * /*C*/, PointerRNA *ptr, PropertyRNA * /*prop*/, bool *r_free)
{
  const bNodeTree *ntree = reinterpret_cast<const bNodeTree *>(ptr->owner_id);
  const auto &item = *static_cast<const GeometryNodeClosureToListItem *>(ptr->data);
  const auto socket_type = eNodeSocketDatatype(item.socket_type);

  if (!ntree) {
    return rna_enum_dummy_NULL_items;
  }
  const bool is_geometry_nodes = ntree->type == NTREE_GEOMETRY;
  const bool supports_fields = is_geometry_nodes &&
                               nodes::socket_type_supports_fields(socket_type);
  const bool supports_grids = is_geometry_nodes && nodes::socket_type_supports_grids(socket_type);
  /* Lists of lists are sort of arbitrarily not supported yet. */
  const bool supports_lists = false;

  *r_free = true;
  EnumPropertyItem *items = nullptr;
  int items_count = 0;

  for (const EnumPropertyItem *item = rna_enum_node_socket_structure_type_items; item->identifier;
       item++)
  {
    switch (NodeSocketInterfaceStructureType(item->value)) {
      case NodeSocketInterfaceStructureType::Single: {
        RNA_enum_item_add(&items, &items_count, item);
        break;
      }
      case NodeSocketInterfaceStructureType::Auto: {
        break;
      }
      case NodeSocketInterfaceStructureType::Dynamic: {
        if (supports_fields || supports_grids) {
          RNA_enum_item_add(&items, &items_count, item);
        }
        break;
      }
      case NodeSocketInterfaceStructureType::Field: {
        if (supports_fields) {
          RNA_enum_item_add(&items, &items_count, item);
        }
        break;
      }
      case NodeSocketInterfaceStructureType::Grid: {
        if (supports_grids) {
          RNA_enum_item_add(&items, &items_count, item);
        }
        break;
      }
      case NodeSocketInterfaceStructureType::List: {
        if (supports_lists) {
          RNA_enum_item_add(&items, &items_count, item);
        }
        break;
      }
    }
  }
  RNA_enum_item_end(&items, &items_count);
  return items;
}

static IndexSwitchItem *rna_NodeIndexSwitchItems_new(ID *id, bNode *node, Main *bmain)
{
  IndexSwitchItem *new_item = nodes::socket_items::add_item<IndexSwitchItemsAccessor>(*node);

  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
  BKE_ntree_update_tag_node_property(ntree, node);
  BKE_main_ensure_invariants(*bmain, ntree->id);
  WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);

  return new_item;
}

static NodeGeometryRasterizePointsItem *rna_NodeGeometryRasterizePointsItems_new(ID *id,
                                                                                 bNode *node,
                                                                                 Main *bmain,
                                                                                 const char *name)
{
  NodeGeometryRasterizePointsItem *new_item =
      nodes::socket_items::add_item_with_name<RasterizePointsItemsAccessor>(*node, name);

  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
  BKE_ntree_update_tag_node_property(ntree, node);
  BKE_main_ensure_invariants(*bmain, ntree->id);
  WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);

  return new_item;
}

static CombineListItem *rna_NodeCombineListItems_new(ID *id, bNode *node, Main *bmain)
{
  CombineListItem *new_item = nodes::socket_items::add_item<CombineListItemsAccessor>(*node);

  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
  BKE_ntree_update_tag_node_property(ntree, node);
  BKE_main_ensure_invariants(*bmain, ntree->id);
  WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);

  return new_item;
}

/* The same as #grid_socket_type_items_filter_fn. */
static const EnumPropertyItem *rna_NodeFieldToGridItem_data_type_itemf(bContext * /*C*/,
                                                                       PointerRNA * /*ptr*/,
                                                                       PropertyRNA * /*prop*/,
                                                                       bool *r_free)
{
  *r_free = true;
  return itemf_function_check(
      rna_enum_node_socket_data_type_items, [](const EnumPropertyItem *item) {
        return nodes::socket_type_supports_grids(eNodeSocketDatatype(item->value));
      });
}

static const EnumPropertyItem *rna_NodeGeometryCaptureAttributeItem_data_type_itemf(
    bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free)
{
  *r_free = true;
  /* See #attribute_type_type_with_socket_fn. */
  return itemf_function_check(rna_enum_attribute_type_items, [](const EnumPropertyItem *item) {
    return ELEM(item->value,
                CD_PROP_FLOAT,
                CD_PROP_FLOAT3,
                CD_PROP_COLOR,
                CD_PROP_BOOL,
                CD_PROP_INT32,
                CD_PROP_QUATERNION,
                CD_PROP_FLOAT4X4,
                CD_PROP_STRING);
  });
}

static int rna_ImageBakeImage_color_space_get(PointerRNA *ptr)
{
  bNode *node = ptr->data_as<bNode>();
  NodeImageBakeImage *storage = static_cast<NodeImageBakeImage *>(node->storage);
  /* Empty name = Auto (special enum value -1). */
  if (storage->color_space[0] == '\0') {
    return -1;
  }
  return IMB_colormanagement_colorspace_get_named_index(storage->color_space);
}

static void rna_ImageBakeImage_color_space_set(PointerRNA *ptr, int value)
{
  bNode *node = ptr->data_as<bNode>();
  NodeImageBakeImage *storage = static_cast<NodeImageBakeImage *>(node->storage);
  if (value < 0) {
    storage->color_space[0] = '\0';
    return;
  }
  const char *name = IMB_colormanagement_colorspace_get_indexed_name(value);
  if (name && name[0]) {
    STRNCPY_UTF8(storage->color_space, name);
  }
  else {
    storage->color_space[0] = '\0';
  }
}

static const EnumPropertyItem *rna_ImageBakeImage_color_space_itemf(bContext * /*C*/,
                                                                    PointerRNA * /*ptr*/,
                                                                    PropertyRNA * /*prop*/,
                                                                    bool *r_free)
{
  EnumPropertyItem *items = nullptr;
  int totitem = 0;

  /* First entry: automatic default based on file format. */
  static const EnumPropertyItem auto_item = {
      -1, "AUTO", 0, "Auto", "Use scene linear for EXR/HDR, default byte space otherwise"};
  RNA_enum_item_add(&items, &totitem, &auto_item);
  IMB_colormanagement_colorspace_items_add(&items, &totitem);
  RNA_enum_item_end(&items, &totitem);

  *r_free = true;
  return items;
}

static NodeEnumItem *rna_NodeMenuSwitchItems_new(ID *id,
                                                 bNode *node,
                                                 Main *bmain,
                                                 const char *name)
{
  NodeEnumItem *new_item = nodes::socket_items::add_item_with_name<MenuSwitchItemsAccessor>(*node,
                                                                                            name);

  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(id);
  BKE_ntree_update_tag_node_property(ntree, node);
  BKE_main_ensure_invariants(*bmain, ntree->id);
  WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);

  return new_item;
}

static PointerRNA rna_NodeMenuSwitch_enum_definition_get(PointerRNA *ptr)
{
  /* Return node itself. The data is now directly available on the node and does not have to be
   * accessed through "enum_definition". */
  return *ptr;
}

static int rna_MergeLayers_mode_get(PointerRNA *ptr)
{
  bNode *node = ptr->data_as<bNode>();
  const bNodeSocket *socket = bke::node_find_socket(*node, SOCK_IN, "Mode"_ustr);
  return socket->default_value_typed<bNodeSocketValueMenu>()->value;
}

static void rna_MergeLayers_mode_set(PointerRNA *ptr, int value)
{
  bNode *node = ptr->data_as<bNode>();
  bNodeSocket *socket = bke::node_find_socket(*node, SOCK_IN, "Mode"_ustr);
  socket->default_value_typed<bNodeSocketValueMenu>()->value = value;
}

static int rna_SetGreasePencilColor_mode_get(PointerRNA *ptr)
{
  bNode *node = ptr->data_as<bNode>();
  const bNodeSocket *socket = bke::node_find_socket(*node, SOCK_IN, "Mode"_ustr);
  return socket->default_value_typed<bNodeSocketValueMenu>()->value;
}

static void rna_SetGreasePencilColor_mode_set(PointerRNA *ptr, int value)
{
  bNode *node = ptr->data_as<bNode>();
  bNodeSocket *socket = bke::node_find_socket(*node, SOCK_IN, "Mode"_ustr);
  socket->default_value_typed<bNodeSocketValueMenu>()->value = value;
}

static int rna_SetGreasePencilDepth_depth_order_get(PointerRNA *ptr)
{
  bNode *node = ptr->data_as<bNode>();
  const bNodeSocket *socket = bke::node_find_socket(*node, SOCK_IN, "Depth Order"_ustr);
  return socket->default_value_typed<bNodeSocketValueMenu>()->value;
}

static void rna_SetGreasePencilDepth_depth_order_set(PointerRNA *ptr, int value)
{
  bNode *node = ptr->data_as<bNode>();
  bNodeSocket *socket = bke::node_find_socket(*node, SOCK_IN, "Depth Order"_ustr);
  socket->default_value_typed<bNodeSocketValueMenu>()->value = value;
}

}  // namespace blender

static int rna_GeometryNodeModalKeymapItem_map_type_get(PointerRNA *ptr)
{
  const auto &item = *static_cast<const GeometryNodeModalKeymapItem *>(ptr->data);
  return WM_keymap_map_type_from_event_type(item.type);
}

/** Mirrors #rna_wmKeyMapItem_map_type_set: the map type is derived from the event type. */
static void rna_GeometryNodeModalKeymapItem_map_type_set(PointerRNA *ptr, const int value)
{
  auto &item = *static_cast<GeometryNodeModalKeymapItem *>(ptr->data);
  if (value == WM_keymap_map_type_from_event_type(item.type)) {
    return;
  }
  switch (value) {
    case KMI_TYPE_KEYBOARD:
      item.type = EVT_AKEY;
      item.val = KM_PRESS;
      break;
    case KMI_TYPE_MOUSE:
      item.type = LEFTMOUSE;
      item.val = KM_PRESS;
      break;
    case KMI_TYPE_TEXTINPUT:
      item.type = wmEventType(KM_TEXTINPUT);
      item.val = KM_NOTHING;
      break;
    case KMI_TYPE_TIMER:
      item.type = TIMER;
      item.val = KM_NOTHING;
      break;
    case KMI_TYPE_NDOF:
      item.type = NDOF_MOTION;
      item.val = KM_NOTHING;
      break;
  }
}

static void rna_GeometryNodeModalKeymapItem_update(Main *bmain, Scene *scene, PointerRNA *ptr)
{
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(ptr->owner_id);
  BKE_ntree_update_tag_modal_keymap(ntree);
  rna_NodeTree_update_asset(bmain, scene, ptr);
}

/** Image Process Import Geo: only COP-Import Geo* locked Geometry groups. */
static bool rna_ImageImportGeo_node_tree_poll(PointerRNA * /*ptr*/, const PointerRNA value)
{
  const bNodeTree *ngroup = static_cast<const bNodeTree *>(value.data);
  if (!ngroup || ngroup->type != NTREE_GEOMETRY) {
    return false;
  }
  if (nodes::image_points::is_import_geo_geometry_tree(*ngroup)) {
    return true;
  }
  return nodes::image_points::is_import_geo_tree_name(ngroup->id.name + 2);
}

static void rna_ImageImportGeo_node_tree_set(PointerRNA *ptr,
                                             PointerRNA value,
                                             ReportList * /*reports*/)
{
  bNode *node = ptr->data_as<bNode>();
  bNodeTree *ngroup = static_cast<bNodeTree *>(value.data);

  if (ngroup) {
    if (ngroup->type != NTREE_GEOMETRY ||
        !(nodes::image_points::is_import_geo_geometry_tree(*ngroup) ||
          nodes::image_points::is_import_geo_tree_name(ngroup->id.name + 2)))
    {
      return;
    }
  }

  if (node->id) {
    id_us_min(node->id);
    node->id = nullptr;
  }
  if (ngroup) {
    node->id = &ngroup->id;
    id_us_plus(node->id);
    nodes::image_points::lock_import_geo_geometry_tree(*ngroup);
  }
}

/** Image Process Import Points: only COP-Import Point* locked Geometry groups. */
static bool rna_ImageImportPoints_node_tree_poll(PointerRNA * /*ptr*/, const PointerRNA value)
{
  const bNodeTree *ngroup = static_cast<const bNodeTree *>(value.data);
  if (!ngroup || ngroup->type != NTREE_GEOMETRY) {
    return false;
  }
  /* Locked flag and/or COP-Import Point name prefix. */
  if (nodes::image_points::is_import_points_geometry_tree(*ngroup)) {
    return true;
  }
  return nodes::image_points::is_import_points_tree_name(ngroup->id.name + 2);
}

static void rna_ImageImportPoints_node_tree_set(PointerRNA *ptr,
                                                PointerRNA value,
                                                ReportList * /*reports*/)
{
  bNode *node = ptr->data_as<bNode>();
  bNodeTree *ngroup = static_cast<bNodeTree *>(value.data);

  if (ngroup) {
    if (ngroup->type != NTREE_GEOMETRY ||
        !(nodes::image_points::is_import_points_geometry_tree(*ngroup) ||
          nodes::image_points::is_import_points_tree_name(ngroup->id.name + 2)))
    {
      return;
    }
  }

  if (node->id) {
    id_us_min(node->id);
    node->id = nullptr;
  }
  if (ngroup) {
    node->id = &ngroup->id;
    id_us_plus(node->id);
    /* Ensure lock + fixed interface when user assigns an existing COP group. */
    nodes::image_points::lock_import_points_geometry_tree(*ngroup);
  }
}

/**
 * Solver param edits (slider drag while paused):
 * Do NOT wipe the GPU cache here — invalidate_after(INT_MIN) freed textures and
 * forced a cold multi-frame re-sim on every drag (lag + noise explosion).
 * execute_gpu_pavel detects solver_key mismatch, keeps the last GPU snapshot
 * for display-only (hold_display), and applies new params on the next real
 * frame advance or start-frame reseed.
 */
static void rna_NodeImageFluid_solver_update(Main *bmain, Scene * /*scene*/, PointerRNA *ptr)
{
  rna_Node_update(bmain, nullptr, ptr);
}

#else

namespace blender {

static void def_geo_image_texture(BlenderRNA * /*brna*/, StructRNA *srna)
{
  /* Match ShaderNodeTexImage sampling enums (Image is a socket, not node->id). */
  static const EnumPropertyItem prop_projection_items[] = {
      {SHD_PROJ_FLAT,
       "FLAT",
       0,
       "Flat",
       "Image is projected flat using the X and Y coordinates of the texture vector"},
      {SHD_PROJ_BOX,
       "BOX",
       0,
       "Box",
       "Image is projected using different components for each side of the object space bounding "
       "box"},
      {SHD_PROJ_SPHERE,
       "SPHERE",
       0,
       "Sphere",
       "Image is projected spherically using the Z axis as central"},
      {SHD_PROJ_TUBE,
       "TUBE",
       0,
       "Tube",
       "Image is projected from the tube using the Z axis as central"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeGeometryImageTexture", "storage");

  prop = RNA_def_property(srna, "interpolation", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, sh_tex_prop_interpolation_items);
  RNA_def_property_ui_text(prop, "Interpolation", "Texture interpolation");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "projection", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, prop_projection_items);
  RNA_def_property_ui_text(
      prop, "Projection", "Method to project 2D image on object with a 3D texture vector");
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_ID_IMAGE);
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "projection_blend", PROP_FLOAT, PROP_FACTOR);
  RNA_def_property_float_sdna(prop, nullptr, "projection_blend");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_range(prop, 0.0f, 1.0f, 1, 3);
  RNA_def_property_ui_text(
      prop, "Projection Blend", "For box projection, amount of blend to use between sides");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "extension", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, prop_image_extension);
  RNA_def_property_ui_text(
      prop, "Extension", "How the image is extrapolated past its original bounds");
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_ID_IMAGE);
  RNA_def_property_update(prop, 0, "rna_Node_update");

  /* Same as ShaderNodeTexImage: Source / movie frames / multilayer via ImageUser. */
  prop = RNA_def_property(srna, "image_user", PROP_POINTER, PROP_NONE);
  RNA_def_property_flag(prop, PROP_NEVER_NULL);
  RNA_def_property_pointer_sdna(prop, nullptr, "iuser");
  RNA_def_property_ui_text(
      prop,
      "Image User",
      "Parameters defining which layer, pass and frame of the image is displayed");
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void rna_def_geo_gizmo_transform(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeGeometryTransformGizmo", "storage");

  prop = RNA_def_property(srna, "use_translation_x", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GEO_NODE_TRANSFORM_GIZMO_USE_TRANSLATION_X);
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_ui_text(prop, "Use Translation X", nullptr);
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "use_translation_y", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GEO_NODE_TRANSFORM_GIZMO_USE_TRANSLATION_Y);
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_ui_text(prop, "Use Translation Y", nullptr);
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "use_translation_z", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GEO_NODE_TRANSFORM_GIZMO_USE_TRANSLATION_Z);
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_ui_text(prop, "Use Translation Z", nullptr);
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "use_rotation_x", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GEO_NODE_TRANSFORM_GIZMO_USE_ROTATION_X);
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_ui_text(prop, "Use Rotation X", nullptr);
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "use_rotation_y", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GEO_NODE_TRANSFORM_GIZMO_USE_ROTATION_Y);
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_ui_text(prop, "Use Rotation Y", nullptr);
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "use_rotation_z", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GEO_NODE_TRANSFORM_GIZMO_USE_ROTATION_Z);
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_ui_text(prop, "Use Rotation Z", nullptr);
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "use_scale_x", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GEO_NODE_TRANSFORM_GIZMO_USE_SCALE_X);
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_ui_text(prop, "Use Scale X", nullptr);
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "use_scale_y", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GEO_NODE_TRANSFORM_GIZMO_USE_SCALE_Y);
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_ui_text(prop, "Use Scale Y", nullptr);
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "use_scale_z", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GEO_NODE_TRANSFORM_GIZMO_USE_SCALE_Z);
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_ui_text(prop, "Use Scale Z", nullptr);
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void def_geo_curve_sample(BlenderRNA * /*brna*/, StructRNA *srna)
{
  static EnumPropertyItem mode_items[] = {
      {GEO_NODE_CURVE_SAMPLE_FACTOR,
       "FACTOR",
       0,
       "Factor",
       "Find sample positions on the curve using a factor of its total length"},
      {GEO_NODE_CURVE_SAMPLE_LENGTH,
       "LENGTH",
       0,
       "Length",
       "Find sample positions on the curve using a distance from its beginning"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_struct_sdna_from(srna, "NodeGeometryCurveSample", "storage");

  PropertyRNA *prop;
  prop = RNA_def_property(srna, "mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, mode_items);
  RNA_def_property_ui_text(prop, "Mode", "Method for sampling input");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");

  prop = RNA_def_property(srna, "use_all_curves", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_ui_text(prop,
                           "All Curves",
                           "Sample lengths based on the total length of all curves, rather than "
                           "using a length inside each selected curve");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");

  prop = RNA_def_property(srna, "data_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_attribute_type_items);
  RNA_def_property_enum_funcs(
      prop, nullptr, nullptr, "rna_GeometryNodeAttributeType_type_with_socket_itemf");
  RNA_def_property_enum_default(prop, CD_PROP_FLOAT);
  RNA_def_property_ui_text(prop, "Data Type", "");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");
}

static void def_geo_distribute_points_on_faces(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  static const EnumPropertyItem rna_node_geometry_distribute_points_on_faces_mode_items[] = {
      {GEO_NODE_POINT_DISTRIBUTE_POINTS_ON_FACES_RANDOM,
       "RANDOM",
       0,
       "Random",
       "Distribute points randomly on the surface"},
      {GEO_NODE_POINT_DISTRIBUTE_POINTS_ON_FACES_POISSON,
       "POISSON",
       0,
       "Poisson Disk",
       "Distribute the points randomly on the surface while taking a minimum distance between "
       "points into account"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  prop = RNA_def_property(srna, "distribute_method", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, rna_node_geometry_distribute_points_on_faces_mode_items);
  RNA_def_property_enum_default(prop, GEO_NODE_POINT_DISTRIBUTE_POINTS_ON_FACES_RANDOM);
  RNA_def_property_ui_text(prop, "Distribution Method", "Method to use for scattering points");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");

  prop = RNA_def_property(srna, "use_legacy_normal", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "custom2", 1);
  RNA_def_property_ui_text(prop,
                           "Legacy Normal",
                           "Output the normal and rotation values that have been output "
                           "before the node started taking smooth normals into account");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");
}

static void def_geo_curve_set_handle_type(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeGeometryCurveSetHandles", "storage");

  prop = RNA_def_property(srna, "handle_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "handle_type");
  RNA_def_property_ui_text(prop, "Handle Type", "");
  RNA_def_property_enum_items(prop, rna_node_geometry_curve_handle_type_items);
  RNA_def_property_enum_default(prop, GEO_NODE_CURVE_HANDLE_AUTO);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");

  prop = RNA_def_property(srna, "mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_flag(prop, PROP_ENUM_FLAG);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_enum_items(prop, rna_enum_node_geometry_curve_handle_side_items);
  RNA_def_property_enum_default(prop, GEO_NODE_CURVE_HANDLE_LEFT | GEO_NODE_CURVE_HANDLE_RIGHT);
  RNA_def_property_ui_text(prop, "Mode", "Whether to update left and right handles");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");
}

static void def_geo_simulation_input(BlenderRNA *brna, StructRNA *srna)
{
  RNA_def_struct_sdna_from(srna, "NodeGeometrySimulationInput", "storage");

  def_common_zone_input(brna, srna);
}

static void def_image_fluid_sim_input(BlenderRNA *brna, StructRNA *srna)
{
  RNA_def_struct_sdna_from(srna, "NodeImageFluidSimInput", "storage");
  def_common_zone_input(brna, srna);
}

static void rna_def_image_fluid_color_item(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeImageFluidColorItem", nullptr);
  RNA_def_struct_ui_text(srna, "Fluid Color Item", "Color field simulated in the fluid zone");
  RNA_def_struct_sdna(srna, "NodeImageFluidColorItem");
  rna_def_node_item_array_socket_item_common(srna, "FluidColorItemsAccessor", false);
}

static void rna_def_image_fluid_color_items(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeImageFluidColorItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Items", "Collection of fluid color fields");
  rna_def_node_item_array_new_with_socket_and_name(
      srna, "NodeImageFluidColorItem", "FluidColorItemsAccessor");
  rna_def_node_item_array_common_functions(srna, "NodeImageFluidColorItem", "FluidColorItemsAccessor");
}

static const EnumPropertyItem rna_enum_image_fluid_boundary_items[] = {
    {NODE_IMAGE_FLUID_BOUNDARY_OUTFLOW,
     "OUTFLOW",
     0,
     "Flow Out",
     "Open edge: fluid and dye leave the domain"},
    {NODE_IMAGE_FLUID_BOUNDARY_WRAP,
     "WRAP",
     0,
     "Wrap",
     "Periodic edge: fluid re-enters on the opposite side"},
    {NODE_IMAGE_FLUID_BOUNDARY_BOUNCE,
     "BOUNCE",
     0,
     "Bounce",
     "Solid wall: normal velocity is reflected"},
    {0, nullptr, 0, nullptr, nullptr},
};

static void def_image_fluid_sim_output(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;
  rna_def_image_fluid_color_item(brna);
  rna_def_image_fluid_color_items(brna);

  RNA_def_struct_sdna_from(srna, "NodeImageFluidSimOutput", "storage");

  prop = RNA_def_property(srna, "color_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "color_items", "color_items_num");
  RNA_def_property_struct_type(prop, "NodeImageFluidColorItem");
  RNA_def_property_ui_text(prop, "Color Items", "Color density fields advected by the fluid");
  RNA_def_property_srna(prop, "NodeImageFluidColorItems");

  prop = RNA_def_property(srna, "color_active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "color_active_index");
  RNA_def_property_ui_text(prop, "Active Color Index", "Index of the active color field");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "cached_frames", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "cached_frames");
  RNA_def_property_range(prop, 1, INT_MAX);
  RNA_def_property_ui_range(prop, 1, 500, 1, 0);
  RNA_def_property_ui_text(
      prop, "Cached Frames", "Number of recent fluid frames kept as a full dense cache");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "checkpoint_rate", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "checkpoint_rate");
  RNA_def_property_range(prop, 0, INT_MAX);
  RNA_def_property_ui_range(prop, 0, 250, 1, 0);
  RNA_def_property_ui_text(
      prop, "Checkpoint Rate", "Store a checkpoint every N frames (0 disables checkpoints)");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  /* --- Web demo param surface (paveldogreat.github.io/WebGL-Fluid-Simulation) --- */

  prop = RNA_def_property(srna, "density_diffusion", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "dissipation");
  RNA_def_property_range(prop, 0.0f, 4.0f);
  RNA_def_property_ui_range(prop, 0.0f, 4.0f, 0.1f, 2);
  RNA_def_property_ui_text(
      prop,
      "Density Dissipation",
      "How fast the color/dye fades during advection. 0 = never, "
      "1.0 = the WebGL Fluid default. Higher = dye disappears quickly");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "velocity_diffusion", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "viscosity");
  RNA_def_property_range(prop, 0.0f, 4.0f);
  RNA_def_property_ui_range(prop, 0.0f, 4.0f, 0.1f, 2);
  RNA_def_property_ui_text(
      prop,
      "Velocity Dissipation",
      "How fast velocity decays during self-advection. 0 = inviscid, "
      "0.2 = WebGL Fluid default. Higher = more damping");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "pressure", PROP_FLOAT, PROP_FACTOR);
  RNA_def_property_float_sdna(prop, nullptr, "buoyancy");
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_text(
      prop,
      "Pressure",
      "Warm-start decay factor for the pressure field. 0 = cold start, "
      "0.8 = WebGL Fluid default, 1 = fully retain last pressure");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "vorticity", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "vorticity");
  RNA_def_property_range(prop, 0.0f, 50.0f);
  RNA_def_property_ui_range(prop, 0.0f, 50.0f, 1.0f, 0);
  RNA_def_property_ui_text(
      prop,
      "Vorticity",
      "Curl confinement strength. 0 = none, 30 = WebGL Fluid default. "
      "Adds small-scale swirl to the velocity field");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "pressure_vcycles", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "pressure_vcycles");
  RNA_def_property_range(prop, 1, 6);
  RNA_def_property_ui_range(prop, 1, 4, 1, 0);
  RNA_def_property_ui_text(
      prop,
      "Pressure V-Cycles",
      "Multigrid V-cycles per projection (3 default). Few cycles = hundreds of Jacobi iterations");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "pressure_iterations", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "pressure_iterations");
  RNA_def_property_range(prop, 1, 200);
  RNA_def_property_ui_range(prop, 1, 80, 1, 0);
  RNA_def_property_ui_text(
      prop,
      "Pressure Iterations",
      "Jacobi iterations on the coarsest multigrid level. 20 = default");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "pressure_smooth_iterations", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "pressure_smooth_iterations");
  RNA_def_property_range(prop, 1, 8);
  RNA_def_property_ui_range(prop, 1, 6, 1, 0);
  RNA_def_property_ui_text(
      prop,
      "Pressure Smoothing",
      "Jacobi pre/post smoothing per multigrid level (2 default)");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "splat_radius", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "splat_radius");
  RNA_def_property_range(prop, 0.01f, 1.0f);
  RNA_def_property_ui_text(
      prop, "Splat Radius",
      "[Hidden: vestigial WebGL demo param]");
  RNA_def_property_flag(prop, PROP_HIDDEN);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "splat_force", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "splat_force");
  RNA_def_property_range(prop, 0.0f, 20000.0f);
  RNA_def_property_ui_range(prop, 0.0f, 10000.0f, 100.0f, 0);
  RNA_def_property_ui_text(
      prop,
      "Velocity Scale",
      "[Hidden: vestigial WebGL demo param]");
  RNA_def_property_flag(prop, PROP_HIDDEN);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "sim_resolution", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "sim_resolution");
  RNA_def_property_range(prop, 32, 512);
  RNA_def_property_ui_range(prop, 32, 512, 1, 0);
  RNA_def_property_ui_text(
      prop,
      "Sim Resolution",
      "Velocity grid minimum edge, aspect-corrected like WebGL Fluid (default 128)");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "dye_resolution", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "dye_resolution");
  RNA_def_property_range(prop, 128, 2048);
  RNA_def_property_ui_range(prop, 128, 2048, 1, 0);
  RNA_def_property_ui_text(
      prop,
      "Dye Resolution",
      "Color grid minimum edge, aspect-corrected like WebGL Fluid (default 1024)");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "dt_scale", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "dt_scale");
  RNA_def_property_range(prop, 0.0f, 10.0f);
  RNA_def_property_ui_range(prop, 0.0f, 10.0f, 0.1f, 2);
  RNA_def_property_ui_text(
      prop,
      "Time Scale",
      "Simulation speed multiplier (1 = real-time). High values use substeps (max 12) so the "
      "slider keeps working past ~4x");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "boundary_left", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "boundary_left");
  RNA_def_property_enum_items(prop, rna_enum_image_fluid_boundary_items);
  RNA_def_property_ui_text(prop, "Left Boundary", "Boundary mode for the left edge");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "boundary_right", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "boundary_right");
  RNA_def_property_enum_items(prop, rna_enum_image_fluid_boundary_items);
  RNA_def_property_ui_text(prop, "Right Boundary", "Boundary mode for the right edge");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "boundary_bottom", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "boundary_bottom");
  RNA_def_property_enum_items(prop, rna_enum_image_fluid_boundary_items);
  RNA_def_property_ui_text(prop, "Bottom Boundary", "Boundary mode for the bottom edge");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");

  prop = RNA_def_property(srna, "boundary_top", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "boundary_top");
  RNA_def_property_enum_items(prop, rna_enum_image_fluid_boundary_items);
  RNA_def_property_ui_text(prop, "Top Boundary", "Boundary mode for the top edge");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_NodeImageFluid_solver_update");
}

static void def_geo_repeat_input(BlenderRNA *brna, StructRNA *srna)
{
  RNA_def_struct_sdna_from(srna, "NodeGeometryRepeatInput", "storage");

  def_common_zone_input(brna, srna);
}

static void def_geo_foreach_geometry_element_input(BlenderRNA *brna, StructRNA *srna)
{
  RNA_def_struct_sdna_from(srna, "NodeGeometryForeachGeometryElementInput", "storage");

  def_common_zone_input(brna, srna);
}

static void rna_def_geo_simulation_state_item(BlenderRNA *brna)
{
  PropertyRNA *prop;

  StructRNA *srna = RNA_def_struct(brna, "SimulationStateItem", nullptr);
  RNA_def_struct_ui_text(srna, "Simulation Item", "");
  RNA_def_struct_sdna(srna, "NodeSimulationItem");

  rna_def_node_item_array_socket_item_common(srna, "SimulationItemsAccessor", true);

  prop = RNA_def_property(srna, "attribute_domain", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_attribute_domain_items);
  RNA_def_property_enum_funcs(
      prop, nullptr, nullptr, "rna_GeometryNodeAttributeDomain_attribute_domain_itemf");
  RNA_def_property_ui_text(
      prop,
      "Attribute Domain",
      "Domain where the field is captured if it is not already an attribute-field");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(
      prop, NC_NODE | NA_EDITED, "rna_Node_ItemArray_item_update<SimulationItemsAccessor>");
}

static void rna_def_geo_simulation_state_items(BlenderRNA *brna)
{
  StructRNA *srna;

  srna = RNA_def_struct(brna, "NodeGeometrySimulationOutputItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Items", "Collection of simulation items");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "SimulationStateItem", "SimulationItemsAccessor");
  rna_def_node_item_array_common_functions(srna, "SimulationStateItem", "SimulationItemsAccessor");
}

static void def_geo_simulation_output(BlenderRNA *brna, StructRNA *srna)
{
  static const EnumPropertyItem cache_limit_mode_items[] = {
      {GEO_NODE_SIMULATION_CACHE_LIMIT_FRAMES,
       "FRAMES",
       0,
       "Frames",
       "Limit the number of cached frames"},
      {GEO_NODE_SIMULATION_CACHE_LIMIT_MEMORY,
       "MEMORY",
       0,
       "Memory",
       "Limit approximate memory occupied by cached frames"},
      {0, nullptr, 0, nullptr, nullptr},
  };
  PropertyRNA *prop;

  rna_def_geo_simulation_state_item(brna);
  rna_def_geo_simulation_state_items(brna);

  RNA_def_struct_sdna_from(srna, "NodeGeometrySimulationOutput", "storage");

  prop = RNA_def_property(srna, "state_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "items", "items_num");
  RNA_def_property_struct_type(prop, "SimulationStateItem");
  RNA_def_property_ui_text(prop, "Items", "");
  RNA_def_property_srna(prop, "NodeGeometrySimulationOutputItems");

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_item", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "SimulationStateItem");
  RNA_def_property_pointer_funcs(prop,
                                 "rna_Node_ItemArray_active_get<SimulationItemsAccessor>",
                                 "rna_Node_ItemArray_active_set<SimulationItemsAccessor>",
                                 nullptr,
                                 nullptr);
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_NO_DEG_UPDATE);
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "cache_limit_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "cache_limit_mode");
  RNA_def_property_enum_items(prop, cache_limit_mode_items);
  RNA_def_property_ui_text(prop, "Cache Limit Mode", "How the simulation cache limit is measured");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "cache_limit", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "cache_limit");
  RNA_def_property_range(prop, 1, INT_MAX);
  RNA_def_property_ui_range(prop, 1, 10000, 1, 0);
  RNA_def_property_ui_text(
      prop, "Cache Limit", "Maximum number of unbaked simulation frames kept in memory");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "cache_memory_limit", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "cache_memory_limit");
  RNA_def_property_range(prop, 0.001f, 1.0e6f);
  RNA_def_property_ui_range(prop, 0.1f, 65536.0f, 10, 1);
  RNA_def_property_ui_text(
      prop,
      "Memory Limit (MB)",
      "Approximate maximum memory for unbaked simulation frames in megabytes (MB, 1024×1024 "
      "bytes); older frames are freed first");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "cached_frames", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "cached_frames");
  RNA_def_property_range(prop, 1, INT_MAX);
  RNA_def_property_ui_range(prop, 1, 500, 1, 0);
  RNA_def_property_ui_text(
      prop,
      "Cached Frames",
      "GPU Texture Editor: number of recent simulation frames kept as a full dense cache");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "checkpoint_rate", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "checkpoint_rate");
  RNA_def_property_range(prop, 0, INT_MAX);
  RNA_def_property_ui_range(prop, 0, 500, 1, 0);
  RNA_def_property_ui_text(
      prop,
      "Checkpoint Rate",
      "GPU Texture Editor: store a simulation checkpoint every N frames (0 disables). Scrubbing "
      "outside the dense cache seeks the nearest earlier checkpoint and recomputes forward");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_geo_merge_layers(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop = RNA_def_property(srna, "mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_node_grease_pencil_merge_mode_items);
  RNA_def_property_ui_text(prop, "Mode", "Determines how to choose which layers are merged");
  RNA_def_property_enum_funcs(
      prop, "rna_MergeLayers_mode_get", "rna_MergeLayers_mode_set", nullptr);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
  RNA_def_property_deprecated(prop, "Replaced by '.inputs[\"Mode\"]'.", 503, 600);
}

static void def_geo_set_grease_pencil_color(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop = RNA_def_property(srna, "mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_node_grease_pencil_stroke_type_items);
  RNA_def_property_ui_text(prop, "Mode", "Set the color and opacity for strokes or fills");
  RNA_def_property_enum_funcs(
      prop, "rna_SetGreasePencilColor_mode_get", "rna_SetGreasePencilColor_mode_set", nullptr);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
  RNA_def_property_deprecated(prop, "Replaced by '.inputs[\"Mode\"]'.", 503, 600);
}

static void def_geo_set_grease_pencil_depth(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop = RNA_def_property(srna, "depth_order", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_stroke_depth_order_items);
  RNA_def_property_ui_text(prop, "Depth Order", "");
  RNA_def_property_enum_funcs(prop,
                              "rna_SetGreasePencilDepth_depth_order_get",
                              "rna_SetGreasePencilDepth_depth_order_set",
                              nullptr);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
  RNA_def_property_deprecated(prop, "Replaced by '.inputs[\"Depth Order\"]'.", 503, 600);
}

static void rna_def_geo_repeat_item(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "RepeatItem", nullptr);
  RNA_def_struct_ui_text(srna, "Repeat Item", "");
  RNA_def_struct_sdna(srna, "NodeRepeatItem");

  rna_def_node_item_array_socket_item_common(srna, "RepeatItemsAccessor", true);
}

static void rna_def_geo_repeat_items(BlenderRNA *brna)
{
  StructRNA *srna;

  srna = RNA_def_struct(brna, "NodeGeometryRepeatOutputItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Items", "Collection of repeat items");

  rna_def_node_item_array_new_with_socket_and_name(srna, "RepeatItem", "RepeatItemsAccessor");
  rna_def_node_item_array_common_functions(srna, "RepeatItem", "RepeatItemsAccessor");
}

static void def_geo_repeat_output(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_geo_repeat_item(brna);
  rna_def_geo_repeat_items(brna);

  RNA_def_struct_sdna_from(srna, "NodeGeometryRepeatOutput", "storage");

  prop = RNA_def_property(srna, "repeat_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "items", "items_num");
  RNA_def_property_struct_type(prop, "RepeatItem");
  RNA_def_property_ui_text(prop, "Items", "");
  RNA_def_property_srna(prop, "NodeGeometryRepeatOutputItems");

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_item", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "RepeatItem");
  RNA_def_property_pointer_funcs(prop,
                                 "rna_Node_ItemArray_active_get<RepeatItemsAccessor>",
                                 "rna_Node_ItemArray_active_set<RepeatItemsAccessor>",
                                 nullptr,
                                 nullptr);
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_NO_DEG_UPDATE);
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "inspection_index", PROP_INT, PROP_NONE);
  RNA_def_property_ui_range(prop, 0, INT32_MAX, 1, -1);
  RNA_def_property_ui_text(prop,
                           "Inspection Index",
                           "Iteration index that is used by inspection features like the viewer "
                           "node or socket inspection");
  RNA_def_property_update(prop, NC_NODE, "rna_Node_update");
}

static void rna_def_geo_viewer_item(BlenderRNA *brna)
{
  PropertyRNA *prop;

  StructRNA *srna = RNA_def_struct(brna, "NodeGeometryViewerItem", nullptr);
  RNA_def_struct_ui_text(srna, "Viewer Item", "");
  RNA_def_struct_sdna(srna, "NodeGeometryViewerItem");

  rna_def_node_item_array_socket_item_common(srna, "GeoViewerItemsAccessor", true);

  prop = RNA_def_property(srna, "auto_remove", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NODE_GEO_VIEWER_ITEM_FLAG_AUTO_REMOVE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(
      prop, "Auto Remove", "Remove the item automatically when it is unlinked");
  RNA_def_property_update(
      prop, NC_NODE | NA_EDITED, "rna_Node_ItemArray_item_update<GeoViewerItemsAccessor>");
}

static void rna_def_geo_viewer_items(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeGeometryViewerItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Items", "Collection of viewer items");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "NodeGeometryViewerItem", "GeoViewerItemsAccessor");
  rna_def_node_item_array_common_functions(
      srna, "NodeGeometryViewerItem", "GeoViewerItemsAccessor");
}

static void rna_def_geo_viewer(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_geo_viewer_item(brna);
  rna_def_geo_viewer_items(brna);

  prop = RNA_def_property(srna, "ui_shortcut", PROP_INT, PROP_NONE);
  RNA_def_property_int_sdna(prop, nullptr, "custom1");
  RNA_def_property_int_funcs(prop, nullptr, "rna_Node_Viewer_shortcut_node_set", nullptr);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_IGNORE);
  RNA_def_property_int_default(prop, NODE_VIEWER_SHORTCUT_NONE);
  RNA_def_property_update(prop, NC_NODE | ND_DISPLAY, nullptr);

  RNA_def_struct_sdna_from(srna, "NodeGeometryViewer", "storage");

  prop = RNA_def_property(srna, "viewer_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "items", "items_num");
  RNA_def_property_struct_type(prop, "NodeGeometryViewerItem");
  RNA_def_property_ui_text(prop, "Viewer Items", "");
  RNA_def_property_srna(prop, "NodeGeometryViewerItems");

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_item", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "NodeGeometryViewerItem");
  RNA_def_property_ui_text(prop, "Active Item", "");
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_NO_DEG_UPDATE);
  RNA_def_property_pointer_funcs(prop,
                                 "rna_Node_ItemArray_active_get<GeoViewerItemsAccessor>",
                                 "rna_Node_ItemArray_active_set<GeoViewerItemsAccessor>",
                                 nullptr,
                                 nullptr);

  prop = RNA_def_property(srna, "domain", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_attribute_domain_with_auto_items);
  RNA_def_property_enum_default(prop, int(bke::AttrDomainSelection::Auto));
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Domain", "Domain to evaluate fields on");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void rna_def_geo_foreach_geometry_element_input_item(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "ForeachGeometryElementInputItem", nullptr);
  RNA_def_struct_ui_text(srna, "For Each Geometry Element Item", "");
  RNA_def_struct_sdna(srna, "NodeForeachGeometryElementInputItem");

  rna_def_node_item_array_socket_item_common(
      srna, "ForeachGeometryElementInputItemsAccessor", true);
}

static void rna_def_geo_foreach_geometry_element_input_items(BlenderRNA *brna)
{
  StructRNA *srna;

  srna = RNA_def_struct(brna, "NodeGeometryForeachGeometryElementInputItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Input Items", "Collection of input items");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "ForeachGeometryElementInputItem", "ForeachGeometryElementInputItemsAccessor");
  rna_def_node_item_array_common_functions(
      srna, "ForeachGeometryElementInputItem", "ForeachGeometryElementInputItemsAccessor");
}

static void rna_def_geo_foreach_geometry_element_main_item(BlenderRNA *brna)
{
  StructRNA *srna;

  srna = RNA_def_struct(brna, "ForeachGeometryElementMainItem", nullptr);
  RNA_def_struct_ui_text(srna, "For Each Geometry Element Item", "");
  RNA_def_struct_sdna(srna, "NodeForeachGeometryElementMainItem");

  rna_def_node_item_array_socket_item_common(
      srna, "ForeachGeometryElementMainItemsAccessor", true);
}

static void rna_def_geo_foreach_geometry_element_main_items(BlenderRNA *brna)
{
  StructRNA *srna;

  srna = RNA_def_struct(brna, "NodeGeometryForeachGeometryElementMainItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Main Items", "Collection of main items");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "ForeachGeometryElementMainItem", "ForeachGeometryElementMainItemsAccessor");
  rna_def_node_item_array_common_functions(
      srna, "ForeachGeometryElementMainItem", "ForeachGeometryElementMainItemsAccessor");
}

static void rna_def_geo_foreach_geometry_element_generation_item(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "ForeachGeometryElementGenerationItem", nullptr);
  RNA_def_struct_ui_text(srna, "For Each Geometry Element Item", "");
  RNA_def_struct_sdna(srna, "NodeForeachGeometryElementGenerationItem");

  rna_def_node_item_array_socket_item_common(
      srna, "ForeachGeometryElementGenerationItemsAccessor", true);

  prop = RNA_def_property(srna, "domain", PROP_ENUM, PROP_NONE);
  RNA_def_property_ui_text(prop, "Domain", "Domain that the field is evaluated on");
  RNA_def_property_enum_items(prop, rna_enum_attribute_domain_items);
  RNA_def_property_update(
      prop,
      NC_NODE | NA_EDITED,
      "rna_Node_ItemArray_item_update<ForeachGeometryElementGenerationItemsAccessor>");
}

static void rna_def_geo_foreach_geometry_element_generation_items(BlenderRNA *brna)
{
  StructRNA *srna;

  srna = RNA_def_struct(brna, "NodeGeometryForeachGeometryElementGenerationItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Generation Items", "Collection of generation items");

  rna_def_node_item_array_new_with_socket_and_name(
      srna,
      "ForeachGeometryElementGenerationItem",
      "ForeachGeometryElementGenerationItemsAccessor");
  rna_def_node_item_array_common_functions(srna,
                                           "ForeachGeometryElementGenerationItem",
                                           "ForeachGeometryElementGenerationItemsAccessor");
}

static void def_geo_foreach_geometry_element_output(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_geo_foreach_geometry_element_input_item(brna);
  rna_def_geo_foreach_geometry_element_input_items(brna);

  rna_def_geo_foreach_geometry_element_main_item(brna);
  rna_def_geo_foreach_geometry_element_main_items(brna);

  rna_def_geo_foreach_geometry_element_generation_items(brna);
  rna_def_geo_foreach_geometry_element_generation_item(brna);

  static const EnumPropertyItem iteration_mode_items[] = {
      {0, "ELEMENT", 0, "Element", "Run the zone once for every element of the domain"},
      {1,
       "GROUP",
       0,
       "Group",
       "Run the zone once for every group of elements that share a Group ID. Each iteration "
       "receives its whole group as one geometry, and the iterations run on all threads"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  prop = RNA_def_property(srna, "iteration_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, iteration_mode_items);
  RNA_def_property_ui_text(prop, "Mode", "What the zone runs once for");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");

  RNA_def_struct_sdna_from(srna, "NodeGeometryForeachGeometryElementOutput", "storage");

  prop = RNA_def_property(srna, "input_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "input_items.items", "input_items.items_num");
  RNA_def_property_struct_type(prop, "ForeachGeometryElementInputItem");
  RNA_def_property_srna(prop, "NodeGeometryForeachGeometryElementInputItems");

  prop = RNA_def_property(srna, "main_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "main_items.items", "main_items.items_num");
  RNA_def_property_struct_type(prop, "ForeachGeometryElementMainItem");
  RNA_def_property_srna(prop, "NodeGeometryForeachGeometryElementMainItems");

  prop = RNA_def_property(srna, "generation_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(
      prop, nullptr, "generation_items.items", "generation_items.items_num");
  RNA_def_property_struct_type(prop, "ForeachGeometryElementGenerationItem");
  RNA_def_property_srna(prop, "NodeGeometryForeachGeometryElementGenerationItems");

  prop = RNA_def_property(srna, "active_input_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "input_items.active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_generation_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "generation_items.active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_main_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "main_items.active_index");
  RNA_def_property_ui_text(prop, "Active Main Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "domain", PROP_ENUM, PROP_NONE);
  RNA_def_property_ui_text(prop, "Domain", "Geometry domain that is iterated over");
  RNA_def_property_enum_items(prop, rna_enum_attribute_domain_items);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "inspection_index", PROP_INT, PROP_NONE);
  RNA_def_property_ui_range(prop, 0, INT32_MAX, 1, -1);
  RNA_def_property_ui_text(prop,
                           "Inspection Index",
                           "Iteration index that is used by inspection features like the viewer "
                           "node or socket inspection");
  RNA_def_property_update(prop, NC_NODE, "rna_Node_update");
}

static void rna_def_geo_capture_attribute_item(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeGeometryCaptureAttributeItem", nullptr);
  RNA_def_struct_ui_text(srna, "Capture Attribute Item", "");
  RNA_def_struct_sdna(srna, "NodeGeometryAttributeCaptureItem");

  rna_def_node_item_array_socket_item_common(srna, "CaptureAttributeItemsAccessor", false);
  PropertyRNA *prop = RNA_def_property(srna, "data_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_attribute_type_items);
  RNA_def_property_enum_funcs(
      prop, nullptr, nullptr, "rna_NodeGeometryCaptureAttributeItem_data_type_itemf");
  RNA_def_property_ui_text(prop, "Data Type", "");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(
      prop, NC_NODE | NA_EDITED, "rna_Node_ItemArray_item_update<CaptureAttributeItemsAccessor>");
}

static void rna_def_geo_capture_attribute_items(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeGeometryCaptureAttributeItems", nullptr);
  RNA_def_struct_ui_text(srna, "Items", "Collection of capture attribute items");
  RNA_def_struct_sdna(srna, "bNode");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "NodeGeometryCaptureAttributeItem", "CaptureAttributeItemsAccessor");
  rna_def_node_item_array_common_functions(
      srna, "NodeGeometryCaptureAttributeItem", "CaptureAttributeItemsAccessor");
}

static void rna_def_geo_capture_attribute(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_geo_capture_attribute_item(brna);
  rna_def_geo_capture_attribute_items(brna);

  RNA_def_struct_sdna_from(srna, "NodeGeometryAttributeCapture", "storage");

  prop = RNA_def_property(srna, "capture_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "capture_items", "capture_items_num");
  RNA_def_property_struct_type(prop, "NodeGeometryCaptureAttributeItem");
  RNA_def_property_ui_text(prop, "Items", "");
  RNA_def_property_srna(prop, "NodeGeometryCaptureAttributeItems");

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_item", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "RepeatItem");
  RNA_def_property_pointer_funcs(prop,
                                 "rna_Node_ItemArray_active_get<CaptureAttributeItemsAccessor>",
                                 "rna_Node_ItemArray_active_set<CaptureAttributeItemsAccessor>",
                                 nullptr,
                                 nullptr);
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_NO_DEG_UPDATE);
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "domain", PROP_ENUM, PROP_NONE);
  RNA_def_property_ui_text(prop, "Domain", "Which domain to store the data in");
  RNA_def_property_enum_items(prop, rna_enum_attribute_domain_items);
  RNA_def_property_enum_funcs(
      prop, nullptr, nullptr, "rna_GeometryNodeAttributeDomain_attribute_domain_itemf");
  RNA_def_property_update(prop, NC_NODE, "rna_Node_update");
}

static void def_set_closure_default(BlenderRNA * /*brna*/, StructRNA * /*srna*/) {}

static void rna_def_geo_bake_item(BlenderRNA *brna)
{
  PropertyRNA *prop;

  StructRNA *srna = RNA_def_struct(brna, "NodeGeometryBakeItem", nullptr);
  RNA_def_struct_ui_text(srna, "Bake Item", "");

  rna_def_node_item_array_socket_item_common(srna, "BakeItemsAccessor", true);

  prop = RNA_def_property(srna, "attribute_domain", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_attribute_domain_items);
  RNA_def_property_enum_funcs(
      prop, nullptr, nullptr, "rna_GeometryNodeAttributeDomain_attribute_domain_itemf");
  RNA_def_property_ui_text(
      prop,
      "Attribute Domain",
      "Domain where the field is captured if it is not already an attribute-field");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(
      prop, NC_NODE | NA_EDITED, "rna_Node_ItemArray_item_update<BakeItemsAccessor>");

  prop = RNA_def_property(srna, "is_attribute", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GEO_NODE_BAKE_ITEM_IS_ATTRIBUTE);
  RNA_def_property_ui_text(prop, "Is Attribute", "Bake item is an attribute stored on a geometry");
  RNA_def_property_update(
      prop, NC_NODE | NA_EDITED, "rna_Node_ItemArray_item_update<BakeItemsAccessor>");
}

static void rna_def_geo_bake_items(BlenderRNA *brna)
{
  StructRNA *srna;

  srna = RNA_def_struct(brna, "NodeGeometryBakeItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Items", "Collection of bake items");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "NodeGeometryBakeItem", "BakeItemsAccessor");
  rna_def_node_item_array_common_functions(srna, "NodeGeometryBakeItem", "BakeItemsAccessor");
}

static void rna_def_geo_bake(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_geo_bake_item(brna);
  rna_def_geo_bake_items(brna);

  RNA_def_struct_sdna_from(srna, "NodeGeometryBake", "storage");

  prop = RNA_def_property(srna, "bake_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "items", "items_num");
  RNA_def_property_struct_type(prop, "NodeGeometryBakeItem");
  RNA_def_property_ui_text(prop, "Items", "");
  RNA_def_property_srna(prop, "NodeGeometryBakeItems");

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_item", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "RepeatItem");
  RNA_def_property_pointer_funcs(prop,
                                 "rna_Node_ItemArray_active_get<BakeItemsAccessor>",
                                 "rna_Node_ItemArray_active_set<BakeItemsAccessor>",
                                 nullptr,
                                 nullptr);
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_NO_DEG_UPDATE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_update(prop, NC_NODE, nullptr);
}

static void rna_def_image_bake_item(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeImageBakeItem", nullptr);
  RNA_def_struct_ui_text(
      srna, "Bake Image Item", "Named Color image slot written on Bake (name = filename)");

  /* Name only ?? all items are Color. */
  rna_def_node_item_array_socket_item_common(srna, "ImageBakeItemsAccessor", false);
}

static void rna_def_image_bake_items(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeImageBakeItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Items", "Collection of bake image slots");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "NodeImageBakeItem", "ImageBakeItemsAccessor");
  rna_def_node_item_array_common_functions(srna, "NodeImageBakeItem", "ImageBakeItemsAccessor");
}

static void def_image_import_points(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  /* Switchable among COP-Import Point* locked Geometry groups only (search filter via poll). */
  prop = RNA_def_property(srna, "node_tree", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "NodeTree");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_pointer_funcs(prop,
                                 nullptr,
                                 "rna_ImageImportPoints_node_tree_set",
                                 nullptr,
                                 "rna_ImageImportPoints_node_tree_poll");
  RNA_def_property_ui_text(
      prop,
      "Node Group",
      "COP Import Points Geometry group (name COP-Import Point[.N]). Search lists only those "
      "groups. Group Output: Points, Rotation, Size, ID. Position/Size use unit square [0,1]");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_image_import_geo(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  /* Switchable among COP-Import Geo* locked Geometry groups only (search filter via poll). */
  prop = RNA_def_property(srna, "node_tree", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "NodeTree");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_pointer_funcs(prop,
                                 nullptr,
                                 "rna_ImageImportGeo_node_tree_set",
                                 nullptr,
                                 "rna_ImageImportGeo_node_tree_poll");
  RNA_def_property_ui_text(
      prop,
      "Node Group",
      "COP Import Geo Geometry group (name COP-Import Geo[.N]). Search lists only those "
      "groups. Group Output: Geometry");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_image_bake_image(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  static const EnumPropertyItem format_items[] = {
      {0, "PNG", 0, "PNG", "Portable Network Graphics"},
      {1, "JPEG", 0, "JPEG", "JPEG"},
      {2, "OPEN_EXR", 0, "OpenEXR", "OpenEXR (float)"},
      {3, "TIFF", 0, "TIFF", "TIFF"},
      {4, "BMP", 0, "BMP", "Bitmap"},
      {5, "TARGA", 0, "Targa", "Targa"},
      {6, "WEBP", 0, "WebP", "WebP"},
      {7, "HDR", 0, "Radiance HDR", "Radiance HDR (float)"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  rna_def_image_bake_item(brna);
  rna_def_image_bake_items(brna);

  RNA_def_struct_sdna_from(srna, "NodeImageBakeImage", "storage");

  prop = RNA_def_property(srna, "format", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "format");
  RNA_def_property_enum_items(prop, format_items);
  RNA_def_property_ui_text(prop, "Format", "Image file format written on Bake");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "color_space", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_color_space_convert_default_items);
  RNA_def_property_enum_funcs(prop,
                              "rna_ImageBakeImage_color_space_get",
                              "rna_ImageBakeImage_color_space_set",
                              "rna_ImageBakeImage_color_space_itemf");
  RNA_def_property_ui_text(prop,
                           "Color Space",
                           "Color space of the written file. Auto picks scene linear for EXR/HDR "
                           "and the default byte space for other formats");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "bake_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "items", "items_num");
  RNA_def_property_struct_type(prop, "NodeImageBakeItem");
  RNA_def_property_ui_text(
      prop,
      "Items",
      "Color image slots to bake (item name = filename; use Prefix for a common prefix)");
  RNA_def_property_srna(prop, "NodeImageBakeItems");

  prop = RNA_def_property(srna, "active_item_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active bake item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, nullptr);
}

static void rna_def_rasterize_points_item(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  static const EnumPropertyItem type_items[] = {
      {GEO_NODE_RASTERIZE_POINTS_ITEM_TYPE_SCALAR,
       "SCALAR",
       0,
       "Scalar",
       "Rasterize a scalar attribute"},
      {GEO_NODE_RASTERIZE_POINTS_ITEM_TYPE_VECTOR,
       "VECTOR",
       0,
       "Vector",
       "Rasterize a vector attribute"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  srna = RNA_def_struct(brna, "NodeGeometryRasterizePointsItem", nullptr);
  RNA_def_struct_ui_text(srna, "Rasterize Points Item", "");
  RNA_def_struct_sdna(srna, "NodeGeometryRasterizePointsItem");

  prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
  RNA_def_property_string_funcs(
      prop, nullptr, nullptr, "rna_Node_ItemArray_item_name_set<RasterizePointsItemsAccessor>");
  RNA_def_property_ui_text(prop, "Name", "");
  RNA_def_struct_name_property(srna, prop);
  RNA_def_property_update(
      prop, NC_NODE | NA_EDITED, "rna_Node_ItemArray_item_update<RasterizePointsItemsAccessor>");

  prop = RNA_def_property(srna, "type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, type_items);
  RNA_def_property_ui_text(prop, "Type", "");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(
      prop, NC_NODE | NA_EDITED, "rna_Node_ItemArray_item_update<RasterizePointsItemsAccessor>");
}

static void rna_def_geo_rasterize_points_items(BlenderRNA *brna)
{
  StructRNA *srna;
  FunctionRNA *func;
  PropertyRNA *parm;

  srna = RNA_def_struct(brna, "NodeGeometryRasterizePointsItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Items", "Collection of grid items");

  func = RNA_def_function(srna, "new", "rna_NodeGeometryRasterizePointsItems_new");
  RNA_def_function_ui_description(func, "Add an a new point attribute item");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN);
  parm = RNA_def_string(func, "name", nullptr, MAX_NAME, "Name", "");
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  /* return value */
  parm = RNA_def_pointer(func, "item", "NodeGeometryRasterizePointsItem", "Item", "New item");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, ParameterFlag(0));
  RNA_def_function_return(func, parm);

  rna_def_node_item_array_common_functions(
      srna, "NodeGeometryRasterizePointsItem", "RasterizePointsItemsAccessor");
}

static void def_geo_rasterize_points(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_rasterize_points_item(brna);
  rna_def_geo_rasterize_points_items(brna);

  RNA_def_struct_sdna_from(srna, "NodeGeometryRasterizePoints", "storage");

  prop = RNA_def_property(srna, "rasterize_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "items", "items_num");
  RNA_def_property_struct_type(prop, "NodeGeometryRasterizePointsItem");
  RNA_def_property_ui_text(prop, "Items", "");
  RNA_def_property_srna(prop, "NodeGeometryRasterizePointsItems");

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_item", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "NodeGeometryRasterizePointsItem");
  RNA_def_property_pointer_funcs(prop,
                                 "rna_Node_ItemArray_active_get<RasterizePointsItemsAccessor>",
                                 "rna_Node_ItemArray_active_set<RasterizePointsItemsAccessor>",
                                 nullptr,
                                 nullptr);
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_NO_DEG_UPDATE);
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_update(prop, NC_NODE, nullptr);
}

static void rna_def_expression_input_item(BlenderRNA *brna)
{
  StructRNA *srna;

  srna = RNA_def_struct(brna, "NodeExpressionInputItem", nullptr);
  RNA_def_struct_ui_text(srna, "Expression Input Item", "");
  RNA_def_struct_sdna(srna, "NodeExpressionInputItem");

  rna_def_node_item_array_socket_item_common(srna, "ExpressionInputItemsAccessor", true);
}

static void rna_def_expression_input_items(BlenderRNA *brna)
{
  StructRNA *srna;

  srna = RNA_def_struct(brna, "NodeExpressionInputItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Input Items", "Collection of input items");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "NodeExpressionInputItem", "ExpressionInputItemsAccessor");
  rna_def_node_item_array_common_functions(
      srna, "NodeExpressionInputItem", "ExpressionInputItemsAccessor");
}

static void rna_def_expression_item(BlenderRNA *brna)
{
  StructRNA *srna;

  srna = RNA_def_struct(brna, "NodeExpressionItem", nullptr);
  RNA_def_struct_ui_text(srna, "Expression Output Item", "");
  RNA_def_struct_sdna(srna, "NodeExpressionItem");

  rna_def_node_item_array_socket_item_common(srna, "ExpressionItemsAccessor", true);
}

static void rna_def_expression_items(BlenderRNA *brna)
{
  StructRNA *srna;

  srna = RNA_def_struct(brna, "NodeExpressionItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Expression Items", "Collection of expression items");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "NodeExpressionItem", "ExpressionItemsAccessor");
  rna_def_node_item_array_common_functions(srna, "NodeExpressionItem", "ExpressionItemsAccessor");
}

static void def_expression(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_expression_input_item(brna);
  rna_def_expression_input_items(brna);

  rna_def_expression_item(brna);
  rna_def_expression_items(brna);

  RNA_def_struct_sdna_from(srna, "NodeExpression", "storage");

  prop = RNA_def_property(srna, "input_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "input_items.items", "input_items.items_num");
  RNA_def_property_struct_type(prop, "NodeExpressionInputItem");
  RNA_def_property_srna(prop, "NodeExpressionInputItems");

  prop = RNA_def_property(srna, "expression_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(
      prop, nullptr, "expression_items.items", "expression_items.items_num");
  RNA_def_property_struct_type(prop, "NodeExpressionItem");
  RNA_def_property_srna(prop, "NodeExpressionItems");

  prop = RNA_def_property(srna, "active_input_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "input_items.active_index");
  RNA_def_property_ui_text(prop, "Active Input Index", "Index of the active input item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_expression_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "expression_items.active_index");
  RNA_def_property_ui_text(prop, "Active Expression Index", "Index of the active expression item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);
}

static void rna_def_index_switch_item(BlenderRNA *brna)
{
  PropertyRNA *prop;

  StructRNA *srna = RNA_def_struct(brna, "IndexSwitchItem", nullptr);
  RNA_def_struct_ui_text(srna, "Index Switch Item", "");
  RNA_def_struct_sdna(srna, "IndexSwitchItem");

  prop = RNA_def_property(srna, "identifier", PROP_INT, PROP_NONE);
  RNA_def_property_ui_range(prop, 0, INT32_MAX, 1, -1);
  RNA_def_property_ui_text(prop, "Identifier", "Consistent identifier used for the item");
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_update(prop, NC_NODE, "rna_Node_update");
}

static void rna_def_geo_index_switch_items(BlenderRNA *brna)
{
  StructRNA *srna;
  FunctionRNA *func;
  PropertyRNA *parm;

  srna = RNA_def_struct(brna, "NodeIndexSwitchItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Items", "Collection of index_switch items");

  func = RNA_def_function(srna, "new", "rna_NodeIndexSwitchItems_new");
  RNA_def_function_ui_description(func, "Add an item at the end");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN);
  /* Return value. */
  parm = RNA_def_pointer(func, "item", "IndexSwitchItem", "Item", "New item");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, ParameterFlag(0));
  RNA_def_function_return(func, parm);

  rna_def_node_item_array_common_functions(srna, "IndexSwitchItem", "IndexSwitchItemsAccessor");
}

static void def_geo_index_switch(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_index_switch_item(brna);
  rna_def_geo_index_switch_items(brna);

  RNA_def_struct_sdna_from(srna, "NodeIndexSwitch", "storage");

  prop = RNA_def_property(srna, "index_switch_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "items", "items_num");
  RNA_def_property_struct_type(prop, "IndexSwitchItem");
  RNA_def_property_ui_text(prop, "Items", "");
  RNA_def_property_srna(prop, "NodeIndexSwitchItems");
}

static void rna_def_combine_list_item(BlenderRNA *brna)
{
  PropertyRNA *prop;

  StructRNA *srna = RNA_def_struct(brna, "CombineListItem", nullptr);
  RNA_def_struct_ui_text(srna, "Combine List Item", "");
  RNA_def_struct_sdna(srna, "CombineListItem");

  prop = RNA_def_property(srna, "identifier", PROP_INT, PROP_NONE);
  RNA_def_property_ui_range(prop, 0, INT32_MAX, 1, -1);
  RNA_def_property_ui_text(prop, "Identifier", "Consistent identifier used for the item");
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_update(prop, NC_NODE, "rna_Node_update");
}

static void rna_def_geo_combine_list_items(BlenderRNA *brna)
{
  StructRNA *srna;
  FunctionRNA *func;
  PropertyRNA *parm;

  srna = RNA_def_struct(brna, "NodeCombineListItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(srna, "Items", "Collection of combine list items");

  func = RNA_def_function(srna, "new", "rna_NodeCombineListItems_new");
  RNA_def_function_ui_description(func, "Add an item at the end");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN);

  parm = RNA_def_pointer(func, "item", "CombineListItem", "Item", "New item");
  RNA_def_function_return(func, parm);

  rna_def_node_item_array_common_functions(srna, "CombineListItem", "CombineListItemsAccessor");
}

static void def_geo_combine_list(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_combine_list_item(brna);
  rna_def_geo_combine_list_items(brna);

  RNA_def_struct_sdna_from(srna, "NodeCombineList", "storage");

  prop = RNA_def_property(srna, "combine_list_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "items", "items_num");
  RNA_def_property_struct_type(prop, "CombineListItem");
  RNA_def_property_ui_text(prop, "Items", "");
  RNA_def_property_srna(prop, "NodeCombineListItems");
}

static void rna_def_geo_field_to_grid_item(BlenderRNA *brna)
{
  PropertyRNA *prop;

  StructRNA *srna = RNA_def_struct(brna, "GeometryNodeFieldToGridItem", nullptr);
  RNA_def_struct_ui_text(srna, "Field to Grid Item", "");
  RNA_def_struct_sdna(srna, "GeometryNodeFieldToGridItem");

  rna_def_node_item_array_socket_item_common(srna, "FieldToGridItemsAccessor", false);
  prop = RNA_def_property(srna, "data_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_node_socket_data_type_items);
  RNA_def_property_enum_funcs(prop, nullptr, nullptr, "rna_NodeFieldToGridItem_data_type_itemf");
  RNA_def_property_ui_text(prop, "Data Type", "");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(
      prop, NC_NODE | NA_EDITED, "rna_Node_ItemArray_item_update<FieldToGridItemsAccessor>");

  prop = RNA_def_property(srna, "identifier", PROP_INT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
}

static void rna_def_geo_field_to_grid_items(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "GeometryNodeFieldToGridItems", nullptr);
  RNA_def_struct_ui_text(srna, "Items", "Collection of field to grid items");
  RNA_def_struct_sdna(srna, "bNode");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "GeometryNodeFieldToGridItem", "FieldToGridItemsAccessor");
  rna_def_node_item_array_common_functions(
      srna, "GeometryNodeFieldToGridItem", "FieldToGridItemsAccessor");
}

static void rna_def_geo_closure_to_list_item(BlenderRNA *brna)
{
  PropertyRNA *prop;

  StructRNA *srna = RNA_def_struct(brna, "GeometryNodeClosureToListItem", nullptr);
  RNA_def_struct_ui_text(srna, "Closure to List Item", "");
  RNA_def_struct_sdna(srna, "GeometryNodeClosureToListItem");

  rna_def_node_item_array_socket_item_common(srna, "ClosureToListItemsAccessor", true);

  prop = RNA_def_property(srna, "structure_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_node_socket_structure_type_items);
  RNA_def_property_enum_funcs(
      prop, nullptr, nullptr, "rna_GeometryNodeClosureToListItem_structure_type_itemf");

  prop = RNA_def_property(srna, "identifier", PROP_INT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
}

static void rna_def_geo_closure_to_list_items(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "GeometryNodeClosureToListItems", nullptr);
  RNA_def_struct_ui_text(srna, "Items", "Collection of closure to list items");
  RNA_def_struct_sdna(srna, "bNode");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "GeometryNodeClosureToListItem", "ClosureToListItemsAccessor");
  rna_def_node_item_array_common_functions(
      srna, "GeometryNodeClosureToListItem", "ClosureToListItemsAccessor");
}

static void def_geo_closure_to_list(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_geo_closure_to_list_item(brna);
  rna_def_geo_closure_to_list_items(brna);

  RNA_def_struct_sdna_from(srna, "GeometryNodeClosureToList", "storage");

  prop = RNA_def_property(srna, "list_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "items", "items_num");
  RNA_def_property_struct_type(prop, "GeometryNodeClosureToListItem");
  RNA_def_property_ui_text(prop, "Items", "");
  RNA_def_property_srna(prop, "GeometryNodeClosureToListItems");

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_item", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "RepeatItem");
  RNA_def_property_pointer_funcs(prop,
                                 "rna_Node_ItemArray_active_get<ClosureToListItemsAccessor>",
                                 "rna_Node_ItemArray_active_set<ClosureToListItemsAccessor>",
                                 nullptr,
                                 nullptr);
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_NO_DEG_UPDATE);
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_update(prop, NC_NODE, nullptr);
}

static void def_geo_field_to_grid(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_geo_field_to_grid_item(brna);
  rna_def_geo_field_to_grid_items(brna);

  RNA_def_struct_sdna_from(srna, "GeometryNodeFieldToGrid", "storage");

  prop = RNA_def_property(srna, "grid_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "items", "items_num");
  RNA_def_property_struct_type(prop, "GeometryNodeFieldToGridItem");
  RNA_def_property_ui_text(prop, "Items", "");
  RNA_def_property_srna(prop, "GeometryNodeFieldToGridItems");

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_item", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "RepeatItem");
  RNA_def_property_pointer_funcs(prop,
                                 "rna_Node_ItemArray_active_get<FieldToGridItemsAccessor>",
                                 "rna_Node_ItemArray_active_set<FieldToGridItemsAccessor>",
                                 nullptr,
                                 nullptr);
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_NO_DEG_UPDATE);
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "data_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_node_socket_data_type_items);
  RNA_def_property_enum_funcs(prop, nullptr, nullptr, "rna_NodeFieldToGridItem_data_type_itemf");
  RNA_def_property_ui_text(prop, "Data Type", "Data type for topology grid");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");
}

static void rna_def_geo_field_to_list_item(BlenderRNA *brna)
{
  PropertyRNA *prop;

  StructRNA *srna = RNA_def_struct(brna, "GeometryNodeFieldToListItem", nullptr);
  RNA_def_struct_ui_text(srna, "Field to List Item", "");
  RNA_def_struct_sdna(srna, "GeometryNodeFieldToListItem");

  rna_def_node_item_array_socket_item_common(srna, "FieldToListItemsAccessor", true);

  prop = RNA_def_property(srna, "identifier", PROP_INT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
}

static void rna_def_geo_field_to_list_items(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "GeometryNodeFieldToListItems", nullptr);
  RNA_def_struct_ui_text(srna, "Items", "Collection of field to list items");
  RNA_def_struct_sdna(srna, "bNode");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "GeometryNodeFieldToListItem", "FieldToListItemsAccessor");
  rna_def_node_item_array_common_functions(
      srna, "GeometryNodeFieldToListItem", "FieldToListItemsAccessor");
}

static void def_geo_field_to_list(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_geo_field_to_list_item(brna);
  rna_def_geo_field_to_list_items(brna);

  RNA_def_struct_sdna_from(srna, "GeometryNodeFieldToList", "storage");

  prop = RNA_def_property(srna, "list_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "items", "items_num");
  RNA_def_property_struct_type(prop, "GeometryNodeFieldToListItem");
  RNA_def_property_ui_text(prop, "Items", "");
  RNA_def_property_srna(prop, "GeometryNodeFieldToListItems");

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_item", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "GeometryNodeFieldToListItem");
  RNA_def_property_pointer_funcs(prop,
                                 "rna_Node_ItemArray_active_get<FieldToListItemsAccessor>",
                                 "rna_Node_ItemArray_active_set<FieldToListItemsAccessor>",
                                 nullptr,
                                 nullptr);
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_NO_DEG_UPDATE);
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_update(prop, NC_NODE, nullptr);
}

static void def_geo_curve_handle_type_selection(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeGeometryCurveSelectHandles", "storage");

  prop = RNA_def_property(srna, "handle_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "handle_type");
  RNA_def_property_ui_text(prop, "Handle Type", "");
  RNA_def_property_enum_items(prop, rna_node_geometry_curve_handle_type_items);
  RNA_def_property_enum_default(prop, GEO_NODE_CURVE_HANDLE_AUTO);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");

  prop = RNA_def_property(srna, "mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_flag(prop, PROP_ENUM_FLAG);
  RNA_def_property_enum_items(prop, rna_enum_node_geometry_curve_handle_side_items);
  RNA_def_property_enum_default(prop, GEO_NODE_CURVE_HANDLE_LEFT | GEO_NODE_CURVE_HANDLE_RIGHT);
  RNA_def_property_ui_text(prop, "Mode", "Whether to check the type of left and right handles");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");
}

static void def_geo_sample_index(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeGeometrySampleIndex", "storage");

  prop = RNA_def_property(srna, "data_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_attribute_type_items);
  RNA_def_property_enum_funcs(
      prop, nullptr, nullptr, "rna_GeometryNodeAttributeType_type_with_socket_itemf");
  RNA_def_property_enum_default(prop, CD_PROP_FLOAT);
  RNA_def_property_ui_text(prop, "Data Type", "");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");

  prop = RNA_def_property(srna, "domain", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_attribute_domain_items);
  RNA_def_property_enum_funcs(
      prop, nullptr, nullptr, "rna_GeometryNodeAttributeDomain_attribute_domain_itemf");
  RNA_def_property_enum_default(prop, int(bke::AttrDomain::Point));
  RNA_def_property_ui_text(prop, "Domain", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "clamp", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_ui_text(prop,
                           "Clamp",
                           "Clamp the indices to the size of the attribute domain instead of "
                           "outputting a default value for invalid indices");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_geo_input_material(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "material", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Material");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(prop, "Material", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_geo_input_collection(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "collection", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Collection");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(prop, "Collection", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_geo_input_font(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "font", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "VectorFont");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(prop, "Font", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_geo_input_normal(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop = RNA_def_property(srna, "legacy_corner_normals", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "custom1", 1);
  RNA_def_property_ui_text(
      prop,
      "Flat Corner Normals",
      "Always use face normals for the face corner domain, matching old behavior of the node");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_geo_input_object(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "object", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Object");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(prop, "Object", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_geo_image(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "image", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Image");
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_ID_REFCOUNT);
  RNA_def_property_ui_text(prop, "Image", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
  RNA_def_property_pointer_funcs(
      prop, nullptr, nullptr, nullptr, "rna_Image_no_renderresult_or_viewer_poll");
}

static void def_geo_modal_event(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "GeometryNodeModalEvent", "storage");

  prop = RNA_def_property(srna, "event_name", PROP_STRING, PROP_NONE);
  RNA_def_property_string_sdna(prop, nullptr, "name");
  RNA_def_property_ui_text(
      prop, "Name", "Name of the event that the node tool reacts to while it keeps running");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "description", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(
      prop, "Description", "Explanation of what the node tool does when the event is processed");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void rna_def_geometry_node_modal_keymap_item(BlenderRNA *brna)
{
  PropertyRNA *prop;

  StructRNA *srna = RNA_def_struct(brna, "GeometryNodeModalKeymapItem", nullptr);
  RNA_def_struct_sdna(srna, "GeometryNodeModalKeymapItem");
  RNA_def_struct_ui_text(srna,
                         "Modal Keymap Item",
                         "Default key binding for one of the events that a node tool reacts to");

  prop = RNA_def_property(srna, "event_name", PROP_STRING, PROP_NONE);
  RNA_def_property_string_sdna(prop, nullptr, "event_name");
  RNA_def_property_ui_text(
      prop, "Name", "Name of the event, matching the name of a Modal Event node");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_GeometryNodeModalKeymapItem_update");

  prop = RNA_def_property(srna, "map_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_keymap_item_map_type_items);
  RNA_def_property_enum_funcs(prop,
                              "rna_GeometryNodeModalKeymapItem_map_type_get",
                              "rna_GeometryNodeModalKeymapItem_map_type_set",
                              nullptr);
  RNA_def_property_ui_text(prop, "Map Type", "Type of event mapping");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_GeometryNodeModalKeymapItem_update");

  prop = RNA_def_property(srna, "type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "type");
  RNA_def_property_enum_items(prop, rna_enum_event_type_items);
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_UI_EVENTS);
  RNA_def_property_ui_text(prop, "Type", "Type of event that triggers this item by default");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_GeometryNodeModalKeymapItem_update");

  prop = RNA_def_property(srna, "value", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "val");
  RNA_def_property_enum_items(prop, rna_enum_event_value_items);
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_UI_EVENTS);
  RNA_def_property_ui_text(prop, "Value", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_GeometryNodeModalKeymapItem_update");

  prop = RNA_def_property(srna, "key_modifier", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "keymodifier");
  RNA_def_property_enum_items(prop, rna_enum_event_type_items);
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_UI_EVENTS);
  RNA_def_property_ui_text(prop, "Key Modifier", "Regular key pressed as a modifier");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_GeometryNodeModalKeymapItem_update");

  prop = RNA_def_property(srna, "repeat", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "repeat", 1);
  RNA_def_property_ui_text(
      prop, "Repeat", "Run the tool again for every key-repeat event while the key is held");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_GeometryNodeModalKeymapItem_update");

  struct {
    const char *identifier;
    const char *dna_name;
    const char *ui_name;
  } modifiers[] = {
      {"shift", "shift", "Shift"},
      {"ctrl", "ctrl", "Ctrl"},
      {"alt", "alt", "Alt"},
      {"oskey", "oskey", "OS Key"},
      {"hyper", "hyper", "Hyper"},
  };
  for (const auto &modifier : modifiers) {
    prop = RNA_def_property(srna, modifier.identifier, PROP_BOOLEAN, PROP_NONE);
    RNA_def_property_boolean_sdna(prop, nullptr, modifier.dna_name, KM_MOD_HELD);
    RNA_def_property_ui_text(prop, modifier.ui_name, "");
    RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_GeometryNodeModalKeymapItem_update");
  }
}

static void rna_def_geo_menu_switch_item(BlenderRNA *brna)
{
  PropertyRNA *prop;

  StructRNA *srna = RNA_def_struct(brna, "NodeEnumItem", nullptr);
  RNA_def_struct_ui_text(srna, "Enum Item", "");

  prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
  RNA_def_property_string_funcs(
      prop, nullptr, nullptr, "rna_Node_ItemArray_item_name_set<MenuSwitchItemsAccessor>");
  RNA_def_property_ui_text(prop, "Name", "");
  RNA_def_struct_name_property(srna, prop);
  RNA_def_property_update(
      prop, NC_NODE | NA_EDITED, "rna_Node_ItemArray_item_update<MenuSwitchItemsAccessor>");

  prop = RNA_def_property(srna, "description", PROP_STRING, PROP_NONE);
  RNA_def_property_string_sdna(prop, nullptr, "description");
  RNA_def_property_ui_text(prop, "Description", "");
  RNA_def_property_update(
      prop, NC_NODE | NA_EDITED, "rna_Node_ItemArray_item_update<MenuSwitchItemsAccessor>");
}

static void rna_def_geo_menu_switch_items(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *parm;
  FunctionRNA *func;

  srna = RNA_def_struct(brna, "NodeMenuSwitchItems", nullptr);
  RNA_def_struct_sdna(srna, "bNode");
  RNA_def_struct_ui_text(
      srna, "Enum Definition Items", "Collection of items that make up an enum");

  func = RNA_def_function(srna, "new", "rna_NodeMenuSwitchItems_new");
  RNA_def_function_ui_description(func, "Add an a new enum item");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID | FUNC_USE_MAIN);
  parm = RNA_def_string(func, "name", nullptr, MAX_NAME, "Name", "");
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  /* return value */
  parm = RNA_def_pointer(func, "item", "NodeEnumItem", "Item", "New item");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, ParameterFlag(0));
  RNA_def_function_return(func, parm);

  rna_def_node_item_array_common_functions(srna, "NodeEnumItem", "MenuSwitchItemsAccessor");
}

static void def_geo_menu_switch(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_geo_menu_switch_item(brna);
  rna_def_geo_menu_switch_items(brna);

  RNA_def_struct_sdna_from(srna, "NodeMenuSwitch", "storage");

  prop = RNA_def_property(srna, "enum_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(
      prop, nullptr, "enum_definition.items_array", "enum_definition.items_num");
  RNA_def_property_struct_type(prop, "NodeEnumItem");
  RNA_def_property_ui_text(prop, "Items", "");
  RNA_def_property_srna(prop, "NodeMenuSwitchItems");

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "enum_definition.active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_item", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "NodeEnumItem");
  RNA_def_property_pointer_funcs(prop,
                                 "rna_Node_ItemArray_active_get<MenuSwitchItemsAccessor>",
                                 "rna_Node_ItemArray_active_set<MenuSwitchItemsAccessor>",
                                 nullptr,
                                 nullptr);
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_ui_text(prop, "Active Item", "Active item");
  RNA_def_property_update(prop, NC_NODE, nullptr);

  /* This exists only for backward compatibility. */
  prop = RNA_def_property(srna, "enum_definition", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "Node");
  RNA_def_property_pointer_funcs(
      prop, "rna_NodeMenuSwitch_enum_definition_get", nullptr, nullptr, nullptr);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_NO_COMPARISON);
  RNA_def_property_ui_text(prop,
                           "Enum Definition (deprecated)",
                           "The enum definition can now be accessed directly on the node. This "
                           "exists for backward compatibility.");
}

void RNA_def_nodes_geometry(BlenderRNA *brna)
{
  const auto define = [&](const char *base_name,
                          const char *struct_name,
                          void (*func)(BlenderRNA *, StructRNA *) = nullptr) {
    StructRNA *srna = define_specific_node(brna, struct_name, base_name);
    if (func) {
      func(brna, srna);
    }
  };

  /* clang-format off */

  define("GeometryNode", "GeometryNodeAccumulateField");
  define("GeometryNode", "GeometryNodeAttributeDomainSize");
  define("GeometryNode", "GeometryNodeAttributeStatistic");
  define("GeometryNode", "GeometryNodeAttributeTransfer");
  define("GeometryNode", "GeometryNodeBake", rna_def_geo_bake);
  define("GeometryNode", "GeometryNodeBlurAttribute");
  define("GeometryNode", "GeometryNodeBoneInfo");
  define("GeometryNode", "GeometryNodeBoundBox");
  define("GeometryNode", "GeometryNodeCameraInfo");
  define("GeometryNode", "GeometryNodeCaptureAttribute", rna_def_geo_capture_attribute);
  define("GeometryNode", "GeometryNodeClosureToList", def_geo_closure_to_list);
  define("GeometryNode", "GeometryNodeClusterByConnected");
  define("GeometryNode", "GeometryNodeClusterByDistance");
  define("GeometryNode", "GeometryNodeCollectionChildren");
  define("GeometryNode", "GeometryNodeCollectionInfo");
  define("GeometryNode", "GeometryNodeCgalAdvancingFront");
  define("GeometryNode", "GeometryNodeCgalAlphaShape");
  define("GeometryNode", "GeometryNodeCgalAlphaWrap");
  define("GeometryNode", "GeometryNodeCgalCorefine");
  define("GeometryNode", "GeometryNodeCgalDelaunay3D");
  define("GeometryNode", "GeometryNodeCgalFair");
  define("GeometryNode", "GeometryNodeCgalMeshBoolean");
  define("GeometryNode", "GeometryNodeCgalMinSphere");
  define("GeometryNode", "GeometryNodeCgalOptimalBBox");
  define("GeometryNode", "GeometryNodeCgalRepair");
  define("GeometryNode", "GeometryNodeCgalSmoothShape");
  define("GeometryNode", "GeometryNodeCgalOrientOutward");
  define("GeometryNode", "GeometryNodeCgalRepairSelfIntersect");
  define("GeometryNode", "GeometryNodeCgalSplitLongEdges");
  define("GeometryNode", "GeometryNodeCgalDoesSelfIntersect");
  define("GeometryNode", "GeometryNodeCgalMeshVolume");
  define("GeometryNode", "GeometryNodeCgalEstimateNormals");
  define("GeometryNode", "GeometryNodeCgalPoisson");
  define("GeometryNode", "GeometryNodeCgalSideOfMesh");
  define("GeometryNode", "GeometryNodeCgalSkeleton");
  define("GeometryNode", "GeometryNodeCgalSegmentation");
  define("GeometryNode", "GeometryNodeCgalMinEllipsoid");
  define("GeometryNode", "GeometryNodeCgalIsClosed");
  define("GeometryNode", "GeometryNodeCgalCentroid");
  define("GeometryNode", "GeometryNodeCgalAutorefine");
  define("GeometryNode", "GeometryNodeCgalRemoveDegenerate");
  define("GeometryNode", "GeometryNodeCgalCurvature");
  define("GeometryNode", "GeometryNodeCgalScaleSpace");
  define("GeometryNode", "GeometryNodeCgalPrincipalCurvature");
  define("GeometryNode", "GeometryNodeCgalShapeDiameter");
  define("GeometryNode", "GeometryNodeCgalMarkSelfIntersect");
  define("GeometryNode", "GeometryNodeCgalWLOP");
  define("GeometryNode", "GeometryNodeCgalHierarchySimplify");
  define("GeometryNode", "GeometryNodeCgalEdgeAwareUpsample");
  define("GeometryNode", "GeometryNodeCgalVcmEstimateNormals");
  define("GeometryNode", "GeometryNodeCgalPcaEstimateNormals");
  define("GeometryNode", "GeometryNodeCgalMstOrientNormals");
  define("GeometryNode", "GeometryNodeCgalClusterPointSet");
  define("GeometryNode", "GeometryNodeCgalRegionGrowing");
  define("GeometryNode", "GeometryNodeCgalSurfaceShortestPath");
  define("GeometryNode", "GeometryNodeCgalJetSmooth");
  define("GeometryNode", "GeometryNodeCgalBilateralSmooth");
  define("GeometryNode", "GeometryNodeCgalRemoveOutliers");
  define("GeometryNode", "GeometryNodeCgalArapDeform");
  define("GeometryNode", "GeometryNodeCgalLscmUV");
  define("GeometryNode", "GeometryNodeCgalArapUV");
  define("GeometryNode", "GeometryNodeCgalUVDistortion");
  define("GeometryNode", "GeometryNodeCgalMeshIntersection");
  define("GeometryNode", "GeometryNodeCgalSkinSurface");
  define("GeometryNode", "GeometryNodeCgalMinSphereOfSpheres");
  define("GeometryNode", "GeometryNodeCgalMinRectangle2");
  define("GeometryNode", "GeometryNodeCgalMinEllipse2");
  define("GeometryNode", "GeometryNodeCgalMinParallelogram2");
  define("GeometryNode", "GeometryNodeCgalMeshSlicer");
  define("GeometryNode", "GeometryNodeCgalVcmFeatureEdges");
  define("GeometryNode", "GeometryNodeCgalDelaunay2");
  define("GeometryNode", "GeometryNodeCgalAlphaShape2");
  define("GeometryNode", "GeometryNodeCgalVoronoi2");
  define("GeometryNode", "GeometryNodeCgalStraightSkeleton2");
  define("GeometryNode", "GeometryNodeCgalPolygonOffset2");
  define("GeometryNode", "GeometryNodeCgalMinAnnulus2");
  define("GeometryNode", "GeometryNodeCgalConvexPartition2");
  define("GeometryNode", "GeometryNodeCgalMaxAreaKGon2");
  define("GeometryNode", "GeometryNodeCgalWidth3");
  define("GeometryNode", "GeometryNodeCgalMinkowskiSum2");
  define("GeometryNode", "GeometryNodeCgalExtrudeSkeleton");
  define("GeometryNode", "GeometryNodeCgalPolygonFill2");
  define("GeometryNode", "GeometryNodeCgalDoIntersect2");
  define("GeometryNode", "GeometryNodeCgalMedialAxis2");
  define("GeometryNode", "GeometryNodeCgalBooleanOps2");
  define("GeometryNode", "GeometryNodeCgalLargestEmptyIsoRectangle2");
  define("GeometryNode", "GeometryNodeCgalPolygonRepair2");
  /* GeometryNodeCgalRegularizeContour2 deleted */
  define("GeometryNode", "GeometryNodeCgalMongeJetFit");
  define("GeometryNode", "GeometryNodeCgalConvexDecomposition3");
  define("GeometryNode", "GeometryNodeCgalLaplaceDeform");
  define("GeometryNode", "GeometryNodeCgalSurfaceDelaunayRemesh");
  define("GeometryNode", "GeometryNodeCgalVoronoi3");
  define("GeometryNode", "GeometryNodeCgalVisibility2");
  define("GeometryNode", "GeometryNodeCgalRefineMesh2");
  define("GeometryNode", "GeometryNodeCgalAlphaComplex3");
  define("GeometryNode", "GeometryNodeCgalMinkowskiSum3");
  define("GeometryNode", "GeometryNodeCgalLargestEmptyCircle2");
  define("GeometryNode", "GeometryNodeCgalLargestInscribedCircle2");
  define("GeometryNode", "GeometryNodeCgalMinWidth2");
  define("GeometryNode", "GeometryNodeCgalPeriodicDelaunay2");
  define("GeometryNode", "GeometryNodeCgalArrangement2");
  define("GeometryNode", "GeometryNodeCgalDelaunayOnSphere");
  define("GeometryNode", "GeometryNodeCgalPeriodicDelaunay3");
  define("GeometryNode", "GeometryNodeCgalDoIntersect3");
  define("GeometryNode", "GeometryNodeCgalNaturalNeighbor2");
  /* GeometryNodeCgalCageDeform2 deleted */
  define("GeometryNode", "GeometryNodeCgalFillPolyline");
  define("GeometryNode", "GeometryNodeCgalMaxPerimeterKGon2");
  define("GeometryNode", "GeometryNodeCgalConstrainedDelaunay2");
  define("GeometryNode", "GeometryNodeCgalExteriorSkeleton2");
  define("GeometryNode", "GeometryNodeCgalVoronoiOnSphere");
  /* GeometryNodeCgalVisibilityHoles2 deleted */
  define("GeometryNode", "GeometryNodeCgalKnnGraph3");
  define("GeometryNode", "GeometryNodeCgalNaturalNeighbor3");
  define("GeometryNode", "GeometryNodeCgalPeriodicVoronoi2");
  define("GeometryNode", "GeometryNodeCgalPeriodicVoronoi3");
  define("GeometryNode", "GeometryNodeCgalEuclideanMst3");
  define("GeometryNode", "GeometryNodeCgalConvexLayers2");
  define("GeometryNode", "GeometryNodeCgalConvexLayers3");
  define("GeometryNode", "GeometryNodeCgalSelfIntersectionCurves");
  define("GeometryNode", "GeometryNodeCgalCrust2");
  define("GeometryNode", "GeometryNodeCgalGeodesicVoronoi");
  /* GeometryNodeCgalQemSimplify deleted */
  define("GeometryNode", "GeometryNodeCgalVerticalDecomp2");
  define("GeometryNode", "GeometryNodeCgalSimplePolygon2");
  define("GeometryNode", "GeometryNodeCgalSsabPartition2");
  define("GeometryNode", "GeometryNodeCgalSnapBorders");
  define("GeometryNode", "GeometryNodeCgalLargestEmptySphere3");
  define("GeometryNode", "GeometryNodeCgalInscribedSphere3");
  define("GeometryNode", "GeometryNodeCgalMinCylinder3");
  define("GeometryNode", "GeometryNodeCgalFairHoleFill");
  define("GeometryNode", "GeometryNodeCgalSimplifyPolyline3");
  /* Sphere/Cylinder/Circle/Line Region Growing merged into Point Shape Fitting */
  define("GeometryNode", "GeometryNodeCgalPointShapeFitting");
  /* GeometryNodeCgalPolarDual3 deleted */
  define("GeometryNode", "GeometryNodeCgalMinAnnulus3");
  /* GeometryNodeCgalPcaEllipsoid deleted */
  define("GeometryNode", "GeometryNodeCgalCollapseShortEdges");
  /* GeometryNodeCgalPolylineLineFitting deleted */
  /* GeometryNodeCgalSnapMeshes deleted */
  /* GeometryNodeCgalEnvelopeSimplify deleted */
  /* GeometryNodeCgalContractMesh deleted */
  define("GeometryNode", "GeometryNodeCgalRegularizeOpenContour2");
  /* GeometryNodeCgalHarmonicDeform2 deleted */
  define("GeometryNode", "GeometryNodeCgalConstrainedSimplify");
  /* GeometryNodeCgalSphereSlice deleted */
  /* GeometryNodeCgalCylinderSlice deleted */
  /* GeometryNodeCgalMergeCoplanar deleted */
  /* GeometryNodeCgalMvcUv deleted */
  /* GeometryNodeCgalCdtPlus2 deleted */
  define("GeometryNode", "GeometryNodeCgalYMonotonePartition2");
  define("GeometryNode", "GeometryNodeCgalPolylineSimplify2");
  define("GeometryNode", "GeometryNodeCgalGeodesicIsolines");
  define("GeometryNode", "GeometryNodeCgalContourStack");
  define("GeometryNode", "GeometryNodeCgalSkeletonSpokes");
  /* GeometryNodeCgalKineticPartition deleted */
  /* GeometryNodeCgalKineticReconstruct deleted */
  /* GeometryNodeCgalMinkowskiGlide3 deleted */
  /* GeometryNodeCgalRidgeCurves deleted */
  define("GeometryNode", "GeometryNodeCgalSphereIntersect");
  define("GeometryNode", "GeometryNodeCgalSphereArrangement");
  /* GeometryNodeCgalSegmentVoronoiLinf2 deleted */
  define("GeometryNode", "GeometryNodeCgalGraphcutSegment");
  /* GeometryNodeCgalCollisionDetect deleted */
  /* GeometryNodeCgalHyperbolicDelaunay2 deleted */
  /* GeometryNodeCgalVolumeMesh3 deleted */
  define("GeometryNode", "GeometryNodeCgalTetRemesh");
  /* GeometryNodeCgalPeriodicMesh3 deleted */
  /* GeometryNodeCgalSurfaceMesher deleted */
  define("GeometryNode", "GeometryNodeCgalPointFeatures");
  /* GeometryNodeCgalClassify deleted */
  define("GeometryNode", "GeometryNodeCgalRegisterIcp");
  define("GeometryNode", "GeometryNodeCgalOptimalTransport");
  /* GeometryNodeCenterOfMass deleted */
  /* GeometryNodeSupportPolygon deleted */
  /* GeometryNodeBalanceCheck deleted */
  /* GeometryNodeInnerCarve deleted */
  define("GeometryNode", "GeometryNodeMakeItStand");
  define("GeometryNode", "GeometryNodeRestingBodies");
  define("GeometryNode", "GeometryNodeBijectiveUV");
  define("GeometryNode", "GeometryNodeCgalSuper4pcs");
  /* GeometryNodeCgalRegularizeSegments2 deleted */
  /* GeometryNodeCgalPolyfit deleted */
  define("GeometryNode", "GeometryNodeCgalConstrainedDt3");
  define("GeometryNode", "GeometryNodeCgalDualContour");
  /* GeometryNodeCgalFrechetDistance deleted */
  define("GeometryNode", "GeometryNodeCgalAlphaWrap2");
  define("GeometryNode", "GeometryNodeCgalCageDeform3");
  /* GeometryNodeCgalAcvdRemesh deleted */
  /* GeometryNodeCgalPoissonDisk deleted */
  define("GeometryNode", "GeometryNodeCgalApproxConvexDecomp");
  /* GeometryNodeCgalExtremePoint3 deleted */
  /* GeometryNodeCgalBarycentric3 deleted */

  define("GeometryNode", "GeometryNodeCgalMinCircle2");
  define("GeometryNode", "GeometryNodeCgalVsaApproximate");
  define("GeometryNode", "GeometryNodeCgalUnionOfBalls");
  define("GeometryNode", "GeometryNodeCgalExactGeodesic");
  define("GeometryNode", "GeometryNodeCgalRefineIsolevel");
  define("GeometryNode", "GeometryNodeCgalRepairDegeneracies");
  define("GeometryNode", "GeometryNodeConstructCurves");
  define("GeometryNode", "GeometryNodeConvexHull");
  define("GeometryNode", "GeometryNodeCornersOfEdge");
  define("GeometryNode", "GeometryNodeCornersOfFace");
  define("GeometryNode", "GeometryNodeCornersOfVertex");
  define("GeometryNode", "GeometryNodeCubeGridTopology");
  define("GeometryNode", "GeometryNodeCurveArc");
  define("GeometryNode", "GeometryNodeCurveEndpointSelection");
  define("GeometryNode", "GeometryNodeCurveHandleTypeSelection", def_geo_curve_handle_type_selection);
  define("GeometryNode", "GeometryNodeCurveIntersections");
  define("GeometryNode", "GeometryNodeCurveLength");
  define("GeometryNode", "GeometryNodeCurveOfPoint");
  define("GeometryNode", "GeometryNodeCurvePrimitiveBezierSegment");
  define("GeometryNode", "GeometryNodeCurvePrimitiveCircle");
  define("GeometryNode", "GeometryNodeCurvePrimitiveLine");
  define("GeometryNode", "GeometryNodeCurvePrimitiveQuadrilateral");
  define("GeometryNode", "GeometryNodeCurveQuadraticBezier");
  define("GeometryNode", "GeometryNodeCurveSetHandles", def_geo_curve_set_handle_type);
  define("GeometryNode", "GeometryNodeCurveSpiral");
  define("GeometryNode", "GeometryNodeCurveSplineType");
  define("GeometryNode", "GeometryNodeCurveStar");
  define("GeometryNode", "GeometryNodeCurvesToGreasePencil");
  define("GeometryNode", "GeometryNodeCurveToMesh");
  define("GeometryNode", "GeometryNodeCurveToPoints");
  define("GeometryNode", "GeometryNodeDeformCurvesOnSurface");
  define("GeometryNode", "GeometryNodeDebug");
  /* GeometryNodeDelaunay3D deleted (non-CGAL Bowyer–Watson; use GeometryNodeCgalDelaunay3D) */
  define("GeometryNode", "GeometryNodeDelaunayTriangulation");
  define("GeometryNode", "GeometryNodeDeleteGeometry");
  define("GeometryNode", "GeometryNodeDissolveEdges");
  define("GeometryNode", "GeometryNodeDistributePointsInGrid");
  define("GeometryNode", "GeometryNodeDistributePointsInVolume");
  define("GeometryNode", "GeometryNodeDistributePointsOnFaces", def_geo_distribute_points_on_faces);
  define("GeometryNode", "GeometryNodeDualMesh");
  define("GeometryNode", "GeometryNodeDuplicateElements");
  define("GeometryNode", "GeometryNodeEdgePathsToCurves");
  define("GeometryNode", "GeometryNodeEdgePathsToSelection");
  define("GeometryNode", "GeometryNodeEdgesOfCorner");
  define("GeometryNode", "GeometryNodeEdgesOfVertex");
  define("GeometryNode", "GeometryNodeEdgesToFaceGroups");
  define("GeometryNode", "GeometryNodeExtrudeMesh");
  define("GeometryNode", "GeometryNodeSPOMPrisms");
  define("GeometryNode", "GeometryNodeFaceOfCorner");
  define("GeometryNode", "GeometryNodeFieldAtIndex");
  define("GeometryNode", "GeometryNodeFieldAverage");
  define("GeometryNode", "GeometryNodeFieldMinAndMax");
  define("GeometryNode", "GeometryNodeFieldOnDomain");
  define("GeometryNode", "GeometryNodeWriteAtIndex");
  define("GeometryNode", "GeometryNodeFieldToGrid", def_geo_field_to_grid);
  define("GeometryNode", "GeometryNodeFieldToList", def_geo_field_to_list);
  define("GeometryNode", "GeometryNodeFieldVariance");
  define("GeometryNode", "GeometryNodeFillCurve");
  define("GeometryNode", "GeometryNodeFilletCurve");
  define("GeometryNode", "GeometryNodeFilterList");
  define("GeometryNode", "GeometryNodeFlipFaces");
  define("GeometryNode", "GeometryNodeForeachGeometryElementInput", def_geo_foreach_geometry_element_input);
  define("GeometryNode", "GeometryNodeForeachGeometryElementOutput", def_geo_foreach_geometry_element_output);
  define("GeometryNode", "GeometryNodeGeometryClip");
  define("GeometryNode", "GeometryNodeSelectElements");
  define("GeometryNode", "GeometryNodeEditElements");
  define("GeometryNode", "GeometryNodePython", def_geo_python);
  define("GeometryNode", "GeometryNodeWindingNumber");
  define("GeometryNode", "GeometryNodeWrangle", def_geo_wrangle);
  define("GeometryNode", "GeometryNodeGeometryToInstance");
  define("GeometryNode", "GeometryNodeGetAttributeNames");
  define("GeometryNode", "GeometryNodeGetGeometryBundle");
  define("GeometryNode", "GeometryNodeGetGeometryComponent");
  define("GeometryNode", "GeometryNodeGetGridNames");
  define("GeometryNode", "GeometryNodeGetNamedGrid");
  define("GeometryNode", "GeometryNodeGizmoDial");
  define("GeometryNode", "GeometryNodeGizmoLinear");
  define("GeometryNode", "GeometryNodeGizmoTransform", rna_def_geo_gizmo_transform);
  define("GeometryNode", "GeometryNodeGuideGeometry");
  define("GeometryNode", "GeometryNodeGreasePencilColor");
  define("GeometryNode", "GeometryNodeGreasePencilDrawTime");
  define("GeometryNode", "GeometryNodeGreasePencilFillID");
  define("GeometryNode", "GeometryNodeGreasePencilOpacity");
  define("GeometryNode", "GeometryNodeGreasePencilStrokeSoftness");
  define("GeometryNode", "GeometryNodeGreasePencilStrokeVisibility");
  define("GeometryNode", "GeometryNodeGreasePencilToCurves");
  define("GeometryNode", "GeometryNodeGridAdvect");
  define("GeometryNode", "GeometryNodeGridCurl");
  define("GeometryNode", "GeometryNodeGridDeactivateVoxels");
  define("GeometryNode", "GeometryNodeGridDilateAndErode");
  define("GeometryNode", "GeometryNodeGridDivergence");
  define("GeometryNode", "GeometryNodeGridGradient");
  define("GeometryNode", "GeometryNodeGridInfo");
  define("GeometryNode", "GeometryNodeGridLaplacian");
  define("GeometryNode", "GeometryNodeGridMean");
  define("GeometryNode", "GeometryNodeGridMedian");
  define("GeometryNode", "GeometryNodeGridPrune");
  define("GeometryNode", "GeometryNodeGridClip");
  define("GeometryNode", "GeometryNodeGridSolvePoisson");
  define("GeometryNode", "GeometryNodeGridToMesh");
  define("GeometryNode", "GeometryNodeGridToPoints");
  define("GeometryNode", "GeometryNodeGridTopologyBoolean");
  define("GeometryNode", "GeometryNodeGridVoxelize");
  define("GeometryNode", "GeometryNodeImageInfo");
  define("GeometryNode", "GeometryNodeImageTexture", def_geo_image_texture);
  define("GeometryNode", "GeometryNodeImportCSV");
  define("GeometryNode", "GeometryNodeImportOBJ");
  define("GeometryNode", "GeometryNodeImportPLY");
  define("GeometryNode", "GeometryNodeImportSPZ");
  define("GeometryNode", "GeometryNodeImportSTL");
  define("GeometryNode", "GeometryNodeImportText");
  define("GeometryNode", "GeometryNodeImportVDB");
  define("GeometryNode", "GeometryNodeInstantMeshes");
  define("GeometryNode", "GeometryNodeQuadWild");
  define("GeometryNode", "GeometryNodeTriangleRemesh");
  define("GeometryNode", "GeometryNodeHeatGeodesic");
  define("GeometryNode", "GeometryNodeMeshGraphColoring");
  define("GeometryNode", "GeometryNodeTangentField");
  define("GeometryNode", "GeometryNodeIndexOfNearest");
  define("GeometryNode", "GeometryNodeCombineList", def_geo_combine_list);
  define("GeometryNode", "GeometryNodeIndexSwitch", def_geo_index_switch);
  define("GeometryNode", "GeometryNodeInputActiveCamera");
  define("GeometryNode", "GeometryNodeInputCollection", def_geo_input_collection);
  define("GeometryNode", "GeometryNodeInputCurveHandlePositions");
  define("GeometryNode", "GeometryNodeInputCurveTilt");
  define("GeometryNode", "GeometryNodeInputEdgeSmooth");
  define("GeometryNode", "GeometryNodeInputFont", def_geo_input_font);
  define("GeometryNode", "GeometryNodeInputID");
  define("GeometryNode", "GeometryNodeInputImage", def_geo_image);
  define("GeometryNode", "GeometryNodeInputIndex");
  define("GeometryNode", "GeometryNodeInputInstanceBounds");
  define("GeometryNode", "GeometryNodeInputInstanceReference");
  define("GeometryNode", "GeometryNodeInputInstanceRotation");
  define("GeometryNode", "GeometryNodeInputInstanceScale");
  define("GeometryNode", "GeometryNodeInputMaterial", def_geo_input_material);
  define("GeometryNode", "GeometryNodeInputMaterialIndex");
  define("GeometryNode", "GeometryNodeInputMeshEdgeAngle");
  define("GeometryNode", "GeometryNodeInputMeshEdgeNeighbors");
  define("GeometryNode", "GeometryNodeInputMeshEdgeRings");
  define("GeometryNode", "GeometryNodeInputMeshEdgeVertices");
  define("GeometryNode", "GeometryNodeInputMeshFaceArea");
  define("GeometryNode", "GeometryNodeInputMeshFaceIsPlanar");
  define("GeometryNode", "GeometryNodeInputMeshFaceNeighbors");
  define("GeometryNode", "GeometryNodeInputMeshIsland");
  define("GeometryNode", "GeometryNodeInputMeshVertexNeighbors");
  define("GeometryNode", "GeometryNodeInputNamedAttribute");
  define("GeometryNode", "GeometryNodeInputNamedLayerSelection");
  define("GeometryNode", "GeometryNodeInputNormal", def_geo_input_normal);
  define("GeometryNode", "GeometryNodeInputObject", def_geo_input_object);
  define("GeometryNode", "GeometryNodeInputPosition");
  define("GeometryNode", "GeometryNodeInputRadius");
  define("GeometryNode", "GeometryNodeInputSceneTime");
  define("GeometryNode", "GeometryNodeInputShadeSmooth");
  define("GeometryNode", "GeometryNodeInputShortestEdgePaths");
  define("GeometryNode", "GeometryNodeInputSplineCyclic");
  define("GeometryNode", "GeometryNodeInputSplineResolution");
  define("GeometryNode", "GeometryNodeInputTangent");
  /* Disabled with Viewport Camera node (see node_geo_viewport_camera.cc). Keep ID for source. */
  // define("GeometryNode", "GeometryNodeInputViewportCamera");
  define("GeometryNode", "GeometryNodeInputVoxelIndex");
  define("GeometryNode", "GeometryNodeInstanceOnPoints");
  define("GeometryNode", "GeometryNodeInstancesToPoints");
  define("GeometryNode", "GeometryNodeInstanceTransform");
  define("GeometryNode", "GeometryNodeInterpolateCurves");
  define("GeometryNode", "GeometryNodeIsViewport");
  define("GeometryNode", "GeometryNodeJoinGeometry");
  define("GeometryNode", "GeometryNodeKNearest");
  define("GeometryNode", "GeometryNodeListGetItem");
  define("GeometryNode", "GeometryNodeLinearSolver");
  define("GeometryNode", "GeometryNodeListLength");
  define("GeometryNode", "GeometryNodeLoopSubdivision");
  define("GeometryNode", "GeometryNodeGeometryMaterials");
  define("GeometryNode", "GeometryNodeMaterialSelection");
  define("GeometryNode", "GeometryNodeMenuSwitch", def_geo_menu_switch);
  define("GeometryNode", "GeometryNodeMergeByDistance");
  define("GeometryNode", "GeometryNodeMergeLayers", def_geo_merge_layers);
  define("GeometryNode", "GeometryNodeNamedPortal");
  define("GeometryNode", "GeometryNodeMergePoints");
  define("GeometryNode", "GeometryNodeMeshBevel");
  define("GeometryNode", "GeometryNodeMeshLaplacian");
  define("GeometryNode", "GeometryNodeMeshDivergence");
  define("GeometryNode", "GeometryNodeMeshGradient");
  define("GeometryNode", "GeometryNodeMeshBoolean");
  define("GeometryNode", "GeometryNodeMeshCircle");
  define("GeometryNode", "GeometryNodeMeshCone");
  define("GeometryNode", "GeometryNodeMeshCube");
  define("GeometryNode", "GeometryNodeMeshCylinder");
  define("GeometryNode", "GeometryNodeMeshFaceSetBoundaries");
  define("GeometryNode", "GeometryNodeMeshGrid");
  define("GeometryNode", "GeometryNodeMeshIcoSphere");
  define("GeometryNode", "GeometryNodeMeshLine");
  define("GeometryNode", "GeometryNodeMeshToCurve");
  define("GeometryNode", "GeometryNodeMeshToDensityGrid");
  define("GeometryNode", "GeometryNodeMeshToPoints");
  define("GeometryNode", "GeometryNodeMeshToSDFGrid");
  define("GeometryNode", "GeometryNodeMeshToVolume");
  define("GeometryNode", "GeometryNodeMeshUVSphere");
  define("GeometryNode", "GeometryNodeModalEvent", def_geo_modal_event);
  define("GeometryNode", "GeometryNodeModalTimer");
  define("GeometryNode", "GeometryNodeNURBSOrder");
  define("GeometryNode", "GeometryNodeNURBSWeight");
  define("GeometryNode", "GeometryNodeObjectInfo");
  define("GeometryNode", "GeometryNodeOffsetCornerInFace");
  define("GeometryNode", "GeometryNodeOffsetPointInCurve");
  define("GeometryNode", "GeometryNodePoints");
  define("GeometryNode", "GeometryNodePointsOfCurve");
  define("GeometryNode", "GeometryNodePointsSetType");
  define("GeometryNode", "GeometryNodePointsToCurves");
  define("GeometryNode", "GeometryNodePointsToSDFGrid");
  define("GeometryNode", "GeometryNodePointsToVertices");
  define("GeometryNode", "GeometryNodePointsToVolume");
  define("GeometryNode", "GeometryNodeProximity");
  define("GeometryNode", "GeometryNodeRasterizePoints", def_geo_rasterize_points);
  define("GeometryNode", "GeometryNodeRaycast");
  define("GeometryNode", "GeometryNodeRBFInterpolate");
  define("GeometryNode", "GeometryNodeRealizeInstances");
  define("GeometryNode", "GeometryNodeRemoveAttribute");
  define("GeometryNode", "GeometryNodeRenameAttribute");
  define("GeometryNode", "GeometryNodeRepeatInput", def_geo_repeat_input);
  define("GeometryNode", "GeometryNodeRepeatOutput", def_geo_repeat_output);
  define("GeometryNode", "GeometryNodeReplaceMaterial");
  define("GeometryNode", "GeometryNodeResampleCurve");
  define("GeometryNode", "GeometryNodeReverseCurve");
  define("GeometryNode", "GeometryNodeRotateInstances");
  define("GeometryNode", "GeometryNodeSampleCurve", def_geo_curve_sample);
  define("GeometryNode", "GeometryNodeSampleGrid");
  define("GeometryNode", "GeometryNodeSampleGridIndex");
  define("GeometryNode", "GeometryNodeSampleIndex", def_geo_sample_index);
  define("GeometryNode", "GeometryNodeSampleNearest");
  define("GeometryNode", "GeometryNodeSampleNearestSurface");
  define("GeometryNode", "GeometryNodeSampleSoundFrequencies");
  define("GeometryNode", "GeometryNodeSampleUVSurface");
  define("GeometryNode", "GeometryNodeScaleElements");
  define("GeometryNode", "GeometryNodeScaleInstances");
  define("GeometryNode", "GeometryNodeSDFGridBoolean");
  define("GeometryNode", "GeometryNodeSDFGridFillet");
  define("GeometryNode", "GeometryNodeSDFGridLaplacian");
  define("GeometryNode", "GeometryNodeSDFGridMean");
  define("GeometryNode", "GeometryNodeSDFGridMeanCurvature");
  define("GeometryNode", "GeometryNodeSDFGridMedian");
  define("GeometryNode", "GeometryNodeSDFGridOffset");
  define("GeometryNode", "GeometryNodeSelfObject");
  define("GeometryNode", "GeometryNodeSeparateComponents");
  define("GeometryNode", "GeometryNodeSeparateGeometry");
  define("GeometryNode", "GeometryNodeSetClosureDefault", def_set_closure_default);
  define("GeometryNode", "GeometryNodeSetGroupInputDefault");
  define("GeometryNode", "GeometryNodeSetCurveHandlePositions");
  define("GeometryNode", "GeometryNodeSetCurveNormal");
  define("GeometryNode", "GeometryNodeSetCurveRadius");
  define("GeometryNode", "GeometryNodeSetCurveTilt");
  define("GeometryNode", "GeometryNodeSetGeometryBundle");
  define("GeometryNode", "GeometryNodeSetGeometryName");
  define("GeometryNode", "GeometryNodeSetGreasePencilColor", def_geo_set_grease_pencil_color);
  define("GeometryNode", "GeometryNodeSetGreasePencilDepth", def_geo_set_grease_pencil_depth);
  define("GeometryNode", "GeometryNodeSetGreasePencilSoftness");
  define("GeometryNode", "GeometryNodeSetGridBackground");
  define("GeometryNode", "GeometryNodeSetGridTransform");
  define("GeometryNode", "GeometryNodeSetID");
  define("GeometryNode", "GeometryNodeSetInstanceTransform");
  define("GeometryNode", "GeometryNodeSetMaterial");
  define("GeometryNode", "GeometryNodeSetMaterialIndex");
  define("GeometryNode", "GeometryNodeSetMeshNormal");
  define("GeometryNode", "GeometryNodeSetNURBSOrder");
  define("GeometryNode", "GeometryNodeSetNURBSWeight");
  define("GeometryNode", "GeometryNodeSetPointRadius");
  define("GeometryNode", "GeometryNodeSetPosition");
  define("GeometryNode", "GeometryNodeSetShadeSmooth");
  define("GeometryNode", "GeometryNodeSetSplineCyclic");
  define("GeometryNode", "GeometryNodeSetSplineResolution");
  define("GeometryNode", "GeometryNodeSimulationInput", def_geo_simulation_input);
  define("GeometryNode", "GeometryNodeSimulationOutput", def_geo_simulation_output);
  define("GeometryNode", "GeometryNodeSortElements");
  define("GeometryNode", "GeometryNodeSortList");
  define("GeometryNode", "GeometryNodeSparseMatrixMath");
  define("GeometryNode", "GeometryNodeSplineLength");
  define("GeometryNode", "GeometryNodeSplineParameter");
  define("GeometryNode", "GeometryNodeSplitEdges");
  define("GeometryNode", "GeometryNodeSplitToInstances");
  define("GeometryNode", "GeometryNodeStoreNamedAttribute");
  define("GeometryNode", "GeometryNodeStoreNamedPortal");
  define("GeometryNode", "GeometryNodeStoreNamedGrid");
  define("GeometryNode", "GeometryNodeStringJoin");
  define("GeometryNode", "GeometryNodeStringToCurves");
  define("GeometryNode", "GeometryNodeSubdivideCurve");
  define("GeometryNode", "GeometryNodeSubdivideMesh");
  define("GeometryNode", "GeometryNodeSubdivisionSurface");
  define("GeometryNode", "GeometryNodeSwitch");
  define("GeometryNode", "GeometryNodeTagFilter");
  define("GeometryNode", "GeometryNodeTimeShift");
  define("GeometryNode", "GeometryNodeTool3DCursor");
  define("GeometryNode", "GeometryNodeToolActiveElement");
  define("GeometryNode", "GeometryNodeToolFaceSet");
  define("GeometryNode", "GeometryNodeToolMousePosition");
  define("GeometryNode", "GeometryNodeToolSelection");
  define("GeometryNode", "GeometryNodeToolSetFaceSet");
  define("GeometryNode", "GeometryNodeToolSetSelection");
  define("GeometryNode", "GeometryNodeTransform");
  define("GeometryNode", "GeometryNodeTranslateInstances");
  define("GeometryNode", "GeometryNodeTransferAttributes");
  define("GeometryNode", "GeometryNodeTriangulate");
  define("GeometryNode", "GeometryNodeTrimCurve");
  define("GeometryNode", "GeometryNodeUVPackIslands");
  define("GeometryNode", "GeometryNodeUVTangent");
  define("GeometryNode", "GeometryNodeUVUnwrap");
  define("GeometryNode", "GeometryNodeVertexOfCorner");
  define("GeometryNode", "GeometryNodeViewer", rna_def_geo_viewer);
  define("GeometryNode", "GeometryNodeViewportTransform");
  define("GeometryNode", "GeometryNodeVolumeCube");
  define("GeometryNode", "GeometryNodeVolumeToMesh");
  define("GeometryNode", "GeometryNodeWarning");
  define("GeometryNode", "GeometryNodeXPBDSolver");
  define("GeometryNode", "GeometryNodeVoronoiFracture");
  define("GeometryNode", "GeometryNodeBoxEngineSolver");
  define("GeometryNode", "GeometryNodeBoxEngineSetRigidBody");
  define("GeometryNode", "GeometryNodeBoxEngineSetConstraint");
  define("GeometryNode", "GeometryNodeJoltSolver");
  define("GeometryNode", "GeometryNodeJoltFollowAnimation");
  define("GeometryNode", "GeometryNodeJoltSetRigidBody");
  define("GeometryNode", "GeometryNodeJoltSetConstraint");
  define("GeometryNode", "GeometryNodeFlipSolver");
  define("GeometryNode", "GeometryNodeGroup", def_group);

  /* BIKINI: Image (GPU Texture Editor) and Object node types defined in this file. */
  define("ImageNode", "ImageNodeBakeImage", def_image_bake_image);
  define("ImageNode", "ImageNodeFluidSimInput", def_image_fluid_sim_input);
  define("ImageNode", "ImageNodeFluidSimOutput", def_image_fluid_sim_output);
  define("ImageNode", "ImageNodeImportPoints", def_image_import_points);
  define("ImageNode", "ImageNodeImportGeo", def_image_import_geo);

  /* clang-format on */
}

}  // namespace blender

#endif
