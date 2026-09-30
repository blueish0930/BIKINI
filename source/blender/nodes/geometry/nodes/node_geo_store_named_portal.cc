/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_node_tree_update.hh"

#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_listbase.hh"

#include "MEM_guardedalloc.h"

#include <fmt/format.h>

#include "NOD_node_extra_info.hh"
#include "NOD_rna_define.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_types.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_geo_store_named_portal_cc {

NODE_STORAGE_FUNCS(NodeGeometryPortal)

static const NodeGeometryPortal &portal_storage(const bNode &node)
{
  return *static_cast<const NodeGeometryPortal *>(node.storage);
}

static bool is_store_named_portal(const bNode &node)
{
  return node.is_type("GeometryNodeStoreNamedPortal"_ustr);
}

static bool is_named_portal(const bNode &node)
{
  return node.is_type("GeometryNodeNamedPortal"_ustr);
}

static int portal_name_count(const bNodeTree &tree, const StringRef name)
{
  if (name.is_empty()) {
    return 0;
  }
  int count = 0;
  for (const bNode *node : tree.all_nodes()) {
    if (!is_store_named_portal(*node)) {
      continue;
    }
    if (name == portal_storage(*node).name) {
      count++;
    }
  }
  return count;
}

static bool portal_name_is_valid(const bNodeTree &tree, const StringRef name)
{
  return portal_name_count(tree, name) == 1;
}

static std::string unique_portal_name(const bNodeTree &tree,
                                      const bNode *skip_node,
                                      const StringRef desired_name)
{
  const std::string base = desired_name.is_empty() ? "Portal" : desired_name;
  auto is_available = [&](const StringRef candidate) {
    for (const bNode *node : tree.all_nodes()) {
      if (node == skip_node || !is_store_named_portal(*node)) {
        continue;
      }
      if (candidate == portal_storage(*node).name) {
        return false;
      }
    }
    return true;
  };
  if (is_available(base)) {
    return base;
  }
  for (int suffix = 1; suffix < 1000000; suffix++) {
    const std::string candidate = fmt::format("{}.{:03}", base, suffix);
    if (is_available(candidate)) {
      return candidate;
    }
  }
  return base;
}

static void node_declare_store_named_portal(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry"_ustr)
      .structure_type(StructureType::Single)
      .description("Geometry stored under this portal name");
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .structure_type(StructureType::Single)
      .propagate_all({0})
      .description("Same geometry passed through");
}

static void node_declare_named_portal(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .structure_type(StructureType::Single)
      .description("Geometry from the matching Store Named Portal");
}

static void node_init_store_named_portal(bNodeTree *tree, bNode *node)
{
  auto *storage = MEM_new<NodeGeometryPortal>(__func__);
  const std::string name = unique_portal_name(*tree, node, "Portal");
  STRNCPY_UTF8(storage->name, name.c_str());
  node->storage = storage;
}

static void node_init_named_portal(bNodeTree * /*tree*/, bNode *node)
{
  node->storage = MEM_new<NodeGeometryPortal>(__func__);
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "portal_name", UI_ITEM_NONE, "Portal", ICON_NONE);
}

static void store_named_portal_exec(GeoNodeExecParams params)
{
  params.set_output("Geometry"_ustr, params.extract_input<GeometrySet>("Geometry"_ustr));
}

static void named_portal_fallback_exec(GeoNodeExecParams params)
{
  params.set_output("Geometry"_ustr, GeometrySet());
}

static void store_named_portal_extra_info(NodeExtraInfoParams &params)
{
  const StringRef name = portal_storage(params.node).name;
  if (name.is_empty()) {
    NodeExtraInfoRow row;
    row.text = RPT_("Portal Name is Empty");
    row.icon = ICON_STATUS_ERROR;
    row.tooltip = TIP_("A Store Named Portal must have a non-empty unique name");
    params.rows.append(std::move(row));
    return;
  }
  if (portal_name_count(params.tree, name) > 1) {
    NodeExtraInfoRow row;
    row.text = RPT_("Duplicate Portal Name");
    row.icon = ICON_STATUS_ERROR;
    row.tooltip = TIP_("Store Named Portal names must be unique in this node tree");
    params.rows.append(std::move(row));
  }
}

static void named_portal_extra_info(NodeExtraInfoParams &params)
{
  const StringRef name = portal_storage(params.node).name;
  if (!portal_name_is_valid(params.tree, name)) {
    NodeExtraInfoRow row;
    row.text = RPT_("Invalid Portal");
    row.icon = ICON_STATUS_ERROR;
    row.tooltip = TIP_(
        "No unique Store Named Portal with this name exists in the current node tree");
    params.rows.append(std::move(row));
  }
}

static std::string portal_name_get(PointerRNA *ptr, PropertyRNA * /*prop*/)
{
  const bNode &node = *static_cast<const bNode *>(ptr->data);
  return portal_storage(node).name;
}

static int portal_name_length(PointerRNA *ptr, PropertyRNA * /*prop*/)
{
  const bNode &node = *static_cast<const bNode *>(ptr->data);
  return strlen(portal_storage(node).name);
}

static void portal_name_set(PointerRNA *ptr, PropertyRNA * /*prop*/, const std::string &value)
{
  bNode &node = *static_cast<bNode *>(ptr->data);
  bNodeTree *tree = reinterpret_cast<bNodeTree *>(ptr->owner_id);
  NodeGeometryPortal &storage = *static_cast<NodeGeometryPortal *>(node.storage);

  if (is_store_named_portal(node) && tree) {
    const std::string unique_name = unique_portal_name(*tree, &node, value);
    STRNCPY_UTF8(storage.name, unique_name.c_str());
  }
  else {
    STRNCPY_UTF8(storage.name, value.c_str());
  }
  if (tree) {
    BKE_ntree_update_tag_node_property(tree, &node);
  }
}

static void portal_name_search(const bContext * /*C*/,
                               PointerRNA *ptr,
                               PropertyRNA * /*prop*/,
                               const char * /*edit_text*/,
                               const FunctionRef<void(StringPropertySearchVisitParams)> visit_fn)
{
  const bNode &node = *static_cast<const bNode *>(ptr->data);
  const bNodeTree *tree = reinterpret_cast<const bNodeTree *>(ptr->owner_id);
  if (!tree || !is_named_portal(node)) {
    return;
  }
  for (const bNode *portal : tree->all_nodes()) {
    if (!is_store_named_portal(*portal)) {
      continue;
    }
    const StringRef name = portal_storage(*portal).name;
    if (!portal_name_is_valid(*tree, name)) {
      continue;
    }
    StringPropertySearchVisitParams item;
    item.text = name;
    item.info = TIP_("Store Named Portal in the current node tree");
    visit_fn(std::move(item));
  }
}

static void node_rna(StructRNA *srna, const bool with_search)
{
  PropertyRNA *prop = RNA_def_property(srna, "portal_name", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Portal", "Name of the Store Named Portal in this node tree");
  RNA_def_property_string_maxlength(prop, MAX_NAME);
  RNA_def_property_string_funcs_runtime(
      prop, portal_name_get, portal_name_length, portal_name_set, nullptr, nullptr);
  if (with_search) {
    /* Do not use PROP_STRING_SEARCH_SUGGESTION: RNA string-search items have null
     * #CollItemSearch::data pointers, and #searchbox_apply refuses to apply suggestion
     * rows without a pointer (and also skips writing the name into the edit string).
     * Sort-only search still lists Store Named Portal names and applies the chosen text. */
    RNA_def_property_string_search_func_runtime(prop, portal_name_search, PROP_STRING_SEARCH_SORT);
  }
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update_runtime(prop, rna_Node_socket_update);
  RNA_def_property_update_notifier(prop, NC_NODE | NA_EDITED);
}

static void register_store_named_portal()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeStoreNamedPortal"_ustr, GEO_NODE_STORE_NAMED_PORTAL);
  ntype.ui_name = "Store Named Portal";
  ntype.ui_description = "Publish geometry under a unique name in the current node tree";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare_store_named_portal;
  ntype.initfunc = node_init_store_named_portal;
  ntype.draw_buttons = node_layout;
  ntype.geometry_node_execute = store_named_portal_exec;
  ntype.get_extra_info = store_named_portal_extra_info;
  bke::node_type_storage(ntype, "NodeGeometryPortal", node_free_standard_storage,
                         node_copy_standard_storage);
  bke::node_register_type(ntype);
  node_rna(ntype.rna_ext.srna, false);
}
NOD_REGISTER_NODE(register_store_named_portal)

static void register_named_portal()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeNamedPortal"_ustr, GEO_NODE_NAMED_PORTAL);
  ntype.ui_name = "Named Portal";
  ntype.ui_description =
      "Read geometry from a Store Named Portal in the current node tree";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare_named_portal;
  ntype.initfunc = node_init_named_portal;
  ntype.draw_buttons = node_layout;
  ntype.geometry_node_execute = named_portal_fallback_exec;
  ntype.get_extra_info = named_portal_extra_info;
  bke::node_type_storage(ntype, "NodeGeometryPortal", node_free_standard_storage,
                         node_copy_standard_storage);
  bke::node_register_type(ntype);
  node_rna(ntype.rna_ext.srna, true);
}
NOD_REGISTER_NODE(register_named_portal)

}  // namespace blender::nodes::node_geo_store_named_portal_cc
