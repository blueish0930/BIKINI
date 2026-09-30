/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 *
 * Cordless named portals for Shader, Compositor, and GPU Texture (Image) editors.
 * Geometry Nodes keep their own Geometry-only implementation.
 */

#include "node_util.hh"

#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"

#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLT_translation.hh"

#include "MEM_guardedalloc.h"

#include <fmt/format.h>

#include "NOD_node_declaration.hh"
#include "NOD_node_extra_info.hh"
#include "NOD_register.hh"
#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"
#include "RNA_prototypes.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "node_composite_util.hh"
#include "node_shader_util.hh"

#include "GPU_material.hh"

namespace blender::nodes::node_named_portal_cc {

NODE_STORAGE_FUNCS(NodeGeometryPortal)

static const NodeGeometryPortal &portal_storage(const bNode &node)
{
  return *static_cast<const NodeGeometryPortal *>(node.storage);
}

static bool is_store_named_portal(const bNode &node)
{
  return bke::node_is_store_named_portal(node);
}

static bool is_named_portal(const bNode &node)
{
  return bke::node_is_named_portal(node);
}

static UString store_idname_for_tree(const bNodeTree &tree)
{
  switch (tree.type) {
    case NTREE_SHADER:
      return "ShaderNodeStoreNamedPortal"_ustr;
    case NTREE_COMPOSIT:
    case NTREE_IMAGE:
      return "CompositorNodeStoreNamedPortal"_ustr;
    default:
      return ""_ustr;
  }
}

static UString get_idname_for_tree(const bNodeTree &tree)
{
  switch (tree.type) {
    case NTREE_SHADER:
      return "ShaderNodeNamedPortal"_ustr;
    case NTREE_COMPOSIT:
    case NTREE_IMAGE:
      return "CompositorNodeNamedPortal"_ustr;
    default:
      return ""_ustr;
  }
}

static eNodeSocketDatatype default_data_type_for_tree(const bNodeTree &tree)
{
  switch (tree.type) {
    case NTREE_SHADER:
      return SOCK_FLOAT;
    case NTREE_COMPOSIT:
    case NTREE_IMAGE:
      return SOCK_RGBA;
    default:
      return SOCK_FLOAT;
  }
}

static int portal_name_count(const bNodeTree &tree, const StringRef name)
{
  if (name.is_empty()) {
    return 0;
  }
  int count = 0;
  for (const bNode *node : tree.all_nodes()) {
    if (!is_store_named_portal(*node) || !node->storage) {
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
      if (node == skip_node || !is_store_named_portal(*node) || !node->storage) {
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

static eNodeSocketDatatype portal_data_type(const bNode &node)
{
  if (!node.storage) {
    return SOCK_FLOAT;
  }
  return eNodeSocketDatatype(portal_storage(node).data_type);
}

static StructureType portal_structure_type(const bNodeTree *tree,
                                           const eNodeSocketDatatype data_type)
{
  if (!tree || !ELEM(tree->type, NTREE_COMPOSIT, NTREE_IMAGE)) {
    return StructureType::Single;
  }
  const bool is_single = compositor::Result::is_single_value_only_type(
      compositor::socket_data_type_to_result_type(data_type));
  return is_single ? StructureType::Single : StructureType::Dynamic;
}

static void node_declare_store(NodeDeclarationBuilder &b)
{
  const bNodeTree *tree = b.tree_or_null();
  const bNode *node = b.node_or_null();
  const eNodeSocketDatatype data_type = node ? portal_data_type(*node) : SOCK_FLOAT;
  const StructureType structure_type = portal_structure_type(tree, data_type);

  auto &input = b.add_input(data_type, "Value"_ustr);
  input.structure_type(structure_type);
  input.compositor_realization_mode(CompositorInputRealizationMode::None);

  auto &output = b.add_output(data_type, "Value"_ustr);
  output.structure_type(structure_type);
}

static void node_declare_get(NodeDeclarationBuilder &b)
{
  const bNodeTree *tree = b.tree_or_null();
  const bNode *node = b.node_or_null();
  const eNodeSocketDatatype data_type = node ? portal_data_type(*node) : SOCK_FLOAT;
  const StructureType structure_type = portal_structure_type(tree, data_type);

  /* Hidden input is filled by the cordless Store link (logical origin). Needed so
   * compositor scheduling, mapping, and GET preview actually receive the value. */
  auto &input = b.add_input(data_type, "Value"_ustr);
  input.structure_type(structure_type);
  input.compositor_realization_mode(CompositorInputRealizationMode::None);
  input.hide_value();

  auto &output = b.add_output(data_type, "Value"_ustr);
  output.structure_type(structure_type);
}

static void node_init_store(bNodeTree *tree, bNode *node)
{
  auto *storage = MEM_new<NodeGeometryPortal>(__func__);
  const std::string name = unique_portal_name(*tree, node, "Portal");
  STRNCPY_UTF8(storage->name, name.c_str());
  storage->data_type = int8_t(default_data_type_for_tree(*tree));
  node->storage = storage;
}

static void node_init_get(bNodeTree *tree, bNode *node)
{
  auto *storage = MEM_new<NodeGeometryPortal>(__func__);
  storage->data_type = int8_t(default_data_type_for_tree(*tree));
  node->storage = storage;
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
  layout.prop(ptr, "portal_name", UI_ITEM_NONE, "Portal", ICON_NONE);
}

static void node_update_store(bNodeTree *tree, bNode *node)
{
  if (!tree || !node || !node->storage) {
    return;
  }
  const StringRef name = portal_storage(*node).name;
  const int8_t dt = portal_storage(*node).data_type;
  for (bNode *other : tree->all_nodes()) {
    if (!is_named_portal(*other) || !other->storage) {
      continue;
    }
    if (portal_storage(*other).name != name) {
      continue;
    }
    auto &storage = *static_cast<NodeGeometryPortal *>(other->storage);
    if (storage.data_type == dt) {
      continue;
    }
    storage.data_type = dt;
    BKE_ntree_update_tag_node_property(tree, other);
  }
}

static void node_update_get(bNodeTree *tree, bNode *node)
{
  if (!tree || !node || !node->storage) {
    return;
  }
  if (!node->input_sockets().is_empty()) {
    node->input_socket(0).flag |= SOCK_HIDDEN;
  }
  const bNode *store = bke::node_find_unique_store_named_portal(*tree,
                                                                portal_storage(*node).name);
  if (!store || !store->storage) {
    return;
  }
  const int8_t dt = portal_storage(*store).data_type;
  auto &storage = *static_cast<NodeGeometryPortal *>(node->storage);
  if (storage.data_type != dt) {
    storage.data_type = dt;
  }
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

static const bNode *find_matching_store(const bNodeTree &tree, const StringRef name)
{
  return bke::node_find_unique_store_named_portal(tree, name);
}

static void named_portal_extra_info(NodeExtraInfoParams &params)
{
  const StringRef name = portal_storage(params.node).name;
  const bNode *store = find_matching_store(params.tree, name);
  if (!store) {
    NodeExtraInfoRow row;
    row.text = RPT_("Invalid Portal");
    row.icon = ICON_STATUS_ERROR;
    row.tooltip = TIP_(
        "No unique Store Named Portal with this name exists in the current node tree");
    params.rows.append(std::move(row));
    return;
  }
  if (portal_data_type(*store) != portal_data_type(params.node)) {
    NodeExtraInfoRow row;
    row.text = RPT_("Portal Type Mismatch");
    row.icon = ICON_STATUS_ERROR;
    row.tooltip = TIP_(
        "Named Portal data type must match the Store Named Portal with this name");
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
    if (!is_store_named_portal(*portal) || !portal->storage) {
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

static const EnumPropertyItem *data_type_items_callback(bContext * /*C*/,
                                                        PointerRNA *ptr,
                                                        PropertyRNA * /*prop*/,
                                                        bool *r_free)
{
  *r_free = true;
  const bNodeTree &ntree = *reinterpret_cast<bNodeTree *>(ptr->owner_id);
  bke::bNodeTreeType *ntree_type = ntree.typeinfo;
  return enum_items_filter(
      rna_enum_node_socket_data_type_items, [&](const EnumPropertyItem &item) -> bool {
        bke::bNodeSocketType *socket_type = bke::node_socket_type_find_static(item.value);
        return ntree_type->valid_socket_type &&
               ntree_type->valid_socket_type(ntree_type, socket_type);
      });
}

static void node_rna(StructRNA *srna, const bool with_search)
{
  RNA_def_node_enum(srna,
                    "data_type",
                    "Data Type",
                    "Type of data passed through this portal",
                    rna_enum_node_socket_data_type_items,
                    NOD_storage_enum_accessors(data_type),
                    SOCK_FLOAT,
                    data_type_items_callback);

  PropertyRNA *prop = RNA_def_property(srna, "portal_name", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Portal", "Name of the Store Named Portal in this node tree");
  RNA_def_property_string_maxlength(prop, MAX_NAME);
  RNA_def_property_string_funcs_runtime(
      prop, portal_name_get, portal_name_length, portal_name_set, nullptr, nullptr);
  if (with_search) {
    RNA_def_property_string_search_func_runtime(prop, portal_name_search, PROP_STRING_SEARCH_SORT);
  }
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update_runtime(prop, rna_Node_socket_update);
  RNA_def_property_update_notifier(prop, NC_NODE | NA_EDITED);
}

static void node_gather_link_searches_store(GatherLinkSearchOpParams &params)
{
  const UString idname = store_idname_for_tree(params.node_tree());
  if (idname.is_empty()) {
    return;
  }
  bke::bNodeTreeType &tree_type = *params.node_tree().typeinfo;
  bke::bNodeSocketType *socket_type = bke::node_socket_type_find_static(params.other_socket().type);
  if (!socket_type || !tree_type.valid_socket_type ||
      !tree_type.valid_socket_type(&tree_type, socket_type))
  {
    return;
  }
  params.add_item(IFACE_("Value"), [idname](LinkSearchOpParams &params) {
    bNode &node = params.add_node(idname);
    node_storage(node).data_type = int8_t(params.socket.type);
    params.update_and_connect_available_socket(node, "Value"_ustr);
  });
}

static void node_gather_link_searches_get(GatherLinkSearchOpParams &params)
{
  if (params.in_out() != SOCK_OUT) {
    return;
  }
  const UString idname = get_idname_for_tree(params.node_tree());
  if (idname.is_empty()) {
    return;
  }
  bke::bNodeTreeType &tree_type = *params.node_tree().typeinfo;
  bke::bNodeSocketType *socket_type = bke::node_socket_type_find_static(params.other_socket().type);
  if (!socket_type || !tree_type.valid_socket_type ||
      !tree_type.valid_socket_type(&tree_type, socket_type))
  {
    return;
  }
  params.add_item(IFACE_("Value"), [idname](LinkSearchOpParams &params) {
    bNode &node = params.add_node(idname);
    node_storage(node).data_type = int8_t(params.socket.type);
    params.update_and_connect_available_socket(node, "Value"_ustr);
  });
}

static const bNodeSocket *node_internally_linked_input(const bNodeTree & /*tree*/,
                                                       const bNode &node,
                                                       const bNodeSocket & /*output_socket*/)
{
  if (node.input_sockets().is_empty()) {
    return nullptr;
  }
  return &node.input_socket(0);
}

static int node_gpu_store(GPUMaterial * /*mat*/,
                          bNode * /*node*/,
                          bNodeExecData * /*execdata*/,
                          GPUNodeStack *in,
                          GPUNodeStack *out)
{
  if (in[0].link) {
    out[0].link = in[0].link;
  }
  return true;
}

static int node_gpu_get(GPUMaterial * /*mat*/,
                        bNode * /*node*/,
                        bNodeExecData * /*execdata*/,
                        GPUNodeStack *in,
                        GPUNodeStack *out)
{
  /* Physical hidden input is usually unlinked. Shader stack_index sharing
   * aliases GET output with Store; copy only if a real GPU link exists. */
  if (in[0].link) {
    out[0].link = in[0].link;
  }
  return true;
}

using namespace blender::compositor;

class StoreNamedPortalOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    Result &output = this->get_result("Value");
    const Result &input = this->get_input("Value");
    output.share_data(input);
  }
};

static NodeOperation *get_compositor_operation_store(Context &context, const bNode &node)
{
  return new StoreNamedPortalOperation(context, node);
}

static void finish_store_named_portal(bke::bNodeType &ntype)
{
  ntype.ui_name = "Store Named Portal";
  ntype.ui_description = "Publish a value under a unique name in the current node tree";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare_store;
  ntype.initfunc = node_init_store;
  ntype.updatefunc = node_update_store;
  ntype.draw_buttons = node_layout;
  ntype.get_extra_info = store_named_portal_extra_info;
  ntype.gather_link_search_ops = node_gather_link_searches_store;
  ntype.internally_linked_input = node_internally_linked_input;
  ntype.gpu_fn = node_gpu_store;
  ntype.get_compositor_operation = get_compositor_operation_store;
  bke::node_type_storage(
      ntype, "NodeGeometryPortal", node_free_standard_storage, node_copy_standard_storage);
  bke::node_register_type(ntype);
  node_rna(ntype.rna_ext.srna, false);
}

static void finish_named_portal(bke::bNodeType &ntype)
{
  ntype.ui_name = "Named Portal";
  ntype.ui_description = "Read a value from a Store Named Portal in the current node tree";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare_get;
  ntype.initfunc = node_init_get;
  ntype.updatefunc = node_update_get;
  ntype.draw_buttons = node_layout;
  ntype.get_extra_info = named_portal_extra_info;
  ntype.gather_link_search_ops = node_gather_link_searches_get;
  ntype.internally_linked_input = node_internally_linked_input;
  ntype.gpu_fn = node_gpu_get;
  ntype.get_compositor_operation = get_compositor_operation_store;
  bke::node_type_storage(
      ntype, "NodeGeometryPortal", node_free_standard_storage, node_copy_standard_storage);
  bke::node_register_type(ntype);
  node_rna(ntype.rna_ext.srna, true);
}

}  // namespace blender::nodes::node_named_portal_cc

namespace blender {

void register_named_portal_nodes()
{
  using namespace nodes::node_named_portal_cc;

  static bke::bNodeType shader_store;
  static bke::bNodeType shader_get;
  static bke::bNodeType compositor_store;
  static bke::bNodeType compositor_get;

  sh_node_type_base(&shader_store, "ShaderNodeStoreNamedPortal"_ustr, SH_NODE_STORE_NAMED_PORTAL);
  finish_store_named_portal(shader_store);
  sh_node_type_base(&shader_get, "ShaderNodeNamedPortal"_ustr, SH_NODE_NAMED_PORTAL);
  finish_named_portal(shader_get);

  cmp_node_type_base(
      &compositor_store, "CompositorNodeStoreNamedPortal"_ustr, CMP_NODE_STORE_NAMED_PORTAL);
  finish_store_named_portal(compositor_store);
  cmp_node_type_base(&compositor_get, "CompositorNodeNamedPortal"_ustr, CMP_NODE_NAMED_PORTAL);
  finish_named_portal(compositor_get);
}

}  // namespace blender
