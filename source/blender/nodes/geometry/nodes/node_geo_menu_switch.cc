/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"

#include "node_geometry_util.hh"
#include "shader/node_shader_util.hh"

#include "DNA_node_types.h"

#include "BKE_node_enum.hh"
#include "BKE_node_tree_interface.hh"

#include "FN_multi_function.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "NOD_geo_menu_switch.hh"
#include "NOD_geometry_nodes_list.hh"
#include "NOD_rna_define.hh"
#include "NOD_socket.hh"
#include "NOD_socket_items_blend.hh"
#include "NOD_socket_items_ops.hh"
#include "NOD_socket_items_ui.hh"
#include "NOD_socket_search_link.hh"

#include "BLO_read_write.hh"

#include "RNA_enum_types.hh"
#include "RNA_prototypes.hh"

#include "BKE_node_tree_reference_lifetimes.hh"
#include "BKE_screen.hh"

#include "WM_api.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

namespace blender {

namespace nodes::node_geo_menu_switch_cc {

NODE_STORAGE_FUNCS(NodeMenuSwitch)

/**
 * Detect multi-selection (COW-safe: DNA ListBase only, no topology Spans / dynamic_cast).
 *
 * When multi is on → declare forces Output to StructureType::List and execute packs a GList.
 * Sources: Menu DNA multi flag, linked Menu DNA multi, group-interface multi on Group Input link,
 * or prior Output already declared as List.
 */
static bool menu_value_is_multi(const bNodeSocket &socket)
{
  if (socket.type != SOCK_MENU || socket.default_value == nullptr) {
    return false;
  }
  return (static_cast<const bNodeSocketValueMenu *>(socket.default_value)->runtime_flag &
          bke::NODE_MENU_MULTI_SELECTION) != 0;
}

static bool menu_interface_input_is_multi(const bNodeTree &ntree, const char *identifier)
{
  if (identifier == nullptr || identifier[0] == '\0') {
    return false;
  }
  if (ntree.tree_interface.items_cache_is_available()) {
    const int index = ntree.interface_input_index_by_identifier(identifier);
    if (index >= 0) {
      const Span<const bNodeTreeInterfaceSocket *> iface_inputs = ntree.interface_inputs();
      if (index < iface_inputs.size() && iface_inputs[index] &&
          (iface_inputs[index]->flag & NODE_INTERFACE_SOCKET_MENU_MULTI_SELECTION))
      {
        return true;
      }
    }
  }
  bool found = false;
  ntree.tree_interface.foreach_item([&](const bNodeTreeInterfaceItem &item) {
    if (item.item_type != NodeTreeInterfaceItemType::Socket) {
      return true;
    }
    const auto &iosock = bke::node_interface::get_item_as<bNodeTreeInterfaceSocket>(item);
    if (!(iosock.flag & NODE_INTERFACE_SOCKET_INPUT)) {
      return true;
    }
    if (iosock.identifier && STREQ(iosock.identifier, identifier) &&
        (iosock.flag & NODE_INTERFACE_SOCKET_MENU_MULTI_SELECTION))
    {
      found = true;
      return false;
    }
    return true;
  });
  return found;
}

/**
 * Shared by declare + execute: multi on → List shape and list data.
 *
 * When Menu is linked from Group Input, the group interface multi flag is authoritative —
 * do not trust this node's Menu DNA multi bit alone (it is set by the previous declare and
 * would sticky-keep multi after the interface multi is turned off).
 */
static bool menu_switch_is_multi_selection(const bNodeTree *ntree, const bNode &node)
{
  for (const bNodeSocket *input = static_cast<const bNodeSocket *>(node.inputs.first()); input;
       input = input->next)
  {
    /* Item sockets are also menus when the data type is Menu. Only the switch input decides. */
    if (input->type != SOCK_MENU || !STREQ(input->identifier, "Menu")) {
      continue;
    }
    if (input->link && input->link->fromsock) {
      const bNodeSocket &fromsock = *input->link->fromsock;
      if (ntree != nullptr && input->link->fromnode && input->link->fromnode->is_group_input()) {
        /* Interface multi is the source of truth for this link. */
        if (menu_interface_input_is_multi(*ntree, fromsock.identifier)) {
          return true;
        }
        continue;
      }
      if (menu_value_is_multi(fromsock)) {
        return true;
      }
      continue;
    }
    /* Unlinked Menu: use this socket's DNA multi flag. */
    if (menu_value_is_multi(*input)) {
      return true;
    }
  }
  return false;
}

static void node_declare(nodes::NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  const bNodeTree *ntree = b.tree_or_null();
  const bNode *node = b.node_or_null();
  if (node == nullptr) {
    return;
  }
  /* Storage may not exist yet on some copy/declare paths. */
  if (node->storage == nullptr) {
    return;
  }
  const NodeMenuSwitch &storage = node_storage(*node);
  const eNodeSocketDatatype data_type = storage.data_type;

  StructureType value_structure_type = StructureType::Dynamic;
  StructureType menu_structure_type = value_structure_type;

  /* ntree is null during some COW / early declare paths — never dereference it. */
  if (ntree != nullptr && ntree->type == NTREE_COMPOSIT) {
    const bool is_single_compositor_type = compositor::Result::is_single_value_only_type(
        compositor::socket_data_type_to_result_type(data_type));
    if (is_single_compositor_type) {
      value_structure_type = StructureType::Single;
    }
    menu_structure_type = StructureType::Single;
  }

  /* Multi on → Output is always List (shape + execute pack). Multi off → normal path. */
  const bool menu_is_multi_selection = menu_switch_is_multi_selection(ntree, *node);

  if (menu_is_multi_selection) {
    /* Fixed List — do not use inferred_structure_type() (resets to Dynamic / circle shape). */
    b.add_output(data_type, "Output"_ustr)
        .propagate_all()
        .structure_type(StructureType::List);
  }
  else {
    b.add_output(data_type, "Output"_ustr)
        .propagate_all()
        .inferred_structure_type()
        .structure_type(value_structure_type);
  }

  b.add_default_layout();

  auto &menu = b.add_input<decl::Menu>("Menu"_ustr);
  menu.default_value(MenuValue(storage.enum_definition.items().is_empty() ?
                                   0 :
                                   storage.enum_definition.items().first().identifier));
  /* Multi packs a single MenuValue (bit-field); keep non-multi Dynamic so field menus still work.
   * Always mark multi on the Menu declaration when multi is active so DNA runtime_flag and
   * MenuValue.multi_selection stay set even if only one item is selected. */
  menu.multi_selection(menu_is_multi_selection);
  menu.structure_type(menu_is_multi_selection ? StructureType::Single : menu_structure_type);
  menu.optional_label();

  const Span<NodeEnumItem> enum_items = storage.enum_definition.items();
  for (const int item_i : enum_items.index_range()) {
    const NodeEnumItem &enum_item = enum_items[item_i];
    const UString name(enum_item.name);
    const UString identifier(MenuSwitchItemsAccessor::socket_identifier_for_item(enum_item));
    auto &input = b.add_input(data_type, name, identifier)
                      .compositor_realization_mode(CompositorInputRealizationMode::None)
                      .description("Becomes the output value if it is chosen by the menu input");
    /* socket_name_ptr needs a valid tree ID; skip during COW when ntree is unavailable. */
    if (ntree != nullptr) {
      input.socket_name_ptr(
          &ntree->id, *MenuSwitchItemsAccessor::item_srna, &enum_item, "name");
    }
    /* Labels are ugly in combination with data-block pickers and are usually disabled. */
    input.optional_label(ELEM(data_type,
                              SOCK_OBJECT,
                              SOCK_IMAGE,
                              SOCK_COLLECTION,
                              SOCK_MATERIAL,
                              SOCK_FONT,
                              SOCK_SCENE,
                              SOCK_TEXT_ID,
                              SOCK_MASK,
                              SOCK_SOUND));
    /* Multi: item values are singles packed into a List output — never List-shaped inputs. */
    input.structure_type(menu_is_multi_selection ? StructureType::Dynamic : value_structure_type);
    /* Per-item − button for one-click delete (RCS: Qj5q). */
    input.custom_draw([item_i](CustomSocketDrawParams &params) {
      socket_items::ui::draw_item_socket_with_remove<MenuSwitchItemsAccessor>(params, item_i);
    });
    auto &bool_out = b.add_output<decl::Bool>(name, identifier)
                         .align_with_previous()
                         .description("True if this item is chosen by the menu input")
                         .propagate_references({menu.index()});
    if (menu_is_multi_selection) {
      /* Fixed single bools — do not use inferred_structure_type (it resets type to Dynamic). */
      bool_out.structure_type(StructureType::Single);
    }
    else {
      bool_out.inferred_structure_type({menu.index()});
    }
  }

  b.add_input<decl::Extend>(""_ustr, "__extend__"_ustr)
      .structure_type(StructureType::Dynamic)
      .custom_draw(socket_items::ui::draw_extend_socket_fn<MenuSwitchItemsAccessor>());
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree *tree, bNode *node)
{
  NodeMenuSwitch *data = MEM_new<NodeMenuSwitch>(__func__);
  data->data_type = tree->type == NTREE_GEOMETRY ? SOCK_GEOMETRY : SOCK_RGBA;
  data->enum_definition.next_identifier = 0;
  data->enum_definition.items_array = nullptr;
  data->enum_definition.items_num = 0;
  node->storage = data;

  socket_items::add_item_with_name<MenuSwitchItemsAccessor>(*node, "A");
  socket_items::add_item_with_name<MenuSwitchItemsAccessor>(*node, "B");
}

static void node_free_storage(bNode *node)
{
  socket_items::destruct_array<MenuSwitchItemsAccessor>(*node);
  MEM_delete(reinterpret_cast<NodeMenuSwitch *>(node->storage));
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeMenuSwitch &src_storage = node_storage(*src_node);
  NodeMenuSwitch *dst_storage = MEM_new<NodeMenuSwitch>(__func__, dna::shallow_copy(src_storage));
  dst_node->storage = dst_storage;

  socket_items::copy_array<MenuSwitchItemsAccessor>(*src_node, *dst_node);
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const eNodeSocketDatatype data_type = params.other_socket().type;
  if (params.in_out() == SOCK_IN) {
    if (data_type == SOCK_MENU) {
      params.add_item(IFACE_("Menu"), [](LinkSearchOpParams &params) {
        bNode &node = params.add_node("GeometryNodeMenuSwitch"_ustr);
        params.update_and_connect_available_socket(node, "Menu"_ustr);
      });
    }
  }
  else {
    if (data_type != SOCK_MENU) {
      params.add_item(IFACE_("Output"), [](LinkSearchOpParams &params) {
        bNode &node = params.add_node("GeometryNodeMenuSwitch"_ustr);
        node_storage(node).data_type = params.socket.type;
        params.update_and_connect_available_socket(node, "Output"_ustr);
      });
    }
  }
}

/**
 * Multi-function which evaluates the switch input for each enum item and partially fills the
 * output array with values from the input array where the identifier matches.
 */
class MenuSwitchFn : public mf::MultiFunction {
  const NodeEnumDefinition &enum_def_;
  const CPPType &type_;
  mf::Signature signature_;

 public:
  MenuSwitchFn(const NodeEnumDefinition &enum_def, const CPPType &type)
      : enum_def_(enum_def), type_(type)
  {
    mf::SignatureBuilder builder{"Menu Switch", signature_};
    builder.single_input<MenuValue>("Menu");
    for (const NodeEnumItem &enum_item : enum_def.items()) {
      builder.single_input(enum_item.name, type);
    }
    builder.single_output("Output", type, mf::ParamFlag::SupportsUnusedOutput);
    this->set_signature(&signature_);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const int value_inputs_start = 1;
    const int inputs_num = enum_def_.items_num;
    const VArray<MenuValue> values = params.readonly_single_input<MenuValue>(0, "Menu");
    /* Use one extra mask at the end for invalid indices. */
    const int invalid_index = inputs_num;

    GMutableSpan value_output = params.uninitialized_single_output_if_required(1 + inputs_num,
                                                                               "Output");

    auto find_item_index = [&](const MenuValue value) -> int {
      const MenuValue menu = value.normalized(false, enum_def_.items_num, [&](const int i) {
        return enum_def_.items_array[i].identifier;
      });
      for (const int i : enum_def_.items().index_range()) {
        const NodeEnumItem &item = enum_def_.items()[i];
        if (menu.item_is_selected(item.identifier)) {
          return i;
        }
      }
      return invalid_index;
    };

    if (const std::optional<MenuValue> value = values.get_if_single()) {
      const int index = find_item_index(*value);
      if (index < inputs_num) {
        if (!value_output.is_empty()) {
          const GVArray inputs = params.readonly_single_input(value_inputs_start + index);
          inputs.materialize_to_uninitialized(mask, value_output.data());
        }
      }
      else {
        if (!value_output.is_empty()) {
          type_.fill_construct_indices(type_.default_value(), value_output.data(), mask);
        }
      }
      return;
    }

    IndexMaskMemory memory;
    Array<IndexMask> masks(inputs_num + 1);
    IndexMask::from_groups<int64_t>(
        mask, memory, [&](const int64_t i) { return find_item_index(values[i]); }, masks);

    for (const int item_i : IndexRange(inputs_num)) {
      const IndexMask &mask_for_index = masks[item_i];
      if (!mask_for_index.is_empty() && !value_output.is_empty()) {
        const GVArray inputs = params.readonly_single_input(value_inputs_start + item_i);
        inputs.materialize_to_uninitialized(mask_for_index, value_output.data());
      }
    }

    type_.fill_construct_indices(type_.default_value(), value_output.data(), masks[invalid_index]);
  }

  void hash_unique(UniqueHashBytes &hash) const override
  {
    static constexpr int8_t id = 0;
    hash.add(&id);
    hash.add(&type_);
    hash.add(&enum_def_);
  }
};

class MenuSwitchBoolsFn : public mf::MultiFunction {
 private:
  const NodeEnumDefinition &enum_def_;
  mf::Signature signature_;

 public:
  MenuSwitchBoolsFn(const NodeEnumDefinition &enum_def) : enum_def_(enum_def)
  {
    mf::SignatureBuilder builder{"Menu Switch Bools", signature_};
    builder.single_input<MenuValue>("Menu");
    for (const NodeEnumItem &enum_item : enum_def.items()) {
      builder.single_output(
          enum_item.name, CPPType::get<bool>(), mf::ParamFlag::SupportsUnusedOutput);
    }
    this->set_signature(&signature_);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<MenuValue> values = params.readonly_single_input<MenuValue>(0, "Menu");

    Array<MutableSpan<bool>> item_spans(enum_def_.items_num);
    Vector<int> used_output_items;
    for (const int item_i : IndexRange(enum_def_.items_num)) {
      item_spans[item_i] = params.uninitialized_single_output_if_required<bool>(1 + item_i);
      if (!item_spans[item_i].is_empty()) {
        used_output_items.append(item_i);
      }
    }

    auto ensure_multi = [&](const MenuValue value) -> MenuValue {
      return value.normalized(false, enum_def_.items_num, [&](const int i) {
        return enum_def_.items_array[i].identifier;
      });
    };

    if (const std::optional<MenuValue> value = values.get_if_single()) {
      const MenuValue menu = ensure_multi(*value);
      for (const int item_i : used_output_items) {
        const NodeEnumItem &enum_item = enum_def_.items()[item_i];
        const bool is_selected = menu.item_is_selected(enum_item.identifier);
        index_mask::masked_fill(item_spans[item_i], is_selected, mask);
      }
      return;
    }

    mask.foreach_index([&](const int i) {
      const MenuValue menu = ensure_multi(values[i]);
      for (const int item_i : used_output_items) {
        const NodeEnumItem &enum_item = enum_def_.items_array[item_i];
        const bool is_selected = menu.item_is_selected(enum_item.identifier);
        item_spans[item_i][i] = is_selected;
      }
    });
  }

  void hash_unique(UniqueHashBytes &hash) const override
  {
    static constexpr int8_t id = 0;
    hash.add(&id);
    hash.add(&enum_def_);
  }
};

class LazyFunctionForMenuSwitchNode : public LazyFunction {
 private:
  const bNode &node_;
  bool can_be_field_ = false;
  /** When true, always pack selected inputs into a List — even if only 0 or 1 item is selected. */
  bool force_multi_selection_ = false;
  const NodeEnumDefinition &enum_def_;
  const CPPType *field_base_type_;

 public:
  LazyFunctionForMenuSwitchNode(const bNode &node,
                                GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
      : node_(node), enum_def_(node_storage(node).enum_definition)
  {
    const NodeMenuSwitch &storage = node_storage(node);
    const eNodeSocketDatatype data_type = storage.data_type;
    can_be_field_ = socket_type_supports_fields(data_type);
    force_multi_selection_ = menu_switch_is_multi_selection(&node.owner_tree(), node);
    const bke::bNodeSocketType *socket_type = bke::node_socket_type_find_static(data_type);
    BLI_assert(socket_type != nullptr);
    field_base_type_ = socket_type->base_cpp_type;

    MutableSpan<int> lf_index_by_bsocket = lf_graph_info.mapping.lf_index_by_bsocket;
    debug_name_ = node.name;
    lf_index_by_bsocket[node.input_socket(0).index_in_tree()] = inputs_.append_and_get_index_as(
        "Switch", CPPType::get<SocketValueVariant>(), lf::ValueUsage::Used);
    for (const int i : enum_def_.items().index_range()) {
      const NodeEnumItem &enum_item = enum_def_.items()[i];
      lf_index_by_bsocket[node.input_socket(i + 1).index_in_tree()] =
          inputs_.append_and_get_index_as(
              enum_item.name, CPPType::get<bke::SocketValueVariant>(), lf::ValueUsage::Maybe);
    }
    lf_index_by_bsocket[node.output_socket(0).index_in_tree()] = outputs_.append_and_get_index_as(
        "Value", CPPType::get<bke::SocketValueVariant>());
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const override
  {
    SocketValueVariant condition_variant = params.get_input<SocketValueVariant>(0);
    if (condition_variant.is_context_dependent_field() && can_be_field_) {
      this->execute_field(condition_variant.ensure_type<Field<MenuValue>>(), params);
    }
    else {
      this->execute_single(condition_variant.ensure_type<MenuValue>(), params);
    }
  }

  /**
   * True when this evaluation must pack selected item inputs into a List (including length 0/1).
   * Do not rely only on construction-time #force_multi_selection_: the LF graph can be stale
   * relative to a newer List-shaped Output declaration, which is what the UI shows.
   */
  bool should_output_list(const MenuValue &condition) const
  {
    if (force_multi_selection_ || condition.multi_selection) {
      return true;
    }
    /* Live re-detect (do not sticky-read Output List — that blocks multi-off). */
    return menu_switch_is_multi_selection(&node_.owner_tree(), node_);
  }

  /**
   * When multi-selection mode is active, always treat the condition as multi — even if only one
   * bit is set (values like 1, 2, 4 collide with single-select identifiers and would otherwise
   * fall through to the single-value path, producing non-list data on a List-shaped output).
   */
  MenuValue resolve_menu_value(const MenuValue condition) const
  {
    if (this->should_output_list(condition)) {
      return MenuValue(condition.value, true);
    }
    /* Same rule as socket-usage drawing: an identifier such as 3 is that item, not bits 0|1.
     * Do not re-read this socket's own DNA flag here — a sticky bit would turn a single
     * identifier into the wrong item while the dropdown still shows the one the user picked. */
    return condition.normalized(false, enum_def_.items_num, [&](const int i) {
      return enum_def_.items_array[i].identifier;
    });
  }

  void execute_single(const MenuValue condition_in, lf::Params &params) const
  {
    const MenuValue condition = this->resolve_menu_value(condition_in);
    /* Multi mode (or List-shaped Output): always build a List — length equals selected count
     * (0 empty, 1 one element, N many). Never forward a bare single value. */
    if (condition.multi_selection || this->should_output_list(condition_in)) {
      this->execute_single_multi_selection(MenuValue(condition.value, true), params);
      return;
    }

    bool output_was_set = false;
    for (const int i : IndexRange(enum_def_.items_num)) {
      const NodeEnumItem &enum_item = enum_def_.items_array[i];
      const int input_index = i + 1;
      const bool is_selected = condition.item_is_selected(enum_item.identifier);
      if (is_selected) {
        SocketValueVariant *value_to_forward =
            params.try_get_input_data_ptr_or_request<SocketValueVariant>(input_index);
        if (value_to_forward == nullptr) {
          /* Try again when the value is available. */
          return;
        }

        params.set_output(0, std::move(*value_to_forward));
        output_was_set = true;
      }
      else {
        params.set_input_unused(input_index);
      }
    }
    if (!output_was_set) {
      /* No guarantee that the switch input matches any enum,
       * set default outputs to ensure valid state. */
      set_default_value_for_output_socket(params, 0, node_.output_socket(0));
    }
  }

  /**
   * Multi-selection: collect all selected item inputs into a list (order of menu items).
   * Unselected inputs are marked unused. Empty selection yields an empty list.
   */
  void execute_single_multi_selection(const MenuValue condition, lf::Params &params) const
  {
    Vector<int> selected_indices;
    selected_indices.reserve(enum_def_.items_num);
    for (const int i : IndexRange(enum_def_.items_num)) {
      const NodeEnumItem &enum_item = enum_def_.items_array[i];
      if (condition.item_is_selected(enum_item.identifier)) {
        selected_indices.append(i);
      }
    }

    Array<SocketValueVariant *> selected_values(selected_indices.size(), nullptr);
    for (const int i : selected_indices.index_range()) {
      const int input_index = selected_indices[i] + 1;
      selected_values[i] = params.try_get_input_data_ptr_or_request<SocketValueVariant>(
          input_index);
      if (selected_values[i] == nullptr) {
        /* Try again when values are available. */
        return;
      }
    }
    for (const int i : IndexRange(enum_def_.items_num)) {
      if (!condition.item_is_selected(enum_def_.items_array[i].identifier)) {
        params.set_input_unused(i + 1);
      }
    }

    const int count = selected_indices.size();
    if (count == 0) {
      params.set_output(0,
                        SocketValueVariant::from(GList::create(
                            *field_base_type_, GList::ArrayData::ForConstructed(*field_base_type_, 0), 0)));
      return;
    }

    /* Always build a List of `count` elements (count==1 still a length-1 list, never a bare
     * single GeometrySet/value). Non-single inputs are converted to single when possible. */
    GList::ArrayData array_data = GList::ArrayData::ForUninitialized(*field_base_type_, count);
    void *dst = array_data.span_for_write(*field_base_type_, count).data();
    for (const int i : IndexRange(count)) {
      SocketValueVariant src_variant = *selected_values[i];
      void *dst_i = POINTER_OFFSET(dst, field_base_type_->size * i);
      if (!src_variant.is_single()) {
        src_variant.convert_to_single();
      }
      if (src_variant.is_single()) {
        const GPointer src = src_variant.get_single_ptr();
        if (src.type() == field_base_type_) {
          field_base_type_->copy_construct(src.get(), dst_i);
          continue;
        }
      }
      field_base_type_->copy_construct(field_base_type_->default_value(), dst_i);
    }
    GListPtr list = GList::create(*field_base_type_, std::move(array_data), count);
    BLI_assert(list && list->size() == count);
    params.set_output(0, SocketValueVariant::from(std::move(list)));
  }

  void execute_field(Field<MenuValue> condition, lf::Params &params) const
  {
    /* When the condition is a non-constant field, we need all inputs. */
    const int values_num = this->enum_def_.items_num;
    Array<SocketValueVariant *, 8> input_values(values_num);
    for (const int i : IndexRange(values_num)) {
      const int input_index = i + 1;
      input_values[i] = params.try_get_input_data_ptr_or_request<SocketValueVariant>(input_index);
    }
    if (input_values.as_span().contains(nullptr)) {
      /* Try again when inputs are available. */
      return;
    }

    Vector<GField> item_fields;
    item_fields.reserve(enum_def_.items_num + 1);
    item_fields.append(std::move(condition));
    for (const int i : IndexRange(enum_def_.items_num)) {
      item_fields.append(input_values[i]->extract<GField>());
    }
    std::unique_ptr<MultiFunction> multi_function = std::make_unique<MenuSwitchFn>(
        enum_def_, *field_base_type_);
    fn::FieldOperationPtr operation = FieldOperation::from(std::move(multi_function),
                                                           std::move(item_fields));

    params.set_output(0, SocketValueVariant::from(GField(operation, 0)));
  }
};

class LazyFunctionMenuSwitchBooleanOutputs : public LazyFunction {
 private:
  const NodeEnumDefinition &enum_def_;
  bool can_be_field_ = false;
  bool force_multi_selection_ = false;
  MenuSwitchBoolsFn fn_;

 public:
  LazyFunctionMenuSwitchBooleanOutputs(const bNode &node,
                                       GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
      : enum_def_(node_storage(node).enum_definition), fn_(enum_def_)
  {
    debug_name_ = node.name;
    const NodeMenuSwitch &storage = node_storage(node);
    can_be_field_ = socket_type_supports_fields(storage.data_type);
    force_multi_selection_ = menu_switch_is_multi_selection(&node.owner_tree(), node);

    MutableSpan<int> lf_index_by_bsocket = lf_graph_info.mapping.lf_index_by_bsocket;
    lf_index_by_bsocket[node.input_socket(0).index_in_tree()] = inputs_.append_and_get_index_as(
        "Switch", CPPType::get<SocketValueVariant>(), lf::ValueUsage::Used);
    for (const int i : enum_def_.items().index_range()) {
      const NodeEnumItem &enum_item = enum_def_.items_array[i];
      lf_index_by_bsocket[node.output_socket(i + 1).index_in_tree()] =
          outputs_.append_and_get_index_as(enum_item.name, CPPType::get<SocketValueVariant>());
    }
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const override
  {
    GeoNodesUserData *user_data = dynamic_cast<GeoNodesUserData *>(context.user_data);
    SocketValueVariant condition_variant = params.get_input<SocketValueVariant>(0);
    if (condition_variant.is_single()) {
      this->execute_single(this->ensure_multi_flag(condition_variant.ensure_type<MenuValue>()),
                           params);
      return;
    }

    Array<SocketValueVariant *> input_values = {&condition_variant};
    Array<SocketValueVariant *> output_values(enum_def_.items_num, nullptr);
    for (const int i : IndexRange(enum_def_.items_num)) {
      if (params.get_output_usage(i) != lf::ValueUsage::Unused) {
        output_values[i] = static_cast<SocketValueVariant *>(params.get_output_data_ptr(i));
        new (output_values[i]) SocketValueVariant(false);
      }
    }
    std::string error_message;
    if (!execute_multi_function_on_value_variant(
            fn_, input_values, output_values, user_data, error_message))
    {
      /* Is set to default value already. */
    }
    for (const int i : IndexRange(enum_def_.items_num)) {
      if (output_values[i]) {
        params.output_set(i);
      }
    }
  }

  /** Force multi interpretation whenever multi-selection mode is active on this node. */
  MenuValue ensure_multi_flag(const MenuValue condition) const
  {
    if (force_multi_selection_) {
      return MenuValue(condition.value, true);
    }
    return condition.normalized(false, enum_def_.items_num, [&](const int i) {
      return enum_def_.items_array[i].identifier;
    });
  }

  void execute_single(const MenuValue condition, lf::Params &params) const
  {
    for (const int i : IndexRange(enum_def_.items_num)) {
      const NodeEnumItem &enum_item = enum_def_.items_array[i];
      const bool is_selected = condition.item_is_selected(enum_item.identifier);
      /* Output i corresponds to items_array[i] (same order as declare loop). */
      params.set_output(i, SocketValueVariant(is_selected));
    }
  }
};

/**
 * Outputs booleans that indicate which inputs of a menu switch node
 * are used. Note that it's possible that multiple inputs are used
 * when the condition is a field or multi-selection.
 */
class LazyFunctionForMenuSwitchSocketUsage : public lf::LazyFunction {
  const NodeEnumDefinition &enum_def_;
  bool force_multi_selection_ = false;

 public:
  LazyFunctionForMenuSwitchSocketUsage(const bNode &node)
      : enum_def_(node_storage(node).enum_definition)
  {
    debug_name_ = "Menu Switch Socket Usage";
    force_multi_selection_ = menu_switch_is_multi_selection(&node.owner_tree(), node);
    inputs_.append_as("Condition", CPPType::get<SocketValueVariant>());
    for (const int i : IndexRange(enum_def_.items_num)) {
      const NodeEnumItem &enum_item = enum_def_.items()[i];
      outputs_.append_as(enum_item.name, CPPType::get<bool>());
    }
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const override
  {
    const SocketValueVariant &condition_variant = params.get_input<SocketValueVariant>(0);
    if (condition_variant.is_context_dependent_field()) {
      for (const int i : IndexRange(enum_def_.items_num)) {
        params.set_output(i, true);
      }
      return;
    }
    MenuValue value = condition_variant.copy_as<MenuValue>();
    /* Multi mode uses bits. Single mode keeps the identifier, including ids that are not
     * powers of two (3, 5, 6, ...). */
    value = value.normalized(force_multi_selection_, enum_def_.items_num, [&](const int i) {
      return enum_def_.items()[i].identifier;
    });
    /* Selected items are used; unselected can be unused. Same for multi bit-fields. */
    for (const int i : IndexRange(enum_def_.items_num)) {
      const NodeEnumItem &enum_item = enum_def_.items()[i];
      params.set_output(i, value.item_is_selected(enum_item.identifier));
    }
  }
};

using namespace blender::compositor;

class MenuSwitchOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    Result &value_output = this->get_result("Output");
    /* Same read as the scheduler: a group-interface menu may still be a raw int. */
    const bool multi_if_int = menu_switch_is_multi_selection(&this->node().owner_tree(),
                                                            this->node());
    const NodeEnumDefinition &enum_definition = node_storage(node()).enum_definition;
    const MenuValue menu_raw =
        menu_value_from_result(this->get_input("Menu"), multi_if_int).value_or(MenuValue(-1));
    /* Identifier 3 must stay item 3 when multi is off. Bits only if multi is already set,
     * or the integer is not one of the items (dropped multi flag). */
    const MenuValue menu_identifier = menu_raw.normalized(
        multi_if_int, enum_definition.items_num, [&](const int i) {
          return enum_definition.items()[i].identifier;
        });
    bool found_item = false;

    for (const int i : IndexRange(enum_definition.items_num)) {
      const NodeEnumItem &enum_item = enum_definition.items()[i];
      const std::string identifier = MenuSwitchItemsAccessor::socket_identifier_for_item(
          enum_item);
      const bool is_selected = menu_identifier.item_is_selected(enum_item.identifier);
      Result &item_output = this->get_result(identifier);
      if (item_output.should_compute()) {
        item_output.allocate_single_value();
        item_output.set_single_value(is_selected);
      }
      if (!is_selected) {
        continue;
      }
      /* Multi-selection list output is geometry-nodes only; compositor keeps first match. */
      if (!found_item) {
        const Result &input = this->get_input(identifier);
        value_output.share_data(input);
        found_item = true;
      }
    }

    if (!found_item) {
      /* The menu identifier didn't match any item, so allocate an invalid output. */
      value_output.allocate_invalid();
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new MenuSwitchOperation(context, node);
}

static void node_layout_ex(ui::Layout &layout, bContext *C, PointerRNA *ptr)
{
  bNodeTree &tree = *reinterpret_cast<bNodeTree *>(ptr->owner_id);
  bNode &node = *static_cast<bNode *>(ptr->data);

  layout.prop(ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);

  if (ui::Layout *panel = layout.panel(C, "menu_switch_items", false, IFACE_("Menu Items"))) {
    socket_items::ui::draw_items_list_with_operators<MenuSwitchItemsAccessor>(
        C, panel, tree, node);
    socket_items::ui::draw_active_item_props<MenuSwitchItemsAccessor>(
        tree, node, [&](PointerRNA *item_ptr) {
          panel->use_property_split_set(true);
          panel->use_property_decorate_set(false);
          panel->prop(item_ptr, "description", UI_ITEM_NONE, std::nullopt, ICON_NONE);
        });
  }
}

static void node_operators()
{
  socket_items::ops::make_common_operators<MenuSwitchItemsAccessor>();
}

static bool node_insert_link(bke::NodeInsertLinkParams &params)
{
  return socket_items::try_add_item_via_any_extend_socket<MenuSwitchItemsAccessor>(
      params.ntree, params.node, params.node, params.link);
}

static void node_blend_write(const bNodeTree & /*ntree*/, const bNode &node, BlendWriter &writer)
{
  socket_items::blend_write<MenuSwitchItemsAccessor>(&writer, node);
}

static void node_blend_read(bNodeTree & /*ntree*/, bNode &node, BlendDataReader &reader)
{
  socket_items::blend_read_data<MenuSwitchItemsAccessor>(&reader, node);
}

static const bNodeSocket *node_internally_linked_input(const bNodeTree & /*tree*/,
                                                       const bNode &node,
                                                       const bNodeSocket &output_socket)
{
  const NodeMenuSwitch &storage = node_storage(node);
  if (storage.enum_definition.items_num == 0) {
    return nullptr;
  }
  if (&output_socket == node.outputs.first_) {
    /* Default to the first enum item input. */
    return &node.input_socket(1);
  }
  return nullptr;
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
        return ntree_type->valid_socket_type(ntree_type, socket_type);
      });
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "data_type",
                    "Data Type",
                    "",
                    rna_enum_node_socket_data_type_items,
                    NOD_storage_enum_accessors(data_type),
                    SOCK_GEOMETRY,
                    data_type_items_callback);
}

static void register_node()
{
  static bke::bNodeType ntype;

  common_node_type_base(&ntype, "GeometryNodeMenuSwitch"_ustr, GEO_NODE_MENU_SWITCH);
  ntype.ui_name = "Menu Switch";
  ntype.ui_description = "Select from multiple inputs by name";
  ntype.enum_name_legacy = "MENU_SWITCH";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  bke::node_type_storage(ntype, "NodeMenuSwitch", node_free_storage, node_copy_storage);
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.draw_buttons = node_layout;
  ntype.draw_buttons_ex = node_layout_ex;
  ntype.register_operators = node_operators;
  ntype.insert_link = node_insert_link;
  ntype.ignore_inferred_input_socket_visibility = true;
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  ntype.internally_linked_input = node_internally_linked_input;
  ntype.get_compositor_operation = get_compositor_operation;
  bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(register_node)

}  // namespace nodes::node_geo_menu_switch_cc

namespace nodes {

std::unique_ptr<LazyFunction> get_menu_switch_node_lazy_function(
    const bNode &node, GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
{
  using namespace node_geo_menu_switch_cc;
  BLI_assert(node.type_legacy == GEO_NODE_MENU_SWITCH);
  return std::make_unique<LazyFunctionForMenuSwitchNode>(node, lf_graph_info);
}

std::unique_ptr<LazyFunction> get_menu_switch_node_boolean_outputs_lazy_function(
    const bNode &node, GeometryNodesLazyFunctionGraphInfo &lf_graph_info)
{
  using namespace node_geo_menu_switch_cc;
  BLI_assert(node.type_legacy == GEO_NODE_MENU_SWITCH);
  return std::make_unique<LazyFunctionMenuSwitchBooleanOutputs>(node, lf_graph_info);
}

std::unique_ptr<LazyFunction> get_menu_switch_node_socket_usage_lazy_function(const bNode &node)
{
  using namespace node_geo_menu_switch_cc;
  BLI_assert(node.type_legacy == GEO_NODE_MENU_SWITCH);
  return std::make_unique<LazyFunctionForMenuSwitchSocketUsage>(node);
}

StructRNA **MenuSwitchItemsAccessor::item_srna = &RNA_NodeEnumItem;

void MenuSwitchItemsAccessor::blend_write_item(BlendWriter *writer, const ItemT &item)
{
  writer->write_string(item.name);
  writer->write_string(item.description);
}

void MenuSwitchItemsAccessor::blend_read_data_item(BlendDataReader *reader, ItemT &item)
{
  BLO_read_string(reader, &item.name);
  BLO_read_string(reader, &item.description);
}

}  // namespace nodes
}  // namespace blender
