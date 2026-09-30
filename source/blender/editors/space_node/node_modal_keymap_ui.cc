/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 *
 * Editor for the default modal keymap of a node tool. The events themselves are defined by the
 * Modal Event nodes in the node group and its nested groups, this only assigns a default key
 * binding to each of them. The bindings users actually work with are configured in the keymap
 * editor, this is what a node group ships with.
 */

#include "MEM_guardedalloc.h"

#include "BKE_context.hh"
#include "BKE_lib_id.hh"
#include "BKE_main_invariants.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_screen.hh"

#include "BLI_listbase.hh"
#include "BLI_string_utf8.hh"

#include "BLT_translation.hh"

#include "DNA_screen_types.h"
#include "DNA_space_types.h"

#include "NOD_geometry_nodes_modal_events.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_prototypes.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"
#include "UI_tree_view.hh"

#include "WM_api.hh"
#include "WM_keymap.hh"
#include "WM_types.hh"

#include "ED_screen.hh"

#include "node_intern.hh"

namespace blender::ed::space_node {

/* -------------------------------------------------------------------- */
/** \name Add and Remove Operators
 * \{ */

static bool modal_keymap_edit_poll(bContext *C)
{
  const SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return false;
  }
  return snode->edittree->type == NTREE_GEOMETRY;
}

/**
 * The gathered events and bindings are cached on the node tree and written to the asset meta-data,
 * both of which are refreshed by the node tree update.
 */
static void tag_modal_keymap_changed(bContext &C, bNodeTree &tree)
{
  BKE_ntree_update_tag_modal_keymap(&tree);
  BKE_main_ensure_invariants(*CTX_data_main(&C), tree.id);
  WM_event_add_notifier(&C, NC_NODE | ND_DISPLAY, nullptr);
}

static wmOperatorStatus modal_keymap_item_add_exec(bContext *C, wmOperator *op)
{
  SpaceNode &snode = *CTX_wm_space_node(C);
  bNodeTree &tree = *snode.edittree;
  const std::string event_name = RNA_string_get(op->ptr, "event_name");
  if (event_name.empty()) {
    return OPERATOR_CANCELLED;
  }
  if (!tree.geometry_node_asset_traits) {
    tree.geometry_node_asset_traits = MEM_new<GeometryNodeAssetTraits>(__func__);
  }
  GeometryNodeAssetTraits &traits = *tree.geometry_node_asset_traits;

  const int old_num = traits.modal_keymap_default_num;
  auto *new_items = MEM_new_array_zeroed<GeometryNodeModalKeymapItem>(old_num + 1, __func__);
  if (traits.modal_keymap_default) {
    std::copy_n(traits.modal_keymap_default, old_num, new_items);
    MEM_delete(traits.modal_keymap_default);
  }
  STRNCPY_UTF8(new_items[old_num].event_name, event_name.c_str());
  new_items[old_num].val = KM_PRESS;
  traits.modal_keymap_default = new_items;
  traits.modal_keymap_default_num = old_num + 1;
  traits.modal_keymap_active_index = old_num;

  tag_modal_keymap_changed(*C, tree);
  return OPERATOR_FINISHED;
}

void NODE_OT_modal_keymap_item_add(wmOperatorType *ot)
{
  ot->name = "Add Modal Keymap Item";
  ot->description = "Add a default key binding for one of the node tool's events";
  ot->idname = "NODE_OT_modal_keymap_item_add";

  ot->exec = modal_keymap_item_add_exec;
  ot->poll = modal_keymap_edit_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_string(
      ot->srna, "event_name", nullptr, MAX_NAME, "Name", "Name of the event to bind a key to");
}

static wmOperatorStatus modal_keymap_item_remove_exec(bContext *C, wmOperator *op)
{
  SpaceNode &snode = *CTX_wm_space_node(C);
  bNodeTree &tree = *snode.edittree;
  if (!tree.geometry_node_asset_traits) {
    return OPERATOR_CANCELLED;
  }
  GeometryNodeAssetTraits &traits = *tree.geometry_node_asset_traits;
  const int index = RNA_int_get(op->ptr, "index");
  if (!IndexRange(traits.modal_keymap_default_num).contains(index)) {
    return OPERATOR_CANCELLED;
  }
  const int new_num = traits.modal_keymap_default_num - 1;
  GeometryNodeModalKeymapItem *new_items = nullptr;
  if (new_num > 0) {
    new_items = MEM_new_array_zeroed<GeometryNodeModalKeymapItem>(new_num, __func__);
    std::copy_n(traits.modal_keymap_default, index, new_items);
    std::copy_n(traits.modal_keymap_default + index + 1, new_num - index, new_items + index);
  }
  MEM_delete(traits.modal_keymap_default);
  traits.modal_keymap_default = new_items;
  traits.modal_keymap_default_num = new_num;
  traits.modal_keymap_active_index = std::min(traits.modal_keymap_active_index,
                                              std::max(new_num - 1, 0));

  tag_modal_keymap_changed(*C, tree);
  return OPERATOR_FINISHED;
}

void NODE_OT_modal_keymap_item_remove(wmOperatorType *ot)
{
  ot->name = "Remove Modal Keymap Item";
  ot->description = "Remove one of the node group's default key bindings";
  ot->idname = "NODE_OT_modal_keymap_item_remove";

  ot->exec = modal_keymap_item_remove_exec;
  ot->poll = modal_keymap_edit_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(
      ot->srna, "index", 0, 0, INT_MAX, "Index", "Index of the binding to remove", 0, INT_MAX);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Tree View
 * \{ */

static std::string binding_to_string(const GeometryNodeModalKeymapItem &item)
{
  return WM_keymap_item_raw_to_string(item.shift,
                                      item.ctrl,
                                      item.alt,
                                      item.oskey,
                                      item.hyper,
                                      item.keymodifier,
                                      item.val,
                                      item.type,
                                      false)
      .value_or("");
}

/**
 * One default key binding. Bindings that the edited node group defines itself can be changed here,
 * the ones that come from nested groups are only shown for reference and are edited in the keymap
 * editor of the group that defines them.
 */
class ModalBindingItem : public ui::AbstractTreeViewItem {
  bNodeTree &tree_;
  /** Index into the node group's own bindings, or -1 for a binding from a nested group. */
  int local_index_ = -1;
  /** The key binding in a readable form, since the row is only a label. */
  std::string binding_str_;
  /** Name of the defining node group, for bindings from nested groups. */
  std::string group_name_;

 public:
  ModalBindingItem(bNodeTree &tree, const int local_index) : tree_(tree), local_index_(local_index)
  {
    /* The label identifies the item in the tree, so it has to stay the same while the binding is
     * edited. */
    label_ = std::to_string(local_index);
    binding_str_ = binding_to_string(
        tree.geometry_node_asset_traits->modal_keymap_default[local_index]);
  }

  ModalBindingItem(bNodeTree &tree, const nodes::ModalBinding &binding, Main &bmain) : tree_(tree)
  {
    const ID *group = BKE_libblock_find_session_uid(&bmain, ID_NT, binding.group_session_uid);
    group_name_ = group ? BKE_id_name(*group) : "";
    binding_str_ = binding_to_string(binding.item);
    label_ = binding_str_;
  }

  bool is_local() const
  {
    return local_index_ >= 0;
  }

  void build_row(ui::Layout &row) override
  {
    if (!this->is_local()) {
      /* Bindings from nested groups are only shown for reference, they are edited in the node
       * editor of the group that defines them. */
      ui::Layout &sub = row.row(true);
      sub.enabled_set(false);
      sub.label(binding_str_, ICON_NONE);
      sub.alignment_set(ui::LayoutAlign::Right);
      sub.label(group_name_, ICON_NODETREE);
      return;
    }
    row.label(binding_str_, ICON_NONE);
    PointerRNA op_ptr = row.op("node.modal_keymap_item_remove", "", ICON_X);
    RNA_int_set(&op_ptr, "index", local_index_);
  }

  void on_activate(bContext & /*C*/) override
  {
    if (this->is_local()) {
      tree_.geometry_node_asset_traits->modal_keymap_active_index = local_index_;
    }
  }

  std::optional<bool> should_be_active() const override
  {
    const GeometryNodeAssetTraits *traits = tree_.geometry_node_asset_traits;
    return traits && this->is_local() && traits->modal_keymap_active_index == local_index_;
  }
};

/** One event defined by a Modal Event node, with its default bindings as children. */
class ModalEventItem : public ui::AbstractTreeViewItem {
  std::string description_;

 public:
  ModalEventItem(const nodes::ModalEvent &event) : description_(event.description)
  {
    label_ = event.name;
  }

  void build_row(ui::Layout &row) override
  {
    row.label(label_, ICON_NONE);
    if (!description_.empty()) {
      ui::Layout &sub = row.row(true);
      sub.active_set(false);
      sub.label(description_, ICON_NONE);
    }
    ui::Layout &buttons = row.row(true);
    buttons.alignment_set(ui::LayoutAlign::Right);
    PointerRNA op_ptr = buttons.op("node.modal_keymap_item_add", "", ICON_ADD);
    RNA_string_set(&op_ptr, "event_name", label_.c_str());
  }

  bool supports_collapsing() const override
  {
    return false;
  }
};

class ModalEventsView : public ui::AbstractTreeView {
  bNodeTree &tree_;
  Main &bmain_;

 public:
  ModalEventsView(bNodeTree &tree, Main &bmain) : tree_(tree), bmain_(bmain) {}

  void build_tree() override
  {
    const GeometryNodeAssetTraits *traits = tree_.geometry_node_asset_traits;
    const nodes::ModalEventsInfo *info = tree_.runtime->modal_events.get();

    Set<StringRef> event_names;
    if (info) {
      for (const nodes::ModalEvent &event : info->events) {
        event_names.add(event.name);
        ui::AbstractTreeViewItem &parent = this->add_tree_item<ModalEventItem>(event);
        if (traits) {
          for (const int i : IndexRange(traits->modal_keymap_default_num)) {
            const GeometryNodeModalKeymapItem &item = traits->modal_keymap_default[i];
            if (event.name == item.event_name) {
              parent.add_tree_item<ModalBindingItem>(tree_, i);
            }
          }
        }
        for (const nodes::ModalBinding &binding : info->bindings) {
          if (event.name == binding.item.event_name &&
              binding.group_session_uid != tree_.id.session_uid)
          {
            parent.add_tree_item<ModalBindingItem>(tree_, binding, bmain_);
          }
        }
      }
    }
    /* List the node group's own bindings whose Modal Event node is gone last, so that they can
     * still be seen and removed. The default keymap being out of date is not an error. */
    if (traits) {
      for (const int i : IndexRange(traits->modal_keymap_default_num)) {
        const GeometryNodeModalKeymapItem &item = traits->modal_keymap_default[i];
        if (!event_names.contains(item.event_name)) {
          this->add_tree_item<ModalBindingItem>(tree_, i);
        }
      }
    }
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Panel
 * \{ */

static bool modal_keymap_panel_poll(const bContext *C, PanelType * /*pt*/)
{
  const SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return false;
  }
  const bNodeTree &tree = *snode->edittree;
  if (tree.type != NTREE_GEOMETRY) {
    return false;
  }
  /* Show the tab when the node group defines events at any nesting level, but also when it only
   * has bindings left over from removed Modal Event nodes, so that those can still be cleaned up.
   */
  if (tree.runtime->modal_events && !tree.runtime->modal_events->events.is_empty()) {
    return true;
  }
  const GeometryNodeAssetTraits *traits = tree.geometry_node_asset_traits;
  return traits && traits->modal_keymap_default_num > 0;
}

static void modal_keymap_panel_draw(const bContext *C, Panel *panel)
{
  SpaceNode &snode = *CTX_wm_space_node(C);
  bNodeTree &tree = *snode.edittree;
  ui::Layout &layout = *panel->layout;

  ui::Block *block = layout.block();
  ui::AbstractTreeView *tree_view = block_add_view(
      *block,
      "Node Tool Modal Events",
      std::make_unique<ModalEventsView>(tree, *CTX_data_main(C)));
  tree_view->set_default_rows(4);
  ui::TreeViewBuilder::build_tree_view(*C, *tree_view, layout);

  GeometryNodeAssetTraits *traits = tree.geometry_node_asset_traits;
  if (!traits || !traits->modal_keymap_default ||
      !IndexRange(traits->modal_keymap_default_num).contains(traits->modal_keymap_active_index))
  {
    return;
  }
  GeometryNodeModalKeymapItem &item =
      traits->modal_keymap_default[traits->modal_keymap_active_index];
  PointerRNA item_ptr = RNA_pointer_create_discrete(
      &tree.id, RNA_GeometryNodeModalKeymapItem, &item);

  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  /* Same options as the keymap editor, see `rna_keymap_ui.py`. */
  layout.prop(&item_ptr, "map_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  switch (WM_keymap_map_type_from_event_type(item.type)) {
    case KMI_TYPE_KEYBOARD:
    case KMI_TYPE_MOUSE:
    case KMI_TYPE_NDOF:
      layout.prop(&item_ptr, "type", ui::ITEM_R_FULL_EVENT, std::nullopt, ICON_NONE);
      break;
    default:
      layout.prop(&item_ptr, "type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
      break;
  }
  layout.prop(&item_ptr, "value", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(&item_ptr, "key_modifier", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  ui::Layout &repeat_row = layout.row(true);
  repeat_row.active_set(ELEM(item.val, KM_ANY, KM_PRESS));
  repeat_row.prop(&item_ptr, "repeat", UI_ITEM_NONE, std::nullopt, ICON_NONE);

  ui::Layout &col = layout.column(true);
  col.prop(&item_ptr, "shift", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  col.prop(&item_ptr, "ctrl", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  col.prop(&item_ptr, "alt", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  col.prop(&item_ptr, "oskey", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  col.prop(&item_ptr, "hyper", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

void node_modal_keymap_panel_register(ARegionType *art)
{
  PanelType *pt = MEM_new_zeroed<PanelType>(__func__);
  STRNCPY_UTF8(pt->idname, "NODE_PT_modal_keymap");
  STRNCPY_UTF8(pt->label, N_("Events"));
  STRNCPY_UTF8(pt->category, "Keymap");
  STRNCPY_UTF8(pt->translation_context, BLT_I18NCONTEXT_DEFAULT_BPYRNA);
  pt->draw = modal_keymap_panel_draw;
  pt->poll = modal_keymap_panel_poll;
  BLI_addtail(&art->paneltypes, pt);
}

/** \} */

}  // namespace blender::ed::space_node
