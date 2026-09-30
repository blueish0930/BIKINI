/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 */

#include <cstdlib>

#include "MEM_guardedalloc.h"

#include "DNA_node_types.h"
#include "DNA_space_types.h"

#include "BLI_listbase.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_vector.hh"

#include "BLT_translation.hh"

#include "BKE_action.hh"
#include "BKE_animsys.hh"
#include "BKE_context.hh"
#include "BKE_lib_id.hh"
#include "BKE_library.hh"
#include "BKE_main.hh"
#include "BKE_main_invariants.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_report.hh"

#include "ANIM_action.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "ED_node.hh"
#include "ED_node_preview.hh"
#include "ED_render.hh"
#include "ED_screen.hh"

#include "NOD_image_points.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_prototypes.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "NOD_common.hh"
#include "NOD_composite.hh"
#include "NOD_geometry.hh"
#include "NOD_node_declaration.hh"
#include "NOD_shader.h"
#include "NOD_socket.hh"
#include "NOD_texture.h"

#include "node_intern.hh" /* own include */

namespace blender::ed::space_node {

/* -------------------------------------------------------------------- */
/** \name Local Utilities
 * \{ */

static bool node_group_operator_active_poll(bContext *C)
{
  if (ED_operator_node_active(C)) {
    SpaceNode *snode = CTX_wm_space_node(C);

    /* Group operators only defined for standard node tree types.
     * Disabled otherwise to allow python-nodes define their own operators
     * with same key-map.
     * Image Process is included so Import Points can double-click / Tab into its nested
     * Geometry Node tree. */
    if (STR_ELEM(snode->tree_idname,
                 "ShaderNodeTree",
                 "CompositorNodeTree",
                 "TextureNodeTree",
                 "GeometryNodeTree",
                 "ImageNodeTree"))
    {
      return true;
    }
  }
  return false;
}

static bool node_group_operator_editable(bContext *C)
{
  if (ED_operator_node_editable(C)) {
    SpaceNode *snode = CTX_wm_space_node(C);

    /* Group operators only defined for standard node tree types.
     * Disabled otherwise to allow python-nodes define their own operators
     * with same key-map. */
    if (ED_node_is_shader(snode) || ED_node_is_compositor(snode) || ED_node_is_texture(snode) ||
        ED_node_is_geometry(snode) || ED_node_is_image(snode))
    {
      return true;
    }
  }
  return false;
}

static StringRef group_ntree_idname(bContext *C)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  return snode->tree_idname;
}

UString node_group_idname(const bContext *C)
{
  SpaceNode *snode = CTX_wm_space_node(C);

  if (ED_node_is_shader(snode)) {
    return ntreeType_Shader->group_idname;
  }
  if (ED_node_is_compositor(snode)) {
    return ntreeType_Composite->group_idname;
  }
  if (ED_node_is_texture(snode)) {
    return ntreeType_Texture->group_idname;
  }
  if (ED_node_is_geometry(snode)) {
    return ntreeType_Geometry->group_idname;
  }

  return ""_ustr;
}

static bNode *node_group_get_active(bContext *C, const UString node_idname)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  bNode *node = bke::node_get_active(*snode->edittree);

  if (node && node->idname == node_idname) {
    return node;
  }
  return nullptr;
}

bool node_tree_session_is_unlocked(const SpaceNode &snode, const bNodeTree &ntree)
{
  if (!snode.runtime) {
    return false;
  }
  if (snode.runtime->unlocked_tree_session_uids.contains(ntree.id.session_uid)) {
    return true;
  }
  if (!bke::node_tree_is_locked(ntree)) {
    return false;
  }
  /* Nested groups locked with the same key as an already-authenticated ancestor
   * may be entered without typing the password again. Independent locked groups
   * (even with the same typed password) use a different salt and must not inherit. */
  for (const bNodeTreePath &path : snode.treepath) {
    const bNodeTree *ancestor = path.nodetree;
    if (!ancestor || ancestor == &ntree) {
      continue;
    }
    if (!snode.runtime->unlocked_tree_session_uids.contains(ancestor->id.session_uid)) {
      continue;
    }
    if (bke::node_tree_lock_shares_key(*ancestor, ntree)) {
      return true;
    }
  }
  return false;
}

static void node_group_mark_decrypted_session(SpaceNode &snode, const bNodeTree &ntree)
{
  if (snode.runtime) {
    snode.runtime->unlocked_tree_session_uids.add(ntree.id.session_uid);
  }
}

static bNodeTree *node_group_password_op_tree(bContext *C, wmOperator *op)
{
  const std::string tree_name = RNA_string_get(op->ptr, "node_tree_name");
  if (!tree_name.empty()) {
    Main *bmain = CTX_data_main(C);
    if (bmain) {
      if (bNodeTree *ntree = id_cast<bNodeTree *>(
              BKE_libblock_find_name(bmain, ID_NT, tree_name.c_str())))
      {
        return ntree;
      }
    }
  }
  SpaceNode *snode = CTX_wm_space_node(C);
  return snode ? snode->edittree : nullptr;
}

static wmOperatorStatus node_group_enter_or_prompt(bContext *C,
                                                   ARegion *region,
                                                   SpaceNode *snode,
                                                   bNodeTree *ngroup,
                                                   bNode *gnode)
{
  if (!ngroup) {
    return OPERATOR_CANCELLED;
  }
  if (bke::node_tree_is_locked(*ngroup) && !node_tree_session_is_unlocked(*snode, *ngroup)) {
    wmOperatorType *ot = WM_operatortype_find("NODE_OT_group_password_enter", false);
    if (!ot) {
      return OPERATOR_CANCELLED;
    }
    PointerRNA ptr = WM_operator_properties_create_ptr(ot);
    RNA_string_set(&ptr, "node_tree_name", ngroup->id.name + 2);
    if (gnode) {
      RNA_string_set(&ptr, "node_name", gnode->name);
    }
    RNA_boolean_set(&ptr, "do_push", true);
    WM_operator_name_call_ptr(C, ot, wm::OpCallContext::InvokeDefault, &ptr, nullptr);
    WM_operator_properties_free(&ptr);
    return OPERATOR_FINISHED;
  }
  if (snode->runtime && bke::node_tree_is_locked(*ngroup)) {
    /* Remember this visit so the editor stays unblocked while inside. Pop removes it. */
    snode->runtime->unlocked_tree_session_uids.add(ngroup->id.session_uid);
  }
  ED_node_tree_push(region, snode, ngroup, gnode);
  return OPERATOR_FINISHED;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Lock / Unlock Node Group
 * \{ */

static wmOperatorStatus node_group_lock_toggle_invoke(bContext *C,
                                                      wmOperator *op,
                                                      const wmEvent * /*event*/)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  bNodeTree *group = snode ? snode->edittree : nullptr;
  if (!group || (group->id.flag & ID_FLAG_EMBEDDED_DATA)) {
    return OPERATOR_CANCELLED;
  }

  const bool removing_lock = bke::node_tree_is_locked(*group);
  RNA_boolean_set(op->ptr, "removing_lock", removing_lock);

  if (removing_lock) {
    return WM_operator_props_dialog_popup(
        C,
        op,
        320,
        IFACE_("Unlock Node Group"),
        IFACE_("Unlock"),
        false,
        IFACE_("Enter the password to remove the lock."),
        true);
  }
  return WM_operator_props_dialog_popup(
      C,
      op,
      320,
      IFACE_("Lock Node Group"),
      IFACE_("Lock"),
      false,
      IFACE_("Set a password. The node graph is stored encrypted in the .blend file and "
             "appears empty in other Blender versions, but still computes automatically. "
             "This password is required to view or edit the internals."),
      true);
}

static wmOperatorStatus node_group_lock_toggle_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  bNodeTree *group = snode ? snode->edittree : nullptr;
  if (!group || (group->id.flag & ID_FLAG_EMBEDDED_DATA)) {
    return OPERATOR_CANCELLED;
  }

  const std::string password = RNA_string_get(op->ptr, "password");
  const bool removing_lock = RNA_boolean_get(op->ptr, "removing_lock");

  if (removing_lock) {
    if (!bke::node_tree_lock_unlock_contents(*CTX_data_main(C), *group, password, op->reports)) {
      return OPERATOR_CANCELLED;
    }
    bke::node_tree_clear_lock(*group);
    if (snode->runtime) {
      snode->runtime->unlocked_tree_session_uids.remove(group->id.session_uid);
    }
  }
  else {
    if (password.empty()) {
      BKE_report(op->reports, RPT_ERROR, "Password cannot be empty");
      return OPERATOR_CANCELLED;
    }
    const std::string confirm = RNA_string_get(op->ptr, "confirm_password");
    if (password != confirm) {
      BKE_report(op->reports, RPT_ERROR, "Passwords do not match");
      return OPERATOR_CANCELLED;
    }
    Main *bmain = CTX_data_main(C);
    if (!bke::node_tree_lock_apply(*bmain, *group, password, op->reports)) {
      return OPERATOR_CANCELLED;
    }
    bke::node_tree_lock_session_acquire(*group);
    /* Stay inside after locking; re-entry later will require the password. */
    node_group_mark_decrypted_session(*snode, *group);
  }

  WM_event_add_notifier(C, NC_NODE | ND_DISPLAY, group);
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_NODE, nullptr);
  return OPERATOR_FINISHED;
}

static void node_group_lock_toggle_ui(bContext * /*C*/, wmOperator *op)
{
  ui::Layout &layout = *op->layout;
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(op->ptr, "password", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  if (!RNA_boolean_get(op->ptr, "removing_lock")) {
    layout.prop(op->ptr, "confirm_password", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  }
}

void NODE_OT_group_lock_toggle(wmOperatorType *ot)
{
  ot->name = "Lock Node Group";
  ot->description = "Lock or unlock this node group with a password";
  ot->idname = "NODE_OT_group_lock_toggle";

  ot->invoke = node_group_lock_toggle_invoke;
  ot->exec = node_group_lock_toggle_exec;
  ot->ui = node_group_lock_toggle_ui;
  ot->poll = node_group_operator_active_poll;

  /* Do not push undo: copying a large node tree into the undo stack is slow,
   * and storing a lock password in undo is also undesirable. */
  ot->flag = 0;

  PropertyRNA *prop = RNA_def_string(
      ot->srna, "password", nullptr, 128, "Password", "Password for this node group");
  RNA_def_property_subtype(prop, PROP_PASSWORD);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  ot->prop = prop;

  prop = RNA_def_string(ot->srna,
                        "confirm_password",
                        nullptr,
                        128,
                        "Confirm",
                        "Re-enter the password to confirm");
  RNA_def_property_subtype(prop, PROP_PASSWORD);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);

  prop = RNA_def_boolean(ot->srna, "removing_lock", false, "Removing Lock", "");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Enter Locked Node Group (Password)
 * \{ */

static wmOperatorStatus node_group_password_enter_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  bNodeTree *ngroup = node_group_password_op_tree(C, op);
  if (!snode || !ngroup) {
    return OPERATOR_CANCELLED;
  }

  const std::string password = RNA_string_get(op->ptr, "password");
  Main *bmain = CTX_data_main(C);
  if (!bke::node_tree_lock_unlock_contents(*bmain, *ngroup, password, op->reports)) {
    return OPERATOR_CANCELLED;
  }
  bke::node_tree_lock_session_acquire(*ngroup);

  node_group_mark_decrypted_session(*snode, *ngroup);
  if (snode->runtime) {
    snode->runtime->password_prompt_shown = false;
  }

  if (RNA_boolean_get(op->ptr, "do_push")) {
    const std::string node_name = RNA_string_get(op->ptr, "node_name");
    bNode *gnode = nullptr;
    if (!node_name.empty() && snode->edittree) {
      gnode = bke::node_find_node_by_name(*snode->edittree, node_name);
    }
    ED_node_tree_push(region, snode, ngroup, gnode);
  }

  WM_event_add_notifier(C, NC_SCENE | ND_NODES, nullptr);
  WM_event_add_notifier(C, NC_NODE | ND_DISPLAY, nullptr);
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_NODE, nullptr);
  return OPERATOR_FINISHED;
}

static wmOperatorStatus node_group_password_enter_invoke(bContext *C,
                                                         wmOperator *op,
                                                         const wmEvent * /*event*/)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  bNodeTree *ngroup = node_group_password_op_tree(C, op);
  if (!ngroup) {
    return OPERATOR_CANCELLED;
  }
  if (!bke::node_tree_is_locked(*ngroup) ||
      (snode && node_tree_session_is_unlocked(*snode, *ngroup)))
  {
    return node_group_password_enter_exec(C, op);
  }
  return WM_operator_props_dialog_popup(C,
                                        op,
                                        320,
                                        IFACE_("Node Group Locked"),
                                        IFACE_("Unlock"),
                                        false,
                                        IFACE_("Enter the password to view this node group."),
                                        true);
}

static void node_group_password_enter_cancel(bContext *C, wmOperator *op)
{
  if (RNA_boolean_get(op->ptr, "do_push")) {
    /* Never entered the group. */
    return;
  }
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  if (snode && snode->treepath.first() != snode->treepath.last()) {
    ED_node_tree_pop(region, snode);
  }
}

static void node_group_password_enter_ui(bContext * /*C*/, wmOperator *op)
{
  ui::Layout &layout = *op->layout;
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(op->ptr, "password", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

void NODE_OT_group_password_enter(wmOperatorType *ot)
{
  ot->name = "Enter Node Group Password";
  ot->description = "Enter the password to view a locked node group";
  ot->idname = "NODE_OT_group_password_enter";

  ot->invoke = node_group_password_enter_invoke;
  ot->exec = node_group_password_enter_exec;
  ot->cancel = node_group_password_enter_cancel;
  ot->ui = node_group_password_enter_ui;
  ot->poll = node_group_operator_active_poll;

  PropertyRNA *prop = RNA_def_string(
      ot->srna, "password", nullptr, 128, "Password", "Password for this node group");
  RNA_def_property_subtype(prop, PROP_PASSWORD);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  ot->prop = prop;

  prop = RNA_def_string(ot->srna, "node_tree_name", nullptr, MAX_ID_NAME - 2, "Node Tree", "");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_string(ot->srna, "node_name", nullptr, 64, "Node Name", "");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_boolean(
      ot->srna, "do_push", false, "Push", "Enter the node group after unlocking");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Unlock Locked Groups For Evaluation (File Load)
 * \{ */

static bool unlock_locked_groups_poll(bContext *C)
{
  Main *bmain = CTX_data_main(C);
  if (!bmain) {
    return false;
  }
  for (bNodeTree &tree : bmain->nodetrees) {
    if (bke::node_tree_contents_sealed(tree)) {
      return true;
    }
  }
  return false;
}

static wmOperatorStatus unlock_locked_groups_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  if (!bmain) {
    return OPERATOR_CANCELLED;
  }
  const std::string password = RNA_string_get(op->ptr, "password");
  if (password.empty()) {
    BKE_report(op->reports, RPT_ERROR, "Password cannot be empty");
    return OPERATOR_CANCELLED;
  }

  int unlocked = 0;
  int failed = 0;
  for (bNodeTree &tree : bmain->nodetrees) {
    if (!bke::node_tree_contents_sealed(tree)) {
      continue;
    }
    if (bke::node_tree_lock_unlock_contents(*bmain, tree, password, nullptr)) {
      /* Decrypt for modifiers / evaluation only. Do not mark the editor session
       * unlocked: viewing internals still requires entering the group password. */
      unlocked++;
      DEG_id_tag_update(&tree.id, ID_RECALC_NTREE_OUTPUT);
    }
    else {
      failed++;
    }
  }

  if (unlocked == 0) {
    BKE_report(op->reports, RPT_ERROR, "Incorrect password");
    return OPERATOR_CANCELLED;
  }

  BKE_ntree_update(*bmain);
  DEG_relations_tag_update(bmain);
  WM_event_add_notifier(C, NC_SCENE | ND_NODES, nullptr);
  WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, nullptr);
  WM_event_add_notifier(C, NC_NODE | ND_DISPLAY, nullptr);
  if (failed > 0) {
    BKE_report(op->reports,
               RPT_WARNING,
               "Some locked node groups use a different password and were not decrypted");
  }
  return OPERATOR_FINISHED;
}

static wmOperatorStatus unlock_locked_groups_invoke(bContext *C,
                                                    wmOperator *op,
                                                    const wmEvent * /*event*/)
{
  if (!unlock_locked_groups_poll(C)) {
    return OPERATOR_CANCELLED;
  }
  return WM_operator_props_dialog_popup(
      C,
      op,
      320,
      IFACE_("Locked Node Groups"),
      IFACE_("Unlock"),
      false,
      IFACE_("Enter the password so locked node groups can compute. "
             "Viewing the node graph still requires the password."),
      true);
}

static void unlock_locked_groups_ui(bContext * /*C*/, wmOperator *op)
{
  ui::Layout &layout = *op->layout;
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(op->ptr, "password", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

void NODE_OT_unlock_locked_groups(wmOperatorType *ot)
{
  ot->name = "Unlock Locked Node Groups";
  ot->description =
      "Decrypt locked node groups so they can evaluate; does not open them in the editor";
  ot->idname = "NODE_OT_unlock_locked_groups";

  ot->invoke = unlock_locked_groups_invoke;
  ot->exec = unlock_locked_groups_exec;
  ot->ui = unlock_locked_groups_ui;
  ot->poll = unlock_locked_groups_poll;
  ot->flag = 0;

  PropertyRNA *prop = RNA_def_string(
      ot->srna, "password", nullptr, 128, "Password", "Password for locked node groups");
  RNA_def_property_subtype(prop, PROP_PASSWORD);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  ot->prop = prop;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Edit Group Operator
 * \{ */

static wmOperatorStatus node_group_edit_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  const UString node_idname = node_group_idname(C);
  const bool exit = RNA_boolean_get(op->ptr, "exit");

  ED_preview_kill_jobs(CTX_wm_manager(C), CTX_data_main(C));

  bNode *gnode = node_group_get_active(C, node_idname);

  /* Special hosts that store a nested tree on node->id but are not standard group nodes. */
  if (!gnode && snode->edittree && !exit) {
    bNode *active = bke::node_get_active(*snode->edittree);
    if (active && (nodes::image_points::is_image_import_nested_host(*active) ||
                   active->is_type("CompositorNodeImageProcess"_ustr)))
    {
      gnode = active;
    }
  }

  if (gnode && !exit) {
    bNodeTree *ngroup = nullptr;
    if (nodes::image_points::is_image_import_nested_host(*gnode)) {
      Main *bmain = CTX_data_main(C);
      if (bmain) {
        ngroup = nodes::image_points::ensure_import_nested_geometry_tree(*bmain, *gnode);
      }
    }
    else if (gnode->is_type("CompositorNodeImageProcess"_ustr)) {
      /* Nested GPU Texture Editor (ImageNodeTree) group. */
      if (gnode->id && GS(gnode->id->name) == ID_NT) {
        bNodeTree *tree = id_cast<bNodeTree *>(gnode->id);
        if (tree && tree->type == NTREE_IMAGE && !ID_MISSING(tree)) {
          ngroup = tree;
        }
      }
    }
    else {
      ngroup = id_cast<bNodeTree *>(gnode->id);
    }

    if (ngroup) {
      const wmOperatorStatus status = node_group_enter_or_prompt(C, region, snode, ngroup, gnode);
      WM_event_add_notifier(C, NC_SCENE | ND_NODES, nullptr);
      WM_event_add_notifier(C, NC_NODE | ND_NODE_GIZMO, nullptr);
      return status;
    }
  }
  else {
    ED_node_tree_pop(region, snode);
  }

  WM_event_add_notifier(C, NC_SCENE | ND_NODES, nullptr);
  WM_event_add_notifier(C, NC_NODE | ND_NODE_GIZMO, nullptr);

  return OPERATOR_FINISHED;
}

void NODE_OT_group_edit(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Edit Group";
  ot->description = "Edit node group";
  ot->idname = "NODE_OT_group_edit";

  /* API callbacks. */
  ot->exec = node_group_edit_exec;
  ot->poll = node_group_operator_active_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  PropertyRNA *prop = RNA_def_boolean(ot->srna, "exit", false, "Exit", "");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Enter group at cursor, or exit when not hovering any node.
 * \{ */

static wmOperatorStatus node_group_enter_exit_invoke(bContext *C,
                                                     wmOperator * /*op*/,
                                                     const wmEvent *event)
{
  SpaceNode &snode = *CTX_wm_space_node(C);
  ARegion &region = *CTX_wm_region(C);

  /* Don't interfere when the mouse is interacting with some button. See #147282. */
  if (ISMOUSE_BUTTON(event->type) && ui::but_find_mouse_over(&region, event)) {
    return OPERATOR_PASS_THROUGH | OPERATOR_CANCELLED;
  }

  float2 cursor;
  ui::view2d_region_to_view(&region.v2d, event->mval[0], event->mval[1], &cursor.x, &cursor.y);
  bNode *node = node_under_mouse_get(snode, cursor);

  if (!node || node->is_frame()) {
    ED_node_tree_pop(&region, &snode);
    return OPERATOR_FINISHED;
  }
  /* GPU Texture Editor Import Points / Import Geo: nest GeometryNodeTree (path only). */
  if (blender::nodes::image_points::is_image_import_nested_host(*node)) {
    Main *bmain = CTX_data_main(C);
    if (!bmain || !snode.nodetree || snode.nodetree->type != NTREE_IMAGE) {
      return OPERATOR_PASS_THROUGH;
    }
    bNodeTree *geo = blender::nodes::image_points::ensure_import_nested_geometry_tree(*bmain,
                                                                                     *node);
    if (!geo || node->id != &geo->id) {
      return OPERATOR_PASS_THROUGH;
    }
    /* Path push only — do not change tree_idname (stay GPU Texture Editor). */
    return node_group_enter_or_prompt(C, &region, &snode, geo, node);
  }
  /* Compositor → GPU Texture Editor group (Tab / double-click). */
  if (node->is_type("CompositorNodeImageProcess"_ustr)) {
    if (!node->id || GS(node->id->name) != ID_NT) {
      return OPERATOR_PASS_THROUGH;
    }
    bNodeTree *img_group = id_cast<bNodeTree *>(node->id);
    if (!img_group || img_group->type != NTREE_IMAGE || ID_MISSING(img_group)) {
      return OPERATOR_PASS_THROUGH;
    }
    return node_group_enter_or_prompt(C, &region, &snode, img_group, node);
  }
  if (!node->is_group()) {
    return OPERATOR_PASS_THROUGH;
  }
  if (node->is_custom_group()) {
    return OPERATOR_PASS_THROUGH;
  }
  bNodeTree *group = id_cast<bNodeTree *>(node->id);
  if (!group || ID_MISSING(group)) {
    return OPERATOR_PASS_THROUGH;
  }
  return node_group_enter_or_prompt(C, &region, &snode, group, node);
}

void NODE_OT_group_enter_exit(wmOperatorType *ot)
{
  ot->name = "Enter/Exit Group";
  ot->description = "Enter or exit node group based on cursor location";
  ot->idname = "NODE_OT_group_enter_exit";

  ot->invoke = node_group_enter_exit_invoke;
  ot->poll = node_group_operator_active_poll;

  ot->flag = OPTYPE_REGISTER;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Ungroup Operator
 * \{ */

/**
 * \return True if successful.
 */
static void node_group_ungroup(bContext &C, bNodeTree &ntree, bNode &group_node)
{
  Main &bmain = *CTX_data_main(&C);
  NodeSetInterfaceParams params;
  params.skip_hidden = false;

  const bNodeTree &ngroup = *reinterpret_cast<const bNodeTree *>(group_node.id);
  const NodeTreeInterfaceMapping io_mapping = map_group_node_interface(params, ntree, group_node);

  const NodeSetCopy copied_nodes = NodeSetCopy::from_predicate(
      bmain,
      ngroup,
      [&](const bNode &node) {
        if (node.is_group_input() || node.is_group_output()) {
          return false;
        }
        return true;
      },
      ntree);
  const InterfaceProxyNodes proxy_nodes = connect_copied_nodes_to_external_sockets(
      C, ngroup, copied_nodes, io_mapping, &group_node);

  /* Center nodes on the bounds of the original group node. */
  if (const std::optional<Bounds<float2>> bounds = node_location_bounds(Span{&group_node})) {
    const float2 center = bounds->center();
    for (bNode *node : copied_nodes.node_map().values()) {
      node->location[0] += center[0];
      node->location[1] += center[1];
    }
    for (bNode *node : proxy_nodes.values()) {
      node->location[0] += center[0];
      node->location[1] += center[1];
    }
  }
  /* Attach to the same parent as the group node. */
  if (group_node.parent) {
    for (bNode *node : copied_nodes.node_map().values()) {
      node->parent = group_node.parent;
    }
    for (bNode *node : proxy_nodes.values()) {
      node->parent = group_node.parent;
    }
  }

  update_nested_node_refs_after_ungroup(ntree, group_node, copied_nodes);

  /* Delete the original group instance. */
  bke::node_remove_node(&bmain, ntree, group_node, true);

  /* Select ungrouped nodes. */
  for (bNode *node : copied_nodes.node_map().values()) {
    bke::node_set_selected(*node, true);
  }
  for (bNode *node : proxy_nodes.values()) {
    bke::node_set_selected(*node, true);
  }
}

static wmOperatorStatus node_group_ungroup_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  SpaceNode *snode = CTX_wm_space_node(C);
  const UString node_idname = node_group_idname(C);

  ED_preview_kill_jobs(CTX_wm_manager(C), bmain);

  Vector<bNode *> nodes_to_ungroup;
  for (bNode *node : snode->edittree->all_nodes()) {
    if (node->is_selected()) {
      if (node->idname == node_idname) {
        if (node->id != nullptr) {
          nodes_to_ungroup.append(node);
        }
      }
    }
  }
  if (nodes_to_ungroup.is_empty()) {
    return OPERATOR_CANCELLED;
  }

  for (bNode *node : nodes_to_ungroup) {
    bNodeTree *ngroup = id_cast<bNodeTree *>(node->id);
    if (ngroup && bke::node_tree_is_locked(*ngroup)) {
      BKE_reportf(op->reports,
                  RPT_ERROR,
                  "Cannot ungroup locked node group '%s'",
                  ngroup->id.name + 2);
      return OPERATOR_CANCELLED;
    }
  }

  node_deselect_all(*snode->edittree);
  for (bNode *node : nodes_to_ungroup) {
    node_group_ungroup(*C, *snode->edittree, *node);
  }
  WM_event_handling_break(*C);
  BKE_main_ensure_invariants(*CTX_data_main(C));
  return OPERATOR_FINISHED;
}

void NODE_OT_group_ungroup(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Ungroup";
  ot->description = "Ungroup selected nodes";
  ot->idname = "NODE_OT_group_ungroup";

  /* API callbacks. */
  ot->exec = node_group_ungroup_exec;
  ot->poll = node_group_operator_editable;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Separate Operator
 * \{ */

/**
 * \return True if successful.
 */
static bool node_group_separate_selected(
    Main &bmain, bNodeTree &ntree, bNodeTree &ngroup, const float2 &offset, const bool make_copy)
{
  node_deselect_all(ntree);

  /* Add selected nodes into the ntree, ignoring interface nodes. */
  VectorSet<bNode *> nodes_to_move = get_selected_nodes(ngroup);
  nodes_to_move.remove_if(
      [](const bNode *node) { return node->is_group_input() || node->is_group_output(); });

  NodeSetCopy copied_nodes = NodeSetCopy::from_nodes(
      bmain, ngroup, nodes_to_move.as_span(), ntree);
  for (bNode *node : copied_nodes.node_map().values()) {
    node->location[0] += offset[0];
    node->location[1] += offset[1];
  }

  if (!make_copy) {
    for (bNode *node : nodes_to_move) {
      bke::node_remove_node(&bmain, ngroup, *node, true);
    }
  }

  return true;
}

enum eNodeGroupSeparateType {
  NODE_GS_COPY,
  NODE_GS_MOVE,
};

/* Operator Property */
static const EnumPropertyItem node_group_separate_types[] = {
    {NODE_GS_COPY, "COPY", 0, "Copy", "Copy to parent node tree, keep group intact"},
    {NODE_GS_MOVE, "MOVE", 0, "Move", "Move to parent node tree, remove from group"},
    {0, nullptr, 0, nullptr, nullptr},
};

static wmOperatorStatus node_group_separate_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  ARegion *region = CTX_wm_region(C);
  SpaceNode *snode = CTX_wm_space_node(C);
  int type = RNA_enum_get(op->ptr, "type");

  ED_preview_kill_jobs(CTX_wm_manager(C), bmain);

  /* are we inside of a group? */
  bNodeTree *ngroup = snode->edittree;
  bNodeTree *nparent = ED_node_tree_get(snode, 1);
  if (!nparent) {
    BKE_report(op->reports, RPT_WARNING, "Not inside node group");
    return OPERATOR_CANCELLED;
  }
  /* get node tree offset */
  const float2 offset = space_node_group_offset(*snode);

  switch (type) {
    case NODE_GS_COPY:
      if (!node_group_separate_selected(*bmain, *nparent, *ngroup, offset, true)) {
        BKE_report(op->reports, RPT_WARNING, "Cannot separate nodes");
        return OPERATOR_CANCELLED;
      }
      break;
    case NODE_GS_MOVE:
      if (!node_group_separate_selected(*bmain, *nparent, *ngroup, offset, false)) {
        BKE_report(op->reports, RPT_WARNING, "Cannot separate nodes");
        return OPERATOR_CANCELLED;
      }
      break;
  }

  /* switch to parent tree */
  ED_node_tree_pop(region, snode);

  BKE_main_ensure_invariants(*CTX_data_main(C));

  return OPERATOR_FINISHED;
}

static wmOperatorStatus node_group_separate_invoke(bContext *C,
                                                   wmOperator * /*op*/,
                                                   const wmEvent * /*event*/)
{
  ui::PopupMenu *pup = ui::popup_menu_begin(
      C, CTX_IFACE_(BLT_I18NCONTEXT_OPERATOR_DEFAULT, "Separate"), ICON_NONE);
  ui::Layout *layout = popup_menu_layout(pup);

  layout->operator_context_set(wm::OpCallContext::ExecDefault);
  PointerRNA op_ptr = layout->op("NODE_OT_group_separate", IFACE_("Copy"), ICON_NONE);
  RNA_enum_set(&op_ptr, "type", NODE_GS_COPY);
  op_ptr = layout->op("NODE_OT_group_separate", IFACE_("Move"), ICON_NONE);
  RNA_enum_set(&op_ptr, "type", NODE_GS_MOVE);

  popup_menu_end(C, pup);

  return OPERATOR_INTERFACE;
}

void NODE_OT_group_separate(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Separate";
  ot->description = "Separate selected nodes from the node group";
  ot->idname = "NODE_OT_group_separate";

  /* API callbacks. */
  ot->invoke = node_group_separate_invoke;
  ot->exec = node_group_separate_exec;
  ot->poll = node_group_operator_editable;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_enum(ot->srna, "type", node_group_separate_types, NODE_GS_COPY, "Type", "");
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Make Group Operator
 * \{ */

static VectorSet<bNode *> get_nodes_to_group(bNodeTree &node_tree, bNode *group_node)
{
  VectorSet<bNode *> nodes_to_group = get_selected_nodes(node_tree);
  nodes_to_group.remove_if(
      [](bNode *node) { return node->is_group_input() || node->is_group_output(); });
  nodes_to_group.remove(group_node);
  return nodes_to_group;
}

static bool node_group_make_test_selected(bNodeTree &ntree,
                                          const VectorSet<bNode *> &nodes_to_group,
                                          const StringRef ntree_idname,
                                          ReportList &reports)
{
  if (nodes_to_group.is_empty()) {
    return false;
  }
  /* make a local pseudo node tree to pass to the node poll functions */
  bNodeTree *ngroup = bke::node_tree_add_tree(nullptr, "Pseudo Node Group", ntree_idname);
  BLI_SCOPED_DEFER([&]() {
    bke::node_tree_free_tree(*ngroup);
    MEM_delete(ngroup);
  });

  /* check poll functions for selected nodes */
  for (bNode *node : nodes_to_group) {
    const char *disabled_hint = nullptr;
    if (node->typeinfo->poll_instance &&
        !node->typeinfo->poll_instance(node, ngroup, &disabled_hint))
    {
      if (disabled_hint) {
        BKE_reportf(&reports,
                    RPT_WARNING,
                    "Cannot add node '%s' in a group:\n  %s",
                    node->name,
                    disabled_hint);
      }
      else {
        BKE_reportf(&reports, RPT_WARNING, "Cannot add node '%s' in a group", node->name);
      }
      return false;
    }
  }

  /* check if all connections are OK, no unselected node has both
   * inputs and outputs to a selection */
  ntree.ensure_topology_cache();
  for (bNode *node : ntree.all_nodes()) {
    if (nodes_to_group.contains(node)) {
      continue;
    }
    auto sockets_connected_to_group = [&](const Span<bNodeSocket *> sockets) {
      for (const bNodeSocket *socket : sockets) {
        for (const bNodeSocket *other_socket : socket->directly_linked_sockets()) {
          if (nodes_to_group.contains(const_cast<bNode *>(&other_socket->owner_node()))) {
            return true;
          }
        }
      }
      return false;
    };
    if (sockets_connected_to_group(node->input_sockets()) &&
        sockets_connected_to_group(node->output_sockets()))
    {
      return false;
    }
  }
  /* Check if zone pairs are fully selected.
   * Zone input or output nodes can only be grouped together with the paired node. */
  for (const bke::bNodeZoneType *zone_type : bke::all_zone_types()) {
    if (zone_type == nullptr) {
      continue;
    }
    for (bNode *input_node : ntree.nodes_by_type(zone_type->input_idname)) {
      if (input_node == nullptr) {
        continue;
      }
      if (bNode *output_node = zone_type->get_corresponding_output(ntree, *input_node)) {
        const bool input_selected = nodes_to_group.contains(input_node);
        const bool output_selected = nodes_to_group.contains(output_node);
        if (input_selected && !output_selected) {
          BKE_reportf(&reports,
                      RPT_WARNING,
                      "Cannot add zone input node '%s' to a group without its paired output '%s'",
                      input_node->name,
                      output_node->name);
          return false;
        }
        if (output_selected && !input_selected) {
          BKE_reportf(&reports,
                      RPT_WARNING,
                      "Cannot add zone output node '%s' to a group without its paired input '%s'",
                      output_node->name,
                      input_node->name);
          return false;
        }
      }
    }
  }

  return true;
}

static void node_group_make_insert_selected(const bContext &C,
                                            bNodeTree &ntree,
                                            bNode *gnode,
                                            const Span<bNode *> nodes)
{
  Main &bmain = *CTX_data_main(&C);
  bNodeTree &group = *reinterpret_cast<bNodeTree *>(gnode->id);

  NodeSetInterfaceParams params;
  /* TODO This is inconsistent with the single-node wrapper case, which does expose hidden
   * sockets and only hides them on the group node instance. */
  params.skip_hidden = true;
  /* Expose only connected sockets if there is more than one node. */
  params.skip_unconnected = (nodes.size() > 1);
  /* Share external connections if a socket has multiple links. */
  params.use_unique_input = false;
  params.use_unique_output = false;
  const NodeTreeInterfaceMapping io_mapping = build_node_set_interface(
      params, ntree, nodes, group);

  /* Copy nodes into the group. */
  node_deselect_all(group);
  const NodeSetCopy copied_nodes = NodeSetCopy::from_nodes(bmain, ntree, nodes, group);
  /* Connect exposed sockets to group input/output nodes. */
  connect_copied_nodes_to_interface(C, copied_nodes, io_mapping);

  update_nested_node_refs_after_moving_nodes_into_group(ntree, *gnode, copied_nodes);
  BKE_main_ensure_invariants(bmain, Span<ID *>{&group.id});

  /* Connect the group node to external sockets. */
  connect_group_node_to_external_sockets(*gnode, io_mapping);

  /* Remove original nodes from the tree, everything has been copied to the group. */
  for (bNode *node : nodes) {
    bke::node_remove_node(&bmain, ntree, *node, true);
  }

  BKE_main_ensure_invariants(bmain);
}

static bNode *node_group_make_from_nodes(const bContext &C,
                                         bNodeTree &ntree,
                                         const Span<bNode *> nodes_to_group,
                                         const UString ntype,
                                         const StringRef ntreetype)
{
  Main *bmain = CTX_data_main(&C);

  /* New node-tree. */
  bNodeTree *ngroup = bke::node_tree_add_tree(bmain, "NodeGroup", ntreetype);
  BKE_id_move_to_same_lib(*bmain, ngroup->id, ntree.id);

  /* make group node */
  bNode *gnode = bke::node_add_node(&C, ntree, ntype);
  gnode->id = id_cast<ID *>(ngroup);

  if (const std::optional<Bounds<float2>> bounds = node_location_bounds(nodes_to_group)) {
    gnode->location[0] = nearest_node_grid_coord(bounds->center()[0]);
    gnode->location[1] = nearest_node_grid_coord(bounds->center()[1]);
  }
  if (bNode *parent = ed::space_node::find_common_parent_node(nodes_to_group)) {
    gnode->parent = parent;
  }

  node_group_make_insert_selected(C, ntree, gnode, nodes_to_group);

  return gnode;
}

static bNode *node_group_make_from_node_declaration(bContext &C,
                                                    bNodeTree &ntree,
                                                    bNode &src_node,
                                                    const UString node_idname)
{
  Main &bmain = *CTX_data_main(&C);

  bNodeTree *wrapper_group = bke::node_tree_add_tree(
      &bmain, bke::node_label(ntree, src_node), ntree.idname);
  wrapper_group->color_tag = int(bke::node_color_tag(src_node));
  if (!src_node.is_reroute()) {
    wrapper_group->default_group_node_width = src_node.width;
  }

  NodeSetInterfaceParams params;
  /* Hidden sockets are exposed but hidden on the group node instance. */
  params.skip_hidden = false;
  /* Expose all sockets even if unconnected. */
  params.skip_unconnected = false;
  const NodeTreeInterfaceMapping io_mapping = build_node_declaration_interface(
      params, src_node, *wrapper_group);

  const NodeSetCopy copied_nodes = NodeSetCopy::from_nodes(
      bmain, ntree, {&src_node}, *wrapper_group);
  connect_copied_nodes_to_interface(C, copied_nodes, io_mapping);

  BKE_main_ensure_invariants(bmain, wrapper_group->id);

  /* Create a group node. */
  bNode *gnode = bke::node_add_node(&C, ntree, node_idname);
  STRNCPY_UTF8(gnode->name, BKE_id_name(wrapper_group->id));
  bke::node_unique_name(ntree, *gnode);

  /* Assign the newly created wrapper group to the new group node. */
  gnode->id = &wrapper_group->id;

  /* Position node exactly where the old node was. */
  gnode->parent = src_node.parent;

  if (!src_node.is_reroute()) {
    gnode->width = std::max<float>(src_node.width, bke::NodeWidth::GroupMin);
  }
  copy_v2_v2(gnode->location, src_node.location);

  BKE_main_ensure_invariants(bmain);
  ntree.ensure_topology_cache();

  connect_group_node_to_external_sockets(*gnode, io_mapping);

  /* Remove the old node because it has been replaced. Use the name of the removed node for the
   * new group node. This also keeps animation data working. */
  std::string old_node_name = src_node.name;
  bke::node_remove_node(&bmain, ntree, src_node, true, false);
  STRNCPY(gnode->name, old_node_name.c_str());

  /* Clear already created nested node refs to create new stable ones below. */
  MEM_SAFE_DELETE(wrapper_group->nested_node_refs);
  wrapper_group->nested_node_refs_num = 0;
  update_nested_node_refs_after_moving_nodes_into_group(ntree, *gnode, copied_nodes);

  BKE_ntree_update_tag_node_property(&ntree, gnode);
  BKE_main_ensure_invariants(bmain);
  return gnode;
}

static wmOperatorStatus node_group_make_exec(bContext *C, wmOperator *op)
{
  ARegion &region = *CTX_wm_region(C);
  SpaceNode &snode = *CTX_wm_space_node(C);
  bNodeTree &ntree = *snode.edittree;
  const StringRef ntree_idname = group_ntree_idname(C);
  const UString node_idname = node_group_idname(C);
  Main *bmain = CTX_data_main(C);

  ED_preview_kill_jobs(CTX_wm_manager(C), CTX_data_main(C));

  VectorSet<bNode *> nodes_to_group = get_nodes_to_group(ntree, nullptr);
  if (!node_group_make_test_selected(ntree, nodes_to_group, ntree_idname, *op->reports)) {
    return OPERATOR_CANCELLED;
  }

  bNode *gnode = nullptr;
  if (nodes_to_group.size() == 1 && nodes_to_group[0]->declaration()) {
    gnode = node_group_make_from_node_declaration(*C, ntree, *nodes_to_group[0], node_idname);
  }
  else {
    gnode = node_group_make_from_nodes(
        *C, ntree, std::move(nodes_to_group), node_idname, ntree_idname);
  }

  if (gnode) {
    bNodeTree *ngroup = id_cast<bNodeTree *>(gnode->id);

    bke::node_set_active(ntree, *gnode);
    if (ngroup) {
      ED_node_tree_push(&region, &snode, ngroup, gnode);
    }
  }

  WM_event_add_notifier(C, NC_NODE | NA_ADDED, nullptr);
  WM_event_handling_break(*C);
  /* We broke relations in node tree, need to rebuild them in the graphs. */
  DEG_relations_tag_update(bmain);

  return OPERATOR_FINISHED;
}

void NODE_OT_group_make(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Make Group";
  ot->description = "Make group from selected nodes";
  ot->idname = "NODE_OT_group_make";

  /* API callbacks. */
  ot->exec = node_group_make_exec;
  ot->poll = node_group_operator_editable;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Group Insert Operator
 * \{ */

static wmOperatorStatus node_group_insert_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  bNodeTree *ntree = snode->edittree;
  const UString node_idname = node_group_idname(C);

  ED_preview_kill_jobs(CTX_wm_manager(C), CTX_data_main(C));

  bNode *gnode = node_group_get_active(C, node_idname);
  if (!gnode || !gnode->id) {
    return OPERATOR_CANCELLED;
  }

  bNodeTree *ngroup = reinterpret_cast<bNodeTree *>(gnode->id);
  if (bke::node_tree_is_locked(*ngroup) && !node_tree_session_is_unlocked(*snode, *ngroup)) {
    BKE_report(op->reports, RPT_ERROR, "Cannot insert into a locked node group");
    return OPERATOR_CANCELLED;
  }
  VectorSet<bNode *> nodes_to_group = get_nodes_to_group(*ntree, gnode);

  /* Make sure that there won't be a node group containing itself afterwards. */
  for (const bNode *group : nodes_to_group) {
    if (!group->is_group() || group->id == nullptr) {
      continue;
    }
    if (bke::node_tree_contains_tree(*reinterpret_cast<bNodeTree *>(group->id), *ngroup)) {
      BKE_reportf(
          op->reports, RPT_WARNING, "Cannot insert group '%s' in '%s'", group->name, gnode->name);
      return OPERATOR_CANCELLED;
    }
  }

  if (!node_group_make_test_selected(*ntree, nodes_to_group, ngroup->idname, *op->reports)) {
    return OPERATOR_CANCELLED;
  }

  node_group_make_insert_selected(*C, *ntree, gnode, nodes_to_group);

  bke::node_set_active(*ntree, *gnode);
  ED_node_tree_push(region, snode, ngroup, gnode);

  return OPERATOR_FINISHED;
}

void NODE_OT_group_insert(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Group Insert";
  ot->description = "Insert selected nodes into a node group";
  ot->idname = "NODE_OT_group_insert";

  /* API callbacks. */
  ot->exec = node_group_insert_exec;
  ot->poll = node_group_operator_editable;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Set Default Group Width Operator
 * \{ */

static bool node_default_group_width_set_poll(bContext *C)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode) {
    return false;
  }
  bNodeTree *ntree = snode->edittree;
  if (!ntree) {
    return false;
  }
  if (!ID_IS_EDITABLE(ntree)) {
    return false;
  }
  if (snode->nodetree == snode->edittree) {
    /* Top-level node group does not have enough context to set the node width. */
    CTX_wm_operator_poll_msg_set(C, "There is no parent group node in this context");
    return false;
  }
  return true;
}

static wmOperatorStatus node_default_group_width_set_exec(bContext *C, wmOperator * /*op*/)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  bNodeTree *ntree = snode->edittree;

  bNodeTreePath *last_path_item = snode->treepath.last();
  bNodeTreePath *parent_path_item = last_path_item->prev;
  if (!parent_path_item) {
    return OPERATOR_CANCELLED;
  }
  bNodeTree *parent_ntree = parent_path_item->nodetree;
  if (!parent_ntree) {
    return OPERATOR_CANCELLED;
  }
  parent_ntree->ensure_topology_cache();
  bNode *parent_node = bke::node_find_node_by_name(*parent_ntree, last_path_item->node_name);
  if (!parent_node) {
    return OPERATOR_CANCELLED;
  }
  ntree->default_group_node_width = parent_node->width;
  WM_event_add_notifier(C, NC_NODE | NA_EDITED, nullptr);
  return OPERATOR_CANCELLED;
}

void NODE_OT_default_group_width_set(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Set Default Group Node Width";
  ot->description = "Set the width based on the parent group node in the current context";
  ot->idname = "NODE_OT_default_group_width_set";

  /* API callbacks. */
  ot->exec = node_default_group_width_set_exec;
  ot->poll = node_default_group_width_set_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

}  // namespace blender::ed::space_node
