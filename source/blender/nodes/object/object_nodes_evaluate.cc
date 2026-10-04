/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_set.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_vector.hh"

#include "DNA_material_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_windowmanager_types.h"

#include "BKE_callbacks.hh"
#include "BKE_context.hh"
#include "BKE_global.hh"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
#include "BKE_main.hh"
#include "BKE_material.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_object.hh"
#include "BKE_scene.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "NOD_object.hh"

namespace blender {

static thread_local bool g_object_nodes_evaluating = false;

struct ObjectEvalGuard {
  ObjectEvalGuard()
  {
    g_object_nodes_evaluating = true;
  }
  ~ObjectEvalGuard()
  {
    g_object_nodes_evaluating = false;
  }
};

static bool node_is(const bNode &node, const char *idname)
{
  return STREQ(node.idname, idname);
}

static const bNodeSocket *find_input(const bNode &node, const StringRef identifier)
{
  return bke::node_find_socket(node, SOCK_IN, UString(identifier));
}

static const bNodeSocket *find_linked_from(const bNodeTree &ntree, const bNodeSocket &input)
{
  for (const bNodeLink &link : ntree.links) {
    if (link.tosock != &input) {
      continue;
    }
    if (link.flag & NODE_LINK_MUTED) {
      continue;
    }
    if (link.fromnode && (link.fromnode->flag & NODE_MUTED)) {
      continue;
    }
    return link.fromsock;
  }
  return nullptr;
}

static Object *socket_object_value(const bNodeTree &ntree,
                                   const bNodeSocket *socket,
                                   const Map<const bNodeSocket *, Object *> &values)
{
  if (socket == nullptr) {
    return nullptr;
  }
  if (const bNodeSocket *from = find_linked_from(ntree, *socket)) {
    if (Object *const *ob = values.lookup_ptr(from)) {
      return *ob;
    }
  }
  if (socket->type == SOCK_OBJECT && socket->default_value) {
    Object *ob = static_cast<bNodeSocketValueObject *>(socket->default_value)->value;
    if (ob && !ID_MISSING(&ob->id)) {
      return ob;
    }
  }
  return nullptr;
}

static float3 socket_vector_value(const bNodeTree &ntree,
                                  const bNodeSocket *socket,
                                  const float3 fallback)
{
  if (socket == nullptr) {
    return fallback;
  }
  /* Object Editor writes use the socket default (or a linked constant default). */
  UNUSED_VARS(ntree);
  if (socket->type == SOCK_VECTOR && socket->default_value) {
    const auto *value = static_cast<const bNodeSocketValueVector *>(socket->default_value);
    return float3(value->value);
  }
  if (socket->type == SOCK_ROTATION && socket->default_value) {
    const auto *value = static_cast<const bNodeSocketValueRotation *>(socket->default_value);
    return float3(value->value_euler);
  }
  return fallback;
}

static bool socket_bool_value(const bNodeSocket *socket, const bool fallback)
{
  if (socket == nullptr || socket->type != SOCK_BOOLEAN || socket->default_value == nullptr) {
    return fallback;
  }
  return static_cast<const bNodeSocketValueBoolean *>(socket->default_value)->value != 0;
}

static float4 socket_color_value(const bNodeSocket *socket, const float4 fallback)
{
  if (socket == nullptr || socket->type != SOCK_RGBA || socket->default_value == nullptr) {
    return fallback;
  }
  return float4(static_cast<const bNodeSocketValueRGBA *>(socket->default_value)->value);
}

static bool node_reaches_type(const bNodeTree &ntree,
                              const bNode &from_node,
                              const char *type_idname)
{
  Vector<const bNode *> stack;
  Set<const bNode *> visited;
  stack.append(&from_node);
  while (!stack.is_empty()) {
    const bNode *cur = stack.pop_last();
    if (!visited.add(cur)) {
      continue;
    }
    if (cur != &from_node && node_is(*cur, type_idname)) {
      return true;
    }
    for (const bNodeLink &link : ntree.links) {
      if (link.fromnode != cur || link.tonode == nullptr) {
        continue;
      }
      if (link.flag & NODE_LINK_MUTED) {
        continue;
      }
      stack.append(link.tonode);
    }
  }
  return false;
}

static bool object_is_valid(const Object *ob)
{
  return ob && !ID_MISSING(&ob->id);
}

static void assign_node_object_id(bNode &node, Object *ob)
{
  ID *new_id = ob ? &ob->id : nullptr;
  if (node.id == new_id) {
    return;
  }
  if (node.id) {
    id_us_min(node.id);
  }
  node.id = new_id;
  if (node.id) {
    id_us_plus(node.id);
  }
}

static void apply_transform(Object &ob, const bNodeTree &ntree, const bNode &node)
{
  const auto *storage = static_cast<const NodeObjectSetTransform *>(node.storage);
  if (storage == nullptr) {
    return;
  }
  if (storage->apply_location) {
    const float3 loc = socket_vector_value(ntree, find_input(node, "Location"), float3(ob.loc));
    copy_v3_v3(ob.loc, loc);
  }
  if (storage->apply_rotation) {
    const float3 rot = socket_vector_value(ntree, find_input(node, "Rotation"), float3(ob.rot));
    copy_v3_v3(ob.rot, rot);
  }
  if (storage->apply_scale) {
    const float3 scale = socket_vector_value(ntree, find_input(node, "Scale"), float3(ob.scale));
    copy_v3_v3(ob.scale, scale);
  }
  DEG_id_tag_update(&ob.id, ID_RECALC_TRANSFORM);
}

static void apply_visibility(Object &ob, const bNode &node)
{
  SET_FLAG_FROM_TEST(ob.visibility_flag,
                     socket_bool_value(find_input(node, "Hide Viewport"),
                                       (ob.visibility_flag & OB_HIDE_VIEWPORT) != 0),
                     OB_HIDE_VIEWPORT);
  SET_FLAG_FROM_TEST(ob.visibility_flag,
                     socket_bool_value(find_input(node, "Hide Select"),
                                       (ob.visibility_flag & OB_HIDE_SELECT) != 0),
                     OB_HIDE_SELECT);
  SET_FLAG_FROM_TEST(ob.visibility_flag,
                     socket_bool_value(find_input(node, "Hide Render"),
                                       (ob.visibility_flag & OB_HIDE_RENDER) != 0),
                     OB_HIDE_RENDER);
  SET_FLAG_FROM_TEST(ob.visibility_flag,
                     socket_bool_value(find_input(node, "Holdout"),
                                       (ob.visibility_flag & OB_HOLDOUT) != 0),
                     OB_HOLDOUT);
  SET_FLAG_FROM_TEST(ob.visibility_flag,
                     socket_bool_value(find_input(node, "Shadow Catcher"),
                                       (ob.visibility_flag & OB_SHADOW_CATCHER) != 0),
                     OB_SHADOW_CATCHER);
  DEG_id_tag_update(&ob.id, ID_RECALC_SYNC_TO_EVAL);
}

static void apply_viewport_display(Object &ob, const bNode &node)
{
  const auto *storage = static_cast<const NodeObjectViewportDisplay *>(node.storage);
  if (storage) {
    ob.dt = eDrawType(storage->display_type);
    ob.boundtype = eObject_BoundType(storage->bounds_type);
  }
  SET_FLAG_FROM_TEST(ob.dtx,
                     socket_bool_value(find_input(node, "Show Name"), (ob.dtx & OB_DRAWNAME) != 0),
                     OB_DRAWNAME);
  SET_FLAG_FROM_TEST(ob.dtx,
                     socket_bool_value(find_input(node, "Show Axis"), (ob.dtx & OB_AXIS) != 0),
                     OB_AXIS);
  SET_FLAG_FROM_TEST(
      ob.dtx, socket_bool_value(find_input(node, "Show Wire"), (ob.dtx & OB_DRAWWIRE) != 0), OB_DRAWWIRE);
  SET_FLAG_FROM_TEST(ob.dtx,
                     socket_bool_value(find_input(node, "In Front"), (ob.dtx & OB_DRAW_IN_FRONT) != 0),
                     OB_DRAW_IN_FRONT);
  SET_FLAG_FROM_TEST(ob.dtx,
                     socket_bool_value(find_input(node, "Show Bounds"), (ob.dtx & OB_DRAWBOUNDOX) != 0),
                     OB_DRAWBOUNDOX);
  const float4 color = socket_color_value(find_input(node, "Color"), float4(ob.color));
  copy_v4_v4(ob.color, color);
  DEG_id_tag_update(&ob.id, ID_RECALC_SYNC_TO_EVAL);
}

static void apply_parent(Object &ob, Object *parent, const bNode &node)
{
  const auto *storage = static_cast<const NodeObjectParent *>(node.storage);
  const bool keep_transform = storage ? storage->keep_transform != 0 : true;
  const bool clear_mode = storage && storage->mode != 0;
  const bool no_inverse = storage && storage->bind_type != 0;
  const char clear_type = storage ? storage->clear_type : 0;

  if (clear_mode) {
    if (ob.parent == nullptr && clear_type != 2) {
      return;
    }
    if (clear_type == 1) {
      const float4x4 world = ob.object_to_world();
      ob.parent = nullptr;
      ob.partype = PAROBJECT;
      ob.parsubstr[0] = '\0';
      unit_m4(ob.parentinv);
      BKE_object_apply_mat4(&ob, world.ptr(), true, false);
    }
    else if (clear_type == 2) {
      unit_m4(ob.parentinv);
    }
    else {
      ob.parent = nullptr;
      ob.partype = PAROBJECT;
      ob.parsubstr[0] = '\0';
      unit_m4(ob.parentinv);
    }
    DEG_id_tag_update(&ob.id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY | ID_RECALC_SYNC_TO_EVAL);
    return;
  }

  if (parent == nullptr) {
    return;
  }
  if (BKE_object_parent_loop_check(parent, &ob)) {
    return;
  }
  if (ob.parent == parent && ob.partype == PAROBJECT && !no_inverse) {
    return;
  }

  ob.parent = parent;
  ob.partype = PAROBJECT;
  ob.parsubstr[0] = '\0';

  if (no_inverse) {
    if (keep_transform) {
      BKE_object_apply_parent_inverse(&ob);
    }
    else {
      unit_m4(ob.parentinv);
      zero_v3(ob.loc);
    }
  }
  else if (keep_transform) {
    const float4x4 child_world = ob.object_to_world();
    const float4x4 inv_parent = math::invert(parent->object_to_world());
    const float4x4 parentinv = inv_parent * child_world;
    copy_m4_m4(ob.parentinv, parentinv.ptr());
  }
  else {
    unit_m4(ob.parentinv);
  }
  DEG_id_tag_update(&ob.id, ID_RECALC_TRANSFORM | ID_RECALC_SYNC_TO_EVAL);
}

static void apply_material_slots(Main &bmain, Object &ob, const bNode &node)
{
  const auto *storage = static_cast<const NodeObjectMaterialSlots *>(node.storage);
  if (storage == nullptr || BKE_object_material_array_p(&ob) == nullptr) {
    return;
  }
  const int target = std::max(storage->items_num, 0);
  while (ob.totcol < target) {
    const int prev = ob.totcol;
    if (!BKE_object_material_slot_add(&bmain, &ob) || ob.totcol <= prev) {
      break;
    }
  }
  while (ob.totcol > target) {
    ob.actcol = ob.totcol;
    if (!BKE_object_material_slot_remove(&bmain, &ob)) {
      break;
    }
  }
  for (int i = 0; i < target && i < ob.totcol; i++) {
    Material *ma = storage->items[i].material;
    if (ma && ID_MISSING(&ma->id)) {
      ma = nullptr;
    }
    BKE_object_material_assign(&bmain, &ob, ma, short(i + 1), BKE_MAT_ASSIGN_USERPREF);
  }
  if (target > 0) {
    ob.actcol = std::clamp(storage->active_index, 0, target - 1) + 1;
  }
  else {
    ob.actcol = 0;
  }
  DEG_id_tag_update(&ob.id, ID_RECALC_GEOMETRY | ID_RECALC_SYNC_TO_EVAL);
}

static Object *ensure_created_object(Main &bmain,
                                     Scene &scene,
                                     ViewLayer &view_layer,
                                     bNode &node)
{
  const auto *storage = static_cast<const NodeObjectCreate *>(node.storage);
  const ObjectType type = ObjectType(storage ? storage->object_type : OB_MESH);
  const char *name = (storage && storage->name[0]) ? storage->name : "Object";

  if (node.id && GS(node.id->name) == ID_OB && !ID_MISSING(node.id)) {
    Object *ob = reinterpret_cast<Object *>(node.id);
    if (!STREQ(ob->id.name + 2, name)) {
      BKE_id_rename(bmain, ob->id, name);
    }
    return ob;
  }

  Object *ob = BKE_object_add(&bmain, &scene, &view_layer, type, name);
  assign_node_object_id(node, ob);
  DEG_id_tag_update(&ob->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY | ID_RECALC_SYNC_TO_EVAL);
  return ob;
}

static void set_output_object(Map<const bNodeSocket *, Object *> &values,
                              const bNode &node,
                              Object *ob)
{
  const bNodeSocket *out = bke::node_find_socket(node, SOCK_OUT, "Object"_ustr);
  if (out) {
    values.add(out, ob);
  }
}

bool ntreeObjectNodesEvaluate(Main &bmain, Scene &scene, ViewLayer &view_layer, bNodeTree &ntree)
{
  if (ntree.type != NTREE_OBJECT) {
    return false;
  }
  if (g_object_nodes_evaluating) {
    return false;
  }
  if (G.is_rendering) {
    return false;
  }
  if (ntree.id.tag & (ID_TAG_NO_MAIN | ID_TAG_COPIED_ON_EVAL | ID_TAG_LOCALIZED)) {
    return false;
  }

  ObjectEvalGuard eval_guard;

  ntree.ensure_topology_cache();

  Map<const bNodeSocket *, Object *> values;
  Set<Object *> to_delete;
  bool relations_changed = false;

  for (const bNode *node_c : ntree.toposort_left_to_right()) {
    bNode &node = const_cast<bNode &>(*node_c);
    if (node.is_frame() || node.is_reroute()) {
      if (node.is_reroute()) {
        const bNodeSocket *in = static_cast<const bNodeSocket *>(node.inputs.first());
        const bNodeSocket *out = static_cast<const bNodeSocket *>(node.outputs.first());
        if (in && out) {
          values.add(out, socket_object_value(ntree, in, values));
        }
      }
      continue;
    }

    const bool live = node_reaches_type(ntree, node, "ObjectNodeOutput");
    const bool deleted = node_reaches_type(ntree, node, "ObjectNodeDelete");

    if (node_is(node, "ObjectNodeObject")) {
      const auto *storage = static_cast<const NodeObjectInput *>(node.storage);
      Object *ob = nullptr;
      if (storage && storage->use_active) {
        BKE_view_layer_synced_ensure(bmain, &scene, &view_layer);
        ob = BKE_view_layer_active_object_get(&view_layer);
      }
      else {
        ob = socket_object_value(ntree, find_input(node, "Object"), values);
      }
      if (!object_is_valid(ob)) {
        ob = nullptr;
      }
      set_output_object(values, node, ob);
    }
    else if (node_is(node, "ObjectNodeCreate")) {
      Object *ob = nullptr;
      if (deleted) {
        if (node.id && GS(node.id->name) == ID_OB && !ID_MISSING(node.id)) {
          to_delete.add(reinterpret_cast<Object *>(node.id));
        }
        assign_node_object_id(node, nullptr);
      }
      else if (live) {
        ob = ensure_created_object(bmain, scene, view_layer, node);
        relations_changed = true;
      }
      set_output_object(values, node, ob);
    }
    else if (node_is(node, "ObjectNodeSetTransform") || node_is(node, "ObjectNodeSetVisibility") ||
             node_is(node, "ObjectNodeViewportDisplay") ||
             node_is(node, "ObjectNodeSetMaterialSlots") ||
             node_is(node, "ObjectNodeSetModifiers") || node_is(node, "ObjectNodeSetParent"))
    {
      Object *ob = socket_object_value(ntree, find_input(node, "Object"), values);
      if (object_is_valid(ob) && live && !deleted) {
        if (node_is(node, "ObjectNodeSetTransform")) {
          apply_transform(*ob, ntree, node);
        }
        else if (node_is(node, "ObjectNodeSetVisibility")) {
          apply_visibility(*ob, node);
        }
        else if (node_is(node, "ObjectNodeViewportDisplay")) {
          apply_viewport_display(*ob, node);
        }
        else if (node_is(node, "ObjectNodeSetMaterialSlots")) {
          apply_material_slots(bmain, *ob, node);
        }
        else if (node_is(node, "ObjectNodeSetModifiers")) {
          /* Live editor of Object.modifiers. Do not rebuild the stack — that would wipe
           * Geometry Nodes groups, Subdivision settings, and every other modifier property. */
        }
        else if (node_is(node, "ObjectNodeSetParent")) {
          Object *parent = socket_object_value(ntree, find_input(node, "Parent"), values);
          if (parent == ob) {
            parent = nullptr;
          }
          apply_parent(*ob, object_is_valid(parent) ? parent : nullptr, node);
          relations_changed = true;
        }
      }
      set_output_object(values, node, ob);
    }
    else if (node_is(node, "ObjectNodeDelete")) {
      Object *ob = socket_object_value(ntree, find_input(node, "Object"), values);
      if (object_is_valid(ob)) {
        to_delete.add(ob);
        relations_changed = true;
      }
    }
    else if (node_is(node, "ObjectNodeOutput") || node_is(node, "ObjectNodeGroup") ||
             node_is(node, "NodeGroupInput") || node_is(node, "NodeGroupOutput"))
    {
      /* Pass-through Object sockets on groups / output. */
      for (const bNodeSocket *sock : node.input_sockets()) {
        if (sock->type == SOCK_OBJECT) {
          UNUSED_VARS(socket_object_value(ntree, sock, values));
        }
      }
    }
  }

  for (Object *ob : to_delete) {
    if (!object_is_valid(ob)) {
      continue;
    }
    for (bNode *node : ntree.all_nodes()) {
      if (node->id == &ob->id) {
        assign_node_object_id(*node, nullptr);
      }
    }
    BKE_id_delete(&bmain, &ob->id);
    relations_changed = true;
  }

  if (relations_changed) {
    DEG_relations_tag_update(&bmain);
  }
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
  WM_main_add_notifier(NC_OBJECT | ND_TRANSFORM, nullptr);
  WM_main_add_notifier(NC_SCENE | ND_OB_ACTIVE, nullptr);

  return true;
}

void ntreeObjectNodesEvaluateAll(Main &bmain, Scene *scene)
{
  ViewLayer *view_layer = nullptr;
  if (scene == nullptr) {
    scene = static_cast<Scene *>(bmain.scenes.first());
  }
  if (scene) {
    view_layer = BKE_view_layer_default_view(scene);
  }
  if (!scene || !view_layer) {
    return;
  }
  for (bNodeTree &ntree : bmain.nodetrees) {
    if (ntree.type == NTREE_OBJECT) {
      ntreeObjectNodesEvaluate(bmain, *scene, *view_layer, ntree);
    }
  }
}

static void object_nodes_load_post_cb(Main *bmain,
                                      PointerRNA ** /*pointers*/,
                                      const int /*num_pointers*/,
                                      void * /*arg*/)
{
  if (!bmain) {
    return;
  }
  ntreeObjectNodesEvaluateAll(*bmain, nullptr);
}

void ntreeObjectNodesRegisterLoadHooks()
{
  static bCallbackFuncStore store_load = {nullptr};
  if (store_load.func) {
    return;
  }
  store_load.func = object_nodes_load_post_cb;
  store_load.alloc = 0;
  BKE_callback_add(&store_load, BKE_CB_EVT_LOAD_POST);
}

}  // namespace blender

/* Stale discover entry from an unfinished CGAL node. Keep the link closed. */
namespace blender::nodes::node_geo_cgal_circle_arrangement_2_cc {
void node_register_discover() {}
}

namespace blender {

void node_tree_object_default_init(const bContext *C, bNodeTree *ntree)
{
  BLI_assert(ntree != nullptr && ntree->type == NTREE_OBJECT);
  BLI_assert(ntree->nodes.count() == 0);

  bNode *input = bke::node_add_node(C, *ntree, "ObjectNodeObject"_ustr);
  bNode *xform = bke::node_add_node(C, *ntree, "ObjectNodeSetTransform"_ustr);
  bNode *output = bke::node_add_node(C, *ntree, "ObjectNodeOutput"_ustr);

  input->location[0] = -360.0f;
  input->location[1] = 80.0f;
  xform->location[0] = -40.0f;
  xform->location[1] = 80.0f;
  output->location[0] = 280.0f;
  output->location[1] = 80.0f;

  if (auto *storage = static_cast<NodeObjectInput *>(input->storage)) {
    storage->use_active = 1;
  }

  bke::node_add_link(*ntree,
                     *input,
                     *bke::node_find_socket(*input, SOCK_OUT, "Object"_ustr),
                     *xform,
                     *bke::node_find_socket(*xform, SOCK_IN, "Object"_ustr));
  bke::node_add_link(*ntree,
                     *xform,
                     *bke::node_find_socket(*xform, SOCK_OUT, "Object"_ustr),
                     *output,
                     *bke::node_find_socket(*output, SOCK_IN, "Object"_ustr));

  bke::node_set_active(*ntree, *output);
}

}  // namespace blender
