/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 */

#include "DNA_customdata_types.h"
#include "DNA_image_types.h"
#include "DNA_node_tree_interface_types.h"
#include "DNA_screen_types.h"

#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_set.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_vector.hh"

#include "NOD_socket.hh"

#include "BKE_appdir.hh"
#include "BKE_blender_copybuffer.hh"
#include "BKE_blendfile.hh"
#include "BKE_context.hh"
#include "BKE_global.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
#include "BKE_library.hh"
#include "BKE_main.hh"
#include "BKE_main_idmap.hh"
#include "BKE_main_invariants.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_report.hh"

#include "BLO_readfile.hh"

#include "BLI_path_utils.hh"

#include "ED_node.hh"
#include "ED_render.hh"
#include "ED_screen.hh"

#include "UI_view2d.hh"

#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "DEG_depsgraph_build.hh"

#include "node_intern.hh"

namespace blender::ed::space_node {

/* -------------------------------------------------------------------- */
/** \name Local Utilities
 * \{ */

static void node_copybuffer_filepath_get(char filepath[FILE_MAX], size_t filepath_maxncpy)
{
  BLI_path_join(filepath, filepath_maxncpy, BKE_tempdir_base(), "copybuffer_nodes.blend");
}

/**
 * Cross-editor paste: a geometry node group of only math/vector/etc. nodes can be pasted into
 * the shader editor (and vice versa) because those node types already poll in both trees.
 * Group nodes themselves do not — `GeometryNodeGroup` is not a `ShaderNodeGroup`.
 * When every nested node and interface socket is valid in the destination tree, duplicate the
 * group as the destination type and retarget the group node.
 */

static bool node_can_paste_into_tree(const bNode &node,
                                     const bNodeTree &dst_tree,
                                     Map<const bNodeTree *, bool> &group_ok,
                                     Set<const bNodeTree *> &visiting,
                                     const char **r_hint);

static bool group_interface_valid_in_tree(const bNodeTree &src_group,
                                          const bNodeTree &dst_tree,
                                          const char **r_hint)
{
  if (dst_tree.typeinfo == nullptr || dst_tree.typeinfo->valid_socket_type == nullptr) {
    return true;
  }
  bool ok = true;
  src_group.tree_interface.foreach_item([&](const bNodeTreeInterfaceItem &item) {
    if (item.item_type != NodeTreeInterfaceItemType::Socket) {
      return true;
    }
    const auto &socket = reinterpret_cast<const bNodeTreeInterfaceSocket &>(item);
    bke::bNodeSocketType *stype = socket.socket_typeinfo();
    if (stype && !dst_tree.typeinfo->valid_socket_type(dst_tree.typeinfo, stype)) {
      if (r_hint) {
        *r_hint = "Node group has socket types not supported in this editor";
      }
      ok = false;
      return false;
    }
    return true;
  });
  return ok;
}

static bool group_tree_can_convert(const bNodeTree *src_group,
                                   const bNodeTree &dst_tree,
                                   Map<const bNodeTree *, bool> &group_ok,
                                   Set<const bNodeTree *> &visiting,
                                   const char **r_hint)
{
  if (dst_tree.typeinfo == nullptr || dst_tree.typeinfo->group_idname.is_empty()) {
    if (r_hint) {
      *r_hint = "Destination editor has no group node type";
    }
    return false;
  }
  if (bke::node_type_find(dst_tree.typeinfo->group_idname) == nullptr) {
    return false;
  }
  if (src_group == nullptr) {
    return true;
  }
  if (src_group->type == dst_tree.type) {
    return false;
  }
  if (const bool *cached = group_ok.lookup_ptr(src_group)) {
    return *cached;
  }
  if (!visiting.add(src_group)) {
    /* Cycle while checking nested groups; treat as compatible for now. */
    return true;
  }

  bool ok = group_interface_valid_in_tree(*src_group, dst_tree, r_hint);
  if (ok) {
    for (const bNode &node : src_group->nodes) {
      const char *hint = nullptr;
      if (!node_can_paste_into_tree(node, dst_tree, group_ok, visiting, &hint)) {
        if (r_hint) {
          *r_hint = hint ? hint : "Node group contains nodes not supported in this editor";
        }
        ok = false;
        break;
      }
    }
  }

  visiting.remove(src_group);
  group_ok.add(src_group, ok);
  return ok;
}

static bool named_attribute_is_uv(const bNode &node)
{
  if (!node.is_type("GeometryNodeInputNamedAttribute"_ustr)) {
    return false;
  }
  const auto *storage = static_cast<const NodeGeometryInputNamedAttribute *>(node.storage);
  if (storage == nullptr ||
      !ELEM(storage->data_type, CD_PROP_FLOAT2, CD_PROP_FLOAT3, CD_PROP_FLOAT))
  {
    return false;
  }
  const bNodeSocket *name_sock = bke::node_find_socket(node, SOCK_IN, "Name"_ustr);
  if (name_sock == nullptr || name_sock->default_value == nullptr) {
    return false;
  }
  const char *name = name_sock->default_value_typed<bNodeSocketValueString>()->value;
  return STR_ELEM(name, "UVMap", "uv_map", "UV", "uv", "UVMap.001");
}

/**
 * Shader Texture Coordinate / Geometry inputs map to geometry-node equivalents (and back).
 * Compositor and other tree types stay pure-compute only.
 */
static bool node_has_cross_editor_expand(const bNode &node, const bNodeTree &dst_tree)
{
  if (dst_tree.type == NTREE_GEOMETRY) {
    return node.is_type("ShaderNodeTexCoord"_ustr) || node.is_type("ShaderNodeNewGeometry"_ustr) ||
           node.is_type("ShaderNodeTexImage"_ustr);
  }
  if (dst_tree.type == NTREE_SHADER) {
    return node.is_type("GeometryNodeInputPosition"_ustr) ||
           node.is_type("GeometryNodeInputNormal"_ustr) || named_attribute_is_uv(node) ||
           node.is_type("GeometryNodeImageTexture"_ustr);
  }
  return false;
}

static bool node_can_paste_into_tree(const bNode &node,
                                     const bNodeTree &dst_tree,
                                     Map<const bNodeTree *, bool> &group_ok,
                                     Set<const bNodeTree *> &visiting,
                                     const char **r_hint)
{
  if (!node.typeinfo->poll_instance || node.typeinfo->poll_instance(&node, &dst_tree, r_hint)) {
    return true;
  }
  if (node_has_cross_editor_expand(node, dst_tree)) {
    return true;
  }
  if (node.type_legacy == NODE_GROUP) {
    return group_tree_can_convert(
        reinterpret_cast<const bNodeTree *>(node.id), dst_tree, group_ok, visiting, r_hint);
  }
  return false;
}

static void retarget_group_node_to_tree_type(bNode &node, const bNodeTree &dst_tree)
{
  if (node.type_legacy != NODE_GROUP) {
    return;
  }
  const UString dst_idname = dst_tree.typeinfo->group_idname;
  if (node.is_type(dst_idname)) {
    return;
  }
  STRNCPY_UTF8(node.idname, dst_idname.c_str());
  bke::bNodeType *ntype = bke::node_type_find(dst_idname);
  BLI_assert(ntype != nullptr);
  node.typeinfo = ntype;
  node.type_legacy = ntype->type_legacy;
}

struct CrossEditorExpandCache {
  bNode *gn_position = nullptr;
  bNode *gn_normal = nullptr;
  bNode *gn_self_object = nullptr;
  bNode *gn_object_info = nullptr;
  bNode *sh_tex_coord = nullptr;
};

static bNodeSocket *sock_out(bNode &node, const UString id)
{
  return bke::node_find_socket(node, SOCK_OUT, id);
}
static bNodeSocket *sock_in(bNode &node, const UString id)
{
  return bke::node_find_socket(node, SOCK_IN, id);
}

static void link_ident(bNodeTree &tree,
                       bNode &from,
                       const UString from_id,
                       bNode &to,
                       const UString to_id)
{
  bNodeSocket *fs = sock_out(from, from_id);
  bNodeSocket *ts = sock_in(to, to_id);
  if (fs && ts) {
    bke::node_add_link(tree, from, *fs, to, *ts);
  }
}

static void map_src_out(const bNode &src,
                        const UString src_id,
                        bNode &dst_node,
                        const UString dst_id,
                        Map<const bNodeSocket *, bNodeSocket *> &socket_map,
                        Map<const bNodeSocket *, bNode *> &sock_nodes)
{
  const bNodeSocket *ss = bke::node_find_socket(src, SOCK_OUT, src_id);
  bNodeSocket *ds = sock_out(dst_node, dst_id);
  if (ss && ds) {
    socket_map.add(ss, ds);
    sock_nodes.add(ss, &dst_node);
  }
}

static void map_src_in(const bNode &src,
                       const UString src_id,
                       bNode &dst_node,
                       const UString dst_id,
                       Map<const bNodeSocket *, bNodeSocket *> &socket_map,
                       Map<const bNodeSocket *, bNode *> &sock_nodes)
{
  const bNodeSocket *ss = bke::node_find_socket(src, SOCK_IN, src_id);
  bNodeSocket *ds = sock_in(dst_node, dst_id);
  if (ss && ds) {
    socket_map.add(ss, ds);
    sock_nodes.add(ss, &dst_node);
  }
}

static bool output_is_linked(const bNodeTree &tree, const bNode &node, const char *identifier)
{
  for (const bNodeLink &link : tree.links) {
    if (link.fromnode == &node && STREQ(link.fromsock->identifier, identifier)) {
      return true;
    }
  }
  return false;
}

static bNode &add_mapped_node(bNodeTree &tree,
                              const UString idname,
                              const float2 loc,
                              bNode *frame)
{
  bNode *node = bke::node_add_node(nullptr, tree, idname);
  node->location[0] = loc.x;
  node->location[1] = loc.y;
  node->flag |= NODE_SELECT;
  if (frame) {
    bke::node_attach_node(tree, *node, *frame);
  }
  return *node;
}

static bNode &ensure_cached_node(bNodeTree &tree,
                                 bNode *&cache,
                                 const UString idname,
                                 const float2 loc,
                                 bNode *frame)
{
  if (cache) {
    return *cache;
  }
  cache = &add_mapped_node(tree, idname, loc, frame);
  return *cache;
}

static bNode *add_frame(bNodeTree &tree, const float2 loc, const char *label)
{
  bNode &frame = add_mapped_node(tree, "NodeFrame"_ustr, loc, nullptr);
  STRNCPY_UTF8(frame.label, label);
  return &frame;
}

static void set_vector_math_op(bNodeTree &tree, bNode &node, const int op)
{
  node.custom1 = int16_t(op);
  if (node.typeinfo->updatefunc) {
    node.typeinfo->updatefunc(&tree, &node);
  }
}

/** Shader Texture Coordinate → GN: only sockets that are actually linked. */
static bNode *expand_shader_tex_coord(bNodeTree &tree,
                                      const bNodeTree &src_tree,
                                      const bNode &src,
                                      const float2 origin,
                                      Map<const bNodeSocket *, bNodeSocket *> &socket_map,
                                      Map<const bNodeSocket *, bNode *> &sock_nodes,
                                      CrossEditorExpandCache &cache)
{
  const bool use_generated = output_is_linked(src_tree, src, "Generated");
  const bool use_normal = output_is_linked(src_tree, src, "Normal");
  const bool use_uv = output_is_linked(src_tree, src, "UV");
  const bool use_object = output_is_linked(src_tree, src, "Object");
  /* Unconnected node: object-space position is the GN default. */
  const bool need_object = use_object || (!use_generated && !use_normal && !use_uv);
  const bool need_position = use_generated || need_object;

  const float dx = 200.0f;
  const float dy = 140.0f;
  const int parts = int(use_generated) + int(use_normal) + int(use_uv) + int(need_object);
  bNode *frame = (parts > 1 || use_generated) ?
                     add_frame(tree, origin + float2(-20.0f, 40.0f), "Texture Coordinate") :
                     nullptr;

  bNode *pos = nullptr;
  if (need_position) {
    pos = &ensure_cached_node(
        tree, cache.gn_position, "GeometryNodeInputPosition"_ustr, origin, frame);
  }
  if (need_object && pos) {
    map_src_out(src, "Object"_ustr, *pos, "Position"_ustr, socket_map, sock_nodes);
  }

  bNode *primary = pos;

  if (use_normal) {
    bNode &nor = ensure_cached_node(tree,
                                    cache.gn_normal,
                                    "GeometryNodeInputNormal"_ustr,
                                    origin + float2(0.0f, -dy),
                                    frame);
    map_src_out(src, "Normal"_ustr, nor, "Normal"_ustr, socket_map, sock_nodes);
    if (primary == nullptr) {
      primary = &nor;
    }
  }

  if (use_uv) {
    bNode &uv = add_mapped_node(tree,
                                "GeometryNodeInputNamedAttribute"_ustr,
                                origin + float2(0.0f, -2.0f * dy),
                                frame);
    auto *uv_storage = static_cast<NodeGeometryInputNamedAttribute *>(uv.storage);
    if (uv_storage) {
      uv_storage->data_type = CD_PROP_FLOAT3;
      nodes::update_node_declaration_and_sockets(tree, uv);
    }
    if (bNodeSocket *name_sock = sock_in(uv, "Name"_ustr)) {
      if (bNodeSocketValueString *val = name_sock->default_value_typed<bNodeSocketValueString>()) {
        STRNCPY_UTF8(val->value, "UVMap");
      }
    }
    map_src_out(src, "UV"_ustr, uv, "Attribute"_ustr, socket_map, sock_nodes);
    if (primary == nullptr) {
      primary = &uv;
    }
  }

  if (use_generated && pos) {
    bNode &minmax = add_mapped_node(
        tree, "GeometryNodeFieldMinAndMax"_ustr, origin + float2(dx, 0.0f), frame);
    minmax.custom1 = CD_PROP_FLOAT3;
    nodes::update_node_declaration_and_sockets(tree, minmax);

    bNode &sub_min = add_mapped_node(
        tree, "ShaderNodeVectorMath"_ustr, origin + float2(2.0f * dx, 0.0f), frame);
    set_vector_math_op(tree, sub_min, NODE_VECTOR_MATH_SUBTRACT);
    bNode &size = add_mapped_node(
        tree, "ShaderNodeVectorMath"_ustr, origin + float2(2.0f * dx, -dy), frame);
    set_vector_math_op(tree, size, NODE_VECTOR_MATH_SUBTRACT);
    bNode &div = add_mapped_node(
        tree, "ShaderNodeVectorMath"_ustr, origin + float2(3.0f * dx, 0.0f), frame);
    set_vector_math_op(tree, div, NODE_VECTOR_MATH_DIVIDE);

    link_ident(tree, *pos, "Position"_ustr, minmax, "Value"_ustr);
    link_ident(tree, *pos, "Position"_ustr, sub_min, "Vector"_ustr);
    link_ident(tree, minmax, "Min"_ustr, sub_min, "Vector_001"_ustr);
    link_ident(tree, minmax, "Max"_ustr, size, "Vector"_ustr);
    link_ident(tree, minmax, "Min"_ustr, size, "Vector_001"_ustr);
    link_ident(tree, sub_min, "Vector"_ustr, div, "Vector"_ustr);
    link_ident(tree, size, "Vector"_ustr, div, "Vector_001"_ustr);
    map_src_out(src, "Generated"_ustr, div, "Vector"_ustr, socket_map, sock_nodes);
    primary = &div;
  }

  return primary ? primary : frame;
}

/**
 * Shader Geometry Position/Normal are world-space. GN Position/Normal are object-space,
 * so apply the object's own object-to-world matrix. Only linked outputs are built.
 */
static bNode *expand_shader_geometry(bNodeTree &tree,
                                     const bNodeTree &src_tree,
                                     const bNode &src,
                                     const float2 origin,
                                     Map<const bNodeSocket *, bNodeSocket *> &socket_map,
                                     Map<const bNodeSocket *, bNode *> &sock_nodes,
                                     CrossEditorExpandCache &cache)
{
  const bool use_pos = output_is_linked(src_tree, src, "Position");
  const bool use_nor = output_is_linked(src_tree, src, "Normal");
  const bool use_true = output_is_linked(src_tree, src, "True Normal");
  const bool need_pos = use_pos || (!use_nor && !use_true);
  const bool need_nor = use_nor || use_true;
  const bool need_world = need_pos || need_nor;

  const float dx = 200.0f;
  const float dy = 140.0f;
  const int parts = int(need_pos) + int(use_nor) + int(use_true);
  bNode *frame = (parts > 1) ? add_frame(tree, origin + float2(-20.0f, 40.0f), "Geometry (World)") :
                               nullptr;

  bNode *ob_info = nullptr;
  if (need_world) {
    bNode &self_ob = ensure_cached_node(tree,
                                        cache.gn_self_object,
                                        "GeometryNodeSelfObject"_ustr,
                                        origin + float2(-2.0f * dx, -dy),
                                        frame);
    ob_info = &ensure_cached_node(tree,
                                  cache.gn_object_info,
                                  "GeometryNodeObjectInfo"_ustr,
                                  origin + float2(-dx, -dy),
                                  frame);
    if (auto *storage = static_cast<NodeGeometryObjectInfo *>(ob_info->storage)) {
      storage->transform_space = GEO_NODE_TRANSFORM_SPACE_ORIGINAL;
    }
    bool already_linked = false;
    for (bNodeLink &link : tree.links) {
      if (link.tonode == ob_info && link.fromnode == &self_ob) {
        already_linked = true;
        break;
      }
    }
    if (!already_linked) {
      link_ident(tree, self_ob, "Self Object"_ustr, *ob_info, "Object"_ustr);
    }
  }

  bNode *primary = nullptr;
  if (need_pos && ob_info) {
    bNode &pos = ensure_cached_node(
        tree, cache.gn_position, "GeometryNodeInputPosition"_ustr, origin, frame);
    bNode &xf_pos = add_mapped_node(
        tree, "FunctionNodeTransformPoint"_ustr, origin + float2(dx, 0.0f), frame);
    link_ident(tree, pos, "Position"_ustr, xf_pos, "Vector"_ustr);
    link_ident(tree, *ob_info, "Transform"_ustr, xf_pos, "Transform"_ustr);
    map_src_out(src, "Position"_ustr, xf_pos, "Vector"_ustr, socket_map, sock_nodes);
    primary = &xf_pos;
  }

  bNode *nor = nullptr;
  if (need_nor) {
    nor = &ensure_cached_node(tree,
                              cache.gn_normal,
                              "GeometryNodeInputNormal"_ustr,
                              origin + float2(0.0f, -dy),
                              frame);
  }
  if (use_nor && nor && ob_info) {
    bNode &xf_nor = add_mapped_node(
        tree, "FunctionNodeTransformDirection"_ustr, origin + float2(dx, -dy), frame);
    link_ident(tree, *nor, "Normal"_ustr, xf_nor, "Direction"_ustr);
    link_ident(tree, *ob_info, "Transform"_ustr, xf_nor, "Transform"_ustr);
    map_src_out(src, "Normal"_ustr, xf_nor, "Direction"_ustr, socket_map, sock_nodes);
    if (primary == nullptr) {
      primary = &xf_nor;
    }
  }
  if (use_true && nor && ob_info) {
    bNode &xf_true = add_mapped_node(
        tree, "FunctionNodeTransformDirection"_ustr, origin + float2(dx, -2.0f * dy), frame);
    link_ident(tree, *nor, "True Normal"_ustr, xf_true, "Direction"_ustr);
    link_ident(tree, *ob_info, "Transform"_ustr, xf_true, "Transform"_ustr);
    map_src_out(src, "True Normal"_ustr, xf_true, "Direction"_ustr, socket_map, sock_nodes);
    if (primary == nullptr) {
      primary = &xf_true;
    }
  }
  return primary ? primary : frame;
}

static void copy_image_user(ImageUser &dst, const ImageUser &src)
{
  dst = src;
}

static bNode *convert_shader_image_texture(bNodeTree &tree,
                                           const bNode &src,
                                           const float2 origin,
                                           Map<const bNodeSocket *, bNodeSocket *> &socket_map,
                                           Map<const bNodeSocket *, bNode *> &sock_nodes)
{
  bNode &dst = add_mapped_node(tree, "GeometryNodeImageTexture"_ustr, origin, nullptr);
  STRNCPY_UTF8(dst.name, src.name);
  STRNCPY_UTF8(dst.label, src.label);
  dst.width = src.width;
  bke::node_unique_name(tree, dst);

  const auto *src_tex = static_cast<const NodeTexImage *>(src.storage);
  auto *dst_tex = static_cast<NodeGeometryImageTexture *>(dst.storage);
  if (src_tex && dst_tex) {
    dst_tex->interpolation = int8_t(src_tex->interpolation);
    dst_tex->extension = int8_t(src_tex->extension);
    dst_tex->projection = src_tex->projection;
    dst_tex->projection_blend = src_tex->projection_blend;
    copy_image_user(dst_tex->iuser, src_tex->iuser);
  }
  Image *ima = nullptr;
  if (const bNodeSocket *img_sock = bke::node_find_socket(src, SOCK_IN, "Image"_ustr)) {
    if (const auto *val = img_sock->default_value_typed<bNodeSocketValueImage>()) {
      ima = val->value;
    }
  }
  if (!ima) {
    ima = reinterpret_cast<Image *>(src.id);
  }
  if (ima) {
    if (bNodeSocket *img_sock = sock_in(dst, "Image"_ustr)) {
      if (auto *val = img_sock->default_value_typed<bNodeSocketValueImage>()) {
        val->value = ima;
        id_us_plus(&ima->id);
      }
    }
  }
  map_src_in(src, "Image"_ustr, dst, "Image"_ustr, socket_map, sock_nodes);
  map_src_in(src, "Vector"_ustr, dst, "Vector"_ustr, socket_map, sock_nodes);
  map_src_out(src, "Color"_ustr, dst, "Color"_ustr, socket_map, sock_nodes);
  map_src_out(src, "Alpha"_ustr, dst, "Alpha"_ustr, socket_map, sock_nodes);
  return &dst;
}

static bNode *convert_geo_image_texture(bNodeTree &tree,
                                        const bNode &src,
                                        const float2 origin,
                                        Map<const bNodeSocket *, bNodeSocket *> &socket_map,
                                        Map<const bNodeSocket *, bNode *> &sock_nodes)
{
  bNode &dst = add_mapped_node(tree, "ShaderNodeTexImage"_ustr, origin, nullptr);
  STRNCPY_UTF8(dst.name, src.name);
  STRNCPY_UTF8(dst.label, src.label);
  dst.width = src.width;
  bke::node_unique_name(tree, dst);

  const auto *src_tex = static_cast<const NodeGeometryImageTexture *>(src.storage);
  auto *dst_tex = static_cast<NodeTexImage *>(dst.storage);
  if (src_tex && dst_tex) {
    dst_tex->interpolation = src_tex->interpolation;
    dst_tex->extension = src_tex->extension;
    dst_tex->projection = src_tex->projection;
    dst_tex->projection_blend = src_tex->projection_blend;
    copy_image_user(dst_tex->iuser, src_tex->iuser);
  }
  Image *ima = nullptr;
  if (const bNodeSocket *img_sock = bke::node_find_socket(src, SOCK_IN, "Image"_ustr)) {
    if (const auto *val = img_sock->default_value_typed<bNodeSocketValueImage>()) {
      ima = val->value;
    }
  }
  if (ima) {
    dst.id = &ima->id;
    id_us_plus(dst.id);
    if (bNodeSocket *dst_sock = sock_in(dst, "Image"_ustr)) {
      if (auto *val = dst_sock->default_value_typed<bNodeSocketValueImage>()) {
        val->value = ima;
        id_us_plus(&ima->id);
      }
    }
  }
  map_src_in(src, "Image"_ustr, dst, "Image"_ustr, socket_map, sock_nodes);
  map_src_in(src, "Vector"_ustr, dst, "Vector"_ustr, socket_map, sock_nodes);
  map_src_out(src, "Color"_ustr, dst, "Color"_ustr, socket_map, sock_nodes);
  map_src_out(src, "Alpha"_ustr, dst, "Alpha"_ustr, socket_map, sock_nodes);
  return &dst;
}

static bNode *expand_geo_to_shader_tex_coord(bNodeTree &tree,
                                             const bNode &src,
                                             const float2 origin,
                                             Map<const bNodeSocket *, bNodeSocket *> &socket_map,
                                             Map<const bNodeSocket *, bNode *> &sock_nodes,
                                             CrossEditorExpandCache &cache)
{
  bNode &tex = ensure_cached_node(
      tree, cache.sh_tex_coord, "ShaderNodeTexCoord"_ustr, origin, nullptr);
  if (src.is_type("GeometryNodeInputPosition"_ustr)) {
    map_src_out(src, "Position"_ustr, tex, "Object"_ustr, socket_map, sock_nodes);
  }
  else if (src.is_type("GeometryNodeInputNormal"_ustr)) {
    map_src_out(src, "Normal"_ustr, tex, "Normal"_ustr, socket_map, sock_nodes);
  }
  else if (named_attribute_is_uv(src)) {
    map_src_out(src, "Attribute"_ustr, tex, "UV"_ustr, socket_map, sock_nodes);
  }
  return &tex;
}

static bool expand_node_for_dest(bNodeTree &dst_tree,
                                 const bNodeTree &src_tree,
                                 const bNode &src,
                                 const float2 origin,
                                 Map<const bNodeSocket *, bNodeSocket *> &socket_map,
                                 Map<const bNodeSocket *, bNode *> &sock_nodes,
                                 Map<const bNode *, bNode *> &node_map,
                                 CrossEditorExpandCache &cache)
{
  bNode *primary = nullptr;
  if (dst_tree.type == NTREE_GEOMETRY) {
    if (src.is_type("ShaderNodeTexCoord"_ustr)) {
      primary = expand_shader_tex_coord(
          dst_tree, src_tree, src, origin, socket_map, sock_nodes, cache);
    }
    else if (src.is_type("ShaderNodeNewGeometry"_ustr)) {
      primary = expand_shader_geometry(
          dst_tree, src_tree, src, origin, socket_map, sock_nodes, cache);
    }
    else if (src.is_type("ShaderNodeTexImage"_ustr)) {
      primary = convert_shader_image_texture(dst_tree, src, origin, socket_map, sock_nodes);
    }
  }
  else if (dst_tree.type == NTREE_SHADER) {
    if (src.is_type("GeometryNodeInputPosition"_ustr) ||
        src.is_type("GeometryNodeInputNormal"_ustr) || named_attribute_is_uv(src))
    {
      primary = expand_geo_to_shader_tex_coord(
          dst_tree, src, origin, socket_map, sock_nodes, cache);
    }
    else if (src.is_type("GeometryNodeImageTexture"_ustr)) {
      primary = convert_geo_image_texture(dst_tree, src, origin, socket_map, sock_nodes);
    }
  }
  if (primary == nullptr) {
    return false;
  }
  node_map.add(&src, primary);
  return true;
}

static void replace_mappable_inputs_in_tree(Main &bmain, bNodeTree &tree)
{
  Vector<bNode *> to_replace;
  for (bNode &node : tree.nodes) {
    if (node_has_cross_editor_expand(node, tree)) {
      to_replace.append(&node);
    }
  }
  if (to_replace.is_empty()) {
    return;
  }

  CrossEditorExpandCache cache;
  for (bNode *src : to_replace) {
    Map<const bNodeSocket *, bNodeSocket *> socket_map;
    Map<const bNodeSocket *, bNode *> sock_nodes;
    Map<const bNode *, bNode *> node_map;
    const float2 origin(src->location[0], src->location[1]);
    if (!expand_node_for_dest(
            tree, tree, *src, origin, socket_map, sock_nodes, node_map, cache))
    {
      continue;
    }

    auto remap_end = [&](bNode *&node_p, bNodeSocket *&sock_p) -> bool {
      bNodeSocket *dst_sock = socket_map.lookup_default(sock_p, nullptr);
      bNode *dst_node = sock_nodes.lookup_default(sock_p, nullptr);
      if (dst_sock && dst_node) {
        node_p = dst_node;
        sock_p = dst_sock;
        return true;
      }
      return false;
    };

    Vector<bNodeLink *> remove_links;
    for (bNodeLink &link : tree.links) {
      if (link.fromnode == src) {
        if (!remap_end(link.fromnode, link.fromsock)) {
          remove_links.append(&link);
        }
      }
      else if (link.tonode == src) {
        /* Incoming links (e.g. shader Image Texture Vector) must be retargeted onto the
         * replacement node. Dropping them is why group paste lost the Vector connection
         * while ungrouped copy (which uses socket_map for both ends) did not. */
        if (!remap_end(link.tonode, link.tosock)) {
          remove_links.append(&link);
        }
      }
    }
    for (bNodeLink *link : remove_links) {
      bke::node_remove_link(&tree, *link);
    }
    bke::node_remove_node(&bmain, tree, *src, true);
  }
}

static bNodeTree *get_converted_group_tree(Main &bmain,
                                           const bNodeTree &src_group,
                                           const bNodeTree &dst_tree,
                                           Map<const bNodeTree *, bNodeTree *> &converted,
                                           const bool do_user_count)
{
  if (src_group.type == dst_tree.type) {
    return const_cast<bNodeTree *>(&src_group);
  }
  if (bNodeTree **cached = converted.lookup_ptr(&src_group)) {
    return *cached;
  }

  bNodeTree *copy = bke::node_tree_copy_tree(&bmain, src_group);
  STRNCPY_UTF8(copy->idname, dst_tree.typeinfo->idname.c_str());
  for (bNode &node : copy->nodes) {
    retarget_group_node_to_tree_type(node, dst_tree);
  }
  bke::node_tree_set_type(*copy);
  /* Insert before nested conversion so cyclic groups cannot recurse forever. */
  converted.add(&src_group, copy);

  for (bNode &node : copy->nodes) {
    if (node.type_legacy != NODE_GROUP || node.id == nullptr) {
      continue;
    }
    const bNodeTree *inner_src = reinterpret_cast<const bNodeTree *>(node.id);
    bNodeTree *inner_dst = get_converted_group_tree(
        bmain, *inner_src, dst_tree, converted, do_user_count);
    if (inner_dst != inner_src) {
      if (do_user_count) {
        id_us_min(node.id);
        id_us_plus(&inner_dst->id);
      }
      node.id = &inner_dst->id;
    }
  }

  replace_mappable_inputs_in_tree(bmain, *copy);
  return copy;
}

/** Selected nodes plus zone pairs, without mutating the editor selection. */
static VectorSet<bNode *> nodes_to_copy_from_tree(bNodeTree &tree)
{
  VectorSet<bNode *> selected;
  for (bNode &node : tree.nodes) {
    if (node.flag & NODE_SELECT) {
      selected.add(&node);
    }
  }
  if (selected.is_empty()) {
    return selected;
  }
  tree.ensure_topology_cache();
  for (const bke::bNodeZoneType *zone_type : bke::all_zone_types()) {
    if (zone_type == nullptr) {
      continue;
    }
    for (bNode *input_node : tree.nodes_by_type(zone_type->input_idname)) {
      if (input_node == nullptr) {
        continue;
      }
      if (bNode *output_node = zone_type->get_corresponding_output(tree, *input_node)) {
        if (selected.contains(input_node)) {
          selected.add(output_node);
        }
        else if (selected.contains(output_node)) {
          selected.add(input_node);
        }
      }
    }
  }
  return selected;
}



/** Returns the number of nodes copied. */
static int node_copy_local(Main *bmain,
                           bNodeTree &from_tree,
                           bNodeTree &to_tree,
                           const bool allow_duplicate_names,
                           const bool do_user_count,
                           const float2 offset,
                           const bool snap_to_grid,
                           ReportList *reports,
                           Map<const bNodeTree *, bNodeTree *> *r_converted_groups)
{
  const VectorSet<bNode *> nodes_to_copy = nodes_to_copy_from_tree(from_tree);

  Map<const bNode *, bNode *> node_map;
  Map<const bNodeSocket *, bNodeSocket *> socket_map;
  Map<const bNodeSocket *, bNode *> sock_nodes;
  Map<const bNodeTree *, bool> group_ok;
  Set<const bNodeTree *> visiting;
  Map<const bNodeTree *, bNodeTree *> converted_groups;
  CrossEditorExpandCache expand_cache;

  for (bNode *node : nodes_to_copy) {
    const char *disabled_hint = nullptr;
    const bool poll_ok = !node->typeinfo->poll_instance ||
                         node->typeinfo->poll_instance(node, &to_tree, &disabled_hint);
    const bool can_expand = !poll_ok && node_has_cross_editor_expand(*node, to_tree);
    const bool can_convert_group = !poll_ok && !can_expand && bmain != nullptr &&
                                   node->type_legacy == NODE_GROUP &&
                                   group_tree_can_convert(
                                       reinterpret_cast<const bNodeTree *>(node->id),
                                       to_tree,
                                       group_ok,
                                       visiting,
                                       &disabled_hint);

    if (can_expand) {
      float2 loc(node->location[0] + offset.x, node->location[1] + offset.y);
      if (snap_to_grid) {
        loc.x = nearest_node_grid_coord(loc.x);
        loc.y = nearest_node_grid_coord(loc.y);
      }
      expand_node_for_dest(
          to_tree, from_tree, *node, loc, socket_map, sock_nodes, node_map, expand_cache);
      continue;
    }

    if (poll_ok || can_convert_group) {
      int flags = LIB_ID_COPY_DEFAULT;
      if (!do_user_count) {
        flags |= LIB_ID_CREATE_NO_USER_REFCOUNT;
      }

      bNode *new_node = bke::node_copy_with_mapping(
          &to_tree, *node, flags, std::nullopt, std::nullopt, socket_map, allow_duplicate_names);
      node_map.add_new(node, new_node);
      for (bNodeSocket &sock : node->outputs) {
        if (socket_map.contains(&sock)) {
          sock_nodes.add(&sock, new_node);
        }
      }
      for (bNodeSocket &sock : node->inputs) {
        if (socket_map.contains(&sock)) {
          sock_nodes.add(&sock, new_node);
        }
      }
      new_node->location[0] += offset.x;
      new_node->location[1] += offset.y;
      if (snap_to_grid) {
        new_node->location[0] = nearest_node_grid_coord(new_node->location[0]);
        new_node->location[1] = nearest_node_grid_coord(new_node->location[1]);
      }

      if (can_convert_group) {
        retarget_group_node_to_tree_type(*new_node, to_tree);
        if (new_node->id) {
          bNodeTree *src_group = reinterpret_cast<bNodeTree *>(new_node->id);
          bNodeTree *converted = get_converted_group_tree(
              *bmain, *src_group, to_tree, converted_groups, do_user_count);
          if (converted != src_group) {
            if (do_user_count) {
              id_us_min(new_node->id);
              id_us_plus(&converted->id);
            }
            new_node->id = &converted->id;
          }
        }
        BKE_ntree_update_tag_node_type(&to_tree, new_node);
      }
    }
    else {
      if (disabled_hint) {
        BKE_reportf(reports,
                    RPT_ERROR,
                    "Cannot add node %s into node tree %s: %s",
                    node->name,
                    to_tree.id.name + 2,
                    disabled_hint);
      }
      else {
        BKE_reportf(reports,
                    RPT_ERROR,
                    "Cannot add node %s into node tree %s",
                    node->name,
                    to_tree.id.name + 2);
      }
    }
  }

  if (r_converted_groups) {
    *r_converted_groups = std::move(converted_groups);
  }

  if (node_map.is_empty()) {
    return 0;
  }

  for (bNode *new_node : node_map.values()) {
    /* Parent pointer must be redirected to new node or detached if parent is not copied. */
    if (new_node->parent) {
      if (node_map.contains(new_node->parent)) {
        new_node->parent = node_map.lookup(new_node->parent);
      }
      else {
        bke::node_detach_node(to_tree, *new_node);
      }
    }
  }

  remap_node_pairing(to_tree, node_map);

  /* Copy links between selected nodes. Prefer socket_map so expanded Texture Coordinate /
   * Geometry inputs can reconnect to their replacement subgraphs. */
  for (bNodeLink &link : from_tree.links) {
    BLI_assert(link.tonode);
    BLI_assert(link.fromnode);
    if (link.tonode->is_selected() && link.fromnode->is_selected()) {
      bNodeSocket *from = socket_map.lookup_default(link.fromsock, nullptr);
      bNodeSocket *to = socket_map.lookup_default(link.tosock, nullptr);
      bNode *from_node = sock_nodes.lookup_default(link.fromsock, nullptr);
      bNode *to_node = sock_nodes.lookup_default(link.tosock, nullptr);
      if (!from || !to || !from_node || !to_node) {
        if (!node_map.contains(link.tonode) || !node_map.contains(link.fromnode)) {
          /* If copying a node fails, skip copying their links. */
          continue;
        }
        from_node = node_map.lookup(link.fromnode);
        to_node = node_map.lookup(link.tonode);
        from = bke::node_find_socket(*from_node, SOCK_OUT, link.fromsock->identifier_ustr());
        to = bke::node_find_socket(*to_node, SOCK_IN, link.tosock->identifier_ustr());
        if (!from || !to) {
          continue;
        }
      }

      bNodeLink &new_link = bke::node_add_link(to_tree, *from_node, *from, *to_node, *to);
      new_link.multi_input_sort_id = link.multi_input_sort_id;
    }
  }

  to_tree.ensure_topology_cache();
  for (bNode *new_node : node_map.values()) {
    /* Update multi input socket indices in case all connected nodes weren't copied. */
    update_multi_input_indices_for_removed_links(*new_node);
  }

  return node_map.size();
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Copy
 * \{ */

static wmOperatorStatus node_clipboard_copy_exec(bContext *C, wmOperator *op)
{
  using namespace blender::bke::blendfile;

  Main *bmain = CTX_data_main(C);
  SpaceNode *snode = CTX_wm_space_node(C);
  bNodeTree *node_tree = snode->edittree;

  VectorSet<bNode *> selected = get_selected_nodes(*node_tree);

  PartialWriteContext copy_buffer{*bmain};
  bNodeTree *copy_tree = reinterpret_cast<bNodeTree *>(
      copy_buffer.id_create(ID_NT,
                            "Copy Tree",
                            nullptr,
                            {(PartialWriteContext::IDAddOperations::SET_FAKE_USER |
                              PartialWriteContext::IDAddOperations::SET_CLIPBOARD_MARK)}));

  STRNCPY_UTF8(copy_tree->idname, node_tree->typeinfo->idname.c_str());
  bke::node_tree_set_type(*copy_tree);

  /* Copy node interface to avoid losing links to Group Input and Group Output nodes.
   * Note: this doesn't create new interface items if they don't exist. */
  /* #bNodeTreeInterface::copy_data() allocates runtime memory, so we need to free it to avoid
   * memory leak. */
  copy_tree->tree_interface.free_data();
  copy_tree->tree_interface.copy_data(node_tree->tree_interface, LIB_ID_COPY_DEFAULT);

  const int num_copied = node_copy_local(
      bmain, *node_tree, *copy_tree, true, false, float2(0), false, op->reports, nullptr);
  if (num_copied == 0) {
    BKE_report(op->reports, RPT_ERROR, "No nodes selected to copy");
    return OPERATOR_CANCELLED;
  }

  auto add_tree_ids_dependencies_cb = [&copy_buffer,
                                       copy_tree](LibraryIDLinkCallbackData *cb_data) -> int {
    /* Embedded or null IDs usages can be ignored here. */
    if (cb_data->cb_flag & (IDWALK_CB_EMBEDDED | IDWALK_CB_EMBEDDED_NOT_OWNING)) {
      return IDWALK_RET_NOP;
    }
    ID *id_src = *cb_data->id_pointer;
    if (!id_src) {
      return IDWALK_RET_NOP;
    }

    ID *id_dst = nullptr;

    if (id_src == &copy_tree->id) {
      /* #copy_tree is just a container for the copied nodes, so it can be safely ignored. */
      return IDWALK_RET_NOP;
    }

    auto partial_write_dependencies_filter_cb = [](LibraryIDLinkCallbackData *cb_deps_data,
                                                   PartialWriteContext::IDAddOptions /*options*/) {
      ID *id_deps_src = *cb_deps_data->id_pointer;
      const ID_Type id_type = id_deps_src->id_type();
      if (id_type == ID_SCE) {
        /* Note: Scenes referenced in the Render Layers node are cleared. At this stage, we
         * don't know if the target blender instance will have a scene with identical name, so
         * they are saved in the copy buffer as empty scenes. The pasting code deletes the
         * extra scenes, see #node_clipboard_paste_exec(). */
        return PartialWriteContext::IDAddOperations::CLEAR_DEPENDENCIES;
      }
      /* All ID datablocks exposed through nodes are added here. */
      return PartialWriteContext::IDAddOperations::ADD_DEPENDENCIES;
    };

    id_dst = copy_buffer.id_add(
        id_src, {PartialWriteContext::IDAddOperations::NOP}, partial_write_dependencies_filter_cb);

    *cb_data->id_pointer = id_dst;
    return IDWALK_RET_NOP;
  };

  BKE_library_foreach_ID_link(
      nullptr, &copy_tree->id, add_tree_ids_dependencies_cb, nullptr, IDWALK_NOP);

  char filepath[FILE_MAX];
  node_copybuffer_filepath_get(filepath, sizeof(filepath));
  if (!copy_buffer.write_as_copypaste_buffer(filepath, *op->reports)) {
    BLI_assert_unreachable();
    BKE_report(op->reports, RPT_ERROR, "Unable to write to copy buffer on disk.");
    return OPERATOR_CANCELLED;
  };

  BKE_reportf(op->reports, RPT_INFO, "Copied %d selected node(s)", num_copied);
  return OPERATOR_FINISHED;
}

void NODE_OT_clipboard_copy(wmOperatorType *ot)
{
  ot->name = "Copy to Clipboard";
  ot->description = "Copy the selected nodes to the internal clipboard";
  ot->idname = "NODE_OT_clipboard_copy";

  ot->exec = node_clipboard_copy_exec;
  ot->poll = ED_operator_node_active;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Paste
 * \{ */

static StringRef scene_lib_filepath(const Scene &scene)
{
  if (scene.id.lib && scene.id.lib->runtime) {
    return scene.id.lib->runtime->filepath_abs;
  }
  return "";
}

static wmOperatorStatus node_clipboard_paste_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  Main *bmain_dst = CTX_data_main(C);

  char filepath[FILE_MAX];
  node_copybuffer_filepath_get(filepath, sizeof(filepath));
  Main *bmain_src = BKE_copybuffer_read(*bmain_dst, filepath, op->reports, FILTER_ID_NT);
  if (!bmain_src) {
    BKE_report(op->reports, RPT_ERROR, "No data to paste");
    return OPERATOR_CANCELLED;
  }

  ED_preview_kill_jobs(CTX_wm_manager(C), CTX_data_main(C));

  /* We don't want to paste scenes referenced by the Render Layers node if they don't exist in the
   * destination bmain. */
  Set<std::pair<StringRef, StringRef>> dst_scenes;
  for (Scene &scene : bmain_dst->scenes) {
    /* Packed scenes are currently not needed so they are skipped.
     * TODO: Support packed scenes. */
    if (ID_IS_PACKED(&scene.id)) {
      continue;
    }
    dst_scenes.add({scene.id.name, scene_lib_filepath(scene)});
  }

  for (Scene &scene : bmain_src->scenes.items_mutable()) {
    if (ID_IS_PACKED(&scene.id)) {
      continue;
    }
    /* All scenes that will be added through merging the two bmains are removed. */
    if (!dst_scenes.contains({scene.id.name, scene_lib_filepath(scene)})) {
      BKE_id_delete(bmain_src, &scene.id);
    }
  }

  bNodeTree *from_tree = nullptr;
  FOREACH_NODETREE_BEGIN (bmain_src, node_tree, id) {
    if (node_tree->id.flag & ID_FLAG_CLIPBOARD_MARK) {
      from_tree = node_tree;
      break;
    }
  }
  FOREACH_NODETREE_END;
  if (from_tree == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "No data to paste");
    BKE_main_free(bmain_src);
    return OPERATOR_CANCELLED;
  }

  MainMergeReport merge_reports = {};
  /* We need to ensure that the source 'clipboard marked' main NodeTree is always merged into
   * destination Main, even in case there would be a name collision with an existing ID (see also
   * #158049). */
  Set<ID *> force_merge_ids = {id_cast<ID *>(from_tree)};
  /* Frees bmain_src. */
  BKE_main_merge(bmain_dst, &force_merge_ids, &bmain_src, merge_reports);

  bNodeTree *to_tree = snode->edittree;
  node_deselect_all(*to_tree);

  float2 offset(0);
  PropertyRNA *offset_prop = RNA_struct_find_property(op->ptr, "offset");
  if (RNA_property_is_set(op->ptr, offset_prop)) {
    rctf bbox;
    float2 center;
    BLI_rctf_init_minmax(&bbox);
    for (bNode *node : from_tree->all_nodes()) {
      bbox.xmin = math::min(node->location[0], bbox.xmin);
      bbox.ymin = math::min(node->location[1], bbox.ymin);

      bbox.xmax = math::max(node->location[0] + node->width, bbox.xmax);
      bbox.ymax = math::max(node->location[1] - node->height / 2, bbox.ymax);
    }
    center.x = BLI_rctf_cent_x(&bbox);
    center.y = BLI_rctf_cent_y(&bbox);

    float2 mouse_location;
    RNA_property_float_get_array(op->ptr, offset_prop, mouse_location);
    offset = mouse_location / UI_SCALE_FAC - center;
  }

  const bool snap_to_grid = CTX_data_scene(C)->toolsettings->snap_flag_node & SCE_SNAP;
  Map<const bNodeTree *, bNodeTree *> converted_groups;
  const int num_copied = node_copy_local(bmain_dst,
                                         *from_tree,
                                         *snode->edittree,
                                         false,
                                         true,
                                         offset,
                                         snap_to_grid,
                                         op->reports,
                                         &converted_groups);
  if (num_copied == 0) {
    /* Note: we don't return OPERATOR_CANCELLED here although the copy fails to avoid corrupting
     * the undo stack after merging two bmains. */
  };
  BKE_id_delete(bmain_dst, &from_tree->id);

  /* Clipboard merge may have left unused source-type group copies after conversion. */
  for (const auto &item : converted_groups.items()) {
    const bNodeTree *src = item.key;
    const bNodeTree *dst = item.value;
    if (src != dst && ID_REAL_USERS(&src->id) <= 0) {
      BKE_id_delete(bmain_dst, &const_cast<bNodeTree *>(src)->id);
    }
  }

  BKE_main_ensure_invariants(*bmain_dst);
  /* Pasting nodes can create arbitrary new relations because nodes can reference IDs. */
  DEG_relations_tag_update(bmain_dst);

  return OPERATOR_FINISHED;
}

static wmOperatorStatus node_clipboard_paste_invoke(bContext *C,
                                                    wmOperator *op,
                                                    const wmEvent *event)
{
  const ARegion *region = CTX_wm_region(C);
  float2 cursor;
  ui::view2d_region_to_view(&region->v2d, event->mval[0], event->mval[1], &cursor.x, &cursor.y);
  RNA_float_set_array(op->ptr, "offset", cursor);
  return node_clipboard_paste_exec(C, op);
}

void NODE_OT_clipboard_paste(wmOperatorType *ot)
{
  ot->name = "Paste from Clipboard";
  ot->description = "Paste nodes from the internal clipboard to the active node tree";
  ot->idname = "NODE_OT_clipboard_paste";

  ot->invoke = node_clipboard_paste_invoke;
  ot->exec = node_clipboard_paste_exec;
  ot->poll = ED_operator_node_editable;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  PropertyRNA *prop = RNA_def_float_array(
      ot->srna,
      "offset",
      2,
      nullptr,
      -FLT_MAX,
      FLT_MAX,
      "Location",
      "The 2D view location for the center of the new nodes, or unchanged if not set",
      -FLT_MAX,
      FLT_MAX);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

/** \} */

}  // namespace blender::ed::space_node
