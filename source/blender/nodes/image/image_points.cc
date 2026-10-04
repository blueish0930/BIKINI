/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_image_points.hh"

#include "BLI_array.hh"
#include "BLI_map.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_resource_scope.hh"
#include "BLI_string.hh"
#include "BLI_string_ref.hh"
#include "BLI_string_utf8.hh"
#include "BLI_ustring.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"

#include "DNA_ID.h"
#include "DNA_node_types.h"
#include "DNA_node_tree_interface_types.h"
#include "DNA_object_types.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_compute_contexts.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_geometry_nodes_reference_set.hh"
#include "BKE_geometry_set.hh"
#include "BKE_idprop.hh"
#include "BKE_instances.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_socket_value.hh"
#include "BKE_node_tree_interface.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_pointcloud.hh"

#include "GEO_join_geometries.hh"
#include "GEO_realize_instances.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include <memory>
#include <string>

#include "FN_field.hh"
#include "FN_lazy_function_execute.hh"

#include "NOD_eval_log.hh"
#include "NOD_geometry_nodes_execute.hh"
#include "NOD_geometry_nodes_lazy_function.hh"

#include "MEM_guardedalloc.h"

namespace blender::nodes::image_points {

namespace lf = fn::lazy_function;

/* -------------------------------------------------------------------- */
/** \name Points geometry cache
 * \{ */

static PointsGeometryCache *g_active_cache = nullptr;
static StampAttrMapsCache *g_active_stamp_attr_cache = nullptr;
static StampAttrGpuCache *g_active_stamp_attr_gpu_cache = nullptr;
/**
 * session_uid of nested COP-Import Point tree → last successful cook geometry (for Bundle UI).
 * Heap-allocated and never destroyed: a file-scope Map dtor runs from atexit after #GPU_exit
 * and Mesh GPU batch caches then crash (AV in mesh_free_data).
 */
static Map<uint, bke::GeometrySet> &import_geometry_cache()
{
  static Map<uint, bke::GeometrySet> *map = new Map<uint, bke::GeometrySet>();
  return *map;
}

void PointsGeometryCache::clear()
{
  geometry_by_key_.clear();
}

std::string PointsGeometryCache::store(bke::GeometrySet geometry)
{
  /* Stable key from geometry content pointer is not needed; sequential cook is single-threaded. */
  std::string key = "pts_" + std::to_string(geometry_by_key_.size());
  geometry_by_key_.add_overwrite(key, std::move(geometry));
  return key;
}

const bke::GeometrySet *PointsGeometryCache::lookup(const StringRef key) const
{
  return geometry_by_key_.lookup_ptr(key);
}

PointsGeometryCache *active_cache()
{
  return g_active_cache;
}

void set_active_cache(PointsGeometryCache *cache)
{
  g_active_cache = cache;
}

void StampAttrMapsCache::clear()
{
  maps_by_node_key_.clear();
}

void StampAttrMapsCache::store(const StringRef node_key, StampAttrMaps maps)
{
  maps_by_node_key_.add_overwrite(std::string(node_key), std::move(maps));
}

const StampAttrMaps *StampAttrMapsCache::lookup(const StringRef node_key) const
{
  return maps_by_node_key_.lookup_ptr(node_key);
}

const StampAttrMaps *StampAttrMapsCache::lookup_single_if_unique() const
{
  if (maps_by_node_key_.size() != 1) {
    return nullptr;
  }
  for (const auto &item : maps_by_node_key_.items()) {
    return &item.value;
  }
  return nullptr;
}

int StampAttrMapsCache::size() const
{
  return int(maps_by_node_key_.size());
}

void StampAttrMapsCache::foreach_maps(
    const FunctionRef<void(StringRef key, const StampAttrMaps &maps)> fn) const
{
  for (const auto &item : maps_by_node_key_.items()) {
    fn(item.key, item.value);
  }
}

StampAttrMapsCache *active_stamp_attr_cache()
{
  return g_active_stamp_attr_cache;
}

void set_active_stamp_attr_cache(StampAttrMapsCache *cache)
{
  g_active_stamp_attr_cache = cache;
}

void StampAttrGpuCache::clear()
{
  maps_by_node_key_.clear();
}

void StampAttrGpuCache::store(const StringRef node_key, StampAttrGpuMaps maps)
{
  maps_by_node_key_.add_overwrite(std::string(node_key), std::move(maps));
}

const StampAttrGpuMaps *StampAttrGpuCache::lookup(const StringRef node_key) const
{
  return maps_by_node_key_.lookup_ptr(node_key);
}

StampAttrGpuCache *active_stamp_attr_gpu_cache()
{
  return g_active_stamp_attr_gpu_cache;
}

void set_active_stamp_attr_gpu_cache(StampAttrGpuCache *cache)
{
  g_active_stamp_attr_gpu_cache = cache;
}

std::string stamp_node_cache_key(const bNode &node)
{
  return std::to_string(node.owner_tree().id.session_uid) + ":" +
         std::to_string(node.identifier);
}

const bke::GeometrySet *lookup_last_import_points_geometry(const bNodeTree &geometry_tree)
{
  return import_geometry_cache().lookup_ptr(geometry_tree.id.session_uid);
}

const bke::GeometrySet *lookup_last_import_geo_geometry(const bNodeTree &geometry_tree)
{
  return lookup_last_import_points_geometry(geometry_tree);
}

static void store_last_import_points_geometry(const bNodeTree &geometry_tree,
                                              bke::GeometrySet geometry)
{
  /* Own the mesh so we do not share GPU batches with G_MAIN (freed in #BKE_blender_free
   * before this cache is cleared). Drop batches so atexit cannot free them after GPU_exit. */
  geometry.ensure_owns_direct_data();
  if (Mesh *mesh = geometry.get_mesh_for_write()) {
    BKE_mesh_runtime_clear_cache(mesh);
  }
  import_geometry_cache().add_overwrite(geometry_tree.id.session_uid, std::move(geometry));
}

void clear_session_geometry_caches()
{
  Map<uint, bke::GeometrySet> &map = import_geometry_cache();
  for (bke::GeometrySet &geo : map.values()) {
    if (Mesh *mesh = geo.get_mesh_for_write()) {
      BKE_mesh_runtime_clear_cache(mesh);
    }
  }
  map.clear();
}

void tag_point_stamps_for_import_geometry(bNodeTree &host_tree, const bNode &import_points_node)
{
  host_tree.ensure_topology_cache();
  for (bNode *node : host_tree.all_nodes()) {
    if (!node || !node->typeinfo) {
      continue;
    }
    if (StringRef(node->idname) != "ImageNodePointStamp") {
      continue;
    }
    bool linked = false;
    for (const bNodeSocket *input : node->input_sockets()) {
      if (!input || input->type != SOCK_GEOMETRY) {
        continue;
      }
      for (const bNodeLink *link : input->directly_linked_links()) {
        if (link && link->is_used() && link->fromnode == &import_points_node) {
          linked = true;
          break;
        }
      }
      if (linked) {
        break;
      }
    }
    if (linked) {
      /* Context-dependent declare rebuilds Bundle items from last Import Points geometry. */
      BKE_ntree_update_tag_node_property(&host_tree, node);
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Import Points nested Geometry tree
 * \{ */

static constexpr StringRefNull tree_idname = "GeometryNodeTree";

static void mark_import_points_locked(bNodeTree &ntree)
{
  IDProperty *group = IDP_EnsureProperties(&ntree.id);
  if (IDProperty *existing = IDP_GetPropertyFromGroup(group, import_points_locked_prop)) {
    IDP_int_set(existing, 1);
  }
  else {
    IDProperty *prop = IDP_NewInt(1, import_points_locked_prop);
    IDP_AddToGroup(group, prop);
  }

  /* Never usable as Geometry Nodes Modifier / Tool asset — nested Image Process only. */
  if (ntree.geometry_node_asset_traits) {
    ntree.geometry_node_asset_traits->flag &= ~(GEO_NODE_ASSET_MODIFIER | GEO_NODE_ASSET_TOOL);
  }
}

static bool matches_unique_prefix(const StringRef name, const StringRef prefix)
{
  /* Exact base name or Blender unique suffixes: "Name", "Name.001", ... */
  if (name == prefix) {
    return true;
  }
  return name.startswith(prefix) && name.size() > prefix.size() && name[prefix.size()] == '.';
}

bool is_import_points_tree_name(const StringRef name)
{
  return matches_unique_prefix(name, import_points_tree_name_prefix);
}

bool is_import_geo_tree_name(const StringRef name)
{
  return matches_unique_prefix(name, import_geo_tree_name_prefix);
}

/** Ensure datablock uses short COP-Import Point[.N] name (migrate old IP./long names). */
static void ensure_import_points_tree_name_prefix(Main &bmain, bNodeTree &ntree)
{
  const StringRef name = ntree.id.name + 2;
  if (is_import_points_tree_name(name)) {
    return;
  }
  /* Rename to base; BKE unique-ifies to COP-Import Point.001 if needed. */
  BKE_libblock_rename(bmain, ntree.id, import_points_tree_name_prefix.c_str());
}

static void ensure_size_socket_pixel_default(bNodeTree &ntree)
{
  ntree.ensure_interface_cache();
  for (bNodeTreeInterfaceSocket *sock : ntree.interface_outputs()) {
    if (!sock || sock->name() != StringRef("Size")) {
      continue;
    }
    /* Prefer Pixel subtype so UI shows px, not factor/percent. */
    const StringRef type = sock->socket_type ? sock->socket_type : "";
    if (type.startswith("NodeSocketVector") && !type.endswith("Pixel")) {
      /* Recreate as pixel vector (same storage layout; resets defaults). */
      sock->set_socket_type("NodeSocketVectorPixel");
    }
    if (!sock->socket_data) {
      continue;
    }
    if (sock->socket_typeinfo() && sock->socket_typeinfo()->type == SOCK_VECTOR) {
      auto *data = static_cast<bNodeSocketValueVector *>(sock->socket_data);
      data->subtype = PROP_PIXEL;
      /* Size is 2D; values >1 are pixels at 1024 reference (resolution-independent). */
      data->dimensions = 2;
      /* Migrate invalid zeros; keep unit fractions (≤1) or pixel defaults. */
      if (data->value[0] <= 0.0f) {
        data->value[0] = data->value[1] = 64.0f;
      }
      data->value[2] = 0.0f;
      data->min = 0.0f;
      data->max = 16384.0f;
    }
  }
}

bool is_import_points_geometry_tree(const bNodeTree &ntree)
{
  if (ntree.type != NTREE_GEOMETRY) {
    return false;
  }
  const IDProperty *group = ntree.id.properties;
  if (!group) {
    /* Also accept name-only COP groups so the picker shows them before lock is written. */
    return is_import_points_tree_name(ntree.id.name + 2);
  }
  const IDProperty *prop = IDP_GetPropertyFromGroup(group, import_points_locked_prop);
  if (prop && prop->type == IDP_INT && IDP_int_get(prop) != 0) {
    return true;
  }
  return is_import_points_tree_name(ntree.id.name + 2);
}

static bool fixed_interface_ok(const bNodeTree &ntree)
{
  ntree.ensure_interface_cache();
  if (!ntree.interface_inputs().is_empty() || ntree.interface_outputs().size() != 4) {
    return false;
  }
  auto match = [&](const int i, const StringRef name, const eNodeSocketDatatype type) {
    const bNodeTreeInterfaceSocket *sock = ntree.interface_outputs()[i];
    return sock && sock->name() == name && sock->socket_typeinfo() &&
           sock->socket_typeinfo()->type == type;
  };
  return match(0, "Points", SOCK_GEOMETRY) && match(1, "Rotation", SOCK_ROTATION) &&
         match(2, "Size", SOCK_VECTOR) && match(3, "ID", SOCK_INT);
}

static void ensure_fixed_group_output_interface(bNodeTree &ntree)
{
  if (fixed_interface_ok(ntree)) {
    return;
  }

  ntree.tree_interface.clear_items();

  ntree.tree_interface.add_socket("Points",
                                  "Point cloud geometry (required)",
                                  "NodeSocketGeometry",
                                  NODE_INTERFACE_SOCKET_OUTPUT,
                                  nullptr);
  ntree.tree_interface.add_socket("Rotation",
                                  "Per-point rotation written as attribute \"rotation\"",
                                  "NodeSocketRotation",
                                  NODE_INTERFACE_SOCKET_OUTPUT,
                                  nullptr);
  bNodeTreeInterfaceSocket *size_sock = ntree.tree_interface.add_socket(
      "Size",
      "Per-point stamp size (width, height). ≤1 = unit-square fraction; >1 = pixels at 1024 "
      "reference so layout stays stable when Image Process resolution changes. Attribute \"size\"",
      "NodeSocketVectorPixel",
      NODE_INTERFACE_SOCKET_OUTPUT,
      nullptr);
  if (size_sock && size_sock->socket_data) {
    auto *data = static_cast<bNodeSocketValueVector *>(size_sock->socket_data);
    /* Default stamp: 64×64 px at 1024 reference ≈ 1/16 of image, independent of output W×H. */
    data->value[0] = data->value[1] = 64.0f;
    data->value[2] = 0.0f;
    data->dimensions = 2;
    data->subtype = PROP_PIXEL;
    data->min = 0.0f;
    data->max = 16384.0f;
  }
  ntree.tree_interface.add_socket(
      "ID", "Stamp index, written as attribute \"id\"", "NodeSocketInt", NODE_INTERFACE_SOCKET_OUTPUT, nullptr);

  ntree.ensure_interface_cache();
  for (bNodeTreeInterfaceSocket *sock : ntree.interface_outputs()) {
    if (sock->name() != StringRef("Points")) {
      sock->attribute_domain = int(bke::AttrDomain::Point);
    }
  }

  BKE_ntree_update_tag_all(&ntree);
}

void lock_import_points_geometry_tree(bNodeTree &ntree)
{
  if (ntree.type != NTREE_GEOMETRY) {
    return;
  }
  if (!is_import_points_geometry_tree(ntree)) {
    return;
  }

  mark_import_points_locked(ntree);
  ensure_fixed_group_output_interface(ntree);
  ensure_size_socket_pixel_default(ntree);

  /* No Group Input nodes allowed. */
  Vector<bNode *> to_remove;
  for (bNode *node : ntree.all_nodes()) {
    if (node->is_group_input()) {
      to_remove.append(node);
    }
  }
  for (bNode *node : to_remove) {
    bke::node_remove_node(nullptr, ntree, *node, true);
  }
}

bNodeTree *ensure_import_points_geometry_tree(Main &bmain, bNode &import_points_node)
{
  if (import_points_node.id && GS(import_points_node.id->name) == ID_NT) {
    bNodeTree *existing = reinterpret_cast<bNodeTree *>(import_points_node.id);
    if (existing->type == NTREE_GEOMETRY) {
      mark_import_points_locked(*existing);
      ensure_import_points_tree_name_prefix(bmain, *existing);
      lock_import_points_geometry_tree(*existing);
      return existing;
    }
  }

  /* Short reusable name: "COP-Import Point", "COP-Import Point.001", ... */
  bNodeTree *ntree = bke::node_tree_add_tree(
      &bmain, import_points_tree_name_prefix.c_str(), tree_idname);
  if (!ntree) {
    return nullptr;
  }

  mark_import_points_locked(*ntree);
  ensure_fixed_group_output_interface(*ntree);
  ensure_size_socket_pixel_default(*ntree);

  /* Layout: position unit-square [0,1]²; Size is stamp extent in pixels.
   *   [Points] ──► [Group Output]
   *                  Points / Rotation / Size / ID
   * No Group Input node. */
  bNode *group_out = bke::node_add_static_node(nullptr, *ntree, NODE_GROUP_OUTPUT);
  if (group_out) {
    group_out->location[0] = 200.0f;
    group_out->location[1] = 0.0f;
    STRNCPY_UTF8(group_out->name, "Group Output");
  }
  bNode *points = bke::node_add_node(nullptr, *ntree, "GeometryNodePoints"_ustr);
  if (points) {
    points->location[0] = -280.0f;
    points->location[1] = 40.0f;
    if (bNodeSocket *count_sock = bke::node_find_socket(*points, SOCK_IN, "Count"_ustr)) {
      count_sock->default_value_typed<bNodeSocketValueInt>()->value = 1;
    }
    if (bNodeSocket *pos_sock = bke::node_find_socket(*points, SOCK_IN, "Position"_ustr)) {
      float *v = pos_sock->default_value_typed<bNodeSocketValueVector>()->value;
      /* Center of unit square (not pixel space). */
      v[0] = 0.5f;
      v[1] = 0.5f;
      v[2] = 0.0f;
    }
  }

  /* Assign to Import Points *before* tree update so the host node owns the link. */
  if (import_points_node.id && import_points_node.id != &ntree->id) {
    id_us_min(import_points_node.id);
  }
  if (import_points_node.id != &ntree->id) {
    import_points_node.id = &ntree->id;
    id_us_plus(import_points_node.id);
  }

  BKE_ntree_update_after_single_tree_change(bmain, *ntree);

  if (group_out && points) {
    ntree->ensure_topology_cache();
    bNodeSocket *geo_out = bke::node_find_socket(*points, SOCK_OUT, "Geometry"_ustr);
    bNodeSocket *geo_in = nullptr;
    for (bNodeSocket *sock : group_out->input_sockets()) {
      if (sock->is_available() && sock->name == StringRef("Points")) {
        geo_in = sock;
        break;
      }
    }
    if (geo_out && geo_in && !geo_in->is_directly_linked()) {
      bke::node_add_link(*ntree, *points, *geo_out, *group_out, *geo_in);
      BKE_ntree_update_after_single_tree_change(bmain, *ntree);
    }
  }

  if (import_points_node.runtime && import_points_node.runtime->owner_tree &&
      import_points_node.runtime->owner_tree->type == NTREE_IMAGE)
  {
    BKE_ntree_update_tag_node_property(import_points_node.runtime->owner_tree,
                                       &import_points_node);
  }

  return ntree;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Import Geo nested Geometry tree
 * \{ */

static void mark_import_geo_locked(bNodeTree &ntree)
{
  IDProperty *group = IDP_EnsureProperties(&ntree.id);
  if (IDProperty *existing = IDP_GetPropertyFromGroup(group, import_geo_locked_prop)) {
    IDP_int_set(existing, 1);
  }
  else {
    IDProperty *prop = IDP_NewInt(1, import_geo_locked_prop);
    IDP_AddToGroup(group, prop);
  }

  /* Never usable as Geometry Nodes Modifier / Tool asset — nested Image Process only. */
  if (ntree.geometry_node_asset_traits) {
    ntree.geometry_node_asset_traits->flag &= ~(GEO_NODE_ASSET_MODIFIER | GEO_NODE_ASSET_TOOL);
  }
}

static void ensure_import_geo_tree_name_prefix(Main &bmain, bNodeTree &ntree)
{
  const StringRef name = ntree.id.name + 2;
  if (is_import_geo_tree_name(name)) {
    return;
  }
  BKE_libblock_rename(bmain, ntree.id, import_geo_tree_name_prefix.c_str());
}

bool is_import_geo_geometry_tree(const bNodeTree &ntree)
{
  if (ntree.type != NTREE_GEOMETRY) {
    return false;
  }
  const IDProperty *group = ntree.id.properties;
  if (!group) {
    return is_import_geo_tree_name(ntree.id.name + 2);
  }
  const IDProperty *prop = IDP_GetPropertyFromGroup(group, import_geo_locked_prop);
  if (prop && prop->type == IDP_INT && IDP_int_get(prop) != 0) {
    return true;
  }
  return is_import_geo_tree_name(ntree.id.name + 2);
}

bool is_image_process_nested_geometry_tree(const bNodeTree &ntree)
{
  return is_import_points_geometry_tree(ntree) || is_import_geo_geometry_tree(ntree);
}

bool is_image_import_nested_host(const bNode &node)
{
  return node.is_type("ImageNodeImportPoints"_ustr) || node.is_type("ImageNodeImportGeo"_ustr);
}

bNodeTree *ensure_import_nested_geometry_tree(Main &bmain, bNode &node)
{
  if (node.is_type("ImageNodeImportPoints"_ustr)) {
    return ensure_import_points_geometry_tree(bmain, node);
  }
  if (node.is_type("ImageNodeImportGeo"_ustr)) {
    return ensure_import_geo_geometry_tree(bmain, node);
  }
  return nullptr;
}

static bool geo_fixed_interface_ok(const bNodeTree &ntree)
{
  ntree.ensure_interface_cache();
  if (!ntree.interface_inputs().is_empty() || ntree.interface_outputs().size() != 1) {
    return false;
  }
  const bNodeTreeInterfaceSocket *sock = ntree.interface_outputs()[0];
  return sock && sock->name() == StringRef("Geometry") && sock->socket_typeinfo() &&
         sock->socket_typeinfo()->type == SOCK_GEOMETRY;
}

static void ensure_fixed_geo_group_output_interface(bNodeTree &ntree)
{
  if (geo_fixed_interface_ok(ntree)) {
    return;
  }

  ntree.tree_interface.clear_items();
  ntree.tree_interface.add_socket("Geometry",
                                  "Geometry from the nested Geometry Node tree (mesh, curves, "
                                  "points, realized instances)",
                                  "NodeSocketGeometry",
                                  NODE_INTERFACE_SOCKET_OUTPUT,
                                  nullptr);
  BKE_ntree_update_tag_all(&ntree);
}

void lock_import_geo_geometry_tree(bNodeTree &ntree)
{
  if (ntree.type != NTREE_GEOMETRY) {
    return;
  }
  if (!is_import_geo_geometry_tree(ntree)) {
    return;
  }

  mark_import_geo_locked(ntree);
  ensure_fixed_geo_group_output_interface(ntree);

  Vector<bNode *> to_remove;
  for (bNode *node : ntree.all_nodes()) {
    if (node->is_group_input()) {
      to_remove.append(node);
    }
  }
  for (bNode *node : to_remove) {
    bke::node_remove_node(nullptr, ntree, *node, true);
  }
}

bNodeTree *ensure_import_geo_geometry_tree(Main &bmain, bNode &import_geo_node)
{
  if (import_geo_node.id && GS(import_geo_node.id->name) == ID_NT) {
    bNodeTree *existing = reinterpret_cast<bNodeTree *>(import_geo_node.id);
    if (existing->type == NTREE_GEOMETRY) {
      mark_import_geo_locked(*existing);
      ensure_import_geo_tree_name_prefix(bmain, *existing);
      lock_import_geo_geometry_tree(*existing);
      return existing;
    }
  }

  bNodeTree *ntree = bke::node_tree_add_tree(
      &bmain, import_geo_tree_name_prefix.c_str(), tree_idname);
  if (!ntree) {
    return nullptr;
  }

  mark_import_geo_locked(*ntree);
  ensure_fixed_geo_group_output_interface(*ntree);

  /* Default graph: Cube → Group Output (Geometry). Replace with Object Info to import a
   * scene object. No Group Input node. */
  bNode *group_out = bke::node_add_static_node(nullptr, *ntree, NODE_GROUP_OUTPUT);
  if (group_out) {
    group_out->location[0] = 200.0f;
    group_out->location[1] = 0.0f;
    STRNCPY_UTF8(group_out->name, "Group Output");
  }
  bNode *cube = bke::node_add_node(nullptr, *ntree, "GeometryNodeMeshCube"_ustr);
  if (cube) {
    cube->location[0] = -280.0f;
    cube->location[1] = 40.0f;
  }

  if (import_geo_node.id && import_geo_node.id != &ntree->id) {
    id_us_min(import_geo_node.id);
  }
  if (import_geo_node.id != &ntree->id) {
    import_geo_node.id = &ntree->id;
    id_us_plus(import_geo_node.id);
  }

  BKE_ntree_update_after_single_tree_change(bmain, *ntree);

  if (group_out && cube) {
    ntree->ensure_topology_cache();
    bNodeSocket *mesh_out = bke::node_find_socket(*cube, SOCK_OUT, "Mesh"_ustr);
    bNodeSocket *geo_in = nullptr;
    for (bNodeSocket *sock : group_out->input_sockets()) {
      if (sock->is_available() && sock->name == StringRef("Geometry")) {
        geo_in = sock;
        break;
      }
    }
    if (mesh_out && geo_in && !geo_in->is_directly_linked()) {
      bke::node_add_link(*ntree, *cube, *mesh_out, *group_out, *geo_in);
      BKE_ntree_update_after_single_tree_change(bmain, *ntree);
    }
  }

  if (import_geo_node.runtime && import_geo_node.runtime->owner_tree &&
      import_geo_node.runtime->owner_tree->type == NTREE_IMAGE)
  {
    BKE_ntree_update_tag_node_property(import_geo_node.runtime->owner_tree, &import_geo_node);
  }

  return ntree;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Evaluate Import Points tree
 * \{ */

/**
 * Convert mesh verts, curve control points, and realized instances into a single point cloud.
 * Point Stamp / attribute write only operate on point domain data.
 */
static bke::GeometrySet convert_geometry_to_points(bke::GeometrySet geometry)
{
  if (geometry.is_empty()) {
    return geometry;
  }

  /* Realize instances so nested mesh/curve geometry becomes real components. */
  if (geometry.has_instances()) {
    geometry::RealizeInstancesOptions options;
    geometry = geometry::realize_instances(std::move(geometry), options).geometry;
  }

  /* Fast path: already only points. */
  if (geometry.has_pointcloud() && !geometry.has_mesh() && !geometry.has_curves() &&
      !geometry.has_grease_pencil() && !geometry.has_instances())
  {
    return geometry;
  }

  Vector<bke::GeometrySet> point_sets;

  if (geometry.has_pointcloud()) {
    bke::GeometrySet only = geometry;
    only.keep_only({bke::GeometryComponent::Type::PointCloud, bke::GeometryComponent::Type::Edit});
    if (!only.is_empty()) {
      point_sets.append(std::move(only));
    }
  }

  if (const Mesh *mesh = geometry.get_mesh()) {
    if (mesh->verts_num > 0) {
      PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, mesh->verts_num);
      const bke::AttributeAccessor src = mesh->attributes();
      bke::MutableAttributeAccessor dst = pc->attributes_for_write();
      /* Position. */
      {
        const VArraySpan<float3> pos = *src.lookup_or_default<float3>(
            "position", bke::AttrDomain::Point, float3(0.0f));
        pc->positions_for_write().copy_from(pos);
      }
      src.foreach_attribute([&](const bke::AttributeIter &iter) {
        if (iter.domain != bke::AttrDomain::Point) {
          return;
        }
        if (iter.name == "position" || iter.name == ".select_edge" || iter.name == ".select_poly") {
          return;
        }
        const bke::GAttributeReader reader = iter.get();
        if (!reader) {
          return;
        }
        const StringRef dst_name = iter.name == ".select_vert" ? ".selection" : iter.name;
        dst.add(dst_name,
                bke::AttrDomain::Point,
                iter.data_type,
                bke::AttributeInitVArray(reader.varray));
      });
      bke::GeometrySet set;
      set.replace_pointcloud(pc);
      point_sets.append(std::move(set));
    }
  }

  if (const Curves *curves_id = geometry.get_curves()) {
    const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    const int n = curves.points_num();
    if (n > 0) {
      PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, n);
      const bke::AttributeAccessor src = curves.attributes();
      bke::MutableAttributeAccessor dst = pc->attributes_for_write();
      pc->positions_for_write().copy_from(curves.positions());
      src.foreach_attribute([&](const bke::AttributeIter &iter) {
        if (iter.domain != bke::AttrDomain::Point) {
          return;
        }
        if (iter.name == "position") {
          return;
        }
        const bke::GAttributeReader reader = iter.get();
        if (!reader) {
          return;
        }
        dst.add(iter.name,
                bke::AttrDomain::Point,
                iter.data_type,
                bke::AttributeInitVArray(reader.varray));
      });
      bke::GeometrySet set;
      set.replace_pointcloud(pc);
      point_sets.append(std::move(set));
    }
  }

  if (point_sets.is_empty()) {
    return {};
  }
  if (point_sets.size() == 1) {
    return std::move(point_sets[0]);
  }
  return geometry::join_geometries(point_sets, {});
}

static void write_field_attribute(bke::GeometrySet &geometry,
                                  const StringRef name,
                                  const fn::GField &field)
{
  if (geometry.is_empty() || !geometry.has_pointcloud()) {
    return;
  }

  bke::GeometryComponent &component = geometry.get_component_for_write<bke::PointCloudComponent>();
  const bke::AttributeAccessor attributes = *component.attributes();
  if (!attributes.domain_supported(bke::AttrDomain::Point)) {
    return;
  }
  const int domain_size = attributes.domain_size(bke::AttrDomain::Point);
  if (domain_size <= 0) {
    return;
  }
  const CPPType &type = field.cpp_type();
  if (type.size == 0 || type.alignment == 0) {
    return;
  }

  /* Write via attribute writer (not AttributeInitMoveArray + custom allocator) so existing
   * attributes like "radius" stay valid and the buffer ownership is unambiguous. */
  bke::MutableAttributeAccessor write_attrs = *component.attributes_for_write();
  const bke::AttrType data_type = bke::cpp_type_to_attribute_type(type);
  if (const std::optional<bke::AttributeMetaData> meta = write_attrs.lookup_meta_data(name)) {
    if (meta->domain != bke::AttrDomain::Point || meta->data_type != data_type) {
      write_attrs.remove(name);
    }
  }

  bke::GAttributeWriter writer = write_attrs.lookup_or_add_for_write(
      name, bke::AttrDomain::Point, data_type);
  if (!writer) {
    return;
  }
  bke::GeometryFieldContext field_context{component, bke::AttrDomain::Point};
  fn::FieldEvaluator evaluator{field_context, domain_size};
  evaluator.add_with_destination(field, writer.varray);
  evaluator.evaluate();
  writer.finish();
}

static bke::GeometrySet evaluate_nested_import_tree(const bNodeTree &geometry_tree,
                                                    const bool as_points)
{
  if (geometry_tree.type != NTREE_GEOMETRY) {
    return {};
  }

  /* Re-entrancy guard: Point Stamp declare / Capture Attribute UI can re-enter evaluation while
   * sockets are mid-update — nested eval crashes. Return empty and wait for the outer cook. */
  static thread_local int evaluate_depth = 0;
  if (evaluate_depth > 0) {
    return {};
  }
  const struct EvaluateDepthGuard {
    EvaluateDepthGuard()
    {
      evaluate_depth++;
    }
    ~EvaluateDepthGuard()
    {
      evaluate_depth--;
    }
  } depth_guard;
  UNUSED_VARS(depth_guard);

  /* Do not mutate the tree during evaluation (removing nodes mid-declare/eval can crash).
   * Interface lock is applied from Import Points update / ensure paths. */

  const std::shared_ptr<const GeometryNodesLazyFunctionGraphInfo> &lf_graph_info =
      ensure_geometry_nodes_lazy_function_graph(geometry_tree);
  if (!lf_graph_info || !lf_graph_info->function.function) {
    return {};
  }

  const GeometryNodesGroupFunction &function = lf_graph_info->function;
  const lf::LazyFunction &lazy_function = *function.function;
  const int num_inputs = lazy_function.inputs().size();
  const int num_outputs = lazy_function.outputs().size();

  Array<GMutablePointer> param_inputs(num_inputs);
  Array<GMutablePointer> param_outputs(num_outputs);
  Array<std::optional<lf::ValueUsage>> param_input_usages(num_inputs);
  Array<lf::ValueUsage> param_output_usages(num_outputs);
  Array<bool> param_set_outputs(num_outputs, false);

  if (!function.outputs.main.is_empty()) {
    param_output_usages.as_mutable_span().slice(function.outputs.main).fill(lf::ValueUsage::Used);
  }
  if (!function.outputs.input_usages.is_empty()) {
    param_output_usages.as_mutable_span()
        .slice(function.outputs.input_usages)
        .fill(lf::ValueUsage::Unused);
  }

  /* Eval log is expensive (socket value logging). Only build it when not inside Image Process
   * cook (active_cache set during cook). UI attribute search can use the last cook's host log. */
  std::unique_ptr<eval_log::NodesEvalLog> eval_log;
  if (active_cache() == nullptr) {
    eval_log = std::make_unique<eval_log::NodesEvalLog>();
  }

  /* Dummy self object so Object Info / Collection Info do not null-deref when called from Image
   * Process (no modifier / operator context). Without a depsgraph, DEG_*_is_evaluated returns
   * true; Object Info can still read original mesh data via object_get_evaluated_geometry_set. */
  Object dummy_self = dna::shallow_zero_initialize();
  BLI_strncpy(dummy_self.id.name, "OBImportPointsSelf", sizeof(dummy_self.id.name));
  GeoNodesModifierData modifier_data{};
  modifier_data.self_object = &dummy_self;
  modifier_data.depsgraph = nullptr;

  GeoNodesCallData call_data{};
  call_data.root_ntree = &geometry_tree;
  call_data.call_depth_limit = 64;
  call_data.eval_log = eval_log.get();
  call_data.modifier_data = &modifier_data;

  GeoNodesUserData user_data;
  user_data.call_data = &call_data;

  /* Must match get_node_editor_root_compute_context for nested Import Points (DataBlock of geo
   * tree) so attribute search / inspect logs resolve. */
  bke::DataBlockComputeContext base_compute_context(nullptr, geometry_tree.id);
  user_data.compute_context = &base_compute_context;

  ResourceScope scope;
  LinearAllocator<> &allocator = scope.allocator();

  geometry_tree.ensure_interface_cache();

  /* No group inputs expected. If the LF graph still exposes main inputs (stale interface),
   * supply empty SocketValueVariant so Capture Attribute graphs do not read null. */
  for (const int i : function.inputs.main.index_range()) {
    const int param_i = function.inputs.main[i];
    if (param_i < 0 || param_i >= num_inputs) {
      continue;
    }
    bke::SocketValueVariant &value = scope.construct<bke::SocketValueVariant>();
    param_inputs[param_i] = &value;
  }

  /* Prepare output-usage inputs (anonymous attribute lifetimes for Capture Attribute). */
  Array<bool> output_used_inputs(geometry_tree.interface_outputs().size(), true);
  for (const int i : geometry_tree.interface_outputs().index_range()) {
    if (i >= function.inputs.output_usages.size()) {
      break;
    }
    const int param_i = function.inputs.output_usages[i];
    if (param_i >= 0 && param_i < num_inputs) {
      param_inputs[param_i] = &output_used_inputs[i];
    }
  }

  Array<bke::GeometryNodesReferenceSet> references_to_propagate(
      function.inputs.references_to_propagate.geometry_outputs.size());
  for (const int i : references_to_propagate.index_range()) {
    const int param_i = function.inputs.references_to_propagate.range[i];
    if (param_i >= 0 && param_i < num_inputs) {
      param_inputs[param_i] = &references_to_propagate[i];
    }
  }

  for (const int i : IndexRange(num_outputs)) {
    const lf::Output &lf_output = lazy_function.outputs()[i];
    const CPPType &type = *lf_output.type;
    void *buffer = allocator.allocate(type);
    param_outputs[i] = {type, buffer};
  }

  GeoNodesLocalUserData local_user_data(user_data);
  lf::Context lf_context(lazy_function.init_storage(allocator), &user_data, &local_user_data);
  lf::BasicParams lf_params{lazy_function,
                            param_inputs,
                            param_outputs,
                            param_input_usages,
                            param_output_usages,
                            param_set_outputs};
  lazy_function.execute(lf_params, lf_context);
  lazy_function.destruct_storage(lf_context.storage);

  /* Keep eval log for UI attribute search when we created one (non-cook path). */
  if (eval_log) {
    const_cast<bNodeTree &>(geometry_tree).runtime->image_process_eval_log = std::move(eval_log);
  }

  if (function.outputs.main.is_empty()) {
    for (const int i : IndexRange(num_outputs)) {
      if (param_set_outputs[i]) {
        param_outputs[i].destruct();
      }
    }
    return {};
  }

  /* Main outputs follow interface order: Points, Rotation, Size, ID. */
  bke::GeometrySet geometry;
  const int main_start = function.outputs.main.start();
  if (param_set_outputs[main_start]) {
    bke::SocketValueVariant *variant = param_outputs[main_start].get<bke::SocketValueVariant>();
    if (variant) {
      geometry = variant->extract<bke::GeometrySet>();
    }
  }

  if (as_points) {
    /* Mesh verts / curve points / instances → unified point cloud before writing attrs / stamp.
     * Preserves all Point-domain attributes (Store Named Attribute, radius, etc.). */
    geometry = convert_geometry_to_points(std::move(geometry));
  }
  else if (geometry.has_instances()) {
    geometry::RealizeInstancesOptions options;
    geometry = geometry::realize_instances(std::move(geometry), options).geometry;
  }

  auto try_write_field = [&](const int interface_index, const StringRef attr_name) {
    if (geometry.is_empty() || !geometry.has_pointcloud()) {
      return;
    }
    const int param_index = main_start + interface_index;
    if (param_index >= num_outputs || !param_set_outputs[param_index]) {
      return;
    }
    bke::SocketValueVariant *variant = param_outputs[param_index].get<bke::SocketValueVariant>();
    if (!variant) {
      return;
    }
    if (variant->is_field()) {
      write_field_attribute(geometry, attr_name, variant->copy_as<fn::GField>());
    }
    else if (variant->is_single()) {
      /* Broadcast single values as constant attributes. */
      const GPointer single = variant->get_single_ptr();
      if (!single || !single.type()) {
        return;
      }
      const CPPType &type = *single.type();
      fn::GField field = fn::GField::from_constant(type, single.get());
      write_field_attribute(geometry, attr_name, field);
    }
  };

  /* Only write fixed Import Points Group Output sockets. Import Geo has Geometry only. */
  if (as_points) {
    try_write_field(1, attr_rotation);
    try_write_field(2, attr_size);
    try_write_field(3, attr_id);
  }

  for (const int i : IndexRange(num_outputs)) {
    if (param_set_outputs[i]) {
      param_outputs[i].destruct();
    }
  }

  /* Keep a copy for Point Stamp Bundle declare/update (never re-eval mid tree-update).
   * GeometrySet is share-counted; overwriting the same tree key does not grow unbounded. */
  if (!geometry.is_empty()) {
    store_last_import_points_geometry(geometry_tree, geometry);
  }

  return geometry;
}

bke::GeometrySet evaluate_import_points_tree(const bNodeTree &geometry_tree)
{
  return evaluate_nested_import_tree(geometry_tree, true);
}

bke::GeometrySet evaluate_import_geo_tree(const bNodeTree &geometry_tree)
{
  return evaluate_nested_import_tree(geometry_tree, false);
}

/** \} */

}  // namespace blender::nodes::image_points
