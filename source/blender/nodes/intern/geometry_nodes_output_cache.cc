/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_geometry_nodes_output_cache.hh"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

#include "BLI_function_ref.hh"
#include "BLI_generic_span.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_hash.hh"
#include "BLI_implicit_sharing.hh"
#include "BLI_listbase.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"
#include "BLI_string.hh"
#include "BLI_vector_set.hh"

#include "NOD_geometry_nodes_lazy_function.hh"

#include "DNA_ID.h"
#include "DNA_action_types.h"
#include "DNA_anim_types.h"
#include "DNA_camera_types.h"
#include "DNA_collection_types.h"
#include "DNA_curves_types.h"
#include "DNA_customdata_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_node_tree_interface_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_pointcloud_types.h"
#include "DNA_scene_types.h"

#include "BKE_anim_data.hh"
#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_customdata.hh"
#include "BKE_geometry_nodes_reference_set.hh"
#include "BKE_geometry_set.hh"
#include "BKE_geometry_set_instances.hh"
#include "BKE_idprop.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_types.hh"
#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_node_runtime.hh"
#include "BKE_object.hh"
#include "BKE_pointcloud.hh"

#include "DEG_depsgraph_build.hh"
#include "DEG_depsgraph_query.hh"

#include "FN_lazy_function.hh"

#include "NOD_dependencies.hh"
#include "NOD_node_declaration.hh"

#include "RNA_access.hh"
#include "RNA_path.hh"

namespace blender::nodes {

using bke::SocketValueVariant;
namespace lf = fn::lazy_function;

/* -------------------------------------------------------------------- */
/** \name Hashing
 * \{ */

static uint64_t mix(const uint64_t seed, const uint64_t value)
{
  uint64_t x = seed ^ (value + 0x9E3779B97F4A7C15ull + (seed << 6) + (seed >> 2));
  x ^= x >> 33;
  x *= 0xFF51AFD7ED558CCDull;
  x ^= x >> 33;
  x *= 0xC4CEB9FE1A85EC53ull;
  x ^= x >> 33;
  return x;
}

static uint64_t hash_bytes(const void *data, const int64_t size)
{
  if (data == nullptr || size <= 0) {
    return 0;
  }
  const ComputeContextHash hash = ComputeContextHash::from_bytes(data, size);
  return mix(hash.v1, hash.v2);
}

static uint64_t hash_text(const StringRef str)
{
  return hash_bytes(str.data(), str.size());
}

/**
 * Session uid of the original data-block. Evaluated copies do not carry one (it reads as
 * zero), so comparing theirs would make every data-block look like the same one.
 */
static uint32_t stable_session_uid(const ID *id)
{
  if (id == nullptr) {
    return 0;
  }
  if (const ID *id_orig = DEG_get_original(id)) {
    return id_orig->session_uid;
  }
  return id->session_uid;
}

/** A token that never equals an earlier one, for data whose content cannot be identified. */
static uint64_t unique_token()
{
  static std::atomic<uint64_t> counter{1};
  return mix(0x554E4951ull, counter.fetch_add(1, std::memory_order_relaxed));
}

/**
 * Content hashes of shared arrays, remembered per #ImplicitSharingInfo. An array that is still
 * the same shared data at the same version is not read again, so an unchanged input geometry
 * costs one map lookup per attribute.
 */
struct GeometryNodesOutputCache::ContentHasher {
  struct Memo {
    int64_t version = 0;
    int64_t size = 0;
    uint64_t hash = 0;
  };
  Map<const ImplicitSharingInfo *, Memo> memo;

  ~ContentHasher()
  {
    for (const ImplicitSharingInfo *info : memo.keys()) {
      info->remove_weak_user_and_delete_if_last();
    }
  }

  uint64_t shared(const ImplicitSharingInfo *info,
                  const int64_t size,
                  const FunctionRef<uint64_t()> compute)
  {
    if (info == nullptr) {
      return compute();
    }
    const int64_t version = info->version();
    if (Memo *known = memo.lookup_ptr(info)) {
      if (known->version != version || known->size != size) {
        *known = {version, size, compute()};
      }
      return known->hash;
    }
    /* The weak user keeps the address from being reused by another array. */
    info->add_weak_user();
    const uint64_t hash = compute();
    memo.add_new(info, {version, size, hash});
    return hash;
  }

  uint64_t bytes(const void *data, const int64_t size, const ImplicitSharingInfo *info)
  {
    return this->shared(info, size, [&]() { return hash_bytes(data, size); });
  }

  void remove_expired()
  {
    memo.remove_if([](const auto &item) {
      if (item.key->is_expired()) {
        item.key->remove_weak_user_and_delete_if_last();
        return true;
      }
      return false;
    });
  }
};

static uint64_t attributes_token(const bke::AttributeAccessor &attributes,
                                 GeometryNodesOutputCache::ContentHasher &hasher)
{
  /* Summed, so the storage order of attributes does not matter. */
  uint64_t token = 0;
  attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    const bke::GAttributeReader reader = iter.get();
    if (!reader) {
      return;
    }
    const GVArray &varray = reader.varray;
    const CPPType &type = varray.type();
    const int64_t size = varray.size();
    uint64_t part = mix(hash_text(iter.name), uint64_t(iter.domain));
    part = mix(part, uint64_t(iter.data_type));
    part = mix(part, uint64_t(size));
    if (!type.is_trivial) {
      part = mix(part, unique_token());
    }
    else if (varray.is_span()) {
      const GSpan span = varray.get_internal_span();
      part = mix(part, hasher.bytes(span.data(), size * type.size, reader.sharing_info));
    }
    else if (varray.is_single()) {
      BUFFER_FOR_CPP_TYPE_VALUE(type, buffer);
      varray.get_internal_single(buffer);
      part = mix(part, hash_bytes(buffer, type.size));
    }
    else {
      /* Vertex groups show up as computed arrays. They are hashed from the weights instead. */
      return;
    }
    token += part;
  });
  return token;
}

static uint64_t deform_verts_token(const Span<MDeformVert> dverts,
                                   const ImplicitSharingInfo *sharing_info,
                                   GeometryNodesOutputCache::ContentHasher &hasher)
{
  if (dverts.is_empty()) {
    return 0;
  }
  return hasher.shared(sharing_info, dverts.size(), [&]() {
    uint64_t token = uint64_t(dverts.size());
    for (const MDeformVert &dvert : dverts) {
      token = mix(token, uint64_t(dvert.totweight));
      if (dvert.dw != nullptr && dvert.totweight > 0) {
        token = mix(token, hash_bytes(dvert.dw, sizeof(MDeformWeight) * dvert.totweight));
      }
    }
    return token;
  });
}

static uint64_t materials_token(Material *const *materials, const int materials_num)
{
  uint64_t token = uint64_t(materials_num);
  for (int i = 0; i < materials_num; i++) {
    const Material *material = materials ? materials[i] : nullptr;
    token = mix(token, stable_session_uid(reinterpret_cast<const ID *>(material)));
  }
  return token;
}

static uint64_t mesh_token(const Mesh &mesh, GeometryNodesOutputCache::ContentHasher &hasher)
{
  uint64_t token = mix(0x4D455348ull, uint64_t(mesh.verts_num));
  token = mix(token, uint64_t(mesh.edges_num));
  token = mix(token, uint64_t(mesh.faces_num));
  token = mix(token, uint64_t(mesh.corners_num));
  token = mix(token, attributes_token(mesh.attributes(), hasher));
  const Span<int> face_offsets = mesh.face_offsets();
  token = mix(token,
              hasher.bytes(face_offsets.data(),
                           face_offsets.size_in_bytes(),
                           mesh.runtime->face_offsets_sharing_info));
  const int dvert_layer = CustomData_get_layer_index(&mesh.vert_data, CD_MDEFORMVERT);
  if (dvert_layer != -1) {
    token = mix(token,
                deform_verts_token(
                    mesh.deform_verts(), mesh.vert_data.layers[dvert_layer].sharing_info, hasher));
    for (const bDeformGroup &group : mesh.vertex_group_names) {
      token = mix(token, hash_text(group.name));
    }
  }
  token = mix(token, materials_token(mesh.mat, mesh.totcol));
  return token;
}

static uint64_t curves_token(const Curves &curves_id,
                             GeometryNodesOutputCache::ContentHasher &hasher)
{
  const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
  uint64_t token = mix(0x43555256ull, uint64_t(curves.points_num()));
  token = mix(token, uint64_t(curves.curves_num()));
  token = mix(token, attributes_token(curves.attributes(), hasher));
  const Span<int> offsets = curves.offsets();
  token = mix(token,
              hasher.bytes(offsets.data(),
                           offsets.size_in_bytes(),
                           curves.runtime->curve_offsets_sharing_info));
  const Span<float> knots = curves.nurbs_custom_knots();
  token = mix(token,
              hasher.bytes(
                  knots.data(), knots.size_in_bytes(), curves.runtime->custom_knots_sharing_info));
  token = mix(token, materials_token(curves_id.mat, curves_id.totcol));
  return token;
}

static uint64_t geometry_token(const bke::GeometrySet &geometry,
                               GeometryNodesOutputCache::ContentHasher &hasher);

static uint64_t instances_token(const bke::Instances &instances,
                                GeometryNodesOutputCache::ContentHasher &hasher)
{
  uint64_t token = mix(0x494E5354ull, uint64_t(instances.instances_num()));
  token = mix(token, attributes_token(instances.attributes(), hasher));
  for (const bke::InstanceReference &reference : instances.references()) {
    token = mix(token, uint64_t(reference.type()));
    switch (reference.type()) {
      case bke::InstanceReference::Type::Object:
        token = mix(token, stable_session_uid(&reference.object().id));
        break;
      case bke::InstanceReference::Type::Collection:
        token = mix(token, stable_session_uid(&reference.collection().id));
        break;
      case bke::InstanceReference::Type::GeometrySet:
        token = mix(token, geometry_token(reference.geometry_set(), hasher));
        break;
      case bke::InstanceReference::Type::None:
        break;
    }
  }
  return token;
}

/**
 * Identity of a geometry's content. Equal tokens mean equal data; pointers of the geometry
 * itself are never used, because the depsgraph hands out a new #Mesh on every evaluation.
 */
static uint64_t geometry_token(const bke::GeometrySet &geometry,
                               GeometryNodesOutputCache::ContentHasher &hasher)
{
  uint64_t token = 0x47454F4Dull;
  if (const Mesh *mesh = geometry.get_mesh()) {
    token = mix(token, mesh_token(*mesh, hasher));
  }
  if (const PointCloud *pointcloud = geometry.get_pointcloud()) {
    token = mix(token, mix(0x50434C44ull, uint64_t(pointcloud->totpoint)));
    token = mix(token, attributes_token(pointcloud->attributes(), hasher));
    token = mix(token, materials_token(pointcloud->mat, pointcloud->totcol));
  }
  if (const Curves *curves = geometry.get_curves()) {
    token = mix(token, curves_token(*curves, hasher));
  }
  if (const bke::Instances *instances = geometry.get_instances()) {
    token = mix(token, instances_token(*instances, hasher));
  }
  if (geometry.has_volume() || geometry.has_grease_pencil()) {
    /* No cheap content identity for these yet, so they always count as changed. */
    token = mix(token, unique_token());
  }
  return token;
}

static uint64_t idproperty_token(const IDProperty &prop)
{
  uint64_t token = mix(uint64_t(prop.type), hash_text(prop.name));
  switch (prop.type) {
    case IDP_STRING: {
      const char *str = static_cast<const char *>(prop.data.pointer);
      token = mix(token, str ? hash_text(str) : 0);
      break;
    }
    case IDP_INT:
    case IDP_FLOAT:
    case IDP_DOUBLE:
    case IDP_BOOLEAN: {
      token = mix(token, uint64_t(uint32_t(prop.data.val)));
      token = mix(token, uint64_t(uint32_t(prop.data.val2)));
      break;
    }
    case IDP_ARRAY: {
      int64_t item_size = 0;
      switch (prop.subtype) {
        case IDP_INT:
        case IDP_FLOAT:
          item_size = 4;
          break;
        case IDP_DOUBLE:
          item_size = 8;
          break;
        case IDP_BOOLEAN:
          item_size = 1;
          break;
        default:
          break;
      }
      token = mix(token, uint64_t(prop.len));
      token = mix(token, hash_bytes(prop.data.pointer, item_size * prop.len));
      break;
    }
    case IDP_GROUP: {
      for (const IDProperty &child : prop.data.group) {
        token = mix(token, idproperty_token(child));
      }
      break;
    }
    case IDP_ID: {
      token = mix(token, stable_session_uid(static_cast<const ID *>(prop.data.pointer)));
      break;
    }
    case IDP_IDPARRAY: {
      const IDProperty *items = static_cast<const IDProperty *>(prop.data.pointer);
      for (int i = 0; i < prop.len; i++) {
        token = mix(token, idproperty_token(items[i]));
      }
      break;
    }
    default:
      break;
  }
  return token;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Debug Output
 * \{ */

static bool debug_enabled()
{
  static const bool enabled = std::getenv("BLENDER_GN_CACHE_DEBUG") != nullptr;
  return enabled;
}

static void debug_print(const char *event, const bNode &node, const char *detail)
{
  if (!debug_enabled()) {
    return;
  }
  fprintf(stderr, "GN_CACHE %s node=\"%s\"%s%s\n", event, node.name, detail ? " " : "", detail ? detail : "");
  fflush(stderr);
}

/** \} */

bool socket_value_is_output_cacheable(const SocketValueVariant &value)
{
  if (value.is_field() || value.is_volume_grid()) {
    return false;
  }
  if (value.is_list()) {
    return true;
  }
  if (!value.is_single()) {
    return false;
  }
  switch (value.socket_type()) {
    case SOCK_CLOSURE:
    case SOCK_BUNDLE:
    case SOCK_SHADER:
    case SOCK_CUSTOM:
      return false;
    default:
      return true;
  }
}

/* -------------------------------------------------------------------- */
/** \name Tree Edit Generations
 * \{ */

struct GeometryNodesOutputCacheTreeState {
  Mutex mutex;
  uint64_t topology_gen = 0;
  Map<int32_t, uint64_t> node_gen;
  /** Node identifiers tagged since the last propagate (property, socket, link target, …). */
  VectorSet<int32_t> dirty_seeds;
};

static Map<uint32_t, std::unique_ptr<GeometryNodesOutputCacheTreeState>> &tree_states()
{
  static Map<uint32_t, std::unique_ptr<GeometryNodesOutputCacheTreeState>> states;
  return states;
}

static Mutex &tree_states_mutex()
{
  static Mutex mutex;
  return mutex;
}

static uint32_t original_tree_session_uid(const bNodeTree &tree)
{
  /* Lazy-function evaluation runs on an independent tree copy. DEG_get_original does not map
   * that copy back, but the graph info stores the original session uid at build time. */
  if (tree.runtime && tree.runtime->self_geometry_nodes_lazy_function_graph_info) {
    return tree.runtime->self_geometry_nodes_lazy_function_graph_info->original_tree_session_uid;
  }
  if (const ID *id_orig = DEG_get_original(&tree.id)) {
    return id_orig->session_uid;
  }
  return tree.id.session_uid;
}

static GeometryNodesOutputCacheTreeState &tree_state_for(const uint32_t session_uid)
{
  std::lock_guard lock{tree_states_mutex()};
  std::unique_ptr<GeometryNodesOutputCacheTreeState> &state_ptr = tree_states().lookup_or_add_cb(
      session_uid, []() { return std::make_unique<GeometryNodesOutputCacheTreeState>(); });
  return *state_ptr;
}

static Set<GeometryNodesOutputCache *> &registered_caches()
{
  static Set<GeometryNodesOutputCache *> caches;
  return caches;
}

static Mutex &registered_caches_mutex()
{
  static Mutex mutex;
  return mutex;
}

GeometryNodesOutputCache::GeometryNodesOutputCache()
{
  std::lock_guard lock{registered_caches_mutex()};
  registered_caches().add(this);
}

GeometryNodesOutputCache::~GeometryNodesOutputCache()
{
  std::lock_guard lock{registered_caches_mutex()};
  registered_caches().remove(this);
}

void GeometryNodesOutputCache::discard_node(const uint32_t tree_session_uid, const int32_t node_id)
{
  std::lock_guard lock{mutex_};
  entries_.remove_if([&](const auto &item) {
    return item.key.tree_session_uid == tree_session_uid && item.key.node_id == node_id;
  });
  NodeRef ref;
  ref.tree_session_uid = tree_session_uid;
  ref.node_id = node_id;
  frozen_time_ns_.remove(ref);
  frozen_time_state_.remove(ref);
}

static Vector<GeometryNodesOutputCache *> snapshot_registered_caches()
{
  std::lock_guard lock{registered_caches_mutex()};
  Vector<GeometryNodesOutputCache *> caches;
  caches.reserve(registered_caches().size());
  for (GeometryNodesOutputCache *cache : registered_caches()) {
    caches.append(cache);
  }
  return caches;
}

void geometry_nodes_output_cache_discard_node(bNodeTree *ntree, const bNode *node)
{
  if (ntree == nullptr || node == nullptr) {
    return;
  }
  const uint32_t uid = original_tree_session_uid(*ntree);
  /* Copy the registry first so we never hold registered_caches_mutex_ and a cache mutex together
   * (that deadlocked the UI toggle against a concurrent modifier eval). */
  const Vector<GeometryNodesOutputCache *> caches = snapshot_registered_caches();
  for (GeometryNodesOutputCache *cache : caches) {
    cache->discard_node(uid, node->identifier);
  }
}

int GeometryNodesOutputCache::debug_count_node(const uint32_t tree_session_uid,
                                               const int32_t node_id) const
{
  std::lock_guard lock{mutex_};
  int count = 0;
  for (const auto &item : entries_.items()) {
    if (item.key.tree_session_uid == tree_session_uid && item.key.node_id == node_id) {
      count++;
    }
  }
  return count;
}

int geometry_nodes_output_cache_debug_count(const bNodeTree *ntree, const bNode *node)
{
  if (ntree == nullptr || node == nullptr) {
    return 0;
  }
  const uint32_t uid = original_tree_session_uid(*ntree);
  int count = 0;
  const Vector<GeometryNodesOutputCache *> caches = snapshot_registered_caches();
  for (GeometryNodesOutputCache *cache : caches) {
    count += cache->debug_count_node(uid, node->identifier);
  }
  return count;
}

uint32_t geometry_nodes_output_cache_tree_uid(const bNodeTree &tree)
{
  return original_tree_session_uid(tree);
}

/**
 * Hash the unlinked input values of a node.
 *
 * \param r_complete: Set to false when a value could not be read. The hash then does not
 * identify the node's inputs on its own.
 */
static uint64_t hash_node_inputs(const bNode &node, bool *r_complete)
{
  /* Use SOCK_IS_LINKED (maintained by tree update), not is_directly_linked(), so eval/cache
   * hits never rebuild topology under a mutex that ntree update may already hold. */
  uint64_t h = uint64_t(node.identifier);
  for (const bNodeSocket *socket : node.input_sockets()) {
    if (!socket->is_available()) {
      continue;
    }
    const bool linked = (socket->flag & SOCK_IS_LINKED) != 0;
    h = get_default_hash(h, socket->index(), int(socket->type), linked);
    if (linked) {
      continue;
    }
    const void *value = socket->default_value;
    if (value == nullptr) {
      continue;
    }
    const auto id_hash = [](const ID *id) { return uint64_t(stable_session_uid(id)); };
    switch (eNodeSocketDatatype(socket->type)) {
      case SOCK_FLOAT:
        h = mix(h, hash_bytes(&static_cast<const bNodeSocketValueFloat *>(value)->value, 4));
        break;
      case SOCK_INT:
        h = mix(h, uint64_t(uint32_t(static_cast<const bNodeSocketValueInt *>(value)->value)));
        break;
      case SOCK_BOOLEAN:
        h = mix(h, uint64_t(static_cast<const bNodeSocketValueBoolean *>(value)->value));
        break;
      case SOCK_VECTOR: {
        const auto &typed = *static_cast<const bNodeSocketValueVector *>(value);
        h = mix(h, hash_bytes(typed.value, sizeof(typed.value)));
        break;
      }
      case SOCK_RGBA: {
        const auto &typed = *static_cast<const bNodeSocketValueRGBA *>(value);
        h = mix(h, hash_bytes(typed.value, sizeof(typed.value)));
        break;
      }
      case SOCK_ROTATION: {
        const auto &typed = *static_cast<const bNodeSocketValueRotation *>(value);
        h = mix(h, hash_bytes(typed.value_euler, sizeof(typed.value_euler)));
        break;
      }
      case SOCK_STRING:
        h = mix(h, hash_text(static_cast<const bNodeSocketValueString *>(value)->value));
        break;
      case SOCK_MENU:
        h = mix(h, uint64_t(uint32_t(static_cast<const bNodeSocketValueMenu *>(value)->value)));
        break;
      case SOCK_OBJECT:
        h = mix(h, id_hash(reinterpret_cast<const ID *>(
                       static_cast<const bNodeSocketValueObject *>(value)->value)));
        break;
      case SOCK_COLLECTION:
        h = mix(h, id_hash(reinterpret_cast<const ID *>(
                       static_cast<const bNodeSocketValueCollection *>(value)->value)));
        break;
      case SOCK_MATERIAL:
        h = mix(h, id_hash(reinterpret_cast<const ID *>(
                       static_cast<const bNodeSocketValueMaterial *>(value)->value)));
        break;
      case SOCK_IMAGE:
        h = mix(h, id_hash(reinterpret_cast<const ID *>(
                       static_cast<const bNodeSocketValueImage *>(value)->value)));
        break;
      case SOCK_GEOMETRY:
      case SOCK_MATRIX:
      case SOCK_BUNDLE:
      case SOCK_CLOSURE:
        /* Nothing to read: unlinked sockets of these types always hold the same value. */
        break;
      default:
        if (r_complete != nullptr) {
          *r_complete = false;
        }
        break;
    }
  }
  return h;
}

uint64_t geometry_nodes_output_cache_hash_node_inputs(const bNode &node)
{
  return hash_node_inputs(node, nullptr);
}

void geometry_nodes_output_cache_copy_dirty_seeds(const bNodeTree &ntree, Vector<int32_t> &r_seeds)
{
  if (ntree.type != NTREE_GEOMETRY || !DEG_is_original(&ntree.id)) {
    return;
  }
  GeometryNodesOutputCacheTreeState &state = tree_state_for(original_tree_session_uid(ntree));
  std::lock_guard lock{state.mutex};
  for (const int32_t id : state.dirty_seeds) {
    r_seeds.append(id);
  }
}

bool geometry_nodes_output_cache_only_viewer_seeds(const bNodeTree &ntree)
{
  if (ntree.type != NTREE_GEOMETRY || !DEG_is_original(&ntree.id)) {
    return false;
  }
  ntree.ensure_topology_cache();
  GeometryNodesOutputCacheTreeState &state = tree_state_for(original_tree_session_uid(ntree));
  std::lock_guard lock{state.mutex};
  if (state.dirty_seeds.is_empty()) {
    return false;
  }
  for (const int32_t id : state.dirty_seeds) {
    const bNode *node = ntree.node_by_id(id);
    if (node == nullptr || node->type_legacy != GEO_NODE_VIEWER) {
      return false;
    }
  }
  return true;
}

void geometry_nodes_output_cache_tag_node(bNodeTree *ntree, const bNode *node)
{
  if (ntree == nullptr || ntree->type != NTREE_GEOMETRY || node == nullptr) {
    return;
  }
  /* Evaluated copies must not bump the original tree's generation every depsgraph eval. */
  if (!DEG_is_original(&ntree->id)) {
    return;
  }
  GeometryNodesOutputCacheTreeState &state = tree_state_for(original_tree_session_uid(*ntree));
  std::lock_guard lock{state.mutex};
  state.dirty_seeds.add(node->identifier);
}

static bool tree_flag_is_unknown_full_invalidation(const uint64_t changed_flag)
{
  /* Only unknown-scope edits (not link/remove — those tag specific downstream seeds). */
  constexpr uint64_t unknown_bits = (1u << 1); /* NTREE_CHANGED_ANY */
  if (changed_flag == uint64_t(-1)) {          /* NTREE_CHANGED_ALL */
    return true;
  }
  return (changed_flag & unknown_bits) != 0;
}

/**
 * Visit the start nodes and every node that can receive a value from them.
 *
 * Zones pass values without links: the output node of every zone depends on its input node,
 * and in zones that run more than once (repeat, simulation) the input node reads what the
 * output node produced in the previous step.
 */
static void foreach_downstream(const bNodeTree &tree,
                               const Span<const bNode *> starts,
                               const FunctionRef<void(const bNode &)> visit)
{
  tree.ensure_topology_cache();
  Stack<const bNode *> stack;
  Set<const bNode *> visited;
  for (const bNode *start : starts) {
    stack.push(start);
  }
  while (!stack.is_empty()) {
    const bNode *node = stack.pop();
    if (!visited.add(node)) {
      continue;
    }
    visit(*node);
    for (const bNodeSocket *output : node->output_sockets()) {
      if (!output->is_available()) {
        continue;
      }
      for (const bNodeLink *link : output->directly_linked_links()) {
        if (link->is_muted() || !link->is_available()) {
          continue;
        }
        if (link->tonode != nullptr) {
          stack.push(link->tonode);
        }
      }
    }
    if (const bke::bNodeZoneType *zone_type = bke::zone_type_by_node_type(node->type_legacy)) {
      if (node->type_legacy == zone_type->input_type) {
        if (zone_type->output_id_requires_storage() && node->storage == nullptr) {
          continue;
        }
        if (const bNode *output_node = zone_type->get_corresponding_output(tree, *node)) {
          stack.push(output_node);
        }
      }
      else if (ELEM(node->type_legacy, GEO_NODE_REPEAT_OUTPUT, GEO_NODE_SIMULATION_OUTPUT)) {
        if (const bNode *input_node = zone_type->get_corresponding_input(tree, *node)) {
          stack.push(input_node);
        }
      }
    }
  }
}

static void collect_downstream_bumps(const bNodeTree &tree,
                                     const bNode &start,
                                     Vector<std::pair<uint32_t, int32_t>> &r_nodes)
{
  const uint32_t uid = original_tree_session_uid(tree);
  /* Do not recursively wipe every node inside a group. Nested trees have their own gens, and
   * inner nodes mix the state of the calling group node into theirs, so a change that reaches
   * the group node already makes them miss. */
  foreach_downstream(tree, {&start}, [&](const bNode &node) {
    r_nodes.append({uid, node.identifier});
  });
}

static void apply_gen_bumps(const Span<std::pair<uint32_t, int32_t>> bumps)
{
  for (const auto &[uid, node_id] : bumps) {
    GeometryNodesOutputCacheTreeState &state = tree_state_for(uid);
    std::lock_guard lock{state.mutex};
    uint64_t &gen = state.node_gen.lookup_or_add(node_id, 0);
    gen++;
  }
}

void geometry_nodes_output_cache_propagate(bNodeTree &ntree)
{
  if (ntree.type != NTREE_GEOMETRY) {
    return;
  }
  /* COW / localized copies of the tree run updates too; generations are owned by the original. */
  if (!DEG_is_original(&ntree.id)) {
    return;
  }

  ntree.ensure_topology_cache();

  VectorSet<int32_t> seed_ids;
  {
    GeometryNodesOutputCacheTreeState &state = tree_state_for(original_tree_session_uid(ntree));
    std::lock_guard lock{state.mutex};
    for (const int32_t id : state.dirty_seeds) {
      seed_ids.add(id);
    }
    state.dirty_seeds.clear();
  }

  for (const bNode *node : ntree.all_nodes()) {
    const uint64_t node_flag = node->runtime->changed_flag;
    /* PARENT / INTERNAL_LINK: mute-link refresh on ANY/LINK. Socket flags: declaration
     * refresh tags every reused socket. Neither means this node's output changed. */
    if (node_flag & ((1u << 2) | (1u << 3))) { /* NODE_PROPERTY | NODE_OUTPUT */
      seed_ids.add(node->identifier);
    }
  }

  /* Viewer preview (Ctrl-Shift or activate) tags only Viewer nodes. Those have no downstream
   * producers; treating them as unknown ANY used to full-invalidate every node_gen. */
  VectorSet<int32_t> producer_seeds;
  for (const int32_t id : seed_ids) {
    const bNode *seed = ntree.node_by_id(id);
    if (seed == nullptr) {
      continue;
    }
    if (seed->type_legacy == GEO_NODE_VIEWER) {
      continue;
    }
    producer_seeds.add(id);
  }

  if (producer_seeds.is_empty()) {
    if (seed_ids.is_empty() &&
        tree_flag_is_unknown_full_invalidation(ntree.runtime->changed_flag))
    {
      GeometryNodesOutputCacheTreeState &state = tree_state_for(original_tree_session_uid(ntree));
      std::lock_guard lock{state.mutex};
      state.topology_gen++;
      state.node_gen.clear();
      const uint32_t uid = original_tree_session_uid(ntree);
      ntree.ensure_topology_cache();
      const Vector<GeometryNodesOutputCache *> caches = snapshot_registered_caches();
      for (GeometryNodesOutputCache *cache : caches) {
        for (const bNode *node : ntree.all_nodes()) {
          cache->discard_node(uid, node->identifier);
        }
      }
    }
    return;
  }

  Vector<std::pair<uint32_t, int32_t>> bumps;
  for (const int32_t id : producer_seeds) {
    const bNode *seed = ntree.node_by_id(id);
    if (seed == nullptr) {
      continue;
    }
    collect_downstream_bumps(ntree, *seed, bumps);
  }
  apply_gen_bumps(bumps);

  /* Free stored geometry/lists for dirtied nodes now, not on the next store. Otherwise a List
   * parameter edit kept the old key and leaked RAM on every change. */
  Set<GeometryNodesOutputCache::NodeRef> unique;
  for (const auto &[uid, node_id] : bumps) {
    GeometryNodesOutputCache::NodeRef ref;
    ref.tree_session_uid = uid;
    ref.node_id = node_id;
    unique.add(ref);
  }
  const Vector<GeometryNodesOutputCache *> caches = snapshot_registered_caches();
  for (GeometryNodesOutputCache *cache : caches) {
    for (const GeometryNodesOutputCache::NodeRef &ref : unique) {
      cache->discard_node(ref.tree_session_uid, ref.node_id);
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Evaluation State
 * \{ */

/** What a node's result can depend on besides its own tree. */
enum : uint8_t {
  /** Inputs of the compute context: modifier inputs, group inputs, closure inputs. */
  DEP_CONTEXT = 1 << 0,
  DEP_IDS = 1 << 1,
  DEP_TIME = 1 << 2,
  DEP_CAMERA = 1 << 3,
  DEP_ANIMATION = 1 << 4,
  DEP_ALL = (1 << 5) - 1,
};

struct GeometryNodesOutputCacheEvalState {
  uint32_t root_tree_uid = 0;
  Map<uint32_t, uint64_t> topology_gen_by_tree;
  Map<uint32_t, Map<int32_t, uint64_t>> node_gen_by_tree;
  /** `DEP_*` flags of every node of every tree in the hierarchy. */
  Map<uint32_t, Map<int32_t, uint8_t>> dep_flags_by_tree;
  /** Nodes whose original (or eval) flag has NODE_OUTPUT_CACHE. */
  Set<GeometryNodesOutputCache::NodeRef> enabled_nodes;
  /** Root tree: identity of every modifier input, by interface index. */
  Vector<uint64_t> root_input_tokens;
  /** Root tree: combined identity of the modifier inputs each node is downstream of. */
  Map<int32_t, uint64_t> root_context_token;
  uint64_t all_root_inputs_token = 0;
  uint64_t ids_token = 0;
  uint64_t time_token = 0;
  uint64_t camera_token = 0;
  uint64_t animation_token = 0;
};

/** Group node's `id` is a nested node tree — never a mesh/object/image. File load and
 * custom groups can leave a non-null pointer of the wrong ID type; casting that to
 * `bNodeTree` and walking `.runtime` / `group_nodes()` AVs (River Demo.crash.txt). */
static const bNodeTree *nested_group_tree(const bNode &node)
{
  if (!node.is_group()) {
    return nullptr;
  }
  const ID *id = node.id;
  if (id == nullptr || GS(id->name) != ID_NT || ID_MISSING(id)) {
    return nullptr;
  }
  const bNodeTree *group = reinterpret_cast<const bNodeTree *>(id);
  if (group->runtime == nullptr) {
    return nullptr;
  }
  return group;
}

static void snapshot_tree_gens(const bNodeTree &tree,
                               GeometryNodesOutputCacheEvalState &state,
                               Set<uint32_t> &visited_trees)
{
  if (tree.runtime == nullptr) {
    return;
  }
  const uint32_t uid = original_tree_session_uid(tree);
  if (!visited_trees.add(uid)) {
    return;
  }
  {
    GeometryNodesOutputCacheTreeState &tree_state = tree_state_for(uid);
    std::lock_guard lock{tree_state.mutex};
    state.topology_gen_by_tree.add_overwrite(uid, tree_state.topology_gen);
    state.node_gen_by_tree.add_overwrite(uid, tree_state.node_gen);
  }
  /* Walk DNA ListBase, not `group_nodes()`: the topology cache can hold dangling node
   * pointers if it was built before a COW/rebuild. */
  for (const bNode &node : tree.nodes) {
    if (const bNodeTree *group = nested_group_tree(node)) {
      snapshot_tree_gens(*group, state, visited_trees);
    }
  }
}

static bool socket_holds_data_block(const bNodeSocket &socket)
{
  /* Materials are left out: nodes pass them along as handles and never read them. */
  return ELEM(socket.type,
              SOCK_OBJECT,
              SOCK_COLLECTION,
              SOCK_IMAGE,
              SOCK_TEXTURE,
              SOCK_FONT,
              SOCK_SCENE,
              SOCK_TEXT_ID,
              SOCK_MASK,
              SOCK_SOUND);
}

/** True for nodes that read a data-block or the evaluation mode. */
static bool is_ids_seed(const bNode &node)
{
  if (ELEM(node.type_legacy, GEO_NODE_IS_VIEWPORT, GEO_NODE_SELF_OBJECT)) {
    return true;
  }
  /* These only pass data-block handles along. A node behind them that reads the handle is a
   * seed itself; group nodes take the dependencies of their tree instead. */
  if (node.is_group() || node.is_reroute() || node.is_group_input() || node.is_group_output() ||
      ELEM(node.type_legacy, GEO_NODE_SWITCH, GEO_NODE_INDEX_SWITCH, GEO_NODE_MENU_SWITCH) ||
      bke::zone_type_by_node_type(node.type_legacy) != nullptr)
  {
    return false;
  }
  if (node.id != nullptr && GS(node.id->name) != ID_NT) {
    return true;
  }
  for (const bNodeSocket *socket : node.input_sockets()) {
    if (socket->is_available() && socket_holds_data_block(*socket)) {
      return true;
    }
  }
  return false;
}

static bool is_time_seed(const bNode &node)
{
  if (ELEM(node.type_legacy,
           GEO_NODE_INPUT_SCENE_TIME,
           GEO_NODE_TIME_SHIFT,
           GEO_NODE_SIMULATION_INPUT,
           GEO_NODE_SIMULATION_OUTPUT,
           GEO_NODE_BAKE))
  {
    return true;
  }
  /* Solvers advance their own state with the scene frame. */
  if (node.is_type("GeometryNodeJoltSolver"_ustr) || node.is_type("GeometryNodeFlipSolver"_ustr) ||
      node.is_type("GeometryNodeXPBDSolver"_ustr) ||
      node.is_type("GeometryNodeBoxEngineSolver"_ustr))
  {
    return true;
  }
  for (const bNodeSocket *socket : node.input_sockets()) {
    if ((socket->flag & SOCK_IS_LINKED) != 0 || socket->runtime == nullptr ||
        socket->runtime->declaration == nullptr)
    {
      continue;
    }
    if (socket->runtime->declaration->default_input_type == NODE_DEFAULT_INPUT_SCENE_FRAME) {
      return true;
    }
  }
  return false;
}

static bool is_camera_seed(const bNode &node)
{
  return ELEM(node.type_legacy, GEO_NODE_VIEWPORT_CAMERA, GEO_NODE_INPUT_ACTIVE_CAMERA);
}

static uint64_t hash_rna_value(PointerRNA &id_ptr, const char *path, const int array_index)
{
  PointerRNA ptr;
  PropertyRNA *prop = nullptr;
  if (!RNA_path_resolve_property(&id_ptr, path, &ptr, &prop) || prop == nullptr) {
    return 0;
  }
  const bool is_array = RNA_property_array_check(prop);
  if (is_array && (array_index < 0 || array_index >= RNA_property_array_length(&ptr, prop))) {
    return 0;
  }
  switch (RNA_property_type(prop)) {
    case PROP_FLOAT: {
      const float value = is_array ? RNA_property_float_get_index(&ptr, prop, array_index) :
                                     RNA_property_float_get(&ptr, prop);
      return hash_bytes(&value, sizeof(value));
    }
    case PROP_INT: {
      const int value = is_array ? RNA_property_int_get_index(&ptr, prop, array_index) :
                                   RNA_property_int_get(&ptr, prop);
      return mix(1, uint64_t(uint32_t(value)));
    }
    case PROP_BOOLEAN: {
      const bool value = is_array ? RNA_property_boolean_get_index(&ptr, prop, array_index) :
                                    RNA_property_boolean_get(&ptr, prop);
      return mix(2, uint64_t(value));
    }
    case PROP_ENUM:
      return mix(3, uint64_t(uint32_t(RNA_property_enum_get(&ptr, prop))));
    default:
      return 0;
  }
}

/**
 * Animation and drivers write into the evaluated tree without a tree update, so no generation
 * changes. Find the nodes they target and hash the values they currently have.
 */
static uint64_t collect_animated_nodes(const bNodeTree &tree, Set<int32_t> &r_animated_nodes)
{
  if (!BKE_animdata_id_is_animated(&tree.id)) {
    return 0;
  }
  Map<StringRef, const bNode *> node_by_name;
  for (const bNode &node : tree.nodes) {
    node_by_name.add(node.name, &node);
  }
  ID *id = const_cast<ID *>(&tree.id);
  PointerRNA id_ptr = RNA_id_pointer_create(id);
  uint64_t token = 1;
  BKE_fcurves_id_cb(id, [&](ID * /*id*/, FCurve *fcurve) {
    if (fcurve->rna_path_ptr == nullptr) {
      return;
    }
    const char *rna_path = fcurve->rna_path_ptr;
    char node_name[256];
    if (!BLI_str_quoted_substr(rna_path, "nodes[", node_name, sizeof(node_name))) {
      return;
    }
    const bNode *node = node_by_name.lookup_default(node_name, nullptr);
    if (node == nullptr) {
      return;
    }
    r_animated_nodes.add(node->identifier);
    token = mix(token, uint64_t(node->identifier));
    token = mix(token, hash_text(rna_path));
    token = mix(token, uint64_t(fcurve->array_index));
    token = mix(token, hash_rna_value(id_ptr, rna_path, fcurve->array_index));
  });
  return token;
}

/** Works out for every node of a tree hierarchy which external state it is downstream of. */
struct TreeClassifier {
  GeometryNodesOutputCacheEvalState &state;
  /** Flags that reach a Group Output node, per tree. */
  Map<uint32_t, uint8_t> output_flags_by_tree;
  Set<uint32_t> visiting;
  uint8_t seen_flags = 0;

  uint8_t classify(const bNodeTree &tree)
  {
    if (tree.runtime == nullptr) {
      return DEP_ALL & ~DEP_CONTEXT;
    }
    const uint32_t uid = original_tree_session_uid(tree);
    if (const uint8_t *known = output_flags_by_tree.lookup_ptr(uid)) {
      return *known;
    }
    if (!visiting.add(uid)) {
      /* The group is already on this path. Its output can depend on anything. */
      return DEP_ALL & ~DEP_CONTEXT;
    }
    tree.ensure_topology_cache();

    Set<int32_t> animated_nodes;
    if (const uint64_t animation = collect_animated_nodes(tree, animated_nodes)) {
      state.animation_token += mix(uid, animation);
    }

    Vector<std::pair<const bNode *, uint8_t>> seeds;
    for (const bNode *node : tree.all_nodes()) {
      if (node == nullptr) {
        continue;
      }
      uint8_t flags = 0;
      if (animated_nodes.contains(node->identifier)) {
        flags |= DEP_ANIMATION;
      }
      if (!node->is_muted()) {
        if (const bNodeTree *group = nested_group_tree(*node)) {
          flags |= this->classify(*group);
        }
        if (node->is_group_input() || node->type_legacy == NODE_CLOSURE_INPUT) {
          flags |= DEP_CONTEXT;
        }
        if (is_ids_seed(*node)) {
          flags |= DEP_IDS;
        }
        if (is_time_seed(*node)) {
          flags |= DEP_TIME;
        }
        if (is_camera_seed(*node)) {
          flags |= DEP_CAMERA;
        }
      }
      if (flags != 0) {
        seeds.append({node, flags});
      }
    }

    Map<int32_t, uint8_t> node_flags;
    for (const bNode *node : tree.all_nodes()) {
      if (node != nullptr) {
        node_flags.add(node->identifier, 0);
      }
    }
    uint8_t output_flags = 0;
    for (const uint8_t flag : {DEP_CONTEXT, DEP_IDS, DEP_TIME, DEP_CAMERA, DEP_ANIMATION}) {
      Vector<const bNode *> starts;
      for (const auto &[node, flags] : seeds) {
        if (flags & flag) {
          starts.append(node);
        }
      }
      if (starts.is_empty()) {
        continue;
      }
      seen_flags |= flag;
      foreach_downstream(tree, starts, [&](const bNode &node) {
        node_flags.lookup_or_add(node.identifier, 0) |= flag;
        if (node.is_group_output()) {
          output_flags |= flag;
        }
      });
    }
    state.dep_flags_by_tree.add_overwrite(uid, std::move(node_flags));

    visiting.remove(uid);
    /* What a group node gets from its own inputs is decided by the links of the calling tree. */
    output_flags &= ~DEP_CONTEXT;
    output_flags_by_tree.add_overwrite(uid, output_flags);
    return output_flags;
  }
};

static uint64_t object_token(const Object &object,
                             const EvalDependencies::ObjectDependencyInfo &info,
                             const Object *self_object,
                             GeometryNodesOutputCache::ContentHasher &hasher,
                             Set<const Collection *> &visited_collections);

static uint64_t collection_token(const Collection &collection,
                                 const Object *self_object,
                                 GeometryNodesOutputCache::ContentHasher &hasher,
                                 Set<const Collection *> &visited_collections)
{
  uint64_t token = mix(0x434F4C4Cull, stable_session_uid(&collection.id));
  if (!visited_collections.add(&collection)) {
    return token;
  }
  token = mix(token, hash_bytes(collection.instance_offset, sizeof(collection.instance_offset)));
  for (const CollectionObject &collection_object : collection.gobject) {
    if (collection_object.ob != nullptr) {
      token = mix(token,
                  object_token(*collection_object.ob,
                               EvalDependencies::all_object_deps,
                               self_object,
                               hasher,
                               visited_collections));
    }
  }
  for (const CollectionChild &child : collection.children) {
    if (child.collection != nullptr) {
      token = mix(token,
                  collection_token(*child.collection, self_object, hasher, visited_collections));
    }
  }
  return token;
}

static uint64_t object_token(const Object &object,
                             const EvalDependencies::ObjectDependencyInfo &info,
                             const Object *self_object,
                             GeometryNodesOutputCache::ContentHasher &hasher,
                             Set<const Collection *> &visited_collections)
{
  uint64_t token = mix(0x4F424A45ull, stable_session_uid(&object.id));
  token = mix(token, uint64_t(object.type));
  if (info.transform) {
    const float4x4 &transform = object.object_to_world();
    token = mix(token, hash_bytes(&transform, sizeof(float4x4)));
  }
  /* The modifier's own geometry is what is being evaluated right now. */
  const bool is_self = self_object != nullptr &&
                       DEG_get_original(&object.id) == DEG_get_original(&self_object->id);
  if (info.geometry && !is_self) {
    if (object.type == OB_EMPTY && object.instance_collection != nullptr) {
      token = mix(token,
                  collection_token(
                      *object.instance_collection, self_object, hasher, visited_collections));
    }
    else if (DEG_object_has_geometry_component(const_cast<Object *>(&object))) {
      token = mix(token, geometry_token(bke::object_get_evaluated_geometry_set(object), hasher));
    }
  }
  if (info.camera_parameters && object.type == OB_CAMERA && object.data != nullptr) {
    const Camera &camera = *reinterpret_cast<const Camera *>(object.data);
    for (const float value : {camera.lens,
                              camera.ortho_scale,
                              camera.clip_start,
                              camera.clip_end,
                              camera.sensor_x,
                              camera.sensor_y,
                              camera.shiftx,
                              camera.shifty})
    {
      token = mix(token, hash_bytes(&value, sizeof(value)));
    }
    token = mix(token, uint64_t(camera.type));
    token = mix(token, uint64_t(camera.sensor_fit));
  }
  if (info.pose && object.type == OB_ARMATURE && object.pose != nullptr) {
    for (const bPoseChannel &channel : object.pose->chanbase) {
      token = mix(token, hash_bytes(channel.pose_mat, sizeof(channel.pose_mat)));
    }
  }
  return token;
}

/** Hash the state of every data-block the modifier depends on, as evaluated in this depsgraph. */
static uint64_t compute_ids_token(const bNodeTree &root_tree,
                                  const GeometryNodesOutputCache::PrepareParams &params,
                                  GeometryNodesOutputCache::ContentHasher &hasher)
{
  /* Use the list the original tree keeps. Collecting it from the evaluated tree does not work:
   * dependencies are keyed by session uid, which evaluated data-blocks do not have. */
  EvalDependencies deps;
  const bNodeTree *tree_orig = reinterpret_cast<const bNodeTree *>(DEG_get_original(&root_tree.id));
  if (tree_orig != nullptr && tree_orig->runtime != nullptr &&
      tree_orig->runtime->eval_dependencies != nullptr)
  {
    deps.merge(*tree_orig->runtime->eval_dependencies);
  }
  if (params.properties != nullptr) {
    IDP_foreach_property(const_cast<IDProperty *>(params.properties),
                         IDP_TYPE_FILTER_ID,
                         [&](IDProperty *property) {
                           if (ID *id = IDP_ID_get(property)) {
                             deps.add_generic_id_full(const_cast<ID *>(DEG_get_original(id)));
                           }
                         });
  }

  /* Map iteration order is not stable across rebuilds, so sort by session uid. */
  Vector<std::pair<uint32_t, uint64_t>> parts;
  parts.reserve(deps.ids.size());
  Set<const Collection *> visited_collections;
  for (ID *id : deps.ids.values()) {
    if (id == nullptr) {
      continue;
    }
    uint64_t part = mix(uint64_t(GS(id->name)), id->session_uid);
    switch (GS(id->name)) {
      case ID_OB: {
        const Object *object = reinterpret_cast<const Object *>(id);
        if (params.depsgraph != nullptr) {
          object = DEG_get_evaluated(params.depsgraph, object);
        }
        visited_collections.clear();
        part = mix(part,
                   object_token(*object,
                                deps.objects_info.lookup_default(id->session_uid, {}),
                                params.self_object,
                                hasher,
                                visited_collections));
        break;
      }
      case ID_GR: {
        const Collection *collection = reinterpret_cast<const Collection *>(id);
        if (params.depsgraph != nullptr) {
          collection = DEG_get_evaluated(params.depsgraph, collection);
        }
        visited_collections.clear();
        part = mix(
            part,
            collection_token(*collection, params.self_object, hasher, visited_collections));
        break;
      }
      default:
        break;
    }
    parts.append({id->session_uid, part});
  }
  std::sort(parts.begin(), parts.end());

  uint64_t token = 0x49445321ull;
  for (const auto &[uid, part] : parts) {
    token = mix(token, part);
  }
  if (params.depsgraph != nullptr) {
    token = mix(token, uint64_t(DEG_get_mode(params.depsgraph)));
  }
  /* Relative transforms and the Self Object node read the modifier object's transform. */
  if (deps.needs_own_transform && params.self_object != nullptr) {
    const float4x4 &transform = params.self_object->object_to_world();
    token = mix(token, hash_bytes(&transform, sizeof(float4x4)));
  }
  return token;
}

static uint64_t compute_camera_token(const GeometryNodesOutputCache::PrepareParams &params,
                                     GeometryNodesOutputCache::ContentHasher &hasher)
{
  uint64_t token = 0x43414D21ull;
  if (const std::optional<GeoNodesViewportCameraData> viewport = get_last_viewport_camera()) {
    token = mix(token, hash_bytes(&viewport->object_to_world, sizeof(float4x4)));
    token = mix(token, hash_bytes(&viewport->lens, sizeof(viewport->lens)));
    token = mix(token, uint64_t(viewport->is_perspective));
  }
  if (params.depsgraph != nullptr) {
    if (const Scene *scene = DEG_get_evaluated_scene(params.depsgraph)) {
      if (scene->camera != nullptr) {
        EvalDependencies::ObjectDependencyInfo info;
        info.transform = true;
        info.camera_parameters = true;
        Set<const Collection *> visited_collections;
        token = mix(
            token,
            object_token(*scene->camera, info, params.self_object, hasher, visited_collections));
      }
    }
  }
  return token;
}

/**
 * One token per interface input of the root tree: the content of the modifier's geometry for
 * geometry inputs, the stored panel value for the others.
 */
static Vector<uint64_t> compute_root_input_tokens(
    const bNodeTree &root_tree,
    const GeometryNodesOutputCache::PrepareParams &params,
    GeometryNodesOutputCache::ContentHasher &hasher)
{
  root_tree.ensure_interface_cache();
  const Span<const bNodeTreeInterfaceSocket *> inputs = root_tree.interface_inputs();
  Vector<uint64_t> tokens(inputs.size(), 0);

  const IDProperty *properties = params.properties;
  const IDProperty *inputs_group = nullptr;
  if (properties != nullptr && properties->type == IDP_GROUP) {
    inputs_group = IDP_GetPropertyFromGroup_null(properties, "inputs");
    if (inputs_group != nullptr && inputs_group->type != IDP_GROUP) {
      inputs_group = nullptr;
    }
  }
  /* Used when an input's value cannot be found by name: any panel change counts then. */
  const uint64_t all_properties_token = properties ? idproperty_token(*properties) : 0;

  bool used_input_geometry = false;
  for (const int i : inputs.index_range()) {
    const bNodeTreeInterfaceSocket &input = *inputs[i];
    if (STREQ(input.socket_type, "NodeSocketGeometry")) {
      /* Only the first geometry input receives the modifier's geometry. */
      if (!used_input_geometry && params.input_geometry != nullptr) {
        tokens[i] = geometry_token(*params.input_geometry, hasher);
        used_input_geometry = true;
      }
      continue;
    }
    const IDProperty *value = nullptr;
    if (inputs_group != nullptr && input.identifier != nullptr) {
      value = IDP_GetPropertyFromGroup_null(inputs_group, input.identifier);
    }
    tokens[i] = value ? idproperty_token(*value) : all_properties_token;
  }
  return tokens;
}

static void compute_root_context_tokens(const bNodeTree &root_tree,
                                        const Span<uint64_t> input_tokens,
                                        GeometryNodesOutputCacheEvalState &state)
{
  root_tree.ensure_topology_cache();
  for (const int i : input_tokens.index_range()) {
    const uint64_t part = mix(uint64_t(i) + 1, input_tokens[i]);
    state.all_root_inputs_token += part;
    Vector<const bNode *> starts;
    for (const bNode *group_input : root_tree.group_input_nodes()) {
      if (i >= group_input->output_sockets().size()) {
        continue;
      }
      for (const bNodeLink *link : group_input->output_socket(i).directly_linked_links()) {
        if (!link->is_muted() && link->is_available() && link->tonode != nullptr) {
          starts.append(link->tonode);
        }
      }
    }
    if (starts.is_empty()) {
      continue;
    }
    foreach_downstream(root_tree, starts, [&](const bNode &node) {
      state.root_context_token.lookup_or_add(node.identifier, 0) += part;
    });
  }
}

std::shared_ptr<const GeometryNodesOutputCacheEvalState> GeometryNodesOutputCache::prepare_for_eval(
    const PrepareParams &params)
{
  const bNodeTree &root_tree = *params.root_tree;
  auto state = std::make_shared<GeometryNodesOutputCacheEvalState>();
  state->root_tree_uid = original_tree_session_uid(root_tree);

  Set<NodeRef> live;
  {
    Stack<const bNodeTree *> trees;
    trees.push(&root_tree);
    Set<uint32_t> visited;
    while (!trees.is_empty()) {
      const bNodeTree &tree = *trees.pop();
      const uint32_t uid = original_tree_session_uid(tree);
      if (!visited.add(uid)) {
        continue;
      }
      if (tree.runtime == nullptr) {
        continue;
      }
      tree.ensure_topology_cache();
      for (const bNode *node : tree.all_nodes()) {
        if (node == nullptr) {
          continue;
        }
        NodeRef ref;
        ref.tree_session_uid = uid;
        ref.node_id = node->identifier;
        live.add(ref);
        if ((node->flag & NODE_OUTPUT_CACHE) != 0) {
          state->enabled_nodes.add(ref);
        }
        if (const bNodeTree *group = nested_group_tree(*node)) {
          trees.push(group);
        }
      }
      /* Eval/COW copies often lag the original DNA flag by one depsgraph cycle. Read the
       * original ListBase — no topology rebuild. Python/UI toggle writes the original. */
      if (const ID *orig_id = DEG_get_original(&tree.id)) {
        const bNodeTree &orig_tree = *reinterpret_cast<const bNodeTree *>(orig_id);
        for (const bNode &orig_node : orig_tree.nodes) {
          if ((orig_node.flag & NODE_OUTPUT_CACHE) != 0) {
            NodeRef ref;
            ref.tree_session_uid = uid;
            ref.node_id = orig_node.identifier;
            state->enabled_nodes.add(ref);
          }
        }
      }
    }
  }

  if (state->enabled_nodes.is_empty()) {
    /* Nothing is cached in this tree: keep the evaluation free of any cache work. */
    std::lock_guard lock{mutex_};
    entries_.clear();
    frozen_time_ns_.clear();
    frozen_time_state_.clear();
    return nullptr;
  }

  /* Snapshot tree state and walk the graph WITHOUT holding mutex_. Holding the cache mutex while
   * taking tree_state locks deadlocked against the node-editor cache toggle. */
  {
    Set<uint32_t> visited_trees;
    snapshot_tree_gens(root_tree, *state, visited_trees);
  }

  TreeClassifier classifier{*state};
  classifier.classify(root_tree);

  {
    std::lock_guard lock{prepare_mutex_};
    if (!content_hasher_) {
      content_hasher_ = std::make_unique<ContentHasher>();
    }
    state->root_input_tokens = compute_root_input_tokens(root_tree, params, *content_hasher_);
    compute_root_context_tokens(root_tree, state->root_input_tokens, *state);
    if (classifier.seen_flags & DEP_IDS) {
      state->ids_token = compute_ids_token(root_tree, params, *content_hasher_);
    }
    if (classifier.seen_flags & DEP_CAMERA) {
      state->camera_token = compute_camera_token(params, *content_hasher_);
    }
    content_hasher_->remove_expired();
  }
  state->time_token = mix(0x54494D45ull, hash_bytes(&params.ctime, sizeof(params.ctime)));

  {
    std::lock_guard lock{mutex_};
    entries_.remove_if([&](const auto &item) {
      NodeRef ref;
      ref.tree_session_uid = item.key.tree_session_uid;
      ref.node_id = item.key.node_id;
      return !live.contains(ref) || !state->enabled_nodes.contains(ref);
    });
    frozen_time_ns_.remove_if([&](const auto &item) { return !live.contains(item.key); });
    frozen_time_state_.remove_if([&](const auto &item) { return !live.contains(item.key); });
  }
  return state;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Lookup and Store
 * \{ */

static const char *state_difference(const GeometryNodesOutputCache::StateToken &stored,
                                    const GeometryNodesOutputCache::StateToken &current)
{
  if (stored.tree != current.tree) {
    return "(node or upstream edited)";
  }
  if (stored.context != current.context) {
    return "(input geometry or values changed)";
  }
  if (stored.ids != current.ids) {
    return "(referenced data-block changed)";
  }
  if (stored.time != current.time) {
    return "(frame changed)";
  }
  if (stored.camera != current.camera) {
    return "(camera changed)";
  }
  if (stored.animation != current.animation) {
    return "(animated value changed)";
  }
  return nullptr;
}

/** True when everything asked for now was also asked for when the entry was cooked. */
static bool requirements_are_covered(const GeometryNodesOutputCache::Requirements &stored,
                                     const GeometryNodesOutputCache::Requirements &current)
{
  for (const auto &[index, used] : current.usages) {
    if (!used) {
      continue;
    }
    bool covered = false;
    for (const auto &[stored_index, stored_used] : stored.usages) {
      if (stored_index == index) {
        covered = stored_used;
        break;
      }
    }
    if (!covered) {
      return false;
    }
  }
  for (const auto &[index, names] : current.reference_sets) {
    if (!names || names->is_empty()) {
      continue;
    }
    const Set<std::string> *stored_names = nullptr;
    for (const auto &[stored_index, stored_set] : stored.reference_sets) {
      if (stored_index == index) {
        stored_names = stored_set.get();
        break;
      }
    }
    if (stored_names == nullptr) {
      return false;
    }
    for (const std::string &name : *names) {
      if (!stored_names->contains(name)) {
        return false;
      }
    }
  }
  return true;
}

GeometryNodesOutputCache::Lookup GeometryNodesOutputCache::try_restore(
    const Key &key,
    const StateToken &state,
    const Requirements &requirements,
    lf::Params &params,
    const bool restore_usage_outputs,
    int64_t &r_exec_time_ns,
    const char *&r_miss_reason) const
{
  Entry entry_copy;
  {
    std::lock_guard lock{mutex_};
    const Entry *entry = entries_.lookup_ptr(key);
    if (entry == nullptr) {
      r_miss_reason = "(nothing stored)";
      return Lookup::Miss;
    }
    if (entry->state != state) {
      r_miss_reason = state_difference(entry->state, state);
      return Lookup::Miss;
    }
    if (!requirements_are_covered(entry->requirements, requirements)) {
      r_miss_reason = "(a consumer needs an attribute that was not cooked)";
      return Lookup::Miss;
    }
    entry_copy = *entry;
  }

  Map<int, const SocketValueVariant *> value_by_output;
  for (const CachedSocket &socket : entry_copy.sockets) {
    value_by_output.add(socket.output_index, &socket.value);
  }

  const Span<lf::Output> lf_outputs = params.fn_.outputs();
  for (const int lf_index : lf_outputs.index_range()) {
    if (lf_outputs[lf_index].type != &CPPType::get<SocketValueVariant>()) {
      continue;
    }
    /* Unconnected outputs stay Maybe; nodes often never write them. Requiring them in the cache
     * made every eval a miss and recook (the flicker + stutter). */
    if (params.get_output_usage(lf_index) != lf::ValueUsage::Used) {
      continue;
    }
    if (params.output_was_set(lf_index)) {
      continue;
    }
    if (!value_by_output.contains(lf_index)) {
      r_miss_reason = "(a used output was not cooked)";
      return Lookup::Miss;
    }
  }

  for (const int lf_index : lf_outputs.index_range()) {
    /* Only fill Used sockets. Writing Maybe outputs made the executor forward/destruct
     * values it did not own, which aborted with MEM_freeN(nullptr) on the user's startup file. */
    if (params.get_output_usage(lf_index) != lf::ValueUsage::Used) {
      continue;
    }
    if (params.output_was_set(lf_index)) {
      continue;
    }
    if (lf_outputs[lf_index].type == &CPPType::get<SocketValueVariant>()) {
      const SocketValueVariant *const *cached = value_by_output.lookup_ptr(lf_index);
      if (cached == nullptr || *cached == nullptr) {
        continue;
      }
      if (params.get_output_data_ptr(lf_index) == nullptr) {
        continue;
      }
      params.set_output(lf_index, **cached);
    }
    else if (restore_usage_outputs && lf_outputs[lf_index].type == &CPPType::get<bool>()) {
      if (params.get_output_data_ptr(lf_index) == nullptr) {
        continue;
      }
      /* Zone input-usage: cache hit means those inputs are not pulled. */
      params.set_output(lf_index, false);
    }
  }

  r_exec_time_ns = entry_copy.exec_time_ns;
  return Lookup::Hit;
}

bool GeometryNodesOutputCache::store(const Key &key,
                                     const StateToken &state,
                                     Requirements requirements,
                                     Vector<CachedSocket> sockets,
                                     const int64_t exec_time_ns,
                                     int64_t &r_frozen_ns)
{
  std::lock_guard lock{mutex_};

  if (sockets.is_empty()) {
    entries_.remove(key);
    return false;
  }

  /* Keep the first complete-eval time per node until its tree state changes. Later passes of
   * the same cook only re-run the wrapper and would show a few milliseconds. */
  NodeRef ref;
  ref.tree_session_uid = key.tree_session_uid;
  ref.node_id = key.node_id;
  int64_t &frozen = frozen_time_ns_.lookup_or_add(ref, 0);
  uint64_t &frozen_state = frozen_time_state_.lookup_or_add(ref, state.tree);
  if (frozen_state != state.tree) {
    frozen_state = state.tree;
    frozen = exec_time_ns;
  }
  else if (exec_time_ns > frozen) {
    frozen = exec_time_ns;
  }
  r_frozen_ns = frozen;

  Entry entry;
  entry.state = state;
  entry.requirements = std::move(requirements);
  entry.sockets = std::move(sockets);
  entry.exec_time_ns = frozen;
  entries_.add_overwrite(key, std::move(entry));
  return true;
}

static const GeometryNodesOutputCacheEvalState *eval_state_from(const GeoNodesUserData &user_data)
{
  if (user_data.call_data == nullptr || user_data.call_data->output_cache == nullptr ||
      user_data.compute_context == nullptr)
  {
    return nullptr;
  }
  return user_data.call_data->output_cache_eval;
}

static GeometryNodesOutputCache::NodeRef site_node_ref(const OutputCacheSite &site)
{
  GeometryNodesOutputCache::NodeRef ref;
  ref.tree_session_uid = original_tree_session_uid(*site.tree);
  ref.node_id = site.node->identifier;
  return ref;
}

bool geometry_nodes_output_cache_site_enabled(const GeoNodesUserData &user_data,
                                              const OutputCacheSite &site)
{
  const GeometryNodesOutputCacheEvalState *state = eval_state_from(user_data);
  if (state == nullptr || site.tree == nullptr || site.node == nullptr) {
    return false;
  }
  return state->enabled_nodes.contains(site_node_ref(site));
}

static GeometryNodesOutputCache::StateToken site_state_token(
    const GeometryNodesOutputCacheEvalState &state,
    const GeoNodesUserData &user_data,
    const OutputCacheSite &site)
{
  const GeometryNodesOutputCache::NodeRef ref = site_node_ref(site);
  uint64_t node_gen = 0;
  if (const Map<int32_t, uint64_t> *gens = state.node_gen_by_tree.lookup_ptr(ref.tree_session_uid))
  {
    node_gen = gens->lookup_default(ref.node_id, 0);
  }
  /* A node the classification did not see can depend on anything. */
  uint8_t flags = DEP_ALL;
  if (const Map<int32_t, uint8_t> *tree_flags = state.dep_flags_by_tree.lookup_ptr(
          ref.tree_session_uid))
  {
    flags = tree_flags->lookup_default(ref.node_id, DEP_ALL);
  }

  GeometryNodesOutputCache::StateToken token;
  token.tree = mix(0x54524545ull, state.topology_gen_by_tree.lookup_default(ref.tree_session_uid, 0));
  token.tree = mix(token.tree, node_gen);
  token.tree = mix(token.tree,
                   geometry_nodes_output_cache_hash_node_inputs(site.sockets_node ?
                                                                    *site.sockets_node :
                                                                    *site.node));
  if (flags & DEP_CONTEXT) {
    const uint64_t caller_token = user_data.output_cache_caller_token;
    if (caller_token != 0) {
      token.context = caller_token;
    }
    else if (ref.tree_session_uid == state.root_tree_uid) {
      token.context = mix(
          1, state.root_context_token.lookup_default(ref.node_id, state.all_root_inputs_token));
    }
    else {
      token.context = mix(1, state.all_root_inputs_token);
    }
  }
  if (flags & DEP_IDS) {
    token.ids = state.ids_token;
  }
  if (flags & DEP_TIME) {
    token.time = state.time_token;
  }
  if (flags & DEP_CAMERA) {
    token.camera = state.camera_token;
  }
  if (flags & DEP_ANIMATION) {
    token.animation = mix(1, state.animation_token);
  }
  return token;
}

static uint64_t combine_state_token(const GeometryNodesOutputCache::StateToken &state)
{
  uint64_t token = 0x53544154ull;
  for (const uint64_t part :
       {state.tree, state.context, state.ids, state.time, state.camera, state.animation})
  {
    token = mix(token, part);
  }
  return token;
}

uint64_t geometry_nodes_output_cache_child_token(const GeoNodesUserData &user_data,
                                                 const OutputCacheSite &site)
{
  const GeometryNodesOutputCacheEvalState *state = eval_state_from(user_data);
  if (state == nullptr || site.tree == nullptr || site.node == nullptr) {
    return 0;
  }
  /* Describe what flows into the node: its own unlinked values and the state of every node
   * linked to it. The node's own generation is left out on purpose. It also changes when the
   * tree behind a group node is edited, and that must not invalidate nodes inside the group
   * that come before the edit. */
  const bNode &node = *site.node;
  bool complete = true;
  uint64_t token = mix(0x4348494Cull, uint64_t(node.identifier));
  token = mix(token, hash_node_inputs(node, &complete));
  const bool is_root_context = user_data.output_cache_caller_token == 0 &&
                               original_tree_session_uid(*site.tree) == state->root_tree_uid;
  for (const bNodeSocket *socket : node.input_sockets()) {
    if (!socket->is_available()) {
      continue;
    }
    for (const bNodeLink *link : socket->directly_linked_links()) {
      if (link->fromnode == nullptr || link->fromsock == nullptr) {
        continue;
      }
      token = mix(token, uint64_t(socket->index()));
      const int source_index = link->fromsock->index();
      if (is_root_context && link->fromnode->is_group_input() &&
          state->root_input_tokens.index_range().contains(source_index))
      {
        /* Only this modifier input, not all of them. */
        token = mix(token, mix(uint64_t(source_index) + 1, state->root_input_tokens[source_index]));
        continue;
      }
      const OutputCacheSite source{site.tree, link->fromnode};
      token = mix(token, combine_state_token(site_state_token(*state, user_data, source)));
    }
  }
  if (!complete) {
    token = mix(token, combine_state_token(site_state_token(*state, user_data, site)));
  }
  /* The root context is recognized by a zero token. */
  return token == 0 ? 1 : token;
}

/**
 * Read the inputs that say what consumers need from the node. They are cheap usage values, not
 * socket data, so asking for them does not evaluate anything upstream.
 */
static bool gather_requirements(lf::Params &params,
                                GeometryNodesOutputCache::Requirements &r_requirements)
{
  const Span<lf::Input> inputs = params.fn_.inputs();
  bool all_available = true;
  for (const int i : inputs.index_range()) {
    const CPPType *type = inputs[i].type;
    if (type == &CPPType::get<bool>()) {
      if (const void *value = params.try_get_input_data_ptr_or_request(i)) {
        r_requirements.usages.append({i, *static_cast<const bool *>(value)});
      }
      else {
        all_available = false;
      }
    }
    else if (type == &CPPType::get<bke::GeometryNodesReferenceSet>()) {
      if (const void *value = params.try_get_input_data_ptr_or_request(i)) {
        r_requirements.reference_sets.append(
            {i, static_cast<const bke::GeometryNodesReferenceSet *>(value)->names});
      }
      else {
        all_available = false;
      }
    }
  }
  return all_available;
}

OutputCacheLookup geometry_nodes_output_cache_restore(const lf::Context &context,
                                                      lf::Params &params,
                                                      const OutputCacheSite &site,
                                                      OutputCacheSession &session,
                                                      const bool body_runs_for_side_effects)
{
  using Status = OutputCacheSession::Status;
  const GeoNodesUserData &user_data = *static_cast<const GeoNodesUserData *>(context.user_data);

  if (session.status == Status::Disabled) {
    return OutputCacheLookup::Disabled;
  }
  if (session.status == Status::Miss) {
    /* The node is being evaluated. A later pass must not switch to stored values. */
    return OutputCacheLookup::Miss;
  }
  const GeometryNodesOutputCacheEvalState *state = eval_state_from(user_data);
  if (session.status == Status::Undecided) {
    if (state == nullptr || !geometry_nodes_output_cache_site_enabled(user_data, site)) {
      session.status = Status::Disabled;
      return OutputCacheLookup::Disabled;
    }
    GeometryNodesOutputCache::Requirements requirements;
    if (!gather_requirements(params, requirements)) {
      return OutputCacheLookup::Pending;
    }
    session.requirements = std::move(requirements);
    session.key.tree_session_uid = original_tree_session_uid(*site.tree);
    session.key.node_id = site.node->identifier;
    session.key.context_hash = user_data.compute_context->hash();
    session.state = site_state_token(*state, user_data, site);
  }

  int64_t exec_time_ns = 0;
  const char *miss_reason = nullptr;
  const GeometryNodesOutputCache::Lookup lookup = user_data.call_data->output_cache->try_restore(
      session.key,
      session.state,
      session.requirements,
      params,
      !body_runs_for_side_effects,
      exec_time_ns,
      miss_reason);
  if (lookup == GeometryNodesOutputCache::Lookup::Miss) {
    if (session.status == Status::Undecided) {
      debug_print("miss ", *site.node, miss_reason);
    }
    session.status = Status::Miss;
    return OutputCacheLookup::Miss;
  }
  if (session.status == Status::Undecided) {
    debug_print("hit  ", *site.node, body_runs_for_side_effects ? "(body runs for viewer)" : nullptr);
  }
  session.status = Status::Hit;
  if (!body_runs_for_side_effects) {
    geometry_nodes_output_cache_skip_unprovided_inputs(params, int(params.fn_.inputs().size()));
  }
  geometry_nodes_output_cache_log_hit(context, site.node->identifier, exec_time_ns);
  return OutputCacheLookup::Hit;
}

void geometry_nodes_output_cache_merge_captured(
    Vector<GeometryNodesOutputCache::CachedSocket> &accumulated,
    Vector<GeometryNodesOutputCache::CachedSocket> &&captured)
{
  for (GeometryNodesOutputCache::CachedSocket &socket : captured) {
    bool replaced = false;
    for (GeometryNodesOutputCache::CachedSocket &known : accumulated) {
      if (known.output_index == socket.output_index) {
        known.value = std::move(socket.value);
        replaced = true;
        break;
      }
    }
    if (!replaced) {
      accumulated.append(std::move(socket));
    }
  }
  captured.clear();
}

bool geometry_nodes_output_cache_store(const lf::Context &context,
                                       lf::Params &params,
                                       const OutputCacheSite &site,
                                       OutputCacheSession &session,
                                       Vector<GeometryNodesOutputCache::CachedSocket> &&captured,
                                       int64_t &exec_time_ns)
{
  const GeoNodesUserData &user_data = *static_cast<const GeoNodesUserData *>(context.user_data);
  if (session.status != OutputCacheSession::Status::Miss) {
    return false;
  }
  geometry_nodes_output_cache_merge_captured(session.captured, std::move(captured));
  /* Outputs can arrive over several passes. An entry with some of them missing would never
   * be usable, so wait for the pass that completes the set. */
  if (session.captured.is_empty() ||
      !geometry_nodes_output_cache_used_values_ready(params, int(params.fn_.outputs().size())))
  {
    return false;
  }
  int64_t frozen_ns = exec_time_ns;
  if (!user_data.call_data->output_cache->store(session.key,
                                                session.state,
                                                session.requirements,
                                                session.captured,
                                                exec_time_ns,
                                                frozen_ns))
  {
    return false;
  }
  debug_print("store", *site.node, nullptr);
  exec_time_ns = frozen_ns;
  geometry_nodes_output_cache_log_frozen_overlay(context, site.node->identifier, frozen_ns);
  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Timing Overlay
 * \{ */

void geometry_nodes_output_cache_log_hit(const lf::Context &context,
                                         const int32_t node_id,
                                         const int64_t exec_time_ns)
{
  auto &user_data = *static_cast<GeoNodesUserData *>(context.user_data);
  auto &local_user_data = *static_cast<GeoNodesLocalUserData *>(context.local_user_data);
  eval_log::NodeTreeLogger *logger = local_user_data.try_get_tree_logger(user_data);
  if (logger == nullptr) {
    return;
  }
  /* Lazy wrappers are invoked many times per eval. Overlay time is the SUM of samples, so a
   * second hit must not inject the frozen duration again (downstream edits were multiplying it).
   * Always record frozen_overlay so a live 50–60ms sample is replaced by the first complete cook.
   */
  const eval_log::TimePoint end = eval_log::Clock::now();
  const eval_log::TimePoint start = end -
                                    std::chrono::nanoseconds(std::max<int64_t>(exec_time_ns, 0));
  logger->frozen_overlay_times.append(*logger->allocator, {node_id, start, end});
  logger->cache_hit_node_ids.append(*logger->allocator, node_id);
  for (const eval_log::NodeTreeLogger::NodeExecutionTime &t : logger->node_execution_times) {
    if (t.node_id == node_id) {
      return;
    }
  }
  logger->node_execution_times.append(*logger->allocator, {node_id, start, end});
}

void geometry_nodes_output_cache_mark_frozen(const lf::Context &context, const int32_t node_id)
{
  auto &user_data = *static_cast<GeoNodesUserData *>(context.user_data);
  auto &local_user_data = *static_cast<GeoNodesLocalUserData *>(context.local_user_data);
  if (eval_log::NodeTreeLogger *logger = local_user_data.try_get_tree_logger(user_data)) {
    logger->cache_hit_node_ids.append(*logger->allocator, node_id);
  }
}

void geometry_nodes_output_cache_log_frozen_overlay(const lf::Context &context,
                                                    const int32_t node_id,
                                                    const int64_t exec_time_ns)
{
  auto &user_data = *static_cast<GeoNodesUserData *>(context.user_data);
  auto &local_user_data = *static_cast<GeoNodesLocalUserData *>(context.local_user_data);
  eval_log::NodeTreeLogger *logger = local_user_data.try_get_tree_logger(user_data);
  if (logger == nullptr) {
    return;
  }
  const eval_log::TimePoint end = eval_log::Clock::now();
  const eval_log::TimePoint start = end -
                                    std::chrono::nanoseconds(std::max<int64_t>(exec_time_ns, 0));
  logger->frozen_overlay_times.append(*logger->allocator, {node_id, start, end});
  logger->cache_hit_node_ids.append(*logger->allocator, node_id);
}

/** \} */

}  // namespace blender::nodes
