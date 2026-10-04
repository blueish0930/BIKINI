/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_geometry_nodes_output_cache.hh"

#include <algorithm>
#include <chrono>
#include <memory>
#include <variant>

#include "BLI_color_types.hh"
#include "BLI_function_ref.hh"
#include "BLI_generic_span.hh"
#include "BLI_hash.hh"
#include "BLI_listbase.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"
#include "BLI_vector_set.hh"

#include "NOD_geometry_nodes_lazy_function.hh"

#include "DNA_ID.h"
#include "DNA_color_types.h"
#include "DNA_colorband_types.h"
#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_pointcloud_types.h"

#include "BKE_anim_data.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_node_runtime.hh"
#include "BKE_object.hh"
#include "BKE_pointcloud.hh"

#include "DEG_depsgraph_query.hh"

#include "FN_lazy_function.hh"

#include "NOD_dependencies.hh"
#include "NOD_geometry_nodes_closure.hh"
#include "NOD_geometry_nodes_list.hh"

namespace blender::nodes {

using bke::SocketValueVariant;
namespace lf = fn::lazy_function;

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

bool geometry_nodes_output_cache_node_enabled(const bNode &node,
                                              const GeometryNodesOutputCache *cache)
{
  if ((node.flag & NODE_OUTPUT_CACHE) != 0) {
    return true;
  }
  if (cache == nullptr) {
    return false;
  }
  /* Eval/COW copies often lag the original DNA flag by one depsgraph cycle. Never
   * ensure_topology_cache on the original from a worker (that crashed For Each). */
  const bNodeTree *tree = (node.runtime != nullptr) ? node.runtime->owner_tree : nullptr;
  if (tree == nullptr) {
    return false;
  }
  return cache->is_node_enabled(original_tree_session_uid(*tree), node.identifier);
}

bool GeometryNodesOutputCache::is_node_enabled(const uint32_t tree_session_uid,
                                               const int32_t node_id) const
{
  NodeRef ref;
  ref.tree_session_uid = tree_session_uid;
  ref.node_id = node_id;
  std::lock_guard lock{mutex_};
  return enabled_nodes_.contains(ref);
}

void GeometryNodesOutputCache::discard_node(const uint32_t tree_session_uid, const int32_t node_id)
{
  std::lock_guard lock{mutex_};
  Vector<Key> stale;
  for (const auto &item : entries_.items()) {
    if (item.key.tree_session_uid == tree_session_uid && item.key.node_id == node_id) {
      stale.append(item.key);
    }
  }
  for (const Key &key : stale) {
    entries_.remove(key);
  }
  NodeRef ref;
  ref.tree_session_uid = tree_session_uid;
  ref.node_id = node_id;
  frozen_time_ns_.remove(ref);
  frozen_time_gen_.remove(ref);
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

Vector<GeometryNodesOutputCache::CachedSocket> geometry_nodes_output_cache_collect_outputs(
    lf::Params &params, const int outputs_num)
{
  Vector<GeometryNodesOutputCache::CachedSocket> sockets;
  const Span<lf::Output> outputs = params.fn_.outputs();
  for (int i = 0; i < outputs_num; i++) {
    if (!params.output_was_set(i)) {
      continue;
    }
    if (outputs[i].type != &CPPType::get<SocketValueVariant>()) {
      continue;
    }
    void *ptr = params.get_output_data_ptr(i);
    if (ptr == nullptr) {
      continue;
    }
    const auto &value = *static_cast<const SocketValueVariant *>(ptr);
    if (!socket_value_is_output_cacheable(value)) {
      continue;
    }
    GeometryNodesOutputCache::CachedSocket cached;
    cached.output_index = i;
    cached.value = value;
    cached.value.ensure_owns_direct_data();
    sockets.append(std::move(cached));
  }
  return sockets;
}

static void foreach_downstream(const bNodeTree &tree,
                               const bNode &start,
                               const FunctionRef<void(const bNode &)> visit);

uint32_t geometry_nodes_output_cache_tree_uid(const bNodeTree &tree)
{
  return original_tree_session_uid(tree);
}

uint64_t geometry_nodes_output_cache_hash_node_inputs(const bNode &node)
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
    if (socket->default_value == nullptr) {
      continue;
    }
    switch (eNodeSocketDatatype(socket->type)) {
      case SOCK_FLOAT: {
        const auto &value = *static_cast<const bNodeSocketValueFloat *>(socket->default_value);
        h = get_default_hash(h, value.value);
        break;
      }
      case SOCK_INT: {
        const auto &value = *static_cast<const bNodeSocketValueInt *>(socket->default_value);
        h = get_default_hash(h, value.value);
        break;
      }
      case SOCK_BOOLEAN: {
        const auto &value = *static_cast<const bNodeSocketValueBoolean *>(socket->default_value);
        h = get_default_hash(h, int(value.value));
        break;
      }
      case SOCK_VECTOR: {
        const auto &value = *static_cast<const bNodeSocketValueVector *>(socket->default_value);
        h = get_default_hash(h, value.value[0], value.value[1], value.value[2], value.value[3]);
        break;
      }
      case SOCK_RGBA: {
        const auto &value = *static_cast<const bNodeSocketValueRGBA *>(socket->default_value);
        h = get_default_hash(h, value.value[0], value.value[1], value.value[2], value.value[3]);
        break;
      }
      default:
        break;
    }
  }
  return h;
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

static void collect_downstream_bumps(const bNodeTree &tree,
                                     const bNode &start,
                                     Vector<std::pair<uint32_t, int32_t>> &r_nodes)
{
  const uint32_t uid = original_tree_session_uid(tree);
  /* Do not recursively wipe every node inside a group. Nested trees have their own gens, and
   * inner keys include caller_token so a group input disconnect already misses. Recursing here
   * made Viewer-on-parent (group tagged NODE_OUTPUT) recook Repeat and every inner node. */
  foreach_downstream(tree, start, [&](const bNode &node) {
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

static void foreach_downstream(const bNodeTree &tree,
                               const bNode &start,
                               const FunctionRef<void(const bNode &)> visit)
{
  tree.ensure_topology_cache();
  Stack<const bNode *> stack;
  Set<const bNode *> visited;
  stack.push(&start);
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

static uint64_t hash_float3_components(const float3 &v)
{
  return get_default_hash(v.x, v.y, v.z);
}

static uint64_t hash_sampled_positions(const Span<float3> positions)
{
  if (positions.is_empty()) {
    return 0;
  }
  const int n = positions.size();
  /* Hash components, never the float3 struct (padding would flicker the token). */
  return get_default_hash(hash_float3_components(positions[0]),
                          hash_float3_components(positions[n / 2]),
                          hash_float3_components(positions[n - 1]));
}

static uint64_t hash_transform_stable(const float4x4 &t)
{
  return get_default_hash(hash_float3_components(t.location()),
                          hash_float3_components(t.x_axis()),
                          hash_float3_components(t.y_axis()),
                          hash_float3_components(t.z_axis()));
}

static uint64_t hash_geometry_content(const bke::GeometrySet &geometry)
{
  /* Content only — never pointers. Depsgraph copy-for-eval allocates a new Mesh every cook, so a
   * pointer token made every node-property edit look like a new modifier input and recooked the
   * whole tree. */
  uint64_t token = geometry.is_empty() ? uint64_t(0x1) : uint64_t(0x2);

  if (const Mesh *mesh = geometry.get_mesh()) {
    token = get_default_hash(token,
                             mesh->verts_num,
                             mesh->edges_num,
                             mesh->faces_num,
                             hash_sampled_positions(mesh->vert_positions()));
  }
  if (const PointCloud *points = geometry.get_pointcloud()) {
    token = get_default_hash(
        token, points->totpoint, hash_sampled_positions(points->positions()));
  }
  if (const Curves *curves_id = geometry.get_curves()) {
    const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    token = get_default_hash(token,
                             curves.points_num(),
                             curves.curves_num(),
                             hash_sampled_positions(curves.positions()));
  }
  if (const bke::Instances *instances = geometry.get_instances()) {
    token = get_default_hash(token, instances->instances_num());
    const Span<float4x4> transforms = instances->transforms();
    if (!transforms.is_empty()) {
      const int n = transforms.size();
      const auto hash_transform = [&](const float4x4 &t) {
        token = get_default_hash(token, hash_transform_stable(t));
      };
      hash_transform(transforms[0]);
      hash_transform(transforms[n / 2]);
      hash_transform(transforms[n - 1]);
    }
  }
  return token;
}

uint64_t geometry_nodes_output_cache_hash_value(const SocketValueVariant &value)
{
  if (value.is_field() || value.is_volume_grid()) {
    return uint64_t(0xF1E1D);
  }
  if (value.is_list()) {
    const GListPtr list = value.copy_as<GListPtr>();
    if (!list) {
      return uint64_t(0x15);
    }
    uint64_t token = get_default_hash(uint64_t(0x15), list->size(), int(value.socket_type()));
    const std::variant<GSpan, GPointer> values = list->values();
    if (const GSpan *span = std::get_if<GSpan>(&values)) {
      if (!span->is_empty()) {
        const int n = span->size();
        const CPPType &type = span->type();
        if (type.is<SocketValueVariant>()) {
          const Span<SocketValueVariant> typed = span->typed<SocketValueVariant>();
          token = get_default_hash(token,
                                   geometry_nodes_output_cache_hash_value(typed[0]),
                                   geometry_nodes_output_cache_hash_value(typed[n / 2]),
                                   geometry_nodes_output_cache_hash_value(typed[n - 1]));
        }
        else if (type.is<float>()) {
          const Span<float> typed = span->typed<float>();
          token = get_default_hash(token, typed[0], typed[n / 2], typed[n - 1]);
        }
        else if (type.is<int>()) {
          const Span<int> typed = span->typed<int>();
          token = get_default_hash(token, typed[0], typed[n / 2], typed[n - 1]);
        }
      }
    }
    return token;
  }
  if (!value.is_single()) {
    return uint64_t(0x2);
  }
  switch (value.socket_type()) {
    case SOCK_FLOAT:
      return get_default_hash(value.copy_as<float>());
    case SOCK_INT:
      return get_default_hash(value.copy_as<int>());
    case SOCK_BOOLEAN:
      return get_default_hash(value.copy_as<bool>());
    case SOCK_VECTOR:
      return hash_float3_components(value.copy_as<float3>());
    case SOCK_RGBA: {
      const ColorGeometry4f c = value.copy_as<ColorGeometry4f>();
      return get_default_hash(c.r, c.g, c.b, c.a);
    }
    case SOCK_GEOMETRY:
      return hash_geometry_content(value.copy_as<bke::GeometrySet>());
    case SOCK_CLOSURE: {
      const ClosurePtr closure = value.copy_as<ClosurePtr>();
      if (!closure) {
        return uint64_t(SOCK_CLOSURE);
      }
      uint64_t token = uint64_t(SOCK_CLOSURE);
      if (const std::optional<ClosureSourceLocation> &loc = closure->source_location()) {
        token = get_default_hash(token, loc->closure_output_node_id, loc->compute_context_hash.v1,
                                 loc->compute_context_hash.v2);
      }
      for (const SocketValueVariant *captured : closure->captured_values()) {
        if (captured) {
          token = get_default_hash(token, geometry_nodes_output_cache_hash_value(*captured));
        }
      }
      return token;
    }
    case SOCK_CURVE: {
      const auto *curve_ptr = value.get_if<CurveMapping *>();
      const CurveMapping *curve = curve_ptr ? *curve_ptr : nullptr;
      if (curve == nullptr) {
        return uint64_t(SOCK_CURVE);
      }
      uint64_t token = get_default_hash(uint64_t(SOCK_CURVE), curve->cur, curve->flag);
      for (int channel = 0; channel < CM_TOT; channel++) {
        const CurveMap &map = curve->cm[channel];
        token = get_default_hash(token, map.totpoint);
        if (map.curve == nullptr) {
          continue;
        }
        for (int point = 0; point < map.totpoint; point++) {
          token = get_default_hash(token, map.curve[point].x, map.curve[point].y);
        }
      }
      return token;
    }
    case SOCK_COLOR_RAMP: {
      const auto *band_ptr = value.get_if<ColorBand *>();
      const ColorBand *band = band_ptr ? *band_ptr : nullptr;
      if (band == nullptr) {
        return uint64_t(SOCK_COLOR_RAMP);
      }
      uint64_t token = get_default_hash(
          uint64_t(SOCK_COLOR_RAMP), band->tot, band->ipotype, band->color_mode);
      const int stops = min_ii(int(band->tot), 32);
      for (int i = 0; i < stops; i++) {
        const CBData &stop = band->data[i];
        token = get_default_hash(token, stop.pos, stop.r, stop.g, stop.b, stop.a);
      }
      return token;
    }
    default:
      return uint64_t(value.socket_type());
  }
}

static uint64_t hash_object_eval(const Object &object_eval)
{
  uint64_t token = get_default_hash(object_eval.id.session_uid);
  token = get_default_hash(token, hash_transform_stable(object_eval.object_to_world()));
  if (const Mesh *mesh = BKE_object_get_evaluated_mesh(&object_eval)) {
    token = get_default_hash(token,
                             mesh->verts_num,
                             mesh->edges_num,
                             mesh->faces_num,
                             hash_sampled_positions(mesh->vert_positions()));
  }
  return token;
}

static uint64_t compute_objects_token(const bNodeTree &root_tree,
                                      const Object *self_object,
                                      Depsgraph *depsgraph)
{
  if (root_tree.runtime == nullptr || root_tree.runtime->eval_dependencies == nullptr) {
    return 0;
  }
  const EvalDependencies &deps = *root_tree.runtime->eval_dependencies;
  const uint32_t self_uid = self_object ? self_object->id.session_uid : 0;
  /* Map iteration order is not stable across rebuilds; mixing hashes in visit order made
   * object-dependent geometry miss every redraw while List (not object-dependent) still hit. */
  Vector<std::pair<uint32_t, uint64_t>> parts;
  parts.reserve(deps.ids.size());
  for (ID *id : deps.ids.values()) {
    if (id == nullptr) {
      continue;
    }
    if (self_object && id->session_uid == self_uid) {
      continue;
    }
    uint64_t part = get_default_hash(id->session_uid, GS(id->name));
    if (GS(id->name) == ID_OB && depsgraph != nullptr) {
      const Object *object_eval = DEG_get_evaluated(depsgraph, reinterpret_cast<Object *>(id));
      if (object_eval == nullptr) {
        object_eval = reinterpret_cast<Object *>(id);
      }
      part = get_default_hash(part, hash_object_eval(*object_eval));
    }
    parts.append({id->session_uid, part});
  }
  std::sort(parts.begin(), parts.end());
  uint64_t token = 0;
  for (const auto &[uid, part] : parts) {
    token = get_default_hash(token, uid, part);
  }
  return token;
}

static uint64_t compute_camera_token(const bNodeTree &root_tree)
{
  if (root_tree.runtime == nullptr || root_tree.runtime->eval_dependencies == nullptr) {
    return 0;
  }
  if (!root_tree.runtime->eval_dependencies->needs_viewport_camera) {
    return 0;
  }
  const std::optional<GeoNodesViewportCameraData> viewport = get_last_viewport_camera();
  if (!viewport) {
    return 0;
  }
  return get_default_hash(
      hash_transform_stable(viewport->object_to_world), viewport->lens, viewport->is_perspective);
}

static bool is_time_source_node(const bNode &node)
{
  return ELEM(node.type_legacy,
              GEO_NODE_INPUT_SCENE_TIME,
              GEO_NODE_TIME_SHIFT,
              GEO_NODE_SIMULATION_INPUT,
              GEO_NODE_SIMULATION_OUTPUT);
}

static bool is_id_source_node(const bNode &node)
{
  return ELEM(node.type_legacy,
              GEO_NODE_OBJECT_INFO,
              GEO_NODE_COLLECTION_INFO,
              GEO_NODE_IMAGE_TEXTURE,
              GEO_NODE_IMAGE);
}

static bool is_camera_source_node(const bNode &node)
{
  return ELEM(node.type_legacy, GEO_NODE_VIEWPORT_CAMERA, GEO_NODE_INPUT_ACTIVE_CAMERA);
}

static bool node_is_group_output(const bNode &node)
{
  return node.is_group_output();
}

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
                               Map<uint32_t, uint64_t> &topology_gen_by_tree,
                               Map<uint32_t, Map<int32_t, uint64_t>> &node_gen_by_tree,
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
    GeometryNodesOutputCacheTreeState &state = tree_state_for(uid);
    std::lock_guard lock{state.mutex};
    topology_gen_by_tree.add_overwrite(uid, state.topology_gen);
    node_gen_by_tree.add_overwrite(uid, state.node_gen);
  }
  /* Walk DNA ListBase, not `group_nodes()`: the topology cache can hold dangling node
   * pointers if it was built before a COW/rebuild. */
  for (const bNode &node : tree.nodes) {
    if (const bNodeTree *group = nested_group_tree(node)) {
      snapshot_tree_gens(*group, topology_gen_by_tree, node_gen_by_tree, visited_trees);
    }
  }
}

static bool collect_dependent_from_pred_impl(const bNodeTree &tree,
                                             const FunctionRef<bool(const bNode &)> is_seed,
                                             Set<GeometryNodesOutputCache::NodeRef> &r_nodes,
                                             Set<uint32_t> &visiting)
{
  if (tree.runtime == nullptr) {
    return false;
  }
  const uint32_t uid = original_tree_session_uid(tree);
  if (!visiting.add(uid)) {
    /* The group is already on this path. Treat its output as dependent so an output cache
     * cannot reuse a value after an input to a recursive call changes. */
    return true;
  }
  tree.ensure_topology_cache();
  bool output_affected = false;

  Vector<const bNode *> seeds;
  for (const bNode *node : tree.all_nodes()) {
    if (node == nullptr || node->is_muted()) {
      continue;
    }
    if (const bNodeTree *group = nested_group_tree(*node)) {
      const bool nested_output = collect_dependent_from_pred_impl(*group, is_seed, r_nodes, visiting);
      if (nested_output) {
        seeds.append(node);
      }
    }
    if (is_seed(*node)) {
      seeds.append(node);
    }
  }

  for (const bNode *seed : seeds) {
    foreach_downstream(tree, *seed, [&](const bNode &node) {
      GeometryNodesOutputCache::NodeRef ref;
      ref.tree_session_uid = uid;
      ref.node_id = node.identifier;
      r_nodes.add(ref);
      if (node_is_group_output(node)) {
        output_affected = true;
      }
    });
  }
  visiting.remove(uid);
  return output_affected;
}

static bool collect_dependent_from_pred(const bNodeTree &tree,
                                        const FunctionRef<bool(const bNode &)> is_seed,
                                        Set<GeometryNodesOutputCache::NodeRef> &r_nodes)
{
  Set<uint32_t> visiting;
  return collect_dependent_from_pred_impl(tree, is_seed, r_nodes, visiting);
}

void GeometryNodesOutputCache::prepare_for_eval(const bNodeTree &root_tree,
                                                const bke::GeometrySet &input_geometry,
                                                const Object *self_object,
                                                Depsgraph *depsgraph,
                                                float ctime)
{
  /* Snapshot tree state and walk the graph WITHOUT holding mutex_. Holding the cache mutex while
   * taking tree_state locks deadlocked against the node-editor cache toggle. */
  Map<uint32_t, uint64_t> topology_gen_by_tree;
  Map<uint32_t, Map<int32_t, uint64_t>> node_gen_by_tree;
  Set<uint32_t> visited_trees;
  snapshot_tree_gens(root_tree, topology_gen_by_tree, node_gen_by_tree, visited_trees);

  Set<NodeRef> object_dependent;
  Set<NodeRef> time_dependent;
  Set<NodeRef> camera_dependent;
  collect_dependent_from_pred(root_tree, is_id_source_node, object_dependent);
  collect_dependent_from_pred(root_tree, is_time_source_node, time_dependent);
  collect_dependent_from_pred(root_tree, is_camera_source_node, camera_dependent);

  const uint64_t input_token = hash_geometry_content(input_geometry);
  const uint64_t objects_token = compute_objects_token(root_tree, self_object, depsgraph);
  const uint64_t camera_token = compute_camera_token(root_tree);

  Set<NodeRef> live;
  Set<NodeRef> enabled_nodes;
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
          enabled_nodes.add(ref);
        }
        if (const bNodeTree *group = nested_group_tree(*node)) {
          trees.push(group);
        }
      }
      /* Original ListBase — no topology rebuild. Python/UI toggle writes the original. */
      if (const ID *orig_id = DEG_get_original(&tree.id)) {
        const bNodeTree &orig_tree = *reinterpret_cast<const bNodeTree *>(orig_id);
        for (const bNode *orig_node = orig_tree.nodes.first();
             orig_node != nullptr;
             orig_node = orig_node->next)
        {
          if ((orig_node->flag & NODE_OUTPUT_CACHE) != 0) {
            NodeRef ref;
            ref.tree_session_uid = uid;
            ref.node_id = orig_node->identifier;
            enabled_nodes.add(ref);
          }
        }
      }
    }
  }

  std::lock_guard lock{mutex_};

  topology_gen_by_tree_ = std::move(topology_gen_by_tree);
  node_gen_by_tree_ = std::move(node_gen_by_tree);
  object_dependent_ = std::move(object_dependent);
  time_dependent_ = std::move(time_dependent);
  camera_dependent_ = std::move(camera_dependent);
  enabled_nodes_ = std::move(enabled_nodes);

  /* Never wipe the whole table: viewport + render (or orig + eval) share this cache and would
   * otherwise ping-pong-clear each other every redraw. Separate input tokens keep both worlds. */
  if (has_prev_input_token_ && input_token != current_input_token_) {
    prev_input_token_ = current_input_token_;
    Vector<Key> stale;
    for (const auto &item : entries_.items()) {
      if (item.key.input_token != input_token && item.key.input_token != prev_input_token_) {
        stale.append(item.key);
      }
    }
    for (const Key &key : stale) {
      entries_.remove(key);
    }
  }

  current_input_token_ = input_token;
  current_objects_token_ = objects_token;
  current_camera_token_ = camera_token;
  current_ctime_ = ctime;
  has_prev_input_token_ = true;

  Vector<Key> dead;
  for (const auto &item : entries_.items()) {
    NodeRef ref;
    ref.tree_session_uid = item.key.tree_session_uid;
    ref.node_id = item.key.node_id;
    if (!live.contains(ref)) {
      dead.append(item.key);
    }
  }
  for (const Key &key : dead) {
    NodeRef ref;
    ref.tree_session_uid = key.tree_session_uid;
    ref.node_id = key.node_id;
    entries_.remove(key);
    frozen_time_ns_.remove(ref);
    frozen_time_gen_.remove(ref);
  }
}

static bool node_ref_contains(const Set<GeometryNodesOutputCache::NodeRef> &nodes,
                              const uint32_t tree_uid,
                              const int32_t node_id)
{
  GeometryNodesOutputCache::NodeRef ref;
  ref.tree_session_uid = tree_uid;
  ref.node_id = node_id;
  return nodes.contains(ref);
}

GeometryNodesOutputCache::Hit GeometryNodesOutputCache::try_restore(const Key &key_in,
                                                                    lf::Params &params) const
{
  Hit result;
  Entry entry_copy;
  {
    std::lock_guard lock{mutex_};

    Key key = key_in;
    key.input_token = current_input_token_;

    const uint64_t node_gen = [&]() -> uint64_t {
      if (const Map<int32_t, uint64_t> *per_node = node_gen_by_tree_.lookup_ptr(
              key.tree_session_uid))
      {
        return per_node->lookup_default(key.node_id, 0);
      }
      return 0;
    }();

    const Entry *entry = entries_.lookup_ptr(key);
    if (entry == nullptr) {
      return result;
    }
    if (entry->node_gen != node_gen) {
      return result;
    }
    if (node_ref_contains(object_dependent_, key.tree_session_uid, key.node_id) &&
        entry->objects_token != current_objects_token_)
    {
      return result;
    }
    if (node_ref_contains(camera_dependent_, key.tree_session_uid, key.node_id) &&
        entry->camera_token != current_camera_token_)
    {
      return result;
    }
    if (node_ref_contains(time_dependent_, key.tree_session_uid, key.node_id) &&
        entry->ctime != current_ctime_)
    {
      return result;
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
      return result;
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
    if (params.get_output_data_ptr(lf_index) == nullptr) {
      continue;
    }
    if (lf_outputs[lf_index].type == &CPPType::get<SocketValueVariant>()) {
      const SocketValueVariant *const *cached = value_by_output.lookup_ptr(lf_index);
      if (cached == nullptr || *cached == nullptr) {
        continue;
      }
      params.set_output(lf_index, **cached);
    }
    else if (lf_outputs[lf_index].type == &CPPType::get<bool>()) {
      /* Zone input-usage: cache hit means those inputs are not pulled. */
      params.set_output(lf_index, false);
    }
  }

  result.hit = true;
  result.exec_time_ns = entry_copy.exec_time_ns;
  return result;
}

bool GeometryNodesOutputCache::store(const Key &key_in,
                                     Vector<CachedSocket> sockets,
                                     const int64_t exec_time_ns,
                                     int64_t *r_frozen_ns)
{
  std::lock_guard lock{mutex_};

  Key key = key_in;
  key.input_token = current_input_token_;

  /* One live entry per node in this input world. A sockets_token change (List defaults, etc.)
   * used to add a second key and leave the previous list resident. */
  {
    Vector<Key> stale;
    for (const auto &item : entries_.items()) {
      if (item.key.tree_session_uid == key.tree_session_uid && item.key.node_id == key.node_id &&
          item.key.input_token == key.input_token && item.key != key)
      {
        stale.append(item.key);
      }
    }
    for (const Key &stale_key : stale) {
      entries_.remove(stale_key);
    }
  }

  if (sockets.is_empty()) {
    entries_.remove(key);
    return false;
  }

  Entry entry;
  entry.node_gen = [&]() -> uint64_t {
    if (const Map<int32_t, uint64_t> *per_node = node_gen_by_tree_.lookup_ptr(key.tree_session_uid))
    {
      return per_node->lookup_default(key.node_id, 0);
    }
    return 0;
  }();
  entry.topology_gen = topology_gen_by_tree_.lookup_default(key.tree_session_uid, 0);
  entry.objects_token = current_objects_token_;
  entry.camera_token = current_camera_token_;
  entry.ctime = current_ctime_;
  entry.sockets = std::move(sockets);
  entry.exec_time_ns = exec_time_ns;
  /* Keep the first complete-eval time per node until THIS node's gen bumps. Do not fold
   * topology_gen in: viewer rewires used to reset frozen 900ms down to a 50–60ms re-run. */
  NodeRef ref;
  ref.tree_session_uid = key.tree_session_uid;
  ref.node_id = key.node_id;
  int64_t &frozen = frozen_time_ns_.lookup_or_add(ref, 0);
  uint64_t &frozen_gen = frozen_time_gen_.lookup_or_add(ref, entry.node_gen);
  if (frozen_gen != entry.node_gen) {
    frozen_gen = entry.node_gen;
    frozen = exec_time_ns;
  }
  else if (exec_time_ns > frozen) {
    frozen = exec_time_ns;
  }
  entry.exec_time_ns = frozen;
  if (r_frozen_ns != nullptr) {
    *r_frozen_ns = frozen;
  }
  entries_.add_overwrite(key, std::move(entry));
  return true;
}

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

}  // namespace blender::nodes
