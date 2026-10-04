/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <climits>
#include <cstring>
#include <memory>
#include <string>

#include "DNA_node_types.h"
#include "DNA_scene_types.h"

#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "BLI_index_range.hh"
#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

#include "GPU_state.hh"
#include "GPU_texture.hh"

#include "COM_context.hh"
#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_undefined_node_operation.hh"
#include "COM_utilities.hh"

/* Fluid Simulation zone multi-frame helpers (node_image_fluid_sim.cc). */
namespace blender::nodes::image_fluid_zone {
int resume_frame(const bNodeTree &ntree, int target_frame);
void invalidate_after(const bNodeTree &ntree, int keep_frame);
Vector<IndexRange> cached_frame_ranges();
Vector<IndexRange> checkpoint_frame_ranges();
}  // namespace blender::nodes::image_fluid_zone

namespace blender::compositor {

static int g_image_process_repeat_iteration = 0;
static bool g_image_process_bake_write_enabled = false;
static bool g_image_process_fluid_scrub_fill = false;
static uint g_image_process_paint_force_tree_session_uid = 0;
static int g_image_process_paint_force_node_identifier = 0;
static uint g_image_process_cook_tree_session_uid = 0;
/** Non-empty: only these terminal node identifiers are scheduled as cook roots. */
static Vector<int> g_image_process_terminal_filter;

void image_process_set_repeat_iteration(const int iteration)
{
  g_image_process_repeat_iteration = math::max(0, iteration);
}

int image_process_get_repeat_iteration()
{
  return g_image_process_repeat_iteration;
}

void image_process_set_bake_write_enabled(const bool enabled)
{
  g_image_process_bake_write_enabled = enabled;
}

bool image_process_bake_write_enabled()
{
  return g_image_process_bake_write_enabled;
}

void image_process_set_paint_force_schedule(const uint tree_session_uid,
                                            const int node_identifier)
{
  g_image_process_paint_force_tree_session_uid = tree_session_uid;
  g_image_process_paint_force_node_identifier = node_identifier;
}

void image_process_clear_paint_force_schedule()
{
  g_image_process_paint_force_tree_session_uid = 0;
  g_image_process_paint_force_node_identifier = 0;
}

bool image_process_paint_force_schedule_active()
{
  return g_image_process_paint_force_node_identifier != 0;
}

uint image_process_paint_force_tree_session_uid()
{
  return g_image_process_paint_force_tree_session_uid;
}

int image_process_paint_force_node_identifier()
{
  return g_image_process_paint_force_node_identifier;
}

void image_process_set_fluid_scrub_fill(const bool active)
{
  g_image_process_fluid_scrub_fill = active;
}
bool image_process_get_fluid_scrub_fill()
{
  return g_image_process_fluid_scrub_fill;
}

void image_process_set_terminal_filter(const Span<int> node_identifiers)
{
  g_image_process_terminal_filter = Vector<int>(node_identifiers);
}

void image_process_clear_terminal_filter()
{
  g_image_process_terminal_filter.clear();
}

bool image_process_terminal_filter_active()
{
  return !g_image_process_terminal_filter.is_empty();
}

bool image_process_terminal_filter_contains(const int node_identifier)
{
  return g_image_process_terminal_filter.contains(node_identifier);
}

Span<int> image_process_terminal_filter_get()
{
  return g_image_process_terminal_filter.as_span();
}

/* A node operation that allocates all of its outputs as invalid. */
class UndefinedNodeOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    this->allocate_default_remaining_outputs();
  }
};

NodeOperation *get_undefined_node_operation(Context &context, const bNode &node)
{
  return new UndefinedNodeOperation(context, node);
}

/* -------------------------------------------------------------------- */
/** \name Image Process zone state (Repeat multipass + Simulation feedback)
 *
 * Repeat: multipass carries Output→Input via a per-item snapshot.
 *
 * Simulation (Image Process): **no multi-frame dense/checkpoint cache**.
 * Only one feedback slot (last Output state). Each cook:
 *   Input loads last Output → body runs → Output overwrites the slot.
 * No scrub fill, no timeline cache ranges. Independent modules must not re-run this.
 * \{ */

struct ImageZoneCacheKey {
  uint session_uid = 0;
  int32_t output_node_id = 0;
  std::string item_id;

  uint64_t hash() const
  {
    return get_default_hash(session_uid, output_node_id, item_id);
  }

  friend bool operator==(const ImageZoneCacheKey &a, const ImageZoneCacheKey &b)
  {
    return a.session_uid == b.session_uid && a.output_node_id == b.output_node_id &&
           a.item_id == b.item_id;
  }
};

struct ImageZonePayload {
  ResultType type = ResultType::Color;
  int2 size = int2(0);
  bool is_single = false;
  /** Single-value POD only (float/color etc). */
  Vector<uint8_t> bytes;
  /**
   * Full texture state for Repeat multipass / Simulation feedback.
   * Shares GPU (or CPU) storage via Result::share_data — no PCIe download each iteration.
   */
  std::unique_ptr<Result> texture;

  bool empty() const
  {
    if (texture && texture->is_allocated()) {
      return false;
    }
    return bytes.is_empty();
  }

  void clear()
  {
    /* Drop shared GPU/CPU hold; ImplicitSharingPtr on ~Result releases the last user. */
    texture.reset();
    bytes.clear();
    size = int2(0);
    is_single = false;
  }
};

struct ImageZoneCacheEntry {
  /* Repeat: last iteration output (overwritten each pass). */
  int solved_frame = INT_MIN;
  ImageZonePayload output;
};

struct ImageSimZoneKey {
  uint session_uid = 0;
  int32_t output_node_id = 0;

  uint64_t hash() const
  {
    return get_default_hash(session_uid, output_node_id);
  }

  friend bool operator==(const ImageSimZoneKey &a, const ImageSimZoneKey &b)
  {
    return a.session_uid == b.session_uid && a.output_node_id == b.output_node_id;
  }
};

/**
 * Per simulation zone: single last-frame feedback only (no dense/checkpoint history).
 * `feedback` is written by Simulation Output and read by Simulation Input on the next cook.
 * `input_snapshot` is cleared each cook so same-frame re-entry always reloads from feedback.
 */
struct ImageSimZoneCache {
  int last_solved = INT_MIN;
  int input_frame = INT_MIN;
  /** item_id → payload fed into the body for `input_frame` (within one cook only). */
  Map<std::string, ImageZonePayload> input_snapshot;
  /** item_id → last Output state (the only durable feedback). */
  Map<std::string, ImageZonePayload> feedback;
};

static Map<ImageZoneCacheKey, ImageZoneCacheEntry> &image_zone_cache()
{
  static Map<ImageZoneCacheKey, ImageZoneCacheEntry> cache;
  return cache;
}

static Map<ImageSimZoneKey, ImageSimZoneCache> &image_sim_zone_caches()
{
  static Map<ImageSimZoneKey, ImageSimZoneCache> cache;
  return cache;
}

void image_process_set_cook_tree_session_uid(const uint session_uid)
{
  g_image_process_cook_tree_session_uid = session_uid;
}

uint image_process_cook_tree_session_uid()
{
  return g_image_process_cook_tree_session_uid;
}

void image_process_prepare_cook()
{
  /* Drop same-frame input snapshots so each cook reloads from last Output feedback (or cold
   * start). Leaving snapshots across cooks re-used the *input* of the first cook and ignored
   * Output feedback (looked frozen). */
  for (ImageSimZoneCache &sim : image_sim_zone_caches().values()) {
    sim.input_frame = INT_MIN;
    for (ImageZonePayload &p : sim.input_snapshot.values()) {
      p.clear();
    }
    sim.input_snapshot.clear();
  }
}

static void sim_clear_cache(ImageSimZoneCache &sim)
{
  for (ImageZonePayload &p : sim.input_snapshot.values()) {
    p.clear();
  }
  sim.input_snapshot.clear();
  for (ImageZonePayload &p : sim.feedback.values()) {
    p.clear();
  }
  sim.feedback.clear();
  sim.last_solved = INT_MIN;
  sim.input_frame = INT_MIN;
}

void image_process_reset_sim_zone(const uint session_uid, const int32_t output_node_id)
{
  const ImageSimZoneKey key{session_uid, output_node_id};
  if (ImageSimZoneCache *sim = image_sim_zone_caches().lookup_ptr(key)) {
    sim_clear_cache(*sim);
    image_sim_zone_caches().remove(key);
  }
  /* Also drop any residual per-item zone entries keyed by this output. */
  Vector<ImageZoneCacheKey> remove_keys;
  for (const auto &item : image_zone_cache().items()) {
    if (item.key.session_uid == session_uid && item.key.output_node_id == output_node_id) {
      remove_keys.append(item.key);
    }
  }
  for (const ImageZoneCacheKey &k : remove_keys) {
    if (ImageZoneCacheEntry *e = image_zone_cache().lookup_ptr(k)) {
      e->output.clear();
    }
    image_zone_cache().remove(k);
  }
}

void image_process_clear_zone_caches()
{
  /* Explicitly clear payloads so Result/GPU textures free while the GPU context is alive.
   * Static Map atexit after #GPU_exit previously crashed in VKDiscardPool::discard_image. */
  for (ImageZoneCacheEntry &entry : image_zone_cache().values()) {
    entry.output.clear();
  }
  image_zone_cache().clear();

  for (ImageSimZoneCache &sim : image_sim_zone_caches().values()) {
    sim_clear_cache(sim);
  }
  image_sim_zone_caches().clear();
}

/** Overwrite the single feedback slot for this item (no multi-frame history). */
static void sim_store_feedback(ImageSimZoneCache &cache,
                               const int frame,
                               const std::string &item_id,
                               ImageZonePayload &&payload)
{
  cache.feedback.add_overwrite(item_id, std::move(payload));
  cache.last_solved = frame;
}

static bool is_zone_state_socket(const bNodeSocket &socket)
{
  if (!ELEM(socket.type, SOCK_RGBA, SOCK_FLOAT, SOCK_VECTOR)) {
    return false;
  }
  const StringRef id = socket.identifier;
  const StringRef name = socket.name;
  if (id.is_empty() || id.startswith("__")) {
    return false;
  }
  if (ELEM(name, "Delta Time", "Skip", "Geometry", "Iteration", "Iterations", "Break")) {
    return false;
  }
  return true;
}

static int result_element_bytes(const ResultType type)
{
  switch (type) {
    case ResultType::Float:
      return int(sizeof(float));
    case ResultType::Float2:
      return int(sizeof(float) * 2);
    case ResultType::Float3:
      return int(sizeof(float) * 3);
    case ResultType::Color:
    case ResultType::Float4:
      return int(sizeof(float) * 4);
    default:
      return 0;
  }
}

static bool type_is_cacheable(const ResultType type)
{
  return ELEM(type,
              ResultType::Float,
              ResultType::Float2,
              ResultType::Float3,
              ResultType::Float4,
              ResultType::Color);
}

static bool capture_result_to_payload(Context &context, Result &input, ImageZonePayload &payload)
{
  if (!type_is_cacheable(input.type())) {
    return false;
  }

  payload.clear();

  if (input.is_single_value()) {
    const int elem = result_element_bytes(input.type());
    if (elem <= 0) {
      return false;
    }
    payload.type = input.type();
    payload.is_single = true;
    payload.size = int2(1);
    payload.bytes.reinitialize(elem);
    switch (input.type()) {
      case ResultType::Float: {
        const float v = input.get_single_value_default<float>();
        std::memcpy(payload.bytes.data(), &v, sizeof(float));
        break;
      }
      case ResultType::Float2: {
        const float2 v = input.get_single_value_default<float2>();
        std::memcpy(payload.bytes.data(), &v, sizeof(float2));
        break;
      }
      case ResultType::Float3: {
        const float3 v = input.get_single_value_default<float3>();
        std::memcpy(payload.bytes.data(), &v, sizeof(float3));
        break;
      }
      case ResultType::Float4: {
        const float4 v = input.get_single_value_default<float4>();
        std::memcpy(payload.bytes.data(), &v, sizeof(float4));
        break;
      }
      case ResultType::Color: {
        const Color c = input.get_single_value_default<Color>();
        std::memcpy(payload.bytes.data(), &c, sizeof(Color));
        break;
      }
      default:
        payload.clear();
        return false;
    }
    return true;
  }

  if (!input.is_allocated()) {
    return false;
  }

  /* Own a durable copy (not texture-pool / not a share of a transient op result).
   * share_data alone was freed by multipass free_results and TexturePool::reset between
   * cooks — Simulation looked dead and Repeat feedback could point at recycled memory. */
  payload.type = input.type();
  payload.is_single = false;
  payload.size = input.domain().data_size;
  payload.texture = std::make_unique<Result>(context, input.type(), input.precision());
  /* allocate_texture asserts should_compute(); keep a hold so the cache Result is valid. */
  payload.texture->set_reference_count(1);
  payload.texture->allocate_texture(input.domain(), false);
  if (!payload.texture->is_allocated()) {
    payload.clear();
    return false;
  }
  if (input.is_stored_on_gpu() && payload.texture->is_stored_on_gpu()) {
    gpu::Texture *dst = payload.texture->gpu_texture();
    gpu::Texture *src = input.gpu_texture();
    if (dst == nullptr || src == nullptr) {
      payload.clear();
      return false;
    }
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_UPDATE);
    GPU_texture_copy(dst, src);
  }
  else if (!input.is_stored_on_gpu() && input.cpu_data().data() &&
           payload.texture->is_stored_on_gpu())
  {
    /* CPU input → temporary GPU then copy, or upload path. */
    Result gpu = input.upload_to_gpu(false);
    if (gpu.is_allocated() && gpu.is_stored_on_gpu() && gpu.gpu_texture() != nullptr &&
        payload.texture->gpu_texture() != nullptr)
    {
      GPU_memory_barrier(GPU_BARRIER_TEXTURE_UPDATE);
      GPU_texture_copy(payload.texture->gpu_texture(), gpu.gpu_texture());
    }
    else {
      gpu.release();
      payload.clear();
      return false;
    }
    gpu.release();
  }
  else if (!input.is_stored_on_gpu() && input.cpu_data().data() &&
           !payload.texture->is_stored_on_gpu())
  {
    if (payload.texture->size_in_bytes() == input.size_in_bytes()) {
      std::memcpy(payload.texture->cpu_data_for_write().data(),
                  input.cpu_data().data(),
                  size_t(input.size_in_bytes()));
    }
  }
  payload.texture->set_transformation(input.domain().transformation);
  return true;
}

static bool load_payload_to_result(Context &context,
                                   const ImageZonePayload &payload,
                                   Result &output)
{
  if (payload.empty() || !type_is_cacheable(payload.type)) {
    return false;
  }
  if (output.type() != payload.type) {
    return false;
  }

  if (payload.is_single) {
    output.allocate_single_value();
    switch (payload.type) {
      case ResultType::Float: {
        float v = 0.0f;
        std::memcpy(&v, payload.bytes.data(), sizeof(float));
        output.set_single_value(v);
        break;
      }
      case ResultType::Float2: {
        float2 v(0.0f);
        std::memcpy(&v, payload.bytes.data(), sizeof(float2));
        output.set_single_value(v);
        break;
      }
      case ResultType::Float3: {
        float3 v(0.0f);
        std::memcpy(&v, payload.bytes.data(), sizeof(float3));
        output.set_single_value(v);
        break;
      }
      case ResultType::Float4: {
        float4 v(0.0f);
        std::memcpy(&v, payload.bytes.data(), sizeof(float4));
        output.set_single_value(v);
        break;
      }
      case ResultType::Color: {
        Color c(0, 0, 0, 1);
        std::memcpy(&c, payload.bytes.data(), sizeof(Color));
        output.set_single_value(c);
        break;
      }
      default:
        return false;
    }
    return true;
  }

  /* Texture path: re-share GPU/CPU storage into the new output (no upload). */
  if (payload.texture && payload.texture->is_allocated()) {
    if (payload.texture->is_stored_on_gpu()) {
      /* Keep output's Context; only attach texture + sharing_info. */
      output.share_data(payload.texture->gpu_texture(), payload.texture->sharing_info());
      output.set_transformation(payload.texture->domain().transformation);
      return true;
    }
    if (payload.texture->cpu_data().data()) {
      output.share_data(payload.texture->cpu_data().data(),
                        payload.texture->domain().data_size,
                        payload.texture->sharing_info());
      output.set_transformation(payload.texture->domain().transformation);
      return true;
    }
  }

  /* Legacy fallback: POD bytes (should not hit after GPU share path). */
  if (payload.size.x < 1 || payload.size.y < 1 || payload.bytes.is_empty()) {
    return false;
  }

  Result cpu = context.create_result(payload.type);
  cpu.allocate_texture(Domain(payload.size), false, ResultStorageType::CPUImage);
  if (!cpu.cpu_data().data() || cpu.size_in_bytes() != int64_t(payload.bytes.size())) {
    cpu.release();
    return false;
  }
  std::memcpy(cpu.cpu_data_for_write().data(), payload.bytes.data(), size_t(payload.bytes.size()));

  if (context.use_gpu()) {
    Result gpu = cpu.upload_to_gpu(true);
    output.share_data(gpu);
    gpu.release();
  }
  else {
    output.share_data(cpu);
  }
  cpu.release();
  return true;
}

static void allocate_zero_output(Result &out_result)
{
  out_result.allocate_single_value();
  switch (out_result.type()) {
    case ResultType::Float:
      out_result.set_single_value(0.0f);
      break;
    case ResultType::Int:
      out_result.set_single_value(0);
      break;
    case ResultType::Color:
      out_result.set_single_value(Color(0.0f, 0.0f, 0.0f, 1.0f));
      break;
    case ResultType::Float3:
      out_result.set_single_value(float3(0.0f));
      break;
    case ResultType::Float2:
      out_result.set_single_value(float2(0.0f));
      break;
    case ResultType::Float4:
      out_result.set_single_value(float4(0.0f));
      break;
    case ResultType::Bool:
      out_result.set_single_value(false);
      break;
    default:
      out_result.allocate_invalid();
      break;
  }
}

static float scene_fps(const Scene &scene)
{
  const float base = scene.r.frs_sec_base != 0 ? float(scene.r.frs_sec_base) : 1.0f;
  return math::max(float(scene.r.frs_sec) / base, 1e-6f);
}

static int scene_sim_start_frame(const Scene &scene)
{
  return (scene.r.flag & SCER_PRV_RANGE) ? scene.r.psfra : scene.r.sfra;
}

static const bNodeTree *node_owner_tree_if_valid(const bNode &node)
{
  if (!is_plausible_pointer(node.runtime)) {
    return nullptr;
  }
  const bNodeTree *tree = node.runtime->owner_tree;
  if (!is_plausible_pointer(tree)) {
    return nullptr;
  }
  return tree;
}

static uint node_tree_session_uid_or_cook(const bNode &node)
{
  if (const bNodeTree *tree = node_owner_tree_if_valid(node)) {
    return tree->id.session_uid;
  }
  return image_process_cook_tree_session_uid();
}

static const bNodeSocket *find_matching_input(const bNode &node, const bNodeSocket &output)
{
  /* ListBase, not topology-cache vectors: Python evaluate() can leave runtime->inputs empty. */
  for (const bNodeSocket &input : node.inputs) {
    if (!is_socket_available(&input)) {
      continue;
    }
    if (StringRef(input.identifier) == StringRef(output.identifier) &&
        !StringRef(input.identifier).startswith("__"))
    {
      return &input;
    }
  }
  for (const bNodeSocket &input : node.inputs) {
    if (!is_socket_available(&input)) {
      continue;
    }
    if (!StringRef(input.name).is_empty() && StringRef(input.name) == StringRef(output.name)) {
      return &input;
    }
  }
  return nullptr;
}

static bool pass_through_matching(Result &out_result, Result &in_result)
{
  if (!(in_result.is_allocated() || in_result.is_single_value())) {
    return false;
  }
  if (in_result.type() != out_result.type()) {
    return false;
  }
  out_result.share_data(in_result);
  return true;
}

/** \} */

/**
 * Zone boundary pass-through with Repeat multipass feedback and Simulation multi-frame cache.
 */
class PassThroughMatchingSocketsOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    const bool is_sim_out = this->node().is_type("GeometryNodeSimulationOutput"_ustr);
    const bool is_sim_in = this->node().is_type("GeometryNodeSimulationInput"_ustr);
    const bool is_rep_out = this->node().is_type("GeometryNodeRepeatOutput"_ustr);
    const bool is_rep_in = this->node().is_type("GeometryNodeRepeatInput"_ustr);
    /* owner_tree() can be null during Python tree.evaluate(); ID.session_uid is at 0x144. */
    const int frame = this->context().get_frame_number();
    const int start_frame = scene_sim_start_frame(this->context().get_scene());
    const bool sim_at_start = is_sim_in && frame <= start_frame;
    const uint session_uid = node_tree_session_uid_or_cook(this->node());
    const int rep_iter = image_process_get_repeat_iteration();

    int32_t zone_output_id = 0;
    if (is_sim_in || is_rep_in) {
      if (this->node().storage == nullptr) {
        this->allocate_default_remaining_outputs();
        return;
      }
      if (is_sim_in) {
        const auto &storage = *static_cast<const NodeGeometrySimulationInput *>(
            this->node().storage);
        zone_output_id = storage.output_node_id;
      }
      else {
        const auto &storage = *static_cast<const NodeGeometryRepeatInput *>(this->node().storage);
        zone_output_id = storage.output_node_id;
      }
    }
    else if (is_sim_out || is_rep_out) {
      zone_output_id = this->node().identifier;
    }

    /* Scrub/loop back to the start frame: drop accumulated feedback so Sim Input
     * re-reads initials (same as Geometry Nodes start-frame reset). */
    if (is_sim_in && zone_output_id != 0 && sim_at_start) {
      if (ImageSimZoneCache *sim_cache = image_sim_zone_caches().lookup_ptr(
              ImageSimZoneKey{session_uid, zone_output_id}))
      {
        if (sim_cache->last_solved > start_frame) {
          sim_clear_cache(*sim_cache);
        }
      }
    }

    /* ---- Zone Output: capture state ---- */
    if ((is_sim_out || is_rep_out) && zone_output_id != 0) {
      ImageSimZoneCache *sim_cache = nullptr;
      if (is_sim_out) {
        sim_cache = &image_sim_zone_caches().lookup_or_add_default(
            ImageSimZoneKey{session_uid, zone_output_id});
      }

      for (const bNodeSocket &input : this->node().inputs) {
        if (!is_socket_available(&input) || !is_zone_state_socket(input)) {
          continue;
        }
        if (!this->has_input(input.identifier)) {
          continue;
        }
        Result &in_result = this->get_input(input.identifier);

        if (is_rep_out) {
          ImageZoneCacheKey key{session_uid, zone_output_id, std::string(input.identifier)};
          ImageZoneCacheEntry &entry = image_zone_cache().lookup_or_add_default(key);
          capture_result_to_payload(this->context(), in_result, entry.output);
          entry.solved_frame = -1 - rep_iter;
        }
        else if (sim_cache) {
          ImageZonePayload payload;
          if (capture_result_to_payload(this->context(), in_result, payload)) {
            sim_store_feedback(
                *sim_cache, frame, std::string(input.identifier), std::move(payload));
          }
        }
      }
      /* Feedback is source of truth for the next cook; drop within-cook input snapshots. */
      if (is_sim_out && sim_cache) {
        sim_cache->input_frame = INT_MIN;
        for (ImageZonePayload &p : sim_cache->input_snapshot.values()) {
          p.clear();
        }
        sim_cache->input_snapshot.clear();
      }
    }

    /* ---- Zone Input / Output pass-through for sockets ---- */
    for (const bNodeSocket &output : this->node().outputs) {
      if (!is_socket_available(&output)) {
        continue;
      }
      if (!this->has_result(output.identifier)) {
        continue;
      }
      Result &out_result = this->get_result(output.identifier);
      if (!out_result.should_compute()) {
        continue;
      }

      /* Repeat Input: Iteration index. */
      if (is_rep_in && StringRef(output.name) == "Iteration") {
        out_result.allocate_single_value();
        out_result.set_single_value(rep_iter);
        continue;
      }

      /* Simulation Input: Delta Time from last solved frame (single feedback slot). */
      if (is_sim_in && StringRef(output.name) == "Delta Time") {
        float dt = 0.0f;
        if (zone_output_id != 0 && !sim_at_start) {
          if (const ImageSimZoneCache *sim_cache = image_sim_zone_caches().lookup_ptr(
                  ImageSimZoneKey{session_uid, zone_output_id}))
          {
            if (sim_cache->last_solved != INT_MIN && sim_cache->last_solved < frame) {
              dt = float(frame - sim_cache->last_solved) / scene_fps(this->context().get_scene());
            }
            else if (sim_cache->last_solved == frame) {
              /* Same frame re-cook after feedback: use one scene frame of dt. */
              dt = 1.0f / scene_fps(this->context().get_scene());
            }
          }
        }
        out_result.allocate_single_value();
        out_result.set_single_value(dt);
        continue;
      }

      /* Simulation Input: load single feedback slot, else cold-start initials. */
      if (is_sim_in && is_zone_state_socket(output) && zone_output_id != 0) {
        ImageSimZoneCache &sim_cache = image_sim_zone_caches().lookup_or_add_default(
            ImageSimZoneKey{session_uid, zone_output_id});
        const std::string item_id(output.identifier);

        bool loaded = false;

        /* Within-cook snapshot (body re-read of same Input in one schedule). */
        if (sim_cache.input_frame == frame) {
          if (const ImageZonePayload *snap = sim_cache.input_snapshot.lookup_ptr(item_id)) {
            if (!snap->empty()) {
              loaded = load_payload_to_result(this->context(), *snap, out_result);
            }
          }
        }

        /* Durable feedback from previous cook's Output. Start frame always
         * re-reads the external initials so Reset / rewind match frame 1. */
        if (!loaded && !sim_at_start) {
          if (const ImageZonePayload *payload = sim_cache.feedback.lookup_ptr(item_id)) {
            if (!payload->empty()) {
              loaded = load_payload_to_result(this->context(), *payload, out_result);
            }
          }
        }

        if (!loaded) {
          /* Cold start: external initial values on Sim Input sockets. */
          if (const bNodeSocket *matching = find_matching_input(this->node(), output)) {
            if (this->has_input(matching->identifier)) {
              Result &in_result = this->get_input(matching->identifier);
              loaded = pass_through_matching(out_result, in_result);
            }
          }
        }

        if (loaded) {
          ImageZonePayload snap;
          if (capture_result_to_payload(this->context(), out_result, snap)) {
            sim_cache.input_snapshot.add_overwrite(item_id, std::move(snap));
            sim_cache.input_frame = frame;
          }
          continue;
        }
      }

      /* Repeat Input: after first iteration, feed last Output state. */
      if (is_rep_in && is_zone_state_socket(output) && zone_output_id != 0 && rep_iter > 0) {
        ImageZoneCacheKey key{session_uid, zone_output_id, std::string(output.identifier)};
        if (const ImageZoneCacheEntry *entry = image_zone_cache().lookup_ptr(key)) {
          if (load_payload_to_result(this->context(), entry->output, out_result)) {
            continue;
          }
        }
      }

      /* Default: name/identifier-matched pass-through (also Sim Output UI outputs). */
      if (const bNodeSocket *matching = find_matching_input(this->node(), output)) {
        if (this->has_input(matching->identifier)) {
          Result &in_result = this->get_input(matching->identifier);
          if (pass_through_matching(out_result, in_result)) {
            continue;
          }
        }
      }

      allocate_zero_output(out_result);
    }
  }
};

NodeOperation *get_pass_through_matching_sockets_operation(Context &context, const bNode &node)
{
  return new PassThroughMatchingSocketsOperation(context, node);
}

int image_process_sim_resume_frame(const bNodeTree &ntree, const int target_frame)
{
  /* Simulation: no multi-frame fill — single feedback slot advances one cook per frame.
   * Fluid may still request fill for its own dense cache; honor that only. */
  int resume = target_frame;
  const int fluid_resume = nodes::image_fluid_zone::resume_frame(ntree, target_frame);
  resume = math::min(resume, fluid_resume);

  constexpr int max_fill = 64;
  if (target_frame - resume > max_fill) {
    resume = target_frame - max_fill;
  }
  return resume;
}

void image_process_sim_invalidate_after(const bNodeTree &ntree, const int keep_frame)
{
  /* No multi-frame sim history to invalidate (single feedback slot is not frame-ranged).
   * Fluid may still keep a dense window — forward invalidation there. */
  nodes::image_fluid_zone::invalidate_after(ntree, keep_frame);
}

static Vector<IndexRange> ranges_from_sorted_frames(const Vector<int> &frames)
{
  Vector<IndexRange> ranges;
  for (const int frame : frames) {
    if (ranges.is_empty() || frame - ranges.last().last() > 1) {
      ranges.append(IndexRange(frame, 1));
    }
    else {
      const IndexRange prev = ranges.pop_last();
      ranges.append(IndexRange(prev.start(), prev.size() + 1));
    }
  }
  return ranges;
}

Vector<IndexRange> image_process_sim_cached_frame_ranges()
{
  /* Simulation has no multi-frame cache; only report fluid ranges if any. */
  VectorSet<int> frames;
  for (const IndexRange &r : nodes::image_fluid_zone::cached_frame_ranges()) {
    for (const int f : r) {
      frames.add(f);
    }
  }
  Vector<int> sorted = frames.extract_vector();
  std::sort(sorted.begin(), sorted.end());
  return ranges_from_sorted_frames(sorted);
}

Vector<IndexRange> image_process_sim_checkpoint_frame_ranges()
{
  VectorSet<int> frames;
  for (const IndexRange &r : nodes::image_fluid_zone::checkpoint_frame_ranges()) {
    for (const int f : r) {
      frames.add(f);
    }
  }
  Vector<int> sorted = frames.extract_vector();
  std::sort(sorted.begin(), sorted.end());
  return ranges_from_sorted_frames(sorted);
}

}  // namespace blender::compositor
