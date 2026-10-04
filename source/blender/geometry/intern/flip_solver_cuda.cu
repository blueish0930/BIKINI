/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <cooperative_groups.h>
#include <cub/cub.cuh>
#include <cuda_runtime.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <mutex>
#include <vector>

#include "flip_solver_cuda_collision.hh"

namespace {

struct Params {
  int nx, ny, nz, particles;
  float3 origin, domain_max, dx, inverse_dx, gravity;
  float dt, density, flip_ratio, padding, classification_radius;
  bool has_collider;
  /* Resolve particle contacts on the device from the collider SDF (#solid_cell_phi). */
  bool gpu_collision;
  float collision_margin, collision_friction;
  const float *solid_cell_phi;
  /* Exact outward collider normals at cell centers, or null. */
  const float3 *solid_cell_normal;
  const unsigned char *blocked[3];
  /* #face_blocked precomputed per face: blocked, or next to a solid cell. */
  const unsigned char *face_solid[3];
  const float *solid_velocity[3];
  /* When set, grid kernels only visit these 8x8x8 tiles (linear tile indices). */
  const int *tiles;
  /* Number of active tiles, read on the device so the host never waits for it. */
  const int *tile_count;
  int3 tile_dims;
};

constexpr int tile_size = 8;
constexpr int tile_cells = tile_size * tile_size * tile_size;

/* Keeps freed device allocations for reuse by the next solve (usually the next frame with the
 * same grid). Bounded, so a changed resolution does not keep stale sizes forever. */
class DevicePool {
  struct Entry {
    void *data;
    size_t bytes;
  };
  static constexpr size_t max_cached_bytes = size_t(1) << 30;
  std::mutex mutex_;
  std::vector<Entry> entries_;
  size_t cached_bytes_ = 0;

 public:
  static DevicePool &get()
  {
    static DevicePool pool;
    return pool;
  }
  void *acquire(const size_t bytes)
  {
    {
      std::lock_guard lock(mutex_);
      for (size_t i = 0; i < entries_.size(); i++) {
        if (entries_[i].bytes == bytes) {
          void *data = entries_[i].data;
          cached_bytes_ -= bytes;
          entries_[i] = entries_.back();
          entries_.pop_back();
          return data;
        }
      }
    }
    void *data = nullptr;
    if (cudaMalloc(&data, bytes) != cudaSuccess) {
      /* Out of memory: drop the cache and retry once. */
      cudaGetLastError();
      this->clear();
      if (cudaMalloc(&data, bytes) != cudaSuccess) return nullptr;
    }
    return data;
  }
  void release(void *data, const size_t bytes)
  {
    {
      std::lock_guard lock(mutex_);
      if (cached_bytes_ + bytes <= max_cached_bytes) {
        entries_.push_back({data, bytes});
        cached_bytes_ += bytes;
        return;
      }
    }
    cudaFree(data);
  }
  void clear()
  {
    std::lock_guard lock(mutex_);
    for (const Entry &entry : entries_) cudaFree(entry.data);
    entries_.clear();
    cached_bytes_ = 0;
  }
};

template<typename T> struct Buffer {
  T *data = nullptr;
  size_t bytes = 0;
  ~Buffer() { this->reset(); }
  void reset()
  {
    if (data) DevicePool::get().release(data, bytes);
    data = nullptr;
    bytes = 0;
  }
  bool allocate(const size_t count)
  {
    bytes = std::max<size_t>(sizeof(T) * count, sizeof(T));
    data = static_cast<T *>(DevicePool::get().acquire(bytes));
    return data != nullptr;
  }
  Buffer(const Buffer &) = delete;
  Buffer &operator=(const Buffer &) = delete;
  Buffer() = default;
};

/* Time per solver step when BLENDER_FLIP_PROFILE is set. Events are recorded in stream order,
 * and each segment between two events is attributed to the step that ended it. Gaps where the
 * device waits for the host (callbacks, uploads, readbacks) are therefore included. */
enum class Step : int {
  Setup,
  HostPrepare,
  Classify,
  GpuReseed,
  Tiles,
  P2GClear,
  P2GScatter,
  P2GNormalize,
  Extrapolate,
  Boundary,
  Viscosity,
  PressureSetup,
  PressurePcg,
  Project,
  G2P,
  HostAfter,
  Finish,
  Count,
};

struct StepTimer {
  bool enabled = false;
  std::vector<cudaEvent_t> events;
  std::vector<int> steps;
  double upload_ms = 0.0, download_ms = 0.0;
  int upload_particles = 0, download_particles = 0;
  void note_transfer(const double up, const double down, const int up_count, const int down_count)
  {
    upload_ms = up;
    download_ms = down;
    upload_particles = up_count;
    download_particles = down_count;
  }
  long long active_tiles = 0, total_tiles = 0, active_cells = 0;
  int tile_samples = 0;

  void note_tiles(const int active, const int total, const int cells)
  {
    if (!enabled) return;
    active_tiles += active;
    total_tiles += total;
    active_cells += cells;
    tile_samples++;
  }

  StepTimer()
  {
    enabled = std::getenv("BLENDER_FLIP_PROFILE") != nullptr;
    if (enabled) this->lap(Step::Setup);
  }
  ~StepTimer()
  {
    for (cudaEvent_t event : events) cudaEventDestroy(event);
  }
  void lap(const Step step)
  {
    if (!enabled) return;
    cudaEvent_t event;
    cudaEventCreate(&event);
    cudaEventRecord(event);
    events.push_back(event);
    steps.push_back(int(step));
  }
  void print()
  {
    if (!enabled || events.size() < 2) return;
    cudaEventSynchronize(events.back());
    double ms[int(Step::Count)] = {};
    for (size_t i = 1; i < events.size(); i++) {
      float elapsed = 0.0f;
      cudaEventElapsedTime(&elapsed, events[i - 1], events[i]);
      ms[steps[i]] += elapsed;
    }
    std::fprintf(stderr,
                 "FLIP_CUDA_STEPS setup=%.3f host_prepare=%.3f classify=%.3f gpu_reseed=%.3f "
                 "tiles=%.3f p2g_clear=%.3f p2g_scatter=%.3f p2g_normalize=%.3f "
                 "extrapolate=%.3f boundary=%.3f viscosity=%.3f pressure_setup=%.3f "
                 "pressure_pcg=%.3f project=%.3f g2p=%.3f host_after=%.3f finish=%.3f\n",
                 ms[0], ms[1], ms[2], ms[3], ms[4], ms[5], ms[6], ms[7], ms[8], ms[9], ms[10],
                 ms[11], ms[12], ms[13], ms[14], ms[15], ms[16]);
    std::fprintf(stderr, "FLIP_CUDA_TRANSFER upload_ms=%.3f (%d particles) download_ms=%.3f (%d particles)\n",
                 upload_ms, upload_particles, download_ms, download_particles);
    if (tile_samples > 0) {
      std::fprintf(stderr,
                   "FLIP_CUDA_TILES avg_active_tiles=%.1f total_tiles=%.1f "
                   "avg_fluid_cells=%.1f\n",
                   double(active_tiles) / tile_samples, double(total_tiles) / tile_samples,
                   double(active_cells) / tile_samples);
    }
  }
};

struct CapturedGraph {
  cudaGraphExec_t executable = nullptr;
  ~CapturedGraph() { if (executable) cudaGraphExecDestroy(executable); }
};

/* Device buffer whose size changes between substeps. Capacities are powers of two, so the pool
 * sees a few distinct sizes instead of one per substep. */
template<typename T> struct GrowBuffer {
  Buffer<T> buffer;
  size_t capacity = 0;
  bool ensure(const size_t count)
  {
    if (count <= capacity && buffer.data) return true;
    size_t next = 1024;
    while (next < count) next <<= 1;
    buffer.reset();
    capacity = next;
    return buffer.allocate(next);
  }
  T *data() const { return buffer.data; }
};

/* One cached set of static collider fields, keyed by collider geometry and grid. Copied into
 * the runner's own buffers, so concurrent solves never share live device memory. */
struct DeviceColliderCache {
  uint64_t key = 0;
  int cells = 0;
  int counts[3] = {0, 0, 0};
  bool has_normal = false;
  float *cell_phi = nullptr;
  float3 *cell_normal = nullptr;
  unsigned char *blocked[3] = {nullptr, nullptr, nullptr};
  float *velocity[3] = {nullptr, nullptr, nullptr};

  static DeviceColliderCache &get()
  {
    /* Intentionally never destroyed: the CUDA context may be gone at static destruction. */
    static DeviceColliderCache *cache = new DeviceColliderCache();
    return *cache;
  }
  static std::mutex &mutex()
  {
    static std::mutex m;
    return m;
  }
  void release()
  {
    cudaFree(cell_phi);
    cudaFree(cell_normal);
    for (int axis = 0; axis < 3; axis++) {
      cudaFree(blocked[axis]);
      cudaFree(velocity[axis]);
      blocked[axis] = nullptr;
      velocity[axis] = nullptr;
    }
    cell_phi = nullptr;
    cell_normal = nullptr;
    key = 0;
  }
};

struct Face {
  Buffer<float> value, weight, alternate, old;
  Buffer<unsigned char> valid, alternate_valid;
  Buffer<int> active, active_count;
  int count = 0;
  bool allocate(const int n, const bool viscosity)
  {
    count = n;
    return value.allocate(n) && weight.allocate(n) && alternate.allocate(n) && old.allocate(n) &&
           valid.allocate(n) && alternate_valid.allocate(n) &&
           (!viscosity || (active.allocate(n) && active_count.allocate(1)));
  }
};

struct ColliderBuffers {
  Buffer<float> cell_phi;
  Buffer<float3> cell_normal;
  Buffer<unsigned char> face_solid[3];
  Buffer<unsigned char> blocked[3];
  Buffer<float> velocity[3];

  bool allocate(const int cells, const int counts[3])
  {
    if (!cell_phi.allocate(cells) || !cell_normal.allocate(cells)) return false;
    for (int axis = 0; axis < 3; axis++) {
      if (!blocked[axis].allocate(counts[axis]) || !velocity[axis].allocate(counts[axis]) ||
          !face_solid[axis].allocate(counts[axis]))
      {
        return false;
      }
    }
    return true;
  }
};

__device__ int cell_index(const Params p, const int x, const int y, const int z)
{
  return (z * p.ny + y) * p.nx + x;
}

__device__ int3 face_size(const Params p, const int axis)
{
  return make_int3(p.nx + (axis == 0), p.ny + (axis == 1), p.nz + (axis == 2));
}

__device__ int face_index(const int3 size, const int x, const int y, const int z)
{
  return (z * size.y + y) * size.x + x;
}

__device__ int3 face_coord(const int3 size, const int index)
{
  const int z = index / (size.x * size.y);
  const int left = index - z * size.x * size.y;
  return make_int3(left % size.x, left / size.x, z);
}

/* Grid kernels loop over elements with a fixed launch size: either linearly over the whole grid,
 * or over the active tiles, whose count is only known on the device. */
__device__ int grid_limit(const Params p, const int3 size)
{
  return p.tiles ? *p.tile_count * tile_cells : size.x * size.y * size.z;
}

__device__ bool grid_element(const Params p, const int3 size, const int t, int &index, int3 &c)
{
  if (p.tiles == nullptr) {
    index = t;
    c = face_coord(size, t);
    return true;
  }
  const int tile = p.tiles[t / tile_cells];
  const int tx = tile % p.tile_dims.x;
  const int rest = tile / p.tile_dims.x;
  const int ty = rest % p.tile_dims.y;
  const int tz = rest / p.tile_dims.y;
  const int local = t % tile_cells;
  c = make_int3(tx * tile_size + local % tile_size,
                ty * tile_size + (local / tile_size) % tile_size,
                tz * tile_size + local / (tile_size * tile_size));
  if (c.x >= size.x || c.y >= size.y || c.z >= size.z) return false;
  index = face_index(size, c.x, c.y, c.z);
  return true;
}

#define GRID_LOOP_BEGIN(p, size, index, c) \
  { \
    const int3 grid_size_ = (size); \
    const int grid_limit_ = grid_limit((p), grid_size_); \
    for (int grid_t_ = blockIdx.x * blockDim.x + threadIdx.x; grid_t_ < grid_limit_; \
         grid_t_ += blockDim.x * gridDim.x) \
    { \
      int index; \
      int3 c; \
      if (!grid_element((p), grid_size_, grid_t_, index, c)) continue;
/* Inside the loop body, `continue` skips to the next element. */
#define GRID_LOOP_END \
    } \
  }

__device__ float axis_value(const float3 value, const int axis)
{
  return axis == 0 ? value.x : axis == 1 ? value.y : value.z;
}

__device__ float3 grid_position(const Params p, const float3 position)
{
  return make_float3((position.x - p.origin.x) * p.inverse_dx.x,
                     (position.y - p.origin.y) * p.inverse_dx.y,
                     (position.z - p.origin.z) * p.inverse_dx.z);
}

__device__ int3 particle_cell(const Params p, const float3 position)
{
  const float3 g = grid_position(p, position);
  return make_int3(max(0, min(p.nx - 1, int(floorf(g.x)))),
                   max(0, min(p.ny - 1, int(floorf(g.y)))),
                   max(0, min(p.nz - 1, int(floorf(g.z)))));
}

__global__ void mark_tiles(const Params p, const float3 *positions, int *flags)
{
  const int particle = blockIdx.x * blockDim.x + threadIdx.x;
  if (particle >= p.particles) return;
  const int3 cell = particle_cell(p, positions[particle]);
  const int tile = ((cell.z / tile_size) * p.tile_dims.y + cell.y / tile_size) * p.tile_dims.x +
                   cell.x / tile_size;
  flags[tile] = 1;
}

/* Active tiles: tiles with particles, dilated by one tile. Every quantity the solver reads near
 * particles (classification, extrapolation layers, stencils) stays within one tile of them. */
__global__ void collect_tiles(const Params p, const int *flags, int *tiles, int *count)
{
  const int tile = blockIdx.x * blockDim.x + threadIdx.x;
  const int total = p.tile_dims.x * p.tile_dims.y * p.tile_dims.z;
  if (tile >= total) return;
  const int tx = tile % p.tile_dims.x;
  const int rest = tile / p.tile_dims.x;
  const int ty = rest % p.tile_dims.y;
  const int tz = rest / p.tile_dims.y;
  for (int z = max(0, tz - 1); z <= min(p.tile_dims.z - 1, tz + 1); z++) {
    for (int y = max(0, ty - 1); y <= min(p.tile_dims.y - 1, ty + 1); y++) {
      for (int x = max(0, tx - 1); x <= min(p.tile_dims.x - 1, tx + 1); x++) {
        if (flags[(z * p.tile_dims.y + y) * p.tile_dims.x + x]) {
          tiles[atomicAdd(count, 1)] = tile;
          return;
        }
      }
    }
  }
}

__global__ void clear_faces(const Params p, const int axis, float *a, float *b)
{
  GRID_LOOP_BEGIN(p, face_size(p, axis), index, c)
  a[index] = 0.0f;
  b[index] = 0.0f;
  GRID_LOOP_END
}

__global__ void copy_faces(const Params p, const int axis, const float *source, float *target)
{
  GRID_LOOP_BEGIN(p, face_size(p, axis), index, c)
  target[index] = source[index];
  GRID_LOOP_END
}

__device__ bool fluid_at(const Params p, const int *fluid, int x, int y, int z);

__device__ bool solid_at(const Params p, const int x, const int y, const int z)
{
  if (x < 0 || y < 0 || z < 0 || x >= p.nx || y >= p.ny || z >= p.nz) return true;
  return p.has_collider && p.solid_cell_phi[cell_index(p, x, y, z)] < 0.0f;
}

/* Whether a face is closed by the collider: sampled as blocked, or next to a solid cell. */
__device__ bool face_blocked_from_fields(const Params p, const int axis, const int3 c)
{
  if (!p.has_collider) return false;
  const int index = face_index(face_size(p, axis), c.x, c.y, c.z);
  const int3 lower = make_int3(c.x - (axis == 0), c.y - (axis == 1), c.z - (axis == 2));
  return p.blocked[axis][index] ||
         (lower.x >= 0 && lower.y >= 0 && lower.z >= 0 &&
          solid_at(p, lower.x, lower.y, lower.z)) ||
         (c.x < p.nx && c.y < p.ny && c.z < p.nz && solid_at(p, c.x, c.y, c.z));
}

__device__ bool face_blocked(const Params p, const int axis, const int3 c)
{
  if (!p.has_collider) return false;
  return p.face_solid[axis][face_index(face_size(p, axis), c.x, c.y, c.z)];
}

/* The collider is fixed during a substep, so #face_blocked is evaluated once per face. */
__global__ void compute_face_solid(const Params p, const int axis, unsigned char *face_solid)
{
  const int3 size = face_size(p, axis);
  const int count = size.x * size.y * size.z;
  for (int index = blockIdx.x * blockDim.x + threadIdx.x; index < count;
       index += blockDim.x * gridDim.x)
  {
    face_solid[index] = face_blocked_from_fields(p, axis, face_coord(size, index));
  }
}

__global__ void mask_solid_cells(const Params p, int *fluid)
{
  GRID_LOOP_BEGIN(p, make_int3(p.nx, p.ny, p.nz), index, c)
  if (p.solid_cell_phi[index] < 0.0f) fluid[index] = 0;
  GRID_LOOP_END
}

__device__ uint32_t cpu_hash_u32(uint32_t value)
{
  value ^= value >> 16;
  value *= 0x7feb352dU;
  value ^= value >> 15;
  value *= 0x846ca68bU;
  value ^= value >> 16;
  return value;
}

__device__ float cpu_hash_unit(const uint32_t a, const uint32_t b)
{
  return float(cpu_hash_u32(a ^ cpu_hash_u32(b + 0x9e3779b9U)) & 0x00ffffffU) /
         float(0x01000000U);
}

__device__ int3 cpu_cell(const Params p, const float3 position)
{
  const float qx = __fdiv_rn(__fsub_rn(position.x, p.origin.x), p.dx.x);
  const float qy = __fdiv_rn(__fsub_rn(position.y, p.origin.y), p.dx.y);
  const float qz = __fdiv_rn(__fsub_rn(position.z, p.origin.z), p.dx.z);
  return make_int3(max(0, min(p.nx - 1, int(floorf(qx)))),
                   max(0, min(p.ny - 1, int(floorf(qy)))),
                   max(0, min(p.nz - 1, int(floorf(qz)))));
}

__device__ float3 cpu_cell_center(const Params p, const int x, const int y, const int z)
{
  return make_float3(__fadd_rn(p.origin.x, __fmul_rn(float(x) + 0.5f, p.dx.x)),
                     __fadd_rn(p.origin.y, __fmul_rn(float(y) + 0.5f, p.dx.y)),
                     __fadd_rn(p.origin.z, __fmul_rn(float(z) + 0.5f, p.dx.z)));
}

/* Same rule and float operations as #FlipSolverCore::classify_cells_particle_sdf: a cell is
 * fluid when it contains a particle or a particle is closer to its center than the radius. */
__global__ void classify_particles(const Params p,
                                   const float3 *positions,
                                   int *fluid)
{
  const int particle = blockIdx.x * blockDim.x + threadIdx.x;
  if (particle >= p.particles) return;
  const float3 position = positions[particle];
  const int3 cell = cpu_cell(p, position);
  atomicExch(fluid + cell_index(p, cell.x, cell.y, cell.z), 1);
  const float radius = p.classification_radius;
  if (radius <= 0.0f) return;
  for (int z = max(0, cell.z - 1); z <= min(p.nz - 1, cell.z + 1); z++) {
    for (int y = max(0, cell.y - 1); y <= min(p.ny - 1, cell.y + 1); y++) {
      for (int x = max(0, cell.x - 1); x <= min(p.nx - 1, cell.x + 1); x++) {
        const float3 center = cpu_cell_center(p, x, y, z);
        const float dx = __fsub_rn(center.x, position.x);
        const float dy = __fsub_rn(center.y, position.y);
        const float dz = __fsub_rn(center.z, position.z);
        const float distance = __fsqrt_rn(
            __fadd_rn(__fadd_rn(__fmul_rn(dx, dx), __fmul_rn(dy, dy)), __fmul_rn(dz, dz)));
        if (distance < radius) {
          atomicExch(fluid + cell_index(p, x, y, z), 1);
        }
      }
    }
  }
}

__global__ void scatter_faces(const Params p,
                              const float3 *positions,
                              const float3 *velocities,
                              const int axis,
                              float *values,
                              float *weights)
{
  const int particle = blockIdx.x * blockDim.x + threadIdx.x;
  if (particle >= p.particles) return;
  float3 sample = grid_position(p, positions[particle]);
  if (axis != 0) sample.x -= 0.5f;
  if (axis != 1) sample.y -= 0.5f;
  if (axis != 2) sample.z -= 0.5f;
  const int3 size = face_size(p, axis);
  const int x0 = int(floorf(sample.x));
  const int y0 = int(floorf(sample.y));
  const int z0 = int(floorf(sample.z));
  for (int oz = 0; oz < 2; oz++) {
    const int z = z0 + oz;
    if (z < 0 || z >= size.z) continue;
    const float wz = fmaxf(0.0f, 1.0f - fabsf(sample.z - z));
    for (int oy = 0; oy < 2; oy++) {
      const int y = y0 + oy;
      if (y < 0 || y >= size.y) continue;
      const float wy = fmaxf(0.0f, 1.0f - fabsf(sample.y - y));
      for (int ox = 0; ox < 2; ox++) {
        const int x = x0 + ox;
        if (x < 0 || x >= size.x) continue;
        const float wx = fmaxf(0.0f, 1.0f - fabsf(sample.x - x));
        const float w = wx * wy * wz;
        if (w <= 0.0f) continue;
        const int index = face_index(size, x, y, z);
        atomicAdd(values + index, axis_value(velocities[particle], axis) * w);
        atomicAdd(weights + index, w);
      }
    }
  }
}

__global__ void normalize_faces(const Params p,
                                const int axis,
                                float *values,
                                const float *weights,
                                unsigned char *valid)
{
  GRID_LOOP_BEGIN(p, face_size(p, axis), index, c)
  const float w = weights[index];
  if (w > 1.0e-12f) {
    values[index] /= w;
    valid[index] = 1;
  }
  else {
    values[index] = 0.0f;
    valid[index] = 0;
  }
  GRID_LOOP_END
}

__global__ void extrapolate_faces(const Params p,
                                  const int axis,
                                  const float *input,
                                  const unsigned char *valid,
                                  float *output,
                                  unsigned char *output_valid)
{
  const int3 size = face_size(p, axis);
  GRID_LOOP_BEGIN(p, face_size(p, axis), index, c)
  output[index] = input[index];
  output_valid[index] = valid[index];
  if (valid[index] || face_blocked(p, axis, c)) continue;
  if ((axis == 0 && (c.x == 0 || c.x == p.nx)) ||
      (axis == 1 && (c.y == 0 || c.y == p.ny)) ||
      (axis == 2 && (c.z == 0 || c.z == p.nz))) continue;
  float sum = 0.0f;
  int found = 0;
  const int3 neighbors[6] = {make_int3(c.x - 1, c.y, c.z), make_int3(c.x + 1, c.y, c.z),
                             make_int3(c.x, c.y - 1, c.z), make_int3(c.x, c.y + 1, c.z),
                             make_int3(c.x, c.y, c.z - 1), make_int3(c.x, c.y, c.z + 1)};
  for (int i = 0; i < 6; i++) {
    const int3 n = neighbors[i];
    if (n.x < 0 || n.y < 0 || n.z < 0 || n.x >= size.x || n.y >= size.y || n.z >= size.z) {
      continue;
    }
    const int ni = face_index(size, n.x, n.y, n.z);
    if (valid[ni] && !face_blocked(p, axis, n)) { sum += input[ni]; found++; }
  }
  if (found) { output[index] = sum / found; output_valid[index] = 1; }
  GRID_LOOP_END
}

__device__ bool fluid_at(const Params p, const int *fluid, const int x, const int y,
                         const int z)
{
  return x >= 0 && y >= 0 && z >= 0 && x < p.nx && y < p.ny && z < p.nz &&
         fluid[cell_index(p, x, y, z)] != 0;
}

__device__ bool touches_fluid(const Params p, const int *fluid, const int axis,
                              const int3 c)
{
  return axis == 0 ? fluid_at(p, fluid, c.x - 1, c.y, c.z) ||
                         fluid_at(p, fluid, c.x, c.y, c.z) :
         axis == 1 ? fluid_at(p, fluid, c.x, c.y - 1, c.z) ||
                         fluid_at(p, fluid, c.x, c.y, c.z) :
                     fluid_at(p, fluid, c.x, c.y, c.z - 1) ||
                         fluid_at(p, fluid, c.x, c.y, c.z);
}

__global__ void force_and_boundary(const Params p, const int *fluid, const int axis,
                                   const bool apply_force, float *values)
{
  GRID_LOOP_BEGIN(p, face_size(p, axis), index, c)
  if (face_blocked(p, axis, c)) {
    values[index] = p.solid_velocity[axis][index];
  }
  else if ((axis == 0 && (c.x == 0 || c.x == p.nx)) ||
      (axis == 1 && (c.y == 0 || c.y == p.ny)) ||
      (axis == 2 && (c.z == 0 || c.z == p.nz))) {
    values[index] = 0.0f;
  }
  else if (apply_force && touches_fluid(p, fluid, axis, c)) {
    values[index] += axis_value(p.gravity, axis) * p.dt;
  }
  GRID_LOOP_END
}

__device__ bool viscosity_face_active(const Params p, const int *fluid,
                                      const int axis, const int3 c)
{
  if ((axis == 0 && (c.x == 0 || c.x == p.nx)) ||
      (axis == 1 && (c.y == 0 || c.y == p.ny)) ||
      (axis == 2 && (c.z == 0 || c.z == p.nz))) return false;
  return touches_fluid(p, fluid, axis, c) && !face_blocked(p, axis, c);
}

__global__ void collect_viscosity_faces(const Params p, const int *fluid,
                                        const int axis, int *active, int *count)
{
  GRID_LOOP_BEGIN(p, face_size(p, axis), index, c)
  if (viscosity_face_active(p, fluid, axis, c)) {
    active[atomicAdd(count, 1)] = index;
  }
  GRID_LOOP_END
}

__device__ void viscosity_row(const Params p, const int *fluid, const int axis,
                              const int3 c, const float *input, const float viscosity,
                              float &diagonal, float &sum, float &boundary_sum)
{
  const int3 size = face_size(p, axis);
  const float coefficients[3] = {p.dt * viscosity * p.inverse_dx.x * p.inverse_dx.x,
                                 p.dt * viscosity * p.inverse_dx.y * p.inverse_dx.y,
                                 p.dt * viscosity * p.inverse_dx.z * p.inverse_dx.z};
  diagonal = 1.0f;
  sum = 0.0f;
  boundary_sum = 0.0f;
  for (int neighbor_axis = 0; neighbor_axis < 3; neighbor_axis++) {
    for (int direction = -1; direction <= 1; direction += 2) {
      int3 n = c;
      if (neighbor_axis == 0) n.x += direction;
      else if (neighbor_axis == 1) n.y += direction;
      else n.z += direction;
      if (n.x < 0 || n.y < 0 || n.z < 0 ||
          n.x >= size.x || n.y >= size.y || n.z >= size.z) continue;
      const float coefficient = coefficients[neighbor_axis];
      if (viscosity_face_active(p, fluid, axis, n)) {
        diagonal += coefficient;
        sum += coefficient * input[face_index(size, n.x, n.y, n.z)];
      }
      else if (face_blocked(p, axis, n)) {
        diagonal += coefficient;
        boundary_sum += coefficient * p.solid_velocity[axis][face_index(size, n.x, n.y, n.z)];
      }
      else if (neighbor_axis == axis &&
               ((axis == 0 && (n.x == 0 || n.x == p.nx)) ||
                (axis == 1 && (n.y == 0 || n.y == p.ny)) ||
                (axis == 2 && (n.z == 0 || n.z == p.nz)))) {
        diagonal += coefficient;
      }
    }
  }
}

__global__ void viscosity_jacobi(const Params p, const int *fluid, const int axis,
                                 const int *active, const int count, const float viscosity,
                                 const float *rhs, const float *input, float *output)
{
  const int slot = blockIdx.x * blockDim.x + threadIdx.x;
  if (slot >= count) return;
  const int index = active[slot];
  const int3 c = face_coord(face_size(p, axis), index);
  float diagonal, sum, boundary_sum;
  viscosity_row(p, fluid, axis, c, input, viscosity, diagonal, sum, boundary_sum);
  output[index] = (rhs[index] + sum + boundary_sum) / diagonal;
}

__global__ void measure_viscosity(const Params p, const int *fluid, const int axis,
                                  const int *active, const int count, const float viscosity,
                                  const float *rhs, const float *solution,
                                  double *residual_squared, double *rhs_squared)
{
  const int slot = blockIdx.x * blockDim.x + threadIdx.x;
  if (slot >= count) return;
  const int index = active[slot];
  const int3 c = face_coord(face_size(p, axis), index);
  float diagonal, sum, boundary_sum;
  viscosity_row(p, fluid, axis, c, solution, viscosity, diagonal, sum, boundary_sum);
  const double effective_rhs = double(rhs[index]) + double(boundary_sum);
  const double residual = effective_rhs - double(diagonal * solution[index] - sum);
  atomicAdd(residual_squared, residual * residual);
  atomicAdd(rhs_squared, effective_rhs * effective_rhs);
}

__global__ void pressure_rhs(const Params p, const int *fluid,
                             const float *u, const float *v, const float *w,
                             float *rhs, float *diagonal)
{
  GRID_LOOP_BEGIN(p, make_int3(p.nx, p.ny, p.nz), index, cell)
  if (!fluid[index]) { rhs[index] = 0.0f; diagonal[index] = 1.0f; continue; }
  const int x = cell.x, y = cell.y, z = cell.z;
  const int3 us = face_size(p, 0), vs = face_size(p, 1), ws = face_size(p, 2);
  const float div = (u[face_index(us, x + 1, y, z)] - u[face_index(us, x, y, z)]) *
                        p.inverse_dx.x +
                    (v[face_index(vs, x, y + 1, z)] - v[face_index(vs, x, y, z)]) *
                        p.inverse_dx.y +
                    (w[face_index(ws, x, y, z + 1)] - w[face_index(ws, x, y, z)]) *
                        p.inverse_dx.z;
  rhs[index] = -p.density / p.dt * div;
  const float cx = p.inverse_dx.x * p.inverse_dx.x;
  const float cy = p.inverse_dx.y * p.inverse_dx.y;
  const float cz = p.inverse_dx.z * p.inverse_dx.z;
  float d = 0.0f;
  if (x > 0 && !face_blocked(p, 0, make_int3(x, y, z)) &&
      !solid_at(p, x - 1, y, z)) d += cx;
  if (x + 1 < p.nx && !face_blocked(p, 0, make_int3(x + 1, y, z)) &&
      !solid_at(p, x + 1, y, z)) d += cx;
  if (y > 0 && !face_blocked(p, 1, make_int3(x, y, z)) &&
      !solid_at(p, x, y - 1, z)) d += cy;
  if (y + 1 < p.ny && !face_blocked(p, 1, make_int3(x, y + 1, z)) &&
      !solid_at(p, x, y + 1, z)) d += cy;
  if (z > 0 && !face_blocked(p, 2, make_int3(x, y, z)) &&
      !solid_at(p, x, y, z - 1)) d += cz;
  if (z + 1 < p.nz && !face_blocked(p, 2, make_int3(x, y, z + 1)) &&
      !solid_at(p, x, y, z + 1)) d += cz;
  diagonal[index] = fmaxf(d, 1.0f);
  GRID_LOOP_END
}

__device__ float pressure_matrix_row(const Params p, const int *fluid,
                                     const float *diagonal, const float *x, const int index)
{
  const int z = index / (p.nx * p.ny);
  const int left = index - z * p.nx * p.ny;
  const int y = left / p.nx;
  const int ix = left % p.nx;
  const float cx = p.inverse_dx.x * p.inverse_dx.x;
  const float cy = p.inverse_dx.y * p.inverse_dx.y;
  const float cz = p.inverse_dx.z * p.inverse_dx.z;
  float value = diagonal[index] * x[index];
  if (fluid_at(p, fluid, ix - 1, y, z) &&
      !face_blocked(p, 0, make_int3(ix, y, z))) value -= cx * x[index - 1];
  if (fluid_at(p, fluid, ix + 1, y, z) &&
      !face_blocked(p, 0, make_int3(ix + 1, y, z))) value -= cx * x[index + 1];
  if (fluid_at(p, fluid, ix, y - 1, z) &&
      !face_blocked(p, 1, make_int3(ix, y, z))) value -= cy * x[index - p.nx];
  if (fluid_at(p, fluid, ix, y + 1, z) &&
      !face_blocked(p, 1, make_int3(ix, y + 1, z))) value -= cy * x[index + p.nx];
  if (fluid_at(p, fluid, ix, y, z - 1) &&
      !face_blocked(p, 2, make_int3(ix, y, z))) value -= cz * x[index - p.nx * p.ny];
  if (fluid_at(p, fluid, ix, y, z + 1) &&
      !face_blocked(p, 2, make_int3(ix, y, z + 1))) value -= cz * x[index + p.nx * p.ny];
  return value;
}

__global__ void pcg_initialize(const Params p, const int *fluid,
                               const int *active, const int *active_count, const float *rhs,
                               const float *diagonal, const float *pressure, float *r,
                               float *direction, double *scalars)
{
  for (int slot = blockIdx.x * blockDim.x + threadIdx.x; slot < *active_count;
       slot += blockDim.x * gridDim.x)
  {
    const int index = active[slot];
    const float residual = rhs[index] - pressure_matrix_row(p, fluid, diagonal, pressure, index);
    const float z = residual / diagonal[index];
    r[index] = residual;
    direction[index] = z;
    atomicAdd(scalars + 0, double(residual) * double(z));
    atomicAdd(scalars + 2, double(residual) * double(residual));
    atomicAdd(scalars + 4, double(rhs[index]) * double(rhs[index]));
  }
}

__global__ void pcg_matrix_dot(const Params p, const int *fluid,
                               const int *active, const int *active_count,
                               const float *diagonal, const float *direction, float *q,
                               double *denominator)
{
  for (int slot = blockIdx.x * blockDim.x + threadIdx.x; slot < *active_count;
       slot += blockDim.x * gridDim.x)
  {
    const int index = active[slot];
    const float value = pressure_matrix_row(p, fluid, diagonal, direction, index);
    q[index] = value;
    atomicAdd(denominator, double(direction[index]) * double(value));
  }
}

__global__ void pcg_update(const Params p, const int *active, const int *active_count,
                           const float *diagonal, float *pressure, float *r,
                           const float *direction, const float *q, const double *rz,
                           const double *denominator, double *next_rz, double *next_residual)
{
  const double den = *denominator;
  if (!isfinite(den) || fabs(den) < 1.0e-30) return;
  const float alpha = float(*rz / den);
  for (int slot = blockIdx.x * blockDim.x + threadIdx.x; slot < *active_count;
       slot += blockDim.x * gridDim.x)
  {
    const int index = active[slot];
    pressure[index] += alpha * direction[index];
    const float residual = r[index] - alpha * q[index];
    r[index] = residual;
    const float z = residual / diagonal[index];
    atomicAdd(next_rz, double(residual) * double(z));
    atomicAdd(next_residual, double(residual) * double(residual));
  }
}

__global__ void pcg_direction_update(const Params p, const int *active, const int *active_count,
                                     const float *diagonal, const float *r,
                                     float *direction, const double *rz,
                                     const double *next_rz)
{
  const double current = *rz;
  const bool valid = isfinite(current) && fabs(current) >= 1.0e-30;
  const float beta = valid ? float(*next_rz / current) : 0.0f;
  for (int slot = blockIdx.x * blockDim.x + threadIdx.x; slot < *active_count;
       slot += blockDim.x * gridDim.x)
  {
    const int index = active[slot];
    direction[index] = valid ? r[index] / diagonal[index] + beta * direction[index] : 0.0f;
  }
}

/* Sum of a value over the block, valid in thread 0. */
__device__ double block_sum(double value)
{
  __shared__ double warp_sums[32];
  for (int offset = 16; offset > 0; offset >>= 1) {
    value += __shfl_down_sync(0xffffffff, value, offset);
  }
  const int lane = threadIdx.x & 31;
  const int warp = threadIdx.x >> 5;
  if (lane == 0) warp_sums[warp] = value;
  __syncthreads();
  value = 0.0;
  if (warp == 0) {
    value = lane < (blockDim.x + 31) / 32 ? warp_sums[lane] : 0.0;
    for (int offset = 16; offset > 0; offset >>= 1) {
      value += __shfl_down_sync(0xffffffff, value, offset);
    }
  }
  __syncthreads();
  return value;
}

/* Per-solve accumulators of the persistent solver. Reductions are double-buffered by iteration
 * parity so that resetting one never races with adding to the other. Cleared before each solve. */
struct PersistentPcgState {
  double initial[4];
  double reduction[2][3];
  int separated;
};

/* Totals over all solves of one frame, read back once at the end. */
struct PersistentPcgTotals {
  int iterations;
  int nonfinite;
};

/* Turns cells in tension that touch a wall into air; returns whether any cell was released.
 * Cells are marked first (-1 still reads as liquid) and released after a sync, so the result does
 * not depend on thread order. Called by all threads of the cooperative grid. */
__device__ bool release_wall_tension(const Params p, int *fluid, const int *active,
                                     const int count, const int first, const int stride,
                                     float *pressure, PersistentPcgState *state,
                                     cooperative_groups::grid_group &grid)
{
  /* Mark wall cells in tension (-1 still reads as liquid for the neighbors), then release
   * them after a sync, so the result does not depend on thread order. */
  int separated = 0;
  for (int slot = first; slot < count; slot += stride) {
    const int index = active[slot];
    if (!fluid[index] || !(pressure[index] < 0.0f)) continue;
    const int z = index / (p.nx * p.ny);
    const int rest = index - z * p.nx * p.ny;
    const int y = rest / p.nx;
    const int x = rest % p.nx;
    const bool touches_wall =
        solid_at(p, x - 1, y, z) || solid_at(p, x + 1, y, z) || solid_at(p, x, y - 1, z) ||
        solid_at(p, x, y + 1, z) || solid_at(p, x, y, z - 1) || solid_at(p, x, y, z + 1) ||
        face_blocked(p, 0, make_int3(x, y, z)) || face_blocked(p, 0, make_int3(x + 1, y, z)) ||
        face_blocked(p, 1, make_int3(x, y, z)) || face_blocked(p, 1, make_int3(x, y + 1, z)) ||
        face_blocked(p, 2, make_int3(x, y, z)) || face_blocked(p, 2, make_int3(x, y, z + 1));
    if (touches_wall) {
      fluid[index] = -1;
      separated++;
    }
  }
  separated = int(block_sum(double(separated)));
  if (threadIdx.x == 0 && separated > 0) atomicAdd(&state->separated, separated);
  grid.sync();
  if (state->separated == 0) return false;
  for (int slot = first; slot < count; slot += stride) {
    const int index = active[slot];
    if (fluid[index] == -1) {
      fluid[index] = 0;
      pressure[index] = 0.0f;
    }
  }
  return true;
}

/* Bits of the six neighbors (-x, +x, -y, +y, -z, +z) that are liquid and connected through an
 * open face, i.e. the off-diagonal entries of #pressure_matrix_row. */
__device__ unsigned char pressure_neighbor_mask(const Params p, const int *fluid, const int index)
{
  const int z = index / (p.nx * p.ny);
  const int rest = index - z * p.nx * p.ny;
  const int y = rest / p.nx;
  const int x = rest % p.nx;
  unsigned char mask = 0;
  if (fluid_at(p, fluid, x - 1, y, z) && !face_blocked(p, 0, make_int3(x, y, z))) mask |= 1;
  if (fluid_at(p, fluid, x + 1, y, z) && !face_blocked(p, 0, make_int3(x + 1, y, z))) mask |= 2;
  if (fluid_at(p, fluid, x, y - 1, z) && !face_blocked(p, 1, make_int3(x, y, z))) mask |= 4;
  if (fluid_at(p, fluid, x, y + 1, z) && !face_blocked(p, 1, make_int3(x, y + 1, z))) mask |= 8;
  if (fluid_at(p, fluid, x, y, z - 1) && !face_blocked(p, 2, make_int3(x, y, z))) mask |= 16;
  if (fluid_at(p, fluid, x, y, z + 1) && !face_blocked(p, 2, make_int3(x, y, z + 1))) mask |= 32;
  return mask;
}

/* Same value and summation order as #pressure_matrix_row, from a precomputed neighbor mask. */
__device__ float pressure_matrix_row_masked(const Params p, const float *diagonal, const float *x,
                                            const int index, const unsigned char mask)
{
  const float cx = p.inverse_dx.x * p.inverse_dx.x;
  const float cy = p.inverse_dx.y * p.inverse_dx.y;
  const float cz = p.inverse_dx.z * p.inverse_dx.z;
  const int sy = p.nx, sz = p.nx * p.ny;
  float value = diagonal[index] * x[index];
  if (mask & 1) value -= cx * x[index - 1];
  if (mask & 2) value -= cx * x[index + 1];
  if (mask & 4) value -= cy * x[index - sy];
  if (mask & 8) value -= cy * x[index + sy];
  if (mask & 16) value -= cz * x[index - sz];
  if (mask & 32) value -= cz * x[index + sz];
  return value;
}

/* Jacobi-preconditioned CG in one cooperative launch: the residual of the warm start, the
 * convergence threshold, the iterations and the convergence test all run on the device, so the
 * host never waits for the solver. Neighbor connectivity is computed once per pass.
 * (A single-reduction variant with two instead of three grid syncs per iteration was measured
 * slower: the iteration cost is dominated by memory latency, not by the synchronizations.)
 *
 * Liquid can leave solid walls: the no-penetration condition at a wall only holds as an
 * inequality, so a wall cannot pull liquid back with tension. After a solve, cells with negative
 * pressure (tension) that touch a solid, a closed collider face or the domain boundary become
 * air (p = 0) and the system is solved again, at most `max_passes` times. Interfaces with air
 * need nothing extra, their pressure is already zero. */
__global__ void pcg_persistent(const Params p, int *fluid, const int *active,
                               const int *active_count, const float *rhs, const float *diagonal,
                               float *pressure, float *r, unsigned char *masks, float *direction,
                               float *q, PersistentPcgState *state, PersistentPcgTotals *totals,
                               const double tolerance, const int max_iterations,
                               const int max_passes)
{
  namespace cg = cooperative_groups;
  cg::grid_group grid = cg::this_grid();
  const int count = *active_count;
  const int stride = blockDim.x * gridDim.x;
  const int first = blockIdx.x * blockDim.x + threadIdx.x;
  const bool leader = blockIdx.x == 0 && threadIdx.x == 0;
  for (int pass = 0; pass < max_passes; pass++) {
    if (leader) {
      for (int k = 0; k < 4; k++) state->initial[k] = 0.0;
      for (int k = 0; k < 3; k++) state->reduction[0][k] = state->reduction[1][k] = 0.0;
    }
    grid.sync();
    {
      double rz = 0.0, rr = 0.0, bb = 0.0;
      for (int slot = first; slot < count; slot += stride) {
        const int index = active[slot];
        if (!fluid[index]) continue;
        /* Each slot stays with the same thread in every loop, so no sync is needed. */
        const unsigned char mask = pressure_neighbor_mask(p, fluid, index);
        masks[slot] = mask;
        const float residual = rhs[index] -
                               pressure_matrix_row_masked(p, diagonal, pressure, index, mask);
        const float z = residual / diagonal[index];
        r[index] = residual;
        direction[index] = z;
        rz += double(residual) * double(z);
        rr += double(residual) * double(residual);
        bb += double(rhs[index]) * double(rhs[index]);
      }
      rz = block_sum(rz);
      rr = block_sum(rr);
      bb = block_sum(bb);
      if (threadIdx.x == 0) {
        atomicAdd(&state->initial[0], rz);
        atomicAdd(&state->initial[2], rr);
        atomicAdd(&state->initial[3], bb);
      }
    }
    grid.sync();
    double rz = state->initial[0];
    double residual = state->initial[2];
    const double target_squared = tolerance * tolerance * fmax(state->initial[3], 1.0e-40);
    int iteration = 0;
    if (state->initial[3] <= 1.0e-30) {
      for (int slot = first; slot < count; slot += stride) {
        const int index = active[slot];
        if (fluid[index]) pressure[index] = 0.0f;
      }
      residual = 0.0;
    }
    while (iteration < max_iterations && residual > target_squared) {
      const int parity = iteration & 1;
      if (leader) {
        state->reduction[parity][1] = 0.0;
        state->reduction[parity][2] = 0.0;
      }
      double partial = 0.0;
      for (int slot = first; slot < count; slot += stride) {
        const int index = active[slot];
        if (!fluid[index]) continue;
        const float value = pressure_matrix_row_masked(p, diagonal, direction, index,
                                                       masks[slot]);
        q[index] = value;
        partial += double(direction[index]) * double(value);
      }
      partial = block_sum(partial);
      if (threadIdx.x == 0) atomicAdd(&state->reduction[parity][0], partial);
      grid.sync();
      const double denominator = state->reduction[parity][0];
      if (!isfinite(denominator) || fabs(denominator) < 1.0e-30) break;
      if (leader) state->reduction[1 - parity][0] = 0.0;
      const float alpha = float(rz / denominator);
      double partial_rz = 0.0, partial_residual = 0.0;
      for (int slot = first; slot < count; slot += stride) {
        const int index = active[slot];
        if (!fluid[index]) continue;
        pressure[index] += alpha * direction[index];
        const float value = r[index] - alpha * q[index];
        r[index] = value;
        const float z = value / diagonal[index];
        partial_rz += double(value) * double(z);
        partial_residual += double(value) * double(value);
      }
      partial_rz = block_sum(partial_rz);
      partial_residual = block_sum(partial_residual);
      if (threadIdx.x == 0) {
        atomicAdd(&state->reduction[parity][1], partial_rz);
        atomicAdd(&state->reduction[parity][2], partial_residual);
      }
      grid.sync();
      const double next_rz = state->reduction[parity][1];
      residual = state->reduction[parity][2];
      const bool valid = isfinite(rz) && fabs(rz) >= 1.0e-30;
      const float beta = valid ? float(next_rz / rz) : 0.0f;
      for (int slot = first; slot < count; slot += stride) {
        const int index = active[slot];
        if (!fluid[index]) continue;
        direction[index] = valid ? r[index] / diagonal[index] + beta * direction[index] : 0.0f;
      }
      rz = next_rz;
      iteration++;
      grid.sync();
    }
    if (leader) {
      totals->iterations += iteration;
      if (!isfinite(residual) || !isfinite(state->initial[3])) totals->nonfinite = 1;
      state->separated = 0;
    }
    if (pass + 1 == max_passes) break;
    grid.sync();
    if (!release_wall_tension(p, fluid, active, count, first, stride, pressure, state, grid)) {
      break;
    }
  }
}

__global__ void measure_pressure(const Params p, const int *fluid,
                                 const float *rhs, const float *diagonal,
                                 const float *pressure, double *residual_squared,
                                 double *rhs_squared, int *fluid_cells)
{
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index >= p.nx * p.ny * p.nz || !fluid[index]) return;
  const int z = index / (p.nx * p.ny);
  const int left = index - z * p.nx * p.ny;
  const int y = left / p.nx;
  const int x = left % p.nx;
  const float cx = p.inverse_dx.x * p.inverse_dx.x;
  const float cy = p.inverse_dx.y * p.inverse_dx.y;
  const float cz = p.inverse_dx.z * p.inverse_dx.z;
  float ap = diagonal[index] * pressure[index];
  if (fluid_at(p, fluid, x - 1, y, z) &&
      !face_blocked(p, 0, make_int3(x, y, z))) ap -= cx * pressure[index - 1];
  if (fluid_at(p, fluid, x + 1, y, z) &&
      !face_blocked(p, 0, make_int3(x + 1, y, z))) ap -= cx * pressure[index + 1];
  if (fluid_at(p, fluid, x, y - 1, z) &&
      !face_blocked(p, 1, make_int3(x, y, z))) ap -= cy * pressure[index - p.nx];
  if (fluid_at(p, fluid, x, y + 1, z) &&
      !face_blocked(p, 1, make_int3(x, y + 1, z))) ap -= cy * pressure[index + p.nx];
  if (fluid_at(p, fluid, x, y, z - 1) &&
      !face_blocked(p, 2, make_int3(x, y, z))) ap -= cz * pressure[index - p.nx * p.ny];
  if (fluid_at(p, fluid, x, y, z + 1) &&
      !face_blocked(p, 2, make_int3(x, y, z + 1))) ap -= cz * pressure[index + p.nx * p.ny];
  const double r = double(rhs[index]) - double(ap);
  atomicAdd(residual_squared, r * r);
  atomicAdd(rhs_squared, double(rhs[index]) * double(rhs[index]));
  atomicAdd(fluid_cells, 1);
}

__global__ void project_faces(const Params p, const int *fluid,
                              const float *pressure, const int axis, float *values)
{
  GRID_LOOP_BEGIN(p, face_size(p, axis), index, c)
  if (face_blocked(p, axis, c)) continue;
  if ((axis == 0 && (c.x == 0 || c.x == p.nx)) ||
      (axis == 1 && (c.y == 0 || c.y == p.ny)) ||
      (axis == 2 && (c.z == 0 || c.z == p.nz))) {
    values[index] = 0.0f;
    continue;
  }
  if (!touches_fluid(p, fluid, axis, c)) continue;
  const int3 lower = make_int3(c.x - (axis == 0), c.y - (axis == 1), c.z - (axis == 2));
  const float hi = fluid_at(p, fluid, c.x, c.y, c.z) ?
                   pressure[cell_index(p, c.x, c.y, c.z)] : 0.0f;
  const float lo = fluid_at(p, fluid, lower.x, lower.y, lower.z) ?
                   pressure[cell_index(p, lower.x, lower.y, lower.z)] : 0.0f;
  values[index] -= p.dt / p.density * axis_value(p.inverse_dx, axis) * (hi - lo);
  GRID_LOOP_END
}

__device__ float sample_face(const Params p, const float *values, const float3 grid,
                             const int axis)
{
  const int3 size = face_size(p, axis);
  const float sx = fminf(size.x - 1.0f, fmaxf(0.0f, grid.x - (axis != 0 ? 0.5f : 0.0f)));
  const float sy = fminf(size.y - 1.0f, fmaxf(0.0f, grid.y - (axis != 1 ? 0.5f : 0.0f)));
  const float sz = fminf(size.z - 1.0f, fmaxf(0.0f, grid.z - (axis != 2 ? 0.5f : 0.0f)));
  const int x0 = int(floorf(sx)), y0 = int(floorf(sy)), z0 = int(floorf(sz));
  const int x1 = min(x0 + 1, size.x - 1), y1 = min(y0 + 1, size.y - 1);
  const int z1 = min(z0 + 1, size.z - 1);
  const float fx = sx - x0, fy = sy - y0, fz = sz - z0;
  float sum = 0.0f;
  for (int oz = 0; oz < 2; oz++) {
    const int z = oz ? z1 : z0;
    const float wz = oz ? fz : 1.0f - fz;
    for (int oy = 0; oy < 2; oy++) {
      const int y = oy ? y1 : y0;
      const float wy = oy ? fy : 1.0f - fy;
      for (int ox = 0; ox < 2; ox++) {
        const int x = ox ? x1 : x0;
        const float wx = ox ? fx : 1.0f - fx;
        sum += values[face_index(size, x, y, z)] * wx * wy * wz;
      }
    }
  }
  return sum;
}

__device__ float3 sample_grid(const Params p, const float3 grid, const float *u,
                              const float *v, const float *w)
{
  return make_float3(sample_face(p, u, grid, 0), sample_face(p, v, grid, 1),
                     sample_face(p, w, grid, 2));
}

/* Trilinear collider signed distance from the cell-center samples. The CPU only fills a narrow
 * band around the collider; outside of it the samples are infinite, which reads as "far". */
__device__ float solid_phi_at(const Params p, const float3 position)
{
  const float3 g = grid_position(p, position);
  const float sx = fminf(p.nx - 1.0f, fmaxf(0.0f, g.x - 0.5f));
  const float sy = fminf(p.ny - 1.0f, fmaxf(0.0f, g.y - 0.5f));
  const float sz = fminf(p.nz - 1.0f, fmaxf(0.0f, g.z - 0.5f));
  const int x0 = int(floorf(sx)), y0 = int(floorf(sy)), z0 = int(floorf(sz));
  const int x1 = min(x0 + 1, p.nx - 1), y1 = min(y0 + 1, p.ny - 1), z1 = min(z0 + 1, p.nz - 1);
  const float fx = sx - x0, fy = sy - y0, fz = sz - z0;
  float sum = 0.0f;
  for (int oz = 0; oz < 2; oz++) {
    const float wz = oz ? fz : 1.0f - fz;
    const int z = oz ? z1 : z0;
    for (int oy = 0; oy < 2; oy++) {
      const float wy = oy ? fy : 1.0f - fy;
      const int y = oy ? y1 : y0;
      for (int ox = 0; ox < 2; ox++) {
        const int x = ox ? x1 : x0;
        const int index = cell_index(p, x, y, z);
        float phi = p.solid_cell_phi[index];
        if (!isfinite(phi)) return __int_as_float(0x7f800000); /* +inf: far from the collider. */
        if (p.solid_cell_normal) {
          /* Distance to the tangent plane at the sample's closest point. Exact for flat parts of
           * the collider, and much closer than plain interpolation where it curves. */
          const float3 n = p.solid_cell_normal[index];
          phi += n.x * (position.x - (p.origin.x + (x + 0.5f) * p.dx.x)) +
                 n.y * (position.y - (p.origin.y + (y + 0.5f) * p.dx.y)) +
                 n.z * (position.z - (p.origin.z + (z + 0.5f) * p.dx.z));
        }
        sum += phi * (ox ? fx : 1.0f - fx) * wy * wz;
      }
    }
  }
  return sum;
}

/* Interpolated exact normal, when available. Returns false where it is degenerate. */
__device__ bool solid_sample_normal(const Params p, const float3 position, float3 &r_normal)
{
  if (!p.solid_cell_normal) return false;
  const float3 g = grid_position(p, position);
  const float sx = fminf(p.nx - 1.0f, fmaxf(0.0f, g.x - 0.5f));
  const float sy = fminf(p.ny - 1.0f, fmaxf(0.0f, g.y - 0.5f));
  const float sz = fminf(p.nz - 1.0f, fmaxf(0.0f, g.z - 0.5f));
  const int x0 = int(floorf(sx)), y0 = int(floorf(sy)), z0 = int(floorf(sz));
  const int x1 = min(x0 + 1, p.nx - 1), y1 = min(y0 + 1, p.ny - 1), z1 = min(z0 + 1, p.nz - 1);
  const float fx = sx - x0, fy = sy - y0, fz = sz - z0;
  float3 sum = make_float3(0.0f, 0.0f, 0.0f);
  for (int oz = 0; oz < 2; oz++) {
    const float wz = oz ? fz : 1.0f - fz;
    for (int oy = 0; oy < 2; oy++) {
      const float wy = oy ? fy : 1.0f - fy;
      for (int ox = 0; ox < 2; ox++) {
        const float w = (ox ? fx : 1.0f - fx) * wy * wz;
        const float3 n = p.solid_cell_normal[cell_index(p, ox ? x1 : x0, oy ? y1 : y0,
                                                        oz ? z1 : z0)];
        sum = make_float3(sum.x + w * n.x, sum.y + w * n.y, sum.z + w * n.z);
      }
    }
  }
  const float length = sqrtf(sum.x * sum.x + sum.y * sum.y + sum.z * sum.z);
  if (length < 0.5f) return false;
  r_normal = make_float3(sum.x / length, sum.y / length, sum.z / length);
  return true;
}

__device__ float solid_phi_derivative(const Params p, const float3 position, const float center,
                                      const float3 offset, const float h)
{
  const float plus = solid_phi_at(
      p, make_float3(position.x + offset.x, position.y + offset.y, position.z + offset.z));
  const float minus = solid_phi_at(
      p, make_float3(position.x - offset.x, position.y - offset.y, position.z - offset.z));
  if (isfinite(plus) && isfinite(minus)) return (plus - minus) / (2.0f * h);
  if (isfinite(plus)) return (plus - center) / h;
  if (isfinite(minus)) return (center - minus) / h;
  return 0.0f;
}

__device__ float3 solid_normal_at(const Params p, const float3 position)
{
  float3 exact;
  if (solid_sample_normal(p, position, exact)) return exact;
  const float hx = 0.5f * p.dx.x, hy = 0.5f * p.dx.y, hz = 0.5f * p.dx.z;
  const float center = solid_phi_at(p, position);
  const float3 n = make_float3(
      solid_phi_derivative(p, position, center, make_float3(hx, 0.0f, 0.0f), hx),
      solid_phi_derivative(p, position, center, make_float3(0.0f, hy, 0.0f), hy),
      solid_phi_derivative(p, position, center, make_float3(0.0f, 0.0f, hz), hz));
  const float length = sqrtf(n.x * n.x + n.y * n.y + n.z * n.z);
  if (length < 1.0e-12f) return make_float3(0.0f, 0.0f, 1.0f);
  return make_float3(n.x / length, n.y / length, n.z / length);
}

__device__ float3 lerp_position(const float3 a, const float3 b, const float t)
{
  return make_float3(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t);
}

/* Mirrors #resolve_flip_particle_collision on the CPU, with the exact mesh query replaced by the
 * collider SDF: find the first contact along the substep segment, push the particle out to the
 * margin, remove the inward relative velocity and apply friction to the tangential part. */
__device__ void collide_particle(const Params p, const float3 old_position, float3 &position,
                                 float3 &velocity)
{
  const float margin = p.collision_margin;
  float phi = solid_phi_at(p, position);
  if (!(phi < margin)) {
    /* Catch thin walls crossed within one substep. */
    const float3 middle = lerp_position(old_position, position, 0.5f);
    if (!(solid_phi_at(p, middle) < margin)) return;
    position = middle;
    phi = solid_phi_at(p, position);
  }
  if (solid_phi_at(p, old_position) > margin) {
    const float3 end = position;
    float low = 0.0f, high = 1.0f;
    for (int iteration = 0; iteration < 8; iteration++) {
      const float middle = 0.5f * (low + high);
      if (solid_phi_at(p, lerp_position(old_position, end, middle)) < margin) high = middle;
      else low = middle;
    }
    position = lerp_position(old_position, end, high);
    phi = solid_phi_at(p, position);
  }
  if (!(phi < margin)) return;
  const float3 n = solid_normal_at(p, position);
  const float push = margin - phi + fmaxf(1.0e-6f, margin * 0.01f);
  position = make_float3(position.x + push * n.x, position.y + push * n.y,
                         position.z + push * n.z);
  const float3 g = grid_position(p, position);
  const float3 solid = make_float3(sample_face(p, p.solid_velocity[0], g, 0),
                                   sample_face(p, p.solid_velocity[1], g, 1),
                                   sample_face(p, p.solid_velocity[2], g, 2));
  float3 relative = make_float3(velocity.x - solid.x, velocity.y - solid.y,
                                velocity.z - solid.z);
  const float inward = relative.x * n.x + relative.y * n.y + relative.z * n.z;
  if (inward < 0.0f) {
    relative = make_float3(relative.x - inward * n.x, relative.y - inward * n.y,
                           relative.z - inward * n.z);
  }
  const float normal_speed = fmaxf(relative.x * n.x + relative.y * n.y + relative.z * n.z, 0.0f);
  const float keep = 1.0f - p.collision_friction;
  velocity = make_float3(solid.x + normal_speed * n.x + (relative.x - normal_speed * n.x) * keep,
                         solid.y + normal_speed * n.y + (relative.y - normal_speed * n.y) * keep,
                         solid.z + normal_speed * n.z + (relative.z - normal_speed * n.z) * keep);
}

/* -------------------------------------------------------------------------------------------
 * Reseeding with the rules of the CPU solver (#FlipSolverCore::reseed_and_cull_particles):
 * cull crowded cells by hash order, seed sparse interior fluid cells from the nearest particle,
 * then keep the particle budget by removing hash-ordered particles from cells above the minimum.
 * Arithmetic that decides cells and positions uses round-to-nearest intrinsics to avoid FMA
 * contraction, so it follows the CPU's float operations. */


/* Order by (hash, id, index). IDs can repeat (e.g. emitted particles), so like the CPU the
 * particle index breaks remaining ties. */
__device__ bool hash_order_less(const long long a,
                                const int index_a,
                                const long long b,
                                const int index_b,
                                const uint32_t salt)
{
  const uint32_t ha = cpu_hash_u32(uint32_t(a) ^ salt);
  const uint32_t hb = cpu_hash_u32(uint32_t(b) ^ salt);
  if (ha != hb) return ha < hb;
  if (a != b) return a < b;
  return index_a < index_b;
}

__global__ void reseed_sanitize(const Params p, const int n, float3 *positions, float3 *velocities)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  float3 position = positions[i];
  float3 velocity = velocities[i];
  const float3 center = make_float3(__fmul_rn(__fadd_rn(p.origin.x, p.domain_max.x), 0.5f),
                                    __fmul_rn(__fadd_rn(p.origin.y, p.domain_max.y), 0.5f),
                                    __fmul_rn(__fadd_rn(p.origin.z, p.domain_max.z), 0.5f));
  if (!isfinite(position.x) || !isfinite(position.y) || !isfinite(position.z)) position = center;
  if (!isfinite(velocity.x) || !isfinite(velocity.y) || !isfinite(velocity.z)) {
    velocity = make_float3(0.0f, 0.0f, 0.0f);
  }
  const float lower[3] = {__fadd_rn(p.origin.x, p.padding), __fadd_rn(p.origin.y, p.padding),
                          __fadd_rn(p.origin.z, p.padding)};
  const float upper[3] = {__fsub_rn(p.domain_max.x, p.padding),
                          __fsub_rn(p.domain_max.y, p.padding),
                          __fsub_rn(p.domain_max.z, p.padding)};
  float *pc[3] = {&position.x, &position.y, &position.z};
  float *vc[3] = {&velocity.x, &velocity.y, &velocity.z};
  for (int axis = 0; axis < 3; axis++) {
    if (*pc[axis] < lower[axis]) {
      *pc[axis] = lower[axis];
      if (*vc[axis] < 0.0f) *vc[axis] = 0.0f;
    }
    else if (*pc[axis] > upper[axis]) {
      *pc[axis] = upper[axis];
      if (*vc[axis] > 0.0f) *vc[axis] = 0.0f;
    }
  }
  positions[i] = position;
  velocities[i] = velocity;
}

__global__ void reseed_bin(const Params p, const int n, const float3 *positions, int *keys,
                           int *index, int *counts, const long long *ids,
                           unsigned long long *max_id)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const int3 c = cpu_cell(p, positions[i]);
  const int cell = cell_index(p, c.x, c.y, c.z);
  keys[i] = cell;
  index[i] = i;
  atomicAdd(counts + cell, 1);
  atomicMax(max_id, static_cast<unsigned long long>(max(ids[i], 0LL)));
}

__global__ void reseed_cull(const int n, const int *keys, const long long *ids,
                            const int *counts, const int *offsets, const int *sorted,
                            const int max_count, const int target, unsigned char *keep,
                            int *kept_in_cell, int *culled)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const int cell = keys[i];
  bool kept = true;
  if (counts[cell] > max_count) {
    int rank = 0;
    for (int o = offsets[cell]; o < offsets[cell + 1]; o++) {
      const int j = sorted[o];
      if (j != i && hash_order_less(ids[j], j, ids[i], i, uint32_t(cell))) rank++;
    }
    kept = rank < target;
  }
  keep[i] = kept;
  if (kept) atomicAdd(kept_in_cell + cell, 1);
  else atomicAdd(culled, 1);
}

__global__ void reseed_candidates(const Params p, const int *fluid, const int *counts,
                                  const int *offsets, const int *sorted,
                                  const float3 *positions, const float3 *velocities,
                                  const int min_count, const int target, int *candidate_count,
                                  int *parent, float3 *average_velocity)
{
  GRID_LOOP_BEGIN(p, make_int3(p.nx, p.ny, p.nz), cell, c)
  candidate_count[cell] = 0;
  parent[cell] = -1;
  const int count = counts[cell];
  if (!fluid[cell] || count >= min_count) continue;
  if (count == 0) {
    /* Empty cells at the liquid surface are not refilled. */
    if (!fluid_at(p, fluid, c.x - 1, c.y, c.z) || !fluid_at(p, fluid, c.x + 1, c.y, c.z) ||
        !fluid_at(p, fluid, c.x, c.y - 1, c.z) || !fluid_at(p, fluid, c.x, c.y + 1, c.z) ||
        !fluid_at(p, fluid, c.x, c.y, c.z - 1) || !fluid_at(p, fluid, c.x, c.y, c.z + 1))
    {
      continue;
    }
  }
  /* #nearest_source_particle: growing cubes, first strictly closer particle wins. */
  const float3 center = cpu_cell_center(p, c.x, c.y, c.z);
  int best = -1;
  float best_distance = __int_as_float(0x7f800000);
  for (int radius = 0; radius <= 2 && best < 0; radius++) {
    for (int k = max(0, c.z - radius); k <= min(p.nz - 1, c.z + radius); k++) {
      for (int j = max(0, c.y - radius); j <= min(p.ny - 1, c.y + radius); j++) {
        for (int ii = max(0, c.x - radius); ii <= min(p.nx - 1, c.x + radius); ii++) {
          const int neighbor = cell_index(p, ii, j, k);
          for (int o = offsets[neighbor]; o < offsets[neighbor + 1]; o++) {
            const int particle = sorted[o];
            const float3 q = positions[particle];
            const float dx = __fsub_rn(q.x, center.x);
            const float dy = __fsub_rn(q.y, center.y);
            const float dz = __fsub_rn(q.z, center.z);
            const float d = __fadd_rn(__fadd_rn(__fmul_rn(dx, dx), __fmul_rn(dy, dy)),
                                      __fmul_rn(dz, dz));
            if (d < best_distance) {
              best_distance = d;
              best = particle;
            }
          }
        }
      }
    }
  }
  if (best < 0) continue;
  parent[cell] = best;
  candidate_count[cell] = max(0, target - count);
  float3 sum = make_float3(0.0f, 0.0f, 0.0f);
  int samples = 0;
  for (int o = offsets[cell]; o < offsets[cell + 1]; o++) {
    const float3 v = velocities[sorted[o]];
    sum = make_float3(__fadd_rn(sum.x, v.x), __fadd_rn(sum.y, v.y), __fadd_rn(sum.z, v.z));
    samples++;
  }
  average_velocity[cell] = samples > 0 ?
                               make_float3(__fdiv_rn(sum.x, float(samples)),
                                           __fdiv_rn(sum.y, float(samples)),
                                           __fdiv_rn(sum.z, float(samples))) :
                               velocities[best];
  GRID_LOOP_END
}

__global__ void reseed_fill_candidates(const Params p, const int *candidate_count,
                                       const int *candidate_offsets, const int *counts,
                                       int *candidate_cell, int *candidate_slot)
{
  GRID_LOOP_BEGIN(p, make_int3(p.nx, p.ny, p.nz), cell, c)
  const int start = candidate_offsets[cell];
  for (int i = 0; i < candidate_count[cell]; i++) {
    candidate_cell[start + i] = cell;
    candidate_slot[start + i] = counts[cell] + i;
  }
  GRID_LOOP_END
}

/* Removal order that keeps the particle budget, equal to the CPU's staged selection:
 * stage 0: removable particles (a cell can give up its count above the minimum, candidates
 *          before retained particles, each in cell hash order),
 * stage 1: the remaining candidates, stage 2: the remaining retained particles.
 * Within a stage particles are ordered by (hash(id), id); the particle index breaks ties because
 * the list is first sorted by index. Removing the first `excess` entries of this order removes
 * the same particles as the CPU. */
__device__ unsigned long long budget_order_key(const int stage, const long long id)
{
  const uint32_t h = cpu_hash_u32(uint32_t(id));
  return (static_cast<unsigned long long>(stage) << 62) |
         (static_cast<unsigned long long>(h) << 30) |
         (static_cast<unsigned long long>(uint32_t(id)) & 0x3fffffffULL);
}

__global__ void reseed_order(const int n, const int candidate_total, const int *keys,
                             const unsigned char *keep, const long long *ids,
                             const int *kept_in_cell, const int *candidate_count,
                             const int *offsets, const int *sorted, const int *candidate_cell,
                             const int *candidate_offsets, const unsigned long long *max_id,
                             const int min_count, const bool keep_budget,
                             unsigned long long *list_keys, int *list_values, int *flags)
{
  const int t = blockIdx.x * blockDim.x + threadIdx.x;
  if (t > n + candidate_total) return;
  if (t == n + candidate_total) {
    flags[t] = 0;
    return;
  }
  if (t < n) {
    const int i = t;
    flags[t] = keep[i];
    if (!keep_budget) return;
    list_values[t] = t;
    if (!keep[i]) {
      list_keys[t] = ~0ULL;
      return;
    }
    const int cell = keys[i];
    const int removable = max(0, kept_in_cell[cell] + candidate_count[cell] - min_count);
    const int retained_slots = removable - candidate_count[cell];
    int stage = 2;
    if (retained_slots > 0) {
      int rank = 0;
      for (int o = offsets[cell]; o < offsets[cell + 1]; o++) {
        const int j = sorted[o];
        if (j != i && keep[j] && hash_order_less(ids[j], j, ids[i], i, uint32_t(cell))) rank++;
      }
      if (rank < retained_slots) stage = 0;
    }
    list_keys[t] = budget_order_key(stage, ids[i]);
    return;
  }
  const int g = t - n;
  flags[t] = 1;
  if (!keep_budget) return;
  list_values[t] = t;
  const int cell = candidate_cell[g];
  const int removable = max(0, kept_in_cell[cell] + candidate_count[cell] - min_count);
  const long long next_id = static_cast<long long>(*max_id) + 1;
  const long long id = next_id + g;
  int stage = 1;
  if (removable > 0) {
    int rank = 0;
    const int start = candidate_offsets[cell];
    for (int other = start; other < start + candidate_count[cell]; other++) {
      if (other != g && hash_order_less(next_id + other, other, id, g, uint32_t(cell))) rank++;
    }
    if (rank < removable) stage = 0;
  }
  list_keys[t] = budget_order_key(stage, id);
}

__global__ void reseed_unflag(const int count, const int *values, int *flags)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < count) flags[values[i]] = 0;
}

/* Writes survivors: retained particles in their order, then candidates in their order. One
 * exclusive scan over both parts gives the output index directly. */
__global__ void reseed_write(const Params p, const int n, const int candidate_total,
                             const int *flags, const int *scan, const float3 *positions,
                             const float3 *velocities, const long long *ids, const int *sources,
                             const int *candidate_cell, const int *candidate_slot,
                             const int *parent, const float3 *average_velocity,
                             const unsigned long long *max_id, const int retained,
                             const int *culled, int *totals, float3 *out_positions,
                             float3 *out_velocities, long long *out_ids, int *out_sources)
{
  const int t = blockIdx.x * blockDim.x + threadIdx.x;
  if (t == 0) {
    /* Statistics stay on the device until the end of the frame. */
    const int retained_total = scan[n];
    totals[0] += scan[n + candidate_total] - retained_total;
    totals[1] += *culled + (retained - retained_total);
  }
  if (t >= n + candidate_total || !flags[t]) return;
  const int o = scan[t];
  if (t < n) {
    out_positions[o] = positions[t];
    out_velocities[o] = velocities[t];
    out_ids[o] = ids[t];
    out_sources[o] = sources[t];
    return;
  }
  const int g = t - n;
  const int cell = candidate_cell[g];
  const int slot = candidate_slot[g];
  const int z = cell / (p.nx * p.ny);
  const int rest = cell - z * p.nx * p.ny;
  const int y = rest / p.nx;
  const int x = rest % p.nx;
  const float3 cell_min = make_float3(__fadd_rn(p.origin.x, __fmul_rn(float(x), p.dx.x)),
                                      __fadd_rn(p.origin.y, __fmul_rn(float(y), p.dx.y)),
                                      __fadd_rn(p.origin.z, __fmul_rn(float(z), p.dx.z)));
  const float margin = 0.01f * fminf(p.dx.x, fminf(p.dx.y, p.dx.z));
  const int attempts = p.has_collider ? 8 : 1;
  float3 spawn = cpu_cell_center(p, x, y, z);
  for (int attempt = 0; attempt < attempts; attempt++) {
    const uint32_t seed = uint32_t(slot * 24 + attempt * 3);
    const float lx = __fadd_rn(0.1f, __fmul_rn(0.8f, cpu_hash_unit(uint32_t(cell), seed)));
    const float ly = __fadd_rn(0.1f, __fmul_rn(0.8f, cpu_hash_unit(uint32_t(cell), seed + 1)));
    const float lz = __fadd_rn(0.1f, __fmul_rn(0.8f, cpu_hash_unit(uint32_t(cell), seed + 2)));
    const float3 candidate = make_float3(__fadd_rn(cell_min.x, __fmul_rn(lx, p.dx.x)),
                                         __fadd_rn(cell_min.y, __fmul_rn(ly, p.dx.y)),
                                         __fadd_rn(cell_min.z, __fmul_rn(lz, p.dx.z)));
    if (!p.has_collider || solid_phi_at(p, candidate) > margin) {
      spawn = candidate;
      break;
    }
  }
  out_positions[o] = spawn;
  out_velocities[o] = average_velocity[cell];
  out_ids[o] = static_cast<long long>(*max_id) + 1 + g;
  out_sources[o] = sources[parent[cell]];
}






/* Pushes particles that start inside the collider back to its surface. */
__global__ void resolve_inside_collider(const Params p, const int n, float3 *positions,
                                        float3 *velocities)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  float3 position = positions[i];
  float3 velocity = velocities[i];
  collide_particle(p, position, position, velocity);
  positions[i] = position;
  velocities[i] = velocity;
}

__global__ void grid_to_particles(const Params p, float3 *positions, float3 *velocities,
                                  const float *u, const float *v, const float *w,
                                  const float *u_old, const float *v_old, const float *w_old)
{
  const int particle = blockIdx.x * blockDim.x + threadIdx.x;
  if (particle >= p.particles) return;
  float3 position = positions[particle];
  const float3 g = grid_position(p, position);
  const float3 pic = sample_grid(p, g, u, v, w);
  const float3 old = sample_grid(p, g, u_old, v_old, w_old);
  const float3 v0 = velocities[particle];
  float3 velocity = make_float3((1.0f - p.flip_ratio) * pic.x +
                                   p.flip_ratio * (v0.x + pic.x - old.x),
                               (1.0f - p.flip_ratio) * pic.y +
                                   p.flip_ratio * (v0.y + pic.y - old.y),
                               (1.0f - p.flip_ratio) * pic.z +
                                   p.flip_ratio * (v0.z + pic.z - old.z));
  const float3 midpoint = make_float3(position.x + 0.5f * p.dt * pic.x,
                                      position.y + 0.5f * p.dt * pic.y,
                                      position.z + 0.5f * p.dt * pic.z);
  const float3 advect = sample_grid(p, grid_position(p, midpoint), u, v, w);
  position.x += p.dt * advect.x;
  position.y += p.dt * advect.y;
  position.z += p.dt * advect.z;
  if (p.gpu_collision) {
    collide_particle(p, positions[particle], position, velocity);
  }
  const float3 lo = make_float3(p.origin.x + p.padding, p.origin.y + p.padding,
                                p.origin.z + p.padding);
  const float3 hi = make_float3(p.origin.x + p.nx * p.dx.x - p.padding,
                                p.origin.y + p.ny * p.dx.y - p.padding,
                                p.origin.z + p.nz * p.dx.z - p.padding);
  if (position.x < lo.x) { position.x = lo.x; velocity.x = fmaxf(velocity.x, 0.0f); }
  if (position.y < lo.y) { position.y = lo.y; velocity.y = fmaxf(velocity.y, 0.0f); }
  if (position.z < lo.z) { position.z = lo.z; velocity.z = fmaxf(velocity.z, 0.0f); }
  if (position.x > hi.x) { position.x = hi.x; velocity.x = fminf(velocity.x, 0.0f); }
  if (position.y > hi.y) { position.y = hi.y; velocity.y = fminf(velocity.y, 0.0f); }
  if (position.z > hi.z) { position.z = hi.z; velocity.z = fminf(velocity.z, 0.0f); }
  positions[particle] = position;
  velocities[particle] = velocity;
}

void set_error(char *error, const size_t error_size, const char *stage, const cudaError_t code)
{
  if (error && error_size) std::snprintf(error, error_size, "%s: %s", stage, cudaGetErrorString(code));
}

}  // namespace

extern "C" bool flip_solver_cuda_run(const float *input_positions,
                                     const float *input_velocities,
                                     const int64_t *input_ids,
                                     const int *input_sources,
                                     const int particle_count,
                                     const int nx,
                                     const int ny,
                                     const int nz,
                                     const float *domain_min,
                                     const float *domain_max,
                                     const float *gravity,
                                     const float density,
                                     const float flip_ratio,
                                     const float padding,
                                     const float classification_radius_scale,
                                     const int extrapolation_layers,
                                     const int pressure_iterations,
                                     const float pressure_tolerance,
                                     const float viscosity,
                                     const int viscosity_iterations,
                                     const float viscosity_tolerance,
                                     const bool reseeding,
                                     const int target_particles_per_cell,
                                     const int min_particles_per_cell,
                                     const int max_particles_per_cell,
                                     const float substep_dt,
                                     const int substeps,
                                     const bool has_collider,
                                     const bool gpu_particle_collision,
                                     const float collider_friction,
                                     const bool host_callback_every_step,
                                     const uint64_t static_collider_key,
                                     bool (*host_step_callback)(void *, int, bool,
                                                                FlipCudaColliderFields *),
                                     void *host_step_context,
                                     float *output_positions,
                                     float *output_velocities,
                                     int64_t *output_ids,
                                     int *output_sources,
                                     double *output_relative_residual,
                                     int *output_fluid_cells,
                                     int *output_pressure_iterations,
                                     int *output_viscosity_iterations,
                                     double *output_viscosity_residual,
                                     int *output_reseeded,
                                     int *output_culled,
                                     int *output_particle_count,
                                     char *error,
                                     const size_t error_size)
{
  cudaError_t status = cudaSetDevice(0);
  if (status != cudaSuccess) { set_error(error, error_size, "CUDA device", status); return false; }
  StepTimer timer;
  Params p{};
  p.nx = nx; p.ny = ny; p.nz = nz; p.particles = particle_count;
  p.origin = make_float3(domain_min[0], domain_min[1], domain_min[2]);
  p.domain_max = make_float3(domain_max[0], domain_max[1], domain_max[2]);
  p.dx = make_float3((domain_max[0] - domain_min[0]) / nx,
                     (domain_max[1] - domain_min[1]) / ny,
                     (domain_max[2] - domain_min[2]) / nz);
  p.inverse_dx = make_float3(1.0f / p.dx.x, 1.0f / p.dx.y, 1.0f / p.dx.z);
  p.gravity = make_float3(gravity[0], gravity[1], gravity[2]);
  p.dt = substep_dt; p.density = density; p.flip_ratio = flip_ratio;
  p.padding = fminf(padding, 0.49f * fminf(p.dx.x, fminf(p.dx.y, p.dx.z)));
  p.classification_radius = fminf(fmaxf(classification_radius_scale, 0.0f), 1.0f) *
                            fminf(p.dx.x, fminf(p.dx.y, p.dx.z));
  p.has_collider = has_collider;
  p.gpu_collision = has_collider && gpu_particle_collision;
  /* Larger than the CPU margin (1% of a cell) to absorb the remaining SDF interpolation error.
   * With tangent-plane distances, 5% of a cell keeps penetration at the CPU level. */
  p.collision_margin = 0.05f * fminf(p.dx.x, fminf(p.dx.y, p.dx.z));
  p.collision_friction = fminf(fmaxf(collider_friction, 0.0f), 1.0f);
  const int cells = nx * ny * nz;
  Buffer<float3> positions, velocities;
  Buffer<long long> ids;
  Buffer<int> sources;
  /* Reseeding: particles are rebuilt into the second set of arrays, then swapped. */
  Buffer<float3> positions2, velocities2;
  Buffer<long long> ids2;
  Buffer<int> sources2;
  Buffer<int> bin_keys, bin_keys_sorted, bin_index, bin_sorted;
  Buffer<int> cell_counts, cell_offsets, kept_in_cell, candidate_count, candidate_offsets;
  Buffer<int> candidate_parent;
  Buffer<float3> candidate_velocity;
  Buffer<unsigned char> keep;
  Buffer<int> reseed_counters;
  Buffer<int> reseed_totals;
  Buffer<unsigned long long> max_id;
  GrowBuffer<int> candidate_cell, candidate_slot, survive_flags, survive_scan;
  GrowBuffer<unsigned long long> list_keys, list_keys_sorted;
  GrowBuffer<int> list_values, list_values_sorted;
  GrowBuffer<unsigned char> cub_storage;
  Buffer<int> fluid;
  Buffer<float> pressure, rhs, diagonal;
  Buffer<int> pressure_active, pressure_active_count;
  Buffer<float> pcg_r, pcg_direction, pcg_q;
  Buffer<unsigned char> pcg_masks;
  Buffer<double> pcg_scalars;
  Buffer<PersistentPcgState> pcg_state;
  Buffer<PersistentPcgTotals> pcg_totals;
  Buffer<double> residual_squared, rhs_squared;
  Buffer<int> fluid_count;
  Buffer<int> tile_flags, tile_list, tile_count_buffer;
  Face faces[3];
  ColliderBuffers collider_buffers;
  const int counts[3] = {(nx + 1) * ny * nz, nx * (ny + 1) * nz, nx * ny * (nz + 1)};
  /* Tiles also cover the faces at index n on each axis. */
  p.tile_dims = make_int3(nx / tile_size + 1, ny / tile_size + 1, nz / tile_size + 1);
  const int total_tiles = p.tile_dims.x * p.tile_dims.y * p.tile_dims.z;
  p.tiles = nullptr;
  p.tile_count = nullptr;
  int grid_blocks = 160;
  {
    int device = 0, sm_count = 0;
    cudaGetDevice(&device);
    cudaDeviceGetAttribute(&sm_count, cudaDevAttrMultiProcessorCount, device);
    if (sm_count > 0) grid_blocks = sm_count * 8;
  }
  if (!positions.allocate(particle_count) || !velocities.allocate(particle_count) ||
      (reseeding &&
       (!ids.allocate(particle_count) || !sources.allocate(particle_count) ||
        !positions2.allocate(particle_count) || !velocities2.allocate(particle_count) ||
        !ids2.allocate(particle_count) || !sources2.allocate(particle_count) ||
        !bin_keys.allocate(particle_count) || !bin_keys_sorted.allocate(particle_count) ||
        !bin_index.allocate(particle_count) || !bin_sorted.allocate(particle_count) ||
        !cell_counts.allocate(cells + 1) || !cell_offsets.allocate(cells + 1) ||
        !kept_in_cell.allocate(cells) || !candidate_count.allocate(cells + 1) ||
        !candidate_offsets.allocate(cells + 1) || !candidate_parent.allocate(cells) ||
        !candidate_velocity.allocate(cells) || !keep.allocate(particle_count) ||
        !reseed_counters.allocate(4) || !reseed_totals.allocate(2) ||
        !max_id.allocate(1))) ||
      !fluid.allocate(cells) || !pressure.allocate(cells) ||
      !pressure_active.allocate(cells) || !pressure_active_count.allocate(1) ||
      !rhs.allocate(cells) ||
      !diagonal.allocate(cells) || !residual_squared.allocate(1) ||
      !rhs_squared.allocate(1) || !fluid_count.allocate(1) ||
      !tile_flags.allocate(total_tiles) || !tile_list.allocate(total_tiles) ||
      !tile_count_buffer.allocate(1) ||
      !pcg_r.allocate(cells) || !pcg_direction.allocate(cells) ||
      !pcg_q.allocate(cells) || !pcg_masks.allocate(cells) || !pcg_scalars.allocate(6) || !pcg_state.allocate(1) || !pcg_totals.allocate(1) ||
      !faces[0].allocate(counts[0], viscosity > 1.0e-12f) ||
      !faces[1].allocate(counts[1], viscosity > 1.0e-12f) ||
      !faces[2].allocate(counts[2], viscosity > 1.0e-12f) ||
      (has_collider && !collider_buffers.allocate(cells, counts)))
  {
    set_error(error, error_size, "CUDA allocation", cudaGetLastError());
    return false;
  }
  cudaDeviceSynchronize();
  const auto upload_start = std::chrono::steady_clock::now();
  status = cudaMemcpy(positions.data, input_positions, particle_count * sizeof(float3),
                      cudaMemcpyHostToDevice);
  if (status == cudaSuccess) {
    status = cudaMemcpy(velocities.data, input_velocities, particle_count * sizeof(float3),
                        cudaMemcpyHostToDevice);
  }
  if (reseeding && status == cudaSuccess) {
    status = cudaMemcpy(ids.data, input_ids, particle_count * sizeof(int64_t),
                        cudaMemcpyHostToDevice);
  }
  if (reseeding && status == cudaSuccess) {
    status = cudaMemcpy(sources.data, input_sources, particle_count * sizeof(int),
                        cudaMemcpyHostToDevice);
  }
  if (status != cudaSuccess) { set_error(error, error_size, "CUDA upload", status); return false; }
  const double upload_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - upload_start).count();
  if (has_collider) {
    p.solid_cell_phi = collider_buffers.cell_phi.data;
    for (int axis = 0; axis < 3; axis++) {
      p.blocked[axis] = collider_buffers.blocked[axis].data;
      p.face_solid[axis] = collider_buffers.face_solid[axis].data;
      cudaMemset(collider_buffers.face_solid[axis].data, 0, counts[axis]);
      p.solid_velocity[axis] = collider_buffers.velocity[axis].data;
    }
  }
  cudaMemset(pressure.data, 0, cells * sizeof(float));
  cudaMemset(pcg_totals.data, 0, sizeof(PersistentPcgTotals));
  if (reseeding) cudaMemset(reseed_totals.data, 0, 2 * sizeof(int));
  /* Reseeding can lower the particle count (never raise it), so the arrays keep their size. */
  int n = particle_count;
  int particle_groups = (n + 255) / 256;
  int total_seeded = 0;
  int total_culled = 0;
  int cell_bits = 1;
  while ((1 << cell_bits) < cells) cell_bits++;
  /* Runs a CUB algorithm with a shared, growing temporary storage. */
  const auto run_cub = [&](auto &&algorithm) -> bool {
    size_t bytes = 0;
    if (algorithm(nullptr, bytes) != cudaSuccess) return false;
    if (!cub_storage.ensure(bytes)) return false;
    return algorithm(cub_storage.data(), bytes) == cudaSuccess;
  };
  /* Sorts the first #count entries of the removal list by key and marks the first #take. */
  /* One reseeding pass with the CPU rules over the active tiles #pt. One host wait. */
  const auto reseed_particles = [&](const Params &pt) -> bool {
    const int target = target_particles_per_cell;
    const int max_count = max_particles_per_cell;
    const int min_count = min_particles_per_cell;
    cudaMemsetAsync(cell_counts.data, 0, (cells + 1) * sizeof(int));
    cudaMemsetAsync(kept_in_cell.data, 0, cells * sizeof(int));
    cudaMemsetAsync(candidate_count.data, 0, (cells + 1) * sizeof(int));
    cudaMemsetAsync(reseed_counters.data, 0, 4 * sizeof(int));
    cudaMemsetAsync(max_id.data, 0, sizeof(unsigned long long));
    reseed_bin<<<particle_groups, 256>>>(p, n, positions.data, bin_keys.data, bin_index.data,
                                         cell_counts.data, ids.data, max_id.data);
    /* Stable sort: particles of a cell stay in ascending index order, like the CPU bins. */
    if (!run_cub([&](void *storage, size_t &bytes) {
          return cub::DeviceRadixSort::SortPairs(storage, bytes, bin_keys.data,
                                                 bin_keys_sorted.data, bin_index.data,
                                                 bin_sorted.data, n, 0, cell_bits);
        }) ||
        !run_cub([&](void *storage, size_t &bytes) {
          return cub::DeviceScan::ExclusiveSum(storage, bytes, cell_counts.data,
                                               cell_offsets.data, cells + 1);
        }))
    {
      return false;
    }
    reseed_cull<<<particle_groups, 256>>>(n, bin_keys.data, ids.data, cell_counts.data,
                                          cell_offsets.data, bin_sorted.data, max_count, target,
                                          keep.data, kept_in_cell.data, reseed_counters.data);
    /* Fluid cells lie in the active tiles; candidate counts elsewhere stay zero. */
    reseed_candidates<<<grid_blocks, 256>>>(pt, fluid.data, cell_counts.data, cell_offsets.data,
                                            bin_sorted.data, positions.data, velocities.data,
                                            min_count, target, candidate_count.data,
                                            candidate_parent.data, candidate_velocity.data);
    if (!run_cub([&](void *storage, size_t &bytes) {
          return cub::DeviceScan::ExclusiveSum(storage, bytes, candidate_count.data,
                                               candidate_offsets.data, cells + 1);
        }))
    {
      return false;
    }
    /* The only wait of a reseeding pass: candidate and cull counts size the next steps. */
    cudaMemcpyAsync(reseed_counters.data + 2, candidate_offsets.data + cells, sizeof(int),
                    cudaMemcpyDeviceToDevice);
    int counters[3] = {};
    cudaMemcpy(counters, reseed_counters.data, 3 * sizeof(int), cudaMemcpyDeviceToHost);
    const int culled = counters[0];
    const int candidates = counters[2];
    const int retained = n - culled;
    const int virtual_count = retained + candidates;
    const int entries = n + candidates;
    const bool keep_budget = virtual_count > n;
    if (!candidate_cell.ensure(candidates) || !candidate_slot.ensure(candidates) ||
        !survive_flags.ensure(entries + 1) || !survive_scan.ensure(entries + 1) ||
        (keep_budget && (!list_keys.ensure(entries) || !list_values.ensure(entries) ||
                         !list_keys_sorted.ensure(entries) ||
                         !list_values_sorted.ensure(entries))))
    {
      return false;
    }
    if (candidates > 0) {
      reseed_fill_candidates<<<grid_blocks, 256>>>(pt, candidate_count.data,
                                                   candidate_offsets.data, cell_counts.data,
                                                   candidate_cell.data(), candidate_slot.data());
    }
    reseed_order<<<(entries + 1 + 255) / 256, 256>>>(
        n, candidates, bin_keys.data, keep.data, ids.data, kept_in_cell.data,
        candidate_count.data, cell_offsets.data, bin_sorted.data, candidate_cell.data(),
        candidate_offsets.data, max_id.data, min_count, keep_budget, list_keys.data(),
        list_values.data(), survive_flags.data());
    if (keep_budget) {
      /* Keep the particle budget: drop the first `excess` entries of the removal order. The
       * list is in index order, so a stable key sort resolves ties by index like the CPU. */
      if (!run_cub([&](void *storage, size_t &bytes) {
            return cub::DeviceRadixSort::SortPairs(storage, bytes, list_keys.data(),
                                                   list_keys_sorted.data(), list_values.data(),
                                                   list_values_sorted.data(), entries);
          }))
      {
        return false;
      }
      const int excess = virtual_count - n;
      reseed_unflag<<<(excess + 255) / 256, 256>>>(excess, list_values_sorted.data(),
                                                    survive_flags.data());
    }
    if (!run_cub([&](void *storage, size_t &bytes) {
          return cub::DeviceScan::ExclusiveSum(storage, bytes, survive_flags.data(),
                                               survive_scan.data(), entries + 1);
        }))
    {
      return false;
    }
    reseed_write<<<std::max(1, (entries + 255) / 256), 256>>>(
        p, n, candidates, survive_flags.data(), survive_scan.data(), positions.data,
        velocities.data, ids.data, sources.data, candidate_cell.data(), candidate_slot.data(),
        candidate_parent.data, candidate_velocity.data, max_id.data, retained,
        reseed_counters.data, reseed_totals.data, positions2.data, velocities2.data,
        ids2.data, sources2.data);
    std::swap(positions.data, positions2.data);
    std::swap(velocities.data, velocities2.data);
    std::swap(ids.data, ids2.data);
    std::swap(sources.data, sources2.data);
    /* With the budget kept the count is unchanged, otherwise every particle survives. */
    n = std::min(n, virtual_count);
    p.particles = n;
    particle_groups = std::max(1, (n + 255) / 256);
    return cudaGetLastError() == cudaSuccess;
  };
  const int cell_groups = (cells + 255) / 256;
  constexpr int pcg_groups = 256;
  cudaStream_t pcg_stream = nullptr;
  const auto pcg_iteration = [&](const int iteration) {
    const int current = iteration & 1;
    const int next = 1 - current;
    cudaMemsetAsync(pcg_scalars.data + 5, 0, sizeof(double), pcg_stream);
    pcg_matrix_dot<<<pcg_groups, 256, 0, pcg_stream>>>(p, fluid.data,
                                          pressure_active.data, pressure_active_count.data,
                                          diagonal.data,
                                          pcg_direction.data, pcg_q.data,
                                          pcg_scalars.data + 5);
    cudaMemsetAsync(pcg_scalars.data + next, 0, sizeof(double), pcg_stream);
    cudaMemsetAsync(pcg_scalars.data + 2 + next, 0, sizeof(double), pcg_stream);
    pcg_update<<<pcg_groups, 256, 0, pcg_stream>>>(p, pressure_active.data,
                                     pressure_active_count.data, diagonal.data, pressure.data,
                                     pcg_r.data, pcg_direction.data, pcg_q.data,
                                     pcg_scalars.data + current,
                                     pcg_scalars.data + 5,
                                     pcg_scalars.data + next,
                                     pcg_scalars.data + 2 + next);
    pcg_direction_update<<<pcg_groups, 256, 0, pcg_stream>>>(p, pressure_active.data,
                                               pressure_active_count.data, diagonal.data,
                                               pcg_r.data, pcg_direction.data,
                                               pcg_scalars.data + current,
                                               pcg_scalars.data + next);
  };
  /* Cooperative launches need every block resident at once: size the grid by occupancy. */
  int persistent_blocks = 0;
  {
    int device = 0, cooperative = 0, sm_count = 0, blocks_per_sm = 0;
    cudaGetDevice(&device);
    cudaDeviceGetAttribute(&cooperative, cudaDevAttrCooperativeLaunch, device);
    cudaDeviceGetAttribute(&sm_count, cudaDevAttrMultiProcessorCount, device);
    if (cooperative &&
        cudaOccupancyMaxActiveBlocksPerMultiprocessor(&blocks_per_sm, pcg_persistent, 256, 0) ==
            cudaSuccess)
    {
      persistent_blocks = blocks_per_sm * sm_count;
    }
    cudaGetLastError();
  }
  /* Without cooperative launches, a graph batches the launches of 16 PCG iterations. When
   * capturing fails, keep solving on the GPU with regular launches. */
  CapturedGraph pcg_graph;
  if (persistent_blocks == 0 && pressure_iterations >= 16) {
    /* Load the kernels before capturing: lazy module loading inside a capture invalidates it. */
    cudaFuncAttributes attributes;
    cudaFuncGetAttributes(&attributes, pcg_matrix_dot);
    cudaFuncGetAttributes(&attributes, pcg_update);
    cudaFuncGetAttributes(&attributes, pcg_direction_update);
    cudaDeviceSynchronize();
    cudaGetLastError();
    cudaGraph_t graph = nullptr;
    bool captured = cudaStreamCreateWithFlags(&pcg_stream, cudaStreamNonBlocking) ==
                    cudaSuccess;
    if (captured) {
      /* Thread-local mode: CUDA calls made by other threads must not invalidate the capture. */
      captured = cudaStreamBeginCapture(pcg_stream, cudaStreamCaptureModeThreadLocal) ==
                 cudaSuccess;
      if (captured) {
        for (int iteration = 0; iteration < 16; iteration++) pcg_iteration(iteration);
        captured = cudaStreamEndCapture(pcg_stream, &graph) == cudaSuccess && graph != nullptr;
      }
      cudaStreamDestroy(pcg_stream);
      pcg_stream = nullptr;
    }
    if (captured) {
      captured = cudaGraphInstantiate(&pcg_graph.executable, graph, nullptr, nullptr, 0) ==
                 cudaSuccess;
    }
    if (graph) {
      cudaGraphDestroy(graph);
    }
    if (!captured) {
      pcg_graph.executable = nullptr;
      /* Clear the non-sticky capture error so later status checks are not affected. */
      cudaGetLastError();
      if (std::getenv("BLENDER_FLIP_PROFILE")) {
        std::fprintf(stderr, "FLIP_CUDA pressure graph capture failed, using plain launches\n");
      }
    }
  }
  const auto update_face_solid = [&]() {
    for (int axis = 0; axis < 3; axis++) {
      compute_face_solid<<<grid_blocks, 256>>>(p, axis, collider_buffers.face_solid[axis].data);
    }
  };
  /* Static collider fields from an earlier frame. */
  bool collider_cached = false;
  if (has_collider && static_collider_key != 0) {
    std::lock_guard lock(DeviceColliderCache::mutex());
    DeviceColliderCache &cache = DeviceColliderCache::get();
    if (cache.key == static_collider_key && cache.cells == cells && cache.counts[0] == counts[0] &&
        cache.counts[1] == counts[1] && cache.counts[2] == counts[2])
    {
      cudaMemcpy(collider_buffers.cell_phi.data, cache.cell_phi, cells * sizeof(float),
                 cudaMemcpyDeviceToDevice);
      if (cache.has_normal) {
        cudaMemcpy(collider_buffers.cell_normal.data, cache.cell_normal, cells * sizeof(float3),
                   cudaMemcpyDeviceToDevice);
      }
      for (int axis = 0; axis < 3; axis++) {
        cudaMemcpy(collider_buffers.blocked[axis].data, cache.blocked[axis], counts[axis],
                   cudaMemcpyDeviceToDevice);
        cudaMemcpy(collider_buffers.velocity[axis].data, cache.velocity[axis],
                   counts[axis] * sizeof(float), cudaMemcpyDeviceToDevice);
      }
      p.solid_cell_normal = cache.has_normal ? collider_buffers.cell_normal.data : nullptr;
      update_face_solid();
      collider_cached = cudaGetLastError() == cudaSuccess;
    }
  }
  timer.lap(Step::Setup);
  int total_pressure_iterations = 0;
  int total_viscosity_iterations = 0;
  double maximum_viscosity_residual = 0.0;
  /* Without CPU reseeding or CPU collision, the host only prepares the first substep (collider
   * fields, particles that start inside the collider); later substeps stay on the device. */
  const bool host_every_step = host_step_callback != nullptr && host_callback_every_step;
  for (int step = 0; step < substeps; step++) {
    if (host_step_callback != nullptr && (host_every_step || (step == 0 && !collider_cached))) {
      FlipCudaColliderFields fields;
      if (!host_step_callback(host_step_context, step, false, &fields)) {
        if (error && error_size) {
          std::snprintf(error, error_size, "FLIP GPU substep preparation failed");
        }
        return false;
      }
      if (has_collider && fields.dirty) {
        status = cudaMemcpy(collider_buffers.cell_phi.data, fields.cell_phi,
                            cells * sizeof(float), cudaMemcpyHostToDevice);
        if (status == cudaSuccess && fields.cell_normal) {
          status = cudaMemcpy(collider_buffers.cell_normal.data, fields.cell_normal,
                              cells * sizeof(float3), cudaMemcpyHostToDevice);
        }
        p.solid_cell_normal = fields.cell_normal ? collider_buffers.cell_normal.data : nullptr;
        for (int axis = 0; axis < 3 && status == cudaSuccess; axis++) {
          status = cudaMemcpy(collider_buffers.blocked[axis].data, fields.blocked[axis],
                              counts[axis] * sizeof(unsigned char), cudaMemcpyHostToDevice);
          if (status == cudaSuccess) {
            status = cudaMemcpy(collider_buffers.velocity[axis].data, fields.velocity[axis],
                                counts[axis] * sizeof(float), cudaMemcpyHostToDevice);
          }
        }
        if (status != cudaSuccess) {
          set_error(error, error_size, "CUDA collider upload", status);
          return false;
        }
        update_face_solid();
        if (static_collider_key != 0 && fields.complete) {
          std::lock_guard lock(DeviceColliderCache::mutex());
          DeviceColliderCache &cache = DeviceColliderCache::get();
          if (cache.cells != cells || cache.counts[0] != counts[0] ||
              cache.counts[1] != counts[1] || cache.counts[2] != counts[2] ||
              cache.cell_phi == nullptr)
          {
            cache.release();
            bool ok = cudaMalloc(&cache.cell_phi, cells * sizeof(float)) == cudaSuccess &&
                      cudaMalloc(&cache.cell_normal, cells * sizeof(float3)) == cudaSuccess;
            for (int axis = 0; axis < 3 && ok; axis++) {
              ok = cudaMalloc(&cache.blocked[axis], counts[axis]) == cudaSuccess &&
                   cudaMalloc(&cache.velocity[axis], counts[axis] * sizeof(float)) ==
                       cudaSuccess;
            }
            if (!ok) {
              cache.release();
              cudaGetLastError();
            }
            cache.cells = ok ? cells : 0;
            for (int axis = 0; axis < 3; axis++) cache.counts[axis] = ok ? counts[axis] : 0;
          }
          if (cache.cell_phi) {
            cudaMemcpy(cache.cell_phi, collider_buffers.cell_phi.data, cells * sizeof(float),
                       cudaMemcpyDeviceToDevice);
            cache.has_normal = fields.cell_normal != nullptr;
            if (cache.has_normal) {
              cudaMemcpy(cache.cell_normal, collider_buffers.cell_normal.data,
                         cells * sizeof(float3), cudaMemcpyDeviceToDevice);
            }
            for (int axis = 0; axis < 3; axis++) {
              cudaMemcpy(cache.blocked[axis], collider_buffers.blocked[axis].data, counts[axis],
                         cudaMemcpyDeviceToDevice);
              cudaMemcpy(cache.velocity[axis], collider_buffers.velocity[axis].data,
                         counts[axis] * sizeof(float), cudaMemcpyDeviceToDevice);
            }
            cache.key = static_collider_key;
          }
        }
      }
      status = cudaMemcpy(positions.data, output_positions, n * sizeof(float3),
                          cudaMemcpyHostToDevice);
      if (status == cudaSuccess) {
        status = cudaMemcpy(velocities.data, output_velocities, n * sizeof(float3),
                            cudaMemcpyHostToDevice);
      }
      if (status != cudaSuccess) {
        set_error(error, error_size, "CUDA reseed upload", status);
        return false;
      }
    }
    timer.lap(Step::HostPrepare);
    /* Like the CPU substep: replace non-finite values and clamp into the domain first. */
    reseed_sanitize<<<particle_groups, 256>>>(p, n, positions.data, velocities.data);
    if (step == 0 && collider_cached && p.gpu_collision) {
      resolve_inside_collider<<<particle_groups, 256>>>(p, n, positions.data, velocities.data);
    }
    /* Active tiles: particles plus one tile of margin. Reseeding only adds particles to
     * existing fluid cells, so the tiles stay valid after it. */
    Params pt = p;
    if (extrapolation_layers + 2 <= tile_size) {
      cudaMemsetAsync(tile_flags.data, 0, total_tiles * sizeof(int));
      cudaMemsetAsync(tile_count_buffer.data, 0, sizeof(int));
      mark_tiles<<<particle_groups, 256>>>(p, positions.data, tile_flags.data);
      collect_tiles<<<(total_tiles + 255) / 256, 256>>>(p, tile_flags.data, tile_list.data,
                                                         tile_count_buffer.data);
      pt.tiles = tile_list.data;
      pt.tile_count = tile_count_buffer.data;
    }
    timer.lap(Step::Tiles);
    const auto classify = [&]() {
      cudaMemsetAsync(fluid.data, 0, cells * sizeof(int));
      classify_particles<<<particle_groups, 256>>>(p, positions.data, fluid.data);
      if (has_collider) {
        mask_solid_cells<<<grid_blocks, 256>>>(pt, fluid.data);
      }
    };
    classify();
    timer.lap(Step::Classify);
    if (reseeding && n >= target_particles_per_cell) {
      if (!reseed_particles(pt)) {
        set_error(error, error_size, "CUDA reseeding", cudaGetLastError());
        return false;
      }
      classify();
    }
    timer.lap(Step::GpuReseed);
    /* Grid kernels loop over their elements, so a fixed launch fills the device. */
    const auto face_groups = [&](const int /*count*/) { return grid_blocks; };
    const int grid_cell_groups = grid_blocks;
    const auto extrapolate = [&](Face &f, const int axis, const Params &params) {
      for (int layer = 0; layer < extrapolation_layers; layer++) {
        extrapolate_faces<<<grid_blocks, 256>>>(params, axis, f.value.data, f.valid.data,
                                                f.alternate.data, f.alternate_valid.data);
        std::swap(f.value.data, f.alternate.data);
        std::swap(f.valid.data, f.alternate_valid.data);
      }
    };
    for (int axis = 0; axis < 3; axis++) {
      Face &f = faces[axis];
      if (pt.tiles) {
        clear_faces<<<face_groups(f.count), 256>>>(pt, axis, f.value.data, f.weight.data);
      }
      else {
        cudaMemset(f.value.data, 0, f.count * sizeof(float));
        cudaMemset(f.weight.data, 0, f.count * sizeof(float));
      }
      timer.lap(Step::P2GClear);
      scatter_faces<<<particle_groups, 256>>>(p, positions.data, velocities.data, axis,
                                               f.value.data, f.weight.data);
      timer.lap(Step::P2GScatter);
      normalize_faces<<<face_groups(f.count), 256>>>(pt, axis, f.value.data, f.weight.data,
                                                      f.valid.data);
      timer.lap(Step::P2GNormalize);
      extrapolate(f, axis, pt);
      timer.lap(Step::Extrapolate);
      force_and_boundary<<<face_groups(f.count), 256>>>(pt, fluid.data, axis,
                                                         false, f.value.data);
      if (pt.tiles) {
        copy_faces<<<face_groups(f.count), 256>>>(pt, axis, f.value.data, f.old.data);
      }
      else {
        cudaMemcpy(f.old.data, f.value.data, f.count * sizeof(float),
                   cudaMemcpyDeviceToDevice);
      }
      force_and_boundary<<<face_groups(f.count), 256>>>(pt, fluid.data, axis,
                                                         true, f.value.data);
      timer.lap(Step::Boundary);
    }
    if (viscosity > 1.0e-12f) {
      for (int axis = 0; axis < 3; axis++) {
        Face &f = faces[axis];
        cudaMemset(f.active_count.data, 0, sizeof(int));
        collect_viscosity_faces<<<face_groups(f.count), 256>>>(
            pt, fluid.data, axis, f.active.data, f.active_count.data);
        int active_count = 0;
        status = cudaMemcpy(&active_count, f.active_count.data, sizeof(int),
                            cudaMemcpyDeviceToHost);
        if (status != cudaSuccess) {
          set_error(error, error_size, "CUDA viscosity face collection", status);
          return false;
        }
        if (active_count == 0) continue;
        cudaMemcpy(f.weight.data, f.value.data, f.count * sizeof(float),
                   cudaMemcpyDeviceToDevice);
        cudaMemcpy(f.alternate.data, f.value.data, f.count * sizeof(float),
                   cudaMemcpyDeviceToDevice);
        double relative_residual = 1.0;
        int iteration = 0;
        while (iteration < viscosity_iterations && relative_residual > viscosity_tolerance) {
          const int batch_end = std::min(iteration + 8, viscosity_iterations);
          for (; iteration < batch_end; iteration++) {
            viscosity_jacobi<<<(active_count + 255) / 256, 256>>>(
                p, fluid.data, axis, f.active.data, active_count, viscosity,
                f.weight.data, f.value.data, f.alternate.data);
            std::swap(f.value.data, f.alternate.data);
          }
          cudaMemset(residual_squared.data, 0, sizeof(double));
          cudaMemset(rhs_squared.data, 0, sizeof(double));
          measure_viscosity<<<(active_count + 255) / 256, 256>>>(
              p, fluid.data, axis, f.active.data, active_count, viscosity,
              f.weight.data, f.value.data, residual_squared.data, rhs_squared.data);
          double squared[2];
          status = cudaMemcpy(&squared[0], residual_squared.data, sizeof(double),
                              cudaMemcpyDeviceToHost);
          if (status == cudaSuccess) {
            status = cudaMemcpy(&squared[1], rhs_squared.data, sizeof(double),
                                cudaMemcpyDeviceToHost);
          }
          if (status != cudaSuccess || !std::isfinite(squared[0]) ||
              !std::isfinite(squared[1])) {
            if (status != cudaSuccess) set_error(error, error_size, "CUDA viscosity measurement", status);
            else if (error && error_size) std::snprintf(error, error_size, "CUDA viscosity residual is nonfinite");
            return false;
          }
          relative_residual = std::sqrt(squared[0] / std::max(squared[1], 1.0e-40));
        }
        total_viscosity_iterations += iteration;
        maximum_viscosity_residual = std::max(maximum_viscosity_residual, relative_residual);
        force_and_boundary<<<face_groups(f.count), 256>>>(pt, fluid.data, axis,
                                                             false, f.value.data);
      }
    }
    timer.lap(Step::Viscosity);
    pressure_rhs<<<grid_cell_groups, 256>>>(pt, fluid.data, faces[0].value.data,
        faces[1].value.data, faces[2].value.data, rhs.data, diagonal.data);
    /* Liquid cells in index order: neighboring rows stay close in memory during the solve. */
    if (!run_cub([&](void *storage, size_t &bytes) {
          return cub::DeviceSelect::Flagged(storage, bytes, cub::CountingInputIterator<int>(0),
                                            fluid.data, pressure_active.data,
                                            pressure_active_count.data, cells);
        }))
    {
      set_error(error, error_size, "CUDA liquid cell list", cudaGetLastError());
      return false;
    }
    int iteration = 0;
    if (persistent_blocks > 0) {
      /* Enqueue only: the solve checks convergence itself and the host does not wait. */
      timer.lap(Step::PressureSetup);
      cudaMemsetAsync(pcg_state.data, 0, sizeof(PersistentPcgState));
      Params kernel_params = p;
      int *fluid_arg = fluid.data;
      const int *active_arg = pressure_active.data;
      const int *active_count_arg = pressure_active_count.data;
      const float *rhs_arg = rhs.data;
      const float *diagonal_arg = diagonal.data;
      float *pressure_arg = pressure.data;
      float *r_arg = pcg_r.data;
      unsigned char *masks_arg = pcg_masks.data;
      float *direction_arg = pcg_direction.data;
      float *q_arg = pcg_q.data;
      PersistentPcgState *state_arg = pcg_state.data;
      PersistentPcgTotals *totals_arg = pcg_totals.data;
      double tolerance_arg = pressure_tolerance;
      int max_arg = pressure_iterations;
      /* The first solve plus up to two passes that release cells in tension. */
      int passes_arg = 3;
      void *args[] = {&kernel_params, &fluid_arg, &active_arg, &active_count_arg, &rhs_arg,
                      &diagonal_arg, &pressure_arg, &r_arg, &masks_arg, &direction_arg,
                      &q_arg, &state_arg, &totals_arg, &tolerance_arg, &max_arg, &passes_arg};
      status = cudaLaunchCooperativeKernel((const void *)pcg_persistent, dim3(persistent_blocks),
                                           dim3(256), args, 0, nullptr);
      if (status != cudaSuccess) {
        set_error(error, error_size, "CUDA persistent pressure solve", status);
        return false;
      }
    }
    else {
      cudaMemset(pcg_scalars.data, 0, 6 * sizeof(double));
      pcg_initialize<<<pcg_groups, 256>>>(p, fluid.data, pressure_active.data,
                                            pressure_active_count.data, rhs.data, diagonal.data,
                                            pressure.data, pcg_r.data,
                                            pcg_direction.data, pcg_scalars.data);
      double initial_norms[3] = {};
      cudaMemcpy(initial_norms, pcg_scalars.data + 2, 3 * sizeof(double),
                 cudaMemcpyDeviceToHost);
      double residual_squared_host = initial_norms[0];
      const double rhs_squared_host = initial_norms[2];
      if (!std::isfinite(residual_squared_host) || !std::isfinite(rhs_squared_host)) {
        if (error && error_size) std::snprintf(error, error_size, "CUDA pressure initialized with nonfinite residual");
        return false;
      }
      const double target_squared = double(pressure_tolerance) * pressure_tolerance *
                                    fmax(rhs_squared_host, 1.0e-40);
      timer.lap(Step::PressureSetup);
      while (iteration < pressure_iterations && residual_squared_host > target_squared)
      {
        const int batch_end = std::min(iteration + 16, pressure_iterations);
        if (pcg_graph.executable && batch_end - iteration == 16) {
          status = cudaGraphLaunch(pcg_graph.executable, nullptr);
          if (status != cudaSuccess) {
            set_error(error, error_size, "CUDA pressure graph launch", status);
            return false;
          }
          iteration = batch_end;
        }
        else {
          for (; iteration < batch_end; iteration++) pcg_iteration(iteration);
        }
        cudaMemcpy(&residual_squared_host, pcg_scalars.data + 2 + (iteration & 1),
                   sizeof(double), cudaMemcpyDeviceToHost);
        if (!std::isfinite(residual_squared_host)) {
          if (error && error_size) std::snprintf(error, error_size, "CUDA pressure produced nonfinite residual");
          return false;
        }
      }
    }
    total_pressure_iterations += iteration;
    if (timer.enabled) {
      int active_tiles_host = 0, fluid_cells_host = 0;
      cudaMemcpy(&active_tiles_host, tile_count_buffer.data, sizeof(int), cudaMemcpyDeviceToHost);
      cudaMemcpy(&fluid_cells_host, pressure_active_count.data, sizeof(int),
                 cudaMemcpyDeviceToHost);
      timer.note_tiles(active_tiles_host, total_tiles, fluid_cells_host);
    }
    timer.lap(Step::PressurePcg);
    for (int axis = 0; axis < 3; axis++) {
      Face &f = faces[axis];
      project_faces<<<face_groups(f.count), 256>>>(pt, fluid.data, pressure.data,
                                                     axis, f.value.data);
      extrapolate(f, axis, pt);
      force_and_boundary<<<face_groups(f.count), 256>>>(pt, fluid.data, axis,
                                                         false, f.value.data);
    }
    timer.lap(Step::Project);
    grid_to_particles<<<particle_groups, 256>>>(p, positions.data, velocities.data,
       faces[0].value.data, faces[1].value.data, faces[2].value.data,
       faces[0].old.data, faces[1].old.data, faces[2].old.data);
    timer.lap(Step::G2P);
    if (host_every_step) {
      status = cudaMemcpy(output_positions, positions.data, n * sizeof(float3),
                          cudaMemcpyDeviceToHost);
      if (status == cudaSuccess) {
        status = cudaMemcpy(output_velocities, velocities.data, n * sizeof(float3),
                            cudaMemcpyDeviceToHost);
      }
      if (status != cudaSuccess) {
        set_error(error, error_size, "CUDA substep download", status);
        return false;
      }
      if (!host_step_callback(host_step_context, step, true, nullptr)) {
        if (error && error_size) std::snprintf(error, error_size, "FLIP GPU particle collision failed");
        return false;
      }
      timer.lap(Step::HostAfter);
    }
  }
  cudaMemset(residual_squared.data, 0, sizeof(double));
  cudaMemset(rhs_squared.data, 0, sizeof(double));
  cudaMemset(fluid_count.data, 0, sizeof(int));
  measure_pressure<<<cell_groups, 256>>>(p, fluid.data, rhs.data, diagonal.data,
                                         pressure.data, residual_squared.data,
                                         rhs_squared.data, fluid_count.data);
  status = cudaDeviceSynchronize();
  if (status != cudaSuccess) { set_error(error, error_size, "CUDA solve", status); return false; }
  double residual_sum = 0.0, rhs_sum = 0.0;
  int final_fluid_cells = 0;
  cudaMemcpy(&residual_sum, residual_squared.data, sizeof(double), cudaMemcpyDeviceToHost);
  cudaMemcpy(&rhs_sum, rhs_squared.data, sizeof(double), cudaMemcpyDeviceToHost);
  cudaMemcpy(&final_fluid_cells, fluid_count.data, sizeof(int), cudaMemcpyDeviceToHost);
  if (!std::isfinite(residual_sum) || !std::isfinite(rhs_sum)) {
    if (error && error_size) std::snprintf(error, error_size, "CUDA pressure measurement is nonfinite");
    return false;
  }
  if (output_relative_residual) {
    *output_relative_residual = sqrt(residual_sum / fmax(rhs_sum, 1.0e-40));
  }
  if (output_fluid_cells) *output_fluid_cells = final_fluid_cells;
  if (persistent_blocks > 0) {
    PersistentPcgTotals totals{};
    cudaMemcpy(&totals, pcg_totals.data, sizeof(totals), cudaMemcpyDeviceToHost);
    if (totals.nonfinite) {
      if (error && error_size) std::snprintf(error, error_size, "CUDA pressure produced nonfinite residual");
      return false;
    }
    total_pressure_iterations += totals.iterations;
  }
  if (output_pressure_iterations) *output_pressure_iterations = total_pressure_iterations;
  if (output_viscosity_iterations) *output_viscosity_iterations = total_viscosity_iterations;
  if (output_viscosity_residual) *output_viscosity_residual = maximum_viscosity_residual;
  if (reseeding) {
    int totals[2] = {};
    cudaMemcpy(totals, reseed_totals.data, sizeof(totals), cudaMemcpyDeviceToHost);
    total_seeded = totals[0];
    total_culled = totals[1];
  }
  if (output_reseeded) *output_reseeded = total_seeded;
  if (output_culled) *output_culled = total_culled;
  if (output_particle_count) *output_particle_count = n;
  cudaDeviceSynchronize();
  const auto download_start = std::chrono::steady_clock::now();
  if (!host_every_step) {
    status = cudaMemcpy(output_positions, positions.data, n * sizeof(float3),
                        cudaMemcpyDeviceToHost);
    if (status == cudaSuccess) {
      status = cudaMemcpy(output_velocities, velocities.data, n * sizeof(float3),
                          cudaMemcpyDeviceToHost);
    }
  }
  if (reseeding && status == cudaSuccess) {
    status = cudaMemcpy(output_ids, ids.data, n * sizeof(int64_t),
                        cudaMemcpyDeviceToHost);
  }
  if (reseeding && status == cudaSuccess) {
    status = cudaMemcpy(output_sources, sources.data, n * sizeof(int),
                        cudaMemcpyDeviceToHost);
  }
  if (status != cudaSuccess) { set_error(error, error_size, "CUDA download", status); return false; }
  const double download_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - download_start).count();
  timer.note_transfer(upload_ms, download_ms, particle_count, n);
  timer.lap(Step::Finish);
  timer.print();
  return true;
}
