/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include <algorithm>

#include "BLI_array_utils.hh"
#include "BLI_math_euler.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion.hh"

#include "PRF_profile.hh"

#include "BKE_attribute_math.hh"

namespace blender::bke::attribute_math {

template<>
math::Quaternion mix2(const float factor, const math::Quaternion &a, const math::Quaternion &b)
{
  return math::interpolate(a, b, factor);
}

template<>
math::Quaternion mix3(const float3 &weights,
                      const math::Quaternion &v0,
                      const math::Quaternion &v1,
                      const math::Quaternion &v2)
{
  const float3 expmap_mixed = mix3(weights, v0.expmap(), v1.expmap(), v2.expmap());
  return math::Quaternion::expmap(expmap_mixed);
}

template<>
math::Quaternion mix4(const float4 &weights,
                      const math::Quaternion &v0,
                      const math::Quaternion &v1,
                      const math::Quaternion &v2,
                      const math::Quaternion &v3)
{
  const float3 expmap_mixed = mix4(weights, v0.expmap(), v1.expmap(), v2.expmap(), v3.expmap());
  return math::Quaternion::expmap(expmap_mixed);
}

template<> float4x4 mix2(const float factor, const float4x4 &a, const float4x4 &b)
{
  return math::interpolate_linear(a, b, factor);
}

template<>
float4x4 mix3(const float3 &weights, const float4x4 &v0, const float4x4 &v1, const float4x4 &v2)
{
  return v0 * weights.x + v1 * weights.y + v2 * weights.z;
}

template<>
float4x4 mix4(const float4 &weights,
              const float4x4 &v0,
              const float4x4 &v1,
              const float4x4 &v2,
              const float4x4 &v3)
{
  return v0 * weights.x + v1 * weights.y + v2 * weights.z + v3 * weights.w;
}

ColorGeometry4fMixer::ColorGeometry4fMixer(MutableSpan<ColorGeometry4f> buffer,
                                           ColorGeometry4f default_color)
    : ColorGeometry4fMixer(buffer, buffer.index_range(), default_color)
{
}

ColorGeometry4fMixer::ColorGeometry4fMixer(MutableSpan<ColorGeometry4f> buffer,
                                           const IndexMask &mask,
                                           const ColorGeometry4f default_color)
    : buffer_(buffer), default_color_(default_color), total_weights_(buffer.size(), 0.0f)
{
  const ColorGeometry4f zero{0.0f, 0.0f, 0.0f, 0.0f};
  index_mask::masked_fill(buffer_, zero, mask);
}

void ColorGeometry4fMixer::set(const int64_t index,
                               const ColorGeometry4f &color,
                               const float weight)
{
  buffer_[index].r = color.r * weight;
  buffer_[index].g = color.g * weight;
  buffer_[index].b = color.b * weight;
  buffer_[index].a = color.a * weight;
  total_weights_[index] = weight;
}

void ColorGeometry4fMixer::mix_in(const int64_t index,
                                  const ColorGeometry4f &color,
                                  const float weight)
{
  ColorGeometry4f &output_color = buffer_[index];
  output_color.r += color.r * weight;
  output_color.g += color.g * weight;
  output_color.b += color.b * weight;
  output_color.a += color.a * weight;
  total_weights_[index] += weight;
}

void ColorGeometry4fMixer::finalize()
{
  this->finalize(buffer_.index_range());
}

void ColorGeometry4fMixer::finalize(const IndexMask &mask)
{
  mask.foreach_index([&](const int64_t i) {
    const float weight = total_weights_[i];
    ColorGeometry4f &output_color = buffer_[i];
    if (weight > 0.0f) {
      const float weight_inv = 1.0f / weight;
      output_color.r *= weight_inv;
      output_color.g *= weight_inv;
      output_color.b *= weight_inv;
      output_color.a *= weight_inv;
    }
    else {
      output_color = default_color_;
    }
  });
}

ColorGeometry4bMixer::ColorGeometry4bMixer(MutableSpan<ColorGeometry4b> buffer,
                                           const ColorGeometry4b default_color)
    : ColorGeometry4bMixer(buffer, buffer.index_range(), default_color)
{
}

ColorGeometry4bMixer::ColorGeometry4bMixer(MutableSpan<ColorGeometry4b> buffer,
                                           const IndexMask &mask,
                                           const ColorGeometry4b default_color)
    : buffer_(buffer),
      default_color_(default_color),
      total_weights_(buffer.size(), 0.0f),
      accumulation_buffer_(buffer.size(), float4(0, 0, 0, 0))
{
  const ColorGeometry4b zero{0, 0, 0, 0};
  index_mask::masked_fill(buffer_, zero, mask);
}

void ColorGeometry4bMixer::ColorGeometry4bMixer::set(int64_t index,
                                                     const ColorGeometry4b &color,
                                                     const float weight)
{
  accumulation_buffer_[index][0] = color.r * weight;
  accumulation_buffer_[index][1] = color.g * weight;
  accumulation_buffer_[index][2] = color.b * weight;
  accumulation_buffer_[index][3] = color.a * weight;
  total_weights_[index] = weight;
}

void ColorGeometry4bMixer::mix_in(int64_t index, const ColorGeometry4b &color, float weight)
{
  float4 &accum_value = accumulation_buffer_[index];
  accum_value[0] += color.r * weight;
  accum_value[1] += color.g * weight;
  accum_value[2] += color.b * weight;
  accum_value[3] += color.a * weight;
  total_weights_[index] += weight;
}

void ColorGeometry4bMixer::finalize()
{
  this->finalize(buffer_.index_range());
}

void ColorGeometry4bMixer::finalize(const IndexMask &mask)
{
  PRF_scope_with_name("ColorGeometry4bMixer::finalize", ProfileCategory::Default);
  mask.foreach_index([&](const int64_t i) {
    const float weight = total_weights_[i];
    const float4 &accum_value = accumulation_buffer_[i];
    ColorGeometry4b &output_color = buffer_[i];
    if (weight > 0.0f) {
      const float weight_inv = 1.0f / weight;
      output_color.r = accum_value[0] * weight_inv;
      output_color.g = accum_value[1] * weight_inv;
      output_color.b = accum_value[2] * weight_inv;
      output_color.a = accum_value[3] * weight_inv;
    }
    else {
      output_color = default_color_;
    }
  });
}

float4x4Mixer::float4x4Mixer(MutableSpan<float4x4> buffer)
    : float4x4Mixer(buffer, buffer.index_range())
{
}

float4x4Mixer::float4x4Mixer(MutableSpan<float4x4> buffer, const IndexMask & /*mask*/)
    : buffer_(buffer),
      total_weights_(buffer.size(), 0.0f),
      matrix_buffer_(buffer.size(), float4x4::all(0.0f))
{
}

void float4x4Mixer::float4x4Mixer::set(int64_t index, const float4x4 &value, const float weight)
{
  matrix_buffer_[index] = value * weight;
  total_weights_[index] = weight;
}

void float4x4Mixer::mix_in(int64_t index, const float4x4 &value, float weight)
{
  matrix_buffer_[index] += value * weight;
  total_weights_[index] += weight;
}

void float4x4Mixer::finalize()
{
  this->finalize(buffer_.index_range());
}

void float4x4Mixer::finalize(const IndexMask &mask)
{
  PRF_scope_with_name("float4x4Mixer::finalize", ProfileCategory::Default);
  mask.foreach_index([&](const int64_t i) {
    const float weight = total_weights_[i];
    if (weight > 0.0f) {
      const float weight_inv = math::rcp(weight);
      buffer_[i] = matrix_buffer_[i] * weight_inv;
    }
    else {
      buffer_[i] = float4x4::identity();
    }
  });
}

float4x4 mix_indices(const Span<float4x4> src, const Span<int> indices)
{
  float4x4 accum = float4x4::all(0.0f);
  for (const int i : indices) {
    accum += src[i];
  }

  const float weight_inv = math::safe_rcp(float(indices.size()));
  return indices.is_empty() ? float4x4::identity() : accum * weight_inv;
}

template<typename T>
void mix_groups(const Span<T> src,
                const OffsetIndices<int> groups,
                const Span<int> all_indices,
                const IndexMask &dst_mask,
                MutableSpan<T> dst)
{
  PRF_scope_with_name("attribute_math::mix_groups", ProfileCategory::Default);
  dst_mask.foreach_index([&](const int dst_i, const int group_i) {
    dst[dst_i] = mix_indices(src, all_indices.slice(groups[group_i]));
  });
}

float4x4 mix_indices(const Span<float4x4> src, const Span<int> indices, const Span<float> weights)
{
  float4x4 accum = float4x4::all(0.0f);
  float total_weight = 0.0f;
  for (const int i : indices.index_range()) {
    const float weight = weights[i];
    accum += src[indices[i]] * weight;
    total_weight += weight;
  }

  const float weight_inv = math::safe_rcp(total_weight);
  return total_weight == 0.0f ? float4x4::identity() : accum * weight_inv;
}

template<typename T>
void mix_groups(const Span<T> src,
                const OffsetIndices<int> groups,
                const Span<int> all_indices,
                const Span<float> all_weights,
                const IndexMask &dst_mask,
                MutableSpan<T> dst)
{
  PRF_scope_with_name("attribute_math::mix_groups", ProfileCategory::Default);
  dst_mask.foreach_index([&](const int dst_i, const int group_i) {
    dst[dst_i] = mix_indices(
        src, all_indices.slice(groups[group_i]), all_weights.slice(groups[group_i]));
  });
}

void mix_groups(const GSpan src,
                const OffsetIndices<int> groups,
                const Span<int> all_indices,
                const std::optional<Span<float>> all_weights,
                GMutableSpan dst)
{
  BLI_assert(groups.size() == dst.size());
  mix_groups(src, groups, all_indices, all_weights, IndexMask(dst.size()), dst);
}

void mix_groups(const GSpan src,
                const OffsetIndices<int> groups,
                const Span<int> all_indices,
                const std::optional<Span<float>> all_weights,
                const IndexMask &dst_mask,
                GMutableSpan dst)
{
  BLI_assert(groups.size() == dst_mask.size());
  BLI_assert(groups.total_size() == all_indices.size());
  BLI_assert(!all_weights || groups.total_size() == all_weights->size());

  to_static_type(src.type(), [&]<typename T>() {
    if constexpr (!std::is_same_v<T, std::string>) {
      threading::parallel_for(
          groups.index_range(),
          2048,
          [&](const IndexRange range) {
            const IndexMask dst_mask_slice = dst_mask.slice(range);
            if (all_weights) {
              mix_groups(src.typed<T>(),
                         groups.slice(range),
                         all_indices,
                         *all_weights,
                         dst_mask_slice,
                         dst.typed<T>());
            }
            else {
              mix_groups(src.typed<T>(),
                         groups.slice(range),
                         all_indices,
                         dst_mask_slice,
                         dst.typed<T>());
            }
          },
          threading::accumulated_task_sizes(
              [&](const IndexRange range) { return groups[range].size(); }));
    }
  });
}

void gather_mix(const GVArray &src,
                const Span<int> map,
                const Span<int> map2,
                const Span<float> factors,
                GMutableSpan dst)
{
  to_static_type(src.type(), [&]<typename T>() {
    gather_mix(src.typed<T>(), map, map2, factors, dst.typed<T>());
  });
}

}  // namespace blender::bke::attribute_math
