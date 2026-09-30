/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Image Process COM backend for NodeCombineBundle:
 * collect Color/Float/Vector item images into the shared bundle maps cache and emit a
 * String key on the Bundle output so Bake Image can write any Combine Bundle. */

#include "BLI_array.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"

#include "DNA_node_types.h"

#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "NOD_geo_bundle.hh"
#include "NOD_image_points.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"

namespace blender::nodes::node_image_combine_bundle_com_cc {

using namespace blender::compositor;
using namespace blender::nodes::image_points;

NODE_STORAGE_FUNCS(NodeCombineBundle)

class CombineBundleCompositorOperation : public NodeOperation {
 public:
  CombineBundleCompositorOperation(Context &context, const bNode &node)
      : NodeOperation(context, node)
  {
    for (const bNodeSocket *sock : node.input_sockets()) {
      if (!sock->is_available() || sock->type == SOCK_CUSTOM) {
        continue;
      }
      if (!this->has_input(sock->identifier)) {
        continue;
      }
      InputDescriptor &desc = this->get_input_descriptor(sock->identifier);
      desc.skip_type_conversion = true;
      desc.realization_mode = InputRealizationMode::None;
    }
  }

  void execute() override
  {
    Result *bundle_out = nullptr;
    for (const bNodeSocket *sock : this->node().output_sockets()) {
      if (sock->type == SOCK_BUNDLE && sock->is_available() && this->has_result(sock->identifier))
      {
        bundle_out = &this->get_result(sock->identifier);
        break;
      }
    }

    const NodeCombineBundle &storage = node_storage(this->node());
    const int2 domain_size = math::max(this->context().get_compositing_domain().data_size,
                                       int2(1));

    StampAttrMaps maps;
    maps.size = domain_size;

    auto ensure_size = [&](const int2 size) {
      if (size.x > maps.size.x || size.y > maps.size.y) {
        maps.size = math::max(size, int2(1));
      }
    };

    auto result_to_rgba = [&](Result &image, const int2 size) -> Array<float4> {
      const int64_t n = int64_t(size.x) * size.y;
      Array<float4> rgba(n, float4(0.0f));
      if (!image.is_allocated() && !image.is_single_value()) {
        return rgba;
      }

      auto fill_color = [&](const Color &c) {
        rgba.fill(float4(c.r, c.g, c.b, c.a));
      };

      if (image.is_single_value()) {
        if (image.type() == ResultType::Color) {
          fill_color(image.get_single_value_default<Color>());
        }
        else if (image.type() == ResultType::Float) {
          const float v = image.get_single_value_default<float>();
          fill_color(Color(v, v, v, 1.0f));
        }
        else if (image.type() == ResultType::Float3) {
          const float3 v = image.get_single_value_default<float3>();
          fill_color(Color(v.x, v.y, v.z, 1.0f));
        }
        else if (image.type() == ResultType::Float4) {
          const float4 v = image.get_single_value_default<float4>();
          fill_color(Color(v.x, v.y, v.z, v.w));
        }
        return rgba;
      }

      Result cpu = this->context().use_gpu() ? image.download_to_cpu() : image;
      const bool own_cpu = this->context().use_gpu();
      const int2 src_size = math::max(cpu.domain().data_size, int2(1));
      const int64_t src_n = int64_t(src_size.x) * src_size.y;

      if (cpu.type() == ResultType::Color && cpu.cpu_data().data()) {
        const Color *src = static_cast<const Color *>(cpu.cpu_data().data());
        for (const int64_t p : IndexRange(math::min(n, src_n))) {
          rgba[p] = float4(src[p].r, src[p].g, src[p].b, src[p].a);
        }
      }
      else if (cpu.type() == ResultType::Float && cpu.cpu_data().data()) {
        const float *src = static_cast<const float *>(cpu.cpu_data().data());
        for (const int64_t p : IndexRange(math::min(n, src_n))) {
          const float v = src[p];
          rgba[p] = float4(v, v, v, 1.0f);
        }
      }
      else if (cpu.type() == ResultType::Float3 && cpu.cpu_data().data()) {
        const float3 *src = static_cast<const float3 *>(cpu.cpu_data().data());
        for (const int64_t p : IndexRange(math::min(n, src_n))) {
          rgba[p] = float4(src[p].x, src[p].y, src[p].z, 1.0f);
        }
      }
      else if (cpu.type() == ResultType::Float4 && cpu.cpu_data().data()) {
        const float4 *src = static_cast<const float4 *>(cpu.cpu_data().data());
        for (const int64_t p : IndexRange(math::min(n, src_n))) {
          rgba[p] = src[p];
        }
      }

      if (own_cpu) {
        cpu.release();
      }
      return rgba;
    };

    for (const int i : IndexRange(storage.items_num)) {
      const NodeCombineBundleItem &item = storage.items[i];
      if (!item.name || item.name[0] == '\0') {
        continue;
      }
      if (!ELEM(item.socket_type, SOCK_RGBA, SOCK_FLOAT, SOCK_VECTOR)) {
        continue;
      }
      const std::string identifier = CombineBundleItemsAccessor::socket_identifier_for_item(item);
      if (!this->has_input(identifier)) {
        continue;
      }
      Result &image = this->get_input(identifier);
      if (!image.is_allocated() && !image.is_single_value()) {
        continue;
      }
      int2 size = domain_size;
      if (!image.is_single_value() && image.is_allocated()) {
        size = math::max(image.domain().data_size, int2(1));
      }
      ensure_size(size);
      maps.maps.add_overwrite(item.name, result_to_rgba(image, maps.size));
    }

    const std::string key = stamp_node_cache_key(this->node());
    if (StampAttrMapsCache *cache = active_stamp_attr_cache()) {
      if (!maps.maps.is_empty()) {
        cache->store(key, std::move(maps));
      }
    }

    if (bundle_out) {
      bundle_out->allocate_single_value();
      if (bundle_out->type() == ResultType::String) {
        bundle_out->set_single_value(key);
      }
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new CombineBundleCompositorOperation(context, node);
}

/** Install Image Process COM for the shared NodeCombineBundle type (after type registration). */
void register_combine_bundle_image_com()
{
  bke::bNodeType *ntype = bke::node_type_find("NodeCombineBundle"_ustr);
  if (!ntype) {
    return;
  }
  ntype->get_compositor_operation = get_compositor_operation;
}

}  // namespace blender::nodes::node_image_combine_bundle_com_cc

namespace blender {

void register_image_combine_bundle_compositor()
{
  nodes::node_image_combine_bundle_com_cc::register_combine_bundle_image_com();
}

}  // namespace blender
