/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once
#include "BLI_array.hh"
#include "BLI_color_types.hh"
#include "BLI_hash.hh"
#include "BLI_string_ref.hh"
#include "GPU_vertex_buffer.hh"

namespace blender::draw {
struct MaterialAttributeColumn {
  std::string name;
  Array<ColorGeometry4f> values;
  bool curve_domain = false;
};
inline gpu::VertBufPtr
build_material_attribute_atlas(Span<MaterialAttributeColumn> columns) {
  int table_size = 1;
  while (table_size < columns.size() * 2) {
    table_size *= 2;
  }
  int64_t size = 1 + table_size * 2;
  for (const auto &column : columns) {
    size += column.values.size();
  }
  const GPUVertFormat format = GPU_vertformat_from_attribute(
      "material_attribute_atlas", gpu::VertAttrType::SFLOAT_32_32_32_32);
  gpu::VertBufPtr buffer(GPU_vertbuf_create_with_format_ex(
      format, GPU_USAGE_STATIC | GPU_USAGE_FLAG_BUFFER_TEXTURE_ONLY));
  GPU_vertbuf_data_alloc(*buffer, size);
  auto data = buffer->data<ColorGeometry4f>();
  data.fill(ColorGeometry4f(0.0f));
  data[0] = {float(table_size), 0, 0, 0};
  uint32_t offset = 1 + table_size * 2;
  for (const auto &column : columns) {
    const uint64_t hash = get_default_hash(StringRef(column.name));
    int slot = uint32_t(hash) & (table_size - 1);
    while (data[1 + 2 * slot].a != 0.0f) {
      slot = (slot + 1) & (table_size - 1);
    }
    data[1 + 2 * slot] = {float(hash & 0xFFFFFF),
                          float((hash >> 24) & 0xFFFFFF), float(hash >> 48), 1};
    data[2 + 2 * slot] = {float(offset & 0xFFFF), float(offset >> 16),
                          column.curve_domain ? 1.0f : 0.0f,
                          float(column.values.size())};
    data.slice(offset, column.values.size()).copy_from(column.values);
    offset += column.values.size();
  }
  return buffer;
}
} // namespace blender::draw
