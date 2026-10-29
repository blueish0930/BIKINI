/* SPDX-FileCopyrightText: 2021 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#include "BLI_string.hh"
#include "BLI_hash.hh"
#include "BLI_shader_matrix_attribute.hh"
#include "BLI_math_matrix_types.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_legacy_convert.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.hh"
#include "BKE_shader_matrix_attribute.hh"

#include "attribute_convert.hh"
#include "draw_attributes.hh"
#include "draw_subdivision.hh"
#include "extract_mesh.hh"

#include "GPU_vertex_buffer.hh"

namespace blender::draw {

/* ---------------------------------------------------------------------- */
/** \name Extract Attributes
 * \{ */

static GPUVertFormat attribute_vbo_format(const MeshRenderData &mr,
                                          const StringRef name,
                                          const bke::AttrType type)
{
  char attr_name[32], attr_safe_name[GPU_MAX_SAFE_ATTR_NAME];
  GPU_vertformat_safe_attr_name(name, attr_safe_name);
  /* Attributes use auto-name. */
  SNPRINTF(attr_name, "a%s", attr_safe_name);

  GPUVertFormat format = init_format_for_attribute(type, attr_name);
  GPU_vertformat_deinterleave(&format);

  if (mr.active_color_name && name == mr.active_color_name) {
    GPU_vertformat_alias_add(&format, "ac");
  }
  if (mr.default_color_name && name == mr.default_color_name) {
    GPU_vertformat_alias_add(&format, "c");
  }
  return format;
}

static gpu::VertBufPtr vbo_create(const GPUVertFormat &format, const int size)
{
  gpu::VertBufPtr vbo = gpu::VertBufPtr(GPU_vertbuf_create_with_format(format));
  GPU_vertbuf_data_alloc(*vbo, size);
  return vbo;
}

static int domain_size(const MeshRenderData &mr, const bke::AttrDomain domain)
{
  switch (domain) {
    case bke::AttrDomain::Point:
      return mr.verts_num;
    case bke::AttrDomain::Edge:
      return mr.edges_num;
    case bke::AttrDomain::Face:
      return mr.faces_num;
    case bke::AttrDomain::Corner:
      return mr.corners_num;
    default:
      BLI_assert_unreachable();
      return 0;
  }
}

/**
 * Copy values from vertices, edges, or faces to face corners on the GPU.
 * \param src: A value for every element in \a domain, with the given format.
 */
static gpu::VertBufPtr copy_to_corners_gpu(const MeshRenderData &mr,
                                           MeshBufferCache &cache,
                                           const bke::AttrDomain domain,
                                           gpu::VertBuf &src,
                                           const GPUVertFormat &format)
{
  gpu::VertBufPtr vbo = gpu::VertBufPtr(GPU_vertbuf_create_on_device(format, mr.corners_num));
  switch (domain) {
    case bke::AttrDomain::Point:
      gather_vert_to_corner_gpu(mr, cache, src, *vbo);
      break;
    case bke::AttrDomain::Edge:
      gather_edge_to_corner_gpu(mr, cache, src, *vbo);
      break;
    case bke::AttrDomain::Face:
      scatter_face_to_corner_gpu(mr, cache, src, *vbo);
      break;
    default:
      BLI_assert_unreachable();
  }
  return vbo;
}

/** Convert BMesh attribute values to the GPU format, in the index order of their domain. */
static void extract_data_bmesh(const MeshRenderData &mr,
                               const BMDataLayerLookup &attr,
                               gpu::VertBuf &vbo,
                               const int matrix_column = 0)
{
  BMesh &bm = *mr.bm;
  bke::attribute_math::to_static_type(attr.type, [&]<typename T>() {
    using Converter = AttributeConverter<T>;
    using VBOType = std::conditional_t<std::is_same_v<T,float4x4>,ColorGeometry4f,
                                       typename Converter::VBOType>;
    if constexpr (!std::is_void_v<VBOType>) {
      MutableSpan data = vbo.data<VBOType>();
      const auto convert = [&](const BMHeader &head) {
        const T &value = *static_cast<const T *>(POINTER_OFFSET(head.data, attr.offset));
        if constexpr (std::is_same_v<T,float4x4>) {
          const float4 column = value[matrix_column];
          return ColorGeometry4f(column.x,column.y,column.z,column.w);
        }
        else {
          return Converter::convert(value);
        }
      };
      switch (attr.domain) {
        case bke::AttrDomain::Point:
          threading::parallel_for(IndexRange(bm.totvert), 4096, [&](const IndexRange range) {
            for (const int i : range) {
              data[i] = convert(BM_vert_at_index(&bm, i)->head);
            }
          });
          break;
        case bke::AttrDomain::Edge:
          threading::parallel_for(IndexRange(bm.totedge), 4096, [&](const IndexRange range) {
            for (const int i : range) {
              data[i] = convert(BM_edge_at_index(&bm, i)->head);
            }
          });
          break;
        case bke::AttrDomain::Face:
          threading::parallel_for(IndexRange(bm.totface), 4096, [&](const IndexRange range) {
            for (const int i : range) {
              data[i] = convert(BM_face_at_index(&bm, i)->head);
            }
          });
          break;
        case bke::AttrDomain::Corner:
          threading::parallel_for(IndexRange(bm.totface), 2048, [&](const IndexRange range) {
            for (const int face_index : range) {
              const BMFace &face = *BM_face_at_index(&bm, face_index);
              const BMLoop *loop = BM_FACE_FIRST_LOOP(&face);
              for ([[maybe_unused]] const int i : IndexRange(face.len)) {
                data[BM_elem_index_get(loop)] = convert(loop->head);
                loop = loop->next;
              }
            }
          });
          break;
        default:
          BLI_assert_unreachable();
      }
    }
  });
}

static gpu::VertBufPtr extract_attribute_data(const MeshRenderData &mr,
                                              MeshBufferCache &cache,
                                              const BMDataLayerLookup &attr,
                                              const GPUVertFormat &format,
                                              const int matrix_column = 0)
{
  gpu::VertBufPtr src = vbo_create(format, domain_size(mr, attr.domain));
  extract_data_bmesh(mr, attr, *src, matrix_column);
  if (attr.domain == bke::AttrDomain::Corner) {
    return src;
  }
  return copy_to_corners_gpu(mr, cache, attr.domain, *src, format);
}

static gpu::VertBufPtr extract_attribute_data(const MeshRenderData &mr,
                                              MeshBufferCache &cache,
                                              const bke::GAttributeReader &attr,
                                              const GPUVertFormat &format)
{
  if (attr.varray.is_single()) {
    gpu::VertBufPtr vbo = vbo_create(format, mr.corners_num);
    bke::attribute_math::to_static_type(attr.varray.type(), [&]<typename T>() {
      const VArray<T> &src = attr.varray.typed<T>();
      if constexpr (!std::is_void_v<typename AttributeConverter<T>::VBOType>) {
        using Converter = AttributeConverter<T>;
        using VBOType = typename Converter::VBOType;
        MutableSpan data = vbo->data<VBOType>();
        data.fill(Converter::convert(src.get_internal_single()));
      }
    });
    return vbo;
  }
  gpu::VertBufPtr src = vbo_create(format, domain_size(mr, attr.domain));
  vertbuf_data_extract_direct(GVArraySpan(*attr), *src);
  if (attr.domain == bke::AttrDomain::Corner) {
    return src;
  }
  return copy_to_corners_gpu(mr, cache, attr.domain, *src, format);
}

static bke::GAttributeReader lookup_shader_attribute(const bke::AttributeAccessor &attributes,
                                                    const StringRef name)
{
  if (const auto column = shader_attribute_request(name)) {
    const auto source = bke::lookup_shader_attribute_request(attributes, name);
    if (!source) {
      return {VArray<ColorGeometry4f>::from_single(ColorGeometry4f(0.0f),
                                                 attributes.domain_size(bke::AttrDomain::Point)),
              bke::AttrDomain::Point,
              nullptr};
    }
    return source;
  }
  return attributes.lookup(name);
}

gpu::VertBufPtr extract_attribute(const MeshRenderData &mr,
                                  MeshBufferCache &cache,
                                  const StringRef name)
{
  if (mr.extract_type == MeshExtractType::BMesh) {
    const auto column = shader_attribute_request(name);
    const BMDataLayerLookup attr = BM_data_layer_lookup(*mr.bm, column ? column->name : name);
    if (column && (!attr || attr.type != (column->kind == ShaderAttributeRequestKind::Rotation ?
                                           bke::AttrType::Quaternion : bke::AttrType::Float4x4))) {
      gpu::VertBufPtr vbo = vbo_create(
          attribute_vbo_format(mr, name, bke::AttrType::ColorFloat), mr.corners_num);
      vbo->data<ColorGeometry4f>().fill(ColorGeometry4f(0.0f));
      return vbo;
    }
    if (!attr) {
      return {};
    }
    return extract_attribute_data(mr, cache, attr,
        attribute_vbo_format(mr,name,column ? bke::AttrType::ColorFloat : attr.type),
        column ? column->column : 0);
  }
  const bke::AttributeAccessor attributes = mr.mesh->attributes();
  const bke::GAttributeReader attr = lookup_shader_attribute(attributes, name);
  if (!attr) {
    return {};
  }
  const bke::AttrType type = bke::cpp_type_to_attribute_type(attr.varray.type());
  return extract_attribute_data(mr, cache, attr, attribute_vbo_format(mr, name, type));
}

gpu::VertBufPtr extract_attribute_subdiv(const MeshRenderData &mr,
                                         MeshBufferCache &cache,
                                         const DRWSubdivCache &subdiv_cache,
                                         const StringRef name)
{
  BLI_assert(mr.corners_num == subdiv_cache.mesh->corners_num);

  /* Extract the attribute on the coarse face corners, to be interpolated to the subdivided face
   * corners. The compute shader only expects floats. */
  gpu::VertBufPtr coarse_vbo;
  bke::AttrType type;
  if (mr.extract_type == MeshExtractType::BMesh) {
    const auto column = shader_attribute_request(name);
    const BMDataLayerLookup attr = BM_data_layer_lookup(*mr.bm, column ? column->name : name);
    if (column && (!attr || attr.type != (column->kind == ShaderAttributeRequestKind::Rotation ?
                                           bke::AttrType::Quaternion : bke::AttrType::Float4x4))) {
      gpu::VertBufPtr vbo = vbo_create(
          attribute_vbo_format(mr, name, bke::AttrType::ColorFloat),
          subdiv_cache.num_subdiv_loops);
      vbo->data<ColorGeometry4f>().fill(ColorGeometry4f(0.0f));
      return vbo;
    }
    if (!attr) {
      return {};
    }
    type = column ? bke::AttrType::ColorFloat : attr.type;
    coarse_vbo = extract_attribute_data(mr, cache, attr, init_format_for_attribute(type, "data"),
                                        column ? column->column : 0);
  }
  else {
    const bke::AttributeAccessor attributes = mr.mesh->attributes();
    const bke::GAttributeReader attr = lookup_shader_attribute(attributes, name);
    if (!attr) {
      return {};
    }
    type = bke::cpp_type_to_attribute_type(attr.varray.type());
    coarse_vbo = extract_attribute_data(mr, cache, attr, init_format_for_attribute(type, "data"));
  }

  gpu::VertBufPtr vbo = gpu::VertBufPtr(GPU_vertbuf_create_on_device(
      attribute_vbo_format(mr, name, type), subdiv_cache.num_subdiv_loops));

  bke::attribute_math::to_static_type(type, [&]<typename T>() {
    using Converter = AttributeConverter<T>;
    if constexpr (!std::is_void_v<typename Converter::VBOType>) {
      draw_subdiv_interp_custom_data(subdiv_cache,
                                     *coarse_vbo,
                                     *vbo,
                                     Converter::gpu_component_type,
                                     Converter::gpu_component_len,
                                     0);
    }
  });

  return vbo;
}

gpu::VertBufPtr extract_attr_viewer(const MeshRenderData &mr, MeshBufferCache &cache)
{
  static const GPUVertFormat format = GPU_vertformat_from_attribute(
      "attribute_value", gpu::VertAttrType::SFLOAT_32_32_32_32);
  const bke::AttributeAccessor attributes = mr.mesh->attributes();
  /* Prefer Geometry Nodes Debug `.debug_color` over official Viewer `.viewer`.
   * Debug writes a dedicated composite layer for viewport color; when both exist
   * (Viewer preview + Debug chain), Debug is the intentional surface preview.
   * Fall back to `.viewer`, then magenta default if neither is present. */
  StringRefNull attr_name = ".viewer";
  if (attributes.contains(".debug_color")) {
    attr_name = ".debug_color";
  }
  bke::GAttributeReader attr = attributes.lookup(
      attr_name, std::nullopt, bke::AttrType::ColorFloat);
  if (!attr) {
    gpu::VertBufPtr vbo = vbo_create(format, mr.corners_num);
    vbo->data<ColorGeometry4f>().fill(ColorGeometry4f(1.0f, 0.0f, 1.0f, 1.0f));
    return vbo;
  }

  /* The GPU edge to corner gather only uses one edge per corner. Interpolate on the CPU instead,
   * mixing both face edges adjacent to each corner, so the value shows along the edge. */
  if (attr.domain == bke::AttrDomain::Edge) {
    attr = attributes.lookup(attr_name, bke::AttrDomain::Corner, bke::AttrType::ColorFloat);
  }

  return extract_attribute_data(mr, cache, attr, format);
}

gpu::VertBufPtr extract_material_attributes(const MeshRenderData &mr,
                                            MeshBufferCache &cache,
                                            Span<std::string> names,
                                            const DRWSubdivCache *subdiv_cache)
{
  int table_size = 1;
  while (table_size < names.size() * 2) { table_size *= 2; }
  const int corner_count = subdiv_cache ? subdiv_cache->num_subdiv_loops : mr.corners_num;
  const int header_size = 1 + table_size * 2;
  const GPUVertFormat format = GPU_vertformat_from_attribute(
      "material_attribute_atlas", gpu::VertAttrType::SFLOAT_32_32_32_32);
  gpu::VertBufPtr atlas(GPU_vertbuf_create_with_format_ex(
      format, GPU_USAGE_STATIC | GPU_USAGE_FLAG_BUFFER_TEXTURE_ONLY));
  GPU_vertbuf_data_alloc(*atlas, header_size + int64_t(names.size()) * corner_count);
  MutableSpan<ColorGeometry4f> data = atlas->data<ColorGeometry4f>();
  data.fill(ColorGeometry4f(0.0f));
  data[0] = ColorGeometry4f(float(table_size), float(corner_count), 0.0f, 0.0f);
  for (const int i : names.index_range()) {
    const uint64_t hash = get_default_hash(StringRef(names[i]));
    int slot = uint32_t(hash) & (table_size - 1);
    while (data[1 + slot * 2].a != 0.0f) { slot = (slot + 1) & (table_size - 1); }
    data[1 + slot * 2] = ColorGeometry4f(float(hash & 0xFFFFFF),
        float((hash >> 24) & 0xFFFFFF), float(hash >> 48), 1.0f);
    const uint32_t offset = header_size + int64_t(i) * corner_count;
    data[2 + slot * 2] = ColorGeometry4f(float(offset & 0xFFFF), float(offset >> 16), 0, 0);
    const auto column = shader_attribute_request(names[i]);
    Array<ColorGeometry4f> coarse(mr.corners_num,
                                  ColorGeometry4f(0, 0, 0, column ? 0.0f : 1.0f));
    if (mr.extract_type == MeshExtractType::BMesh) {
      const StringRef source_name = names[i];
      const BMDataLayerLookup attr = BM_data_layer_lookup(*mr.bm, column ? column->name : source_name);
      if (attr && (!column || attr.type == (column->kind == ShaderAttributeRequestKind::Rotation ?
                                              bke::AttrType::Quaternion : bke::AttrType::Float4x4))) {
        bke::attribute_math::to_static_type(attr.type, [&]<typename T>() {
          auto convert = [&](const BMHeader &head) -> ColorGeometry4f {
            const T &v = *static_cast<const T *>(POINTER_OFFSET(head.data, attr.offset));
            if constexpr (std::is_same_v<T,float4x4>) {
              if (column) { const float4 c=v[column->column]; return {c.x,c.y,c.z,c.w}; }
            }
            else if constexpr (std::is_same_v<T,ColorGeometry4f>) { return v; }
            else if constexpr (std::is_same_v<T,math::Quaternion>) { return {v.w,v.x,v.y,v.z}; }
            else if constexpr (std::is_same_v<T,float3>) { return {v.x,v.y,v.z,1}; }
            else if constexpr (std::is_same_v<T,float2>) { return {v.x,v.y,0,1}; }
            else if constexpr (std::is_convertible_v<T,float>) { float f=float(v); return {f,f,f,1}; }
            return {0,0,0,1};
          };
          for (int face_index=0;face_index<mr.bm->totface;face_index++) {
            BMFace *face=BM_face_at_index(mr.bm,face_index);BMLoop *loop=BM_FACE_FIRST_LOOP(face);
            for (int j=0;j<face->len;j++,loop=loop->next) {
              const BMHeader &head=attr.domain==bke::AttrDomain::Point ? loop->v->head :
                                   attr.domain==bke::AttrDomain::Edge ? loop->e->head :
                                   attr.domain==bke::AttrDomain::Face ? face->head : loop->head;
              coarse[BM_elem_index_get(loop)]=convert(head);
            }
          }
        });
      }
    }
    else {
      const bke::AttributeAccessor attributes=mr.mesh->attributes();
      if (column) {
        const bke::GAttributeReader attr=lookup_shader_attribute(attributes,names[i]);
        if (attr) {
          const VArray<ColorGeometry4f> values=attributes.adapt_domain(
              attr.varray.typed<ColorGeometry4f>(),attr.domain,bke::AttrDomain::Corner);
          if (values) {
            values.materialize(coarse);
          }
        }
      }
      else if (const auto attr=attributes.lookup<ColorGeometry4f>(names[i],bke::AttrDomain::Corner)) {
        attr.varray.materialize(coarse);
      }
      else if (const auto attr=attributes.lookup<float2>(names[i],bke::AttrDomain::Corner)) {
        for(int j=0;j<coarse.size();j++){float2 v=attr.varray[j];coarse[j]={v.x,v.y,0,1};}
      }
    }
    MutableSpan<ColorGeometry4f> values=data.slice(offset,corner_count);
    if (!subdiv_cache) { values.copy_from(coarse); }
    else {
      const GPUVertFormat rgba=GPU_vertformat_from_attribute("data",gpu::VertAttrType::SFLOAT_32_32_32_32);
      gpu::VertBufPtr src=vbo_create(rgba,coarse.size());src->data<ColorGeometry4f>().copy_from(coarse);
      gpu::VertBufPtr dst(GPU_vertbuf_create_on_device(rgba,corner_count));
      draw_subdiv_interp_custom_data(*subdiv_cache,*src,*dst,GPU_COMP_F32,4,0);
      GPU_vertbuf_read(dst.get(),values.data());
    }
  }
  UNUSED_VARS(cache);
  return atlas;
}

/** \} */

}  // namespace blender::draw
