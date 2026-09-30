/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cfloat>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>

#include "MEM_guardedalloc.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_filters.hh"
#include "BKE_attribute_math.hh"
#include "BKE_bvh.hh"
#include "BKE_bvhutils.hh"
#include "BKE_curves.hh"
#include "BKE_customdata.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"
#include "BKE_pointcloud.hh"

#include "BLI_array.hh"
#include "BLI_bounds.hh"
#include "BLI_color_types.hh"
#include "BLI_kdopbvh.hh"
#include "BLI_cpp_type.hh"
#include "BLI_execution_mode.hh"
#include "BLI_index_mask.hh"
#include "BLI_implicit_sharing.hh"
#include "BLI_kdtree.hh"
#include "BLI_map.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_offset_indices.hh"
#include "BLI_virtual_array.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_resource_scope.hh"
#include "BLI_set.hh"
#include "BLI_task.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_node_types.h"
#include "DNA_pointcloud_types.h"

#include "GEO_mesh_copy_selection.hh"
#include "GEO_mesh_selection.hh"

#include "FN_field.hh"
#include "FN_field_evaluation.hh"

#include "GEO_foreach_geometry.hh"

#include "NOD_vex.hh"
#include "vex_gpu.hh"
#include "vex_program.hh"

namespace blender::nodes::vex {
namespace {

bke::AttrDomain to_attr_domain(const Domain domain)
{
  switch (domain) {
    case Domain::Edge:
      return bke::AttrDomain::Edge;
    case Domain::Face:
      return bke::AttrDomain::Face;
    case Domain::Corner:
      return bke::AttrDomain::Corner;
    case Domain::Instance:
      return bke::AttrDomain::Instance;
    case Domain::Curve:
      return bke::AttrDomain::Curve;
    case Domain::Point:
      return bke::AttrDomain::Point;
  }
  return bke::AttrDomain::Point;
}

bke::AttrType to_attr_type(const Type type)
{
  switch (type) {
    case Type::Bool:
      return bke::AttrType::Bool;
    case Type::Int:
      return bke::AttrType::Int32;
    case Type::Vector2:
      return bke::AttrType::Float2;
    case Type::Vector:
      return bke::AttrType::Float3;
    case Type::Vector4:
      return bke::AttrType::Float4;
    case Type::Matrix:
      return bke::AttrType::Float4x4;
    case Type::Color:
      return bke::AttrType::ColorFloat;
    case Type::Rotation:
      return bke::AttrType::Quaternion;
    case Type::Matrix2:
    case Type::Matrix3:
    case Type::IntArray:
    case Type::FloatArray:
    case Type::VecArray:
    case Type::StringArray:
    case Type::MatArray:
    case Type::RayArray:
      return bke::AttrType::WrangleArray;
    case Type::String:
      return bke::AttrType::String;
    case Type::Float:
    default:
      return bke::AttrType::Float;
  }
}

Value value_from_varray(const GVArray &varray, const int index, Vector<std::string> *interned)
{
  if (!varray || index < 0 || index >= varray.size()) {
    return Value::from_float(0.0f);
  }
  const CPPType &type = varray.type();
  if (type.is<float>()) {
    return Value::from_float(varray.typed<float>()[index]);
  }
  if (type.is<int>()) {
    return Value::from_int(varray.typed<int>()[index]);
  }
  if (type.is<bool>()) {
    return Value::from_bool(varray.typed<bool>()[index]);
  }
  if (type.is<float3>()) {
    return Value::from_vec(varray.typed<float3>()[index]);
  }
  if (type.is<float2>()) {
    return Value::from_vec2(varray.typed<float2>()[index]);
  }
  if (type.is<float4>()) {
    return Value::from_vec4(varray.typed<float4>()[index], Type::Vector4);
  }
  if (type.is<float4x4>()) {
    return Value::from_matrix(varray.typed<float4x4>()[index]);
  }
  if (type.is<math::Quaternion>()) {
    const math::Quaternion q = varray.typed<math::Quaternion>()[index];
    return Value::from_vec4(float4(q.x, q.y, q.z, q.w), Type::Rotation);
  }
  if (type.is<ColorGeometry4f>()) {
    const ColorGeometry4f c = varray.typed<ColorGeometry4f>()[index];
    return Value::from_vec4(float4(c.r, c.g, c.b, c.a), Type::Color);
  }
  if (type.is<std::string>()) {
    if (interned) {
      interned->append(varray.typed<std::string>()[index]);
      return Value::from_str_i(100000 + int(interned->size()) - 1);
    }
    return Value::from_str_i(-1);
  }
  if (type.is<MStringProperty>()) {
    const MStringProperty &p = varray.typed<MStringProperty>()[index];
    if (interned) {
      interned->append(std::string(p.s, uint8_t(p.s_len)));
      return Value::from_str_i(100000 + int(interned->size()) - 1);
    }
    return Value::from_str_i(-1);
  }
  if (type.is<bke::WrangleArrayValue>()) {
    return packed_to_value(varray.typed<bke::WrangleArrayValue>()[index]);
  }
  return Value::from_float(0.0f);
}

struct ElemUser {
  struct ElementSampleCache {
    int geo = 0;
    int domain = 0;
    std::string name;
    GVArray values;
    AttrRT direct;
    Array<bool> owned_b;
    Array<int> owned_i;
    Array<float> owned_f;
    Array<float2> owned_v2;
    Array<float3> owned_v3;
    Array<float4> owned_v4;
    Array<ColorGeometry4f> owned_color;
  };
  Vector<const bke::GeometrySet *> geos;
  const bke::GeometrySet *self = nullptr; /* geometry currently being wrangled (geo 0) */
  Vector<std::string> parm_names;
  Vector<GVArray> parm_arrays;
  Vector<std::string> interned_strings;
  Span<AttrInfo> attr_infos;
  Span<AttrRT> sample_attrs; /* working arrays `point(0, …)` reads (Jacobi snapshot) */
  Vector<ElementSampleCache> element_samples;
  bke::AttrDomain attr_domain = bke::AttrDomain::Point;
};

static std::string sample_attr_name(StringRef name)
{
  if (name.size() >= 2 && ((name[0] == '"' && name[name.size() - 1] == '"') ||
                           (name[0] == '\'' && name[name.size() - 1] == '\'')))
  {
    name = name.substr(1, name.size() - 2);
  }
  if (name.is_empty() || name == "P" || name == "p" || name == "vector") {
    return "position";
  }
  if (name == "N" || name == "n") {
    return "normal";
  }
  if (name == "Cd") {
    return "color";
  }
  return std::string(name);
}

static void prewarm_element_samples(const Program &program, ElemUser &user)
{
  user.element_samples.clear();
  for (const Program::ElementSample &sample : program.element_samples) {
    ElemUser::ElementSampleCache cache;
    cache.geo = sample.geo;
    cache.domain = sample.domain;
    cache.name = sample.name;
    const bke::GeometrySet *geo = nullptr;
    if (sample.geo >= 0 && sample.geo < user.geos.size()) {
      geo = user.geos[sample.geo];
    }
    if (geo == nullptr) {
      user.element_samples.append(std::move(cache));
      continue;
    }
    bke::AttrDomain domain = bke::AttrDomain::Point;
    switch (sample.domain) {
      case 1:
        domain = bke::AttrDomain::Edge;
        break;
      case 2:
        domain = bke::AttrDomain::Face;
        break;
      case 3:
        domain = bke::AttrDomain::Corner;
        break;
      case 4:
        domain = bke::AttrDomain::Instance;
        break;
      case 5:
        domain = bke::AttrDomain::Curve;
        break;
      default:
        break;
    }
    bke::GAttributeReader reader;
    if (const Mesh *mesh = geo->get_mesh()) {
      if (ELEM(domain,
               bke::AttrDomain::Point,
               bke::AttrDomain::Edge,
               bke::AttrDomain::Face,
               bke::AttrDomain::Corner))
      {
        reader = mesh->attributes().lookup(sample.name, domain);
      }
    }
    if (!reader && domain == bke::AttrDomain::Point) {
      if (const PointCloud *pointcloud = geo->get_pointcloud()) {
        reader = pointcloud->attributes().lookup(sample.name, domain);
      }
    }
    if (!reader && ELEM(domain, bke::AttrDomain::Point, bke::AttrDomain::Curve)) {
      if (const Curves *curves = geo->get_curves()) {
        reader = curves->geometry.wrap().attributes().lookup(sample.name, domain);
      }
    }
    if (!reader && domain == bke::AttrDomain::Instance) {
      if (const bke::Instances *instances = geo->get_instances()) {
        reader = instances->attributes().lookup(sample.name, domain);
      }
    }
    if (!reader) {
      user.element_samples.append(std::move(cache));
      continue;
    }
    cache.values = std::move(reader.varray);
    cache.direct.size = int(cache.values.size());
    if (cache.values.type().is<bool>()) {
      cache.direct.type = Type::Bool;
      if (cache.values.is_span()) {
        cache.direct.rb = cache.values.get_internal_span().typed<bool>().data();
      }
      else {
        cache.owned_b.reinitialize(cache.direct.size);
        cache.values.typed<bool>().materialize(cache.owned_b.as_mutable_span());
        cache.direct.rb = cache.owned_b.data();
      }
    }
    else if (cache.values.type().is<int>()) {
      cache.direct.type = Type::Int;
      if (cache.values.is_span()) {
        cache.direct.ri = cache.values.get_internal_span().typed<int>().data();
      }
      else {
        cache.owned_i.reinitialize(cache.direct.size);
        cache.values.typed<int>().materialize(cache.owned_i.as_mutable_span());
        cache.direct.ri = cache.owned_i.data();
      }
    }
    else if (cache.values.type().is<float>()) {
      cache.direct.type = Type::Float;
      if (cache.values.is_span()) {
        cache.direct.rf = cache.values.get_internal_span().typed<float>().data();
      }
      else {
        cache.owned_f.reinitialize(cache.direct.size);
        cache.values.typed<float>().materialize(cache.owned_f.as_mutable_span());
        cache.direct.rf = cache.owned_f.data();
      }
    }
    else if (cache.values.type().is<float2>()) {
      cache.direct.type = Type::Vector2;
      if (cache.values.is_span()) {
        cache.direct.r2 = cache.values.get_internal_span().typed<float2>().data();
      }
      else {
        cache.owned_v2.reinitialize(cache.direct.size);
        cache.values.typed<float2>().materialize(cache.owned_v2.as_mutable_span());
        cache.direct.r2 = cache.owned_v2.data();
      }
    }
    else if (cache.values.type().is<float3>()) {
      cache.direct.type = Type::Vector;
      if (cache.values.is_span()) {
        cache.direct.rv = cache.values.get_internal_span().typed<float3>().data();
      }
      else {
        cache.owned_v3.reinitialize(cache.direct.size);
        cache.values.typed<float3>().materialize(cache.owned_v3.as_mutable_span());
        cache.direct.rv = cache.owned_v3.data();
      }
    }
    else if (cache.values.type().is<float4>()) {
      cache.direct.type = Type::Vector4;
      if (cache.values.is_span()) {
        cache.direct.r4 = cache.values.get_internal_span().typed<float4>().data();
      }
      else {
        cache.owned_v4.reinitialize(cache.direct.size);
        cache.values.typed<float4>().materialize(cache.owned_v4.as_mutable_span());
        cache.direct.r4 = cache.owned_v4.data();
      }
    }
    else if (cache.values.type().is<ColorGeometry4f>()) {
      cache.direct.type = Type::Color;
      if (cache.values.is_span()) {
        cache.direct.r4 = reinterpret_cast<const float4 *>(
            cache.values.get_internal_span().typed<ColorGeometry4f>().data());
      }
      else {
        cache.owned_color.reinitialize(cache.direct.size);
        cache.values.typed<ColorGeometry4f>().materialize(cache.owned_color.as_mutable_span());
        cache.direct.r4 = reinterpret_cast<const float4 *>(cache.owned_color.data());
      }
    }
    user.element_samples.append(std::move(cache));
  }
}

static Value value_from_sample_cache(ElemUser::ElementSampleCache &sample,
                                     const int index,
                                     Vector<std::string> *interned)
{
  const AttrRT &a = sample.direct;
  if (index >= 0 && index < a.size) {
    switch (a.type) {
      case Type::Bool:
        if (a.rb) {
          return Value::from_bool(a.rb[index]);
        }
        break;
      case Type::Int:
        if (a.ri) {
          return Value::from_int(a.ri[index]);
        }
        break;
      case Type::Float:
        if (a.rf) {
          return Value::from_float(a.rf[index]);
        }
        break;
      case Type::Vector2:
        if (a.r2) {
          return Value::from_vec2(a.r2[index]);
        }
        break;
      case Type::Vector:
        if (a.rv) {
          return Value::from_vec(a.rv[index]);
        }
        break;
      case Type::Vector4:
        if (a.r4) {
          return Value::from_vec4(a.r4[index], Type::Vector4);
        }
        break;
      case Type::Color:
        if (a.r4) {
          return Value::from_vec4(a.r4[index], Type::Color);
        }
        break;
      default:
        break;
    }
  }
  return value_from_varray(sample.values, index, interned);
}

static bool is_position_name(const StringRef n)
{
  return n == "position" || n == "P" || n == "p" || n == "vector";
}

static bool is_normal_name(const StringRef n)
{
  return n == "normal" || n == "N" || n == "n";
}

static Value read_named_attr(const bke::AttributeAccessor &attrs,
                             const StringRef name,
                             const std::optional<bke::AttrDomain> domain,
                             const int index,
                             Vector<std::string> *interned)
{
  const bke::GAttributeReader reader = domain ? attrs.lookup(name, *domain) : attrs.lookup(name);
  if (!reader) {
    Value miss;
    miss.type = Type::Void;
    return miss;
  }
  return value_from_varray(reader.varray, index, interned);
}

static Value sample_mesh(const Mesh &mesh,
                         const bke::AttrDomain ad,
                         const StringRef lookup,
                         const int index,
                         Vector<std::string> *interned)
{
  const Span<float3> pos = mesh.vert_positions();
  if (is_position_name(lookup)) {
    if (ad == bke::AttrDomain::Point) {
      if (index >= 0 && index < pos.size()) {
        return Value::from_vec(pos[index]);
      }
    }
    else if (ad == bke::AttrDomain::Edge) {
      const Span<int2> edges = mesh.edges();
      if (index >= 0 && index < edges.size()) {
        const int2 e = edges[index];
        if (e[0] >= 0 && e[0] < pos.size() && e[1] >= 0 && e[1] < pos.size()) {
          return Value::from_vec((pos[e[0]] + pos[e[1]]) * 0.5f);
        }
      }
    }
    else if (ad == bke::AttrDomain::Face) {
      if (index >= 0 && index < mesh.faces_num) {
        const IndexRange face = mesh.faces()[index];
        const Span<int> cv = mesh.corner_verts();
        float3 sum(0.0f);
        int n = 0;
        for (const int c : face) {
          const int v = cv[c];
          if (v >= 0 && v < pos.size()) {
            sum += pos[v];
            n++;
          }
        }
        if (n > 0) {
          return Value::from_vec(sum / float(n));
        }
      }
    }
    else if (ad == bke::AttrDomain::Corner) {
      const Span<int> cv = mesh.corner_verts();
      if (index >= 0 && index < cv.size()) {
        const int v = cv[index];
        if (v >= 0 && v < pos.size()) {
          return Value::from_vec(pos[v]);
        }
      }
    }
  }
  if (is_normal_name(lookup) && ad == bke::AttrDomain::Point) {
    const Span<float3> nrm = mesh.vert_normals();
    if (index >= 0 && index < nrm.size()) {
      return Value::from_vec(nrm[index]);
    }
  }
  Value v = read_named_attr(mesh.attributes(), lookup, ad, index, interned);
  if (v.type != Type::Void) {
    return v;
  }
  return read_named_attr(mesh.attributes(), lookup, std::nullopt, index, interned);
}

Value load_elem_fn(void *user,
                   const int geo_index,
                   const int domain,
                   const StringRef name,
                   const int index,
                   const Type /*expect*/,
                   std::string & /*error*/)
{
  auto *u = static_cast<ElemUser *>(user);
  if (!u) {
    return Value::from_float(0.0f);
  }
  const std::string lookup_early = sample_attr_name(name);
  int bind_domain = 0;
  switch (u->attr_domain) {
    case bke::AttrDomain::Edge:
      bind_domain = 1;
      break;
    case bke::AttrDomain::Face:
      bind_domain = 2;
      break;
    case bke::AttrDomain::Corner:
      bind_domain = 3;
      break;
    case bke::AttrDomain::Instance:
      bind_domain = 4;
      break;
    case bke::AttrDomain::Curve:
      bind_domain = 5;
      break;
    default:
      break;
  }
  auto find_prebound_sample = [&]() -> std::optional<Value> {
    for (const ElemUser::ElementSampleCache &sample : u->element_samples) {
      if (sample.geo == std::max(geo_index, 0) && sample.domain == domain &&
          sample.name == lookup_early)
      {
        const AttrRT &a = sample.direct;
        if (index >= 0 && index < a.size) {
          switch (a.type) {
            case Type::Bool:
              if (a.rb) {
                return Value::from_bool(a.rb[index]);
              }
              break;
            case Type::Int:
              if (a.ri) {
                return Value::from_int(a.ri[index]);
              }
              break;
            case Type::Float:
              if (a.rf) {
                return Value::from_float(a.rf[index]);
              }
              break;
            case Type::Vector2:
              if (a.r2) {
                return Value::from_vec2(a.r2[index]);
              }
              break;
            case Type::Vector:
              if (a.rv) {
                return Value::from_vec(a.rv[index]);
              }
              break;
            case Type::Vector4:
              if (a.r4) {
                return Value::from_vec4(a.r4[index], Type::Vector4);
              }
              break;
            case Type::Color:
              if (a.r4) {
                return Value::from_vec4(a.r4[index], Type::Color);
              }
              break;
            default:
              break;
          }
        }
        return value_from_varray(sample.values, index, &u->interned_strings);
      }
    }
    return std::nullopt;
  };
  /* A bind-domain snapshot is only valid for a sample on that same domain. Cross-domain reads
   * (e.g. a face program calling corner()) must use the prebound source-domain array. */
  if (geo_index > 0 || domain != bind_domain) {
    if (const std::optional<Value> cached = find_prebound_sample()) {
      return *cached;
    }
  }
  if (geo_index <= 0 && !u->attr_infos.is_empty() && !u->sample_attrs.is_empty()) {
    const int n = int(std::min(u->attr_infos.size(), u->sample_attrs.size()));
    for (int i = 0; i < n; i++) {
      if (u->attr_infos[i].name != lookup_early) {
        continue;
      }
      const AttrRT &a = u->sample_attrs[i];
      if (index < 0 || index >= a.size) {
        break;
      }
      switch (a.type) {
        case Type::Bool:
          if (a.rb) {
            return Value::from_bool(a.rb[index]);
          }
          if (a.wb) {
            return Value::from_bool(a.wb[index]);
          }
          break;
        case Type::Int:
          if (a.ri) {
            return Value::from_int(a.ri[index]);
          }
          if (a.wi) {
            return Value::from_int(a.wi[index]);
          }
          break;
        case Type::Float:
          if (a.rf) {
            return Value::from_float(a.rf[index]);
          }
          if (a.wf) {
            return Value::from_float(a.wf[index]);
          }
          break;
        case Type::Vector:
          if (a.rv) {
            return Value::from_vec(a.rv[index]);
          }
          if (a.wv) {
            return Value::from_vec(a.wv[index]);
          }
          break;
        case Type::Vector2:
          if (a.r2) {
            return Value::from_vec2(a.r2[index]);
          }
          if (a.w2) {
            return Value::from_vec2(a.w2[index]);
          }
          break;
        case Type::Vector4:
          if (a.r4) {
            return Value::from_vec4(a.r4[index], Type::Vector4);
          }
          if (a.w4) {
            return Value::from_vec4(a.w4[index], Type::Vector4);
          }
          break;
        case Type::Color:
          if (a.r4) {
            return Value::from_vec4(a.r4[index], Type::Color);
          }
          if (a.w4) {
            return Value::from_vec4(a.w4[index], Type::Color);
          }
          break;
        default:
          break;
      }
      break;
    }
  }
  if (const std::optional<Value> cached = find_prebound_sample()) {
    return *cached;
  }
  const bke::GeometrySet *gptr = nullptr;
  if (geo_index <= 0 && u->self) {
    gptr = u->self;
  }
  else if (geo_index >= 0 && geo_index < u->geos.size()) {
    gptr = u->geos[geo_index];
  }
  else if (u->self) {
    gptr = u->self;
  }
  if (gptr == nullptr) {
    return Value::from_float(0.0f);
  }
  const bke::GeometrySet &geo = *gptr;
  const std::string lookup = sample_attr_name(name);
  bke::AttrDomain ad = bke::AttrDomain::Point;
  switch (domain) {
    case 1:
      ad = bke::AttrDomain::Edge;
      break;
    case 2:
      ad = bke::AttrDomain::Face;
      break;
    case 3:
      ad = bke::AttrDomain::Corner;
      break;
    case 4:
      ad = bke::AttrDomain::Instance;
      break;
    case 5:
      ad = bke::AttrDomain::Curve;
      break;
    default:
      ad = bke::AttrDomain::Point;
      break;
  }

  if (const Mesh *mesh = geo.get_mesh()) {
    if (ELEM(ad,
             bke::AttrDomain::Point,
             bke::AttrDomain::Edge,
             bke::AttrDomain::Face,
             bke::AttrDomain::Corner))
    {
      Value v = sample_mesh(*mesh, ad, lookup, index, &u->interned_strings);
      if (v.type != Type::Void) {
        return v;
      }
    }
  }
  if (ad == bke::AttrDomain::Point) {
    if (const PointCloud *pc = geo.get_pointcloud()) {
      if (is_position_name(lookup)) {
        const Span<float3> pos = pc->positions();
        if (index >= 0 && index < pos.size()) {
          return Value::from_vec(pos[index]);
        }
      }
      Value v = read_named_attr(
          pc->attributes(), lookup, bke::AttrDomain::Point, index, &u->interned_strings);
      if (v.type != Type::Void) {
        return v;
      }
    }
  }
  if (ELEM(ad, bke::AttrDomain::Point, bke::AttrDomain::Curve)) {
    if (const Curves *curves_id = geo.get_curves()) {
      const bke::CurvesGeometry &cg = curves_id->geometry.wrap();
      if (is_position_name(lookup)) {
        const Span<float3> pos = cg.positions();
        if (ad == bke::AttrDomain::Point) {
          if (index >= 0 && index < pos.size()) {
            return Value::from_vec(pos[index]);
          }
        }
        else if (ad == bke::AttrDomain::Curve) {
          const OffsetIndices<int> pts = cg.points_by_curve();
          if (index >= 0 && index < pts.size() && !pts[index].is_empty()) {
            const int pi = pts[index].first();
            if (pi >= 0 && pi < pos.size()) {
              return Value::from_vec(pos[pi]);
            }
          }
        }
      }
      Value v = read_named_attr(cg.attributes(), lookup, ad, index, &u->interned_strings);
      if (v.type != Type::Void) {
        return v;
      }
    }
  }
  if (ad == bke::AttrDomain::Instance) {
    if (const bke::Instances *inst = geo.get_instances()) {
      if (lookup == "position" || is_position_name(lookup)) {
        /* Instance positions live on the transform translation. */
        if (index >= 0 && index < inst->instances_num()) {
          const float4x4 &tr = inst->transforms()[index];
          return Value::from_vec(tr.location());
        }
      }
      Value v = read_named_attr(
          inst->attributes(), lookup, bke::AttrDomain::Instance, index, &u->interned_strings);
      if (v.type != Type::Void) {
        return v;
      }
    }
  }
  return Value::from_float(0.0f);
}

Value load_sample_fn(void *user,
                     const int sample_slot,
                     const int index,
                     std::string &error)
{
  auto *u = static_cast<ElemUser *>(user);
  if (!u || sample_slot < 0 || sample_slot >= u->element_samples.size()) {
    return Value::from_float(0.0f);
  }
  ElemUser::ElementSampleCache &sample = u->element_samples[sample_slot];
  int bind_domain = 0;
  switch (u->attr_domain) {
    case bke::AttrDomain::Edge:
      bind_domain = 1;
      break;
    case bke::AttrDomain::Face:
      bind_domain = 2;
      break;
    case bke::AttrDomain::Corner:
      bind_domain = 3;
      break;
    case bke::AttrDomain::Instance:
      bind_domain = 4;
      break;
    case bke::AttrDomain::Curve:
      bind_domain = 5;
      break;
    default:
      break;
  }
  /* Same-domain self reads must retain the immutable Jacobi snapshot when the sampled attribute is
   * also written. Cross-domain and secondary-geometry samples can use the prebound typed span. */
  if (sample.geo <= 0 && sample.domain == bind_domain && !u->sample_attrs.is_empty()) {
    return load_elem_fn(
        user, sample.geo, sample.domain, sample.name, index, Type::Float, error);
  }
  if (sample.values) {
    return value_from_sample_cache(sample, index, &u->interned_strings);
  }
  return load_elem_fn(user, sample.geo, sample.domain, sample.name, index, Type::Float, error);
}

Value load_parm_fn(void *user, const StringRef name, const int index, std::string & /*error*/)
{
  auto *u = static_cast<ElemUser *>(user);
  if (!u || name.is_empty()) {
    return Value::from_float(0.0f);
  }
  for (const int i : u->parm_names.index_range()) {
    if (u->parm_names[i] == name) {
      const GVArray &va = u->parm_arrays[i];
      if (va && va.type().is<std::string>() && index >= 0 && index < va.size()) {
        u->interned_strings.append(va.typed<std::string>()[index]);
        return Value::from_str_i(100000 + int(u->interned_strings.size()) - 1);
      }
      return value_from_varray(va, index, &u->interned_strings);
    }
  }
  std::string dummy;
  int vex_domain = 0;
  switch (u->attr_domain) {
    case bke::AttrDomain::Edge:
      vex_domain = 1;
      break;
    case bke::AttrDomain::Face:
      vex_domain = 2;
      break;
    case bke::AttrDomain::Corner:
      vex_domain = 3;
      break;
    case bke::AttrDomain::Instance:
      vex_domain = 4;
      break;
    case bke::AttrDomain::Curve:
      vex_domain = 5;
      break;
    default:
      break;
  }
  return load_elem_fn(user, 0, vex_domain, name, index, Type::Float, dummy);
}

struct BindState {
  ResourceScope scope;
  Vector<bke::GSpanAttributeWriter *> writers;
  Vector<Array<float>> rf;
  Vector<Array<int>> ri;
  Vector<Array<bool>> rb;
  Vector<Array<float3>> rv;
  Vector<Array<float2>> r2;
  Vector<Array<float4>> rv4;
  Vector<Array<ColorGeometry4f>> rc;
  Vector<Array<math::Quaternion>> rq;
  Vector<Array<bke::WrangleArrayValue>> rarr;
  Vector<Array<bke::WrangleArrayValue>> warr;
  Vector<Array<MStringProperty>> rs;
  Vector<Array<MStringProperty>> ws;
  Vector<Array<float4x4>> rmats;
  Array<AttrRT> rt;
  struct CopyBack {
    const void *src = nullptr;
    void *dst = nullptr;
    size_t bytes = 0;
  };
  Vector<CopyBack> copy_back;
};

bool bind_attrs(bke::MutableAttributeAccessor attributes,
                const bke::AttrDomain domain,
                const int domain_size,
                const IndexMask &mask,
                const Program &program,
                BindState &state)
{
  state.rt.reinitialize(program.attrs.size());
  for (const int i : program.attrs.index_range()) {
    const AttrInfo &info = program.attrs[i];
    AttrRT &a = state.rt[i];
    a.type = info.type;
    a.size = domain_size;
    if (domain_size == 0 || mask.is_empty()) {
      continue;
    }
    const bke::AttrType at = to_attr_type(info.type);
    if (info.write) {
      /* i[]@pts must replace an existing float3 `pts`, otherwise the spreadsheet
       * still shows Vector and the packed list is never stored. */
      bke::GAttributeWriter aw = attributes.convert_or_add_for_write_only(
          info.name, domain, at);
      if (!aw) {
        const std::optional<bke::AttributeMetaData> meta = attributes.lookup_meta_data(info.name);
        if (meta && (meta->domain != domain || meta->data_type != at)) {
          attributes.remove(info.name);
        }
        aw = attributes.lookup_or_add_for_write(
            info.name, domain, at, bke::AttributeInitDefaultValue());
      }
      if (!aw) {
        return false;
      }
      bke::GSpanAttributeWriter &writer = state.scope.construct<bke::GSpanAttributeWriter>(
          std::move(aw), true);
      if (!writer) {
        return false;
      }
      state.writers.append(&writer);
      switch (info.type) {
        case Type::Bool: {
          MutableSpan<bool> span = writer.span.typed<bool>();
          a.wb = span.data();
          a.rb = span.data();
          break;
        }
        case Type::Int: {
          MutableSpan<int> span = writer.span.typed<int>();
          a.wi = span.data();
          a.ri = span.data();
          break;
        }
        case Type::Vector: {
          MutableSpan<float3> span = writer.span.typed<float3>();
          a.wv = span.data();
          a.rv = span.data();
          break;
        }
        case Type::Vector2: {
          MutableSpan<float2> span = writer.span.typed<float2>();
          a.w2 = span.data();
          a.r2 = span.data();
          break;
        }
        case Type::Vector4: {
          MutableSpan<float4> span = writer.span.typed<float4>();
          a.w4 = span.data();
          a.r4 = span.data();
          break;
        }
        case Type::Color: {
          MutableSpan<ColorGeometry4f> span = writer.span.typed<ColorGeometry4f>();
          a.w4 = reinterpret_cast<float4 *>(span.data());
          a.r4 = reinterpret_cast<const float4 *>(span.data());
          break;
        }
        case Type::Rotation: {
          MutableSpan<math::Quaternion> span = writer.span.typed<math::Quaternion>();
          a.wq = span.data();
          a.rq = span.data();
          break;
        }
        case Type::Matrix: {
          MutableSpan<float4x4> span = writer.span.typed<float4x4>();
          a.wm = span.data();
          a.rm = span.data();
          break;
        }
        case Type::Matrix2:
        case Type::Matrix3:
        case Type::IntArray:
        case Type::FloatArray:
        case Type::VecArray:
        case Type::StringArray:
        case Type::MatArray:
        case Type::RayArray: {
          MutableSpan<bke::WrangleArrayValue> span = writer.span.typed<bke::WrangleArrayValue>();
          if (!span.data() || span.size() < domain_size) {
            return false;
          }
          /* Write-only lists: convert_or_add_for_write_only already gave a unique
           * span. Cloning 512-byte slots for every point was slower than a 100-iter
           * Jacobi on P. */
          if (!info.read) {
            a.warr = span.data();
            a.rarr = span.data();
            break;
          }
          state.warr.append({});
          Array<bke::WrangleArrayValue> &buf = state.warr.last();
          buf.reinitialize(domain_size);
          memcpy(buf.data(), span.data(), sizeof(bke::WrangleArrayValue) * size_t(domain_size));
          a.warr = buf.data();
          a.rarr = buf.data();
          if (span.data() != buf.data()) {
            state.copy_back.append({buf.data(),
                                    span.data(),
                                    sizeof(bke::WrangleArrayValue) * size_t(domain_size)});
          }
          break;
        }
        case Type::String: {
          MutableSpan<MStringProperty> span = writer.span.typed<MStringProperty>();
          state.ws.append({});
          Array<MStringProperty> &buf = state.ws.last();
          buf.reinitialize(domain_size);
          if (span.data() && domain_size > 0) {
            memcpy(buf.data(), span.data(), sizeof(MStringProperty) * size_t(domain_size));
          }
          a.ws = buf.data();
          if (span.data() && span.data() != buf.data()) {
            state.copy_back.append(
                {buf.data(), span.data(), sizeof(MStringProperty) * size_t(domain_size)});
          }
          break;
        }
        case Type::Float:
        default: {
          MutableSpan<float> span = writer.span.typed<float>();
          a.wf = span.data();
          a.rf = span.data();
          break;
        }
      }
    }
    else {
      const bke::GAttributeReader reader = attributes.lookup(info.name, domain);
      if (!reader) {
        continue;
      }
      const GVArray &varray = reader.varray;
      switch (info.type) {
        case Type::Bool: {
          state.rb.append({});
          Array<bool> &buf = state.rb.last();
          buf.reinitialize(domain_size);
          if (varray.type().is<bool>()) {
            varray.typed<bool>().materialize(mask, buf);
          }
          a.rb = buf.data();
          break;
        }
        case Type::Int: {
          state.ri.append({});
          Array<int> &buf = state.ri.last();
          buf.reinitialize(domain_size);
          if (varray.type().is<int>()) {
            varray.typed<int>().materialize(mask, buf);
          }
          a.ri = buf.data();
          break;
        }
        case Type::Vector: {
          state.rv.append({});
          Array<float3> &buf = state.rv.last();
          buf.reinitialize(domain_size);
          if (varray.type().is<float3>()) {
            varray.typed<float3>().materialize(mask, buf);
          }
          a.rv = buf.data();
          break;
        }
        case Type::Vector2: {
          state.r2.append({});
          Array<float2> &buf = state.r2.last();
          buf.reinitialize(domain_size);
          if (varray.type().is<float2>()) {
            varray.typed<float2>().materialize(mask, buf);
          }
          a.r2 = buf.data();
          break;
        }
        case Type::Vector4: {
          state.rv4.append({});
          Array<float4> &buf = state.rv4.last();
          buf.reinitialize(domain_size);
          if (varray.type().is<float4>()) {
            varray.typed<float4>().materialize(mask, buf);
          }
          a.r4 = buf.data();
          break;
        }
        case Type::Color: {
          state.rc.append({});
          Array<ColorGeometry4f> &buf = state.rc.last();
          buf.reinitialize(domain_size);
          if (varray.type().is<ColorGeometry4f>()) {
            varray.typed<ColorGeometry4f>().materialize(mask, buf);
          }
          a.r4 = reinterpret_cast<const float4 *>(buf.data());
          break;
        }
        case Type::Rotation: {
          state.rq.append({});
          Array<math::Quaternion> &buf = state.rq.last();
          buf.reinitialize(domain_size);
          if (varray.type().is<math::Quaternion>()) {
            varray.typed<math::Quaternion>().materialize(mask, buf);
          }
          a.rq = buf.data();
          break;
        }
        case Type::Matrix: {
          state.rmats.append({});
          Array<float4x4> &buf = state.rmats.last();
          buf.reinitialize(domain_size);
          if (varray.type().is<float4x4>()) {
            varray.typed<float4x4>().materialize(mask, buf);
          }
          a.rm = buf.data();
          break;
        }
        case Type::Matrix2:
        case Type::Matrix3:
        case Type::IntArray:
        case Type::FloatArray:
        case Type::VecArray:
        case Type::StringArray:
        case Type::MatArray:
        case Type::RayArray: {
          if (varray.type().is<bke::WrangleArrayValue>()) {
            if (varray.is_span()) {
              a.rarr = varray.get_internal_span().typed<bke::WrangleArrayValue>().data();
            }
            else {
              state.rarr.append({});
              Array<bke::WrangleArrayValue> &buf = state.rarr.last();
              buf.reinitialize(domain_size);
              varray.typed<bke::WrangleArrayValue>().materialize(mask, buf);
              a.rarr = buf.data();
            }
          }
          break;
        }
        case Type::String: {
          if (varray.type().is<MStringProperty>()) {
            if (varray.is_span()) {
              a.rs = varray.get_internal_span().typed<MStringProperty>().data();
            }
            else {
              state.rs.append({});
              Array<MStringProperty> &buf = state.rs.last();
              buf.reinitialize(domain_size);
              varray.typed<MStringProperty>().materialize(mask, buf);
              a.rs = buf.data();
            }
          }
          break;
        }
        case Type::Float:
        default: {
          state.rf.append({});
          Array<float> &buf = state.rf.last();
          buf.reinitialize(domain_size);
          if (varray.type().is<float>()) {
            varray.typed<float>().materialize(mask, buf);
          }
          a.rf = buf.data();
          break;
        }
      }
    }
  }
  return true;
}

void finish_bind(BindState &state)
{
  for (const BindState::CopyBack &cb : state.copy_back) {
    if (cb.src && cb.dst && cb.bytes > 0) {
      memcpy(cb.dst, cb.src, cb.bytes);
    }
  }
  for (bke::GSpanAttributeWriter *w : state.writers) {
    if (w) {
      w->finish();
    }
  }
  state.writers.clear();
  state.copy_back.clear();
}

void append_positions_to_mesh(Mesh &mesh, const Span<float3> extra)
{
  if (extra.is_empty()) {
    return;
  }
  const int old_num = mesh.verts_num;
  mesh.verts_num += int(extra.size());
  CustomData_realloc(&mesh.vert_data, old_num, mesh.verts_num);
  mesh.attribute_storage.wrap().resize(bke::AttrDomain::Point, mesh.verts_num);
  MutableSpan<float3> pos = mesh.vert_positions_for_write();
  pos.slice(old_num, extra.size()).copy_from(extra);
  mesh.tag_positions_changed();
  mesh.tag_topology_changed();
}

void append_positions_to_pointcloud(PointCloud &pc, const Span<float3> extra)
{
  if (extra.is_empty()) {
    return;
  }
  const int old_num = pc.totpoint;
  pointcloud_resize(pc, old_num + int(extra.size()));
  pc.positions_for_write().slice(old_num, extra.size()).copy_from(extra);
}

void merge_addpoints(bke::GeometrySet &geometry, const Span<float3> extra)
{
  if (extra.is_empty()) {
    return;
  }
  if (geometry.has_mesh()) {
    append_positions_to_mesh(*geometry.get_mesh_for_write(), extra);
  }
  else if (geometry.has_pointcloud()) {
    append_positions_to_pointcloud(*geometry.get_pointcloud_for_write(), extra);
  }
  else {
    PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, int(extra.size()));
    pc->positions_for_write().copy_from(extra);
    geometry.replace_pointcloud(pc);
  }
}

enum class PrimKind : int8_t { Edge = 0, Face = 1, Curve = 2, Invalid = -1 };

PrimKind parse_prim_kind(const Value &v, const VMEnv &env)
{
  if (v.type == Type::String) {
    const StringRef s = value_string(v, env);
    if (s == "edge" || s == "edges" || s == "polyline" || s == "poly_line" || s == "EDGE") {
      return PrimKind::Edge;
    }
    if (s == "face" || s == "faces" || s == "poly" || s == "polygon" || s == "prim" ||
        s == "primitive" || s == "FACE" || s == "POLY")
    {
      return PrimKind::Face;
    }
    if (s == "curve" || s == "curves" || s == "spline" || s == "CURVE") {
      return PrimKind::Curve;
    }
    return PrimKind::Invalid;
  }
  const int k = v.as_int();
  if (k == 0) {
    return PrimKind::Edge;
  }
  if (k == 1) {
    return PrimKind::Face;
  }
  if (k == 2) {
    return PrimKind::Curve;
  }
  return PrimKind::Invalid;
}

void expand_mesh_topology(Mesh &mesh,
                          const int edge_expand,
                          const int face_expand,
                          const int loop_expand)
{
  if (edge_expand > 0) {
    if (mesh.edges_num == 0) {
      mesh.attributes_for_write().add(".edge_verts",
                                      bke::AttrDomain::Edge,
                                      bke::AttrType::Int32_2D,
                                      bke::AttributeInitConstruct());
    }
    const int old_edges = mesh.edges_num;
    mesh.edges_num += edge_expand;
    CustomData_realloc(&mesh.edge_data, old_edges, mesh.edges_num);
    mesh.attribute_storage.wrap().resize(bke::AttrDomain::Edge, mesh.edges_num);
  }
  if (face_expand > 0) {
    const int old_faces = mesh.faces_num;
    mesh.faces_num += face_expand;
    CustomData_realloc(&mesh.face_data, old_faces, mesh.faces_num);
    mesh.attribute_storage.wrap().resize(bke::AttrDomain::Face, mesh.faces_num);
    implicit_sharing::resize_trivial_array(&mesh.face_offset_indices,
                                           &mesh.runtime->face_offsets_sharing_info,
                                           old_faces == 0 ? 0 : (old_faces + 1),
                                           mesh.faces_num + 1);
    mesh.face_offset_indices[0] = 0;
    mesh.face_offset_indices[mesh.faces_num] = mesh.corners_num + loop_expand;
  }
  if (loop_expand > 0) {
    if (mesh.corners_num == 0) {
      mesh.attributes_for_write().add(".corner_vert",
                                      bke::AttrDomain::Corner,
                                      bke::AttrType::Int32,
                                      bke::AttributeInitConstruct());
      mesh.attributes_for_write().add(".corner_edge",
                                      bke::AttrDomain::Corner,
                                      bke::AttrType::Int32,
                                      bke::AttributeInitConstruct());
    }
    const int old_corners = mesh.corners_num;
    mesh.corners_num += loop_expand;
    CustomData_realloc(&mesh.corner_data, old_corners, mesh.corners_num);
    mesh.attribute_storage.wrap().resize(bke::AttrDomain::Corner, mesh.corners_num);
  }
}

Mesh *ensure_mesh_for_prims(bke::GeometrySet &geometry)
{
  if (Mesh *mesh = geometry.get_mesh_for_write()) {
    return mesh;
  }
  const PointCloud *pc = geometry.get_pointcloud();
  if (pc == nullptr || pc->totpoint <= 0) {
    return nullptr;
  }
  Mesh *mesh = BKE_mesh_new_nomain(pc->totpoint, 0, 0, 0);
  mesh->vert_positions_for_write().copy_from(pc->positions());
  mesh->tag_positions_changed();
  geometry.replace_mesh(mesh);
  return geometry.get_mesh_for_write();
}

Span<float3> prim_source_positions(const bke::GeometrySet &geometry)
{
  if (const Mesh *mesh = geometry.get_mesh()) {
    return mesh->vert_positions();
  }
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    return pc->positions();
  }
  return {};
}

struct PendingAddCurve {
  Vector<int> pts;
  bool cyclic = false;
};

void merge_add_curves(bke::GeometrySet &geometry,
                      const Span<PendingAddCurve> curves,
                      const Span<float3> positions)
{
  if (curves.is_empty()) {
    return;
  }
  int extra_pts = 0;
  for (const PendingAddCurve &c : curves) {
    extra_pts += int(c.pts.size());
  }
  if (extra_pts <= 0) {
    return;
  }
  Curves *id = geometry.get_curves_for_write();
  int old_pts = 0;
  int old_cu = 0;
  if (id == nullptr) {
    id = bke::curves_new_nomain(extra_pts, int(curves.size()));
    geometry.replace_curves(id);
    id = geometry.get_curves_for_write();
  }
  else {
    bke::CurvesGeometry &cg = id->geometry.wrap();
    old_pts = cg.points_num();
    old_cu = cg.curves_num();
    cg.resize(old_pts + extra_pts, old_cu + int(curves.size()));
  }
  if (id == nullptr) {
    return;
  }
  bke::CurvesGeometry &cg = id->geometry.wrap();
  MutableSpan<float3> pos = cg.positions_for_write();
  MutableSpan<int> offsets = cg.offsets_for_write();
  int p = old_pts;
  for (int i = 0; i < int(curves.size()); i++) {
    offsets[old_cu + i] = p;
    for (const int vi : curves[i].pts) {
      pos[p++] = (vi >= 0 && vi < positions.size()) ? positions[vi] : float3(0.0f);
    }
  }
  offsets[old_cu + int(curves.size())] = old_pts + extra_pts;
  {
    MutableSpan<int8_t> types = cg.curve_types_for_write();
    for (int i = 0; i < int(curves.size()); i++) {
      types[old_cu + i] = CURVE_TYPE_POLY;
    }
    cg.update_curve_types();
  }
  MutableSpan<bool> cyclic = cg.cyclic_for_write();
  for (int i = 0; i < int(curves.size()); i++) {
    cyclic[old_cu + i] = curves[i].cyclic;
  }
  cg.tag_topology_changed();
}

bool program_uses_addpoint(const Program &program)
{
  for (const Inst &in : program.code) {
    if (in.op == Op::Call && in.imm == int(Builtin::Addpoint)) {
      return true;
    }
  }
  return false;
}

bool program_uses_call(const Program &program, const Builtin id)
{
  for (const Inst &in : program.code) {
    if (in.op == Op::Call && in.imm == int(id)) {
      return true;
    }
  }
  return false;
}

bool program_uses_geo_side_effects(const Program &program)
{
  return program_uses_call(program, Builtin::Accumulate) ||
         program_uses_call(program, Builtin::DeleteGeometry) ||
         program_uses_call(program, Builtin::SetAttribute) ||
         program_uses_call(program, Builtin::Addpoint) ||
         program_uses_call(program, Builtin::Addprim);
}

bool program_uses_spatial(const Program &program)
{
  return program_uses_call(program, Builtin::GeometryProximity) ||
         program_uses_call(program, Builtin::Raycast) ||
         program_uses_call(program, Builtin::RaycastAll) ||
         program_uses_call(program, Builtin::SampleNearestSurface);
}

bool program_uses_topology(const Program &program)
{
  for (const Inst &in : program.code) {
    if (in.op == Op::FaceCorners) {
      return true;
    }
  }
  return program_uses_call(program, Builtin::PointNeighbours) ||
         program_uses_call(program, Builtin::PointEdges) ||
         program_uses_call(program, Builtin::PointFaces) ||
         program_uses_call(program, Builtin::PointCorners) ||
         program_uses_call(program, Builtin::EdgePoints) ||
         program_uses_call(program, Builtin::EdgeFaces) ||
         program_uses_call(program, Builtin::FacePoints) ||
         program_uses_call(program, Builtin::FaceEdges) ||
         program_uses_call(program, Builtin::FaceNeighbours) ||
         program_uses_call(program, Builtin::FaceCorners) ||
         program_uses_call(program, Builtin::Neighbours) ||
         program_uses_call(program, Builtin::CornersOfFace) ||
         program_uses_call(program, Builtin::CornersOfVertex) ||
         program_uses_call(program, Builtin::CornersOfEdge) ||
         program_uses_call(program, Builtin::EdgesOfVertex) ||
         program_uses_call(program, Builtin::EdgesOfCorner) ||
         program_uses_call(program, Builtin::FacesOfVertex) ||
         program_uses_call(program, Builtin::DeleteGeometry);
}

struct TopoUser;
const bke::GeometrySet *geo_at(TopoUser *u, int gi);

struct TopoUser {
  bke::GeometrySet *owner = nullptr;
  Span<const bke::GeometrySet *> geos;
  const Mesh *mesh = nullptr;
  const PointCloud *points = nullptr;
  const bke::CurvesGeometry *curves = nullptr;
  std::optional<bke::AttributeAccessor> attrs;
  Span<float3> positions;
  KDTree<float3> *kdtrees[8] = {};
  Array<int> v2e_off;
  Array<int> v2e_idx;
  Array<int> e2c_off;
  Array<int> e2c_idx;
  GroupedSpan<int> v2e;
  GroupedSpan<int> e2c;
  bool v2e_ok = false;
  bool e2c_ok = false;
  Array<int> point_to_curve;
  std::optional<Bounds<float3>> bounds;
  std::mutex mutex;
  /* Parallel addpoint: unique indices, short lock around append. */
  std::mutex add_mutex;
  Vector<float3> add_pos;
  Vector<int> add_src; /* -1 = from position, else copy P from this index. */
  Vector<int2> add_edges;
  Vector<Vector<int>> add_faces;
  Vector<PendingAddCurve> add_curves;
  std::atomic<bool> geo_forced = false;
  std::atomic<int> geo_forced_from = 0;
  int npoints_orig = 0;
  int nedges_orig = 0;
  int nfaces_orig = 0;
  int ncurves_orig = 0;
  bke::MutableAttributeAccessor *write_attrs = nullptr;
  ResourceScope write_scope;
  struct AttrSink {
    Type type = Type::Float;
    int size = 0;
    float *f = nullptr;
    int *i = nullptr;
    float3 *v = nullptr;
    float2 *v2 = nullptr;
    bool *b = nullptr;
    float4 *c = nullptr;
    math::Quaternion *q = nullptr;
  };
  Map<std::string, AttrSink> attr_sinks;
  Vector<bke::GSpanAttributeWriter *> extra_writers;
  std::mutex attr_sink_mutex;
  Span<AttrInfo> attr_infos;
  MutableSpan<AttrRT> attr_rt;
  struct PendingSet {
    int index = 0;
    bool is_new = false;
    bke::AttrDomain domain = bke::AttrDomain::Point;
    std::string name;
    Type type = Type::Float;
    float4 v{0.0f, 0.0f, 0.0f, 0.0f};
    int i = 0;
    int mode = 0;
  };
  Vector<PendingSet> pending_sets;
  std::mutex pending_mutex;
  Set<int> remove_points;
  Set<int> remove_edges;
  Set<int> remove_faces;
  Set<int> remove_curves;
  Set<int> remove_instances;
  GeometryNodeDeleteGeometryMode delete_mode_point = GEO_NODE_DELETE_GEOMETRY_MODE_ALL;
  GeometryNodeDeleteGeometryMode delete_mode_edge = GEO_NODE_DELETE_GEOMETRY_MODE_ALL;
  GeometryNodeDeleteGeometryMode delete_mode_face = GEO_NODE_DELETE_GEOMETRY_MODE_ALL;
  Map<int, float> acc_f;
  Map<int, float3> acc_v;

  /* Cached C/Embree acceleration for spatial builtins. Built once on the
   * main thread, then queried with no node evaluation and no per-element rebuild. */
  struct SpatialAccel {
    const Mesh *mesh = nullptr;
    const PointCloud *pc = nullptr;
    const bke::bvh::Tree *tris = nullptr;
    const bke::bvh::Tree *verts = nullptr;
    const bke::bvh::Tree *edges = nullptr;
    bke::BVHTreeFromMesh faces{};
    const bke::bvh::Tree *points = nullptr;
    bool verts_ok = false;
    bool edges_ok = false;
    bool faces_ok = false;
    bool points_ok = false;
  };
  SpatialAccel spatial[8];

  /* C parallel precompute of geometry_proximity(pos, dist) — one tree, both outputs. */
  int prox_geo = 0;
  int prox_domain = 2;
  Span<float3> prox_samples;
  Array<float3> prox_pos;
  Array<float> prox_dist;
  bool prox_ok = false;
  /** Precompute already wrote bound attribute arrays; skip the extra memcpy. */
  bool spatial_wrote_attrs = false;

  Array<int8_t> ray_hit;
  Array<int> ray_face;
  Array<float3> ray_pos;
  Array<float3> ray_n;
  Array<float> ray_dist;
  bool ray_ok = false;
  int ray_geo = 0;

  Array<float3> sample_v;
  Array<float> sample_f;
  bool sample_ok = false;
  bool sample_is_vec = false;
  int sample_geo = 0;

  /* CSR k-nearest table for nearestpoints(geo, k, "k") at each element's P. */
  Array<int> knn_off;
  Array<int> knn_idx;
  int knn_k = 0;
  int knn_geo = 0;
  bool knn_ok = false;

  /* Lock-free delete flags; each element writes its own index. */
  Array<uint8_t> del_point;
  Array<uint8_t> del_edge;
  Array<uint8_t> del_face;
  Array<uint8_t> del_curve;
  Array<uint8_t> del_inst;
  bke::AttrDomain bind_domain = bke::AttrDomain::Point;
  bke::GeometryComponent::Type bind_type = bke::GeometryComponent::Type::Mesh;

  ~TopoUser()
  {
    for (int i = 0; i < 8; i++) {
      if (kdtrees[i]) {
        kdtree_free(kdtrees[i]);
      }
    }
  }

  void ensure_v2e()
  {
    if (v2e_ok || !mesh) {
      return;
    }
    std::lock_guard lock(mutex);
    if (v2e_ok || !mesh) {
      return;
    }
    v2e = bke::mesh::build_vert_to_edge_map(mesh->edges(), mesh->verts_num, v2e_off, v2e_idx);
    v2e_ok = true;
  }
  void ensure_e2c()
  {
    if (e2c_ok || !mesh) {
      return;
    }
    std::lock_guard lock(mutex);
    if (e2c_ok || !mesh) {
      return;
    }
    e2c = bke::mesh::build_edge_to_corner_map(
        mesh->corner_edges(), mesh->edges_num, e2c_off, e2c_idx);
    e2c_ok = true;
  }
  void prewarm_topology()
  {
    if (!mesh) {
      return;
    }
    ensure_v2e();
    ensure_e2c();
    (void)mesh->vert_to_face_map();
    (void)mesh->vert_to_corner_map();
    (void)mesh->corner_to_face_map();
    (void)mesh->corner_verts();
    (void)mesh->corner_edges();
    (void)mesh->faces();
    (void)mesh->edges();
  }
  KDTree<float3> *tree_for(const int gi)
  {
    const int slot = (gi >= 0 && gi < 8) ? gi : 0;
    if (kdtrees[slot]) {
      return kdtrees[slot];
    }
    std::lock_guard lock(mutex);
    if (kdtrees[slot]) {
      return kdtrees[slot];
    }
    Span<float3> pos;
    /* Geo 0 must use the span bound on this TopoUser. After
     * `positions_for_write()` the GeometrySet pointer is unique, but the
     * *previous* `positions()` span is already dangling. */
    if (gi <= 0 && !positions.is_empty()) {
      pos = positions;
    }
    else {
      const bke::GeometrySet *g = geo_at(this, gi);
      if (g) {
        if (const Mesh *m = g->get_mesh()) {
          pos = m->vert_positions();
        }
        else if (const PointCloud *pc = g->get_pointcloud()) {
          pos = pc->positions();
        }
        else if (const Curves *c = g->get_curves()) {
          pos = c->geometry.wrap().positions();
        }
      }
    }
    if (pos.is_empty()) {
      pos = positions;
    }
    if (pos.is_empty()) {
      return nullptr;
    }
    KDTree<float3> *tree = kdtree_new<float3>(uint(pos.size()));
    for (const int i : pos.index_range()) {
      kdtree_insert<float3>(tree, i, pos[i]);
    }
    kdtree_balance(tree);
    kdtrees[slot] = tree;
    return tree;
  }
  void ensure_kdtree()
  {
    tree_for(0);
  }
  void invalidate_spatial()
  {
    for (int i = 0; i < 8; i++) {
      if (kdtrees[i]) {
        kdtree_free(kdtrees[i]);
        kdtrees[i] = nullptr;
      }
    }
    knn_ok = false;
    knn_off = {};
    knn_idx = {};
    knn_k = 0;
  }
  void ensure_bounds()
  {
    if (bounds || !owner) {
      return;
    }
    bounds = owner->compute_boundbox_without_instances(true);
  }
  void ensure_point_to_curve()
  {
    if (!point_to_curve.is_empty() || !curves) {
      return;
    }
    point_to_curve = curves->point_to_curve_map();
  }
};

Value empty_int_arr()
{
  return value_from_int_array({});
}

/* Last raycastall() hits on this thread. Extract builtins copy these. */
struct RaycastAllOut {
  Value hit;
  Value face;
  Value pos;
  Value n;
  Value dist;
};
thread_local RaycastAllOut tls_rayall;

void reset_raycast_all_out()
{
  tls_rayall.hit = empty_int_arr();
  tls_rayall.face = empty_int_arr();
  tls_rayall.pos = value_from_vec_array({});
  tls_rayall.n = value_from_vec_array({});
  tls_rayall.dist = value_from_float_array({});
}

void parse_ray_query(const Span<Value> args,
                      const VMEnv &env,
                      TopoUser *u,
                      int &r_geo,
                      float3 &r_origin,
                      float3 &r_dir,
                      float &r_length)
{
  const bool full = args.size() >= 4;
  r_geo = full ? args[0].as_int() : 0;
  r_origin = (env.index >= 0 && env.index < u->positions.size()) ? u->positions[env.index] :
                                                                   float3(0.0f);
  r_dir = float3(0.0f, 0.0f, -1.0f);
  r_length = 100.0f;
  if (full) {
    r_origin = args[1].as_vec();
    r_dir = args[2].as_vec();
    r_length = args[3].as_float();
  }
  else if (args.size() >= 3) {
    r_origin = args[0].as_vec();
    r_dir = args[1].as_vec();
    r_length = args[2].as_float();
  }
  else if (args.size() == 2) {
    r_origin = args[0].as_vec();
    r_dir = args[1].as_vec();
  }
  else if (args.size() == 1) {
    r_dir = args[0].as_vec();
  }
}

TopoUser::SpatialAccel &spatial_of(TopoUser *u, int gi);
void spatial_ensure_tris(TopoUser::SpatialAccel &a);

Value eval_raycast_all(TopoUser *u, const Span<Value> args, VMEnv &env)
{
  reset_raycast_all_out();
  env.hit_face = -1;
  env.hit_dist = 0.0f;
  env.hit_pos = float3(0.0f);
  env.hit_n = float3(0.0f);

  int geo_i = 0;
  float3 origin(0.0f);
  float3 dir(0.0f, 0.0f, -1.0f);
  float length = 100.0f;
  parse_ray_query(args, env, u, geo_i, origin, dir, length);

  TopoUser::SpatialAccel &accel = spatial_of(u, geo_i);
  spatial_ensure_tris(accel);
  if (accel.tris == nullptr || accel.mesh == nullptr) {
    return value_from_ray_array({});
  }

  bke::bvh::Ray ray{};
  ray.origin = origin;
  ray.direction = dir;
  ray.dist_max = length;

  struct HitRec {
    int tri = -1;
    int face = -1;
    float3 pos = float3(0.0f);
    float3 n = float3(0.0f);
    float dist = 0.0f;
  };
  Vector<HitRec> hits;
  const Span<int> tri_faces = accel.mesh->corner_tri_faces();
  accel.tris->ray_intersect_all(ray, [&](const bke::bvh::RayHit &hit) {
    HitRec rec;
    rec.tri = hit.index;
    rec.face = (hit.index >= 0 && hit.index < tri_faces.size()) ? tri_faces[hit.index] :
                                                                  hit.index;
    rec.pos = hit.position(ray);
    rec.n = math::normalize(hit.normal);
    rec.dist = hit.distance;
    hits.append(rec);
  });
  std::sort(hits.begin(), hits.end(), [](const HitRec &a, const HitRec &b) {
    return a.dist < b.dist;
  });
  Vector<HitRec> uniq;
  uniq.reserve(hits.size());
  for (const HitRec &h : hits) {
    bool seen = false;
    for (const HitRec &kept : uniq) {
      if (kept.tri == h.tri) {
        seen = true;
        break;
      }
    }
    if (!seen) {
      uniq.append(h);
    }
  }
  constexpr int k_max = bke::WrangleArrayValue::max_values / 3;
  if (uniq.size() > k_max) {
    uniq.resize(k_max);
  }

  Vector<int> is_hit;
  Vector<int> faces;
  Vector<float3> pos;
  Vector<float3> n;
  Vector<float> dist;
  Vector<RayHit> rays;
  is_hit.reserve(uniq.size());
  faces.reserve(uniq.size());
  pos.reserve(uniq.size());
  n.reserve(uniq.size());
  dist.reserve(uniq.size());
  rays.reserve(uniq.size());
  for (const HitRec &h : uniq) {
    is_hit.append(1);
    faces.append(h.face);
    pos.append(h.pos);
    n.append(h.n);
    dist.append(h.dist);
    RayHit rec;
    rec.hit = 1;
    rec.face = h.face;
    rec.pos = h.pos;
    rec.n = h.n;
    rec.dist = h.dist;
    rays.append(rec);
  }
  if (!uniq.is_empty()) {
    env.hit_face = uniq[0].face;
    env.hit_pos = uniq[0].pos;
    env.hit_n = uniq[0].n;
    env.hit_dist = uniq[0].dist;
  }
  tls_rayall.hit = value_from_int_array(std::move(is_hit));
  tls_rayall.face = value_from_int_array(std::move(faces));
  tls_rayall.pos = value_from_vec_array(std::move(pos));
  tls_rayall.n = value_from_vec_array(std::move(n));
  tls_rayall.dist = value_from_float_array(std::move(dist));
  return value_from_ray_array(std::move(rays));
}

static Value intern_env_ray(VMEnv &env, const int numeric)
{
  RayHit rec;
  rec.hit = env.hit_face >= 0 ? 1 : 0;
  rec.face = env.hit_face;
  rec.pos = env.hit_pos;
  rec.n = env.hit_n;
  rec.dist = env.hit_dist;
  Value r = value_from_ray(rec);
  r.v.x = float(numeric);
  return r;
}

static RayHit ray_arg_or_env(const Span<Value> args, const VMEnv &env)
{
  if (!args.is_empty() && args[0].type == Type::Ray) {
    if (const RayHit *h = ray_from_value(args[0])) {
      return *h;
    }
    return {};
  }
  RayHit h;
  h.hit = env.hit_face >= 0 ? 1 : 0;
  h.face = env.hit_face;
  h.pos = env.hit_pos;
  h.n = env.hit_n;
  h.dist = env.hit_dist;
  return h;
}

Value span_to_int_arr(const Span<int> src)
{
  return value_from_int_span(src);
}

Value range_to_int_arr(const IndexRange range)
{
  return value_from_int_range(int(range.start()), int(range.size()));
}

int arg_or_index(const Span<Value> args, const int i, const int fallback)
{
  return i < args.size() ? args[i].as_int() : fallback;
}

StringRef arg_string(const Span<Value> args, const int i, const VMEnv &env)
{
  if (i >= args.size()) {
    return {};
  }
  return value_string(args[i], env);
}

const bke::GeometrySet *geo_at(TopoUser *u, const int gi)
{
  if (u == nullptr) {
    return nullptr;
  }
  /* Input 0 is the geometry currently being wrangled (`owner`). The pointer in
   * `geos[0]` is the pre-foreach set, which `foreach_real_geometry` strips down
   * to instances-only — sampling it makes pointneighbours / point() see nothing. */
  if (gi <= 0) {
    if (u->owner) {
      return u->owner;
    }
  }
  if (gi >= 0 && gi < u->geos.size() && u->geos[gi]) {
    return u->geos[gi];
  }
  return u->owner;
}

const Mesh *mesh_at(TopoUser *u, const int gi)
{
  if (u == nullptr) {
    return nullptr;
  }
  /* Geo 0 is the component being wrangled. `u->mesh` is bound from that copy
   * and stays valid even if `geos[0]` was stripped to instances-only. */
  if (gi <= 0 && u->mesh) {
    return u->mesh;
  }
  if (const bke::GeometrySet *g = geo_at(u, gi)) {
    if (const Mesh *mesh = g->get_mesh()) {
      return mesh;
    }
  }
  return (gi <= 0) ? u->mesh : nullptr;
}

const bke::CurvesGeometry *curves_at(TopoUser *u, const int gi)
{
  if (u == nullptr) {
    return nullptr;
  }
  if (gi <= 0 && u->curves) {
    return u->curves;
  }
  if (const bke::GeometrySet *g = geo_at(u, gi)) {
    if (const Curves *c = g->get_curves()) {
      return &c->geometry.wrap();
    }
  }
  return (gi <= 0) ? u->curves : nullptr;
}

int value_as_elem_index(const Value &v, const int fallback)
{
  if (type_is_array(v.type)) {
    return fallback;
  }
  return v.as_int();
}

void parse_geo_elem(const Span<Value> args, const int fallback_index, int &r_geo, int &r_elem)
{
  if (args.size() >= 2) {
    r_geo = value_as_elem_index(args[0], 0);
    r_elem = value_as_elem_index(args[1], fallback_index);
  }
  else if (args.size() == 1) {
    r_geo = 0;
    r_elem = value_as_elem_index(args[0], fallback_index);
  }
  else {
    r_geo = 0;
    r_elem = fallback_index;
  }
}

struct MeshTopoCache {
  const Mesh *mesh = nullptr;
  Array<int> v2e_off;
  Array<int> v2e_idx;
  Array<int> e2f_off;
  Array<int> e2f_idx;
  Array<int> e2c_off;
  Array<int> e2c_idx;
  GroupedSpan<int> v2e;
  GroupedSpan<int> e2f;
  GroupedSpan<int> e2c;
  bool v2e_ok = false;
  bool e2f_ok = false;
  bool e2c_ok = false;

  void ensure_v2e()
  {
    if (v2e_ok || mesh == nullptr) {
      return;
    }
    v2e = bke::mesh::build_vert_to_edge_map(mesh->edges(), mesh->verts_num, v2e_off, v2e_idx);
    v2e_ok = true;
  }

  void ensure_e2f()
  {
    if (e2f_ok || mesh == nullptr) {
      return;
    }
    e2f = bke::mesh::build_edge_to_face_map(
        mesh->faces(), mesh->corner_edges(), mesh->edges_num, e2f_off, e2f_idx);
    e2f_ok = true;
  }

  void ensure_e2c()
  {
    if (e2c_ok || mesh == nullptr) {
      return;
    }
    e2c = bke::mesh::build_edge_to_corner_map(
        mesh->corner_edges(), mesh->edges_num, e2c_off, e2c_idx);
    e2c_ok = true;
  }
};

thread_local Vector<std::unique_ptr<MeshTopoCache>> tls_mesh_topo;

MeshTopoCache &mesh_topo(const Mesh *mesh)
{
  for (std::unique_ptr<MeshTopoCache> &slot : tls_mesh_topo) {
    if (slot->mesh == mesh) {
      return *slot;
    }
  }
  if (tls_mesh_topo.size() >= 8) {
    tls_mesh_topo.remove(0);
  }
  tls_mesh_topo.append(std::make_unique<MeshTopoCache>());
  MeshTopoCache &cache = *tls_mesh_topo.last();
  cache.mesh = mesh;
  return cache;
}

void append_unique_int(Vector<int> &vals, const int v)
{
  for (const int x : vals) {
    if (x == v) {
      return;
    }
  }
  vals.append(v);
}

int domain_from_attr(const bke::AttrDomain domain)
{
  switch (domain) {
    case bke::AttrDomain::Edge:
      return int(bke::AttrDomain::Edge);
    case bke::AttrDomain::Face:
      return int(bke::AttrDomain::Face);
    case bke::AttrDomain::Corner:
      return int(bke::AttrDomain::Corner);
    case bke::AttrDomain::Curve:
      return int(bke::AttrDomain::Curve);
    case bke::AttrDomain::Instance:
      return int(bke::AttrDomain::Instance);
    default:
      return int(bke::AttrDomain::Point);
  }
}

int parse_domain_arg(const Value &v, const VMEnv &env)
{
  if (v.type == Type::String) {
    const StringRef s = value_string(v, env);
    if (s == "point" || s == "points" || s == "POINT" || s == "Points") {
      return int(bke::AttrDomain::Point);
    }
    if (s == "edge" || s == "edges" || s == "EDGE" || s == "Edges") {
      return int(bke::AttrDomain::Edge);
    }
    if (s == "face" || s == "faces" || s == "FACE" || s == "Faces" || s == "prim" ||
        s == "primitive" || s == "PRIM")
    {
      return int(bke::AttrDomain::Face);
    }
    if (s == "corner" || s == "corners" || s == "CORNER") {
      return int(bke::AttrDomain::Corner);
    }
    if (s == "curve" || s == "curves" || s == "spline" || s == "CURVE") {
      return int(bke::AttrDomain::Curve);
    }
    if (s == "instance" || s == "instances" || s == "INSTANCE") {
      return int(bke::AttrDomain::Instance);
    }
  }
  return v.as_int();
}

void note_geo_forced(TopoUser &u, const int geo)
{
  if (geo == 0) {
    return;
  }
  int expected = 0;
  u.geo_forced_from.compare_exchange_strong(expected, geo, std::memory_order_relaxed);
  u.geo_forced.store(true, std::memory_order_relaxed);
}

int add_point_atomic(TopoUser &u, const float3 &p, const int src)
{
  std::lock_guard lock(u.add_mutex);
  const int i = int(u.add_pos.size());
  u.add_pos.append(p);
  u.add_src.append(src);
  return i;
}

void merge_addprims(bke::GeometrySet &geometry, TopoUser &user)
{
  std::lock_guard lock(user.add_mutex);
  if (!user.add_edges.is_empty() || !user.add_faces.is_empty()) {
    Mesh *mesh = ensure_mesh_for_prims(geometry);
    if (mesh) {
      const int nverts = mesh->verts_num;
      Vector<int2> edges;
      edges.reserve(user.add_edges.size());
      for (const int2 e : user.add_edges) {
        if (e[0] >= 0 && e[0] < nverts && e[1] >= 0 && e[1] < nverts && e[0] != e[1]) {
          edges.append(e);
        }
      }
      Vector<Vector<int>> faces;
      int extra_corners = 0;
      faces.reserve(user.add_faces.size());
      for (const Vector<int> &src : user.add_faces) {
        Vector<int> verts;
        verts.reserve(src.size());
        for (const int v : src) {
          if (v >= 0 && v < nverts && (verts.is_empty() || verts.last() != v)) {
            verts.append(v);
          }
        }
        if (verts.size() >= 3) {
          extra_corners += int(verts.size());
          faces.append(std::move(verts));
        }
      }
      const int old_edges = mesh->edges_num;
      const int old_faces = mesh->faces_num;
      const int old_corners = mesh->corners_num;
      expand_mesh_topology(*mesh, int(edges.size()), int(faces.size()), extra_corners);
      if (!edges.is_empty()) {
        mesh->edges_for_write().slice(old_edges, edges.size()).copy_from(edges);
      }
      if (!faces.is_empty()) {
        MutableSpan<int> offsets = mesh->face_offsets_for_write();
        MutableSpan<int> corner_verts = mesh->corner_verts_for_write();
        int corner = old_corners;
        for (int i = 0; i < int(faces.size()); i++) {
          offsets[old_faces + i] = corner;
          for (const int v : faces[i]) {
            corner_verts[corner++] = v;
          }
        }
        offsets[mesh->faces_num] = mesh->corners_num;
        bke::mesh_calc_edges(*mesh, true, false);
      }
      else {
        mesh->tag_topology_changed();
      }
    }
    user.add_edges.clear();
    user.add_faces.clear();
  }
  if (!user.add_curves.is_empty()) {
    merge_add_curves(geometry, user.add_curves, prim_source_positions(geometry));
    user.add_curves.clear();
  }
}

enum class SetMode : int8_t { Set = 0, Add = 1, Min = 2, Max = 3 };

SetMode parse_set_mode(const Value &v, const VMEnv &env)
{
  if (v.type == Type::String) {
    const StringRef s = value_string(v, env);
    if (s == "add" || s == "Add" || s == "ADD") {
      return SetMode::Add;
    }
    if (s == "min" || s == "Min" || s == "MIN") {
      return SetMode::Min;
    }
    if (s == "max" || s == "Max" || s == "MAX") {
      return SetMode::Max;
    }
    return SetMode::Set;
  }
  const int m = v.as_int();
  if (m == 1) {
    return SetMode::Add;
  }
  if (m == 2) {
    return SetMode::Min;
  }
  if (m == 3) {
    return SetMode::Max;
  }
  return SetMode::Set;
}

void atomic_set_f(float &slot, const float v, const SetMode mode)
{
  std::atomic_ref<float> a(slot);
  if (mode == SetMode::Set) {
    a.store(v, std::memory_order_relaxed);
    return;
  }
  float cur = a.load(std::memory_order_relaxed);
  while (true) {
    float next = cur;
    if (mode == SetMode::Add) {
      next = cur + v;
    }
    else if (mode == SetMode::Min) {
      next = std::min(cur, v);
    }
    else {
      next = std::max(cur, v);
    }
    if (a.compare_exchange_weak(cur, next, std::memory_order_relaxed)) {
      return;
    }
  }
}

void atomic_set_i(int &slot, const int v, const SetMode mode)
{
  std::atomic_ref<int> a(slot);
  if (mode == SetMode::Set) {
    a.store(v, std::memory_order_relaxed);
    return;
  }
  int cur = a.load(std::memory_order_relaxed);
  while (true) {
    int next = cur;
    if (mode == SetMode::Add) {
      next = cur + v;
    }
    else if (mode == SetMode::Min) {
      next = std::min(cur, v);
    }
    else {
      next = std::max(cur, v);
    }
    if (a.compare_exchange_weak(cur, next, std::memory_order_relaxed)) {
      return;
    }
  }
}

void atomic_set_b(bool &slot, const bool v, const SetMode mode)
{
  std::atomic_ref<bool> a(slot);
  if (mode == SetMode::Add) {
    if (v) {
      a.store(true, std::memory_order_relaxed);
    }
    return;
  }
  a.store(v, std::memory_order_relaxed);
}

void atomic_set_v3(float3 &slot, const float3 v, const SetMode mode)
{
  atomic_set_f(slot.x, v.x, mode);
  atomic_set_f(slot.y, v.y, mode);
  atomic_set_f(slot.z, v.z, mode);
}

void atomic_write_attrrt(AttrRT &a, const int index, const Value &val, const SetMode mode)
{
  if (index < 0 || index >= a.size) {
    return;
  }
  if (a.wf) {
    atomic_set_f(a.wf[index], val.as_float(), mode);
  }
  else if (a.wi) {
    atomic_set_i(a.wi[index], val.as_int(), mode);
  }
  else if (a.wb) {
    atomic_set_b(a.wb[index], val.as_bool(), mode);
  }
  else if (a.wv) {
    atomic_set_v3(a.wv[index], val.as_vec(), mode);
  }
  else if (a.w2) {
    const float2 p = val.as_vec2();
    atomic_set_f(a.w2[index].x, p.x, mode);
    atomic_set_f(a.w2[index].y, p.y, mode);
  }
  else if (a.wq) {
    const math::Quaternion q = value_as_quat(val);
    atomic_set_f(a.wq[index].x, q.x, mode);
    atomic_set_f(a.wq[index].y, q.y, mode);
    atomic_set_f(a.wq[index].z, q.z, mode);
    atomic_set_f(a.wq[index].w, q.w, mode);
  }
  else if (a.w4) {
    const float4 c = (val.type == Type::Color || val.type == Type::Vector4) ?
                         val.v :
                         float4(val.as_vec().x, val.as_vec().y, val.as_vec().z, val.v.w);
    atomic_set_f(a.w4[index].x, c.x, mode);
    atomic_set_f(a.w4[index].y, c.y, mode);
    atomic_set_f(a.w4[index].z, c.z, mode);
    atomic_set_f(a.w4[index].w, c.w, mode);
  }
}

Type type_from_value(const Value &v)
{
  switch (v.type) {
    case Type::Int:
    case Type::Bool:
    case Type::Vector2:
    case Type::Vector:
    case Type::Vector4:
    case Type::Color:
    case Type::Rotation:
    case Type::Matrix2:
    case Type::Matrix3:
    case Type::Matrix:
    case Type::String:
    case Type::Ray:
    case Type::RayArray:
      return v.type;
    default:
      return Type::Float;
  }
}

TopoUser::AttrSink *ensure_attr_sink(TopoUser &u,
                                     const bke::AttrDomain domain,
                                     const StringRef name,
                                     const Type type)
{
  const std::string aname = sample_attr_name(name);
  const std::string key = std::to_string(int(domain)) + "|" + aname;
  std::lock_guard lock(u.attr_sink_mutex);
  if (TopoUser::AttrSink *s = u.attr_sinks.lookup_ptr(key)) {
    return s;
  }
  if (!u.write_attrs) {
    return nullptr;
  }
  bke::GAttributeWriter aw = u.write_attrs->lookup_or_add_for_write(
      aname, domain, to_attr_type(type), bke::AttributeInitDefaultValue());
  if (!aw) {
    return nullptr;
  }
  bke::GSpanAttributeWriter &writer = u.write_scope.construct<bke::GSpanAttributeWriter>(
      std::move(aw), true);
  if (!writer) {
    return nullptr;
  }
  TopoUser::AttrSink s;
  s.type = type;
  s.size = int(writer.span.size());
  switch (type) {
    case Type::Bool:
      if (writer.span.type().is<bool>()) {
        s.b = writer.span.typed<bool>().data();
      }
      break;
    case Type::Int:
      if (writer.span.type().is<int>()) {
        s.i = writer.span.typed<int>().data();
      }
      break;
    case Type::Vector:
      if (writer.span.type().is<float3>()) {
        s.v = writer.span.typed<float3>().data();
      }
      break;
    case Type::Vector2:
      if (writer.span.type().is<float2>()) {
        s.v2 = writer.span.typed<float2>().data();
      }
      break;
    case Type::Vector4:
      if (writer.span.type().is<float4>()) {
        s.c = writer.span.typed<float4>().data();
      }
      break;
    case Type::Color:
      if (writer.span.type().is<ColorGeometry4f>()) {
        s.c = reinterpret_cast<float4 *>(writer.span.typed<ColorGeometry4f>().data());
      }
      break;
    case Type::Rotation:
      if (writer.span.type().is<math::Quaternion>()) {
        s.q = writer.span.typed<math::Quaternion>().data();
      }
      break;
    default:
      if (writer.span.type().is<float>()) {
        s.f = writer.span.typed<float>().data();
        s.type = Type::Float;
      }
      else if (writer.span.type().is<float3>()) {
        s.v = writer.span.typed<float3>().data();
        s.type = Type::Vector;
      }
      else if (writer.span.type().is<int>()) {
        s.i = writer.span.typed<int>().data();
        s.type = Type::Int;
      }
      break;
  }
  u.extra_writers.append(&writer);
  u.attr_sinks.add_new(key, s);
  return u.attr_sinks.lookup_ptr(key);
}

void finish_extra_writers(TopoUser &u)
{
  for (bke::GSpanAttributeWriter *w : u.extra_writers) {
    if (w) {
      w->finish();
    }
  }
  u.extra_writers.clear();
  u.attr_sinks.clear();
}

void apply_pending_sets(TopoUser &u, bke::GeometrySet &geo)
{
  if (u.pending_sets.is_empty()) {
    return;
  }
  std::optional<bke::MutableAttributeAccessor> attrs;
  if (geo.has_mesh()) {
    attrs = geo.get_mesh_for_write()->attributes_for_write();
  }
  else if (geo.has_pointcloud()) {
    attrs = geo.get_pointcloud_for_write()->attributes_for_write();
  }
  if (!attrs) {
    return;
  }
  const int old_n = u.npoints_orig;
  for (const TopoUser::PendingSet &p : u.pending_sets) {
    const int index = p.is_new ? old_n + p.index : p.index;
    const std::string aname = sample_attr_name(p.name);
    bke::GSpanAttributeWriter writer = attrs->lookup_or_add_for_write_span(
        aname, p.domain, to_attr_type(p.type));
    if (!writer || index < 0 || index >= writer.span.size()) {
      if (writer) {
        writer.finish();
      }
      continue;
    }
    Value val;
    val.type = p.type;
    val.i = p.i;
    val.v = p.v;
    if (writer.span.type().is<float>()) {
      writer.span.typed<float>()[index] = val.as_float();
    }
    else if (writer.span.type().is<int>()) {
      writer.span.typed<int>()[index] = val.as_int();
    }
    else if (writer.span.type().is<bool>()) {
      writer.span.typed<bool>()[index] = val.as_bool();
    }
    else if (writer.span.type().is<float3>()) {
      writer.span.typed<float3>()[index] = val.as_vec();
    }
    else if (writer.span.type().is<float4>()) {
      writer.span.typed<float4>()[index] = val.as_vec4();
    }
    else if (writer.span.type().is<math::Quaternion>()) {
      writer.span.typed<math::Quaternion>()[index] = value_as_quat(val);
    }
    else if (writer.span.type().is<ColorGeometry4f>()) {
      const float4 c = val.v;
      writer.span.typed<ColorGeometry4f>()[index] = ColorGeometry4f(c.x, c.y, c.z, c.w);
    }
    writer.finish();
  }
}

void atomic_write_sink(TopoUser::AttrSink &s, const int index, const Value &val, const SetMode mode)
{
  if (index < 0 || index >= s.size) {
    return;
  }
  if (s.f) {
    atomic_set_f(s.f[index], val.as_float(), mode);
  }
  else if (s.i) {
    atomic_set_i(s.i[index], val.as_int(), mode);
  }
  else if (s.b) {
    atomic_set_b(s.b[index], val.as_bool(), mode);
  }
  else if (s.v) {
    atomic_set_v3(s.v[index], val.as_vec(), mode);
  }
  else if (s.v2) {
    const float2 p = val.as_vec2();
    atomic_set_f(s.v2[index].x, p.x, mode);
    atomic_set_f(s.v2[index].y, p.y, mode);
  }
  else if (s.c) {
    const float4 c = (val.type == Type::Vector4 || val.type == Type::Color) ?
                         val.v :
                         float4(val.as_vec().x, val.as_vec().y, val.as_vec().z, 1.0f);
    atomic_set_f(s.c[index].x, c.x, mode);
    atomic_set_f(s.c[index].y, c.y, mode);
    atomic_set_f(s.c[index].z, c.z, mode);
    atomic_set_f(s.c[index].w, c.w, mode);
  }
  else if (s.q) {
    const math::Quaternion q = value_as_quat(val);
    atomic_set_f(s.q[index].x, q.x, mode);
    atomic_set_f(s.q[index].y, q.y, mode);
    atomic_set_f(s.q[index].z, q.z, mode);
    atomic_set_f(s.q[index].w, q.w, mode);
  }
}

void set_attribute_atomic(TopoUser &u,
                          const bke::AttrDomain domain,
                          const int index,
                          const StringRef name,
                          const Value &val,
                          const SetMode mode)
{
  if (domain == u.bind_domain) {
    const std::string want = sample_attr_name(name);
    for (const int i : u.attr_infos.index_range()) {
      if (sample_attr_name(u.attr_infos[i].name) != want) {
        continue;
      }
      if (i < u.attr_rt.size()) {
        atomic_write_attrrt(u.attr_rt[i], index, val, mode);
        return;
      }
    }
  }
  if (TopoUser::AttrSink *s = ensure_attr_sink(u, domain, name, type_from_value(val))) {
    atomic_write_sink(*s, index, val, mode);
  }
}

GeometryNodeDeleteGeometryMode parse_delete_mode(const Value &v, const VMEnv &env)
{
  if (v.type == Type::String) {
    const StringRef s = value_string(v, env);
    if (s == "edge_face" || s == "edgeface" || s == "EDGE_FACE" || s == "edges_faces" ||
        s == "only_edges_faces")
    {
      return GEO_NODE_DELETE_GEOMETRY_MODE_EDGE_FACE;
    }
    if (s == "only_face" || s == "onlyface" || s == "ONLY_FACE") {
      return GEO_NODE_DELETE_GEOMETRY_MODE_ONLY_FACE;
    }
    return GEO_NODE_DELETE_GEOMETRY_MODE_ALL;
  }
  const int m = v.as_int();
  if (m == 1) {
    return GEO_NODE_DELETE_GEOMETRY_MODE_EDGE_FACE;
  }
  if (m == 2) {
    return GEO_NODE_DELETE_GEOMETRY_MODE_ONLY_FACE;
  }
  return GEO_NODE_DELETE_GEOMETRY_MODE_ALL;
}

void mark_del_index(Array<uint8_t> &mask, const int index)
{
  if (index >= 0 && index < mask.size()) {
    mask[index] = 1;
  }
}

void mark_del_span(Array<uint8_t> &mask, const Span<int> indices)
{
  for (const int index : indices) {
    mark_del_index(mask, index);
  }
}

void mark_delete_geometry(TopoUser &u,
                          const int target_domain,
                          const int src_domain,
                          const int src_index,
                          const bool expand,
                          const GeometryNodeDeleteGeometryMode mode)
{
  const Mesh *mesh = u.mesh;
  const int use_index = src_index;
  if (!expand || target_domain == src_domain) {
    if (target_domain == int(bke::AttrDomain::Edge)) {
      mark_del_index(u.del_edge, use_index);
      u.delete_mode_edge = mode;
    }
    else if (target_domain == int(bke::AttrDomain::Face)) {
      mark_del_index(u.del_face, use_index);
      u.delete_mode_face = mode;
    }
    else if (target_domain == int(bke::AttrDomain::Curve)) {
      mark_del_index(u.del_curve, use_index);
    }
    else if (target_domain == int(bke::AttrDomain::Instance)) {
      mark_del_index(u.del_inst, use_index);
    }
    else if (target_domain == int(bke::AttrDomain::Corner) && mesh) {
      if (use_index >= 0 && use_index < mesh->corners_num) {
        mark_del_index(u.del_point, mesh->corner_verts()[use_index]);
        u.delete_mode_point = mode;
      }
    }
    else {
      mark_del_index(u.del_point, use_index);
      u.delete_mode_point = mode;
    }
    return;
  }

  /* Current wrangle element → related elements on the requested domain.
   * `delete_geometry("face", i@index)` on points deletes faces of this point, not face #ptnum. */
  if (mesh) {
    if (src_domain == int(bke::AttrDomain::Point)) {
      if (target_domain == int(bke::AttrDomain::Edge)) {
        u.ensure_v2e();
        if (use_index >= 0 && use_index < u.v2e.size()) {
          mark_del_span(u.del_edge, u.v2e[use_index]);
        }
        u.delete_mode_edge = mode;
        return;
      }
      if (target_domain == int(bke::AttrDomain::Face)) {
        const GroupedSpan<int> v2f = mesh->vert_to_face_map();
        if (use_index >= 0 && use_index < v2f.size()) {
          mark_del_span(u.del_face, v2f[use_index]);
        }
        u.delete_mode_face = mode;
        return;
      }
      if (target_domain == int(bke::AttrDomain::Corner)) {
        mark_del_index(u.del_point, use_index);
        u.delete_mode_point = mode;
        return;
      }
    }
    else if (src_domain == int(bke::AttrDomain::Edge)) {
      if (use_index < 0 || use_index >= mesh->edges_num) {
        return;
      }
      const int2 edge = mesh->edges()[use_index];
      if (target_domain == int(bke::AttrDomain::Point)) {
        mark_del_index(u.del_point, edge[0]);
        mark_del_index(u.del_point, edge[1]);
        u.delete_mode_point = mode;
        return;
      }
      if (target_domain == int(bke::AttrDomain::Face)) {
        u.ensure_e2c();
        const Span<int> c2f = mesh->corner_to_face_map();
        if (use_index < u.e2c.size()) {
          for (const int corner : u.e2c[use_index]) {
            if (corner >= 0 && corner < c2f.size()) {
              mark_del_index(u.del_face, c2f[corner]);
            }
          }
        }
        u.delete_mode_face = mode;
        return;
      }
    }
    else if (src_domain == int(bke::AttrDomain::Face)) {
      if (use_index < 0 || use_index >= mesh->faces_num) {
        return;
      }
      const IndexRange face = mesh->faces()[use_index];
      if (target_domain == int(bke::AttrDomain::Point)) {
        mark_del_span(u.del_point, mesh->corner_verts().slice(face));
        u.delete_mode_point = mode;
        return;
      }
      if (target_domain == int(bke::AttrDomain::Edge)) {
        mark_del_span(u.del_edge, mesh->corner_edges().slice(face));
        u.delete_mode_edge = mode;
        return;
      }
    }
    else if (src_domain == int(bke::AttrDomain::Corner)) {
      if (use_index < 0 || use_index >= mesh->corners_num) {
        return;
      }
      if (target_domain == int(bke::AttrDomain::Point)) {
        mark_del_index(u.del_point, mesh->corner_verts()[use_index]);
        u.delete_mode_point = mode;
        return;
      }
      if (target_domain == int(bke::AttrDomain::Edge)) {
        mark_del_index(u.del_edge, mesh->corner_edges()[use_index]);
        u.delete_mode_edge = mode;
        return;
      }
      if (target_domain == int(bke::AttrDomain::Face)) {
        const Span<int> c2f = mesh->corner_to_face_map();
        if (use_index < c2f.size()) {
          mark_del_index(u.del_face, c2f[use_index]);
        }
        u.delete_mode_face = mode;
        return;
      }
    }
  }

  if (u.curves) {
    if (src_domain == int(bke::AttrDomain::Point) &&
        target_domain == int(bke::AttrDomain::Curve))
    {
      u.ensure_point_to_curve();
      if (use_index >= 0 && use_index < u.point_to_curve.size()) {
        mark_del_index(u.del_curve, u.point_to_curve[use_index]);
      }
      return;
    }
    if (src_domain == int(bke::AttrDomain::Curve) &&
        target_domain == int(bke::AttrDomain::Point))
    {
      const OffsetIndices points = u.curves->points_by_curve();
      if (use_index >= 0 && use_index < points.size()) {
        for (const int point : points[use_index]) {
          mark_del_index(u.del_point, point);
        }
      }
      return;
    }
  }

  /* Fallback: treat the given index as an index on the target domain. */
  mark_delete_geometry(u, target_domain, target_domain, use_index, false, mode);
}

Value sample_tri_attr(const Mesh &mesh,
                      const int tri_i,
                      const float3 bary,
                      const StringRef name,
                      Vector<std::string> *interned)
{
  const Span<int3> tris = mesh.corner_tris();
  if (tri_i < 0 || tri_i >= tris.size()) {
    return Value::from_float(0.0f);
  }
  const int3 tri = tris[tri_i];
  const Span<int> corner_verts = mesh.corner_verts();
  const int v0 = corner_verts[tri[0]];
  const int v1 = corner_verts[tri[1]];
  const int v2 = corner_verts[tri[2]];
  if (name.is_empty() || name == "position" || name == "P" || name == "vector") {
    const Span<float3> pos = mesh.vert_positions();
    return Value::from_vec(bke::attribute_math::mix3(bary, pos[v0], pos[v1], pos[v2]));
  }
  const bke::GAttributeReader reader = mesh.attributes().lookup(name);
  if (!reader) {
    return Value::from_float(0.0f);
  }
  if (reader.domain == bke::AttrDomain::Face) {
    const Span<int> tri_faces = mesh.corner_tri_faces();
    const int face = (tri_i < tri_faces.size()) ? tri_faces[tri_i] : tri_i;
    return value_from_varray(reader.varray, face, interned);
  }
  auto mix_three = [&](const int i0, const int i1, const int i2) -> Value {
    const GVArray &va = reader.varray;
    if (va.type().is<float>()) {
      const VArray<float> src = va.typed<float>();
      return Value::from_float(bke::attribute_math::mix3(bary, src[i0], src[i1], src[i2]));
    }
    if (va.type().is<float3>()) {
      const VArray<float3> src = va.typed<float3>();
      return Value::from_vec(bke::attribute_math::mix3(bary, src[i0], src[i1], src[i2]));
    }
    if (va.type().is<int>()) {
      const VArray<int> src = va.typed<int>();
      return Value::from_int(bke::attribute_math::mix3(bary, src[i0], src[i1], src[i2]));
    }
    if (va.type().is<bool>()) {
      const VArray<bool> src = va.typed<bool>();
      return Value::from_bool(bke::attribute_math::mix3(bary, src[i0], src[i1], src[i2]));
    }
    if (va.type().is<ColorGeometry4f>()) {
      const VArray<ColorGeometry4f> src = va.typed<ColorGeometry4f>();
      const ColorGeometry4f c = bke::attribute_math::mix3(bary, src[i0], src[i1], src[i2]);
      return Value::from_vec4(float4(c.r, c.g, c.b, c.a), Type::Color);
    }
    const int pick = (bary.x >= bary.y && bary.x >= bary.z) ? i0 : (bary.y >= bary.z ? i1 : i2);
    return value_from_varray(va, pick, interned);
  };
  if (reader.domain == bke::AttrDomain::Corner) {
    return mix_three(tri[0], tri[1], tri[2]);
  }
  return mix_three(v0, v1, v2);
}

TopoUser::SpatialAccel &spatial_of(TopoUser *u, const int gi)
{
  const int slot = (gi >= 0 && gi < 8) ? gi : 0;
  TopoUser::SpatialAccel &a = u->spatial[slot];
  const bke::GeometrySet *g = geo_at(u, gi);
  const Mesh *mesh = g ? g->get_mesh() : nullptr;
  const PointCloud *pc = g ? g->get_pointcloud() : nullptr;
  if (a.mesh != mesh || a.pc != pc) {
    a = {};
    a.mesh = mesh;
    a.pc = pc;
  }
  return a;
}

void spatial_ensure_tris(TopoUser::SpatialAccel &a)
{
  if (a.tris != nullptr || a.mesh == nullptr || a.mesh->faces_num == 0) {
    return;
  }
  /* Same Embree tree as the Geometry Raycast node. Do not rebuild a second HIGH
   * BVH — that plus a broken packet AABB was slower than the native nodes. */
  a.tris = &a.mesh->bvh_tris();
}

void spatial_ensure_faces(TopoUser *u, TopoUser::SpatialAccel &a)
{
  if (a.faces_ok) {
    return;
  }
  std::lock_guard lock(u->mutex);
  if (a.faces_ok) {
    return;
  }
  if (a.mesh && a.mesh->faces_num > 0) {
    /* Same kdop tree as the Geometry Proximity node. */
    a.faces = a.mesh->bvh_corner_tris();
  }
  a.faces_ok = true;
}

void spatial_ensure_verts(TopoUser *u, TopoUser::SpatialAccel &a)
{
  if (a.verts_ok) {
    return;
  }
  std::lock_guard lock(u->mutex);
  if (a.verts_ok) {
    return;
  }
  if (a.mesh) {
    a.verts = &a.mesh->bvh_verts();
  }
  a.verts_ok = true;
}

void spatial_ensure_edges(TopoUser *u, TopoUser::SpatialAccel &a)
{
  if (a.edges_ok) {
    return;
  }
  std::lock_guard lock(u->mutex);
  if (a.edges_ok) {
    return;
  }
  if (a.mesh) {
    a.edges = &a.mesh->bvh_edges();
  }
  a.edges_ok = true;
}

void spatial_ensure_points(TopoUser *u, TopoUser::SpatialAccel &a)
{
  if (a.points_ok) {
    return;
  }
  std::lock_guard lock(u->mutex);
  if (a.points_ok) {
    return;
  }
  if (a.pc) {
    a.points = &a.pc->bvh_tree();
  }
  a.points_ok = true;
}

bool proximity_query_accel(TopoUser *u,
                           TopoUser::SpatialAccel &a,
                           int domain,
                           float3 sample,
                           float3 &r_pos,
                           float &r_dist);
Span<float3> origins_from_position(TopoUser &u, const VMEnv &env, const Program &program);

void prewarm_spatial(TopoUser &topo, const Program &program)
{
  if (!program_uses_spatial(program)) {
    return;
  }
  const int ngeo = std::max(int(topo.geos.size()), 1);
  auto foreach_required_geo = [&](const uint8_t mask,
                                  const bool dynamic,
                                  const FunctionRef<void(int)> fn) {
    if (dynamic || mask == 0) {
      for (int gi = 0; gi < ngeo && gi < 8; gi++) {
        fn(gi);
      }
      return;
    }
    for (int gi = 0; gi < ngeo && gi < 8; gi++) {
      if (mask & uint8_t(1u << gi)) {
        fn(gi);
      }
    }
  };

  const bool need_ray = program_uses_call(program, Builtin::Raycast) ||
                        program_uses_call(program, Builtin::RaycastAll);
  if (need_ray) {
    foreach_required_geo(program.ray_geo_mask, program.ray_geo_dynamic, [&](const int gi) {
      spatial_ensure_tris(spatial_of(&topo, gi));
    });
  }
  if (program_uses_call(program, Builtin::SampleNearestSurface)) {
    foreach_required_geo(
        program.sample_geo_mask, program.sample_geo_dynamic, [&](const int gi) {
          spatial_ensure_tris(spatial_of(&topo, gi));
        });
  }
  if (program_uses_call(program, Builtin::GeometryProximity)) {
    foreach_required_geo(program.prox_geo_mask, program.prox_geo_dynamic, [&](const int gi) {
      TopoUser::SpatialAccel &a = spatial_of(&topo, gi);
      const int d = program.prox_batch ? program.prox_domain : -1;
      if (d == 2 || d < 0) {
        spatial_ensure_faces(&topo, a);
      }
      if (d == 1 || d < 0) {
        spatial_ensure_edges(&topo, a);
      }
      if (d == 0 || d < 0) {
        spatial_ensure_verts(&topo, a);
        spatial_ensure_points(&topo, a);
      }
    });
  }
}

static Span<float3> attr_positions(const VMEnv &env, const int slot, const int domain_size)
{
  if (slot < 0 || slot >= env.attrs.size()) {
    return {};
  }
  const AttrRT &a = env.attrs[slot];
  const float3 *ptr = a.wv ? a.wv : a.rv;
  if (ptr == nullptr || a.size < domain_size) {
    return {};
  }
  return Span<float3>(ptr, a.size);
}

/* Match official Geometry Node grains: Raycast MF default is 10000,
 * Proximity overrides min_grain_size to 512. Packet Embree on unsorted
 * wrangle rays was ~2× slower — keep scalar rtcIntersect1. */
static constexpr int k_raycast_grain = 10000;
static constexpr int k_proximity_grain = 512;

template<typename Fn>
static void for_spatial_slots(const Vector<int> &all, const int primary, Fn &&fn)
{
  if (!all.is_empty()) {
    for (const int s : all) {
      fn(s);
    }
    return;
  }
  if (primary >= 0) {
    fn(primary);
  }
}

template<typename Fn>
static void spatial_foreach(const IndexMask &mask,
                            const int domain_size,
                            const int grain,
                            Fn &&fn)
{
  if (domain_size > 0 && mask.size() == domain_size) {
    threading::parallel_for(IndexRange(domain_size), grain, [&](const IndexRange range) {
      for (const int64_t i : range) {
        fn(int(i));
      }
    });
    return;
  }
  mask.foreach_index([&](const int i) { fn(i); }, exec_mode::grain_size(grain));
}

static Span<float3> attr_float3_span(const VMEnv &env, const int slot, const int n)
{
  return attr_positions(env, slot, n);
}

void precompute_proximity(TopoUser &u,
                          const Program &program,
                          const IndexMask &mask,
                          const int domain_size,
                          const VMEnv &env)
{
  u.prox_ok = false;
  if (!program.prox_batch || domain_size <= 0 || mask.is_empty()) {
    return;
  }
  Span<float3> samples = {};
  if (program.prox_sample_attr >= 0) {
    samples = attr_float3_span(env, program.prox_sample_attr, domain_size);
  }
  if (samples.is_empty()) {
    samples = origins_from_position(u, env, program);
  }
  if (samples.is_empty()) {
    samples = u.positions;
  }
  if (samples.size() < domain_size) {
    return;
  }
  TopoUser::SpatialAccel &accel = spatial_of(&u, program.prox_geo);
  const int domain = program.prox_domain;
  /* Same kdop trees as Geometry Proximity. Do not build Embree here. */
  if (domain == 2) {
    spatial_ensure_faces(&u, accel);
  }
  else if (domain == 1) {
    spatial_ensure_edges(&u, accel);
  }
  else if (accel.mesh) {
    spatial_ensure_verts(&u, accel);
  }
  else {
    spatial_ensure_points(&u, accel);
  }
  u.prox_geo = program.prox_geo;
  u.prox_domain = domain;
  u.prox_samples = samples;

  /* Always fill cache arrays so a later VM Call does not re-query (that was ~100×). */
  u.prox_pos.reinitialize(domain_size);
  u.prox_dist.reinitialize(domain_size);
  Vector<float3 *> dst_pos;
  Vector<float *> dst_dist;
  for_spatial_slots(program.prox_pos_attrs, program.prox_pos_attr, [&](const int slot) {
    if (slot < env.attrs.size() && env.attrs[slot].wv) {
      dst_pos.append(env.attrs[slot].wv);
    }
  });
  for_spatial_slots(program.prox_dist_attrs, program.prox_dist_attr, [&](const int slot) {
    if (slot < env.attrs.size() && env.attrs[slot].wf) {
      dst_dist.append(env.attrs[slot].wf);
    }
  });

  spatial_foreach(mask, domain_size, k_proximity_grain, [&](const int i) {
    float3 pos(0.0f);
    float dist = 0.0f;
    proximity_query_accel(&u, accel, domain, samples[i], pos, dist);
    u.prox_pos[i] = pos;
    u.prox_dist[i] = dist;
    for (float3 *p : dst_pos) {
      p[i] = pos;
    }
    for (float *p : dst_dist) {
      p[i] = dist;
    }
  });
  u.prox_ok = true;
  u.spatial_wrote_attrs = u.spatial_wrote_attrs || !dst_pos.is_empty() || !dst_dist.is_empty();
}

Span<float3> origins_from_position(TopoUser &u, const VMEnv &env, const Program &program)
{
  for (const int i : program.attrs.index_range()) {
    if (i >= env.attrs.size()) {
      break;
    }
    if (program.attrs[i].name == "position" || program.attrs[i].name == "P") {
      const float3 *ptr = env.attrs[i].rv ? env.attrs[i].rv : env.attrs[i].wv;
      if (ptr != nullptr && env.attrs[i].size > 0) {
        return Span<float3>(ptr, env.attrs[i].size);
      }
    }
  }
  return u.positions;
}

bool position_written_before_spatial(const Program &program)
{
  int pos_slot = -1;
  for (const int i : program.attrs.index_range()) {
    if (program.attrs[i].name == "position" || program.attrs[i].name == "P") {
      pos_slot = i;
      break;
    }
  }
  if (pos_slot < 0) {
    return false;
  }
  bool wrote = false;
  for (const Inst &in : program.code) {
    if ((in.op == Op::StoreAttr || in.op == Op::AttrAdd || in.op == Op::AttrMulAdd ||
         in.op == Op::WhileCmpAdd) &&
        ((in.op == Op::WhileCmpAdd) ?
             (in.imm >= 0 && in.imm < program.while_adds.size() &&
              program.while_adds[in.imm].attr == pos_slot) :
             (in.imm == pos_slot)))
    {
      wrote = true;
    }
    if (in.op == Op::Call &&
        ELEM(in.imm,
             int(Builtin::Raycast),
             int(Builtin::RaycastAll),
             int(Builtin::GeometryProximity),
             int(Builtin::SampleNearestSurface)))
    {
      if (wrote) {
        return true;
      }
    }
  }
  return false;
}

void precompute_raycast(TopoUser &u,
                        const Program &program,
                        const IndexMask &mask,
                        const int domain_size,
                        const VMEnv &env)
{
  u.ray_ok = false;
  if (!program.ray_batch || domain_size <= 0 || mask.is_empty()) {
    return;
  }
  Span<float3> origins = {};
  if (program.ray_orig_attr >= 0) {
    origins = attr_positions(env, program.ray_orig_attr, domain_size);
  }
  if (origins.is_empty()) {
    origins = origins_from_position(u, env, program);
  }
  if (origins.is_empty()) {
    origins = u.positions;
  }
  if (origins.size() < domain_size) {
    return;
  }
  TopoUser::SpatialAccel &accel = spatial_of(&u, program.ray_geo);
  spatial_ensure_tris(accel);
  if (accel.tris == nullptr || accel.mesh == nullptr) {
    return;
  }
  u.ray_geo = program.ray_geo;
  Span<float3> dirs = {};
  if (program.ray_dir_attr >= 0) {
    dirs = attr_positions(env, program.ray_dir_attr, domain_size);
  }
  const float3 const_dir = program.ray_dir;
  const float length = program.ray_len;
  const bool normalize_dir = program.ray_dir_normalize;
  const bke::bvh::Tree &tree = *accel.tris;

  Vector<int *> dst_hit_i;
  Vector<bool *> dst_hit_b;
  Vector<float *> dst_hit_f;
  Vector<float3 *> dst_pos;
  Vector<float3 *> dst_n;
  Vector<float *> dst_dist;
  for_spatial_slots(program.ray_hit_attrs, program.ray_hit_attr, [&](const int slot) {
    if (slot < 0 || slot >= env.attrs.size()) {
      return;
    }
    AttrRT &a = env.attrs[slot];
    if (a.wi) {
      dst_hit_i.append(a.wi);
    }
    else if (a.wb) {
      dst_hit_b.append(a.wb);
    }
    else if (a.wf) {
      dst_hit_f.append(a.wf);
    }
  });
  for_spatial_slots(program.ray_pos_attrs, program.ray_pos_attr, [&](const int slot) {
    if (slot < env.attrs.size() && env.attrs[slot].wv) {
      dst_pos.append(env.attrs[slot].wv);
    }
  });
  for_spatial_slots(program.ray_n_attrs, program.ray_n_attr, [&](const int slot) {
    if (slot < env.attrs.size() && env.attrs[slot].wv) {
      dst_n.append(env.attrs[slot].wv);
    }
  });
  for_spatial_slots(program.ray_dist_attrs, program.ray_dist_attr, [&](const int slot) {
    if (slot < env.attrs.size() && env.attrs[slot].wf) {
      dst_dist.append(env.attrs[slot].wf);
    }
  });
  const bool direct = !dst_hit_i.is_empty() || !dst_hit_b.is_empty() || !dst_hit_f.is_empty() ||
                      !dst_pos.is_empty() || !dst_n.is_empty() || !dst_dist.is_empty();
  u.ray_hit.reinitialize(domain_size);
  u.ray_face.reinitialize(domain_size);
  u.ray_pos.reinitialize(domain_size);
  u.ray_n.reinitialize(domain_size);
  u.ray_dist.reinitialize(domain_size);
  int8_t *tmp_hit = u.ray_hit.data();
  int *tmp_face = u.ray_face.data();
  float3 *tmp_pos = u.ray_pos.data();
  float3 *tmp_n = u.ray_n.data();
  float *tmp_dist = u.ray_dist.data();

  /* Same scalar Embree path as GeometryNodeRaycast. rtcIntersect8 + COHERENT
   * on unsorted wrangle rays was about 2× slower than rtcIntersect1. */
  spatial_foreach(mask, domain_size, k_raycast_grain, [&](const int i) {
    bke::bvh::Ray ray{};
    ray.origin = origins[i];
    float3 dir = dirs.is_empty() ? const_dir : dirs[i];
    if (normalize_dir) {
      dir = math::normalize(dir);
    }
    ray.direction = dir;
    ray.dist_max = length;
    if (const std::optional<bke::bvh::RayHit> hit = tree.ray_intersect(ray)) {
      for (int *p : dst_hit_i) {
        p[i] = 1;
      }
      for (bool *p : dst_hit_b) {
        p[i] = true;
      }
      for (float *p : dst_hit_f) {
        p[i] = 1.0f;
      }
      const float3 hp = hit->position(ray);
      const float3 hn = math::normalize(hit->normal);
      for (float3 *p : dst_pos) {
        p[i] = hp;
      }
      for (float3 *p : dst_n) {
        p[i] = hn;
      }
      for (float *p : dst_dist) {
        p[i] = hit->distance;
      }
      if (tmp_hit) {
        tmp_hit[i] = 1;
        tmp_face[i] = hit->index;
        tmp_pos[i] = hp;
        tmp_n[i] = hn;
        tmp_dist[i] = hit->distance;
      }
    }
    else {
      for (int *p : dst_hit_i) {
        p[i] = 0;
      }
      for (bool *p : dst_hit_b) {
        p[i] = false;
      }
      for (float *p : dst_hit_f) {
        p[i] = 0.0f;
      }
      for (float3 *p : dst_pos) {
        p[i] = float3(0.0f);
      }
      for (float3 *p : dst_n) {
        p[i] = float3(0.0f);
      }
      for (float *p : dst_dist) {
        p[i] = length;
      }
      if (tmp_hit) {
        tmp_hit[i] = 0;
        tmp_face[i] = -1;
        tmp_pos[i] = float3(0.0f);
        tmp_n[i] = float3(0.0f);
        tmp_dist[i] = length;
      }
    }
  });
  u.ray_ok = true;
  u.spatial_wrote_attrs = u.spatial_wrote_attrs || direct;
}

void precompute_sample(TopoUser &u,
                       const Program &program,
                       const IndexMask &mask,
                       const int domain_size,
                       const VMEnv &env)
{
  u.sample_ok = false;
  if (!program.sample_batch || domain_size <= 0 || mask.is_empty()) {
    return;
  }
  Span<float3> origins = origins_from_position(u, env, program);
  if (origins.size() < domain_size) {
    return;
  }
  TopoUser::SpatialAccel &accel = spatial_of(&u, program.sample_geo);
  spatial_ensure_tris(accel);
  if (accel.tris == nullptr || accel.mesh == nullptr) {
    return;
  }
  const Mesh &mesh = *accel.mesh;
  const StringRef name = program.sample_attr.empty() ? StringRef("position") :
                                                       StringRef(program.sample_attr);
  const bool want_vec = name.is_empty() || name == "position" || name == "P" || name == "vector";
  u.sample_geo = program.sample_geo;
  u.sample_is_vec = want_vec;
  if (want_vec) {
    u.sample_v.reinitialize(domain_size);
    u.sample_v.fill(float3(0.0f));
  }
  else {
    u.sample_f.reinitialize(domain_size);
    u.sample_f.fill(0.0f);
  }
  const bke::bvh::Tree *tree = accel.tris;
  mask.foreach_index(
      [&](const int64_t i) {
        const std::optional<bke::bvh::ClosestPointResult> closest = tree->closest_point(
            origins[int(i)]);
        if (!closest) {
          return;
        }
        const Value v = sample_tri_attr(
            mesh, int(closest->index), closest->bary_coord, name, nullptr);
        if (want_vec) {
          u.sample_v[i] = v.as_vec();
        }
        else {
          u.sample_f[i] = v.as_float();
        }
      },
      exec_mode::grain_size(256));
  u.sample_ok = true;
}

bool proximity_query_accel(TopoUser *u,
                           TopoUser::SpatialAccel &a,
                           const int domain,
                           const float3 sample,
                           float3 &r_pos,
                           float &r_dist)
{
  r_pos = float3(0.0f);
  r_dist = 0.0f;

  BVHTreeNearest nearest;
  nearest.index = -1;
  nearest.dist_sq = FLT_MAX;
  nearest.co[0] = nearest.co[1] = nearest.co[2] = 0.0f;

  /* Faces: same kdop tree as Geometry Proximity. Embree point-query is slower
   * than BLI_bvhtree_find_nearest at ~1M sample points. */
  if (domain == 2) {
    spatial_ensure_faces(u, a);
    if (a.faces.tree) {
      BLI_bvhtree_find_nearest(a.faces.tree,
                               sample,
                               &nearest,
                               a.faces.nearest_callback,
                               const_cast<bke::BVHTreeFromMesh *>(&a.faces));
    }
  }
  else if (domain == 1) {
    spatial_ensure_edges(u, a);
    if (a.edges) {
      if (const std::optional<bke::bvh::ClosestPointResult> hit = a.edges->closest_point(sample))
      {
        nearest.index = int(hit->index);
        nearest.dist_sq = math::distance_squared(sample, hit->position);
        copy_v3_v3(nearest.co, hit->position);
      }
    }
  }
  else if (a.mesh) {
    spatial_ensure_verts(u, a);
    if (a.verts) {
      if (const std::optional<bke::bvh::ClosestPointResult> hit = a.verts->closest_point(sample))
      {
        nearest.index = int(hit->index);
        nearest.dist_sq = math::distance_squared(sample, hit->position);
        copy_v3_v3(nearest.co, hit->position);
      }
    }
  }
  else {
    spatial_ensure_points(u, a);
    if (a.points) {
      if (const std::optional<bke::bvh::ClosestPointResult> hit = a.points->closest_point(sample))
      {
        nearest.index = int(hit->index);
        nearest.dist_sq = math::distance_squared(sample, hit->position);
        copy_v3_v3(nearest.co, hit->position);
      }
    }
  }
  if (nearest.index < 0) {
    return false;
  }
  r_pos = float3(nearest.co[0], nearest.co[1], nearest.co[2]);
  r_dist = std::sqrt(nearest.dist_sq);
  return true;
}

static void copy_vec_attr(AttrRT &a, const Span<float3> src, const IndexMask &mask)
{
  if (!a.wv || src.is_empty()) {
    return;
  }
  const int n = std::min(a.size, int(src.size()));
  if (n <= 0) {
    return;
  }
  if (mask.size() == n) {
    memcpy(a.wv, src.data(), sizeof(float3) * size_t(n));
    return;
  }
  mask.foreach_index_optimized<int>(
      [&](const int i) {
        if (i >= 0 && i < n) {
          a.wv[i] = src[i];
        }
      },
      exec_mode::grain_size(4096));
}

static void copy_float_attr(AttrRT &a, const Span<float> src, const IndexMask &mask)
{
  if (!a.wf || src.is_empty()) {
    return;
  }
  const int n = std::min(a.size, int(src.size()));
  if (n <= 0) {
    return;
  }
  if (mask.size() == n) {
    memcpy(a.wf, src.data(), sizeof(float) * size_t(n));
    return;
  }
  mask.foreach_index_optimized<int>(
      [&](const int i) {
        if (i >= 0 && i < n) {
          a.wf[i] = src[i];
        }
      },
      exec_mode::grain_size(4096));
}

static void copy_hit_attr(AttrRT &a, const Span<int8_t> src, const IndexMask &mask)
{
  const int n = std::min(a.size, int(src.size()));
  if (n <= 0) {
    return;
  }
  auto write = [&](const int i) {
    if (i < 0 || i >= n) {
      return;
    }
    const int hit = int(src[i]);
    if (a.wi) {
      a.wi[i] = hit;
    }
    else if (a.wb) {
      a.wb[i] = hit != 0;
    }
    else if (a.wf) {
      a.wf[i] = float(hit);
    }
  };
  if (mask.size() == n) {
    for (int i = 0; i < n; i++) {
      write(i);
    }
    return;
  }
  mask.foreach_index_optimized<int>(write, exec_mode::grain_size(4096));
}

bool apply_spatial_kernel(TopoUser &u,
                          const Program &program,
                          VMEnv &env,
                          const IndexMask &mask,
                          const int domain_size)
{
  if (!program.spatial_only || domain_size <= 0 || mask.is_empty()) {
    return false;
  }
  if (program.prox_batch && !u.prox_ok) {
    return false;
  }
  if (program.ray_batch && !u.ray_ok) {
    return false;
  }
  if (u.spatial_wrote_attrs) {
    return true;
  }
  auto attr = [&](const int slot) -> AttrRT * {
    if (slot < 0 || slot >= env.attrs.size()) {
      return nullptr;
    }
    return &env.attrs[slot];
  };
  if (u.prox_ok) {
    for_spatial_slots(program.prox_pos_attrs, program.prox_pos_attr, [&](const int slot) {
      if (AttrRT *a = attr(slot)) {
        copy_vec_attr(*a, u.prox_pos, mask);
      }
    });
    for_spatial_slots(program.prox_dist_attrs, program.prox_dist_attr, [&](const int slot) {
      if (AttrRT *a = attr(slot)) {
        copy_float_attr(*a, u.prox_dist, mask);
      }
    });
  }
  if (u.ray_ok) {
    for_spatial_slots(program.ray_hit_attrs, program.ray_hit_attr, [&](const int slot) {
      if (AttrRT *a = attr(slot)) {
        copy_hit_attr(*a, u.ray_hit, mask);
      }
    });
    for_spatial_slots(program.ray_pos_attrs, program.ray_pos_attr, [&](const int slot) {
      if (AttrRT *a = attr(slot)) {
        copy_vec_attr(*a, u.ray_pos, mask);
      }
    });
    for_spatial_slots(program.ray_n_attrs, program.ray_n_attr, [&](const int slot) {
      if (AttrRT *a = attr(slot)) {
        copy_vec_attr(*a, u.ray_n, mask);
      }
    });
    for_spatial_slots(program.ray_dist_attrs, program.ray_dist_attr, [&](const int slot) {
      if (AttrRT *a = attr(slot)) {
        copy_float_attr(*a, u.ray_dist, mask);
      }
    });
  }
  return true;
}

Value geo_builtin_fn(void *user,
                     const int builtin_id,
                     const Span<Value> args,
                     VMEnv &env,
                     std::string & /*error*/)
{
  auto *u = static_cast<TopoUser *>(user);
  const Builtin id = Builtin(builtin_id);
  if (!u) {
    return Value::from_int(0);
  }

  switch (id) {
    case Builtin::CornersOfFace: {
      if (!u->mesh) {
        return empty_int_arr();
      }
      int face = args.is_empty() ? -1 : arg_or_index(args, 0, env.index);
      if (args.is_empty()) {
        if (env.index >= 0 && env.index < u->mesh->faces_num) {
          face = env.index;
        }
        else {
          const GroupedSpan<int> v2f = u->mesh->vert_to_face_map();
          if (env.index >= 0 && env.index < v2f.size() && !v2f[env.index].is_empty()) {
            face = v2f[env.index][0];
          }
        }
      }
      if (face < 0 || face >= u->mesh->faces_num) {
        return empty_int_arr();
      }
      return range_to_int_arr(u->mesh->faces()[face]);
    }
    case Builtin::CornersOfVertex: {
      if (!u->mesh) {
        return empty_int_arr();
      }
      const int v = arg_or_index(args, 0, env.index);
      const GroupedSpan<int> map = u->mesh->vert_to_corner_map();
      if (v < 0 || v >= map.size()) {
        return empty_int_arr();
      }
      return span_to_int_arr(map[v]);
    }
    case Builtin::CornersOfEdge: {
      int gi = 0;
      int e = env.index;
      parse_geo_elem(args, env.index, gi, e);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh) {
        return empty_int_arr();
      }
      MeshTopoCache &cache = mesh_topo(mesh);
      cache.ensure_e2c();
      if (e < 0 || e >= cache.e2c.size()) {
        return empty_int_arr();
      }
      return span_to_int_arr(cache.e2c[e]);
    }
    case Builtin::EdgesOfVertex: {
      if (!u->mesh) {
        return empty_int_arr();
      }
      u->ensure_v2e();
      const int v = arg_or_index(args, 0, env.index);
      if (v < 0 || v >= u->v2e.size()) {
        return empty_int_arr();
      }
      return span_to_int_arr(u->v2e[v]);
    }
    case Builtin::EdgesOfCorner: {
      int gi = 0;
      int c = env.index;
      parse_geo_elem(args, env.index, gi, c);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh || c < 0 || c >= mesh->corners_num) {
        return empty_int_arr();
      }
      const Span<int> corner_edges = mesh->corner_edges();
      const Span<int> c2f = mesh->corner_to_face_map();
      if (c >= c2f.size()) {
        return empty_int_arr();
      }
      const IndexRange face = mesh->faces()[c2f[c]];
      Vector<int> vals;
      /* [0] = prev (incoming), [1] = next (outgoing along face winding). */
      vals.append(corner_edges[bke::mesh::face_corner_prev(face, c)]);
      vals.append(corner_edges[c]);
      return value_from_int_array(std::move(vals));
    }
    case Builtin::FaceOfCorner: {
      if (!u->mesh) {
        return Value::from_int(-1);
      }
      const int c = arg_or_index(args, 0, env.index);
      const Span<int> c2f = u->mesh->corner_to_face_map();
      return Value::from_int((c >= 0 && c < c2f.size()) ? c2f[c] : -1);
    }
    case Builtin::VertexOfCorner: {
      int gi = 0;
      int c = env.index;
      parse_geo_elem(args, env.index, gi, c);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh) {
        return Value::from_int(-1);
      }
      const Span<int> cv = mesh->corner_verts();
      return Value::from_int((c >= 0 && c < cv.size()) ? cv[c] : -1);
    }
    case Builtin::OffsetCornerInFace: {
      int gi = 0;
      int c = env.index;
      int off = 1;
      if (args.size() >= 3) {
        gi = value_as_elem_index(args[0], 0);
        c = value_as_elem_index(args[1], env.index);
        off = args[2].as_int();
      }
      else {
        parse_geo_elem(args, env.index, gi, c);
      }
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh) {
        return Value::from_int(-1);
      }
      const Span<int> c2f = mesh->corner_to_face_map();
      if (c < 0 || c >= c2f.size()) {
        return Value::from_int(-1);
      }
      const IndexRange face = mesh->faces()[c2f[c]];
      const int local = c - int(face.start());
      return Value::from_int(int(face.start()) + math::mod_periodic<int>(local + off, int(face.size())));
    }
    case Builtin::FacesOfVertex: {
      if (!u->mesh) {
        return empty_int_arr();
      }
      const int v = arg_or_index(args, 0, env.index);
      const GroupedSpan<int> map = u->mesh->vert_to_face_map();
      if (v < 0 || v >= map.size()) {
        return empty_int_arr();
      }
      return span_to_int_arr(map[v]);
    }
    case Builtin::Neighbours: {
      if (!u->mesh) {
        return empty_int_arr();
      }
      u->ensure_v2e();
      const int v = arg_or_index(args, 0, env.index);
      if (v < 0 || v >= u->v2e.size()) {
        return empty_int_arr();
      }
      const Span<int2> edges = u->mesh->edges();
      Vector<int> vals;
      vals.reserve(u->v2e[v].size());
      for (const int e : u->v2e[v]) {
        const int2 ev = edges[e];
        vals.append(ev[0] == v ? ev[1] : ev[0]);
      }
      return value_from_int_array(std::move(vals));
    }
    case Builtin::PointNeighbours: {
      int gi = 0;
      int v = env.index;
      parse_geo_elem(args, env.index, gi, v);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh) {
        const bke::CurvesGeometry *cg = curves_at(u, gi);
        if (cg == nullptr) {
          return empty_int_arr();
        }
        if (cg == u->curves) {
          u->ensure_point_to_curve();
        }
        const Array<int> p2c_tmp = (cg == u->curves) ? Array<int>() : cg->point_to_curve_map();
        const Span<int> p2c = (cg == u->curves) ? u->point_to_curve.as_span() : p2c_tmp.as_span();
        if (v < 0 || v >= p2c.size()) {
          return empty_int_arr();
        }
        const int ci = p2c[v];
        const OffsetIndices<int> pts = cg->points_by_curve();
        if (ci < 0 || ci >= pts.size()) {
          return empty_int_arr();
        }
        const IndexRange curve_pts = pts[ci];
        const int n = int(curve_pts.size());
        if (n <= 1) {
          return empty_int_arr();
        }
        const int local = v - int(curve_pts.start());
        const VArray<bool> cyclic = cg->cyclic();
        const bool wrap = (ci < cyclic.size()) ? cyclic[ci] : false;
        Vector<int> vals;
        if (local > 0) {
          vals.append(curve_pts[local - 1]);
        }
        else if (wrap) {
          vals.append(curve_pts[n - 1]);
        }
        if (local < n - 1) {
          vals.append(curve_pts[local + 1]);
        }
        else if (wrap) {
          vals.append(curve_pts[0]);
        }
        return value_from_int_array(std::move(vals));
      }
      Vector<int> vals;
      GroupedSpan<int> v2e_span;
      if (mesh == u->mesh) {
        u->ensure_v2e();
        v2e_span = u->v2e;
      }
      else {
        MeshTopoCache &cache = mesh_topo(mesh);
        cache.ensure_v2e();
        v2e_span = cache.v2e;
      }
      if (v >= 0 && v < v2e_span.size()) {
        const Span<int2> edges = mesh->edges();
        vals.reserve(v2e_span[v].size());
        for (const int e : v2e_span[v]) {
          if (e < 0 || e >= edges.size()) {
            continue;
          }
          const int2 ev = edges[e];
          vals.append(ev[0] == v ? ev[1] : ev[0]);
        }
      }
      if (vals.is_empty()) {
        const GroupedSpan<int> v2c = mesh->vert_to_corner_map();
        if (v >= 0 && v < v2c.size()) {
          const Span<int> cv = mesh->corner_verts();
          const Span<int> c2f = mesh->corner_to_face_map();
          const OffsetIndices<int> faces = mesh->faces();
          for (const int c : v2c[v]) {
            if (c < 0 || c >= c2f.size()) {
              continue;
            }
            const int f = c2f[c];
            if (f < 0 || f >= mesh->faces_num) {
              continue;
            }
            const IndexRange face = faces[f];
            append_unique_int(vals, cv[bke::mesh::face_corner_prev(face, c)]);
            append_unique_int(vals, cv[bke::mesh::face_corner_next(face, c)]);
          }
        }
      }
      return value_from_int_array(std::move(vals));
    }
    case Builtin::PointEdges: {
      int gi = 0;
      int v = env.index;
      parse_geo_elem(args, env.index, gi, v);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh) {
        return empty_int_arr();
      }
      MeshTopoCache &cache = mesh_topo(mesh);
      cache.ensure_v2e();
      const GroupedSpan<int> v2e = cache.v2e;
      if (v < 0 || v >= v2e.size()) {
        return empty_int_arr();
      }
      return span_to_int_arr(v2e[v]);
    }
    case Builtin::PointFaces: {
      int gi = 0;
      int v = env.index;
      parse_geo_elem(args, env.index, gi, v);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh) {
        return empty_int_arr();
      }
      const GroupedSpan<int> map = mesh->vert_to_face_map();
      if (v < 0 || v >= map.size()) {
        return empty_int_arr();
      }
      return span_to_int_arr(map[v]);
    }
    case Builtin::PointCorners: {
      int gi = 0;
      int v = env.index;
      parse_geo_elem(args, env.index, gi, v);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh) {
        return empty_int_arr();
      }
      const GroupedSpan<int> map = mesh->vert_to_corner_map();
      if (v < 0 || v >= map.size()) {
        return empty_int_arr();
      }
      return span_to_int_arr(map[v]);
    }
    case Builtin::EdgePoints: {
      int gi = 0;
      int e = env.index;
      parse_geo_elem(args, env.index, gi, e);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh) {
        return empty_int_arr();
      }
      const Span<int2> edges = mesh->edges();
      if (e < 0 || e >= edges.size()) {
        return empty_int_arr();
      }
      Vector<int> vals;
      vals.append(edges[e][0]);
      vals.append(edges[e][1]);
      return value_from_int_array(std::move(vals));
    }
    case Builtin::EdgeFaces: {
      int gi = 0;
      int e = env.index;
      parse_geo_elem(args, env.index, gi, e);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh) {
        return empty_int_arr();
      }
      MeshTopoCache &cache = mesh_topo(mesh);
      cache.ensure_e2f();
      const GroupedSpan<int> e2f = cache.e2f;
      if (e < 0 || e >= e2f.size()) {
        return empty_int_arr();
      }
      return span_to_int_arr(e2f[e]);
    }
    case Builtin::FacePoints: {
      int gi = 0;
      int f = env.index;
      parse_geo_elem(args, env.index, gi, f);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh || f < 0 || f >= mesh->faces_num) {
        return empty_int_arr();
      }
      const IndexRange face = mesh->faces()[f];
      const Span<int> cv = mesh->corner_verts();
      Vector<int> vals;
      vals.reserve(face.size());
      for (const int c : face) {
        vals.append(cv[c]);
      }
      return value_from_int_array(std::move(vals));
    }
    case Builtin::FaceEdges: {
      int gi = 0;
      int f = env.index;
      parse_geo_elem(args, env.index, gi, f);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh || f < 0 || f >= mesh->faces_num) {
        return empty_int_arr();
      }
      const IndexRange face = mesh->faces()[f];
      const Span<int> ce = mesh->corner_edges();
      Vector<int> vals;
      vals.reserve(face.size());
      for (const int c : face) {
        vals.append(ce[c]);
      }
      return value_from_int_array(std::move(vals));
    }
    case Builtin::FaceNeighbours: {
      int gi = 0;
      int f = env.index;
      parse_geo_elem(args, env.index, gi, f);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh || f < 0 || f >= mesh->faces_num) {
        return empty_int_arr();
      }
      MeshTopoCache &cache = mesh_topo(mesh);
      cache.ensure_e2f();
      const GroupedSpan<int> e2f = cache.e2f;
      const IndexRange face = mesh->faces()[f];
      const Span<int> ce = mesh->corner_edges();
      Vector<int> vals;
      for (const int c : face) {
        const int e = ce[c];
        if (e < 0 || e >= e2f.size()) {
          continue;
        }
        for (const int nf : e2f[e]) {
          if (nf != f) {
            append_unique_int(vals, nf);
          }
        }
      }
      return value_from_int_array(std::move(vals));
    }
    case Builtin::FaceCorners: {
      int gi = 0;
      int f = env.index;
      parse_geo_elem(args, env.index, gi, f);
      const Mesh *mesh = mesh_at(u, gi);
      if (!mesh || f < 0 || f >= mesh->faces_num) {
        return empty_int_arr();
      }
      return range_to_int_arr(mesh->faces()[f]);
    }
    case Builtin::CornerFace: {
      int gi = 0;
      int c = env.index;
      parse_geo_elem(args, env.index, gi, c);
      const Mesh *mesh = mesh_at(u, gi);
      Vector<int> vals;
      vals.append(-1);
      vals.append(-1);
      if (mesh) {
        const Span<int> c2f = mesh->corner_to_face_map();
        if (c >= 0 && c < c2f.size()) {
          const int f = c2f[c];
          vals[0] = f;
          if (f >= 0 && f < mesh->faces_num) {
            vals[1] = c - int(mesh->faces()[f].start());
          }
        }
      }
      return value_from_int_array(std::move(vals));
    }
    case Builtin::PointsOfCurve: {
      int gi = 0;
      int c = env.index;
      parse_geo_elem(args, env.index, gi, c);
      const bke::CurvesGeometry *cg = curves_at(u, gi);
      if (!cg) {
        return empty_int_arr();
      }
      const OffsetIndices pts = cg->points_by_curve();
      if (c < 0 || c >= pts.size()) {
        return empty_int_arr();
      }
      return range_to_int_arr(pts[c]);
    }
    case Builtin::CurveOfPoint: {
      int gi = 0;
      int p = env.index;
      parse_geo_elem(args, env.index, gi, p);
      const bke::CurvesGeometry *cg = curves_at(u, gi);
      if (!cg) {
        return Value::from_int(-1);
      }
      const Array<int> p2c_tmp = (cg == u->curves) ? Array<int>() : cg->point_to_curve_map();
      if (cg == u->curves) {
        u->ensure_point_to_curve();
      }
      const Span<int> p2c = (cg == u->curves) ? u->point_to_curve.as_span() : p2c_tmp.as_span();
      return Value::from_int((p >= 0 && p < p2c.size()) ? p2c[p] : -1);
    }
    case Builtin::PointCurve: {
      int gi = 0;
      int p = env.index;
      parse_geo_elem(args, env.index, gi, p);
      Vector<int> vals;
      vals.append(-1);
      vals.append(-1);
      const bke::CurvesGeometry *cg = curves_at(u, gi);
      if (!cg) {
        return value_from_int_array(std::move(vals));
      }
      const OffsetIndices pts = cg->points_by_curve();
      const Array<int> p2c_tmp = (cg == u->curves) ? Array<int>() : cg->point_to_curve_map();
      if (cg == u->curves) {
        u->ensure_point_to_curve();
      }
      const Span<int> p2c = (cg == u->curves) ? u->point_to_curve.as_span() : p2c_tmp.as_span();
      if (p >= 0 && p < p2c.size()) {
        const int ci = p2c[p];
        vals[0] = ci;
        if (ci >= 0 && ci < pts.size()) {
          vals[1] = p - int(pts[ci].start());
        }
      }
      return value_from_int_array(std::move(vals));
    }
    case Builtin::NearestPoints:
    case Builtin::NearPoints: {
      /* nearestpoints(geo, k_or_r, mode)
       *   mode 0/"k":  k-nearest
       *   mode 1/"r":  all within radius
       *   mode 2/"rk": at most k within radius
       * Signatures for rk: nearestpoints(geo, r, k, "rk") or nearestpoints(geo, r, "rk", k).
       * Optional last arg is sample position. */
      int geo = 0;
      int k = 8;
      float radius = 1.0f;
      int mode = (id == Builtin::NearPoints) ? 1 : 0;
      float3 pos = (env.index >= 0 && env.index < u->positions.size()) ? u->positions[env.index] :
                                                                         float3(0.0f);
      bool custom_pos = false;
      const int narg = int(args.size());
      auto is_str = [](const Value &v) { return v.type == Type::String; };
      auto is_vec = [](const Value &v) {
        return v.type == Type::Vector || v.type == Type::Color;
      };
      auto mode_of = [&](const Value &v) -> int {
        if (is_str(v)) {
          return nearestpoints_mode_from_name(value_string(v, env));
        }
        const int m = v.as_int();
        if (m == 2) {
          return 2;
        }
        return m != 0 ? 1 : 0;
      };
      if (narg >= 4 && (is_str(args[3]) || (args[3].type == Type::Int && args[3].as_int() == 2 &&
                                           !is_str(args[2]))))
      {
        geo = args[0].as_int();
        radius = args[1].as_float();
        k = std::max(args[2].as_int(), 0);
        mode = mode_of(args[3]);
        if (narg >= 5) {
          pos = args[4].as_vec();
          custom_pos = true;
        }
      }
      else if (narg >= 4 && ((is_str(args[2]) && mode_of(args[2]) == 2) ||
                             (args[2].type == Type::Int && args[2].as_int() == 2)) &&
               !is_vec(args[3]))
      {
        geo = args[0].as_int();
        radius = args[1].as_float();
        mode = 2;
        k = std::max(args[3].as_int(), 0);
        if (narg >= 5) {
          pos = args[4].as_vec();
          custom_pos = true;
        }
      }
      else if (narg >= 3) {
        geo = args[0].as_int();
        k = std::max(args[1].as_int(), 0);
        radius = args[1].as_float();
        mode = mode_of(args[2]);
        if (narg >= 4) {
          pos = args[3].as_vec();
          custom_pos = true;
        }
      }
      else if (narg == 2) {
        geo = args[0].as_int();
        k = std::max(args[1].as_int(), 0);
        radius = args[1].as_float();
        if (args[1].type == Type::Float) {
          mode = 1;
        }
      }
      else if (narg == 1) {
        k = std::max(args[0].as_int(), 0);
        radius = args[0].as_float();
        if (args[0].type == Type::Float) {
          mode = 1;
        }
      }
      k = std::max(k, 0);
      const int skip_id = (geo <= 0) ? env.index : -1;
      /* Same neighbors every call at current P: index a CSR table built once. */
      if (mode == 0 && !custom_pos && u->knn_ok && geo == u->knn_geo && k == u->knn_k &&
          env.index >= 0 && env.index + 1 < u->knn_off.size())
      {
        const int a = u->knn_off[env.index];
        const int b = u->knn_off[env.index + 1];
        const int n = std::max(0, b - a);
        if (a >= 0 && a + n <= u->knn_idx.size()) {
          return value_from_int_span(u->knn_idx.as_span().slice(a, n));
        }
      }
      KDTree<float3> *tree = u->tree_for(geo);
      if (!tree) {
        return empty_int_arr();
      }
      if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) {
        return empty_int_arr();
      }
      auto skip_append = [&](Vector<int> &ids, const int index, const int cap) {
        if (skip_id >= 0 && index == skip_id) {
          return;
        }
        if (cap >= 0 && ids.size() >= cap) {
          return;
        }
        ids.append(index);
      };
      if (mode == 1) {
        KDTreeNearest<float3> *found = nullptr;
        const int nfound = kdtree_range_search<float3>(tree, pos, &found, radius);
        Vector<int> ids;
        ids.reserve(std::max(nfound, 0));
        for (int i = 0; i < nfound; i++) {
          skip_append(ids, found[i].index, -1);
        }
        MEM_SAFE_DELETE(found);
        return value_from_int_array(std::move(ids));
      }
      if (mode == 2) {
        if (k <= 0 || !(radius > 0.0f) || !std::isfinite(radius)) {
          return empty_int_arr();
        }
        /* Large k: range-search then keep the closest k. Small k: k-nearest then dist <= r. */
        if (k > 256) {
          KDTreeNearest<float3> *found = nullptr;
          const int nfound = kdtree_range_search<float3>(tree, pos, &found, radius);
          Vector<int> ids;
          ids.reserve(std::min(std::max(nfound, 0), k));
          for (int i = 0; i < nfound; i++) {
            skip_append(ids, found[i].index, k);
          }
          MEM_SAFE_DELETE(found);
          return value_from_int_array(std::move(ids));
        }
        const int want = k + (skip_id >= 0 ? 1 : 0);
        Array<KDTreeNearest<float3>> nearest(want);
        const int nfound = kdtree_find_nearest_n<float3>(tree, pos, nearest.data(), uint(want));
        Vector<int> ids;
        ids.reserve(k);
        for (int i = 0; i < nfound && ids.size() < k; i++) {
          if (nearest[i].dist > radius) {
            break;
          }
          skip_append(ids, nearest[i].index, k);
        }
        return value_from_int_array(std::move(ids));
      }
      if (k <= 0) {
        return empty_int_arr();
      }
      const int want = k + (skip_id >= 0 ? 1 : 0);
      Array<KDTreeNearest<float3>> nearest(want);
      const int nfound = kdtree_find_nearest_n<float3>(tree, pos, nearest.data(), uint(want));
      Vector<int> ids;
      ids.reserve(k);
      for (int i = 0; i < nfound && ids.size() < k; i++) {
        skip_append(ids, nearest[i].index, k);
      }
      return value_from_int_array(std::move(ids));
    }
    case Builtin::Raycast: {
      env.hit_face = -1;
      env.hit_dist = 0.0f;
      env.hit_pos = float3(0.0f);
      env.hit_n = float3(0.0f);
      const bool full = args.size() >= 4;
      const int geo_i = full ? args[0].as_int() : 0;
      if (u->ray_ok && geo_i == u->ray_geo && env.index >= 0 && env.index < u->ray_hit.size())
      {
        const int i = env.index;
        env.hit_face = u->ray_face[i];
        env.hit_pos = u->ray_pos[i];
        env.hit_n = u->ray_n[i];
        env.hit_dist = u->ray_dist[i];
        return intern_env_ray(env, full ? int(u->ray_hit[i]) : env.hit_face);
      }
      TopoUser::SpatialAccel &accel = spatial_of(u, geo_i);
      spatial_ensure_tris(accel);
      if (accel.tris == nullptr || accel.mesh == nullptr) {
        return intern_env_ray(env, full ? 0 : -1);
      }
      float3 origin = (env.index >= 0 && env.index < u->positions.size()) ?
                          u->positions[env.index] :
                          float3(0.0f);
      float3 dir(0.0f, 0.0f, -1.0f);
      float length = 100.0f;
      if (full) {
        origin = args[1].as_vec();
        dir = args[2].as_vec();
        length = args[3].as_float();
      }
      else if (args.size() >= 3) {
        origin = args[0].as_vec();
        dir = args[1].as_vec();
        length = args[2].as_float();
      }
      else if (args.size() == 2) {
        origin = args[0].as_vec();
        dir = args[1].as_vec();
      }
      else if (args.size() == 1) {
        dir = args[0].as_vec();
      }
      bke::bvh::Ray ray{};
      ray.origin = origin;
      ray.direction = dir;
      ray.dist_max = length;
      if (const std::optional<bke::bvh::RayHit> hit = accel.tris->ray_intersect(ray)) {
        const Span<int> tri_faces = accel.mesh->corner_tri_faces();
        env.hit_face = (hit->index >= 0 && hit->index < tri_faces.size()) ? tri_faces[hit->index] :
                                                                            hit->index;
        env.hit_pos = hit->position(ray);
        env.hit_n = math::normalize(hit->normal);
        env.hit_dist = hit->distance;
        return intern_env_ray(env, full ? 1 : env.hit_face);
      }
      return intern_env_ray(env, full ? 0 : -1);
    }
    case Builtin::RayIsHit: {
      const RayHit h = ray_arg_or_env(args, env);
      return Value::from_int(h.hit || h.face >= 0 ? 1 : 0);
    }
    case Builtin::RayHitPos: {
      const RayHit h = ray_arg_or_env(args, env);
      return Value::from_vec(h.pos);
    }
    case Builtin::RayHitN: {
      const RayHit h = ray_arg_or_env(args, env);
      return Value::from_vec(h.n);
    }
    case Builtin::RayHitDist: {
      const RayHit h = ray_arg_or_env(args, env);
      return Value::from_float(h.dist);
    }
    case Builtin::RaycastAll:
      return eval_raycast_all(u, args, env);
    case Builtin::RayIsHitArr:
      if (tls_rayall.hit.type != Type::IntArray) {
        reset_raycast_all_out();
      }
      return tls_rayall.hit;
    case Builtin::RayHitPosArr:
      if (tls_rayall.pos.type != Type::VecArray) {
        reset_raycast_all_out();
      }
      return tls_rayall.pos;
    case Builtin::RayHitNArr:
      if (tls_rayall.n.type != Type::VecArray) {
        reset_raycast_all_out();
      }
      return tls_rayall.n;
    case Builtin::RayHitDistArr:
      if (tls_rayall.dist.type != Type::FloatArray) {
        reset_raycast_all_out();
      }
      return tls_rayall.dist;
    case Builtin::BBoxMin:
    case Builtin::BBoxMax:
    case Builtin::BoundingBox: {
      const int gi = args.is_empty() ? 0 : args[0].as_int();
      const bke::GeometrySet *g = geo_at(u, gi);
      std::optional<Bounds<float3>> bounds;
      if (g) {
        bounds = g->compute_boundbox_without_instances(true);
      }
      if (id == Builtin::BoundingBox) {
        Vector<float3> vals;
        vals.append(bounds ? bounds->min : float3(0.0f));
        vals.append(bounds ? bounds->max : float3(0.0f));
        return value_from_vec_array(std::move(vals));
      }
      if (id == Builtin::BBoxMin) {
        return Value::from_vec(bounds ? bounds->min : float3(0.0f));
      }
      return Value::from_vec(bounds ? bounds->max : float3(0.0f));
    }
    case Builtin::GeometryProximity: {
      const int gi = args.is_empty() ? 0 : args[0].as_int();
      const int domain = args.size() > 1 ? parse_domain_arg(args[1], env) : 2;
      float3 sample = (env.index >= 0 && env.index < u->positions.size()) ?
                          u->positions[env.index] :
                          float3(0.0f);
      if (args.size() > 2) {
        sample = args[2].as_vec();
      }
      float3 pos(0.0f);
      float dist = 0.0f;
      bool from_cache = false;
      if (u->prox_ok && gi == u->prox_geo && domain == u->prox_domain && env.index >= 0 &&
          env.index < u->prox_pos.size())
      {
        pos = u->prox_pos[env.index];
        dist = u->prox_dist[env.index];
        from_cache = true;
      }
      if (!from_cache) {
        proximity_query_accel(u, spatial_of(u, gi), domain, sample, pos, dist);
      }
      env.hit_pos = pos;
      env.hit_dist = dist;
      env.hit_n = float3(0.0f);
      return Value::from_float(dist);
    }
    case Builtin::SampleNearestSurface: {
      const int gi = args.is_empty() ? 0 : args[0].as_int();
      const StringRef data_type = args.size() > 1 ? arg_string(args, 1, env) : StringRef("position");
      if (u->sample_ok && gi == u->sample_geo && env.index >= 0) {
        if (u->sample_is_vec && env.index < u->sample_v.size()) {
          env.hit_pos = u->sample_v[env.index];
          return Value::from_vec(u->sample_v[env.index]);
        }
        if (!u->sample_is_vec && env.index < u->sample_f.size()) {
          return Value::from_float(u->sample_f[env.index]);
        }
      }
      float3 sample = (env.index >= 0 && env.index < u->positions.size()) ?
                          u->positions[env.index] :
                          float3(0.0f);
      if (args.size() > 2) {
        sample = args[2].as_vec();
      }
      TopoUser::SpatialAccel &accel = spatial_of(u, gi);
      spatial_ensure_tris(accel);
      if (accel.tris == nullptr || accel.mesh == nullptr) {
        env.hit_pos = float3(0.0f);
        return Value::from_float(0.0f);
      }
      const std::optional<bke::bvh::ClosestPointResult> closest = accel.tris->closest_point(
          sample);
      if (!closest) {
        env.hit_pos = float3(0.0f);
        return Value::from_float(0.0f);
      }
      env.hit_pos = closest->position;
      env.hit_dist = math::distance(sample, closest->position);
      Vector<std::string> *interned = env.runtime_s;
      return sample_tri_attr(
          *accel.mesh, int(closest->index), closest->bary_coord, data_type, interned);
    }
    case Builtin::DeleteGeometry: {
      int domain_i = 0;
      if (args.size() >= 4 ||
          (args.size() >= 3 && args[0].type != Type::String && args[1].type == Type::String))
      {
        note_geo_forced(*u, args[0].as_int());
        domain_i = 1;
      }
      const int src_domain = domain_from_attr(u->bind_domain);
      const int domain = args.is_empty() ? src_domain : parse_domain_arg(args[domain_i], env);
      const int index = args.size() > domain_i + 1 ? args[domain_i + 1].as_int() : env.index;
      const GeometryNodeDeleteGeometryMode mode = args.size() > domain_i + 2 ?
                                                      parse_delete_mode(args[domain_i + 2], env) :
                                                      GEO_NODE_DELETE_GEOMETRY_MODE_ALL;
      /* Omitted index, or `i@index`, means "this element". Map it onto the requested domain. */
      const bool expand = (domain != src_domain) &&
                          (args.size() < domain_i + 2 || index == env.index);
      mark_delete_geometry(*u, domain, src_domain, index, expand, mode);
      return Value::from_int(index);
    }
    case Builtin::Addpoint: {
      int geo = 0;
      const Value *src = nullptr;
      if (args.size() >= 2) {
        geo = args[0].as_int();
        src = &args[1];
      }
      else if (args.size() == 1) {
        src = &args[0];
      }
      note_geo_forced(*u, geo);
      int copy_from = -1;
      float3 p(0.0f);
      if (src == nullptr) {
        p = (env.index >= 0 && env.index < u->positions.size()) ? u->positions[env.index] : p;
      }
      else if (src->type == Type::Int || src->type == Type::Bool) {
        copy_from = src->as_int();
        if (copy_from >= 0 && copy_from < u->positions.size()) {
          p = u->positions[copy_from];
        }
      }
      else {
        p = src->as_vec();
      }
      const int slot = add_point_atomic(*u, p, copy_from);
      return Value::from_int(u->npoints_orig + slot);
    }
    case Builtin::Addprim: {
      int base = 0;
      if (args.size() >= 3 && args[0].type != Type::String && args[1].type != Type::IntArray) {
        note_geo_forced(*u, args[0].as_int());
        base = 1;
      }
      if (args.size() < base + 2) {
        return Value::from_int(-1);
      }
      const PrimKind kind = parse_prim_kind(args[base], env);
      const Vector<int> pts = int_array_values(args[base + 1]);
      if (kind == PrimKind::Invalid) {
        return Value::from_int(-1);
      }
      bool close = false;
      if (args.size() > base + 2) {
        close = args[base + 2].as_bool();
      }
      std::lock_guard lock(u->add_mutex);
      if (kind == PrimKind::Edge) {
        if (pts.size() < 2) {
          return Value::from_int(-1);
        }
        const int first = u->nedges_orig + int(u->add_edges.size());
        for (int i = 0; i + 1 < int(pts.size()); i++) {
          u->add_edges.append(int2(pts[i], pts[i + 1]));
        }
        if (close && pts.size() >= 3) {
          u->add_edges.append(int2(pts.last(), pts[0]));
        }
        return Value::from_int(first);
      }
      if (kind == PrimKind::Face) {
        if (pts.size() < 3) {
          return Value::from_int(-1);
        }
        const int first = u->nfaces_orig + int(u->add_faces.size());
        u->add_faces.append(pts);
        return Value::from_int(first);
      }
      if (pts.is_empty()) {
        return Value::from_int(-1);
      }
      const int first = u->ncurves_orig + int(u->add_curves.size());
      PendingAddCurve c;
      c.pts = pts;
      c.cyclic = close;
      u->add_curves.append(std::move(c));
      return Value::from_int(first);
    }
    case Builtin::SetAttribute: {
      int base = 0;
      if (args.size() >= 6 ||
          (args.size() >= 5 && args[0].type != Type::String && args[1].type == Type::String))
      {
        note_geo_forced(*u, args[0].as_int());
        base = 1;
      }
      if (args.size() < base + 4) {
        return Value::from_int(-1);
      }
      const int domain = parse_domain_arg(args[base], env);
      const int index = args[base + 1].as_int();
      const StringRef name = value_string(args[base + 2], env);
      const Value &val = args[base + 3];
      const SetMode mode = args.size() > base + 4 ? parse_set_mode(args[base + 4], env) :
                                                    SetMode::Set;
      if (index >= u->npoints_orig && domain == int(bke::AttrDomain::Point)) {
        TopoUser::PendingSet p;
        p.index = index - u->npoints_orig;
        p.is_new = true;
        p.domain = bke::AttrDomain::Point;
        p.name = std::string(name);
        p.type = type_from_value(val);
        p.v = float4(val.as_vec().x, val.as_vec().y, val.as_vec().z, val.v.w);
        p.i = val.as_int();
        p.mode = int(mode);
        std::lock_guard lock(u->pending_mutex);
        u->pending_sets.append(std::move(p));
        return Value::from_int(index);
      }
      set_attribute_atomic(
          *u, bke::AttrDomain(domain), index, name, val, mode);
      return Value::from_int(index);
    }
    case Builtin::Accumulate: {
      const int group = args.size() > 1 ? args[1].as_int() : 0;
      std::lock_guard lock(u->mutex);
      if (!args.is_empty() && args[0].type == Type::Vector) {
        float3 &tot = u->acc_v.lookup_or_add(group, float3(0.0f));
        tot += args[0].as_vec();
        return Value::from_vec(tot);
      }
      float &tot = u->acc_f.lookup_or_add(group, 0.0f);
      tot += args.is_empty() ? 0.0f : args[0].as_float();
      return Value::from_float(tot);
    }
    case Builtin::Npoints:
    case Builtin::Nedges:
    case Builtin::Nfaces:
    case Builtin::Ncorners: {
      const int gi = args.is_empty() ? 0 : args[0].as_int();
      const bke::GeometrySet *g = (gi >= 0 && gi < u->geos.size()) ? u->geos[gi] : u->owner;
      if (!g) {
        return Value::from_int(0);
      }
      if (const Mesh *mesh = g->get_mesh()) {
        if (id == Builtin::Npoints) {
          return Value::from_int(mesh->verts_num);
        }
        if (id == Builtin::Nedges) {
          return Value::from_int(mesh->edges_num);
        }
        if (id == Builtin::Nfaces) {
          return Value::from_int(mesh->faces_num);
        }
        return Value::from_int(mesh->corners_num);
      }
      if (const PointCloud *pc = g->get_pointcloud()) {
        return Value::from_int(id == Builtin::Npoints ? pc->totpoint : 0);
      }
      if (const Curves *curves_id = g->get_curves()) {
        return Value::from_int(id == Builtin::Npoints ? curves_id->geometry.wrap().points_num() :
                              id == Builtin::Nedges ? curves_id->geometry.wrap().curves_num() :
                                                      0);
      }
      return Value::from_int(0);
    }
    case Builtin::FieldAverage:
    case Builtin::FieldMin:
    case Builtin::FieldMax:
    case Builtin::FieldMinMax: {
      if (!u->attrs) {
        return Value::from_float(0.0f);
      }
      const StringRef name = arg_string(args, 0, env);
      if (name.is_empty()) {
        return Value::from_float(0.0f);
      }
      const bke::GAttributeReader reader = u->attrs->lookup(name);
      if (!reader) {
        return Value::from_float(0.0f);
      }
      const GVArray &va = reader.varray;
      if (va.is_empty()) {
        return Value::from_float(0.0f);
      }
      if (va.type().is<float3>()) {
        const VArraySpan<float3> span(va.typed<float3>());
        float3 acc(0.0f);
        float3 mn(FLT_MAX);
        float3 mx(-FLT_MAX);
        for (const float3 &p : span) {
          acc += p;
          mn = math::min(mn, p);
          mx = math::max(mx, p);
        }
        if (id == Builtin::FieldAverage) {
          return Value::from_vec(acc / float(span.size()));
        }
        if (id == Builtin::FieldMinMax) {
          return Value::from_vec(float3(math::length(mn), math::length(mx), 0.0f));
        }
        return Value::from_vec(id == Builtin::FieldMin ? mn : mx);
      }
      if (va.type().is<int>()) {
        const VArraySpan<int> span(va.typed<int>());
        double acc = 0.0;
        int mn = INT_MAX;
        int mx = INT_MIN;
        for (const int v : span) {
          acc += double(v);
          mn = std::min(mn, v);
          mx = std::max(mx, v);
        }
        if (id == Builtin::FieldAverage) {
          return Value::from_float(float(acc / double(span.size())));
        }
        if (id == Builtin::FieldMinMax) {
          return Value::from_vec(float3(float(mn), float(mx), 0.0f));
        }
        return Value::from_int(id == Builtin::FieldMin ? mn : mx);
      }
      if (!va.type().is<float>()) {
        return Value::from_float(0.0f);
      }
      const VArraySpan<float> span(va.typed<float>());
      if (span.is_empty()) {
        return Value::from_float(0.0f);
      }
      double acc = 0.0;
      float mn = FLT_MAX;
      float mx = -FLT_MAX;
      for (const float v : span) {
        acc += double(v);
        mn = std::min(mn, v);
        mx = std::max(mx, v);
      }
      if (id == Builtin::FieldAverage) {
        return Value::from_float(float(acc / double(span.size())));
      }
      if (id == Builtin::FieldMinMax) {
        return Value::from_vec(float3(mn, mx, 0.0f));
      }
      return Value::from_float(id == Builtin::FieldMin ? mn : mx);
    }
    default:
      break;
  }
  return Value::from_float(0.0f);
}

bool mask_any_set(const Span<uint8_t> mask)
{
  for (const uint8_t v : mask) {
    if (v) {
      return true;
    }
  }
  return false;
}

void fill_keep_bools(const Span<uint8_t> mask, MutableSpan<bool> keep)
{
  const int m = int(mask.size());
  threading::parallel_for(keep.index_range(), 4096, [&](const IndexRange range) {
    for (const int64_t i : range) {
      keep[i] = i >= m || mask[int(i)] == 0;
    }
  });
}

IndexMask keep_mask_from_delete(const Span<uint8_t> mask,
                                const int n,
                                IndexMaskMemory &memory)
{
  return IndexMask::from_predicate(IndexMask(n), memory, [&](const int64_t i) {
    return i >= int64_t(mask.size()) || mask[int(i)] == 0;
  });
}

void remap_del_mask(Array<uint8_t> &del, const IndexMask &keep)
{
  Array<uint8_t> next(int(keep.size()), uint8_t(0));
  keep.foreach_index([&](const int64_t old_i, const int64_t new_i) {
    if (old_i >= 0 && old_i < del.size() && del[int(old_i)] && new_i >= 0 &&
        new_i < next.size())
    {
      next[int(new_i)] = 1;
    }
  });
  del = std::move(next);
}

void apply_one_mesh_delete(bke::GeometrySet &geo,
                           Array<uint8_t> &del,
                           const bke::AttrDomain domain,
                           const GeometryNodeDeleteGeometryMode mode,
                           Array<uint8_t> &del_point,
                           Array<uint8_t> &del_edge,
                           Array<uint8_t> &del_face)
{
  if (!geo.has_mesh() || !mask_any_set(del)) {
    return;
  }
  const Mesh &src = *geo.get_mesh();
  const int n = src.attributes().domain_size(domain);
  if (n <= 0) {
    del.fill(0);
    return;
  }
  Array<bool> keep_buf(n);
  fill_keep_bools(del, keep_buf);
  const VArray<bool> keep = VArray<bool>::from_span(keep_buf.as_span());
  const VArraySpan<bool> span(keep);

  IndexMaskMemory memory;
  IndexMask vert_keep;
  IndexMask edge_keep;
  IndexMask face_keep;
  const bool compact_verts = mode == GEO_NODE_DELETE_GEOMETRY_MODE_ALL;
  const bool compact_edges = mode != GEO_NODE_DELETE_GEOMETRY_MODE_ONLY_FACE;
  switch (domain) {
    case bke::AttrDomain::Point: {
      if (compact_verts) {
        vert_keep = IndexMask::from_bools(span, memory);
      }
      else {
        vert_keep = IndexMask(src.verts_num);
      }
      if (compact_edges) {
        edge_keep = geometry::edge_selection_from_vert(src.edges(), span, memory);
      }
      else {
        edge_keep = IndexMask(src.edges_num);
      }
      face_keep = geometry::face_selection_from_vert(
          src.faces(), src.corner_verts(), span, memory);
      break;
    }
    case bke::AttrDomain::Edge: {
      edge_keep = compact_edges ? IndexMask::from_bools(span, memory) : IndexMask(src.edges_num);
      if (compact_verts) {
        vert_keep = geometry::vert_selection_from_edge(
            src.edges(), edge_keep, src.verts_num, memory);
      }
      else {
        vert_keep = IndexMask(src.verts_num);
      }
      face_keep = geometry::face_selection_from_edge(
          src.faces(), src.corner_edges(), span, memory);
      break;
    }
    case bke::AttrDomain::Face: {
      face_keep = IndexMask::from_bools(span, memory);
      if (compact_verts) {
        vert_keep = geometry::vert_selection_from_face(
            src.faces(), face_keep, src.corner_verts(), src.verts_num, memory);
      }
      else {
        vert_keep = IndexMask(src.verts_num);
      }
      if (compact_edges) {
        edge_keep = geometry::edge_selection_from_face(
            src.faces(), face_keep, src.corner_edges(), src.edges_num, memory);
      }
      else {
        edge_keep = IndexMask(src.edges_num);
      }
      break;
    }
    default:
      del.fill(0);
      return;
  }

  std::optional<Mesh *> dst;
  if (mode == GEO_NODE_DELETE_GEOMETRY_MODE_EDGE_FACE) {
    dst = geometry::mesh_copy_selection_keep_verts(src, keep, domain);
  }
  else if (mode == GEO_NODE_DELETE_GEOMETRY_MODE_ONLY_FACE) {
    dst = geometry::mesh_copy_selection_keep_edges(src, keep, domain);
  }
  else {
    dst = geometry::mesh_copy_selection(src, keep, domain);
  }

  if (dst.has_value() && *dst == nullptr) {
    geo.replace_mesh(nullptr);
    del_point.reinitialize(0);
    del_edge.reinitialize(0);
    del_face.reinitialize(0);
    return;
  }

  if (dst.has_value()) {
    if (&del != &del_point && compact_verts) {
      remap_del_mask(del_point, vert_keep);
    }
    if (&del != &del_edge && compact_edges) {
      remap_del_mask(del_edge, edge_keep);
    }
    if (&del != &del_face) {
      remap_del_mask(del_face, face_keep);
    }
    geo.replace_mesh(*dst);
  }
  del.fill(0);
}

void apply_removes(bke::GeometrySet &geo, TopoUser &user)
{
  const bool wrangle_mesh = user.bind_type == bke::GeometryComponent::Type::Mesh;
  const bool wrangle_points = user.bind_type == bke::GeometryComponent::Type::PointCloud;
  const bool wrangle_curves = user.bind_type == bke::GeometryComponent::Type::Curve;
  const bool wrangle_inst = user.bind_type == bke::GeometryComponent::Type::Instance;

  /* Face then edge then point so remaining masks remap onto the compacted mesh. */
  if (wrangle_mesh && geo.has_mesh() &&
      (mask_any_set(user.del_point) || mask_any_set(user.del_edge) || mask_any_set(user.del_face)))
  {
    apply_one_mesh_delete(geo,
                          user.del_face,
                          bke::AttrDomain::Face,
                          user.delete_mode_face,
                          user.del_point,
                          user.del_edge,
                          user.del_face);
    apply_one_mesh_delete(geo,
                          user.del_edge,
                          bke::AttrDomain::Edge,
                          user.delete_mode_edge,
                          user.del_point,
                          user.del_edge,
                          user.del_face);
    apply_one_mesh_delete(geo,
                          user.del_point,
                          bke::AttrDomain::Point,
                          user.delete_mode_point,
                          user.del_point,
                          user.del_edge,
                          user.del_face);
  }
  if (wrangle_points && geo.has_pointcloud() && mask_any_set(user.del_point)) {
    const PointCloud &src = *geo.get_pointcloud();
    IndexMaskMemory memory;
    const IndexMask keep = keep_mask_from_delete(user.del_point, src.totpoint, memory);
    if (keep.size() != src.totpoint) {
      if (keep.is_empty()) {
        geo.replace_pointcloud(nullptr);
      }
      else {
        PointCloud *dst = BKE_pointcloud_new_nomain(PointCloudType::Points, int(keep.size()));
        bke::gather_attributes(src.attributes(),
                               bke::AttrDomain::Point,
                               bke::AttrDomain::Point,
                               bke::AttributeFilter{},
                               keep,
                               dst->attributes_for_write());
        geo.replace_pointcloud(dst);
      }
    }
  }
  if (wrangle_curves && geo.has_curves() &&
      (mask_any_set(user.del_point) || mask_any_set(user.del_curve)))
  {
    Curves *curves_id = geo.get_curves_for_write();
    bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    IndexMaskMemory memory;
    if (mask_any_set(user.del_curve)) {
      const IndexMask keep = keep_mask_from_delete(user.del_curve, curves.curves_num(), memory);
      if (keep.size() != curves.curves_num()) {
        curves = bke::curves_copy_curve_selection(curves, keep, bke::AttributeFilter{});
      }
    }
    else {
      const IndexMask keep = keep_mask_from_delete(user.del_point, curves.points_num(), memory);
      if (keep.size() != curves.points_num()) {
        curves = bke::curves_copy_point_selection(curves, keep, bke::AttributeFilter{});
      }
    }
  }
  if (wrangle_inst && geo.has_instances() && mask_any_set(user.del_inst)) {
    bke::Instances &inst = *geo.get_instances_for_write();
    IndexMaskMemory memory;
    const IndexMask keep = keep_mask_from_delete(user.del_inst, inst.instances_num(), memory);
    if (keep.size() != inst.instances_num()) {
      inst.remove(keep, bke::AttributeFilter{});
    }
  }
}

void fill_counts(const bke::GeometrySet &geo, VMEnv &env)
{
  env.npoints = 0;
  env.nedges = 0;
  env.nfaces = 0;
  env.ncorners = 0;
  if (const Mesh *mesh = geo.get_mesh()) {
    env.npoints = mesh->verts_num;
    env.nedges = mesh->edges_num;
    env.nfaces = mesh->faces_num;
    env.ncorners = mesh->corners_num;
    return;
  }
  if (const PointCloud *pc = geo.get_pointcloud()) {
    env.npoints = pc->totpoint;
    return;
  }
  if (const Curves *curves_id = geo.get_curves()) {
    env.npoints = curves_id->geometry.wrap().points_num();
    env.nfaces = curves_id->geometry.wrap().curves_num();
  }
}

bool is_topo_array_builtin(const Builtin id)
{
  /* Point-domain C kernel only. Edge/corner/curve queries must go through the VM
   * so `fn(0, i@index)` keeps its arguments instead of being rewritten to empty. */
  return ELEM(id,
              Builtin::PointNeighbours,
              Builtin::PointEdges,
              Builtin::PointFaces,
              Builtin::PointCorners,
              Builtin::Neighbours,
              Builtin::CornersOfVertex,
              Builtin::EdgesOfVertex,
              Builtin::FacesOfVertex);
}

bool decode_single_topo_store(const Program &program, Builtin &r_id, int &r_attr)
{
  r_id = Builtin::Sin;
  r_attr = -1;
  bool saw_call = false;
  for (const Inst &in : program.code) {
    switch (in.op) {
      case Op::Nop:
      case Op::Pop:
      case Op::Dup:
      case Op::PushIndex:
      case Op::PushLocal:
      case Op::StoreLocal:
        break;
      case Op::PushI:
        /* `fn(0, i@index)` — geo 0 is the wrangled mesh, same as the C kernel. */
        if (in.imm < 0 || in.imm >= program.const_i.size() || program.const_i[in.imm] != 0) {
          return false;
        }
        break;
      case Op::Call:
        /* nargs 0/1 is `fn()` / `fn(index)`. nargs 2 is only `fn(0, index)` (PushI 0 above).
         * `fn(1, index)` must use the VM so it samples extra geometry. */
        if (saw_call || in.a > 2 || !is_topo_array_builtin(Builtin(in.imm))) {
          return false;
        }
        r_id = Builtin(in.imm);
        saw_call = true;
        break;
      case Op::StoreAttr:
        if (!saw_call || r_attr >= 0) {
          return false;
        }
        r_attr = in.imm;
        break;
      default:
        return false;
    }
  }
  return saw_call && r_attr >= 0;
}

void c_fill_topo_attr(TopoUser &u,
                      AttrRT &a,
                      const Builtin id,
                      const IndexMask &mask)
{
  if (a.warr == nullptr || u.mesh == nullptr) {
    return;
  }
  const Mesh &mesh = *u.mesh;
  u.prewarm_topology();
  mask.foreach_index(
      [&](const int64_t i) {
        const int elem = int(i);
        Vector<int> vals;
        switch (id) {
          case Builtin::PointNeighbours:
          case Builtin::Neighbours: {
            u.ensure_v2e();
            if (elem >= 0 && elem < u.v2e.size()) {
              const Span<int2> edges = mesh.edges();
              vals.reserve(u.v2e[elem].size());
              for (const int e : u.v2e[elem]) {
                if (e < 0 || e >= edges.size()) {
                  continue;
                }
                const int2 ev = edges[e];
                vals.append(ev[0] == elem ? ev[1] : ev[0]);
              }
            }
            if (vals.is_empty()) {
              const GroupedSpan<int> v2c = mesh.vert_to_corner_map();
              if (elem >= 0 && elem < v2c.size()) {
                const Span<int> cv = mesh.corner_verts();
                const Span<int> c2f = mesh.corner_to_face_map();
                const OffsetIndices<int> faces = mesh.faces();
                for (const int c : v2c[elem]) {
                  if (c < 0 || c >= c2f.size()) {
                    continue;
                  }
                  const int f = c2f[c];
                  if (f < 0 || f >= mesh.faces_num) {
                    continue;
                  }
                  const IndexRange face = faces[f];
                  append_unique_int(vals, cv[bke::mesh::face_corner_prev(face, c)]);
                  append_unique_int(vals, cv[bke::mesh::face_corner_next(face, c)]);
                }
              }
            }
            break;
          }
          case Builtin::PointEdges:
          case Builtin::EdgesOfVertex: {
            if (elem >= 0 && elem < u.v2e.size()) {
              vals.extend(u.v2e[elem]);
            }
            break;
          }
          case Builtin::PointFaces:
          case Builtin::FacesOfVertex: {
            const GroupedSpan<int> map = mesh.vert_to_face_map();
            if (elem >= 0 && elem < map.size()) {
              vals.extend(map[elem]);
            }
            break;
          }
          case Builtin::PointCorners:
          case Builtin::CornersOfVertex: {
            const GroupedSpan<int> map = mesh.vert_to_corner_map();
            if (elem >= 0 && elem < map.size()) {
              vals.extend(map[elem]);
            }
            break;
          }
          case Builtin::FacePoints: {
            if (elem < 0 || elem >= mesh.faces_num) {
              break;
            }
            const IndexRange face = mesh.faces()[elem];
            const Span<int> cv = mesh.corner_verts();
            vals.reserve(face.size());
            for (const int c : face) {
              vals.append(cv[c]);
            }
            break;
          }
          case Builtin::FaceCorners:
          case Builtin::CornersOfFace: {
            if (elem >= 0 && elem < mesh.faces_num) {
              const IndexRange face = mesh.faces()[elem];
              vals.reserve(face.size());
              for (const int c : face) {
                vals.append(c);
              }
            }
            break;
          }
          case Builtin::FaceEdges: {
            if (elem < 0 || elem >= mesh.faces_num) {
              break;
            }
            const IndexRange face = mesh.faces()[elem];
            const Span<int> ce = mesh.corner_edges();
            vals.reserve(face.size());
            for (const int c : face) {
              vals.append(ce[c]);
            }
            break;
          }
          case Builtin::EdgePoints: {
            const Span<int2> edges = mesh.edges();
            if (elem >= 0 && elem < edges.size()) {
              vals.append(edges[elem][0]);
              vals.append(edges[elem][1]);
            }
            break;
          }
          default:
            break;
        }
        a.warr[i].set_ints(vals);
      },
      exec_mode::grain_size(1024));
}

void build_point_neighbor_table(TopoUser &u, const int n, Array<int> &r_off, Array<int> &r_idx)
{
  if (n <= 0) {
    r_off.reinitialize(1);
    r_off[0] = 0;
    r_idx = {};
    return;
  }
  /* Same CSR as Blur Attribute: vert→edge map, then rewrite slots to other verts. */
  if (u.mesh && n == u.mesh->verts_num) {
    const Mesh &mesh = *u.mesh;
    const Span<int2> edges = mesh.edges();
    bke::mesh::build_vert_to_edge_map(edges, mesh.verts_num, r_off, r_idx);
    const OffsetIndices<int> offsets(r_off);
    threading::parallel_for(IndexRange(mesh.verts_num), 2048, [&](const IndexRange range) {
      for (const int vert : range) {
        MutableSpan<int> neighbors = r_idx.as_mutable_span().slice(offsets[vert]);
        for (const int i : neighbors.index_range()) {
          neighbors[i] = bke::mesh::edge_other_vert(edges[neighbors[i]], vert);
        }
      }
    });
    return;
  }
  r_off.reinitialize(n + 1);
  r_off.fill(0);
  Vector<int> idx;
  if (u.mesh) {
    const Mesh &mesh = *u.mesh;
    u.ensure_v2e();
    const Span<int2> edges = mesh.edges();
    for (int v = 0; v < n; v++) {
      r_off[v] = int(idx.size());
      if (v < u.v2e.size()) {
        for (const int e : u.v2e[v]) {
          if (e < 0 || e >= edges.size()) {
            continue;
          }
          const int2 ev = edges[e];
          idx.append(ev[0] == v ? ev[1] : ev[0]);
        }
      }
    }
    r_off[n] = int(idx.size());
    r_idx = Array<int>(idx.as_span());
    return;
  }
  if (u.curves) {
    u.ensure_point_to_curve();
    const Span<int> p2c = u.point_to_curve.as_span();
    const OffsetIndices<int> pts = u.curves->points_by_curve();
    const VArray<bool> cyclic = u.curves->cyclic();
    for (int v = 0; v < n; v++) {
      r_off[v] = int(idx.size());
      if (v < 0 || v >= p2c.size()) {
        continue;
      }
      const int ci = p2c[v];
      if (ci < 0 || ci >= pts.size()) {
        continue;
      }
      const IndexRange curve_pts = pts[ci];
      const int cn = int(curve_pts.size());
      if (cn <= 1) {
        continue;
      }
      const int local = v - int(curve_pts.start());
      const bool wrap = (ci < cyclic.size()) ? cyclic[ci] : false;
      if (local > 0) {
        idx.append(curve_pts[local - 1]);
      }
      else if (wrap) {
        idx.append(curve_pts[cn - 1]);
      }
      if (local < cn - 1) {
        idx.append(curve_pts[local + 1]);
      }
      else if (wrap) {
        idx.append(curve_pts[0]);
      }
    }
    r_off[n] = int(idx.size());
    r_idx = Array<int>(idx.as_span());
    return;
  }
  r_off[n] = 0;
  r_idx = {};
}

int resolve_array_passes(const Program &program, const ElemUser &elem_user);

bool attr_name_in(const Span<std::string> names, const StringRef name)
{
  for (const std::string &n : names) {
    if (n == name) {
      return true;
    }
  }
  return false;
}

float parm_float0(const ElemUser &elem_user, const StringRef name, const float fallback)
{
  if (name.is_empty()) {
    return fallback;
  }
  for (const int i : elem_user.parm_names.index_range()) {
    if (elem_user.parm_names[i] != name) {
      continue;
    }
    const GVArray &va = elem_user.parm_arrays[i];
    if (!va || va.is_empty()) {
      break;
    }
    if (va.type().is<float>()) {
      return va.typed<float>()[0];
    }
    if (va.type().is<int>()) {
      return float(va.typed<int>()[0]);
    }
    break;
  }
  return fallback;
}

const GVArray *parm_varray(const ElemUser &elem_user, const StringRef name)
{
  if (name.is_empty()) {
    return nullptr;
  }
  for (const int i : elem_user.parm_names.index_range()) {
    if (elem_user.parm_names[i] == name) {
      return &elem_user.parm_arrays[i];
    }
  }
  return nullptr;
}

struct PointNeighborCache {
  const void *key = nullptr;
  int n = 0;
  int extra = 0;
  Array<int> off;
  Array<int> idx;
};

bool get_point_neighbor_table(TopoUser &u, const int n, Span<int> &r_off, Span<int> &r_idx)
{
  thread_local PointNeighborCache cache;
  const void *key = nullptr;
  int extra = 0;
  if (u.mesh) {
    key = u.mesh;
    extra = u.mesh->edges_num;
  }
  else if (u.curves) {
    key = u.curves;
    extra = int(u.curves->points_num()) ^ (int(u.curves->curves_num()) << 16);
  }
  else if (u.points) {
    key = u.points;
    extra = u.points->totpoint;
  }
  if (key != nullptr && cache.key == key && cache.n == n && cache.extra == extra &&
      cache.off.size() == n + 1)
  {
    r_off = cache.off;
    r_idx = cache.idx;
    if (n > 1 && cache.idx.is_empty()) {
      return false;
    }
    return true;
  }
  build_point_neighbor_table(u, n, cache.off, cache.idx);
  cache.key = key;
  cache.n = n;
  cache.extra = extra;
  r_off = cache.off;
  r_idx = cache.idx;
  /* Point clouds have no mesh topology. An empty table used to count as success
   * and Jacobi "smooth" became a no-op — radius packing loops produced no output. */
  if (n > 1 && cache.idx.is_empty()) {
    return false;
  }
  return cache.off.size() == n + 1;
}

bool get_knn_neighbor_table(TopoUser &u,
                            const int geo,
                            const int k_in,
                            const int n,
                            Span<int> &r_off,
                            Span<int> &r_idx)
{
  thread_local PointNeighborCache cache;
  const void *key = u.points ? static_cast<const void *>(u.points) :
                    u.mesh   ? static_cast<const void *>(u.mesh) :
                    u.curves ? static_cast<const void *>(u.curves) :
                               static_cast<const void *>(u.owner);
  const int extra = (geo << 16) ^ std::max(k_in, 0) ^ (n * 1315423911);
  if (key != nullptr && cache.key == key && cache.n == n && cache.extra == extra &&
      cache.off.size() == n + 1)
  {
    r_off = cache.off;
    r_idx = cache.idx;
    return cache.off.size() == n + 1 && !cache.idx.is_empty();
  }
  KDTree<float3> *tree = u.tree_for(geo);
  Span<float3> pos = u.positions;
  if (pos.size() < n || tree == nullptr || n <= 0 || k_in <= 0) {
    cache = {};
    return false;
  }
  const int k = std::max(k_in, 0);
  const int skip_self = (geo <= 0) ? 1 : 0;
  const int want = k + skip_self;
  /* Precompute is optional. If n*k would be huge, skip the table and query per point. */
  if (k <= 0 || want <= 0 || int64_t(n) * int64_t(std::max(k, 1)) > 50'000'000) {
    cache = {};
    return false;
  }
  Array<int> packed(n * std::max(k, 1));
  Array<int> counts(n, 0);
  threading::parallel_for(IndexRange(n), 256, [&](const IndexRange range) {
    Array<KDTreeNearest<float3>> nearest(std::max(want, 1));
    for (const int64_t i64 : range) {
      const int i = int(i64);
      const int found = kdtree_find_nearest_n<float3>(
          tree, pos[i], nearest.data(), uint(want));
      int c = 0;
      int *out = packed.data() + i * k;
      for (int j = 0; j < found && c < k; j++) {
        const int ni = nearest[j].index;
        if (ni < 0 || ni >= n) {
          continue;
        }
        if (skip_self && ni == i) {
          continue;
        }
        out[c++] = ni;
      }
      counts[i] = c;
    }
  });
  cache.off.reinitialize(n + 1);
  int total = 0;
  for (int i = 0; i < n; i++) {
    cache.off[i] = total;
    total += counts[i];
  }
  cache.off[n] = total;
  cache.idx.reinitialize(total);
  /* Do not touch thread_local `cache` from TBB workers — each worker has its own
   * empty TLS instance. Snapshot the buffers on this thread first. */
  int *idx_data = cache.idx.data();
  const int *off_data = cache.off.data();
  const int *packed_data = packed.data();
  const int *counts_data = counts.data();
  threading::parallel_for(IndexRange(n), 1024, [&](const IndexRange range) {
    for (const int64_t i64 : range) {
      const int i = int(i64);
      const int a = off_data[i];
      const int c = counts_data[i];
      if (c > 0) {
        memcpy(idx_data + a, packed_data + i * k, sizeof(int) * size_t(c));
      }
    }
  });
  cache.key = key;
  cache.n = n;
  cache.extra = extra;
  r_off = cache.off;
  r_idx = cache.idx;
  return total > 0;
}

static bool attr_is_int_array(const Program &program, const StringRef name)
{
  for (const AttrInfo &info : program.attrs) {
    if (info.name == name) {
      return info.type == Type::IntArray;
    }
  }
  return false;
}

static bool attr_is_vector_attr(const Program &program, const StringRef name)
{
  for (const AttrInfo &info : program.attrs) {
    if (info.name == name) {
      return info.type == Type::Vector;
    }
  }
  return false;
}

static bool write_int_csr_attr(bke::MutableAttributeAccessor attributes,
                               const StringRef name,
                               const bke::AttrDomain domain,
                               const int domain_size,
                               const IndexMask &mask,
                               const Span<int> off,
                               const Span<int> idx)
{
  if (name.is_empty() || domain_size <= 0 || off.size() != domain_size + 1) {
    return false;
  }
  bke::GSpanAttributeWriter writer = attributes.convert_or_add_for_write_only_span(
      name, domain, bke::AttrType::WrangleArray);
  if (!writer) {
    const std::optional<bke::AttributeMetaData> meta = attributes.lookup_meta_data(name);
    if (meta && (meta->domain != domain || meta->data_type != bke::AttrType::WrangleArray)) {
      attributes.remove(name);
    }
    writer = attributes.convert_or_add_for_write_only_span(
        name, domain, bke::AttrType::WrangleArray);
  }
  if (!writer) {
    return false;
  }
  MutableSpan<bke::WrangleArrayValue> span = writer.span.typed<bke::WrangleArrayValue>();
  auto write_one = [&](const int i) {
    if (i < 0 || i >= domain_size || i >= span.size()) {
      return;
    }
    const int a = off[i];
    const int b = off[i + 1];
    const int nn = std::max(0, b - a);
    if (nn <= 0 || a < 0 || a + nn > idx.size()) {
      span[i].set_ints({});
      return;
    }
    span[i].set_ints(idx.slice(a, nn));
  };
  if (mask.size() == domain_size) {
    threading::parallel_for(IndexRange(domain_size), 1024, [&](const IndexRange range) {
      for (const int64_t i : range) {
        write_one(int(i));
      }
    });
  }
  else {
    mask.foreach_index([&](const int64_t i) { write_one(int(i)); });
  }
  writer.finish();
  return true;
}

void jacobi_smooth_kernel(const Span<int> off,
                          const Span<int> idx,
                          const IndexMask &mask,
                          const int domain_size,
                          const int niter,
                          const float fac,
                          const float *fac_per,
                          float3 *live,
                          const Span<float3 *> avgs)
{
  if (niter <= 0 || domain_size <= 0 || live == nullptr) {
    return;
  }
  /* Same grain as Blur Attribute. parallel_for treats grain as the single-thread
   * threshold (`n <= grain` → serial). 4096 made typical meshes 2× slower. */
  constexpr int grain = 1024;
  Array<float3> scratch(domain_size);
  const OffsetIndices<int> groups(off);
  const Span<int> nbrs = idx;
  const bool full = mask.size() == domain_size;
  const bool uniform_fac = fac_per == nullptr;

  auto one_pass = [&](const float3 *from, float3 *to, const bool write_avg) {
    auto body = [&](const int i) {
      const IndexRange g = groups[i];
      const int nn = int(g.size());
      if (nn <= 0) {
        to[i] = from[i];
        if (write_avg) {
          for (float3 *avg : avgs) {
            avg[i] = from[i];
          }
        }
        return;
      }
      float3 o(0.0f);
      int valid = 0;
      const int *np = &nbrs[g.start()];
      for (int k = 0; k < nn; k++) {
        const int ni = np[k];
        if (uint(ni) >= uint(domain_size)) {
          continue;
        }
        o += from[ni];
        valid++;
      }
      if (valid <= 0) {
        to[i] = from[i];
        if (write_avg) {
          for (float3 *avg : avgs) {
            avg[i] = from[i];
          }
        }
        return;
      }
      const float3 avg = o * (1.0f / float(valid));
      const float f = uniform_fac ? fac : fac_per[i];
      to[i] = from[i] + (avg - from[i]) * f;
      if (write_avg) {
        for (float3 *a : avgs) {
          a[i] = avg;
        }
      }
    };
    if (full) {
      threading::parallel_for(IndexRange(domain_size), grain, [&](const IndexRange range) {
        for (const int64_t i : range) {
          body(int(i));
        }
      });
    }
    else {
      memcpy(to, from, sizeof(float3) * size_t(domain_size));
      mask.foreach_index([&](const int64_t i) { body(int(i)); }, exec_mode::grain_size(grain));
    }
  };

  float3 *buf_a = live;
  float3 *buf_b = scratch.data();
  for (int it = 0; it < niter; it++) {
    const float3 *from = (it % 2 == 0) ? buf_a : buf_b;
    float3 *to = (it % 2 == 0) ? buf_b : buf_a;
    one_pass(from, to, it == niter - 1 && !avgs.is_empty());
  }
  if (niter % 2 == 1) {
    memcpy(live, scratch.data(), sizeof(float3) * size_t(domain_size));
  }
}

void fill_jacobi_fac(const ElemUser &elem_user,
                     const StringRef name,
                     const int domain_size,
                     float &r_fac,
                     const float *&r_fac_per,
                     Array<float> &fac_buf)
{
  r_fac = parm_float0(elem_user, name, 1.0f);
  r_fac_per = nullptr;
  if (const GVArray *va = parm_varray(elem_user, name)) {
    if (*va && va->type().is<float>() && !va->is_single() && va->size() >= domain_size) {
      fac_buf.reinitialize(domain_size);
      va->typed<float>().materialize(fac_buf.as_mutable_span());
      r_fac_per = fac_buf.data();
    }
  }
}

void fill_topo_user(TopoUser &topo, bke::GeometrySet &owner, const ElemUser &elem_user)
{
  topo.owner = &owner;
  topo.geos = elem_user.geos;
  topo.mesh = owner.get_mesh();
  topo.points = owner.get_pointcloud();
  if (const Curves *curves_id = owner.get_curves()) {
    topo.curves = &curves_id->geometry.wrap();
  }
  if (topo.mesh) {
    topo.positions = topo.mesh->vert_positions();
    topo.attrs = topo.mesh->attributes();
  }
  else if (topo.points) {
    topo.positions = topo.points->positions();
    topo.attrs = topo.points->attributes();
  }
  else if (topo.curves) {
    topo.positions = topo.curves->positions();
    topo.attrs = topo.curves->attributes();
  }
}

bool try_jacobi_smooth_direct(const Program &program,
                              bke::GeometrySet &owner,
                              bke::MutableAttributeAccessor attributes,
                              TopoUser &topo,
                              const ElemUser &elem_user,
                              const IndexMask &mask,
                              const int domain_size,
                              const bke::AttrDomain bind_domain)
{
  if (!program.jacobi_smooth || domain_size <= 0) {
    return false;
  }
  if (!program.array_pass_peeled && !program.jacobi_knn) {
    return false;
  }
  if (bind_domain != bke::AttrDomain::Point) {
    return false;
  }
  const int niter = resolve_array_passes(program, elem_user);
  if (niter <= 0) {
    return true;
  }

  std::string smooth_name = "position";
  if (!program.jacobi_sample_attrs.is_empty()) {
    smooth_name = program.jacobi_sample_attrs[0];
  }
  const bool smooth_is_P = is_position_name(smooth_name);

  ResourceScope writers_scope;
  Vector<bke::SpanAttributeWriter<float3> *> finish_list;
  float3 *live = nullptr;
  Mesh *mesh_w = nullptr;
  PointCloud *pc_w = nullptr;
  bke::CurvesGeometry *curves_w = nullptr;

  if (smooth_is_P) {
    if (owner.get_mesh()) {
      mesh_w = owner.get_mesh_for_write();
      if (!mesh_w || mesh_w->verts_num != domain_size) {
        return false;
      }
      MutableSpan<float3> wpos = mesh_w->vert_positions_for_write();
      if (wpos.size() < domain_size) {
        return false;
      }
      live = wpos.data();
      topo.mesh = mesh_w;
      topo.positions = wpos;
    }
    else if (owner.get_pointcloud()) {
      pc_w = owner.get_pointcloud_for_write();
      if (!pc_w || pc_w->totpoint != domain_size) {
        return false;
      }
      /* `positions_for_write()` may copy-on-write the attribute array. The
       * span from `fill_topo_user` is then dangling — kNN queries crashed
       * on point clouds because they still used that old pointer. */
      MutableSpan<float3> wpos = pc_w->positions_for_write();
      if (wpos.size() < domain_size) {
        return false;
      }
      live = wpos.data();
      topo.points = pc_w;
      topo.positions = wpos;
    }
    else if (owner.get_curves()) {
      Curves *cid = owner.get_curves_for_write();
      if (!cid) {
        return false;
      }
      curves_w = &cid->geometry.wrap();
      if (curves_w->points_num() != domain_size) {
        return false;
      }
      MutableSpan<float3> wpos = curves_w->positions_for_write();
      if (wpos.size() < domain_size) {
        return false;
      }
      live = wpos.data();
      topo.curves = curves_w;
      topo.positions = wpos;
    }
  }
  else {
    bke::SpanAttributeWriter<float3> tmp = attributes.lookup_or_add_for_write_span<float3>(
        smooth_name, bind_domain);
    if (!tmp || tmp.span.size() < domain_size) {
      return false;
    }
    bke::SpanAttributeWriter<float3> &w = writers_scope.construct<bke::SpanAttributeWriter<float3>>(
        std::move(tmp));
    live = w.span.data();
    finish_list.append(&w);
  }
  if (live == nullptr) {
    return false;
  }

  Span<int> off;
  Span<int> idx;
  if (program.jacobi_knn) {
    if (!get_knn_neighbor_table(
            topo, program.jacobi_knn_geo, program.jacobi_knn_k, domain_size, off, idx))
    {
      return false;
    }
  }
  else if (!get_point_neighbor_table(topo, domain_size, off, idx)) {
    return false;
  }

  /* i[]@pts must stay an int array. Writing it as float3 is why a for-loop
   * nearestpoints dump showed up as Vector in the spreadsheet. */
  for (const AttrInfo &info : program.attrs) {
    if (info.write && info.type == Type::IntArray) {
      write_int_csr_attr(attributes, info.name, bind_domain, domain_size, mask, off, idx);
    }
  }
  for (const std::string &name : program.jacobi_avg_attrs) {
    if (is_position_name(name) || name == smooth_name || attr_is_vector_attr(program, name)) {
      continue;
    }
    if (!attr_is_int_array(program, name)) {
      write_int_csr_attr(attributes, name, bind_domain, domain_size, mask, off, idx);
    }
  }

  Vector<float3 *> avg_ptrs;
  for (const std::string &name : program.jacobi_avg_attrs) {
    if (is_position_name(name) || name == smooth_name) {
      continue;
    }
    if (attr_is_int_array(program, name) || !attr_is_vector_attr(program, name)) {
      continue;
    }
    bke::SpanAttributeWriter<float3> tmp = attributes.lookup_or_add_for_write_span<float3>(
        name, bind_domain);
    if (!tmp || tmp.span.size() < domain_size) {
      continue;
    }
    bke::SpanAttributeWriter<float3> &w = writers_scope.construct<bke::SpanAttributeWriter<float3>>(
        std::move(tmp));
    avg_ptrs.append(w.span.data());
    finish_list.append(&w);
  }

  float fac = 1.0f;
  const float *fac_per = nullptr;
  Array<float> fac_buf;
  fill_jacobi_fac(elem_user, program.jacobi_fac_ch, domain_size, fac, fac_per, fac_buf);

  jacobi_smooth_kernel(off, idx, mask, domain_size, niter, fac, fac_per, live, avg_ptrs);

  for (bke::SpanAttributeWriter<float3> *w : finish_list) {
    w->finish();
  }
  if (mesh_w) {
    mesh_w->tag_positions_changed();
  }
  else if (pc_w) {
    pc_w->tag_positions_changed();
  }
  else if (curves_w) {
    curves_w->tag_positions_changed();
  }
  return true;
}

/* Blur Attribute-style Jacobi fallback after bind_attrs. */
bool try_jacobi_smooth(const Program &program,
                       BindState &state,
                       TopoUser &topo,
                       const ElemUser &elem_user,
                       const IndexMask &mask,
                       const int domain_size,
                       const bke::AttrDomain bind_domain)
{
  if (!program.jacobi_smooth || domain_size <= 0) {
    return false;
  }
  if (!program.array_pass_peeled && !program.jacobi_knn) {
    return false;
  }
  if (bind_domain != bke::AttrDomain::Point) {
    return false;
  }
  const int niter = resolve_array_passes(program, elem_user);
  if (niter <= 0) {
    return true;
  }

  AttrRT *smooth = nullptr;
  Vector<float3 *> avgs;
  for (const int i : program.attrs.index_range()) {
    if (i >= state.rt.size() || program.attrs[i].type != Type::Vector) {
      continue;
    }
    AttrRT &a = state.rt[i];
    if (a.wv == nullptr || a.size < domain_size) {
      continue;
    }
    if (attr_name_in(program.jacobi_sample_attrs, program.attrs[i].name) ||
        program.attrs[i].name == "position")
    {
      if (smooth == nullptr) {
        smooth = &a;
      }
    }
    else if (program.attrs[i].write && program.attrs[i].type == Type::Vector &&
             attr_name_in(program.jacobi_avg_attrs, program.attrs[i].name))
    {
      avgs.append(a.wv);
    }
  }
  if (smooth == nullptr) {
    return false;
  }

  Span<int> off;
  Span<int> idx;
  if (program.jacobi_knn) {
    if (!get_knn_neighbor_table(
            topo, program.jacobi_knn_geo, program.jacobi_knn_k, domain_size, off, idx))
    {
      return false;
    }
  }
  else if (!get_point_neighbor_table(topo, domain_size, off, idx)) {
    return false;
  }
  for (const int i : program.attrs.index_range()) {
    if (i >= state.rt.size() || program.attrs[i].type != Type::IntArray) {
      continue;
    }
    AttrRT &a = state.rt[i];
    if (a.warr == nullptr || a.size < domain_size) {
      continue;
    }
    auto write_one = [&](const int pi) {
      if (pi < 0 || pi >= domain_size) {
        return;
      }
      const int begin = off[pi];
      const int nn = std::max(0, off[pi + 1] - begin);
      if (nn <= 0 || begin < 0 || begin + nn > idx.size()) {
        a.warr[pi].set_ints({});
        return;
      }
      a.warr[pi].set_ints(idx.slice(begin, nn));
    };
    if (mask.size() == domain_size) {
      threading::parallel_for(IndexRange(domain_size), 1024, [&](const IndexRange range) {
        for (const int64_t pi : range) {
          write_one(int(pi));
        }
      });
    }
    else {
      mask.foreach_index([&](const int64_t pi) { write_one(int(pi)); });
    }
  }

  float fac = 1.0f;
  const float *fac_per = nullptr;
  Array<float> fac_buf;
  fill_jacobi_fac(elem_user, program.jacobi_fac_ch, domain_size, fac, fac_per, fac_buf);

  jacobi_smooth_kernel(off, idx, mask, domain_size, niter, fac, fac_per, smooth->wv, avgs);
  return true;
}

int resolve_array_passes(const Program &program, const ElemUser &elem_user)
{
  if (!program.array_pass_peeled && !program.jacobi_knn) {
    return 1;
  }
  int n = std::max(program.array_passes, 0);
  if (program.array_passes_ch.empty()) {
    return n;
  }
  for (const int i : elem_user.parm_names.index_range()) {
    if (elem_user.parm_names[i] != program.array_passes_ch) {
      continue;
    }
    const GVArray &va = elem_user.parm_arrays[i];
    if (!va) {
      break;
    }
    if (va.type().is<int>()) {
      const VArray<int> src = va.typed<int>();
      n = src.is_empty() ? 0 : src[0];
    }
    else if (va.type().is<float>()) {
      const VArray<float> src = va.typed<float>();
      n = src.is_empty() ? 0 : int(src[0]);
    }
    break;
  }
  return std::clamp(n, 0, 65536);
}

void snapshot_attr_rt(const AttrRT &src, AttrRT &dst, Array<float3> &buf_v, Array<float> &buf_f, Array<int> &buf_i)
{
  dst = src;
  dst.wv = nullptr;
  dst.wf = nullptr;
  dst.wi = nullptr;
  dst.wb = nullptr;
  dst.w4 = nullptr;
  dst.wq = nullptr;
  dst.wm = nullptr;
  dst.warr = nullptr;
  dst.ws = nullptr;
  switch (src.type) {
    case Type::Vector: {
      buf_v.reinitialize(src.size);
      const float3 *p = src.wv ? src.wv : src.rv;
      if (p && src.size > 0) {
        memcpy(buf_v.data(), p, sizeof(float3) * size_t(src.size));
      }
      dst.rv = buf_v.data();
      break;
    }
    case Type::Float: {
      buf_f.reinitialize(src.size);
      const float *p = src.wf ? src.wf : src.rf;
      if (p && src.size > 0) {
        memcpy(buf_f.data(), p, sizeof(float) * size_t(src.size));
      }
      dst.rf = buf_f.data();
      break;
    }
    case Type::Int:
    case Type::Bool: {
      buf_i.reinitialize(src.size);
      if (src.type == Type::Int) {
        const int *p = src.wi ? src.wi : src.ri;
        if (p && src.size > 0) {
          memcpy(buf_i.data(), p, sizeof(int) * size_t(src.size));
        }
      }
      else {
        const bool *p = src.wb ? src.wb : src.rb;
        for (int i = 0; i < src.size; i++) {
          buf_i[i] = (p && p[i]) ? 1 : 0;
        }
      }
      dst.ri = buf_i.data();
      dst.rb = nullptr;
      break;
    }
    default:
      break;
  }
}

static Span<float3> lookup_float3_attr(const bke::AttributeAccessor &attributes,
                                       const StringRef name,
                                       const bke::AttrDomain domain,
                                       const int n,
                                       Array<float3> &storage)
{
  if (name.is_empty() || n <= 0) {
    return {};
  }
  const bke::GAttributeReader reader = attributes.lookup(name, domain);
  if (!reader) {
    return {};
  }
  const GVArray &va = reader.varray;
  if (!va || !va.type().is<float3>() || va.size() < n) {
    return {};
  }
  const VArray<float3> typed = va.typed<float3>();
  if (typed.is_span()) {
    const Span<float3> span = typed.get_internal_span();
    if (span.size() >= n) {
      return span.take_front(n);
    }
  }
  storage.reinitialize(n);
  typed.materialize(storage.as_mutable_span());
  return storage;
}

static const AttrInfo *attr_info_at(const Program &program, const int slot)
{
  if (slot < 0 || slot >= program.attrs.size()) {
    return nullptr;
  }
  return &program.attrs[slot];
}

/**
 * Skip bind/VM for programs that are only proximity/raycast + stores.
 * Writes output attributes in place using the same trees and grain as the
 * official Geometry Proximity / Raycast nodes.
 */
bool try_spatial_direct(const Program &program,
                        bke::GeometrySet & /*owner*/,
                        bke::MutableAttributeAccessor attributes,
                        TopoUser &topo,
                        const IndexMask &mask,
                        const int domain_size,
                        const bke::AttrDomain bind_domain)
{
  if (!program.spatial_only || program.sample_batch || domain_size <= 0 || mask.is_empty()) {
    return false;
  }
  if (!program.prox_batch && !program.ray_batch) {
    return false;
  }
  if (position_written_before_spatial(program)) {
    return false;
  }

  Span<float3> positions = topo.positions;
  if (positions.size() < domain_size) {
    return false;
  }

  ResourceScope scope;
  Vector<bke::SpanAttributeWriter<float3> *> finish3;
  Vector<bke::SpanAttributeWriter<float> *> finishf;
  Vector<bke::SpanAttributeWriter<int> *> finishi;
  Vector<bke::SpanAttributeWriter<bool> *> finishb;

  auto out_f3 = [&](const int slot) -> float3 * {
    const AttrInfo *info = attr_info_at(program, slot);
    if (!info) {
      return nullptr;
    }
    bke::SpanAttributeWriter<float3> tmp =
        attributes.convert_or_add_for_write_only_span<float3>(info->name, bind_domain);
    if (!tmp || tmp.span.size() < domain_size) {
      return nullptr;
    }
    auto &w = scope.construct<bke::SpanAttributeWriter<float3>>(std::move(tmp));
    finish3.append(&w);
    return w.span.data();
  };
  auto out_f = [&](const int slot) -> float * {
    const AttrInfo *info = attr_info_at(program, slot);
    if (!info) {
      return nullptr;
    }
    bke::SpanAttributeWriter<float> tmp =
        attributes.convert_or_add_for_write_only_span<float>(info->name, bind_domain);
    if (!tmp || tmp.span.size() < domain_size) {
      return nullptr;
    }
    auto &w = scope.construct<bke::SpanAttributeWriter<float>>(std::move(tmp));
    finishf.append(&w);
    return w.span.data();
  };
  auto out_i = [&](const int slot) -> int * {
    const AttrInfo *info = attr_info_at(program, slot);
    if (!info || info->type != Type::Int) {
      return nullptr;
    }
    bke::SpanAttributeWriter<int> tmp =
        attributes.convert_or_add_for_write_only_span<int>(info->name, bind_domain);
    if (!tmp || tmp.span.size() < domain_size) {
      return nullptr;
    }
    auto &w = scope.construct<bke::SpanAttributeWriter<int>>(std::move(tmp));
    finishi.append(&w);
    return w.span.data();
  };
  auto out_b = [&](const int slot) -> bool * {
    const AttrInfo *info = attr_info_at(program, slot);
    if (!info || info->type != Type::Bool) {
      return nullptr;
    }
    bke::SpanAttributeWriter<bool> tmp =
        attributes.convert_or_add_for_write_only_span<bool>(info->name, bind_domain);
    if (!tmp || tmp.span.size() < domain_size) {
      return nullptr;
    }
    auto &w = scope.construct<bke::SpanAttributeWriter<bool>>(std::move(tmp));
    finishb.append(&w);
    return w.span.data();
  };

  bool did = false;

  if (program.prox_batch) {
    Span<float3> samples = positions;
    Array<float3> sample_storage;
    if (program.prox_sample_attr >= 0) {
      if (const AttrInfo *info = attr_info_at(program, program.prox_sample_attr)) {
        samples = lookup_float3_attr(
            attributes, info->name, bind_domain, domain_size, sample_storage);
      }
    }
    if (samples.size() < domain_size) {
      samples = positions;
    }
    if (samples.size() < domain_size) {
      return false;
    }
    bool writes_position = false;
    for_spatial_slots(program.prox_pos_attrs, program.prox_pos_attr, [&](const int slot) {
      const AttrInfo *pos_info = attr_info_at(program, slot);
      if (pos_info && is_position_name(pos_info->name)) {
        writes_position = true;
      }
    });
    if (writes_position) {
      sample_storage = Array<float3>(samples);
      samples = sample_storage;
    }
    TopoUser::SpatialAccel &accel = spatial_of(&topo, program.prox_geo);
    const int domain = program.prox_domain;
    if (domain == 2) {
      spatial_ensure_faces(&topo, accel);
    }
    else if (domain == 1) {
      spatial_ensure_edges(&topo, accel);
    }
    else if (accel.mesh) {
      spatial_ensure_verts(&topo, accel);
    }
    else {
      spatial_ensure_points(&topo, accel);
    }
    Vector<float3 *> dst_pos;
    Vector<float *> dst_dist;
    for_spatial_slots(program.prox_pos_attrs, program.prox_pos_attr, [&](const int slot) {
      if (float3 *p = out_f3(slot)) {
        dst_pos.append(p);
      }
    });
    for_spatial_slots(program.prox_dist_attrs, program.prox_dist_attr, [&](const int slot) {
      if (float *p = out_f(slot)) {
        dst_dist.append(p);
      }
    });
    if (dst_pos.is_empty() && dst_dist.is_empty()) {
      return false;
    }
    spatial_foreach(mask, domain_size, k_proximity_grain, [&](const int i) {
      float3 pos(0.0f);
      float dist = 0.0f;
      proximity_query_accel(&topo, accel, domain, samples[i], pos, dist);
      for (float3 *p : dst_pos) {
        p[i] = pos;
      }
      for (float *p : dst_dist) {
        p[i] = dist;
      }
    });
    did = true;
  }

  if (program.ray_batch) {
    Span<float3> origins = positions;
    Array<float3> origin_storage;
    if (program.ray_orig_attr >= 0) {
      if (const AttrInfo *info = attr_info_at(program, program.ray_orig_attr)) {
        origins = lookup_float3_attr(
            attributes, info->name, bind_domain, domain_size, origin_storage);
      }
    }
    bool writes_position = false;
    for_spatial_slots(program.ray_pos_attrs, program.ray_pos_attr, [&](const int slot) {
      const AttrInfo *pos_info = attr_info_at(program, slot);
      if (pos_info && is_position_name(pos_info->name)) {
        writes_position = true;
      }
    });
    if (writes_position) {
      origin_storage = Array<float3>(origins);
      origins = origin_storage;
    }
    if (origins.size() < domain_size) {
      return false;
    }
    TopoUser::SpatialAccel &accel = spatial_of(&topo, program.ray_geo);
    spatial_ensure_tris(accel);
    if (accel.tris == nullptr || accel.mesh == nullptr) {
      return false;
    }
    Span<float3> dirs = {};
    Array<float3> dir_storage;
    if (program.ray_dir_attr >= 0) {
      if (const AttrInfo *info = attr_info_at(program, program.ray_dir_attr)) {
        dirs = lookup_float3_attr(attributes, info->name, bind_domain, domain_size, dir_storage);
      }
    }
    const float3 const_dir = program.ray_dir;
    const float length = program.ray_len;
    const bool normalize_dir = program.ray_dir_normalize;
    const bke::bvh::Tree &tree = *accel.tris;
    Vector<int *> dst_hit_i;
    Vector<bool *> dst_hit_b;
    Vector<float *> dst_hit_f;
    Vector<float3 *> dst_pos;
    Vector<float3 *> dst_n;
    Vector<float *> dst_dist;
    for_spatial_slots(program.ray_hit_attrs, program.ray_hit_attr, [&](const int slot) {
      if (int *p = out_i(slot)) {
        dst_hit_i.append(p);
      }
      else if (bool *p = out_b(slot)) {
        dst_hit_b.append(p);
      }
      else if (float *p = out_f(slot)) {
        dst_hit_f.append(p);
      }
    });
    for_spatial_slots(program.ray_pos_attrs, program.ray_pos_attr, [&](const int slot) {
      if (float3 *p = out_f3(slot)) {
        dst_pos.append(p);
      }
    });
    for_spatial_slots(program.ray_n_attrs, program.ray_n_attr, [&](const int slot) {
      if (float3 *p = out_f3(slot)) {
        dst_n.append(p);
      }
    });
    for_spatial_slots(program.ray_dist_attrs, program.ray_dist_attr, [&](const int slot) {
      if (float *p = out_f(slot)) {
        dst_dist.append(p);
      }
    });
    spatial_foreach(mask, domain_size, k_raycast_grain, [&](const int i) {
      bke::bvh::Ray ray{};
      ray.origin = origins[i];
      float3 dir = dirs.is_empty() ? const_dir : dirs[i];
      if (normalize_dir) {
        dir = math::normalize(dir);
      }
      ray.direction = dir;
      ray.dist_max = length;
      if (const std::optional<bke::bvh::RayHit> hit = tree.ray_intersect(ray)) {
        for (int *p : dst_hit_i) {
          p[i] = 1;
        }
        for (bool *p : dst_hit_b) {
          p[i] = true;
        }
        for (float *p : dst_hit_f) {
          p[i] = 1.0f;
        }
        const float3 hp = hit->position(ray);
        const float3 hn = math::normalize(hit->normal);
        for (float3 *p : dst_pos) {
          p[i] = hp;
        }
        for (float3 *p : dst_n) {
          p[i] = hn;
        }
        for (float *p : dst_dist) {
          p[i] = hit->distance;
        }
      }
      else {
        for (int *p : dst_hit_i) {
          p[i] = 0;
        }
        for (bool *p : dst_hit_b) {
          p[i] = false;
        }
        for (float *p : dst_hit_f) {
          p[i] = 0.0f;
        }
        for (float3 *p : dst_pos) {
          p[i] = float3(0.0f);
        }
        for (float3 *p : dst_n) {
          p[i] = float3(0.0f);
        }
        for (float *p : dst_dist) {
          p[i] = length;
        }
      }
    });
    did = true;
  }

  for (bke::SpanAttributeWriter<float3> *w : finish3) {
    w->finish();
  }
  for (bke::SpanAttributeWriter<float> *w : finishf) {
    w->finish();
  }
  for (bke::SpanAttributeWriter<int> *w : finishi) {
    w->finish();
  }
  for (bke::SpanAttributeWriter<bool> *w : finishb) {
    w->finish();
  }
  return did;
}

ExecOutput run_on_accessor(const Program &program,
                           bke::GeometrySet &owner,
                           bke::MutableAttributeAccessor attributes,
                           const bke::GeometryFieldContext *field_context,
                           const int domain_size,
                           const fn::Field<bool> &selection,
                           ElemUser &elem_user,
                           Vector<float3> &addpoints,
                           const Span<ChField> ch_parms)
{
  using ProfileClock = std::chrono::steady_clock;
  const bool profile = std::getenv("BLENDER_VEX_PROFILE") != nullptr;
  const auto profile_start = ProfileClock::now();
  auto profile_selection = profile_start;
  auto profile_prewarm = profile_start;
  auto profile_bind = profile_start;
  auto profile_prepare = profile_start;
  auto profile_vm = profile_start;
  ExecOutput out;
  const bke::AttrDomain bind_domain = field_context ? field_context->domain() :
                                                      bke::AttrDomain::Point;

  elem_user.self = &owner;

  if (!selection.depends_on_input() && !fn::evaluate_constant_field(selection)) {
    return out;
  }

  std::optional<fn::FieldEvaluator> eval;
  IndexMask mask(domain_size);
  if (domain_size > 0 && field_context) {
    bool parms_const = true;
    for (const ChField &parm : ch_parms) {
      if (parm.field.depends_on_input()) {
        parms_const = false;
        break;
      }
    }
    if (!selection.depends_on_input() && parms_const) {
      elem_user.parm_names.clear();
      elem_user.parm_arrays.clear();
      const int64_t parm_n = std::max<int64_t>(domain_size, 1);
      for (const ChField &parm : ch_parms) {
        const CPPType &type = parm.field.cpp_type();
        BUFFER_FOR_CPP_TYPE_VALUE(type, value);
        fn::evaluate_constant_field(parm.field, value);
        elem_user.parm_names.append(parm.name);
        elem_user.parm_arrays.append(GVArray::from_single(type, parm_n, value));
        type.destruct(value);
      }
    }
    else {
      eval.emplace(*field_context, domain_size);
      eval->set_selection(selection);
      for (const ChField &parm : ch_parms) {
        eval->add(parm.field);
      }
      eval->evaluate();
      mask = eval->get_evaluated_selection_as_mask();
      elem_user.parm_names.clear();
      elem_user.parm_arrays.clear();
      for (const int i : ch_parms.index_range()) {
        elem_user.parm_names.append(ch_parms[i].name);
        elem_user.parm_arrays.append(eval->get_evaluated(i));
      }
    }
  }
  else {
    elem_user.parm_names.clear();
    elem_user.parm_arrays.clear();
  }

  if (domain_size > 0 && mask.is_empty()) {
    return out;
  }
  profile_selection = ProfileClock::now();

  prewarm_element_samples(program, elem_user);

  TopoUser topo;
  fill_topo_user(topo, owner, elem_user);
  if (field_context) {
    topo.bind_domain = field_context->domain();
    topo.bind_type = field_context->type();
  }
  profile_prewarm = ProfileClock::now();

  /* Neighbor gather/lerp stays on CPU like Blur Attribute. Skip bind/VM. */
  if (try_jacobi_smooth_direct(
          program, owner, attributes, topo, elem_user, mask, domain_size, bind_domain))
  {
    return out;
  }

  /* Raycast / proximity: same trees and grain as the official nodes, no bind/VM. */
  if (try_spatial_direct(program, owner, attributes, topo, mask, domain_size, bind_domain)) {
    return out;
  }

  BindState state;
  if (!bind_attrs(attributes, bind_domain, domain_size, mask, program, state)) {
    out.ok = false;
    out.error = "Failed to bind attributes";
    return out;
  }
  profile_bind = ProfileClock::now();

  fill_topo_user(topo, owner, elem_user);
  if (field_context) {
    topo.bind_domain = field_context->domain();
    topo.bind_type = field_context->type();
  }

  if (try_jacobi_smooth(
          program, state, topo, elem_user, mask, domain_size, bind_domain))
  {
    finish_bind(state);
    return out;
  }

  if (program_uses_call(program, Builtin::NearestPoints) ||
      program_uses_call(program, Builtin::NearPoints))
  {
    topo.ensure_kdtree();
    const int knn_k = program.knn_precompute ? program.knn_k :
                      program.jacobi_knn     ? program.jacobi_knn_k :
                                               0;
    const int knn_geo = program.knn_precompute ? program.knn_geo : program.jacobi_knn_geo;
    if (knn_k > 0 && bind_domain == bke::AttrDomain::Point) {
      Span<int> off;
      Span<int> idx;
      if (get_knn_neighbor_table(topo, knn_geo, knn_k, domain_size, off, idx)) {
        topo.knn_off = Array<int>(off);
        topo.knn_idx = Array<int>(idx);
        topo.knn_k = knn_k;
        topo.knn_geo = knn_geo;
        topo.knn_ok = true;
      }
    }
  }
  if (program_uses_topology(program)) {
    topo.prewarm_topology();
  }
  prewarm_spatial(topo, program);

  {
    Builtin topo_id = Builtin::Sin;
    int topo_attr = -1;
    if (decode_single_topo_store(program, topo_id, topo_attr) && topo_attr >= 0 &&
        topo_attr < state.rt.size() && state.rt[topo_attr].warr && topo.mesh != nullptr &&
        bind_domain == bke::AttrDomain::Point)
    {
      c_fill_topo_attr(topo, state.rt[topo_attr], topo_id, mask);
      finish_bind(state);
      return out;
    }
  }

  VMEnv env;
  fill_counts(owner, env);
  env.attrs = state.rt.as_mutable_span();
  env.const_s = program.const_s.as_span();
  env.runtime_s = &elem_user.interned_strings;
  env.elem_user = &elem_user;
  env.load_elem = load_elem_fn;
  env.load_sample = load_sample_fn;
  Array<AttrRT> static_samples(elem_user.element_samples.size());
  int vex_bind_domain = 0;
  switch (bind_domain) {
    case bke::AttrDomain::Edge:
      vex_bind_domain = 1;
      break;
    case bke::AttrDomain::Face:
      vex_bind_domain = 2;
      break;
    case bke::AttrDomain::Corner:
      vex_bind_domain = 3;
      break;
    case bke::AttrDomain::Instance:
      vex_bind_domain = 4;
      break;
    case bke::AttrDomain::Curve:
      vex_bind_domain = 5;
      break;
    default:
      break;
  }
  for (const int i : elem_user.element_samples.index_range()) {
    const ElemUser::ElementSampleCache &sample = elem_user.element_samples[i];
    if (sample.geo > 0 || sample.domain != vex_bind_domain) {
      static_samples[i] = sample.direct;
    }
  }
  env.static_samples = static_samples;
  if (topo.mesh) {
    env.face_offsets = topo.mesh->face_offsets();
  }
  env.parm_user = &elem_user;
  env.load_parm = load_parm_fn;
  env.addpoints = &addpoints;
  env.topo_user = &topo;
  env.topo_fn = geo_builtin_fn;
  topo.write_attrs = &attributes;
  topo.attr_infos = program.attrs;
  topo.attr_rt = state.rt.as_mutable_span();
  topo.npoints_orig = env.npoints;
  topo.nedges_orig = env.nedges;
  topo.nfaces_orig = env.nfaces;
  topo.ncurves_orig = topo.curves ? topo.curves->curves_num() : 0;

  elem_user.attr_infos = program.attrs;

  {
    const bool spatial_stable = !position_written_before_spatial(program);
    if (program.spatial_only && spatial_stable && !program.sample_batch) {
      prewarm_spatial(topo, program);
      precompute_proximity(topo, program, mask, domain_size, env);
      precompute_raycast(topo, program, mask, domain_size, env);
      if (apply_spatial_kernel(topo, program, env, mask, domain_size)) {
        finish_bind(state);
        return out;
      }
    }
  }

  Vector<Array<float3>> snap_v(state.rt.size());
  Vector<Array<float>> snap_f(state.rt.size());
  Vector<Array<int>> snap_i(state.rt.size());
  Array<AttrRT> sample_rt(state.rt.size());
  auto attr_needs_snapshot = [&](const int i) {
    if (i < 0 || i >= program.attrs.size() || !program.attrs[i].write) {
      return false;
    }
    if (program.sampled_self_attr_dynamic) {
      return true;
    }
    return program.sampled_self_attrs.contains(program.attrs[i].name);
  };
  auto take_snapshot = [&]() {
    for (const int i : state.rt.index_range()) {
      if (attr_needs_snapshot(i)) {
        snapshot_attr_rt(state.rt[i], sample_rt[i], snap_v[i], snap_f[i], snap_i[i]);
      }
    }
  };
  /* Only sampled attributes that are also written need a stable Jacobi view. Static sampling of
   * unrelated read-only attributes (for example corner(0, @UV, i) while writing distortion) can
   * read the geometry directly without copying every bound attribute. Dynamic names remain
   * conservative to preserve parallel VM correctness. */
  const bool needs_snapshot = program.sampled_self_attr_dynamic ||
                              !program.sampled_self_attrs.is_empty();
  if (domain_size > 0 && needs_snapshot) {
    take_snapshot();
    elem_user.sample_attrs = sample_rt;
  }

  Span<int> nbr_off;
  Span<int> nbr_idx;
  if (program.gpu_neighbors || program.jacobi_smooth) {
    get_point_neighbor_table(topo, domain_size, nbr_off, nbr_idx);
    env.gpu_nbr_off = nbr_off;
    env.gpu_nbr_idx = nbr_idx;
  }

  Vector<Array<int>> ch_i(program.gpu_ch.size());
  Vector<Array<float>> ch_f(program.gpu_ch.size());
  Vector<Array<float3>> ch_v(program.gpu_ch.size());
  Array<VMEnv::ChSrc> ch_src(program.gpu_ch.size());
  for (const int ci : program.gpu_ch.index_range()) {
    const Program::GpuCh &ch = program.gpu_ch[ci];
    VMEnv::ChSrc src;
    src.type = ch.type;
    const GVArray *va = nullptr;
    for (const int pi : elem_user.parm_names.index_range()) {
      if (elem_user.parm_names[pi] == ch.name) {
        va = &elem_user.parm_arrays[pi];
        break;
      }
    }
    if (ch.type == Type::Int || ch.type == Type::Bool) {
      ch_i[ci].reinitialize(std::max(domain_size, 1));
      ch_i[ci].fill(0);
      if (va && *va && domain_size > 0) {
        if (va->type().is<int>()) {
          va->typed<int>().materialize(ch_i[ci].as_mutable_span());
        }
        else if (va->type().is<bool>()) {
          Array<bool> tmp(domain_size);
          va->typed<bool>().materialize(tmp.as_mutable_span());
          for (int i = 0; i < domain_size; i++) {
            ch_i[ci][i] = tmp[i] ? 1 : 0;
          }
        }
        else if (va->type().is<float>()) {
          Array<float> tmp(domain_size);
          va->typed<float>().materialize(tmp.as_mutable_span());
          for (int i = 0; i < domain_size; i++) {
            ch_i[ci][i] = int(tmp[i]);
          }
        }
      }
      src.i = ch_i[ci].data();
    }
    else if (ch.type == Type::Vector || ch.type == Type::Color) {
      ch_v[ci].reinitialize(std::max(domain_size, 1));
      ch_v[ci].fill(float3(0.0f));
      if (va && *va && domain_size > 0 && va->type().is<float3>()) {
        va->typed<float3>().materialize(ch_v[ci].as_mutable_span());
      }
      src.v = ch_v[ci].data();
    }
    else {
      ch_f[ci].reinitialize(std::max(domain_size, 1));
      ch_f[ci].fill(0.0f);
      if (va && *va && domain_size > 0) {
        if (va->type().is<float>()) {
          va->typed<float>().materialize(ch_f[ci].as_mutable_span());
        }
        else if (va->type().is<int>()) {
          Array<int> tmp(domain_size);
          va->typed<int>().materialize(tmp.as_mutable_span());
          for (int i = 0; i < domain_size; i++) {
            ch_f[ci][i] = float(tmp[i]);
          }
        }
      }
      src.f = ch_f[ci].data();
    }
    ch_src[ci] = src;
  }
  env.gpu_ch_src = ch_src;

  const int niter = resolve_array_passes(program, elem_user);
  env.array_passes = std::max(niter, 1);

  if (program_uses_call(program, Builtin::DeleteGeometry)) {
    /* `reinitialize` default-constructs uint8_t as uninitialized garbage. Zero-fill
     * or random elements are treated as already deleted and the mesh looks shredded. */
    topo.del_point = Array<uint8_t>(std::max(env.npoints, 0), uint8_t(0));
    topo.del_edge = Array<uint8_t>(std::max(env.nedges, 0), uint8_t(0));
    topo.del_face = Array<uint8_t>(std::max(env.nfaces, 0), uint8_t(0));
    int ncurves = 0;
    int ninst = 0;
    if (topo.curves) {
      ncurves = topo.curves->curves_num();
    }
    if (const bke::Instances *inst = owner.get_instances()) {
      ninst = inst->instances_num();
    }
    topo.del_curve = Array<uint8_t>(std::max(ncurves, 0), uint8_t(0));
    topo.del_inst = Array<uint8_t>(std::max(ninst, 0), uint8_t(0));
  }

  const bool spatial_stable = !position_written_before_spatial(program);
  if (spatial_stable) {
    precompute_proximity(topo, program, mask, domain_size, env);
    precompute_raycast(topo, program, mask, domain_size, env);
    precompute_sample(topo, program, mask, domain_size, env);
  }

  if (program.spatial_only && spatial_stable && !program.sample_batch &&
      apply_spatial_kernel(topo, program, env, mask, domain_size))
  {
    finish_bind(state);
    return out;
  }

  profile_prepare = ProfileClock::now();
  std::string error;
  std::atomic<bool> ok = true;

  if (program.array_pass_peeled && niter <= 0) {
    finish_bind(state);
    return out;
  }

  if (program.gpu_ok && domain_size > 0 && !program_uses_addpoint(program) &&
      !program_uses_spatial(program) && !program_uses_geo_side_effects(program))
  {
    if (gpu_try_run(program, env, mask, domain_size, error)) {
      finish_bind(state);
      return out;
    }
    error.clear();
  }

  auto run_mask = [&](const IndexMask &run) {
    if (domain_size <= 0) {
      env.index = 0;
      if (!vm_run(program, env, error, nullptr)) {
        ok = false;
      }
      return;
    }
    if (run.is_empty()) {
      return;
    }
    /* addpoint / setattribute use atomics (or a short append lock), so the VM
     * can run length-n lanes on all cores. */
    if (!vm_run_array(program, env, run, error)) {
      ok = false;
    }
  };

  const int cpu_passes = program.array_pass_peeled ? std::max(niter, 1) : 1;
  for (int pass = 0; pass < cpu_passes; pass++) {
    if (pass > 0 && domain_size > 0) {
      take_snapshot();
      /* Positions changed last pass: rebuild the k-d tree / kNN table so
       * nearestpoints inside the peeled loop sees the new P. */
      if (program_uses_call(program, Builtin::NearestPoints) ||
          program_uses_call(program, Builtin::NearPoints))
      {
        topo.invalidate_spatial();
        topo.ensure_kdtree();
        const int knn_k = program.knn_precompute ? program.knn_k :
                          program.jacobi_knn     ? program.jacobi_knn_k :
                                                   0;
        const int knn_geo = program.knn_precompute ? program.knn_geo : program.jacobi_knn_geo;
        if (knn_k > 0 && bind_domain == bke::AttrDomain::Point) {
          Span<int> off;
          Span<int> idx;
          if (get_knn_neighbor_table(topo, knn_geo, knn_k, domain_size, off, idx)) {
            topo.knn_off = Array<int>(off);
            topo.knn_idx = Array<int>(idx);
            topo.knn_k = knn_k;
            topo.knn_geo = knn_geo;
            topo.knn_ok = true;
          }
        }
      }
    }
    if (domain_size <= 0) {
      run_mask(IndexMask(0));
    }
    else {
      run_mask(mask);
    }
    if (!ok.load()) {
      break;
    }
  }
  profile_vm = ProfileClock::now();

  finish_bind(state);
  finish_extra_writers(topo);
  if (ok.load()) {
    apply_removes(owner, topo);
    {
      std::lock_guard lock(topo.add_mutex);
      addpoints = topo.add_pos;
    }
    if (!addpoints.is_empty()) {
      merge_addpoints(owner, addpoints);
      addpoints.clear();
      apply_pending_sets(topo, owner);
    }
    else if (!topo.pending_sets.is_empty()) {
      apply_pending_sets(topo, owner);
    }
    merge_addprims(owner, topo);
  }
  if (topo.geo_forced.load(std::memory_order_relaxed)) {
    const int g = topo.geo_forced_from.load(std::memory_order_relaxed);
    out.warning = "geo 输入为 " + std::to_string(g) +
                  "，已自动改为 0（addpoint / addprim / setattribute / delete_geometry 只支持当前几何）";
  }
  if (!ok.load()) {
    out.ok = false;
    out.error = error.empty() ? "Wrangle evaluation failed" : error;
  }
  if (profile) {
    const auto profile_end = ProfileClock::now();
    auto ms = [](const auto a, const auto b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };
    std::fprintf(stderr,
                 "VEX_CORE domain=%d mask=%lld select=%.3f prewarm=%.3f bind=%.3f "
                 "prepare=%.3f vm=%.3f finish=%.3f total=%.3f\n",
                 domain_size,
                 static_cast<long long>(mask.size()),
                 ms(profile_start, profile_selection),
                 ms(profile_selection, profile_prewarm),
                 ms(profile_prewarm, profile_bind),
                 ms(profile_bind, profile_prepare),
                 ms(profile_prepare, profile_vm),
                 ms(profile_vm, profile_end),
                 ms(profile_start, profile_end));
  }
  return out;
}

ExecOutput run_on_component(const Program &program,
                            bke::GeometrySet &owner,
                            bke::GeometryComponent &component,
                            const bke::AttrDomain domain,
                            const fn::Field<bool> &selection,
                            ElemUser &elem_user,
                            Vector<float3> &addpoints,
                            const Span<ChField> ch_parms)
{
  ExecOutput out;
  component.ensure_owns_direct_data();
  elem_user.attr_domain = domain;
  std::optional<bke::MutableAttributeAccessor> attributes = component.attributes_for_write();
  if (!attributes) {
    out.ok = false;
    out.error = "Cannot write attributes on this geometry";
    return out;
  }
  const int domain_size = component.attribute_domain_size(domain);
  const bke::GeometryFieldContext field_context(component, domain);
  return run_on_accessor(program,
                         owner,
                         *attributes,
                         &field_context,
                         domain_size,
                         selection,
                         elem_user,
                         addpoints,
                         ch_parms);
}

}  // namespace

ExecOutput execute(const Program &program,
                   bke::GeometrySet &geometry,
                   const Span<const bke::GeometrySet *> extra_geometry,
                   const Domain domain,
                   const fn::Field<bool> &selection,
                   const Span<ChField> ch_parms)
{
  ExecOutput out;
  geometry.ensure_owns_direct_data();
  ElemUser elem_user;
  elem_user.geos.append(&geometry);
  for (const bke::GeometrySet *g : extra_geometry) {
    if (g) {
      elem_user.geos.append(g);
    }
  }

  if (domain == Domain::Instance) {
    if (geometry.has_instances()) {
      bke::GeometryComponent &component = geometry.get_component_for_write(
          bke::GeometryComponent::Type::Instance);
      Vector<float3> addpoints;
      out = run_on_component(program,
                             geometry,
                             component,
                             bke::AttrDomain::Instance,
                             selection,
                             elem_user,
                             addpoints,
                             ch_parms);
    }
    return out;
  }

  auto run_realized = [&](bke::GeometrySet &geo) {
    if (!out.ok) {
      return;
    }
    ElemUser local_user;
    local_user.geos = elem_user.geos;
    if (local_user.geos.is_empty()) {
      local_user.geos.append(&geo);
    }
    else {
      local_user.geos[0] = &geo;
    }
    local_user.self = &geo;
    const bke::AttrDomain ad = to_attr_domain(domain);
    Vector<float3> addpoints;
    if (geo.has_mesh() && ELEM(ad,
                               bke::AttrDomain::Point,
                               bke::AttrDomain::Edge,
                               bke::AttrDomain::Face,
                               bke::AttrDomain::Corner))
    {
      bke::GeometryComponent &component = geo.get_component_for_write(
          bke::GeometryComponent::Type::Mesh);
      ExecOutput part = run_on_component(
          program, geo, component, ad, selection, local_user, addpoints, ch_parms);
      if (!part.ok) {
        out = std::move(part);
      }
      else if (!part.warning.empty() && out.warning.empty()) {
        out.warning = std::move(part.warning);
      }
    }
    else if (geo.has_pointcloud() && ad == bke::AttrDomain::Point) {
      bke::GeometryComponent &component = geo.get_component_for_write(
          bke::GeometryComponent::Type::PointCloud);
      ExecOutput part = run_on_component(
          program, geo, component, ad, selection, local_user, addpoints, ch_parms);
      if (!part.ok) {
        out = std::move(part);
      }
      else if (!part.warning.empty() && out.warning.empty()) {
        out.warning = std::move(part.warning);
      }
    }
    else if (geo.has_curves() && ELEM(ad, bke::AttrDomain::Point, bke::AttrDomain::Curve)) {
      bke::GeometryComponent &component = geo.get_component_for_write(
          bke::GeometryComponent::Type::Curve);
      ExecOutput part = run_on_component(
          program, geo, component, ad, selection, local_user, addpoints, ch_parms);
      if (!part.ok) {
        out = std::move(part);
      }
      else if (!part.warning.empty() && out.warning.empty()) {
        out.warning = std::move(part.warning);
      }
    }
    if (out.ok) {
      merge_addpoints(geo, addpoints);
    }
  };

  /* Copying the mesh out and joining it back is ~10ms of fixed cost. Blur Attribute
   * edits in place; do the same when there are no instances to realize. */
  if (!geometry.has_instances()) {
    run_realized(geometry);
  }
  else {
    geometry::foreach_real_geometry(geometry, run_realized);
  }

  return out;
}

PureEvalOutput execute_pure(const Program &program)
{
  PureEvalOutput out;
  VMEnv env;
  Vector<std::string> interned;
  env.runtime_s = &interned;
  Value ret;
  std::string error;
  if (!vm_run(program, env, error, &ret)) {
    out.ok = false;
    out.error = error;
    return out;
  }
  out.has_return = true;
  out.return_int = ret.as_int();
  out.return_float = ret.as_float();
  out.return_vec = ret.as_vec();
  if (ret.type == Type::String) {
    out.return_string = std::string(value_string(ret, env));
  }
  return out;
}

}  // namespace blender::nodes::vex
