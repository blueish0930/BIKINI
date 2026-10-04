/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Image Process: Point Stamp — instance stamp textures on points (rotation / size / id).
 * Perspective projection + Z-sorted occlusion, matching 3D Instance-on-Points plane look. */

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "BLI_array.hh"
#include "BLI_color.hh"
#include "BLI_color_types.hh"
#include "BLI_map.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_node_types.h"
#include "DNA_pointcloud_types.h"

#include "BKE_anonymous_attribute_id.hh"
#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_mesh.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_pointcloud.hh"

#include "BLI_math_base.hh"

#include "COM_domain.hh"
#include "COM_input_descriptor.hh"
#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_undefined_node_operation.hh"
#include "COM_utilities.hh"

#include "NOD_geo_bundle.hh"
#include "NOD_image_points.hh"
#include "NOD_socket_declarations.hh"
#include "NOD_socket_declarations_geometry.hh"
#include "NOD_socket_items.hh"

#include "UI_resources.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_point_stamp_cc {

using namespace blender::compositor;
using namespace blender::nodes::image_points;

/* -------------------------------------------------------------------- */
/** \name Bundle signature from point attributes
 * \{ */

static bool attr_type_allowed(const bke::AttrType type)
{
  /* Bundle items are Color maps painted over each stamp region with the attribute value. */
  return ELEM(type,
              bke::AttrType::Float,
              bke::AttrType::Int32,
              bke::AttrType::Bool,
              bke::AttrType::Float2,
              bke::AttrType::Float3,
              bke::AttrType::Float4,
              bke::AttrType::ColorFloat);
}

static bool attr_name_for_bundle(const StringRef name)
{
  /* Skip internal/anonymous Capture Attribute IDs (.a_… / _a_…). Keep radius/size/id and all
   * user Store Named Attribute names so Separate Bundle can expose them. */
  if (name.is_empty() || name.startswith(".") || name.startswith("_a_") ||
      bke::attribute_name_is_anonymous(name))
  {
    return false;
  }
  /* Skip only pure topology / reserved names — keep "radius", "size", "id", custom attrs.
   * Geometry "alpha" is skipped so the synthetic stamp coverage mask owns that Bundle slot. */
  return name != "position" && name != "rotation" && name != attr_rotation && name != "Color" &&
         name != "Bundle" && name != ".selection" && name != ".select_vert" &&
         name != attr_alpha;
}

struct AttrBundleItem {
  std::string name;
  bke::AttrType type = bke::AttrType::Float;
};

/** Map attribute data type → Separate Bundle / hidden multi-output socket type. */
static eNodeSocketDatatype socket_type_for_attr(const bke::AttrType data_type)
{
  switch (data_type) {
    case bke::AttrType::Float:
    case bke::AttrType::Bool:
      return SOCK_FLOAT;
    case bke::AttrType::Int32:
      return SOCK_INT;
    case bke::AttrType::Float2:
    case bke::AttrType::Float3:
    case bke::AttrType::Float4:
      return SOCK_VECTOR;
    case bke::AttrType::ColorFloat:
    default:
      return SOCK_RGBA;
  }
}

static std::optional<int> vector_dimensions_for_attr(const bke::AttrType data_type)
{
  switch (data_type) {
    case bke::AttrType::Float2:
      return 2;
    case bke::AttrType::Float3:
      return 3;
    case bke::AttrType::Float4:
      return 4;
    default:
      return std::nullopt;
  }
}

static Vector<AttrBundleItem> attributes_from_geometry(const bke::GeometrySet *geometry)
{
  /* Fixed leading order: radius → size → id, then remaining point attributes. */
  std::optional<AttrBundleItem> radius_item;
  std::optional<AttrBundleItem> size_item;
  std::optional<AttrBundleItem> id_item;
  Vector<AttrBundleItem> others;

  if (geometry) {
    std::optional<bke::AttributeAccessor> attrs;
    if (geometry->has_pointcloud()) {
      attrs = geometry->get_pointcloud()->attributes();
    }
    else if (geometry->has_mesh()) {
      attrs = geometry->get_mesh()->attributes();
    }
    else if (geometry->has_curves()) {
      attrs = geometry->get_curves()->geometry.wrap().attributes();
    }
    if (attrs) {
      attrs->foreach_attribute([&](const bke::AttributeIter &iter) {
        if (iter.domain != bke::AttrDomain::Point) {
          return;
        }
        if (!attr_type_allowed(iter.data_type)) {
          return;
        }
        if (!attr_name_for_bundle(iter.name)) {
          return;
        }
        AttrBundleItem item{std::string(iter.name), iter.data_type};
        if (iter.name == "radius") {
          radius_item = item;
        }
        else if (iter.name == attr_size) {
          size_item = item;
        }
        else if (iter.name == attr_id) {
          id_item = item;
        }
        else {
          others.append(std::move(item));
        }
      });
    }
  }

  Vector<AttrBundleItem> items;
  /* Fixed order: radius → size → id → alpha (stamp coverage mask) → other point attrs.
   * Float/Vector maps have no alpha channel; "alpha" is 1 only on valid stamp texels. */
  items.append(radius_item.value_or(AttrBundleItem{"radius", bke::AttrType::Float}));
  items.append(size_item.value_or(AttrBundleItem{std::string(attr_size), bke::AttrType::Float2}));
  items.append(id_item.value_or(AttrBundleItem{std::string(attr_id), bke::AttrType::Int32}));
  items.append(AttrBundleItem{std::string(attr_alpha), bke::AttrType::Float});
  items.extend(others);
  return items;
}

static Vector<std::string> attribute_names_from_geometry(const bke::GeometrySet *geometry)
{
  Vector<std::string> names;
  for (const AttrBundleItem &item : attributes_from_geometry(geometry)) {
    names.append(item.name);
  }
  return names;
}

/**
 * Resolve point geometry for Bundle signature during declare/update.
 *
 * NEVER evaluates the nested Geometry Node tree here: tree updates (link / Store Named Attribute
 * socket rebuild) re-enter declare while depsgraph/self-object context is missing, which crashed
 * Object Info (and similar) via DEG_object_transform_is_evaluated(null).
 *
 * Order: live COM cook cache → last successful Import Points cook geometry.
 */
static bke::GeometrySet geometry_from_points_links(const bNode &node)
{
  auto geometry_from_import = [](const bNode &from) -> bke::GeometrySet {
    if (!from.is_type("ImageNodeImportPoints"_ustr) || !from.id || GS(from.id->name) != ID_NT) {
      return {};
    }
    const bNodeTree &geo_tree = *reinterpret_cast<const bNodeTree *>(from.id);
    if (geo_tree.type != NTREE_GEOMETRY) {
      return {};
    }
    /* Active cook: geometry is already in PointsGeometryCache (keyed from COM). Scan all entries
     * only if needed — prefer last-import map which is tree-session keyed. */
    if (const bke::GeometrySet *last = lookup_last_import_points_geometry(geo_tree)) {
      return *last;
    }
    return {};
  };

  /* Prefer live cook cache when available (Import Points already stored under COM string key). */
  if (PointsGeometryCache *cache = active_cache()) {
    for (const bNodeSocket *socket : node.input_sockets()) {
      if (socket->type != SOCK_GEOMETRY || !socket->is_available()) {
        continue;
      }
      /* During cook, Point Stamp operation reads the linked key; declare path has no Result.
       * Fall through to last-import geometry. */
      UNUSED_VARS(cache, socket);
    }
  }

  for (const bNodeSocket *socket : node.input_sockets()) {
    if (socket->type != SOCK_GEOMETRY || !socket->is_available()) {
      continue;
    }
    for (const bNodeLink *link : socket->directly_linked_links()) {
      if (!link->is_used() || !link->fromnode) {
        continue;
      }
      bke::GeometrySet geo = geometry_from_import(*link->fromnode);
      if (!geo.is_empty()) {
        return geo;
      }
    }
  }
  return {};
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Declaration
 * \{ */

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Points"_ustr)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description(
          "Point cloud from Import Points. Position XYZ in unit-square [0,1] (Z = depth). "
          "Size: ≤1 = fraction of image; >1 = pixels at 1024 reference (stable when W×H changes)");

  b.add_input<decl::Color>("Stamp"_ustr)
      .multi_input()
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description(
          "Stamp textures (join multi-input). Color keeps RGB+A; Float masks → white×alpha. "
          "Point samples stamp[id % n] on a 3D UV plane (rotation + size), perspective + Z sort");

  bke::GeometrySet geo_for_sig;
  if (const bNode *node = b.node_or_null()) {
    geo_for_sig = geometry_from_points_links(*node);
  }

  b.add_output<decl::Color>("Color"_ustr)
      .structure_type(StructureType::Dynamic)
      .description(
          "Stamped textures on points: perspective view of UV planes, occluded by point Z order");

  /* Plain Bundle (not typed FlatBundle). Separate Bundle Sync rebuilds items from point attrs. */
  b.add_output<decl::Bundle>("Bundle"_ustr)
      .structure_type(StructureType::Single)
      .description(
          "Per-attribute maps on stamp regions. Connect Separate Bundle and Sync to expand "
          "items (float / vector / color match original attributes)");

  /* Hidden multi-outputs carry attribute maps for COM / Separate Bundle / Bake.
   * Types match original attributes. Only Color + Bundle are user-facing. */
  for (const AttrBundleItem &item : attributes_from_geometry(
           geo_for_sig.is_empty() ? nullptr : &geo_for_sig))
  {
    switch (item.type) {
      case bke::AttrType::Float:
      case bke::AttrType::Bool:
        b.add_output<decl::Float>(UString(item.name))
            .structure_type(StructureType::Dynamic)
            .description("Internal attribute map (hidden)");
        break;
      case bke::AttrType::Int32:
        b.add_output<decl::Int>(UString(item.name))
            .structure_type(StructureType::Dynamic)
            .description("Internal attribute map (hidden)");
        break;
      case bke::AttrType::Float2:
        b.add_output<decl::Vector>(UString(item.name))
            .dimensions(2)
            .structure_type(StructureType::Dynamic)
            .description("Internal attribute map (hidden)");
        break;
      case bke::AttrType::Float3:
        b.add_output<decl::Vector>(UString(item.name))
            .dimensions(3)
            .structure_type(StructureType::Dynamic)
            .description("Internal attribute map (hidden)");
        break;
      case bke::AttrType::Float4:
        b.add_output<decl::Vector>(UString(item.name))
            .dimensions(4)
            .structure_type(StructureType::Dynamic)
            .description("Internal attribute map (hidden)");
        break;
      default:
        b.add_output<decl::Color>(UString(item.name))
            .structure_type(StructureType::Dynamic)
            .description("Internal attribute map (hidden)");
        break;
    }
  }
}

/** True if Separate Bundle storage already matches the desired attr list (avoids thrashing). */
static bool separate_bundle_items_match(const NodeSeparateBundle &storage,
                                        const Span<AttrBundleItem> items)
{
  if (storage.items_num != items.size()) {
    return false;
  }
  for (const int i : items.index_range()) {
    const NodeSeparateBundleItem &cur = storage.items[i];
    if (!cur.name || StringRef(cur.name) != items[i].name) {
      return false;
    }
    if (cur.socket_type != socket_type_for_attr(items[i].type)) {
      return false;
    }
  }
  return true;
}

/**
 * Auto-sync linked Separate Bundle items from point attributes (plain Bundle path).
 * Only mutates when the item list actually changes — rebuild every cook was thrashing UI/DNA.
 */
static void auto_sync_linked_separate_bundles(bNodeTree &ntree,
                                              bNode &stamp_node,
                                              const Span<AttrBundleItem> items)
{
  ntree.ensure_topology_cache();
  bNodeSocket *bundle_out = nullptr;
  for (bNodeSocket *socket : stamp_node.output_sockets()) {
    if (socket->type == SOCK_BUNDLE) {
      bundle_out = socket;
      break;
    }
  }
  if (!bundle_out) {
    return;
  }

  for (bNodeLink *link : ntree.all_links()) {
    if (link->fromsock != bundle_out || !link->tonode) {
      continue;
    }
    bNode &to_node = *link->tonode;
    if (!to_node.is_type("NodeSeparateBundle"_ustr)) {
      continue;
    }
    auto &storage = *static_cast<NodeSeparateBundle *>(to_node.storage);
    if (separate_bundle_items_match(storage, items)) {
      continue;
    }
    Map<std::string, int> old_identifiers;
    for (const int i : IndexRange(storage.items_num)) {
      old_identifiers.add_new(StringRef(storage.items[i].name), storage.items[i].identifier);
    }
    socket_items::clear<SeparateBundleItemsAccessor>(to_node);
    for (const AttrBundleItem &item : items) {
      const eNodeSocketDatatype sock_type = socket_type_for_attr(item.type);
      const std::optional<int> dims = vector_dimensions_for_attr(item.type);
      NodeSeparateBundleItem &new_item =
          *socket_items::add_item_with_socket_type_and_name<SeparateBundleItemsAccessor>(
              ntree, to_node, sock_type, item.name.c_str(), dims);
      new_item.structure_type = NodeSocketInterfaceStructureType::Dynamic;
      if (const std::optional<int> id = old_identifiers.lookup_try(item.name)) {
        new_item.identifier = *id;
      }
    }
    BKE_ntree_update_tag_node_property(&ntree, &to_node);
  }
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  if (!ntree || !node) {
    return;
  }
  bke::GeometrySet geo = geometry_from_points_links(*node);
  const Vector<AttrBundleItem> items = attributes_from_geometry(
      geo.is_empty() ? nullptr : &geo);
  auto_sync_linked_separate_bundles(*ntree, *node, items);

  /* Only Color + Bundle are user-facing; attribute maps stay evaluable but hidden. */
  for (bNodeSocket *sock : node->output_sockets()) {
    if (!sock) {
      continue;
    }
    if (sock->type == SOCK_BUNDLE) {
      sock->flag &= ~SOCK_HIDDEN;
      continue;
    }
    if (StringRef(sock->identifier) == "Color" || StringRef(sock->name) == "Color") {
      sock->flag &= ~SOCK_HIDDEN;
    }
    else if (ELEM(sock->type, SOCK_RGBA, SOCK_FLOAT, SOCK_VECTOR, SOCK_INT, SOCK_BOOLEAN)) {
      sock->flag |= SOCK_HIDDEN;
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name COM evaluation — perspective UV planes + Z occlusion + attribute maps
 * \{ */

/** Camera distance along +Z looking toward -Z. At z=0 projection matches unit-square ortho. */
static constexpr float stamp_camera_distance = 1.0f;

struct PointStampData {
  float3 position;
  math::Quaternion rotation;
  float3 size;
  int id;
  /** Per-bundle-item solid color for this point (same order as attribute_names). */
  Vector<Color> attr_colors;
};

static Color attribute_value_to_color(const bke::GAttributeReader &reader, const int index)
{
  if (!reader || !reader.varray) {
    return Color(0.0f, 0.0f, 0.0f, 1.0f);
  }
  const CPPType &type = reader.varray.type();
  if (type.is<float>()) {
    const float v = reader.varray.get<float>(index);
    return Color(v, v, v, 1.0f);
  }
  if (type.is<int>()) {
    const float v = float(reader.varray.get<int>(index));
    return Color(v, v, v, 1.0f);
  }
  if (type.is<bool>()) {
    const float v = reader.varray.get<bool>(index) ? 1.0f : 0.0f;
    return Color(v, v, v, 1.0f);
  }
  if (type.is<float2>()) {
    const float2 v = reader.varray.get<float2>(index);
    return Color(v.x, v.y, 0.0f, 1.0f);
  }
  if (type.is<float3>()) {
    const float3 v = reader.varray.get<float3>(index);
    return Color(v.x, v.y, v.z, 1.0f);
  }
  if (type.is<float4>()) {
    const float4 v = reader.varray.get<float4>(index);
    return Color(v.x, v.y, v.z, v.w);
  }
  if (type.is<ColorGeometry4f>()) {
    const ColorGeometry4f c = reader.varray.get<ColorGeometry4f>(index);
    return Color(c.r, c.g, c.b, c.a);
  }
  return Color(0.0f, 0.0f, 0.0f, 1.0f);
}

static Vector<PointStampData> gather_points(const bke::GeometrySet &geometry,
                                            const Span<std::string> attr_names)
{
  Vector<PointStampData> points;
  std::optional<bke::AttributeAccessor> attrs;
  int count = 0;
  if (geometry.has_pointcloud()) {
    const PointCloud &pc = *geometry.get_pointcloud();
    attrs = pc.attributes();
    count = pc.totpoint;
  }
  else if (geometry.has_mesh()) {
    const Mesh &mesh = *geometry.get_mesh();
    attrs = mesh.attributes();
    count = mesh.verts_num;
  }
  else if (geometry.has_curves()) {
    /* Fallback if Import Points conversion was skipped — use curve control points. */
    const Curves &curves_id = *geometry.get_curves();
    const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
    attrs = curves.attributes();
    count = curves.points_num();
  }
  if (!attrs || count <= 0) {
    return points;
  }

  const VArraySpan<float3> positions = *attrs->lookup_or_default<float3>(
      "position", bke::AttrDomain::Point, float3(0.5f, 0.5f, 0.0f));
  const VArray<math::Quaternion> rotations = *attrs->lookup_or_default<math::Quaternion>(
      attr_rotation, bke::AttrDomain::Point, math::Quaternion::identity());
  /* Size: 2D preferred (width, height) in pixels; float3 / float fallback; then radius. */
  const std::optional<bke::AttributeMetaData> size_meta = attrs->lookup_meta_data(attr_size);
  const bool size_is_float2 = size_meta && size_meta->data_type == bke::AttrType::Float2;
  const bool size_is_float = size_meta && size_meta->data_type == bke::AttrType::Float;
  const bool has_size = size_meta.has_value();
  const VArray<float2> sizes2 = *attrs->lookup_or_default<float2>(
      attr_size, bke::AttrDomain::Point, float2(64.0f));
  const VArray<float3> sizes3 = *attrs->lookup_or_default<float3>(
      attr_size, bke::AttrDomain::Point, float3(64.0f));
  const VArray<float> sizes1 = *attrs->lookup_or_default<float>(
      attr_size, bke::AttrDomain::Point, 64.0f);
  /* GeometryNodePoints uses "radius" (unit space). Keep it and use as stamp size when "size"
   * is absent so radius is not "lost" for stamping. */
  const VArray<float> radii = *attrs->lookup_or_default<float>(
      "radius", bke::AttrDomain::Point, 0.1f);
  const VArray<int> ids = *attrs->lookup_or_default<int>(attr_id, bke::AttrDomain::Point, 0);

  Vector<bke::GAttributeReader> attr_readers;
  attr_readers.reserve(attr_names.size());
  for (const std::string &name : attr_names) {
    attr_readers.append(attrs->lookup(name));
  }

  points.reinitialize(count);
  threading::parallel_for(IndexRange(count), 1024, [&](const IndexRange range) {
    for (const int i : range) {
      PointStampData &pt = points[i];
      pt.position = positions[i];
      pt.rotation = rotations[i];
      if (has_size && size_is_float2) {
        const float2 s = sizes2[i];
        pt.size = float3(s.x, s.y, 0.0f);
      }
      else if (has_size && size_is_float) {
        const float s = sizes1[i];
        pt.size = float3(s, s, 0.0f);
      }
      else if (has_size) {
        pt.size = sizes3[i];
      }
      else {
        /* No "size": use radius. ≤1 = unit-square diameter; >1 = pixels at 1024 reference. */
        const float r = math::max(radii[i], 1e-6f);
        if (r <= 1.0f + 1e-5f) {
          pt.size = float3(r * 2.0f, r * 2.0f, 0.0f);
        }
        else {
          pt.size = float3(r, r, 0.0f);
        }
      }
      pt.id = ids[i];
      pt.attr_colors.reinitialize(attr_names.size());
      for (const int a : attr_names.index_range()) {
        if (attr_names[a] == attr_id) {
          const float v = float(pt.id);
          pt.attr_colors[a] = Color(v, v, v, 1.0f);
        }
        else if (attr_names[a] == attr_size) {
          pt.attr_colors[a] = Color(pt.size.x, pt.size.y, pt.size.z, 1.0f);
        }
        else if (attr_names[a] == attr_alpha) {
          /* Coverage mask is painted per-pixel from stamp coverage, not a point attribute. */
          pt.attr_colors[a] = Color(1.0f, 1.0f, 1.0f, 1.0f);
        }
        else if (attr_names[a] == "radius") {
          const float r = radii[i];
          pt.attr_colors[a] = Color(r, r, r, 1.0f);
        }
        else {
          pt.attr_colors[a] = attribute_value_to_color(attr_readers[a], i);
        }
      }
    }
  });
  return points;
}

/** Project unit-space world point to pixel coords. Returns false if behind camera. */
static bool project_world_to_pixel(const float3 &world,
                                   const int2 res,
                                   const float cam_dist,
                                   float2 &r_pixel,
                                   float &r_depth)
{
  const float denom = cam_dist - world.z;
  if (denom <= 1e-5f) {
    return false;
  }
  const float scale = cam_dist / denom;
  const float2 ndc(0.5f + (world.x - 0.5f) * scale, 0.5f + (world.y - 0.5f) * scale);
  r_pixel = float2(ndc.x * float(res.x), ndc.y * float(res.y));
  r_depth = denom; /* Smaller = closer to camera. */
  return true;
}

/**
 * Reference resolution for pixel-sized stamps.
 * Pixel size values (e.g. 64) mean "pixels at this reference", not at the current editor
 * resolution — so changing Image Process W×H only changes output pixel density, not layout.
 */
static constexpr float stamp_size_reference_resolution = 1024.0f;

/** Unit-space instance scale from size attribute (unit fraction if |s| ≤ 1, else pixels @ 1024). */
static float3 size_to_unit_scale(const float3 &size, const int2 /*res*/)
{
  float3 s(math::abs(size.x), math::abs(size.y), math::abs(size.z));
  if (s.y < 1e-8f) {
    s.y = s.x;
  }
  if (s.z < 1e-8f) {
    s.z = s.x;
  }
  /* Values > 1: pixels relative to fixed 1024² reference (resolution-independent layout).
   * Values ≤ 1: already unit-square fractions of the image. */
  if (math::max(s.x, s.y) > 1.0f + 1e-5f) {
    const float ref = stamp_size_reference_resolution;
    s.x /= ref;
    s.y /= ref;
    s.z /= ref;
  }
  return math::max(s, float3(1e-6f));
}

/** World position of local plane corner; local UV in [-0.5, 0.5]², z=0 in instance space. */
static float3 plane_world_point(const PointStampData &pt,
                                const float3 &unit_scale,
                                const float lu,
                                const float lv)
{
  const float3x3 rot = math::from_rotation<float3x3>(pt.rotation);
  const float3 local(lu * unit_scale.x, lv * unit_scale.y, 0.0f);
  return pt.position + rot * local;
}

static bool barycentric_weights(const float2 &p,
                                const float2 &a,
                                const float2 &b,
                                const float2 &c,
                                float3 &r_w)
{
  const float2 v0 = b - a;
  const float2 v1 = c - a;
  const float2 v2 = p - a;
  const float d00 = math::dot(v0, v0);
  const float d01 = math::dot(v0, v1);
  const float d11 = math::dot(v1, v1);
  const float d20 = math::dot(v2, v0);
  const float d21 = math::dot(v2, v1);
  const float denom = d00 * d11 - d01 * d01;
  if (math::abs(denom) < 1e-20f) {
    return false;
  }
  const float v = (d11 * d20 - d01 * d21) / denom;
  const float w = (d00 * d21 - d01 * d20) / denom;
  const float u = 1.0f - v - w;
  r_w = float3(u, v, w);
  constexpr float eps = -1e-3f;
  return u >= eps && v >= eps && w >= eps;
}

/** Perspective-correct UV from projected triangle (clip_w = camera depth at corners). */
static bool triangle_uv(const float2 &p,
                        const float2 &s0,
                        const float2 &s1,
                        const float2 &s2,
                        const float w0,
                        const float w1,
                        const float w2,
                        const float2 &uv0,
                        const float2 &uv1,
                        const float2 &uv2,
                        float2 &r_uv)
{
  float3 bw;
  if (!barycentric_weights(p, s0, s1, s2, bw)) {
    return false;
  }
  const float inv0 = 1.0f / math::max(w0, 1e-8f);
  const float inv1 = 1.0f / math::max(w1, 1e-8f);
  const float inv2 = 1.0f / math::max(w2, 1e-8f);
  const float inv_w = bw.x * inv0 + bw.y * inv1 + bw.z * inv2;
  if (inv_w <= 1e-12f) {
    return false;
  }
  r_uv = (bw.x * uv0 * inv0 + bw.y * uv1 * inv1 + bw.z * uv2 * inv2) / inv_w;
  return true;
}

/**
 * COM Color is premultiplied scene-linear. Sample stamp at UV in [0,1]².
 *
 * Stamp inputs use skip_type_conversion so Float masks stay Float (not converted to
 * Color(v,v,v,1) which paints opaque rectangles). Float → white mask with alpha = value.
 * Color → full RGBA (not alpha-only).
 */
static Color sample_result(const Result &stamp, const float2 uv)
{
  if (!stamp.is_allocated()) {
    return Color(0.0f, 0.0f, 0.0f, 0.0f);
  }

  auto sample_texel = [&](const int2 texel) -> Color {
    switch (stamp.type()) {
      case ResultType::Color: {
        /* Full premultiplied color — keep RGB, do not collapse to alpha-only white. */
        return stamp.load_pixel<Color, true>(texel);
      }
      case ResultType::Float: {
        /* Mask (Ellipse etc.): premultiplied white, alpha = mask. */
        const float a = math::clamp(stamp.load_pixel<float, true>(texel), 0.0f, 1.0f);
        return Color(a, a, a, a);
      }
      case ResultType::Float3: {
        const float3 v = stamp.load_pixel<float3, true>(texel);
        return Color(v.x, v.y, v.z, 1.0f);
      }
      case ResultType::Float4: {
        const float4 v = stamp.load_pixel<float4, true>(texel);
        /* Treat as premultiplied if RGB already ≤ A; else premultiply straight RGBA. */
        const float a = math::clamp(v.w, 0.0f, 1.0f);
        if (a <= 0.0f) {
          return Color(0.0f, 0.0f, 0.0f, 0.0f);
        }
        if (v.x <= a + 1e-4f && v.y <= a + 1e-4f && v.z <= a + 1e-4f) {
          return Color(v.x, v.y, v.z, a);
        }
        return Color(v.x * a, v.y * a, v.z * a, a);
      }
      default:
        return Color(0.0f, 0.0f, 0.0f, 0.0f);
    }
  };

  if (stamp.is_single_value()) {
    return sample_texel(int2(0));
  }
  const int2 size = stamp.domain().data_size;
  if (size.x <= 0 || size.y <= 0) {
    return Color(0.0f, 0.0f, 0.0f, 0.0f);
  }
  /* Bilinear sample in UV space for smoother stamps. */
  const float u = math::clamp(uv.x, 0.0f, 1.0f) * float(size.x) - 0.5f;
  const float v = math::clamp(uv.y, 0.0f, 1.0f) * float(size.y) - 0.5f;
  const int x0 = math::clamp(int(math::floor(u)), 0, size.x - 1);
  const int y0 = math::clamp(int(math::floor(v)), 0, size.y - 1);
  const int x1 = math::min(x0 + 1, size.x - 1);
  const int y1 = math::min(y0 + 1, size.y - 1);
  const float fx = u - float(x0);
  const float fy = v - float(y0);

  const Color c00 = sample_texel(int2(x0, y0));
  const Color c10 = sample_texel(int2(x1, y0));
  const Color c01 = sample_texel(int2(x0, y1));
  const Color c11 = sample_texel(int2(x1, y1));
  auto to_f4 = [](const Color &c) { return float4(c.r, c.g, c.b, c.a); };
  auto from_f4 = [](const float4 &v) { return Color(v.x, v.y, v.z, v.w); };
  const float4 c0 = math::interpolate(to_f4(c00), to_f4(c10), fx);
  const float4 c1 = math::interpolate(to_f4(c01), to_f4(c11), fx);
  return from_f4(math::interpolate(c0, c1, fy));
}

/** Premultiplied-alpha Porter-Duff "over" (COM Color is premultiplied). */
static void over_blend(Color &dst, const Color &src)
{
  if (src.a <= 0.0f) {
    return;
  }
  const float inv_sa = 1.0f - math::clamp(src.a, 0.0f, 1.0f);
  dst = Color(src.r + dst.r * inv_sa,
              src.g + dst.g * inv_sa,
              src.b + dst.b * inv_sa,
              src.a + dst.a * inv_sa);
}

/* -------------------------------------------------------------------- */
/** \name Tile rasterization (point stamps)
 *
 * Far→near ordered quads are binned into screen tiles; tiles run in parallel so many stamps
 * no longer serialize a single CPU loop. Within a tile, depth order is preserved for correct
 * Porter-Duff over.
 * \{ */

static constexpr int stamp_tile_size = 64;

/** Pre-projected stamp quad ready for tile rasterization. */
struct StampPrim {
  float2 screen[4];
  float depth[4];
  float inv_depth[4];
  float mean_depth = 0.0f;
  int x0 = 0, x1 = -1, y0 = 0, y1 = -1;
  const Result *stamp = nullptr;
  /** One solid Color per needed attribute output (same order as attr_outputs_needed). */
  Vector<Color> attr_colors;
};

/**
 * Edge-function barycentrics (cheaper than full barycentric_weights each pixel).
 * Returns false if p is outside the triangle (with small epsilon).
 */
static bool triangle_bary_edges(const float2 &p,
                                const float2 &a,
                                const float2 &b,
                                const float2 &c,
                                float3 &r_w)
{
  const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  if (math::abs(area) < 1e-20f) {
    return false;
  }
  const float inv_area = 1.0f / area;
  const float w0 = ((b.x - p.x) * (c.y - p.y) - (b.y - p.y) * (c.x - p.x)) * inv_area;
  const float w1 = ((c.x - p.x) * (a.y - p.y) - (c.y - p.y) * (a.x - p.x)) * inv_area;
  const float w2 = 1.0f - w0 - w1;
  constexpr float eps = -1e-3f;
  if (w0 < eps || w1 < eps || w2 < eps) {
    return false;
  }
  r_w = float3(w0, w1, w2);
  return true;
}

static bool triangle_uv_fast(const float2 &p,
                             const float2 &s0,
                             const float2 &s1,
                             const float2 &s2,
                             const float inv0,
                             const float inv1,
                             const float inv2,
                             const float2 &uv0,
                             const float2 &uv1,
                             const float2 &uv2,
                             float2 &r_uv)
{
  float3 bw;
  if (!triangle_bary_edges(p, s0, s1, s2, bw)) {
    return false;
  }
  const float inv_w = bw.x * inv0 + bw.y * inv1 + bw.z * inv2;
  if (inv_w <= 1e-12f) {
    return false;
  }
  r_uv = (bw.x * uv0 * inv0 + bw.y * uv1 * inv1 + bw.z * uv2 * inv2) / inv_w;
  return true;
}

/** Nearest-neighbor sample (much cheaper than bilinear for dense point clouds). */
static Color sample_result_nearest(const Result &stamp, const float2 uv)
{
  if (!stamp.is_allocated()) {
    return Color(0.0f, 0.0f, 0.0f, 0.0f);
  }
  if (stamp.is_single_value()) {
    switch (stamp.type()) {
      case ResultType::Color:
        return stamp.load_pixel<Color, true>(int2(0));
      case ResultType::Float: {
        const float a = math::clamp(stamp.load_pixel<float, true>(int2(0)), 0.0f, 1.0f);
        return Color(a, a, a, a);
      }
      case ResultType::Float3: {
        const float3 v = stamp.load_pixel<float3, true>(int2(0));
        return Color(v.x, v.y, v.z, 1.0f);
      }
      case ResultType::Float4: {
        const float4 v = stamp.load_pixel<float4, true>(int2(0));
        const float a = math::clamp(v.w, 0.0f, 1.0f);
        if (a <= 0.0f) {
          return Color(0.0f, 0.0f, 0.0f, 0.0f);
        }
        if (v.x <= a + 1e-4f && v.y <= a + 1e-4f && v.z <= a + 1e-4f) {
          return Color(v.x, v.y, v.z, a);
        }
        return Color(v.x * a, v.y * a, v.z * a, a);
      }
      default:
        return Color(0.0f, 0.0f, 0.0f, 0.0f);
    }
  }
  const int2 size = stamp.domain().data_size;
  if (size.x <= 0 || size.y <= 0) {
    return Color(0.0f, 0.0f, 0.0f, 0.0f);
  }
  const int x = math::clamp(int(math::floor(math::clamp(uv.x, 0.0f, 1.0f) * float(size.x))),
                            0,
                            size.x - 1);
  const int y = math::clamp(int(math::floor(math::clamp(uv.y, 0.0f, 1.0f) * float(size.y))),
                            0,
                            size.y - 1);
  const int2 texel(x, y);
  switch (stamp.type()) {
    case ResultType::Color:
      return stamp.load_pixel<Color, true>(texel);
    case ResultType::Float: {
      const float a = math::clamp(stamp.load_pixel<float, true>(texel), 0.0f, 1.0f);
      return Color(a, a, a, a);
    }
    case ResultType::Float3: {
      const float3 v = stamp.load_pixel<float3, true>(texel);
      return Color(v.x, v.y, v.z, 1.0f);
    }
    case ResultType::Float4: {
      const float4 v = stamp.load_pixel<float4, true>(texel);
      const float a = math::clamp(v.w, 0.0f, 1.0f);
      if (a <= 0.0f) {
        return Color(0.0f, 0.0f, 0.0f, 0.0f);
      }
      if (v.x <= a + 1e-4f && v.y <= a + 1e-4f && v.z <= a + 1e-4f) {
        return Color(v.x, v.y, v.z, a);
      }
      return Color(v.x * a, v.y * a, v.z * a, a);
    }
    default:
      return Color(0.0f, 0.0f, 0.0f, 0.0f);
  }
}

static void paint_stamp_in_rect(const StampPrim &prim,
                                const int x0,
                                const int x1,
                                const int y0,
                                const int y1,
                                const int res_x,
                                const bool need_color,
                                MutableSpan<Color> color_pixels,
                                MutableSpan<Array<Color>> attr_pixels,
                                const Span<std::string> attr_outputs_needed)
{
  static const float2 tex_uv[4] = {
      float2(0.0f, 0.0f), float2(1.0f, 0.0f), float2(1.0f, 1.0f), float2(0.0f, 1.0f)};

  for (int y = y0; y <= y1; y++) {
    for (int x = x0; x <= x1; x++) {
      const float2 p(float(x) + 0.5f, float(y) + 0.5f);
      float2 uv;
      if (!triangle_uv_fast(p,
                            prim.screen[0],
                            prim.screen[1],
                            prim.screen[2],
                            prim.inv_depth[0],
                            prim.inv_depth[1],
                            prim.inv_depth[2],
                            tex_uv[0],
                            tex_uv[1],
                            tex_uv[2],
                            uv) &&
          !triangle_uv_fast(p,
                            prim.screen[0],
                            prim.screen[2],
                            prim.screen[3],
                            prim.inv_depth[0],
                            prim.inv_depth[2],
                            prim.inv_depth[3],
                            tex_uv[0],
                            tex_uv[2],
                            tex_uv[3],
                            uv))
      {
        continue;
      }
      if (uv.x < 0.0f || uv.x > 1.0f || uv.y < 0.0f || uv.y > 1.0f) {
        continue;
      }

      float cover = 1.0f;
      Color stamp_sample(0.0f, 0.0f, 0.0f, 0.0f);
      if (prim.stamp) {
        stamp_sample = sample_result_nearest(*prim.stamp, uv);
        cover = stamp_sample.a;
        if (cover <= 0.0f) {
          continue;
        }
      }
      else if (attr_outputs_needed.is_empty()) {
        continue;
      }

      const int64_t pix = int64_t(y) * res_x + x;
      if (need_color && prim.stamp && stamp_sample.a > 0.0f) {
        over_blend(color_pixels[pix], stamp_sample);
      }

      for (const int out_i : attr_outputs_needed.index_range()) {
        if (out_i >= prim.attr_colors.size()) {
          break;
        }
        const std::string &aname = attr_outputs_needed[out_i];
        if (aname == attr_alpha) {
          Color &dst = attr_pixels[out_i][pix];
          const float m = math::max(dst.r, cover);
          dst = Color(m, m, m, m);
          continue;
        }
        const Color src = prim.attr_colors[out_i];
        const float a = cover * math::clamp(src.a, 0.0f, 1.0f);
        if (a <= 0.0f) {
          continue;
        }
        over_blend(attr_pixels[out_i][pix], Color(src.r * a, src.g * a, src.b * a, a));
      }
    }
  }
}

/** \} */

/** Own a CPU copy of a stamp so load_pixel works after GPU inputs are released. */
static Result stamp_to_cpu(Context &context, Result &stamp)
{
  if (stamp.is_single_value()) {
    /* Keep as single; caller samples via load_pixel with CouldBeSingleValue. */
    Result copy = context.create_result(stamp.type());
    copy.allocate_single_value();
    switch (stamp.type()) {
      case ResultType::Color:
        copy.set_single_value(stamp.get_single_value_default<Color>());
        break;
      case ResultType::Float:
        copy.set_single_value(stamp.get_single_value_default<float>());
        break;
      case ResultType::Float3:
        copy.set_single_value(stamp.get_single_value_default<float3>());
        break;
      case ResultType::Float4:
        copy.set_single_value(stamp.get_single_value_default<float4>());
        break;
      default:
        break;
    }
    return copy;
  }
  if (context.use_gpu()) {
    return stamp.download_to_cpu();
  }
  /* Already on CPU — share underlying buffer into a result we own a reference on. */
  Result copy = context.create_result(stamp.type());
  copy.share_data(stamp);
  return copy;
}

class PointStampOperation : public NodeOperation {
 public:
  PointStampOperation(Context &context, const bNode &node) : NodeOperation(context, node)
  {
    /* Points is a String cache key — never convert Color/Float into it (avoids convert shader
     * crash when a wrong type is still linked). Descriptor is always declared for Points. */
    InputDescriptor &points_desc = this->get_input_descriptor("Points");
    points_desc.skip_type_conversion = true;
    points_desc.expects_single_value = true;
    points_desc.realization_mode = InputRealizationMode::None;

    /* Multi-input stamps are pre-declared Stamp / Stamp__1..__31; sample in local UV, do not
     * realize onto another stamp's domain (reconnect safety + correct unit-square mapping).
     *
     * Critical: skip Color type conversion. Without this, Float masks (Ellipse) are converted to
     * Color(v,v,v,1) with opaque alpha → solid rectangular stamps. Color images stay Color so RGB
     * is preserved (not collapsed to alpha-only white). */
    for (const int i : IndexRange(32)) {
      const std::string id = (i == 0) ? std::string("Stamp") : ("Stamp__" + std::to_string(i));
      InputDescriptor &desc = this->get_input_descriptor(id);
      desc.realization_mode = InputRealizationMode::None;
      desc.skip_type_conversion = true;
    }
  }

  Domain compute_domain() override
  {
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    Result &color_out = this->get_result("Color");
    Result *bundle_out = nullptr;
    for (const bNodeSocket *socket : this->node().output_sockets()) {
      if (socket->type == SOCK_BUNDLE && socket->is_available()) {
        bundle_out = &this->get_result(socket->identifier);
        break;
      }
    }

    /* Resolve points geometry from cache key on the Points input. */
    const bke::GeometrySet *geometry = nullptr;
    bke::GeometrySet empty;
    bool points_input_ok = false;
    bool points_type_error = false;
    if (this->has_input("Points")) {
      Result &points_in = this->get_input("Points");
      /* Geometry sockets map to String cache keys. Wrong type (e.g. Color miswired) or
       * unlinked defaults that are not a real string must not crash evaluation. */
      if (points_in.type() != ResultType::String ||
          !(points_in.is_single_value() || points_in.is_allocated()))
      {
        points_type_error = true;
        this->add_warning(
            nodes::NodeWarningType::Error,
            "Points expects geometry from Import Points (wrong socket type connected). "
            "Evaluation stopped for this node");
      }
      else if (!points_in.is_single_value()) {
        /* Allocated non-single (should not happen for Geometry keys) — treat as missing. */
        points_type_error = true;
        this->add_warning(nodes::NodeWarningType::Error,
                          "Points input is not a geometry cache key from Import Points");
      }
      else {
        const std::string key = points_in.get_single_value_default<std::string>();
        if (PointsGeometryCache *cache = active_cache()) {
          geometry = cache->lookup(key);
        }
        points_input_ok = true;
        if (!geometry && !key.empty() && key != points_result_type_name) {
          this->add_warning(nodes::NodeWarningType::Warning,
                            "Points data missing — check Import Points evaluation");
        }
      }
    }
    else {
      this->add_warning(nodes::NodeWarningType::Error, "Points input is not connected");
    }
    if (!geometry) {
      geometry = &empty;
    }

    /* Multi-input stamps: map only real links (Stamp, Stamp__1, ...). Pre-declared slots beyond
     * the link count are unlinked placeholders — do not treat them as stamps (would break id %).
     * Always has_input before get_input so reconnect cannot crash. */
    Vector<Result *> stamps;
    {
      const bNodeSocket *stamp_sock = nullptr;
      for (const bNodeSocket *input : this->node().input_sockets()) {
        if (input->type == SOCK_RGBA && input->is_multi_input()) {
          stamp_sock = input;
          break;
        }
      }
      int link_count = 0;
      if (stamp_sock) {
        for (const bNodeLink *link : stamp_sock->directly_linked_links()) {
          if (link && link->is_used() && !link->is_muted() && link->fromsock) {
            link_count++;
          }
        }
      }
      const int count = math::clamp(link_count, 0, 32);
      for (const int i : IndexRange(count)) {
        const std::string id = (i == 0) ? std::string("Stamp") : ("Stamp__" + std::to_string(i));
        if (!this->has_input(id)) {
          continue;
        }
        Result &stamp = this->get_input(id);
        if (!stamp.is_allocated()) {
          continue;
        }
        stamps.append(&stamp);
      }
    }

    const Domain domain = this->context().get_compositing_domain();
    const int2 res = math::max(domain.data_size, int2(1));
    const float cam_dist = stamp_camera_distance;

    const Vector<std::string> attr_names = attribute_names_from_geometry(
        (points_input_ok && !points_type_error) ? geometry : nullptr);

    Vector<PointStampData> points = (points_input_ok && !points_type_error) ?
                                        gather_points(*geometry, attr_names) :
                                        Vector<PointStampData>{};
    /* Far → near by point Z (higher Z is closer to camera at +Z). */
    std::sort(points.begin(), points.end(), [](const PointStampData &a, const PointStampData &b) {
      return a.position.z < b.position.z;
    });

    Vector<Result> cpu_stamps;
    Vector<const Result *> stamp_ptrs;
    cpu_stamps.reserve(stamps.size());
    if (!points_type_error) {
      for (Result *stamp : stamps) {
        if (!stamp || !stamp->is_allocated()) {
          continue;
        }
        if (!ELEM(stamp->type(),
                  ResultType::Color,
                  ResultType::Float,
                  ResultType::Float3,
                  ResultType::Float4))
        {
          continue;
        }
        cpu_stamps.append(stamp_to_cpu(this->context(), *stamp));
      }
      stamp_ptrs.reserve(cpu_stamps.size());
      for (Result &r : cpu_stamps) {
        stamp_ptrs.append(&r);
      }
    }

    /* Manual Bake Image needs full Color + attr maps even when COM usage tracking misses
     * Bundle (it is only a String cache key). */
    const bool bake_write = compositor::image_process_bake_write_enabled();
    const bool need_color = color_out.should_compute() || bake_write;
    /* Only cook attribute maps that are actually needed (linked/hidden socket should_compute, or
     * Bundle is requested / Bake is running). */
    const bool need_bundle = (bundle_out && bundle_out->should_compute()) || bake_write;
    Vector<std::string> attr_outputs_needed;
    attr_outputs_needed.reserve(attr_names.size());
    for (const std::string &name : attr_names) {
      bool needed = need_bundle;
      if (!needed && this->has_result(name)) {
        needed = this->get_result(name).should_compute();
      }
      if (needed) {
        attr_outputs_needed.append(name);
      }
    }

    Array<Color> color_pixels;
    Vector<Array<Color>> attr_pixels;
    if (need_color) {
      color_pixels.reinitialize(int64_t(res.x) * res.y);
      color_pixels.fill(Color(0.0f, 0.0f, 0.0f, 0.0f));
    }
    attr_pixels.reinitialize(attr_outputs_needed.size());
    for (Array<Color> &buf : attr_pixels) {
      buf.reinitialize(int64_t(res.x) * res.y);
      buf.fill(Color(0.0f, 0.0f, 0.0f, 0.0f));
    }

    Map<std::string, int> attr_name_to_src;
    for (const int i : attr_names.index_range()) {
      attr_name_to_src.add(attr_names[i], i);
    }

    /* Tile-rasterize projected stamp quads in parallel (not one serial CPU loop per point). */
    const bool do_paint = !points.is_empty() &&
                          (need_color || !attr_outputs_needed.is_empty()) &&
                          (!stamp_ptrs.is_empty() || !attr_outputs_needed.is_empty());
    if (do_paint) {
      /* 1) Project all points → primitives (parallel). */
      Array<StampPrim> prims_raw(points.size());
      Array<char> prim_valid(points.size(), 0);
      threading::parallel_for(points.index_range(), 256, [&](const IndexRange range) {
        for (const int i : range) {
          const PointStampData &pt = points[i];
          const float3 unit_scale = size_to_unit_scale(pt.size, res);
          const float2 local_uv[4] = {float2(-0.5f, -0.5f),
                                      float2(0.5f, -0.5f),
                                      float2(0.5f, 0.5f),
                                      float2(-0.5f, 0.5f)};

          StampPrim prim;
          bool all_ok = true;
          for (const int c : IndexRange(4)) {
            const float3 world = plane_world_point(pt, unit_scale, local_uv[c].x, local_uv[c].y);
            if (!project_world_to_pixel(world, res, cam_dist, prim.screen[c], prim.depth[c])) {
              all_ok = false;
              break;
            }
            prim.inv_depth[c] = 1.0f / math::max(prim.depth[c], 1e-8f);
          }
          if (!all_ok) {
            continue;
          }

          float min_x = prim.screen[0].x, max_x = prim.screen[0].x;
          float min_y = prim.screen[0].y, max_y = prim.screen[0].y;
          float depth_sum = 0.0f;
          for (const int c : IndexRange(4)) {
            min_x = math::min(min_x, prim.screen[c].x);
            max_x = math::max(max_x, prim.screen[c].x);
            min_y = math::min(min_y, prim.screen[c].y);
            max_y = math::max(max_y, prim.screen[c].y);
            depth_sum += prim.depth[c];
          }
          prim.mean_depth = depth_sum * 0.25f;
          prim.x0 = math::clamp(int(std::floor(min_x)), 0, res.x - 1);
          prim.x1 = math::clamp(int(std::ceil(max_x)), 0, res.x - 1);
          prim.y0 = math::clamp(int(std::floor(min_y)), 0, res.y - 1);
          prim.y1 = math::clamp(int(std::ceil(max_y)), 0, res.y - 1);
          if (prim.x0 > prim.x1 || prim.y0 > prim.y1) {
            continue;
          }

          if (!stamp_ptrs.is_empty()) {
            const int stamp_i = math::mod_periodic(pt.id, int(stamp_ptrs.size()));
            if (stamp_i >= 0 && stamp_i < stamp_ptrs.size()) {
              prim.stamp = stamp_ptrs[stamp_i];
            }
          }

          /* Pack only needed attribute colors (avoids per-pixel map lookup). */
          prim.attr_colors.reinitialize(attr_outputs_needed.size());
          for (const int out_i : attr_outputs_needed.index_range()) {
            const std::string &aname = attr_outputs_needed[out_i];
            if (aname == attr_alpha) {
              prim.attr_colors[out_i] = Color(1.0f, 1.0f, 1.0f, 1.0f);
              continue;
            }
            const int src_i = attr_name_to_src.lookup_default(aname, -1);
            if (src_i >= 0 && src_i < pt.attr_colors.size()) {
              prim.attr_colors[out_i] = pt.attr_colors[src_i];
            }
            else {
              prim.attr_colors[out_i] = Color(0.0f, 0.0f, 0.0f, 0.0f);
            }
          }

          prims_raw[i] = std::move(prim);
          prim_valid[i] = 1;
        }
      });

      Vector<StampPrim> prims;
      prims.reserve(points.size());
      for (const int i : points.index_range()) {
        if (prim_valid[i]) {
          prims.append(std::move(prims_raw[i]));
        }
      }

      /* Far → near (larger mean depth = farther under our camera convention? depth=denom =
       * cam_dist - z; smaller depth = closer). Painter's algorithm: far first. */
      std::sort(prims.begin(), prims.end(), [](const StampPrim &a, const StampPrim &b) {
        return a.mean_depth > b.mean_depth;
      });

      /* 2) Bin primitives into screen tiles. */
      const int tiles_x = (res.x + stamp_tile_size - 1) / stamp_tile_size;
      const int tiles_y = (res.y + stamp_tile_size - 1) / stamp_tile_size;
      const int tile_count = tiles_x * tiles_y;
      Array<Vector<int>> tile_prims(tile_count);
      for (const int pi : prims.index_range()) {
        const StampPrim &prim = prims[pi];
        const int tx0 = math::clamp(prim.x0 / stamp_tile_size, 0, tiles_x - 1);
        const int tx1 = math::clamp(prim.x1 / stamp_tile_size, 0, tiles_x - 1);
        const int ty0 = math::clamp(prim.y0 / stamp_tile_size, 0, tiles_y - 1);
        const int ty1 = math::clamp(prim.y1 / stamp_tile_size, 0, tiles_y - 1);
        for (int ty = ty0; ty <= ty1; ty++) {
          for (int tx = tx0; tx <= tx1; tx++) {
            tile_prims[ty * tiles_x + tx].append(pi);
          }
        }
      }

      /* 3) Rasterize tiles in parallel — each tile keeps far→near order for correct over. */
      threading::parallel_for(IndexRange(tile_count), 1, [&](const IndexRange range) {
        for (const int ti : range) {
          const Vector<int> &list = tile_prims[ti];
          if (list.is_empty()) {
            continue;
          }
          const int tx = ti % tiles_x;
          const int ty = ti / tiles_x;
          const int tile_x0 = tx * stamp_tile_size;
          const int tile_y0 = ty * stamp_tile_size;
          const int tile_x1 = math::min(tile_x0 + stamp_tile_size - 1, res.x - 1);
          const int tile_y1 = math::min(tile_y0 + stamp_tile_size - 1, res.y - 1);

          for (const int pi : list) {
            const StampPrim &prim = prims[pi];
            const int x0 = math::max(prim.x0, tile_x0);
            const int x1 = math::min(prim.x1, tile_x1);
            const int y0 = math::max(prim.y0, tile_y0);
            const int y1 = math::min(prim.y1, tile_y1);
            if (x0 > x1 || y0 > y1) {
              continue;
            }
            paint_stamp_in_rect(prim,
                                x0,
                                x1,
                                y0,
                                y1,
                                res.x,
                                need_color,
                                color_pixels,
                                attr_pixels,
                                attr_outputs_needed);
          }
        }
      });
    }

    auto upload_color_buffer = [&](Result &output, const Span<Color> pixels, const bool force) {
      if (!force && !output.should_compute()) {
        return;
      }
      Result cpu = this->context().create_result(ResultType::Color);
      cpu.allocate_texture(Domain(res), false, ResultStorageType::CPUImage);
      parallel_for(res, [&](const int2 texel) {
        cpu.store_pixel(texel, pixels[int64_t(texel.y) * res.x + texel.x]);
      });
      if (this->context().use_gpu()) {
        Result gpu = cpu.upload_to_gpu(true);
        output.share_data(gpu);
        gpu.release();
      }
      else {
        output.share_data(cpu);
      }
      cpu.release();
    };

    /** Upload attr map using the declared socket type (float/vec/color), not always Color. */
    auto upload_typed_attr = [&](Result &output, const Span<Color> pixels) {
      ResultType out_type = output.type();
      /* Texture-only types; fall back to Color if a non-image type slipped through. */
      if (!ELEM(out_type,
                ResultType::Float,
                ResultType::Float2,
                ResultType::Float3,
                ResultType::Float4,
                ResultType::Int,
                ResultType::Color))
      {
        out_type = ResultType::Color;
      }
      Result cpu = this->context().create_result(out_type);
      cpu.allocate_texture(Domain(res), false, ResultStorageType::CPUImage);
      parallel_for(res, [&](const int2 texel) {
        const Color c = pixels[int64_t(texel.y) * res.x + texel.x];
        const float inv_a = (c.a > 1e-8f) ? (1.0f / c.a) : 0.0f;
        switch (out_type) {
          case ResultType::Float:
            cpu.store_pixel(texel, c.r * inv_a);
            break;
          case ResultType::Float2:
            cpu.store_pixel(texel, float2(c.r, c.g) * inv_a);
            break;
          case ResultType::Float3:
            cpu.store_pixel(texel, float3(c.r, c.g, c.b) * inv_a);
            break;
          case ResultType::Float4:
            cpu.store_pixel(texel, float4(float3(c.r, c.g, c.b) * inv_a, c.a));
            break;
          case ResultType::Int:
            cpu.store_pixel(texel, int(c.r * inv_a));
            break;
          case ResultType::Color:
          default:
            cpu.store_pixel(texel, c);
            break;
        }
      });
      if (this->context().use_gpu()) {
        Result gpu = cpu.upload_to_gpu(true);
        output.share_data(gpu);
        gpu.release();
      }
      else {
        output.share_data(cpu);
      }
      cpu.release();
    };

    if (need_color) {
      upload_color_buffer(color_out, color_pixels, false);
    }
    /* Only upload to sockets that exist on this operation. Attribute names from the cooked
     * geometry can lag behind declare (new Store Named Attribute / radius) — get_result used
     * to assert/crash when the socket was missing. Maps still go into stamp_attr_cache. */
    for (const int i : attr_outputs_needed.index_range()) {
      if (!this->has_result(attr_outputs_needed[i])) {
        continue;
      }
      Result &out = this->get_result(attr_outputs_needed[i]);
      upload_typed_attr(out, attr_pixels[i]);
    }

    /* Publish attr maps for Separate Bundle / Bake Image COM (Bundle is only a String key).
     * Always publish when cache is active so Bake can resolve maps by node key. */
    const std::string stamp_key = stamp_node_cache_key(this->node());
    if (StampAttrMapsCache *attr_cache = active_stamp_attr_cache()) {
      StampAttrMaps maps;
      maps.size = res;
      for (const int i : attr_outputs_needed.index_range()) {
        Array<float4> rgba(int64_t(res.x) * res.y);
        const Span<Color> src = attr_pixels[i];
        for (const int64_t p : src.index_range()) {
          rgba[p] = float4(src[p].r, src[p].g, src[p].b, src[p].a);
        }
        maps.maps.add_overwrite(attr_outputs_needed[i], std::move(rgba));
      }
      for (const std::string &name : attr_names) {
        if (!maps.maps.contains(name)) {
          Array<float4> empty(int64_t(res.x) * res.y, float4(0.0f));
          maps.maps.add_overwrite(name, std::move(empty));
        }
      }
      /* Stash stamped Color under "Color" so Bake Image Bundle can write Color.<ext>. */
      if (!color_pixels.is_empty()) {
        Array<float4> rgba(int64_t(res.x) * res.y);
        for (const int64_t p : color_pixels.index_range()) {
          rgba[p] = float4(
              color_pixels[p].r, color_pixels[p].g, color_pixels[p].b, color_pixels[p].a);
        }
        maps.maps.add_overwrite("Color", std::move(rgba));
      }
      /* Manual bake must always leave a non-empty cache entry so Bake can write files. */
      if (bake_write && maps.maps.is_empty()) {
        Array<float4> empty(int64_t(res.x) * res.y, float4(0.0f));
        maps.maps.add_overwrite("Color", std::move(empty));
      }
      attr_cache->store(stamp_key, std::move(maps));
    }

    for (Result &r : cpu_stamps) {
      r.release();
    }

    /* Always emit the Bundle key when requested or when Bake is writing (usage can miss Bundle). */
    if (bundle_out && (bundle_out->should_compute() || bake_write)) {
      bundle_out->allocate_single_value();
      if (bundle_out->type() == ResultType::String) {
        bundle_out->set_single_value(stamp_key);
      }
    }

    /* Bundle UI sync is handled in node_update on graph edits — never tag/redeclare from COM
     * cook (feedback loop with ntreeImageNodesEvaluate). */
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new PointStampOperation(context, node);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Registration
 * \{ */

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodePointStamp"_ustr);
  ntype.ui_name = "Point Stamp";
  ntype.ui_description =
      "Instance stamp textures as 3D UV planes on points (rotation, size.xy, id). Perspective "
      "view with Z-sorted occlusion via tiled CPU rasterization. Bundle auto-syncs Separate "
      "Bundle / Get Bundle Item maps from point attributes. Wrong Points type stops evaluation";
  ntype.nclass = NODE_CLASS_OP_FILTER;
  ntype.declare = node_declare;
  ntype.updatefunc = node_update;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

/** \} */

}  // namespace blender::nodes::node_image_point_stamp_cc
