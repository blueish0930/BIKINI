/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_curves.h"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.hh"
#include "BKE_pointcloud.hh"

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_map.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_mutex.hh"
#include "BLI_span.hh"

#include "BLO_read_write.hh"

#include "BLT_translation_any_thread.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_ID.h"
#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_node_types.h"
#include "DNA_pointcloud_types.h"

#include "NOD_geo_edit_elements.hh"
#include "NOD_geo_elements_component.hh"
#include "NOD_rna_define.hh"

#include "RNA_access.hh"
#include "RNA_enum_types.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "MEM_guardedalloc.h"

#include "node_geometry_util.hh"

#include <cstring>
#include <fmt/format.h>

namespace blender {

namespace nodes::node_geo_edit_elements_cc {

NODE_STORAGE_FUNCS(NodeGeometryEditElements)

struct CacheKey {
  uint session_uid;
  int32_t node_id;

  uint64_t hash() const
  {
    return get_default_hash(session_uid, node_id);
  }
  friend bool operator==(const CacheKey &a, const CacheKey &b) = default;
};

static Mutex &cache_mutex()
{
  static Mutex mutex;
  return mutex;
}

static Map<CacheKey, bke::GeometrySet> &geometry_cache()
{
  static Map<CacheKey, bke::GeometrySet> map;
  return map;
}

static CacheKey make_key(const bNodeTree &tree, const bNode &node)
{
  const bNodeTree &tree_orig = *DEG_get_original(&tree);
  return {tree_orig.id.session_uid, node.identifier};
}

static Mutex &force_eval_mutex()
{
  static Mutex mutex;
  return mutex;
}

static std::optional<EditElementsForceEvalRequest> &force_eval_request()
{
  static std::optional<EditElementsForceEvalRequest> request;
  return request;
}

/**
 * Own the data and drop GPU batch caches. The process-exit static Map destructor
 * can run after #GPU_exit; freeing batch caches then crashes (see crash after Select).
 */
static void prepare_geometry_for_cache(bke::GeometrySet &geometry)
{
  geometry.ensure_owns_direct_data();
  if (Mesh *mesh = geometry.get_mesh_for_write()) {
    BKE_mesh_runtime_clear_cache(mesh);
  }
}

static void free_legacy_positions(NodeGeometryEditElements &storage)
{
  if (storage.positions) {
    MEM_delete(storage.positions);
    storage.positions = nullptr;
  }
  storage.positions_num = 0;
}

}  // namespace nodes::node_geo_edit_elements_cc

namespace nodes {

void edit_elements_set_force_eval_request(const EditElementsForceEvalRequest &request)
{
  using namespace node_geo_edit_elements_cc;
  std::lock_guard lock{force_eval_mutex()};
  force_eval_request() = request;
}

void edit_elements_clear_force_eval_request()
{
  using namespace node_geo_edit_elements_cc;
  std::lock_guard lock{force_eval_mutex()};
  force_eval_request().reset();
}

std::optional<EditElementsForceEvalRequest> edit_elements_get_force_eval_request()
{
  using namespace node_geo_edit_elements_cc;
  std::lock_guard lock{force_eval_mutex()};
  return force_eval_request();
}

void edit_elements_cache_geometry(const bNodeTree &tree,
                                  const bNode &node,
                                  bke::GeometrySet geometry)
{
  using namespace node_geo_edit_elements_cc;
  prepare_geometry_for_cache(geometry);
  std::lock_guard lock{cache_mutex()};
  geometry_cache().add_overwrite(make_key(tree, node), std::move(geometry));
}

std::optional<bke::GeometrySet> edit_elements_copy_cached_geometry(const bNodeTree &tree,
                                                                   const bNode &node)
{
  using namespace node_geo_edit_elements_cc;
  std::lock_guard lock{cache_mutex()};
  if (const bke::GeometrySet *geometry = geometry_cache().lookup_ptr(make_key(tree, node))) {
    return *geometry;
  }
  return std::nullopt;
}

void edit_elements_clear_cache(const bNodeTree &tree, const bNode &node)
{
  using namespace node_geo_edit_elements_cc;
  std::lock_guard lock{cache_mutex()};
  geometry_cache().remove(make_key(tree, node));
}

void edit_elements_free_all_caches()
{
  using namespace node_geo_edit_elements_cc;
  std::lock_guard lock{cache_mutex()};
  geometry_cache().clear();
}

const ID *edit_elements_get_result(const bNode &node)
{
  if (node.id && ELEM(GS(node.id->name), ID_ME, ID_CV, ID_PT)) {
    return node.id;
  }
  return nullptr;
}

bke::GeometryComponent::Type edit_elements_result_component_type(const bNode &node)
{
  using namespace node_geo_edit_elements_cc;
  const ID *result = edit_elements_get_result(node);
  if (result == nullptr) {
    return bke::GeometryComponent::Type::Mesh;
  }
  switch (GS(result->name)) {
    case ID_CV:
      return bke::GeometryComponent::Type::Curve;
    case ID_PT:
      /* Points are either a point cloud or stand for instances. */
      return node_storage(node).result_component_type ==
                         int8_t(bke::GeometryComponent::Type::Instance) ?
                 bke::GeometryComponent::Type::Instance :
                 bke::GeometryComponent::Type::PointCloud;
    default:
      return bke::GeometryComponent::Type::Mesh;
  }
}

void edit_elements_set_result(bNode &node,
                              ID *result,
                              const bke::GeometryComponent::Type component_type)
{
  using namespace node_geo_edit_elements_cc;
  NodeGeometryEditElements &storage = node_storage(node);
  free_legacy_positions(storage);

  ID *old_id = node.id;
  node.id = nullptr;
  if (result) {
    node.id = result;
    id_us_plus(node.id);
    storage.result_component_type = int8_t(component_type);
    storage.component_type = int8_t(component_type);
  }
  else {
    storage.result_component_type = int8_t(bke::GeometryComponent::Type::Mesh);
  }
  if (old_id) {
    id_us_min(old_id);
  }
}

PointCloud *elements_instances_to_points(const bke::Instances &instances)
{
  PointCloud *points = BKE_pointcloud_new_nomain(PointCloudType::Points,
                                                 instances.instances_num());
  const Span<float4x4> transforms = instances.transforms();
  MutableSpan<float3> positions = points->positions_for_write();
  for (const int i : positions.index_range()) {
    positions[i] = transforms[i].location();
  }
  bke::SpanAttributeWriter<int> sources =
      points->attributes_for_write().lookup_or_add_for_write_only_span<int>(
          elements_instance_source_attribute, bke::AttrDomain::Point);
  array_utils::fill_index_range(sources.span);
  sources.finish();
  return points;
}

bke::Instances *elements_points_to_instances(const bke::Instances &src_instances,
                                             const PointCloud &points)
{
  const VArraySpan<int> sources = *points.attributes().lookup_or_default<int>(
      elements_instance_source_attribute, bke::AttrDomain::Point, -1);
  const Span<float3> positions = points.positions();

  /* The instances may have changed since the points were made from them. */
  const uint src_num = uint(src_instances.instances_num());
  Vector<int> src_indices;
  Vector<int> point_indices;
  for (const int i : sources.index_range()) {
    if (uint(sources[i]) < src_num) {
      src_indices.append(sources[i]);
      point_indices.append(i);
    }
  }

  bke::Instances *dst_instances = new bke::Instances();
  for (const bke::InstanceReference &reference : src_instances.references()) {
    dst_instances->add_new_reference(reference);
  }
  dst_instances->resize(int(src_indices.size()));
  bke::gather_attributes(src_instances.attributes(),
                         bke::AttrDomain::Instance,
                         bke::AttrDomain::Instance,
                         {},
                         src_indices.as_span(),
                         dst_instances->attributes_for_write());
  MutableSpan<float4x4> transforms = dst_instances->transforms_for_write();
  for (const int i : transforms.index_range()) {
    transforms[i].location() = positions[point_indices[i]];
  }
  dst_instances->tag_reference_handles_changed();
  dst_instances->remove_unused_references();
  return dst_instances;
}

void edit_elements_set_positions(bNode &node, const Span<float3> positions)
{
  using namespace node_geo_edit_elements_cc;
  /* Legacy path: clear full mesh result so positions apply on passthrough. */
  if (node.id) {
    id_us_min(node.id);
    node.id = nullptr;
  }
  NodeGeometryEditElements &storage = node_storage(node);
  free_legacy_positions(storage);
  storage.positions_num = int(positions.size());
  if (positions.is_empty()) {
    return;
  }
  storage.positions = reinterpret_cast<float(*)[3]>(
      MEM_new_array_uninitialized<float>(size_t(positions.size()) * 3, __func__));
  for (const int i : positions.index_range()) {
    const float3 &p = positions[i];
    storage.positions[i][0] = p.x;
    storage.positions[i][1] = p.y;
    storage.positions[i][2] = p.z;
  }
}

bool is_edit_elements_node(const bNode &node)
{
  return node.is_type("GeometryNodeEditElements"_ustr);
}

}  // namespace nodes

namespace nodes::node_geo_edit_elements_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  /* Required so draw_buttons (Reset / component) appear on the node body. */
  b.add_default_layout();

  b.add_input<decl::Geometry>("Geometry"_ustr)
      .supported_type({GeometryComponent::Type::Mesh,
                       GeometryComponent::Type::Curve,
                       GeometryComponent::Type::PointCloud,
                       GeometryComponent::Type::Instance})
      .description("Geometry whose mesh, curves, point cloud or instances Enter opens for editing")
      .is_default_link_socket();
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "The input geometry with the edited component replaced by the stored result, if any");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryEditElements *data = MEM_new<NodeGeometryEditElements>(__func__);
  data->positions_num = 0;
  data->positions = nullptr;
  data->component_type = int8_t(GeometryComponent::Type::Mesh);
  data->result_component_type = int8_t(GeometryComponent::Type::Mesh);
  node->storage = data;
  node->id = nullptr;
  /* Ensure the node body buttons (Reset, component, ...) are visible. */
  node->flag |= NODE_OPTIONS;
}

static void node_free_storage(bNode *node)
{
  NodeGeometryEditElements *storage = static_cast<NodeGeometryEditElements *>(node->storage);
  if (storage) {
    free_legacy_positions(*storage);
    MEM_delete(storage);
  }
  node->storage = nullptr;
  /* node->id (the edit result) is freed via standard node ID user handling. */
}

static void node_copy_storage(bNodeTree * /*tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryEditElements &src = node_storage(*src_node);
  NodeGeometryEditElements *dst = MEM_new<NodeGeometryEditElements>(__func__);
  dst->component_type = src.component_type;
  dst->result_component_type = src.result_component_type;
  dst->positions_num = src.positions_num;
  if (src.positions_num > 0 && src.positions != nullptr) {
    dst->positions = reinterpret_cast<float(*)[3]>(
        MEM_new_array_uninitialized<float>(size_t(src.positions_num) * 3, __func__));
    memcpy(dst->positions, src.positions, sizeof(float[3]) * size_t(src.positions_num));
  }
  else {
    dst->positions = nullptr;
  }
  dst_node->storage = dst;
  /* node->id is duplicated by the generic node-copy path. */
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  const NodeGeometryEditElements &storage = node_storage(node);
  if (storage.positions_num > 0 && storage.positions != nullptr) {
    writer.write_float3_array(storage.positions_num,
                              reinterpret_cast<const float *>(storage.positions));
  }
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  NodeGeometryEditElements &storage = node_storage(node);
  if (storage.positions_num > 0) {
    if (!BLO_read_array(&reader, &storage.positions, storage.positions_num)) {
      storage.positions = nullptr;
      storage.positions_num = 0;
    }
  }
  else {
    storage.positions = nullptr;
  }
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  const bNode *node = static_cast<const bNode *>(ptr->data);

  /* Node body only: Reset + Edit (no description text; not N-panel). */
  {
    ui::Layout &row = layout.row(true);
    row.scale_y_set(1.2f);
    PointerRNA reset_ptr = row.op("node.edit_elements_reset", IFACE_("Reset"), ICON_FILE_REFRESH);
    if (node) {
      RNA_int_set(&reset_ptr, "node_id", node->identifier);
    }
    row.op("node.edit_elements_edit", IFACE_("Edit"), ICON_EDITMODE_HLT);
  }

  /* A stored result is of one component, it has to be reset before another one is edited. */
  ui::Layout &col = layout.column(false);
  col.enabled_set(node == nullptr || edit_elements_get_result(*node) == nullptr);
  col.prop(ptr, "component_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_layout_ex(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  const bNode *node = static_cast<const bNode *>(ptr->data);
  ui::Layout &col = layout.column(false);
  col.enabled_set(node == nullptr || edit_elements_get_result(*node) == nullptr);
  col.prop(ptr, "component_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(
      srna,
      "component_type",
      "Component",
      "Geometry component that is edited in the 3D Viewport. When the geometry has no elements of "
      "it, the first component that has any is edited instead",
      rna_enum_geometry_component_type_items,
      EnumRNAAccessors(
          [](PointerRNA *ptr, PropertyRNA * /*prop*/) -> int {
            const bNode &node = *static_cast<const bNode *>(ptr->data);
            if (edit_elements_get_result(node)) {
              return int(edit_elements_result_component_type(node));
            }
            return int(node_storage(node).component_type);
          },
          [](PointerRNA *ptr, PropertyRNA * /*prop*/, const int value) {
            bNode &node = *static_cast<bNode *>(ptr->data);
            node_storage(node).component_type = int8_t(value);
          }),
      int(GeometryComponent::Type::Mesh),
      [](bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free) {
        *r_free = true;
        return enum_items_filter(rna_enum_geometry_component_type_items,
                                 [](const EnumPropertyItem &item) -> bool {
                                   return elements_component_slot(
                                              GeometryComponent::Type(item.value)) != -1;
                                 });
      });
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry"_ustr);

  /* Always cache input cook so Enter can seed from the previous node. */
  edit_elements_cache_geometry(params.node().owner_tree(), params.node(), geometry_set);

  /* The stored edit result replaces its component, the others pass through. */
  if (const ID *stored = edit_elements_get_result(params.node())) {
    switch (edit_elements_result_component_type(params.node())) {
      case GeometryComponent::Type::Mesh:
        geometry_set.replace_mesh(
            BKE_mesh_copy_for_eval(*reinterpret_cast<const Mesh *>(stored)));
        break;
      case GeometryComponent::Type::Curve:
        geometry_set.replace_curves(
            BKE_curves_copy_for_eval(reinterpret_cast<const Curves *>(stored)));
        break;
      case GeometryComponent::Type::PointCloud:
        geometry_set.replace_pointcloud(
            BKE_pointcloud_copy_for_eval(reinterpret_cast<const PointCloud *>(stored)));
        break;
      case GeometryComponent::Type::Instance:
        if (const bke::Instances *instances = geometry_set.get_instances()) {
          geometry_set.replace_instances(elements_points_to_instances(
              *instances, *reinterpret_cast<const PointCloud *>(stored)));
        }
        else {
          params.error_message_add(
              NodeWarningType::Warning,
              BLT_translate_do_tooltip_any_thread(
                  "The edited instances are not in the input geometry anymore"));
        }
        break;
      default:
        break;
    }
    params.set_output("Geometry"_ustr, std::move(geometry_set));
    return;
  }

  /* Legacy: absolute positions applied when topology still matches. */
  const NodeGeometryEditElements &storage = node_storage(params.node());
  if (Mesh *mesh = geometry_set.get_mesh_for_write()) {
    if (storage.positions_num > 0 && storage.positions != nullptr) {
      if (mesh->verts_num == storage.positions_num) {
        MutableSpan<float3> positions = mesh->vert_positions_for_write();
        for (const int i : positions.index_range()) {
          positions[i] = float3(storage.positions[i]);
        }
        mesh->tag_positions_changed();
      }
      else {
        params.error_message_add(
            NodeWarningType::Warning,
            TIP_("Legacy stored positions do not match mesh vertex count — re-edit with Enter"));
      }
    }
  }
  else if (storage.positions_num > 0) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Edit Elements expects a mesh component"));
  }

  params.set_output("Geometry"_ustr, std::move(geometry_set));
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeEditElements"_ustr, GEO_NODE_EDIT_ELEMENTS);
  ntype.ui_name = "Edit Elements";
  ntype.ui_description =
      "Enter free Edit Mode on a temporary copy of the cooked input (Enter): mesh, curves, "
      "point cloud, or instances as one point each. All edit tools are allowed; Esc/Enter/Tab "
      "writes the result into the node. Also search: Edit Mesh, Free Edit, Capture Mesh Edit";
  ntype.enum_name_legacy = "EDIT_ELEMENTS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.draw_buttons_ex = node_layout_ex;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  bke::node_type_storage(ntype, "NodeGeometryEditElements", node_free_storage, node_copy_storage);
  ntype.default_width = bke::NodeWidth::_180;
  bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace nodes::node_geo_edit_elements_cc

}  // namespace blender
