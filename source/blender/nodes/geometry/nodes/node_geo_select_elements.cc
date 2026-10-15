/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <fmt/format.h>

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_runtime.hh"

#include "BLI_array.hh"
#include "BLI_index_mask.hh"
#include "BLI_map.hh"
#include "BLI_mutex.hh"
#include "BLI_set.hh"
#include "BLI_span.hh"

#include "BLO_read_write.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_node_types.h"
#include "DNA_pointcloud_types.h"

#include "NOD_geo_elements_component.hh"
#include "NOD_geo_select_elements.hh"
#include "NOD_rna_define.hh"

#include "RNA_access.hh"
#include "RNA_enum_types.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender {

namespace nodes::node_geo_select_elements_cc {

NODE_STORAGE_FUNCS(NodeGeometrySelectElements)

/* -------------------------------------------------------------------- */
/** \name Geometry cache for interactive selection
 * \{ */

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
  /* Always key by the original tree. Modifier eval uses the CoW/evaluated tree which has a
   * different session_uid; Enter-to-edit looks up with the editor's original edittree. */
  const bNodeTree &tree_orig = *DEG_get_original(&tree);
  return {tree_orig.id.session_uid, node.identifier};
}

static Mutex &force_eval_mutex()
{
  static Mutex mutex;
  return mutex;
}

static std::optional<SelectElementsForceEvalRequest> &force_eval_request()
{
  static std::optional<SelectElementsForceEvalRequest> request;
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

/* Indices are allocated with #MEM_new_array_uninitialized, so free with #MEM_delete. */
static void free_selection(NodeGeometrySelectElementsSelection &selection)
{
  if (selection.indices) {
    MEM_delete(selection.indices);
    selection.indices = nullptr;
  }
  selection.indices_num = 0;
}

}  // namespace nodes::node_geo_select_elements_cc

namespace nodes {

void select_elements_set_force_eval_request(const SelectElementsForceEvalRequest &request)
{
  using namespace node_geo_select_elements_cc;
  std::lock_guard lock{force_eval_mutex()};
  force_eval_request() = request;
}

void select_elements_clear_force_eval_request()
{
  using namespace node_geo_select_elements_cc;
  std::lock_guard lock{force_eval_mutex()};
  force_eval_request().reset();
}

std::optional<SelectElementsForceEvalRequest> select_elements_get_force_eval_request()
{
  using namespace node_geo_select_elements_cc;
  std::lock_guard lock{force_eval_mutex()};
  return force_eval_request();
}

void select_elements_cache_geometry(const bNodeTree &tree,
                                    const bNode &node,
                                    bke::GeometrySet geometry)
{
  using namespace node_geo_select_elements_cc;
  prepare_geometry_for_cache(geometry);
  std::lock_guard lock{cache_mutex()};
  geometry_cache().add_overwrite(make_key(tree, node), std::move(geometry));
}

std::optional<bke::GeometrySet> select_elements_copy_cached_geometry(const bNodeTree &tree,
                                                                     const bNode &node)
{
  using namespace node_geo_select_elements_cc;
  std::lock_guard lock{cache_mutex()};
  if (const bke::GeometrySet *geometry = geometry_cache().lookup_ptr(make_key(tree, node))) {
    return *geometry;
  }
  return std::nullopt;
}

void select_elements_clear_cache(const bNodeTree &tree, const bNode &node)
{
  using namespace node_geo_select_elements_cc;
  std::lock_guard lock{cache_mutex()};
  geometry_cache().remove(make_key(tree, node));
}

void select_elements_free_all_caches()
{
  using namespace node_geo_select_elements_cc;
  std::lock_guard lock{cache_mutex()};
  geometry_cache().clear();
}

void select_elements_set_indices(bNode &node,
                                 const bke::GeometryComponent::Type component_type,
                                 const bke::AttrDomain domain,
                                 const Span<int> indices)
{
  using namespace node_geo_select_elements_cc;
  const int slot = elements_component_slot(component_type);
  if (slot == -1) {
    return;
  }
  NodeGeometrySelectElementsSelection &selection = node_storage(node).selections[slot];
  free_selection(selection);
  selection.domain = int8_t(elements_component_sanitize_domain(component_type, domain));
  if (indices.is_empty()) {
    return;
  }
  selection.indices_num = int(indices.size());
  selection.indices = MEM_new_array_uninitialized<int>(size_t(indices.size()), __func__);
  memcpy(selection.indices, indices.data(), sizeof(int) * size_t(indices.size()));
}

Span<int> select_elements_get_indices(const bNode &node,
                                      const bke::GeometryComponent::Type component_type,
                                      bke::AttrDomain &r_domain)
{
  using namespace node_geo_select_elements_cc;
  const int slot = elements_component_slot(component_type);
  if (slot == -1) {
    r_domain = bke::AttrDomain::Point;
    return {};
  }
  const NodeGeometrySelectElementsSelection &selection = node_storage(node).selections[slot];
  r_domain = elements_component_sanitize_domain(component_type, bke::AttrDomain(selection.domain));
  return Span<int>(selection.indices, selection.indices_num);
}

void select_elements_clear_indices(bNode &node)
{
  using namespace node_geo_select_elements_cc;
  for (NodeGeometrySelectElementsSelection &selection : node_storage(node).selections) {
    free_selection(selection);
  }
}

bool is_select_elements_node(const bNode &node)
{
  return node.is_type("GeometryNodeSelectElements"_ustr);
}

}  // namespace nodes

namespace nodes::node_geo_select_elements_cc {

/** \} */

/* -------------------------------------------------------------------- */
/** \name Node declare / layout / storage
 * \{ */

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  /* Required so draw_buttons (Reset / component / domain) appear on the node body. */
  b.add_default_layout();

  b.add_input<decl::Geometry>("Geometry"_ustr)
      .supported_type({GeometryComponent::Type::Mesh,
                       GeometryComponent::Type::Curve,
                       GeometryComponent::Type::PointCloud,
                       GeometryComponent::Type::Instance})
      .description(
          "Geometry to select on. Mesh, curves, point cloud and instances each have their own "
          "selection")
      .is_default_link_socket();
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("The input geometry with the selection stored on it");
  b.add_output<decl::Bool>("Selection"_ustr)
      .anonymous_attribute_output()
      .description(
          "The interactively selected elements, stored as an attribute on the geometry so that it "
          "follows the elements through later nodes. Press Enter with only this node selected to "
          "edit the selection in the 3D Viewport; Esc confirms and exits");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySelectElements *data = MEM_new<NodeGeometrySelectElements>(__func__);
  data->domain = int8_t(bke::AttrDomain::Point);
  data->component_type = int8_t(GeometryComponent::Type::Mesh);
  node->storage = data;
  /* Ensure the node body buttons (Reset, domain, …) are visible. */
  node->flag |= NODE_OPTIONS;
}

static void node_free_storage(bNode *node)
{
  NodeGeometrySelectElements *storage = static_cast<NodeGeometrySelectElements *>(node->storage);
  if (storage) {
    if (storage->indices) {
      MEM_delete(storage->indices);
      storage->indices = nullptr;
    }
    for (NodeGeometrySelectElementsSelection &selection : storage->selections) {
      free_selection(selection);
    }
    MEM_delete(storage);
  }
  node->storage = nullptr;
}

static void node_copy_storage(bNodeTree * /*tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometrySelectElements &src = node_storage(*src_node);
  NodeGeometrySelectElements *dst = MEM_new<NodeGeometrySelectElements>(__func__);
  dst->domain = src.domain;
  dst->component_type = src.component_type;
  for (const int i : IndexRange(ARRAY_SIZE(src.selections))) {
    const NodeGeometrySelectElementsSelection &src_selection = src.selections[i];
    NodeGeometrySelectElementsSelection &dst_selection = dst->selections[i];
    dst_selection.domain = src_selection.domain;
    if (src_selection.indices_num > 0 && src_selection.indices != nullptr) {
      dst_selection.indices_num = src_selection.indices_num;
      dst_selection.indices = MEM_new_array_uninitialized<int>(size_t(src_selection.indices_num),
                                                               __func__);
      memcpy(dst_selection.indices,
             src_selection.indices,
             sizeof(int) * size_t(src_selection.indices_num));
    }
  }
  dst_node->storage = dst;
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  const NodeGeometrySelectElements &storage = node_storage(node);
  for (const NodeGeometrySelectElementsSelection &selection : storage.selections) {
    if (selection.indices_num > 0 && selection.indices != nullptr) {
      writer.write_int32_array(selection.indices_num, selection.indices);
    }
  }
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  NodeGeometrySelectElements &storage = node_storage(node);
  for (NodeGeometrySelectElementsSelection &selection : storage.selections) {
    if (selection.indices_num > 0) {
      if (!BLO_read_array(&reader, &selection.indices, selection.indices_num)) {
        selection.indices = nullptr;
        selection.indices_num = 0;
      }
    }
    else {
      selection.indices = nullptr;
    }
  }

  /* Files from before every component had its own selection stored a single one. */
  if (storage.indices_num > 0) {
    if (BLO_read_array(&reader, &storage.indices, storage.indices_num)) {
      const bke::AttrDomain domain = bke::AttrDomain(storage.domain);
      const GeometryComponent::Type component_type = domain == bke::AttrDomain::Curve ?
                                                         GeometryComponent::Type::Curve :
                                                         GeometryComponent::Type::Mesh;
      NodeGeometrySelectElementsSelection &selection =
          storage.selections[elements_component_slot(component_type)];
      if (selection.indices == nullptr) {
        selection.domain = storage.domain;
        selection.indices = storage.indices;
        selection.indices_num = storage.indices_num;
        storage.component_type = int8_t(component_type);
      }
      else {
        MEM_delete(storage.indices);
      }
    }
  }
  storage.indices = nullptr;
  storage.indices_num = 0;
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  const bNode *node = static_cast<const bNode *>(ptr->data);

  /* Node body only: Reset + Edit (no description text; not N-panel). */
  {
    ui::Layout &row = layout.row(true);
    row.scale_y_set(1.2f);
    PointerRNA reset_ptr = row.op(
        "node.select_elements_reset", IFACE_("Reset"), ICON_FILE_REFRESH);
    if (node) {
      RNA_int_set(&reset_ptr, "node_id", node->identifier);
    }
    row.op("node.select_elements_edit", IFACE_("Edit"), ICON_EDITMODE_HLT);
  }

  layout.prop(ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_layout_ex(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "mode", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

/** What the Edit button selects, a component together with a domain. */
enum class SelectMode : int8_t {
  Point = 0,
  Edge = 1,
  Face = 2,
  Curve = 3,
  PointCloud = 4,
  Instances = 5,
};

static SelectMode select_mode_get(const NodeGeometrySelectElements &storage)
{
  const bke::AttrDomain domain = bke::AttrDomain(storage.domain);
  switch (GeometryComponent::Type(storage.component_type)) {
    case GeometryComponent::Type::Curve:
      /* Control points are points, also when curves are preferred over a mesh for them. */
      return domain == bke::AttrDomain::Curve ? SelectMode::Curve : SelectMode::Point;
    case GeometryComponent::Type::PointCloud:
      return SelectMode::PointCloud;
    case GeometryComponent::Type::Instance:
      return SelectMode::Instances;
    default:
      break;
  }
  switch (domain) {
    case bke::AttrDomain::Edge:
      return SelectMode::Edge;
    case bke::AttrDomain::Face:
      return SelectMode::Face;
    default:
      return SelectMode::Point;
  }
}

static void select_mode_set(NodeGeometrySelectElements &storage, const SelectMode mode)
{
  GeometryComponent::Type component_type = GeometryComponent::Type::Mesh;
  bke::AttrDomain domain = bke::AttrDomain::Point;
  switch (mode) {
    case SelectMode::Point:
      break;
    case SelectMode::Edge:
      domain = bke::AttrDomain::Edge;
      break;
    case SelectMode::Face:
      domain = bke::AttrDomain::Face;
      break;
    case SelectMode::Curve:
      component_type = GeometryComponent::Type::Curve;
      domain = bke::AttrDomain::Curve;
      break;
    case SelectMode::PointCloud:
      component_type = GeometryComponent::Type::PointCloud;
      break;
    case SelectMode::Instances:
      component_type = GeometryComponent::Type::Instance;
      domain = bke::AttrDomain::Instance;
      break;
  }
  storage.component_type = int8_t(component_type);
  storage.domain = int8_t(domain);
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem domain_items[] = {
      {int(bke::AttrDomain::Point),
       "POINT",
       0,
       "Point",
       "Select mesh vertices, curve control points or points of a point cloud"},
      {int(bke::AttrDomain::Edge), "EDGE", 0, "Edge", "Select mesh edges"},
      {int(bke::AttrDomain::Face), "FACE", 0, "Face", "Select mesh faces"},
      {int(bke::AttrDomain::Curve), "CURVE", 0, "Spline", "Select whole curves/splines"},
      {int(bke::AttrDomain::Instance), "INSTANCE", 0, "Instance", "Select instances"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem mode_items[] = {
      {int(SelectMode::Point),
       "POINT",
       ICON_VERTEXSEL,
       "Point",
       "Select the vertices of the mesh in Edit Mode. Without a mesh, the control points of the "
       "curves or the points of the point cloud"},
      {int(SelectMode::Edge), "EDGE", ICON_EDGESEL, "Edge", "Select mesh edges in Edit Mode"},
      {int(SelectMode::Face), "FACE", ICON_FACESEL, "Face", "Select mesh faces in Edit Mode"},
      {int(SelectMode::Curve),
       "CURVE",
       ICON_CURVES_DATA,
       "Curve",
       "Select whole curves. Every curve is a temporary object that is selected in Object Mode"},
      {int(SelectMode::PointCloud),
       "POINTCLOUD",
       ICON_POINTCLOUD_DATA,
       "Point Cloud",
       "Select the points of the point cloud in Edit Mode"},
      {int(SelectMode::Instances),
       "INSTANCES",
       ICON_EMPTY_AXIS,
       "Instances",
       "Select instances. Every instance is a temporary object that is selected in Object Mode"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "What is selected when the selection is edited in the 3D Viewport",
                    mode_items,
                    EnumRNAAccessors(
                        [](PointerRNA *ptr, PropertyRNA * /*prop*/) -> int {
                          const bNode &node = *static_cast<const bNode *>(ptr->data);
                          return int(select_mode_get(node_storage(node)));
                        },
                        [](PointerRNA *ptr, PropertyRNA * /*prop*/, const int value) {
                          bNode &node = *static_cast<bNode *>(ptr->data);
                          select_mode_set(node_storage(node), SelectMode(value));
                        }),
                    int(SelectMode::Point));

  /* The parts of the mode on their own, for scripts. */
  RNA_def_node_enum(
      srna,
      "component_type",
      "Component",
      "Geometry component whose selection is edited in the 3D Viewport. When the geometry has no "
      "elements of it, the first component that has any is edited instead",
      rna_enum_geometry_component_type_items,
      NOD_storage_enum_accessors(component_type),
      int(GeometryComponent::Type::Mesh),
      [](bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free) {
        *r_free = true;
        return enum_items_filter(rna_enum_geometry_component_type_items,
                                 [](const EnumPropertyItem &item) -> bool {
                                   return elements_component_slot(
                                              GeometryComponent::Type(item.value)) != -1;
                                 });
      });

  /* The domains depend on the component, the stored one may be of another component. */
  RNA_def_node_enum(
      srna,
      "domain",
      "Domain",
      "Kind of element that is selected when the selection is edited",
      domain_items,
      EnumRNAAccessors(
          [](PointerRNA *ptr, PropertyRNA * /*prop*/) -> int {
            const bNode &node = *static_cast<const bNode *>(ptr->data);
            const NodeGeometrySelectElements &storage = node_storage(node);
            return int(elements_component_sanitize_domain(
                GeometryComponent::Type(storage.component_type),
                bke::AttrDomain(storage.domain)));
          },
          [](PointerRNA *ptr, PropertyRNA * /*prop*/, const int value) {
            bNode &node = *static_cast<bNode *>(ptr->data);
            node_storage(node).domain = int8_t(value);
          }),
      int(bke::AttrDomain::Point),
      [](bContext * /*C*/, PointerRNA *ptr, PropertyRNA * /*prop*/, bool *r_free) {
        *r_free = true;
        const bNode *node = static_cast<const bNode *>(ptr->data);
        if (node == nullptr || node->storage == nullptr) {
          return enum_items_filter(domain_items,
                                   [](const EnumPropertyItem & /*item*/) { return true; });
        }
        const GeometryComponent::Type component_type = GeometryComponent::Type(
            node_storage(*node).component_type);
        return enum_items_filter(domain_items, [&](const EnumPropertyItem &item) -> bool {
          return elements_component_supports_domain(component_type, bke::AttrDomain(item.value));
        });
      });
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Execute
 * \{ */

static void store_selection(MutableAttributeAccessor attributes,
                            const StringRef attribute_id,
                            const GeometryComponent::Type component_type,
                            const NodeGeometrySelectElementsSelection &selection)
{
  const bke::AttrDomain domain = elements_component_sanitize_domain(
      component_type, bke::AttrDomain(selection.domain));
  SpanAttributeWriter<bool> writer = attributes.lookup_or_add_for_write_only_span<bool>(
      attribute_id, domain);
  if (!writer) {
    return;
  }
  writer.span.fill(false);
  /* The geometry may have changed since the elements were selected. */
  const uint domain_size = uint(writer.span.size());
  for (const int index : Span<int>(selection.indices, selection.indices_num)) {
    if (uint(index) < domain_size) {
      writer.span[index] = true;
    }
  }
  writer.finish();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  /* Input Geometry is the *previous node's cooked output*. Cache it for Enter, then pass it on
   * with nothing but the selection attribute added, never the host object datablock. */
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry"_ustr);

  /* Always cache for Enter-to-edit, even when empty. Copy so the map keeps its own GeometrySet
   * while we still move the original to the output socket. */
  select_elements_cache_geometry(params.node().owner_tree(), params.node(), geometry_set);

  /* The selection is an attribute on the elements instead of a field that compares indices, so
   * it stays on the selected elements when later nodes add, remove or reorder elements. A
   * component without a selection gets no attribute, the field is false there. */
  const std::optional<std::string> selection_id =
      params.get_output_anonymous_attribute_id_if_needed("Selection"_ustr);
  if (selection_id) {
    const NodeGeometrySelectElements &storage = node_storage(params.node());
    for (const GeometryComponent::Type component_type : elements_component_types) {
      const NodeGeometrySelectElementsSelection &selection =
          storage.selections[elements_component_slot(component_type)];
      if (selection.indices_num == 0 || selection.indices == nullptr) {
        continue;
      }
      if (elements_component_size(geometry_set, component_type) == 0) {
        continue;
      }
      switch (component_type) {
        case GeometryComponent::Type::Mesh:
          store_selection(geometry_set.get_mesh_for_write()->attributes_for_write(),
                          *selection_id,
                          component_type,
                          selection);
          break;
        case GeometryComponent::Type::Curve:
          store_selection(
              geometry_set.get_curves_for_write()->geometry.wrap().attributes_for_write(),
              *selection_id,
              component_type,
              selection);
          break;
        case GeometryComponent::Type::PointCloud:
          store_selection(geometry_set.get_pointcloud_for_write()->attributes_for_write(),
                          *selection_id,
                          component_type,
                          selection);
          break;
        case GeometryComponent::Type::Instance:
          store_selection(geometry_set.get_instances_for_write()->attributes_for_write(),
                          *selection_id,
                          component_type,
                          selection);
          break;
        default:
          break;
      }
    }
  }

  params.set_output("Geometry"_ustr, std::move(geometry_set));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Register
 * \{ */

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeSelectElements"_ustr, GEO_NODE_SELECT_ELEMENTS);
  ntype.ui_name = "Select Elements";
  ntype.ui_description =
      "Store a selection on the geometry that is made in the 3D Viewport on a temporary copy of "
      "the evaluated input: points, edges and faces in Edit Mode, whole curves and instances as "
      "one object each. With only this node selected, press Enter to select; Esc confirms";
  ntype.enum_name_legacy = "SELECT_ELEMENTS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.draw_buttons_ex = node_layout_ex;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  bke::node_type_storage(ntype, "NodeGeometrySelectElements", node_free_storage, node_copy_storage);
  ntype.default_width = bke::NodeWidth::_180;
  bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

/** \} */

}  // namespace nodes::node_geo_select_elements_cc

}  // namespace blender
