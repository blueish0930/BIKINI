/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <fmt/format.h>

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_fields.hh"
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

void select_elements_set_indices(bNode &node, const bke::AttrDomain domain, const Span<int> indices)
{
  using namespace node_geo_select_elements_cc;
  NodeGeometrySelectElements &storage = node_storage(node);
  storage.domain = int8_t(domain);
  /* indices allocated with MEM_new_array_uninitialized → free with MEM_delete (not operator delete). */
  if (storage.indices) {
    MEM_delete(storage.indices);
    storage.indices = nullptr;
  }
  storage.indices_num = int(indices.size());
  if (indices.is_empty()) {
    storage.indices = nullptr;
    return;
  }
  storage.indices = MEM_new_array_uninitialized<int>(size_t(indices.size()), __func__);
  memcpy(storage.indices, indices.data(), sizeof(int) * size_t(indices.size()));
}

bool is_select_elements_node(const bNode &node)
{
  return node.is_type("GeometryNodeSelectElements"_ustr);
}

}  // namespace nodes

namespace nodes::node_geo_select_elements_cc {

/** \} */

/* -------------------------------------------------------------------- */
/** \name Selection field
 * \{ */

class SelectElementsFieldInput final : public bke::GeometryFieldInput {
 private:
  const bke::AttrDomain domain_;
  Array<int> indices_;

 public:
  SelectElementsFieldInput(const bke::AttrDomain domain, Span<int> indices)
      : bke::GeometryFieldInput(CPPType::get<bool>(), "Select Elements"),
        domain_(domain),
        indices_(indices)
  {
    /* Indices are expected sorted for binary search; sort a copy if needed. */
    if (!std::is_sorted(indices_.begin(), indices_.end())) {
      std::sort(indices_.begin(), indices_.end());
    }
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask & /*mask*/) const final
  {
    const std::optional<AttributeAccessor> attributes_opt = context.attributes();
    if (!attributes_opt) {
      return {};
    }
    const AttributeAccessor &attributes = *attributes_opt;

    const int domain_size = attributes.domain_size(domain_);
    Array<bool> selection(domain_size, false);
    for (const int index : indices_) {
      if (uint(index) < uint(domain_size)) {
        selection[index] = true;
      }
    }

    VArray<bool> domain_selection = VArray<bool>::from_container(std::move(selection));
    if (context.domain() == domain_) {
      return domain_selection;
    }
    return attributes.adapt_domain<bool>(std::move(domain_selection), domain_, context.domain());
  }

  std::optional<bke::AttrDomain> preferred_domain(
      const GeometryComponent & /*component*/) const final
  {
    return domain_;
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep & /*deep_hash_cache*/) const override
  {
    static constexpr int8_t id = 0;
    hash.add(&id);
    hash.add(domain_);
    hash.add(Span(indices_));
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Node declare / layout / storage
 * \{ */

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  /* Required so draw_buttons (Reset / domain / status) appear on the node body. */
  b.add_default_layout();

  b.add_input<decl::Geometry>("Geometry"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::Mesh, GeometryComponent::Type::Curve})
      .description("Mesh or curves to select on (only one component type at a time)")
      .is_default_link_socket();
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Unmodified input geometry");
  b.add_output<decl::Bool>("Selection"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "True for interactively selected elements on the stored domain. Press Enter with only "
          "this node selected to edit the selection in the 3D Viewport; Esc confirms and exits");
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometrySelectElements *data = MEM_new<NodeGeometrySelectElements>(__func__);
  data->domain = int8_t(bke::AttrDomain::Point);
  data->indices_num = 0;
  data->indices = nullptr;
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
    MEM_delete(storage);
  }
  node->storage = nullptr;
}

static void node_copy_storage(bNodeTree * /*tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometrySelectElements &src = node_storage(*src_node);
  NodeGeometrySelectElements *dst = MEM_new<NodeGeometrySelectElements>(__func__);
  dst->domain = src.domain;
  dst->indices_num = src.indices_num;
  if (src.indices_num > 0 && src.indices != nullptr) {
    dst->indices = MEM_new_array_uninitialized<int>(size_t(src.indices_num), __func__);
    memcpy(dst->indices, src.indices, sizeof(int) * size_t(src.indices_num));
  }
  else {
    dst->indices = nullptr;
  }
  dst_node->storage = dst;
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  const NodeGeometrySelectElements &storage = node_storage(node);
  if (storage.indices_num > 0 && storage.indices != nullptr) {
    writer.write_int32_array(storage.indices_num, storage.indices);
  }
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  NodeGeometrySelectElements &storage = node_storage(node);
  if (storage.indices_num > 0) {
    if (!BLO_read_array(&reader, &storage.indices, storage.indices_num)) {
      storage.indices = nullptr;
      storage.indices_num = 0;
    }
  }
  else {
    storage.indices = nullptr;
  }
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

  layout.prop(ptr, "domain", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_layout_ex(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "domain", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem domain_items[] = {
      {int(bke::AttrDomain::Point), "POINT", 0, "Point", "Select mesh vertices or curve control points"},
      {int(bke::AttrDomain::Edge), "EDGE", 0, "Edge", "Select mesh edges"},
      {int(bke::AttrDomain::Face), "FACE", 0, "Face", "Select mesh faces"},
      {int(bke::AttrDomain::Curve), "CURVE", 0, "Spline", "Select whole curves/splines"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "domain",
                    "Domain",
                    "Domain of the stored interactive selection",
                    domain_items,
                    NOD_storage_enum_accessors(domain),
                    int(bke::AttrDomain::Point));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Execute
 * \{ */

static int count_supported_components(const GeometrySet &geometry)
{
  int count = 0;
  if (geometry.has_mesh()) {
    count++;
  }
  if (geometry.has_curves()) {
    count++;
  }
  return count;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  /* Input Geometry is the *previous node's cooked output*. Cache it for Enter, then pass the
   * same GeometrySet through to the Geometry output (unmodified) so downstream nodes see the
   * identical cook — never the host object datablock. */
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry"_ustr);

  /* Always cache for Enter-to-edit, even when empty. Copy so the map keeps its own GeometrySet
   * while we still move the original to the output socket. */
  select_elements_cache_geometry(params.node().owner_tree(), params.node(), geometry_set);

  if (count_supported_components(geometry_set) > 1) {
    params.error_message_add(
        NodeWarningType::Error,
        TIP_("Select Elements supports only one geometry component at a time "
             "(mesh or curves, not both)"));
    /* Still pass geometry through so topology stays available downstream / in cache. */
    params.set_output("Geometry"_ustr, std::move(geometry_set));
    params.set_output("Selection"_ustr, Field<bool>(false));
    return;
  }

  const NodeGeometrySelectElements &storage = node_storage(params.node());
  const bke::AttrDomain domain = bke::AttrDomain(storage.domain);
  const Span<int> indices(storage.indices, storage.indices_num);

  if (geometry_set.has_mesh()) {
    if (!ELEM(domain, bke::AttrDomain::Point, bke::AttrDomain::Edge, bke::AttrDomain::Face)) {
      params.error_message_add(NodeWarningType::Error,
                               TIP_("Mesh selection domain must be Point, Edge, or Face"));
    }
  }
  else if (geometry_set.has_curves()) {
    if (!ELEM(domain, bke::AttrDomain::Point, bke::AttrDomain::Curve)) {
      params.error_message_add(NodeWarningType::Error,
                               TIP_("Curve selection domain must be Point or Spline"));
    }
  }

  /* Downstream Geometry = same as previous node cook (passthrough). */
  params.set_output("Geometry"_ustr, std::move(geometry_set));
  params.set_output("Selection"_ustr,
                    Field<bool>::from_input<SelectElementsFieldInput>(domain, indices));
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
      "Pass geometry through and output a selection field from components selected with "
      "native Edit Mode tools on a temporary copy of the evaluated input geometry. "
      "With only this node selected, press Enter to enter Edit Mode selection; Esc confirms";
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
