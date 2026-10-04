/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edcurves
 */

#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_listbase.hh"
#include "BLI_math_matrix.hh"
#include "BLI_rect.hh"
#include "BLI_string_utf8.hh"
#include "BLI_time.hh"

#include "DNA_camera_types.h"
#include "DNA_key_types.h"
#include "ED_curves.hh"
#include "ED_grease_pencil.hh"
#include "ED_object.hh"
#include "ED_screen.hh"
#include "ED_select_utils.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "wm_event_system.hh"

#include "BKE_asset.hh"
#include "BKE_bake_data_block_id.hh"
#include "BKE_bake_data_block_map.hh"
#include "BKE_compute_context_cache.hh"
#include "BKE_compute_contexts.hh"
#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_customdata.hh"
#include "BKE_editmesh.hh"
#include "BKE_geometry_nodes_reference_set.hh"
#include "BKE_geometry_set.hh"
#include "BKE_global.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_idprop_hash.hh"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
#include "BKE_main.hh"
#include "BKE_material.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_wrapper.hh"
#include "BKE_node_runtime.hh"
#include "BKE_object.hh"
#include "BKE_object_types.hh"
#include "BKE_paint.hh"
#include "BKE_paint_bvh.hh"
#include "BKE_pointcloud.hh"
#include "BKE_report.hh"
#include "BKE_scene.hh"
#include "BKE_screen.hh"
#include "BKE_wm_runtime.hh"
#include "BKE_workspace.hh"

#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"
#include "DEG_depsgraph_query.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"
#include "RNA_prototypes.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "FN_lazy_function_execute.hh"

#include "ED_asset.hh"
#include "ED_asset_menu_utils.hh"
#include "ED_geometry.hh"
#include "ED_mesh.hh"
#include "ED_sculpt.hh"

#include "intern/bmesh_mesh_convert.hh"

#include "BLT_translation.hh"

#include "NOD_dependencies.hh"
#include "NOD_geometry_nodes_caller_ui.hh"
#include "NOD_geometry_nodes_execute.hh"
#include "NOD_geometry_nodes_lazy_function.hh"
#include "NOD_geometry_nodes_srna.hh"

#include "AS_asset_catalog.hh"
#include "AS_asset_catalog_path.hh"
#include "AS_asset_library.hh"
#include "AS_asset_representation.hh"

#include "PRF_profile.hh"

#include <xxhash.h>

#include "geometry_intern.hh"

#include <fmt/format.h>

namespace blender::ed::geometry {

using asset_system::AssetRepresentation;

struct ErrorsForType {
  int duplicate_count = 0;
  bool is_builtin_operator = false;
  bool invalid_metadata = false;
  Vector<std::string> idname_validation_errors;
  Vector<std::string> invalid_input_metadata_errors;

  friend bool operator==(const ErrorsForType &a, const ErrorsForType &b) = default;
};

struct RegistrationData {
  struct TypeTreeItem {
    std::string name;
    Vector<wmOperatorType *> types;
    Vector<std::unique_ptr<TypeTreeItem>> children;
  };
  using Errors = Map<std::string, ErrorsForType>;
  Vector<wmOperatorType *> local_types;
  Vector<wmOperatorType *> unassigned_types;
  Vector<std::unique_ptr<TypeTreeItem>> menu_path_tree_roots;
  Errors errors_by_idname;
};

static RegistrationData &get_registration_data()
{
  static RegistrationData data;
  return data;
}

/**
 * Abstraction layer over local node groups and assets, so operators can be registered and
 * referenced regardless of that distinction.
 */
struct OperatorTypeData : public wmOperatorType::TypeData {
  std::string name;
  std::string idname;
  StringRefNull custom_idname;
  std::string description;
  GeometryNodeAssetTraitFlag flag;

  std::unique_ptr<IDProperty, bke::idprop::IDPropertyDeleter> asset_meta_data_properties;
  Vector<StructRNA *> generated_structs;
  Vector<Array<EnumPropertyItem, 0>> enum_item_storage;
  Array<EnumPropertyItem, 0> modal_keymap_items;
  /** Owns the identifier strings referenced by #modal_keymap_items. */
  Array<std::string, 0> modal_keymap_item_identifiers;

  struct LocalRef {
    uint32_t session_uid;
  };
  std::variant<AssetWeakReference, LocalRef> group_ref;

  UniqueHash hash;

  static std::optional<OperatorTypeData> from_asset(const AssetRepresentation &asset,
                                                    RegistrationData::Errors &errors);
  static std::optional<OperatorTypeData> from_group(const bNodeTree &group,
                                                    RegistrationData::Errors &errors);

 private:
  /** Should be called after any data changes. */
  void ensure_hash();
};

void OperatorTypeData::ensure_hash()
{
  XXH3_state_t *hash_state = XXH3_createState();
  BLI_SCOPED_DEFER([&hash_state]() -> void { XXH3_freeState(hash_state); })
  XXH3_128bits_reset(hash_state);
  XXH3_128bits_update(hash_state, this->name.data(), this->name.size());
  XXH3_128bits_update(hash_state, this->description.data(), this->description.size());
  XXH3_128bits_update(hash_state, &this->flag, sizeof(this->flag));
  std::visit(
      [&](const auto &value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, AssetWeakReference>) {
          XXH3_128bits_update(
              hash_state, &value.asset_library_type, sizeof(value.asset_library_type));
          if (value.asset_library_identifier) {
            XXH3_128bits_update(hash_state,
                                value.asset_library_identifier,
                                strlen(value.asset_library_identifier));
          }
          if (value.relative_asset_identifier) {
            XXH3_128bits_update(hash_state,
                                value.relative_asset_identifier,
                                strlen(value.relative_asset_identifier));
          }
        }
        else if constexpr (std::is_same_v<T, LocalRef>) {
          XXH3_128bits_update(hash_state, &value.session_uid, sizeof(value.session_uid));
        }
        else {
          BLI_assert_unreachable_static_t(T);
        }
      },
      this->group_ref);
  bke::idprop::hash(*this->asset_meta_data_properties, hash_state);
  static_assert(sizeof(this->hash) == sizeof(XXH128_hash_t));
  const XXH128_hash_t xxh3_hash = XXH3_128bits_digest(hash_state);
  this->hash = {xxh3_hash.low64, xxh3_hash.high64};
}

static std::optional<std::string> operator_idname_get(const StringRefNull custom_idname,
                                                      RegistrationData::Errors *errors)
{
  ReportList reports;
  BKE_reports_init(&reports, RPT_STORE | RPT_PRINT_HANDLED_BY_OWNER);
  BLI_SCOPED_DEFER([&]() { BKE_reports_free(&reports); });
  if (!WM_operator_idname_ok_or_report(&reports, custom_idname.c_str())) {
    if (errors) {
      ErrorsForType &errors_for_type = errors->lookup_or_add_default_as(custom_idname);
      for (Report &report : reports.list) {
        errors_for_type.idname_validation_errors.append_as(report.message);
      }
    }
    return std::nullopt;
  }
  char idname_buf[OP_MAX_TYPENAME];
  WM_operator_bl_idname(idname_buf, custom_idname.c_str());
  return idname_buf;
}

static std::optional<StringRefNull> custom_idname_for_asset(const AssetRepresentation &asset)
{
  const AssetMetaData &metadata = asset.get_metadata();
  const IDProperty *id_property = BKE_asset_metadata_idprop_find(&metadata, "node_tool_idname");
  if (!id_property || id_property->type != IDP_STRING) {
    return std::nullopt;
  }
  return IDP_string_get(id_property);
}

std::optional<OperatorTypeData> OperatorTypeData::from_asset(
    const asset_system::AssetRepresentation &asset, RegistrationData::Errors &errors)
{
  const std::optional<StringRefNull> custom_idname = custom_idname_for_asset(asset);
  if (!custom_idname) {
    return std::nullopt;
  }
  std::optional<std::string> idname = operator_idname_get(*custom_idname, &errors);
  if (!idname) {
    return std::nullopt;
  }

  const AssetMetaData &metadata = asset.get_metadata();
  OperatorTypeData type_data;
  type_data.name = asset.get_name();
  type_data.idname = std::move(*idname);
  type_data.custom_idname = *custom_idname;
  type_data.description = metadata.description ? metadata.description : "";
  const IDProperty *traits_flag = BKE_asset_metadata_idprop_find(
      &metadata, "geometry_node_asset_traits_flag");
  if (!traits_flag || traits_flag->type != IDP_INT) {
    ErrorsForType &errors_for_type = errors.lookup_or_add_default_as(*custom_idname);
    errors_for_type.invalid_metadata = true;
    return std::nullopt;
  }
  type_data.flag = GeometryNodeAssetTraitFlag(IDP_int_get(traits_flag));
  type_data.group_ref = asset.make_weak_reference();

  const IDProperty *properties = BKE_asset_metadata_idprop_find(&metadata, "properties");
  if (!properties || properties->type != IDP_GROUP) {
    ErrorsForType &errors_for_type = errors.lookup_or_add_default_as(*custom_idname);
    errors_for_type.invalid_metadata = true;
    return std::nullopt;
  }
  const IDProperty *inputs = IDP_GetPropertyFromGroup(properties, "inputs");
  if (!inputs || inputs->type != IDP_GROUP) {
    ErrorsForType &errors_for_type = errors.lookup_or_add_default_as(*custom_idname);
    errors_for_type.invalid_metadata = true;
    return std::nullopt;
  }
  for (const IDProperty &input_prop : inputs->data.group) {
    if (input_prop.type != IDP_GROUP ||
        !IDP_GetPropertyTypeFromGroup(&input_prop, "type", IDP_INT))
    {
      ErrorsForType &errors_for_type = errors.lookup_or_add_default_as(*custom_idname);
      errors_for_type.invalid_input_metadata_errors.append(input_prop.name);
      return std::nullopt;
    }
  }
  type_data.asset_meta_data_properties =
      std::unique_ptr<IDProperty, bke::idprop::IDPropertyDeleter>(IDP_CopyProperty(properties));

  type_data.ensure_hash();
  return type_data;
}

static std::optional<StringRefNull> custom_idname_for_group(const bNodeTree &group)
{
  const char *idname = group.geometry_node_asset_traits->node_tool_idname;
  if (!idname) {
    return std::nullopt;
  }
  return StringRefNull(idname);
}

std::optional<OperatorTypeData> OperatorTypeData::from_group(const bNodeTree &group,
                                                             RegistrationData::Errors &errors)
{
  const std::optional<StringRefNull> custom_idname = custom_idname_for_group(group);
  if (!custom_idname) {
    return std::nullopt;
  }
  std::optional<std::string> idname = operator_idname_get(*custom_idname, &errors);
  if (!idname) {
    return std::nullopt;
  }
  OperatorTypeData type_data;
  type_data.name = BKE_id_name(group.id);
  type_data.idname = std::move(*idname);
  type_data.custom_idname = *custom_idname;
  type_data.description = group.description ? group.description : "";
  type_data.flag = GeometryNodeAssetTraitFlag(group.geometry_node_asset_traits->flag);
  type_data.group_ref = OperatorTypeData::LocalRef{group.id.session_uid};

  type_data.asset_meta_data_properties =
      std::unique_ptr<IDProperty, bke::idprop::IDPropertyDeleter>(
          bke::node_create_asset_meta_data_properties(group));

  type_data.ensure_hash();
  return type_data;
}

GeometryNodeAssetTraitFlag asset_flag_for_context(const Object &active_object);

/* -------------------------------------------------------------------- */
/** \name Operator
 * \{ */

static const bNodeTree *get_asset_or_local_node_group(const bContext &C,
                                                      const wmOperatorType &ot,
                                                      ReportList *reports)
{
  Main &bmain = *CTX_data_main(&C);
  const auto &type_data = static_cast<const OperatorTypeData &>(*ot.custom_data);
  return std::visit(
      [&](const auto &value) -> const bNodeTree * {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, OperatorTypeData::LocalRef>) {
          return id_cast<const bNodeTree *>(
              BKE_libblock_find_session_uid(&bmain, ID_NT, value.session_uid));
        }
        else if constexpr (std::is_same_v<T, AssetWeakReference>) {
          const asset_system::AssetRepresentation *asset = ed::asset::find_asset_from_weak_ref(
              C, value, reports);
          if (!asset) {
            return nullptr;
          }
          return id_cast<const bNodeTree *>(asset::asset_local_id_ensure_imported(bmain, *asset));
        }
        else {
          BLI_assert_unreachable_static_t(T);
        }
      },
      type_data.group_ref);
}

static const bNodeTree *get_node_group(const bContext &C,
                                       const wmOperatorType &ot,
                                       ReportList *reports)
{
  const bNodeTree *group = get_asset_or_local_node_group(C, ot, reports);
  if (!group) {
    return nullptr;
  }
  if (group->type != NTREE_GEOMETRY) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "Asset is not a geometry node group");
    }
    return nullptr;
  }
  return group;
}

GeoOperatorLog::~GeoOperatorLog() = default;

/**
 * The socket value log is stored statically so it can be used in the node editor. A fancier
 * storage system shouldn't be necessary, since the goal is just to be able to debug intermediate
 * values when building a tool.
 */
static GeoOperatorLog &get_static_eval_log()
{
  static GeoOperatorLog log;
  return log;
}

const GeoOperatorLog &node_group_operator_static_eval_log()
{
  return get_static_eval_log();
}

/** Find all the visible node editors to log values for. */
static void find_verbose_log_contexts(const Main &bmain,
                                      Set<ComputeContextHash> &r_verbose_log_contexts)
{
  wmWindowManager *wm = bmain.wm.first();
  if (wm == nullptr) {
    return;
  }
  for (const wmWindow &window : wm->windows) {
    const bScreen *screen = BKE_workspace_active_screen_get(window.workspace_hook);
    for (const ScrArea &area : screen->areabase) {
      const SpaceLink *sl = area.spacedata.first_as<SpaceLink>();
      if (sl->spacetype == SPACE_NODE) {
        const SpaceNode &snode = *reinterpret_cast<const SpaceNode *>(sl);
        if (snode.edittree == nullptr) {
          continue;
        }
        if (snode.node_tree_sub_type != SNODE_GEOMETRY_TOOL) {
          continue;
        }
        bke::ComputeContextCache compute_context_cache;
        const Map<const bke::bNodeTreeZone *, ComputeContextHash> hash_by_zone =
            nodes::eval_log::NodesEvalLog::get_context_hash_by_zone_for_node_editor(
                snode, compute_context_cache);
        for (const ComputeContextHash &hash : hash_by_zone.values()) {
          r_verbose_log_contexts.add(hash);
        }
      }
    }
  }
}

static const ImplicitSharingInfo *get_vertex_group_sharing_info(const Mesh &mesh)
{
  const int layer_index = CustomData_get_layer_index(&mesh.vert_data, CD_MDEFORMVERT);
  if (layer_index == -1) {
    return nullptr;
  }
  return mesh.vert_data.layers[layer_index].sharing_info;
}

/**
 * This class adds a user to shared mesh data, requiring modifications of the mesh to reallocate
 * the data and its sharing info. This allows tracking which data is modified without having to
 * explicitly compare it.
 */
class MeshState {
  VectorSet<ImplicitSharingPtr<>> sharing_infos_;

 public:
  MeshState(const Mesh &mesh)
  {
    if (mesh.runtime->face_offsets_sharing_info) {
      this->freeze_shared_state(*mesh.runtime->face_offsets_sharing_info);
    }
    mesh.attributes().foreach_attribute([&](const bke::AttributeIter &iter) {
      const bke::GAttributeReader attribute = iter.get();
      if (attribute.varray.size() == 0) {
        return;
      }
      if (attribute.sharing_info) {
        this->freeze_shared_state(*attribute.sharing_info);
      }
    });
    if (const ImplicitSharingInfo *sharing_info = get_vertex_group_sharing_info(mesh)) {
      this->freeze_shared_state(*sharing_info);
    }
  }

  void freeze_shared_state(const ImplicitSharingInfo &sharing_info)
  {
    if (sharing_infos_.add(ImplicitSharingPtr<>{&sharing_info})) {
      sharing_info.add_user();
    }
  }
};

static std::string shape_key_attribute_name(const KeyBlock &kb)
{
  return fmt::format(".kb:{}", kb.name);
}

/** Support shape keys by propagating them through geometry nodes as attributes. */
static void add_shape_keys_as_attributes(Mesh &mesh, const Key &key)
{
  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
  for (const KeyBlock &kb : key.block) {
    if (&kb == key.refkey) {
      /* The basis key will just receive values from the mesh positions. */
      continue;
    }
    const Span<float3> key_data(static_cast<float3 *>(kb.data), kb.totelem);
    attributes.add<float3>(shape_key_attribute_name(kb),
                           bke::AttrDomain::Point,
                           bke::AttributeInitVArray(VArray<float3>::from_span(key_data)));
  }
}

/* Copy shape key attributes back to the key data-block. */
static void store_attributes_to_shape_keys(const Mesh &mesh, Key &key)
{
  const bke::AttributeAccessor attributes = mesh.attributes();
  for (KeyBlock &kb : key.block) {
    const VArray attr = *attributes.lookup<float3>(shape_key_attribute_name(kb),
                                                   bke::AttrDomain::Point);
    if (!attr) {
      continue;
    }
    MEM_delete(static_cast<float3 *>(kb.data));
    kb.data = MEM_new_array_uninitialized<float3>(attr.size(), __func__);
    kb.totelem = attr.size();
    attr.materialize({static_cast<float3 *>(kb.data), attr.size()});
  }
  if (KeyBlock *kb = key.refkey) {
    const Span<float3> positions = mesh.vert_positions();
    MEM_delete(static_cast<float3 *>(kb->data));
    kb->data = MEM_new_array_uninitialized<float3>(positions.size(), __func__);
    kb->totelem = positions.size();
    array_utils::copy(positions, MutableSpan(static_cast<float3 *>(kb->data), positions.size()));
  }
}

static void remove_shape_key_attributes(Mesh &mesh, const Key &key)
{
  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
  for (KeyBlock &kb : key.block) {
    attributes.remove(shape_key_attribute_name(kb));
  }
}

/**
 * Geometry nodes currently requires working on "evaluated" data-blocks (rather than "original"
 * data-blocks that are part of a #Main data-base). This could change in the future, but for now,
 * we need to create evaluated copies of geometry before passing it to geometry nodes. Implicit
 * sharing lets us avoid copying attribute data though.
 */
static bke::GeometrySet get_original_geometry_eval_copy(Depsgraph &depsgraph,
                                                        Object &object,
                                                        Vector<MeshState> &orig_mesh_states)
{
  switch (object.type) {
    case OB_CURVES: {
      Curves *curves = BKE_curves_copy_for_eval(id_cast<const Curves *>(object.data));
      return bke::GeometrySet::from_curves(curves);
    }
    case OB_POINTCLOUD: {
      PointCloud *points = BKE_pointcloud_copy_for_eval(id_cast<const PointCloud *>(object.data));
      return bke::GeometrySet::from_pointcloud(points);
    }
    case OB_MESH: {
      Mesh *mesh = id_cast<Mesh *>(object.data);

      if (mesh->runtime->edit_mesh && mesh->runtime->edit_mesh->bm) {
        /* Snapshot the mesh the user is editing. Do not free that #BMesh here: Grab and the
         * depsgraph still point at it until the tool result is written back. */
        Mesh *mesh_copy = BKE_mesh_new_nomain(0, 0, 0, 0);
        BMeshToMeshParams params{};
        params.calc_object_remap = false;
        BM_mesh_bm_to_me(nullptr, mesh->runtime->edit_mesh->bm, mesh_copy, &params);
        if (Key *key = mesh->key) {
          add_shape_keys_as_attributes(*mesh_copy, *key);
        }
        orig_mesh_states.append_as(*mesh_copy);
        return bke::GeometrySet::from_mesh(mesh_copy);
      }

      if (bke::pbvh::Tree *pbvh = bke::object::pbvh_get(object)) {
        /* Currently many sculpt mode operations do not tag normals dirty (see use of
         * #Mesh::tag_positions_changed_no_normals()), so access within geometry nodes cannot
         * know that normals are out of date and recalculate them. Update them here instead. */
        bke::pbvh::update_normals(depsgraph, object, *pbvh);
      }

      if (Key *key = mesh->key) {
        /* Add the shape key attributes to the original mesh before the evaluated copy. Applying
         * the result of the operator in sculpt mode uses the attributes on the original mesh to
         * detect which changes. */
        add_shape_keys_as_attributes(*mesh, *key);
      }

      Mesh *mesh_copy = BKE_mesh_copy_for_eval(*mesh);
      orig_mesh_states.append_as(*mesh_copy);
      return bke::GeometrySet::from_mesh(mesh_copy);
    }
    case OB_GREASE_PENCIL: {
      const GreasePencil *grease_pencil = id_cast<const GreasePencil *>(object.data);
      GreasePencil *grease_pencil_copy = BKE_grease_pencil_copy_for_eval(grease_pencil);
      grease_pencil_copy->runtime->eval_frame = int(DEG_get_ctime(&depsgraph));
      return bke::GeometrySet::from_grease_pencil(grease_pencil_copy);
    }
    default:
      return {};
  }
}

static void store_mesh_state_for_comparison(Object &object, Vector<MeshState> &orig_mesh_states)
{
  switch (object.type) {
    case OB_MESH: {
      Mesh *mesh = id_cast<Mesh *>(object.data);
      if (!mesh->runtime->edit_mesh) {
        orig_mesh_states.append_as(*mesh);
      }
      break;
    }
    default: {
      break;
    }
  }
}

static void get_geometry_active_indices(const Object &object,
                                        nodes::GeoNodesOperatorData &operator_data)
{
  switch (object.type) {
    case OB_MESH: {
      const Mesh *mesh = id_cast<const Mesh *>(object.data);
      if (std::shared_ptr<BMEditMesh> &em = mesh->runtime->edit_mesh) {
        operator_data.active_point_index = BM_mesh_active_vert_index_get(em->bm);
        operator_data.active_edge_index = BM_mesh_active_edge_index_get(em->bm);
        operator_data.active_face_index = BM_mesh_active_face_index_get(em->bm, false, true);
      }
      break;
    }
    case OB_GREASE_PENCIL: {
      const GreasePencil *grease_pencil = id_cast<const GreasePencil *>(object.data);
      if (const bke::greasepencil::Layer *active_layer = grease_pencil->get_active_layer()) {
        operator_data.active_layer_index = *grease_pencil->get_layer_index(*active_layer);
      }
      break;
    }
    default: {
      break;
    }
  }
}

static bool another_modal_operator_is_running(const bContext &C, const wmOperator &self);

static void store_result_geometry(const bContext &C,
                                  const wmOperator &op,
                                  const Depsgraph &depsgraph,
                                  Main &bmain,
                                  Scene &scene,
                                  Object &object,
                                  const RegionView3D *rv3d,
                                  bke::GeometrySet geometry)
{
  geometry.ensure_owns_direct_data();
  switch (object.type) {
    case OB_CURVES: {
      Curves &curves = *id_cast<Curves *>(object.data);
      Curves *new_curves = geometry.get_curves_for_write();
      if (!new_curves) {
        curves.geometry.wrap() = {};
        DEG_id_tag_update(&curves.id, ID_RECALC_GEOMETRY);
        break;
      }

      /* Anonymous attributes shouldn't be available on the applied geometry. */
      new_curves->geometry.wrap().attributes_for_write().remove_anonymous();

      curves.geometry.wrap() = std::move(new_curves->geometry.wrap());
      BKE_object_material_from_eval_data(&bmain, &object, &new_curves->id);
      DEG_id_tag_update(&curves.id, ID_RECALC_GEOMETRY);
      break;
    }
    case OB_POINTCLOUD: {
      PointCloud &points = *id_cast<PointCloud *>(object.data);
      PointCloud *new_points =
          geometry.get_component_for_write<bke::PointCloudComponent>().release();
      if (!new_points) {
        points.attribute_storage.wrap() = {};
        points.totpoint = 0;
        DEG_id_tag_update(&points.id, ID_RECALC_GEOMETRY);
        break;
      }

      /* Anonymous attributes shouldn't be available on the applied geometry. */
      new_points->attributes_for_write().remove_anonymous();

      BKE_object_material_from_eval_data(&bmain, &object, &new_points->id);
      BKE_pointcloud_nomain_to_pointcloud(new_points, &points);
      DEG_id_tag_update(&points.id, ID_RECALC_GEOMETRY);
      break;
    }
    case OB_MESH: {
      Mesh &mesh = *id_cast<Mesh *>(object.data);

      Mesh *new_mesh = geometry.get_component_for_write<bke::MeshComponent>().release();
      if (new_mesh) {
        /* Anonymous attributes shouldn't be available on the applied geometry. */
        new_mesh->attributes_for_write().remove_anonymous();
        BKE_object_material_from_eval_data(&bmain, &object, &new_mesh->id);
      }
      else {
        new_mesh = BKE_mesh_new_nomain(0, 0, 0, 0);
      }

      if (Key *key = mesh.key) {
        /* Copy the evaluated shape key attributes back to the key data-block. */
        store_attributes_to_shape_keys(*new_mesh, *key);
      }

      if (object.mode == OB_MODE_SCULPT) {
        sculpt_paint::store_mesh_from_eval(op, scene, depsgraph, rv3d, object, new_mesh);
        if (Key *key = mesh.key) {
          /* Make sure to free the shape key attributes after `sculpt_paint::store_mesh_from_eval`
           * because that uses the attributes to detect changes, for a critical performance
           * optimization when only changing specific attributes. */
          remove_shape_key_attributes(mesh, *key);
        }
      }
      else {
        if (Key *key = mesh.key) {
          /* Make sure to free the attributes before converting to #BMesh for edit mode; removing
           * attributes on #BMesh requires reallocating the dynamic AoS storage. */
          remove_shape_key_attributes(*new_mesh, *key);
        }
        if (object.mode == OB_MODE_EDIT) {
          if (another_modal_operator_is_running(C, op)) {
            /* Grab / Extrude still holds this #BMesh. Keep the user's edit. */
            BKE_id_free(nullptr, new_mesh);
          }
          else {
            EDBM_mesh_make_from_mesh(&object, new_mesh, scene.toolsettings->selectmode, true);
            BKE_editmesh_looptris_and_normals_calc(mesh.runtime->edit_mesh.get(),
                                                   BKE_editmesh_bmesh_get_for_write(&mesh));
            BKE_id_free(nullptr, new_mesh);
            DEG_id_tag_update(&mesh.id, ID_RECALC_GEOMETRY);
          }
        }
        else {
          BKE_mesh_nomain_to_mesh(new_mesh, &mesh, &object, false);
          DEG_id_tag_update(&mesh.id, ID_RECALC_GEOMETRY);
        }
      }

      break;
    }
    case OB_GREASE_PENCIL: {
      const int eval_frame = int(DEG_get_ctime(&depsgraph));

      GreasePencil &grease_pencil = *id_cast<GreasePencil *>(object.data);
      Vector<int> editable_layer_indices;
      for (const int layer_i : grease_pencil.layers().index_range()) {
        const bke::greasepencil::Layer &layer = grease_pencil.layer(layer_i);
        if (!layer.is_editable()) {
          continue;
        }
        editable_layer_indices.append(layer_i);
      }

      bool inserted_new_keyframe = false;
      for (const int layer_i : editable_layer_indices) {
        bke::greasepencil::Layer &layer = grease_pencil.layer(layer_i);
        /* TODO: For now, we always create a blank keyframe, but it might be good to expose this as
         * an option and allow to duplicate the previous key. */
        const bool duplicate_previous_key = false;
        ed::greasepencil::ensure_active_keyframe(
            scene, grease_pencil, layer, duplicate_previous_key, inserted_new_keyframe);
      }
      GreasePencil *new_grease_pencil =
          geometry.get_component_for_write<bke::GreasePencilComponent>().get_for_write();
      if (!new_grease_pencil) {
        /* Clear the Grease Pencil geometry. */
        for (const int layer_i : editable_layer_indices) {
          bke::greasepencil::Layer &layer = grease_pencil.layer(layer_i);
          if (bke::greasepencil::Drawing *drawing_orig = grease_pencil.get_drawing_at(layer,
                                                                                      eval_frame))
          {
            drawing_orig->strokes_for_write() = {};
            drawing_orig->tag_topology_changed();
          }
        }
      }
      else {
        IndexMaskMemory memory;
        const IndexMask editable_layers = IndexMask::from_indices(editable_layer_indices.as_span(),
                                                                  memory);
        ed::greasepencil::apply_eval_grease_pencil_data(
            *new_grease_pencil, eval_frame, editable_layers, grease_pencil);

        /* There might be layers with empty names after evaluation. Make sure to rename them. */
        bke::greasepencil::ensure_non_empty_layer_names(bmain, grease_pencil);
        BKE_object_material_from_eval_data(&bmain, &object, &new_grease_pencil->id);
      }

      DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
      if (inserted_new_keyframe) {
        WM_event_add_notifier(&C, NC_GPENCIL | NA_EDITED, nullptr);
      }
      break;
    }
    default:
      break;
  }
}

/**
 * Gather IDs used by the node group, and the node group itself if there are any. We need to use
 * *all* IDs because the only mechanism we have to replace the socket ID pointers with their
 * evaluated counterparts is evaluating the node group data-block itself.
 */
static void gather_node_group_ids(const bNodeTree &node_tree, Set<ID *> &ids)
{
  const int orig_size = ids.size();
  BLI_assert(node_tree.runtime->eval_dependencies);
  for (ID *id : node_tree.runtime->eval_dependencies->ids.values()) {
    ids.add(id);
  }
  if (ids.size() != orig_size) {
    /* Only evaluate the node group if it references data-blocks. In that case it needs to be
     * evaluated so that ID pointers are switched to point to evaluated data-blocks. */
    ids.add(const_cast<ID *>(&node_tree.id));
  }
}

static std::optional<ID_Type> socket_type_to_id_type(const eNodeSocketDatatype socket_type)
{
  switch (socket_type) {
    case SOCK_CUSTOM:
    case SOCK_FLOAT:
    case SOCK_VECTOR:
    case SOCK_INT_VECTOR:
    case SOCK_RGBA:
    case SOCK_SHADER:
    case SOCK_BOOLEAN:
    case SOCK_INT:
    case SOCK_STRING:
    case SOCK_GEOMETRY:
    case SOCK_ROTATION:
    case SOCK_MENU:
    case SOCK_MATRIX:
    case SOCK_BUNDLE:
    case SOCK_CLOSURE:
    case SOCK_COLOR_RAMP:
    case SOCK_CURVE:
      return std::nullopt;
    case SOCK_OBJECT:
      return ID_OB;
    case SOCK_IMAGE:
      return ID_IM;
    case SOCK_COLLECTION:
      return ID_GR;
    case SOCK_TEXTURE:
      return ID_TE;
    case SOCK_MATERIAL:
      return ID_MA;
    case SOCK_FONT:
      return ID_VF;
    case SOCK_SCENE:
      return ID_SCE;
    case SOCK_TEXT_ID:
      return ID_TXT;
    case SOCK_MASK:
      return ID_MSK;
    case SOCK_SOUND:
      return ID_SO;
  }
  return std::nullopt;
}

/**
 * Gather IDs referenced from node group input properties (the redo panel). In the end, the group
 * input properties will be copied to contain evaluated data-blocks from the active and/or an extra
 * depsgraph.
 */
static Map<std::string, ID *> gather_input_ids(const Main &bmain,
                                               const bNodeTree &node_group,
                                               const PointerRNA &properties_ptr)
{
  PointerRNA inputs_ptr = RNA_pointer_get(const_cast<PointerRNA *>(&properties_ptr), "inputs");

  Map<std::string, ID *> ids;
  for (const bNodeTreeInterfaceSocket *input : node_group.interface_inputs()) {
    const std::optional<ID_Type> id_type = socket_type_to_id_type(input->socket_typeinfo()->type);
    if (!id_type) {
      continue;
    }
    PointerRNA input_props_ptr = RNA_pointer_get(&inputs_ptr, input->identifier);
    std::string name = RNA_string_get(&input_props_ptr, "value");
    ID *id = BKE_libblock_find_name(&const_cast<Main &>(bmain), *id_type, name.c_str());
    if (!id) {
      continue;
    }
    ids.add(std::move(name), id);
  }
  return ids;
}

static Depsgraph *build_extra_depsgraph(const Depsgraph &depsgraph_active, const Set<ID *> &ids)
{
  Depsgraph *depsgraph = DEG_graph_new(DEG_get_bmain(&depsgraph_active),
                                       DEG_get_input_scene(&depsgraph_active),
                                       DEG_get_input_view_layer(&depsgraph_active),
                                       DEG_get_mode(&depsgraph_active));
  DEG_graph_build_from_ids(depsgraph, Vector<ID *>(ids.begin(), ids.end()));
  DEG_evaluate_on_refresh(depsgraph);
  return depsgraph;
}

static bool object_has_editable_data(const Main &bmain, const Object &object)
{
  if (!ELEM(object.type, OB_CURVES, OB_POINTCLOUD, OB_MESH, OB_GREASE_PENCIL)) {
    return false;
  }
  if (!BKE_id_is_editable(&bmain, static_cast<const ID *>(object.data))) {
    return false;
  }
  return true;
}

static Vector<Object *> gather_supported_objects(const bContext &C,
                                                 const Main &bmain,
                                                 const eObjectMode mode)
{
  Vector<Object *> objects;
  Set<const ID *> unique_object_data;

  auto handle_object = [&](Object *object) {
    if (object->mode != mode) {
      return;
    }
    if (!unique_object_data.add(static_cast<const ID *>(object->data))) {
      return;
    }
    if (!object_has_editable_data(bmain, *object)) {
      return;
    }
    objects.append(object);
  };

  if (mode == OB_MODE_OBJECT) {
    CTX_DATA_BEGIN (&C, Object *, object, selected_objects) {
      handle_object(object);
    }
    CTX_DATA_END;
  }
  else {
    const Main *bmain = CTX_data_main(&C);
    Scene *scene = CTX_data_scene(&C);
    ViewLayer *view_layer = CTX_data_view_layer(&C);
    View3D *v3d = CTX_wm_view3d(&C);
    Object *active_object = CTX_data_active_object(&C);
    if (v3d && active_object) {
      FOREACH_OBJECT_IN_MODE_BEGIN (bmain, scene, view_layer, v3d, active_object->type, mode, ob) {
        handle_object(ob);
      }
      FOREACH_OBJECT_IN_MODE_END;
    }
  }
  return objects;
}

/**
 * Input node values are stored as operator properties in order to support redoing from the redo
 * panel for a few reasons:
 *  1. Some data (like the mouse position) cannot be retrieved from the `exec` callback used for
 *     operator redo. Redo is meant to just call the operator again with the exact same properties.
 *  2. While adjusting an input in the redo panel, the user doesn't expect anything else to change.
 *     If we retrieve other data like the viewport transform on every execution, that won't be the
 *     case.
 * We use operator RNA properties instead of operator custom data because the custom data struct
 * isn't maintained for the redo `exec` call.
 */
static void store_input_node_values_rna_props(const bContext &C,
                                              wmOperator &op,
                                              const wmEvent &event)
{
  Scene *scene = CTX_data_scene(&C);
  /* NOTE: `region` and `rv3d` may be null when called from a script. */
  const ARegion *region = CTX_wm_region(&C);
  const RegionView3D *rv3d = CTX_wm_region_view3d(&C);
  const View3D *v3d = CTX_wm_view3d(&C);

  /* Mouse position node inputs. */
  RNA_int_set_array(op.ptr, "mouse_position", event.mval);
  RNA_int_set_array(
      op.ptr,
      "region_size",
      region ? int2(BLI_rcti_size_x(&region->winrct), BLI_rcti_size_y(&region->winrct)) : int2(0));

  /* 3D cursor node inputs. */
  const View3DCursor &cursor = scene->cursor;
  RNA_float_set_array(op.ptr, "cursor_position", cursor.location);
  math::Quaternion cursor_rotation = cursor.rotation();
  RNA_float_set_array(op.ptr, "cursor_rotation", &cursor_rotation.w);

  /* Viewport transform node inputs. */
  RNA_float_set_array(op.ptr,
                      "viewport_projection_matrix",
                      rv3d ? float4x4(rv3d->winmat).base_ptr() : float4x4::identity().base_ptr());
  RNA_float_set_array(op.ptr,
                      "viewport_view_matrix",
                      rv3d ? float4x4(rv3d->viewmat).base_ptr() : float4x4::identity().base_ptr());
  RNA_boolean_set(op.ptr, "viewport_is_perspective", rv3d ? bool(rv3d->is_persp) : true);
  RNA_float_set(op.ptr, "viewport_lens", v3d ? v3d->lens : 50.0f);
  RNA_float_set(op.ptr, "viewport_clip_start", v3d ? v3d->clip_start : 0.01f);
  RNA_float_set(op.ptr, "viewport_clip_end", v3d ? v3d->clip_end : 1000.0f);
}

struct DataPerZone {
  nodes::SimulationZoneBehavior behavior;
  std::optional<bke::bake::BakeValues> state;
};

class NodeOperatorBakeDataBlockMap : public bke::bake::BakeDataBlockMap {
 private:
  std::mutex mutex_;
  Map<bke::bake::BakeDataBlockID, uint32_t> session_uid_by_key_;
  Main &bmain_;

 public:
  NodeOperatorBakeDataBlockMap(Main &bmain) : bmain_(bmain) {}

  ID *lookup_or_remember_missing(const bke::bake::BakeDataBlockID &key) override
  {
    std::lock_guard lock{mutex_};
    const uint32_t session_uid = session_uid_by_key_.lookup_or_add_cb(key, [&]() -> uint32_t {
      ID *id = BKE_libblock_find_name_and_library(
          &bmain_, short(key.type), key.id_name.c_str(), key.lib_name.c_str());
      return id ? id->session_uid : MAIN_ID_SESSION_UID_UNSET;
    });
    if (session_uid == MAIN_ID_SESSION_UID_UNSET) {
      return nullptr;
    }
    return BKE_libblock_find_session_uid(&bmain_, short(key.type), session_uid);
  }

  void try_add(ID &id) override
  {
    ID *id_orig = (id.tag & ID_TAG_NO_MAIN) ? DEG_get_original(&id) : &id;
    std::lock_guard lock{mutex_};
    session_uid_by_key_.add_overwrite(bke::bake::BakeDataBlockID(*id_orig), id_orig->session_uid);
  }
};

struct NodeOperatorCustomData {
  double last_time;
  bool initialized = false;
  Array<bke::GeometrySet> geometry_orig;
  /** Geometry fed into the node group. Starts as #geometry_orig, then follows user edits. */
  Array<bke::GeometrySet> geometry_input;
  bool user_mesh_dirty = false;
  Array<Map<int, std::unique_ptr<DataPerZone>>> simulation_data_by_zone;
  NodeOperatorBakeDataBlockMap bake_data_block_map;
  wmTimer *timer = nullptr;
  NodeOperatorCustomData(Main &bmain)
      : last_time(BLI_time_now_seconds()), bake_data_block_map(bmain)
  {
  }
};

/**
 * The window manager rewrites events that the operator's modal keymap matches, so the original
 * event type has to be read from #wmEvent::prev_type in that case. Used for the keys that the
 * operator handles itself that don't necessarily have modal events.
 */
static wmEventType original_event_type(const wmEvent &event)
{
  return event.type == EVT_MODAL_MAP ? event.prev_type : event.type;
}

/**
 * \return The name of the event that Modal Event nodes use to react to this event, or none if the
 * event isn't part of the tool's modal keymap.
 */
/** Edit-mode operators such as Grab hold raw #BMesh pointers. Rebuilding the edit
 * mesh while they run frees those elements. */
static bool another_modal_operator_is_running(const bContext &C, const wmOperator &self)
{
  const wmWindow *win = CTX_wm_window(&C);
  if (win == nullptr || win->runtime == nullptr) {
    return false;
  }
  for (const wmEventHandler &handler : win->runtime->modalhandlers) {
    if (handler.type != WM_HANDLER_TYPE_OP) {
      continue;
    }
    const wmOperator *other = reinterpret_cast<const wmEventHandler_Op *>(&handler)->op;
    if (other != nullptr && other != &self) {
      return true;
    }
  }
  return false;
}

static std::optional<StringRef> modal_event_name_for_event(const wmOperatorType &ot,
                                                           const wmEvent &event)
{
  if (event.type != EVT_MODAL_MAP || !ot.modalkeymap || !ot.modalkeymap->modal_items) {
    return std::nullopt;
  }
  const auto *items = static_cast<const EnumPropertyItem *>(ot.modalkeymap->modal_items);
  /* The UI name rather than the identifier, since that's the name the Modal Event nodes store. */
  const char *name = nullptr;
  if (!RNA_enum_name(items, event.val, &name)) {
    return std::nullopt;
  }
  return StringRef(name);
}

class NodeOperatorSimulationParams : public nodes::GeoNodesSimulationParams {
 private:
  mutable std::mutex mutex_;
  mutable Map<int, nodes::SimulationZoneBehavior *> behaviors_;
  Map<int, std::unique_ptr<DataPerZone>> &data_by_zone_;
  bke::bake::BakeDataBlockMap &data_block_map_;

 public:
  float delta_time = 0.0f;

  NodeOperatorSimulationParams(Map<int, std::unique_ptr<DataPerZone>> &data_by_zone,
                               bke::bake::BakeDataBlockMap &data_block_map,
                               const float delta_time)
      : data_by_zone_{data_by_zone}, data_block_map_(data_block_map), delta_time(delta_time)
  {
  }

  nodes::SimulationZoneBehavior *get(const int zone_id) const override
  {
    std::lock_guard lock{mutex_};
    return behaviors_.lookup_or_add_cb(zone_id, [&]() {
      DataPerZone &data = *data_by_zone_.lookup_or_add_cb(
          zone_id,
          [&]() -> std::unique_ptr<DataPerZone> { return std::make_unique<DataPerZone>(); });
      if (data.state.has_value()) {
        auto input = nodes::sim_input::UseCache();
        input.values = std::move(*data.state);
        input.delta_time = this->delta_time;
        data.behavior.input = std::move(input);
      }
      else {
        data.behavior.input = nodes::sim_input::PassThrough();
      }

      data.behavior.output = nodes::sim_output::StoreNewState{
          [&data](bke::bake::BakeValues values) { data.state = std::move(values); }};
      data.behavior.data_block_map = &data_block_map_;
      return &data.behavior;
    });
  }
};

static void run_node_group_end(bContext &C, wmOperator &op)
{
  auto *op_data = static_cast<NodeOperatorCustomData *>(op.customdata);
  if (!op_data) {
    return;
  }
  if (op_data->timer) {
    WM_event_timer_remove(CTX_wm_manager(&C), op_data->timer->win, op_data->timer);
  }
  MEM_delete(op_data);
  op.customdata = nullptr;
}

static void store_geometries(bContext &C,
                             wmOperator &op,
                             Depsgraph &depsgraph_active,
                             const Span<Object *> objects,
                             const Span<bke::GeometrySet> geometries)
{
  Main *bmain = CTX_data_main(&C);
  Scene *scene = CTX_data_scene(&C);
  RegionView3D *rv3d = CTX_wm_region_view3d(&C);
  for (const int object_i : objects.index_range()) {
    Object &object = *objects[object_i];
    Vector<MeshState> orig_mesh_states;
    store_mesh_state_for_comparison(object, orig_mesh_states);
    store_result_geometry(
        C, op, depsgraph_active, *bmain, *scene, object, rv3d, geometries[object_i]);
    WM_event_add_notifier(&C, NC_GEOM | ND_DATA, object.data);
  }
}

static void run_node_group_cancel(bContext *C, wmOperator *op)
{
  run_node_group_end(*C, *op);
}

static wmOperatorStatus run_node_group_execute(bContext *C,
                                               wmOperator *op,
                                               const wmEvent *event,
                                               const bool is_invoke)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  Object *active_object = CTX_data_active_object(C);
  if (!active_object) {
    return OPERATOR_CANCELLED;
  }
  const eObjectMode mode = active_object->mode;

  if (!op->customdata) {
    op->customdata = MEM_new<NodeOperatorCustomData>(__func__, *bmain);
  }
  NodeOperatorCustomData &op_data = *static_cast<NodeOperatorCustomData *>(op->customdata);

  /* The event is null when the operator is redone from the redo panel or called from a script.
   * Then the previously stored properties are reused and the operator can't run modally. */
  const bool can_run_modal = event != nullptr;
  if (event) {
    store_input_node_values_rna_props(*C, *op, *event);
  }

  /* Only this operator's timer should trigger execution. */
  const bool is_timer_event = event && event->type == TIMER && op_data.timer &&
                              event->customdata == op_data.timer;

  const bNodeTree *node_tree_orig = get_node_group(*C, *op->type, op->reports);
  if (!node_tree_orig) {
    run_node_group_end(*C, *op);
    return OPERATOR_CANCELLED;
  }

  const Vector<Object *> objects = gather_supported_objects(*C, *bmain, mode);

  Depsgraph *depsgraph_active = CTX_data_ensure_evaluated_depsgraph(C);
  Set<ID *> extra_ids;
  gather_node_group_ids(*node_tree_orig, extra_ids);
  const Map<std::string, ID *> input_ids = gather_input_ids(*bmain, *node_tree_orig, *op->ptr);
  for (ID *id : input_ids.values()) {
    /* Skip IDs that are already fully evaluated in the active depsgraph. */
    if (!DEG_id_is_fully_evaluated(depsgraph_active, id)) {
      extra_ids.add(id);
    }
  }

  StringRef active_event;
  if (event) {
    const wmEventType type = original_event_type(*event);
    if (type == EVT_ESCKEY) {
      store_geometries(*C, *op, *depsgraph_active, objects, op_data.geometry_orig);
      run_node_group_cancel(C, op);
      return OPERATOR_CANCELLED;
    }
    if (op_data.timer && ISTIMER(type) && !is_timer_event) {
      /* Timers from other parts of Blender should not trigger another execution. */
      return OPERATOR_PASS_THROUGH;
    }
    if (const std::optional<StringRef> event_name = modal_event_name_for_event(*op->type, *event))
    {
      active_event = *event_name;
    }
    else if (!is_invoke && !is_timer_event && !ELEM(type, EVT_PADENTER, EVT_RETKEY)) {
      /* The event is not part of the tool's modal keymap, so no Modal Event node can react to it.
       * Evaluating the whole node group would just be wasted work. */
      return OPERATOR_PASS_THROUGH;
    }
  }

  const nodes::GeoNodesOperatorDepsgraphs depsgraphs{
      depsgraph_active,
      extra_ids.is_empty() ? nullptr : build_extra_depsgraph(*depsgraph_active, extra_ids),
  };

  const bNodeTree *node_tree = nullptr;
  if (depsgraphs.extra) {
    node_tree = DEG_get_evaluated(depsgraphs.extra, node_tree_orig);
  }
  else {
    node_tree = node_tree_orig;
  }
  node_tree->ensure_interface_cache();
  const Span<const bNodeTreeInterfaceSocket *> interface_inputs = node_tree->interface_inputs();
  const Span<const bNodeTreeInterfaceSocket *> interface_outputs = node_tree->interface_outputs();

  const nodes::GeometryNodesLazyFunctionGraphInfo *lf_graph_info =
      nodes::ensure_geometry_nodes_lazy_function_graph(*node_tree).get();
  if (lf_graph_info == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Cannot evaluate node group");
    run_node_group_end(*C, *op);
    return OPERATOR_CANCELLED;
  }

  if (!node_tree->group_output_node()) {
    BKE_report(op->reports, RPT_ERROR, "Node group must have a group output node");
    run_node_group_end(*C, *op);
    return OPERATOR_CANCELLED;
  }
  if (interface_outputs.is_empty() ||
      !STREQ(interface_outputs[0]->socket_type, "NodeSocketGeometry"))
  {
    BKE_report(op->reports, RPT_ERROR, "Node group's first output must be a geometry");
    run_node_group_end(*C, *op);
    return OPERATOR_CANCELLED;
  }

  PointerRNA inputs_ptr = RNA_pointer_get(op->ptr, "inputs");

  bke::OperatorComputeContext compute_context;
  Set<ComputeContextHash> verbose_log_contexts;
  GeoOperatorLog &eval_log = get_static_eval_log();
  eval_log.log = std::make_unique<nodes::eval_log::NodesEvalLog>();
  eval_log.node_group_name = node_tree->id.name + 2;
  find_verbose_log_contexts(*bmain, verbose_log_contexts);

  /* May be null if operator called from outside 3D view context. */
  Vector<MeshState> orig_mesh_states;

  if (!op_data.initialized) {
    op_data.geometry_orig.reinitialize(objects.size());
    for (const int object_i : objects.index_range()) {
      op_data.geometry_orig[object_i] = get_original_geometry_eval_copy(
          *depsgraph_active, *objects[object_i], orig_mesh_states);
    }
    op_data.simulation_data_by_zone.reinitialize(objects.size());
    op_data.geometry_input = op_data.geometry_orig;
    op_data.initialized = true;
  }

  if (op_data.user_mesh_dirty && !another_modal_operator_is_running(*C, *op)) {
    Vector<MeshState> edited_mesh_states;
    for (const int object_i : objects.index_range()) {
      Object &object = *objects[object_i];
      if (object.type != OB_MESH || object.mode != OB_MODE_EDIT) {
        continue;
      }
      op_data.geometry_input[object_i] = get_original_geometry_eval_copy(
          *depsgraph_active, object, edited_mesh_states);
      op_data.simulation_data_by_zone[object_i].clear();
    }
    op_data.user_mesh_dirty = false;
  }

  const double now = BLI_time_now_seconds();
  const float delta_time = float(now - op_data.last_time);
  op_data.last_time = now;

  /* The operator keeps running if one of the evaluations has an enabled Modal Timer or Modal Event
   * node, but the timer is only added when a Modal Timer node asks for it. */
  std::atomic<bool> modal_requested = false;
  std::atomic<bool> timer_requested = false;

  Array<bke::GeometrySet> output_geometries(objects.size());
  for (const int object_i : objects.index_range()) {
    Object &object = *objects[object_i];
    nodes::GeoNodesOperatorData operator_eval_data{};
    operator_eval_data.mode = mode;
    operator_eval_data.is_timer_event = is_timer_event;
    operator_eval_data.active_event = active_event;
    operator_eval_data.modal_requested = &modal_requested;
    operator_eval_data.timer_requested = &timer_requested;
    operator_eval_data.depsgraphs = &depsgraphs;
    operator_eval_data.self_object_orig = &object;
    operator_eval_data.scene_orig = scene;
    operator_eval_data.input_ids = &input_ids;
    RNA_int_get_array(op->ptr, "mouse_position", operator_eval_data.mouse_position);
    RNA_int_get_array(op->ptr, "region_size", operator_eval_data.region_size);
    RNA_float_get_array(op->ptr, "cursor_position", operator_eval_data.cursor_position);
    RNA_float_get_array(op->ptr, "cursor_rotation", &operator_eval_data.cursor_rotation.w);
    RNA_float_get_array(
        op->ptr, "viewport_projection_matrix", operator_eval_data.viewport_winmat.base_ptr());
    RNA_float_get_array(
        op->ptr, "viewport_view_matrix", operator_eval_data.viewport_viewmat.base_ptr());
    operator_eval_data.viewport_is_perspective = RNA_boolean_get(op->ptr,
                                                                 "viewport_is_perspective");
    Object viewport_camera{};
    bke::id::ID_Runtime viewport_camera_id_runtime{};
    bke::ObjectRuntime viewport_camera_runtime;
    Camera viewport_camera_data{};
    bke::id::ID_Runtime viewport_camera_data_id_runtime{};
    viewport_camera.id.runtime = &viewport_camera_id_runtime;
    viewport_camera.id.tag |= ID_TAG_LOCALIZED;
    viewport_camera.type = OB_CAMERA;
    viewport_camera.data = &viewport_camera_data.id;
    viewport_camera.runtime = &viewport_camera_runtime;
    viewport_camera_data.id.runtime = &viewport_camera_data_id_runtime;
    viewport_camera_data.id.tag |= ID_TAG_LOCALIZED;
    viewport_camera_runtime.object_to_world = math::invert(operator_eval_data.viewport_viewmat);
    viewport_camera_runtime.world_to_object = operator_eval_data.viewport_viewmat;
    viewport_camera_data.type = operator_eval_data.viewport_is_perspective ? CAM_PERSP : CAM_ORTHO;
    viewport_camera_data.lens = RNA_float_get(op->ptr, "viewport_lens");
    viewport_camera_data.clip_start = RNA_float_get(op->ptr, "viewport_clip_start");
    viewport_camera_data.clip_end = RNA_float_get(op->ptr, "viewport_clip_end");
    if (!operator_eval_data.viewport_is_perspective &&
        operator_eval_data.viewport_winmat[1][1] != 0.0f)
    {
      viewport_camera_data.ortho_scale = 2.0f / operator_eval_data.viewport_winmat[1][1];
    }
    operator_eval_data.viewport_camera = &viewport_camera;

    get_geometry_active_indices(object, operator_eval_data);

    NodeOperatorSimulationParams simulation_params(
        op_data.simulation_data_by_zone[object_i], op_data.bake_data_block_map, delta_time);

    nodes::GeoNodesCallData call_data{};
    call_data.operator_data = &operator_eval_data;
    call_data.simulation_params = &simulation_params;
    call_data.eval_log = eval_log.log.get();
    if (&object == active_object) {
      /* Only log values from the active object. */
      call_data.verbose_log_contexts = &verbose_log_contexts;
    }

    store_mesh_state_for_comparison(object, orig_mesh_states);

    const nodes::GeometryNodesLazyFunctionGraphInfo &lf_graph_info =
        *nodes::ensure_geometry_nodes_lazy_function_graph(*node_tree);
    const nodes::GeometryNodesGroupFunction &function = lf_graph_info.function;
    const lf::LazyFunction &lazy_function = *function.function;
    const int num_inputs = lazy_function.inputs().size();
    const int num_outputs = lazy_function.outputs().size();

    nodes::GeoNodesUserData user_data;
    user_data.call_data = &call_data;
    call_data.root_ntree = node_tree;

    user_data.compute_context = &compute_context;

    ResourceScope scope;
    LinearAllocator<> &allocator = scope.allocator();

    /* Prepare main inputs. */
    Array<GMutablePointer> param_inputs(num_inputs);
    for (const int i : interface_inputs.index_range()) {
      const bNodeTreeInterfaceSocket &interface_socket = *interface_inputs[i];
      const bke::bNodeSocketType *typeinfo = interface_socket.socket_typeinfo();
      const eNodeSocketDatatype socket_type = typeinfo ? typeinfo->type : SOCK_CUSTOM;
      if (socket_type == SOCK_GEOMETRY && i == 0) {
        bke::SocketValueVariant &value = scope.construct<bke::SocketValueVariant>();
        value = bke::SocketValueVariant::from(op_data.geometry_input[object_i]);
        param_inputs[function.inputs.main[0]] = &value;
        continue;
      }

      PointerRNA input_props_ptr = RNA_pointer_get(&inputs_ptr, interface_socket.identifier);
      bke::SocketValueVariant value = init_socket_cpp_value(
          &call_data, &input_props_ptr, *node_tree, interface_socket);
      param_inputs[function.inputs.main[i]] = &scope.construct<bke::SocketValueVariant>(
          std::move(value));
    }

    /* Prepare used-outputs inputs. */
    Array<bool> output_used_inputs(interface_outputs.size(), true);
    for (const int i : interface_outputs.index_range()) {
      param_inputs[function.inputs.output_usages[i]] = &output_used_inputs[i];
    }

    /* No anonymous attributes have to be propagated. */
    Array<bke::GeometryNodesReferenceSet> references_to_propagate(
        function.inputs.references_to_propagate.geometry_outputs.size());
    for (const int i : references_to_propagate.index_range()) {
      param_inputs[function.inputs.references_to_propagate.range[i]] = &references_to_propagate[i];
    }

    /* Prepare memory for output values. */
    Array<GMutablePointer> param_outputs(num_outputs);
    for (const int i : IndexRange(num_outputs)) {
      const lf::Output &lf_output = lazy_function.outputs()[i];
      const CPPType &type = *lf_output.type;
      void *buffer = allocator.allocate(type);
      param_outputs[i] = {type, buffer};
    }

    /* We want to evaluate the main outputs, but don't care about which inputs are used for now. */
    Array<lf::ValueUsage> param_output_usages(num_outputs);
    param_output_usages.as_mutable_span().slice(function.outputs.main).fill(lf::ValueUsage::Used);
    param_output_usages.as_mutable_span()
        .slice(function.outputs.input_usages)
        .fill(lf::ValueUsage::Unused);

    nodes::GeoNodesLocalUserData local_user_data(user_data);

    lf::Context lf_context(lazy_function.init_storage(allocator), &user_data, &local_user_data);
    Array<std::optional<lf::ValueUsage>> param_input_usages(num_inputs);
    Array<bool> param_set_outputs(num_outputs, false);
    lf::BasicParams lf_params{lazy_function,
                              param_inputs,
                              param_outputs,
                              param_input_usages,
                              param_output_usages,
                              param_set_outputs};
    {
      nodes::ScopedComputeContextTimer timer{lf_context};
      lazy_function.execute(lf_params, lf_context);
    }
    lazy_function.destruct_storage(lf_context.storage);

    output_geometries[object_i] =
        param_outputs[0].get<bke::SocketValueVariant>()->extract<bke::GeometrySet>();

    for (const int i : IndexRange(num_outputs)) {
      if (param_set_outputs[i]) {
        GMutablePointer &ptr = param_outputs[i];
        ptr.destruct();
      }
    }
  }

  if (another_modal_operator_is_running(*C, *op)) {
    op_data.user_mesh_dirty = true;
  }

  store_geometries(*C, *op, *depsgraph_active, objects, output_geometries);

  nodes::eval_log::NodeTreeLog &tree_log = eval_log.log->get_tree_log(compute_context.hash());
  tree_log.ensure_node_warnings(*bmain);
  for (const nodes::NodeWarning &warning : tree_log.all_warnings) {
    if (warning.type == nodes::NodeWarningType::Info) {
      BKE_report(op->reports, RPT_INFO, warning.message.c_str());
    }
    else {
      BKE_report(op->reports, RPT_WARNING, warning.message.c_str());
    }
  }

  wmWindow *window = CTX_wm_window(C);
  if (!can_run_modal || !modal_requested || !window) {
    run_node_group_end(*C, *op);
    return OPERATOR_FINISHED;
  }
  if (ELEM(original_event_type(*event), EVT_PADENTER, EVT_RETKEY)) {
    /* Hard-coded "end" key. Let the evaluation happen first though, unlike "escape." */
    run_node_group_end(*C, *op);
    return OPERATOR_FINISHED;
  }

  if (timer_requested && !op_data.timer) {
    op_data.timer = WM_event_timer_add(CTX_wm_manager(C), window, TIMER, 1.0 / 100.0);
  }
  else if (!timer_requested && op_data.timer) {
    WM_event_timer_remove(CTX_wm_manager(C), op_data.timer->win, op_data.timer);
    op_data.timer = nullptr;
  }
  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus run_node_group_exec(bContext *C, wmOperator *op)
{
  return run_node_group_execute(C, op, nullptr, false);
}

static wmOperatorStatus run_node_group_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  return run_node_group_execute(C, op, event, false);
}

static wmOperatorStatus run_node_group_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  const bNodeTree *node_tree = get_node_group(*C, *op->type, op->reports);
  if (!node_tree) {
    return OPERATOR_CANCELLED;
  }

  const wmOperatorStatus retval = run_node_group_execute(C, op, event, true);
  if (retval & OPERATOR_RUNNING_MODAL) {
    WM_event_add_modal_handler(C, op);
  }
  return retval;
}

static void run_node_group_ui(bContext *C, wmOperator *op)
{
  ui::Layout &layout = *op->layout;
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  Main *bmain = CTX_data_main(C);
  PointerRNA bmain_ptr = RNA_main_pointer_create(bmain);

  const bNodeTree *node_tree = get_node_group(*C, *op->type, nullptr);
  if (!node_tree) {
    return;
  }

  bke::OperatorComputeContext compute_context;
  GeoOperatorLog &eval_log = get_static_eval_log();

  nodes::eval_log::NodeTreeLog *tree_log = eval_log.log ? &eval_log.log->get_tree_log(
                                                              compute_context.hash()) :
                                                          nullptr;
  nodes::draw_geometry_nodes_operator_redo_ui(
      *C, *op, const_cast<bNodeTree &>(*node_tree), tree_log);
}

static bool run_node_ui_poll(wmOperatorType * /*ot*/, PointerRNA *ptr)
{
  bool result = false;
  RNA_STRUCT_BEGIN (ptr, prop) {
    int flag = RNA_property_flag(prop);
    if ((flag & PROP_HIDDEN) == 0) {
      result = true;
      break;
    }
  }
  RNA_STRUCT_END;
  return result;
}

static bool run_node_group_poll(bContext *C, wmOperatorType *ot)
{
  const auto &type_data = *static_cast<const OperatorTypeData *>(ot->custom_data.get());
  const Object *active_object = CTX_data_active_object(C);
  if (!active_object) {
    return false;
  }
  const GeometryNodeAssetTraitFlag flag = asset_flag_for_context(*active_object);
  if ((type_data.flag & flag) != flag) {
    return false;
  }
  return true;
}

static Array<EnumPropertyItem, 0> get_input_enum_items(const IDProperty &input_idprop)
{
  const IDProperty *items_idprop = IDP_GetPropertyFromGroup(&input_idprop, "items");
  if (!items_idprop || items_idprop->type != IDP_GROUP) {
    return {rna_enum_dummy_NULL_items[0]};
  }

  Vector<EnumPropertyItem> items;
  for (const IDProperty &item_idprop : items_idprop->data.group) {
    if (item_idprop.type != IDP_GROUP) {
      continue;
    }
    items.append(EnumPropertyItem{
        .value = IDP_group_lookup_int(item_idprop, "value").value_or(0),
        .identifier = item_idprop.name,
        .icon = ICON_NONE,
        .name = item_idprop.name,
        .description = IDP_group_lookup_string(item_idprop, "description").value_or("").c_str(),
    });
  }

  if (items.is_empty()) {
    return {rna_enum_dummy_NULL_items[0]};
  }

  items.append({0, nullptr, 0, nullptr, nullptr});

  return Array<EnumPropertyItem, 0>(items.as_span());
}

static void make_common_type_prop(StructRNA &srna,
                                  const EnumPropertyItem *items,
                                  const nodes::GeometryNodesInputType default_type)
{
  PropertyRNA *prop = RNA_def_enum(&srna, "type", items, int(default_type), "Input Type", "");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
}

static void make_common_attribute_name_prop(StructRNA &srna,
                                            const StringRefNull name,
                                            const StringRefNull description,
                                            const IDProperty &input_idprop)
{
  const std::optional<StringRefNull> default_name = IDP_group_lookup_string(
      input_idprop, "default_attribute_name");
  RNA_def_string(&srna,
                 "attribute_name",
                 default_name.has_value() ? default_name->c_str() : nullptr,
                 0,
                 name.c_str(),
                 description.c_str());
}

static void make_common_value_and_attribute_props(StructRNA &srna,
                                                  const StringRefNull name,
                                                  const StringRefNull description,
                                                  const IDProperty &input_idprop)
{
  make_common_type_prop(srna,
                        nodes::geometry_nodes_input_type_items_value_or_attribute,
                        nodes::GeometryNodesInputType::Value);
  make_common_attribute_name_prop(srna, name, description, input_idprop);
}

static void make_common_value_props(StructRNA &srna)
{
  make_common_type_prop(
      srna, nodes::geometry_nodes_input_type_items_value, nodes::GeometryNodesInputType::Value);
}

static StructRNA *get_input_socket_struct_rna(IDProperty &input_idprop,
                                              OperatorTypeData &type_data)
{
  const StringRefNull identifier = input_idprop.name;
  const std::optional<int> type = IDP_group_lookup_int(input_idprop, "type");
  if (!type) {
    return nullptr;
  }
  StructRNA *srna = RNA_def_struct_ptr(
      &RNA_blender_rna_get(), identifier.c_str(), RNA_PropertyGroup);
  BLI_assert(!RNA_struct_in_public_namespace(srna));
  type_data.generated_structs.append(srna);
  // RNA_def_struct_path_func_runtime(srna, rna_NodesModifierPropertyInput_path);
  const StringRefNull name = IDP_group_lookup_string(input_idprop, "name").value_or(identifier);
  const StringRefNull description =
      IDP_group_lookup_string(input_idprop, "description").value_or("");
  RNA_def_struct_ui_text(srna, name.c_str(), description.c_str());

  switch (eNodeSocketDatatype(*type)) {
    case SOCK_FLOAT: {
      PropertyRNA *prop = RNA_def_float(
          srna,
          "value",
          IDP_group_lookup_float(input_idprop, "default_value").value_or(0.0f),
          -FLT_MAX,
          FLT_MAX,
          name.c_str(),
          description.c_str(),
          IDP_group_lookup_float(input_idprop, "min").value_or(-FLT_MAX),
          IDP_group_lookup_float(input_idprop, "max").value_or(FLT_MAX));
      RNA_def_property_subtype(
          prop,
          PropertySubType(IDP_group_lookup_int(input_idprop, "subtype").value_or(PROP_NONE)));
      make_common_value_and_attribute_props(*srna, name, description, input_idprop);
      break;
    }
    case SOCK_VECTOR: {
      const int dimensions = IDP_group_lookup_int(input_idprop, "dimensions").value_or(3);
      std::optional<Span<float>> defaults = IDP_group_lookup_float_array(
          input_idprop, "default_value", dimensions);
      PropertyRNA *prop = RNA_def_float_array(
          srna,
          "value",
          dimensions,
          defaults ? defaults->data() : nullptr,
          -FLT_MAX,
          FLT_MAX,
          name.c_str(),
          description.c_str(),
          IDP_group_lookup_float(input_idprop, "min").value_or(-FLT_MAX),
          IDP_group_lookup_float(input_idprop, "max").value_or(FLT_MAX));
      RNA_def_property_subtype(
          prop,
          PropertySubType(IDP_group_lookup_int(input_idprop, "subtype").value_or(PROP_NONE)));
      make_common_value_and_attribute_props(*srna, name, description, input_idprop);
      break;
    }
    case SOCK_RGBA: {
      std::optional<Span<float>> defaults = IDP_group_lookup_float_array(
          input_idprop, "default_value", 4);
      PropertyRNA *prop = RNA_def_float_color(srna,
                                              "value",
                                              4,
                                              defaults ? defaults->data() : nullptr,
                                              -FLT_MAX,
                                              FLT_MAX,
                                              name.c_str(),
                                              description.c_str(),
                                              0.0f,
                                              1.0f);
      RNA_def_property_subtype(prop, PROP_COLOR);
      make_common_value_and_attribute_props(*srna, name, description, input_idprop);
      break;
    }
    case SOCK_BOOLEAN: {
      RNA_def_boolean(srna,
                      "value",
                      IDP_group_lookup_bool(input_idprop, "default_value").value_or(false),
                      name.c_str(),
                      description.c_str());
      make_common_type_prop(*srna,
                            nodes::geometry_nodes_input_type_items_value_or_attribute_or_layer,
                            nodes::GeometryNodesInputType::Value);
      make_common_attribute_name_prop(*srna, name, description, input_idprop);
      break;
    }
    case SOCK_INT: {
      PropertyRNA *prop = RNA_def_int(
          srna,
          "value",
          IDP_group_lookup_int(input_idprop, "default_value").value_or(0),
          INT_MIN,
          INT_MAX,
          name.c_str(),
          description.c_str(),
          IDP_group_lookup_int(input_idprop, "min").value_or(INT_MIN),
          IDP_group_lookup_int(input_idprop, "max").value_or(INT_MIN));
      RNA_def_property_subtype(
          prop,
          PropertySubType(IDP_group_lookup_int(input_idprop, "subtype").value_or(PROP_NONE)));
      make_common_value_and_attribute_props(*srna, name, description, input_idprop);
      break;
    }
    case SOCK_STRING: {
      const StringRefNull default_value =
          IDP_group_lookup_string(input_idprop, "default_value").value_or("");
      PropertyRNA *prop = RNA_def_string(srna,
                                         "value",
                                         default_value.is_empty() ? nullptr :
                                                                    default_value.c_str(),
                                         0,
                                         name.c_str(),
                                         description.c_str());
      RNA_def_property_subtype(
          prop,
          PropertySubType(IDP_group_lookup_int(input_idprop, "subtype").value_or(PROP_NONE)));
      make_common_value_props(*srna);
      break;
    }
    case SOCK_IMAGE:
    case SOCK_COLLECTION:
    case SOCK_MATERIAL:
    case SOCK_FONT:
    case SOCK_SOUND:
    case SOCK_OBJECT: {
      RNA_def_string(srna, "value", nullptr, 0, name.c_str(), description.c_str());
      make_common_value_props(*srna);
      break;
    }
    case SOCK_ROTATION: {
      std::optional<Span<float>> defaults = IDP_group_lookup_float_array(
          input_idprop, "default_value", 3);
      RNA_def_float_rotation(srna,
                             "value",
                             3,
                             defaults ? defaults->data() : nullptr,
                             -FLT_MAX,
                             FLT_MAX,
                             name.c_str(),
                             description.c_str(),
                             -FLT_MAX,
                             FLT_MAX);
      make_common_value_and_attribute_props(*srna, name, description, input_idprop);
      break;
    }
    case SOCK_MENU: {
      type_data.enum_item_storage.append_as(get_input_enum_items(input_idprop));
      int default_value = IDP_group_lookup_int(input_idprop, "default_value").value_or(0);
      if (std::ranges::none_of(
              type_data.enum_item_storage.last(),
              [&](const EnumPropertyItem &item) { return item.value == default_value; }))
      {
        /* Default value must be used by one of the enum items. */
        default_value = 0;
      }
      RNA_def_enum(srna,
                   "value",
                   type_data.enum_item_storage.last().data(),
                   IDP_group_lookup_int(input_idprop, "default_value").value_or(0),
                   name.c_str(),
                   description.c_str());
      make_common_value_props(*srna);
      break;
    }
    default:
      break;
  }

  return srna;
}

static StructRNA *create_inputs_srna(const IDProperty &properties, OperatorTypeData &type_data)
{
  StructRNA *srna = RNA_def_struct_ptr(
      &RNA_blender_rna_get(), "GeometryNodesInterfaceInputs", RNA_PropertyGroup);
  BLI_assert(!RNA_struct_in_public_namespace(srna));
  type_data.generated_structs.append(srna);

  const IDProperty &inputs_props = *IDP_GetPropertyFromGroup(&properties, "inputs");

  for (IDProperty &input_idprop : inputs_props.data.group) {
    if (input_idprop.type != IDP_GROUP) {
      continue;
    }
    StructRNA *input_srna = get_input_socket_struct_rna(input_idprop, type_data);
    if (!input_srna) {
      continue;
    }
    BLI_assert(!RNA_struct_in_public_namespace(srna));
    RNA_def_pointer_runtime(srna,
                            input_idprop.name,
                            input_srna,
                            RNA_struct_ui_name(input_srna),
                            RNA_struct_ui_description(input_srna));
  }
  return srna;
}

static StructRNA *create_panels_srna(const IDProperty &properties,
                                     Vector<StructRNA *> &r_generated)
{
  const IDProperty *panels_props = IDP_GetPropertyFromGroup(&properties, "panels");
  if (!panels_props || panels_props->type != IDP_GROUP) {
    return nullptr;
  }
  StructRNA *srna = RNA_def_struct_ptr(
      &RNA_blender_rna_get(), "GeometryNodesInterfacePanels", RNA_PropertyGroup);
  BLI_assert(!RNA_struct_in_public_namespace(srna));
  r_generated.append(srna);
  for (IDProperty &panel_prop : panels_props->data.group) {
    if (panel_prop.type != IDP_BOOLEAN) {
      continue;
    }
    RNA_def_boolean(srna, panel_prop.name, IDP_bool_get(&panel_prop), "Is Open", "");
  }
  return srna;
}

/**
 * The events and their default key bindings are read back from the asset meta-data properties,
 * because those are available for both local node groups and assets that aren't loaded. Since the
 * properties are part of #OperatorTypeData::hash, the modal keymap is only rebuilt when the events
 * or their defaults actually changed.
 */
static std::string identifier_from_event_name(const StringRef name)
{
  std::string identifier;
  identifier.reserve(name.size());
  for (const char c : name) {
    identifier.push_back(std::isalnum(uchar(c)) ? c : '_');
  }
  if (!identifier.empty() && std::isdigit(uchar(identifier[0]))) {
    identifier.insert(identifier.begin(), '_');
  }
  return identifier;
}

/** Build the modal keymap's item array from the events listed in the meta-data properties. */
static void ensure_modal_keymap_items(OperatorTypeData &type_data)
{
  const IDProperty *events = IDP_GetPropertyFromGroup(type_data.asset_meta_data_properties.get(),
                                                      "modal_events");
  if (!events || events->type != IDP_GROUP) {
    return;
  }
  Vector<const IDProperty *> event_props;
  Vector<std::string> identifiers;
  for (const IDProperty &event : events->data.group) {
    if (event.type != IDP_STRING) {
      continue;
    }
    std::string identifier = identifier_from_event_name(event.name);
    if (identifier.empty()) {
      continue;
    }
    if (identifiers.contains(identifier)) {
      /* Two names that only differ in characters that aren't part of an identifier end up with
       * the same identifier. Only the first of them can be bound to a key. */
      continue;
    }
    identifiers.append(std::move(identifier));
    event_props.append(&event);
  }
  if (identifiers.is_empty()) {
    return;
  }

  type_data.modal_keymap_item_identifiers = Array<std::string, 0>(identifiers.as_span());

  Vector<EnumPropertyItem> items;
  for (const int i : event_props.index_range()) {
    items.append({i,
                  type_data.modal_keymap_item_identifiers[i].c_str(),
                  0,
                  event_props[i]->name,
                  IDP_string_get(event_props[i])});
  }
  items.append({0, nullptr, 0, nullptr, nullptr});
  type_data.modal_keymap_items = Array<EnumPropertyItem, 0>(items.as_span());
}

static void add_default_modal_keymap_items(wmKeyMap &keymap,
                                           const IDProperty &properties,
                                           const Span<EnumPropertyItem> items)
{
  const IDProperty *keymap_props = IDP_GetPropertyFromGroup(&properties, "modal_keymap_default");
  if (!keymap_props || keymap_props->type != IDP_GROUP) {
    return;
  }
  for (const IDProperty &item_props : keymap_props->data.group) {
    if (item_props.type != IDP_GROUP) {
      continue;
    }
    const std::optional<StringRefNull> event_name = IDP_group_lookup_string(item_props,
                                                                            "event_name");
    if (!event_name) {
      continue;
    }
    /* The default keymap can be out of date when the Modal Event node that defined an event has
     * been removed or renamed. That's not an error, the binding is just skipped. */
    const std::string identifier = identifier_from_event_name(*event_name);
    int propvalue;
    if (!RNA_enum_value_from_id(items.data(), identifier.c_str(), &propvalue)) {
      continue;
    }
    KeyMapItem_Params params{};
    params.type = IDP_group_lookup_int(item_props, "type").value_or(0);
    params.value = IDP_group_lookup_int(item_props, "val").value_or(0);
    params.direction = IDP_group_lookup_int(item_props, "direction").value_or(0);
    params.modifier = 0;
    if (IDP_group_lookup_int(item_props, "shift").value_or(0)) {
      params.modifier |= KM_SHIFT;
    }
    if (IDP_group_lookup_int(item_props, "ctrl").value_or(0)) {
      params.modifier |= KM_CTRL;
    }
    if (IDP_group_lookup_int(item_props, "alt").value_or(0)) {
      params.modifier |= KM_ALT;
    }
    if (IDP_group_lookup_int(item_props, "oskey").value_or(0)) {
      params.modifier |= KM_OSKEY;
    }
    if (IDP_group_lookup_int(item_props, "hyper").value_or(0)) {
      params.modifier |= KM_HYPER;
    }
    params.keymodifier = IDP_group_lookup_int(item_props, "keymodifier").value_or(0);
    /* Not #WM_modalkeymap_add_item_str: that stores the name for #wm_user_modal_keymap_set_items
     * to resolve later, but it only does that for keymaps that don't have their items yet. */
    wmKeyMapItem &kmi = *WM_modalkeymap_add_item(&keymap, &params, propvalue);
    if (!IDP_group_lookup_int(item_props, "repeat").value_or(0)) {
      kmi.flag |= KMI_REPEAT_IGNORE;
    }
  }
}

/**
 * Create the modal keymap of a node tool and fill it with the default bindings its node groups
 * define. Called by the window manager whenever the default key configuration is created or
 * re-created, and directly when the node group changed.
 */
static void node_tool_modal_keymap_ensure(wmOperatorType *ot, wmKeyConfig *keyconf)
{
  auto &type_data = static_cast<OperatorTypeData &>(*ot->custom_data);
  if (type_data.modal_keymap_items.is_empty()) {
    ensure_modal_keymap_items(type_data);
    if (type_data.modal_keymap_items.is_empty()) {
      return;
    }
  }
  /* The keymap name is its identity, so the operator identifier is used rather than the tool name,
   * which isn't unique. */
  wmKeyMap *keymap = WM_modalkeymap_ensure(
      keyconf, type_data.idname.c_str(), type_data.modal_keymap_items.data());
  /* Not #WM_modalkeymap_assign, because that looks the operator type up by name and it is only
   * added to the map after the registration callback returns (see #WM_operatortype_append_ptr). */
  ot->modalkeymap = keymap;

  /* #WM_modalkeymap_ensure keeps the items of a keymap that already exists, so replace them
   * instead of adding a second copy of every binding. */
  WM_keymap_clear(keymap);
  add_default_modal_keymap_items(
      *keymap, *type_data.asset_meta_data_properties, type_data.modal_keymap_items.as_span());
}

static void register_node_tool(wmOperatorType *ot,
                               std::unique_ptr<OperatorTypeData> &type_data_ptr)
{
  OperatorTypeData &type_data = *type_data_ptr;
  ot->custom_data = std::move(type_data_ptr);

  PropertyRNA *prop;
  ot->name = type_data.name.c_str();
  ot->idname = type_data.idname.c_str();
  ot->description = type_data.description.empty() ? nullptr : type_data.description.c_str();

  ot->pyop_poll = run_node_group_poll;
  ot->invoke = run_node_group_invoke;
  ot->exec = run_node_group_exec;
  ot->modal = run_node_group_modal;
  ot->cancel = run_node_group_cancel;
  ot->ui = run_node_group_ui;
  ot->ui_poll = run_node_ui_poll;
  ot->modal_keymap_ensure = node_tool_modal_keymap_ensure;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_NODE_TOOL;
  if (type_data.flag & GEO_NODE_ASSET_WAIT_FOR_CURSOR) {
    ot->flag |= OPTYPE_DEPENDS_ON_CURSOR;
  }

  StructRNA *inputs_srna = create_inputs_srna(*type_data.asset_meta_data_properties, type_data);
  RNA_def_pointer_runtime(ot->srna, "inputs", inputs_srna, "Inputs", "Settings for input sockets");
  if (StructRNA *panels_srna = create_panels_srna(*type_data.asset_meta_data_properties,
                                                  type_data.generated_structs))
  {
    RNA_def_pointer_runtime(ot->srna, "panels", panels_srna, "Panels", "Settings for panels");
  }

  /* See comment for #store_input_node_values_rna_props. */
  prop = RNA_def_int_array(ot->srna,
                           "mouse_position",
                           2,
                           nullptr,
                           INT_MIN,
                           INT_MAX,
                           "Mouse Position",
                           "Mouse coordinates in region space",
                           INT_MIN,
                           INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
  prop = RNA_def_int_array(
      ot->srna, "region_size", 2, nullptr, 0, INT_MAX, "Region Size", "", 0, INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
  prop = RNA_def_float_array(ot->srna,
                             "cursor_position",
                             3,
                             nullptr,
                             -FLT_MAX,
                             FLT_MAX,
                             "3D Cursor Position",
                             "",
                             -FLT_MAX,
                             FLT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
  prop = RNA_def_float_array(ot->srna,
                             "cursor_rotation",
                             4,
                             nullptr,
                             -FLT_MAX,
                             FLT_MAX,
                             "3D Cursor Rotation",
                             "",
                             -FLT_MAX,
                             FLT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
  prop = RNA_def_float_array(ot->srna,
                             "viewport_projection_matrix",
                             16,
                             nullptr,
                             -FLT_MAX,
                             FLT_MAX,
                             "Viewport Projection Transform",
                             "",
                             -FLT_MAX,
                             FLT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
  prop = RNA_def_float_array(ot->srna,
                             "viewport_view_matrix",
                             16,
                             nullptr,
                             -FLT_MAX,
                             FLT_MAX,
                             "Viewport View Transform",
                             "",
                             -FLT_MAX,
                             FLT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
  prop = RNA_def_boolean(
      ot->srna, "viewport_is_perspective", false, "Viewport Is Perspective", "");
  RNA_def_property_flag(prop, PROP_HIDDEN);
  prop = RNA_def_float(ot->srna, "viewport_lens", 50.0f, 0.0f, FLT_MAX, "Viewport Lens", "", 0.0f, FLT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
  prop = RNA_def_float(ot->srna,
                       "viewport_clip_start",
                       0.01f,
                       0.0f,
                       FLT_MAX,
                       "Viewport Clip Start",
                       "",
                       0.0f,
                       FLT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
  prop = RNA_def_float(ot->srna,
                       "viewport_clip_end",
                       1000.0f,
                       0.0f,
                       FLT_MAX,
                       "Viewport Clip End",
                       "",
                       0.0f,
                       FLT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN);
}

void ui_template_node_operator_registration_errors(ui::Layout &layout,
                                                   const StringRefNull idname_py)
{
  const RegistrationData::Errors &errors = get_registration_data().errors_by_idname;
  const ErrorsForType *errors_for_type = errors.lookup_ptr(idname_py);
  if (!errors_for_type) {
    return;
  }
  ui::Layout &col = layout.column(false);
  if (errors_for_type->is_builtin_operator) {
    col.label(TIP_("Operator is already registered"), ICON_STATUS_ERROR);
  }
  if (errors_for_type->duplicate_count != 0) {
    col.label(fmt::format(fmt::runtime(TIP_("Duplicates: {}")), errors_for_type->duplicate_count),
              ICON_STATUS_ERROR);
  }
  for (const std::string &error : errors_for_type->idname_validation_errors) {
    col.label(error, ICON_STATUS_ERROR);
  }
}

static RegistrationData::TypeTreeItem *find_tree_child(
    const Span<std::unique_ptr<RegistrationData::TypeTreeItem>> children, const StringRef name)
{
  for (const std::unique_ptr<RegistrationData::TypeTreeItem> &child : children) {
    if (child->name == name) {
      return child.get();
    }
  }
  return nullptr;
}

static const RegistrationData::TypeTreeItem *find_tree_node(
    const Span<std::unique_ptr<RegistrationData::TypeTreeItem>> nodes, StringRef path)
{
  BLI_assert(!path.is_empty());
  const int64_t sep = path.find_first_of('/');
  if (sep == StringRef::not_found) {
    return find_tree_child(nodes, path);
  }
  const RegistrationData::TypeTreeItem *child = find_tree_child(nodes, path.substr(0, sep));
  if (!child) {
    return nullptr;
  }
  return find_tree_node(child->children, path.substr(sep + 1));
}

static RegistrationData::TypeTreeItem &ensure_child(
    Vector<std::unique_ptr<RegistrationData::TypeTreeItem>> &children, const StringRef name)
{
  if (RegistrationData::TypeTreeItem *existing = find_tree_child(children, name)) {
    return *existing;
  }
  auto new_item = std::make_unique<RegistrationData::TypeTreeItem>();
  new_item->name = name;
  RegistrationData::TypeTreeItem &ref = *new_item;
  children.append(std::move(new_item));
  return ref;
}

static void tree_add_type(Vector<std::unique_ptr<RegistrationData::TypeTreeItem>> &nodes,
                          StringRef path,
                          wmOperatorType *ot)
{
  BLI_assert(!path.is_empty());
  const int64_t sep = path.find_first_of('/');
  if (sep == StringRef::not_found) {
    RegistrationData::TypeTreeItem &child = ensure_child(nodes, path);
    child.types.append(ot);
    return;
  }
  RegistrationData::TypeTreeItem &child = ensure_child(nodes, path.substr(0, sep));
  tree_add_type(child.children, path.substr(sep + 1), ot);
}

struct GetIDName {
  StringRefNull operator()(const std::unique_ptr<OperatorTypeData> &value) const
  {
    return value->idname;
  }
};

using OperatorsToRegister = CustomIDVectorSet<std::unique_ptr<OperatorTypeData>, GetIDName>;

static OperatorsToRegister get_node_tools_type_data(const bContext &C,
                                                    Main &bmain,
                                                    RegistrationData::Errors &errors)
{
  OperatorsToRegister all_types;
  for (bNodeTree &ntree : bmain.nodetrees) {
    if (ID_IS_ASSET(&ntree.id)) {
      continue;
    }
    if (!ntree.geometry_node_asset_traits) {
      continue;
    }
    if ((ntree.geometry_node_asset_traits->flag & GEO_NODE_ASSET_TOOL) == 0) {
      continue;
    }
    std::optional<OperatorTypeData> type_data = OperatorTypeData::from_group(ntree, errors);
    if (!type_data) {
      continue;
    }
    if (!all_types.add(std::make_unique<OperatorTypeData>(std::move(*type_data)))) {
      errors.lookup_or_add_default_as(type_data->custom_idname).duplicate_count++;
    }
  }

  const AssetLibraryReference library_ref = asset_system::all_library_reference();
  ed::asset::list::storage_fetch(&library_ref, &C);
  if (ed::asset::list::library_get_once_available(library_ref)) {
    ed::asset::list::iterate(library_ref, [&](AssetRepresentation &asset) {
      if (asset.get_id_type() != ID_NT) {
        return true;
      }
      const AssetMetaData &meta_data = asset.get_metadata();
      const IDProperty *tree_type = BKE_asset_metadata_idprop_find(&meta_data, "type");
      if (tree_type == nullptr) {
        return true;
      }
      if (IDP_int_get(tree_type) != NTREE_GEOMETRY) {
        return true;
      }
      const IDProperty *traits_flag = BKE_asset_metadata_idprop_find(
          &meta_data, "geometry_node_asset_traits_flag");
      if (!traits_flag) {
        return true;
      }
      if (traits_flag->type != IDP_INT) {
        return true;
      }
      if ((IDP_int_get(traits_flag) & GEO_NODE_ASSET_TOOL) == 0) {
        return true;
      }
      std::optional<OperatorTypeData> type_data = OperatorTypeData::from_asset(asset, errors);
      if (!type_data) {
        return true;
      }
      if (!all_types.add(std::make_unique<OperatorTypeData>(std::move(*type_data)))) {
        errors.lookup_or_add_default_as(type_data->custom_idname).duplicate_count++;
      }
      return true;
    });
  }

  return all_types;
}

static void show_error_reports(const bContext &C, RegistrationData::Errors errors)
{
  wmWindowManager &wm = *CTX_wm_manager(&C);
  RegistrationData &registration_data = get_registration_data();
  if (errors == registration_data.errors_by_idname) {
    /* Don't display the same errors twice. That can be very noisy since this operator registration
     * process runs so often. */
    return;
  }
  ReportList *reports = CTX_wm_reports(&C);
  for (const RegistrationData::Errors::Item &item : errors.items()) {
    if (item.value.is_builtin_operator) {
      BKE_reportf(reports,
                  RPT_ERROR,
                  "Error registering node tool \"%s\", operator is already registered",
                  item.key.c_str());
    }
    if (item.value.duplicate_count != 0) {
      BKE_reportf(reports,
                  RPT_ERROR,
                  "Error registering node tool \"%s\", %d duplicate(s)",
                  item.key.c_str(),
                  item.value.duplicate_count);
    }
    if (item.value.invalid_metadata) {
      BKE_reportf(
          reports,
          RPT_ERROR,
          "Node tool \"%s\" asset has invalid metadata. Asset meta-data may be out of date",
          item.key.c_str());
    }
    for (const std::string &error : item.value.idname_validation_errors) {
      BKE_reportf(reports,
                  RPT_ERROR,
                  "Error registering node tool \"%s\", %s",
                  item.key.c_str(),
                  error.c_str());
    }
    for (const std::string &error : item.value.invalid_input_metadata_errors) {
      BKE_reportf(reports,
                  RPT_ERROR,
                  "Error registering node tool \"%s\". Invalid metadata for input \"%s\". "
                  "Asset meta-data may be out of date",
                  item.key.c_str(),
                  error.c_str());
    }
  }
  registration_data.errors_by_idname = std::move(errors);
  WM_report_banner_show(&wm, nullptr);
}

void register_node_group_operators(const bContext &C)
{
  PRF_scope(ProfileCategory::Core);
  wmWindowManager &wm = *CTX_wm_manager(&C);
  Main &bmain = *CTX_data_main(&C);
  RegistrationData &registration_data = get_registration_data();

  RegistrationData::Errors errors;

  OperatorsToRegister types_to_register = get_node_tools_type_data(C, bmain, errors);

  /* Remove node tools that conflict with builtin operators. */
  types_to_register.remove_if([&](const std::unique_ptr<OperatorTypeData> &item) {
    if (wmOperatorType *ot = WM_operatortype_find(item->idname.c_str(), true)) {
      if ((ot->flag & OPTYPE_NODE_TOOL) == 0) {
        errors.lookup_or_add_default(item->custom_idname).is_builtin_operator = true;
        return true;
      }
    }
    return false;
  });

  Set<wmOperatorType *> types_to_remove;

  /* Remove old operators for now-unused identifier names. */
  for (wmOperatorType *ot : WM_operatortypes_registered_get()) {
    if ((ot->flag & OPTYPE_NODE_TOOL) == 0) {
      continue;
    }
    if (types_to_register.contains_as(ot->idname)) {
      continue;
    }
    types_to_remove.add(ot);
  }

  /* Remove operators that are already registered and haven't changed. */
  types_to_register.remove_if([&](const std::unique_ptr<OperatorTypeData> &item) {
    if (wmOperatorType *ot = WM_operatortype_find(item->idname.c_str(), true)) {
      const auto &type_data = static_cast<const OperatorTypeData &>(*ot->custom_data);
      if (type_data.hash == item->hash) {
        return true;
      }
    }
    return false;
  });

  /* Remove changed operators so they can be re-registered. */
  for (const std::unique_ptr<OperatorTypeData> &type : types_to_register) {
    if (wmOperatorType *ot = WM_operatortype_find(type->idname.c_str(), true)) {
      types_to_remove.add_new(ot);
    }
  }

  show_error_reports(C, std::move(errors));

  if (types_to_remove.is_empty() && types_to_register.is_empty()) {
    return;
  }

  if (!types_to_remove.is_empty()) {
    WM_operator_stack_clear(&wm, types_to_remove);
    WM_operator_handlers_clear(&wm, types_to_remove);
    for (wmOperatorType *ot : types_to_remove) {
      auto &type_data = static_cast<OperatorTypeData &>(*ot->custom_data);

      for (StructRNA *srna : type_data.generated_structs) {
        /* Avoids warning when freeing the #StructRNA. */
        RNA_struct_py_type_set(srna, nullptr);
        RNA_struct_free(&RNA_blender_rna_get(), srna);
      }

      WM_operatortype_remove_ptr(ot);
    }
  }

  Vector<std::unique_ptr<OperatorTypeData>> vector = types_to_register.extract_vector();
  Vector<StringRefNull> registered_idnames;
  for (std::unique_ptr<OperatorTypeData> &type : vector) {
    registered_idnames.append(type->idname);
    WM_operatortype_append_ptr(
        [](wmOperatorType *ot, void *user_data) {
          register_node_tool(ot, *static_cast<std::unique_ptr<OperatorTypeData> *>(user_data));
        },
        &type);
  }

  /* Build the modal keymaps of the tools that were just registered. Only those, so that editing
   * one node group doesn't rebuild the keymaps of every other tool. */
  if (wm.runtime->defaultconf) {
    for (const StringRefNull idname : registered_idnames) {
      if (wmOperatorType *ot = WM_operatortype_find(idname.c_str(), true)) {
        WM_keyconfig_operator_modal_keymap_ensure(ot, wm.runtime->defaultconf);
      }
    }
  }

  registration_data.local_types.clear();
  registration_data.unassigned_types.clear();
  registration_data.menu_path_tree_roots.clear();
  const asset_system::AssetLibrary *all_library = asset::list::library_get_once_available(
      asset_system::all_library_reference());
  for (wmOperatorType *ot : WM_operatortypes_registered_get()) {
    if ((ot->flag & OPTYPE_NODE_TOOL) == 0) {
      continue;
    }
    const auto &type_data = static_cast<OperatorTypeData &>(*ot->custom_data);
    const auto *asset_ref = std::get_if<AssetWeakReference>(&type_data.group_ref);
    if (!asset_ref) {
      registration_data.local_types.append(ot);
      continue;
    }
    const asset_system::AssetRepresentation *asset = ed::asset::find_asset_from_weak_ref(
        C, *asset_ref, nullptr);
    BLI_assert(asset != nullptr);
    const bUUID catalog_id = asset->get_metadata().catalog_id;
    const asset_system::AssetCatalog *catalog = all_library->catalog_service().find_catalog(
        catalog_id);
    if (!catalog) {
      registration_data.unassigned_types.append(ot);
      continue;
    }
    tree_add_type(registration_data.menu_path_tree_roots, catalog->path.str(), ot);
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Menu
 * \{ */

static bool asset_menu_poll(const bContext *C, MenuType * /*mt*/)
{
  return CTX_wm_view3d(C);
}

static GeometryNodeAssetTraitFlag asset_flag_for_context(const ObjectType type,
                                                         const eObjectMode mode)
{
  switch (type) {
    case OB_MESH: {
      switch (mode) {
        case OB_MODE_OBJECT:
          return (GEO_NODE_ASSET_TOOL | GEO_NODE_ASSET_OBJECT | GEO_NODE_ASSET_MESH);
        case OB_MODE_EDIT:
          return (GEO_NODE_ASSET_TOOL | GEO_NODE_ASSET_EDIT | GEO_NODE_ASSET_MESH);
        case OB_MODE_SCULPT:
          return (GEO_NODE_ASSET_TOOL | GEO_NODE_ASSET_SCULPT | GEO_NODE_ASSET_MESH);
        default:
          break;
      }
      break;
    }
    case OB_CURVES: {
      switch (mode) {
        case OB_MODE_OBJECT:
          return (GEO_NODE_ASSET_TOOL | GEO_NODE_ASSET_OBJECT | GEO_NODE_ASSET_CURVE);
        case OB_MODE_EDIT:
          return (GEO_NODE_ASSET_TOOL | GEO_NODE_ASSET_EDIT | GEO_NODE_ASSET_CURVE);
        case OB_MODE_SCULPT_CURVES:
          return (GEO_NODE_ASSET_TOOL | GEO_NODE_ASSET_SCULPT | GEO_NODE_ASSET_CURVE);
        default:
          break;
      }
      break;
    }
    case OB_POINTCLOUD: {
      switch (mode) {
        case OB_MODE_OBJECT:
          return (GEO_NODE_ASSET_TOOL | GEO_NODE_ASSET_OBJECT | GEO_NODE_ASSET_POINTCLOUD);
        case OB_MODE_EDIT:
          return (GEO_NODE_ASSET_TOOL | GEO_NODE_ASSET_EDIT | GEO_NODE_ASSET_POINTCLOUD);
        default:
          break;
      }
      break;
    }
    case OB_GREASE_PENCIL: {
      switch (mode) {
        case OB_MODE_OBJECT:
          return (GEO_NODE_ASSET_TOOL | GEO_NODE_ASSET_OBJECT | GEO_NODE_ASSET_GREASE_PENCIL);
        case OB_MODE_EDIT:
          return (GEO_NODE_ASSET_TOOL | GEO_NODE_ASSET_EDIT | GEO_NODE_ASSET_GREASE_PENCIL);
        case OB_MODE_SCULPT_GREASE_PENCIL:
          return (GEO_NODE_ASSET_TOOL | GEO_NODE_ASSET_SCULPT | GEO_NODE_ASSET_GREASE_PENCIL);
        case OB_MODE_PAINT_GREASE_PENCIL:
          return (GEO_NODE_ASSET_TOOL | GEO_NODE_ASSET_PAINT | GEO_NODE_ASSET_GREASE_PENCIL);
        default:
          break;
      }
    }
    default:
      break;
  }
  return GeometryNodeAssetTraitFlag(0);
}

GeometryNodeAssetTraitFlag asset_flag_for_context(const Object &active_object)
{
  return asset_flag_for_context(active_object.type, active_object.mode);
}

/**
 * Avoid adding a separate root catalog when the assets have already been added to one of the
 * builtin menus. The need to define the builtin menu labels here is non-ideal. We don't have
 * any UI introspection that can do this though.
 */
static Set<StringRef> get_builtin_menus(const ObjectType object_type, const eObjectMode mode)
{
  Set<StringRef> menus;
  switch (object_type) {
    case OB_CURVES:
      menus.add_new("View");
      menus.add_new("Select");
      menus.add_new("Curves");
      break;
    case OB_POINTCLOUD:
      menus.add_new("View");
      menus.add_new("Select");
      menus.add_new("Point Cloud");
      break;
    case OB_MESH:
      switch (mode) {
        case OB_MODE_OBJECT:
          menus.add_new("View");
          menus.add_new("Select");
          menus.add_new("Add");
          menus.add_new("Object");
          menus.add_new("Object/Apply");
          menus.add_new("Object/Convert");
          menus.add_new("Object/Quick Effects");
          break;
        case OB_MODE_EDIT:
          menus.add_new("View");
          menus.add_new("Select");
          menus.add_new("Add");
          menus.add_new("Mesh");
          menus.add_new("Mesh/Extrude");
          menus.add_new("Mesh/Clean Up");
          menus.add_new("Mesh/Delete");
          menus.add_new("Mesh/Merge");
          menus.add_new("Mesh/Normals");
          menus.add_new("Mesh/Shading");
          menus.add_new("Mesh/Split");
          menus.add_new("Mesh/Weights");
          menus.add_new("Vertex");
          menus.add_new("Edge");
          menus.add_new("Face");
          menus.add_new("Face/Face Data");
          menus.add_new("UV");
          menus.add_new("UV/Unwrap");
          break;
        case OB_MODE_SCULPT:
          menus.add_new("View");
          menus.add_new("Sculpt");
          menus.add_new("Mask");
          menus.add_new("Face Sets");
          break;
        case OB_MODE_VERTEX_PAINT:
          menus.add_new("View");
          menus.add_new("Paint");
          break;
        case OB_MODE_WEIGHT_PAINT:
          menus.add_new("View");
          menus.add_new("Weights");
          break;
        default:
          break;
      }
      break;
    case OB_GREASE_PENCIL: {
      switch (mode) {
        case OB_MODE_OBJECT:
          menus.add_new("View");
          menus.add_new("Select");
          menus.add_new("Add");
          menus.add_new("Object");
          menus.add_new("Object/Apply");
          menus.add_new("Object/Convert");
          menus.add_new("Object/Quick Effects");
          break;
        case OB_MODE_EDIT:
          menus.add_new("View");
          menus.add_new("Select");
          menus.add_new("Grease Pencil");
          menus.add_new("Stroke");
          menus.add_new("Point");
          break;
        case OB_MODE_SCULPT_GREASE_PENCIL:
          menus.add_new("View");
          break;
        case OB_MODE_PAINT_GREASE_PENCIL:
          menus.add_new("View");
          menus.add_new("Draw");
          break;
        default:
          break;
      }
    }
    default:
      break;
  }
  return menus;
}

static bool menu_operators_poll(const bContext &C, const RegistrationData::TypeTreeItem &node)
{
  if (std::ranges::any_of(
          node.types,
          [&](wmOperatorType *ot) { return WM_operator_poll(&const_cast<bContext &>(C), ot); }))
  {
    return true;
  }
  if (std::ranges::any_of(node.children,
                          [&](const std::unique_ptr<RegistrationData::TypeTreeItem> &child) {
                            return menu_operators_poll(C, *child);
                          }))
  {
    return true;
  }
  return false;
}

static void catalog_assets_draw(const bContext *C, Menu *menu)
{
  const Object *active_object = CTX_data_active_object(C);
  if (!active_object) {
    return;
  }
  const std::optional<StringRefNull> path = CTX_data_string_get(C, "asset_catalog_path");
  if (!path) {
    return;
  }
  const RegistrationData &data = get_registration_data();
  const RegistrationData::TypeTreeItem *node = find_tree_node(data.menu_path_tree_roots, *path);
  if (!node) {
    return;
  }

  ui::Layout &layout = *menu->layout;
  bool add_separator = true;

  for (wmOperatorType *ot : node->types) {
    if (!WM_operator_poll(const_cast<bContext *>(C), ot)) {
      continue;
    }
    if (add_separator) {
      layout.separator();
      add_separator = false;
    }
    layout.op(ot, std::nullopt, ICON_NONE, wm::OpCallContext::InvokeRegionWin, UI_ITEM_NONE);
  }

  const Set<StringRef> builtin_menus = get_builtin_menus(active_object->type, active_object->mode);

  for (const std::unique_ptr<RegistrationData::TypeTreeItem> &child : node->children) {
    if (!menu_operators_poll(*C, *child)) {
      continue;
    }
    const std::string child_path = fmt::format("{}/{}", *path, child->name);
    if (builtin_menus.contains_as(child_path)) {
      continue;
    }
    if (add_separator) {
      layout.separator();
      add_separator = false;
    }
    layout.context_string_set("asset_catalog_path", child_path);
    layout.menu("GEO_MT_node_operator_catalog_assets", IFACE_(child->name), ICON_NONE);
  }
}

MenuType node_group_operator_assets_menu()
{
  MenuType type{};
  STRNCPY_UTF8(type.idname, "GEO_MT_node_operator_catalog_assets");
  type.poll = asset_menu_poll;
  type.draw = catalog_assets_draw;
  type.listener = asset::list::asset_reading_region_listen_fn;
  type.flag = MenuTypeFlag::ContextDependent;
  return type;
}

static void catalog_assets_draw_unassigned(const bContext *C, Menu *menu)
{
  const RegistrationData &data = get_registration_data();
  ui::Layout &layout = *menu->layout;
  bool add_separator = false;
  for (wmOperatorType *ot : data.unassigned_types) {
    if (!WM_operator_poll(const_cast<bContext *>(C), ot)) {
      continue;
    }
    layout.op(ot, std::nullopt, ICON_NONE, wm::OpCallContext::InvokeRegionWin, UI_ITEM_NONE);
    add_separator = true;
  }

  bool first = true;
  for (wmOperatorType *ot : data.local_types) {
    if (!WM_operator_poll(const_cast<bContext *>(C), ot)) {
      continue;
    }
    if (add_separator) {
      layout.separator();
      add_separator = false;
    }
    if (first) {
      layout.label(IFACE_("Non-Assets"), ICON_NONE);
      first = false;
    }
    layout.op(ot, std::nullopt, ICON_NONE, wm::OpCallContext::InvokeRegionWin, UI_ITEM_NONE);
  }
}

MenuType node_group_operator_assets_menu_unassigned()
{
  MenuType type{};
  STRNCPY_UTF8(type.label, N_("Unassigned Node Tools"));
  STRNCPY_UTF8(type.idname, "GEO_MT_node_operator_unassigned");
  type.poll = asset_menu_poll;
  type.draw = catalog_assets_draw_unassigned;
  type.listener = asset::list::asset_reading_region_listen_fn;
  type.flag = MenuTypeFlag::ContextDependent;
  type.description = N_(
      "Tool node group assets not assigned to a catalog.\n"
      "Catalogs can be assigned in the Asset Browser");
  return type;
}

void ui_template_node_operator_asset_menu_items(ui::Layout &layout,
                                                const bContext &C,
                                                const StringRef path)
{
  const RegistrationData &data = get_registration_data();
  const RegistrationData::TypeTreeItem *node = find_tree_node(data.menu_path_tree_roots, path);
  if (!node) {
    return;
  }
  if (!menu_operators_poll(C, *node)) {
    return;
  }
  ui::Layout &col = layout.column(false);
  col.context_string_set("asset_catalog_path", path);
  col.menu_contents("GEO_MT_node_operator_catalog_assets");
}

void ui_template_node_operator_asset_root_items(ui::Layout &layout, const bContext &C)
{
  const Object *active_object = CTX_data_active_object(&C);
  if (!active_object) {
    return;
  }
  const RegistrationData &data = get_registration_data();
  const Set<StringRef> builtin_menus = get_builtin_menus(active_object->type, active_object->mode);
  for (const std::unique_ptr<RegistrationData::TypeTreeItem> &root : data.menu_path_tree_roots) {
    if (builtin_menus.contains_as(root->name)) {
      continue;
    }
    if (!menu_operators_poll(C, *root)) {
      continue;
    }
    layout.context_string_set("asset_catalog_path", root->name);
    layout.menu("GEO_MT_node_operator_catalog_assets", IFACE_(root->name), ICON_NONE);
  }

  if (!data.unassigned_types.is_empty() || !data.local_types.is_empty()) {
    layout.menu("GEO_MT_node_operator_unassigned", "", ICON_FILE_HIDDEN);
  }
}

/** \} */

}  // namespace blender::ed::geometry
