/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgeometry_edit
 *
 * Temp Object + native Edit Mode for Select Elements / Edit Elements.
 *
 * The edited component decides the object: a mesh, curves or point cloud object. Instances have
 * no edit mode, they are edited as a point cloud with one point per instance.
 *
 * Selecting whole curves or instances works on objects instead: every curve or instance becomes
 * a temporary object, and the objects are selected in Object Mode.
 */

#include "BKE_collection.hh"
#include "BKE_context.hh"
#include "BKE_curves.h"
#include "BKE_curves.hh"
#include "BKE_editmesh.hh"
#include "BKE_instances.hh"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_material.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"
#include "BKE_object.hh"
#include "BKE_pointcloud.hh"
#include "BKE_report.hh"
#include "BKE_scene.hh"
#include "BKE_screen.hh"

#include "BLI_array.hh"
#include "BLI_map.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"
#include "DEG_depsgraph_query.hh"

#include "DNA_collection_types.h"
#include "DNA_curves_types.h"
#include "DNA_layer_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_pointcloud_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_enums.h"
#include "DNA_space_types.h"
#include "DNA_view3d_types.h"
#include "DNA_windowmanager_types.h"

#include "ED_curves.hh"
#include "ED_geometry_edit.hh"
#include "ED_mesh.hh"
#include "ED_object.hh"
#include "ED_outliner.hh"
#include "ED_screen.hh"
#include "ED_select_utils.hh"
#include "ED_undo.hh"

#include "GEO_realize_instances.hh"

#include "MEM_guardedalloc.h"

#include "NOD_geo_elements_component.hh"

#include "WM_api.hh"
#include "WM_toolsystem.hh"
#include "WM_types.hh"

#include "bmesh.hh"

#include "geometry_edit_intern.hh"
#include "geometry_edit_view.hh"

namespace blender::ed::geometry_edit {

GeometryEditSession &session_state()
{
  static GeometryEditSession state;
  return state;
}

short domain_to_selectmode(const bke::AttrDomain domain)
{
  switch (domain) {
    case bke::AttrDomain::Edge:
      return SCE_SELECT_EDGE;
    case bke::AttrDomain::Face:
      return SCE_SELECT_FACE;
    case bke::AttrDomain::Point:
    default:
      return SCE_SELECT_VERTEX;
  }
}

bke::AttrDomain selectmode_to_domain(const short selectmode)
{
  if (selectmode & SCE_SELECT_FACE) {
    return bke::AttrDomain::Face;
  }
  if (selectmode & SCE_SELECT_EDGE) {
    return bke::AttrDomain::Edge;
  }
  return bke::AttrDomain::Point;
}

void apply_seed_selection(BMesh &bm, const bke::AttrDomain domain, const Span<int> indices)
{
  BM_mesh_elem_table_ensure(&bm, BM_VERT | BM_EDGE | BM_FACE);
  BM_mesh_elem_hflag_disable_all(&bm, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT, false);

  for (const int index : indices) {
    switch (domain) {
      case bke::AttrDomain::Point: {
        if (BMVert *v = BM_vert_at_index_find(&bm, index)) {
          BM_vert_select_set(&bm, v, true);
        }
        break;
      }
      case bke::AttrDomain::Edge: {
        if (BMEdge *e = BM_edge_at_index_find(&bm, index)) {
          BM_edge_select_set(&bm, e, true);
        }
        break;
      }
      case bke::AttrDomain::Face: {
        if (BMFace *f = BM_face_at_index_find(&bm, index)) {
          BM_face_select_set(&bm, f, true);
        }
        break;
      }
      default:
        break;
    }
  }
  BM_mesh_select_mode_flush(&bm);
}

void read_selection_indices(BMesh &bm, const bke::AttrDomain domain, Vector<int> &r_indices)
{
  r_indices.clear();
  BM_mesh_elem_table_ensure(&bm, BM_VERT | BM_EDGE | BM_FACE);
  switch (domain) {
    case bke::AttrDomain::Point: {
      BMVert *v;
      BMIter iter;
      int i;
      BM_ITER_MESH_INDEX (v, &iter, &bm, BM_VERTS_OF_MESH, i) {
        if (BM_elem_flag_test(v, BM_ELEM_SELECT)) {
          r_indices.append(i);
        }
      }
      break;
    }
    case bke::AttrDomain::Edge: {
      BMEdge *e;
      BMIter iter;
      int i;
      BM_ITER_MESH_INDEX (e, &iter, &bm, BM_EDGES_OF_MESH, i) {
        if (BM_elem_flag_test(e, BM_ELEM_SELECT)) {
          r_indices.append(i);
        }
      }
      break;
    }
    case bke::AttrDomain::Face: {
      BMFace *f;
      BMIter iter;
      int i;
      BM_ITER_MESH_INDEX (f, &iter, &bm, BM_FACES_OF_MESH, i) {
        if (BM_elem_flag_test(f, BM_ELEM_SELECT)) {
          r_indices.append(i);
        }
      }
      break;
    }
    default:
      break;
  }
}

void session_tag_redraw(bContext *C)
{
  WM_main_add_notifier(NC_SPACE | ND_SPACE_VIEW3D, nullptr);
  if (C) {
    tag_all_view3d_redraw(C);
  }
  WM_main_add_notifier(NC_GEOM | ND_SELECT, nullptr);
  WM_main_add_notifier(NC_SCENE | ND_MODE, nullptr);
}

/* -------------------------------------------------------------------- */
/** \name Edited data of the session
 * \{ */

static BMesh *session_edit_bmesh()
{
  GeometryEditSession &state = session_state();
  if (!state.temp_object || state.temp_object->type != OB_MESH) {
    return nullptr;
  }
  BMEditMesh *em = BKE_editmesh_from_object(state.temp_object);
  return em ? em->bm : nullptr;
}

static Mesh *session_mesh()
{
  GeometryEditSession &state = session_state();
  if (!state.temp_object || state.temp_object->type != OB_MESH) {
    return nullptr;
  }
  return reinterpret_cast<Mesh *>(state.temp_data);
}

static Curves *session_curves()
{
  GeometryEditSession &state = session_state();
  if (!state.temp_object || state.temp_object->type != OB_CURVES) {
    return nullptr;
  }
  return reinterpret_cast<Curves *>(state.temp_data);
}

static PointCloud *session_pointcloud()
{
  GeometryEditSession &state = session_state();
  if (!state.temp_object || state.temp_object->type != OB_POINTCLOUD) {
    return nullptr;
  }
  return reinterpret_cast<PointCloud *>(state.temp_data);
}

/** Selection of curves or a point cloud in Edit Mode, everything is selected without it. */
static Array<bool> read_selection_attribute(const bke::AttributeAccessor attributes,
                                            const bke::AttrDomain domain)
{
  const VArray<bool> selection = *attributes.lookup_or_default<bool>(".selection", domain, true);
  Array<bool> result(selection.size());
  selection.materialize(result);
  return result;
}

static void write_curves_selection(bke::CurvesGeometry &curves,
                                   const bke::AttrDomain domain,
                                   const Span<bool> selection)
{
  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  ed::curves::remove_selection_attributes(attributes);
  /* Also the handles of Bezier curves, they would count as selected without their attribute. */
  ed::curves::foreach_selection_attribute_writer(
      curves, domain, [&](bke::GSpanAttributeWriter &writer) {
        writer.span.typed<bool>().copy_from(selection);
      });
}

static void write_pointcloud_selection(PointCloud &pointcloud, const Span<bool> selection)
{
  bke::MutableAttributeAccessor attributes = pointcloud.attributes_for_write();
  attributes.remove(".selection");
  attributes.add<bool>(".selection",
                       bke::AttrDomain::Point,
                       bke::AttributeInitVArray(VArray<bool>::from_span(selection)));
}

static Array<bool> selection_from_indices(const Span<int> indices, const int size)
{
  Array<bool> selection(size, false);
  for (const int index : indices) {
    if (uint(index) < uint(size)) {
      selection[index] = true;
    }
  }
  return selection;
}

static void indices_from_selection(const Span<bool> selection, Vector<int> &r_indices)
{
  r_indices.clear();
  for (const int i : selection.index_range()) {
    if (selection[i]) {
      r_indices.append(i);
    }
  }
}

static bool positions_match(const Span<float3> a, const Span<float3> b)
{
  if (a.size() != b.size()) {
    return false;
  }
  constexpr float eps2 = 1e-12f;
  for (const int i : a.index_range()) {
    if (math::distance_squared(a[i], b[i]) > eps2) {
      return false;
    }
  }
  return true;
}

/** \} */

bool session_topology_ok()
{
  GeometryEditSession &state = session_state();
  if (state.elements_as_objects) {
    /* Nothing of the geometry can be changed through the objects. */
    return state.active;
  }
  if (!state.active || !state.temp_object) {
    return false;
  }
  if (const Curves *curves_id = session_curves()) {
    return curves_id->geometry.point_num == state.seed_verts &&
           curves_id->geometry.curve_num == state.seed_curves;
  }
  if (const PointCloud *pointcloud = session_pointcloud()) {
    return pointcloud->totpoint == state.seed_verts;
  }
  BMesh *bm = session_edit_bmesh();
  if (!bm) {
    return false;
  }
  return bm->totvert == state.seed_verts && bm->totedge == state.seed_edges &&
         bm->totface == state.seed_faces;
}

bool session_positions_ok()
{
  GeometryEditSession &state = session_state();
  if (state.kind != SessionKind::SelectOnly) {
    return true;
  }
  if (state.elements_as_objects) {
    return state.active;
  }
  if (!state.active || !state.temp_object) {
    return false;
  }
  if (const Curves *curves_id = session_curves()) {
    return positions_match(curves_id->geometry.wrap().positions(), state.seed_positions);
  }
  if (const PointCloud *pointcloud = session_pointcloud()) {
    return positions_match(pointcloud->positions(), state.seed_positions);
  }
  BMesh *bm = session_edit_bmesh();
  if (!bm || bm->totvert != state.seed_positions.size()) {
    return false;
  }
  constexpr float eps2 = 1e-12f;
  BMVert *v;
  BMIter iter;
  int i = 0;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    if (math::distance_squared(float3(v->co), state.seed_positions[i]) > eps2) {
      return false;
    }
    i++;
  }
  return true;
}

/**
 * Restore select-only rest positions without ED_undo_pop.
 * Undo-pop can step past the session (or hit an empty stack) and free the temp object while
 * #GeometryEditSession still holds pointers — that path caused access-violation crashes.
 */
static bool restore_seed_positions_in_edit_mesh()
{
  GeometryEditSession &state = session_state();
  BMesh *bm = session_edit_bmesh();
  if (!bm || bm->totvert != state.seed_positions.size()) {
    return false;
  }
  BMVert *v;
  BMIter iter;
  int i = 0;
  BM_ITER_MESH_INDEX (v, &iter, bm, BM_VERTS_OF_MESH, i) {
    const float3 p = state.seed_positions[i];
    v->co[0] = p.x;
    v->co[1] = p.y;
    v->co[2] = p.z;
  }
  BM_mesh_normals_update(bm);
  if (state.temp_object) {
    DEG_id_tag_update(&state.temp_object->id, ID_RECALC_GEOMETRY);
  }
  if (state.temp_data) {
    DEG_id_tag_update(state.temp_data, ID_RECALC_GEOMETRY);
  }
  return true;
}

/**
 * Curves and point clouds are edited in place, so a select-only session takes them back to how
 * they were at session start. The selection made so far is kept while it still fits.
 */
static void restore_seed_geometry(const bool keep_selection)
{
  GeometryEditSession &state = session_state();
  if (Curves *curves_id = session_curves()) {
    const Curves *seed = state.seed_geometry.get_curves();
    if (seed == nullptr) {
      return;
    }
    bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    const bke::AttrDomain domain = bke::AttrDomain(curves_id->selection_domain);
    std::optional<Array<bool>> selection;
    if (keep_selection) {
      selection = read_selection_attribute(curves.attributes(), domain);
    }
    curves = seed->geometry.wrap();
    if (selection) {
      write_curves_selection(curves, domain, *selection);
    }
  }
  else if (PointCloud *pointcloud = session_pointcloud()) {
    const PointCloud *seed = state.seed_geometry.get_pointcloud();
    if (seed == nullptr) {
      return;
    }
    std::optional<Array<bool>> selection;
    if (keep_selection) {
      selection = read_selection_attribute(pointcloud->attributes(), bke::AttrDomain::Point);
    }
    BKE_pointcloud_nomain_to_pointcloud(BKE_pointcloud_copy_for_eval(seed), pointcloud);
    if (selection) {
      write_pointcloud_selection(*pointcloud, *selection);
    }
  }
  else {
    return;
  }
  DEG_id_tag_update(state.temp_data, ID_RECALC_GEOMETRY);
  DEG_id_tag_update(&state.temp_object->id, ID_RECALC_GEOMETRY);
}

bool session_enforce_constraints(bContext *C)
{
  GeometryEditSession &state = session_state();
  if (state.elements_as_objects) {
    return state.active;
  }
  if (!state.active || !state.temp_object) {
    return false;
  }

  /* Full edit: no constraints — any Edit Mode operation is allowed. */
  if (state.kind == SessionKind::MeshEdit) {
    return true;
  }

  bool ok = session_topology_ok();
  if (state.kind == SessionKind::SelectOnly) {
    ok = ok && session_positions_ok();
  }

  if (ok) {
    return true;
  }

  if (state.temp_object->type != OB_MESH) {
    restore_seed_geometry(session_topology_ok());
    WM_global_report(RPT_WARNING, "Select Elements: selection only — edit was undone");
    return true;
  }

  /* Select-only: prefer restoring seed positions (no undo stack side effects). */
  if (state.kind == SessionKind::SelectOnly && session_topology_ok()) {
    if (restore_seed_positions_in_edit_mesh() && session_positions_ok()) {
      WM_global_report(RPT_WARNING, "Select Elements: selection only — transform was restored");
      return true;
    }
  }

  /* Topology changed (or position restore failed): try one mesh-edit undo step only when the
   * temp object is still the active edit object. */
  if (C) {
    Object *obact = CTX_data_active_object(C);
    if (obact == state.temp_object && (obact->mode & OB_MODE_EDIT)) {
      ED_undo_pop(C);
    }
  }

  /* Re-validate after possible undo — session pointers may be stale if undo destroyed the temp. */
  if (!session_state().active || !session_state().temp_object) {
    WM_global_report(RPT_ERROR, "Geometry Edit session lost after undo — re-enter with Enter");
    return false;
  }

  ok = session_topology_ok();
  if (state.kind == SessionKind::SelectOnly) {
    if (!session_positions_ok()) {
      restore_seed_positions_in_edit_mesh();
    }
    ok = session_topology_ok() && session_positions_ok();
  }

  if (ok) {
    WM_global_report(RPT_WARNING, "Select Elements: selection only — edit was undone");
    return true;
  }

  WM_global_report(RPT_ERROR, "Geometry Edit session invalid — cancel and re-enter");
  return false;
}

static void hide_view3d_gizmos(bContext *C, GeometryEditSession &state)
{
  state.gizmo_backups.clear();
  const bScreen *screen = CTX_wm_screen(C);
  if (!screen) {
    return;
  }
  for (const ScrArea &area : screen->areabase) {
    if (area.spacetype != SPACE_VIEW3D) {
      continue;
    }
    View3D *v3d = area.spacedata.first_as<View3D>();
    if (!v3d) {
      continue;
    }
    View3DGizmoBackup bak;
    bak.v3d = v3d;
    bak.gizmo_flag = eView3D_GizmoFlag(v3d->gizmo_flag);
    state.gizmo_backups.append(bak);
    if (state.kind == SessionKind::SelectOnly) {
      /* Hide all tool/context gizmos (left-side transform widget included). */
      v3d->gizmo_flag = eView3D_GizmoFlag(
          v3d->gizmo_flag | V3D_GIZMO_HIDE | V3D_GIZMO_HIDE_CONTEXT | V3D_GIZMO_HIDE_TOOL |
          V3D_GIZMO_HIDE_MODIFIER);
    }
    /* MeshEdit: leave gizmos alone — full Edit Mode toolset is allowed. */
  }
}

static bool view3d_still_on_screen(const bContext *C, const View3D *v3d)
{
  if (!C || !v3d) {
    return false;
  }
  const bScreen *screen = CTX_wm_screen(C);
  if (!screen) {
    return false;
  }
  for (const ScrArea &area : screen->areabase) {
    if (area.spacetype != SPACE_VIEW3D) {
      continue;
    }
    if (area.spacedata.first_as<const View3D>() == v3d) {
      return true;
    }
  }
  return false;
}

static void restore_view3d_gizmos(bContext *C, GeometryEditSession &state)
{
  for (const View3DGizmoBackup &bak : state.gizmo_backups) {
    /* Layout changes during the session free View3D — never write into dangling pointers. */
    if (bak.v3d && view3d_still_on_screen(C, bak.v3d)) {
      bak.v3d->gizmo_flag = eView3D_GizmoFlag(bak.gizmo_flag);
    }
  }
  state.gizmo_backups.clear();
}

static bool space_node_still_on_screen(const bContext *C, const SpaceNode *snode, ScrArea **r_area)
{
  if (!C || !snode) {
    return false;
  }
  const bScreen *screen = CTX_wm_screen(C);
  if (!screen) {
    return false;
  }
  for (ScrArea &area : screen->areabase) {
    if (area.spacetype != SPACE_NODE) {
      continue;
    }
    if (area.spacedata.first_as<SpaceNode>() == snode) {
      if (r_area) {
        *r_area = &area;
      }
      return true;
    }
  }
  return false;
}

/**
 * Pin every Geometry Nodes editor on the screen so activating the temp edit object
 * does not replace the tree. Only record editors we ourselves pinned (user pins stay).
 */
static void pin_node_editor_for_session(bContext *C, GeometryEditSession &state)
{
  state.pinned_editors.clear();

  const bScreen *screen = CTX_wm_screen(C);
  if (!screen) {
    return;
  }

  bool any = false;
  for (ScrArea &area : screen->areabase) {
    if (area.spacetype != SPACE_NODE) {
      continue;
    }
    SpaceNode *sn = area.spacedata.first_as<SpaceNode>();
    if (!sn || !sn->edittree || sn->edittree->type != NTREE_GEOMETRY) {
      continue;
    }
    if (sn->flag & SNODE_PIN) {
      continue; /* User already pinned — leave alone on exit. */
    }
    sn->flag |= SNODE_PIN;
    state.pinned_editors.append(sn);
    any = true;
    ED_area_tag_redraw(&area);
  }
  if (any) {
    WM_main_add_notifier(NC_SPACE | ND_SPACE_NODE, nullptr);
  }
}

static void restore_node_editor_pin(bContext *C, GeometryEditSession &state)
{
  if (state.pinned_editors.is_empty()) {
    return;
  }
  const bScreen *screen = CTX_wm_screen(C);
  for (SpaceNode *sn : state.pinned_editors) {
    /* Only clear if the SpaceNode is still live on screen. */
    ScrArea *area = nullptr;
    if (space_node_still_on_screen(C, sn, &area)) {
      sn->flag &= ~SNODE_PIN;
      if (area) {
        ED_area_tag_redraw(area);
      }
    }
    else if (screen) {
      /* Fallback: if pointer is still in spacedata of any area, clear pin. */
      for (ScrArea &a : screen->areabase) {
        if (a.spacetype == SPACE_NODE && a.spacedata.first_as<SpaceNode>() == sn) {
          sn->flag &= ~SNODE_PIN;
          ED_area_tag_redraw(&a);
          break;
        }
      }
    }
  }
  state.pinned_editors.clear();
  WM_main_add_notifier(NC_SPACE | ND_SPACE_NODE, nullptr);
}

static void restore_host_object(bContext *C, GeometryEditSession &state)
{
  if (!state.host_object) {
    return;
  }
  Object *ob = state.host_object;
  ob->visibility_flag = state.prev_visibility_flag;

  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  if (bmain && scene && view_layer) {
    BKE_view_layer_synced_ensure(*bmain, scene, view_layer);
    if (Base *base = BKE_view_layer_base_find(view_layer, ob)) {
      /* Always re-select the Geometry Nodes host after the session. */
      base->flag |= BASE_SELECTED;
    }
  }
  DEG_id_tag_update(&ob->id, ID_RECALC_SYNC_TO_EVAL | ID_RECALC_BASE_FLAGS);
  WM_main_add_notifier(NC_SCENE | ND_OB_SELECT, scene);
  WM_main_add_notifier(NC_OBJECT | ND_DRAW, ob);
  /* Keep host_object until session_free_data activates it, then clear. */
}

/* -------------------------------------------------------------------- */
/** \name Temporary objects
 * \{ */

/** The materials of evaluated geometry are evaluated data-blocks, #Main needs the originals. */
static void copy_materials(Main &bmain, ID &dst, const Span<Material *> materials)
{
  for (Material *material : materials) {
    BKE_id_material_append(&bmain, &dst, material ? DEG_get_original(material) : nullptr);
  }
}

static Object *add_mesh_object(Main &bmain,
                               Scene &scene,
                               ViewLayer &view_layer,
                               const Mesh &src_mesh,
                               const char *name)
{
  Mesh *mesh = BKE_mesh_add(&bmain, name);
  /* The object takes over the user of the new data, so that removing the object leaves it
   * without users. */
  Object *ob = BKE_object_add_for_data(
      &bmain, &scene, &view_layer, OB_MESH, name, &mesh->id, false);
  BKE_mesh_nomain_to_mesh(BKE_mesh_copy_for_eval(src_mesh), mesh, ob);
  copy_materials(bmain, mesh->id, Span(src_mesh.mat, src_mesh.totcol));
  return ob;
}

static Object *add_curves_object(Main &bmain,
                                 Scene &scene,
                                 ViewLayer &view_layer,
                                 const Curves &src_curves,
                                 const char *name)
{
  Curves *curves_id = BKE_curves_add(&bmain, name);
  curves_id->geometry.wrap() = src_curves.geometry.wrap();
  curves_id->flag = src_curves.flag;
  curves_id->symmetry = src_curves.symmetry;
  copy_materials(bmain, curves_id->id, Span(src_curves.mat, src_curves.totcol));
  return BKE_object_add_for_data(
      &bmain, &scene, &view_layer, OB_CURVES, name, &curves_id->id, false);
}

static Object *add_pointcloud_object(Main &bmain,
                                     Scene &scene,
                                     ViewLayer &view_layer,
                                     const PointCloud &src_pointcloud,
                                     const char *name)
{
  PointCloud *pointcloud = BKE_pointcloud_add(&bmain, name);
  BKE_pointcloud_nomain_to_pointcloud(BKE_pointcloud_copy_for_eval(&src_pointcloud), pointcloud);
  copy_materials(
      bmain, pointcloud->id, Span(src_pointcloud.mat, src_pointcloud.totcol));
  return BKE_object_add_for_data(
      &bmain, &scene, &view_layer, OB_POINTCLOUD, name, &pointcloud->id, false);
}

/** Remove a temporary object together with its data, unless something else took the data. */
static void remove_session_object(bContext *C, Object *&ob)
{
  if (!ob) {
    return;
  }
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  if (bmain && scene) {
    ID *data = static_cast<ID *>(ob->data);
    if (ob->mode & OB_MODE_EDIT) {
      object::editmode_exit_ex(bmain, scene, ob, object::EM_FREEDATA);
    }
    object::base_free_and_unlink(bmain, scene, ob);
    if (data && data->us == 0) {
      BKE_id_delete(bmain, data);
    }
  }
  ob = nullptr;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name One object per curve or instance
 * \{ */

static Object *add_element_object(Main &bmain,
                                  Collection &collection,
                                  const ObjectType type,
                                  ID *data,
                                  const float4x4 &object_to_world)
{
  Object *ob = BKE_object_add_only_object(&bmain, type, ".GN_GeometryEdit_Element");
  if (data) {
    ob->data = data;
    id_us_plus(data);
  }
  ob->visibility_flag |= OB_HIDE_RENDER;
  BKE_collection_object_add(&bmain, &collection, ob);
  BKE_object_apply_mat4(ob, object_to_world.ptr(), false, false);
  return ob;
}

struct ElementObjectData {
  ID *data = nullptr;
  ObjectType type = OB_EMPTY;
};

/** What is instanced as data of an object: its mesh, otherwise its curves or its points. */
static ElementObjectData add_instance_reference_data(Main &bmain,
                                                     const bke::InstanceReference &reference)
{
  const char *name = ".GN_GeometryEdit_Element";
  bke::GeometrySet geometry;
  reference.to_geometry_set(geometry);
  geometry::RealizeInstancesOptions options;
  geometry = geometry::realize_instances(std::move(geometry), options).geometry;

  if (const Mesh *mesh = geometry.get_mesh(); mesh && mesh->verts_num > 0) {
    Mesh *dst = BKE_mesh_add(&bmain, name);
    BKE_mesh_nomain_to_mesh(BKE_mesh_copy_for_eval(*mesh), dst, nullptr);
    return {&dst->id, OB_MESH};
  }
  if (const Curves *curves_id = geometry.get_curves();
      curves_id && curves_id->geometry.point_num > 0)
  {
    Curves *dst = BKE_curves_add(&bmain, name);
    dst->geometry.wrap() = curves_id->geometry.wrap();
    return {&dst->id, OB_CURVES};
  }
  if (const PointCloud *pointcloud = geometry.get_pointcloud();
      pointcloud && pointcloud->totpoint > 0)
  {
    PointCloud *dst = BKE_pointcloud_add(&bmain, name);
    BKE_pointcloud_nomain_to_pointcloud(BKE_pointcloud_copy_for_eval(pointcloud), dst);
    return {&dst->id, OB_POINTCLOUD};
  }
  /* Nothing to show, an empty can still be selected. */
  return {};
}

/**
 * Make the objects that stand for the curves or instances, in a collection of their own.
 * \return The object of every element.
 */
static Vector<Object *> add_element_objects(Main &bmain,
                                            Scene &scene,
                                            const SessionParams &params,
                                            GeometryEditSession &state)
{
  Collection *collection = BKE_collection_add(
      &bmain, scene.master_collection, ".GN_GeometryEdit_Elements");
  state.elements_collection_uid = collection->id.session_uid;

  /* One sync of the view layers at the end instead of one per object. */
  BKE_layer_collection_resync_forbid(bmain);

  Vector<Object *> objects;
  /* The objects own their data, the user from creating it is not needed. */
  Vector<ID *> new_data;

  if (params.component_type == bke::GeometryComponent::Type::Curve) {
    const bke::CurvesGeometry &curves = params.curves->geometry.wrap();
    objects.reserve(curves.curves_num());
    for (const int curve_i : curves.curves_range()) {
      Curves *curves_id = BKE_curves_add(&bmain, ".GN_GeometryEdit_Element");
      curves_id->geometry.wrap() = bke::curves_copy_curve_selection(
          curves, IndexMask(IndexRange(curve_i, 1)), {});
      new_data.append(&curves_id->id);
      objects.append(add_element_object(
          bmain, *collection, OB_CURVES, &curves_id->id, params.object_to_world));
    }
  }
  else {
    const bke::Instances &instances = *params.instances;
    const Span<bke::InstanceReference> references = instances.references();
    const Span<int> handles = instances.reference_handles();
    const Span<float4x4> transforms = instances.transforms();
    /* Instances of the same reference share its data. */
    Array<std::optional<ElementObjectData>> data_by_handle(references.size());
    objects.reserve(instances.instances_num());
    for (const int instance_i : IndexRange(instances.instances_num())) {
      ElementObjectData data;
      const int handle = handles[instance_i];
      if (references.index_range().contains(handle)) {
        if (!data_by_handle[handle]) {
          data_by_handle[handle] = add_instance_reference_data(bmain, references[handle]);
          if (data_by_handle[handle]->data) {
            new_data.append(data_by_handle[handle]->data);
          }
        }
        data = *data_by_handle[handle];
      }
      objects.append(add_element_object(bmain,
                                        *collection,
                                        data.type,
                                        data.data,
                                        params.object_to_world * transforms[instance_i]));
    }
  }

  /* The rest of the geometry, only to look at. */
  if (params.reference_mesh && params.reference_mesh->verts_num > 0) {
    Mesh *mesh = BKE_mesh_add(&bmain, ".GN_GeometryEdit_Reference");
    BKE_mesh_nomain_to_mesh(BKE_mesh_copy_for_eval(*params.reference_mesh), mesh, nullptr);
    new_data.append(&mesh->id);
    Object *reference = BKE_object_add_only_object(&bmain, OB_MESH, ".GN_GeometryEdit_Reference");
    reference->data = &mesh->id;
    id_us_plus(&mesh->id);
    reference->dt = OB_WIRE;
    reference->visibility_flag |= OB_HIDE_RENDER | OB_HIDE_SELECT;
    BKE_collection_object_add(&bmain, collection, reference);
    BKE_object_apply_mat4(reference, params.object_to_world.ptr(), false, false);
  }

  for (ID *data : new_data) {
    id_us_min(data);
  }

  BKE_layer_collection_resync_allow(bmain);
  BKE_main_collection_sync(&bmain);

  state.element_object_uids.clear();
  state.element_object_uids.reserve(objects.size());
  for (const Object *ob : objects) {
    state.element_object_uids.append(ob->id.session_uid);
  }
  return objects;
}

static Collection *find_elements_collection(Main &bmain, const GeometryEditSession &state)
{
  if (state.elements_collection_uid == 0) {
    return nullptr;
  }
  for (Collection &collection : bmain.collections) {
    if (collection.id.session_uid == state.elements_collection_uid) {
      return &collection;
    }
  }
  return nullptr;
}

/** Delete everything in the collection of the element objects, and the collection. */
static void remove_element_objects(bContext *C, GeometryEditSession &state)
{
  Main *bmain = CTX_data_main(C);
  if (state.elements_as_objects && bmain) {
    if (Collection *collection = find_elements_collection(*bmain, state)) {
      BKE_main_id_tag_all(bmain, ID_TAG_DOIT, false);
      for (CollectionObject &collection_object : collection->gobject) {
        Object *ob = collection_object.ob;
        ob->id.tag |= ID_TAG_DOIT;
        if (ID *data = static_cast<ID *>(ob->data)) {
          data->tag |= ID_TAG_DOIT;
        }
      }
      collection->id.tag |= ID_TAG_DOIT;
      BKE_id_multi_tagged_delete(bmain);
    }
  }
  state.elements_as_objects = false;
  state.element_object_uids.clear();
  state.elements_collection_uid = 0;
}

/** Indices of the elements whose object is selected. */
static void read_selected_element_objects(bContext *C,
                                          const GeometryEditSession &state,
                                          Vector<int> &r_indices)
{
  r_indices.clear();
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  if (!bmain || !scene || !view_layer) {
    return;
  }
  Collection *collection = find_elements_collection(*bmain, state);
  if (collection == nullptr) {
    return;
  }
  /* Only objects that still exist, duplicates of them are not elements. */
  Map<uint, Object *> object_by_uid;
  for (CollectionObject &collection_object : collection->gobject) {
    object_by_uid.add(collection_object.ob->id.session_uid, collection_object.ob);
  }
  BKE_view_layer_synced_ensure(*bmain, scene, view_layer);
  for (const int element_i : state.element_object_uids.index_range()) {
    Object *ob = object_by_uid.lookup_default(state.element_object_uids[element_i], nullptr);
    if (ob == nullptr) {
      continue;
    }
    const Base *base = BKE_view_layer_base_find(view_layer, ob);
    if (base && (base->flag & BASE_SELECTED)) {
      r_indices.append(element_i);
    }
  }
}

/** \} */

static void session_free_data(bContext *C, GeometryEditSession &state)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);

  restore_view3d_gizmos(C, state);
  restore_node_editor_pin(C, state);

  if (state.prop_edit_saved && scene) {
    scene->toolsettings->proportional_edit = state.prev_prop_edit;
    state.prop_edit_saved = false;
  }

  remove_session_object(C, state.temp_object);
  state.temp_data = nullptr;
  remove_session_object(C, state.reference_object);
  remove_element_objects(C, state);
  restore_host_object(C, state);

  /*
   * After Esc/Enter/Tab (or cancel): make the Geometry Nodes host object the real
   * active object (not only BASE_SELECTED — that shows blue in the Outliner without
   * basact / outliner active sync). Fall back to the pre-session active object only
   * if no host was recorded.
   */
  Object *activate = state.host_object ? state.host_object : state.prev_active_object;
  if (activate && bmain && scene && view_layer && C) {
    BKE_view_layer_synced_ensure(*bmain, scene, view_layer);
    View3D *v3d = CTX_wm_view3d(C);
    object::base_deselect_all(*bmain, scene, view_layer, v3d, SEL_DESELECT);

    if (Base *base = BKE_view_layer_base_find(view_layer, activate)) {
      /* base_select syncs Object.flag from Base (raw |= BASE_SELECTED does not). */
      object::base_select(base, object::BA_SELECT);
      BKE_view_layer_base_select_and_set_active(view_layer, base);
      object::base_activate(C, base);
    }
    else {
      object::jump_to_object(C, activate, true);
    }

    /* Host is object-mode; clear any leftover Edit Mode from the temp object. */
    object::mode_set(C, OB_MODE_OBJECT);

    DEG_id_tag_update(&activate->id, ID_RECALC_SYNC_TO_EVAL | ID_RECALC_BASE_FLAGS);
    DEG_id_tag_update(&scene->id, ID_RECALC_SELECT | ID_RECALC_BASE_FLAGS);
    ED_outliner_select_sync_from_object_tag(C);
    WM_event_add_notifier(C, NC_SCENE | ND_OB_SELECT, scene);
    WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, scene);
    WM_event_add_notifier(C, NC_SCENE | ND_MODE, nullptr);
  }
  state.host_object = nullptr;
  state.prev_active_object = nullptr;

  state.ntree = nullptr;
  state.node_identifier = 0;
  state.active = false;
  state.kind = SessionKind::SelectOnly;
  state.component_type = bke::GeometryComponent::Type::Mesh;
  state.domain = bke::AttrDomain::Point;
  state.object_to_world = float4x4::identity();
  state.seed_verts = state.seed_edges = state.seed_faces = state.seed_curves = 0;
  state.seed_positions.clear();
  state.seed_geometry = {};
  state.pinned_editors.clear();

  if (bmain && scene) {
    DEG_relations_tag_update(bmain);
  }
}

bool session_is_active()
{
  return session_state().active;
}

SessionKind session_kind()
{
  return session_state().kind;
}

bke::GeometryComponent::Type session_component_type()
{
  return session_state().component_type;
}

bool session_selects_objects()
{
  return session_state().active && session_state().elements_as_objects;
}

/** The selection to start a select-only session with, on the domain that is selected on. */
static void apply_seed_selection_to_session(const SessionParams &params,
                                            const bke::AttrDomain domain)
{
  if (Curves *curves_id = session_curves()) {
    bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    const bke::AttrDomain seed_domain = params.seed_domain == bke::AttrDomain::Curve ?
                                            bke::AttrDomain::Curve :
                                            bke::AttrDomain::Point;
    Array<bool> selection = selection_from_indices(
        params.seed_indices, curves.attributes().domain_size(seed_domain));
    if (seed_domain != domain) {
      const VArray<bool> adapted = curves.attributes().adapt_domain<bool>(
          VArray<bool>::from_span(selection), seed_domain, domain);
      Array<bool> adapted_selection(adapted.size());
      adapted.materialize(adapted_selection);
      selection = std::move(adapted_selection);
    }
    curves_id->selection_domain = char(domain);
    write_curves_selection(curves, domain, selection);
    return;
  }
  if (PointCloud *pointcloud = session_pointcloud()) {
    write_pointcloud_selection(
        *pointcloud, selection_from_indices(params.seed_indices, pointcloud->totpoint));
    return;
  }
}

bool session_begin(bContext *C, const SessionParams &params, ReportList *reports)
{
  GeometryEditSession &state = session_state();
  if (state.active) {
    BKE_report(reports, RPT_WARNING, "Geometry Edit session already active");
    return false;
  }

  using ComponentType = bke::GeometryComponent::Type;
  const bool is_mesh = params.component_type == ComponentType::Mesh;
  const bool is_curves = params.component_type == ComponentType::Curve;
  const bool is_instances = params.component_type == ComponentType::Instance;
  const bool is_points = ELEM(
      params.component_type, ComponentType::PointCloud, ComponentType::Instance);

  /* Whole curves and instances are selected as objects. */
  const bool as_objects = params.elements_as_objects &&
                          params.kind == SessionKind::SelectOnly &&
                          ((is_curves && params.domain == bke::AttrDomain::Curve) || is_instances);

  bool has_data = false;
  if (as_objects) {
    has_data = is_curves ? (params.curves && params.curves->geometry.curve_num > 0) :
                           (params.instances && params.instances->instances_num() > 0);
  }
  else {
    has_data = (is_mesh && params.mesh && params.mesh->verts_num > 0) ||
               (is_curves && params.curves && params.curves->geometry.point_num > 0) ||
               (is_points && params.pointcloud && params.pointcloud->totpoint > 0);
  }
  if (!has_data) {
    BKE_report(reports, RPT_ERROR, "Nothing to edit in the Geometry input of the node");
    return false;
  }

  if (params.kind == SessionKind::SelectOnly &&
      !nodes::elements_component_supports_domain(params.component_type, params.domain))
  {
    BKE_report(reports, RPT_ERROR, "The geometry component cannot be selected on this domain");
    return false;
  }

  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  if (!bmain || !scene || !view_layer) {
    BKE_report(reports, RPT_ERROR, "Invalid context");
    return false;
  }

  /* The session starts from Object Mode. */
  if (const Object *obact = CTX_data_active_object(C)) {
    if (obact->mode != OB_MODE_OBJECT) {
      object::mode_set(C, OB_MODE_OBJECT);
    }
  }

  /* Pin first — before temp object becomes active and would switch unpinned node editors. */
  pin_node_editor_for_session(C, state);

  state.kind = params.kind;
  state.component_type = params.component_type;
  state.elements_as_objects = as_objects;
  state.seed_verts = state.seed_edges = state.seed_faces = state.seed_curves = 0;
  state.seed_positions.clear();

  Vector<Object *> element_objects;
  Object *ob = nullptr;
  if (as_objects) {
    element_objects = add_element_objects(*bmain, *scene, params, state);
  }
  else {
    /* The rest of the geometry, only to look at. */
    if (params.reference_mesh && params.reference_mesh->verts_num > 0) {
      Object *reference = add_mesh_object(
          *bmain, *scene, *view_layer, *params.reference_mesh, ".GN_GeometryEdit_Reference");
      BKE_object_apply_mat4(reference, params.object_to_world.ptr(), false, false);
      reference->dt = OB_WIRE;
      reference->visibility_flag |= OB_HIDE_RENDER | OB_HIDE_SELECT;
      state.reference_object = reference;
    }

    if (is_mesh) {
      ob = add_mesh_object(*bmain, *scene, *view_layer, *params.mesh, ".GN_GeometryEdit_Temp");
    }
    else if (is_curves) {
      ob = add_curves_object(
          *bmain, *scene, *view_layer, *params.curves, ".GN_GeometryEdit_Temp");
    }
    else {
      ob = add_pointcloud_object(
          *bmain, *scene, *view_layer, *params.pointcloud, ".GN_GeometryEdit_Temp");
    }
    BKE_object_apply_mat4(ob, params.object_to_world.ptr(), false, false);
    ob->visibility_flag |= OB_HIDE_RENDER;

    state.temp_object = ob;
    state.temp_data = static_cast<ID *>(ob->data);

    if (const Mesh *mesh = session_mesh()) {
      state.seed_verts = mesh->verts_num;
      state.seed_edges = mesh->edges_num;
      state.seed_faces = mesh->faces_num;
      state.seed_positions.extend(mesh->vert_positions());
    }
    else if (const Curves *curves_id = session_curves()) {
      state.seed_verts = curves_id->geometry.point_num;
      state.seed_curves = curves_id->geometry.curve_num;
      state.seed_positions.extend(curves_id->geometry.wrap().positions());
    }
    else if (const PointCloud *pointcloud = session_pointcloud()) {
      state.seed_verts = pointcloud->totpoint;
      state.seed_positions.extend(pointcloud->positions());
    }
  }

  state.host_object = params.host_object;
  state.prev_base_selected = false;
  state.prev_active_object = nullptr;
  BKE_view_layer_synced_ensure(*bmain, scene, view_layer);
  if (Base *act = BKE_view_layer_active_base_get(view_layer)) {
    /* Adding the temporary objects made them active. */
    if (!ELEM(act->object, ob, state.reference_object)) {
      state.prev_active_object = act->object;
    }
  }

  if (Object *host_object = params.host_object) {
    state.prev_visibility_flag = eObject_VisibilityFlag(host_object->visibility_flag);
    host_object->visibility_flag |= OB_HIDE_VIEWPORT;
    if (Base *base = BKE_view_layer_base_find(view_layer, host_object)) {
      state.prev_base_selected = (base->flag & BASE_SELECTED) != 0;
      base->flag &= ~BASE_SELECTED;
    }
    DEG_id_tag_update(&host_object->id, ID_RECALC_SYNC_TO_EVAL | ID_RECALC_BASE_FLAGS);
  }

  object::base_deselect_all(*bmain, scene, view_layer, nullptr, SEL_DESELECT);

  /* Domain / select mode. A full edit keeps the current multi-mode tools free. */
  const bke::AttrDomain use_domain = (params.kind == SessionKind::MeshEdit) ?
                                         bke::AttrDomain::Point :
                                         params.domain;

  if (as_objects) {
    /* The stored selection, as selected objects. A selection of control points selects the
     * curves that have all their points selected. */
    Array<bool> selection;
    if (is_curves && params.seed_domain == bke::AttrDomain::Point) {
      const bke::AttributeAccessor attributes = params.curves->geometry.wrap().attributes();
      const Array<bool> point_selection = selection_from_indices(
          params.seed_indices, attributes.domain_size(bke::AttrDomain::Point));
      const VArray<bool> adapted = attributes.adapt_domain<bool>(
          VArray<bool>::from_span(point_selection),
          bke::AttrDomain::Point,
          bke::AttrDomain::Curve);
      selection.reinitialize(adapted.size());
      adapted.materialize(selection);
    }
    else {
      selection = selection_from_indices(params.seed_indices, int(element_objects.size()));
    }
    Base *first_selected = nullptr;
    for (const int i : element_objects.index_range()) {
      if (i >= selection.size() || !selection[i]) {
        continue;
      }
      if (Base *base = BKE_view_layer_base_find(view_layer, element_objects[i])) {
        object::base_select(base, object::BA_SELECT);
        if (first_selected == nullptr) {
          first_selected = base;
        }
      }
    }
    view_layer->basact = first_selected;
    if (first_selected) {
      object::base_activate(C, first_selected);
    }
    DEG_id_tag_update(&scene->id, ID_RECALC_SELECT | ID_RECALC_BASE_FLAGS);
    WM_event_add_notifier(C, NC_SCENE | ND_OB_SELECT, scene);
    WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, scene);
  }
  else {
    Base *temp_base = BKE_view_layer_base_find(view_layer, ob);
    if (temp_base) {
      temp_base->flag |= BASE_SELECTED;
      view_layer->basact = temp_base;
      object::base_activate(C, temp_base);
    }
    if (params.kind == SessionKind::SelectOnly && is_mesh) {
      scene->toolsettings->selectmode = domain_to_selectmode(use_domain);
    }
  }

  /* Remember proportional setting; a full edit leaves user preference alone. */
  state.prev_prop_edit = scene->toolsettings->proportional_edit;
  state.prop_edit_saved = true;

  hide_view3d_gizmos(C, state);

  state.active = true;

  if (!as_objects) {
    if (!object::editmode_enter_ex(bmain, scene, ob, 0)) {
      session_free_data(C, state);
      BKE_report(reports, RPT_ERROR, "Failed to enter Edit Mode");
      return false;
    }

    if (is_mesh) {
      BMEditMesh *em = BKE_editmesh_from_object(ob);
      if (!em || !em->bm) {
        session_free_data(C, state);
        BKE_report(reports, RPT_ERROR, "No edit mesh");
        return false;
      }

      if (params.kind == SessionKind::SelectOnly) {
        /* Select on the domain of the stored selection first, then switch to the one that is
         * selected on now, which converts the selection. */
        const bke::AttrDomain seed_domain = nodes::elements_component_sanitize_domain(
            params.component_type, params.seed_domain);
        em->bm->selectmode = domain_to_selectmode(seed_domain);
        apply_seed_selection(*em->bm, seed_domain, params.seed_indices);
      }
      EDBM_selectmode_set(em, em->bm, domain_to_selectmode(use_domain));
      EDBM_selectmode_flush(em->bm, em->selectmode);
    }
    else if (params.kind == SessionKind::SelectOnly) {
      apply_seed_selection_to_session(params, use_domain);
      /* What a select-only session goes back to, with the selection it starts with. */
      if (const Curves *curves_id = session_curves()) {
        state.seed_geometry = bke::GeometrySet::from_curves(BKE_curves_copy_for_eval(curves_id));
      }
      else if (const PointCloud *pointcloud = session_pointcloud()) {
        state.seed_geometry = bke::GeometrySet::from_pointcloud(
            BKE_pointcloud_copy_for_eval(pointcloud));
      }
    }
    DEG_id_tag_update(&ob->id, ID_RECALC_GEOMETRY);
    DEG_id_tag_update(state.temp_data, ID_RECALC_GEOMETRY);
  }

  state.ntree = params.ntree;
  state.node_identifier = params.node_identifier;
  state.domain = use_domain;
  state.object_to_world = params.object_to_world;

  /* Activate the Select tool so left-drag selects (not Move / Cursor / Annotate). */
  WM_toolsystem_ref_set_by_id(C, "builtin.select");
  WM_toolsystem_update_from_context_view3d(C);

  DEG_relations_tag_update(bmain);
  session_tag_redraw(C);
  return true;
}

void session_cancel(bContext *C)
{
  GeometryEditSession &state = session_state();
  if (!state.active) {
    return;
  }
  session_free_data(C, state);
  session_tag_redraw(C);
}

bool session_commit_selection(bContext *C, bke::AttrDomain &r_domain, Vector<int> &r_indices)
{
  GeometryEditSession &state = session_state();
  if (state.active && state.elements_as_objects) {
    read_selected_element_objects(C, state, r_indices);
    r_domain = state.component_type == bke::GeometryComponent::Type::Curve ?
                   bke::AttrDomain::Curve :
                   bke::AttrDomain::Instance;
    session_free_data(C, state);
    session_tag_redraw(C);
    return true;
  }
  if (!state.active || !state.temp_object || state.kind != SessionKind::SelectOnly) {
    return false;
  }

  if (state.temp_object->type == OB_MESH) {
    if (Scene *scene = CTX_data_scene(C)) {
      state.domain = selectmode_to_domain(scene->toolsettings->selectmode);
    }
  }

  if (!session_enforce_constraints(C)) {
    session_free_data(C, state);
    return false;
  }

  if (const Curves *curves_id = session_curves()) {
    state.domain = curves_id->selection_domain == char(bke::AttrDomain::Curve) ?
                       bke::AttrDomain::Curve :
                       bke::AttrDomain::Point;
    indices_from_selection(
        read_selection_attribute(curves_id->geometry.wrap().attributes(), state.domain),
        r_indices);
    r_domain = state.domain;
  }
  else if (const PointCloud *pointcloud = session_pointcloud()) {
    indices_from_selection(
        read_selection_attribute(pointcloud->attributes(), bke::AttrDomain::Point), r_indices);
    /* A point stands for an instance when instances are selected. */
    r_domain = state.component_type == bke::GeometryComponent::Type::Instance ?
                   bke::AttrDomain::Instance :
                   bke::AttrDomain::Point;
  }
  else {
    BMesh *bm = session_edit_bmesh();
    if (!bm) {
      session_free_data(C, state);
      return false;
    }
    r_domain = state.domain;
    read_selection_indices(*bm, state.domain, r_indices);
  }

  session_free_data(C, state);
  session_tag_redraw(C);
  return true;
}

bool session_commit_result(bContext *C, ID **r_result)
{
  *r_result = nullptr;
  GeometryEditSession &state = session_state();
  if (!state.active || !state.temp_object || state.kind != SessionKind::MeshEdit) {
    return false;
  }

  Main *bmain = CTX_data_main(C);
  if (!bmain) {
    session_free_data(C, state);
    return false;
  }

  if (state.temp_object->type == OB_MESH) {
    BMesh *bm = session_edit_bmesh();
    if (!bm) {
      session_free_data(C, state);
      return false;
    }

    /* Convert edit BMesh → nomain Mesh while still in Edit Mode. */
    const Mesh *temp_mesh = session_mesh();
    Mesh *nomain = BKE_mesh_from_bmesh_for_eval_nomain(bm, nullptr, temp_mesh);
    if (!nomain) {
      session_free_data(C, state);
      return false;
    }

    Mesh *mesh = BKE_mesh_add(bmain, "EditElements");
    if (!mesh) {
      BKE_id_free(nullptr, nomain);
      session_free_data(C, state);
      return false;
    }
    BKE_mesh_nomain_to_mesh(nomain, mesh, nullptr);
    copy_materials(*bmain, mesh->id, Span(temp_mesh->mat, temp_mesh->totcol));
    *r_result = &mesh->id;
  }
  else {
    /* Curves and point clouds are edited in place, the edited data itself is the result. */
    if (Curves *curves_id = session_curves()) {
      bke::MutableAttributeAccessor attributes = curves_id->geometry.wrap().attributes_for_write();
      ed::curves::remove_selection_attributes(attributes);
    }
    else if (PointCloud *pointcloud = session_pointcloud()) {
      pointcloud->attributes_for_write().remove(".selection");
    }
    ID *result = state.temp_data;
    /* Keeps the data when the temporary object is removed. */
    id_us_plus(result);
    BKE_libblock_rename(*bmain, *result, "EditElements");
    *r_result = result;
  }

  session_free_data(C, state);
  session_tag_redraw(C);
  return true;
}

bke::AttrDomain session_domain()
{
  return session_state().domain;
}

void session_set_domain(const bke::AttrDomain domain)
{
  GeometryEditSession &state = session_state();
  if (!state.active || !state.temp_object || state.kind != SessionKind::SelectOnly) {
    return;
  }
  if (state.temp_object->type != OB_MESH) {
    return;
  }
  if (!ELEM(domain, bke::AttrDomain::Point, bke::AttrDomain::Edge, bke::AttrDomain::Face)) {
    return;
  }
  state.domain = domain;
  BMEditMesh *em = BKE_editmesh_from_object(state.temp_object);
  if (em) {
    EDBM_selectmode_set(em, em->bm, domain_to_selectmode(domain));
  }
}

}  // namespace blender::ed::geometry_edit
