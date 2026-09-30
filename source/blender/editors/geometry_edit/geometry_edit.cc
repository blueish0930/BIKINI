/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgeometry_edit
 *
 * Temp Object + native Mesh Edit Mode for Select Elements / Edit Elements.
 */

#include "BKE_context.hh"
#include "BKE_editmesh.hh"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"
#include "BKE_object.hh"
#include "BKE_report.hh"
#include "BKE_scene.hh"
#include "BKE_screen.hh"

#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"

#include "DNA_layer_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_enums.h"
#include "DNA_space_types.h"
#include "DNA_view3d_types.h"
#include "DNA_windowmanager_types.h"

#include "ED_geometry_edit.hh"
#include "ED_mesh.hh"
#include "ED_object.hh"
#include "ED_outliner.hh"
#include "ED_screen.hh"
#include "ED_select_utils.hh"
#include "ED_undo.hh"

#include "MEM_guardedalloc.h"

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

void read_vertex_positions(BMesh &bm, Vector<float3> &r_positions)
{
  r_positions.clear();
  r_positions.reserve(bm.totvert);
  BMVert *v;
  BMIter iter;
  BM_ITER_MESH (v, &iter, &bm, BM_VERTS_OF_MESH) {
    r_positions.append(float3(v->co));
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

static BMesh *session_edit_bmesh()
{
  GeometryEditSession &state = session_state();
  if (!state.temp_object) {
    return nullptr;
  }
  BMEditMesh *em = BKE_editmesh_from_object(state.temp_object);
  return em ? em->bm : nullptr;
}

bool session_topology_ok()
{
  GeometryEditSession &state = session_state();
  if (!state.active || !state.temp_object) {
    return false;
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
  if (!state.active || !state.temp_object) {
    return false;
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
  if (state.temp_mesh) {
    DEG_id_tag_update(&state.temp_mesh->id, ID_RECALC_GEOMETRY);
  }
  return true;
}

bool session_enforce_constraints(bContext *C)
{
  GeometryEditSession &state = session_state();
  if (!state.active || !state.temp_object) {
    return false;
  }

  /* Full mesh edit: no constraints — any Edit Mode operation is allowed. */
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

static void remove_temp_object(bContext *C, GeometryEditSession &state)
{
  if (!state.temp_object) {
    return;
  }

  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  Object *ob = state.temp_object;

  if (bmain && scene && (ob->mode & OB_MODE_EDIT)) {
    object::editmode_exit_ex(bmain, scene, ob, object::EM_FREEDATA);
  }

  if (bmain && scene) {
    object::base_free_and_unlink(bmain, scene, ob);
  }

  state.temp_object = nullptr;
  state.temp_mesh = nullptr;
}

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

  remove_temp_object(C, state);
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

    /* Host is object-mode; clear any leftover Edit Mode from the temp mesh. */
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
  state.domain = bke::AttrDomain::Point;
  state.object_to_world = float4x4::identity();
  state.seed_verts = state.seed_edges = state.seed_faces = 0;
  state.seed_positions.clear();
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

bool session_begin(bContext *C,
                   bNodeTree *ntree,
                   const int32_t node_identifier,
                   Object *host_object,
                   const Mesh &src_mesh,
                   const float4x4 &object_to_world,
                   const bke::AttrDomain domain,
                   const Span<int> seed_indices,
                   const SessionKind kind,
                   ReportList *reports)
{
  GeometryEditSession &state = session_state();
  if (state.active) {
    BKE_report(reports, RPT_WARNING, "Geometry Edit session already active");
    return false;
  }

  if (kind == SessionKind::SelectOnly) {
    if (!ELEM(domain, bke::AttrDomain::Point, bke::AttrDomain::Edge, bke::AttrDomain::Face)) {
      BKE_report(reports, RPT_ERROR, "Select Elements requires Point, Edge, or Face domain");
      return false;
    }
  }

  if (src_mesh.verts_num == 0) {
    BKE_report(reports, RPT_ERROR, "Empty mesh from node Geometry input");
    return false;
  }

  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  if (!bmain || !scene || !view_layer) {
    BKE_report(reports, RPT_ERROR, "Invalid context");
    return false;
  }

  /* Pin first — before temp object becomes active and would switch unpinned node editors. */
  pin_node_editor_for_session(C, state);

  Mesh *me = BKE_mesh_add(bmain, ".GN_GeometryEdit_Temp");
  Mesh *nomain = BKE_mesh_copy_for_eval(src_mesh);
  if (!me || !nomain) {
    if (me) {
      BKE_id_delete(bmain, &me->id);
    }
    restore_node_editor_pin(C, state);
    BKE_report(reports, RPT_ERROR, "Failed to copy input mesh");
    return false;
  }

  Object *ob = BKE_object_add_for_data(
      bmain, scene, view_layer, OB_MESH, ".GN_GeometryEdit_Temp", &me->id, true);
  if (!ob) {
    BKE_id_free(nullptr, nomain);
    BKE_id_delete(bmain, &me->id);
    restore_node_editor_pin(C, state);
    BKE_report(reports, RPT_ERROR, "Failed to create temp object");
    return false;
  }

  BKE_mesh_nomain_to_mesh(nomain, me, ob);
  BKE_object_apply_mat4(ob, object_to_world.ptr(), false, false);
  ob->visibility_flag |= OB_HIDE_RENDER;

  state.seed_verts = me->verts_num;
  state.seed_edges = me->edges_num;
  state.seed_faces = me->faces_num;
  state.seed_positions.clear();
  {
    const Span<float3> pos = me->vert_positions();
    state.seed_positions.extend(pos);
  }

  state.host_object = host_object;
  state.prev_base_selected = false;
  state.prev_active_object = nullptr;
  BKE_view_layer_synced_ensure(*bmain, scene, view_layer);
  if (Base *act = BKE_view_layer_active_base_get(view_layer)) {
    state.prev_active_object = act->object;
  }

  if (host_object) {
    state.prev_visibility_flag = host_object->visibility_flag;
    host_object->visibility_flag |= OB_HIDE_VIEWPORT;
    if (Base *base = BKE_view_layer_base_find(view_layer, host_object)) {
      state.prev_base_selected = (base->flag & BASE_SELECTED) != 0;
      base->flag &= ~BASE_SELECTED;
    }
    DEG_id_tag_update(&host_object->id, ID_RECALC_SYNC_TO_EVAL | ID_RECALC_BASE_FLAGS);
  }

  object::base_deselect_all(*bmain, scene, view_layer, nullptr, SEL_DESELECT);
  Base *temp_base = BKE_view_layer_base_find(view_layer, ob);
  if (temp_base) {
    temp_base->flag |= BASE_SELECTED;
    view_layer->basact = temp_base;
    object::base_activate(C, temp_base);
  }

  /* Domain / select mode. MeshEdit keeps current multi-mode tools free. */
  const bke::AttrDomain use_domain = (kind == SessionKind::MeshEdit) ? bke::AttrDomain::Point :
                                                                       domain;
  if (kind == SessionKind::SelectOnly) {
    scene->toolsettings->selectmode = domain_to_selectmode(use_domain);
  }

  /* Remember proportional setting; MeshEdit leaves user preference alone. */
  state.prev_prop_edit = scene->toolsettings->proportional_edit;
  state.prop_edit_saved = true;

  state.kind = kind;
  hide_view3d_gizmos(C, state);

  if (!object::editmode_enter_ex(bmain, scene, ob, 0)) {
    state.temp_object = ob;
    state.temp_mesh = me;
    state.active = true;
    session_free_data(C, state);
    BKE_report(reports, RPT_ERROR, "Failed to enter Edit Mode");
    return false;
  }

  BMEditMesh *em = BKE_editmesh_from_object(ob);
  if (!em || !em->bm) {
    state.temp_object = ob;
    state.temp_mesh = me;
    state.active = true;
    session_free_data(C, state);
    BKE_report(reports, RPT_ERROR, "No edit mesh");
    return false;
  }

  em->bm->selectmode = domain_to_selectmode(use_domain);
  if (kind == SessionKind::SelectOnly) {
    apply_seed_selection(*em->bm, use_domain, seed_indices);
  }
  EDBM_selectmode_set(em, em->bm, domain_to_selectmode(use_domain));
  EDBM_selectmode_flush(em->bm, em->selectmode);
  DEG_id_tag_update(&ob->id, ID_RECALC_GEOMETRY);
  DEG_id_tag_update(&me->id, ID_RECALC_GEOMETRY);

  state.active = true;
  state.ntree = ntree;
  state.node_identifier = node_identifier;
  state.temp_object = ob;
  state.temp_mesh = me;
  state.domain = use_domain;
  state.object_to_world = object_to_world;

  /* Activate mesh Select tool so left-drag selects (not Move / Cursor / Annotate). */
  WM_toolsystem_ref_set_by_id(C, "builtin.select");
  WM_toolsystem_update_from_context_view3d(C);

  DEG_relations_tag_update(bmain);
  session_tag_redraw(C);

  if (kind == SessionKind::SelectOnly) {
    BKE_reportf(reports,
                RPT_INFO,
                "Select Elements: selection only (%d verts). Esc/Enter/Tab confirm",
                state.seed_verts);
  }
  else {
    BKE_reportf(reports,
                RPT_INFO,
                "Edit Elements: free Mesh Edit Mode (%d verts). "
                "All edit tools allowed · Esc/Enter/Tab save full mesh to node",
                state.seed_verts);
  }
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
  if (!state.active || !state.temp_object || state.kind != SessionKind::SelectOnly) {
    return false;
  }

  if (Scene *scene = CTX_data_scene(C)) {
    state.domain = selectmode_to_domain(scene->toolsettings->selectmode);
  }

  if (!session_enforce_constraints(C)) {
    session_free_data(C, state);
    return false;
  }

  BMesh *bm = session_edit_bmesh();
  if (!bm) {
    session_free_data(C, state);
    return false;
  }

  r_domain = state.domain;
  read_selection_indices(*bm, state.domain, r_indices);
  session_free_data(C, state);
  session_tag_redraw(C);
  return true;
}

bool session_commit_mesh(bContext *C, Mesh **r_mesh)
{
  *r_mesh = nullptr;
  GeometryEditSession &state = session_state();
  if (!state.active || !state.temp_object || state.kind != SessionKind::MeshEdit) {
    return false;
  }

  Main *bmain = CTX_data_main(C);
  if (!bmain) {
    session_free_data(C, state);
    return false;
  }

  BMesh *bm = session_edit_bmesh();
  if (!bm) {
    session_free_data(C, state);
    return false;
  }

  /* Convert edit BMesh → nomain Mesh while still in Edit Mode. */
  Mesh *nomain = BKE_mesh_from_bmesh_for_eval_nomain(bm, nullptr, state.temp_mesh);
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

  *r_mesh = mesh;
  session_free_data(C, state);
  session_tag_redraw(C);
  return true;
}

bool session_commit_positions(bContext *C, Vector<float3> &r_positions)
{
  GeometryEditSession &state = session_state();
  if (!state.active || !state.temp_object || state.kind != SessionKind::MeshEdit) {
    return false;
  }

  BMesh *bm = session_edit_bmesh();
  if (!bm) {
    session_free_data(C, state);
    return false;
  }

  read_vertex_positions(*bm, r_positions);
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
