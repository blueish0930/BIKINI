/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edmesh
 *
 * based on mouse cursor position, split of vertices along the closest edge.
 */

#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "BKE_context.hh"
#include "BKE_editmesh.hh"
#include "BKE_layer.hh"

#include "BLI_math_geom_c.hh"
#include "BLI_math_matrix.hh"
#include "BLI_vector.hh"

#include "WM_types.hh"

#include "ED_mesh.hh"
#include "ED_transform.hh"
#include "ED_view3d.hh"
#include "RNA_access.hh"
#include "RNA_define.hh"

#include "bmesh.hh"

#include "mesh_intern.hh" /* own include */

namespace blender {

/* uses total number of selected edges around a vertex to choose how to extend */
#define USE_TRICKY_EXTEND

static wmOperatorStatus edbm_rip_edge_exec(bContext *C, wmOperator *op)
{
  const Main *bmain = CTX_data_main(C);
  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  const Vector<Object *> objects = BKE_view_layer_array_from_objects_in_edit_mode_unique_data(
      *bmain, scene, view_layer, CTX_wm_view3d(C));

  float3 ray_loc;
  float3 ray_dir;
  RNA_float_get_array(op->ptr, "location", ray_loc);
  RNA_float_get_array(op->ptr, "direction", ray_dir);
  normalize_v3(ray_dir);

  for (Object *obedit : objects) {
    BMEditMesh *em = BKE_editmesh_from_object(obedit);
    BMesh *bm = BKE_editmesh_bmesh_get_for_write(obedit);

    BMIter viter;
    BMVert *v;
    bool changed = false;

    if (bm->totvertsel == 0) {
      continue;
    }
    bool has_active_side = false;
    float3 active_side_co = EDBM_mirror_active_center_capture(obedit, em, has_active_side);

    /* BIKINI: mesh symmetry support. Split the user's selection first, then repeat on the
     * mirrored selection of every enabled symmetry axis, using the pick ray mirrored into
     * this object's frame so the edge choice matches the mirrored side. Mirror counterparts
     * are gathered up-front because splitting changes the topology (and with it the vertex
     * indices the mirror cache is built on). */
    Mesh *mesh = id_cast<Mesh *>(obedit->data);
    Vector<BMVert *> mir_verts[3];
    Vector<BMEdge *> mir_edges[3];
    if (mesh->symmetry != 0) {
      Vector<BMVert *> sel_verts;
      Vector<BMEdge *> sel_edges;
      BMIter citer;
      BMEdge *e_cap;
      BM_ITER_MESH (v, &viter, bm, BM_VERTS_OF_MESH) {
        if (BM_elem_flag_test(v, BM_ELEM_SELECT) && !BM_elem_flag_test(v, BM_ELEM_HIDDEN)) {
          sel_verts.append(v);
        }
      }
      BM_ITER_MESH (e_cap, &citer, bm, BM_EDGES_OF_MESH) {
        if (BM_elem_flag_test(e_cap, BM_ELEM_SELECT) &&
            !BM_elem_flag_test(e_cap, BM_ELEM_HIDDEN))
        {
          sel_edges.append(e_cap);
        }
      }
      const bool use_topology = (mesh->editflag & ME_EDIT_MIRROR_TOPO) != 0;
      for (int axis = 0; axis < 3; axis++) {
        if (!(mesh->symmetry & (ME_SYMMETRY_X << axis))) {
          continue;
        }
        EDBM_verts_mirror_cache_begin(em, bm, axis, true, true, true, use_topology);
        for (BMVert *v_sel : sel_verts) {
          BMVert *v_mirr = EDBM_verts_mirror_get(em, bm, v_sel);
          if (v_mirr && v_mirr != v_sel && !BM_elem_flag_test(v_mirr, BM_ELEM_HIDDEN) &&
              !BM_elem_flag_test(v_mirr, BM_ELEM_SELECT))
          {
            mir_verts[axis].append(v_mirr);
          }
        }
        for (BMEdge *e_sel : sel_edges) {
          BMEdge *e_mirr = EDBM_verts_mirror_get_edge(em, bm, e_sel);
          if (e_mirr && e_mirr != e_sel && !BM_elem_flag_test(e_mirr, BM_ELEM_HIDDEN) &&
              !BM_elem_flag_test(e_mirr, BM_ELEM_SELECT))
          {
            mir_edges[axis].append(e_mirr);
          }
        }
        EDBM_verts_mirror_cache_end(em);
      }
    }

    /* New verts created by each pass, combined again at the end so the follow-up move
     * (the rip_edge_move macro) drags every side at once. */
    Vector<BMVert *> split_verts;
    Vector<BMEdge *> keep_edges;

    for (int pass = -1; pass < 3; pass++) {
      float3 ray_loc_pass = ray_loc;
      float3 ray_dir_pass = ray_dir;

      if (pass >= 0) {
        if (mir_verts[pass].is_empty() && mir_edges[pass].is_empty()) {
          continue;
        }
        /* Switch to the mirrored selection of this axis. */
        EDBM_flag_disable_all(em->bm, BM_ELEM_SELECT);
        BM_select_history_clear(bm);
        for (BMVert *v_mirr : mir_verts[pass]) {
          BM_vert_select_set(bm, v_mirr, true);
        }
        for (BMEdge *e_mirr : mir_edges[pass]) {
          BM_edge_select_set(bm, e_mirr, true);
        }

        /* Mirror the pick ray: into local space, flip the axis, back to world. */
        const float4x4 &object_to_world = obedit->object_to_world();
        const float4x4 world_to_object = math::invert(object_to_world);
        float3 ray_loc_local = math::transform_point(world_to_object, ray_loc);
        float3 ray_dir_local = math::transform_direction(world_to_object, ray_dir);
        ray_loc_local[pass] = -ray_loc_local[pass];
        ray_dir_local[pass] = -ray_dir_local[pass];
        ray_loc_pass = math::transform_point(object_to_world, ray_loc_local);
        ray_dir_pass = math::normalize(math::transform_direction(object_to_world,
                                                                 ray_dir_local));
      }

      /* clear tags. */
      BM_ITER_MESH (v, &viter, bm, BM_VERTS_OF_MESH) {
        BM_elem_flag_disable(v, BM_ELEM_TAG);
      }

      /* operate on selected verts */
      BM_ITER_MESH (v, &viter, bm, BM_VERTS_OF_MESH) {
        BMIter eiter;
        BMEdge *e;

        if (BM_elem_flag_test(v, BM_ELEM_SELECT) && BM_elem_flag_test(v, BM_ELEM_TAG) == false)
        {
          float angle_best = FLT_MAX;
          BMEdge *e_best = nullptr;

#ifdef USE_TRICKY_EXTEND
          /* first check if we can select the edge to split based on selection-only */
          int tot_sel = 0;

          BM_ITER_ELEM (e, &eiter, v, BM_EDGES_OF_VERT) {
            if (!BM_elem_flag_test(e, BM_ELEM_HIDDEN)) {
              if (BM_elem_flag_test(e, BM_ELEM_SELECT)) {
                e_best = e;
                tot_sel += 1;
              }
            }
          }
          if (tot_sel != 1) {
            e_best = nullptr;
          }

          /* only one edge selected, operate on that */
          if (e_best) {
            goto found_edge;
          }
          /* none selected, fall through and find one */
          else if (tot_sel == 0) {
            /* pass */
          }
          /* selection not 0 or 1, do nothing */
          else {
            goto found_edge;
          }
#endif

          BM_ITER_ELEM (e, &eiter, v, BM_EDGES_OF_VERT) {
            if (!BM_elem_flag_test(e, BM_ELEM_HIDDEN)) {
              BMVert *v_other = BM_edge_other_vert(e, v);

              const float4x4 &object_to_world = obedit->object_to_world();
              const float3 v_world = math::transform_point(object_to_world, float3(v->co));
              float3 projected_point;
              closest_to_line_v3(
                  projected_point, v_world, ray_loc_pass, ray_loc_pass + ray_dir_pass);

              float3 v_ray_dir = math::normalize(projected_point - v_world);
              const float3 v_other_world = math::transform_point(object_to_world,
                                                                 float3(v_other->co));
              const float3 v_edge_dir = math::normalize(v_other_world - v_world);

              float angle_test = angle_normalized_v3v3(v_ray_dir, v_edge_dir);
              if (angle_test < angle_best) {
                angle_best = angle_test;
                e_best = e;
              }
            }
          }

#ifdef USE_TRICKY_EXTEND
        found_edge:
#endif
          if (e_best) {
            const bool e_select = BM_elem_flag_test_bool(e_best, BM_ELEM_SELECT);
            BMVert *v_new;
            BMEdge *e_new;

            v_new = BM_edge_split(bm, e_best, v, &e_new, 0.0f);

            BM_vert_select_set(bm, v, false);
            BM_edge_select_set(bm, e_new, false);

            BM_vert_select_set(bm, v_new, true);
            if (e_select) {
              BM_edge_select_set(bm, e_best, true);
              keep_edges.append(e_best);
            }
            split_verts.append(v_new);
            BM_elem_flag_enable(v_new, BM_ELEM_TAG); /* prevent further splitting */

            /* When UV sync select is enabled, the wrong UV's will be selected
             * because the existing loops will have the selection and the new ones won't.
             * transfer the selection state to the new loops. */
            if (bm->uv_select_sync_valid) {
              if (e_best->l) {
                BMLoop *l_iter, *l_first;
                l_iter = l_first = e_best->l;
                do {
                  bool was_select = false;
                  if (l_iter->next->e == e_new) {
                    if (BM_elem_flag_test(l_iter, BM_ELEM_SELECT_UV)) {
                      BM_loop_edge_uvselect_set(bm, l_iter->next, false);
                      was_select = true;
                    }
                  }
                  else {
                    BLI_assert(l_iter->prev->e == e_new);
                    if (BM_elem_flag_test(l_iter->prev, BM_ELEM_SELECT_UV)) {
                      BM_loop_edge_uvselect_set(bm, l_iter->prev, false);
                      was_select = true;
                    }
                  }
                  if (was_select) {
                    BM_loop_edge_uvselect_set(bm, l_iter, true);
                  }

                } while ((l_iter = l_iter->radial_next) != l_first);
              }
            }

            changed = true;
          }
        }
      }
    }

    if (changed) {
      /* Restore the combined selection of all passes. */
      if (!mir_verts[0].is_empty() || !mir_verts[1].is_empty() || !mir_verts[2].is_empty() ||
          !mir_edges[0].is_empty() || !mir_edges[1].is_empty() || !mir_edges[2].is_empty())
      {
        EDBM_flag_disable_all(em->bm, BM_ELEM_SELECT);
        for (BMVert *v_new : split_verts) {
          BM_vert_select_set(bm, v_new, true);
        }
        for (BMEdge *e_keep : keep_edges) {
          BM_edge_select_set(bm, e_keep, true);
        }
      }

      BM_select_history_clear(bm);

      BM_mesh_select_mode_flush(bm);
      if (has_active_side) {
        EDBM_mirror_active_side_restore(obedit, em, active_side_co, true);
      }

      EDBMUpdate_Params params{};
      params.calc_looptris = true;
      params.calc_normals = false;
      params.is_destructive = true;
      EDBM_update(id_cast<Mesh *>(obedit->data), &params);
    }
  }

  return OPERATOR_FINISHED;
}

static wmOperatorStatus edbm_rip_edge_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  ARegion *region = CTX_wm_region(C);
  const float2 mval_fl = {float(event->mval[0]), float(event->mval[1])};

  float3 ray_start, ray_dir;
  ED_view3d_win_to_ray(region, mval_fl, ray_start, ray_dir);
  normalize_v3(ray_dir);

  float3 ray_start_proj;
  /* Project ray_start onto a plane defined by the ray direction to avoid precision
   * issues especially for orthographic views where the far-clipping is used to calculate
   * the ray_start. */
  project_plane_normalized_v3_v3v3(ray_start_proj, ray_start, ray_dir);

  RNA_float_set_array(op->ptr, "location", ray_start_proj);
  RNA_float_set_array(op->ptr, "direction", ray_dir);

  return edbm_rip_edge_exec(C, op);
}

void MESH_OT_rip_edge(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Extend Vertices";
  ot->idname = "MESH_OT_rip_edge";
  ot->description = "Extend vertices along the edge closest to the cursor";

  /* API callbacks. */
  ot->invoke = edbm_rip_edge_invoke;
  ot->exec = edbm_rip_edge_exec;
  ot->poll = EDBM_view3d_poll;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_DEPENDS_ON_CURSOR;

  PropertyRNA *prop;

  prop = RNA_def_float_vector(ot->srna,
                              "location",
                              3,
                              nullptr,
                              -FLT_MAX,
                              FLT_MAX,
                              "Location",
                              "World-space ray origin for extending vertices",
                              -1.0f,
                              1.0f);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_float_vector(ot->srna,
                              "direction",
                              3,
                              nullptr,
                              -FLT_MAX,
                              FLT_MAX,
                              "Direction",
                              "World-space direction vector for extending vertices",
                              -1.0f,
                              1.0f);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

}  // namespace blender
