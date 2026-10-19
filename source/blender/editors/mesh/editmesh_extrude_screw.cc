/* SPDX-FileCopyrightText: 2004 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edmesh
 */

#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "BKE_context.hh"
#include "BKE_editmesh.hh"
#include "BKE_layer.hh"
#include "BKE_report.hh"

#include "BLI_array.hh"
#include "BLI_kdtree_new.hh"
#include "BLI_math_bits.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_rotation_c.hh"
#include "BLI_math_vector_c.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_types.hh"

#include "ED_mesh.hh"
#include "ED_screen.hh"
#include "ED_view3d.hh"

#include "mesh_intern.hh" /* own include */

#include <algorithm>

namespace blender {

/* -------------------------------------------------------------------- */
/** \name Screw Operator
 * \{ */

/**
 * BIKINI: Screw needs one string of connected verts, so with mesh symmetry it is not
 * given the mirrored selection and only built the user's side. Copy what it made onto
 * every mirror side and weld the copy to that side's profile. Screwing each side on its
 * own would not mirror: the screw direction comes from the order of the profile's two
 * end verts, which a mirror copy does not keep.
 */
static void edbm_screw_mirror_result(BMesh *bm, const int symmetry, const Set<BMVert *> &old_verts)
{
  BM_mesh_elem_hflag_disable_all(bm, BM_ALL_NOLOOP, BM_ELEM_TAG, false);
  BMIter iter;
  BMVert *v;
  BMEdge *e;
  BMFace *f;
  bool any_new = false;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    if (!old_verts.contains(v)) {
      BM_elem_flag_enable(v, BM_ELEM_TAG);
      any_new = true;
    }
  }
  if (!any_new) {
    return;
  }
  /* Everything the new verts belong to, including the profile it was spun from. */
  BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
    BMLoop *l = f->l_first;
    do {
      if (BM_elem_flag_test(l->v, BM_ELEM_TAG)) {
        BM_elem_flag_enable(f, BM_ELEM_TAG);
        break;
      }
    } while ((l = l->next) != f->l_first);
  }
  BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
    if (BM_elem_flag_test(e->v1, BM_ELEM_TAG) || BM_elem_flag_test(e->v2, BM_ELEM_TAG)) {
      BM_elem_flag_enable(e, BM_ELEM_TAG);
    }
  }
  BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
    if (BM_elem_flag_test(f, BM_ELEM_TAG)) {
      BMLoop *l = f->l_first;
      do {
        BM_elem_flag_enable(l->e, BM_ELEM_TAG);
      } while ((l = l->next) != f->l_first);
    }
  }
  BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
    if (BM_elem_flag_test(e, BM_ELEM_TAG)) {
      BM_elem_flag_enable(e->v1, BM_ELEM_TAG);
      BM_elem_flag_enable(e->v2, BM_ELEM_TAG);
    }
  }

  /* The mirror copy of the profile lands on the other side's profile: weld it there. */
  Vector<BMVert *> old_list;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    if (old_verts.contains(v)) {
      old_list.append(v);
    }
  }
  /* Snapshot of the profile positions, which the tree references. */
  Array<float3> old_positions(old_list.size());
  for (const int i : old_list.index_range()) {
    old_positions[i] = old_list[i]->co;
  }
  const KDTreeNew<float3> tree(old_positions);
  const float weld_dist = 1e-4f;

  for (int mask = 1; mask < 8; mask++) {
    if ((mask & symmetry) != mask) {
      continue;
    }
    BMOperator dupop;
    BMO_op_initf(bm, &dupop, BMO_FLAG_DEFAULTS, "duplicate geom=%hvef", BM_ELEM_TAG);
    BMO_op_exec(bm, &dupop);
    Vector<BMVert *> dup_verts;
    Vector<BMFace *> dup_faces;
    BMOIter siter;
    BMElem *ele;
    BMO_ITER (ele, &siter, dupop.slots_out, "geom.out", BM_ALL_NOLOOP) {
      /* Copies keep the source flags: untag so the next copy does not include them. */
      BM_elem_flag_disable(ele, BM_ELEM_SELECT | BM_ELEM_TAG);
      if (ele->head.htype == BM_VERT) {
        dup_verts.append(reinterpret_cast<BMVert *>(ele));
      }
      else if (ele->head.htype == BM_FACE) {
        dup_faces.append(reinterpret_cast<BMFace *>(ele));
      }
    }
    BMO_op_finish(bm, &dupop);

    for (BMVert *dv : dup_verts) {
      for (int a = 0; a < 3; a++) {
        if (mask & (1 << a)) {
          dv->co[a] = -dv->co[a];
        }
      }
    }
    if (count_bits_i(mask) % 2) {
      for (BMFace *df : dup_faces) {
        BM_face_normal_flip(bm, df);
      }
    }

    BMOperator weldop;
    BMO_op_init(bm, &weldop, BMO_FLAG_DEFAULTS, "weld_verts");
    BMOpSlot *slot_targetmap = BMO_slot_get(weldop.slots_in, "targetmap");
    bool any_weld = false;
    for (BMVert *dv : dup_verts) {
      float dist_sq;
      const int i = tree.find_nearest(float3(dv->co), &dist_sq);
      if (i != -1 && dist_sq <= weld_dist * weld_dist) {
        BMO_slot_map_elem_insert(&weldop, slot_targetmap, dv, old_list[i]);
        any_weld = true;
      }
    }
    if (any_weld) {
      BMO_op_exec(bm, &weldop);
    }
    BMO_op_finish(bm, &weldop);
  }
  BM_mesh_elem_hflag_disable_all(bm, BM_ALL_NOLOOP, BM_ELEM_TAG, false);
}

static wmOperatorStatus edbm_screw_exec(bContext *C, wmOperator *op)
{
  BMEdge *eed;
  BMVert *eve, *v1, *v2;
  BMIter iter, eiter;
  float dvec[3], nor[3], cent[3], axis[3], v1_co_global[3], v2_co_global[3];
  int steps, turns;
  int valence;
  uint objects_empty_len = 0;
  uint failed_axis_len = 0;
  uint failed_verts_len = 0;

  turns = RNA_int_get(op->ptr, "turns");
  steps = RNA_int_get(op->ptr, "steps");
  RNA_float_get_array(op->ptr, "center", cent);
  RNA_float_get_array(op->ptr, "axis", axis);

  const Main *bmain = CTX_data_main(C);
  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  Vector<Object *> objects = BKE_view_layer_array_from_objects_in_edit_mode_unique_data(
      *bmain, scene, view_layer, CTX_wm_view3d(C));

  for (Object *obedit : objects) {
    BMesh *bm = BKE_editmesh_bmesh_get_for_write(obedit);

    if (bm->totvertsel < 2) {
      if (bm->totvertsel == 0) {
        objects_empty_len++;
      }
      continue;
    }

    if (is_zero_v3(axis)) {
      failed_axis_len++;
      continue;
    }

    /* find two vertices with valence count == 1, more or less is wrong */
    v1 = nullptr;
    v2 = nullptr;

    BM_ITER_MESH (eve, &iter, bm, BM_VERTS_OF_MESH) {
      valence = 0;
      BM_ITER_ELEM (eed, &eiter, eve, BM_EDGES_OF_VERT) {
        if (BM_elem_flag_test(eed, BM_ELEM_SELECT)) {
          valence++;
        }
      }

      if (valence == 1) {
        if (v1 == nullptr) {
          v1 = eve;
        }
        else if (v2 == nullptr) {
          v2 = eve;
        }
        else {
          v1 = nullptr;
          break;
        }
      }
    }

    if (v1 == nullptr || v2 == nullptr) {
      failed_verts_len++;
      continue;
    }

    /* BIKINI: remember the mesh before the spin to mirror what it adds, see
     * #edbm_screw_mirror_result. Only axes the selection is on one side of. */
    BMEditMesh *em = BKE_editmesh_from_object(obedit);
    bool one_side = false;
    const float3 side_co = EDBM_mirror_user_side_capture(obedit, em, one_side);
    int mirror_mask = 0;
    Set<BMVert *> old_verts;
    if (one_side) {
      const Mesh *mesh = id_cast<const Mesh *>(obedit->data);
      for (int a = 0; a < 3; a++) {
        if ((mesh->symmetry & (ME_SYMMETRY_X << a)) && side_co[a] != 0.0f) {
          mirror_mask |= 1 << a;
        }
      }
      if (mirror_mask) {
        old_verts.reserve(bm->totvert);
        BM_ITER_MESH (eve, &iter, bm, BM_VERTS_OF_MESH) {
          old_verts.add(eve);
        }
      }
    }

    copy_v3_v3(nor, obedit->object_to_world().ptr()[2]);

    /* calculate dvec */
    mul_v3_m4v3(v1_co_global, obedit->object_to_world().ptr(), v1->co);
    mul_v3_m4v3(v2_co_global, obedit->object_to_world().ptr(), v2->co);
    sub_v3_v3v3(dvec, v1_co_global, v2_co_global);
    mul_v3_fl(dvec, 1.0f / steps);

    if (dot_v3v3(nor, dvec) > 0.0f) {
      negate_v3(dvec);
    }

    BMOperator spinop;
    if (!EDBM_op_init(
            bm,
            &spinop,
            op,
            "spin geom=%hvef cent=%v axis=%v dvec=%v steps=%i angle=%f space=%m4 use_duplicate=%b",
            BM_ELEM_SELECT,
            cent,
            axis,
            dvec,
            turns * steps,
            DEG2RADF(360.0f * turns),
            obedit->object_to_world().ptr(),
            false))
    {
      continue;
    }

    BMO_op_exec(bm, &spinop);
    EDBM_flag_disable_all(bm, BM_ELEM_SELECT);
    BMO_slot_buffer_hflag_enable(
        bm, spinop.slots_out, "geom_last.out", BM_ALL_NOLOOP, BM_ELEM_SELECT, true);

    if (!EDBM_op_finish(bm, &spinop, op, true)) {
      continue;
    }

    if (mirror_mask) {
      edbm_screw_mirror_result(bm, mirror_mask, old_verts);
    }

    EDBMUpdate_Params params{};
    params.calc_looptris = true;
    params.calc_normals = false;
    params.is_destructive = true;
    EDBM_update(id_cast<Mesh *>(obedit->data), &params);
  }

  if (failed_axis_len == objects.size() - objects_empty_len) {
    BKE_report(op->reports, RPT_ERROR, "Invalid/unset axis");
  }
  else if (failed_verts_len == objects.size() - objects_empty_len) {
    BKE_report(op->reports, RPT_ERROR, "You have to select a string of connected vertices too");
  }

  return OPERATOR_FINISHED;
}

/* get center and axis, in global coords */
static wmOperatorStatus edbm_screw_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  Scene *scene = CTX_data_scene(C);
  RegionView3D *rv3d = ED_view3d_context_rv3d(C);

  PropertyRNA *prop;
  prop = RNA_struct_find_property(op->ptr, "center");
  if (!RNA_property_is_set(op->ptr, prop)) {
    RNA_property_float_set_array(op->ptr, prop, scene->cursor.location);
  }
  if (rv3d) {
    prop = RNA_struct_find_property(op->ptr, "axis");
    if (!RNA_property_is_set(op->ptr, prop)) {
      RNA_property_float_set_array(op->ptr, prop, rv3d->viewinv[1]);
    }
  }

  return edbm_screw_exec(C, op);
}

void MESH_OT_screw(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Screw";
  ot->description =
      "Extrude selected vertices in screw-shaped rotation around the cursor in indicated viewport";
  ot->idname = "MESH_OT_screw";

  /* API callbacks. */
  ot->invoke = edbm_screw_invoke;
  ot->exec = edbm_screw_exec;
  ot->poll = ED_operator_editmesh;

  /* flags */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* props */
  RNA_def_int(ot->srna, "steps", 9, 1, 100000, "Steps", "Steps", 3, 256);
  RNA_def_int(ot->srna, "turns", 1, 1, 100000, "Turns", "Turns", 1, 256);

  RNA_def_float_vector_xyz(ot->srna,
                           "center",
                           3,
                           nullptr,
                           -1e12f,
                           1e12f,
                           "Center",
                           "Center in global view space",
                           -1e4f,
                           1e4f);
  RNA_def_float_vector(
      ot->srna, "axis", 3, nullptr, -1.0f, 1.0f, "Axis", "Axis in global view space", -1.0f, 1.0f);
}

/** \} */

}  // namespace blender
