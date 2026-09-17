# SPDX-FileCopyrightText: 2026 BIKINI
# SPDX-License-Identifier: GPL-3.0-or-later
"""Operators: expand-then-call wrappers, extrude split, mirrored loop cut."""

from __future__ import annotations

import json

import bmesh
import bpy
from bpy.props import BoolProperty, IntProperty, StringProperty
from bpy.types import Operator
from bpy_extras import view3d_utils
from mathutils import Vector

from . import core

# Macros that extrude/duplicate/rip then transform. Split so X-Mirror
# can drive the other side from the user's side after the topology step.
_SPLIT_MACROS = {
    "mesh.extrude_region_move": ("mesh.extrude_region", "transform.translate"),
    "mesh.extrude_context_move": ("mesh.extrude_context", "transform.translate"),
    "mesh.extrude_faces_move": ("mesh.extrude_faces_indiv", "transform.shrink_fatten"),
    "mesh.extrude_edges_move": ("mesh.extrude_edges_indiv", "transform.translate"),
    "mesh.extrude_vertices_move": ("mesh.extrude_verts_indiv", "transform.translate"),
    "mesh.extrude_region_shrink_fatten": ("mesh.extrude_region", "transform.shrink_fatten"),
    "mesh.duplicate_move": ("mesh.duplicate", "transform.translate"),
    "mesh.rip_move": ("mesh.rip", "transform.translate"),
    "mesh.rip_edge_move": ("mesh.rip_edge", "transform.translate"),
}

# Keymap / menu idnames that should expand selection then invoke the original.
WRAP_TARGETS = {
    "mesh.inset",
    "mesh.bevel",
    "mesh.fill",
    "mesh.fill_grid",
    "mesh.fill_holes",
    "mesh.poke",
    "mesh.bridge_edge_loops",
    "mesh.edge_face_add",
    "mesh.subdivide",
    "mesh.unsubdivide",
    "mesh.subdivide_edgering",
    "mesh.delete",
    "mesh.dissolve_verts",
    "mesh.dissolve_edges",
    "mesh.dissolve_faces",
    "mesh.dissolve_limited",
    "mesh.dissolve_mode",
    "mesh.dissolve_degenerate",
    "mesh.edge_collapse",
    "mesh.edge_split",
    "mesh.edge_rotate",
    "mesh.split",
    "mesh.vert_connect",
    "mesh.vert_connect_path",
    "mesh.quads_convert_to_tris",
    "mesh.tris_convert_to_quads",
    "mesh.merge",
    "mesh.remove_doubles",
    "mesh.solidify",
    "mesh.spin",
    "mesh.screw",
    "mesh.offset_edge_loops_slide",
    "mesh.offset_edge_loops",
    "mesh.flip_normals",
    "mesh.mark_sharp",
    "mesh.mark_seam",
    "mesh.hide",
    "mesh.normals_make_consistent",
    "mesh.rip",
    "mesh.rip_edge",
    "mesh.extrude_region",
    "mesh.extrude_context",
    "mesh.extrude_faces_indiv",
    "mesh.extrude_edges_indiv",
    "mesh.extrude_verts_indiv",
    "mesh.extrude_manifold",
    "mesh.duplicate",
    "mesh.loopcut",
    * _SPLIT_MACROS.keys(),
}

_RESTORE_SNAPSHOT = {
    "mesh.mark_sharp",
    "mesh.mark_seam",
    "mesh.hide",
    "mesh.normals_make_consistent",
    "mesh.flip_normals",
}


def _call_ops(target: str, exec_ctx: str, kwargs: dict | None):
    mod, name = target.split(".", 1)
    opmod = getattr(bpy.ops, mod, None)
    if opmod is None:
        return {'CANCELLED'}
    op = getattr(opmod, name, None)
    if op is None:
        return {'CANCELLED'}
    try:
        if kwargs:
            return op(exec_ctx, **kwargs)
        return op(exec_ctx)
    except RuntimeError:
        return {'CANCELLED'}


def _json_kwargs(text: str) -> dict:
    if not text:
        return {}
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return {}
    return _tuples(data)


def _tuples(obj):
    if isinstance(obj, dict):
        return {k: _tuples(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return tuple(_tuples(v) for v in obj)
    return obj


def _snapshot_select(context) -> dict:
    out = {}
    for obj in core.edit_objects(context):
        mesh = obj.data
        bm = bmesh.from_edit_mesh(mesh)
        out[obj.name] = {
            "v": {v.index for v in bm.verts if v.select},
            "e": {e.index for e in bm.edges if e.select},
            "f": {f.index for f in bm.faces if f.select},
        }
    return out


def _restore_select(context, snap: dict):
    for obj in core.edit_objects(context):
        rec = snap.get(obj.name)
        if rec is None:
            continue
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.faces.ensure_lookup_table()
        for v in bm.verts:
            v.select = v.index in rec["v"]
        for e in bm.edges:
            e.select = e.index in rec["e"]
        for f in bm.faces:
            f.select = f.index in rec["f"]
        bmesh.update_edit_mesh(obj.data, loop_triangles=False, destructive=False)


class MESH_OT_topology_mirror_invoke(Operator):
    """Expand the mirrored selection, then run the original mesh operator."""
    bl_idname = "mesh.topology_mirror_invoke"
    bl_label = "Topology Mirror"
    bl_options = {'INTERNAL'}

    target: StringProperty()
    props_json: StringProperty(default="{}")
    restore_user_side: BoolProperty(default=False)

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH'

    def invoke(self, context, _event):
        return self._run(context, invoke=True)

    def execute(self, context):
        return self._run(context, invoke=False)

    def _run(self, context, invoke: bool):
        target = self.target
        kwargs = _json_kwargs(self.props_json)
        exec_ctx = 'INVOKE_DEFAULT' if invoke else 'EXEC_DEFAULT'

        if not target or not core.should_mirror(context):
            return _call_ops(target, exec_ctx, kwargs) if target else {'CANCELLED'}

        if target in ("mesh.loopcut_slide", "mesh.loopcut"):
            return bpy.ops.mesh.topology_mirror_loopcut('INVOKE_DEFAULT')

        captured = core.capture_context(context)
        snap = _snapshot_select(context)
        core.expand_context(context)

        split = _SPLIT_MACROS.get(target)
        if split is not None:
            first, second = split
            first_kwargs = {}
            second_kwargs = {}
            # Macro keymap stores nested MESH_OT_* / TRANSFORM_OT_* groups.
            for key, val in list(kwargs.items()):
                if key.startswith("MESH_OT_") or key.startswith("TRANSFORM_OT_"):
                    if key.startswith("TRANSFORM_OT_"):
                        second_kwargs = dict(val) if isinstance(val, dict) else {}
                    else:
                        first_kwargs = dict(val) if isinstance(val, dict) else {}
                else:
                    first_kwargs[key] = val
            result = _call_ops(first, 'EXEC_DEFAULT', first_kwargs or None)
            if result == {'CANCELLED'}:
                _restore_select(context, snap)
                return result
            core.restore_user_side_context(context, captured)
            return _call_ops(second, exec_ctx, second_kwargs or None)

        result = _call_ops(target, exec_ctx, kwargs)
        if result == {'CANCELLED'}:
            _restore_select(context, snap)
            return result
        if result == {'RUNNING_MODAL'}:
            return result
        if target in _RESTORE_SNAPSHOT:
            _restore_select(context, snap)
        elif self.restore_user_side:
            core.restore_user_side_context(context, captured)
        else:
            core.restore_active_context(context, captured)
        return result


class MESH_OT_topology_mirror_expand(Operator):
    """Select the mirrored counterparts of the current selection."""
    bl_idname = "mesh.topology_mirror_expand"
    bl_label = "Expand Mirror Selection"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH'

    def execute(self, context):
        if not core.expand_context(context):
            self.report({'INFO'}, "No mirror counterparts (enable X/Y/Z Mirror)")
            return {'CANCELLED'}
        return {'FINISHED'}


# ---------------------------------------------------------------------------
# Override Python extrude operators (same bl_idname as the builtins)
# ---------------------------------------------------------------------------

def _extrude_split(context, region_op: str, transform_op: str, transform_kwargs: dict,
                   fallback_macro: str | None = None, fallback_kwargs: dict | None = None):
    if not core.should_mirror(context) and fallback_macro:
        return _call_ops(fallback_macro, 'INVOKE_REGION_WIN', fallback_kwargs)
    captured = core.capture_context(context)
    if core.should_mirror(context):
        core.expand_context(context)
    result = _call_ops(region_op, 'EXEC_DEFAULT', None)
    if result == {'CANCELLED'}:
        return result
    if core.should_mirror(context):
        core.restore_user_side_context(context, captured)
    return _call_ops(transform_op, 'INVOKE_REGION_WIN', transform_kwargs)


class VIEW3D_OT_edit_mesh_extrude_move(Operator):
    """Extrude region together along the average normal"""
    bl_label = "Extrude and Move on Normals"
    bl_idname = "view3d.edit_mesh_extrude_move_normal"

    dissolve_and_intersect: BoolProperty(
        name="Dissolve and Intersect",
        default=False,
        description="Dissolves adjacent faces and intersects new geometry",
    )

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH'

    def execute(self, context):
        from bpy_extras.object_utils import object_report_if_active_shape_key_is_locked

        ob = context.object
        if ob is not None and object_report_if_active_shape_key_is_locked(ob, self):
            return {'CANCELLED'}

        mesh = ob.data
        totface = mesh.total_face_sel
        totedge = mesh.total_edge_sel

        tx = {
            "orient_type": 'NORMAL',
            "constraint_axis": (False, False, True),
            "release_confirm": False,
        }
        if totface >= 1:
            if self.dissolve_and_intersect:
                if core.should_mirror(context):
                    captured = core.capture_context(context)
                    core.expand_context(context)
                    result = bpy.ops.mesh.extrude_manifold(
                        'INVOKE_REGION_WIN',
                        MESH_OT_extrude_region={"use_dissolve_ortho_edges": True},
                        TRANSFORM_OT_translate=tx,
                    )
                    if result != {'CANCELLED'}:
                        core.restore_active_context(context, captured)
                    return result
                return bpy.ops.mesh.extrude_manifold(
                    'INVOKE_REGION_WIN',
                    MESH_OT_extrude_region={"use_dissolve_ortho_edges": True},
                    TRANSFORM_OT_translate=tx,
                )
            return _extrude_split(
                context,
                "mesh.extrude_region",
                "transform.translate",
                tx,
                fallback_macro="mesh.extrude_region_move",
                fallback_kwargs={"TRANSFORM_OT_translate": tx},
            )
        if totedge == 1:
            tx_free = {"constraint_axis": (False, False, False), "release_confirm": False}
            return _extrude_split(
                context,
                "mesh.extrude_region",
                "transform.translate",
                tx_free,
                fallback_macro="mesh.extrude_region_move",
                fallback_kwargs={"TRANSFORM_OT_translate": tx_free},
            )
        tx_any = {"release_confirm": False}
        return _extrude_split(
            context,
            "mesh.extrude_region",
            "transform.translate",
            tx_any,
            fallback_macro="mesh.extrude_region_move",
            fallback_kwargs={"TRANSFORM_OT_translate": tx_any},
        )

    def invoke(self, context, _event):
        return self.execute(context)


class VIEW3D_OT_edit_mesh_extrude_individual_move(Operator):
    """Extrude each individual face separately along local normals"""
    bl_label = "Extrude Individual and Move"
    bl_idname = "view3d.edit_mesh_extrude_individual_move"

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH'

    def execute(self, context):
        from bpy_extras.object_utils import object_report_if_active_shape_key_is_locked

        ob = context.object
        if ob is not None and object_report_if_active_shape_key_is_locked(ob, self):
            return {'CANCELLED'}

        mesh = ob.data
        select_mode = context.tool_settings.mesh_select_mode
        totface = mesh.total_face_sel
        totedge = mesh.total_edge_sel

        if select_mode[2] and totface == 1:
            tx = {
                "orient_type": 'NORMAL',
                "constraint_axis": (False, False, True),
                "release_confirm": False,
            }
            return _extrude_split(
                context, "mesh.extrude_region", "transform.translate", tx,
                fallback_macro="mesh.extrude_region_move",
                fallback_kwargs={"TRANSFORM_OT_translate": tx},
            )
        if select_mode[2] and totface > 1:
            return _extrude_split(
                context, "mesh.extrude_faces_indiv", "transform.shrink_fatten",
                {"release_confirm": False},
                fallback_macro="mesh.extrude_faces_move",
                fallback_kwargs={"TRANSFORM_OT_shrink_fatten": {"release_confirm": False}},
            )
        if select_mode[1] and totedge >= 1:
            return _extrude_split(
                context, "mesh.extrude_edges_indiv", "transform.translate",
                {"release_confirm": False},
                fallback_macro="mesh.extrude_edges_move",
                fallback_kwargs={"TRANSFORM_OT_translate": {"release_confirm": False}},
            )
        return _extrude_split(
            context, "mesh.extrude_verts_indiv", "transform.translate",
            {"release_confirm": False},
            fallback_macro="mesh.extrude_vertices_move",
            fallback_kwargs={"TRANSFORM_OT_translate": {"release_confirm": False}},
        )

    def invoke(self, context, _event):
        return self.execute(context)


class VIEW3D_OT_edit_mesh_extrude_shrink_fatten(Operator):
    """Extrude region together along local normals"""
    bl_label = "Extrude and Move on Individual Normals"
    bl_idname = "view3d.edit_mesh_extrude_move_shrink_fatten"

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH'

    def execute(self, context):
        from bpy_extras.object_utils import object_report_if_active_shape_key_is_locked

        ob = context.object
        if ob is not None and object_report_if_active_shape_key_is_locked(ob, self):
            return {'CANCELLED'}
        return _extrude_split(
            context,
            "mesh.extrude_region",
            "transform.shrink_fatten",
            {"release_confirm": False},
            fallback_macro="mesh.extrude_region_shrink_fatten",
            fallback_kwargs={"TRANSFORM_OT_shrink_fatten": {"release_confirm": False}},
        )

    def invoke(self, context, _event):
        return self.execute(context)


# ---------------------------------------------------------------------------
# Mirrored loop cut
# ---------------------------------------------------------------------------

def _edge_ring(edge):
    ring = []
    visited = set()
    stack = [edge]
    while stack:
        e = stack.pop()
        if e in visited:
            continue
        visited.add(e)
        ring.append(e)
        for face in e.link_faces:
            if len(face.verts) != 4:
                continue
            for loop in face.loops:
                if loop.edge == e:
                    opp = loop.link_loop_next.link_loop_next.edge
                    if opp not in visited:
                        stack.append(opp)
    return ring


def _closest_edge(context, obj, coord):
    region = context.region
    rv3d = context.region_data
    if region is None or rv3d is None:
        return None
    origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord)
    direction = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord)
    imw = obj.matrix_world.inverted()
    ray_orig = imw @ origin
    ray_dir = (imw.to_3x3() @ direction).normalized()
    bm = bmesh.from_edit_mesh(obj.data)
    best = None
    best_dist = 0.03
    # Distance in 3D from ray to edge, biased to near-camera hits.
    for edge in bm.edges:
        if edge.hide:
            continue
        a = Vector(edge.verts[0].co)
        b = Vector(edge.verts[1].co)
        ab = b - a
        # Closest points between ray and segment.
        d1 = ray_dir
        d2 = ab
        w = ray_orig - a
        a_dot = d1.dot(d1)
        b_dot = d1.dot(d2)
        c_dot = d2.dot(d2)
        d_dot = d1.dot(w)
        e_dot = d2.dot(w)
        denom = a_dot * c_dot - b_dot * b_dot
        if abs(denom) < 1e-12:
            continue
        sc = (b_dot * e_dot - c_dot * d_dot) / denom
        tc = (a_dot * e_dot - b_dot * d_dot) / denom
        tc = max(0.0, min(1.0, tc))
        if sc < 0.0:
            continue
        p1 = ray_orig + d1 * sc
        p2 = a + d2 * tc
        dist = (p1 - p2).length
        # Screen-space-ish: prefer edges closer to the camera.
        dist *= (1.0 + sc * 0.15)
        if dist < best_dist:
            best_dist = dist
            best = edge
    return best


def _draw_rings_callback(op):
    import gpu
    from gpu_extras.batch import batch_for_shader

    shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    gpu.state.blend_set('ALPHA')
    gpu.state.depth_test_set('LESS_EQUAL')
    gpu.state.line_width_set(2.0)
    color = (0.25, 0.85, 0.95, 0.95)
    verts = []
    for a, b in op._preview_segments:
        verts.append(a)
        verts.append(b)
    if verts:
        batch = batch_for_shader(shader, 'LINES', {"pos": verts})
        shader.bind()
        shader.uniform_float("color", color)
        batch.draw(shader)
    gpu.state.depth_test_set('NONE')
    gpu.state.blend_set('NONE')
    gpu.state.line_width_set(1.0)


class MESH_OT_topology_mirror_loopcut(Operator):
    """Loop cut on the hovered ring and its mirrored counterpart."""
    bl_idname = "mesh.topology_mirror_loopcut"
    bl_label = "Loop Cut (Topology Mirror)"
    bl_options = {'REGISTER', 'UNDO'}

    number_cuts: IntProperty(name="Number of Cuts", default=1, min=1, max=500)

    def invoke(self, context, event):
        if context.space_data is None or context.space_data.type != 'VIEW_3D':
            return self.execute(context)
        self._preview_segments = []
        self._hover = None  # (obj, edge_index)
        self._handle = bpy.types.SpaceView3D.draw_handler_add(
            _draw_rings_callback, (self,), 'WINDOW', 'POST_VIEW')
        context.window_manager.modal_handler_add(self)
        self._update_hover(context, event)
        return {'RUNNING_MODAL'}

    def modal(self, context, event):
        if event.type in {'MIDDLEMOUSE', 'WHEELUPMOUSE', 'WHEELDOWNMOUSE'} and event.alt:
            return {'PASS_THROUGH'}
        if event.type == 'MOUSEMOVE':
            self._update_hover(context, event)
            context.area.tag_redraw()
            return {'RUNNING_MODAL'}
        if event.type in {'WHEELUPMOUSE', 'NUMPAD_PLUS'} and event.value == 'PRESS':
            self.number_cuts = min(500, self.number_cuts + 1)
            self._update_hover(context, event)
            context.area.tag_redraw()
            return {'RUNNING_MODAL'}
        if event.type in {'WHEELDOWNMOUSE', 'NUMPAD_MINUS'} and event.value == 'PRESS':
            self.number_cuts = max(1, self.number_cuts - 1)
            self._update_hover(context, event)
            context.area.tag_redraw()
            return {'RUNNING_MODAL'}
        if event.type in {'LEFTMOUSE', 'RET', 'NUMPAD_ENTER'} and event.value == 'PRESS':
            self._cleanup(context)
            return self._cut(context)
        if event.type in {'RIGHTMOUSE', 'ESC'}:
            self._cleanup(context)
            return {'CANCELLED'}
        return {'RUNNING_MODAL'}

    def execute(self, context):
        return self._cut(context)

    def _cleanup(self, context):
        if getattr(self, "_handle", None) is not None:
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, 'WINDOW')
            self._handle = None
        if context.area:
            context.area.tag_redraw()

    def _update_hover(self, context, event):
        coord = (event.mouse_region_x, event.mouse_region_y)
        self._hover = None
        self._preview_segments = []
        best_obj = None
        best_edge = None
        for obj in core.edit_objects(context):
            edge = _closest_edge(context, obj, coord)
            if edge is not None:
                best_obj = obj
                best_edge = edge
                break
        if best_obj is None or best_edge is None:
            return
        self._hover = (best_obj.name, best_edge.index)
        self._preview_segments = self._ring_segments(best_obj, best_edge)

    def _ring_segments(self, obj, edge):
        mesh = obj.data
        bm = bmesh.from_edit_mesh(mesh)
        starts = [edge]
        if core.mesh_axes(mesh):
            starts.extend(core.find_mirror_edge(bm, mesh, edge))
        mw = obj.matrix_world
        segs = []
        seen = set()
        for start in starts:
            for e in _edge_ring(start):
                if e.index in seen:
                    continue
                seen.add(e.index)
                segs.append((mw @ e.verts[0].co, mw @ e.verts[1].co))
        return segs

    def _cut(self, context):
        obj = None
        edge = None
        hover = getattr(self, "_hover", None)
        if hover:
            obj = bpy.data.objects.get(hover[0])
            if obj is not None and obj.mode == 'EDIT':
                bm = bmesh.from_edit_mesh(obj.data)
                bm.edges.ensure_lookup_table()
                idx = hover[1]
                if 0 <= idx < len(bm.edges):
                    edge = bm.edges[idx]
        if obj is None or edge is None:
            self.report({'WARNING'}, "No edge under cursor")
            return {'CANCELLED'}

        mesh = obj.data
        bm = bmesh.from_edit_mesh(mesh)
        starts = [edge]
        if core.should_mirror(context) and core.mesh_axes(mesh):
            starts.extend(core.find_mirror_edge(bm, mesh, edge))

        cut_edges = []
        seen = set()
        for start in starts:
            for e in _edge_ring(start):
                if e.index not in seen:
                    seen.add(e.index)
                    cut_edges.append(e)
        if not cut_edges:
            return {'CANCELLED'}

        result = bmesh.ops.subdivide_edges(
            bm,
            edges=cut_edges,
            cuts=self.number_cuts,
            use_grid_fill=True,
            use_single_edge=True,
            quad_corner_type='PATH',
        )
        for ele in bm.edges:
            ele.select = False
        for ele in bm.faces:
            ele.select = False
        for ele in bm.verts:
            ele.select = False
        for geom in result.get("geom", ()):
            if isinstance(geom, bmesh.types.BMEdge):
                geom.select = True
            elif isinstance(geom, bmesh.types.BMVert):
                geom.select = True
        bm.select_flush_mode()
        bmesh.update_edit_mesh(mesh)
        try:
            return bpy.ops.transform.edge_slide('INVOKE_DEFAULT')
        except RuntimeError:
            return {'FINISHED'}


classes = (
    MESH_OT_topology_mirror_invoke,
    MESH_OT_topology_mirror_expand,
    MESH_OT_topology_mirror_loopcut,
    VIEW3D_OT_edit_mesh_extrude_move,
    VIEW3D_OT_edit_mesh_extrude_individual_move,
    VIEW3D_OT_edit_mesh_extrude_shrink_fatten,
)
