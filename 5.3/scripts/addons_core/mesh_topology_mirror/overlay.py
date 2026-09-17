# SPDX-FileCopyrightText: 2026 BIKINI
# SPDX-License-Identifier: GPL-3.0-or-later
"""Viewport overlay: cyan highlight of unselected mirror counterparts."""

from __future__ import annotations

import bpy
import gpu
from gpu_extras.batch import batch_for_shader

from . import core

_handle = None
_cache_key = None
_cache_geom = None


def _overlay_enabled(context) -> bool:
    prefs = core.addon_prefs()
    if prefs is not None and not prefs.show_overlay:
        return False
    return context.mode == 'EDIT_MESH' and core.should_mirror(context)


def _theme_color(context):
    prefs = core.addon_prefs()
    if prefs is not None:
        c = prefs.overlay_color
        return (c[0], c[1], c[2], c[3])
    try:
        col = context.preferences.themes[0].view_3d.edge_sharp
        return (col[0], col[1], col[2], 0.95)
    except Exception:
        return (0.25, 0.85, 0.95, 0.90)


def _cache_signature(context):
    parts = []
    for obj in core.edit_objects(context):
        mesh = obj.data
        parts.append((
            obj.as_pointer(),
            len(mesh.vertices),
            len(mesh.edges),
            len(mesh.polygons),
            mesh.total_vert_sel,
            mesh.total_edge_sel,
            mesh.total_face_sel,
            core.mesh_axes(mesh),
            bool(mesh.use_mirror_topology),
        ))
    return tuple(parts)


def _geom(context):
    global _cache_key, _cache_geom
    key = _cache_signature(context)
    if key == _cache_key and _cache_geom is not None:
        return _cache_geom
    verts = []
    edges = []
    faces = []
    for obj in core.edit_objects(context):
        mesh = obj.data
        if not core.mesh_axes(mesh):
            continue
        v, e, f = core.mirror_partners_world(obj, mesh)
        verts.extend(v)
        edges.extend(e)
        faces.extend(f)
    _cache_key = key
    _cache_geom = (verts, edges, faces)
    return _cache_geom


def _draw():
    context = bpy.context
    if not _overlay_enabled(context):
        return
    verts, edges, faces = _geom(context)
    if not verts and not edges and not faces:
        return

    shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    color = _theme_color(context)
    gpu.state.blend_set('ALPHA')
    gpu.state.depth_test_set('LESS_EQUAL')
    gpu.state.depth_mask_set(False)

    shader.bind()
    shader.uniform_float("color", color)

    if faces:
        tri_pos = []
        for poly in faces:
            if len(poly) < 3:
                continue
            origin = poly[0]
            for i in range(1, len(poly) - 1):
                tri_pos.extend((origin, poly[i], poly[i + 1]))
        if tri_pos:
            face_color = (color[0], color[1], color[2], color[3] * 0.22)
            shader.uniform_float("color", face_color)
            batch_for_shader(shader, 'TRIS', {"pos": tri_pos}).draw(shader)
            shader.uniform_float("color", color)

    if edges:
        line_pos = []
        for a, b in edges:
            line_pos.append(a)
            line_pos.append(b)
        gpu.state.line_width_set(2.0)
        batch_for_shader(shader, 'LINES', {"pos": line_pos}).draw(shader)
        gpu.state.line_width_set(1.0)

    if verts:
        gpu.state.point_size_set(7.0)
        batch_for_shader(shader, 'POINTS', {"pos": verts}).draw(shader)
        gpu.state.point_size_set(1.0)

    gpu.state.depth_mask_set(True)
    gpu.state.depth_test_set('NONE')
    gpu.state.blend_set('NONE')


def invalidate():
    global _cache_key, _cache_geom
    _cache_key = None
    _cache_geom = None


def register():
    global _handle
    if _handle is None:
        _handle = bpy.types.SpaceView3D.draw_handler_add(_draw, (), 'WINDOW', 'POST_VIEW')


def unregister():
    global _handle
    if _handle is not None:
        bpy.types.SpaceView3D.draw_handler_remove(_handle, 'WINDOW')
        _handle = None
    invalidate()
