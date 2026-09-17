# SPDX-FileCopyrightText: 2026 BIKINI
# SPDX-License-Identifier: GPL-3.0-or-later
"""Remap mesh-edit keymaps and menus onto topology-mirror wrappers."""

from __future__ import annotations

import json

import bpy

from .operators import WRAP_TARGETS, _SPLIT_MACROS

# Original keymap item state to restore on unregister.
_patched: list[tuple] = []
_menu_draws: list[tuple] = []

_LOOPCUT = {"mesh.loopcut_slide", "mesh.loopcut"}

_MENU_TYPES = (
    "VIEW3D_MT_edit_mesh",
    "VIEW3D_MT_edit_mesh_delete",
    "VIEW3D_MT_edit_mesh_merge",
    "VIEW3D_MT_edit_mesh_extrude",
    "VIEW3D_MT_edit_mesh_faces",
    "VIEW3D_MT_edit_mesh_edges",
    "VIEW3D_MT_edit_mesh_vertices",
    "VIEW3D_MT_edit_mesh_context_menu",
    "VIEW3D_MT_edit_mesh_split",
)


def _rna_to_jsonable(ptr) -> dict:
    if ptr is None:
        return {}
    out = {}
    try:
        props = ptr.bl_rna.properties
    except Exception:
        return {}
    for prop in props:
        ident = prop.identifier
        if ident == 'rna_type':
            continue
        try:
            val = getattr(ptr, ident)
        except Exception:
            continue
        if prop.type == 'POINTER':
            nested = _rna_to_jsonable(val)
            if nested:
                out[ident] = nested
            continue
        if prop.type == 'COLLECTION':
            continue
        if isinstance(val, (bytes, bytearray)):
            continue
        if isinstance(val, (int, float, str, bool)) or val is None:
            out[ident] = val
        else:
            try:
                out[ident] = list(val)
            except TypeError:
                pass
    return out


class _LayoutProxy:
    __slots__ = ("_layout",)

    def __init__(self, layout):
        self._layout = layout

    def operator(self, *args, **kwargs):
        if args:
            idname = args[0]
            if idname in WRAP_TARGETS or idname in _LOOPCUT:
                extra = dict(kwargs)
                props = extra.pop("properties", None)
                mapped = self._layout.operator(
                    "mesh.topology_mirror_invoke",
                    *args[1:],
                    **extra,
                )
                if mapped is not None:
                    mapped.target = idname
                    mapped.restore_user_side = idname in _SPLIT_MACROS
                    if idname in _LOOPCUT:
                        mapped.target = "mesh.loopcut_slide"
                    if props:
                        mapped.props_json = json.dumps(props)
                return mapped
        return self._layout.operator(*args, **kwargs)

    def __getattr__(self, name):
        attr = getattr(self._layout, name)
        if name in {
            "row", "column", "box", "split", "column_flow",
            "grid_flow", "menu_pie", "menu",
        }:
            def wrapped(*a, **k):
                result = attr(*a, **k)
                if name == "menu":
                    return result
                return _LayoutProxy(result)
            return wrapped
        return attr


class _MenuSelf:
    def __init__(self, real, layout):
        self._real = real
        self.layout = layout

    def __getattr__(self, name):
        return getattr(self._real, name)


def _wrap_menu_draw(orig):
    def draw(self, context):
        orig(_MenuSelf(self, _LayoutProxy(self.layout)), context)
    draw._tm_orig = orig
    return draw


def _apply_props(ptr, data: dict):
    if not data or ptr is None:
        return
    for key, val in data.items():
        if not hasattr(ptr, key):
            continue
        cur = getattr(ptr, key)
        if isinstance(val, dict):
            _apply_props(cur, val)
            continue
        try:
            setattr(ptr, key, val)
        except Exception:
            try:
                setattr(ptr, key, tuple(val) if isinstance(val, list) else val)
            except Exception:
                pass


def _patch_keymaps():
    wm = bpy.context.window_manager
    kc = wm.keyconfigs.user
    if kc is None:
        return
    for km in kc.keymaps:
        for kmi in km.keymap_items:
            if not kmi.idname:
                continue
            if kmi.idname == "mesh.topology_mirror_invoke":
                try:
                    orig_id = kmi.properties.target
                    orig_props = _json_to_dict(kmi.properties.props_json)
                except Exception:
                    continue
                if orig_id:
                    _patched.append((km, kmi, orig_id, orig_props))
                continue
            if kmi.idname not in WRAP_TARGETS and kmi.idname not in _LOOPCUT:
                continue
            orig_id = kmi.idname
            orig_props = _rna_to_jsonable(kmi.properties)
            try:
                kmi.idname = "mesh.topology_mirror_invoke"
            except Exception:
                continue
            try:
                kmi.properties.target = orig_id
                kmi.properties.props_json = json.dumps(orig_props)
                kmi.properties.restore_user_side = orig_id in _SPLIT_MACROS
            except Exception:
                kmi.idname = orig_id
                continue
            _patched.append((km, kmi, orig_id, orig_props))


def _json_to_dict(text: str) -> dict:
    if not text:
        return {}
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return {}
    return data if isinstance(data, dict) else {}


def _restore_keymaps():
    for _km, kmi, orig_id, orig_props in _patched:
        try:
            if kmi.idname == "mesh.topology_mirror_invoke":
                kmi.idname = orig_id
                _apply_props(kmi.properties, orig_props)
        except ReferenceError:
            pass
    _patched.clear()


def _patch_menus():
    for name in _MENU_TYPES:
        cls = getattr(bpy.types, name, None)
        if cls is None or not hasattr(cls, "draw"):
            continue
        orig = cls.draw
        if getattr(orig, "_tm_orig", None) is not None:
            continue
        cls.draw = _wrap_menu_draw(orig)
        _menu_draws.append((cls, orig))


def _restore_menus():
    for cls, orig in _menu_draws:
        try:
            cls.draw = orig
        except Exception:
            pass
    _menu_draws.clear()


@bpy.app.handlers.persistent
def _load_post(_dummy):
    # Keymaps persist with user prefs; nothing to re-patch unless restore failed.
    pass


def register():
    _patch_keymaps()
    _patch_menus()
    if _load_post not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_load_post)


def unregister():
    if _load_post in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_load_post)
    _restore_menus()
    _restore_keymaps()
