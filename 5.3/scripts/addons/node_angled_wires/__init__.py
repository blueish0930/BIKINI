# SPDX-FileCopyrightText: 2026
# SPDX-License-Identifier: GPL-3.0-or-later
"""Orthogonal Houdini-style node wires (Python overlay).

Supports Blender 3.6 LTS, 4.2 LTS, 4.5 LTS, 5.2 LTS, and 5.3 daily.
"""

from __future__ import annotations

import ctypes
import math
import os
import sys

import bpy
import gpu
from bpy.app.handlers import persistent
from gpu_extras.batch import batch_for_shader

bl_info = {
    "name": "Angled Node Wires",
    "author": "Local",
    "version": (1, 3, 45),
    "blender": (3, 6, 0),
    "location": "Node Editor > Overlays",
    "description": "Orthogonal node links without touching the node tree",
    "category": "Node",
}

CIRCLE_K = 0.5522847498
MIN_ROUND = 3.0
# Same constant as editors/space_node/drawnode.cc (`#define LINK_WIDTH 2.5f`).
LINK_WIDTH = 2.5
INSERT_WIDTH = 5.0
MAX_POLYLINE_WIDTH = 12.0
MAX_VIEW_COORD = 1.0e6
NODE_GRID_UNIT = 20.0
# Overlay framebuffer has depth. Smaller Z is nearer in view2d ortho (-100..100).
OCCLUDE_Z = -12.0
WIRE_Z = 8.0
DEPTH_FAR_Z = 80.0
PTR_SIZE = ctypes.sizeof(ctypes.c_void_p)
FILLET_STEPS = 4
_LINK_FLAG_OFF = 6 * PTR_SIZE
NODE_LINK_INSERT_TARGET = 1 << 0
NODE_LINK_INSERT_TARGET_INVALID = 1 << 5
_BL_VER = bpy.app.version
# 4.2+ draws noodles from bNodeTreeRuntime.links; 3.6 still walks DNA ListBase.
_DRAW_USES_RUNTIME_VEC = _BL_VER >= (4, 2, 0)
# 3.6 uses NODE_LINKFLAG_HILITE on bit 0; 4.2+ reused it as INSERT_TARGET.
_HAS_INSERT_TARGET = True
_WIN32 = sys.platform == "win32"

_SHADER_NAMES = (
    "POLYLINE_SMOOTH_COLOR",
    "3D_POLYLINE_SMOOTH_COLOR",
    "POLYLINE_UNIFORM_COLOR",
    "3D_POLYLINE_UNIFORM_COLOR",
    "SMOOTH_COLOR",
    "3D_SMOOTH_COLOR",
)

_draw_handle_pre = None
_draw_handle_post = None
_draw_handle_stroke = None
_shader = None
_fill_shader = None
_smooth_fill = None
_shader_has_line_width = True
_theme_backup = None
_runtime_off = None
_loc_off = None
_keymap_installed = False
_stroke_region = []
_path_cache = {
    "stamp": None,
    "batch": None,
    "batch_outline": None,
    "batch_under": None,
    "batch_under_outline": None,
    "batch_clip": None,
    "batch_clip_outline": None,
    "batch_insert": None,
    "batch_occlude": None,
    "batch_socks": None,
    "batch_frame": None,
    "batch_frame_outline": None,
}
_tree_runtime_off = None
_links_vec_off = None
_links_lb_off = None
_steer_insert_ptr = None
_idle_link_snapshot = []
_drag_anchor = None
_drag_op_key = None
_prefer_layout_xy = False
_nodes_are_dragging = False
_cache_node_locs = {}
_cache_n_links = -1
_pending_snap = False
_pre_synced = False
_force_wires = False
_was_transform_modal = False
_was_link_modal = False
_sock_local = {}
_wire_hide_ttl = 0
_theme_wire_saved = None
_redraw_left = 0

_SOCKET_COLORS = {
    "CUSTOM": (0.63, 0.63, 0.63, 1.0),
    "VALUE": (0.63, 0.63, 0.63, 1.0),
    "FLOAT": (0.63, 0.63, 0.63, 1.0),
    "INT": (0.35, 0.55, 0.36, 1.0),
    "BOOLEAN": (0.80, 0.65, 0.84, 1.0),
    "VECTOR": (0.39, 0.39, 0.78, 1.0),
    "ROTATION": (0.65, 0.39, 0.78, 1.0),
    "MATRIX": (0.72, 0.20, 0.52, 1.0),
    "RGBA": (0.78, 0.78, 0.16, 1.0),
    "STRING": (0.44, 0.70, 1.00, 1.0),
    "SHADER": (0.39, 0.78, 0.39, 1.0),
    "GEOMETRY": (0.00, 0.84, 0.64, 1.0),
    "OBJECT": (0.93, 0.62, 0.36, 1.0),
    "COLLECTION": (0.96, 0.96, 0.96, 1.0),
    "IMAGE": (0.39, 0.22, 0.39, 1.0),
    "MATERIAL": (0.92, 0.46, 0.51, 1.0),
    "TEXTURE": (0.62, 0.31, 0.64, 1.0),
    "MENU": (0.40, 0.40, 0.40, 1.0),
    "BUNDLE": (0.30, 0.50, 0.50, 1.0),
    "CLOSURE": (0.49, 0.49, 0.23, 1.0),
    "NodeSocketBundle": (0.30, 0.50, 0.50, 1.0),
    "NodeSocketClosure": (0.49, 0.49, 0.23, 1.0),
    "NodeSocketVirtual": (0.20, 0.20, 0.20, 1.0),
    "FONT": (0.39, 0.34, 0.26, 1.0),
    "SCENE": (0.00, 0.00, 0.00, 1.0),
    "TEXT": (0.00, 0.00, 0.00, 1.0),
    "MASK": (0.00, 0.00, 0.00, 1.0),
    "SOUND": (0.39, 0.34, 0.26, 1.0),
    "INT_VECTOR": (0.36, 0.47, 0.61, 1.0),
}

_REMAP = {
    "node.add_reroute": "node.angled_add_reroute",
    "node.links_cut": "node.angled_links_cut",
    "node.links_mute": "node.angled_links_mute",
}

_k32 = ctypes.windll.kernel32 if _WIN32 else None
MEM_COMMIT = 0x1000
PAGE_GUARD = 0x100
_RUNTIME_CANDIDATES = (
    456, 520, 448, 464, 472, 480, 488, 496, 504, 512, 528, 536, 544, 552, 432, 440,
)
_LOC_CANDIDATES = (16, 24, 32, 8, 40, 48, 0)
try:
    _PAGE = int(os.sysconf("SC_PAGESIZE")) if not _WIN32 else 4096
except Exception:
    _PAGE = 4096
_page_cache = {}
_libc = None
_mincore = None
_mach_vm_region = None
_mach_task = 0
_posix_mem_tried = False


class _MBI(ctypes.Structure):
    _fields_ = (
        ("BaseAddress", ctypes.c_void_p),
        ("AllocationBase", ctypes.c_void_p),
        ("AllocationProtect", ctypes.c_uint32),
        ("PartitionId", ctypes.c_uint16),
        ("__pad", ctypes.c_uint16),
        ("RegionSize", ctypes.c_size_t),
        ("State", ctypes.c_uint32),
        ("Protect", ctypes.c_uint32),
        ("Type", ctypes.c_uint32),
    )


class _MachRegionInfo(ctypes.Structure):
    _fields_ = (
        ("protection", ctypes.c_int32),
        ("max_protection", ctypes.c_int32),
        ("inheritance", ctypes.c_uint32),
        ("shared", ctypes.c_int32),
        ("reserved", ctypes.c_int32),
        ("offset", ctypes.c_uint64),
        ("behavior", ctypes.c_int32),
        ("user_wired_count", ctypes.c_uint16),
    )


def _init_posix_mem():
    global _libc, _mincore, _mach_vm_region, _mach_task, _posix_mem_tried
    if _WIN32 or _posix_mem_tried:
        return
    _posix_mem_tried = True
    names = (
        ("/usr/lib/libSystem.B.dylib",) if sys.platform == "darwin" else ("libc.so.6", None)
    )
    for name in names:
        try:
            _libc = ctypes.CDLL(name) if name else ctypes.CDLL(None)
            break
        except OSError:
            continue
    if _libc is None:
        return
    try:
        _mincore = _libc.mincore
        _mincore.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_char_p]
        _mincore.restype = ctypes.c_int
    except Exception:
        _mincore = None
    if sys.platform != "darwin":
        return
    try:
        fn = getattr(_libc, "mach_vm_region", None)
        if fn is None:
            return
        fn.restype = ctypes.c_int
        fn.argtypes = [
            ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_uint64),
            ctypes.POINTER(ctypes.c_uint64),
            ctypes.c_int,
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint32),
        ]
        task_fn = getattr(_libc, "mach_task_self", None) or getattr(_libc, "mach_task_self_", None)
        if task_fn is None:
            return
        task_fn.restype = ctypes.c_uint32
        _mach_task = int(task_fn())
        _mach_vm_region = fn
    except Exception:
        _mach_vm_region = None


def _page_ok_uncached(page):
    if _WIN32:
        mbi = _MBI()
        n = _k32.VirtualQuery(ctypes.c_void_p(int(page)), ctypes.byref(mbi), ctypes.sizeof(mbi))
        if not n or int(mbi.State) != MEM_COMMIT:
            return False
        prot = int(mbi.Protect)
        if (prot & 0xFF) in (0x00, 0x01) or (prot & PAGE_GUARD):
            return False
        return True
    _init_posix_mem()
    if _mach_vm_region is not None:
        address = ctypes.c_uint64(int(page))
        size = ctypes.c_uint64(0)
        info = _MachRegionInfo()
        count = ctypes.c_uint32(ctypes.sizeof(info) // 4)
        obj = ctypes.c_uint32(0)
        kr = _mach_vm_region(
            _mach_task,
            ctypes.byref(address),
            ctypes.byref(size),
            9,
            ctypes.byref(info),
            ctypes.byref(count),
            ctypes.byref(obj),
        )
        if kr != 0 or int(size.value) <= 0:
            return False
        if int(page) < int(address.value) or int(page) >= int(address.value) + int(size.value):
            return False
        return bool(int(info.protection) & 1)
    if _mincore is not None:
        vec = ctypes.create_string_buffer(1)
        return _mincore(ctypes.c_void_p(int(page)), ctypes.c_size_t(_PAGE), vec) == 0
    return False


def _page_ok(page):
    hit = _page_cache.get(page)
    if hit is not None:
        return hit
    ok = _page_ok_uncached(page)
    if len(_page_cache) > 4096:
        _page_cache.clear()
    _page_cache[page] = ok
    return ok


def _readable_range(addr, size):
    if not addr or int(addr) < 0x10000 or int(size) <= 0:
        return False
    start = int(addr)
    last = start + int(size) - 1
    page = start & ~(_PAGE - 1)
    end_page = last & ~(_PAGE - 1)
    while page <= end_page:
        if not _page_ok(page):
            return False
        page += _PAGE
    return True


def _readable(addr):
    return _readable_range(addr, 8)


def _u64(addr):
    return int(ctypes.c_uint64.from_address(int(addr)).value)


def _u64_safe(addr):
    if not _readable_range(addr, 8):
        return None
    try:
        return _u64(addr)
    except Exception:
        return None


# ---------------------------------------------------------------------------
# UI scale / theme
# ---------------------------------------------------------------------------

def _ui_scale():
    prefs = bpy.context.preferences.system
    try:
        dpi = float(getattr(prefs, "dpi", 0) or 0)
        if dpi > 1.0:
            s = dpi / 72.0
            if math.isfinite(s) and s > 0.05:
                return s
    except Exception:
        pass
    try:
        ui = float(getattr(prefs, "ui_scale", 1.0) or 1.0)
        px = float(getattr(prefs, "pixel_size", 1.0) or 1.0)
        s = ui * px
        if math.isfinite(s) and s > 0.05:
            return s
    except Exception:
        pass
    return 1.0


_ZONE_INPUT_IDNAMES = frozenset({
    "GeometryNodeSimulationInput",
    "GeometryNodeRepeatInput",
    "GeometryNodeForeachGeometryElementInput",
    "NodeClosureInput",
    "ImageNodeFluidSimInput",
    "ShaderNodeLightIterInternalInput",
})
_ZONE_OUTPUT_BY_INPUT = {
    "GeometryNodeSimulationInput": "GeometryNodeSimulationOutput",
    "GeometryNodeRepeatInput": "GeometryNodeRepeatOutput",
    "GeometryNodeForeachGeometryElementInput": "GeometryNodeForeachGeometryElementOutput",
    "NodeClosureInput": "NodeClosureOutput",
    "ImageNodeFluidSimInput": "ImageNodeFluidSimOutput",
    "ShaderNodeLightIterInternalInput": "ShaderNodeLightIterInternalOutput",
}
_ZONE_ENDPOINT_IDNAMES = _ZONE_INPUT_IDNAMES | frozenset(_ZONE_OUTPUT_BY_INPUT.values())
_GTE_TREE_IDS = frozenset({
    "GPUTextureEditorNodeTree",
    "GPUTextureEditorTree",
})


def _is_gte_tree(tree):
    if tree is None:
        return False
    return (getattr(tree, "bl_idname", "") or "") in _GTE_TREE_IDS


def _noodle_curving():
    try:
        return int(bpy.context.preferences.themes[0].node_editor.noodle_curving)
    except Exception:
        return 4


def _theme_zero(prop):
    values = tuple(prop)
    if len(values) >= 4:
        return values[:-1] + (0.0,)
    return None


def _ensure_wire_theme_visible():
    """Socket outlines and dragged noodles use TH_WIRE. Never leave them at alpha 0."""
    global _theme_backup
    if _theme_wire_saved is not None:
        return
    try:
        ne = bpy.context.preferences.themes[0].node_editor
        if _theme_backup:
            for name, value in _theme_backup.items():
                if hasattr(ne, name):
                    setattr(ne, name, value)
            _theme_backup = None
            return
        for name in ("wire", "wire_inner", "wire_select"):
            if not hasattr(ne, name):
                continue
            col = list(getattr(ne, name))
            if len(col) >= 4 and float(col[3]) < 0.05:
                col[3] = 1.0
                setattr(ne, name, tuple(col))
    except Exception:
        pass


def _theme_hide_native_wires(hide):
    """Temporarily zero TH_WIRE so C++ cannot redraw bezier after a topology rebuild."""
    global _theme_wire_saved
    try:
        ne = bpy.context.preferences.themes[0].node_editor
    except Exception:
        return
    names = ("wire", "wire_inner")
    if hide:
        if _theme_wire_saved is None:
            saved = {}
            for name in names:
                if hasattr(ne, name):
                    saved[name] = tuple(getattr(ne, name))
            _theme_wire_saved = saved
        for name in names:
            if not hasattr(ne, name):
                continue
            col = list(getattr(ne, name))
            if len(col) >= 4:
                col[3] = 0.0
                setattr(ne, name, tuple(col))
        return
    if not _theme_wire_saved:
        return
    try:
        for name, value in _theme_wire_saved.items():
            if hasattr(ne, name):
                setattr(ne, name, value)
    except Exception:
        pass
    _theme_wire_saved = None


def _apply_theme_hide(_hide):
    # Tree noodles are hidden by emptying runtime.links. Zeroing the wire theme
    # also hides dragged links and socket outlines (TH_WIRE).
    _ensure_wire_theme_visible()


# ---------------------------------------------------------------------------
# Socket location — versioned DNA + one-time probe
# ---------------------------------------------------------------------------

def _node_abs_loc(node):
    loc = getattr(node, "location_absolute", None)
    if loc is not None:
        try:
            return float(loc[0]), float(loc[1])
        except Exception:
            pass
    x = float(node.location[0])
    y = float(node.location[1])
    parent = node.parent
    hops = 0
    while parent is not None and hops < 64:
        x += float(parent.location[0])
        y += float(parent.location[1])
        parent = parent.parent
        hops += 1
    return x, y


def _socket_layout_offsets():
    # 3.6: no short_label → runtime at 456
    # 4.0–5.0: short_label[64] → runtime at 520
    # 5.1+: short_label removed → runtime at 456
    if _BL_VER < (4, 0, 0) or _BL_VER >= (5, 1, 0):
        runtime_off = 456
    else:
        runtime_off = 520
    # 5.2+: UString before location (24), older runtimes keep location at 16
    loc_off = 24 if _BL_VER >= (5, 2, 0) else 16
    if _WIN32:
        loc_off += 8
    return runtime_off, loc_off


def _read_socket_xy(socket, runtime_off, loc_off):
    base = int(socket.as_pointer()) + int(runtime_off)
    if not _readable_range(base, PTR_SIZE):
        return None
    runtime = ctypes.c_void_p.from_address(base).value
    if not runtime:
        return None
    loc = int(runtime) + int(loc_off)
    if not _readable_range(loc, 8):
        return None
    pair = (ctypes.c_float * 2).from_address(loc)
    x, y = float(pair[0]), float(pair[1])
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    if max(abs(x), abs(y)) > 1.0e7:
        return None
    return x, y


def _xy_uninitialized(xy):
    return xy is not None and abs(xy[0]) < 0.05 and abs(xy[1]) < 0.05


def _xy_plausible(xy, node, socket, scale, slop=160.0):
    if xy is None:
        return False
    x, y = xy
    nx, ny = _node_abs_loc(node)
    sx = nx * scale
    sy = ny * scale
    width = float(getattr(node, "width", 140.0)) * scale
    if getattr(node, "bl_idname", "") == "NodeReroute":
        return math.hypot(x - sx, y - sy) < slop
    if _is_gte_tree(getattr(node, "id_data", None)):
        height = 140.0 * scale
        try:
            dimx, dimy = node.dimensions
            if dimx > 1.0:
                width = float(dimx)
            if dimy > 1.0:
                height = float(dimy)
        except Exception:
            pass
        if not (sy + slop >= y >= sy - height - slop):
            return False
        if socket.is_output:
            return abs(x - (sx + width)) < slop
        return abs(x - sx) < slop
    if socket.is_output:
        return abs(x - (sx + width)) < slop and (sy + 120.0) >= y >= (sy - 900.0)
    return abs(x - sx) < slop and (sy + 120.0) >= y >= (sy - 900.0)


def _probe_sock_offs(socket):
    node = getattr(socket, "node", None)
    guess = _socket_layout_offsets()
    if node is None:
        return guess
    scale = _ui_scale() or 1.0
    try:
        xy = _read_socket_xy(socket, guess[0], guess[1])
        if xy is not None and (_xy_uninitialized(xy) or _xy_plausible(xy, node, socket, scale)):
            return guess
    except Exception:
        pass
    for runtime_off in _RUNTIME_CANDIDATES:
        for loc_off in _LOC_CANDIDATES:
            if (runtime_off, loc_off) == guess:
                continue
            try:
                xy = _read_socket_xy(socket, runtime_off, loc_off)
            except Exception:
                continue
            if _xy_uninitialized(xy):
                continue
            if _xy_plausible(xy, node, socket, scale):
                return runtime_off, loc_off
    return guess


def _ensure_sock_offs(socket=None):
    global _runtime_off, _loc_off
    if _runtime_off is None:
        if socket is not None:
            _runtime_off, _loc_off = _probe_sock_offs(socket)
        else:
            _runtime_off, _loc_off = _socket_layout_offsets()
    return _runtime_off, _loc_off


def socket_xy(socket):
    runtime_off, loc_off = _ensure_sock_offs(socket)
    try:
        return _read_socket_xy(socket, runtime_off, loc_off)
    except Exception:
        return None


def _write_socket_xy(socket, x, y):
    runtime_off, loc_off = _ensure_sock_offs(socket)
    try:
        base = int(socket.as_pointer()) + int(runtime_off)
        if not _readable_range(base, PTR_SIZE):
            return False
        runtime = ctypes.c_void_p.from_address(base).value
        loc = int(runtime or 0) + int(loc_off)
        if not runtime or not _readable_range(loc, 8):
            return False
        pair = (ctypes.c_float * 2).from_address(loc)
        pair[0] = float(x)
        pair[1] = float(y)
        return True
    except Exception:
        return False


def _gte_node_rect_view(node, scale):
    """View-space AABB matching GPU Texture Editor ui._node_rect."""
    locx, locy = _node_abs_loc(node)
    x = locx * scale
    y = locy * scale
    dx = float(getattr(node, "width", 140.0) or 140.0) * scale
    dy = 140.0 * scale
    try:
        dimx, dimy = node.dimensions
        if dimx > 1.0:
            dx = float(dimx)
        if dimy > 1.0:
            dy = float(dimy)
    except Exception:
        pass
    if dx < 8.0:
        dx = float(getattr(node, "width", 140.0) or 140.0) * scale
    if dy < 8.0:
        dy = 140.0 * scale
    return (x, y - dy, x + dx, y)


def _layout_socket_xy(socket, node, scale):
    locx, locy = _node_abs_loc(node)
    if node.bl_idname == "NodeReroute":
        return (locx * scale, locy * scale)
    if _is_gte_tree(getattr(node, "id_data", None)):
        return _gte_layout_socket_xy(socket, node, scale)
    width = float(getattr(node, "width", 140.0))
    coll = node.inputs if not socket.is_output else node.outputs
    visible = [s for s in coll if not getattr(s, "hide", False)]
    try:
        index = visible.index(socket)
    except ValueError:
        index = 0
    y = locy - 22.0 * (index + 0.65)
    x = locx if not socket.is_output else locx + width
    return (x * scale, y * scale)


def _gte_layout_socket_xy(socket, node, scale):
    """Fallback only: C++ custom nodes draw outputs, draw_buttons, then inputs.

    Vector/color rows are taller than one grid unit, so runtime.location is the
    real attachment. This packing is used before C++ has written sockets.
    """
    x0, y0, x1, y1 = _gte_node_rect_view(node, scale)
    unit = NODE_GRID_UNIT * scale
    header = unit
    is_out = bool(getattr(socket, "is_output", False))
    coll = node.outputs if is_out else node.inputs
    visible = [s for s in coll if _socket_is_visible(s)]
    try:
        index = visible.index(socket)
    except ValueError:
        index = 0
    if getattr(node, "hide", False):
        cy = (y0 + y1) * 0.5
        return (x1 if is_out else x0, cy)
    if is_out:
        return (x1, y1 - header - (index + 0.5) * unit)
    n = max(len(visible), 1)
    return (x0, y0 + (n - index - 0.5) * unit)


def _socket_dy_node(socket, node, scale):
    """Vertical socket offset in node space. Independent of node.location."""
    key = socket.as_pointer()
    cached = _sock_local.get(key)
    if cached is not None:
        return cached
    locx, locy = _node_abs_loc(node)
    raw = socket_xy(socket)
    slop = 48.0 if _is_gte_tree(getattr(node, "id_data", None)) else 28.0
    if (
        raw is not None
        and not _xy_uninitialized(raw)
        and _xy_plausible(raw, node, socket, scale, slop)
    ):
        dy = raw[1] / scale - locy
    else:
        est = _layout_socket_xy(socket, node, scale)
        dy = est[1] / scale - locy
    _sock_local[key] = dy
    return dy


def connection_xy(socket, node, link, multi_count, scale):
    # PRE_VIEW runs before C++ updates socket runtime. Always parent the
    # wire to the current node location plus a cached local Y offset.
    locx, locy = _node_abs_loc(node)
    tree = getattr(node, "id_data", None)
    if getattr(node, "bl_idname", "") == "NodeReroute":
        xy = (locx * scale, locy * scale)
    elif _is_gte_tree(tree):
        rect = _gte_node_rect_view(node, scale)
        x = rect[2] if socket.is_output else rect[0]
        y = (locy + _socket_dy_node(socket, node, scale)) * scale
        xy = (x, y)
    else:
        width = float(getattr(node, "width", 140.0))
        x = locx + (width if socket.is_output else 0.0)
        y = locy + _socket_dy_node(socket, node, scale)
        xy = (x * scale, y * scale)
    if getattr(socket, "is_multi_input", False) and not socket.is_output and not getattr(node, "hide", False) and link is not None:
        total = max(1, multi_count)
        gap = 0.25 * NODE_GRID_UNIT * scale
        index = int(getattr(link, "multi_input_sort_id", getattr(link, "multi_input_socket_index", 0)) or 0)
        offset = (total * gap - gap) * 0.5
        xy = (xy[0], xy[1] - offset + index * gap)
    return xy


# ---------------------------------------------------------------------------
# Hide native noodles
# ---------------------------------------------------------------------------

def _touch_topology(tree):
    """Force C++ topology cache so runtime.links exists before we hide it."""
    try:
        links = tree.links
        if links:
            link = links[0]
            for sock in (getattr(link, "from_socket", None), getattr(link, "to_socket", None)):
                if sock is not None and hasattr(sock, "is_icon_visible"):
                    bool(sock.is_icon_visible)
                    return
        node = tree.nodes[0]
        coll = node.outputs if node.outputs else node.inputs
        sock = coll[0]
        if hasattr(sock, "is_icon_visible"):
            bool(sock.is_icon_visible)
        else:
            bool(getattr(sock, "enabled", True))
            len(getattr(sock, "links", ()))
    except Exception:
        pass


def _looks_like_ptr_vector(addr):
    begin = _u64_safe(addr)
    end = _u64_safe(addr + 8)
    cap = _u64_safe(addr + 16)
    if not begin or end is None or cap is None:
        return False
    if end < begin or cap < end:
        return False
    size = end - begin
    if size % PTR_SIZE or size > 8 * 1024 * 1024:
        return False
    if size and not _readable_range(begin, min(size, 64)):
        return False
    return True


def _ptr_array_matches(begin, ptrs, allow_reorder):
    n = len(ptrs)
    if not _readable_range(begin, n * PTR_SIZE):
        return False
    got = []
    for i in range(n):
        value = _u64_safe(begin + i * PTR_SIZE)
        if value is None:
            return False
        got.append(value)
    if got == ptrs:
        return True
    return bool(allow_reorder) and sorted(got) == sorted(ptrs)


class NativeLinkHider:
    def __init__(self):
        self._vec_addr = None
        self._saved_end = None
        self._lb_addr = None
        self._saved_first = None
        self._saved_last = None
        self._depth = 0

    def _link_ptrs(self, tree):
        return [link.as_pointer() for link in tree.links]

    def _find_vec_off(self, runtime, ptrs):
        n = len(ptrs)
        expect = n * PTR_SIZE
        max_scan = 32768
        off = 0
        reorder_hit = None
        while off + 24 <= max_scan:
            if not _readable_range(runtime + off, 24):
                break
            begin = _u64(runtime + off)
            end = _u64(runtime + off + 8)
            cap = _u64(runtime + off + 16)
            if begin and end == begin + expect and cap >= end:
                if _ptr_array_matches(begin, ptrs, False):
                    return off
                if reorder_hit is None and _ptr_array_matches(begin, ptrs, True):
                    reorder_hit = off
            off += 8
        return reorder_hit

    def _locate_runtime_vector(self, tree_p, ptrs):
        first, last = ptrs[0], ptrs[-1]
        lb = None
        for off in range(192, 4096, 8):
            a = _u64_safe(tree_p + off)
            b = _u64_safe(tree_p + off + 8)
            if a == first and b == last:
                lb = off
                break
        ranges = []
        if lb is not None:
            ranges.append(range(lb + 16, lb + 4096, 8))
        ranges.append(range(192, 4096, 8))
        seen = set()
        found = None
        for rng in ranges:
            for off in rng:
                if off in seen:
                    continue
                seen.add(off)
                runtime = _u64_safe(tree_p + off)
                if not runtime or not _readable(runtime):
                    continue
                vec_off = self._find_vec_off(runtime, ptrs)
                if vec_off is not None:
                    found = (off, vec_off)
            if found is not None:
                return found
        return found

    def _scan_vector(self, tree):
        global _tree_runtime_off, _links_vec_off
        _touch_topology(tree)
        ptrs = self._link_ptrs(tree)
        n = len(ptrs)
        if n == 0:
            return None
        tree_p = tree.as_pointer()
        if _tree_runtime_off is not None and _links_vec_off is not None:
            runtime = _u64_safe(tree_p + _tree_runtime_off)
            if runtime:
                addr = runtime + _links_vec_off
                begin = _u64_safe(addr)
                end = _u64_safe(addr + 8)
                if begin and end == begin + n * PTR_SIZE and _ptr_array_matches(begin, ptrs, True):
                    return addr
                if _looks_like_ptr_vector(addr):
                    return addr
        located = self._locate_runtime_vector(tree_p, ptrs)
        if located is None:
            return None
        runtime_off, vec_off = located
        runtime = _u64_safe(tree_p + runtime_off)
        if not runtime:
            return None
        _tree_runtime_off = runtime_off
        _links_vec_off = vec_off
        return runtime + vec_off

    def _find_listbase(self, tree):
        global _links_lb_off
        n = len(tree.links)
        if n == 0:
            return None
        tree_p = tree.as_pointer()
        first = tree.links[0].as_pointer()
        last = tree.links[n - 1].as_pointer()
        if _links_lb_off is not None:
            a = _u64_safe(tree_p + _links_lb_off)
            b = _u64_safe(tree_p + _links_lb_off + 8)
            if a == first and b == last:
                return tree_p + _links_lb_off
            _links_lb_off = None
        for off in range(192, 4096, 8):
            a = _u64_safe(tree_p + off)
            b = _u64_safe(tree_p + off + 8)
            if a != first or b != last:
                continue
            if n == 1:
                nxt = _u64_safe(first)
                prv = _u64_safe(first + 8)
                if nxt is None or prv is None or nxt != 0 or prv != 0:
                    continue
            _links_lb_off = off
            return tree_p + off
        return None

    def _poke_empty_vector(self, addr):
        try:
            if not _readable_range(addr, 16):
                return False
            begin_slot = ctypes.c_void_p.from_address(addr)
            end_slot = ctypes.c_void_p.from_address(addr + 8)
            self._vec_addr = addr
            self._saved_end = end_slot.value
            end_slot.value = begin_slot.value
            return True
        except Exception:
            self._vec_addr = None
            self._saved_end = None
            return False

    def _hide_vector(self, tree):
        addr = self._scan_vector(tree)
        if addr is None:
            return False
        return self._poke_empty_vector(addr)

    def _hide_listbase(self, tree):
        addr = self._find_listbase(tree)
        if addr is None:
            return False
        try:
            if not _readable_range(addr, 16):
                return False
            first_slot = ctypes.c_void_p.from_address(addr)
            last_slot = ctypes.c_void_p.from_address(addr + 8)
            self._lb_addr = addr
            self._saved_first = first_slot.value
            self._saved_last = last_slot.value
            first_slot.value = None
            last_slot.value = None
            return True
        except Exception:
            self._lb_addr = None
            self._saved_first = None
            self._saved_last = None
            return False

    def hide(self, tree):
        global _tree_runtime_off, _links_vec_off
        if tree is None:
            return True
        if self._depth:
            self._depth += 1
            return True
        try:
            n_links = len(tree.links)
        except Exception:
            n_links = 0
        ok = False
        if _DRAW_USES_RUNTIME_VEC:
            if not self._hide_vector(tree):
                _tree_runtime_off = None
                _links_vec_off = None
                self._hide_vector(tree)
            ok = self._vec_addr is not None
        else:
            if not self._hide_listbase(tree):
                self._hide_vector(tree)
            ok = self._lb_addr is not None or self._vec_addr is not None
        self._depth = 1
        return bool(ok or n_links == 0)

    def restore_listbase(self):
        if self._lb_addr is None:
            return
        try:
            ctypes.c_void_p.from_address(self._lb_addr).value = self._saved_first
            ctypes.c_void_p.from_address(self._lb_addr + 8).value = self._saved_last
        except Exception:
            pass
        self._lb_addr = None
        self._saved_first = None
        self._saved_last = None

    def restore_vector(self):
        if self._vec_addr is None or self._saved_end is None:
            return
        try:
            ctypes.c_void_p.from_address(self._vec_addr + 8).value = self._saved_end
        except Exception:
            pass
        self._vec_addr = None
        self._saved_end = None

    def restore(self, tree=None):
        if self._depth <= 0:
            return
        self._depth -= 1
        if self._depth > 0:
            return
        self.restore_listbase()
        self.restore_vector()

    def restore_all(self):
        self._depth = 0
        self.restore_listbase()
        self.restore_vector()


_hider = NativeLinkHider()


def _looks_like_drag_link_vector(addr):
    """bNodeLinkDrag starts with Vector<bNodeLink> {begin_, end_, capacity_end_}."""
    begin = _u64_safe(addr)
    end = _u64_safe(addr + 8)
    cap = _u64_safe(addr + 16)
    if not begin or not end or not cap:
        return False
    if end < begin or cap < end:
        return False
    size = end - begin
    if size < 40 or size > 16384 or size % 8:
        return False
    if not _readable_range(begin, min(size, 48)):
        return False
    fromsock = _u64_safe(begin + 32)
    tosock = _u64_safe(begin + 40)
    return bool(fromsock or tosock)


def _wm_operator_customdata(op):
    try:
        base = int(op.as_pointer())
    except Exception:
        return None
    if not _readable_range(base, 160):
        return None
    # wmOperator: next, prev, idname[64], properties, type, customdata
    for off in (96, 104, 88, 112, 80, 120, 72):
        ptr = _u64_safe(base + off)
        if ptr and _looks_like_drag_link_vector(ptr):
            return ptr
    return None


def _iter_nldrag_links(op, vec_addr=None):
    """Read bNodeLinkDrag.links after POST restore. Each row is DNA bNodeLink."""
    addr = vec_addr or _wm_operator_customdata(op)
    if addr is None:
        return []
    begin = _u64_safe(addr)
    end = _u64_safe(addr + 8)
    if not begin or not end or end < begin:
        return []
    nbytes = end - begin
    if nbytes < 40 or nbytes > 16384:
        return []
    for stride in (56, 64, 48, 72):
        if nbytes % stride:
            continue
        tmp = []
        bad = False
        for off in range(0, nbytes, stride):
            if not _readable_range(begin + off, min(stride, 48)):
                bad = True
                break
            fromsock = _u64_safe(begin + off + 32)
            tosock = _u64_safe(begin + off + 40)
            if not fromsock and not tosock:
                continue
            tmp.append(
                (
                    _u64_safe(begin + off + 16),
                    _u64_safe(begin + off + 24),
                    fromsock,
                    tosock,
                )
            )
        if not bad and tmp:
            return tmp
    return []


def _tree_socket_map(tree):
    out = {}
    try:
        nodes = tree.nodes
    except Exception:
        return out
    for node in nodes:
        try:
            socks = list(node.inputs) + list(node.outputs)
        except Exception:
            continue
        for sock in socks:
            try:
                out[int(sock.as_pointer())] = (node, sock)
            except Exception:
                continue
    return out


class LinkDragHider:
    """Hide native rubber-band noodles (snode.runtime->linkdrag), not tree.links."""

    def __init__(self):
        self._end_addr = None
        self._saved_end = None
        self._op_ptr = None
        self._vec_addr = None

    def hide(self, context):
        self.restore(context)
        op = _find_node_link_op(context)
        if op is None:
            self._vec_addr = None
            self._op_ptr = None
            return
        try:
            self._op_ptr = int(op.as_pointer())
        except Exception:
            self._op_ptr = None
        self._hide_vector(op)

    def _hide_vector(self, op):
        addr = None
        try:
            op_ptr = int(op.as_pointer())
        except Exception:
            op_ptr = None
        if self._vec_addr and op_ptr is not None and op_ptr == self._op_ptr:
            addr = self._vec_addr
        if addr is None:
            addr = _wm_operator_customdata(op)
        if addr is None:
            return False
        try:
            if not _readable_range(addr, 16):
                return False
            begin_slot = ctypes.c_void_p.from_address(addr)
            end_slot = ctypes.c_void_p.from_address(addr + 8)
            begin = begin_slot.value
            end = end_slot.value
            if not begin or end == begin:
                return False
            self._vec_addr = addr
            self._end_addr = addr + 8
            self._saved_end = end
            end_slot.value = begin
            return True
        except Exception:
            self._end_addr = None
            self._saved_end = None
            self._vec_addr = None
            self._op_ptr = None
            return False

    def _op_still_modal(self, context):
        if self._op_ptr is None:
            return False
        op = _find_node_link_op(context)
        if op is None:
            return False
        try:
            return int(op.as_pointer()) == self._op_ptr
        except Exception:
            return False

    def restore(self, context=None):
        ctx = context or bpy.context
        if not self._op_still_modal(ctx):
            self._end_addr = None
            self._saved_end = None
            self._op_ptr = None
            self._vec_addr = None
            return
        if self._end_addr is None:
            return
        try:
            ctypes.c_void_p.from_address(self._end_addr).value = self._saved_end
        except Exception:
            pass
        # Keep _vec_addr so POST overlay can read nldrag.links after restore.
        self._end_addr = None
        self._saved_end = None


_linkdrag_hider = LinkDragHider()


# ---------------------------------------------------------------------------
# Orthogonal path
# ---------------------------------------------------------------------------

def _socket_is_visible(sock):
    if sock is None:
        return False
    if getattr(sock, "hide", False):
        return False
    if hasattr(sock, "enabled") and not bool(sock.enabled):
        return False
    if getattr(sock, "is_unavailable", False):
        return False
    return True


def _link_is_drawable(link):
    """Match native node_link_is_hidden / SOCK_UNAVAIL skips."""
    if getattr(link, "is_hidden", False):
        return False
    fs, ts = getattr(link, "from_socket", None), getattr(link, "to_socket", None)
    return _socket_is_visible(fs) and _socket_is_visible(ts)


def _angled_waypoints(p0, p3, from_reroute, to_reroute, scale, curving, from_rect=None, to_rect=None):
    radius = 0.0 if curving <= 0 else float(curving) * 8.0 * scale
    keep = 8.0 * scale
    lead = keep + max(radius, 16.0 * scale)
    unit = NODE_GRID_UNIT * scale
    grid = unit
    align = max(grid, 0.75 * unit)
    dx = p3[0] - p0[0]
    dy = p3[1] - p0[1]
    margin = max(6.0 * scale, 0.3 * unit)

    def straight():
        return [p0, p3]

    def _out_x():
        x = p0[0] + lead
        if from_rect is not None and not from_reroute:
            x = max(x, from_rect[2] + margin)
        return x

    def _in_x():
        x = p3[0] - lead
        if to_rect is not None and not to_reroute:
            x = min(x, to_rect[0] - margin)
        return x

    def corner_start():
        out_x = _out_x()
        return [p0, (out_x, p0[1]), (out_x, p3[1]), p3]

    def corner_end():
        in_x = _in_x()
        return [p0, (in_x, p0[1]), (in_x, p3[1]), p3]

    def double_corner():
        bump = max(radius, 0.8 * unit) + 8.0 * scale
        rail_hi = max(p0[1], p3[1]) + bump
        rail_lo = min(p0[1], p3[1]) - bump
        if from_rect is not None:
            rail_hi = max(rail_hi, from_rect[3] + bump)
            rail_lo = min(rail_lo, from_rect[1] - bump)
        if to_rect is not None:
            rail_hi = max(rail_hi, to_rect[3] + bump)
            rail_lo = min(rail_lo, to_rect[1] - bump)
        up_cost = abs(rail_hi - p0[1]) + abs(rail_hi - p3[1])
        down_cost = abs(p0[1] - rail_lo) + abs(p3[1] - rail_lo)
        rail_y = rail_lo if down_cost < up_cost else rail_hi
        out_x = _out_x()
        in_x = _in_x()
        if in_x >= out_x - 1.0:
            in_x = out_x - max(margin, unit)
        return [p0, (out_x, p0[1]), (out_x, rail_y), (in_x, rail_y), (in_x, p3[1]), p3]

    overlap_x = False
    same_column = False
    if from_rect is not None and to_rect is not None:
        overlap_x = not (from_rect[2] < to_rect[0] or to_rect[2] < from_rect[0])
        if not from_reroute and not to_reroute:
            same_column = abs(from_rect[0] - to_rect[0]) <= align

    if from_reroute and to_reroute and abs(dx) <= align:
        return straight()
    if abs(dy) <= align and dx >= -lead:
        return straight()
    if dx >= 0.0 and dx < lead:
        return straight()
    if same_column:
        return straight()
    if from_reroute != to_reroute and not overlap_x and dx >= -lead:
        return corner_start() if to_reroute else corner_end()
    if dx < -lead:
        return double_corner()
    if dx >= 0.0:
        return corner_start()
    return double_corner()


def _view_zoom():
    region = getattr(bpy.context, "region", None)
    if region is None:
        return 1.0
    try:
        return _view2d_scale_x(region)
    except Exception:
        return 1.0


def _fillet_step_count(radius, zoom):
    px = abs(float(radius)) * max(float(zoom), 0.2)
    return max(FILLET_STEPS, min(8, int(px / 4.0) + 1))


def _filleted_polyline(wps, scale, curving, zoom=None):
    if len(wps) <= 2:
        return list(wps)
    radius = 0.0 if curving <= 0 else float(curving) * 8.0 * scale
    keep = 8.0 * scale
    min_round = MIN_ROUND * scale
    if zoom is None:
        zoom = _view_zoom()
    n = len(wps)
    corner_r = [0.0] * n
    for i in range(1, n - 1):
        ax, ay = wps[i][0] - wps[i - 1][0], wps[i][1] - wps[i - 1][1]
        bx, by = wps[i + 1][0] - wps[i][0], wps[i + 1][1] - wps[i][1]
        lin = math.hypot(ax, ay)
        lout = math.hypot(bx, by)
        keep_in = keep if i == 1 else 0.0
        keep_out = keep if i == n - 2 else 0.0
        max_in = (0.5 * lin if i - 1 >= 1 else lin) - keep_in
        max_out = (0.5 * lout if i + 1 <= n - 2 else lout) - keep_out
        r = min(radius, max_in, max_out)
        corner_r[i] = r if r >= min_round else 0.0

    pts = [wps[0]]
    for i in range(1, n - 1):
        prev, cur, nxt = wps[i - 1], wps[i], wps[i + 1]
        vinx, viny = cur[0] - prev[0], cur[1] - prev[1]
        voutx, vouty = nxt[0] - cur[0], nxt[1] - cur[1]
        lin = math.hypot(vinx, viny)
        lout = math.hypot(voutx, vouty)
        if lin < 1e-4 or lout < 1e-4:
            continue
        tinx, tiny = vinx / lin, viny / lin
        toutx, touty = voutx / lout, vouty / lout
        r = corner_r[i]
        if r < min_round:
            pts.append(cur)
            continue
        fs = (cur[0] - tinx * r, cur[1] - tiny * r)
        fe = (cur[0] + toutx * r, cur[1] + touty * r)
        pts.append(fs)
        cross = tinx * touty - tiny * toutx
        if abs(cross) < 1.0e-4:
            pts.append(fe)
            continue
        if cross > 0.0:
            cx = fs[0] - tiny * r
            cy = fs[1] + tinx * r
        else:
            cx = fs[0] + tiny * r
            cy = fs[1] - tinx * r
        a0 = math.atan2(fs[1] - cy, fs[0] - cx)
        a1 = math.atan2(fe[1] - cy, fe[0] - cx)
        da = a1 - a0
        if cross > 0.0:
            while da <= 1.0e-6:
                da += 2.0 * math.pi
        else:
            while da >= -1.0e-6:
                da -= 2.0 * math.pi
        steps = _fillet_step_count(r, zoom)
        for s in range(1, steps):
            ang = a0 + da * (s / float(steps))
            pts.append((cx + math.cos(ang) * r, cy + math.sin(ang) * r))
        pts.append(fe)
    pts.append(wps[-1])
    return pts


def _mix(a, b, t):
    s = 1.0 - t
    return (a[0] * s + b[0] * t, a[1] * s + b[1] * t, a[2] * s + b[2] * t, 1.0)


_color_by_idname = {}


def _as_rgba(raw):
    return (float(raw[0]), float(raw[1]), float(raw[2]), 1.0)


def _socket_typeinfo_key(socket):
    idname = getattr(socket, "bl_idname", None)
    if idname:
        return str(idname)
    return type(socket).__name__


def _socket_color_fast(socket):
    # Native noodles use typeinfo (bl_idname / draw_color_simple), not DNA socket.type.
    if socket is None:
        return (0.63, 0.63, 0.63, 1.0)
    key = _socket_typeinfo_key(socket)
    cached = _color_by_idname.get(key)
    if cached is not None:
        return cached
    color = None
    try:
        raw = socket.draw_color_simple()
        if raw is not None and len(raw) >= 3:
            color = _as_rgba(raw)
    except Exception:
        pass
    if color is None:
        try:
            node = getattr(socket, "node", None)
            if node is not None:
                raw = socket.draw_color(bpy.context, node)
                if raw is not None and len(raw) >= 3:
                    color = _as_rgba(raw)
        except Exception:
            pass
    if color is None:
        color = _SOCKET_COLORS.get(key)
    if color is None:
        stype = getattr(socket, "type", "") or ""
        if stype and stype != "CUSTOM":
            color = _SOCKET_COLORS.get(stype)
    if color is None:
        color = (0.63, 0.63, 0.63, 1.0)
    _color_by_idname[key] = color
    return color


def _link_insert_ok(link):
    if not _HAS_INSERT_TARGET:
        return False
    addr = int(link.as_pointer()) + _LINK_FLAG_OFF
    if not _readable_range(addr, 4):
        return False
    try:
        flag = int(ctypes.c_int.from_address(addr).value)
    except Exception:
        return False
    return bool(flag & NODE_LINK_INSERT_TARGET) and not (flag & NODE_LINK_INSERT_TARGET_INVALID)


def _insert_signature(tree):
    if not _HAS_INSERT_TARGET:
        return 0
    sig = 0
    for link in tree.links:
        if _link_insert_ok(link):
            sig ^= link.as_pointer() & 0xFFFFFFFF
    return sig


_prev_sel_locs = {}
_warp_backup = []


def _transform_is_modal(context):
    win = getattr(context, "window", None)
    ops = getattr(win, "modal_operators", None) if win is not None else None
    names = []
    if ops:
        try:
            names.extend(str(getattr(mop, "bl_idname", "") or "") for mop in ops)
        except Exception:
            pass
    op = getattr(context, "active_operator", None)
    if op is not None:
        names.append(str(getattr(op, "bl_idname", "") or ""))
    tokens = (
        "transform",
        "translate",
        "attach",
        "resize",
        "node.duplicate",
        "node.delete",
        "node.add",
        "insert_offset",
    )
    for raw in names:
        name = raw.replace("_OT_", ".").lower()
        if any(token in name for token in tokens):
            return True
    return False


def _is_node_dragging(context):
    global _prev_sel_locs, _nodes_are_dragging, _force_wires
    selected = getattr(context, "selected_nodes", None) or ()
    cur = {}
    for node in selected:
        loc = _node_abs_loc(node)
        cur[node.as_pointer()] = (round(loc[0], 2), round(loc[1], 2))
    op_drag = _transform_is_modal(context)
    loc_drag = bool(cur) and bool(_prev_sel_locs) and set(cur) == set(_prev_sel_locs) and cur != _prev_sel_locs
    snapped = False
    if loc_drag and not op_drag:
        for ptr, loc in cur.items():
            prev = _prev_sel_locs.get(ptr)
            if prev is not None and math.hypot(loc[0] - prev[0], loc[1] - prev[1]) > 8.0:
                loc_drag = False
                snapped = True
                break
    _prev_sel_locs = cur
    if snapped:
        _drop_transient_interaction()
        _force_wires = True
        _path_cache["stamp"] = None
    dragging = op_drag or loc_drag
    _nodes_are_dragging = dragging
    return dragging


def _restore_warped_sockets():
    global _warp_backup
    backup = _warp_backup
    _warp_backup = []
    for sock, old_x, old_y, warp_x, warp_y in backup:
        try:
            cur = socket_xy(sock)
            if cur is None:
                continue
            if abs(cur[0] - warp_x) < 0.51 and abs(cur[1] - warp_y) < 0.51:
                _write_socket_xy(sock, old_x, old_y)
        except Exception:
            pass


def _drop_transient_interaction():
    """Right-click cancel does not run undo_post. Drop drag/warp leftovers immediately."""
    global _steer_insert_ptr, _drag_anchor, _drag_op_key, _nodes_are_dragging, _prefer_layout_xy
    _restore_warped_sockets()
    _steer_insert_ptr = None
    _drag_anchor = None
    _drag_op_key = None
    _nodes_are_dragging = False
    _prefer_layout_xy = False
    _linkdrag_hider.restore()


def _request_wire_refresh(redraw=True):
    """Force the next node-editor draw to rebuild wires from live node poses."""
    global _force_wires
    _force_wires = True
    _path_cache["stamp"] = None
    if redraw:
        _tag_node_editors()
        _kick_redraw(3)


def _kick_redraw(frames=3):
    """Redraw after this frame so harvested sockets / undo apply without a click."""
    global _redraw_left
    _redraw_left = max(_redraw_left, max(int(frames), 1))
    timers = getattr(bpy.app, "timers", None)
    if timers is None:
        _tag_node_editors()
        return
    try:
        if not timers.is_registered(_redraw_timer):
            timers.register(_redraw_timer, first_interval=0.0)
    except Exception:
        _tag_node_editors()


def _redraw_timer():
    global _redraw_left
    try:
        _tag_node_editors()
    except Exception:
        pass
    _redraw_left -= 1
    if _redraw_left > 0:
        return 0.0
    return None


def _harvest_socket_locals(tree, scale):
    """Sample socket runtime after C++ has laid nodes out (POST_VIEW)."""
    try:
        links = tree.links
    except Exception:
        return
    changed = False
    gte = _is_gte_tree(tree)
    slop = 64.0 if gte else 36.0
    for link in links:
        for sock, node in ((getattr(link, "from_socket", None), getattr(link, "from_node", None)),
                           (getattr(link, "to_socket", None), getattr(link, "to_node", None))):
            if sock is None or node is None:
                continue
            if getattr(node, "bl_idname", "") == "NodeReroute":
                continue
            raw = socket_xy(sock)
            if raw is None or _xy_uninitialized(raw):
                continue
            if not _xy_plausible(raw, node, sock, scale, slop):
                continue
            locy = _node_abs_loc(node)[1]
            key = sock.as_pointer()
            dy = raw[1] / scale - locy
            prev = _sock_local.get(key)
            if prev is None or abs(prev - dy) > 2.0:
                changed = True
            _sock_local[key] = dy
    if changed:
        _path_cache["stamp"] = None
        _kick_redraw(1)


def _node_locs_map(tree):
    out = {}
    try:
        for node in tree.nodes:
            loc = _node_abs_loc(node)
            out[node.as_pointer()] = (float(loc[0]), float(loc[1]))
    except Exception:
        pass
    return out


def _max_loc_delta(tree):
    if not _cache_node_locs:
        return 0.0
    dmax = 0.0
    try:
        for node in tree.nodes:
            prev = _cache_node_locs.get(node.as_pointer())
            if prev is None:
                continue
            loc = _node_abs_loc(node)
            d = math.hypot(loc[0] - prev[0], loc[1] - prev[1])
            if d > dmax:
                dmax = d
    except Exception:
        return 0.0
    return dmax


def _link_theme_cols(context):
    select_col = (0.90, 0.62, 0.10, 1.0)
    insert_col = (0.90, 0.55, 0.18, 1.0)
    try:
        ws = context.preferences.themes[0].node_editor.wire_select
        select_col = (float(ws[0]), float(ws[1]), float(ws[2]), 1.0)
    except Exception:
        pass
    try:
        na = context.preferences.themes[0].node_editor.node_active
        insert_col = (float(na[0]), float(na[1]), float(na[2]), 1.0)
    except Exception:
        pass
    return select_col, insert_col


def _sync_link_batches(context, space, tree, dragging, prefer_layout=False, force=False):
    """Rebuild GPU batches when the tree pose or topology changed."""
    global _prefer_layout_xy, _cache_node_locs, _cache_n_links
    scale = _ui_scale() or 1.0
    overlay = getattr(space, "overlay", None)
    use_wire_color = True
    if overlay is not None and hasattr(overlay, "show_wire_color"):
        use_wire_color = bool(overlay.show_wire_color)
    curving = _noodle_curving()
    pstamp = _path_stamp(context, tree, scale, curving, use_wire_color, dragging)
    if not force and _path_cache["stamp"] == pstamp:
        return False
    old_pref = _prefer_layout_xy
    _prefer_layout_xy = bool(prefer_layout)
    try:
        select_col, insert_col = _link_theme_cols(context)
        _rebuild_batches(tree, scale, curving, use_wire_color, select_col, insert_col, dragging)
    finally:
        _prefer_layout_xy = old_pref
    _path_cache["stamp"] = pstamp
    _cache_node_locs = _node_locs_map(tree)
    try:
        _cache_n_links = len(tree.links)
    except Exception:
        _cache_n_links = -1
    return True


def _warp_link_through_selection(link, selected, scale):
    """Steer C++ insert-on-link onto the visible orthogonal wire.

    After drawing from real socket positions, poke runtime locations so the
    next ``node_insert_on_link_flags_set`` sees a segment through the node.
    The poke is restored before the next overlay read if C++ has not already
    overwritten it.
    """
    global _warp_backup
    rect = _selection_view_rect(selected, scale)
    if rect is None:
        return
    fs, ts = link.from_socket, link.to_socket
    if fs is None or ts is None:
        return
    x0, y0, x1, y1 = rect
    height = max(1.0, y1 - y0)
    y = y1 - min(6.0, height * 0.12)
    if y < y0:
        y = (y0 + y1) * 0.5
    targets = ((fs, x0 - 12.0, y), (ts, x1 + 12.0, y))
    for sock, nx, ny in targets:
        old = socket_xy(sock)
        if old is None:
            continue
        if _write_socket_xy(sock, nx, ny):
            _warp_backup.append((sock, old[0], old[1], nx, ny))


def _write_insert_target(tree, target_ptr):
    if not _HAS_INSERT_TARGET:
        return
    for link in tree.links:
        addr = int(link.as_pointer()) + _LINK_FLAG_OFF
        if not _readable_range(addr, 4):
            continue
        try:
            slot = ctypes.c_int.from_address(addr)
            value = int(slot.value)
            value &= ~(NODE_LINK_INSERT_TARGET | NODE_LINK_INSERT_TARGET_INVALID)
            if target_ptr is not None and link.as_pointer() == target_ptr:
                value |= NODE_LINK_INSERT_TARGET
            slot.value = value
        except Exception:
            pass


def _seg_hits_rect(a, b, rect):
    x0, y0, x1, y1 = rect
    ax, ay = a
    bx, by = b
    if (x0 <= ax <= x1 and y0 <= ay <= y1) or (x0 <= bx <= x1 and y0 <= by <= y1):
        return True
    corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
    for i in range(4):
        if _isect_seg(a, b, corners[i], corners[(i + 1) % 4]) is not None:
            return True
    return False


def _seg_outside_rect(a, b, rect):
    """Keep pieces of ab that are outside the node body. Do not drop the whole wire."""
    x0, y0, x1, y1 = rect
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    if dx * dx + dy * dy < 1.0e-8:
        if x0 < ax < x1 and y0 < ay < y1:
            return []
        return [(a, b)]
    ts = [0.0, 1.0]
    if abs(dx) > 1.0e-9:
        ts.append((x0 - ax) / dx)
        ts.append((x1 - ax) / dx)
    if abs(dy) > 1.0e-9:
        ts.append((y0 - ay) / dy)
        ts.append((y1 - ay) / dy)
    ts = [t for t in ts if 0.0 <= t <= 1.0]
    ts.sort()
    out = []
    last_t = None
    for t in ts:
        if last_t is not None and t - last_t < 1.0e-6:
            continue
        if last_t is not None:
            tm = 0.5 * (last_t + t)
            mx = ax + dx * tm
            my = ay + dy * tm
            if not (x0 < mx < x1 and y0 < my < y1):
                pa = (ax + dx * last_t, ay + dy * last_t)
                pb = (ax + dx * t, ay + dy * t)
                if math.hypot(pb[0] - pa[0], pb[1] - pa[1]) > 1.0e-4:
                    out.append((pa, pb))
        last_t = t
    return out


def _clip_seg_outside_rects(a, b, rects):
    if not (_finite_pt(a) and _finite_pt(b)):
        return ()
    pieces = [(a, b)]
    minx, maxx = (a[0], b[0]) if a[0] <= b[0] else (b[0], a[0])
    miny, maxy = (a[1], b[1]) if a[1] <= b[1] else (b[1], a[1])
    for rect in rects:
        if rect[2] < minx or rect[0] > maxx or rect[3] < miny or rect[1] > maxy:
            continue
        nxt = []
        for pa, pb in pieces:
            nxt.extend(_seg_outside_rect(pa, pb, rect))
        pieces = nxt
        if not pieces:
            break
    return pieces


def _seg_inside_rect(a, b, rect):
    """Keep pieces of ab that sit inside the node body (drawn under nodes)."""
    x0, y0, x1, y1 = rect
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    if dx * dx + dy * dy < 0.16:
        if x0 <= ax <= x1 and y0 <= ay <= y1:
            return [(a, b)]
        return []
    ts = [0.0, 1.0]
    if abs(dx) > 1.0e-9:
        ts.append((x0 - ax) / dx)
        ts.append((x1 - ax) / dx)
    if abs(dy) > 1.0e-9:
        ts.append((y0 - ay) / dy)
        ts.append((y1 - ay) / dy)
    ts = [t for t in ts if 0.0 <= t <= 1.0]
    ts.sort()
    out = []
    last_t = None
    for t in ts:
        if last_t is not None and t - last_t < 1.0e-5:
            continue
        if last_t is not None:
            tm = 0.5 * (last_t + t)
            mx = ax + dx * tm
            my = ay + dy * tm
            if x0 <= mx <= x1 and y0 <= my <= y1:
                pa = (ax + dx * last_t, ay + dy * last_t)
                pb = (ax + dx * t, ay + dy * t)
                if math.hypot(pb[0] - pa[0], pb[1] - pa[1]) > 0.15:
                    out.append((pa, pb))
        last_t = t
    return out


def _clip_seg_inside_rects(a, b, rects):
    if not (_finite_pt(a) and _finite_pt(b)) or not rects:
        return ()
    out = []
    minx, maxx = (a[0], b[0]) if a[0] <= b[0] else (b[0], a[0])
    miny, maxy = (a[1], b[1]) if a[1] <= b[1] else (b[1], a[1])
    for rect in rects:
        if rect[2] < minx or rect[0] > maxx or rect[3] < miny or rect[1] > maxy:
            continue
        out.extend(_seg_inside_rect(a, b, rect))
    return out


def _node_socket_view_points(node):
    in_xs, out_xs, ys = [], [], []
    try:
        socks = []
        socks.extend(node.inputs)
        n_in = len(socks)
        socks.extend(node.outputs)
        for i, sock in enumerate(socks):
            if not _socket_is_visible(sock):
                continue
            xy = socket_xy(sock)
            if xy is None or not _finite_pt(xy):
                continue
            ys.append(xy[1])
            if i < n_in:
                in_xs.append(xy[0])
            else:
                out_xs.append(xy[0])
    except Exception:
        pass
    return in_xs, out_xs, ys


def _node_view_rect(node, scale, pad=0.0):
    """View-space body aligned to live socket coordinates (same space as the wires)."""
    if node is None:
        return None
    bid = getattr(node, "bl_idname", "")
    if bid == "NodeFrame":
        return None
    locx, locy = _node_abs_loc(node)
    tree = getattr(node, "id_data", None)
    gte = _is_gte_tree(tree)
    if gte and bid != "NodeReroute":
        rect = _gte_node_rect_view(node, scale)
        x0, y0, x1, y1 = rect
        in_xs, out_xs, ys = _node_socket_view_points(node)
        if ys:
            y1 = min(y1, max(ys) + NODE_GRID_UNIT * scale)
            y0 = max(y0, min(ys) - 0.55 * NODE_GRID_UNIT * scale)
        x0 += pad
        y0 += pad
        x1 -= pad
        y1 -= pad
        if x1 <= x0 + 2.0 or y1 <= y0 + 2.0:
            return None
        if not (_finite_pt((x0, y0)) and _finite_pt((x1, y1))):
            return None
        return (x0, y0, x1, y1)
    in_xs, out_xs, ys = _node_socket_view_points(node)
    if bid == "NodeReroute":
        if ys:
            cx = (in_xs[0] if in_xs else (out_xs[0] if out_xs else locx * scale))
            cy = ys[0]
        else:
            cx, cy = locx * scale, locy * scale
        radius = 0.18 * NODE_GRID_UNIT * scale
        radius = max(2.5 * scale, min(radius, 5.0 * scale))
        return (cx - radius, cy - radius, cx + radius, cy + radius)

    width_nu = float(getattr(node, "width", 140.0) or 140.0)
    implied = scale
    if in_xs and out_xs and width_nu > 1.0:
        span = max(out_xs) - min(in_xs)
        if 8.0 < span < 4000.0:
            implied = span / width_nu
    if implied <= 0.0 or not math.isfinite(implied):
        implied = scale

    if in_xs:
        x0 = min(in_xs)
    elif out_xs:
        x0 = max(out_xs) - width_nu * implied
    else:
        x0 = locx * implied
    if out_xs:
        x1 = max(out_xs)
    else:
        x1 = x0 + width_nu * implied

    header = NODE_GRID_UNIT * implied
    y1 = locy * implied
    if ys:
        y_sock = max(ys)
        # Location is the header top. If scale was wrong and the top sits on
        # the sockets, lift it so the header still occludes.
        if y1 < y_sock + 2.0 * implied:
            y1 = y_sock + header

    dim_h = 0.0
    try:
        dim_h = float(node.dimensions.y)
    except Exception:
        pass
    row = 0.55 * header
    if ys:
        sock_h = (max(ys) - min(ys)) + header + row
        height = sock_h
        if 4.0 < dim_h < sock_h + 4.0 * header:
            height = max(height, dim_h)
    elif 4.0 < dim_h < 4000.0:
        height = dim_h
    else:
        height = 80.0 * implied
    y0 = y1 - height
    if ys:
        y0 = min(y0, min(ys) - row)

    x0 += pad
    y0 += pad
    x1 -= pad
    y1 -= pad
    if x1 <= x0 + 2.0 or y1 <= y0 + 2.0:
        return None
    if not (_finite_pt((x0, y0)) and _finite_pt((x1, y1))):
        return None
    if (x1 - x0) > 4000.0 or (y1 - y0) > 4000.0:
        return None
    return (x0, y0, x1, y1)


def _node_socksize(scale):
    # editors/space_node/node_intern.hh: NODE_SOCKSIZE = 0.25 * U.widget_unit
    return 0.25 * NODE_GRID_UNIT * float(scale or 1.0)


def _add_rect_tris(pos, x0, y0, x1, y1, z, cols=None, color=None):
    tris = (
        (x0, y0, z),
        (x1, y0, z),
        (x1, y1, z),
        (x0, y0, z),
        (x1, y1, z),
        (x0, y1, z),
    )
    pos.extend(tris)
    if cols is not None and color is not None:
        cols.extend((color,) * 6)


def _add_disc_tris(pos, cx, cy, r, z, cols=None, color=None, segments=18):
    n = max(8, int(segments))
    for i in range(n):
        a0 = (i / n) * 2.0 * math.pi
        a1 = ((i + 1) / n) * 2.0 * math.pi
        tris = (
            (cx, cy, z),
            (cx + r * math.cos(a0), cy + r * math.sin(a0), z),
            (cx + r * math.cos(a1), cy + r * math.sin(a1), z),
        )
        pos.extend(tris)
        if cols is not None and color is not None:
            cols.extend((color,) * 3)


def _add_diamond_tris(pos, cx, cy, r, z, cols=None, color=None):
    tris = (
        (cx, cy + r, z),
        (cx + r, cy, z),
        (cx, cy - r, z),
        (cx, cy + r, z),
        (cx, cy - r, z),
        (cx - r, cy, z),
    )
    pos.extend(tris)
    if cols is not None and color is not None:
        cols.extend((color,) * 6)


def _add_socket_shape(pos, cx, cy, r, z, shape, cols=None, color=None):
    key = str(shape or "CIRCLE").upper()
    if "DIAMOND" in key:
        _add_diamond_tris(pos, cx, cy, r, z, cols, color)
    elif "SQUARE" in key or "BOX" in key:
        _add_rect_tris(pos, cx - r, cy - r, cx + r, cy + r, z, cols, color)
    else:
        _add_disc_tris(pos, cx, cy, r, z, cols, color)


def _node_depth_rect(node, scale):
    """Visible node body in view space. Must not spill into the stacked gap."""
    if node is None:
        return None
    bid = getattr(node, "bl_idname", "")
    if bid in {"NodeFrame", "NodeReroute"}:
        return None
    locx, locy = _node_abs_loc(node)
    tree = getattr(node, "id_data", None)
    if _is_gte_tree(tree):
        x0, y0, x1, y1 = _gte_node_rect_view(node, scale)
        if x1 <= x0 + 2.0 or y1 <= y0 + 2.0:
            return None
        if not (_finite_pt((x0, y0)) and _finite_pt((x1, y1))):
            return None
        return (x0, y0, x1, y1)
    in_xs, out_xs, ys = _node_socket_view_points(node)
    width_nu = float(getattr(node, "width", 140.0) or 140.0)
    implied = scale
    if in_xs and out_xs and width_nu > 1.0:
        span = max(out_xs) - min(in_xs)
        if 8.0 < span < 4000.0:
            implied = span / width_nu
    if implied <= 0.0 or not math.isfinite(implied):
        implied = scale
    if in_xs:
        x0 = min(in_xs)
    else:
        x0 = locx * implied
    if out_xs:
        x1 = max(out_xs)
    else:
        x1 = x0 + width_nu * implied
    header = NODE_GRID_UNIT * implied
    y1 = locy * implied
    if ys:
        y1 = max(y1, max(ys) + 0.55 * header)
        y0 = min(ys) - 0.40 * header
    else:
        dim_h = 0.0
        try:
            dim_h = float(node.dimensions.y)
        except Exception:
            dim_h = 0.0
        y0 = y1 - (dim_h if 4.0 < dim_h < 4000.0 else 80.0 * implied)
    pad = max(0.75 * implied, 1.0)
    x0 += pad
    y0 += pad
    x1 -= pad
    y1 -= pad
    if x1 <= x0 + 2.0 or y1 <= y0 + 2.0:
        return None
    if not (_finite_pt((x0, y0)) and _finite_pt((x1, y1))):
        return None
    if (x1 - x0) > 4000.0 or (y1 - y0) > 4000.0:
        return None
    return (x0, y0, x1, y1)


def _iter_visible_sockets(node):
    try:
        socks = list(node.inputs) + list(node.outputs)
    except Exception:
        return
    for sock in socks:
        if not _socket_is_visible(sock):
            continue
        xy = socket_xy(sock)
        if xy is None or not _finite_pt(xy):
            continue
        yield sock, xy


def _collect_occlude_tris(tree, scale):
    """Depth-only covering for node bodies and socket shapes."""
    pos = []
    z = OCCLUDE_Z
    r = _node_socksize(scale)
    for node in tree.nodes:
        bid = getattr(node, "bl_idname", "")
        if bid == "NodeFrame":
            continue
        if bid == "NodeReroute":
            in_xs, out_xs, ys = _node_socket_view_points(node)
            locx, locy = _node_abs_loc(node)
            if ys:
                cx = in_xs[0] if in_xs else (out_xs[0] if out_xs else locx * scale)
                cy = ys[0]
            else:
                cx, cy = locx * scale, locy * scale
            _add_disc_tris(pos, cx, cy, r, z)
            continue
        rect = _node_depth_rect(node, scale)
        if rect is not None:
            _add_rect_tris(pos, rect[0], rect[1], rect[2], rect[3], z)
        for sock, xy in _iter_visible_sockets(node):
            shape = getattr(sock, "display_shape", "CIRCLE")
            if getattr(sock, "is_multi_input", False):
                _add_rect_tris(pos, xy[0] - r, xy[1] - r * 2.2, xy[0] + r, xy[1] + r * 2.2, z)
            _add_socket_shape(pos, xy[0], xy[1], r, z, shape)
    return pos


def _collect_cover_rects(tree, scale):
    """Tight node+socket boxes. Used so POST_VIEW wires do not paint over nodes."""
    rects = []
    r = _node_socksize(scale)
    for node in tree.nodes:
        bid = getattr(node, "bl_idname", "")
        if bid == "NodeFrame":
            continue
        if bid == "NodeReroute":
            in_xs, out_xs, ys = _node_socket_view_points(node)
            locx, locy = _node_abs_loc(node)
            if ys:
                cx = in_xs[0] if in_xs else (out_xs[0] if out_xs else locx * scale)
                cy = ys[0]
            else:
                cx, cy = locx * scale, locy * scale
            rects.append((cx - r, cy - r, cx + r, cy + r))
            continue
        rect = _node_depth_rect(node, scale)
        if rect is not None:
            rects.append(rect)
        for sock, xy in _iter_visible_sockets(node):
            if getattr(sock, "is_multi_input", False):
                rects.append((xy[0] - r, xy[1] - r * 2.2, xy[0] + r, xy[1] + r * 2.2))
            else:
                rects.append((xy[0] - r, xy[1] - r, xy[0] + r, xy[1] + r))
    return rects


def _collect_socket_cap_geom(tree, scale):
    """Sockets are drawn by Blender. This overlay never restyles them."""
    return [], []


def _node_occlude_rect(node, scale):
    """Conservative interior of a node. Too-large boxes punch empty holes in the gap."""
    if node is None:
        return None
    bid = getattr(node, "bl_idname", "")
    if bid == "NodeFrame":
        return None
    locx, locy = _node_abs_loc(node)
    tree = getattr(node, "id_data", None)
    if _is_gte_tree(tree) and bid != "NodeReroute":
        x0, y0, x1, y1 = _gte_node_rect_view(node, scale)
        inset_x = max(3.0 * scale, 4.0)
        inset_y = max(1.5 * scale, 2.0)
        x0 += inset_x
        x1 -= inset_x
        y0 += inset_y
        y1 -= inset_y
        if x1 <= x0 + 4.0 or y1 <= y0 + 4.0:
            return None
        if not (_finite_pt((x0, y0)) and _finite_pt((x1, y1))):
            return None
        if (x1 - x0) > 4000.0 or (y1 - y0) > 4000.0:
            return None
        return (x0, y0, x1, y1)
    in_xs, out_xs, ys = _node_socket_view_points(node)
    if bid == "NodeReroute":
        if ys:
            cx = (in_xs[0] if in_xs else (out_xs[0] if out_xs else locx * scale))
            cy = ys[0]
        else:
            cx, cy = locx * scale, locy * scale
        radius = 0.20 * NODE_GRID_UNIT * scale
        radius = max(2.0 * scale, min(radius, 4.5 * scale))
        return (cx - radius, cy - radius, cx + radius, cy + radius)

    width_nu = float(getattr(node, "width", 140.0) or 140.0)
    implied = scale
    if in_xs and out_xs and width_nu > 1.0:
        span = max(out_xs) - min(in_xs)
        if 8.0 < span < 4000.0:
            implied = span / width_nu
    if implied <= 0.0 or not math.isfinite(implied):
        implied = scale
    inset = max(2.0 * implied, 3.0)
    if in_xs:
        x0 = min(in_xs) + inset
    elif out_xs:
        x0 = max(out_xs) - width_nu * implied + inset
    else:
        x0 = locx * implied + inset
    if out_xs:
        x1 = max(out_xs) - inset
    else:
        x1 = x0 + max(8.0, width_nu * implied - 2.0 * inset)
    header = 0.40 * NODE_GRID_UNIT * implied
    foot = 0.18 * NODE_GRID_UNIT * implied
    if ys:
        y1 = max(ys) + header
        y0 = min(ys) - foot
    else:
        y1 = locy * implied
        y0 = y1 - 40.0 * implied
    if x1 <= x0 + 4.0 or y1 <= y0 + 4.0:
        return None
    if not (_finite_pt((x0, y0)) and _finite_pt((x1, y1))):
        return None
    if (x1 - x0) > 4000.0 or (y1 - y0) > 4000.0:
        return None
    return (x0, y0, x1, y1)


def _node_mask_rect(node, scale):
    """Tight visual body used to hide wires. Must not spill into the stacked gap."""
    if node is None:
        return None
    bid = getattr(node, "bl_idname", "")
    if bid == "NodeFrame":
        return None
    locx, locy = _node_abs_loc(node)
    tree = getattr(node, "id_data", None)
    if _is_gte_tree(tree) and bid != "NodeReroute":
        x0, y0, x1, y1 = _gte_node_rect_view(node, scale)
        inset = max(2.0 * scale, 2.0)
        return (x0 + inset, y0 + inset, x1 - inset, y1 - inset) if x1 - x0 > 8.0 and y1 - y0 > 8.0 else None
    in_xs, out_xs, ys = _node_socket_view_points(node)
    if bid == "NodeReroute":
        if ys:
            cx = (in_xs[0] if in_xs else (out_xs[0] if out_xs else locx * scale))
            cy = ys[0]
        else:
            cx, cy = locx * scale, locy * scale
        radius = max(2.0 * scale, min(0.18 * NODE_GRID_UNIT * scale, 4.0 * scale))
        return (cx - radius, cy - radius, cx + radius, cy + radius)
    width_nu = float(getattr(node, "width", 140.0) or 140.0)
    implied = scale
    if in_xs and out_xs and width_nu > 1.0:
        span = max(out_xs) - min(in_xs)
        if 8.0 < span < 4000.0:
            implied = span / width_nu
    if implied <= 0.0 or not math.isfinite(implied):
        implied = scale
    # Tiny inset so socket nubs stay visible; do not use node.dimensions (it fills the gap).
    inset = max(1.25 * implied, 1.5)
    if in_xs:
        x0 = min(in_xs) + inset
    elif out_xs:
        x0 = max(out_xs) - width_nu * implied + inset
    else:
        x0 = locx * implied + inset
    if out_xs:
        x1 = max(out_xs) - inset
    else:
        x1 = x0 + max(8.0, width_nu * implied - 2.0 * inset)
    header = 0.70 * NODE_GRID_UNIT * implied
    foot = 0.28 * NODE_GRID_UNIT * implied
    if ys:
        y1 = max(ys) + header
        y0 = min(ys) - foot
    else:
        y1 = locy * implied
        y0 = y1 - 36.0 * implied
    if x1 <= x0 + 2.0 or y1 <= y0 + 2.0:
        return None
    if not (_finite_pt((x0, y0)) and _finite_pt((x1, y1))):
        return None
    if (x1 - x0) > 4000.0 or (y1 - y0) > 4000.0:
        return None
    return (x0, y0, x1, y1)


def _collect_node_mask_rects(tree, scale):
    rects = []
    for node in tree.nodes:
        rect = _node_mask_rect(node, scale)
        if rect is not None:
            rects.append(rect)
    return rects


def _collect_node_occlude_rects(tree, scale):
    rects = []
    for node in tree.nodes:
        rect = _node_occlude_rect(node, scale)
        if rect is not None:
            rects.append(rect)
    return _separate_stacked_clip_rects(rects)


def _occlude_tris(rects):
    pos = []
    z = OCCLUDE_Z
    for x0, y0, x1, y1 in rects:
        pos.extend(
            (
                (x0, y0, z),
                (x1, y0, z),
                (x1, y1, z),
                (x0, y0, z),
                (x1, y1, z),
                (x0, y1, z),
            )
        )
    return pos


def _separate_stacked_clip_rects(rects):
    """If two clip boxes overlap in a column, raise the upper bottom so the gap wire survives."""
    if len(rects) < 2:
        return rects
    boxes = [list(r) for r in rects]
    gap = 3.0
    n = len(boxes)
    for i in range(n):
        for j in range(i + 1, n):
            ai, aj = (i, j) if boxes[i][3] >= boxes[j][3] else (j, i)
            hi, lo = boxes[ai], boxes[aj]
            if (hi[3] - hi[1]) < 16.0 or (lo[3] - lo[1]) < 16.0:
                continue
            if hi[2] <= lo[0] or lo[2] <= hi[0]:
                continue
            if hi[1] >= lo[3]:
                continue
            hi[1] = lo[3] + gap
            if hi[1] > hi[3] - 8.0:
                hi[1] = hi[3] - 8.0
    out = []
    for x0, y0, x1, y1 in boxes:
        if x1 > x0 + 2.0 and y1 > y0 + 2.0:
            out.append((x0, y0, x1, y1))
    return out


def _collect_node_clip_rects(tree, scale):
    """Every drawn node body, including endpoints and reroutes. Frames stay under wires."""
    rects = []
    for node in tree.nodes:
        rect = _node_body_clip_rect(node, scale)
        if rect is not None:
            rects.append(rect)
    return _separate_stacked_clip_rects(rects)


def _dist_point_seg(px, py, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    len2 = dx * dx + dy * dy
    if len2 < 1e-10:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _selection_view_rect(nodes, scale):
    rect = None
    for node in nodes:
        if node.bl_idname in {"NodeReroute", "NodeFrame"}:
            continue
        locx, locy = _node_abs_loc(node)
        dims = node.dimensions
        x0 = locx * scale
        y1 = locy * scale
        width = dims.x if dims.x > 1.0 else float(getattr(node, "width", 140.0)) * scale
        height = dims.y if dims.y > 1.0 else 80.0 * scale
        x1 = x0 + width
        y0 = y1 - height
        if rect is None:
            rect = [x0, y0, x1, y1]
        else:
            rect[0] = min(rect[0], x0)
            rect[1] = min(rect[1], y0)
            rect[2] = max(rect[2], x1)
            rect[3] = max(rect[3], y1)
    return None if rect is None else tuple(rect)


def _pick_ortho_insert(items, selected, scale):
    if not selected:
        return None
    rect = _selection_view_rect(selected, scale)
    if rect is None:
        return None
    skip = {node.as_pointer() for node in selected}
    cx = (rect[0] + rect[2]) * 0.5
    cy = (rect[1] + rect[3]) * 0.5
    best = None
    best_d = 1e18
    for link, poly in items:
        fn, tn = link.from_node, link.to_node
        if fn is None or tn is None:
            continue
        if fn.as_pointer() in skip or tn.as_pointer() in skip:
            continue
        for i in range(len(poly) - 1):
            if not _seg_hits_rect(poly[i], poly[i + 1], rect):
                continue
            d = _dist_point_seg(cx, cy, poly[i], poly[i + 1])
            if d < best_d:
                best_d = d
                best = link.as_pointer()
    return best


def _iter_link_polys(tree, scale, curving, rects=None):
    multi_counts = {}
    links = tree.links
    zoom = _view_zoom()
    for link in links:
        if not _link_is_drawable(link):
            continue
        sock = link.to_socket
        if sock is not None and getattr(sock, "is_multi_input", False):
            key = sock.as_pointer()
            multi_counts[key] = multi_counts.get(key, 0) + 1

    for link in links:
        if not _link_is_drawable(link):
            continue
        fn, tn = link.from_node, link.to_node
        fs, ts = link.from_socket, link.to_socket
        if fn is None or tn is None or fs is None or ts is None:
            continue
        p0 = connection_xy(fs, fn, link, 1, scale)
        p3 = connection_xy(ts, tn, link, multi_counts.get(ts.as_pointer(), 1), scale)
        if not _finite_pt(p0) or not _finite_pt(p3):
            continue
        if rects is not None:
            from_rect = rects.get(fn.as_pointer())
            to_rect = rects.get(tn.as_pointer())
        else:
            from_rect = _node_view_rect(fn, scale)
            to_rect = _node_view_rect(tn, scale)
        wps = _angled_waypoints(
            p0,
            p3,
            fn.bl_idname == "NodeReroute",
            tn.bl_idname == "NodeReroute",
            scale,
            curving,
            from_rect,
            to_rect,
        )
        yield link, _filleted_polyline(wps, scale, curving, zoom)


def _isect_seg(a, b, c, d):
    x1, y1 = a
    x2, y2 = b
    x3, y3 = c
    x4, y4 = d
    den = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if abs(den) < 1e-12:
        return None
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / den
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / den
    if 0.0 <= ua <= 1.0 and 0.0 <= ub <= 1.0:
        return (x1 + ua * (x2 - x1), y1 + ua * (y2 - y1))
    return None


def _stroke_hits_poly(stroke_view, poly):
    if len(stroke_view) < 2 or len(poly) < 2:
        return None
    for i in range(len(stroke_view) - 1):
        a, b = stroke_view[i], stroke_view[i + 1]
        if a == b:
            continue
        for j in range(len(poly) - 1):
            hit = _isect_seg(a, b, poly[j], poly[j + 1])
            if hit is not None:
                return hit
    return None


def _region_stroke_to_view(view2d, stroke_region):
    out = []
    for x, y in stroke_region:
        vx, vy = view2d.region_to_view(x, y)
        out.append((float(vx), float(vy)))
    return out


# ---------------------------------------------------------------------------
# Draw. Official node_draw_nodetree paints links, then node_draw (body + sockets).
# ---------------------------------------------------------------------------

def _get_shader():
    global _shader
    if _shader is None:
        for name in _SHADER_NAMES:
            try:
                _shader = gpu.shader.from_builtin(name)
                break
            except Exception:
                _shader = None
        if _shader is None:
            try:
                _shader = gpu.shader.from_builtin("UNIFORM_COLOR")
            except Exception:
                _shader = gpu.shader.from_builtin("3D_UNIFORM_COLOR")
    return _shader


def _get_fill_shader():
    global _fill_shader
    if _fill_shader is None:
        for name in ("3D_UNIFORM_COLOR", "UNIFORM_COLOR", "2D_UNIFORM_COLOR"):
            try:
                _fill_shader = gpu.shader.from_builtin(name)
                break
            except Exception:
                _fill_shader = None
    return _fill_shader


def _get_smooth_fill():
    global _smooth_fill
    if _smooth_fill is None:
        for name in ("3D_SMOOTH_COLOR", "SMOOTH_COLOR", "2D_SMOOTH_COLOR"):
            try:
                _smooth_fill = gpu.shader.from_builtin(name)
                break
            except Exception:
                _smooth_fill = None
    return _smooth_fill


def _finite_pt(p):
    try:
        x, y = float(p[0]), float(p[1])
    except Exception:
        return False
    return math.isfinite(x) and math.isfinite(y) and abs(x) < MAX_VIEW_COORD and abs(y) < MAX_VIEW_COORD


def _emit_line(dst_pos, dst_cols, a, b, ca, cb):
    if not (_finite_pt(a) and _finite_pt(b)):
        return
    if math.hypot(b[0] - a[0], b[1] - a[1]) < 1.0e-4:
        return
    dst_pos.extend(((a[0], a[1], WIRE_Z), (b[0], b[1], WIRE_Z)))
    dst_cols.extend((ca, cb))


def _node_visual_rect(node, scale):
    """Visible node AABB in view space (same units as the wires)."""
    if node is None:
        return None
    bid = getattr(node, "bl_idname", "")
    if bid == "NodeFrame":
        return None
    locx, locy = _node_abs_loc(node)
    x0 = locx * scale
    y1 = locy * scale
    if bid == "NodeReroute":
        r = max(3.0 * scale, 0.25 * NODE_GRID_UNIT * scale)
        return (x0 - r, y1 - r, x0 + r, y1 + r)
    w = float(getattr(node, "width", 140.0) or 140.0) * scale
    h = 40.0 * scale
    try:
        dw = float(node.dimensions[0])
        dh = float(node.dimensions[1])
        if dw > 4.0:
            w = max(w, dw)
        if dh > 4.0:
            h = max(h, dh)
    except Exception:
        pass
    x1 = x0 + w
    y0 = y1 - h
    if x1 <= x0 + 2.0 or y1 <= y0 + 2.0:
        return None
    if not (_finite_pt((x0, y0)) and _finite_pt((x1, y1))):
        return None
    return (x0, y0, x1, y1)


def _rect_overlaps(a, b):
    return not (a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1])


def _node_overlay_rect(node, scale):
    """Node AABB in the same view space as the polylines (socket-aligned)."""
    rect = _node_view_rect(node, scale, pad=0.0)
    if rect is not None:
        return rect
    rect = _node_visual_rect(node, scale)
    if rect is None:
        return None
    x0, y0, x1, y1 = rect
    n_vis = 0
    try:
        for sock in node.inputs:
            if _socket_is_visible(sock):
                n_vis += 1
        for sock in node.outputs:
            if _socket_is_visible(sock):
                n_vis += 1
    except Exception:
        n_vis = 0
    row = 0.55 * NODE_GRID_UNIT * scale
    header = NODE_GRID_UNIT * scale
    need_h = header + max(1, n_vis) * row
    if (y1 - y0) < need_h:
        y0 = y1 - need_h
    if x1 <= x0 + 2.0 or y1 <= y0 + 2.0:
        return None
    if not (_finite_pt((x0, y0)) and _finite_pt((x1, y1))):
        return None
    return (x0, y0, x1, y1)


def _build_node_rect_cache(tree, scale):
    cache = {}
    for node in tree.nodes:
        if getattr(node, "bl_idname", "") == "NodeFrame":
            continue
        rect = _node_overlay_rect(node, scale)
        if rect is not None:
            cache[node.as_pointer()] = rect
    return cache


def _collect_frame_rects(tree, scale, node_rects=None):
    """View-space Frame fill. At least children plus official 1.5 widget-unit margin."""
    margin = 1.5 * NODE_GRID_UNIT * scale
    pad = max(1.0, scale)
    out = []
    children_by_frame = {}
    for node in tree.nodes:
        parent = getattr(node, "parent", None)
        if parent is None or getattr(parent, "bl_idname", "") != "NodeFrame":
            continue
        children_by_frame.setdefault(parent.as_pointer(), []).append(node)
    for node in tree.nodes:
        if getattr(node, "bl_idname", "") != "NodeFrame":
            continue
        locx, locy = _node_abs_loc(node)
        w = float(getattr(node, "width", 0.0) or 0.0)
        h = float(getattr(node, "height", 0.0) or 0.0)
        try:
            dw = float(node.dimensions[0])
            dh = float(node.dimensions[1])
        except Exception:
            dw = dh = 0.0
        width = max(w * scale, dw if dw > 4.0 else 0.0)
        height = max(h * scale, dh if dh > 4.0 else 0.0)
        x0 = locx * scale
        y1 = locy * scale
        x1 = x0 + width
        y0 = y1 - height
        first = not (width > 4.0 and height > 4.0)
        for child in children_by_frame.get(node.as_pointer(), ()):
            if node_rects is not None:
                cr = node_rects.get(child.as_pointer())
            else:
                cr = _node_overlay_rect(child, scale)
            if cr is None:
                continue
            cx0, cy0 = cr[0] - margin, cr[1] - margin
            cx1, cy1 = cr[2] + margin, cr[3] + margin
            if first:
                x0, y0, x1, y1 = cx0, cy0, cx1, cy1
                first = False
            else:
                x0 = min(x0, cx0)
                y0 = min(y0, cy0)
                x1 = max(x1, cx1)
                y1 = max(y1, cy1)
        width = x1 - x0
        height = y1 - y0
        if width < 4.0 or height < 4.0:
            continue
        x0 -= pad
        y0 -= pad
        x1 += pad
        y1 += pad
        if not (_finite_pt((x0, y0)) and _finite_pt((x1, y1))):
            continue
        out.append((x0, y0, x1, y1))
    return out


def _zone_paired_output(node, tree):
    out = getattr(node, "paired_output", None)
    if out is not None:
        return out
    want = _ZONE_OUTPUT_BY_INPUT.get(getattr(node, "bl_idname", ""))
    if not want:
        return None
    locx, locy = _node_abs_loc(node)
    best = None
    best_d = None
    for other in tree.nodes:
        if getattr(other, "bl_idname", "") != want:
            continue
        ox, oy = _node_abs_loc(other)
        dx = ox - locx
        if dx < -1.0:
            continue
        dist = dx * dx + (oy - locy) * (oy - locy)
        if best is None or dist < best_d:
            best = other
            best_d = dist
    return best


def _collect_zone_rects(tree, scale, frame_rects, rects=None):
    """AABB of Repeat / Simulation / For Each / Closure zone fills.

    Official node_draw_zones_and_frames paints these under links, same as Frames.
    Union socket-aligned bounds of the pair and every node in the x-span.
    """
    pad = NODE_GRID_UNIT * scale
    extra = 0.5 * NODE_GRID_UNIT * scale
    rects = rects or {}
    out_rects = []
    for node in tree.nodes:
        if getattr(node, "bl_idname", "") not in _ZONE_INPUT_IDNAMES:
            continue
        out = _zone_paired_output(node, tree)
        if out is None:
            continue
        ir = rects.get(node.as_pointer()) or _node_overlay_rect(node, scale)
        oor = rects.get(out.as_pointer()) or _node_overlay_rect(out, scale)
        if ir is None or oor is None:
            continue
        skip = {node.as_pointer(), out.as_pointer()}
        pair_x0 = min(ir[0], oor[0])
        pair_x1 = max(ir[2], oor[2])
        x0 = pair_x0 - pad
        x1 = pair_x1 + pad
        y0 = min(ir[1], oor[1]) - pad
        y1 = max(ir[3], oor[3]) + pad
        for other in tree.nodes:
            if other.as_pointer() in skip:
                continue
            vr = rects.get(other.as_pointer())
            if vr is None or vr[2] < pair_x0 or vr[0] > pair_x1:
                continue
            x0 = min(x0, vr[0] - pad)
            y0 = min(y0, vr[1] - pad)
            x1 = max(x1, vr[2] + pad)
            y1 = max(y1, vr[3] + pad)
        for fr in frame_rects:
            if fr[2] < pair_x0 or fr[0] > pair_x1:
                continue
            x0 = min(x0, fr[0] - pad)
            y0 = min(y0, fr[1] - pad)
            x1 = max(x1, fr[2] + pad)
            y1 = max(y1, fr[3] + pad)
        x0 -= extra
        y0 -= extra
        x1 += extra
        y1 += extra
        if x1 <= x0 + 4.0 or y1 <= y0 + 4.0:
            continue
        if not (_finite_pt((x0, y0)) and _finite_pt((x1, y1))):
            continue
        out_rects.append((x0, y0, x1, y1))
    return out_rects


def _collect_backdrop_rects(tree, scale, rects=None):
    frame_rects = _collect_frame_rects(tree, scale, rects)
    zone_rects = _collect_zone_rects(tree, scale, frame_rects, rects)
    return frame_rects, zone_rects


def _collect_frame_hide_rects(tree, scale, frame_rects, zone_rects, rects=None):
    """Punch POST overlay where a node body (including sockets) sits on a Frame/zone."""
    hide = []
    sock = _node_socksize(scale)
    rects = rects or {}
    backdrops = []
    if frame_rects:
        backdrops.extend(frame_rects)
    if zone_rects:
        backdrops.extend(zone_rects)
    for node in tree.nodes:
        bid = getattr(node, "bl_idname", "")
        if bid == "NodeFrame":
            continue
        parent = getattr(node, "parent", None)
        on_frame = parent is not None and getattr(parent, "bl_idname", "") == "NodeFrame"
        visual = rects.get(node.as_pointer())
        if visual is None:
            visual = _node_overlay_rect(node, scale)
        if visual is None:
            continue
        on_zone = bid in _ZONE_ENDPOINT_IDNAMES
        if not on_zone and not on_frame and backdrops:
            on_zone = any(_rect_overlaps(visual, br) for br in backdrops)
        if not on_frame and not on_zone:
            continue
        x0, y0, x1, y1 = visual
        rr = sock if bid != "NodeReroute" else max(sock, 1.5 * scale)
        halo = max(rr, 1.5 * scale)
        boxed = (x0 - halo, y0 - halo, x1 + halo, y1 + halo)
        if boxed[2] <= boxed[0] + 2.0 or boxed[3] <= boxed[1] + 2.0:
            continue
        hide.append(boxed)
    return hide


def _emit_on_frames(dst_pos, dst_cols, a, b, ca, cb, frames, hides):
    if not frames:
        return
    for aa, bb in _clip_seg_inside_rects(a, b, frames):
        if hides:
            for cc, dd in _clip_seg_outside_rects(aa, bb, hides):
                _emit_line(dst_pos, dst_cols, cc, dd, ca, cb)
        else:
            _emit_line(dst_pos, dst_cols, aa, bb, ca, cb)


def _emit_split_seg(pre_pos, pre_cols, post_pos, post_cols, a, b, ca, cb, frames, hides):
    """PRE draws empty canvas; POST redraws on Frame/zone fills, punched by nodes."""
    if frames:
        for aa, bb in _clip_seg_outside_rects(a, b, frames):
            _emit_line(pre_pos, pre_cols, aa, bb, ca, cb)
        _emit_on_frames(post_pos, post_cols, a, b, ca, cb, frames, hides)
    else:
        _emit_line(pre_pos, pre_cols, a, b, ca, cb)


def _thick_half_view(pixel_w):
    zoom = max(_view_zoom(), 0.08)
    return max(0.2, float(pixel_w) * 0.5 / zoom)


def _emit_thick_line(dst_pos, dst_cols, a, b, ca, cb, half_w, z=None):
    if not (_finite_pt(a) and _finite_pt(b)):
        return
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length < 1.0e-4:
        return
    hw = float(half_w)
    nx = -dy / length * hw
    ny = dx / length * hw
    zz = WIRE_Z if z is None else float(z)
    p0 = (ax + nx, ay + ny, zz)
    p1 = (ax - nx, ay - ny, zz)
    p2 = (bx - nx, by - ny, zz)
    p3 = (bx + nx, by + ny, zz)
    dst_pos.extend((p0, p1, p2, p0, p2, p3))
    dst_cols.extend((ca, ca, cb, ca, cb, cb))


def _emit_line_clipped(dst_pos, dst_cols, a, b, ca, cb, rects):
    if not rects:
        _emit_line(dst_pos, dst_cols, a, b, ca, cb)
        return
    for pa, pb in _clip_seg_outside_rects(a, b, rects):
        _emit_line(dst_pos, dst_cols, pa, pb, ca, cb)


def _emit_line_under(dst_pos, dst_cols, a, b, ca, cb, rects):
    """Pieces that cross a node body. Drawn PRE_VIEW so nodes cover them."""
    if not rects:
        return
    for pa, pb in _clip_seg_inside_rects(a, b, rects):
        _emit_line(dst_pos, dst_cols, pa, pb, ca, cb)


def _set_line_width(shader, width):
    global _shader_has_line_width
    width = max(1.0, min(float(width), MAX_POLYLINE_WIDTH))
    if not math.isfinite(width):
        width = 1.0
    if _shader_has_line_width:
        try:
            shader.uniform_float("lineWidth", width)
            return
        except Exception:
            _shader_has_line_width = False
    # Do not leak a huge GL line width into later UI drawing.


def _view2d_scale_x(region):
    v2d = getattr(region, "view2d", None)
    if v2d is None:
        return 1.0
    try:
        cur = v2d.cur
        span = float(cur.xmax) - float(cur.xmin)
        width = float(region.width)
        if span > 1.0e-3 and width > 1.0:
            zoom = width / span
            if math.isfinite(zoom) and zoom > 0.0:
                return max(0.05, min(zoom, 16.0))
    except Exception:
        pass
    try:
        x0, _y0 = v2d.view_to_region(0.0, 0.0, False)
        x1, _y1 = v2d.view_to_region(100.0, 0.0, False)
        zoom = abs(float(x1) - float(x0)) / 100.0
        if math.isfinite(zoom) and zoom > 0.0:
            return max(0.05, min(zoom, 16.0))
    except TypeError:
        try:
            x0, _y0 = v2d.view_to_region(0.0, 0.0)
            x1, _y1 = v2d.view_to_region(100.0, 0.0)
            zoom = abs(float(x1) - float(x0)) / 100.0
            if math.isfinite(zoom) and zoom > 0.0:
                return max(0.05, min(zoom, 16.0))
        except Exception:
            return 1.0
    except Exception:
        return 1.0
    return 1.0


def _viewport_size(region):
    rw = float(max(int(getattr(region, "width", 0) or 0), 0))
    rh = float(max(int(getattr(region, "height", 0) or 0), 0))
    w = h = 0.0
    try:
        vp = gpu.state.viewport_get()
        w, h = float(vp[2]), float(vp[3])
    except Exception:
        pass
    if (
        not math.isfinite(w)
        or not math.isfinite(h)
        or w < 8.0
        or h < 8.0
        or w > 16384.0
        or h > 16384.0
    ):
        w, h = rw, rh
    return (max(w, 8.0), max(h, 8.0))


def _link_draw_widths(region):
    """Match native nodelink pixel width, clamped for the polyline shader.

    C++ uses ``2.5 * max(UI_SCALE * view2d_scale, 1)``. Unclamped zoom or a
    0-size viewport makes POLYLINE expand to fullscreen garbage.
    """
    ui = _ui_scale() or 1.0
    zoom = _view2d_scale_x(region)
    thickness = LINK_WIDTH * max(ui * zoom, 1.0)
    if not math.isfinite(thickness) or thickness <= 0.0:
        thickness = LINK_WIDTH
    thickness = min(thickness, 6.0)
    # Native nodelink uses thickness for the core and a ~1px dark halo.
    outline = min(thickness + 1.25, MAX_POLYLINE_WIDTH)
    main = thickness
    insert = min(max(INSERT_WIDTH, thickness * 1.6), MAX_POLYLINE_WIDTH)
    return outline, main, insert


def _enabled(context):
    wm = getattr(context, "window_manager", None)
    if wm is None or not getattr(wm, "use_angled_wires", False):
        return False
    space = getattr(context, "space_data", None)
    return space is not None and space.type == "NODE_EDITOR" and space.edit_tree is not None


def _layout_token(tree):
    if getattr(tree, "bl_idname", "") == "GPUTextureEditorNodeTree":
        return 0
    token = 0
    n = 0
    try:
        for link in tree.links:
            for sock in (link.from_socket, link.to_socket):
                if sock is None:
                    continue
                xy = socket_xy(sock)
                if xy is None or _xy_uninitialized(xy):
                    return None
                token ^= (int(xy[0] * 4.0) * 73856093) ^ (int(xy[1] * 4.0) * 19349663)
                n += 1
            if n >= 8:
                break
    except Exception:
        return None
    return token


def _nodes_loc_token(tree):
    acc = 0
    n = 0
    try:
        for node in tree.nodes:
            loc = _node_abs_loc(node)
            acc = (acc * 16777619) ^ (node.as_pointer() & 0xFFFFFFFF)
            acc = (acc * 16777619) ^ (int(loc[0] * 10.0) & 0xFFFFFFFF)
            acc = (acc * 16777619) ^ (int(loc[1] * 10.0) & 0xFFFFFFFF)
            acc = (acc * 16777619) ^ (int(float(getattr(node, "width", 0.0)) * 10.0) & 0xFFFFFFFF)
            acc = (acc * 16777619) ^ int(bool(getattr(node, "hide", False)))
            n += 1
    except Exception:
        return (0, 0)
    return (n, acc & 0xFFFFFFFF)


def _path_stamp(context, tree, scale, curving, wire_color, dragging):
    selected = getattr(context, "selected_nodes", None) or ()
    sel_n = 0
    sel_hash = 0
    for node in selected:
        loc = _node_abs_loc(node)
        sel_n += 1
        sel_hash ^= node.as_pointer() & 0xFFFFFFFF
        sel_hash ^= (int(loc[0] * 10.0) & 0xFFFFFFFF)
        sel_hash ^= (int(loc[1] * 10.0) & 0xFFFFFFFF)
    return (
        tree.as_pointer(),
        len(tree.nodes),
        len(tree.links),
        round(scale, 4),
        round(_view_zoom(), 2),
        curving,
        int(wire_color),
        int(dragging),
        sel_n,
        sel_hash & 0xFFFFFFFF,
        _nodes_loc_token(tree),
    )


def _invalidate_draw_cache():
    global _cache_node_locs, _cache_n_links
    _path_cache["stamp"] = None
    _path_cache["batch"] = None
    _path_cache["batch_outline"] = None
    _path_cache["batch_under"] = None
    _path_cache["batch_under_outline"] = None
    _path_cache["batch_clip"] = None
    _path_cache["batch_clip_outline"] = None
    _path_cache["batch_insert"] = None
    _path_cache["batch_occlude"] = None
    _path_cache["batch_socks"] = None
    _path_cache["batch_frame"] = None
    _path_cache["batch_frame_outline"] = None
    _cache_node_locs = {}
    _cache_n_links = -1


def _rebuild_batches(tree, scale, curving, use_wire_color, select_col, insert_col, dragging):
    pos = []
    cols = []
    ipos = []
    icols = []
    fpos = []
    fcols = []
    dash = 10.0 * scale
    gap = 7.0 * scale
    period = dash + gap
    rects = _build_node_rect_cache(tree, scale)
    frame_rects, zone_rects = _collect_backdrop_rects(tree, scale, rects)
    frames = frame_rects + zone_rects
    hides = _collect_frame_hide_rects(tree, scale, frame_rects, zone_rects, rects) if frames else []
    items = list(_iter_link_polys(tree, scale, curving, rects))
    selected = getattr(bpy.context, "selected_nodes", None) or ()
    armed = dragging
    if not armed:
        for link, _poly in items:
            if _link_insert_ok(link):
                armed = True
                break
    ortho_target = _pick_ortho_insert(items, selected, scale) if armed else None
    global _steer_insert_ptr
    _steer_insert_ptr = ortho_target if dragging else None
    if dragging:
        _write_insert_target(tree, ortho_target)
    for link, poly in items:
        if len(poly) < 2 or any(not _finite_pt(p) for p in poly):
            continue
        fn, tn = link.from_node, link.to_node
        fs, ts = link.from_socket, link.to_socket
        insert_ok = _link_insert_ok(link) or (
            ortho_target is not None and link.as_pointer() == ortho_target
        )
        c0 = _socket_color_fast(fs) if use_wire_color else (0.55, 0.55, 0.55, 1.0)
        c1 = _socket_color_fast(ts) if use_wire_color else c0
        if (fn is not None and fn.select) or (tn is not None and tn.select):
            c0 = _mix(c0, select_col, 0.45)
            c1 = _mix(c1, select_col, 0.45)
        if getattr(link, "is_muted", False) or not getattr(link, "is_valid", True):
            c0 = c1 = (0.84, 0.22, 0.19, 1.0)
        if insert_ok:
            c0 = c1 = insert_col
        muted = bool(getattr(link, "is_muted", False)) and not insert_ok
        n_pts = max(len(poly) - 1, 1)
        acc = 0.0
        dst_pos = ipos if insert_ok else pos
        dst_cols = icols if insert_ok else cols
        for j in range(len(poly) - 1):
            pa, pb = poly[j], poly[j + 1]
            dx, dy = pb[0] - pa[0], pb[1] - pa[1]
            seg = math.hypot(dx, dy)
            if seg < 1.0e-4:
                continue
            tcol0 = _mix(c0, c1, j / n_pts)
            tcol1 = _mix(c0, c1, min(1.0, (j + 1) / n_pts))
            if muted:
                qdx, qdy = dx, dy
                qseg = seg
                t = 0.0
                while t < qseg:
                    local = (acc + t) % period
                    on = local < dash
                    remain = (dash - local) if on else (period - local)
                    step = min(qseg - t, remain)
                    if on and step > 0.35:
                        u0 = t / qseg
                        u1 = min(1.0, (t + step) / qseg)
                        aa = (pa[0] + qdx * u0, pa[1] + qdy * u0)
                        bb = (pa[0] + qdx * u1, pa[1] + qdy * u1)
                        _emit_split_seg(dst_pos, dst_cols, fpos, fcols, aa, bb, tcol0, tcol1, frames, hides)
                    t += max(step, 0.35)
                acc += qseg
                continue
            _emit_split_seg(dst_pos, dst_cols, fpos, fcols, pa, pb, tcol0, tcol1, frames, hides)
            acc += seg

    shader = _get_shader()
    _path_cache["batch"] = None
    _path_cache["batch_outline"] = None
    _path_cache["batch_under"] = None
    _path_cache["batch_under_outline"] = None
    _path_cache["batch_clip"] = None
    _path_cache["batch_clip_outline"] = None
    _path_cache["batch_insert"] = None
    _path_cache["batch_occlude"] = None
    _path_cache["batch_socks"] = None
    _path_cache["batch_frame"] = None
    _path_cache["batch_frame_outline"] = None
    if len(pos) >= 2:
        dark = [(0.0, 0.0, 0.0, 0.45)] * len(pos)
        try:
            _path_cache["batch_outline"] = batch_for_shader(shader, "LINES", {"pos": pos, "color": dark})
            _path_cache["batch"] = batch_for_shader(shader, "LINES", {"pos": pos, "color": cols})
        except Exception:
            _path_cache["batch"] = batch_for_shader(shader, "LINES", {"pos": pos})
    if len(ipos) >= 2:
        try:
            _path_cache["batch_insert"] = batch_for_shader(shader, "LINES", {"pos": ipos, "color": icols})
        except Exception:
            _path_cache["batch_insert"] = batch_for_shader(shader, "LINES", {"pos": ipos})
    if len(fpos) >= 2:
        fdark = [(0.0, 0.0, 0.0, 0.45)] * len(fpos)
        try:
            _path_cache["batch_frame_outline"] = batch_for_shader(
                shader, "LINES", {"pos": fpos, "color": fdark}
            )
            _path_cache["batch_frame"] = batch_for_shader(shader, "LINES", {"pos": fpos, "color": fcols})
        except Exception:
            _path_cache["batch_frame"] = batch_for_shader(shader, "LINES", {"pos": fpos})



def _draw_wire_batches(context, outline, batch, keep_depth=False):
    region = getattr(context, "region", None)
    if region is None or region.type != "WINDOW":
        return
    if int(getattr(region, "width", 0) or 0) < 8 or int(getattr(region, "height", 0) or 0) < 8:
        return
    if batch is None and outline is None:
        return
    shader = _get_shader()
    gpu.state.blend_set("ALPHA")
    try:
        if not keep_depth:
            try:
                gpu.state.depth_test_set("NONE")
                gpu.state.depth_mask_set(False)
            except Exception:
                pass
        shader.bind()
        try:
            shader.uniform_bool("lineSmooth", True)
        except Exception:
            pass
        try:
            w, h = _viewport_size(region)
            shader.uniform_float("viewportSize", (w, h))
        except Exception:
            pass
        outline_w, main_w, _insert_w = _link_draw_widths(region)
        if outline is not None:
            _set_line_width(shader, outline_w)
            outline.draw(shader)
        _set_line_width(shader, main_w)
        if batch is not None:
            batch.draw(shader)
    except Exception:
        pass
    finally:
        try:
            _set_line_width(shader, 1.0)
        except Exception:
            pass
        try:
            gpu.state.line_width_set(1.0)
        except Exception:
            pass
        gpu.state.blend_set("NONE")


def _draw_tri_wires(context, batch, keep_depth=False):
    region = getattr(context, "region", None)
    if region is None or region.type != "WINDOW" or batch is None:
        return
    if int(getattr(region, "width", 0) or 0) < 8 or int(getattr(region, "height", 0) or 0) < 8:
        return
    shader = _get_smooth_fill() or _get_fill_shader()
    if shader is None:
        return
    gpu.state.blend_set("ALPHA")
    try:
        if not keep_depth:
            try:
                gpu.state.depth_test_set("NONE")
                gpu.state.depth_mask_set(False)
            except Exception:
                pass
        shader.bind()
        batch.draw(shader)
    except Exception:
        pass
    finally:
        gpu.state.blend_set("NONE")


def _draw_cached_under_wires(context):
    return


def _draw_cached_main_wires(context, keep_depth=False):
    """Full unclipped noodles. Nodes drawn afterwards cover them."""
    _draw_wire_batches(
        context,
        _path_cache.get("batch_outline"),
        _path_cache.get("batch"),
        keep_depth=False,
    )


def _draw_cached_insert_wires(context, keep_depth=False):
    batch = _path_cache.get("batch_insert")
    if batch is None:
        return
    region = getattr(context, "region", None)
    if region is None or region.type != "WINDOW":
        return
    shader = _get_shader()
    gpu.state.blend_set("ALPHA")
    try:
        try:
            gpu.state.depth_test_set("NONE")
            gpu.state.depth_mask_set(False)
        except Exception:
            pass
        shader.bind()
        try:
            shader.uniform_bool("lineSmooth", True)
        except Exception:
            pass
        try:
            w, h = _viewport_size(region)
            shader.uniform_float("viewportSize", (w, h))
        except Exception:
            pass
        _outline_w, _main_w, insert_w = _link_draw_widths(region)
        _set_line_width(shader, insert_w)
        batch.draw(shader)
    except Exception:
        pass
    finally:
        try:
            _set_line_width(shader, 1.0)
        except Exception:
            pass
        gpu.state.blend_set("NONE")


def _draw_frame_overlay_wires(context):
    """Redraw noodles that cross Frames, after C++ has painted the Frame."""
    _draw_wire_batches(
        context,
        _path_cache.get("batch_frame_outline"),
        _path_cache.get("batch_frame"),
        keep_depth=False,
    )


def _draw_socket_caps():
    return


def _pre_view():
    global _pending_snap, _pre_synced, _nodes_are_dragging, _force_wires, _was_transform_modal
    global _was_link_modal, _wire_hide_ttl, _idle_link_snapshot
    context = bpy.context
    if not _enabled(context):
        _theme_hide_native_wires(False)
        return
    space = getattr(context, "space_data", None)
    tree = getattr(space, "edit_tree", None) if space is not None else None
    try:
        _restore_warped_sockets()
        if tree is None:
            return
        try:
            n_links = len(tree.links)
        except Exception:
            n_links = -1
        dragging = _transform_is_modal(context)
        linking = _find_node_link_op(context) is not None
        links_changed = _cache_n_links >= 0 and n_links != _cache_n_links
        loc_changed = _max_loc_delta(tree) > 0.01
        modal_ended = _was_transform_modal and not dragging
        link_ended = _was_link_modal and not linking
        if links_changed or _pending_snap or link_ended:
            _wire_hide_ttl = 12
        if modal_ended or _pending_snap or links_changed or link_ended:
            _drop_transient_interaction()
            _idle_link_snapshot = _snapshot_links(tree)
            _force_wires = True
            _pending_snap = False
            _path_cache["stamp"] = None
            _kick_redraw(4)
        _was_transform_modal = dragging
        _was_link_modal = linking
        _nodes_are_dragging = dragging
        if not dragging and _wire_hide_ttl:
            _wire_hide_ttl -= 1
        need_force = bool(
            loc_changed
            or links_changed
            or modal_ended
            or link_ended
            or _force_wires
            or _path_cache.get("batch") is None
        )
        # Iterate RNA links first, then hide DNA/runtime so C++ cannot rebuild noodles.
        _sync_link_batches(context, space, tree, dragging, force=need_force)
        _pre_synced = True
        _force_wires = False
        vec_ok = _hider.hide(tree)
        if linking:
            _linkdrag_hider.hide(context)
        else:
            _linkdrag_hider.restore(context)
        if not vec_ok:
            _wire_hide_ttl = max(_wire_hide_ttl, 6)
        need_theme = bool(_wire_hide_ttl > 0) and not linking
        _theme_hide_native_wires(need_theme)
        _draw_cached_main_wires(context)
        _draw_cached_insert_wires(context)
    except Exception:
        pass


def _post_view():
    context = bpy.context
    space = getattr(context, "space_data", None)
    tree = getattr(space, "edit_tree", None) if space is not None else None
    try:
        _linkdrag_hider.restore()
        _hider.restore_listbase()
        global _steer_insert_ptr
        if _enabled(context) and tree is not None:
            _draw_links(context, space, tree)
            if _steer_insert_ptr:
                selected = getattr(context, "selected_nodes", None) or ()
                scale = _ui_scale() or 1.0
                for link in tree.links:
                    if link.as_pointer() == _steer_insert_ptr:
                        _warp_link_through_selection(link, selected, scale)
                        break
        elif not _enabled(context):
            _steer_insert_ptr = None
    except Exception:
        pass
    finally:
        try:
            _theme_hide_native_wires(False)
        except Exception:
            pass
        try:
            _linkdrag_hider.restore()
        except Exception:
            pass
        try:
            _hider.restore(tree)
        except Exception:
            _hider.restore_all()


def _draw_stroke():
    if not _stroke_region or len(_stroke_region) < 2:
        return
    region = bpy.context.region
    if region is None:
        return
    shader = _get_shader()
    pos = [(x, y, 0.0) for x, y in _stroke_region]
    col = (1.0, 0.9, 0.2, 1.0)
    cols = [col] * len(pos)
    gpu.state.blend_set("ALPHA")
    shader.bind()
    try:
        try:
            w, h = _viewport_size(region)
            shader.uniform_float("viewportSize", (w, h))
        except Exception:
            pass
        _set_line_width(shader, 1.5)
        try:
            batch_for_shader(shader, "LINE_STRIP", {"pos": pos, "color": cols}).draw(shader)
        except Exception:
            try:
                shader.uniform_float("color", col)
            except Exception:
                pass
            batch_for_shader(shader, "LINE_STRIP", {"pos": pos}).draw(shader)
    finally:
        try:
            _set_line_width(shader, 1.0)
        except Exception:
            pass
        try:
            gpu.state.line_width_set(1.0)
        except Exception:
            pass
        gpu.state.blend_set("NONE")


def _is_node_link_op(op):
    name = str(getattr(op, "bl_idname", "") or "")
    compact = name.replace("_OT_", ".").replace("_ot_", ".").lower()
    return compact == "node.link" or name in {"NODE_OT_link", "node.link"}


def _iter_modal_ops(context):
    wm = getattr(context, "window_manager", None) if context is not None else None
    windows = []
    if wm is not None:
        try:
            windows.extend(list(wm.windows))
        except Exception:
            pass
    win = getattr(context, "window", None) if context is not None else None
    if win is not None and win not in windows:
        windows.insert(0, win)
    for w in windows:
        ops = getattr(w, "modal_operators", None)
        if not ops:
            continue
        try:
            for op in ops:
                yield op
        except Exception:
            continue


def _find_node_link_op(context):
    """Only live modal NODE_OT_link. active_operator stays set after mouse-up."""
    for op in _iter_modal_ops(context):
        if _is_node_link_op(op):
            return op
    return None


def _snapshot_links(tree):
    out = []
    try:
        for link in tree.links:
            if not _link_is_drawable(link):
                continue
            fn, tn = link.from_node, link.to_node
            fs, ts = link.from_socket, link.to_socket
            if fn is None or tn is None or fs is None or ts is None:
                continue
            out.append((fn, fs, tn, ts))
    except Exception:
        pass
    return out


def _iter_pick_sockets(tree, outputs_first):
    try:
        nodes = tree.nodes
    except Exception:
        return
    for node in nodes:
        if getattr(node, "bl_idname", "") == "NodeFrame":
            continue
        groups = ((node.outputs, True), (node.inputs, False))
        if not outputs_first:
            groups = ((node.inputs, False), (node.outputs, True))
        for coll, is_out in groups:
            try:
                socks = list(coll)
            except Exception:
                continue
            for sock in socks:
                if not _socket_is_visible(sock):
                    continue
                try:
                    xy = connection_xy(sock, node, None, 1, _ui_scale() or 1.0)
                except Exception:
                    xy = socket_xy(sock)
                if xy is None or not _finite_pt(xy):
                    continue
                yield node, sock, xy, is_out


def _nearest_socket(tree, pt, outputs_first, max_dist, skip_node=None, side=None):
    best = None
    best_d = max_dist
    skip_ptr = None
    if skip_node is not None:
        try:
            skip_ptr = skip_node.as_pointer()
        except Exception:
            skip_ptr = None
    for node, sock, xy, is_out in _iter_pick_sockets(tree, outputs_first):
        if side is not None and bool(is_out) != bool(side):
            continue
        if skip_ptr is not None:
            try:
                if node.as_pointer() == skip_ptr:
                    continue
            except Exception:
                pass
        d = math.hypot(xy[0] - pt[0], xy[1] - pt[1])
        if d < best_d:
            best_d = d
            best = (node, sock, xy, is_out)
    return best


def _cursor_view(space, scale):
    try:
        cl = space.cursor_location
        return (float(cl[0]) * scale, float(cl[1]) * scale)
    except Exception:
        return None


def _drag_start_view(op):
    try:
        ds = op.properties.drag_start
        pt = (float(ds[0]), float(ds[1]))
    except Exception:
        return None
    if not _finite_pt(pt):
        return None
    return pt


def _socket_pick_dist(region, scale):
    zoom = _view2d_scale_x(region)
    if not math.isfinite(zoom) or zoom < 1.0e-6:
        zoom = 1.0
    extra = max(5.0, min(30.0, 20.0 / zoom))
    return 0.25 * NODE_GRID_UNIT * scale + extra


def _sock_ptr(sock):
    if sock is None:
        return None
    try:
        return int(sock.as_pointer())
    except Exception:
        return None


def _op_is_detach(op):
    try:
        return bool(op.properties.detach)
    except Exception:
        return False


def _snapshot_incoming(snapshot, sock):
    ptr = _sock_ptr(sock)
    if ptr is None:
        return None
    for fn, fs, tn, ts in snapshot:
        try:
            if int(ts.as_pointer()) == ptr:
                return (fn, fs, tn, ts)
        except Exception:
            continue
    return None


def _snapshot_outgoing(snapshot, sock):
    ptr = _sock_ptr(sock)
    if ptr is None:
        return []
    out = []
    for fn, fs, tn, ts in snapshot:
        try:
            if int(fs.as_pointer()) == ptr:
                out.append((fn, fs, tn, ts))
        except Exception:
            continue
    return out


def _socket_link_limit(sock):
    try:
        v = int(getattr(sock, "link_limit", 0) or 0)
        if v > 0:
            return v
    except Exception:
        pass
    return 0


def _resolve_drag_anchor(tree, start_pt, max_dist, snapshot):
    # Closest socket of either side. Prefer the actual click, not outputs-first.
    hit_in = _nearest_socket(tree, start_pt, False, max_dist, side=False)
    hit_out = _nearest_socket(tree, start_pt, True, max_dist, side=True)
    hit = None
    if hit_in is None:
        hit = hit_out
    elif hit_out is None:
        hit = hit_in
    else:
        d_in = math.hypot(hit_in[2][0] - start_pt[0], hit_in[2][1] - start_pt[1])
        d_out = math.hypot(hit_out[2][0] - start_pt[0], hit_out[2][1] - start_pt[1])
        hit = hit_in if d_in <= d_out else hit_out
    if hit is None:
        return None
    return hit


def _emit_circle(dst_pos, dst_cols, center, radius, col, steps=14):
    if radius < 1.0 or not _finite_pt(center):
        return
    pts = []
    for i in range(steps):
        ang = (2.0 * math.pi * i) / steps
        pts.append((center[0] + radius * math.cos(ang), center[1] + radius * math.sin(ang)))
    for i in range(steps):
        _emit_line(dst_pos, dst_cols, pts[i], pts[(i + 1) % steps], col, col)


def _draw_colored_lines(shader, pos, cols, width):
    if len(pos) < 2:
        return
    _set_line_width(shader, width)
    try:
        batch_for_shader(shader, "LINES", {"pos": pos, "color": cols}).draw(shader)
    except Exception:
        try:
            shader.uniform_float("color", cols[0])
        except Exception:
            pass
        batch_for_shader(shader, "LINES", {"pos": pos}).draw(shader)


def _drag_socket_xy(node, sock, scale):
    if sock is None or node is None:
        return None
    try:
        xy = connection_xy(sock, node, None, 1, scale)
        if xy is not None and _finite_pt(xy):
            return xy
    except Exception:
        pass
    try:
        xy = socket_xy(sock)
        if xy is not None and _finite_pt(xy):
            return xy
    except Exception:
        pass
    return None


def _emit_drag_wire(shader, region, scale, p0, p3, fn, tn, fs, ts, from_sock):
    if not _finite_pt(p0) or not _finite_pt(p3):
        return
    if math.hypot(p3[0] - p0[0], p3[1] - p0[1]) < 0.5:
        return
    curving = _noodle_curving()
    fr = bool(fn) and getattr(fn, "bl_idname", "") == "NodeReroute"
    tr = bool(tn) and getattr(tn, "bl_idname", "") == "NodeReroute"
    from_rect = _node_view_rect(fn, scale) if fn is not None else None
    to_rect = _node_view_rect(tn, scale) if tn is not None else None
    wps = _angled_waypoints(p0, p3, fr, tr, scale, curving, from_rect, to_rect)
    poly = _filleted_polyline(wps, scale, curving)
    c0 = _socket_color_fast(fs if fs is not None else from_sock)
    c1 = _socket_color_fast(ts if ts is not None else (fs if fs is not None else from_sock))
    pos = []
    cols = []
    n_pts = max(len(poly) - 1, 1)
    for j in range(len(poly) - 1):
        tcol0 = _mix(c0, c1, j / n_pts)
        tcol1 = _mix(c0, c1, min(1.0, (j + 1) / n_pts))
        _emit_line(pos, cols, poly[j], poly[j + 1], tcol0, tcol1)
    if not pos:
        return
    outline_w, main_w, insert_w = _link_draw_widths(region)
    outline = []
    ocols = []
    oc = (0.0, 0.0, 0.0, 0.55)
    for j in range(0, len(pos), 2):
        if j + 1 < len(pos):
            outline.extend((pos[j], pos[j + 1]))
            ocols.extend((oc, oc))
    try:
        shader.bind()
        try:
            shader.uniform_bool("lineSmooth", True)
        except Exception:
            pass
        try:
            w, h = _viewport_size(region)
            shader.uniform_float("viewportSize", (w, h))
        except Exception:
            pass
        if outline:
            _draw_colored_lines(shader, outline, ocols, outline_w)
        _draw_colored_lines(shader, pos, cols, max(main_w, insert_w * 0.75))
    except Exception:
        pass


def _draw_drag_link(context, space, tree, region, shader, scale):
    global _idle_link_snapshot, _drag_anchor, _drag_op_key
    op = _find_node_link_op(context)
    if op is None:
        _drag_anchor = None
        _drag_op_key = None
        _idle_link_snapshot = _snapshot_links(tree)
        return
    try:
        op_key = int(op.as_pointer())
    except Exception:
        op_key = id(op)
    cur = _cursor_view(space, scale)
    start_pt = _drag_start_view(op)
    if cur is None or not _finite_pt(cur):
        cur = start_pt
    if cur is None or not _finite_pt(cur):
        return

    nldrag_rows = _iter_nldrag_links(op, getattr(_linkdrag_hider, "_vec_addr", None))
    if nldrag_rows:
        smap = _tree_socket_map(tree)
        drawn = False
        for _fn_ptr, _tn_ptr, fromsock, tosock in nldrag_rows:
            fn, fs = smap.get(int(fromsock or 0), (None, None))
            tn, ts = smap.get(int(tosock or 0), (None, None))
            if fs is None and ts is None:
                continue
            p0 = _drag_socket_xy(fn, fs, scale) if fs is not None else cur
            p3 = _drag_socket_xy(tn, ts, scale) if ts is not None else cur
            if p0 is None:
                p0 = cur
            if p3 is None:
                p3 = cur
            _emit_drag_wire(shader, region, scale, p0, p3, fn, tn, fs, ts, fs or ts)
            drawn = True
        if drawn:
            return

    pick = _socket_pick_dist(region, scale)
    if _is_gte_tree(tree):
        pick *= 2.5
    snap = _idle_link_snapshot or []
    if _drag_anchor is None or _drag_op_key != op_key:
        _drag_op_key = op_key
        _drag_anchor = _resolve_drag_anchor(tree, start_pt or cur, pick, snap)
    hit = _drag_anchor
    if hit is None:
        return
    node, sock, _xy, is_out = hit

    def _hover(want_out, skip):
        h = _nearest_socket(tree, cur, want_out, pick, skip_node=skip, side=want_out)
        if h is None:
            return None, None, cur
        return h[0], h[1], h[2]

    if is_out:
        outgoing = _snapshot_outgoing(snap, sock)
        limit = _socket_link_limit(sock)
        move_existing = bool(outgoing) and (
            _op_is_detach(op) or (limit > 0 and len(outgoing) >= limit)
        )
        if move_existing:
            # Ctrl / at-limit from output: inputs stay, output end follows cursor.
            h_node, h_sock, h_xy = _hover(True, None)
            for _fn, _fs, tn, ts in outgoing:
                p0 = h_xy if h_sock is not None else cur
                p3 = _drag_socket_xy(tn, ts, scale)
                if p3 is None:
                    continue
                _emit_drag_wire(
                    shader, region, scale, p0, p3,
                    h_node, tn, h_sock, ts, h_sock or ts,
                )
            return
        h_node, h_sock, h_xy = _hover(False, node)
        p0 = _drag_socket_xy(node, sock, scale)
        if p0 is None:
            return
        _emit_drag_wire(shader, region, scale, p0, h_xy, node, h_node, sock, h_sock, sock)
        return

    incoming = _snapshot_incoming(snap, sock)
    if incoming is not None:
        # Drag from an occupied input: move the existing link. Empty drop disconnects.
        fn, fs, _tn, _ts = incoming
        h_node, h_sock, h_xy = _hover(False, fn)
        p0 = _drag_socket_xy(fn, fs, scale)
        if p0 is None:
            return
        _emit_drag_wire(shader, region, scale, p0, h_xy, fn, h_node, fs, h_sock, fs)
        return

    # Empty input: new reverse drag from the input toward an output / cursor.
    h_node, h_sock, h_xy = _hover(True, node)
    p3 = _drag_socket_xy(node, sock, scale)
    if p3 is None:
        return
    _emit_drag_wire(shader, region, scale, h_xy, p3, h_node, node, h_sock, sock, sock)


def _region_view_bounds(region):
    v2d = getattr(region, "view2d", None)
    if v2d is None:
        return None
    try:
        a = v2d.region_to_view(0, 0)
        b = v2d.region_to_view(region.width, region.height)
    except TypeError:
        try:
            a = v2d.region_to_view(0, 0, False)
            b = v2d.region_to_view(region.width, region.height, False)
        except Exception:
            return None
    except Exception:
        return None
    x0, x1 = (a[0], b[0]) if a[0] <= b[0] else (b[0], a[0])
    y0, y1 = (a[1], b[1]) if a[1] <= b[1] else (b[1], a[1])
    if not (_finite_pt((x0, y0)) and _finite_pt((x1, y1))):
        return None
    return (x0, y0, x1, y1)


def _far_depth_batch(region, shader):
    bounds = _region_view_bounds(region)
    if bounds is None:
        return None
    x0, y0, x1, y1 = bounds
    pad = 8.0
    x0 -= pad
    y0 -= pad
    x1 += pad
    y1 += pad
    z = DEPTH_FAR_Z
    pos = (
        (x0, y0, z),
        (x1, y0, z),
        (x1, y1, z),
        (x0, y0, z),
        (x1, y1, z),
        (x0, y1, z),
    )
    try:
        return batch_for_shader(shader, "TRIS", {"pos": pos})
    except Exception:
        return None


def _clear_overlay_depth():
    getter = getattr(gpu.state, "active_framebuffer_get", None)
    if getter is not None:
        try:
            fb = getter()
            if fb is not None:
                fb.clear(depth=1.0)
                return True
        except Exception:
            pass
    try:
        import bgl

        bgl.glClear(bgl.GL_DEPTH_BUFFER_BIT)
        return True
    except Exception:
        return False


def _begin_node_depth_mask(context):
    """Depth-only node/socket cover. Frames are skipped so wires sit on them."""
    if getattr(gpu.state, "color_mask_set", None) is None:
        return False
    occ = _path_cache.get("batch_occlude")
    fill = _get_fill_shader()
    region = getattr(context, "region", None)
    if occ is None or fill is None or region is None:
        return False
    try:
        gpu.state.blend_set("NONE")
        gpu.state.depth_mask_set(True)
        gpu.state.depth_test_set("ALWAYS")
        gpu.state.color_mask_set(False, False, False, False)
        fill.bind()
        try:
            fill.uniform_float("color", (0.0, 0.0, 0.0, 1.0))
        except Exception:
            pass
        far = _far_depth_batch(region, fill)
        if far is not None:
            far.draw(fill)
        occ.draw(fill)
        gpu.state.color_mask_set(True, True, True, True)
        gpu.state.depth_mask_set(False)
        gpu.state.depth_test_set("LESS_EQUAL")
        return True
    except Exception:
        _end_node_depth_mask()
        return False


def _end_node_depth_mask():
    try:
        gpu.state.color_mask_set(True, True, True, True)
    except Exception:
        pass
    try:
        gpu.state.depth_mask_set(False)
        gpu.state.depth_test_set("NONE")
    except Exception:
        pass


def _draw_node_occluders():
    return False


def _draw_links(context, space, tree):
    region = context.region
    if region is None or region.type != "WINDOW":
        return
    if int(getattr(region, "width", 0) or 0) < 8 or int(getattr(region, "height", 0) or 0) < 8:
        return
    _restore_warped_sockets()
    scale = _ui_scale() or 1.0
    _harvest_socket_locals(tree, scale)
    dragging = _is_node_dragging(context)
    global _pre_synced
    if not _pre_synced:
        _sync_link_batches(context, space, tree, dragging, force=bool(dragging))
    _pre_synced = False

    global _steer_insert_ptr
    if not dragging:
        _steer_insert_ptr = None

    shader = _get_shader()
    gpu.state.blend_set("ALPHA")
    try:
        try:
            gpu.state.depth_test_set("NONE")
            gpu.state.depth_mask_set(False)
        except Exception:
            pass
        _draw_frame_overlay_wires(context)
        shader.bind()
        try:
            shader.uniform_bool("lineSmooth", True)
        except Exception:
            pass
        try:
            w, h = _viewport_size(region)
            shader.uniform_float("viewportSize", (w, h))
        except Exception:
            pass
        _draw_drag_link(context, space, tree, region, shader, scale)
    except Exception:
        pass
    try:
        gpu.state.depth_test_set("NONE")
        gpu.state.depth_mask_set(False)
    except Exception:
        pass
    try:
        _set_line_width(shader, 1.0)
    except Exception:
        pass
    try:
        gpu.state.line_width_set(1.0)
    except Exception:
        pass
    gpu.state.blend_set("NONE")


# ---------------------------------------------------------------------------
# Reroute / cut / mute against the visible orthogonal path
# ---------------------------------------------------------------------------

def _collect_hits(context, stroke_region):
    space = context.space_data
    tree = space.edit_tree
    region = context.region
    scale = _ui_scale() or 1.0
    curving = _noodle_curving()
    stroke_view = _region_stroke_to_view(region.view2d, stroke_region)
    hits = []
    for link, poly in _iter_link_polys(tree, scale, curving):
        hit = _stroke_hits_poly(stroke_view, poly)
        if hit is not None:
            hits.append((link, hit))
    return hits, scale


def _parent_reroute_to_frame(tree, reroute, pt, scale):
    frames = [n for n in tree.nodes if n.bl_idname == "NodeFrame"]
    frames.sort(key=lambda n: float(getattr(n, "width", 1.0)) * float(getattr(n, "height", 1.0)))
    loc_abs = (pt[0] / scale, pt[1] / scale)
    for frame in frames:
        locx, locy = _node_abs_loc(frame)
        width = float(getattr(frame, "width", 0.0)) * scale
        height = float(getattr(frame, "height", 0.0)) * scale
        x0, y1 = locx * scale, locy * scale
        if x0 <= pt[0] <= x0 + width and (y1 - height) <= pt[1] <= y1:
            try:
                reroute.parent = frame
                if hasattr(reroute, "location_absolute"):
                    reroute.location_absolute = loc_abs
            except Exception:
                pass
            return


def _end_gesture(context):
    _stroke_region.clear()
    try:
        context.window.cursor_modal_restore()
    except Exception:
        pass
    try:
        if context.area:
            context.area.tag_redraw()
    except Exception:
        pass


class _GestureBase:
    bl_options = {"REGISTER", "UNDO"}
    cursor = "CROSSHAIR"

    def invoke(self, context, event):
        if not _enabled(context) or context.region is None:
            return {"PASS_THROUGH"}
        self.path = [(float(event.mouse_region_x), float(event.mouse_region_y))]
        _stroke_region.clear()
        _stroke_region.extend(self.path)
        context.window_manager.modal_handler_add(self)
        try:
            context.window.cursor_modal_set(self.cursor)
        except Exception:
            pass
        context.area.tag_redraw()
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type == "MOUSEMOVE":
            pt = (float(event.mouse_region_x), float(event.mouse_region_y))
            path = self.path
            if not path or abs(pt[0] - path[-1][0]) + abs(pt[1] - path[-1][1]) > 1.0:
                if len(path) < 256:
                    path.append(pt)
                    _stroke_region[:] = path
                    if context.area:
                        context.area.tag_redraw()
            return {"RUNNING_MODAL"}
        if event.type in {"LEFTMOUSE", "RIGHTMOUSE", "MIDDLEMOUSE"} and event.value == "RELEASE":
            _end_gesture(context)
            return self.execute(context)
        if event.type in {"ESC", "RET", "NUMPAD_ENTER"}:
            _end_gesture(context)
            return {"CANCELLED"}
        return {"RUNNING_MODAL"}


class NODE_OT_angled_add_reroute(_GestureBase, bpy.types.Operator):
    bl_idname = "node.angled_add_reroute"
    bl_label = "Add Reroute"
    cursor = "CROSSHAIR"

    def execute(self, context):
        path = getattr(self, "path", None)
        if not path or len(path) < 2:
            return {"CANCELLED"}
        tree = context.space_data.edit_tree
        hits, scale = _collect_hits(context, path)
        if not hits:
            return {"CANCELLED"}

        for node in tree.nodes:
            node.select = False

        grouped = {}
        for link, pt in hits:
            key = link.from_socket.as_pointer()
            grouped.setdefault(key, []).append((link, pt))

        created = []
        for _key, group in grouped.items():
            from_socket = group[0][0].from_socket
            avgx = sum(pt[0] for _l, pt in group) / len(group)
            avgy = sum(pt[1] for _l, pt in group) / len(group)
            pt = (avgx, avgy)
            loc = (pt[0] / scale, pt[1] / scale)
            reroute = tree.nodes.new("NodeReroute")
            if hasattr(reroute, "location_absolute"):
                reroute.location_absolute = loc
            else:
                reroute.location = loc
            _parent_reroute_to_frame(tree, reroute, pt, scale)
            mute_all = all(getattr(link, "is_muted", False) for link, _pt in group)
            new_in = tree.links.new(from_socket, reroute.inputs[0])
            if mute_all and new_in is not None:
                try:
                    new_in.is_muted = True
                except Exception:
                    pass
            for link, _pt in group:
                to_socket = link.to_socket
                muted = bool(getattr(link, "is_muted", False))
                sort_id = getattr(link, "multi_input_sort_id", None)
                try:
                    tree.links.remove(link)
                except Exception:
                    continue
                new_out = tree.links.new(reroute.outputs[0], to_socket)
                if new_out is None:
                    continue
                if muted:
                    try:
                        new_out.is_muted = True
                    except Exception:
                        pass
                if sort_id is not None:
                    try:
                        new_out.multi_input_sort_id = sort_id
                    except Exception:
                        pass
            reroute.select = True
            created.append(reroute)

        if len(created) == 1:
            tree.nodes.active = created[0]
        _invalidate_draw_cache()
        return {"FINISHED"}


class NODE_OT_angled_links_cut(_GestureBase, bpy.types.Operator):
    bl_idname = "node.angled_links_cut"
    bl_label = "Cut Links"
    cursor = "KNIFE"

    def execute(self, context):
        path = getattr(self, "path", None)
        if not path or len(path) < 2:
            return {"CANCELLED"}
        tree = context.space_data.edit_tree
        hits, _scale = _collect_hits(context, path)
        if not hits:
            return {"CANCELLED"}
        for link, _pt in hits:
            try:
                tree.links.remove(link)
            except Exception:
                pass
        _invalidate_draw_cache()
        return {"FINISHED"}


def _propagate_mute(link, muted):
    stack = [link]
    seen = set()
    while stack:
        cur = stack.pop()
        key = cur.as_pointer()
        if key in seen:
            continue
        seen.add(key)
        try:
            cur.is_muted = muted
        except Exception:
            continue
        node = cur.to_node
        if node is not None and node.bl_idname == "NodeReroute" and node.outputs:
            for nxt in node.outputs[0].links:
                stack.append(nxt)


class NODE_OT_angled_links_mute(_GestureBase, bpy.types.Operator):
    bl_idname = "node.angled_links_mute"
    bl_label = "Mute Links"
    cursor = "STOP"

    def execute(self, context):
        path = getattr(self, "path", None)
        if not path or len(path) < 2:
            return {"CANCELLED"}
        hits, _scale = _collect_hits(context, path)
        if not hits:
            return {"CANCELLED"}
        for link, _pt in hits:
            muted = not bool(getattr(link, "is_muted", False))
            _propagate_mute(link, muted)
        _invalidate_draw_cache()
        return {"FINISHED"}


# ---------------------------------------------------------------------------
# Keymap remap (restored before userpref save)
# ---------------------------------------------------------------------------

_keymap_backup = []


def _install_keymap():
    global _keymap_installed
    if _keymap_installed:
        return
    _keymap_backup.clear()
    wm = bpy.context.window_manager
    for kc in (wm.keyconfigs.user, wm.keyconfigs.addon, wm.keyconfigs.default):
        if kc is None:
            continue
        for km in kc.keymaps:
            for kmi in km.keymap_items:
                if kmi.idname in _REMAP:
                    _keymap_backup.append((kmi, kmi.idname))
                    kmi.idname = _REMAP[kmi.idname]
    _keymap_installed = True


def _restore_keymap():
    global _keymap_installed
    for kmi, old in _keymap_backup:
        try:
            if kmi.idname in _REMAP.values():
                kmi.idname = old
        except Exception:
            pass
    _keymap_backup.clear()
    _keymap_installed = False


def _sync_keymap(enabled):
    if enabled:
        _install_keymap()
    else:
        _restore_keymap()


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------

class NODE_OT_angled_wires_toggle(bpy.types.Operator):
    bl_idname = "node.angled_wires_toggle"
    bl_label = "Toggle Angled Wires"
    bl_options = {"REGISTER"}

    def execute(self, context):
        context.window_manager.use_angled_wires = not context.window_manager.use_angled_wires
        return {"FINISHED"}


def _draw_overlay_entry(self, context):
    layout = self.layout
    layout.separator()
    layout.prop(context.window_manager, "use_angled_wires", text="Angled Wires")


def _overlay_panel():
    return getattr(bpy.types, "NODE_PT_overlay", None)


def _on_prop_update(self, context):
    enabled = bool(self.use_angled_wires)
    _apply_theme_hide(enabled)
    _sync_keymap(enabled)
    _invalidate_draw_cache()
    if not enabled:
        _hider.restore_all()
    for window in context.window_manager.windows:
        for area in window.screen.areas:
            if area.type == "NODE_EDITOR":
                area.tag_redraw()


def _reset_interaction_state():
    global _prev_sel_locs, _steer_insert_ptr, _idle_link_snapshot, _drag_anchor, _drag_op_key
    global _prefer_layout_xy, _nodes_are_dragging, _pending_snap, _pre_synced
    global _force_wires, _was_transform_modal, _was_link_modal, _wire_hide_ttl
    _restore_warped_sockets()
    _prev_sel_locs = {}
    _steer_insert_ptr = None
    _idle_link_snapshot = []
    _drag_anchor = None
    _drag_op_key = None
    _prefer_layout_xy = False
    _nodes_are_dragging = False
    _pending_snap = False
    _pre_synced = False
    _force_wires = False
    _was_transform_modal = False
    _was_link_modal = False
    _wire_hide_ttl = 0
    _theme_hide_native_wires(False)
    _sock_local.clear()
    _invalidate_draw_cache()
    _linkdrag_hider.restore()
    _hider.restore_all()


def _tag_node_editors():
    wm = bpy.context.window_manager
    if wm is None:
        return
    try:
        for window in wm.windows:
            for area in window.screen.areas:
                if area.type == "NODE_EDITOR":
                    area.tag_redraw()
    except Exception:
        pass


@persistent
def _on_save_pre(_dummy):
    _theme_hide_native_wires(False)
    _linkdrag_hider.restore()
    _hider.restore_all()
    _restore_keymap()


@persistent
def _on_save_post(_dummy):
    wm = bpy.context.window_manager
    if getattr(wm, "use_angled_wires", False):
        _install_keymap()


@persistent
def _on_load_post(_dummy):
    global _keymap_backup, _keymap_installed, _runtime_off, _loc_off
    global _tree_runtime_off, _links_vec_off, _links_lb_off
    _reset_interaction_state()
    _runtime_off = None
    _loc_off = None
    _tree_runtime_off = None
    _links_vec_off = None
    _links_lb_off = None
    _keymap_backup = []
    _keymap_installed = False
    wm = bpy.context.window_manager
    if getattr(wm, "use_angled_wires", False):
        _apply_theme_hide(True)
        _install_keymap()
        _tag_node_editors()


@persistent
def _on_undo_redo(_dummy):
    global _pending_snap, _wire_hide_ttl
    _drop_transient_interaction()
    _pending_snap = True
    _wire_hide_ttl = 12
    _request_wire_refresh(redraw=True)


@persistent
def _on_depsgraph_update(*args):
    wm = bpy.context.window_manager
    if wm is None or not getattr(wm, "use_angled_wires", False):
        return
    depsgraph = args[1] if len(args) > 1 else (args[0] if args else None)
    hit = False
    try:
        updates = getattr(depsgraph, "updates", None) if depsgraph is not None else None
        if updates:
            for upd in updates:
                idb = getattr(upd, "id", None)
                if idb is None:
                    continue
                try:
                    idb = idb.original
                except Exception:
                    pass
                if isinstance(idb, bpy.types.NodeTree):
                    hit = True
                    break
    except Exception:
        hit = True
    if not hit:
        return
    _force_wires_from_depsgraph()


def _force_wires_from_depsgraph():
    global _force_wires, _wire_hide_ttl
    _force_wires = True
    _path_cache["stamp"] = None
    _wire_hide_ttl = max(_wire_hide_ttl, 4)
    if _transform_is_modal(bpy.context):
        return
    _tag_node_editors()
    _kick_redraw(3)


_APP_HANDLERS = (
    ("save_pre", _on_save_pre),
    ("save_post", _on_save_post),
    ("load_post", _on_load_post),
    ("undo_post", _on_undo_redo),
    ("redo_post", _on_undo_redo),
    ("depsgraph_update_post", _on_depsgraph_update),
)


def _install_app_handlers():
    for name, fn in _APP_HANDLERS:
        group = getattr(bpy.app.handlers, name, None)
        if group is not None and fn not in group:
            group.append(fn)


def _remove_app_handlers():
    for name, fn in _APP_HANDLERS:
        group = getattr(bpy.app.handlers, name, None)
        if group is not None and fn in group:
            group.remove(fn)


classes = (
    NODE_OT_angled_wires_toggle,
    NODE_OT_angled_add_reroute,
    NODE_OT_angled_links_cut,
    NODE_OT_angled_links_mute,
)


def register():
    global _draw_handle_pre, _draw_handle_post, _draw_handle_stroke
    _color_by_idname.clear()
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.WindowManager.use_angled_wires = bpy.props.BoolProperty(
        name="Angled Wires",
        description="Draw orthogonal node links and hide native bezier noodles",
        default=True,
        update=_on_prop_update,
    )
    panel = _overlay_panel()
    if panel is not None:
        try:
            panel.append(_draw_overlay_entry)
        except Exception:
            pass
    if _draw_handle_pre is None:
        _draw_handle_pre = bpy.types.SpaceNodeEditor.draw_handler_add(
            _pre_view, (), "WINDOW", "PRE_VIEW"
        )
    if _draw_handle_post is None:
        _draw_handle_post = bpy.types.SpaceNodeEditor.draw_handler_add(
            _post_view, (), "WINDOW", "POST_VIEW"
        )
    if _draw_handle_stroke is None:
        _draw_handle_stroke = bpy.types.SpaceNodeEditor.draw_handler_add(
            _draw_stroke, (), "WINDOW", "POST_PIXEL"
        )
    _install_app_handlers()
    _apply_theme_hide(True)
    _install_keymap()


def unregister():
    global _draw_handle_pre, _draw_handle_post, _draw_handle_stroke, _shader, _fill_shader
    try:
        if bpy.app.timers.is_registered(_redraw_timer):
            bpy.app.timers.unregister(_redraw_timer)
    except Exception:
        pass
    _restore_keymap()
    _reset_interaction_state()
    _apply_theme_hide(False)
    _stroke_region.clear()
    if _draw_handle_pre is not None:
        bpy.types.SpaceNodeEditor.draw_handler_remove(_draw_handle_pre, "WINDOW")
        _draw_handle_pre = None
    if _draw_handle_post is not None:
        bpy.types.SpaceNodeEditor.draw_handler_remove(_draw_handle_post, "WINDOW")
        _draw_handle_post = None
    if _draw_handle_stroke is not None:
        bpy.types.SpaceNodeEditor.draw_handler_remove(_draw_handle_stroke, "WINDOW")
        _draw_handle_stroke = None
    _remove_app_handlers()
    panel = _overlay_panel()
    if panel is not None:
        try:
            panel.remove(_draw_overlay_entry)
        except Exception:
            pass
    del bpy.types.WindowManager.use_angled_wires
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    _shader = None


if __name__ == "__main__":
    register()
