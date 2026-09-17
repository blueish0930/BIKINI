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
    "version": (1, 3, 2),
    "blender": (3, 6, 0),
    "location": "Node Editor > Overlays",
    "description": "Orthogonal node links without touching the node tree",
    "category": "Node",
}

CIRCLE_K = 0.5522847498
MIN_ROUND = 3.0
LINK_WIDTH = 2.5
INSERT_WIDTH = 5.0
NODE_GRID_UNIT = 20.0
PTR_SIZE = ctypes.sizeof(ctypes.c_void_p)
FILLET_STEPS = 3
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
    "batch_insert": None,
}
_tree_runtime_off = None
_links_vec_off = None
_links_lb_off = None
_steer_insert_ptr = None

_SOCKET_COLORS = {
    "CUSTOM": (0.63, 0.63, 0.63, 1.0),
    "VALUE": (0.63, 0.63, 0.63, 1.0),
    "INT": (0.35, 0.55, 0.36, 1.0),
    "BOOLEAN": (0.80, 0.65, 0.84, 1.0),
    "VECTOR": (0.39, 0.39, 0.78, 1.0),
    "ROTATION": (0.65, 0.39, 0.78, 1.0),
    "MATRIX": (0.65, 0.39, 0.78, 1.0),
    "RGBA": (0.78, 0.78, 0.16, 1.0),
    "STRING": (0.44, 0.70, 1.00, 1.0),
    "SHADER": (0.39, 0.78, 0.39, 1.0),
    "GEOMETRY": (0.00, 0.84, 0.64, 1.0),
    "OBJECT": (0.93, 0.62, 0.36, 1.0),
    "COLLECTION": (0.96, 0.96, 0.96, 1.0),
    "IMAGE": (0.39, 0.22, 0.39, 1.0),
    "MATERIAL": (0.92, 0.46, 0.51, 1.0),
    "TEXTURE": (0.70, 0.46, 0.71, 1.0),
    "MENU": (0.40, 0.40, 0.40, 1.0),
    "BUNDLE": (0.60, 0.60, 0.60, 1.0),
    "CLOSURE": (0.39, 0.78, 0.39, 1.0),
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
    dpi = float(getattr(prefs, "dpi", 72) or 72)
    return dpi / 72.0


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


def _apply_theme_hide(hide):
    global _theme_backup
    try:
        ne = bpy.context.preferences.themes[0].node_editor
        names = [name for name in ("wire", "wire_inner", "wire_select") if hasattr(ne, name)]
        if hide:
            if _theme_backup is None:
                _theme_backup = {name: tuple(getattr(ne, name)) for name in names}
            for name in names:
                zeroed = _theme_zero(getattr(ne, name))
                if zeroed is not None:
                    setattr(ne, name, zeroed)
            return
        if _theme_backup is None:
            return
        for name, value in _theme_backup.items():
            if hasattr(ne, name):
                setattr(ne, name, value)
        _theme_backup = None
    except Exception:
        pass


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


def _xy_plausible(xy, node, socket, scale):
    if xy is None:
        return False
    x, y = xy
    nx, ny = _node_abs_loc(node)
    sx = nx * scale
    sy = ny * scale
    width = float(getattr(node, "width", 140.0)) * scale
    if getattr(node, "bl_idname", "") == "NodeReroute":
        return math.hypot(x - sx, y - sy) < 160.0
    if socket.is_output:
        return abs(x - (sx + width)) < 160.0 and (sy + 120.0) >= y >= (sy - 900.0)
    return abs(x - sx) < 160.0 and (sy + 120.0) >= y >= (sy - 900.0)


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


def connection_xy(socket, node, link, multi_count, scale):
    xy = socket_xy(socket)
    if xy is None:
        locx, locy = _node_abs_loc(node)
        width = float(getattr(node, "width", 140.0))
        coll = node.inputs if not socket.is_output else node.outputs
        visible = [s for s in coll if not getattr(s, "hide", False)]
        try:
            index = visible.index(socket)
        except ValueError:
            index = 0
        y = locy - 22.0 * (index + 0.65)
        x = locx if not socket.is_output else locx + width
        if node.bl_idname == "NodeReroute":
            xy = (locx * scale, locy * scale)
        else:
            xy = (x * scale, y * scale)
    if getattr(socket, "is_multi_input", False) and not socket.is_output and not getattr(node, "hide", False):
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
    try:
        node = tree.nodes[0]
        sock = node.outputs[0] if node.outputs else node.inputs[0]
        if hasattr(sock, "is_icon_visible"):
            bool(sock.is_icon_visible)
        else:
            bool(getattr(sock, "enabled", True))
            len(getattr(sock, "links", ()))
    except Exception:
        pass


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

    def _find_runtime_off(self, tree_p, ptrs):
        first, last = ptrs[0], ptrs[-1]
        lb = None
        for off in range(192, 1600, 8):
            a = _u64_safe(tree_p + off)
            b = _u64_safe(tree_p + off + 8)
            if a is None or b is None:
                break
            if a == first and b == last:
                lb = off
                break
        if lb is None:
            return None
        runtime_off = None
        for off in range(lb + 16, lb + 400, 8):
            p = _u64_safe(tree_p + off)
            if p is None:
                break
            if p and _readable(p):
                runtime_off = off
        return runtime_off

    def _find_vec_off(self, runtime, ptrs):
        n = len(ptrs)
        expect = n * PTR_SIZE
        max_scan = 16384
        off = 0
        while off + 24 <= max_scan:
            if not _readable_range(runtime + off, 24):
                break
            begin = _u64(runtime + off)
            end = _u64(runtime + off + 8)
            cap = _u64(runtime + off + 16)
            if not begin or end != begin + expect or cap < end:
                off += 8
                continue
            if not _readable_range(begin, expect):
                off += 8
                continue
            match = True
            for i, ptr in enumerate(ptrs):
                got = _u64_safe(begin + i * PTR_SIZE)
                if got != ptr:
                    match = False
                    break
            if match:
                return off
            off += 8
        return None

    def _scan_vector(self, tree):
        global _tree_runtime_off, _links_vec_off
        n = len(tree.links)
        if n == 0:
            return None
        tree_p = tree.as_pointer()
        first = tree.links[0].as_pointer()
        if _tree_runtime_off is not None and _links_vec_off is not None:
            runtime = _u64_safe(tree_p + _tree_runtime_off)
            if runtime:
                addr = runtime + _links_vec_off
                begin = _u64_safe(addr)
                end = _u64_safe(addr + 8)
                if begin and end == begin + n * PTR_SIZE and _u64_safe(begin) == first:
                    return addr
        _touch_topology(tree)
        ptrs = self._link_ptrs(tree)
        if not ptrs:
            return None
        runtime_off = self._find_runtime_off(tree_p, ptrs)
        if runtime_off is None:
            return None
        runtime = _u64_safe(tree_p + runtime_off)
        if not runtime or not _readable(runtime):
            return None
        vec_off = self._find_vec_off(runtime, ptrs)
        if vec_off is None:
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
        for off in range(192, 1600, 8):
            a = _u64_safe(tree_p + off)
            b = _u64_safe(tree_p + off + 8)
            if a is None or b is None:
                break
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

    def _hide_vector(self, tree):
        addr = self._scan_vector(tree)
        if addr is None:
            return False
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
        if tree is None:
            return
        if self._depth:
            self._depth += 1
            return
        if _DRAW_USES_RUNTIME_VEC:
            # 4.2+ draws from runtime Vector; RNA still walks ListBase.
            self._hide_vector(tree)
        else:
            # 3.6 draws from DNA ListBase. Restore it before Python iterates.
            if not self._hide_listbase(tree):
                self._hide_vector(tree)
        self._depth = 1

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


# ---------------------------------------------------------------------------
# Orthogonal path
# ---------------------------------------------------------------------------

def _angled_waypoints(p0, p3, from_reroute, to_reroute, scale, curving):
    radius = 0.0 if curving <= 0 else float(curving) * 8.0 * scale
    keep = 8.0 * scale
    lead = keep + max(radius, 16.0 * scale)
    unit = NODE_GRID_UNIT * scale
    grid = unit
    align = max(grid, 0.75 * unit)
    min_rail = max(2.0 * max(radius, 1.0), unit)
    dual_need_x = 2.0 * lead + min_rail
    dx = p3[0] - p0[0]
    dy = p3[1] - p0[1]

    def straight():
        return [p0, p3]

    def corner_start():
        out_x = p0[0] + lead
        return [p0, (out_x, p0[1]), (out_x, p3[1]), p3]

    def corner_end():
        in_x = p3[0] - lead
        return [p0, (in_x, p0[1]), (in_x, p3[1]), p3]

    def double_corner():
        bump = max(radius, 0.8 * unit)
        rail_y = max(p0[1], p3[1]) + bump
        out_x = p0[0] + lead
        in_x = p3[0] - lead
        return [p0, (out_x, p0[1]), (out_x, rail_y), (in_x, rail_y), (in_x, p3[1]), p3]

    if abs(dy) <= align:
        return straight()
    if from_reroute and to_reroute and abs(dx) <= align:
        return straight()
    if dx >= 0.0 and dx < lead:
        return straight()
    if dx < 0.0 and abs(dx) < lead and abs(dy) < 3.0 * unit:
        return straight()
    if from_reroute != to_reroute:
        return corner_start() if to_reroute else corner_end()
    if dx < -lead and abs(dx) >= dual_need_x and abs(dy) > 2.0 * grid:
        return double_corner()
    if dx >= 0.0:
        return corner_start()
    return straight()


def _filleted_polyline(wps, scale, curving):
    if len(wps) <= 2:
        return list(wps)
    radius = 0.0 if curving <= 0 else float(curving) * 8.0 * scale
    keep = 8.0 * scale
    min_round = MIN_ROUND * scale
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
        k = r * CIRCLE_K
        c1 = (fs[0] + tinx * k, fs[1] + tiny * k)
        c2 = (fe[0] - toutx * k, fe[1] - touty * k)
        steps = FILLET_STEPS
        for s in range(1, steps):
            t = s / steps
            u = 1.0 - t
            pts.append((
                (u ** 3) * fs[0] + 3.0 * (u ** 2) * t * c1[0] + 3.0 * u * (t ** 2) * c2[0] + (t ** 3) * fe[0],
                (u ** 3) * fs[1] + 3.0 * (u ** 2) * t * c1[1] + 3.0 * u * (t ** 2) * c2[1] + (t ** 3) * fe[1],
            ))
        pts.append(fe)
    pts.append(wps[-1])
    return pts


def _mix(a, b, t):
    s = 1.0 - t
    return (a[0] * s + b[0] * t, a[1] * s + b[1] * t, a[2] * s + b[2] * t, 1.0)


def _socket_color_fast(socket):
    return _SOCKET_COLORS.get(getattr(socket, "type", ""), (0.7, 0.7, 0.7, 1.0))


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


def _is_node_dragging(context):
    op = getattr(context, "active_operator", None)
    if op is not None:
        name = getattr(op, "bl_idname", "").replace("_OT_", ".").lower()
        if any(token in name for token in ("transform", "translate", "attach")):
            return True
    selected = getattr(context, "selected_nodes", None) or ()
    cur = {}
    for node in selected:
        loc = _node_abs_loc(node)
        cur[node.as_pointer()] = (round(loc[0], 2), round(loc[1], 2))
    global _prev_sel_locs
    dragging = bool(cur) and bool(_prev_sel_locs) and set(cur) == set(_prev_sel_locs) and cur != _prev_sel_locs
    _prev_sel_locs = cur
    return dragging


def _warp_link_through_selection(link, selected, scale):
    """Steer C++ insert-on-link onto the visible orthogonal wire.

    ``node_insert_on_link_flags_set`` tests the evaluated noodle against the
    dragged node. After drawing from real socket positions, poke runtime
    locations so the next flush (including mouse-up) sees a segment through
    the node near its top-left — the same point C++ uses for distance.
    """
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
    _write_socket_xy(fs, x0 - 12.0, y)
    _write_socket_xy(ts, x1 + 12.0, y)


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


def _iter_link_polys(tree, scale, curving):
    multi_counts = {}
    links = tree.links
    for link in links:
        sock = link.to_socket
        if sock is not None and getattr(sock, "is_multi_input", False):
            key = sock.as_pointer()
            multi_counts[key] = multi_counts.get(key, 0) + 1

    for link in links:
        fn, tn = link.from_node, link.to_node
        fs, ts = link.from_socket, link.to_socket
        if fn is None or tn is None or fs is None or ts is None:
            continue
        p0 = connection_xy(fs, fn, link, 1, scale)
        p3 = connection_xy(ts, tn, link, multi_counts.get(ts.as_pointer(), 1), scale)
        wps = _angled_waypoints(
            p0, p3, fn.bl_idname == "NodeReroute", tn.bl_idname == "NodeReroute", scale, curving
        )
        yield link, _filleted_polyline(wps, scale, curving)


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
# Draw (POST_VIEW, view space — pan/zoom does not rebuild geometry)
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


def _set_line_width(shader, width):
    global _shader_has_line_width
    if _shader_has_line_width:
        try:
            shader.uniform_float("lineWidth", width)
            return
        except Exception:
            _shader_has_line_width = False
    try:
        gpu.state.line_width_set(width)
    except Exception:
        pass


def _enabled(context):
    wm = getattr(context, "window_manager", None)
    if wm is None or not getattr(wm, "use_angled_wires", False):
        return False
    space = getattr(context, "space_data", None)
    return space is not None and space.type == "NODE_EDITOR" and space.edit_tree is not None


def _path_stamp(context, tree, scale, curving, wire_color, dragging):
    selected = getattr(context, "selected_nodes", None) or ()
    sel_parts = []
    for node in selected:
        loc = _node_abs_loc(node)
        sel_parts.append((node.as_pointer(), round(loc[0], 3), round(loc[1], 3)))
    return (
        tree.as_pointer(),
        len(tree.nodes),
        len(tree.links),
        round(scale, 4),
        curving,
        int(wire_color),
        int(dragging),
        tuple(sel_parts),
        _insert_signature(tree) if sel_parts else 0,
    )


def _invalidate_draw_cache():
    _path_cache["stamp"] = None
    _path_cache["batch"] = None
    _path_cache["batch_outline"] = None
    _path_cache["batch_insert"] = None


def _rebuild_batches(tree, scale, curving, use_wire_color, select_col, insert_col, dragging):
    pos = []
    cols = []
    ipos = []
    icols = []
    dash = 10.0 * scale
    gap = 7.0 * scale
    period = dash + gap
    items = list(_iter_link_polys(tree, scale, curving))
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
        if ortho_target is not None:
            for link, _poly in items:
                if link.as_pointer() == ortho_target:
                    _warp_link_through_selection(link, selected, scale)
                    break
    for link, poly in items:
        if len(poly) < 2:
            continue
        fn, tn = link.from_node, link.to_node
        fs, ts = link.from_socket, link.to_socket
        insert_ok = _link_insert_ok(link) or (
            ortho_target is not None and link.as_pointer() == ortho_target
        )
        c0 = _socket_color_fast(fs) if use_wire_color else (0.55, 0.55, 0.55, 1.0)
        c1 = _socket_color_fast(ts) if use_wire_color else c0
        if fn.select or tn.select:
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
            if seg < 0.4:
                continue
            tcol0 = _mix(c0, c1, j / n_pts)
            tcol1 = _mix(c0, c1, min(1.0, (j + 1) / n_pts))
            if muted:
                t = 0.0
                while t < seg:
                    local = (acc + t) % period
                    on = local < dash
                    remain = (dash - local) if on else (period - local)
                    step = min(seg - t, remain)
                    if on and step > 0.35:
                        u0 = t / seg
                        u1 = min(1.0, (t + step) / seg)
                        a = (pa[0] + dx * u0, pa[1] + dy * u0)
                        b = (pa[0] + dx * u1, pa[1] + dy * u1)
                        dst_pos.extend(((a[0], a[1], 0.0), (b[0], b[1], 0.0)))
                        dst_cols.extend((tcol0, tcol1))
                    t += max(step, 0.35)
                acc += seg
                continue
            dst_pos.extend(((pa[0], pa[1], 0.0), (pb[0], pb[1], 0.0)))
            dst_cols.extend((tcol0, tcol1))
            acc += seg

    shader = _get_shader()
    _path_cache["batch"] = None
    _path_cache["batch_outline"] = None
    _path_cache["batch_insert"] = None
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


def _pre_view():
    context = bpy.context
    if not _enabled(context):
        return
    try:
        _hider.hide(context.space_data.edit_tree)
    except Exception:
        pass


def _post_view():
    context = bpy.context
    space = getattr(context, "space_data", None)
    tree = getattr(space, "edit_tree", None) if space is not None else None
    try:
        _hider.restore_listbase()
        if _enabled(context):
            _draw_links(context, space, tree)
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
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
        shader.uniform_float("viewportSize", (float(region.width), float(region.height)))
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
    gpu.state.blend_set("NONE")


def _draw_links(context, space, tree):
    region = context.region
    if region is None or region.type != "WINDOW":
        return
    scale = _ui_scale() or 1.0
    overlay = getattr(space, "overlay", None)
    use_wire_color = True
    if overlay is not None and hasattr(overlay, "show_wire_color"):
        use_wire_color = bool(overlay.show_wire_color)
    curving = _noodle_curving()
    dragging = _is_node_dragging(context)
    pstamp = _path_stamp(context, tree, scale, curving, use_wire_color, dragging)
    if _path_cache["stamp"] != pstamp:
        select_col = (0.90, 0.62, 0.10, 1.0)
        try:
            ws = context.preferences.themes[0].node_editor.wire_select
            select_col = (float(ws[0]), float(ws[1]), float(ws[2]), 1.0)
        except Exception:
            pass
        insert_col = (0.90, 0.55, 0.18, 1.0)
        try:
            na = context.preferences.themes[0].node_editor.node_active
            insert_col = (float(na[0]), float(na[1]), float(na[2]), 1.0)
        except Exception:
            pass
        _rebuild_batches(tree, scale, curving, use_wire_color, select_col, insert_col, dragging)
        _path_cache["stamp"] = pstamp

    global _steer_insert_ptr
    if dragging and _steer_insert_ptr:
        selected = getattr(context, "selected_nodes", None) or ()
        for link in tree.links:
            if link.as_pointer() == _steer_insert_ptr:
                _warp_link_through_selection(link, selected, scale)
                break
    elif not dragging:
        _steer_insert_ptr = None

    batch = _path_cache["batch"]
    insert = _path_cache.get("batch_insert")
    if batch is None and insert is None:
        return
    shader = _get_shader()
    gpu.state.blend_set("ALPHA")
    shader.bind()
    try:
        vp = gpu.state.viewport_get()
        shader.uniform_float("viewportSize", (float(vp[2]), float(vp[3])))
    except Exception:
        try:
            shader.uniform_float("viewportSize", (float(region.width), float(region.height)))
        except Exception:
            pass
    try:
        outline = _path_cache["batch_outline"]
        if outline is not None:
            _set_line_width(shader, LINK_WIDTH + 1.6)
            outline.draw(shader)
        _set_line_width(shader, LINK_WIDTH)
        if batch is not None:
            batch.draw(shader)
        insert = _path_cache.get("batch_insert")
        if insert is not None:
            _set_line_width(shader, INSERT_WIDTH)
            insert.draw(shader)
    except Exception:
        try:
            shader.uniform_float("color", (0.7, 0.7, 0.7, 1.0))
        except Exception:
            pass
        _set_line_width(shader, LINK_WIDTH)
        if batch is not None:
            batch.draw(shader)
        insert = _path_cache.get("batch_insert")
        if insert is not None:
            insert.draw(shader)
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


@persistent
def _on_save_pre(_dummy):
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
    global _tree_runtime_off, _links_vec_off, _links_lb_off, _steer_insert_ptr
    _hider.restore_all()
    _invalidate_draw_cache()
    _runtime_off = None
    _loc_off = None
    _tree_runtime_off = None
    _links_vec_off = None
    _links_lb_off = None
    _steer_insert_ptr = None
    _keymap_backup = []
    _keymap_installed = False
    wm = bpy.context.window_manager
    if getattr(wm, "use_angled_wires", False):
        _apply_theme_hide(True)
        _install_keymap()


classes = (
    NODE_OT_angled_wires_toggle,
    NODE_OT_angled_add_reroute,
    NODE_OT_angled_links_cut,
    NODE_OT_angled_links_mute,
)


def register():
    global _draw_handle_pre, _draw_handle_post, _draw_handle_stroke
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
    if _on_save_pre not in bpy.app.handlers.save_pre:
        bpy.app.handlers.save_pre.append(_on_save_pre)
    if _on_save_post not in bpy.app.handlers.save_post:
        bpy.app.handlers.save_post.append(_on_save_post)
    if _on_load_post not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_on_load_post)
    _apply_theme_hide(True)
    _install_keymap()


def unregister():
    global _draw_handle_pre, _draw_handle_post, _draw_handle_stroke, _shader
    _restore_keymap()
    _hider.restore_all()
    _apply_theme_hide(False)
    _stroke_region.clear()
    _invalidate_draw_cache()
    if _draw_handle_pre is not None:
        bpy.types.SpaceNodeEditor.draw_handler_remove(_draw_handle_pre, "WINDOW")
        _draw_handle_pre = None
    if _draw_handle_post is not None:
        bpy.types.SpaceNodeEditor.draw_handler_remove(_draw_handle_post, "WINDOW")
        _draw_handle_post = None
    if _draw_handle_stroke is not None:
        bpy.types.SpaceNodeEditor.draw_handler_remove(_draw_handle_stroke, "WINDOW")
        _draw_handle_stroke = None
    if _on_save_pre in bpy.app.handlers.save_pre:
        bpy.app.handlers.save_pre.remove(_on_save_pre)
    if _on_save_post in bpy.app.handlers.save_post:
        bpy.app.handlers.save_post.remove(_on_save_post)
    if _on_load_post in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_on_load_post)
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
