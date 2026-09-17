# SPDX-FileCopyrightText: 2026 BIKINI
# SPDX-License-Identifier: GPL-3.0-or-later
"""Spatial / topology pairing and mirrored selection expansion.

Ports EditMeshSymmetryHelper + EDBM_select_expand_mirrored from Blender's
edit-mesh mirror path so topology operators can run on both sides at once.
"""

from __future__ import annotations

from collections import defaultdict

import bmesh
import bpy
from mathutils import Vector
from mathutils.kdtree import KDTree

# Same as BM_SEARCH_MAXDIST_MIRR / TRANSFORM_MAXDIST_MIRROR.
DEFAULT_MAXDIST = 0.00002
PLANE_EPS = 1e-4
UINT32 = 0xFFFFFFFF


def addon_prefs():
    addon = bpy.context.preferences.addons.get(__package__)
    return addon.preferences if addon else None


def topology_ops_enabled(context=None) -> bool:
    prefs = addon_prefs()
    if prefs is not None and not prefs.enable_topology_ops:
        return False
    return True


def mesh_axes(mesh) -> tuple[int, ...]:
    axes = []
    if getattr(mesh, "use_mirror_x", False):
        axes.append(0)
    if getattr(mesh, "use_mirror_y", False):
        axes.append(1)
    if getattr(mesh, "use_mirror_z", False):
        axes.append(2)
    return tuple(axes)


def should_mirror(context) -> bool:
    if not topology_ops_enabled(context):
        return False
    if context.mode != 'EDIT_MESH':
        return False
    for obj in context.objects_in_mode:
        if obj.type == 'MESH' and mesh_axes(obj.data):
            return True
    return False


def _maxdist() -> float:
    prefs = addon_prefs()
    if prefs is not None:
        return max(1e-7, float(prefs.spatial_threshold))
    return DEFAULT_MAXDIST


def edit_objects(context):
    objs = getattr(context, "objects_in_mode_unique_data", None)
    if objs:
        return [ob for ob in objs if ob.type == 'MESH']
    return [
        ob for ob in getattr(context, "objects_in_mode", [])
        if ob.type == 'MESH'
    ]


# ---------------------------------------------------------------------------
# Topology hash (official ED_mesh_mirrtopo_init)
# ---------------------------------------------------------------------------

def _topology_lookup(bm) -> list[int]:
    """Return partner vertex index per vertex, or self for unique/center hashes."""
    totvert = len(bm.verts)
    if totvert == 0:
        return []
    bm.verts.ensure_lookup_table()
    bm.verts.index_update()
    bm.edges.index_update()

    topo_hash = [0] * totvert
    for edge in bm.edges:
        i1, i2 = edge.verts[0].index, edge.verts[1].index
        topo_hash[i1] = (topo_hash[i1] + 1) & UINT32
        topo_hash[i2] = (topo_hash[i2] + 1) & UINT32

    topo_pass = 1
    tot_unique_prev = -1
    tot_unique_edges_prev = -1

    while True:
        prev = topo_hash[:]
        tot_unique_edges = 0
        new_hash = topo_hash[:]
        for edge in bm.edges:
            i1, i2 = edge.verts[0].index, edge.verts[1].index
            new_hash[i1] = (new_hash[i1] + (prev[i2] * topo_pass)) & UINT32
            new_hash[i2] = (new_hash[i2] + (prev[i1] * topo_pass)) & UINT32
            if new_hash[i1] != new_hash[i2]:
                tot_unique_edges += 1
        topo_hash = new_hash
        tot_unique = len(set(topo_hash))
        if tot_unique <= tot_unique_prev and tot_unique_edges <= tot_unique_edges_prev:
            break
        tot_unique_prev = tot_unique
        tot_unique_edges_prev = tot_unique_edges
        topo_pass += 1

    groups = defaultdict(list)
    for i, h in enumerate(topo_hash):
        groups[h].append(i)

    lookup = [-1] * totvert
    for idxs in groups.values():
        if len(idxs) == 2:
            lookup[idxs[0]] = idxs[1]
            lookup[idxs[1]] = idxs[0]
        elif len(idxs) == 1:
            lookup[idxs[0]] = idxs[0]
    return lookup


# ---------------------------------------------------------------------------
# Spatial pairing
# ---------------------------------------------------------------------------

def _spatial_lookup(bm, axis: int, maxdist: float) -> dict[int, int]:
    """Mutual 1:1 map of vertex index -> partner index along one axis."""
    tot = len(bm.verts)
    if tot == 0:
        return {}
    bm.verts.ensure_lookup_table()
    bm.verts.index_update()

    tree = KDTree(tot)
    for v in bm.verts:
        tree.insert(v.co, v.index)
    tree.balance()

    maxdist_sq = maxdist * maxdist
    paired: dict[int, int] = {}
    used: set[int] = set()

    for v in bm.verts:
        if v.hide or v.index in used:
            continue
        co = v.co.copy()
        co[axis] *= -1.0
        cluster = []
        cluster_dist = None
        for _loc, idx, dist in tree.find_n(co, 8):
            if dist > maxdist:
                break
            other = bm.verts[idx]
            if other.hide or idx == v.index:
                continue
            if (v.co - other.co).length_squared < maxdist_sq:
                continue
            if cluster_dist is None:
                cluster_dist = dist
            elif dist > cluster_dist + 1e-5:
                break
            cluster.append((idx, other))

        if not cluster:
            if abs(v.co[axis]) <= maxdist:
                paired[v.index] = v.index
                used.add(v.index)
            continue

        best = None
        best_score = -1
        ties = 0
        v_valence = len(v.link_edges)
        for idx, other in cluster:
            if idx in used:
                continue
            score = 2 if len(other.link_edges) == v_valence else 0
            if score > best_score:
                best_score = score
                best = idx
                ties = 1
            elif score == best_score:
                ties += 1
                best = None
        if best is None and len(cluster) == 1 and cluster[0][0] not in used:
            best = cluster[0][0]
            ties = 1
        if best is not None and ties == 1:
            paired[v.index] = best
            paired[best] = v.index
            used.add(v.index)
            used.add(best)
    return paired


def _add_partner(mapping: dict, a, b):
    if a is None or b is None or a == b:
        return
    mapping.setdefault(a, [])
    if b not in mapping[a]:
        mapping[a].append(b)
    mapping.setdefault(b, [])
    if a not in mapping[b]:
        mapping[b].append(a)


def _close_transitive(mapping: dict):
    progress = True
    while progress:
        progress = False
        keys = list(mapping.keys())
        for v in keys:
            partners = list(mapping.get(v, ()))
            for p in partners:
                for p2 in mapping.get(p, ()):
                    if p2 == v:
                        continue
                    dst = mapping.setdefault(v, [])
                    if p2 not in dst:
                        dst.append(p2)
                        progress = True
                    dst2 = mapping.setdefault(p2, [])
                    if v not in dst2:
                        dst2.append(v)


class SymmetryMaps:
    __slots__ = ("vert", "edge", "face")

    def __init__(self):
        self.vert: dict = {}
        self.edge: dict = {}
        self.face: dict = {}


def build_maps(bm, mesh, *, verts=True, edges=True, faces=True) -> SymmetryMaps | None:
    axes = mesh_axes(mesh)
    if not axes:
        return None
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    bm.verts.index_update()
    bm.edges.index_update()
    bm.faces.index_update()

    use_topo = bool(getattr(mesh, "use_mirror_topology", False))
    maxdist = _maxdist()
    maps = SymmetryMaps()
    topo_lookup = _topology_lookup(bm) if use_topo else None

    for axis in axes:
        if use_topo:
            lookup = {
                i: j for i, j in enumerate(topo_lookup)
                if j >= 0
            }
        else:
            lookup = _spatial_lookup(bm, axis, maxdist)

        if verts:
            for i, j in lookup.items():
                if i == j:
                    continue
                _add_partner(maps.vert, bm.verts[i], bm.verts[j])

        if edges:
            for edge in bm.edges:
                if edge.hide:
                    continue
                i1 = lookup.get(edge.verts[0].index, -1)
                i2 = lookup.get(edge.verts[1].index, -1)
                if i1 < 0 or i2 < 0 or i1 == i2:
                    continue
                other = bm.edges.get((bm.verts[i1], bm.verts[i2]))
                if other is not None and other != edge:
                    _add_partner(maps.edge, edge, other)

        if faces:
            for face in bm.faces:
                if face.hide:
                    continue
                mirr_verts = []
                ok = True
                for v in face.verts:
                    j = lookup.get(v.index, -1)
                    if j < 0:
                        ok = False
                        break
                    mirr_verts.append(bm.verts[j])
                if not ok:
                    continue
                other = _face_from_verts(mirr_verts)
                if other is not None and other != face:
                    _add_partner(maps.face, face, other)

    if verts:
        _close_transitive(maps.vert)
    return maps


def _face_from_verts(verts) -> bmesh.types.BMFace | None:
    if not verts:
        return None
    n = len(verts)
    want = set(verts)
    for face in verts[0].link_faces:
        if len(face.verts) == n and set(face.verts) == want:
            return face
    return None


# ---------------------------------------------------------------------------
# Span-plane filter (do not promote bridge faces/edges across the mirror)
# ---------------------------------------------------------------------------

def _edge_spans_plane(edge, axes) -> bool:
    for axis in axes:
        c1 = edge.verts[0].co[axis]
        c2 = edge.verts[1].co[axis]
        if (c1 > PLANE_EPS and c2 < -PLANE_EPS) or (c2 > PLANE_EPS and c1 < -PLANE_EPS):
            return True
    return False


def _face_spans_plane(face, axes) -> bool:
    for axis in axes:
        pos = False
        neg = False
        for v in face.verts:
            c = v.co[axis]
            pos = pos or c > PLANE_EPS
            neg = neg or c < -PLANE_EPS
            if pos and neg:
                return True
    return False


def expand_bmesh(bm, mesh) -> bool:
    """Select mirrored counterparts. Returns True if anything changed."""
    maps = build_maps(bm, mesh)
    if maps is None:
        return False
    axes = mesh_axes(mesh)

    tagged_edges = {e for e in bm.edges if e.select}
    tagged_faces = {f for f in bm.faces if f.select}

    changed = False
    for v in list(bm.verts):
        if v.select and not v.hide:
            for partner in maps.vert.get(v, ()):
                if not partner.hide and not partner.select:
                    partner.select = True
                    changed = True
    for e in list(bm.edges):
        if e.select and not e.hide:
            for partner in maps.edge.get(e, ()):
                if not partner.hide and not partner.select:
                    partner.select = True
                    changed = True
    for f in list(bm.faces):
        if f.select and not f.hide:
            for partner in maps.face.get(f, ()):
                if not partner.hide and not partner.select:
                    partner.select = True
                    changed = True

    bm.select_flush_mode()

    for e in bm.edges:
        if e.select and e not in tagged_edges and e not in maps.edge and _edge_spans_plane(e, axes):
            e.select = False
            changed = True
    for f in bm.faces:
        if f.select and f not in tagged_faces and f not in maps.face and _face_spans_plane(f, axes):
            f.select = False
            changed = True

    for f in bm.faces:
        if f.select:
            for loop in f.loops:
                loop.edge.select = True
                loop.vert.select = True
    for e in bm.edges:
        if e.select:
            e.verts[0].select = True
            e.verts[1].select = True
    return changed


def expand_context(context) -> bool:
    changed = False
    for obj in edit_objects(context):
        mesh = obj.data
        if not mesh_axes(mesh):
            continue
        bm = bmesh.from_edit_mesh(mesh)
        if expand_bmesh(bm, mesh):
            bmesh.update_edit_mesh(mesh, loop_triangles=False, destructive=False)
            changed = True
    return changed


# ---------------------------------------------------------------------------
# Active / user-side restore (so the following transform uses X-Mirror correctly)
# ---------------------------------------------------------------------------

def elem_center(ele) -> Vector | None:
    if isinstance(ele, bmesh.types.BMVert):
        return ele.co.copy()
    if isinstance(ele, bmesh.types.BMEdge):
        return (ele.verts[0].co + ele.verts[1].co) * 0.5
    if isinstance(ele, bmesh.types.BMFace):
        acc = Vector((0.0, 0.0, 0.0))
        for v in ele.verts:
            acc += v.co
        n = len(ele.verts)
        return acc / n if n else None
    return None


def capture_active_center(bm) -> Vector | None:
    if bm.select_history:
        return elem_center(bm.select_history[-1])
    for seq in (bm.faces, bm.edges, bm.verts):
        for ele in seq:
            if ele.select:
                return elem_center(ele)
    return None


def capture_context(context) -> dict:
    data = {}
    for obj in edit_objects(context):
        mesh = obj.data
        if not mesh_axes(mesh):
            continue
        bm = bmesh.from_edit_mesh(mesh)
        data[obj.name] = {
            "center": capture_active_center(bm),
            "axes": mesh_axes(mesh),
        }
    return data


def _same_side(co: Vector, ref: Vector, axes) -> bool:
    for axis in axes:
        rc = ref[axis]
        cc = co[axis]
        if abs(rc) <= PLANE_EPS or abs(cc) <= PLANE_EPS:
            continue
        if (rc > 0.0) != (cc > 0.0):
            return False
    return True


def restore_user_side_bmesh(bm, ref: Vector, axes):
    """Keep selection on the user's side so official X-Mirror drives the other side."""
    for f in bm.faces:
        if f.select:
            c = elem_center(f)
            if c is not None and not _same_side(c, ref, axes):
                f.select = False
    for e in bm.edges:
        if e.select:
            c = elem_center(e)
            if c is not None and not _same_side(c, ref, axes):
                e.select = False
    for v in bm.verts:
        if v.select and not _same_side(v.co, ref, axes):
            v.select = False
    restore_active_side_bmesh(bm, ref, axes)


def restore_active_side_bmesh(bm, ref: Vector, axes):
    """Newest selected element on the same side as `ref` becomes active."""
    best = None
    for ele in bm.select_history:
        c = elem_center(ele)
        if c is not None and _same_side(c, ref, axes):
            best = ele
    if best is None:
        for seq in (bm.faces, bm.edges, bm.verts):
            for ele in seq:
                if not ele.select:
                    continue
                c = elem_center(ele)
                if c is not None and _same_side(c, ref, axes):
                    best = ele
                    break
            if best is not None:
                break
    if best is not None:
        try:
            bm.select_history.add(best)
        except Exception:
            pass


def restore_user_side_context(context, captured: dict):
    for obj in edit_objects(context):
        rec = captured.get(obj.name)
        if not rec or rec["center"] is None:
            continue
        mesh = obj.data
        bm = bmesh.from_edit_mesh(mesh)
        restore_user_side_bmesh(bm, rec["center"], rec["axes"])
        bmesh.update_edit_mesh(mesh, loop_triangles=False, destructive=False)


def restore_active_context(context, captured: dict):
    for obj in edit_objects(context):
        rec = captured.get(obj.name)
        if not rec or rec["center"] is None:
            continue
        mesh = obj.data
        bm = bmesh.from_edit_mesh(mesh)
        restore_active_side_bmesh(bm, rec["center"], rec["axes"])
        bmesh.update_edit_mesh(mesh, loop_triangles=False, destructive=False)


def mirror_partners_world(obj, mesh):
    """World-space coords of unselected mirror verts/edges/faces of the selection."""
    bm = bmesh.from_edit_mesh(mesh)
    maps = build_maps(bm, mesh)
    if maps is None:
        return [], [], []
    mw = obj.matrix_world
    verts = []
    edges = []
    faces = []
    seen_v = set()
    seen_e = set()
    seen_f = set()
    for v in bm.verts:
        if not v.select or v.hide:
            continue
        for p in maps.vert.get(v, ()):
            if p.hide or p.select or p.index in seen_v:
                continue
            seen_v.add(p.index)
            verts.append(mw @ p.co)
    for e in bm.edges:
        if not e.select or e.hide:
            continue
        for p in maps.edge.get(e, ()):
            if p.hide or p.select or p.index in seen_e:
                continue
            seen_e.add(p.index)
            edges.append((mw @ p.verts[0].co, mw @ p.verts[1].co))
    for f in bm.faces:
        if not f.select or f.hide:
            continue
        for p in maps.face.get(f, ()):
            if p.hide or p.select or p.index in seen_f:
                continue
            seen_f.add(p.index)
            cos = [mw @ v.co for v in p.verts]
            faces.append(cos)
    return verts, edges, faces


def find_mirror_edge(bm, mesh, edge) -> list:
    maps = build_maps(bm, mesh, verts=True, edges=True, faces=False)
    if maps is None:
        return []
    return [e for e in maps.edge.get(edge, ()) if e != edge]
