# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""SDF Shape: Distance-only sockets, per-shape param mapping, IP cook, Shader MF."""

import math
import sys
import traceback

import bpy


def fail(msg):
    print("FAIL:", msg)
    raise RuntimeError(msg)


def check(cond, msg):
    if not cond:
        fail(msg)
    print("PASS:", msg)


def near(a, b, tol, msg):
    if abs(a - b) > tol:
        fail(f"{msg}: got {a} expected {b} ± {tol}")
    print(f"PASS: {msg} ({a} ≈ {b})")


def sample_viewer(x, y):
    img = bpy.data.images.get("Viewer Node")
    if img is None:
        fail(f"no Viewer Node; images={[i.name for i in bpy.data.images]}")
    w, h = img.size
    i = (y * w + x) * 4
    return w, h, list(img.pixels[i : i + 4])


def domain_pos(tx, ty, w, h, z=0.0):
    return (((tx + 0.5) / w) * 2.0 - 1.0, ((ty + 0.5) / h) * 2.0 - 1.0, z)


def sd_sphere(p, r):
    return math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2) - r


def sd_box(p, b):
    q = [abs(p[i]) - b[i] for i in range(3)]
    outside = math.sqrt(sum(max(q[i], 0.0) ** 2 for i in range(3)))
    inside = min(max(q[0], max(q[1], q[2])), 0.0)
    return outside + inside


def sd_torus(p, R, r):
    q = (math.sqrt(p[0] ** 2 + p[2] ** 2) - R, p[1])
    return math.sqrt(q[0] ** 2 + q[1] ** 2) - r


def sd_capsule(p, h, r):
    """Centered capsule (endpoints ±h/2 on Y)."""
    half = h * 0.5
    y = min(max(p[1], -half), half)
    return math.sqrt(p[0] ** 2 + (p[1] - y) ** 2 + p[2] ** 2) - r


def available_input_names(node):
    return {s.identifier for s in node.inputs if s.enabled}


def test_distance_only():
    print("=== distance-only sockets ===")
    print("binary_path:", bpy.app.binary_path)
    tree = bpy.data.node_groups.new("DIST_ONLY", "ImageNodeTree")
    sdf = tree.nodes.new("ImageNodeSDFShape")
    out_ids = [s.identifier for s in sdf.outputs]
    check("Distance" in out_ids, "IP has Distance")
    check("Color" not in out_ids, "IP has no Color")
    check(len(sdf.outputs) == 1, f"IP single output ({len(sdf.outputs)})")
    check(sdf.outputs[0].type == "VALUE", "IP Distance is float")
    in_ids = {s.identifier for s in sdf.inputs}
    check("Viz Scale" not in in_ids, "IP no Viz Scale")
    check("Vector" in in_ids, "IP has Vector")
    bpy.data.node_groups.remove(tree)

    # Shader via geometry tree (poll allows GN)
    ng = bpy.data.node_groups.new("sh_dist", "GeometryNodeTree")
    sdf = ng.nodes.new("ShaderNodeSDFShape")
    out_ids = [s.identifier for s in sdf.outputs]
    check("Distance" in out_ids, "Shader has Distance")
    check("Color" not in out_ids, "Shader has no Color")
    check(len(sdf.outputs) == 1, "Shader single output")
    in_ids = {s.identifier for s in sdf.inputs}
    check("Viz Scale" not in in_ids, "Shader no Viz Scale")
    check("Scale" in in_ids, "Shader has Scale")
    bpy.data.node_groups.remove(ng)
    print("DISTANCE_ONLY_OK")


def test_param_map():
    print("=== param→oracle mapping (Shader MF) ===")
    # Drive registered Shader node multi-function via GN Store Named Attribute.

    def eval_dist(shape, pos, set_inputs):
        mesh = bpy.data.meshes.new("m")
        mesh.from_pydata([pos], [], [])
        obj = bpy.data.objects.new("o", mesh)
        bpy.context.scene.collection.objects.link(obj)
        mod = obj.modifiers.new("GN", "NODES")
        ng = bpy.data.node_groups.new("gn", "GeometryNodeTree")
        mod.node_group = ng
        nodes, links = ng.nodes, ng.links
        n_in = nodes.new("NodeGroupInput")
        n_out = nodes.new("NodeGroupOutput")
        ng.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
        ng.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
        pos_n = nodes.new("GeometryNodeInputPosition")
        sdf = nodes.new("ShaderNodeSDFShape")
        sdf.shape_type = shape
        for name, val in set_inputs.items():
            # Dynamic hide may omit or disable sockets that do not apply.
            sock = None
            for s in sdf.inputs:
                if s.identifier == name:
                    sock = s
                    break
            if sock is None or not sock.enabled:
                # Inactive/omitted → cannot mis-drive Distance (that's the point).
                continue
            sock.default_value = val
        store = nodes.new("GeometryNodeStoreNamedAttribute")
        store.data_type = "FLOAT"
        store.domain = "POINT"
        store.inputs["Name"].default_value = "dist"
        links.new(n_in.outputs["Geometry"], store.inputs["Geometry"])
        links.new(pos_n.outputs["Position"], sdf.inputs["Vector"])
        links.new(sdf.outputs["Distance"], store.inputs["Value"])
        links.new(store.outputs["Geometry"], n_out.inputs["Geometry"])
        deps = bpy.context.evaluated_depsgraph_get()
        me = obj.evaluated_get(deps).to_mesh()
        val = float(me.attributes["dist"].data[0].value)
        obj.evaluated_get(deps).to_mesh_clear()
        bpy.data.objects.remove(obj, do_unlink=True)
        bpy.data.meshes.remove(mesh)
        bpy.data.node_groups.remove(ng)
        return val

    # Sphere: only Radius
    p = (2.0, 0.0, 0.0)
    d = eval_dist("SPHERE", p, {"Radius": 1.0, "Scale": 1.0})
    near(d, sd_sphere(p, 1.0), 1e-4, "sphere r=1 at (2,0,0)")

    # Changing Size must not affect Sphere (inactive or ignored)
    d2 = eval_dist("SPHERE", p, {"Radius": 1.0, "Scale": 1.0, "Size": (9.0, 9.0, 9.0)})
    near(d2, d, 1e-4, "sphere ignores Size")

    # Box: only Size
    p0 = (1.0, 0.0, 0.0)
    box_size = (0.3, 0.4, 0.5)
    d = eval_dist("BOX", p0, {"Size": box_size, "Scale": 1.0, "Radius": 99.0})
    near(d, sd_box(p0, box_size), 1e-3, "box size oracle")
    d_wrong = eval_dist("BOX", p0, {"Size": box_size, "Scale": 1.0, "Radius": 0.01})
    near(d_wrong, d, 1e-4, "box ignores Radius")

    # Torus: Radius + Minor Radius
    p_t = (1.5, 0.0, 0.0)
    d = eval_dist("TORUS", p_t, {"Radius": 1.0, "Minor Radius": 0.2, "Scale": 1.0})
    near(d, sd_torus(p_t, 1.0, 0.2), 1e-3, "torus major/minor oracle")
    d_m = eval_dist("TORUS", p_t, {"Radius": 1.0, "Minor Radius": 0.05, "Scale": 1.0})
    check(abs(d_m - d) > 0.05, f"torus minor radius changes DE ({d} vs {d_m})")

    # Capsule: Radius + Height
    p_c = (0.0, 2.0, 0.0)
    d = eval_dist("CAPSULE", p_c, {"Radius": 0.25, "Height": 1.0, "Scale": 1.0})
    near(d, sd_capsule(p_c, 1.0, 0.25), 1e-3, "capsule radius/height oracle")
    d_h = eval_dist("CAPSULE", p_c, {"Radius": 0.25, "Height": 0.2, "Scale": 1.0})
    check(abs(d_h - d) > 0.1, f"capsule height changes DE ({d} vs {d_h})")

    # Dynamic hide: only relevant sockets enabled / present per shape
    def enabled_ids(node):
        return {s.identifier for s in node.inputs if getattr(s, "enabled", True)}

    tree = bpy.data.node_groups.new("avail", "ImageNodeTree")
    sdf = tree.nodes.new("ImageNodeSDFShape")
    sdf.shape_type = "SPHERE"
    en = enabled_ids(sdf)
    print("Sphere enabled sockets:", sorted(en))
    check("Radius" in en and "Vector" in en, "Sphere: Radius+Vector")
    check("Size" not in en and "Height" not in en and "Minor Radius" not in en, "Sphere: no Size/Height/Minor")
    sdf.shape_type = "BOX"
    en = enabled_ids(sdf)
    print("Box enabled sockets:", sorted(en))
    check("Size" in en, "Box: Size")
    check("Radius" not in en, "Box: no Radius")
    sdf.shape_type = "TORUS"
    en = enabled_ids(sdf)
    print("Torus enabled sockets:", sorted(en))
    check("Radius" in en and "Minor Radius" in en, "Torus: R+minor")
    check("Size" not in en, "Torus: no Size")
    sdf.shape_type = "CAPSULE"
    en = enabled_ids(sdf)
    print("Capsule enabled sockets:", sorted(en))
    check("Radius" in en and "Height" in en, "Capsule: R+H")
    check("Size" not in en, "Capsule: no Size")
    all_ids = {s.identifier for s in sdf.inputs}
    check("Thickness" not in all_ids, "no legacy Thickness")
    check("Viz Scale" not in all_ids, "no Viz Scale")
    bpy.data.node_groups.remove(tree)
    print("PARAM_MAP_OK")


def test_ip_distance():
    print("=== Image Process Distance cook ===")
    tree = bpy.data.node_groups.new("IP_D", "ImageNodeTree")
    sdf = tree.nodes.new("ImageNodeSDFShape")
    # Distance → Map Range (-1..1 → 0..1) → Color Ramp → Viewer
    mr = tree.nodes.new("ShaderNodeMapRange")
    ramp = tree.nodes.new("ShaderNodeValToRGB")
    viewer = tree.nodes.new("ImageNodeViewer")
    sdf.shape_type = "SPHERE"
    sdf.inputs["Radius"].default_value = 0.5
    mr.inputs["From Min"].default_value = -1.0
    mr.inputs["From Max"].default_value = 1.0
    mr.inputs["To Min"].default_value = 0.0
    mr.inputs["To Max"].default_value = 1.0
    tree.links.new(sdf.outputs["Distance"], mr.inputs["Value"])
    tree.links.new(mr.outputs["Result"], ramp.inputs["Fac"])
    tree.links.new(ramp.outputs["Color"], viewer.inputs["Image"])

    ok, gpu = tree.evaluate()
    check(ok, f"evaluate ok gpu={gpu}")
    w, h, corner = sample_viewer(0, 0)
    cx, cy = w // 2, h // 2
    _, _, center = sample_viewer(cx, cy)
    print(f"size {w}x{h} center={center} corner={corner}")
    # Interior d more negative → lower mapped Fac than exterior corner
    delta = abs(center[0] - corner[0])
    check(delta > 0.05, f"center vs corner mapped distance delta ({delta})")
    check(center[0] < corner[0], "center darker (more interior) than corner")

    # Oracle: domain center should be inside sphere r=0.5
    pos_c = domain_pos(cx, cy, w, h)
    exp = sd_sphere(pos_c, 0.5)
    check(exp < 0.0, f"oracle center inside d={exp}")

    # Radius change: sample mid-radius ring (not always-interior center)
    mx = int(w * 0.35)
    sdf.inputs["Radius"].default_value = 0.15
    tree.evaluate()
    _, _, c_small = sample_viewer(mx, cy)
    sdf.inputs["Radius"].default_value = 0.9
    tree.evaluate()
    _, _, c_big = sample_viewer(mx, cy)
    print(f"radius mid sample small={c_small} big={c_big}")
    check(
        abs(c_small[0] - c_big[0]) > 0.02,
        f"radius change alters mid sample ({c_small[0]} vs {c_big[0]})",
    )

    # Box vs Sphere at mid
    sdf.shape_type = "SPHERE"
    sdf.inputs["Radius"].default_value = 0.5
    tree.evaluate()
    _, _, sph = sample_viewer(mx, cy)
    sdf.shape_type = "BOX"
    sdf.inputs["Size"].default_value = (0.35, 0.25, 0.2)
    tree.evaluate()
    _, _, box = sample_viewer(mx, cy)
    check(
        abs(sph[0] - box[0]) > 0.005 or abs(sph[1] - box[1]) > 0.005,
        f"BOX vs SPHERE field differs ({sph} vs {box})",
    )

    bpy.data.node_groups.remove(tree)
    print("IP_DISTANCE_OK")


def test_shader_distance():
    print("=== Shader MF Distance ===")
    mesh = bpy.data.meshes.new("m")
    mesh.from_pydata([(2.0, 0.0, 0.0), (0.0, 0.0, 0.0)], [], [])
    obj = bpy.data.objects.new("o", mesh)
    bpy.context.scene.collection.objects.link(obj)
    mod = obj.modifiers.new("GN", "NODES")
    ng = bpy.data.node_groups.new("gn", "GeometryNodeTree")
    mod.node_group = ng
    nodes, links = ng.nodes, ng.links
    n_in = nodes.new("NodeGroupInput")
    n_out = nodes.new("NodeGroupOutput")
    ng.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    ng.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    pos = nodes.new("GeometryNodeInputPosition")
    sdf = nodes.new("ShaderNodeSDFShape")
    check(sdf.outputs[0].identifier == "Distance", "first out Distance")
    check("Scale" in {s.identifier for s in sdf.inputs}, "has Scale")
    sdf.shape_type = "SPHERE"
    sdf.inputs["Radius"].default_value = 1.0
    sdf.inputs["Scale"].default_value = 1.0
    store = nodes.new("GeometryNodeStoreNamedAttribute")
    store.data_type = "FLOAT"
    store.domain = "POINT"
    store.inputs["Name"].default_value = "dist"
    links.new(n_in.outputs["Geometry"], store.inputs["Geometry"])
    links.new(pos.outputs["Position"], sdf.inputs["Vector"])
    links.new(sdf.outputs["Distance"], store.inputs["Value"])
    links.new(store.outputs["Geometry"], n_out.inputs["Geometry"])
    deps = bpy.context.evaluated_depsgraph_get()
    me = obj.evaluated_get(deps).to_mesh()
    d0 = float(me.attributes["dist"].data[0].value)
    d1 = float(me.attributes["dist"].data[1].value)
    near(d0, 1.0, 1e-4, "shader MF sphere exterior (2,0,0) r=1")
    near(d1, -1.0, 1e-4, "shader MF sphere interior (0,0,0) r=1")
    check(d0 != d1, "non-constant Distance across points")
    # Scale multiplies position: scale=2 at (2,0,0) → p=(4,0,0) → d=3
    sdf.inputs["Scale"].default_value = 2.0
    deps = bpy.context.evaluated_depsgraph_get()
    me = obj.evaluated_get(deps).to_mesh()
    d_s = float(me.attributes["dist"].data[0].value)
    near(d_s, 3.0, 1e-3, "Scale multiplies position")
    obj.evaluated_get(deps).to_mesh_clear()
    bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.meshes.remove(mesh)
    bpy.data.node_groups.remove(ng)
    print("SHADER_DISTANCE_OK")


def main():
    print("=== SDF Distance-only + param map verification ===")
    test_distance_only()
    test_param_map()
    test_ip_distance()
    test_shader_distance()
    print("ALL_VERIFICATION_OK")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        traceback.print_exc()
        print("VERIFICATION_FAIL:", e)
        sys.exit(1)
