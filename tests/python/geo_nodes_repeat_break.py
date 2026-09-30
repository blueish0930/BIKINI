"""Background eval: Repeat Zone Break early-exit vs full loop.

Builds a Geometry Nodes tree with a Repeat Zone carrying an Int:
  - Iterations = N (>=3)
  - Body: value = value + 1; Break = (Iteration >= K)
  - Expect zone Int output == K+1 when break-at-K (iteration K sets break true
    after that step's increment; Iteration is 0-based).
  - With Break unlinked / always false, expect N steps of increment.

Run under blender --background --python.
"""

import bpy
from mathutils import Vector

N = 5
K = 2  # break when Iteration >= 2  => after iterations 0,1,2 => value 3


def clear_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def make_object_with_geo_nodes(name: str) -> bpy.types.Object:
    mesh = bpy.data.meshes.new(name + "_mesh")
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    mod = obj.modifiers.new(name="GN", type="NODES")
    ng = bpy.data.node_groups.new(name + "_ng", "GeometryNodeTree")
    mod.node_group = ng
    return obj, ng


def add_socket(tree, in_out, socket_type, name, identifier=None):
    """Create interface socket on the group (Blender 4+ API)."""
    if hasattr(tree, "interface"):
        sock = tree.interface.new_socket(
            name=name, in_out=in_out, socket_type=socket_type
        )
        return sock
    # Fallback older API
    if in_out == "INPUT":
        tree.inputs.new(socket_type, name)
        return tree.inputs[-1]
    tree.outputs.new(socket_type, name)
    return tree.outputs[-1]


def build_repeat_break_tree(ng, iterations, break_k, link_break: bool):
    """
    Group: Geometry in/out passthrough; Int item in Repeat Zone.

    Repeat Input:
      Iterations = iterations
      Item Int (initial 0)
    Body:
      Math Add: Iteration_item + 1
      Compare: Iteration >= break_k  -> Break (if link_break)
    """
    ng.nodes.clear()
    # Group interface
    add_socket(ng, "INPUT", "NodeSocketGeometry", "Geometry")
    add_socket(ng, "OUTPUT", "NodeSocketGeometry", "Geometry")
    add_socket(ng, "OUTPUT", "NodeSocketInt", "Value")

    nodes = ng.nodes
    links = ng.links

    group_in = nodes.new("NodeGroupInput")
    group_in.location = (-600, 0)
    group_out = nodes.new("NodeGroupOutput")
    group_out.location = (800, 0)

    rep_in = nodes.new("GeometryNodeRepeatInput")
    rep_in.location = (-200, 0)
    rep_out = nodes.new("GeometryNodeRepeatOutput")
    rep_out.location = (400, 0)

    # Wire zone pair
    rep_in.pair_with_output(rep_out)

    # Configure items: only Int named Value (clear default Geometry for simplicity)
    # Storage is on output node
    # Use items API if available
    items = rep_out.repeat_items
    # Remove default Geometry if present and add Int
    while len(items) > 0:
        items.remove(items[0])
    item = items.new("INT", "Value")

    # Update sockets after items change
    # Force update
    ng.interface_update(bpy.context)

    # Set iterations
    # Find Iterations socket on rep_in
    # Default default_value
    if "Iterations" in rep_in.inputs:
        rep_in.inputs["Iterations"].default_value = iterations
    else:
        # by index: first input is Iterations
        rep_in.inputs[0].default_value = iterations

    # Initial Value = 0
    value_in = None
    for s in rep_in.inputs:
        if s.name == "Value" or (s.type == "INT" and s.name != "Iterations"):
            value_in = s
            break
    if value_in is None:
        # After items, first non-iterations int
        for s in rep_in.inputs:
            if s.type == "VALUE" or s.bl_idname == "NodeSocketInt":
                if s.name != "Iterations":
                    value_in = s
                    break
    assert value_in is not None, f"No Value input on Repeat Input: {[s.name for s in rep_in.inputs]}"
    value_in.default_value = 0

    # Body: Add 1 to Value
    add = nodes.new("ShaderNodeMath")
    add.operation = "ADD"
    add.location = (100, 100)
    add.inputs[1].default_value = 1.0

    # Get body sockets
    # rep_in outputs: Iteration, Value, ...
    # rep_out inputs: Break, Value, ...
    rep_in_outs = {s.name: s for s in rep_in.outputs}
    rep_out_ins = {s.name: s for s in rep_out.inputs}
    rep_out_outs = {s.name: s for s in rep_out.outputs}

    print("rep_in outputs:", list(rep_in_outs.keys()))
    print("rep_out inputs:", list(rep_out_ins.keys()))
    print("rep_out outputs:", list(rep_out_outs.keys()))

    assert "Break" in rep_out_ins, "Break socket missing on Repeat Output!"
    assert "Value" in rep_in_outs or any(
        s.type == "INT" or s.bl_idname == "NodeSocketInt" for s in rep_in.outputs if s.name != "Iteration"
    ), "Value item missing"

    value_out_from_in = rep_in_outs.get("Value")
    if value_out_from_in is None:
        for s in rep_in.outputs:
            if s.name not in ("Iteration",) and (s.type == "INT" or "Int" in s.bl_idname):
                value_out_from_in = s
                break

    value_in_to_out = rep_out_ins.get("Value")
    if value_in_to_out is None:
        for s in rep_out.inputs:
            if s.name not in ("Break",) and (s.type == "INT" or "Int" in s.bl_idname):
                value_in_to_out = s
                break

    value_out_from_out = rep_out_outs.get("Value")
    if value_out_from_out is None:
        for s in rep_out.outputs:
            if s.type == "INT" or "Int" in s.bl_idname:
                value_out_from_out = s
                break

    # Value -> Add
    links.new(value_out_from_in, add.inputs[0])
    # Add result -> Value on output
    # Math outputs float; need Int conversion
    # Use FunctionNodeIntegerMath if available, else Float to Int
    int_add = None
    try:
        int_add = nodes.new("FunctionNodeIntegerMath")
        int_add.operation = "ADD"
        int_add.location = (100, 100)
        int_add.inputs[1].default_value = 1
        nodes.remove(add)
        add = int_add
        links.new(value_out_from_in, add.inputs[0])
        links.new(add.outputs[0], value_in_to_out)
    except Exception:
        # Float path with Convert
        convert = nodes.new("FunctionNodeFloatToInt")
        convert.location = (250, 100)
        links.new(add.outputs[0], convert.inputs[0])
        links.new(convert.outputs[0], value_in_to_out)

    # Break: Iteration >= K
    if link_break:
        # Compare Iteration >= K
        # Use FunctionNodeCompare or Math greater
        try:
            cmp = nodes.new("FunctionNodeCompare")
            cmp.data_type = "INT"
            cmp.operation = "GREATER_EQUAL"
            cmp.location = (100, -100)
            links.new(rep_in_outs["Iteration"], cmp.inputs[2])  # A for INT often index 2
            # Set B
            # Find B socket
            for s in cmp.inputs:
                if s.name == "B" and s.type == "INT":
                    s.default_value = break_k
                    break
            else:
                # try index
                if len(cmp.inputs) > 3:
                    cmp.inputs[3].default_value = break_k
            links.new(cmp.outputs["Result"], rep_out_ins["Break"])
        except Exception as e:
            print("Compare setup fallback:", e)
            # Boolean Math with static true after K via Integer compare differently
            # Use ShaderNodeMath GREATER_THAN
            m = nodes.new("ShaderNodeMath")
            m.operation = "GREATER_THAN"
            m.location = (100, -100)
            m.inputs[1].default_value = break_k - 0.5  # >= K when float
            links.new(rep_in_outs["Iteration"], m.inputs[0])
            # Need bool - Compare floats
            # Map to bool socket via greater: result float used as bool? Use FunctionNodeBooleanMath
            # Actually Bool sockets may accept float via conversion
            links.new(m.outputs[0], rep_out_ins["Break"])

    # Geometry passthrough outside zone (not through zone)
    links.new(group_in.outputs["Geometry"], group_out.inputs["Geometry"])

    # Zone Value output -> group Value
    # Find group output Value
    go_value = None
    for s in group_out.inputs:
        if s.name == "Value":
            go_value = s
            break
    assert go_value is not None
    links.new(value_out_from_out, go_value)

    return ng


def evaluate_value(obj) -> int:
    """Depsgraph evaluate and read the modifier's evaluated value via attribute.

    Simpler approach: use a Viewer is hard in background. Instead store Value
    as a scene property by using geometry attribute on a single point.

    Easier path: use bpy.ops object evaluated and read interface socket via
    geometry nodes log — not available. Use attribute on mesh:

    Alternative: put the Int into Store Named Attribute on a grid point and read it.
    For simplicity we restructure to output via attribute.
    """
    # Use depsgraph and modifier evaluation through mesh attribute
    # Rebuild approach: Capture attribute
    depsgraph = bpy.context.evaluated_depsgraph_get()
    obj_eval = obj.evaluated_get(depsgraph)
    # Without geometry output of the value, we can't read Int socket directly.
    # Use a hack: custom node tree evaluation via modifiers is geometry-only.
    # Store value as face count of a mesh created from points = value.
    return None


def build_and_run(iterations, break_k, link_break: bool) -> int:
    """Build tree that converts the repeated Int into mesh point count for reading."""
    clear_scene()
    obj, ng = make_object_with_geo_nodes("TestBreak")

    ng.nodes.clear()
    # Interface
    # no geometry input needed
    if hasattr(ng, "interface"):
        # remove default if any
        pass
    add_socket(ng, "OUTPUT", "NodeSocketGeometry", "Geometry")

    nodes = ng.nodes
    links = ng.links

    group_out = nodes.new("NodeGroupOutput")
    group_out.location = (1000, 0)

    rep_in = nodes.new("GeometryNodeRepeatInput")
    rep_in.location = (-200, 0)
    rep_out = nodes.new("GeometryNodeRepeatOutput")
    rep_out.location = (400, 0)
    rep_in.pair_with_output(rep_out)

    items = rep_out.repeat_items
    while len(items) > 0:
        items.remove(items[0])
    items.new("INT", "Value")

    # Iterations
    rep_in.inputs[0].default_value = iterations

    # Find sockets after update
    bpy.context.view_layer.update()

    print("=== sockets after pair ===")
    print("rep_in in:", [(s.name, s.bl_idname) for s in rep_in.inputs])
    print("rep_in out:", [(s.name, s.bl_idname) for s in rep_in.outputs])
    print("rep_out in:", [(s.name, s.bl_idname) for s in rep_out.inputs])
    print("rep_out out:", [(s.name, s.bl_idname) for s in rep_out.outputs])

    # Initial Value
    for s in rep_in.inputs:
        if s.name == "Value":
            s.default_value = 0
            break

    # Integer Math ADD 1
    iadd = nodes.new("FunctionNodeIntegerMath")
    iadd.operation = "ADD"
    iadd.location = (100, 80)
    iadd.inputs[1].default_value = 1

    # Link Value through add
    links.new(rep_in.outputs["Value"], iadd.inputs[0])
    links.new(iadd.outputs[0], rep_out.inputs["Value"])

    # Break
    if link_break:
        cmp = nodes.new("FunctionNodeCompare")
        cmp.data_type = "INT"
        cmp.operation = "GREATER_EQUAL"
        cmp.location = (100, -120)
        links.new(rep_in.outputs["Iteration"], cmp.inputs["A"])
        cmp.inputs["B"].default_value = break_k
        links.new(cmp.outputs["Result"], rep_out.inputs["Break"])
        print(f"Break linked: Iteration >= {break_k}")
    else:
        print("Break unlinked (default false)")

    # Points with Count = zone Value, then Points→Vertices so to_mesh() counts them.
    points = nodes.new("GeometryNodePoints")
    points.location = (600, 80)
    links.new(rep_out.outputs["Value"], points.inputs["Count"])
    p2v = nodes.new("GeometryNodePointsToVertices")
    p2v.location = (780, 80)
    links.new(points.outputs["Geometry"], p2v.inputs["Points"])
    links.new(p2v.outputs["Mesh"], group_out.inputs["Geometry"])

    # Assign modifier
    obj.modifiers["GN"].node_group = ng

    # Evaluate: number of points == carried Int value after the loop
    depsgraph = bpy.context.evaluated_depsgraph_get()
    obj_eval = obj.evaluated_get(depsgraph)
    mesh = obj_eval.to_mesh()
    nverts = len(mesh.vertices)
    obj_eval.to_mesh_clear()
    print(
        f"RESULT iterations={iterations} break_k={break_k} link_break={link_break} "
        f"=> points={nverts}"
    )
    return nverts


def main():
    print("=== Repeat Zone Break verification ===")
    # Structural check: declare has Break
    # Runtime:
    # break at K=2, N=5: iterations 0,1,2 run; after iter 2 Break true; value=3
    v_break = build_and_run(N, K, link_break=True)
    # full loop N=5: value=5
    v_full = build_and_run(N, K, link_break=False)

    expected_break = K + 1  # 0..K inclusive = K+1 increments
    expected_full = N

    print(f"break_at_K: got={v_break} expected={expected_break}")
    print(f"full_loop:  got={v_full} expected={expected_full}")

    ok = (v_break == expected_break) and (v_full == expected_full)
    if ok:
        print("PASS")
    else:
        print("FAIL")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
