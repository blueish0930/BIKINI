# Verify Box2D stack demo on shipped Geometry Nodes (not a reimplementation).
# Usage:
#   blender.exe --background --factory-startup --python verify_box2d_stack_demo.py -- [demo.blend]
#
# Exit 0 only if gating checks pass.

import sys
from pathlib import Path

import bpy

DEFAULT_DEMO = Path(r"E:\aaaaaaaaaaa\Box2D_Stack_Demo.blend")
# Optional second arg after --
argv = sys.argv
demo_path = DEFAULT_DEMO
if "--" in argv:
    extra = argv[argv.index("--") + 1 :]
    if extra:
        demo_path = Path(extra[0])


def fail(msg: str) -> None:
    print("VERIFY_FAIL:", msg)
    raise SystemExit(1)


def main() -> None:
    if not demo_path.is_file():
        fail(f"demo missing: {demo_path}")

    bpy.ops.wm.open_mainfile(filepath=str(demo_path))
    print("OPENED", demo_path)

    # Structural checks: shipped node types present in tree
    needed = {
        "GeometryNodeBoxEngineSolver",
        "GeometryNodeBoxEngineSetRigidBody",
        "GeometryNodeSimulationInput",
        "GeometryNodeSimulationOutput",
    }
    found = set()
    engine_box2d = False
    for ng in bpy.data.node_groups:
        if ng.bl_idname != "GeometryNodeTree":
            continue
        for n in ng.nodes:
            found.add(n.bl_idname)
            if n.bl_idname == "GeometryNodeBoxEngineSolver":
                eng = n.inputs.get("Engine")
                if eng is not None:
                    print("Solver Engine=", eng.default_value)
                    if str(eng.default_value) == "Box2D":
                        engine_box2d = True
    missing = needed - found
    if missing:
        fail(f"missing node types in blend: {missing}")
    if not engine_box2d:
        fail("no BoxEngineSolver with Engine=Box2D")

    # Find object with GN modifier using the demo group
    obj = None
    for o in bpy.data.objects:
        for m in o.modifiers:
            if m.type == "NODES" and m.node_group is not None:
                obj = o
                break
        if obj:
            break
    if obj is None:
        fail("no Geometry Nodes object in demo")

    scene = bpy.context.scene
    scene.frame_start = 1
    scene.frame_set(1)

    def eval_ys():
        deps = bpy.context.evaluated_depsgraph_get()
        ev = obj.evaluated_get(deps)
        me = ev.to_mesh()
        try:
            ys = [v.co.y for v in me.vertices]
            return ys, len(me.vertices)
        finally:
            ev.to_mesh_clear()

    ys0, n0 = eval_ys()
    if n0 < 8:
        fail(f"too few verts at frame 1: {n0}")
    y_max0 = max(ys0)
    y_min0 = min(ys0)
    print(f"F1 verts={n0} y=[{y_min0:.4f},{y_max0:.4f}]")

    # Initial drop height: dynamic boxes start with centers y >= ~1.2
    if y_max0 < 2.0:
        fail(f"initial max y too low (expected stack height): {y_max0}")

    # Play simulation
    end = max(scene.frame_end, 180)
    scene.frame_end = end
    for f in range(1, end + 1):
        scene.frame_set(f)

    ys1, n1 = eval_ys()
    y_max1 = max(ys1)
    y_min1 = min(ys1)
    print(f"F{end} verts={n1} y=[{y_min1:.4f},{y_max1:.4f}]")

    # 1) Max height dropped substantially
    if y_max1 >= y_max0 - 0.5:
        fail(f"stack did not fall enough: max y {y_max0:.3f} -> {y_max1:.3f}")

    # 2) Ground present (min y well below 0) and dynamics not sunk through
    # Ground bottom ~ -2; dynamic bottoms should stay >= -0.15 after rest
    dyn = [y for y in ys1 if y > -0.05]
    if not dyn:
        fail("no vertices above ground plane after sim")
    dyn_min = min(dyn)
    dyn_max = max(dyn)
    print(f"dyn_y=[{dyn_min:.4f},{dyn_max:.4f}]")
    if dyn_min < -0.2:
        fail(f"sustained sink-through: dyn min y={dyn_min:.4f}")

    # 3) Not collapsed to a single point; expect vertical extent of a few boxes
    if (dyn_max - dyn_min) < 0.5:
        fail(f"stack collapsed too flat: dyn span {dyn_max - dyn_min:.4f}")

    # 4) Lowest dynamic material near ground contact (resting), not floating high
    # Bottom of resting box ~ 0; allow some bounce residual
    if dyn_min > 1.5:
        fail(f"boxes still floating high: dyn min y={dyn_min:.4f}")

    print("VERIFY_OK", demo_path)
    raise SystemExit(0)


if __name__ == "__main__":
    main()
