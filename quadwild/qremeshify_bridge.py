# SPDX-License-Identifier: GPL-3.0-or-later
"""
Geometry Nodes bridge: QRemeshify-only remesh/field/layout.

Stages:
  remesh — full operator pipeline (default)
  field  — stop after remeshAndField; copy mesh_rem.obj + .rosy
  layout — stop after trace; copy mesh_rem_p0.obj + .feature

Called by mesh_quadwild.cc:
  blender -b --python qremeshify_bridge.py -- --stage remesh --input ... --output ...
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import sys
import traceback

import bmesh
import bpy
import mathutils

_QUADWILD_DIR = os.path.dirname(os.path.abspath(__file__))
if _QUADWILD_DIR not in sys.path:
    sys.path.insert(0, _QUADWILD_DIR)

from QRemeshify.lib import QWException, Quadwild  # noqa: E402
from QRemeshify.util import exporter  # noqa: E402


def _parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="QRemeshify bridge for Geometry Nodes")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.add_argument(
        "--stage",
        choices=("remesh", "field", "layout"),
        default="remesh",
        help="Pipeline stop stage",
    )
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--alpha", type=float, default=0.005)
    p.add_argument("--sharp-angle", type=float, default=35.0)
    p.add_argument("--remesh", type=int, default=1)
    p.add_argument("--smooth", type=int, default=1)
    p.add_argument("--enable-sharp", type=int, default=1)
    p.add_argument("--rot-scale-matrix", type=float, nargs=16, default=None)
    p.add_argument("--ilp-method", default="LEASTSQUARES")
    p.add_argument("--time-limit", type=int, default=200)
    p.add_argument("--gap-limit", type=float, default=0.0)
    p.add_argument("--minimum-gap", type=float, default=0.4)
    p.add_argument("--isometry", type=int, default=1)
    p.add_argument("--regularity-quads", type=int, default=1)
    p.add_argument("--regularity-non-quads", type=int, default=1)
    p.add_argument("--regularity-non-quads-weight", type=float, default=0.9)
    p.add_argument("--align-singularities", type=int, default=1)
    p.add_argument("--align-singularities-weight", type=float, default=0.1)
    p.add_argument("--repeat-losing-iter", type=int, default=1)
    p.add_argument("--repeat-losing-quads", type=int, default=0)
    p.add_argument("--repeat-losing-non-quads", type=int, default=0)
    p.add_argument("--repeat-losing-align", type=int, default=1)
    p.add_argument("--hard-parity", type=int, default=1)
    p.add_argument("--flow-config", default="SIMPLE")
    p.add_argument("--satsuma-config", default="DEFAULT")
    p.add_argument("--fixed-chart-clusters", type=int, default=0)
    return p.parse_args(argv)


def _load_obj_to_bmesh(path: str) -> bmesh.types.BMesh:
    verts: list[tuple[float, float, float]] = []
    faces: list[list[int]] = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if parts[0] == "v" and len(parts) >= 4:
                verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif parts[0] == "f" and len(parts) >= 4:
                ids: list[int] = []
                for tok in parts[1:]:
                    v = tok.split("/")[0]
                    idx = int(v)
                    if idx < 0:
                        idx = len(verts) + idx + 1
                    ids.append(idx - 1)
                if len(ids) >= 3:
                    faces.append(ids)

    bm = bmesh.new()
    bm_verts = [bm.verts.new(co) for co in verts]
    bm.verts.ensure_lookup_table()
    for face in faces:
        try:
            bm.faces.new([bm_verts[i] for i in face])
        except ValueError:
            pass
    bm.faces.ensure_lookup_table()
    bm.normal_update()
    return bm


def _prep_bm(bm: bmesh.types.BMesh, args: argparse.Namespace) -> None:
    """Sharp mark + triangulate — same as QRemeshify operator.py."""
    enable_sharp = bool(args.enable_sharp) and args.sharp_angle >= 0
    sharp_angle = float(args.sharp_angle)

    if args.rot_scale_matrix is not None:
        m = mathutils.Matrix(
            (
                args.rot_scale_matrix[0:4],
                args.rot_scale_matrix[4:8],
                args.rot_scale_matrix[8:12],
                args.rot_scale_matrix[12:16],
            )
        )
        bmesh.ops.transform(bm, matrix=m, verts=bm.verts)

    if enable_sharp:
        face_set_data_layer = bm.faces.layers.int.get(".sculpt_face_set")
        bm.edges.ensure_lookup_table()
        for edge in bm.edges:
            is_sharp = math.degrees(edge.calc_face_angle(0)) > sharp_angle
            is_material_boundary = (
                len(edge.link_faces) > 1
                and edge.link_faces[0].material_index != edge.link_faces[1].material_index
            )
            is_face_set_boundary = (
                face_set_data_layer is not None
                and len(edge.link_faces) > 1
                and edge.link_faces[0][face_set_data_layer]
                != edge.link_faces[1][face_set_data_layer]
            )
            if is_sharp or edge.is_boundary or edge.seam or is_material_boundary or is_face_set_boundary:
                edge.smooth = False

    bmesh.ops.triangulate(bm, faces=bm.faces, quad_method="SHORT_EDGE", ngon_method="BEAUTY")


def _write_prep_config(path: str, do_remesh: bool, sharp_angle: float) -> None:
    with open(path, "w") as f:
        f.write(f"do_remesh {1 if do_remesh else 0}\n")
        f.write(f"sharp_feature_thr {sharp_angle:.6g}\n")
        f.write("alpha 0.01\n")
        f.write("scaleFact 1\n")


def _run_quadwild_exe(mesh_path: str, do_remesh: bool, sharp_angle: float) -> None:
    """Run quadwild.exe mode 2 (remesh+field+trace). Produces all sidecar files."""
    quadwild_dir = os.path.dirname(os.path.abspath(__file__))
    exe = os.path.join(quadwild_dir, "quadwild.exe")
    if not os.path.isfile(exe):
        raise QWException(f"quadwild.exe not found at {exe}")

    config_dir = os.path.join(quadwild_dir, "config", "prep_config")
    prep_config = os.path.join(os.path.dirname(mesh_path), "prep_setup.txt")
    _write_prep_config(prep_config, do_remesh, sharp_angle)

    import subprocess
    work_dir = os.path.dirname(os.path.abspath(mesh_path))
    result = subprocess.run(
        [exe, mesh_path, "2", prep_config],
        cwd=work_dir,
        capture_output=True,
        text=True,
        timeout=600,
    )
    if result.returncode != 0:
        raise QWException(
            f"quadwild.exe failed (exit {result.returncode}):\n"
            f"{result.stderr[-2000:]}\n{result.stdout[-2000:]}"
        )


def _run_pipeline(bm: bmesh.types.BMesh, args: argparse.Namespace, work_mesh_path: str) -> str:
    enable_sharp = bool(args.enable_sharp) and args.sharp_angle >= 0
    sharp_angle = float(args.sharp_angle)
    stage = args.stage

    _prep_bm(bm, args)

    # Use quadwild.exe for field/layout stages (newer, produces .rosy/.patch/.corners).
    if stage in ("field", "layout"):
        print(f"[QW_BRIDGE] ENTER exe path stage={stage} mesh={work_mesh_path}", file=sys.stderr)
        exporter.export_mesh(bm, work_mesh_path)
        if enable_sharp:
            # quadwild.exe reads sharp from {mesh}_rem.sharp (same as qw.sharp_path)
            sharp_path = os.path.splitext(work_mesh_path)[0] + "_rem.sharp"
            exporter.export_sharp_features(bm, sharp_path, sharp_angle)

        _run_quadwild_exe(work_mesh_path, bool(args.remesh), sharp_angle)

        qw = Quadwild(work_mesh_path)  # for path names only

        if stage == "field":
            if not os.path.isfile(qw.remeshed_path):
                raise QWException(f"Field stage: missing {qw.remeshed_path}")
            shutil.copyfile(qw.remeshed_path, args.output)
            out_base, _ = os.path.splitext(args.output)
            if os.path.isfile(qw.field_path):
                shutil.copyfile(qw.field_path, out_base + ".rosy")
            return args.output

        # layout
        print(f"[QW_BRIDGE] layout: traced_path={qw.traced_path}", file=sys.stderr)
        print(f"[QW_BRIDGE] layout: traced_exists={os.path.isfile(qw.traced_path)}", file=sys.stderr)
        if not os.path.isfile(qw.traced_path):
            raise QWException(f"Layout stage: missing {qw.traced_path}")
        shutil.copyfile(qw.traced_path, args.output)
        out_base, _ = os.path.splitext(args.output)
        print(f"[QW_BRIDGE] layout: out_base={out_base}", file=sys.stderr)
        for tag, src_path in [
            (".rosy", qw.field_path),
            (".feature", os.path.splitext(qw.traced_path)[0] + ".feature"),
            (".corners", os.path.splitext(qw.traced_path)[0] + ".corners"),
            (".patch", os.path.splitext(qw.traced_path)[0] + ".patch"),
        ]:
            exists = os.path.isfile(src_path)
            size = os.path.getsize(src_path) if exists else 0
            print(f"[QW_BRIDGE] layout: sidecar {tag} exists={exists} size={size} src={src_path}", file=sys.stderr)
            if exists:
                shutil.copyfile(src_path, out_base + tag)
        print(f"[QW_BRIDGE] layout: DONE", file=sys.stderr)
        return args.output

    # --- remesh stage: use DLL path ---
    qw = Quadwild(work_mesh_path)
    try:
        exporter.export_mesh(bm, work_mesh_path)
        if enable_sharp:
            exporter.export_sharp_features(bm, qw.sharp_path, sharp_angle)

        qw.remeshAndField(
            remesh=bool(args.remesh),
            enableSharp=enable_sharp,
            sharpAngle=sharp_angle,
        )

        if stage == "field":
            if not os.path.isfile(qw.remeshed_path):
                raise QWException(f"Field stage: missing {qw.remeshed_path}")
            shutil.copyfile(qw.remeshed_path, args.output)
            # Sidecar .rosy next to output (same basename)
            out_base, _ = os.path.splitext(args.output)
            if os.path.isfile(qw.field_path):
                shutil.copyfile(qw.field_path, out_base + ".rosy")
            return args.output

        qw.trace()

        if stage == "layout":
            if not os.path.isfile(qw.traced_path):
                raise QWException(f"Layout stage: missing {qw.traced_path}")
            shutil.copyfile(qw.traced_path, args.output)
            out_base, _ = os.path.splitext(args.output)
            # Sidecar .rosy next to output (same basename)
            if os.path.isfile(qw.field_path):
                shutil.copyfile(qw.field_path, out_base + ".rosy")
            # Patch boundary edges for seam marking
            feat = os.path.splitext(qw.traced_path)[0] + ".feature"
            if os.path.isfile(feat):
                shutil.copyfile(feat, out_base + ".feature")
            # Patch corner vertices (closed polygon per patch)
            corners = os.path.splitext(qw.traced_path)[0] + ".corners"
            if os.path.isfile(corners):
                shutil.copyfile(corners, out_base + ".corners")
            # Patch ID per face
            patchf = os.path.splitext(qw.traced_path)[0] + ".patch"
            if os.path.isfile(patchf):
                shutil.copyfile(patchf, out_base + ".patch")
            return args.output

        # --- full remesh ---
        callback_time = [3.00, 5.000, 10.0, 20.0, 30.0, 60.0, 90.0, 120.0]
        callback_gap = [0.005, 0.02, 0.05, 0.10, 0.15, 0.20, 0.25, 0.3]

        qw.quadrangulate(
            bool(args.smooth),
            float(args.scale),
            int(args.fixed_chart_clusters),
            float(args.alpha),
            str(args.ilp_method),
            int(args.time_limit),
            float(args.gap_limit),
            float(args.minimum_gap),
            bool(args.isometry),
            bool(args.regularity_quads),
            bool(args.regularity_non_quads),
            float(args.regularity_non_quads_weight),
            bool(args.align_singularities),
            float(args.align_singularities_weight),
            bool(args.repeat_losing_iter),
            bool(args.repeat_losing_quads),
            bool(args.repeat_losing_non_quads),
            bool(args.repeat_losing_align),
            bool(args.hard_parity),
            str(args.flow_config),
            str(args.satsuma_config),
            callback_time,
            callback_gap,
        )

        final_path = qw.output_smoothed_path if args.smooth else qw.output_path
        if not os.path.isfile(final_path):
            alt = qw.output_path if args.smooth else qw.output_smoothed_path
            if os.path.isfile(alt):
                final_path = alt
            else:
                raise QWException(f"No output OBJ at {final_path}")
        shutil.copyfile(final_path, args.output)
        return args.output
    finally:
        del qw


def main() -> int:
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = argv[1:]

    args = _parse_args(argv)
    bpy.ops.wm.read_factory_settings(use_empty=True)

    work_dir = os.path.dirname(os.path.abspath(args.input))
    os.makedirs(work_dir, exist_ok=True)
    work_mesh = os.path.join(work_dir, "qremeshify_work.obj")

    bm = _load_obj_to_bmesh(args.input)
    try:
        if len(bm.faces) == 0:
            print("ERROR: input mesh has 0 faces", file=sys.stderr)
            return 2
        path = _run_pipeline(bm, args, work_mesh)
        print(f"QREMESHIFY_BRIDGE_OK stage={args.stage} {path}")
        return 0
    except Exception as e:
        traceback.print_exc()
        print(f"ERROR: {e!r}", file=sys.stderr)
        return 1
    finally:
        bm.free()


if __name__ == "__main__":
    sys.exit(main())
