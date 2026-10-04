# IMAGE_NODES_MVP verification script
# Pipeline: BlankImage -> BrightContrast -> Flip -> Viewer
# Pixel math: color (0.2,0.3,0.4,1), brightness=20, contrast=0 -> (0.4,0.5,0.6,1)
import sys
import bpy

def sample_viewer_pixel(x=0, y=0):
    img = bpy.data.images.get("Image Nodes Viewer")
    if img is None:
        raise RuntimeError("Image Nodes Viewer not found after evaluate()")
    w, h = img.size
    if w < 1 or h < 1:
        raise RuntimeError(f"Viewer size invalid: {w}x{h}")
    # Blender image.pixels are bottom-left origin, RGBA float
    i = (y * w + x) * 4
    px = list(img.pixels[i : i + 4])
    return w, h, px


def expected_bright_contrast(color, brightness, contrast):
    # Same algorithm as compositor / ImageNodeBrightContrast
    scaled_brightness = brightness / 100.0
    delta = contrast / 200.0
    if contrast > 0.0:
        multiplier = 1.0 - delta * 2.0
        multiplier = 1.0 / max(multiplier, 1e-7)
        offset = multiplier * (scaled_brightness - delta)
    else:
        delta = -delta
        multiplier = max(1.0 - delta * 2.0, 0.0)
        offset = multiplier * scaled_brightness + delta
    r = color[0] * multiplier + offset
    g = color[1] * multiplier + offset
    b = color[2] * multiplier + offset
    a = color[3]
    return (r, g, b, a)


def run_once(run_id):
    print(f"=== RUN {run_id} ===")
    print("binary_path:", bpy.app.binary_path)

    # Create distinct ImageNodeTree (not Compositor)
    tree = bpy.data.node_groups.new("ImageNodesTest", type="ImageNodeTree")
    assert tree.bl_idname == "ImageNodeTree", tree.bl_idname
    assert tree.bl_idname != "CompositorNodeTree"
    print("tree_type:", tree.bl_idname)
    print("tree_rna:", type(tree).__name__)

    blank = tree.nodes.new("ImageNodeBlankImage")
    bright = tree.nodes.new("ImageNodeBrightContrast")
    flip = tree.nodes.new("ImageNodeFlip")
    viewer = tree.nodes.new("ImageNodeViewer")
    print("nodes:", [n.bl_idname for n in tree.nodes])

    # Wire Blank -> Bright -> Flip -> Viewer
    tree.links.new(blank.outputs["Image"], bright.inputs["Image"])
    tree.links.new(bright.outputs["Image"], flip.inputs["Image"])
    tree.links.new(flip.outputs["Image"], viewer.inputs["Image"])

    color = (0.2, 0.3, 0.4, 1.0)
    brightness = 20.0
    contrast = 0.0
    blank.inputs["Color"].default_value = color
    blank.inputs["Size"].default_value = (4, 4)
    bright.inputs["Bright"].default_value = brightness
    bright.inputs["Contrast"].default_value = contrast
    flip.inputs["Flip X"].default_value = True
    flip.inputs["Flip Y"].default_value = False

    # Evaluate (GPU-primary)
    result, used_gpu = tree.evaluate()
    print("evaluate_result:", result, "used_gpu:", used_gpu)
    if not result:
        raise RuntimeError("evaluate() returned False")

    w, h, px = sample_viewer_pixel(0, 0)
    print(f"viewer_size: {w}x{h}")
    print(f"pixel_actual: {px}")

    exp = expected_bright_contrast(color, brightness, contrast)
    print(f"pixel_expected: {exp}")
    tol = 1e-4
    for a, e in zip(px, exp):
        if abs(a - e) > tol:
            raise RuntimeError(f"pixel mismatch actual={px} expected={exp}")

    # Sample another pixel (solid fill + flip still uniform)
    _, _, px2 = sample_viewer_pixel(3, 2)
    for a, e in zip(px2, exp):
        if abs(a - e) > tol:
            raise RuntimeError(f"pixel2 mismatch actual={px2} expected={exp}")

    print(f"RUN {run_id} OK used_gpu={used_gpu}")
    # cleanup for second run
    bpy.data.node_groups.remove(tree)
    return used_gpu


def main():
    used = []
    for i in (1, 2):
        used.append(run_once(i))
    print("PIPELINE_SUCCESS")
    print("used_gpu_runs:", used)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        import traceback
        traceback.print_exc()
        print("PIPELINE_FAIL:", e)
        sys.exit(1)
