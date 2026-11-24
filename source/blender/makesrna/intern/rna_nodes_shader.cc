/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup RNA
 */

#include "BLT_translation.hh"

#include "BKE_node.hh"

#include "RNA_enum_types.hh"

#include "UI_interface_c.hh"

#include "rna_internal.hh"
#include "rna_nodetree_common.hh"

namespace blender {

const EnumPropertyItem rna_enum_mapping_type_items[] = {
    {NODE_MAPPING_TYPE_POINT, "POINT", 0, "Point", "Transform a point"},
    {NODE_MAPPING_TYPE_TEXTURE,
     "TEXTURE",
     0,
     "Texture",
     "Transform a texture by inverse mapping the texture coordinate"},
    {NODE_MAPPING_TYPE_VECTOR,
     "VECTOR",
     0,
     "Vector",
     "Transform a direction vector (Location is ignored)"},
    {NODE_MAPPING_TYPE_NORMAL,
     "NORMAL",
     0,
     "Normal",
     "Transform a unit normal vector (Location is ignored)"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_vector_rotate_type_items[] = {
    {NODE_VECTOR_ROTATE_TYPE_AXIS,
     "AXIS_ANGLE",
     0,
     "Axis Angle",
     "Rotate a point using axis angle"},
    {NODE_VECTOR_ROTATE_TYPE_AXIS_X, "X_AXIS", 0, "X Axis", "Rotate a point using X axis"},
    {NODE_VECTOR_ROTATE_TYPE_AXIS_Y, "Y_AXIS", 0, "Y Axis", "Rotate a point using Y axis"},
    {NODE_VECTOR_ROTATE_TYPE_AXIS_Z, "Z_AXIS", 0, "Z Axis", "Rotate a point using Z axis"},
    {NODE_VECTOR_ROTATE_TYPE_EULER_XYZ, "EULER_XYZ", 0, "Euler", "Rotate a point using XYZ order"},
    {0, nullptr, 0, nullptr, nullptr},
};

const EnumPropertyItem rna_enum_node_vec_math_items[] = {
    {NODE_VECTOR_MATH_ADD, "ADD", 0, "Add", "A + B"},
    {NODE_VECTOR_MATH_SUBTRACT, "SUBTRACT", 0, "Subtract", "A - B"},
    {NODE_VECTOR_MATH_MULTIPLY, "MULTIPLY", 0, "Multiply", "Entry-wise multiply"},
    {NODE_VECTOR_MATH_DIVIDE, "DIVIDE", 0, "Divide", "Entry-wise divide"},
    {NODE_VECTOR_MATH_MULTIPLY_ADD, "MULTIPLY_ADD", 0, "Multiply Add", "A * B + C"},
    RNA_ENUM_ITEM_SEPR,
    {NODE_VECTOR_MATH_CROSS_PRODUCT, "CROSS_PRODUCT", 0, "Cross Product", "A cross B"},
    {NODE_VECTOR_MATH_PROJECT, "PROJECT", 0, "Project", "Project A onto B"},
    {NODE_VECTOR_MATH_REFLECT,
     "REFLECT",
     0,
     "Reflect",
     "Reflect A around the normal B. B does not need to be normalized."},
    {NODE_VECTOR_MATH_REFRACT,
     "REFRACT",
     0,
     "Refract",
     "For a given incident vector A, surface normal B and ratio of indices of refraction, Ior, "
     "refract returns the refraction vector, R"},
    {NODE_VECTOR_MATH_FACEFORWARD,
     "FACEFORWARD",
     0,
     "Faceforward",
     "Orients a vector A to point away from a surface B as defined by its normal C. "
     "Returns (dot(B, C) < 0) ? A : -A"},
    {NODE_VECTOR_MATH_DOT_PRODUCT, "DOT_PRODUCT", 0, "Dot Product", "A dot B"},
    RNA_ENUM_ITEM_SEPR,
    {NODE_VECTOR_MATH_DISTANCE, "DISTANCE", 0, "Distance", "Distance between A and B"},
    {NODE_VECTOR_MATH_LENGTH, "LENGTH", 0, "Length", "Length of A"},
    {NODE_VECTOR_MATH_SCALE, "SCALE", 0, "Scale", "A multiplied by Scale"},
    {NODE_VECTOR_MATH_NORMALIZE, "NORMALIZE", 0, "Normalize", "Normalize A"},
    RNA_ENUM_ITEM_SEPR,
    {NODE_VECTOR_MATH_ABSOLUTE, "ABSOLUTE", 0, "Absolute", "Entry-wise absolute"},
    {NODE_VECTOR_MATH_POWER, "POWER", 0, "Power", "Entry-wise power"},
    {NODE_VECTOR_MATH_SIGN, "SIGN", 0, "Sign", "Entry-wise sign"},
    {NODE_VECTOR_MATH_MINIMUM, "MINIMUM", 0, "Minimum", "Entry-wise minimum"},
    {NODE_VECTOR_MATH_MAXIMUM, "MAXIMUM", 0, "Maximum", "Entry-wise maximum"},
    {NODE_VECTOR_MATH_ROUND,
     "ROUND",
     0,
     "Round",
     "Entry-wise round to the nearest integer. Round upward if the fraction part is 0.5"},
    {NODE_VECTOR_MATH_FLOOR, "FLOOR", 0, "Floor", "Entry-wise floor"},
    {NODE_VECTOR_MATH_CEIL, "CEIL", 0, "Ceil", "Entry-wise ceil"},
    {NODE_VECTOR_MATH_FRACTION, "FRACTION", 0, "Fraction", "The fraction part of A entry-wise"},
    {NODE_VECTOR_MATH_MODULO, "MODULO", 0, "Modulo", "Entry-wise modulo using fmod(A,B)"},
    {NODE_VECTOR_MATH_WRAP, "WRAP", 0, "Wrap", "Entry-wise wrap(A,B)"},
    {NODE_VECTOR_MATH_SNAP,
     "SNAP",
     0,
     "Snap",
     "Round A to the largest integer multiple of B less than or equal A"},
    RNA_ENUM_ITEM_SEPR,
    {NODE_VECTOR_MATH_SINE, "SINE", 0, "Sine", "Entry-wise sin(A)"},
    {NODE_VECTOR_MATH_COSINE, "COSINE", 0, "Cosine", "Entry-wise cos(A)"},
    {NODE_VECTOR_MATH_TANGENT, "TANGENT", 0, "Tangent", "Entry-wise tan(A)"},
    {0, nullptr, 0, nullptr, nullptr},
};

const EnumPropertyItem rna_enum_node_map_range_items[] = {
    {NODE_MAP_RANGE_LINEAR,
     "LINEAR",
     0,
     "Linear",
     "Linear interpolation between From Min and From Max values"},
    {NODE_MAP_RANGE_STEPPED,
     "STEPPED",
     0,
     "Stepped Linear",
     "Stepped linear interpolation between From Min and From Max values"},
    {NODE_MAP_RANGE_SMOOTHSTEP,
     "SMOOTHSTEP",
     0,
     "Smooth Step",
     "Smooth Hermite edge interpolation between From Min and From Max values"},
    {NODE_MAP_RANGE_SMOOTHERSTEP,
     "SMOOTHERSTEP",
     0,
     "Smoother Step",
     "Smoother Hermite edge interpolation between From Min and From Max values"},
    {0, nullptr, 0, nullptr, nullptr},
};

const EnumPropertyItem rna_enum_node_clamp_items[] = {
    {NODE_CLAMP_MINMAX, "MINMAX", 0, "Min Max", "Constrain value between min and max"},
    {NODE_CLAMP_RANGE,
     "RANGE",
     0,
     "Range",
     "Constrain value between min and max, swapping arguments when min > max"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_sdf_shape_items[] = {
    /* 3D */
    {NODE_SDF_SHAPE_SPHERE, "SPHERE", 0, "Sphere (3D)", "Exact sphere"},
    {NODE_SDF_SHAPE_BOX, "BOX", 0, "Box (3D)", "Exact box half-extents"},
    {NODE_SDF_SHAPE_TORUS, "TORUS", 0, "Torus (3D)", "Exact torus major/minor"},
    {NODE_SDF_SHAPE_ROUND_BOX, "ROUND_BOX", 0, "Round Box (3D)", "Exact rounded box"},
    {NODE_SDF_SHAPE_BOX_FRAME, "BOX_FRAME", 0, "Box Frame (3D)", "Exact hollow box frame"},
    {NODE_SDF_SHAPE_CAPSULE, "CAPSULE", 0, "Capsule (3D)", "Exact vertical capsule"},
    {NODE_SDF_SHAPE_CAPPED_CYLINDER,
     "CAPPED_CYLINDER",
     0,
     "Capped Cylinder (3D)",
     "Exact vertical capped cylinder"},
    {NODE_SDF_SHAPE_CAPPED_CONE, "CAPPED_CONE", 0, "Capped Cone (3D)", "Exact capped cone"},
    {NODE_SDF_SHAPE_PLANE, "PLANE", 0, "Plane (3D)", "Exact infinite plane Y-up"},
    {NODE_SDF_SHAPE_HEX_PRISM, "HEX_PRISM", 0, "Hex Prism (3D)", "Exact hexagonal prism"},
    {NODE_SDF_SHAPE_OCTAHEDRON, "OCTAHEDRON", 0, "Octahedron (3D)", "Exact octahedron"},
    {NODE_SDF_SHAPE_PYRAMID, "PYRAMID", 0, "Pyramid (3D)", "Exact square pyramid"},
    {NODE_SDF_SHAPE_CAPPED_TORUS, "CAPPED_TORUS", 0, "Capped Torus (3D)", "Exact capped torus"},
    {NODE_SDF_SHAPE_LINK, "LINK", 0, "Link (3D)", "Exact chain link"},
    {NODE_SDF_SHAPE_INFINITE_CYLINDER,
     "INFINITE_CYLINDER",
     0,
     "Infinite Cylinder (3D)",
     "Exact infinite cylinder along Y"},
    {NODE_SDF_SHAPE_CONE, "CONE", 0, "Cone (3D)", "Exact finite cone (angle + height)"},
    {NODE_SDF_SHAPE_ROUNDED_CYLINDER,
     "ROUNDED_CYLINDER",
     0,
     "Rounded Cylinder (3D)",
     "Exact rounded cylinder"},
    {NODE_SDF_SHAPE_SOLID_ANGLE, "SOLID_ANGLE", 0, "Solid Angle (3D)", "Exact solid angle"},
    {NODE_SDF_SHAPE_CUT_SPHERE, "CUT_SPHERE", 0, "Cut Sphere (3D)", "Exact cut sphere"},
    {NODE_SDF_SHAPE_CUT_HOLLOW_SPHERE,
     "CUT_HOLLOW_SPHERE",
     0,
     "Cut Hollow Sphere (3D)",
     "Exact cut hollow sphere"},
    {NODE_SDF_SHAPE_DEATH_STAR, "DEATH_STAR", 0, "Death Star (3D)", "Exact sphere minus sphere"},
    {NODE_SDF_SHAPE_ROUND_CONE, "ROUND_CONE", 0, "Round Cone (3D)", "Exact round cone"},
    {NODE_SDF_SHAPE_RHOMBUS_3D, "RHOMBUS_3D", 0, "Rhombus (3D)", "Exact rhombus prism"},
    {NODE_SDF_SHAPE_ELLIPSOID, "ELLIPSOID", 0, "Ellipsoid (3D)", "Ellipsoid lower-bound SDF"},
    {NODE_SDF_SHAPE_TRI_PRISM, "TRI_PRISM", 0, "Tri Prism (3D)", "Triangular prism lower-bound"},
    {NODE_SDF_SHAPE_INFINITE_CONE, "INFINITE_CONE", 0, "Infinite Cone (3D)", "Exact infinite cone"},
    /* 2D */
    {NODE_SDF_SHAPE_CIRCLE_2D, "CIRCLE_2D", 0, "Circle (2D)", "Exact circle"},
    {NODE_SDF_SHAPE_BOX_2D, "BOX_2D", 0, "Box (2D)", "Exact box"},
    {NODE_SDF_SHAPE_ROUNDED_BOX_2D, "ROUNDED_BOX_2D", 0, "Rounded Box (2D)", "Exact rounded box"},
    {NODE_SDF_SHAPE_CHAMFER_BOX_2D, "CHAMFER_BOX_2D", 0, "Chamfer Box (2D)", "Exact chamfer box"},
    {NODE_SDF_SHAPE_SEGMENT_2D, "SEGMENT_2D", 0, "Segment (2D)", "Exact line segment"},
    {NODE_SDF_SHAPE_RHOMBUS_2D, "RHOMBUS_2D", 0, "Rhombus (2D)", "Exact rhombus"},
    {NODE_SDF_SHAPE_TRAPEZOID_2D, "TRAPEZOID_2D", 0, "Trapezoid (2D)", "Exact isosceles trapezoid"},
    {NODE_SDF_SHAPE_PARALLELOGRAM_2D,
     "PARALLELOGRAM_2D",
     0,
     "Parallelogram (2D)",
     "Exact parallelogram"},
    {NODE_SDF_SHAPE_EQUILATERAL_TRIANGLE_2D,
     "EQUILATERAL_TRIANGLE_2D",
     0,
     "Equilateral Triangle (2D)",
     "Exact equilateral triangle"},
    {NODE_SDF_SHAPE_ISOSCELES_TRIANGLE_2D,
     "ISOSCELES_TRIANGLE_2D",
     0,
     "Isosceles Triangle (2D)",
     "Exact isosceles triangle"},
    {NODE_SDF_SHAPE_TRIANGLE_2D, "TRIANGLE_2D", 0, "Triangle (2D)", "Exact arbitrary triangle"},
    {NODE_SDF_SHAPE_UNEVEN_CAPSULE_2D,
     "UNEVEN_CAPSULE_2D",
     0,
     "Uneven Capsule (2D)",
     "Exact uneven capsule"},
    {NODE_SDF_SHAPE_PENTAGON_2D, "PENTAGON_2D", 0, "Pentagon (2D)", "Exact regular pentagon"},
    {NODE_SDF_SHAPE_HEXAGON_2D, "HEXAGON_2D", 0, "Hexagon (2D)", "Exact regular hexagon"},
    {NODE_SDF_SHAPE_OCTOGON_2D, "OCTOGON_2D", 0, "Octogon (2D)", "Exact regular octogon"},
    {NODE_SDF_SHAPE_HEXAGRAM_2D, "HEXAGRAM_2D", 0, "Hexagram (2D)", "Exact hexagram / Star of David"},
    {NODE_SDF_SHAPE_STAR_2D, "STAR_2D", 0, "Star (2D)", "Exact regular star (n, m)"},
    {NODE_SDF_SHAPE_PIE_2D, "PIE_2D", 0, "Pie (2D)", "Exact pie / circular sector"},
    {NODE_SDF_SHAPE_CUT_DISK_2D, "CUT_DISK_2D", 0, "Cut Disk (2D)", "Exact cut disk"},
    {NODE_SDF_SHAPE_ARC_2D, "ARC_2D", 0, "Arc (2D)", "Exact annular arc"},
    {NODE_SDF_SHAPE_RING_2D, "RING_2D", 0, "Ring (2D)", "Exact ring sector"},
    {NODE_SDF_SHAPE_HORSESHOE_2D, "HORSESHOE_2D", 0, "Horseshoe (2D)", "Exact horseshoe"},
    {NODE_SDF_SHAPE_VESICA_2D, "VESICA_2D", 0, "Vesica (2D)", "Exact vesica piscis"},
    {NODE_SDF_SHAPE_MOON_2D, "MOON_2D", 0, "Moon (2D)", "Exact moon (circle minus circle)"},
    {NODE_SDF_SHAPE_HEART_2D, "HEART_2D", 0, "Heart (2D)", "Exact heart"},
    {NODE_SDF_SHAPE_CROSS_2D, "CROSS_2D", 0, "Cross (2D)", "Cross (exact exterior)"},
    {NODE_SDF_SHAPE_ROUNDED_X_2D, "ROUNDED_X_2D", 0, "Rounded X (2D)", "Exact rounded X"},
    {NODE_SDF_SHAPE_ELLIPSE_2D, "ELLIPSE_2D", 0, "Ellipse (2D)", "Exact ellipse"},
    {NODE_SDF_SHAPE_ORIENTED_BOX_2D,
     "ORIENTED_BOX_2D",
     0,
     "Oriented Box (2D)",
     "Exact oriented box between two points"},
    {NODE_SDF_SHAPE_TUNNEL_2D, "TUNNEL_2D", 0, "Tunnel (2D)", "Exact tunnel"},
    {NODE_SDF_SHAPE_STAIRS_2D, "STAIRS_2D", 0, "Stairs (2D)", "Exact stairs"},
    {NODE_SDF_SHAPE_ROUNDED_CROSS_2D,
     "ROUNDED_CROSS_2D",
     0,
     "Rounded Cross (2D)",
     "Exact rounded cross"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_fractal_primitive_items[] = {
    {NODE_FRACTAL_PRIM_MANDELBROT,
     "MANDELBROT",
     0,
     "Mandelbrot",
     "2D Mandelbrot DE (IQ complex derivative + smooth iteration color)"},
    {NODE_FRACTAL_PRIM_JULIA,
     "JULIA",
     0,
     "Julia",
     "2D Julia DE (IQ complex derivative; c from Julia C)"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_terrain_primitive_items[] = {
    {NODE_TERRAIN_PRIM_MOUNTAIN, "MOUNTAIN", 0, "Mountain", "Ridged mountain ready for erosion"},
    {NODE_TERRAIN_PRIM_MOUNTAINOUS,
     "MOUNTAINOUS",
     0,
     "Mountainous",
     "Heterogeneous highland with light ridging"},
    {NODE_TERRAIN_PRIM_MOUNTAIN_RANGE,
     "MOUNTAIN_RANGE",
     0,
     "Mountain Range",
     "Parallel ridges optimized for large-scale erosion"},
    {NODE_TERRAIN_PRIM_RIDGE, "RIDGE", 0, "Ridge", "Crest axis with gaussian falloff"},
    {NODE_TERRAIN_PRIM_CANYON, "CANYON", 0, "Canyon", "Highland cut by directional incision"},
    {NODE_TERRAIN_PRIM_CRATER, "CRATER", 0, "Crater", "Impact crater with rim and floor"},
    {NODE_TERRAIN_PRIM_DUNES, "DUNES", 0, "Dunes", "Asymmetric wind-shaped dune sea"},
    {NODE_TERRAIN_PRIM_ISLAND, "ISLAND", 0, "Island", "Landmass falloff from Offset"},
    {NODE_TERRAIN_PRIM_VOLCANO, "VOLCANO", 0, "Volcano", "Cone with caldera and rim"},
    {NODE_TERRAIN_PRIM_SLUMP, "SLUMP", 0, "Slump", "Soft slumped hills"},
    {NODE_TERRAIN_PRIM_PLATES, "PLATES", 0, "Plates", "Terraces with cliff edges"},
    {NODE_TERRAIN_PRIM_CRACKS, "CRACKS", 0, "Cracks", "Cracks carved from a plateau"},
    {NODE_TERRAIN_PRIM_RUGGED, "RUGGED", 0, "Rugged", "Slope-aware rocky breakup"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_terrain_erosion_items[] = {
    {NODE_TERRAIN_ERO_COMBINED,
     "COMBINED",
     0,
     "Erosion",
     "Hydraulic plus thermal (Gaea Erosion)"},
    {NODE_TERRAIN_ERO_HYDRAULIC, "HYDRAULIC", 0, "Hydraulic", "Pipe-model hydraulic erosion"},
    {NODE_TERRAIN_ERO_HYDRAULIC2,
     "HYDRAULIC2",
     0,
     "Erosion2",
     "Stronger downcutting and orographic rain"},
    {NODE_TERRAIN_ERO_THERMAL, "THERMAL", 0, "Thermal", "Talus / debris thermal erosion"},
    {NODE_TERRAIN_ERO_THERMAL2, "THERMAL2", 0, "Thermal2", "Thermal with diagonal neighbors"},
    {NODE_TERRAIN_ERO_HYDROFIX,
     "HYDROFIX",
     0,
     "HydroFix",
     "Prep: fill pits for continuous drainage (not an erosion algorithm)"},
    {NODE_TERRAIN_ERO_EASY, "EASY", 0, "Easy Erosion", "Fewer controls, curated defaults"},
    {NODE_TERRAIN_ERO_WIZARD, "WIZARD", 0, "Wizard", "Duration-driven wrapper with extra thermal"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_terrain_simulate_items[] = {
    {NODE_TERRAIN_SIM_SNOW, "SNOW", 0, "Snow", "Snow fall, melt and slope adhesion"},
    {NODE_TERRAIN_SIM_SNOWFIELD, "SNOWFIELD", 0, "Snowfield", "Broad settled snow"},
    {NODE_TERRAIN_SIM_DUSTING, "DUSTING", 0, "Dusting", "Thin dusting of snow or sediment"},
    {NODE_TERRAIN_SIM_RIVERS, "RIVERS", 0, "Rivers", "Carve a drainage / river network"},
    {NODE_TERRAIN_SIM_SEA, "SEA", 0, "Sea", "Water plane and coastal erosion"},
    {NODE_TERRAIN_SIM_LAKE, "LAKE", 0, "Lake", "Fill basins into lakes"},
    {NODE_TERRAIN_SIM_SEDIMENTS, "SEDIMENTS", 0, "Sediments", "Deposit in concavities"},
    {NODE_TERRAIN_SIM_CRUMBLE, "CRUMBLE", 0, "Crumble", "Collapse from edges and convexity"},
    {NODE_TERRAIN_SIM_GLACIER, "GLACIER", 0, "Glacier", "High-altitude glacial flow"},
    {NODE_TERRAIN_SIM_ANASTOMOSIS,
     "ANASTOMOSIS",
     0,
     "Anastomosis",
     "Branching flow-style extraction"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_terrain_modify_items[] = {
    {NODE_TERRAIN_MOD_TERRACE, "TERRACE", 0, "Terrace", "Stepped terraces"},
    {NODE_TERRAIN_MOD_STRATIFY, "STRATIFY", 0, "Stratify", "Broken rock strata"},
    {NODE_TERRAIN_MOD_WARP, "WARP", 0, "Warp", "Domain warp"},
    {NODE_TERRAIN_MOD_SLOPE_WARP, "SLOPE_WARP", 0, "Slope Warp", "Warp along slope"},
    {NODE_TERRAIN_MOD_FOLD, "FOLD", 0, "Fold", "Tectonic-style folding"},
    {NODE_TERRAIN_MOD_THERMAL_SHAPER, "THERMAL_SHAPER", 0, "Thermal Shaper", "Fast convexity/concavity"},
    {NODE_TERRAIN_MOD_STEPS, "STEPS", 0, "Steps", "Hard steps"},
    {NODE_TERRAIN_MOD_SANDSTONE, "SANDSTONE", 0, "Sandstone", "Layered sandstone"},
    {NODE_TERRAIN_MOD_CLAMP, "CLAMP", 0, "Clamp", "Clamp height range"},
    {NODE_TERRAIN_MOD_RECURVE, "RECURVE", 0, "Recurve", "Inflate / deflate by curvature"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_terrain_derive_items[] = {
    {NODE_TERRAIN_DRV_SLOPE, "SLOPE", 0, "Slope", "Slope mask"},
    {NODE_TERRAIN_DRV_CURVATURE, "CURVATURE", 0, "Curvature", "Convex peaks and ridges"},
    {NODE_TERRAIN_DRV_FLOW, "FLOW", 0, "Flow", "Local downhill flow"},
    {NODE_TERRAIN_DRV_HEIGHT, "HEIGHT", 0, "Height", "Height range mask"},
    {NODE_TERRAIN_DRV_PEAKS, "PEAKS", 0, "Peaks", "Peak isolation"},
    {NODE_TERRAIN_DRV_OCCLUSION, "OCCLUSION", 0, "Occlusion", "Cavity / sedimentary occlusion"},
    {NODE_TERRAIN_DRV_ANGLE, "ANGLE", 0, "Angle", "Aspect / facing mask"},
    {NODE_TERRAIN_DRV_SOIL, "SOIL", 0, "Soil", "Soil that settles in concavities"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_terrain_color_items[] = {
    {NODE_TERRAIN_COL_SATMAP, "SATMAP", 0, "SatMap", "Satellite-style height + slope coloring"},
    {NODE_TERRAIN_COL_WEATHERING, "WEATHERING", 0, "Weathering", "Dirt and wear in low slopes"},
    {NODE_TERRAIN_COL_SUPERCOLOR, "SUPERCOLOR", 0, "Super Color", "Flow and peak variation"},
    {NODE_TERRAIN_COL_WATERCOLOR, "WATERCOLOR", 0, "Water Color", "Colorize water bodies"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_terrain_combine_items[] = {
    {NODE_TERRAIN_CMB_MIX, "MIX", 0, "Mix", "Linear blend"},
    {NODE_TERRAIN_CMB_ADD, "ADD", 0, "Add", "Add B onto A"},
    {NODE_TERRAIN_CMB_SUBTRACT, "SUBTRACT", 0, "Subtract", "A minus B"},
    {NODE_TERRAIN_CMB_MULTIPLY, "MULTIPLY", 0, "Multiply", "Multiply heights"},
    {NODE_TERRAIN_CMB_MAX, "MAX", 0, "Max", "Keep the higher surface"},
    {NODE_TERRAIN_CMB_MIN, "MIN", 0, "Min", "Keep the lower surface"},
    {NODE_TERRAIN_CMB_SCREEN, "SCREEN", 0, "Screen", "Screen blend"},
    {NODE_TERRAIN_CMB_OVERLAY, "OVERLAY", 0, "Overlay", "Overlay blend"},
    {NODE_TERRAIN_CMB_HYPOT, "HYPOT", 0, "Hypot", "Hypotenuse combine"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_tex_dimensions_items[] = {
    {1, "1D", 0, "1D", "Use the scalar value W as input"},
    {2, "2D", 0, "2D", "Use the 2D vector (X, Y) as input. The Z component is ignored."},
    {3, "3D", 0, "3D", "Use the 3D vector (X, Y, Z) as input"},
    {4, "4D", 0, "4D", "Use the 4D vector (X, Y, Z, W) as input"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_mix_data_type_items[] = {
    {SOCK_FLOAT, "FLOAT", ICON_NODE_SOCKET_FLOAT, "Float", ""},
    {SOCK_VECTOR, "VECTOR", ICON_NODE_SOCKET_VECTOR, "Vector", ""},
    {SOCK_RGBA, "RGBA", ICON_NODE_SOCKET_RGBA, "Color", ""},
    {SOCK_ROTATION, "ROTATION", ICON_NODE_SOCKET_ROTATION, "Rotation", ""},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_shader_attribute_type_items[] = {
    {SHD_ATTRIBUTE_GEOMETRY,
     "GEOMETRY",
     0,
     "Geometry",
     "The attribute is associated with the object geometry, and its value "
     "varies from vertex to vertex, or within the object volume"},
    {SHD_ATTRIBUTE_OBJECT,
     "OBJECT",
     0,
     "Object",
     "The attribute is associated with the object or mesh data-block itself, "
     "and its value is uniform"},
    {SHD_ATTRIBUTE_INSTANCER,
     "INSTANCER",
     0,
     "Instancer",
     "The attribute is associated with the instancer particle system or object, "
     "falling back to the Object mode if the attribute isn't found, or the object "
     "is not instanced"},
    {SHD_ATTRIBUTE_VIEW_LAYER,
     "VIEW_LAYER",
     0,
     "View Layer",
     "The attribute is associated with the View Layer, Scene or World that is being rendered"},
    {SHD_ATTRIBUTE_LIGHT,
     "LIGHT",
     0,
     "Light",
     "The attribute is associated with the Light that is being rendered"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_shader_attribute_data_type_items[] = {
    {SHD_ATTRIBUTE_DATA_LEGACY,
     "LEGACY",
     0,
     "Legacy",
     "Color, Vector, Fac, and Alpha outputs (older files)"},
    {SHD_ATTRIBUTE_DATA_FLOAT, "FLOAT", 0, "Float", ""},
    {SHD_ATTRIBUTE_DATA_INT, "INT", 0, "Integer", ""},
    {SHD_ATTRIBUTE_DATA_VECTOR, "VECTOR", 0, "Vector", ""},
    {SHD_ATTRIBUTE_DATA_ROTATION, "QUATERNION", ICON_NODE_SOCKET_ROTATION, "Rotation", ""},
    {SHD_ATTRIBUTE_DATA_MATRIX, "MATRIX", 0, "Matrix", ""},
    {SHD_ATTRIBUTE_DATA_COLOR, "FLOAT_COLOR", 0, "Color", ""},
    {SHD_ATTRIBUTE_DATA_BOOLEAN, "BOOLEAN", 0, "Boolean", ""},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_shader_vect_transform_space_items[] = {
    {SHD_VECT_TRANSFORM_SPACE_WORLD, "WORLD", 0, "World", ""},
    {SHD_VECT_TRANSFORM_SPACE_OBJECT, "OBJECT", 0, "Object", ""},
    {SHD_VECT_TRANSFORM_SPACE_CAMERA, "CAMERA", 0, "Camera", ""},
    {SHD_VECT_TRANSFORM_SPACE_LIGHT, "LIGHT", 0, "Light", ""},
    {0, nullptr, 0, nullptr, nullptr},
};

}  // namespace blender

#ifdef RNA_RUNTIME

#  include "DNA_space_types.h"
#  include "DNA_text_types.h"

#  include "DEG_depsgraph_query.hh"

#  include "NOD_shader_raycast.hh"

#  include "RE_engine.h"

namespace blender {

using nodes::RaycastSampleAttributeItemsAccessor;

static void rna_Node_tex_image_update(Main *bmain, Scene * /*scene*/, PointerRNA *ptr)
{
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(ptr->owner_id);
  bNode *node = ptr->data_as<bNode>();

  /* Keep the Image socket default in sync with node->id (Cycles + vanilla .blend compat). */
  if (bNodeSocket *sock = bke::node_find_socket(*node, SOCK_IN, "Image"_ustr)) {
    if (auto *val = sock->default_value_typed<bNodeSocketValueImage>()) {
      Image *ima = id_cast<Image *>(node->id);
      if (val->value != ima) {
        id_us_min(val->value ? &val->value->id : nullptr);
        val->value = ima;
        id_us_plus(ima ? &ima->id : nullptr);
        BKE_ntree_update_tag_socket_property(ntree, sock);
      }
    }
  }

  BKE_ntree_update_tag_node_property(ntree, node);
  BKE_main_ensure_invariants(*bmain, ntree->id);
  DEG_relations_tag_update(bmain);
  WM_main_add_notifier(NC_IMAGE, nullptr);
}

static const EnumPropertyItem *rna_ShaderNodeMix_data_type_itemf(bContext * /*C*/,
                                                                 PointerRNA *ptr,
                                                                 PropertyRNA * /*prop*/,
                                                                 bool *r_free)
{
  *r_free = true;

  const auto rotation_supported_mix = [&](const EnumPropertyItem *item) -> bool {
    const eNodeSocketDatatype data_type = eNodeSocketDatatype(item->value);
    if (data_type == SOCK_ROTATION) {
      const bNodeTree *tree = reinterpret_cast<const bNodeTree *>(ptr->owner_id);
      if (ELEM(tree->type, NTREE_GEOMETRY, NTREE_COMPOSIT, NTREE_SHADER)) {
        return true;
      }
    }
    return ELEM(data_type, SOCK_FLOAT, SOCK_VECTOR, SOCK_RGBA);
  };

  return itemf_function_check(rna_enum_mix_data_type_items, rotation_supported_mix);
}

static void rna_ShaderNodeTexIES_mode_set(PointerRNA *ptr, int value)
{
  bNode *node = ptr->data_as<bNode>();
  NodeShaderTexIES *nss = static_cast<NodeShaderTexIES *>(node->storage);

  if (nss->mode != value) {
    nss->mode = value;
    nss->filepath[0] = '\0';

    /* replace text datablock by filepath */
    if (node->id) {
      Text *text = reinterpret_cast<Text *>(node->id);

      if (value == NODE_IES_EXTERNAL && text->filepath) {
        STRNCPY(nss->filepath, text->filepath);
        BLI_path_abs(nss->filepath, ID_BLEND_PATH_FROM_GLOBAL(&text->id));
        BLI_path_rel(nss->filepath, ID_BLEND_PATH_FROM_GLOBAL(ptr->owner_id));
      }

      id_us_min(node->id);
      node->id = nullptr;
    }
  }
}

static void rna_ShaderNodeScript_mode_set(PointerRNA *ptr, int value)
{
  bNode *node = ptr->data_as<bNode>();
  NodeShaderScript *nss = static_cast<NodeShaderScript *>(node->storage);

  if (nss->mode != value) {
    nss->mode = value;
    nss->filepath[0] = '\0';
    nss->flag &= ~NODE_SCRIPT_AUTO_UPDATE;

    /* replace text data-block by filepath */
    if (node->id) {
      Text *text = reinterpret_cast<Text *>(node->id);

      if (value == NODE_SCRIPT_EXTERNAL && text->filepath) {
        STRNCPY(nss->filepath, text->filepath);
        BLI_path_abs(nss->filepath, ID_BLEND_PATH_FROM_GLOBAL(&text->id));
        BLI_path_rel(nss->filepath, ID_BLEND_PATH_FROM_GLOBAL(ptr->owner_id));
      }

      id_us_min(node->id);
      node->id = nullptr;
    }

    /* remove any bytecode */
    if (nss->bytecode) {
      MEM_delete(nss->bytecode);
      nss->bytecode = nullptr;
    }

    nss->bytecode_hash[0] = '\0';
  }
}

static void rna_ShaderNodeScript_bytecode_get(PointerRNA *ptr, char *value)
{
  bNode *node = ptr->data_as<bNode>();
  NodeShaderScript *nss = static_cast<NodeShaderScript *>(node->storage);

  strcpy(value, (nss->bytecode) ? nss->bytecode : "");
}

static int rna_ShaderNodeScript_bytecode_length(PointerRNA *ptr)
{
  bNode *node = ptr->data_as<bNode>();
  NodeShaderScript *nss = static_cast<NodeShaderScript *>(node->storage);

  return (nss->bytecode) ? strlen(nss->bytecode) : 0;
}

static void rna_ShaderNodeScript_bytecode_set(PointerRNA *ptr, const char *value)
{
  bNode *node = ptr->data_as<bNode>();
  NodeShaderScript *nss = static_cast<NodeShaderScript *>(node->storage);

  if (nss->bytecode) {
    MEM_delete(nss->bytecode);
  }

  if (value && value[0]) {
    nss->bytecode = BLI_strdup(value);
  }
  else {
    nss->bytecode = nullptr;
  }
}

static void rna_ShaderNodeScript_update(Main *bmain, Scene *scene, PointerRNA *ptr)
{
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(ptr->owner_id);
  bNode *node = ptr->data_as<bNode>();
  RenderEngineType *engine_type = (scene != nullptr) ? RE_engines_find(scene->r.engine) : nullptr;

  if (engine_type && engine_type->update_script_node) {
    /* auto update node */
    RenderEngine *engine = RE_engine_create(engine_type);
    engine_type->update_script_node(engine, ntree, node);
    RE_engine_free(engine);
  }

  BKE_ntree_update_tag_node_property(ntree, node);
  BKE_main_ensure_invariants(*bmain, ntree->id);
}

static void rna_ShaderNode_is_active_output_set(PointerRNA *ptr, bool value)
{
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(ptr->owner_id);
  bNode *node = ptr->data_as<bNode>();
  if (value) {
    /* If this node becomes the active output, the others of the same type can't be the active
     * output anymore. */
    for (bNode *other_node : ntree->all_nodes()) {
      if (other_node->type_legacy == node->type_legacy) {
        other_node->flag &= ~NODE_DO_OUTPUT;
      }
    }
    node->flag |= NODE_DO_OUTPUT;
  }
  else {
    node->flag &= ~NODE_DO_OUTPUT;
  }
}

static const EnumPropertyItem *rna_NodeRaycastSampleAttributeItem_data_type_itemf(
    bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free)
{
  *r_free = true;
  /* See #attribute_type_type_with_socket_fn. */
  return itemf_function_check(rna_enum_attribute_type_items, [](const EnumPropertyItem *item) {
    return ELEM(item->value, CD_PROP_FLOAT, CD_PROP_FLOAT3, CD_PROP_COLOR);
  });
}

static const EnumPropertyItem *rna_NodeShaderAttribute_type_itemf(bContext *C,
                                                                  PointerRNA * /*ptr*/,
                                                                  PropertyRNA * /*prop*/,
                                                                  bool *r_free)
{
  if (C == nullptr) {
    return rna_enum_shader_attribute_type_items;
  }

  Scene *scene = CTX_data_scene(C);
  SpaceNode *space_node = CTX_wm_space_node(C);

  if (scene == nullptr || space_node == nullptr) {
    return rna_enum_shader_attribute_type_items;
  }

  *r_free = true;

  bool supports_light_attributes = !STREQ(CTX_data_scene(C)->r.engine, RE_engine_id_CYCLES) &&
                                   CTX_wm_space_node(C)->shaderfrom == SNODE_SHADER_OBJECT;
  return itemf_function_check(
      rna_enum_shader_attribute_type_items, [&](const EnumPropertyItem *item) {
        return supports_light_attributes || item->value != SHD_ATTRIBUTE_LIGHT;
      });
}

static const EnumPropertyItem *rna_NodeShaderAttribute_data_type_itemf(bContext * /*C*/,
                                                                       PointerRNA *ptr,
                                                                       PropertyRNA * /*prop*/,
                                                                       bool *r_free)
{
  const bNode *node = static_cast<const bNode *>(ptr->data);
  const auto *attr = node ? static_cast<const NodeShaderAttribute *>(node->storage) : nullptr;
  const bool show_legacy = attr && attr->data_type == SHD_ATTRIBUTE_DATA_LEGACY;
  *r_free = true;
  return itemf_function_check(rna_enum_shader_attribute_data_type_items,
                              [&](const EnumPropertyItem *item) {
                                return show_legacy || item->value != SHD_ATTRIBUTE_DATA_LEGACY;
                              });
}

static const EnumPropertyItem *rna_NodeShaderVectTransform_space_itemf(bContext *C,
                                                                       PointerRNA * /*ptr*/,
                                                                       PropertyRNA * /*prop*/,
                                                                       bool *r_free)
{
  if (C == nullptr) {
    return rna_enum_shader_vect_transform_space_items;
  }

  Scene *scene = CTX_data_scene(C);
  SpaceNode *space_node = CTX_wm_space_node(C);

  if (scene == nullptr || space_node == nullptr) {
    return rna_enum_shader_vect_transform_space_items;
  }

  *r_free = true;

  bool supports_light_transform = !STREQ(CTX_data_scene(C)->r.engine, RE_engine_id_CYCLES) &&
                                  CTX_wm_space_node(C)->shaderfrom == SNODE_SHADER_OBJECT;
  return itemf_function_check(
      rna_enum_shader_vect_transform_space_items, [&](const EnumPropertyItem *item) {
        return supports_light_transform || item->value != SHD_VECT_TRANSFORM_SPACE_LIGHT;
      });
}

static void rna_ShaderNodeAttribute_name_update(Main *bmain, Scene *scene, PointerRNA *ptr)
{
  bNode *node = ptr->data_as<bNode>();
  if (node && node->storage) {
    const auto &storage = *static_cast<const NodeShaderAttribute *>(node->storage);
    if (bNodeSocket *name_sock = bke::node_find_socket(*node, SOCK_IN, "Name"_ustr)) {
      if (name_sock->default_value) {
        STRNCPY(name_sock->default_value_typed<bNodeSocketValueString>()->value, storage.name);
      }
    }
  }
  rna_Node_update_relations(bmain, scene, ptr);
}

}  // namespace blender

#else

namespace blender {

static const EnumPropertyItem node_metallic_distribution_items[] = {
    {SHD_GLOSSY_BECKMANN, "BECKMANN", 0, "Beckmann", ""},
    {SHD_GLOSSY_GGX, "GGX", 0, "GGX", ""},
    {SHD_GLOSSY_MULTI_GGX,
     "MULTI_GGX",
     0,
     "Multiscatter GGX",
     "GGX with additional correction to account for multiple scattering, preserve energy and "
     "prevent unexpected darkening at high roughness"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_metallic_fresnel_type_items[] = {
    {SHD_PHYSICAL_CONDUCTOR,
     "PHYSICAL_CONDUCTOR",
     0,
     "Physical Conductor",
     "Fresnel conductor based on the complex refractive index per color channel"},
    {SHD_CONDUCTOR_F82,
     "F82",
     0,
     "F82 Tint",
     "An approximation of the Fresnel conductor curve based on the colors at perpendicular and "
     "near-grazing (roughly 82°) angles"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_glossy_items[] = {
    {SHD_GLOSSY_BECKMANN, "BECKMANN", 0, "Beckmann", ""},
    {SHD_GLOSSY_GGX, "GGX", 0, "GGX", ""},
    {SHD_GLOSSY_ASHIKHMIN_SHIRLEY, "ASHIKHMIN_SHIRLEY", 0, "Ashikhmin-Shirley", ""},
    {SHD_GLOSSY_MULTI_GGX,
     "MULTI_GGX",
     0,
     "Multiscatter GGX",
     "GGX with additional correction to account for multiple scattering, preserve energy and "
     "prevent unexpected darkening at high roughness"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_glass_items[] = {
    {SHD_GLOSSY_BECKMANN, "BECKMANN", 0, "Beckmann", ""},
    {SHD_GLOSSY_GGX, "GGX", 0, "GGX", ""},
    {SHD_GLOSSY_MULTI_GGX,
     "MULTI_GGX",
     0,
     "Multiscatter GGX",
     "GGX with additional correction to account for multiple scattering, preserve energy and "
     "prevent unexpected darkening at high roughness"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_refraction_items[] = {
    {SHD_GLOSSY_BECKMANN, "BECKMANN", 0, "Beckmann", ""},
    {SHD_GLOSSY_GGX, "GGX", 0, "GGX", ""},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_sheen_items[] = {
    {SHD_SHEEN_ASHIKHMIN, "ASHIKHMIN", 0, "Ashikhmin", "Classic Ashikhmin velvet (legacy model)"},
    {SHD_SHEEN_MICROFIBER,
     "MICROFIBER",
     0,
     "Microfiber",
     "Microflake-based model of multiple scattering between normal-oriented fibers"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_toon_items[] = {
    {SHD_TOON_DIFFUSE, "DIFFUSE", 0, "Diffuse", "Use diffuse BSDF"},
    {SHD_TOON_GLOSSY, "GLOSSY", 0, "Glossy", "Use glossy BSDF"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_hair_items[] = {
    {SHD_HAIR_REFLECTION,
     "Reflection",
     0,
     "Reflection",
     "The light that bounces off the surface of the hair"},
    {SHD_HAIR_TRANSMISSION,
     "Transmission",
     0,
     "Transmission",
     "The light that passes through the hair and exits on the other side"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_principled_hair_model_items[] = {
    {SHD_PRINCIPLED_HAIR_CHIANG,
     "CHIANG",
     0,
     "Chiang",
     "Near-field hair scattering model by Chiang et al. 2016, suitable for close-up looks, but is "
     "more noisy when viewing from a distance."},
    {SHD_PRINCIPLED_HAIR_HUANG,
     "HUANG",
     0,
     "Huang",
     "Multi-scale hair scattering model by Huang et al. 2022, suitable for viewing both up close "
     "and from a distance, supports elliptical cross-sections and has more precise highlight in "
     "forward scattering directions."},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_principled_hair_parametrization_items[] = {
    {SHD_PRINCIPLED_HAIR_DIRECT_ABSORPTION,
     "ABSORPTION",
     0,
     "Absorption Coefficient",
     "Directly set the absorption coefficient \"sigma_a\" (this is not the most intuitive way to "
     "color hair)"},
    {SHD_PRINCIPLED_HAIR_PIGMENT_CONCENTRATION,
     "MELANIN",
     0,
     "Melanin Concentration",
     "Define the melanin concentrations below to get the most realistic-looking hair (you can get "
     "the concentrations for different types of hair online)"},
    {SHD_PRINCIPLED_HAIR_REFLECTANCE,
     "COLOR",
     0,
     "Direct Coloring",
     "Choose the color of your preference, and the shader will approximate the absorption "
     "coefficient to render lookalike hair"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_script_mode_items[] = {
    {NODE_SCRIPT_INTERNAL, "INTERNAL", 0, "Internal", "Use internal text data-block"},
    {NODE_SCRIPT_EXTERNAL, "EXTERNAL", 0, "External", "Use external .osl or .oso file"},
    {0, nullptr, 0, nullptr, nullptr},
};

static EnumPropertyItem node_ies_mode_items[] = {
    {NODE_IES_INTERNAL, "INTERNAL", 0, "Internal", "Use internal text data-block"},
    {NODE_IES_EXTERNAL, "EXTERNAL", 0, "External", "Use external .ies file"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_principled_distribution_items[] = {
    {SHD_GLOSSY_GGX, "GGX", 0, "GGX", ""},
    {SHD_GLOSSY_MULTI_GGX,
     "MULTI_GGX",
     0,
     "Multiscatter GGX",
     "GGX with additional correction to account for multiple scattering, preserve energy and "
     "prevent unexpected darkening at high roughness"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_light_evaluation_mode_items[] = {
    {SHD_LIGHT_EVAL_DIFFUSE, "DIFFUSE", 0, "Diffuse", ""},
    {SHD_LIGHT_EVAL_GLOSSY, "GLOSSY", 0, "Glossy", ""},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem node_subsurface_method_items[] = {
    {SHD_SUBSURFACE_BURLEY,
     "BURLEY",
     0,
     "Christensen-Burley",
     "Approximation to physically based volume scattering"},
    {SHD_SUBSURFACE_RANDOM_WALK,
     "RANDOM_WALK",
     0,
     "Random Walk",
     "Volumetric approximation to physically based volume scattering, using the scattering radius "
     "as specified"},
    {SHD_SUBSURFACE_RANDOM_WALK_SKIN,
     "RANDOM_WALK_SKIN",
     0,
     "Random Walk (Skin)",
     "Volumetric approximation to physically based volume scattering, with scattering radius "
     "automatically adjusted to match color textures. Designed for skin shading."},
    {SHD_SUBSURFACE_RANDOM_WALK_LEGACY,
     "RANDOM_WALK_LEGACY",
     0,
     "Random Walk (Legacy)",
     "Volumetric approximation to physically based volume scattering, using the scattering radius "
     "as specified"},
    {0, nullptr, 0, nullptr, nullptr}};

static const EnumPropertyItem node_scatter_phase_items[] = {
    {SHD_PHASE_HENYEY_GREENSTEIN,
     "HENYEY_GREENSTEIN",
     0,
     "Henyey-Greenstein",
     "Henyey-Greenstein, default phase function for the scattering of light"},
    {SHD_PHASE_FOURNIER_FORAND,
     "FOURNIER_FORAND",
     0,
     "Fournier-Forand",
     "Fournier-Forand phase function, used for the scattering of light in underwater "
     "environments"},
    {SHD_PHASE_DRAINE,
     "DRAINE",
     0,
     "Draine",
     "Draine phase functions, mostly used for the scattering of light in interstellar dust"},
    {SHD_PHASE_RAYLEIGH,
     "RAYLEIGH",
     0,
     "Rayleigh",
     "Rayleigh phase function, mostly used for particles smaller than the wavelength of light, "
     "such as scattering of sunlight in earth's atmosphere"},
    {SHD_PHASE_MIE,
     "MIE",
     0,
     "Mie",
     "Approximation of Mie scattering in water droplets, used for scattering in clouds and fog"},
    {0, nullptr, 0, nullptr, nullptr},
};

static void def_clamp(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "clamp_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, rna_enum_node_clamp_items);
  RNA_def_property_ui_text(prop, "Clamp Type", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static const EnumPropertyItem rna_enum_node_shader_derivative_operation_items[] = {
    {NODE_SHADER_DERIVATIVE_DDX,
     "DDX",
     0,
     "DDX",
     "Partial derivative with respect to screen-space X (HLSL ddx / GLSL dFdx)"},
    {NODE_SHADER_DERIVATIVE_DDY,
     "DDY",
     0,
     "DDY",
     "Partial derivative with respect to screen-space Y (HLSL ddy / GLSL dFdy)"},
    {NODE_SHADER_DERIVATIVE_FWIDTH,
     "FWIDTH",
     0,
     "Filter Width",
     "abs(DDX) + abs(DDY), for procedural anti-aliasing"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_shader_derivative_data_type_items[] = {
    {SOCK_FLOAT, "FLOAT", ICON_NODE_SOCKET_FLOAT, "Float", ""},
    {SOCK_VECTOR, "VECTOR", ICON_NODE_SOCKET_VECTOR, "Vector", ""},
    {SOCK_RGBA, "RGBA", ICON_NODE_SOCKET_RGBA, "Color", ""},
    {0, nullptr, 0, nullptr, nullptr},
};

static void def_sh_derivative(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "operation", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, rna_enum_node_shader_derivative_operation_items);
  RNA_def_property_ui_text(prop, "Operation", "Which screen-space derivative to compute");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "data_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom2");
  RNA_def_property_enum_items(prop, rna_enum_node_shader_derivative_data_type_items);
  RNA_def_property_ui_text(prop, "Data Type", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_sdf_shape(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "shape_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, rna_enum_node_sdf_shape_items);
  RNA_def_property_ui_text(prop, "Shape", "SDF primitive type (Inigo Quilez exact formulas)");
  /* Refresh socket availability when the primitive changes. */
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_fractal_primitive(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "fractal_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, rna_enum_node_fractal_primitive_items);
  RNA_def_property_ui_text(prop, "Fractal", "2D Mandelbrot or Julia set");
  /* Refresh Julia C socket availability when type changes. */
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_terrain_enum(StructRNA *srna,
                             const char *identifier,
                             const char *ui_name,
                             const char *ui_desc,
                             const EnumPropertyItem *items)
{
  PropertyRNA *prop = RNA_def_property(srna, identifier, PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, items);
  RNA_def_property_ui_text(prop, ui_name, ui_desc);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_terrain_primitive(BlenderRNA * /*brna*/, StructRNA *srna)
{
  def_terrain_enum(srna,
                   "primitive_type",
                   "Type",
                   "Terrain generator",
                   rna_enum_node_terrain_primitive_items);
}

static void def_terrain_erosion(BlenderRNA * /*brna*/, StructRNA *srna)
{
  def_terrain_enum(srna,
                   "erosion_type",
                   "Type",
                   "Hydraulic / thermal / hydro-fix variant",
                   rna_enum_node_terrain_erosion_items);
}

static void def_terrain_simulate(BlenderRNA * /*brna*/, StructRNA *srna)
{
  def_terrain_enum(srna,
                   "simulate_type",
                   "Type",
                   "Snow, water, debris or vegetation-style simulation",
                   rna_enum_node_terrain_simulate_items);
}

static void def_terrain_modify(BlenderRNA * /*brna*/, StructRNA *srna)
{
  def_terrain_enum(srna,
                   "modify_type",
                   "Type",
                   "Terrace, warp, fold and other modifiers",
                   rna_enum_node_terrain_modify_items);
}

static void def_terrain_derive(BlenderRNA * /*brna*/, StructRNA *srna)
{
  def_terrain_enum(srna,
                   "derive_type",
                   "Type",
                   "Mask derived from the heightfield",
                   rna_enum_node_terrain_derive_items);
}

static void def_terrain_color(BlenderRNA * /*brna*/, StructRNA *srna)
{
  def_terrain_enum(srna,
                   "color_type",
                   "Type",
                   "How to colorize the terrain",
                   rna_enum_node_terrain_color_items);
}

static void def_terrain_combine(BlenderRNA * /*brna*/, StructRNA *srna)
{
  def_terrain_enum(srna,
                   "combine_type",
                   "Mode",
                   "Heightfield blend mode",
                   rna_enum_node_terrain_combine_items);
}

const EnumPropertyItem sh_tex_prop_interpolation_items[] = {
    {SHD_INTERP_LINEAR, "Linear", 0, "Linear", "Linear interpolation"},
    {SHD_INTERP_CLOSEST, "Closest", 0, "Closest", "No interpolation (sample closest texel)"},
    {SHD_INTERP_CUBIC, "Cubic", 0, "Cubic", "Cubic interpolation"},
    {SHD_INTERP_SMART, "Smart", 0, "Smart", "Bicubic when magnifying, else bilinear (OSL only)"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_parallax_mode_items[] = {
    {NODE_PARALLAX_OFFSET,
     "OFFSET",
     0,
     "Offset",
     "Single-sample parallax offset. Cheapest, more distortion at grazing angles"},
    {NODE_PARALLAX_STEEP,
     "STEEP",
     0,
     "Steep",
     "Layered ray march without interpolation"},
    {NODE_PARALLAX_POM,
     "POM",
     0,
     "POM",
     "Parallax occlusion mapping with interpolation and optional refinement"},
    {NODE_PARALLAX_POM_SHADOW,
     "POM_SHADOW",
     0,
     "POM + Shadow",
     "POM plus height-field self-shadowing from the Light input"},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem rna_enum_node_parallax_channel_items[] = {
    {NODE_PARALLAX_CHANNEL_R, "R", 0, "Red", "Use the red channel as height"},
    {NODE_PARALLAX_CHANNEL_G, "G", 0, "Green", "Use the green channel as height"},
    {NODE_PARALLAX_CHANNEL_B, "B", 0, "Blue", "Use the blue channel as height"},
    {NODE_PARALLAX_CHANNEL_A, "A", 0, "Alpha", "Use the alpha channel as height"},
    {NODE_PARALLAX_CHANNEL_AVERAGE, "AVERAGE", 0, "Average", "Average of RGB"},
    {NODE_PARALLAX_CHANNEL_LUMINANCE, "LUMINANCE", 0, "Luminance", "Rec. 709 luminance of RGB"},
    {0, nullptr, 0, nullptr, nullptr},
};

static void def_sh_parallax(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeShaderParallax", "storage");

  prop = RNA_def_property(srna, "mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "mode");
  RNA_def_property_enum_items(prop, rna_enum_node_parallax_mode_items);
  RNA_def_property_ui_text(prop, "Mode", "Parallax algorithm");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");

  prop = RNA_def_property(srna, "invert", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NODE_PARALLAX_INVERT);
  RNA_def_property_ui_text(
      prop, "Invert", "Treat the image as a depth map (white is deep) instead of a height map");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "clip", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NODE_PARALLAX_CLIP);
  RNA_def_property_ui_text(
      prop, "Clip", "Keep the original UV when the ray marches outside 0–1");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "channel", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "channel");
  RNA_def_property_enum_items(prop, rna_enum_node_parallax_channel_items);
  RNA_def_property_ui_text(prop, "Channel", "Height map channel");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "interpolation", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "interpolation");
  RNA_def_property_enum_items(prop, sh_tex_prop_interpolation_items);
  RNA_def_property_ui_text(prop, "Interpolation", "Texture interpolation");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "extension", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "extension");
  RNA_def_property_enum_items(prop, prop_image_extension);
  RNA_def_property_ui_text(
      prop, "Extension", "How the image is extrapolated past its original bounds");
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_ID_IMAGE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "image_user", PROP_POINTER, PROP_NONE);
  RNA_def_property_flag(prop, PROP_NEVER_NULL);
  RNA_def_property_pointer_sdna(prop, nullptr, "iuser");
  RNA_def_property_ui_text(
      prop,
      "Image User",
      "Parameters defining which layer, pass and frame of the image is displayed");
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void def_sh_spom(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeShaderParallax", "storage");

  prop = RNA_def_property(srna, "invert", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NODE_PARALLAX_INVERT);
  RNA_def_property_ui_text(
      prop, "Invert", "Treat the image as a depth map (white is deep) instead of a height map");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "channel", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "channel");
  RNA_def_property_enum_items(prop, rna_enum_node_parallax_channel_items);
  RNA_def_property_ui_text(prop, "Channel", "Height map channel");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "interpolation", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "interpolation");
  RNA_def_property_enum_items(prop, sh_tex_prop_interpolation_items);
  RNA_def_property_ui_text(prop, "Interpolation", "Texture interpolation");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "extension", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "extension");
  RNA_def_property_enum_items(prop, prop_image_extension);
  RNA_def_property_ui_text(
      prop, "Extension", "How the image is extrapolated past its original bounds");
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_ID_IMAGE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "image_user", PROP_POINTER, PROP_NONE);
  RNA_def_property_flag(prop, PROP_NEVER_NULL);
  RNA_def_property_pointer_sdna(prop, nullptr, "iuser");
  RNA_def_property_ui_text(
      prop,
      "Image User",
      "Parameters defining which layer, pass and frame of the image is displayed");
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void def_image_camera_view(BlenderRNA * /*brna*/, StructRNA *srna)
{
  static const EnumPropertyItem alpha_mode_items[] = {
      {0, "SKY", 0, "Sky", "Use the sky/world background"},
      {1, "TRANSPARENT", 0, "Transparent", "Transparent background (premultiplied alpha)"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  /* Four 3D Viewport shading modes (preview engines ?? not F12 final render). */
  static const EnumPropertyItem draw_type_items[] = {
      {OB_WIRE, "WIREFRAME", ICON_SHADING_WIRE, "Wireframe", "Wireframe viewport shading"},
      {OB_SOLID, "SOLID", ICON_SHADING_SOLID, "Solid", "Solid viewport shading"},
      {OB_MATERIAL,
       "MATERIAL",
       ICON_SHADING_RENDERED,
       "Material Preview",
       "Material Preview viewport shading (EEVEE lookdev)"},
      {OB_RENDER,
       "RENDERED",
       ICON_SHADING_RENDERED,
       "Rendered",
       "Rendered viewport shading (EEVEE/Workbench preview, not full F12 render)"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  /* Camera picker on node.id ?? filtered to Camera objects only. */
  prop = RNA_def_property(srna, "camera", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Object");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_pointer_funcs(prop, nullptr, nullptr, nullptr, "rna_Camera_object_poll");
  RNA_def_property_ui_text(prop, "Camera", "Camera object used for the view; empty uses scene camera");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  RNA_def_struct_sdna_from(srna, "NodeImageCameraView", "storage");

  prop = RNA_def_property(srna, "draw_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "draw_type");
  RNA_def_property_enum_items(prop, draw_type_items);
  RNA_def_property_ui_text(prop, "Shading", "Viewport shading type used for the offscreen render");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "alpha_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "alpha_mode");
  RNA_def_property_enum_items(prop, alpha_mode_items);
  RNA_def_property_ui_text(prop, "Alpha", "Background alpha for the rendered view");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "show_annotations", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "show_annotations", 1);
  RNA_def_property_ui_text(
      prop,
      "Annotations",
      "Include scene Annotation (grease pencil) strokes drawn in the 3D Viewport");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_image_render_material(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "material", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Material");
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_ID_REFCOUNT);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(
      prop, "Material", "Material temporarily assigned to a UV plane and rendered to Color");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_image_shadertoy(BlenderRNA * /*brna*/, StructRNA *srna)
{
  static const EnumPropertyItem pass_view_items[] = {
      {0, "IMAGE", 0, "Image", "Main Image pass (mainImage)"},
      {1, "COMMON", 0, "Common", "Shared code prepended to every pass"},
      {2, "BUFFER_A", 0, "Buffer A", "Multipass Buffer A"},
      {3, "BUFFER_B", 0, "Buffer B", "Multipass Buffer B"},
      {4, "BUFFER_C", 0, "Buffer C", "Multipass Buffer C"},
      {5, "BUFFER_D", 0, "Buffer D", "Multipass Buffer D"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeImageShaderToy", "storage");

  prop = RNA_def_property(srna, "url", PROP_STRING, PROP_NONE);
  RNA_def_property_string_maxlength(prop, 1024);
  RNA_def_property_ui_text(
      prop, "URL", "ShaderToy page URL or shader ID (https://www.shadertoy.com/view/XXXXXX)");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "shader_id", PROP_STRING, PROP_NONE);
  RNA_def_property_string_maxlength(prop, 16);
  RNA_def_property_ui_text(prop, "ID", "ShaderToy shader ID extracted from the URL");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "shader_name", PROP_STRING, PROP_NONE);
  RNA_def_property_string_sdna(prop, nullptr, "name");
  RNA_def_property_string_maxlength(prop, 256);
  RNA_def_property_ui_text(prop, "Name", "Shader title from ShaderToy");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);

  prop = RNA_def_property(srna, "author", PROP_STRING, PROP_NONE);
  RNA_def_property_string_maxlength(prop, 128);
  RNA_def_property_ui_text(prop, "Author", "ShaderToy username");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);

  prop = RNA_def_property(srna, "status", PROP_STRING, PROP_NONE);
  RNA_def_property_string_maxlength(prop, 256);
  RNA_def_property_ui_text(prop, "Status", "Last fetch / compile status");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);

  prop = RNA_def_property(srna, "warning", PROP_STRING, PROP_NONE);
  RNA_def_property_string_maxlength(prop, 512);
  RNA_def_property_ui_text(prop, "Warning", "Skipped passes or unsupported inputs");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);

  prop = RNA_def_property(srna, "channel_map", PROP_STRING, PROP_NONE);
  RNA_def_property_string_maxlength(prop, 1024);
  RNA_def_property_flag(prop, PROP_HIDDEN);
  RNA_def_property_ui_text(prop, "Channel Map", "Deprecated compact map");

  prop = RNA_def_property(srna, "channels_image", PROP_STRING, PROP_NONE);
  RNA_def_property_string_maxlength(prop, 32);
  RNA_def_property_ui_text(
      prop,
      "Image",
      "Image iChannel0–3. A–D = Buffer, T = texture, K = black, U = cubemap, V = volume. Example: A,B,D,U. Short lists pad with T");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "channels_buffer_a", PROP_STRING, PROP_NONE);
  RNA_def_property_string_maxlength(prop, 32);
  RNA_def_property_ui_text(
      prop, "Buffer A", "Buffer A iChannel0–3 (A–D / T / K / U cubemap / V volume), pad with T");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "channels_buffer_b", PROP_STRING, PROP_NONE);
  RNA_def_property_string_maxlength(prop, 32);
  RNA_def_property_ui_text(
      prop, "Buffer B", "Buffer B iChannel0–3 (A–D / T / K / U cubemap / V volume), pad with T");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "channels_buffer_c", PROP_STRING, PROP_NONE);
  RNA_def_property_string_maxlength(prop, 32);
  RNA_def_property_ui_text(
      prop, "Buffer C", "Buffer C iChannel0–3 (A–D / T / K / U cubemap / V volume), pad with T");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "channels_buffer_d", PROP_STRING, PROP_NONE);
  RNA_def_property_string_maxlength(prop, 32);
  RNA_def_property_ui_text(
      prop, "Buffer D", "Buffer D iChannel0–3 (A–D / T / K / U cubemap / V volume), pad with T");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "api_key", PROP_STRING, PROP_PASSWORD);
  RNA_def_property_string_maxlength(prop, 128);
  RNA_def_property_ui_text(
      prop,
      "API Key",
      "Optional ShaderToy API key (shadertoy.com/howto). "
      "Falls back to SHADERTOY_KEY env, then unofficial POST");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);

  prop = RNA_def_property(srna, "code_common", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Common", "Shared GLSL prepended to every pass");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | ND_DISPLAY, nullptr);

  prop = RNA_def_property(srna, "code_buffer_a", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Buffer A", "Buffer A GLSL (mainImage)");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | ND_DISPLAY, nullptr);

  prop = RNA_def_property(srna, "code_buffer_b", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Buffer B", "Buffer B GLSL (mainImage)");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | ND_DISPLAY, nullptr);

  prop = RNA_def_property(srna, "code_buffer_c", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Buffer C", "Buffer C GLSL (mainImage)");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | ND_DISPLAY, nullptr);

  prop = RNA_def_property(srna, "code_buffer_d", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Buffer D", "Buffer D GLSL (mainImage)");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | ND_DISPLAY, nullptr);

  prop = RNA_def_property(srna, "code_image", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Image", "Image pass GLSL (mainImage) — final Color");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | ND_DISPLAY, nullptr);

  prop = RNA_def_property(srna, "error_log", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Error Log", "Last GPU compile / runtime error");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);

  prop = RNA_def_property(srna, "textbox_state", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "TextboxState");
  RNA_def_property_pointer_sdna(prop, nullptr, "textbox_state");

  prop = RNA_def_property(srna, "pass_view", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, pass_view_items);
  RNA_def_property_ui_text(prop, "Pass", "Which pass to edit");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_NODE | ND_DISPLAY, nullptr);

  prop = RNA_def_property(srna, "param_count", PROP_INT, PROP_NONE);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE | PROP_ANIMATABLE);
  RNA_def_property_ui_text(prop, "Parameter Count", "Number of scanned #define parameters");

  prop = RNA_def_property(srna, "param_value", PROP_FLOAT, PROP_NONE);
  RNA_def_property_array(prop, 16);
  RNA_def_property_ui_range(prop, -10000.0, 10000.0, 0.01, 4);
  RNA_def_property_ui_text(prop, "Parameter", "Shader #define / const value");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "cam_rot", PROP_FLOAT, PROP_EULER);
  RNA_def_property_float_sdna(prop, nullptr, "cam_rot");
  RNA_def_property_array(prop, 3);
  RNA_def_property_flag(prop, PROP_HIDDEN);
  RNA_def_property_ui_text(prop, "Camera Rotation", "Deprecated; use Mouse");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "mouse_pos", PROP_FLOAT, PROP_FACTOR);
  RNA_def_property_float_sdna(prop, nullptr, "mouse_pos");
  RNA_def_property_array(prop, 2);
  RNA_def_property_range(prop, 0.0, 1.0);
  RNA_def_property_ui_range(prop, 0.0, 1.0, 0.01, 3);
  RNA_def_property_ui_text(
      prop,
      "Mouse",
      "Normalized pointer (0–1), same as ShaderToy iMouse.xy / iResolution.xy. "
      "(0,0) is the site default (iMouse = 0). Any other value is a held click at that point");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "mouse_down", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "mouse_down", 1);
  RNA_def_property_flag(prop, PROP_HIDDEN);
  RNA_def_property_ui_text(prop, "Mouse Down", "Deprecated; Mouse non-zero is treated as held");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "reset_buffers", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "reset_buffers", 1);
  RNA_def_property_ui_text(prop, "Reset Buffers", "Clear Buffer A–D ping-pong textures on next cook");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_image_histogram(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeImageHistogram", "storage");

  prop = RNA_def_property(srna, "histogram", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "hist");
  RNA_def_property_struct_type(prop, "Histogram");
  RNA_def_property_ui_text(
      prop, "Histogram", "Statistical view of input color levels (updated when the node cooks)");
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "levels_black", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, "NodeImageHistogram", "hist.levels_black");
  /* HDR: not limited to 0?C1 (over/under exposure correction). */
  RNA_def_property_range(prop, -100.0, 100.0);
  RNA_def_property_ui_range(prop, -1.0, 4.0, 0.01, 4);
  RNA_def_property_ui_text(
      prop,
      "Black",
      "Input black point in scene-linear units (left handle; may be outside 0–1 for HDR)");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "levels_white", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, "NodeImageHistogram", "hist.levels_white");
  RNA_def_property_range(prop, -100.0, 100.0);
  RNA_def_property_ui_range(prop, -1.0, 4.0, 0.01, 4);
  RNA_def_property_ui_text(
      prop,
      "White",
      "Input white point in scene-linear units (right handle; set >1 to pull overexposure down)");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "levels_gamma", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, "NodeImageHistogram", "hist.levels_gamma");
  RNA_def_property_range(prop, 0.01, 10.0);
  RNA_def_property_ui_range(prop, 0.1, 3.0, 0.1, 3);
  RNA_def_property_ui_text(
      prop, "Gamma", "Midtones (also the middle vertical handle on the histogram)");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "use_auto", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "use_auto", 1);
  RNA_def_property_ui_text(prop,
                           "Auto",
                           "Automatically stretch black/white from the input histogram each cook");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_map_range(BlenderRNA * /*brna*/, StructRNA *srna)
{
  static const EnumPropertyItem rna_enum_data_type_items[] = {
      {CD_PROP_FLOAT, "FLOAT", ICON_NODE_SOCKET_FLOAT, "Float", "Floating-point value"},
      {CD_PROP_FLOAT3,
       "FLOAT_VECTOR",
       ICON_NODE_SOCKET_VECTOR,
       "Vector",
       "3D vector with floating-point values"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_struct_sdna_from(srna, "NodeMapRange", "storage");

  PropertyRNA *prop;

  prop = RNA_def_property(srna, "clamp", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "clamp", 1);
  RNA_def_property_ui_text(prop, "Clamp", "Clamp the result to the target range [To Min, To Max]");
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "interpolation_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "interpolation_type");
  RNA_def_property_enum_items(prop, rna_enum_node_map_range_items);
  RNA_def_property_ui_text(prop, "Interpolation Type", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");

  prop = RNA_def_property(srna, "data_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_bitflag_sdna(prop, nullptr, "data_type");
  RNA_def_property_enum_items(prop, rna_enum_data_type_items);
  RNA_def_property_ui_text(prop, "Data Type", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_sh_mix(BlenderRNA * /*brna*/, StructRNA *srna)
{
  static const EnumPropertyItem rna_enum_mix_mode_items[] = {
      {NODE_MIX_MODE_UNIFORM, "UNIFORM", 0, "Uniform", "Use a single factor for all components"},
      {NODE_MIX_MODE_NON_UNIFORM, "NON_UNIFORM", 0, "Non-Uniform", "Per component factor"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeShaderMix", "storage");

  prop = RNA_def_property(srna, "data_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_funcs(prop, nullptr, nullptr, "rna_ShaderNodeMix_data_type_itemf");
  RNA_def_property_enum_items(prop, rna_enum_mix_data_type_items);
  RNA_def_property_enum_default(prop, SOCK_FLOAT);
  RNA_def_property_ui_text(prop, "Data Type", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");

  prop = RNA_def_property(srna, "factor_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_mix_mode_items);
  RNA_def_property_enum_default(prop, NODE_MIX_MODE_UNIFORM);
  RNA_def_property_ui_text(prop, "Factor Mode", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");

  prop = RNA_def_property(srna, "blend_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "blend_type");
  RNA_def_property_enum_items(prop, rna_enum_ramp_blend_items);
  RNA_def_property_ui_text(prop, "Blending Mode", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "clamp_factor", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "clamp_factor", 1);
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_ui_text(prop, "Clamp Factor", "Clamp the factor to [0,1] range");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "clamp_result", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "clamp_result", 1);
  RNA_def_property_ui_text(prop, "Clamp Result", "Clamp the result to [0,1] range");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_vector_math(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "operation", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, rna_enum_node_vec_math_items);
  RNA_def_property_ui_text(prop, "Operation", "");
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_ID_NODETREE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
}

static void def_vector_curve(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "mapping", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "storage");
  RNA_def_property_struct_type(prop, "CurveMapping");
  RNA_def_property_ui_text(prop, "Mapping", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_float_curve(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "mapping", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "storage");
  RNA_def_property_struct_type(prop, "CurveMapping");
  RNA_def_property_ui_text(prop, "Mapping", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_sh_output(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "is_active_output", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NODE_DO_OUTPUT);
  RNA_def_property_ui_text(
      prop, "Active Output", "True if this node is used as the active output");
  RNA_def_property_boolean_funcs(prop, nullptr, "rna_ShaderNode_is_active_output_set");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "target", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, prop_shader_output_target_items);
  RNA_def_property_ui_text(
      prop, "Target", "Which renderer and viewport shading types to use the shaders for");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_sh_output_linestyle(BlenderRNA *brna, StructRNA *srna)
{
  def_sh_output(brna, srna);
  def_mix_rgb(brna, srna);
}

static void def_sh_mapping(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "vector_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, rna_enum_mapping_type_items);
  RNA_def_property_ui_text(prop, "Type", "Type of vector that the mapping transforms");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_sh_vector_rotate(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "rotation_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, rna_enum_vector_rotate_type_items);
  RNA_def_property_ui_text(prop, "Type", "Type of angle input");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");

  prop = RNA_def_property(srna, "invert", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "custom2", 0);
  RNA_def_property_ui_text(prop, "Invert", "Invert the rotation angle");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_sh_attribute(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeShaderAttribute", "storage");

  prop = RNA_def_property(srna, "attribute_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "type");
  RNA_def_property_enum_funcs(prop, nullptr, nullptr, "rna_NodeShaderAttribute_type_itemf");
  RNA_def_property_enum_items(prop, rna_enum_shader_attribute_type_items);
  RNA_def_property_ui_text(prop, "Attribute Type", "General type of the attribute");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update_relations");

  prop = RNA_def_property(srna, "attribute_name", PROP_STRING, PROP_NONE);
  RNA_def_property_string_sdna(prop, nullptr, "name");
  RNA_def_property_ui_text(prop, "Attribute Name", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNodeAttribute_name_update");

  prop = RNA_def_property(srna, "data_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "data_type");
  RNA_def_property_enum_items(prop, rna_enum_shader_attribute_data_type_items);
  RNA_def_property_enum_default(prop, SHD_ATTRIBUTE_DATA_FLOAT);
  RNA_def_property_enum_funcs(prop, nullptr, nullptr, "rna_NodeShaderAttribute_data_type_itemf");
  RNA_def_property_ui_text(prop, "Data Type", "Type of the attribute output");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");
}

static void def_sh_tex(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "texture_mapping", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "base.tex_mapping");
  RNA_def_property_flag(prop, PROP_NEVER_NULL);
  RNA_def_property_ui_text(prop, "Texture Mapping", "Texture coordinate mapping settings");

  prop = RNA_def_property(srna, "color_mapping", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "base.color_mapping");
  RNA_def_property_flag(prop, PROP_NEVER_NULL);
  RNA_def_property_ui_text(prop, "Color Mapping", "Color mapping settings");
}

static void def_sh_tex_sky(BlenderRNA *brna, StructRNA *srna)
{
  static const EnumPropertyItem prop_sky_type[] = {
      {SHD_SKY_SINGLE_SCATTERING,
       "SINGLE_SCATTERING",
       0,
       "Single Scattering",
       "Single scattering sky model"},
      {SHD_SKY_MULTIPLE_SCATTERING,
       "MULTIPLE_SCATTERING",
       0,
       "Multiple Scattering",
       "Multiple scattering sky model (more accurate)"},
      {SHD_SKY_PREETHAM, "PREETHAM", 0, "Preetham", "Preetham 1999 (Legacy)"},
      {SHD_SKY_HOSEK, "HOSEK_WILKIE", 0, "Hosek / Wilkie", "Hosek / Wilkie 2012 (Legacy)"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeTexSky", "storage");
  def_sh_tex(brna, srna);

  prop = RNA_def_property(srna, "sky_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "sky_model");
  RNA_def_property_enum_items(prop, prop_sky_type);
  RNA_def_property_enum_default(prop, SHD_SKY_MULTIPLE_SCATTERING);
  RNA_def_property_ui_text(prop, "Sky Type", "Which sky model should be used");
  RNA_def_property_update(prop, 0, "rna_ShaderNode_socket_update");

  /* Nishita parameters. */
  prop = RNA_def_property(srna, "sun_disc", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_ui_text(prop, "Sun Disc", "Include the sun itself in the output");
  RNA_def_property_boolean_sdna(prop, nullptr, "sun_disc", 1);
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_update(prop, 0, "rna_ShaderNode_socket_update");

  prop = RNA_def_property(srna, "sun_size", PROP_FLOAT, PROP_ANGLE);
  RNA_def_property_ui_text(prop, "Sun Size", "Size of sun disc");
  RNA_def_property_range(prop, 0.0f, M_PI_2);
  RNA_def_property_float_default(prop, DEG2RADF(0.545));
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "sun_intensity", PROP_FLOAT, PROP_NONE);
  RNA_def_property_ui_text(prop, "Sun Intensity", "Strength of Sun");
  RNA_def_property_range(prop, 0.0f, 1000.0f);
  RNA_def_property_float_default(prop, 1.0f);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "sun_elevation", PROP_FLOAT, PROP_ANGLE);
  RNA_def_property_ui_text(prop, "Sun Elevation", "Sun angle from horizon");
  RNA_def_property_float_default(prop, DEG2RADF(15.0f));
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "sun_rotation", PROP_FLOAT, PROP_ANGLE);
  RNA_def_property_ui_text(prop, "Sun Rotation", "Rotation of sun around zenith");
  RNA_def_property_float_default(prop, 0.0f);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "altitude", PROP_FLOAT, PROP_DISTANCE);
  RNA_def_property_ui_text(prop, "Altitude", "Height from sea level");
  RNA_def_property_range(prop, 0.0f, 100000.0f);
  RNA_def_property_ui_range(prop, 0.0f, 100000.0f, 10, 1);
  RNA_def_property_float_default(prop, 100.0f);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "air_density", PROP_FLOAT, PROP_NONE);
  RNA_def_property_ui_text(prop,
                           "Air",
                           "Density of air molecules.\n"
                           "0 means no air, 1 means urban city air");
  RNA_def_property_range(prop, 0.0f, 1000.0f);
  RNA_def_property_float_default(prop, 1.0f);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "aerosol_density", PROP_FLOAT, PROP_NONE);
  RNA_def_property_ui_text(prop,
                           "Aerosols",
                           "Density of dust, pollution and water droplets.\n"
                           "0 means no aerosols, 1 means urban city aerosols");
  RNA_def_property_range(prop, 0.0f, 1000.0f);
  RNA_def_property_float_default(prop, 1.0f);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "ozone_density", PROP_FLOAT, PROP_NONE);
  RNA_def_property_ui_text(prop,
                           "Ozone",
                           "Density of ozone layer.\n"
                           "0 means no ozone, 1 means urban city ozone");
  RNA_def_property_range(prop, 0.0f, 1000.0f);
  RNA_def_property_float_default(prop, 1.0f);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  /* Legacy parameters. */
  prop = RNA_def_property(srna, "sun_direction", PROP_FLOAT, PROP_DIRECTION);
  RNA_def_property_ui_text(prop, "Sun Direction", "Direction from where the sun is shining");
  RNA_def_property_array(prop, 3);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "turbidity", PROP_FLOAT, PROP_NONE);
  RNA_def_property_range(prop, 1.0f, 10.0f);
  RNA_def_property_float_default(prop, 2.2f);
  RNA_def_property_ui_range(prop, 1.0f, 10.0f, 10, 3);
  RNA_def_property_ui_text(prop, "Turbidity", "Atmospheric turbidity");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "ground_albedo", PROP_FLOAT, PROP_FACTOR);
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_float_default(prop, 0.3f);
  RNA_def_property_ui_text(
      prop, "Ground Albedo", "Ground color that is subtly reflected in the sky");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_sh_tex_environment(BlenderRNA *brna, StructRNA *srna)
{
  static const EnumPropertyItem prop_projection_items[] = {
      {SHD_PROJ_EQUIRECTANGULAR,
       "EQUIRECTANGULAR",
       0,
       "Equirectangular",
       "Equirectangular or latitude-longitude projection"},
      {SHD_PROJ_MIRROR_BALL,
       "MIRROR_BALL",
       0,
       "Mirror Ball",
       "Projection from an orthographic photo of a mirror ball"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  prop = RNA_def_property(srna, "image", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Image");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(prop, "Image", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_tex_image_update");
  RNA_def_property_pointer_funcs(
      prop, nullptr, nullptr, nullptr, "rna_Image_no_renderresult_or_viewer_poll");

  RNA_def_struct_sdna_from(srna, "NodeTexEnvironment", "storage");
  def_sh_tex(brna, srna);

  prop = RNA_def_property(srna, "projection", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, prop_projection_items);
  RNA_def_property_ui_text(prop, "Projection", "Projection of the input image");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "interpolation", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, sh_tex_prop_interpolation_items);
  RNA_def_property_ui_text(prop, "Interpolation", "Texture interpolation");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "image_user", PROP_POINTER, PROP_NONE);
  RNA_def_property_flag(prop, PROP_NEVER_NULL);
  RNA_def_property_pointer_sdna(prop, nullptr, "iuser");
  RNA_def_property_ui_text(
      prop,
      "Image User",
      "Parameters defining which layer, pass and frame of the image is displayed");
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void def_sh_tex_gabor(BlenderRNA *brna, StructRNA *srna)
{
  static const EnumPropertyItem prop_gabor_types[] = {
      {SHD_GABOR_TYPE_2D,
       "2D",
       0,
       "2D",
       "Use the 2D vector (X, Y) as input. The Z component is ignored."},
      {SHD_GABOR_TYPE_3D, "3D", 0, "3D", "Use the 3D vector (X, Y, Z) as input"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_struct_sdna_from(srna, "NodeTexGabor", "storage");
  def_sh_tex(brna, srna);

  PropertyRNA *prop;
  prop = RNA_def_property(srna, "gabor_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "type");
  RNA_def_property_enum_items(prop, prop_gabor_types);
  RNA_def_property_ui_text(prop, "Type", "The type of Gabor noise to evaluate");
  RNA_def_property_update(prop, 0, "rna_ShaderNode_socket_update");
}

static void def_sh_tex_image(BlenderRNA *brna, StructRNA *srna)
{
  static const EnumPropertyItem prop_projection_items[] = {
      {SHD_PROJ_FLAT,
       "FLAT",
       0,
       "Flat",
       "Image is projected flat using the X and Y coordinates of the texture vector"},
      {SHD_PROJ_BOX,
       "BOX",
       0,
       "Box",
       "Image is projected using different components for each side of the object space bounding "
       "box"},
      {SHD_PROJ_SPHERE,
       "SPHERE",
       0,
       "Sphere",
       "Image is projected spherically using the Z axis as central"},
      {SHD_PROJ_TUBE,
       "TUBE",
       0,
       "Tube",
       "Image is projected from the tube using the Z axis as central"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  prop = RNA_def_property(srna, "image", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Image");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(prop, "Image", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_tex_image_update");
  RNA_def_property_pointer_funcs(
      prop, nullptr, nullptr, nullptr, "rna_Image_no_renderresult_or_viewer_poll");

  RNA_def_struct_sdna_from(srna, "NodeTexImage", "storage");
  def_sh_tex(brna, srna);

  prop = RNA_def_property(srna, "projection", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, prop_projection_items);
  RNA_def_property_ui_text(
      prop, "Projection", "Method to project 2D image on object with a 3D texture vector");
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_ID_IMAGE);
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "interpolation", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, sh_tex_prop_interpolation_items);
  RNA_def_property_ui_text(prop, "Interpolation", "Texture interpolation");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "projection_blend", PROP_FLOAT, PROP_FACTOR);
  RNA_def_property_ui_text(
      prop, "Projection Blend", "For box projection, amount of blend to use between sides");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "extension", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, prop_image_extension);
  RNA_def_property_ui_text(
      prop, "Extension", "How the image is extrapolated past its original bounds");
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_ID_IMAGE);
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "image_user", PROP_POINTER, PROP_NONE);
  RNA_def_property_flag(prop, PROP_NEVER_NULL);
  RNA_def_property_pointer_sdna(prop, nullptr, "iuser");
  RNA_def_property_ui_text(
      prop,
      "Image User",
      "Parameters defining which layer, pass and frame of the image is displayed");
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void def_sh_tex_gradient(BlenderRNA *brna, StructRNA *srna)
{
  static const EnumPropertyItem prop_gradient_type[] = {
      {SHD_BLEND_LINEAR, "LINEAR", 0, "Linear", "Create a linear progression"},
      {SHD_BLEND_QUADRATIC, "QUADRATIC", 0, "Quadratic", "Create a quadratic progression"},
      {SHD_BLEND_EASING,
       "EASING",
       0,
       "Easing",
       "Create a progression easing from one step to the next"},
      {SHD_BLEND_DIAGONAL, "DIAGONAL", 0, "Diagonal", "Create a diagonal progression"},
      {SHD_BLEND_SPHERICAL, "SPHERICAL", 0, "Spherical", "Create a spherical progression"},
      {SHD_BLEND_QUADRATIC_SPHERE,
       "QUADRATIC_SPHERE",
       0,
       "Quadratic Sphere",
       "Create a quadratic progression in the shape of a sphere"},
      {SHD_BLEND_RADIAL, "RADIAL", 0, "Radial", "Create a radial progression"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeTexGradient", "storage");
  def_sh_tex(brna, srna);

  prop = RNA_def_property(srna, "gradient_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, prop_gradient_type);
  RNA_def_property_ui_text(prop, "Gradient Type", "Style of the color blending");
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void def_sh_tex_noise(BlenderRNA *brna, StructRNA *srna)
{
  static const EnumPropertyItem prop_noise_type[] = {
      {SHD_NOISE_MULTIFRACTAL,
       "MULTIFRACTAL",
       0,
       "Multifractal",
       "More uneven result (varies with location), more similar to a real terrain"},
      {SHD_NOISE_RIDGED_MULTIFRACTAL,
       "RIDGED_MULTIFRACTAL",
       0,
       "Ridged Multifractal",
       "Create sharp peaks"},
      {SHD_NOISE_HYBRID_MULTIFRACTAL,
       "HYBRID_MULTIFRACTAL",
       0,
       "Hybrid Multifractal",
       "Create peaks and valleys with different roughness values"},
      {SHD_NOISE_FBM, "FBM", 0, "fBM", "The standard fractal Perlin noise"},
      {SHD_NOISE_HETERO_TERRAIN,
       "HETERO_TERRAIN",
       0,
       "Hetero Terrain",
       "Similar to Hybrid Multifractal creates a heterogeneous terrain, but with the likeness of "
       "river channels"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeTexNoise", "storage");
  def_sh_tex(brna, srna);

  prop = RNA_def_property(srna, "noise_dimensions", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "dimensions");
  RNA_def_property_enum_items(prop, rna_enum_node_tex_dimensions_items);
  RNA_def_property_ui_text(prop, "Dimensions", "Number of dimensions to output noise for");
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_ID_TEXTURE);
  RNA_def_property_update(prop, 0, "rna_ShaderNode_socket_update");

  prop = RNA_def_property(srna, "noise_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "type");
  RNA_def_property_enum_items(prop, prop_noise_type);
  RNA_def_property_enum_default(prop, SHD_NOISE_FBM);
  RNA_def_property_ui_text(prop, "Type", "Type of the Noise texture");
  RNA_def_property_update(prop, 0, "rna_ShaderNode_socket_update");

  prop = RNA_def_property(srna, "normalize", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "normalize", 0);
  RNA_def_property_boolean_default(prop, true);
  RNA_def_property_ui_text(prop, "Normalize", "Normalize outputs to 0.0 to 1.0 range");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "tiling", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "tiling", 0);
  RNA_def_property_ui_text(
      prop,
      "Tiling",
      "Make the noise repeat seamlessly. Size is the integer wrap period of the already-scaled "
      "coordinate (input × Scale), minimum 2. Size 5 wraps a 5×5×5 cell lattice. Each octave's "
      "wrap is scaled by Lacunarity. Use integer Lacunarity for exact seams");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_sh_tex_checker(BlenderRNA *brna, StructRNA *srna)
{
  RNA_def_struct_sdna_from(srna, "NodeTexChecker", "storage");
  def_sh_tex(brna, srna);
}

static void def_sh_tex_brick(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeTexBrick", "storage");
  def_sh_tex(brna, srna);

  prop = RNA_def_property(srna, "offset_frequency", PROP_INT, PROP_NONE);
  RNA_def_property_int_sdna(prop, nullptr, "offset_freq");
  RNA_def_property_int_default(prop, 2);
  RNA_def_property_range(prop, 1, 99);
  RNA_def_property_ui_text(
      prop,
      "Offset Frequency",
      "How often rows are offset. A value of 2 gives an even/uneven pattern of rows.");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "squash_frequency", PROP_INT, PROP_NONE);
  RNA_def_property_int_sdna(prop, nullptr, "squash_freq");
  RNA_def_property_int_default(prop, 2);
  RNA_def_property_range(prop, 1, 99);
  RNA_def_property_ui_text(
      prop, "Squash Frequency", "How often rows consist of \"squished\" bricks");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "offset", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "offset");
  RNA_def_property_float_default(prop, 0.5f);
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_property_ui_text(
      prop, "Offset Amount", "Determines the brick offset of the various rows");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "squash", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "squash");
  RNA_def_property_float_default(prop, 1.0f);
  RNA_def_property_range(prop, 0.0f, 99.0f);
  RNA_def_property_ui_text(
      prop,
      "Squash Amount",
      "Factor to adjust the brick's width for particular rows determined by the Offset Frequency");
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void def_sh_tex_magic(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeTexMagic", "storage");
  def_sh_tex(brna, srna);

  prop = RNA_def_property(srna, "turbulence_depth", PROP_INT, PROP_NONE);
  RNA_def_property_int_sdna(prop, nullptr, "depth");
  RNA_def_property_int_default(prop, 2);
  RNA_def_property_range(prop, 0, 10);
  RNA_def_property_ui_text(prop, "Depth", "Level of detail in the added turbulent noise");
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void def_sh_tex_voronoi(BlenderRNA *brna, StructRNA *srna)
{
  static EnumPropertyItem prop_distance_items[] = {
      {SHD_VORONOI_EUCLIDEAN, "EUCLIDEAN", 0, "Euclidean", "Euclidean distance"},
      {SHD_VORONOI_MANHATTAN, "MANHATTAN", 0, "Manhattan", "Manhattan distance"},
      {SHD_VORONOI_CHEBYCHEV, "CHEBYCHEV", 0, "Chebychev", "Chebychev distance"},
      {SHD_VORONOI_MINKOWSKI, "MINKOWSKI", 0, "Minkowski", "Minkowski distance"},
      {0, nullptr, 0, nullptr, nullptr}};

  static EnumPropertyItem prop_feature_items[] = {
      {SHD_VORONOI_F1,
       "F1",
       0,
       "F1",
       "Computes the distance to the closest point as well as its position and color"},
      {SHD_VORONOI_F2,
       "F2",
       0,
       "F2",
       "Computes the distance to the second closest point as well as its position and color"},
      {SHD_VORONOI_SMOOTH_F1,
       "SMOOTH_F1",
       0,
       "Smooth F1",
       "Smoothed version of F1. Weighted sum of neighbor voronoi cells."},
      {SHD_VORONOI_DISTANCE_TO_EDGE,
       "DISTANCE_TO_EDGE",
       0,
       "Distance to Edge",
       "Computes the distance to the edge of the voronoi cell"},
      {SHD_VORONOI_N_SPHERE_RADIUS,
       "N_SPHERE_RADIUS",
       0,
       "N-Sphere Radius",
       "Computes the radius of the n-sphere inscribed in the voronoi cell"},
      {0, nullptr, 0, nullptr, nullptr}};

  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeTexVoronoi", "storage");
  def_sh_tex(brna, srna);

  prop = RNA_def_property(srna, "voronoi_dimensions", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "dimensions");
  RNA_def_property_enum_items(prop, rna_enum_node_tex_dimensions_items);
  RNA_def_property_ui_text(prop, "Dimensions", "Number of dimensions to output noise for");
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_ID_TEXTURE);
  RNA_def_property_update(prop, 0, "rna_ShaderNode_socket_update");

  prop = RNA_def_property(srna, "distance", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "distance");
  RNA_def_property_enum_items(prop, prop_distance_items);
  RNA_def_property_ui_text(
      prop, "Distance Metric", "The distance metric used to compute the texture");
  RNA_def_property_update(prop, 0, "rna_ShaderNode_socket_update");

  prop = RNA_def_property(srna, "feature", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "feature");
  RNA_def_property_enum_items(prop, prop_feature_items);
  RNA_def_property_ui_text(
      prop, "Feature Output", "The Voronoi feature that the node will compute");
  RNA_def_property_update(prop, 0, "rna_ShaderNode_socket_update");

  prop = RNA_def_property(srna, "normalize", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "normalize", 0);
  RNA_def_property_ui_text(prop, "Normalize", "Normalize output Distance to 0.0 to 1.0 range");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "tiling", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "tiling", 0);
  RNA_def_property_ui_text(
      prop,
      "Tiling",
      "Make Voronoi repeat seamlessly. Size is the integer wrap period of the already-scaled "
      "coordinate (input × Scale), minimum 2. Size 5 wraps a 5×5×5 cell lattice. Each octave's "
      "wrap is scaled by Lacunarity. Use integer Lacunarity for exact seams");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_sh_tex_wave(BlenderRNA *brna, StructRNA *srna)
{
  static const EnumPropertyItem prop_wave_type_items[] = {
      {SHD_WAVE_BANDS, "BANDS", 0, "Bands", "Use standard wave texture in bands"},
      {SHD_WAVE_RINGS, "RINGS", 0, "Rings", "Use wave texture in rings"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static EnumPropertyItem prop_wave_bands_direction_items[] = {
      {SHD_WAVE_BANDS_DIRECTION_X, "X", 0, "X", "Bands across X axis"},
      {SHD_WAVE_BANDS_DIRECTION_Y, "Y", 0, "Y", "Bands across Y axis"},
      {SHD_WAVE_BANDS_DIRECTION_Z, "Z", 0, "Z", "Bands across Z axis"},
      {SHD_WAVE_BANDS_DIRECTION_DIAGONAL, "DIAGONAL", 0, "Diagonal", "Bands across diagonal axis"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static EnumPropertyItem prop_wave_rings_direction_items[] = {
      {SHD_WAVE_RINGS_DIRECTION_X, "X", 0, "X", "Rings along X axis"},
      {SHD_WAVE_RINGS_DIRECTION_Y, "Y", 0, "Y", "Rings along Y axis"},
      {SHD_WAVE_RINGS_DIRECTION_Z, "Z", 0, "Z", "Rings along Z axis"},
      {SHD_WAVE_RINGS_DIRECTION_SPHERICAL,
       "SPHERICAL",
       0,
       "Spherical",
       "Rings along spherical distance"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem prop_wave_profile_items[] = {
      {SHD_WAVE_PROFILE_SIN, "SIN", 0, "Sine", "Use a standard sine profile"},
      {SHD_WAVE_PROFILE_SAW, "SAW", 0, "Saw", "Use a sawtooth profile"},
      {SHD_WAVE_PROFILE_TRI, "TRI", 0, "Triangle", "Use a triangle profile"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeTexWave", "storage");
  def_sh_tex(brna, srna);

  prop = RNA_def_property(srna, "wave_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "wave_type");
  RNA_def_property_enum_items(prop, prop_wave_type_items);
  RNA_def_property_ui_text(prop, "Wave Type", "");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "bands_direction", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "bands_direction");
  RNA_def_property_enum_items(prop, prop_wave_bands_direction_items);
  RNA_def_property_ui_text(prop, "Bands Direction", "");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "rings_direction", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "rings_direction");
  RNA_def_property_enum_items(prop, prop_wave_rings_direction_items);
  RNA_def_property_ui_text(prop, "Rings Direction", "");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "wave_profile", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "wave_profile");
  RNA_def_property_enum_items(prop, prop_wave_profile_items);
  RNA_def_property_ui_text(prop, "Wave Profile", "");
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void def_sh_tex_white_noise(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "noise_dimensions", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, rna_enum_node_tex_dimensions_items);
  RNA_def_property_ui_text(prop, "Dimensions", "Number of dimensions to output noise for");
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_ID_TEXTURE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_sh_object_transform(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop = RNA_def_property(srna, "object", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Object");
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_ID_REFCOUNT);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(prop, "Object", "Object whose world-space transform is retrieved");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update_relations");
}

static void def_sh_tex_coord(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "object", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Object");
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_ID_REFCOUNT);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(
      prop, "Object", "Use coordinates from this object (for object texture coordinates output)");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update_relations");

  prop = RNA_def_property(srna, "from_instancer", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "custom1", 1);
  RNA_def_property_ui_text(
      prop, "From Instancer", "Use the parent of the instance object if possible");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_sh_vect_transform(BlenderRNA * /*brna*/, StructRNA *srna)
{
  static const EnumPropertyItem prop_vect_type_items[] = {
      {SHD_VECT_TRANSFORM_TYPE_POINT, "POINT", 0, "Point", "Transform a point"},
      {SHD_VECT_TRANSFORM_TYPE_VECTOR, "VECTOR", 0, "Vector", "Transform a direction vector"},
      {SHD_VECT_TRANSFORM_TYPE_NORMAL,
       "NORMAL",
       0,
       "Normal",
       "Transform a normal vector with unit length"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeShaderVectTransform", "storage");

  prop = RNA_def_property(srna, "vector_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "type");
  RNA_def_property_enum_items(prop, prop_vect_type_items);
  RNA_def_property_ui_text(prop, "Type", "");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "convert_from", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_shader_vect_transform_space_items);
  RNA_def_property_enum_funcs(prop, nullptr, nullptr, "rna_NodeShaderVectTransform_space_itemf");
  RNA_def_property_ui_text(prop, "Convert From", "Space to convert from");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "convert_to", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_shader_vect_transform_space_items);
  RNA_def_property_enum_default(prop, SHD_VECT_TRANSFORM_SPACE_OBJECT);
  RNA_def_property_enum_funcs(prop, nullptr, nullptr, "rna_NodeShaderVectTransform_space_itemf");
  RNA_def_property_ui_text(prop, "Convert To", "Space to convert to");
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void def_sh_tex_wireframe(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "use_pixel_size", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "custom1", 1);
  RNA_def_property_ui_text(prop, "Pixel Size", "Use screen pixel size instead of world units");
  RNA_def_property_translation_context(prop, BLT_I18NCONTEXT_UNIT);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_metallic(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "distribution", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, node_metallic_distribution_items);
  RNA_def_property_enum_default(prop, SHD_GLOSSY_MULTI_GGX);
  RNA_def_property_ui_text(prop, "Distribution", "Light scattering distribution on rough surface");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "fresnel_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom2");
  RNA_def_property_enum_items(prop, node_metallic_fresnel_type_items);
  RNA_def_property_enum_default(prop, SHD_CONDUCTOR_F82);
  RNA_def_property_ui_text(prop, "Fresnel Type", "Fresnel method used to tint the metal");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_glossy(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "distribution", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, node_glossy_items);
  RNA_def_property_enum_default(prop, SHD_GLOSSY_MULTI_GGX);
  RNA_def_property_ui_text(prop, "Distribution", "Light scattering distribution on rough surface");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_glass(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "distribution", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, node_glass_items);
  RNA_def_property_enum_default(prop, SHD_GLOSSY_MULTI_GGX);
  RNA_def_property_ui_text(prop, "Distribution", "Light scattering distribution on rough surface");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_sheen(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "distribution", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, node_sheen_items);
  RNA_def_property_enum_default(prop, SHD_SHEEN_MICROFIBER);
  RNA_def_property_ui_text(prop, "Distribution", "Sheen shading model");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_principled(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "distribution", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, node_principled_distribution_items);
  RNA_def_property_enum_default(prop, SHD_GLOSSY_MULTI_GGX);
  RNA_def_property_ui_text(prop, "Distribution", "Light scattering distribution on rough surface");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");

  prop = RNA_def_property(srna, "subsurface_method", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom2");
  RNA_def_property_enum_items(prop, node_subsurface_method_items);
  RNA_def_property_enum_default(prop, SHD_SUBSURFACE_RANDOM_WALK);
  RNA_def_property_ui_text(
      prop, "Subsurface Method", "Method for rendering subsurface scattering");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_light_evaluation(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, node_light_evaluation_mode_items);
  RNA_def_property_ui_text(prop, "Mode", "Defines parametrization of the light evaluation");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_refraction(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "distribution", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, node_refraction_items);
  RNA_def_property_ui_text(prop, "Distribution", "Light scattering distribution on rough surface");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_scatter(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "phase", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, node_scatter_phase_items);
  RNA_def_property_ui_text(prop, "Phase", "Phase function for the scattered light");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_volume_coefficients(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "phase", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, node_scatter_phase_items);
  RNA_def_property_ui_text(prop, "Phase", "Phase function for the scattered light");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_toon(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "component", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, node_toon_items);
  RNA_def_property_ui_text(prop, "Component", "Toon BSDF component to use");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_sh_bump(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "invert", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "custom1", 1);
  RNA_def_property_ui_text(
      prop, "Invert", "Invert the bump mapping direction to push into the surface instead of out");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_hair(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "component", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, node_hair_items);
  RNA_def_property_ui_text(prop, "Component", "Hair BSDF component to use");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

/* RNA initialization for the custom properties. */
static void def_hair_principled(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeShaderHairPrincipled", "storage");

  prop = RNA_def_property(srna, "model", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "model");
  RNA_def_property_ui_text(prop, "Scattering model", "Select from Chiang or Huang model");
  RNA_def_property_enum_items(prop, node_principled_hair_model_items);
  RNA_def_property_enum_default(prop, SHD_PRINCIPLED_HAIR_CHIANG);
  /* Upon editing, update both the node data AND the UI representation */
  /* (This effectively shows/hides the relevant sockets) */
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");

  prop = RNA_def_property(srna, "parametrization", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "parametrization");
  RNA_def_property_ui_text(
      prop, "Color Parametrization", "Select the shader's color parametrization");
  RNA_def_property_enum_items(prop, node_principled_hair_parametrization_items);
  RNA_def_property_enum_default(prop, SHD_PRINCIPLED_HAIR_REFLECTANCE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_sh_uvmap(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "from_instancer", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "custom1", 1);
  RNA_def_property_ui_text(
      prop, "From Instancer", "Use the parent of the instance object if possible");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  RNA_def_struct_sdna_from(srna, "NodeShaderUVMap", "storage");

  prop = RNA_def_property(srna, "uv_map", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "UV Map", "UV coordinates to be used for mapping");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  RNA_def_struct_sdna_from(srna, "bNode", nullptr);
}

static void def_sh_vertex_color(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeShaderVertexColor", "storage");

  prop = RNA_def_property(srna, "layer_name", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Color Attribute", "Color Attribute");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  RNA_def_struct_sdna_from(srna, "bNode", nullptr);
}

static void def_sh_uvalongstroke(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "use_tips", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "custom1", 1);
  RNA_def_property_ui_text(
      prop, "Use Tips", "Lower half of the texture is for tips of the stroke");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_sh_normal_map(BlenderRNA * /*brna*/, StructRNA *srna)
{
  static const EnumPropertyItem prop_space_items[] = {
      {SHD_SPACE_TANGENT, "TANGENT", 0, "Tangent Space", "Tangent space normal mapping"},
      {SHD_SPACE_OBJECT, "OBJECT", 0, "Object Space", "Object space normal mapping"},
      {SHD_SPACE_WORLD, "WORLD", 0, "World Space", "World space normal mapping"},
      {SHD_SPACE_BLENDER_OBJECT,
       "BLENDER_OBJECT",
       0,
       "Blender Object Space",
       "Object space normal mapping, compatible with Blender render baking"},
      {SHD_SPACE_BLENDER_WORLD,
       "BLENDER_WORLD",
       0,
       "Blender World Space",
       "World space normal mapping, compatible with Blender render baking"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem prop_convention_items[] = {
      {SHD_NORMAL_MAP_CONVENTION_OPENGL,
       "OPENGL",
       0,
       "OpenGL",
       "Normal map uses OpenGL convention, with Y axis in the green channel pointing up"},
      {SHD_NORMAL_MAP_CONVENTION_DIRECTX,
       "DIRECTX",
       0,
       "DirectX",
       "Normal map uses DirectX convention, with Y axis in the green channel pointing down"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeShaderNormalMap", "storage");

  prop = RNA_def_property(srna, "space", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, prop_space_items);
  RNA_def_property_ui_text(prop, "Space", "Space of the input normal");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "uv_map", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "UV Map", "UV Map for tangent space maps");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "convention", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, prop_convention_items);
  RNA_def_property_ui_text(prop, "Mode", "OpenGL or DirectX");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  static const EnumPropertyItem prop_base_items[] = {
      {SHD_NORMAL_MAP_BASE_ORIGINAL,
       "ORIGINAL",
       0,
       "Original Base",
       "Apply normal map relative to the smooth undisplaced surface, for normal maps that match "
       "the displacement map"},
      {SHD_NORMAL_MAP_BASE_DISPLACED,
       "DISPLACED",
       0,
       "Displaced Base",
       "Apply normal map relative to the displaced surface, for normal maps that are to be "
       "applied in addition to the displacement map"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  prop = RNA_def_property(srna, "base", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, prop_base_items);
  RNA_def_property_ui_text(
      prop,
      "Base",
      "Base surface that the normal map is applied relative to, when displacement is used");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  RNA_def_struct_sdna_from(srna, "bNode", nullptr);
}

static void def_sh_radial_tiling(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeRadialTiling", "storage");

  prop = RNA_def_property(srna, "normalize", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "normalize", 0);
  RNA_def_property_ui_text(
      prop,
      "Normalize",
      "Normalize the X coordinate of the Segment Coordinates output to a [0, 1] interval and "
      "offset the Y coordinate into a [0, infinity) interval. When checked, the textures are "
      "stretched to fit into each angular segment. When not checked, the parts of the textures "
      "that don't fit into each angular segment are cropped");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void rna_def_sh_raycast_sample_attribute_item(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeRaycastSampleAttributeItem", nullptr);
  RNA_def_struct_ui_text(srna, "Raycast Sample Attribute Item", "");
  RNA_def_struct_sdna(srna, "NodeRaycastSampleAttributeItem");

  rna_def_node_item_array_socket_item_common(srna, "RaycastSampleAttributeItemsAccessor", false);
  PropertyRNA *prop = RNA_def_property(srna, "data_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_attribute_type_items);
  RNA_def_property_enum_funcs(
      prop, nullptr, nullptr, "rna_NodeRaycastSampleAttributeItem_data_type_itemf");
  RNA_def_property_ui_text(prop, "Data Type", "");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop,
                          NC_NODE | NA_EDITED,
                          "rna_Node_ItemArray_item_update<RaycastSampleAttributeItemsAccessor>");
}

static void rna_def_sh_raycast_sample_attribute_items(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeRaycastSampleAttributeItems", nullptr);
  RNA_def_struct_ui_text(srna, "Items", "Collection of capture attribute items");
  RNA_def_struct_sdna(srna, "bNode");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "NodeRaycastSampleAttributeItem", "RaycastSampleAttributeItemsAccessor");
  rna_def_node_item_array_common_functions(
      srna, "NodeRaycastSampleAttributeItem", "RaycastSampleAttributeItemsAccessor");
}

static void def_sh_raycast(BlenderRNA *brna, StructRNA *srna)
{
  rna_def_sh_raycast_sample_attribute_item(brna);
  rna_def_sh_raycast_sample_attribute_items(brna);

  PropertyRNA *prop = RNA_def_property(srna, "only_local", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "custom1", 0);
  RNA_def_property_ui_text(prop, "Only Local", "Only raycast against the object itself");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  RNA_def_struct_sdna_from(srna, "NodeShaderRaycast", "storage");

  prop = RNA_def_property(srna, "sample_attribute_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(
      prop, nullptr, "sample_attribute_items", "sample_attribute_items_num");
  RNA_def_property_struct_type(prop, "NodeRaycastSampleAttributeItem");
  RNA_def_property_ui_text(prop, "Items", "");
  RNA_def_property_srna(prop, "NodeRaycastSampleAttributeItems");

  prop = RNA_def_property(srna, "active_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "active_index");
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_item", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "NodeRaycastSampleAttributeItem");
  RNA_def_property_pointer_funcs(
      prop,
      "rna_Node_ItemArray_active_get<RaycastSampleAttributeItemsAccessor>",
      "rna_Node_ItemArray_active_set<RaycastSampleAttributeItemsAccessor>",
      nullptr,
      nullptr);
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_NO_DEG_UPDATE);
  RNA_def_property_ui_text(prop, "Active Item Index", "Index of the active item");
  RNA_def_property_update(prop, NC_NODE, nullptr);
}

static void def_sh_billboard_displacement(BlenderRNA * /*brna*/, StructRNA *srna)
{
  static const EnumPropertyItem prop_mode_items[] = {
      {NODE_BILLBOARD_VIEW,
       "VIEW",
       0,
       "View",
       "Screen-aligned including camera roll. Axis chooses which object axis faces the viewer"},
      {NODE_BILLBOARD_SPHERICAL,
       "SPHERICAL",
       0,
       "Spherical",
       "Look at the camera; keep Up as close to world up as possible"},
      {NODE_BILLBOARD_CYLINDRICAL,
       "CYLINDRICAL",
       0,
       "Cylindrical",
       "Yaw only around Up (typical for trees)"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem prop_axis_items[] = {
      {NODE_BILLBOARD_AXIS_X, "X", 0, "X", "Object +X faces the viewer; +Z stays camera-up"},
      {NODE_BILLBOARD_AXIS_Y,
       "Y",
       0,
       "Y",
       "Object +Y faces the view direction; object +X matches view +X, +Z matches view +Y"},
      {NODE_BILLBOARD_AXIS_Z,
       "Z",
       0,
       "Z",
       "Object +Z faces the viewer (default plane); +Y stays camera-up"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  prop = RNA_def_property(srna, "mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, prop_mode_items);
  RNA_def_property_ui_text(prop, "Mode", "How the object is rotated to face the view");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");

  prop = RNA_def_property(srna, "axis", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom2");
  RNA_def_property_enum_items(prop, prop_axis_items);
  RNA_def_property_ui_text(prop, "Axis", "Object-local axis that should face the viewer");
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void def_sh_displacement(BlenderRNA * /*brna*/, StructRNA *srna)
{
  static const EnumPropertyItem prop_space_items[] = {
      {SHD_SPACE_OBJECT,
       "OBJECT",
       0,
       "Object Space",
       "Displacement is in object space, affected by object scale"},
      {SHD_SPACE_WORLD,
       "WORLD",
       0,
       "World Space",
       "Displacement is in world space, not affected by object scale"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  prop = RNA_def_property(srna, "space", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, prop_space_items);
  RNA_def_property_ui_text(prop, "Space", "Space of the input height");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  RNA_def_struct_sdna_from(srna, "bNode", nullptr);
}

static void def_sh_vector_displacement(BlenderRNA * /*brna*/, StructRNA *srna)
{
  static const EnumPropertyItem prop_space_items[] = {
      {SHD_SPACE_TANGENT,
       "TANGENT",
       0,
       "Tangent Space",
       "Tangent space vector displacement mapping"},
      {SHD_SPACE_OBJECT, "OBJECT", 0, "Object Space", "Object space vector displacement mapping"},
      {SHD_SPACE_WORLD, "WORLD", 0, "World Space", "World space vector displacement mapping"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  prop = RNA_def_property(srna, "space", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, prop_space_items);
  RNA_def_property_ui_text(prop, "Space", "Space of the input height");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  RNA_def_struct_sdna_from(srna, "bNode", nullptr);
}

static void def_sh_tangent(BlenderRNA * /*brna*/, StructRNA *srna)
{
  static const EnumPropertyItem prop_direction_type_items[] = {
      {SHD_TANGENT_RADIAL, "RADIAL", 0, "Radial", "Radial tangent around the X, Y or Z axis"},
      {SHD_TANGENT_UVMAP, "UV_MAP", 0, "UV Map", "Tangent from UV map"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem prop_axis_items[] = {
      {SHD_TANGENT_AXIS_X, "X", 0, "X", "X axis"},
      {SHD_TANGENT_AXIS_Y, "Y", 0, "Y", "Y axis"},
      {SHD_TANGENT_AXIS_Z, "Z", 0, "Z", "Z axis"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeShaderTangent", "storage");

  prop = RNA_def_property(srna, "direction_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, prop_direction_type_items);
  RNA_def_property_ui_text(prop, "Direction", "Method to use for the tangent");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "axis", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, prop_axis_items);
  RNA_def_property_enum_default(prop, SHD_TANGENT_AXIS_Z);
  RNA_def_property_ui_text(prop, "Axis", "Axis for radial tangents");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "uv_map", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "UV Map", "UV Map for tangent generated from UV");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  RNA_def_struct_sdna_from(srna, "bNode", nullptr);
}

static void def_sh_bevel(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "samples", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "custom1");
  RNA_def_property_range(prop, 2, 128);
  RNA_def_property_ui_range(prop, 2, 16, 1, 1);
  RNA_def_property_int_default(prop, 4);
  RNA_def_property_ui_text(prop, "Samples", "Number of rays to trace per shader evaluation");
  RNA_def_property_update(prop, 0, "rna_Node_update");
}

static void def_sh_ambient_occlusion(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "samples", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "custom1");
  RNA_def_property_range(prop, 1, 128);
  RNA_def_property_int_default(prop, 16);
  RNA_def_property_ui_text(prop, "Samples", "Number of rays to trace per shader evaluation");
  RNA_def_property_update(prop, 0, "rna_Node_update");

  prop = RNA_def_property(srna, "inside", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "custom2", SHD_AO_INSIDE);
  RNA_def_property_ui_text(prop, "Inside", "Trace rays towards the inside of the object");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "only_local", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "custom2", SHD_AO_LOCAL);
  RNA_def_property_ui_text(
      prop, "Only Local", "Only consider the object itself when computing AO");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void def_sh_subsurface(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "falloff", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "custom1");
  RNA_def_property_enum_items(prop, node_subsurface_method_items);
  RNA_def_property_enum_default(prop, SHD_SUBSURFACE_RANDOM_WALK);
  RNA_def_property_ui_text(prop, "Method", "Method for rendering subsurface scattering");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNode_socket_update");
}

static void def_sh_tex_ies(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "ies", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Text");
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_ID_REFCOUNT);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(prop, "IES Text", "Internal IES file");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  RNA_def_struct_sdna_from(srna, "NodeShaderTexIES", "storage");

  prop = RNA_def_property(srna, "filepath", PROP_STRING, PROP_FILEPATH);
  RNA_def_property_ui_text(prop, "File Path", "IES light path");
  RNA_def_property_flag(prop, PROP_PATH_SUPPORTS_BLEND_RELATIVE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_funcs(prop, nullptr, "rna_ShaderNodeTexIES_mode_set", nullptr);
  RNA_def_property_enum_items(prop, node_ies_mode_items);
  RNA_def_property_ui_text(
      prop, "Source", "Whether the IES file is loaded from disk or from a text data-block");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  RNA_def_struct_sdna_from(srna, "bNode", nullptr);
}

static void def_sh_output_aov(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeShaderOutputAOV", "storage");

  prop = RNA_def_property(srna, "aov_name", PROP_STRING, PROP_NONE);
  RNA_def_property_string_sdna(prop, nullptr, "name");
  RNA_def_property_ui_text(prop, "Name", "Name of the AOV that this output writes to");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  RNA_def_struct_sdna_from(srna, "bNode", nullptr);
}

static void def_sh_combsep_color(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeCombSepColor", "storage");

  prop = RNA_def_property(srna, "mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_enum_node_combsep_color_items);
  RNA_def_property_ui_text(prop, "Mode", "Mode of color processing");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_socket_update");
}

static void rna_def_sh_hlsl_input_item(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeShaderHLSLInputItem", nullptr);
  RNA_def_struct_ui_text(srna, "GLSL Input Parameter", "");
  RNA_def_struct_sdna(srna, "NodeShaderHLSLItem");

  rna_def_node_item_array_socket_item_common(srna, "HLSLInputItemsAccessor", true);
}

static void rna_def_sh_hlsl_input_items(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeShaderHLSLInputItems", nullptr);
  RNA_def_struct_ui_text(srna, "Input Parameters", "");
  RNA_def_struct_sdna(srna, "bNode");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "NodeShaderHLSLInputItem", "HLSLInputItemsAccessor");
  rna_def_node_item_array_common_functions(
      srna, "NodeShaderHLSLInputItem", "HLSLInputItemsAccessor");
}

static void rna_def_sh_hlsl_output_item(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeShaderHLSLOutputItem", nullptr);
  RNA_def_struct_ui_text(srna, "GLSL Output Parameter", "");
  RNA_def_struct_sdna(srna, "NodeShaderHLSLItem");

  rna_def_node_item_array_socket_item_common(srna, "HLSLOutputItemsAccessor", true);
}

static void rna_def_sh_hlsl_output_items(BlenderRNA *brna)
{
  StructRNA *srna = RNA_def_struct(brna, "NodeShaderHLSLOutputItems", nullptr);
  RNA_def_struct_ui_text(srna, "Output Parameters", "");
  RNA_def_struct_sdna(srna, "bNode");

  rna_def_node_item_array_new_with_socket_and_name(
      srna, "NodeShaderHLSLOutputItem", "HLSLOutputItemsAccessor");
  rna_def_node_item_array_common_functions(
      srna, "NodeShaderHLSLOutputItem", "HLSLOutputItemsAccessor");
}

void def_geo_python(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  RNA_def_struct_sdna_from(srna, "NodeGeometryPython", "storage");

  prop = RNA_def_property(srna, "code", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Code", "Python source code run when the node evaluates");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "textbox_state", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "TextboxState");
  RNA_def_property_pointer_sdna(prop, nullptr, "textbox_state");
}

void def_geo_wrangle(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  static const EnumPropertyItem rna_node_geo_wrangle_domain_items[] = {
      {GEO_NODE_WRANGLE_POINT, "POINT", 0, "Point", "Run the script once per point"},
      {GEO_NODE_WRANGLE_EDGE, "EDGE", 0, "Edge", "Run the script once per mesh edge"},
      {GEO_NODE_WRANGLE_FACE, "FACE", 0, "Face", "Run the script once per mesh face"},
      {GEO_NODE_WRANGLE_CORNER, "CORNER", 0, "Corner", "Run the script once per face corner"},
      {GEO_NODE_WRANGLE_CURVE, "CURVE", 0, "Curve", "Run the script once per curve/spline"},
      {GEO_NODE_WRANGLE_INSTANCE, "INSTANCE", 0, "Instance", "Run the script once per instance"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  {
    StructRNA *item_srna = RNA_def_struct(brna, "NodeGeometryWrangleInputItem", nullptr);
    RNA_def_struct_ui_text(item_srna, "Wrangle Input Item", "");
    RNA_def_struct_sdna(item_srna, "NodeGeometryWrangleInputItem");
    rna_def_node_item_array_socket_item_common(item_srna, "WrangleInputItemsAccessor", true);
  }
  {
    StructRNA *items_srna = RNA_def_struct(brna, "NodeGeometryWrangleInputItems", nullptr);
    RNA_def_struct_sdna(items_srna, "bNode");
    RNA_def_struct_ui_text(
        items_srna, "Input Items", "Wrangle inputs: extra geometry and compiled constants");
    rna_def_node_item_array_new_with_socket_and_name(
        items_srna, "NodeGeometryWrangleInputItem", "WrangleInputItemsAccessor");
    rna_def_node_item_array_common_functions(
        items_srna, "NodeGeometryWrangleInputItem", "WrangleInputItemsAccessor");
  }

  RNA_def_struct_sdna_from(srna, "NodeGeometryWrangle", "storage");

  prop = RNA_def_property(srna, "code", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(
      prop,
      "Code",
      "VEX-like source. Typing edits a draft (does not recook). Ctrl+Enter, click outside, or "
      "Compile commits and evaluates");
  /* Draft is saved while typing, but must not recook the node tree. Evaluation reads code_run. */
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE | PROP_NO_DEG_UPDATE);

  prop = RNA_def_property(srna, "textbox_state", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "TextboxState");
  RNA_def_property_pointer_sdna(prop, nullptr, "textbox_state");

  prop = RNA_def_property(srna, "domain", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_items(prop, rna_node_geo_wrangle_domain_items);
  RNA_def_property_ui_text(prop, "Domain", "Which elements the script runs over");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "input_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "input_items.items", "input_items.items_num");
  RNA_def_property_struct_type(prop, "NodeGeometryWrangleInputItem");
  RNA_def_property_srna(prop, "NodeGeometryWrangleInputItems");

  prop = RNA_def_property(srna, "active_input_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "input_items.active_index");
  RNA_def_property_ui_text(prop, "Active Input Index", "Index of the active channel parameter");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);
}

static void def_sh_hlsl(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  rna_def_sh_hlsl_input_item(brna);
  rna_def_sh_hlsl_input_items(brna);
  rna_def_sh_hlsl_output_item(brna);
  rna_def_sh_hlsl_output_items(brna);

  RNA_def_struct_sdna_from(srna, "NodeShaderHLSL", "storage");

  prop = RNA_def_property(srna, "code", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Code", "GLSL source code");
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "textbox_state", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "TextboxState");
  RNA_def_property_pointer_sdna(prop, nullptr, "textbox_state");

  prop = RNA_def_property(srna, "input_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "input_items.items", "input_items.items_num");
  RNA_def_property_struct_type(prop, "NodeShaderHLSLInputItem");
  RNA_def_property_ui_text(prop, "Input Parameters", "User-defined input parameters");
  RNA_def_property_srna(prop, "NodeShaderHLSLInputItems");

  prop = RNA_def_property(srna, "output_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "output_items.items", "output_items.items_num");
  RNA_def_property_struct_type(prop, "NodeShaderHLSLOutputItem");
  RNA_def_property_ui_text(prop, "Output Parameters", "User-defined output parameters");
  RNA_def_property_srna(prop, "NodeShaderHLSLOutputItems");

  prop = RNA_def_property(srna, "active_input_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "input_items.active_index");
  RNA_def_property_ui_text(prop, "Active Input Index", "Index of the active input parameter");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_output_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "output_items.active_index");
  RNA_def_property_ui_text(prop, "Active Output Index", "Index of the active output parameter");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);
}

static void def_sh_wrangle(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  {
    StructRNA *item_srna = RNA_def_struct(brna, "NodeShaderWrangleInputItem", nullptr);
    RNA_def_struct_ui_text(item_srna, "Wrangle Input Item", "");
    RNA_def_struct_sdna(item_srna, "NodeShaderWrangleItem");
    rna_def_node_item_array_socket_item_common(item_srna, "ShaderWrangleInputItemsAccessor", true);
  }
  {
    StructRNA *items_srna = RNA_def_struct(brna, "NodeShaderWrangleInputItems", nullptr);
    RNA_def_struct_sdna(items_srna, "bNode");
    RNA_def_struct_ui_text(
        items_srna, "Input Items", "Wrangle inputs: compiled constants and fields");
    rna_def_node_item_array_new_with_socket_and_name(
        items_srna, "NodeShaderWrangleInputItem", "ShaderWrangleInputItemsAccessor");
    rna_def_node_item_array_common_functions(
        items_srna, "NodeShaderWrangleInputItem", "ShaderWrangleInputItemsAccessor");
  }
  {
    StructRNA *item_srna = RNA_def_struct(brna, "NodeShaderWrangleOutputItem", nullptr);
    RNA_def_struct_ui_text(item_srna, "Wrangle Output Item", "");
    RNA_def_struct_sdna(item_srna, "NodeShaderWrangleItem");
    rna_def_node_item_array_socket_item_common(item_srna, "ShaderWrangleOutputItemsAccessor", true);
  }
  {
    StructRNA *items_srna = RNA_def_struct(brna, "NodeShaderWrangleOutputItems", nullptr);
    RNA_def_struct_sdna(items_srna, "bNode");
    RNA_def_struct_ui_text(
        items_srna, "Output Items", "Wrangle outputs: assign the socket name in VEX");
    rna_def_node_item_array_new_with_socket_and_name(
        items_srna, "NodeShaderWrangleOutputItem", "ShaderWrangleOutputItemsAccessor");
    rna_def_node_item_array_common_functions(
        items_srna, "NodeShaderWrangleOutputItem", "ShaderWrangleOutputItemsAccessor");
  }

  RNA_def_struct_sdna_from(srna, "NodeShaderWrangle", "storage");

  prop = RNA_def_property(srna, "code", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(
      prop,
      "Code",
      "VEX source compiled to an EEVEE material function. Typing edits a draft (does not recook). "
      "Ctrl+Enter, click outside, or Compile commits and evaluates");
  /* Draft is saved while typing, but must not recook the material. Evaluation reads code_run. */
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE | PROP_NO_DEG_UPDATE);

  prop = RNA_def_property(srna, "textbox_state", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "TextboxState");
  RNA_def_property_pointer_sdna(prop, nullptr, "textbox_state");

  prop = RNA_def_property(srna, "input_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "input_items.items", "input_items.items_num");
  RNA_def_property_struct_type(prop, "NodeShaderWrangleInputItem");
  RNA_def_property_srna(prop, "NodeShaderWrangleInputItems");

  prop = RNA_def_property(srna, "output_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "output_items.items", "output_items.items_num");
  RNA_def_property_struct_type(prop, "NodeShaderWrangleOutputItem");
  RNA_def_property_ui_text(
      prop, "Output Items", "Named outputs written in VEX (`Output = …`)");
  RNA_def_property_srna(prop, "NodeShaderWrangleOutputItems");

  prop = RNA_def_property(srna, "active_input_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "input_items.active_index");
  RNA_def_property_ui_text(prop, "Active Input Index", "Index of the active channel parameter");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_output_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "output_items.active_index");
  RNA_def_property_ui_text(prop, "Active Output Index", "Index of the active output");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);
}

/* Same data as the Shader Wrangle, with item types of its own: an item finds its node through
 * the accessor, which names the node type. */
static void def_image_wrangle(BlenderRNA *brna, StructRNA *srna)
{
  PropertyRNA *prop;

  {
    StructRNA *item_srna = RNA_def_struct(brna, "NodeImageWrangleInputItem", nullptr);
    RNA_def_struct_ui_text(item_srna, "Image Wrangle Input Item", "");
    RNA_def_struct_sdna(item_srna, "NodeShaderWrangleItem");
    rna_def_node_item_array_socket_item_common(item_srna, "ImageWrangleInputItemsAccessor", true);
  }
  {
    StructRNA *items_srna = RNA_def_struct(brna, "NodeImageWrangleInputItems", nullptr);
    RNA_def_struct_sdna(items_srna, "bNode");
    RNA_def_struct_ui_text(
        items_srna, "Input Items", "Wrangle inputs: compiled constants and images");
    rna_def_node_item_array_new_with_socket_and_name(
        items_srna, "NodeImageWrangleInputItem", "ImageWrangleInputItemsAccessor");
    rna_def_node_item_array_common_functions(
        items_srna, "NodeImageWrangleInputItem", "ImageWrangleInputItemsAccessor");
  }
  {
    StructRNA *item_srna = RNA_def_struct(brna, "NodeImageWrangleOutputItem", nullptr);
    RNA_def_struct_ui_text(item_srna, "Image Wrangle Output Item", "");
    RNA_def_struct_sdna(item_srna, "NodeShaderWrangleItem");
    rna_def_node_item_array_socket_item_common(item_srna, "ImageWrangleOutputItemsAccessor", true);
  }
  {
    StructRNA *items_srna = RNA_def_struct(brna, "NodeImageWrangleOutputItems", nullptr);
    RNA_def_struct_sdna(items_srna, "bNode");
    RNA_def_struct_ui_text(
        items_srna, "Output Items", "Wrangle outputs: assign the socket name in the code");
    rna_def_node_item_array_new_with_socket_and_name(
        items_srna, "NodeImageWrangleOutputItem", "ImageWrangleOutputItemsAccessor");
    rna_def_node_item_array_common_functions(
        items_srna, "NodeImageWrangleOutputItem", "ImageWrangleOutputItemsAccessor");
  }

  RNA_def_struct_sdna_from(srna, "NodeShaderWrangle", "storage");

  prop = RNA_def_property(srna, "code", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(
      prop,
      "Code",
      "VEX source that runs for every pixel. Typing edits a draft (does not recook). "
      "Ctrl+Enter, click outside, or Compile commits and evaluates");
  /* The draft is saved while typing, evaluation reads code_run. */
  RNA_def_property_flag(prop, PROP_TEXTEDIT_UPDATE | PROP_NO_DEG_UPDATE);

  prop = RNA_def_property(srna, "textbox_state", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "TextboxState");
  RNA_def_property_pointer_sdna(prop, nullptr, "textbox_state");

  prop = RNA_def_property(srna, "input_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "input_items.items", "input_items.items_num");
  RNA_def_property_struct_type(prop, "NodeImageWrangleInputItem");
  RNA_def_property_srna(prop, "NodeImageWrangleInputItems");

  prop = RNA_def_property(srna, "output_items", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "output_items.items", "output_items.items_num");
  RNA_def_property_struct_type(prop, "NodeImageWrangleOutputItem");
  RNA_def_property_ui_text(
      prop, "Output Items", "Named outputs written in the code (`Output = ...`)");
  RNA_def_property_srna(prop, "NodeImageWrangleOutputItems");

  prop = RNA_def_property(srna, "active_input_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "input_items.active_index");
  RNA_def_property_ui_text(prop, "Active Input Index", "Index of the active channel parameter");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);

  prop = RNA_def_property(srna, "active_output_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_sdna(prop, nullptr, "output_items.active_index");
  RNA_def_property_ui_text(prop, "Active Output Index", "Index of the active output");
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_flag(prop, PROP_NO_DEG_UPDATE);
  RNA_def_property_update(prop, NC_NODE, nullptr);
}

static void def_sh_script(BlenderRNA * /*brna*/, StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "script", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Text");
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_ID_REFCOUNT);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(prop, "Script", "Internal shader script to define the shader");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNodeScript_update");

  RNA_def_struct_sdna_from(srna, "NodeShaderScript", "storage");

  prop = RNA_def_property(srna, "filepath", PROP_STRING, PROP_FILEPATH);
  RNA_def_property_ui_text(prop, "File Path", "Shader script path");
  RNA_def_property_flag(prop, PROP_PATH_SUPPORTS_BLEND_RELATIVE);
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_ShaderNodeScript_update");

  prop = RNA_def_property(srna, "mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_funcs(prop, nullptr, "rna_ShaderNodeScript_mode_set", nullptr);
  RNA_def_property_enum_items(prop, node_script_mode_items);
  RNA_def_property_ui_text(prop, "Script Source", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "use_auto_update", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", NODE_SCRIPT_AUTO_UPDATE);
  RNA_def_property_ui_text(
      prop,
      "Auto Update",
      "Automatically update the shader when the .osl file changes (external scripts only)");

  prop = RNA_def_property(srna, "bytecode", PROP_STRING, PROP_NONE);
  RNA_def_property_string_funcs(prop,
                                "rna_ShaderNodeScript_bytecode_get",
                                "rna_ShaderNodeScript_bytecode_length",
                                "rna_ShaderNodeScript_bytecode_set");
  RNA_def_property_ui_text(prop, "Bytecode", "Compile bytecode for shader script node");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  prop = RNA_def_property(srna, "bytecode_hash", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(
      prop, "Bytecode Hash", "Hash of compile bytecode, for quick equality checking");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");

  /* needs to be reset to avoid bad pointer type in API functions below */
  RNA_def_struct_sdna_from(srna, "bNode", nullptr);

  /* API functions */

#  if 0 /* XXX TODO: use general node API for this. */
  func = RNA_def_function(srna, "find_socket", "rna_ShaderNodeScript_find_socket");
  RNA_def_function_ui_description(func, "Find a socket by name");
  parm = RNA_def_string(func, "name", nullptr, 0, "Socket name", "");
  RNA_def_parameter_flags(parm, 0, PARM_REQUIRED);
  /*parm =*/RNA_def_boolean(func, "is_output", false, "Output", "Whether the socket is an output");
  parm = RNA_def_pointer(func, "result", "NodeSocket", "", "");
  RNA_def_function_return(func, parm);

  func = RNA_def_function(srna, "add_socket", "rna_ShaderNodeScript_add_socket");
  RNA_def_function_ui_description(func, "Add a socket");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID);
  parm = RNA_def_string(func, "name", nullptr, 0, "Name", "");
  RNA_def_parameter_flags(parm, 0, PARM_REQUIRED);
  parm = RNA_def_enum(func, "type", rna_enum_node_socket_type_items, SOCK_FLOAT, "Type", "");
  RNA_def_parameter_flags(parm, 0, PARM_REQUIRED);
  /*parm =*/RNA_def_boolean(func, "is_output", false, "Output", "Whether the socket is an output");
  parm = RNA_def_pointer(func, "result", "NodeSocket", "", "");
  RNA_def_function_return(func, parm);

  func = RNA_def_function(srna, "remove_socket", "rna_ShaderNodeScript_remove_socket");
  RNA_def_function_ui_description(func, "Remove a socket");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID);
  parm = RNA_def_pointer(func, "sock", "NodeSocket", "Socket", "");
  RNA_def_parameter_flags(parm, PROP_NEVER_NULL, PARM_REQUIRED);
#  endif
}

void RNA_def_nodes_shader(BlenderRNA *brna)
{
  const auto define = [&](const char *base_name,
                          const char *struct_name,
                          void (*func)(BlenderRNA *, StructRNA *) = nullptr) {
    StructRNA *srna = define_specific_node(brna, struct_name, base_name);
    if (func) {
      func(brna, srna);
    }
  };

  /* clang-format off */

  define("ShaderNode", "ShaderNodeAddShader");
  define("ShaderNode", "ShaderNodeAmbientOcclusion", def_sh_ambient_occlusion);
  define("ShaderNode", "ShaderNodeAttribute", def_sh_attribute);
  define("ShaderNode", "ShaderNodeBackground");
  define("ShaderNode", "ShaderNodeBevel", def_sh_bevel);
  define("ShaderNode", "ShaderNodeBlackbody");
  define("ShaderNode", "ShaderNodeBrightContrast");
  define("ShaderNode", "ShaderNodeBsdfAnisotropic", def_glossy);
  define("ShaderNode", "ShaderNodeBsdfDiffuse");
  define("ShaderNode", "ShaderNodeBsdfGlass", def_glass);
  define("ShaderNode", "ShaderNodeBsdfHair", def_hair);
  define("ShaderNode", "ShaderNodeBsdfHairPrincipled", def_hair_principled);
  define("ShaderNode", "ShaderNodeBsdfMetallic", def_metallic);
  define("ShaderNode", "ShaderNodeBsdfPrincipled", def_principled);
  define("ShaderNode", "ShaderNodeBsdfRayPortal");
  define("ShaderNode", "ShaderNodeBsdfRefraction", def_refraction);
  define("ShaderNode", "ShaderNodeBsdfSheen", def_sheen);
  define("ShaderNode", "ShaderNodeBsdfToon", def_toon);
  define("ShaderNode", "ShaderNodeBsdfTranslucent");
  define("ShaderNode", "ShaderNodeBsdfTransparent");
  define("ShaderNode", "ShaderNodeBump", def_sh_bump);
  define("ShaderNode", "ShaderNodeCameraData");
  define("ShaderNode", "ShaderNodeClamp", def_clamp);
  define("ShaderNode", "ShaderNodeDerivative", def_sh_derivative);
  define("ShaderNode", "ShaderNodeCombineColor", def_sh_combsep_color);
  define("ShaderNode", "ShaderNodeCombineXYZ");
  define("ShaderNode", "ShaderNodeDisplacement", def_sh_displacement);
  define("ShaderNode", "ShaderNodeBillboardDisplacement", def_sh_billboard_displacement);
  define("ShaderNode", "ShaderNodeEeveeSpecular");
  define("ShaderNode", "ShaderNodeEmission");
  define("ShaderNode", "ShaderNodeFloatCurve", def_float_curve);
  define("ShaderNode", "ShaderNodeFresnel");
  define("ShaderNode", "ShaderNodeGamma");
  define("ShaderNode", "ShaderNodeHairInfo");
  define("ShaderNode", "ShaderNodeHoldout");
  define("ShaderNode", "ShaderNodeHLSL", def_sh_hlsl);
  define("ShaderNode", "ShaderNodeWrangle", def_sh_wrangle);
  define("ShaderNode", "ShaderNodeSDFShape", def_sdf_shape);
  define("ShaderNode", "ShaderNodeFractalPrimitive", def_fractal_primitive);
  define("ShaderNode", "ShaderNodeHueSaturation");
  define("ShaderNode", "ShaderNodeInvert");
  define("ShaderNode", "ShaderNodeLayerWeight");
  define("ShaderNode", "ShaderNodeLightAccumulation");
  define("ShaderNode", "ShaderNodeLightEvaluation", def_light_evaluation);
  define("ShaderNode", "ShaderNodeLightFalloff");
  define("ShaderNode", "ShaderNodeLightInfo");
  define("ShaderNode", "ShaderNodeLightIterInternalInput");
  define("ShaderNode", "ShaderNodeLightIterInternalOutput");
  define("ShaderNode", "ShaderNodeLightPath");
  define("ShaderNode", "ShaderNodeMapping", def_sh_mapping);
  define("ShaderNode", "ShaderNodeMapRange", def_map_range);
  define("ShaderNode", "ShaderNodeMath", def_math);
  define("ShaderNode", "ShaderNodeMix", def_sh_mix);
  define("ShaderNode", "ShaderNodeMixRGB", def_mix_rgb);
  define("ShaderNode", "ShaderNodeMixShader");
  define("ShaderNode", "ShaderNodeNamedPortal");
  define("ShaderNode", "ShaderNodeNewGeometry");
  define("ShaderNode", "ShaderNodeNormal");
  define("ShaderNode", "ShaderNodeNormalMap", def_sh_normal_map);
  define("ShaderNode", "ShaderNodeObjectInfo");
  define("ShaderNode", "ShaderNodeObjectTransform", def_sh_object_transform);
  define("ShaderNode", "ShaderNodeOutputAOV", def_sh_output_aov);
  define("ShaderNode", "ShaderNodeOutputLight", def_sh_output);
  define("ShaderNode", "ShaderNodeOutputLineStyle", def_sh_output_linestyle);
  define("ShaderNode", "ShaderNodeOutputMaterial", def_sh_output);
  define("ShaderNode", "ShaderNodeOutputWorld", def_sh_output);
  define("ShaderNode", "ShaderNodeParallaxOcclusion", def_sh_parallax);
  define("ShaderNode", "ShaderNodeSPOM", def_sh_spom);
  define("ShaderNode", "ShaderNodeParticleInfo");
  define("ShaderNode", "ShaderNodePointInfo");
  define("ShaderNode", "ShaderNodeRadialTiling", def_sh_radial_tiling);
  define("ShaderNode", "ShaderNodeRaycast", def_sh_raycast);
  define("ShaderNode", "ShaderNodeRGB");
  define("ShaderNode", "ShaderNodeRGBCurve", def_rgb_curve);
  define("ShaderNode", "ShaderNodeRGBToBW");
  define("ShaderNode", "ShaderNodeScript", def_sh_script);
  define("ShaderNode", "ShaderNodeSeparateColor", def_sh_combsep_color);
  define("ShaderNode", "ShaderNodeSeparateXYZ");
  define("ShaderNode", "ShaderNodeShaderToRGB");
  define("ShaderNode", "ShaderNodeShadowRaycast");
  define("ShaderNode", "ShaderNodeSqueeze");
  define("ShaderNode", "ShaderNodeStoreNamedPortal");
  define("ShaderNode", "ShaderNodeSubsurfaceScattering", def_sh_subsurface);
  define("ShaderNode", "ShaderNodeTangent", def_sh_tangent);
  define("ShaderNode", "ShaderNodeTexBrick", def_sh_tex_brick);
  define("ShaderNode", "ShaderNodeTexChecker", def_sh_tex_checker);
  define("ShaderNode", "ShaderNodeTexCoord", def_sh_tex_coord);
  define("ShaderNode", "ShaderNodeTexEnvironment", def_sh_tex_environment);
  define("ShaderNode", "ShaderNodeTexGabor", def_sh_tex_gabor);
  define("ShaderNode", "ShaderNodeTexGradient", def_sh_tex_gradient);
  define("ShaderNode", "ShaderNodeTexIES", def_sh_tex_ies);
  define("ShaderNode", "ShaderNodeTexImage", def_sh_tex_image);
  define("ShaderNode", "ShaderNodeTexMagic", def_sh_tex_magic);
  define("ShaderNode", "ShaderNodeTexNoise", def_sh_tex_noise);
  define("ShaderNode", "ShaderNodeTexSky", def_sh_tex_sky);
  define("ShaderNode", "ShaderNodeTexVoronoi", def_sh_tex_voronoi);
  define("ShaderNode", "ShaderNodeTexWave", def_sh_tex_wave);
  define("ShaderNode", "ShaderNodeTexWhiteNoise", def_sh_tex_white_noise);
  define("ShaderNode", "ShaderNodeUVAlongStroke", def_sh_uvalongstroke);
  define("ShaderNode", "ShaderNodeUVMap", def_sh_uvmap);
  define("ShaderNode", "ShaderNodeValToRGB", def_colorramp);
  define("ShaderNode", "ShaderNodeValue");
  define("ShaderNode", "ShaderNodeVectorCurve", def_vector_curve);
  define("ShaderNode", "ShaderNodeVectorDisplacement", def_sh_vector_displacement);
  define("ShaderNode", "ShaderNodeVectorMath", def_vector_math);
  define("ShaderNode", "ShaderNodeVectorRotate", def_sh_vector_rotate);
  define("ShaderNode", "ShaderNodeVectorTransform", def_sh_vect_transform);
  define("ShaderNode", "ShaderNodeVertexColor", def_sh_vertex_color);
  define("ShaderNode", "ShaderNodeVolumeAbsorption");
  define("ShaderNode", "ShaderNodeVolumeInfo");
  define("ShaderNode", "ShaderNodeVolumePrincipled");
  define("ShaderNode", "ShaderNodeVolumeScatter", def_scatter);
  define("ShaderNode", "ShaderNodeVolumeCoefficients", def_volume_coefficients);
  define("ShaderNode", "ShaderNodeWavelength");
  define("ShaderNode", "ShaderNodeWireframe", def_sh_tex_wireframe);

  /* === BIKINI LuxCore Begin === */
  define("ShaderNode", "ShaderNodeLuxArchGlass");
  define("ShaderNode", "ShaderNodeLuxCarpaint");
  define("ShaderNode", "ShaderNodeLuxCloth");
  define("ShaderNode", "ShaderNodeLuxDisney");
  define("ShaderNode", "ShaderNodeLuxEmission");
  define("ShaderNode", "ShaderNodeLuxFrontBackOpacity");
  define("ShaderNode", "ShaderNodeLuxGlass");
  define("ShaderNode", "ShaderNodeLuxGlossy");
  define("ShaderNode", "ShaderNodeLuxGlossyCoating");
  define("ShaderNode", "ShaderNodeLuxGlossyTranslucent");
  define("ShaderNode", "ShaderNodeLuxMatte");
  define("ShaderNode", "ShaderNodeLuxMatteTranslucent");
  define("ShaderNode", "ShaderNodeLuxMetal");
  define("ShaderNode", "ShaderNodeLuxMirror");
  define("ShaderNode", "ShaderNodeLuxMix");
  define("ShaderNode", "ShaderNodeLuxNull");
  define("ShaderNode", "ShaderNodeLuxRoughGlass");
  define("ShaderNode", "ShaderNodeLuxTwoSided");
  define("ShaderNode", "ShaderNodeLuxVelvet");
  define("ShaderNode", "ShaderNodeLuxVolumeClear");
  define("ShaderNode", "ShaderNodeLuxVolumeHeterogeneous");
  define("ShaderNode", "ShaderNodeLuxVolumeHomogeneous");
  define("ShaderNode", "ShaderNodeLuxOutput");
  /* === BIKINI LuxCore End === */
  define("ShaderNode", "ShaderNodeGroup", def_group);

  /* BIKINI: Image (GPU Texture Editor) and Object node types defined in this file. */
  define("ImageNode", "ImageNodeBrightContrast");
  define("ImageNode", "ImageNodeDivergence");
  define("ImageNode", "ImageNodeFlip");
  define("ImageNode", "ImageNodeFFT");
  define("ImageNode", "ImageNodeGradient");
  define("ImageNode", "ImageNodeIslandUV");
  define("ImageNode", "ImageNodeIslandPadding");
  define("ImageNode", "ImageNodeHeightToNormal");
  define("ImageNode", "ImageNodeNormalToHeight");
  define("ImageNode", "ImageNodePointStamp");
  define("ImageNode", "ImageNodeRasterizeGeometry");
  define("ImageNode", "ImageNodeGeoSDF");
  define("ImageNode", "ImageNodeSDFShape", def_sdf_shape);
  define("ImageNode", "ImageNodeFractalPrimitive", def_fractal_primitive);
  define("ImageNode", "ImageNodePaint");
  define("ImageNode", "ImageNodeSampleAtPixel");
  define("ImageNode", "ImageNodeWriteAtPixel");
  define("ImageNode", "ImageNodeHistogram", def_image_histogram);
  define("ImageNode", "ImageNodeCameraView", def_image_camera_view);
  define("ImageNode", "ImageNodeRenderMaterial", def_image_render_material);
  define("ImageNode", "ImageNodeShaderToy", def_image_shadertoy);
  define("ImageNode", "ImageNodeWrangle", def_image_wrangle);
  define("ImageNode", "ImageNodeTerrainPrimitive", def_terrain_primitive);
  define("ImageNode", "ImageNodeTerrainErosion", def_terrain_erosion);
  define("ImageNode", "ImageNodeTerrainSimulate", def_terrain_simulate);
  define("ImageNode", "ImageNodeTerrainModify", def_terrain_modify);
  define("ImageNode", "ImageNodeTerrainDerive", def_terrain_derive);
  define("ImageNode", "ImageNodeTerrainColor", def_terrain_color);
  define("ImageNode", "ImageNodeTerrainCombine", def_terrain_combine);
  define("ImageNode", "ImageNodeSceneTime");
  define("ImageNode", "ImageNodeSceneFrame"); /* alias of Scene Time */
  define("ImageNode", "ImageNodeFileOutput");

  /* clang-format on */
}

}  // namespace blender

#endif
