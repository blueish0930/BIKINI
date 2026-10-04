/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#pragma once

/**
 * Historically, each builtin node type was given an integer type as part of its definition. These
 * defines are redundant with idnames and shouldn't be used in new code. However in some cases they
 * are used for backwards compatibility when versioning relied on the integer type while the idname
 * changed.
 *
 * See #bNode::type_legacy for more information.
 *
 * NOTE: Values here must not be larger than #NODE_LEGACY_TYPE_GENERATION_START.
 */

/* -------------------------------------------------------------------- */
/** \name Shader Nodes
 * \{ */

// #define SH_NODE_MATERIAL  100
#define SH_NODE_RGB 101
#define SH_NODE_VALUE 102
#define SH_NODE_MIX_RGB_LEGACY 103
#define SH_NODE_VALTORGB 104
#define SH_NODE_RGBTOBW 105
#define SH_NODE_SHADERTORGB 106
// #define SH_NODE_TEXTURE       106
#define SH_NODE_NORMAL 107
// #define SH_NODE_GEOMETRY  108
#define SH_NODE_MAPPING 109
#define SH_NODE_CURVE_VEC 110
#define SH_NODE_CURVE_RGB 111
#define SH_NODE_CAMERA 114
#define SH_NODE_MATH 115
#define SH_NODE_VECTOR_MATH 116
#define SH_NODE_SQUEEZE 117
// #define SH_NODE_MATERIAL_EXT  118
#define SH_NODE_INVERT 119
#define SH_NODE_SEPRGB_LEGACY 120
#define SH_NODE_COMBRGB_LEGACY 121
#define SH_NODE_HUE_SAT 122

#define SH_NODE_OUTPUT_MATERIAL 124
#define SH_NODE_OUTPUT_WORLD 125
#define SH_NODE_OUTPUT_LIGHT 126
#define SH_NODE_FRESNEL 127
#define SH_NODE_MIX_SHADER 128
#define SH_NODE_ATTRIBUTE 129
#define SH_NODE_BACKGROUND 130
#define SH_NODE_BSDF_GLOSSY 131
#define SH_NODE_BSDF_DIFFUSE 132
#define SH_NODE_BSDF_GLOSSY_LEGACY 133
#define SH_NODE_BSDF_GLASS 134
#define SH_NODE_BSDF_TRANSLUCENT 137
#define SH_NODE_BSDF_TRANSPARENT 138
#define SH_NODE_BSDF_SHEEN 139
#define SH_NODE_EMISSION 140
#define SH_NODE_NEW_GEOMETRY 141
#define SH_NODE_LIGHT_PATH 142
#define SH_NODE_TEX_IMAGE 143
#define SH_NODE_TEX_SKY 145
#define SH_NODE_TEX_GRADIENT 146
#define SH_NODE_TEX_VORONOI 147
#define SH_NODE_TEX_MAGIC 148
#define SH_NODE_TEX_WAVE 149
#define SH_NODE_TEX_NOISE 150
#define SH_NODE_TEX_MUSGRAVE_DEPRECATED 152
#define SH_NODE_TEX_COORD 155
#define SH_NODE_ADD_SHADER 156
#define SH_NODE_TEX_ENVIRONMENT 157
// #define SH_NODE_OUTPUT_TEXTURE 158
#define SH_NODE_HOLDOUT 159
#define SH_NODE_LAYER_WEIGHT 160
#define SH_NODE_VOLUME_ABSORPTION 161
#define SH_NODE_VOLUME_SCATTER 162
#define SH_NODE_GAMMA 163
#define SH_NODE_TEX_CHECKER 164
#define SH_NODE_BRIGHTCONTRAST 165
#define SH_NODE_LIGHT_FALLOFF 166
#define SH_NODE_OBJECT_INFO 167
#define SH_NODE_PARTICLE_INFO 168
#define SH_NODE_TEX_BRICK 169
#define SH_NODE_BUMP 170
#define SH_NODE_SCRIPT 171
#define SH_NODE_AMBIENT_OCCLUSION 172
#define SH_NODE_BSDF_REFRACTION 173
#define SH_NODE_TANGENT 174
#define SH_NODE_NORMAL_MAP 175
#define SH_NODE_HAIR_INFO 176
#define SH_NODE_SUBSURFACE_SCATTERING 177
#define SH_NODE_WIREFRAME 178
#define SH_NODE_BSDF_TOON 179
#define SH_NODE_WAVELENGTH 180
#define SH_NODE_BLACKBODY 181
#define SH_NODE_VECT_TRANSFORM 182
#define SH_NODE_SEPHSV_LEGACY 183
#define SH_NODE_COMBHSV_LEGACY 184
#define SH_NODE_BSDF_HAIR 185
// #define SH_NODE_LAMP 186
#define SH_NODE_UVMAP 187
#define SH_NODE_SEPXYZ 188
#define SH_NODE_COMBXYZ 189
#define SH_NODE_OUTPUT_LINESTYLE 190
#define SH_NODE_UVALONGSTROKE 191
// #define SH_NODE_TEX_POINTDENSITY 192
#define SH_NODE_BSDF_PRINCIPLED 193
#define SH_NODE_TEX_IES 194
#define SH_NODE_EEVEE_SPECULAR 195
#define SH_NODE_BEVEL 197
#define SH_NODE_DISPLACEMENT 198
#define SH_NODE_VECTOR_DISPLACEMENT 199
#define SH_NODE_VOLUME_PRINCIPLED 200
/* 201..700 occupied by other node types, continue from 701 */
#define SH_NODE_BSDF_HAIR_PRINCIPLED 701
#define SH_NODE_MAP_RANGE 702
#define SH_NODE_CLAMP 703
#define SH_NODE_TEX_WHITE_NOISE 704
#define SH_NODE_VOLUME_INFO 705
#define SH_NODE_VERTEX_COLOR 706
#define SH_NODE_OUTPUT_AOV 707
#define SH_NODE_VECTOR_ROTATE 708
#define SH_NODE_CURVE_FLOAT 709
#define SH_NODE_POINT_INFO 710
#define SH_NODE_COMBINE_COLOR 711
#define SH_NODE_SEPARATE_COLOR 712
#define SH_NODE_MIX 713
#define SH_NODE_BSDF_RAY_PORTAL 714
#define SH_NODE_TEX_GABOR 715
#define SH_NODE_BSDF_METALLIC 716
#define SH_NODE_VOLUME_COEFFICIENTS 717
#define SH_NODE_RAYCAST 718
#define SH_NODE_SCENE_TIME 719
#define SH_NODE_LIGHT_INFO 720
#define SH_NODE_LIGHT_ACCUMULATION 721
#define SH_NODE_LIGHT_EVALUATION 722
#define SH_NODE_SHADOW_RAYCAST 723
#define SH_NODE_LIGHT_ITER_INTERNAL_INPUT 724
#define SH_NODE_LIGHT_ITER_INTERNAL_OUTPUT 725
/* BIKINI custom shader nodes. IDs 720-725 were used before Material Lighting
 * Nodes landed; blend files at 503.19-29 remap those types in versioning_503. */
#define SH_NODE_HLSL 726
#define SH_NODE_SDF_SHAPE 727
#define SH_NODE_FRACTAL_PRIMITIVE 728
#define SH_NODE_PARALLAX_OCCLUSION 729
#define SH_NODE_SPOM 730
#define SH_NODE_BILLBOARD_DISPLACEMENT 731
#define SH_NODE_WRANGLE 732
#define SH_NODE_DERIVATIVE 733

/** \} */

/* -------------------------------------------------------------------- */
/** \name Composite Nodes
 * \{ */

/* output socket defines */
#define RRES_OUT_IMAGE 0
#define RRES_OUT_ALPHA 1

/* NOTE: types are needed to restore callbacks, don't change values. */
#define CMP_NODE_VIEWER 201
#define CMP_NODE_RGB 202
#define CMP_NODE_VALUE_DEPRECATED 203
#define CMP_NODE_MIX_RGB_DEPRECATED 204
#define CMP_NODE_VALTORGB_DEPRECATED 205
#define CMP_NODE_RGBTOBW 206
#define CMP_NODE_NORMAL 207
#define CMP_NODE_CURVE_VEC_DEPRECATED 208
#define CMP_NODE_CURVE_RGB 209
#define CMP_NODE_ALPHAOVER 210
#define CMP_NODE_BLUR 211
#define CMP_NODE_FILTER 212
#define CMP_NODE_MAP_VALUE_DEPRECATED 213
#define CMP_NODE_TIME 214
#define CMP_NODE_VECBLUR 215
#define CMP_NODE_SEPRGBA_LEGACY 216
#define CMP_NODE_SEPHSVA_LEGACY 217
#define CMP_NODE_SETALPHA 218
#define CMP_NODE_HUE_SAT 219
#define CMP_NODE_IMAGE 220
#define CMP_NODE_R_LAYERS 221
#define CMP_NODE_COMPOSITE_DEPRECATED 222
#define CMP_NODE_OUTPUT_FILE 223
#define CMP_NODE_TEXTURE_DEPRECATED 224
#define CMP_NODE_TRANSLATE 225
#define CMP_NODE_ZCOMBINE 226
#define CMP_NODE_COMBRGBA_LEGACY 227
#define CMP_NODE_DILATEERODE 228
#define CMP_NODE_ROTATE 229
#define CMP_NODE_SCALE 230
#define CMP_NODE_SEPYCCA_LEGACY 231
#define CMP_NODE_COMBYCCA_LEGACY 232
#define CMP_NODE_SEPYUVA_LEGACY 233
#define CMP_NODE_COMBYUVA_LEGACY 234
#define CMP_NODE_DIFF_MATTE 235
#define CMP_NODE_COLOR_SPILL 236
#define CMP_NODE_CHROMA_MATTE 237
#define CMP_NODE_CHANNEL_MATTE 238
#define CMP_NODE_FLIP 239
/* Split viewer node is now a regular split node: CMP_NODE_SPLIT. */
#define CMP_NODE_SPLITVIEWER__DEPRECATED 240
// #define CMP_NODE_INDEX_MASK  241
#define CMP_NODE_MAP_UV 242
#define CMP_NODE_ID_MASK 243
#define CMP_NODE_DEFOCUS 244
#define CMP_NODE_DISPLACE 245
#define CMP_NODE_COMBHSVA_LEGACY 246
#define CMP_NODE_MATH_DEPRECATED 247
#define CMP_NODE_LUMA_MATTE 248
#define CMP_NODE_BRIGHTCONTRAST 249
#define CMP_NODE_GAMMA_DEPRECATED 250
#define CMP_NODE_INVERT 251
#define CMP_NODE_NORMALIZE 252
#define CMP_NODE_CROP 253
#define CMP_NODE_DBLUR 254
#define CMP_NODE_BILATERALBLUR 255
#define CMP_NODE_PREMULKEY 256
#define CMP_NODE_DIST_MATTE 257
#define CMP_NODE_VIEW_LEVELS 258
#define CMP_NODE_COLOR_MATTE 259
#define CMP_NODE_COLORBALANCE 260
#define CMP_NODE_HUECORRECT 261
#define CMP_NODE_MOVIECLIP 262
#define CMP_NODE_STABILIZE2D 263
#define CMP_NODE_TRANSFORM 264
#define CMP_NODE_MOVIEDISTORTION 265
#define CMP_NODE_DOUBLEEDGEMASK 266
#define CMP_NODE_OUTPUT_MULTI_FILE__DEPRECATED \
  267 /* DEPRECATED multi file node has been merged into regular CMP_NODE_OUTPUT_FILE */
#define CMP_NODE_MASK 268
#define CMP_NODE_KEYINGSCREEN 269
#define CMP_NODE_KEYING 270
#define CMP_NODE_TRACKPOS 271
#define CMP_NODE_INPAINT 272
#define CMP_NODE_DESPECKLE 273
#define CMP_NODE_ANTIALIASING 274
#define CMP_NODE_KUWAHARA 275
#define CMP_NODE_SPLIT 276

#define CMP_NODE_GLARE 301
#define CMP_NODE_TONEMAP 302
#define CMP_NODE_LENSDIST 303
#define CMP_NODE_SUNBEAMS_DEPRECATED 304

#define CMP_NODE_COLORCORRECTION 312
#define CMP_NODE_MASK_BOX 313
#define CMP_NODE_MASK_ELLIPSE 314
#define CMP_NODE_BOKEHIMAGE 315
#define CMP_NODE_BOKEHBLUR 316
#define CMP_NODE_SWITCH 317
#define CMP_NODE_PIXELATE 318

#define CMP_NODE_MAP_RANGE_DEPRECATED 319
#define CMP_NODE_PLANETRACKDEFORM 320
#define CMP_NODE_CORNERPIN 321
#define CMP_NODE_SWITCH_VIEW 322
#define CMP_NODE_CRYPTOMATTE_LEGACY 323
#define CMP_NODE_DENOISE 324
#define CMP_NODE_EXPOSURE 325
#define CMP_NODE_CRYPTOMATTE 326
#define CMP_NODE_POSTERIZE 327
#define CMP_NODE_CONVERT_COLOR_SPACE 328
#define CMP_NODE_SCENE_TIME 329
#define CMP_NODE_SEPARATE_XYZ_DEPRECATED 330
#define CMP_NODE_COMBINE_XYZ_DEPRECATED 331
#define CMP_NODE_COMBINE_COLOR 332
#define CMP_NODE_SEPARATE_COLOR 333
#define CMP_NODE_IMAGE_INFO 334
#define CMP_NODE_CONVERT_TO_DISPLAY 335
/** Evaluate an Image Process node group and return Color. */
#define CMP_NODE_IMAGE_PROCESS 336
#define CMP_NODE_NEURAL 337

/* channel toggles */
#define CMP_CHAN_RGB 1
#define CMP_CHAN_A 2

/** \} */

/* -------------------------------------------------------------------- */
/** \name Texture Nodes
 * \{ */

#define TEX_NODE_OUTPUT 401
#define TEX_NODE_CHECKER 402
#define TEX_NODE_TEXTURE 403
#define TEX_NODE_BRICKS 404
#define TEX_NODE_MATH 405
#define TEX_NODE_MIX_RGB 406
#define TEX_NODE_RGBTOBW 407
#define TEX_NODE_VALTORGB 408
#define TEX_NODE_IMAGE 409
#define TEX_NODE_CURVE_RGB 410
#define TEX_NODE_INVERT 411
#define TEX_NODE_HUE_SAT 412
#define TEX_NODE_CURVE_TIME 413
#define TEX_NODE_ROTATE 414
#define TEX_NODE_VIEWER 415
#define TEX_NODE_TRANSLATE 416
#define TEX_NODE_COORD 417
#define TEX_NODE_DISTANCE 418
#define TEX_NODE_COMPOSE_LEGACY 419
#define TEX_NODE_DECOMPOSE_LEGACY 420
#define TEX_NODE_VALTONOR 421
#define TEX_NODE_SCALE 422
#define TEX_NODE_AT 423
#define TEX_NODE_COMBINE_COLOR 424
#define TEX_NODE_SEPARATE_COLOR 425

/* 501-599 reserved. Use like this: TEX_NODE_PROC + TEX_CLOUDS, etc */
#define TEX_NODE_PROC 500
#define TEX_NODE_PROC_MAX 600

/** \} */

/* -------------------------------------------------------------------- */
/** \name Geometry Nodes
 * \{ */

#define GEO_NODE_TRIANGULATE 1000
#define GEO_NODE_TRANSFORM_GEOMETRY 1002
#define GEO_NODE_MESH_BOOLEAN 1003
#define GEO_NODE_OBJECT_INFO 1007
#define GEO_NODE_JOIN_GEOMETRY 1010
#define GEO_NODE_COLLECTION_INFO 1023
#define GEO_NODE_IS_VIEWPORT 1024
#define GEO_NODE_SUBDIVIDE_MESH 1029
#define GEO_NODE_MESH_PRIMITIVE_CUBE 1032
#define GEO_NODE_MESH_PRIMITIVE_CIRCLE 1033
#define GEO_NODE_MESH_PRIMITIVE_UV_SPHERE 1034
#define GEO_NODE_MESH_PRIMITIVE_CYLINDER 1035
#define GEO_NODE_MESH_PRIMITIVE_ICO_SPHERE 1036
#define GEO_NODE_MESH_PRIMITIVE_CONE 1037
#define GEO_NODE_MESH_PRIMITIVE_LINE 1038
#define GEO_NODE_MESH_PRIMITIVE_GRID 1039
#define GEO_NODE_BOUNDING_BOX 1042
#define GEO_NODE_SWITCH 1043
#define GEO_NODE_CURVE_TO_MESH 1045
#define GEO_NODE_RESAMPLE_CURVE 1047
#define GEO_NODE_INPUT_MATERIAL 1050
#define GEO_NODE_REPLACE_MATERIAL 1051
#define GEO_NODE_CURVE_LENGTH 1054
#define GEO_NODE_CONVEX_HULL 1056
#define GEO_NODE_SEPARATE_COMPONENTS 1059
#define GEO_NODE_CURVE_PRIMITIVE_STAR 1062
#define GEO_NODE_CURVE_PRIMITIVE_SPIRAL 1063
#define GEO_NODE_CURVE_PRIMITIVE_QUADRATIC_BEZIER 1064
#define GEO_NODE_CURVE_PRIMITIVE_BEZIER_SEGMENT 1065
#define GEO_NODE_CURVE_PRIMITIVE_CIRCLE 1066
#define GEO_NODE_VIEWER 1067
#define GEO_NODE_CURVE_PRIMITIVE_LINE 1068
#define GEO_NODE_CURVE_PRIMITIVE_QUADRILATERAL 1070
#define GEO_NODE_TRIM_CURVE 1071
#define GEO_NODE_FILL_CURVE 1075
#define GEO_NODE_INPUT_POSITION 1076
#define GEO_NODE_SET_POSITION 1077
#define GEO_NODE_INPUT_INDEX 1078
#define GEO_NODE_INPUT_NORMAL 1079
#define GEO_NODE_CAPTURE_ATTRIBUTE 1080
#define GEO_NODE_MATERIAL_SELECTION 1081
#define GEO_NODE_SET_MATERIAL 1082
#define GEO_NODE_REALIZE_INSTANCES 1083
#define GEO_NODE_ATTRIBUTE_STATISTIC 1084
#define GEO_NODE_SAMPLE_CURVE 1085
#define GEO_NODE_INPUT_TANGENT 1086
#define GEO_NODE_STRING_JOIN 1087
#define GEO_NODE_CURVE_SPLINE_PARAMETER 1088
#define GEO_NODE_FILLET_CURVE 1089
#define GEO_NODE_DISTRIBUTE_POINTS_ON_FACES 1090
#define GEO_NODE_STRING_TO_CURVES 1091
#define GEO_NODE_INSTANCE_ON_POINTS 1092
#define GEO_NODE_MESH_TO_POINTS 1093
#define GEO_NODE_POINTS_TO_VERTICES 1094
#define GEO_NODE_REVERSE_CURVE 1095
#define GEO_NODE_PROXIMITY 1096
#define GEO_NODE_SUBDIVIDE_CURVE 1097
#define GEO_NODE_INPUT_SPLINE_LENGTH 1098
#define GEO_NODE_CURVE_SPLINE_TYPE 1099
#define GEO_NODE_CURVE_SET_HANDLE_TYPE 1100
#define GEO_NODE_POINTS_TO_VOLUME 1101
#define GEO_NODE_CURVE_HANDLE_TYPE_SELECTION 1102
#define GEO_NODE_DELETE_GEOMETRY 1103
#define GEO_NODE_SEPARATE_GEOMETRY 1104
#define GEO_NODE_INPUT_RADIUS 1105
#define GEO_NODE_INPUT_CURVE_TILT 1106
#define GEO_NODE_INPUT_CURVE_HANDLES 1107
#define GEO_NODE_INPUT_FACE_SMOOTH 1108
#define GEO_NODE_INPUT_SPLINE_RESOLUTION 1109
#define GEO_NODE_INPUT_SPLINE_CYCLIC 1110
#define GEO_NODE_SET_CURVE_RADIUS 1111
#define GEO_NODE_SET_CURVE_TILT 1112
#define GEO_NODE_SET_CURVE_HANDLES 1113
#define GEO_NODE_SET_SHADE_SMOOTH 1114
#define GEO_NODE_SET_SPLINE_RESOLUTION 1115
#define GEO_NODE_SET_SPLINE_CYCLIC 1116
#define GEO_NODE_SET_POINT_RADIUS 1117
#define GEO_NODE_INPUT_MATERIAL_INDEX 1118
#define GEO_NODE_SET_MATERIAL_INDEX 1119
#define GEO_NODE_TRANSLATE_INSTANCES 1120
#define GEO_NODE_SCALE_INSTANCES 1121
#define GEO_NODE_ROTATE_INSTANCES 1122
#define GEO_NODE_SPLIT_EDGES 1123
#define GEO_NODE_MESH_TO_CURVE 1124
#define GEO_NODE_TRANSFER_ATTRIBUTE_DEPRECATED 1125
#define GEO_NODE_SUBDIVISION_SURFACE 1126
#define GEO_NODE_CURVE_ENDPOINT_SELECTION 1127
#define GEO_NODE_RAYCAST 1128
#define GEO_NODE_CURVE_TO_POINTS 1130
#define GEO_NODE_INSTANCES_TO_POINTS 1131
#define GEO_NODE_IMAGE_TEXTURE 1132
#define GEO_NODE_VOLUME_TO_MESH 1133
#define GEO_NODE_INPUT_ID 1134
#define GEO_NODE_SET_ID 1135
#define GEO_NODE_ATTRIBUTE_DOMAIN_SIZE 1136
#define GEO_NODE_DUAL_MESH 1137
#define GEO_NODE_INPUT_MESH_EDGE_VERTICES 1138
#define GEO_NODE_INPUT_MESH_FACE_AREA 1139
#define GEO_NODE_INPUT_MESH_FACE_NEIGHBORS 1140
#define GEO_NODE_INPUT_MESH_VERTEX_NEIGHBORS 1141
#define GEO_NODE_GEOMETRY_TO_INSTANCE 1142
#define GEO_NODE_INPUT_MESH_EDGE_NEIGHBORS 1143
#define GEO_NODE_INPUT_MESH_ISLAND 1144
#define GEO_NODE_INPUT_SCENE_TIME 1145
#define GEO_NODE_ACCUMULATE_FIELD 1146
#define GEO_NODE_INPUT_MESH_EDGE_ANGLE 1147
#define GEO_NODE_EVALUATE_AT_INDEX 1148
#define GEO_NODE_CURVE_PRIMITIVE_ARC 1149
#define GEO_NODE_FLIP_FACES 1150
#define GEO_NODE_SCALE_ELEMENTS 1151
#define GEO_NODE_EXTRUDE_MESH 1152
#define GEO_NODE_MERGE_BY_DISTANCE 1153
#define GEO_NODE_DUPLICATE_ELEMENTS 1154
#define GEO_NODE_INPUT_MESH_FACE_IS_PLANAR 1155
#define GEO_NODE_STORE_NAMED_ATTRIBUTE 1156
#define GEO_NODE_INPUT_NAMED_ATTRIBUTE 1157
#define GEO_NODE_REMOVE_ATTRIBUTE 1158
#define GEO_NODE_INPUT_INSTANCE_ROTATION 1159
#define GEO_NODE_INPUT_INSTANCE_SCALE 1160
#define GEO_NODE_VOLUME_CUBE 1161
#define GEO_NODE_POINTS 1162
#define GEO_NODE_EVALUATE_ON_DOMAIN 1163
#define GEO_NODE_MESH_TO_VOLUME 1164
#define GEO_NODE_UV_UNWRAP 1165
#define GEO_NODE_UV_PACK_ISLANDS 1166
#define GEO_NODE_DEFORM_CURVES_ON_SURFACE 1167
#define GEO_NODE_INPUT_SHORTEST_EDGE_PATHS 1168
#define GEO_NODE_EDGE_PATHS_TO_CURVES 1169
#define GEO_NODE_EDGE_PATHS_TO_SELECTION 1170
#define GEO_NODE_MESH_FACE_GROUP_BOUNDARIES 1171
#define GEO_NODE_DISTRIBUTE_POINTS_IN_VOLUME 1172
#define GEO_NODE_SELF_OBJECT 1173
#define GEO_NODE_SAMPLE_INDEX 1174
#define GEO_NODE_SAMPLE_NEAREST 1175
#define GEO_NODE_SAMPLE_NEAREST_SURFACE 1176
#define GEO_NODE_OFFSET_POINT_IN_CURVE 1177
#define GEO_NODE_CURVE_TOPOLOGY_CURVE_OF_POINT 1178
#define GEO_NODE_CURVE_TOPOLOGY_POINTS_OF_CURVE 1179
#define GEO_NODE_MESH_TOPOLOGY_OFFSET_CORNER_IN_FACE 1180
#define GEO_NODE_MESH_TOPOLOGY_CORNERS_OF_FACE 1181
#define GEO_NODE_MESH_TOPOLOGY_CORNERS_OF_VERTEX 1182
#define GEO_NODE_MESH_TOPOLOGY_EDGES_OF_CORNER 1183
#define GEO_NODE_MESH_TOPOLOGY_EDGES_OF_VERTEX 1184
#define GEO_NODE_MESH_TOPOLOGY_FACE_OF_CORNER 1185
#define GEO_NODE_MESH_TOPOLOGY_VERTEX_OF_CORNER 1186
#define GEO_NODE_SAMPLE_UV_SURFACE 1187
#define GEO_NODE_SET_CURVE_NORMAL 1188
#define GEO_NODE_IMAGE_INFO 1189
#define GEO_NODE_BLUR_ATTRIBUTE 1190
#define GEO_NODE_IMAGE 1191
#define GEO_NODE_INTERPOLATE_CURVES 1192
#define GEO_NODE_EDGES_TO_FACE_GROUPS 1193
// #define GEO_NODE_POINTS_TO_SDF_VOLUME 1194
// #define GEO_NODE_MESH_TO_SDF_VOLUME 1195
// #define GEO_NODE_SDF_VOLUME_SPHERE 1196
// #define GEO_NODE_MEAN_FILTER_SDF_VOLUME 1197
// #define GEO_NODE_OFFSET_SDF_VOLUME 1198
#define GEO_NODE_INDEX_OF_NEAREST 1199
/* Function nodes use the range starting at 1200. */
#define GEO_NODE_SIMULATION_INPUT 2100
#define GEO_NODE_SIMULATION_OUTPUT 2101
// #define GEO_NODE_INPUT_SIGNED_DISTANCE 2102
// #define GEO_NODE_SAMPLE_VOLUME 2103
#define GEO_NODE_MESH_TOPOLOGY_CORNERS_OF_EDGE 2104
/* Leaving out two indices to avoid crashes with files that were created during the development of
 * the repeat zone. */
#define GEO_NODE_REPEAT_INPUT 2107
#define GEO_NODE_REPEAT_OUTPUT 2108
#define GEO_NODE_TOOL_SELECTION 2109
#define GEO_NODE_TOOL_SET_SELECTION 2110
#define GEO_NODE_TOOL_3D_CURSOR 2111
#define GEO_NODE_TOOL_FACE_SET 2112
#define GEO_NODE_TOOL_SET_FACE_SET 2113
#define GEO_NODE_POINTS_TO_CURVES 2114
#define GEO_NODE_INPUT_EDGE_SMOOTH 2115
#define GEO_NODE_SPLIT_TO_INSTANCES 2116
#define GEO_NODE_INPUT_NAMED_LAYER_SELECTION 2117
#define GEO_NODE_INDEX_SWITCH 2118
#define GEO_NODE_INPUT_ACTIVE_CAMERA 2119
#define GEO_NODE_BAKE 2120
#define GEO_NODE_GET_NAMED_GRID 2121
#define GEO_NODE_STORE_NAMED_GRID 2122
#define GEO_NODE_SORT_ELEMENTS 2123
#define GEO_NODE_MENU_SWITCH 2124
#define GEO_NODE_SAMPLE_GRID 2125
#define GEO_NODE_MESH_TO_DENSITY_GRID 2126
#define GEO_NODE_MESH_TO_SDF_GRID 2127
#define GEO_NODE_POINTS_TO_SDF_GRID 2128
#define GEO_NODE_GRID_TO_MESH 2129
#define GEO_NODE_DISTRIBUTE_POINTS_IN_GRID 2130
#define GEO_NODE_SDF_GRID_BOOLEAN 2131
#define GEO_NODE_TOOL_VIEWPORT_TRANSFORM 2132
#define GEO_NODE_TOOL_MOUSE_POSITION 2133
#define GEO_NODE_SAMPLE_GRID_INDEX 2134
#define GEO_NODE_TOOL_ACTIVE_ELEMENT 2135
#define GEO_NODE_SET_INSTANCE_TRANSFORM 2136
#define GEO_NODE_INPUT_INSTANCE_TRANSFORM 2137
#define GEO_NODE_IMPORT_STL 2138
#define GEO_NODE_IMPORT_OBJ 2139
#define GEO_NODE_SET_GEOMETRY_NAME 2140
#define GEO_NODE_GIZMO_LINEAR 2141
#define GEO_NODE_GIZMO_DIAL 2142
#define GEO_NODE_GIZMO_TRANSFORM 2143
#define GEO_NODE_CURVES_TO_GREASE_PENCIL 2144
#define GEO_NODE_GREASE_PENCIL_TO_CURVES 2145
#define GEO_NODE_IMPORT_PLY 2146
#define GEO_NODE_WARNING 2147
#define GEO_NODE_FOREACH_GEOMETRY_ELEMENT_INPUT 2148
#define GEO_NODE_FOREACH_GEOMETRY_ELEMENT_OUTPUT 2149
#define GEO_NODE_MERGE_LAYERS 2150
#define GEO_NODE_INPUT_COLLECTION 2151
#define GEO_NODE_INPUT_OBJECT 2152
#define NODE_COMBINE_BUNDLE 2153
#define NODE_SEPARATE_BUNDLE 2154
#define NODE_CLOSURE_OUTPUT 2155
#define NODE_EVALUATE_CLOSURE 2156
#define NODE_CLOSURE_INPUT 2157
#define GEO_NODE_STORE_NAMED_PORTAL 2158
#define GEO_NODE_NAMED_PORTAL 2159
#define GEO_NODE_GEOMETRY_CLIP 2160
#define GEO_NODE_SET_CLOSURE_DEFAULT 2161
#define GEO_NODE_LOOP_SUBDIVISION 2162
#define GEO_NODE_TIME_SHIFT 2163
#define GEO_NODE_K_NEAREST 2164
#define GEO_NODE_DEBUG 2165
#define GEO_NODE_VIEWPORT_CAMERA 2166
#define GEO_NODE_MESH_LAPLACIAN 2167
#define GEO_NODE_SPARSE_MATRIX_MATH 2168
#define GEO_NODE_LINEAR_SOLVER 2169
#define GEO_NODE_SELECT_ELEMENTS 2170
#define GEO_NODE_EDIT_ELEMENTS 2171
/** Image Process Fluid Simulation zone (Stable Fluids 2D black box). */
#define IMG_NODE_FLUID_SIM_INPUT 2172
#define IMG_NODE_FLUID_SIM_OUTPUT 2173
/** Geometry Nodes: run a Python script on evaluate, then pass geometry through. */
#define GEO_NODE_PYTHON 2174
/** Image Process SDF Shape / Fractal Primitive. */
#define IMG_NODE_SDF_SHAPE 2175
#define IMG_NODE_FRACTAL_PRIMITIVE 2176
/** Geometry Nodes: Voronoi fracture mesh by point-cloud sites with per-site gap. */
#define GEO_NODE_VORONOI_FRACTURE 2177
/** Geometry Nodes: Instant Field-Aligned Meshes remeshing (Jakob et al. 2015). */
#define GEO_NODE_INSTANT_MESHES 2178
/** Geometry Nodes: Heat method geodesic distances (Crane et al.). */
#define GEO_NODE_HEAT_GEODESIC 2179
/** Geometry Nodes: Mesh 1-ring graph coloring (Wiki heuristics: DSatur/RLF/锟?. */
#define GEO_NODE_MESH_GRAPH_COLORING 2180
/** Geometry Nodes: N-RoSy tangent field (Directional library), attributes Prefix+index. */
#define GEO_NODE_TANGENT_FIELD 2181
/** Geometry Nodes: QuadWild feature-line driven pure-quad remesh (Pietroni et al. 2021). */
#define GEO_NODE_QUADWILD 2182
/** Geometry Nodes: isotropic / adaptive triangle remesh (pmp-library, Botsch鈥揔obbelt). */
#define GEO_NODE_TRIANGLE_REMESH 2183
/** Geometry Nodes: 3D Delaunay tetrahedralization (Bowyer鈥揥atson); Voronoi dual edges. */
#define GEO_NODE_DELAUNAY_3D 2184
/** Geometry Nodes: atomic scatter-write of fields to arbitrary domain indices. */
#define GEO_NODE_WRITE_AT_INDEX 2185
/** Geometry Nodes: per-face gradient of a scalar field defined on mesh vertices. */
#define GEO_NODE_MESH_GRADIENT 2186
/** Geometry Nodes: per-vertex divergence of a vector field defined on mesh faces. */
#define GEO_NODE_MESH_DIVERGENCE 2187
/** Image Process: gather Color at integer pixel coordinates. */
#define IMG_NODE_SAMPLE_AT_PIXEL 2188
/** Image Process: scatter-write Color at integer pixel coordinates. */
#define IMG_NODE_WRITE_AT_PIXEL 2189
/** Image Process: paint on Color input texture (Image Editor paint session). */
#define IMG_NODE_PAINT 2190
/** Image Process: histogram display + Photoshop-style levels adjust. */
#define IMG_NODE_HISTOGRAM 2191
/** Image Process: Scene Time (seconds + frame). Legacy enum name SCENE_FRAME. */
#define IMG_NODE_SCENE_FRAME 2192
#define IMG_NODE_SCENE_TIME IMG_NODE_SCENE_FRAME
/** Geometry Nodes: RBF kernel-weighted attribute interpolation from sample points. */
#define GEO_NODE_RBF_INTERPOLATE 2193
/** Image Process: render a color view from a Camera object (per-frame, demand-isolated). */
#define IMG_NODE_CAMERA_VIEW 2194
/** Image Process: render a material on a UV plane to a color texture. */
#define IMG_NODE_RENDER_MATERIAL 2226
/** Image Process: ShaderToy URL import + multi-pass GPU runtime. */
#define IMG_NODE_SHADERTOY 2227
/** Image Process: XY-projected mesh silhouette SDF. */
#define IMG_NODE_GEO_SDF 2225
/** Geometry Nodes: Box2D/Box3D black-box rigid body solver. */
#define GEO_NODE_BOX_ENGINE_SOLVER 2195
/** Geometry Nodes: tag rigid body attributes for Box Engine. */
#define GEO_NODE_BOX_ENGINE_SET_RIGID_BODY 2196
/** Geometry Nodes: append constraint points for Box Engine. */
#define GEO_NODE_BOX_ENGINE_SET_CONSTRAINT 2197
/** Geometry Nodes: compact XPBD cloth / soft-body solver. */
#define GEO_NODE_PBD_SOLVER 2202
/** Geometry Nodes: compact SPH / PBF / DFSPH fluid solver. Removed. */
#define GEO_NODE_SPLASH_SOLVER 2203
/** Geometry Nodes: compact Incremental Potential Contact solver. */
#define GEO_NODE_IPC_SOLVER 2204
/* 2205 reserved: removed unimplemented solver. */
/* 2206 reserved: removed unimplemented solver. */
/** Geometry Nodes: tag pin/mass/thickness for deformable solvers. */
#define GEO_NODE_SET_DEFORMABLE 2207
/** Geometry Nodes: mark a mesh as a deformable-solver collider. */
#define GEO_NODE_SET_COLLIDER 2208
/** Geometry Nodes: complete cloth (pin + colliders + self-collision). */
#define GEO_NODE_CLOTH_SOLVER 2209
/** Geometry Nodes: complete soft body (volume + colliders). */
#define GEO_NODE_SOFT_BODY_SOLVER 2210
/** Geometry Nodes: complete point-cloud fluid with mesh colliders. */
#define GEO_NODE_FLUID_SOLVER_LEGACY 2211
/** Geometry Nodes: viewport-only guide geometry output (not rendered, not in spreadsheet). */
#define GEO_NODE_GUIDE_GEOMETRY 2212
/** Geometry Nodes: fallback value when a group input is unlinked outside the group. */
#define GEO_NODE_SET_GROUP_INPUT_DEFAULT 2213
/** Geometry Nodes: write gravity attribute. Removed. */
#define GEO_NODE_SET_GRAVITY 2214
/** Geometry Nodes: per-cloth stretch / bend / mass. */
#define GEO_NODE_SET_CLOTH_MATERIAL 2215
/** Geometry Nodes: per-soft-body stretch / volume / mass. */
#define GEO_NODE_SET_SOFT_MATERIAL 2216
/** Geometry Nodes: radius / viscosity for particle fluids. */
#define GEO_NODE_SET_FLUID_MATERIAL 2217
/* 2218 reserved: removed unimplemented MPM material node. */
/** Geometry Nodes: APIC / FLIP hybrid particle fluid. */
#define GEO_NODE_APIC_FLUID_SOLVER 2219
/** Geometry Nodes: density / viscosity for APIC Fluid. */
#define GEO_NODE_SET_APIC_MATERIAL 2220
/** Geometry Nodes: Houdini-style kernel-weighted attribute transfer from a point field. */
#define GEO_NODE_ATTRIBUTE_TRANSFER 2221
/** Geometry Nodes: Jolt Physics black-box rigid body solver. */
#define GEO_NODE_JOLT_SOLVER 2222
/** Geometry Nodes: tag rigid body attributes for Jolt Solver. */
#define GEO_NODE_JOLT_SET_RIGID_BODY 2223
/** Geometry Nodes: append constraint points for Jolt Solver. */
#define GEO_NODE_JOLT_SET_CONSTRAINT 2224
/** Geometry Nodes: Position Based Fluids particle fluid solver. Removed. */
#define GEO_NODE_PBF_SOLVER 2228
/** Geometry Nodes: APIC particle-in-cell fluid solver. Removed. */
#define GEO_NODE_APIC_SOLVER 2229
/** Geometry Nodes: deterministic point-cloud FLIP liquid solver. */
#define GEO_NODE_FLIP_SOLVER 2230
/** GPU Texture Editor: Gaea-style terrain primitive / erosion / simulate / modify / derive. */
#define IMG_NODE_TERRAIN_PRIMITIVE 2231
#define IMG_NODE_TERRAIN_EROSION 2232
#define IMG_NODE_TERRAIN_SIMULATE 2233
#define IMG_NODE_TERRAIN_MODIFY 2234
#define IMG_NODE_TERRAIN_DERIVE 2235
#define IMG_NODE_TERRAIN_COLOR 2236
#define IMG_NODE_TERRAIN_COMBINE 2237
/** Cordless named portals in Shader / Compositor / Image (GPU Texture) editors. */
#define SH_NODE_STORE_NAMED_PORTAL 2198
#define SH_NODE_NAMED_PORTAL 2199
#define CMP_NODE_STORE_NAMED_PORTAL 2200
#define CMP_NODE_NAMED_PORTAL 2201

/* CGAL Geometry Nodes block: 2300鈥?599 reserved. */
/* 2300 reserved: deleted CGAL Convex Hull (use native GeometryNodeConvexHull) */
#define GEO_NODE_CGAL_ALPHA_SHAPE 2301
#define GEO_NODE_CGAL_MESH_BOOLEAN 2302
/* 2303 reserved: deleted CGAL Simplify */
#define GEO_NODE_CGAL_ISOTROPIC_REMESH 2304
#define GEO_NODE_CGAL_SMOOTH_SHAPE 2305
/* 2306 reserved: removed CGAL Hole Fill; use Fair Hole Fill. */
#define GEO_NODE_CGAL_FAIR 2307
#define GEO_NODE_CGAL_REFINE 2308
#define GEO_NODE_CGAL_CLIP 2309
#define GEO_NODE_CGAL_SUBDIVISION 2310
#define GEO_NODE_CGAL_REPAIR 2311
#define GEO_NODE_CGAL_KEEP_LARGEST 2312
#define GEO_NODE_CGAL_ALPHA_WRAP 2313
#define GEO_NODE_CGAL_ADVANCING_FRONT 2314
#define GEO_NODE_CGAL_DELAUNAY_3D 2315
#define GEO_NODE_CGAL_MIN_SPHERE 2316
#define GEO_NODE_CGAL_OPTIMAL_BBOX 2317
#define GEO_NODE_CGAL_COREFINE 2318
#define GEO_NODE_CGAL_DETECT_FEATURES 2319
#define GEO_NODE_CGAL_ANGLE_AREA_SMOOTH 2320
#define GEO_NODE_CGAL_TANGENTIAL_RELAX 2321
/* 2322 reserved: deleted CGAL Extrude */
#define GEO_NODE_CGAL_REMESH_PLANAR 2323
#define GEO_NODE_CGAL_RANDOM_PERTURB 2324
#define GEO_NODE_CGAL_TRIANGULATE 2325
#define GEO_NODE_CGAL_ORIENT_OUTWARD 2326
#define GEO_NODE_CGAL_REPAIR_SELF_INTERSECT 2327
#define GEO_NODE_CGAL_SPLIT_LONG_EDGES 2328
#define GEO_NODE_CGAL_KEEP_COMPONENT 2329
#define GEO_NODE_CGAL_MERGE_BORDER_VERTS 2330
#define GEO_NODE_CGAL_DOES_SELF_INTERSECT 2331
#define GEO_NODE_CGAL_MESH_VOLUME 2332
#define GEO_NODE_CGAL_MESH_AREA 2333
#define GEO_NODE_CGAL_AVERAGE_SPACING 2334
#define GEO_NODE_CGAL_JET_SMOOTH 2335
#define GEO_NODE_CGAL_BILATERAL_SMOOTH 2336
#define GEO_NODE_CGAL_REMOVE_OUTLIERS 2337
#define GEO_NODE_CGAL_GRID_SIMPLIFY 2338
#define GEO_NODE_CGAL_RANDOM_SIMPLIFY 2339
#define GEO_NODE_CGAL_ESTIMATE_NORMALS 2340
#define GEO_NODE_CGAL_POISSON 2341
#define GEO_NODE_CGAL_SIDE_OF_MESH 2342
#define GEO_NODE_CGAL_DISTANCE_TO_MESH 2343
#define GEO_NODE_CGAL_SAMPLE_POINTS 2344
#define GEO_NODE_CGAL_SKELETON 2345
#define GEO_NODE_CGAL_EXTRACT_BORDER 2346
#define GEO_NODE_CGAL_GEODESIC_DISTANCE 2347
#define GEO_NODE_CGAL_SEGMENTATION 2348
#define GEO_NODE_CGAL_MIN_ELLIPSOID 2349
#define GEO_NODE_CGAL_IS_CLOSED 2350
/* 2351 reserved (removed Component Count) */
#define GEO_NODE_CGAL_CENTROID 2352
/* 2353 reserved (removed Border Length) */
/* 2354锟?356 reserved (removed Hausdorff / Closest Points / Stitch Borders) */
#define GEO_NODE_CGAL_AUTOREFINE 2357
#define GEO_NODE_CGAL_REMOVE_DEGENERATE 2358
/** Mean + Gaussian curvature fields (context evaluation). */
#define GEO_NODE_CGAL_CURVATURE 2359
/* 2360锟?364 reserved (removed Gaussian/Face Areas/AABB/Principal Axes/Detect Planes) */
/* 2365 reserved (removed Does Bound Volume) */
#define GEO_NODE_CGAL_SCALE_SPACE 2366
/* 2367锟?369 reserved (removed Is Manifold / Euler / Face Component) */
#define GEO_NODE_CGAL_PRINCIPAL_CURVATURE 2370
#define GEO_NODE_CGAL_SHAPE_DIAMETER 2371
#define GEO_NODE_CGAL_MARK_SELF_INTERSECT 2372
/* 2373锟?378 reserved (removed Outward/Reverse/Hole/Border/Dihedral/NonManifold) */
#define GEO_NODE_CGAL_WLOP 2379
#define GEO_NODE_CGAL_HIERARCHY_SIMPLIFY 2380
#define GEO_NODE_CGAL_EDGE_AWARE_UPSAMPLE 2381
#define GEO_NODE_CGAL_VCM_ESTIMATE_NORMALS 2382
#define GEO_NODE_CGAL_PCA_ESTIMATE_NORMALS 2383
#define GEO_NODE_CGAL_MST_ORIENT_NORMALS 2384
/* 2385 reserved: deleted Radial Orient Normals */
#define GEO_NODE_CGAL_CLUSTER_POINT_SET 2386
/* 2387 reserved: deleted Polyhedral Envelope */
/* 2388 reserved: deleted CGAL Vertex Normals */
/* 2389 reserved: deleted CGAL Face Normals */
/* 2390 reserved: deleted CGAL Face Aspect Ratio */
/* 2391 reserved: deleted CGAL Vertex Valence */
/* 2392 reserved: deleted CGAL Mean Edge Length */
/* 2393 reserved: deleted CGAL Border Vertex */
/* 2394 reserved: deleted CGAL Face Quality */
#define GEO_NODE_CGAL_REGION_GROWING 2395
#define GEO_NODE_CGAL_SURFACE_SHORTEST_PATH 2396
/* 2397 reserved: deleted Mesh Locate */
/* 2398鈥?401 reserved: deleted Edge Length / Face Perimeter / Points Centroid / Fit Plane */
/* 2402 reserved: deleted Neighbor Scale */
/* 2403 reserved: deleted CGAL Scanline Orient Normals */
/* 2404–2414 reserved: deleted ExtractBorder/Geodesic/…/IsTriangleMesh batch */
/* Catalog batch 21: high-value available algorithms */
/* 2415 reserved: deleted Orient Polygon Soup */
#define GEO_NODE_CGAL_ARAP_DEFORM 2416
#define GEO_NODE_CGAL_LSCM_UV 2417
/* 2418 reserved: deleted CGAL Efficient RANSAC. */
#define GEO_NODE_CGAL_ARAP_UV 2419
/* 2420 reserved: deleted Discrete Conformal UV (fixed circular border) */
/* 2421 reserved: removed CGAL Point Region Growing; use Point Shape Fitting. */
/* 2422–2425 reserved: deleted Mean Value / Discrete Authalic / Barycentric / Iterative Authalic UV
 * (fixed circular border — user rejected disk-shaped unwraps) */
#define GEO_NODE_CGAL_UV_DISTORTION 2426
/* Catalog batch 23: P20 / M07 / M36 + Scale Space options (R06/R07 on existing node) */
#define GEO_NODE_CGAL_REPAIR_DEGENERACIES 2427
#define GEO_NODE_CGAL_MESH_INTERSECTION 2428
/* Catalog batch 24 */
#define GEO_NODE_CGAL_REFINE_ISOLEVEL 2430
#define GEO_NODE_CGAL_EXACT_GEODESIC 2431
#define GEO_NODE_CGAL_SKIN_SURFACE 2432
/* Catalog batch 25: new only (not previously deleted) */
#define GEO_NODE_CGAL_UNION_OF_BALLS 2433
#define GEO_NODE_CGAL_MIN_SPHERE_OF_SPHERES 2434
/* Catalog batch 26: new only (not previously deleted) */
#define GEO_NODE_CGAL_VSA_APPROXIMATE 2435
#define GEO_NODE_CGAL_MIN_CIRCLE_2 2436
#define GEO_NODE_CGAL_MIN_RECTANGLE_2 2437
#define GEO_NODE_CGAL_MIN_ELLIPSE_2 2438
#define GEO_NODE_CGAL_MIN_PARALLELOGRAM_2 2439
/* 2440 reserved: deleted CGAL Convex Hull 2D */
#define GEO_NODE_CGAL_MESH_SLICER 2441
#define GEO_NODE_CGAL_VCM_FEATURE_EDGES 2442
#define GEO_NODE_CGAL_DELAUNAY_2 2443
#define GEO_NODE_CGAL_ALPHA_SHAPE_2 2444
#define GEO_NODE_CGAL_VORONOI_2 2445
#define GEO_NODE_CGAL_STRAIGHT_SKELETON_2 2446
#define GEO_NODE_CGAL_POLYGON_OFFSET_2 2447
/* Catalog batch 28: new only (not previously deleted) */
#define GEO_NODE_CGAL_MIN_ANNULUS_2 2448
#define GEO_NODE_CGAL_CONVEX_PARTITION_2 2449
/* 2450 reserved: deleted CGAL Y-Monotone Partition 2D */
#define GEO_NODE_CGAL_MAX_AREA_K_GON_2 2451
/* 2452 reserved: deleted CGAL Polyline Simplify 2D */
#define GEO_NODE_CGAL_WIDTH_3 2453
#define GEO_NODE_CGAL_MINKOWSKI_SUM_2 2454
#define GEO_NODE_CGAL_EXTRUDE_SKELETON 2455
#define GEO_NODE_CGAL_POLYGON_FILL_2 2456
#define GEO_NODE_CGAL_DO_INTERSECT_2 2457
#define GEO_NODE_CGAL_MEDIAL_AXIS_2 2458
/* Catalog batch 29: new only (not previously deleted) */
#define GEO_NODE_CGAL_BOOLEAN_OPS_2 2459
#define GEO_NODE_CGAL_LARGEST_EMPTY_ISO_RECTANGLE_2 2460
#define GEO_NODE_CGAL_POLYGON_REPAIR_2 2461
/* 2462 reserved: deleted Repair Polygon Soup */
/* 2463 reserved: deleted CGAL Apollonius Graph 2D */
/* 2464 reserved: deleted Rectangular P-Center 2D */
/* 2465 reserved: deleted CGAL Regularize Contour 2D */
#define GEO_NODE_CGAL_MONGE_JET_FIT 2466
#define GEO_NODE_CGAL_CONVEX_DECOMPOSITION_3 2467
#define GEO_NODE_CGAL_LAPLACE_DEFORM 2468
#define GEO_NODE_CGAL_SURFACE_DELAUNAY_REMESH 2469
/* 2470 reserved: deleted GeometryNodeStringToStrokeCurves (单线字) */
/* Catalog batch 31: new only (not previously deleted) */
/* 2471 reserved: deleted CGAL OTR Reconstruct 2D */
/* 2472 reserved: deleted CGAL Regular Triangulation 2D */
/* 2473 reserved: deleted CGAL Power Diagram 2D */
#define GEO_NODE_CGAL_VORONOI_3 2474
#define GEO_NODE_CGAL_VISIBILITY_2 2475
#define GEO_NODE_CGAL_REFINE_MESH_2 2476
#define GEO_NODE_CGAL_ALPHA_COMPLEX_3 2477
#define GEO_NODE_CGAL_MINKOWSKI_SUM_3 2478
/* Catalog batch 32: new only (not previously deleted) */
/* 2479 reserved: deleted CGAL Regular Triangulation 3D */
/* 2480 reserved: deleted CGAL Power Diagram 3D */
#define GEO_NODE_CGAL_LARGEST_EMPTY_CIRCLE_2 2481
#define GEO_NODE_CGAL_LARGEST_INSCRIBED_CIRCLE_2 2482
#define GEO_NODE_CGAL_MIN_WIDTH_2 2483
#define GEO_NODE_CGAL_PERIODIC_DELAUNAY_2 2484
/* 2485 reserved: deleted CGAL Constrained Voronoi 2D */
/* 2486 reserved: deleted CGAL Connect Holes 2D */
/* Catalog batch 33: remaining (deleted IDs stay reserved) */
#define GEO_NODE_CGAL_ARRANGEMENT_2 2487
/* 2488 reserved: deleted CGAL Segment Voronoi 2D */
/* 2489 reserved: deleted CGAL Snap Rounding 2D */
/* 2490 reserved: deleted CGAL Theta Graph 2D */
/* 2491 reserved: deleted CGAL Yao Graph 2D */
#define GEO_NODE_CGAL_DELAUNAY_ON_SPHERE 2492
#define GEO_NODE_CGAL_PERIODIC_DELAUNAY_3 2493
/* 2494 reserved: deleted CGAL All Furthest Neighbors 2D */
/* 2495 reserved: deleted CGAL Project Mesh 2D */
/* Catalog batch 34: new only (not previously deleted) */
#define GEO_NODE_CGAL_DO_INTERSECT_3 2496
/* 2497 reserved: deleted CGAL Split by Mesh */
#define GEO_NODE_CGAL_NATURAL_NEIGHBOR_2 2498
/* 2499 reserved: deleted CGAL Cage Deform 2D */
/* 2500 reserved: deleted CGAL Proximity Graph 2D */
#define GEO_NODE_CGAL_FILL_POLYLINE 2501
/* 2502 reserved: deleted CGAL Regularize Planes */
/* Catalog batch 35: new only (not previously deleted) */
/* 2503 reserved: deleted CGAL Halfspace Intersection 3D. */
/* 2504 reserved: deleted CGAL Lloyd Optimize 2D */
/* 2505 reserved: deleted CGAL Conforming Delaunay 2D */
#define GEO_NODE_CGAL_MAX_PERIMETER_K_GON_2 2506
/* 2507 reserved: deleted CGAL Weighted Skeleton 2D */
#define GEO_NODE_CGAL_CONSTRAINED_DELAUNAY_2 2508
/* 2509 reserved: deleted CGAL Point in Polygon 2D */
/* Catalog batch 36: new only (not previously deleted) */
#define GEO_NODE_CGAL_EXTERIOR_SKELETON_2 2510
/* 2511 reserved: deleted CGAL Weighted Offset 2D */
#define GEO_NODE_CGAL_VORONOI_ON_SPHERE 2512
/* 2513 reserved: deleted CGAL Stream Lines 2D */
/* 2514 reserved: deleted CGAL Upper Envelope 3D */
/* 2515 reserved: deleted CGAL Round Offset 2D */
/* Catalog batch 37: new only (not previously deleted) */
/* 2516 reserved: removed CGAL Radius Graph 3D; use Neighbor Graph 3D. */
/* 2517 reserved: deleted CGAL Visibility Holes 2D */
#define GEO_NODE_CGAL_KNN_GRAPH_3 2518
#define GEO_NODE_CGAL_NATURAL_NEIGHBOR_3 2519
/* 2520 reserved: deleted CGAL Sibson Gradient 2D */
/* Catalog batch 38: 10 new nodes */
/* 2521 reserved: deleted CGAL Octree 3D */
/* 2522 reserved: deleted CGAL Hilbert Path 3D */
/* 2523 reserved: deleted CGAL Envelope 2D */
/* 2524 reserved: deleted CGAL Shortest Cycle */
#define GEO_NODE_CGAL_PERIODIC_VORONOI_2 2525
#define GEO_NODE_CGAL_PERIODIC_VORONOI_3 2526
/* 2527 reserved: deleted CGAL Gabriel Graph 3D */
#define GEO_NODE_CGAL_EUCLIDEAN_MST_3 2528
/* 2529 reserved: deleted CGAL Beta Skeleton 2D */
#define GEO_NODE_CGAL_CONVEX_LAYERS_2 2530
/* Catalog batch 39: new only (not previously deleted) */
/* 2531 reserved: deleted CGAL Farthest Voronoi 2D */
#define GEO_NODE_CGAL_CONVEX_LAYERS_3 2532
/* 2533 reserved: deleted CGAL Overlay 2D */
/* 2534 reserved: deleted CGAL Polygon Kernel 2D */
#define GEO_NODE_CGAL_SELF_INTERSECTION_CURVES 2535
#define GEO_NODE_CGAL_CRUST_2 2536
#define GEO_NODE_CGAL_GEODESIC_VORONOI 2537
/* 2538 reserved: deleted CGAL Isosurface 3D. */
/* Catalog batch 40: new only (not previously deleted) */
/* 2539 reserved: deleted CGAL QEM Simplify */
/* 2540 reserved: deleted CGAL Line Arrangement 2D */
#define GEO_NODE_CGAL_VERTICAL_DECOMP_2 2541
/* 2542 reserved: deleted CGAL Circle Arrangement 2D */
/* 2543 reserved: deleted CGAL Crust 3D */
/* 2544 reserved: deleted CGAL Clipped Voronoi 3D */
/* 2545 reserved: deleted CGAL Complement 2D */
#define GEO_NODE_CGAL_SIMPLE_POLYGON_2 2546
/* Catalog batch 41: new only (not previously deleted) */
/* 2547 reserved: deleted CGAL Split Crossings 2D */
/* 2548 reserved: deleted CGAL Crossing Points 2D */
/* 2549 reserved: deleted CGAL Pullout Directions 2D */
#define GEO_NODE_CGAL_SSAB_PARTITION_2 2550
#define GEO_NODE_CGAL_SNAP_BORDERS 2551
/* 2552 reserved: deleted CGAL Autorefine Clean */
/* 2553 reserved: deleted CGAL Random Polygon 2D */
/* 2554 reserved: deleted CGAL Random Convex Set 2D */
/* Catalog batch 42: new only (not previously deleted) */
#define GEO_NODE_CGAL_LARGEST_EMPTY_SPHERE_3 2555
/* 2556 reserved: deleted CGAL RANSAC Primitives. */
/* 2557 reserved: skipped Tetrahedral Remesh (export-symbol explosion) */
/* 2558 reserved: deleted CGAL Voronoi Slice 3D. */
/* 2559 reserved: deleted CGAL Convex Offset 3D. */
/* 2560 reserved: deleted CGAL Volume Components. */
#define GEO_NODE_CGAL_INSCRIBED_SPHERE_3 2561
#define GEO_NODE_CGAL_MIN_CYLINDER_3 2562
/* Catalog batch 43: new only (not previously deleted) */
#define GEO_NODE_CGAL_FAIR_HOLE_FILL 2563
#define GEO_NODE_CGAL_SIMPLIFY_POLYLINE_3 2564
/* 2565 reserved: removed CGAL Sphere Region Growing; use Point Shape Fitting. */
/* 2566 reserved: removed CGAL Cylinder Region Growing; use Point Shape Fitting. */
/* 2567 reserved: removed CGAL Circle Region Growing; use Point Shape Fitting. */
/* 2568 reserved: removed CGAL Line Region Growing; use Point Shape Fitting. */
#define GEO_NODE_CGAL_POINT_SHAPE_FITTING 2569
/* Catalog batch 44: new only (not previously deleted) */
/* 2570 reserved: deleted CGAL Polar Dual 3D */
#define GEO_NODE_CGAL_MIN_ANNULUS_3 2571
/* 2572 reserved: deleted CGAL PCA Ellipsoid */
#define GEO_NODE_CGAL_COLLAPSE_SHORT_EDGES 2573
/* 2574 reserved: deleted CGAL Polyline Line Fitting */
/* 2575 reserved: deleted CGAL Snap Meshes */
/* Catalog batch 45: new only (not previously deleted) */
/* 2576 reserved: deleted CGAL Envelope Simplify */
/* 2577 reserved: deleted CGAL Contract Mesh */
#define GEO_NODE_CGAL_REGULARIZE_OPEN_CONTOUR_2 2578
/* 2579 reserved: deleted CGAL Harmonic Deform 2D */
#define GEO_NODE_CGAL_CONSTRAINED_SIMPLIFY 2580
/* Catalog batch 46: deleted. IDs reserved. */
/* 2581 reserved: deleted CGAL Sphere Slice */
/* 2582 reserved: deleted CGAL Cylinder Slice */
/* 2583 reserved: deleted CGAL Merge Coplanar */
/* 2584 reserved: deleted CGAL MVC Relax UV */
/* 2585 reserved: deleted CGAL CDT Plus 2D */
/* Catalog batch 47: remaining (deleted IDs stay reserved) */
/* 2586 reserved: deleted CGAL Cone Slice */
/* 2587 reserved: deleted CGAL Clip Box */
#define GEO_NODE_CGAL_Y_MONOTONE_PARTITION_2 2588
#define GEO_NODE_CGAL_POLYLINE_SIMPLIFY_2 2589
/* 2590 reserved: deleted CGAL Clip by Mesh */
/* Catalog batch 48: restored-deleted. IDs reserved, do not reuse. */
/* 2591 reserved: deleted CGAL Proximity Graph 2D (also 2500) */
/* 2592 reserved: deleted CGAL Conforming Delaunay 2D (also 2505) */
/* 2593 reserved: deleted CGAL Rectangular P-Center 2D (also 2464) */
/* 2594 reserved: deleted CGAL Polyline Hull 2D */
/* Catalog batch 49: remaining (deleted IDs stay reserved) */
/* 2595 reserved: deleted CGAL Curve Mesh Intersect */
#define GEO_NODE_CGAL_GEODESIC_ISOLINES 2596
/* 2597 reserved: deleted CGAL Overlap Faces */
#define GEO_NODE_CGAL_CONTOUR_STACK 2598
/* Catalog batch 50: reserved deleted */
/* 2599 reserved: deleted CGAL Alpha Edges 3D */
/* 2600 reserved: deleted CGAL Delaunay Edges 3D */
/* 2601 reserved: deleted CGAL Mesh Edge Path */
/* 2602 reserved: deleted CGAL Curvature Isolines */
/* Catalog batch 51 */
/* 2603 reserved: deleted CGAL Offset Mesh 3D */
/* 2604 reserved: deleted CGAL CDT Hole Fill */
/* 2605 reserved: deleted CGAL Bisector Surface */
/* 2606 reserved: deleted CGAL Projected Outline */
/* Catalog batch 52 */
/* 2607 reserved: deleted CGAL Visibility Graph 2D */
/* 2608 reserved: deleted CGAL Terrain TIN */
/* 2609 reserved: deleted CGAL Principal Lines */
/* 2610 reserved: deleted CGAL Min Circle 3D */
/* Catalog batch 53 */
/* 2611 reserved: deleted CGAL Interior Tets */
/* 2612 reserved: deleted CGAL Surface Delaunay Graph */
/* 2613 reserved: deleted CGAL Split Charts */
/* 2614 reserved: deleted CGAL Restricted Voronoi */
/* Catalog batch 54 */
/* 2615 reserved: deleted CGAL Walk Tets */
#define GEO_NODE_CGAL_SKELETON_SPOKES 2616
/* 2617 reserved: deleted CGAL Radial Slices */
/* 2618 reserved: deleted CGAL Intersection Band */
/* Catalog batch 55: remaining official CGAL (6.0.1 + 6.1/6.2 headers). */
/* 2619 reserved: deleted CGAL Kinetic Partition */
#define GEO_NODE_CGAL_KINETIC_PARTITION 2619
/* 2620 reserved: deleted CGAL Kinetic Reconstruct */
#define GEO_NODE_CGAL_KINETIC_RECONSTRUCT 2620
/* 2621 reserved: deleted CGAL Minkowski Glide 3D */
#define GEO_NODE_CGAL_MINKOWSKI_GLIDE_3 2621
/* 2622 reserved: deleted CGAL Ridge Curves */
#define GEO_NODE_CGAL_RIDGE_CURVES 2622
#define GEO_NODE_CGAL_SPHERE_INTERSECT 2623
#define GEO_NODE_CGAL_SPHERE_ARRANGEMENT 2624
/* 2625 reserved: deleted CGAL Segment Voronoi Linf */
#define GEO_NODE_CGAL_SEGMENT_VORONOI_LINF_2 2625
#define GEO_NODE_CGAL_GRAPHCUT_SEGMENT 2626
/* 2627 reserved: deleted CGAL Collision Detect */
#define GEO_NODE_CGAL_COLLISION_DETECT 2627
/* 2628 reserved: deleted CGAL Hyperbolic Delaunay */
#define GEO_NODE_CGAL_HYPERBOLIC_DELAUNAY_2 2628
/* 2629 reserved: deleted CGAL Volume Mesh 3 */
#define GEO_NODE_CGAL_VOLUME_MESH_3 2629
#define GEO_NODE_CGAL_TET_REMESH 2630
/* 2631 reserved: deleted CGAL Periodic Mesh 3 */
#define GEO_NODE_CGAL_PERIODIC_MESH_3 2631
/* 2632 reserved: deleted CGAL Surface Mesher */
#define GEO_NODE_CGAL_SURFACE_MESHER 2632
#define GEO_NODE_CGAL_POINT_FEATURES 2633
/* 2634 reserved: deleted CGAL Classify */
#define GEO_NODE_CGAL_CLASSIFY 2634
#define GEO_NODE_CGAL_REGISTER_ICP 2635
#define GEO_NODE_CGAL_SUPER4PCS 2636
/* 2637 reserved: deleted CGAL Regularize Segments */
#define GEO_NODE_CGAL_REGULARIZE_SEGMENTS_2 2637
/* 2638 reserved: deleted CGAL PolyFit */
#define GEO_NODE_CGAL_POLYFIT 2638
#define GEO_NODE_CGAL_CONSTRAINED_DT3 2639
#define GEO_NODE_CGAL_DUAL_CONTOUR 2640
/* 2641 reserved: deleted CGAL Frechet Distance */
#define GEO_NODE_CGAL_FRECHET_DISTANCE 2641
#define GEO_NODE_CGAL_ALPHA_WRAP_2 2642
#define GEO_NODE_CGAL_CAGE_DEFORM_3 2643
/* 2644 reserved: deleted CGAL ACVD Remesh */
#define GEO_NODE_CGAL_ACVD_REMESH 2644
/* 2645 reserved: deleted CGAL Poisson Disk Sample */
#define GEO_NODE_CGAL_POISSON_DISK 2645
#define GEO_NODE_CGAL_APPROX_CONVEX_DECOMP 2646
/* 2647 reserved: deleted CGAL Extreme Point 3 */
#define GEO_NODE_CGAL_EXTREME_POINT_3 2647
/* 2648 reserved: deleted CGAL Barycentric 3 */
#define GEO_NODE_CGAL_BARYCENTRIC_3 2648
#define GEO_NODE_CGAL_OPTIMAL_TRANSPORT 2649

/* Make It Stand (Prévost, Whiting, Lefebvre, Sorkine-Hornung, SIGGRAPH 2013). */
/* 2700 reserved: deleted Center of Mass */
/* 2701 reserved: deleted Support Polygon */
/* 2702 reserved: deleted Balance Check */
/* 2703 reserved: deleted Inner Carve */
#define GEO_NODE_MAKE_IT_STAND 2704
/** Geometry Nodes: VEX-like attribute wrangle (bytecode VM). */
#define GEO_NODE_WRANGLE 2705
/* 2710-2714 reserved: deleted Geometry Nodes FLIP
 * (Container, Source, Collide, Solver, Particle Fluid Surface). */
#define GEO_NODE_FLUID_DOMAIN 2710
#define GEO_NODE_FLUID_SOURCE 2711
#define GEO_NODE_FLUID_COLLIDE 2712
#define GEO_NODE_FLUID_SOLVER 2713
#define GEO_NODE_FLUID_SURFACE 2714
/** Prism extrusion for silhouette POM (SPOM). */
#define GEO_NODE_SPOM_PRISMS 2706
/** Generalized winding number (Jacobson 2013 / Barill 2018). */
#define GEO_NODE_WINDING_NUMBER 2707

/* 2429 reserved: deleted Structure Point Set (exploded positions / unusable defaults) */




/** \} */

/* -------------------------------------------------------------------- */
/** \name Function Nodes
 * \{ */

#define FN_NODE_BOOLEAN_MATH 1200
#define FN_NODE_COMPARE 1202
#define FN_NODE_LEGACY_RANDOM_FLOAT 1206
#define FN_NODE_INPUT_VECTOR 1207
#define FN_NODE_INPUT_STRING 1208
#define FN_NODE_FLOAT_TO_INT 1209
#define FN_NODE_VALUE_TO_STRING 1210
#define FN_NODE_STRING_LENGTH 1211
#define FN_NODE_SLICE_STRING 1212
#define FN_NODE_INPUT_SPECIAL_CHARACTERS 1213
#define FN_NODE_RANDOM_VALUE 1214
#define FN_NODE_ROTATE_EULER 1215
#define FN_NODE_ALIGN_EULER_TO_VECTOR 1216
#define FN_NODE_INPUT_COLOR 1217
#define FN_NODE_REPLACE_STRING 1218
#define FN_NODE_INPUT_BOOL 1219
#define FN_NODE_INPUT_INT 1220
#define FN_NODE_SEPARATE_COLOR 1221
#define FN_NODE_COMBINE_COLOR 1222
#define FN_NODE_AXIS_ANGLE_TO_ROTATION 1223
#define FN_NODE_EULER_TO_ROTATION 1224
#define FN_NODE_QUATERNION_TO_ROTATION 1225
#define FN_NODE_ROTATION_TO_AXIS_ANGLE 1226
#define FN_NODE_ROTATION_TO_EULER 1227
#define FN_NODE_ROTATION_TO_QUATERNION 1228
#define FN_NODE_ROTATE_VECTOR 1229
#define FN_NODE_ROTATE_ROTATION 1230
#define FN_NODE_INVERT_ROTATION 1231
#define FN_NODE_TRANSFORM_POINT 1232
#define FN_NODE_TRANSFORM_DIRECTION 1233
#define FN_NODE_MATRIX_MULTIPLY 1234
#define FN_NODE_COMBINE_TRANSFORM 1235
#define FN_NODE_SEPARATE_TRANSFORM 1236
#define FN_NODE_INVERT_MATRIX 1237
#define FN_NODE_TRANSPOSE_MATRIX 1238
#define FN_NODE_PROJECT_POINT 1239
#define FN_NODE_ALIGN_ROTATION_TO_VECTOR 1240
#define FN_NODE_COMBINE_MATRIX 1241
#define FN_NODE_SEPARATE_MATRIX 1242
#define FN_NODE_INPUT_ROTATION 1243
#define FN_NODE_AXES_TO_ROTATION 1244
#define FN_NODE_HASH_VALUE 1245
#define FN_NODE_INTEGER_MATH 1246
#define FN_NODE_MATRIX_DETERMINANT 1247
#define FN_NODE_FIND_IN_STRING 1248

/** \} */
