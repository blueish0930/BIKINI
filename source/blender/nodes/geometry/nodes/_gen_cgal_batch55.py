# Generate catalog-batch-55 geometry node TUs.
# Do not re-emit user-deleted nodes: volume_mesh_3, periodic_mesh_3,
# regularize_segments_2, minkowski_glide_3, kinetic_partition, ridge_curves,
# kinetic_reconstruct, polyfit, segment_voronoi_linf_2, surface_mesher,
# collision_detect, frechet_distance.
from pathlib import Path

DIR = Path(__file__).parent
H = "/* SPDX-FileCopyrightText: 2026 Blender Authors\n *\n * SPDX-License-Identifier: GPL-2.0-or-later */\n\n"

def emit(fname, ident, enum, ui, desc, body):
    ns = f"blender::nodes::node_geo_cgal_{fname}_cc"
    text = H + f'''#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace {ns} {{

{body}

static void node_register()
{{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "{ident}"_ustr, {enum});
  ntype.ui_name = "{ui}";
  ntype.ui_description = "{desc}";
  ntype.enum_name_legacy = "{enum[9:]}";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}}
NOD_REGISTER_NODE(node_register)
}}  // namespace {ns}
'''
    (DIR / f"node_geo_cgal_{fname}.cc").write_text(text, encoding="utf-8")
    print(fname)

POLY = r'''
static Curves *polylines_to_curves(const Span<Vector<float3>> polylines)
{
  int points_num = 0, curves_num = 0;
  for (const Vector<float3> &pl : polylines) {
    if (pl.size() >= 2) {
      points_num += pl.size();
      curves_num++;
    }
  }
  if (curves_num == 0) {
    return bke::curves_new_nomain(0, 0);
  }
  Curves *curves_id = bke::curves_new_nomain(points_num, curves_num);
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  MutableSpan<int> offsets = curves.offsets_for_write();
  MutableSpan<float3> positions = curves.positions_for_write();
  int p = 0, c = 0;
  offsets[0] = 0;
  for (const Vector<float3> &pl : polylines) {
    if (pl.size() < 2) {
      continue;
    }
    for (const float3 &pt : pl) {
      positions[p++] = pt;
    }
    c++;
    offsets[c] = p;
  }
  curves.fill_curve_types(CURVE_TYPE_POLY);
  return curves_id;
}
'''

# 1 kinetic partition
emit("kinetic_partition", "GeometryNodeCgalKineticPartition", "GEO_NODE_CGAL_KINETIC_PARTITION",
     "Kinetic Partition",
     "Split the bounding box into convex volumes by expanding input planes (CGAL Kinetic_space_partition_3). Face attribute volume_id.",
     r'''
static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh);
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Int>("K"_ustr).default_value(1).min(1).max(16);
  b.add_input<decl::Float>("BBox Dilation"_ustr).default_value(1.1f).min(1.0f).max(4.0f);
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m || m->faces_num < 1) {
    params.error_message_add(NodeWarningType::Info, TIP_("Kinetic Partition needs faces"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_kinetic_partition(*m, params.extract_input<int>("K"_ustr),
                                                    params.extract_input<float>("BBox Dilation"_ustr), error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Kinetic Partition failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}
''')

emit("kinetic_reconstruct", "GeometryNodeCgalKineticReconstruct", "GEO_NODE_CGAL_KINETIC_RECONSTRUCT",
     "Kinetic Reconstruct",
     "Piecewise-planar reconstruction from an oriented point cloud (CGAL Kinetic_surface_reconstruction_3).",
     r'''
static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr).only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh});
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Int>("K"_ustr).default_value(1).min(1).max(16);
  b.add_input<decl::Float>("Lambda"_ustr).default_value(0.5f).min(0.0f).max(0.99f);
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Points"_ustr);
  Span<float3> pts;
  Span<float3> nrm;
  if (const PointCloud *pc = g.get_pointcloud()) {
    pts = pc->positions();
    nrm = *pc->attributes().lookup<float3>("normal", bke::AttrDomain::Point);
  }
  else if (const Mesh *m = g.get_mesh()) {
    pts = m->vert_positions();
    nrm = m->vert_normals();
  }
  if (pts.size() < 30 || nrm.size() != pts.size()) {
    params.error_message_add(NodeWarningType::Info, TIP_("Kinetic Reconstruct needs oriented points (normal attribute or mesh verts)"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_kinetic_reconstruct(pts, nrm, params.extract_input<int>("K"_ustr),
                                                        params.extract_input<float>("Lambda"_ustr), error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Kinetic Reconstruct failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}
''')

emit("minkowski_glide_3", "GeometryNodeCgalMinkowskiGlide3", "GEO_NODE_CGAL_MINKOWSKI_GLIDE_3",
     "Minkowski Glide 3D",
     "Sweep a mesh along a polyline (CGAL Nef minkowski_sum_3). Distinct from Convex Minkowski Sum 3D.",
     r'''
static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh);
  b.add_input<decl::Geometry>("Path"_ustr).only_realized_data()
      .supported_type({GeometryComponent::Type::Curve, GeometryComponent::Type::Mesh});
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous();
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet gm = params.extract_input<GeometrySet>("Mesh"_ustr);
  GeometrySet gp = params.extract_input<GeometrySet>("Path"_ustr);
  const Mesh *mesh = gm.get_mesh();
  Vector<float3> path;
  if (const Curves *cu = gp.get_curves()) {
    const bke::CurvesGeometry &cg = cu->geometry.wrap();
    path.extend(cg.positions());
  }
  else if (const Mesh *pm = gp.get_mesh()) {
    path.extend(pm->vert_positions());
  }
  if (!mesh || path.size() < 2) {
    params.error_message_add(NodeWarningType::Info, TIP_("Minkowski Glide 3D needs a mesh and a polyline"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_minkowski_glide_3(*mesh, path, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Minkowski Glide 3D failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}
''')

emit("ridge_curves", "GeometryNodeCgalRidgeCurves", "GEO_NODE_CGAL_RIDGE_CURVES",
     "Ridge Curves",
     "Crest / max / min ridges on a triangle mesh (CGAL Ridges_3 + Monge jet). Distinct from Principal Curvature (a vertex field).",
     POLY + r'''
static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh);
  b.add_output<decl::Geometry>("Curves"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Int>("Mode"_ustr).default_value(2).min(0).max(2).description("0 max, 1 min, 2 crest");
  b.add_input<decl::Int>("Jet Degree"_ustr).default_value(3).min(2).max(4);
  b.add_input<decl::Int>("Neighbors"_ustr).default_value(18).min(6).max(128);
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m || m->faces_num < 1) {
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Vector<Vector<float3>> pl;
  if (!geometry::cgal_mesh_ridge_curves(*m, params.extract_input<int>("Mode"_ustr),
                                        params.extract_input<int>("Jet Degree"_ustr),
                                        params.extract_input<int>("Neighbors"_ustr), pl, error)) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Ridge Curves failed") : error);
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  params.set_output("Curves"_ustr, GeometrySet::from_curves(polylines_to_curves(pl)));
}
''')

emit("sphere_intersect", "GeometryNodeCgalSphereIntersect", "GEO_NODE_CGAL_SPHERE_INTERSECT",
     "Sphere Intersect",
     "Intersection circles of spheres centered at points (Circular kernel constructions).",
     POLY + r'''
static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr).only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh});
  b.add_output<decl::Geometry>("Curves"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Float>("Radius"_ustr).default_value(1.0f).min(0.0f).description("Used when the Radius field is unused.");
  b.add_input<decl::Float>("Point Radius"_ustr).default_value(1.0f).hide_value().structure_type(StructureType::Field);
  b.add_input<decl::Int>("Segments"_ustr).default_value(32).min(8).max(256);
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Points"_ustr);
  const float uniform = params.extract_input<float>("Radius"_ustr);
  Field<float> rad_f = params.extract_input<Field<float>>("Point Radius"_ustr);
  const int seg = params.extract_input<int>("Segments"_ustr);
  Span<float3> pts;
  int n = 0;
  const PointCloud *pc = g.get_pointcloud();
  const Mesh *mesh = g.get_mesh();
  if (pc) { pts = pc->positions(); n = pc->totpoint; }
  else if (mesh) { pts = mesh->vert_positions(); n = mesh->verts_num; }
  if (n < 2) {
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  Array<float> radii(n, uniform);
  if (pc) {
    const bke::PointCloudFieldContext ctx{*pc};
    fn::FieldEvaluator ev{ctx, n};
    ev.add_with_destination(rad_f, radii.as_mutable_span());
    ev.evaluate();
  }
  std::string error;
  Vector<Vector<float3>> pl;
  if (!geometry::cgal_points_sphere_intersect(pts, radii, uniform, seg, pl, error)) {
    params.error_message_add(NodeWarningType::Info, error.empty() ? TIP_("No intersecting spheres") : error);
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  params.set_output("Curves"_ustr, GeometrySet::from_curves(polylines_to_curves(pl)));
}
''')

def mesh_in_out(fname, ident, enum, ui, desc, extra_decl, extra_exec, extra_args):
    emit(fname, ident, enum, ui, desc, f'''
static void node_declare(NodeDeclarationBuilder &b)
{{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh);
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous();
  {extra_decl}
}}
static void node_geo_exec(GeoNodeExecParams params)
{{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m || m->faces_num < 1) {{
    params.error_message_add(NodeWarningType::Info, TIP_("{ui} needs a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }}
  {extra_exec}
  std::string error;
  Mesh *out = geometry::{extra_args};
  if (!out || (out->faces_num == 0 && out->edges_num == 0)) {{
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("{ui} failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }}
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}}
''')

mesh_in_out("sphere_arrangement", "GeometryNodeCgalSphereArrangement", "GEO_NODE_CGAL_SPHERE_ARRANGEMENT",
            "Sphere Arrangement",
            "Great circles of face planes on the unit sphere around Origin.",
            'b.add_input<decl::Vector>("Origin"_ustr).default_value(float3(0,0,0));',
            'const float3 o = params.extract_input<float3>("Origin"_ustr);',
            'cgal_mesh_sphere_arrangement(*m, o, error)')

mesh_in_out("segment_voronoi_linf_2", "GeometryNodeCgalSegmentVoronoiLinf2", "GEO_NODE_CGAL_SEGMENT_VORONOI_LINF_2",
            "Segment Voronoi Linf",
            "L-infinity segment Voronoi diagram of XY edges (CGAL Segment_Delaunay_graph_Linf_2).",
            "", "", "cgal_mesh_segment_voronoi_linf_2(*m, error)")

mesh_in_out("collision_detect", "GeometryNodeCgalCollisionDetect", "GEO_NODE_CGAL_COLLISION_DETECT",
            "Collision Detect",
            "Faces of mesh A that intersect mesh B (CGAL Rigid_triangle_mesh_collision_detection / AABB).",
            'b.add_input<decl::Geometry>("Other"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh);',
            'GeometrySet gb = params.extract_input<GeometrySet>("Other"_ustr);\n  const Mesh *bmesh = gb.get_mesh();\n  if (!bmesh) { params.set_output("Mesh"_ustr, GeometrySet()); return; }',
            'cgal_mesh_rigid_collision(*m, *bmesh, error)')

mesh_in_out("regularize_segments_2", "GeometryNodeCgalRegularizeSegments2", "GEO_NODE_CGAL_REGULARIZE_SEGMENTS_2",
            "Regularize Segments",
            "Snap nearly-parallel XY segments onto consensus lines (Shape_regularization intent, no OSQP).",
            'b.add_input<decl::Float>("Angle"_ustr).default_value(10.0f).min(0.1f).max(45.0f).subtype(PROP_ANGLE);\n  b.add_input<decl::Float>("Max Offset"_ustr).default_value(0.05f).min(0.0f);',
            'const float ang = params.extract_input<float>("Angle"_ustr);\n  const float off = params.extract_input<float>("Max Offset"_ustr);',
            'cgal_mesh_regularize_segments_2(*m, ang, off, error)')

mesh_in_out("volume_mesh_3", "GeometryNodeCgalVolumeMesh3", "GEO_NODE_CGAL_VOLUME_MESH_3",
            "Volume Mesh 3",
            "Quality tetrahedral mesh of a closed solid (CGAL Mesh_3 / make_mesh_3). Tet faces as a triangle mesh.",
            'b.add_input<decl::Float>("Facet Size"_ustr).default_value(0.0f).min(0.0f);\n  b.add_input<decl::Float>("Cell Size"_ustr).default_value(0.0f).min(0.0f);',
            'const float fs = params.extract_input<float>("Facet Size"_ustr);\n  const float cs = params.extract_input<float>("Cell Size"_ustr);',
            'cgal_mesh_volume_mesh_3(*m, fs, cs, error)')

mesh_in_out("tet_remesh", "GeometryNodeCgalTetRemesh", "GEO_NODE_CGAL_TET_REMESH",
            "Tet Remesh",
            "Quality tetrahedral remesh of a closed mesh interior (Mesh_3 sizing).",
            'b.add_input<decl::Float>("Target Edge"_ustr).default_value(0.1f).min(0.0f);',
            'const float te = params.extract_input<float>("Target Edge"_ustr);',
            'cgal_mesh_tet_remesh(*m, te, error)')

mesh_in_out("periodic_mesh_3", "GeometryNodeCgalPeriodicMesh3", "GEO_NODE_CGAL_PERIODIC_MESH_3",
            "Periodic Mesh 3",
            "Volume mesh of a closed solid (periodic Mesh_3 path; MSVC uses the quality Mesh_3 backend).",
            'b.add_input<decl::Float>("Cell Size"_ustr).default_value(0.0f).min(0.0f);',
            'const float cs = params.extract_input<float>("Cell Size"_ustr);',
            'cgal_mesh_periodic_mesh_3(*m, cs, error)')

mesh_in_out("surface_mesher", "GeometryNodeCgalSurfaceMesher", "GEO_NODE_CGAL_SURFACE_MESHER",
            "Surface Mesher",
            "Delaunay surface mesh of a signed-distance isosurface (CGAL Surface_mesher).",
            'b.add_input<decl::Float>("Offset"_ustr).default_value(0.0f);\n  b.add_input<decl::Float>("Angular Bound"_ustr).default_value(30.0f).min(1.0f).max(90.0f);\n  b.add_input<decl::Float>("Radius Bound"_ustr).default_value(0.0f).min(0.0f);',
            'const float off = params.extract_input<float>("Offset"_ustr);\n  const float ang = params.extract_input<float>("Angular Bound"_ustr);\n  const float rad = params.extract_input<float>("Radius Bound"_ustr);',
            'cgal_mesh_surface_mesher(*m, off, ang, rad, error)')

mesh_in_out("constrained_dt3", "GeometryNodeCgalConstrainedDt3", "GEO_NODE_CGAL_CONSTRAINED_DT3",
            "Constraint Delaunay 3D",
            "Interior Delaunay tetrahedra of a closed triangle mesh using only the input vertices.",
            "", "", "cgal_mesh_constrained_delaunay_3(*m, error)")

mesh_in_out("dual_contour", "GeometryNodeCgalDualContour", "GEO_NODE_CGAL_DUAL_CONTOUR",
            "Dual Contour",
            "Marching Cubes (mode 0) or Dual Contouring (mode 1) of signed distance to a mesh (CGAL Isosurfacing_3).",
            'b.add_input<decl::Float>("Isovalue"_ustr).default_value(0.0f);\n  b.add_input<decl::Int>("Resolution"_ustr).default_value(32).min(8).max(96);\n  b.add_input<decl::Int>("Mode"_ustr).default_value(1).min(0).max(1).description("0 Marching Cubes, 1 Dual Contouring");',
            'const float iso = params.extract_input<float>("Isovalue"_ustr);\n  const int res = params.extract_input<int>("Resolution"_ustr);\n  const int mode = params.extract_input<int>("Mode"_ustr);',
            'cgal_mesh_dual_contour(*m, iso, res, mode, error)')

mesh_in_out("cage_deform_3", "GeometryNodeCgalCageDeform3", "GEO_NODE_CGAL_CAGE_DEFORM_3",
            "Cage Deform 3D",
            "Deform an interior mesh with 3D mean-value coordinates of a triangle cage (CGAL 6.2 Barycentric_coordinates_3).",
            'b.add_input<decl::Geometry>("Cage Rest"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh);\n  b.add_input<decl::Geometry>("Cage Pose"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh);',
            '''GeometrySet gr = params.extract_input<GeometrySet>("Cage Rest"_ustr);
  GeometrySet gp = params.extract_input<GeometrySet>("Cage Pose"_ustr);
  const Mesh *rest = gr.get_mesh();
  const Mesh *pose = gp.get_mesh();
  if (!rest || !pose) { params.set_output("Mesh"_ustr, GeometrySet()); return; }''',
            'cgal_mesh_cage_deform_3(*rest, *pose, *m, error)')

emit("graphcut_segment", "GeometryNodeCgalGraphcutSegment", "GEO_NODE_CGAL_GRAPHCUT_SEGMENT",
     "Graphcut Segment",
     "Binary alpha-expansion graph-cut on faces from a Face float field (CGAL BGL graphcut).",
     r'''
static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh);
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Float>("Data"_ustr).default_value(0.5f).min(0.0f).max(1.0f).hide_value().structure_type(StructureType::Field);
  b.add_input<decl::Float>("Smoothness"_ustr).default_value(1.0f).min(0.0f);
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  Field<float> data_f = params.extract_input<Field<float>>("Data"_ustr);
  const float sm = params.extract_input<float>("Smoothness"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m || m->faces_num < 1) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  Array<float> data(m->faces_num, 0.5f);
  const bke::MeshFieldContext ctx{*m, AttrDomain::Face};
  fn::FieldEvaluator ev{ctx, m->faces_num};
  ev.add_with_destination(data_f, data.as_mutable_span());
  ev.evaluate();
  std::string error;
  Mesh *out = geometry::cgal_mesh_graphcut_segment(*m, data, sm, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Graphcut Segment failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}
''')

# Hyperbolic Delaunay node deleted (user request). Do not emit.

emit("point_features", "GeometryNodeCgalPointFeatures", "GEO_NODE_CGAL_POINT_FEATURES",
     "Point Features",
     "Classification eigen features: linearity, planarity, sphericity, omnivariance, anisotropy, verticality.",
     r'''
static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr).only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh});
  b.add_output<decl::Geometry>("Points"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Int>("Neighbors"_ustr).default_value(16).min(4).max(128);
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Points"_ustr);
  const int knn = params.extract_input<int>("Neighbors"_ustr);
  Span<float3> pts;
  if (const PointCloud *pc = g.get_pointcloud()) pts = pc->positions();
  else if (const Mesh *m = g.get_mesh()) pts = m->vert_positions();
  if (pts.size() < 4) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_points_classification_features(pts, knn, g.get_pointcloud(), g.get_mesh(), error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Point Features failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}
''')

# Classify node deleted (user request). Do not emit.

emit("register_icp", "GeometryNodeCgalRegisterIcp", "GEO_NODE_CGAL_REGISTER_ICP",
     "Register ICP",
     "Rigid ICP of source points onto target using CGAL AABB/kd-tree closest points + Kabsch.",
     r'''
static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Source"_ustr).only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh});
  b.add_input<decl::Geometry>("Target"_ustr).only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh});
  b.add_output<decl::Geometry>("Points"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Int>("Iterations"_ustr).default_value(20).min(1).max(80);
}
static Span<float3> pts_of(const GeometrySet &g)
{
  if (const PointCloud *pc = g.get_pointcloud()) return pc->positions();
  if (const Mesh *m = g.get_mesh()) return m->vert_positions();
  return {};
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet gs = params.extract_input<GeometrySet>("Source"_ustr);
  GeometrySet gt = params.extract_input<GeometrySet>("Target"_ustr);
  Span<float3> src = pts_of(gs), tgt = pts_of(gt);
  if (src.size() < 3 || tgt.size() < 3) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_points_register_icp(src, tgt, params.extract_input<int>("Iterations"_ustr), error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("ICP failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}
''')

emit("super4pcs", "GeometryNodeCgalSuper4pcs", "GEO_NODE_CGAL_SUPER4PCS",
     "Super4PCS",
     "Global 4-point congruent set registration plus ICP (OpenGR Super4PCS intent, no extra lib).",
     r'''
static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Source"_ustr).only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh});
  b.add_input<decl::Geometry>("Target"_ustr).only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh});
  b.add_output<decl::Geometry>("Points"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Int>("Samples"_ustr).default_value(80).min(8).max(400);
  b.add_input<decl::Int>("ICP Iterations"_ustr).default_value(12).min(1).max(80);
}
static Span<float3> pts_of(const GeometrySet &g)
{
  if (const PointCloud *pc = g.get_pointcloud()) return pc->positions();
  if (const Mesh *m = g.get_mesh()) return m->vert_positions();
  return {};
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet gs = params.extract_input<GeometrySet>("Source"_ustr);
  GeometrySet gt = params.extract_input<GeometrySet>("Target"_ustr);
  Span<float3> src = pts_of(gs), tgt = pts_of(gt);
  if (src.size() < 4 || tgt.size() < 4) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_points_register_4pcs(src, tgt, params.extract_input<int>("Samples"_ustr),
                                                       params.extract_input<int>("ICP Iterations"_ustr), error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Super4PCS failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}
''')

emit("polyfit", "GeometryNodeCgalPolyfit", "GEO_NODE_CGAL_POLYFIT",
     "PolyFit",
     "Piecewise-planar reconstruction: region-grow planes and emit convex n-gons (no SCIP MIP).",
     r'''
static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr).only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh});
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Int>("Neighbors"_ustr).default_value(12).min(6).max(64);
  b.add_input<decl::Float>("Max Distance"_ustr).default_value(0.0f).min(0.0f);
  b.add_input<decl::Float>("Max Angle"_ustr).default_value(25.0f).min(1.0f).max(90.0f);
  b.add_input<decl::Int>("Min Region"_ustr).default_value(30).min(8).max(100000);
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Points"_ustr);
  Span<float3> pts, nrm;
  if (const PointCloud *pc = g.get_pointcloud()) {
    pts = pc->positions();
  }
  else if (const Mesh *m = g.get_mesh()) {
    pts = m->vert_positions();
    nrm = m->vert_normals();
  }
  if (pts.size() < 12) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_polyfit_reconstruct(pts, nrm, params.extract_input<int>("Neighbors"_ustr),
                                                        params.extract_input<float>("Max Distance"_ustr),
                                                        params.extract_input<float>("Max Angle"_ustr),
                                                        params.extract_input<int>("Min Region"_ustr), error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("PolyFit failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}
''')

emit("frechet_distance", "GeometryNodeCgalFrechetDistance", "GEO_NODE_CGAL_FRECHET_DISTANCE",
     "Frechet Distance",
     "Approximate Fréchet distance of two polylines (CGAL 6.1 Frechet_distance).",
     r'''
static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Curve A"_ustr).only_realized_data()
      .supported_type({GeometryComponent::Type::Curve, GeometryComponent::Type::Mesh});
  b.add_input<decl::Geometry>("Curve B"_ustr).only_realized_data()
      .supported_type({GeometryComponent::Type::Curve, GeometryComponent::Type::Mesh});
  b.add_output<decl::Float>("Distance"_ustr);
  b.add_input<decl::Float>("Error Bound"_ustr).default_value(0.01f).min(1e-6f);
}
static Span<float3> curve_pts(const GeometrySet &g)
{
  if (const Curves *cu = g.get_curves()) return cu->geometry.wrap().positions();
  if (const Mesh *m = g.get_mesh()) return m->vert_positions();
  return {};
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet ga = params.extract_input<GeometrySet>("Curve A"_ustr);
  GeometrySet gb = params.extract_input<GeometrySet>("Curve B"_ustr);
  const float eb = params.extract_input<float>("Error Bound"_ustr);
  Span<float3> a = curve_pts(ga), b = curve_pts(gb);
  float d = 0.0f;
  std::string error;
  if (!geometry::cgal_curves_frechet_distance(a, b, eb, d, error)) {
    if (!error.empty()) params.error_message_add(NodeWarningType::Warning, error);
    params.set_output("Distance"_ustr, 0.0f);
    return;
  }
  params.set_output("Distance"_ustr, d);
}
''')

emit("alpha_wrap_2", "GeometryNodeCgalAlphaWrap2", "GEO_NODE_CGAL_ALPHA_WRAP_2",
     "Alpha Wrap 2D",
     "Watertight 2D wrap of XY points or segments (CGAL 6.2 Alpha_wrap_2).",
     r'''
static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Geometry"_ustr).only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh});
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Float>("Alpha"_ustr).default_value(0.1f).min(1e-6f);
  b.add_input<decl::Float>("Offset"_ustr).default_value(0.01f).min(0.0f);
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Geometry"_ustr);
  const float alpha = params.extract_input<float>("Alpha"_ustr);
  const float offset = params.extract_input<float>("Offset"_ustr);
  Span<float3> pts;
  const Mesh *mesh = g.get_mesh();
  if (const PointCloud *pc = g.get_pointcloud()) pts = pc->positions();
  else if (mesh) pts = mesh->vert_positions();
  std::string error;
  Mesh *out = geometry::cgal_points_alpha_wrap_2(pts, mesh, alpha, offset, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Alpha Wrap 2D failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}
''')

print("generated all batch 55 nodes")

