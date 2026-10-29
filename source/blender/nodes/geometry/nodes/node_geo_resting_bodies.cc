/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "node_geometry_util.hh"
#include "resting_bridge.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

namespace blender::nodes::node_geo_resting_bodies_cc {
static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh"_ustr).supported_type(GeometryComponent::Type::Mesh).only_realized_data();
  /* Retain the identifier so links in existing node groups survive the label change. */
  b.add_input<decl::Vector>("Gravity"_ustr, "Initial Down"_ustr).default_value(float3(0,0,-1))
      .description("Downward direction in the input mesh coordinates; negligible-momentum rolling onto the local Z=0 plane");
  b.add_input<decl::Float>("Progress"_ustr).default_value(1).min(0).max(1)
      .description("Progress along the contact trajectory, measured by accumulated rotation angle");
  b.add_output<decl::Matrix>("Transform"_ustr).description("Rigid transform applied to the input mesh");
  b.add_output<decl::Geometry>("Trajectory"_ustr)
      .description("Noncyclic poly curve following the ordered rolling directions on the unit Gauss sphere");
  b.add_output<decl::Geometry>("Separatrix"_ustr)
      .description("Basin-boundary Bezier curves with two endpoints per arc and spherical tangent handles; rest_pose_a and rest_pose_b are curve attributes");
  b.add_output<decl::Geometry>("Stable Poses"_ustr)
      .description("Gauss sphere points with rest_transform (matrix), rest_probability, rest_normal and rest_height attributes; coplanar poses are merged");
}

static bool volume_center(const Mesh &mesh, std::array<double,3> &center)
{
  if (mesh.verts_num<4 || mesh.faces_num<4) return false;
  Array<int> counts(mesh.edges_num,0);
  Array<int> winding(mesh.edges_num,0);
  const Span<int> corner_edges=mesh.corner_edges();
  const Span<int> corner_vertices=mesh.corner_verts();
  const Span<int2> edges=mesh.edges();
  const OffsetIndices faces=mesh.faces();
  for (const int face_index : faces.index_range()) {
    const IndexRange face=faces[face_index];
    for (const int corner : face) {
      const int edge=corner_edges[corner];
      counts[edge]++;
      winding[edge]+=corner_vertices[corner]==edges[edge].x ? 1 : -1;
    }
  }
  for (int i=0;i<mesh.edges_num;i++) if (counts[i]!=2 || winding[i]!=0) return false;
  const Span<float3> positions=mesh.vert_positions();
  double3 origin(0);
  for (const float3 &p : positions) origin+=double3(p);
  origin/=double(positions.size());
  double scale=0;
  for (const float3 &p : positions) scale=std::max(scale,math::length(double3(p)-origin));
  if (!(scale>0)) return false;
  double volume=0;
  double3 moment(0);
  const Span<int> vertices=mesh.corner_verts();
  for (const int3 &tri : mesh.corner_tris()) {
    const double3 a=(double3(positions[vertices[tri.x]])-origin)/scale;
    const double3 b=(double3(positions[vertices[tri.y]])-origin)/scale;
    const double3 c=(double3(positions[vertices[tri.z]])-origin)/scale;
    const double v=math::dot(a,math::cross(b,c));
    volume+=v; moment+=(a+b+c)*(v/4.0);
  }
  if (std::abs(volume)<1e-12) return false;
  const double3 result=origin+scale*moment/volume;
  center={result.x,result.y,result.z};
  return true;
}

static float4x4 matrix_of(const RRB_Pose &pose)
{
  float4x4 matrix;
  for(int c=0;c<4;c++) for(int r=0;r<4;r++) matrix[c][r]=float(pose.matrix[c*4+r]);
  return matrix;
}
static GeometrySet pose_points(const std::vector<RRB_Pose> &poses)
{
  PointCloud *cloud=BKE_pointcloud_new_nomain(PointCloudType::Points,int(poses.size()));
  auto attrs=cloud->attributes_for_write();
  auto transforms=attrs.lookup_or_add_for_write_span<float4x4>("rest_transform",bke::AttrDomain::Point);
  auto probabilities=attrs.lookup_or_add_for_write_span<float>("rest_probability",bke::AttrDomain::Point);
  auto normals=attrs.lookup_or_add_for_write_span<float3>("rest_normal",bke::AttrDomain::Point);
  auto heights=attrs.lookup_or_add_for_write_span<float>("rest_height",bke::AttrDomain::Point);
  auto positions=cloud->positions_for_write();
  for(int i=0;i<int(poses.size());i++) {
    const auto &p=poses[i];
    positions[i]=float3(float(p.normal[0]),float(p.normal[1]),float(p.normal[2]));
    normals.span[i]=positions[i]; transforms.span[i]=matrix_of(p);
    probabilities.span[i]=float(p.probability); heights.span[i]=float(p.height);
  }
  transforms.finish();probabilities.finish();normals.finish();heights.finish();
  return GeometrySet::from_pointcloud(cloud);
}

static GeometrySet trajectory_curve(const std::vector<RRB_Pose> &trajectory)
{
  if (trajectory.empty()) return {};
  Curves *curves_id = bke::curves_new_nomain_single(int(trajectory.size()),CURVE_TYPE_POLY);
  MutableSpan<float3> positions = curves_id->geometry.wrap().positions_for_write();
  for (int i = 0; i < int(trajectory.size()); i++) {
    const auto &normal = trajectory[i].normal;
    positions[i] = float3(float(normal[0]),float(normal[1]),float(normal[2]));
  }
  return GeometrySet::from_curves(curves_id);
}

static GeometrySet separatrix_geometry(const std::vector<RRB_Separatrix> &arcs)
{
  if (arcs.empty()) return {};
  Curves *curves_id = bke::curves_new_nomain(int(arcs.size())*2,int(arcs.size()));
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  curves.fill_curve_types(CURVE_TYPE_BEZIER);
  curves.resolution_for_write().fill(32);
  curves.cyclic_for_write().fill(false);
  curves.handle_types_left_for_write().fill(BEZIER_HANDLE_ALIGN);
  curves.handle_types_right_for_write().fill(BEZIER_HANDLE_ALIGN);
  MutableSpan<int> offsets = curves.offsets_for_write();
  for (const int i : offsets.index_range()) offsets[i] = i*2;
  MutableSpan<float3> positions = curves.positions_for_write();
  MutableSpan<float3> handles_left = curves.handle_positions_left_for_write();
  MutableSpan<float3> handles_right = curves.handle_positions_right_for_write();
  auto attrs = curves.attributes_for_write();
  auto normals = attrs.lookup_or_add_for_write_span<float3>("rest_normal",bke::AttrDomain::Point);
  auto pose_a = attrs.lookup_or_add_for_write_span<int>("rest_pose_a",bke::AttrDomain::Curve);
  auto pose_b = attrs.lookup_or_add_for_write_span<int>("rest_pose_b",bke::AttrDomain::Curve);
  auto arc_index = attrs.lookup_or_add_for_write_span<int>("rest_arc",bke::AttrDomain::Curve);
  for (int i = 0; i < int(arcs.size()); i++) {
    const auto &arc = arcs[i];
    const double3 a(arc.start[0],arc.start[1],arc.start[2]);
    const double3 b(arc.end[0],arc.end[1],arc.end[2]);
    const double3 cross = math::cross(a,b);
    const double angle = std::atan2(math::length(cross),math::dot(a,b));
    const double3 plane_normal = math::normalize(cross);
    const double3 tangent_a = math::cross(plane_normal,a);
    const double3 tangent_b = math::cross(plane_normal,b);
    /* Cubic circular-arc approximation. Both derivatives lie in the great-circle
     * plane and are orthogonal to the corresponding sphere radius. A single
     * polynomial Bezier is an approximation between its exact sphere endpoints. */
    const double handle_length = (4.0/3.0)*std::tan(angle/4.0);
    const int start = 2*i, end = start+1;
    positions[start] = normals.span[start] = float3(a);
    positions[end] = normals.span[end] = float3(b);
    handles_left[start] = float3(a-handle_length*tangent_a);
    handles_right[start] = float3(a+handle_length*tangent_a);
    handles_left[end] = float3(b-handle_length*tangent_b);
    handles_right[end] = float3(b+handle_length*tangent_b);
    pose_a.span[i] = arc.pose_a;
    pose_b.span[i] = arc.pose_b;
    arc_index.span[i] = i;
  }
  normals.finish();pose_a.finish();pose_b.finish();arc_index.finish();
  return GeometrySet::from_curves(curves_id);
}

struct CacheEntry {
  std::vector<double> positions;
  std::array<double,3> center;
  std::unique_ptr<RRB_Solver,decltype(&rrb_free)> solver{nullptr,rrb_free};
};
static std::mutex cache_mutex;
static std::vector<CacheEntry> cache;

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry=params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh=geometry.get_mesh();
  if(!mesh) { params.set_default_remaining_outputs();return; }
  std::array<double,3> center;
  if(!volume_center(*mesh,center)) {
    params.error_message_add(NodeWarningType::Error,TIP_("Uniform density requires a closed, consistently oriented volumetric mesh"));
    params.set_default_remaining_outputs();return;
  }
  const float3 down=params.extract_input<float3>("Initial Down"_ustr);
  const double direction[3]={down.x,down.y,down.z};
  const float progress=params.extract_input<float>("Progress"_ustr);
  constexpr float angle = 0.002f;
  const bool trace_required = params.output_is_required("Transform"_ustr) ||
                              params.output_is_required("Trajectory"_ustr);
  std::vector<double> positions;
  positions.reserve(mesh->verts_num*3);
  for(const float3 &p:mesh->vert_positions()) positions.insert(positions.end(),{p.x,p.y,p.z});
  std::vector<RRB_Pose> poses,trajectory;
  std::vector<RRB_Separatrix> separatrices;
  RRB_Pose selected{};
  char error[512]={};
  {
    std::lock_guard lock(cache_mutex);
    auto found=std::find_if(cache.begin(),cache.end(),[&](const CacheEntry &entry) {
      return entry.center==center && entry.positions==positions;
    });
    if(found==cache.end()) {
      CacheEntry entry;
      entry.solver.reset(rrb_create(positions.data(),mesh->verts_num,center.data(),error));
      if(!entry.solver) {
        params.error_message_add(NodeWarningType::Error,error);
        params.set_default_remaining_outputs();return;
      }
      entry.positions=std::move(positions);entry.center=center;
      if(cache.size()>=8) cache.erase(cache.begin());
      cache.push_back(std::move(entry)); found=cache.end()-1;
    }
    RRB_Solver *solver=found->solver.get();
    poses.resize(rrb_pose_count(solver));
    for(int i=0;i<int(poses.size());i++) rrb_pose(solver,i,&poses[i]);
    if(trace_required) {
      if(!rrb_trace(solver,direction,angle,error)) {
        params.error_message_add(NodeWarningType::Error,error);
        params.set_default_remaining_outputs();return;
      }
      rrb_sample(solver,progress,&selected);
      RRB_Pose final_pose;
      rrb_sample(solver,1,&final_pose);
      for(const auto &pose:poses) {
        double distance=0;
        for(int j=0;j<3;j++) distance+=std::pow(pose.normal[j]-final_pose.normal[j],2);
        if(distance<1e-12) selected.probability=pose.probability;
      }
      if(params.output_is_required("Trajectory"_ustr)) {
        trajectory.resize(rrb_step_count(solver));
        for(int i=0;i<int(trajectory.size());i++) rrb_step(solver,i,&trajectory[i]);
      }
    }
    if(params.output_is_required("Separatrix"_ustr)) {
      separatrices.resize(rrb_separatrix_count(solver));
      for(int i=0;i<int(separatrices.size());i++) rrb_separatrix(solver,i,&separatrices[i]);
    }
  }
  const float4x4 transform=trace_required ? matrix_of(selected) : float4x4::identity();
  params.set_output("Stable Poses"_ustr,pose_points(poses));
  params.set_output("Trajectory"_ustr,trajectory_curve(trajectory));
  params.set_output("Separatrix"_ustr,separatrix_geometry(separatrices));
  params.set_output("Transform"_ustr,transform);
}
static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype,"GeometryNodeRestingBodies"_ustr,GEO_NODE_RESTING_BODIES);
  ntype.ui_name="Putting Rigid Bodies to Rest";
  ntype.default_width=200;
  ntype.ui_description="Putting Rigid Bodies to Rest (SIGGRAPH 2025): geometric rolling trajectories and analytical stable orientation probabilities under negligible momentum";
  ntype.enum_name_legacy="RESTING_BODIES";
  ntype.nclass=NODE_CLASS_GEOMETRY;
  ntype.declare=node_declare;
  ntype.geometry_node_execute=node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_resting_bodies_cc
