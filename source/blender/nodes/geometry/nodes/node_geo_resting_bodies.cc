/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
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
      .description("Ordered Gauss sphere samples with rest_transform, rest_normal and rest_height attributes");
  b.add_output<decl::Geometry>("Separatrix"_ustr)
      .description("Great-circle attraction-basin boundaries on the unit Gauss sphere; separate wire arcs with rest_pose_a and rest_pose_b edge attributes");
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

static GeometrySet separatrix_geometry(const std::vector<RRB_Separatrix> &arcs)
{
  if (arcs.empty()) return {};
  constexpr double max_angle_step = 0.02;
  std::vector<int> subdivisions;
  int vertices_num = 0, edges_num = 0;
  for (const auto &arc : arcs) {
    const double3 a(arc.start[0],arc.start[1],arc.start[2]);
    const double3 b(arc.end[0],arc.end[1],arc.end[2]);
    const double angle = std::acos(std::clamp(math::dot(a,b),-1.0,1.0));
    const int count = std::max(1,int(std::ceil(angle/max_angle_step)));
    subdivisions.push_back(count);
    vertices_num += count+1; edges_num += count;
  }
  Mesh *mesh = BKE_mesh_new_nomain(vertices_num,edges_num,0,0);
  MutableSpan<float3> positions = mesh->vert_positions_for_write();
  MutableSpan<int2> edges = mesh->edges_for_write();
  auto attrs = mesh->attributes_for_write();
  auto normals = attrs.lookup_or_add_for_write_span<float3>("rest_normal",bke::AttrDomain::Point);
  auto pose_a = attrs.lookup_or_add_for_write_span<int>("rest_pose_a",bke::AttrDomain::Edge);
  auto pose_b = attrs.lookup_or_add_for_write_span<int>("rest_pose_b",bke::AttrDomain::Edge);
  auto arc_index = attrs.lookup_or_add_for_write_span<int>("rest_arc",bke::AttrDomain::Edge);
  int vertex_offset = 0, edge_offset = 0;
  for (int i = 0; i < int(arcs.size()); i++) {
    const auto &arc = arcs[i];
    const double3 a(arc.start[0],arc.start[1],arc.start[2]);
    const double3 b(arc.end[0],arc.end[1],arc.end[2]);
    const double angle = std::acos(std::clamp(math::dot(a,b),-1.0,1.0));
    const int count = subdivisions[i];
    for (int j = 0; j <= count; j++) {
      const double t = double(j)/count;
      const double3 p = angle < 1e-8 ? math::normalize((1-t)*a+t*b) :
          math::normalize((std::sin((1-t)*angle)*a+std::sin(t*angle)*b)/std::sin(angle));
      positions[vertex_offset+j] = float3(p);
      normals.span[vertex_offset+j] = float3(p);
    }
    for (int j = 0; j < count; j++) {
      edges[edge_offset+j] = int2(vertex_offset+j,vertex_offset+j+1);
      pose_a.span[edge_offset+j] = arc.pose_a;
      pose_b.span[edge_offset+j] = arc.pose_b;
      arc_index.span[edge_offset+j] = i;
    }
    vertex_offset += count+1; edge_offset += count;
  }
  normals.finish();pose_a.finish();pose_b.finish();arc_index.finish();
  return GeometrySet::from_mesh(mesh);
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
  params.set_output("Trajectory"_ustr,pose_points(trajectory));
  params.set_output("Separatrix"_ustr,separatrix_geometry(separatrices));
  params.set_output("Transform"_ustr,transform);
}
static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype,"GeometryNodeRestingBodies"_ustr,GEO_NODE_RESTING_BODIES);
  ntype.ui_name="Putting Rigid Bodies to Rest";
  ntype.default_width=240;
  ntype.ui_description="Putting Rigid Bodies to Rest (SIGGRAPH 2025): geometric rolling trajectories and analytical stable orientation probabilities under negligible momentum";
  ntype.enum_name_legacy="RESTING_BODIES";
  ntype.nclass=NODE_CLASS_GEOMETRY;
  ntype.declare=node_declare;
  ntype.geometry_node_execute=node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_resting_bodies_cc
