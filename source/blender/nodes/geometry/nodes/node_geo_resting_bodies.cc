/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_transform.hh"
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
  b.use_custom_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr).supported_type(GeometryComponent::Type::Mesh).only_realized_data();
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous();
  b.add_input<decl::Vector>("Initial Down"_ustr).default_value(float3(0.2f,0.3f,-1.0f))
      .description("Downward direction in the input mesh coordinates; negligible-momentum rolling onto the local Z=0 plane");
  b.add_input<decl::Float>("Progress"_ustr).default_value(1).min(0).max(1)
      .description("Progress along the contact trajectory, measured by accumulated rotation angle");
  b.add_input<decl::Bool>("Use Stable Pose"_ustr).default_value(false)
      .description("Choose a stable orientation directly instead of following the initial direction");
  b.add_input<decl::Int>("Pose Index"_ustr).default_value(0).min(0)
      .description("Stable orientations are ordered by decreasing probability");
  b.add_input<decl::Bool>("Custom Center"_ustr).default_value(false)
      .description("Use a supplied center of mass; otherwise integrate the closed input surface with uniform density");
  b.add_input<decl::Vector>("Center"_ustr).description("Custom center of mass, strictly inside the convex hull");
  b.add_input<decl::Float>("Angle Step"_ustr).default_value(0.04f).min(0.002f).max(0.5f).subtype(PROP_ANGLE)
      .description("Maximum angular spacing of trajectory output points; does not change the analytical contact path");
  b.add_output<decl::Geometry>("Stable Poses"_ustr)
      .description("Gauss sphere points with rest_transform (matrix), rest_probability, rest_normal and rest_height attributes; coplanar poses are merged");
  b.add_output<decl::Geometry>("Trajectory"_ustr)
      .description("Ordered Gauss sphere samples with rest_transform, rest_normal and rest_height attributes");
  b.add_output<decl::Matrix>("Transform"_ustr).description("Rigid transform applied to the input mesh");
  b.add_output<decl::Vector>("Center"_ustr).description("Center of mass in input coordinates");
  b.add_output<decl::Int>("Pose Count"_ustr);
  b.add_output<decl::Float>("Probability"_ustr).description("Resting probability of the chosen/final stable orientation under uniform initial orientations");
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
  const bool custom=params.extract_input<bool>("Custom Center"_ustr);
  const float3 custom_center=params.extract_input<float3>("Center"_ustr);
  std::array<double,3> center{custom_center.x,custom_center.y,custom_center.z};
  if(!custom && !volume_center(*mesh,center)) {
    params.error_message_add(NodeWarningType::Error,TIP_("Uniform density requires a closed, consistently oriented volumetric mesh; alternatively supply Custom Center"));
    params.set_default_remaining_outputs();return;
  }
  const float3 down=params.extract_input<float3>("Initial Down"_ustr);
  const double direction[3]={down.x,down.y,down.z};
  const float progress=params.extract_input<float>("Progress"_ustr);
  const float angle=params.extract_input<float>("Angle Step"_ustr);
  const bool use_pose=params.extract_input<bool>("Use Stable Pose"_ustr);
  const int index=params.extract_input<int>("Pose Index"_ustr);
  std::vector<double> positions;
  positions.reserve(mesh->verts_num*3);
  for(const float3 &p:mesh->vert_positions()) positions.insert(positions.end(),{p.x,p.y,p.z});
  std::vector<RRB_Pose> poses,trajectory;
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
    if(use_pose) selected=poses[std::clamp(index,0,int(poses.size())-1)];
    else {
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
  }
  const float4x4 transform=matrix_of(selected);
  geometry::transform_geometry(geometry,transform);
  params.set_output("Mesh"_ustr,std::move(geometry));
  params.set_output("Stable Poses"_ustr,pose_points(poses));
  params.set_output("Trajectory"_ustr,pose_points(trajectory));
  params.set_output("Transform"_ustr,transform);
  params.set_output("Center"_ustr,float3(float(center[0]),float(center[1]),float(center[2])));
  params.set_output("Pose Count"_ustr,int(poses.size()));
  params.set_output("Probability"_ustr,float(selected.probability));
}
static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype,"GeometryNodeRestingBodies"_ustr,GEO_NODE_RESTING_BODIES);
  ntype.ui_name="Resting Bodies";
  ntype.ui_description="Putting Rigid Bodies to Rest (SIGGRAPH 2025): geometric rolling trajectories and analytical stable orientation probabilities under negligible momentum";
  ntype.enum_name_legacy="RESTING_BODIES";
  ntype.nclass=NODE_CLASS_GEOMETRY;
  ntype.declare=node_declare;
  ntype.geometry_node_execute=node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_resting_bodies_cc
