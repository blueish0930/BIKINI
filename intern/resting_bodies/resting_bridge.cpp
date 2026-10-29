/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "resting_bridge.h"
#include "boundary_tools.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cstring>
#include <memory>
#include <set>

using V = Eigen::Vector3d;
using M = Eigen::Matrix4d;
using Q = Eigen::Quaterniond;
static V ev(Vector3 v) { return {v.x, v.y, v.z}; }
static Vector3 gv(V v) { return {v.x(), v.y(), v.z()}; }
static void error_text(char *output, const char *message)
{
  std::strncpy(output, message, 511);
  output[511] = 0;
}
struct Step {
  M transform;
  V normal;
  V pivot;
  double angle;
};
struct RRB_Solver {
  /* Reverse destruction order keeps geometry alive until solver data is released. */
  std::unique_ptr<ManifoldSurfaceMesh> mesh;
  std::unique_ptr<VertexPositionGeometry> geometry;
  std::unique_ptr<Forward3DSolver> core;
  std::vector<V> points;
  V center;
  double scale;
  std::vector<RRB_Pose> poses;
  std::vector<RRB_Separatrix> separatrices;
  std::vector<Step> steps;
  V last_direction = V::Zero();
  double last_step = 0;
};
static double support(const RRB_Solver &s, const V &n)
{
  double height = -std::numeric_limits<double>::infinity();
  for (const V &p : s.points) height = std::max(height, p.dot(n));
  return height;
}
static M grounded_transform(const RRB_Solver &s, const V &n)
{
  M result = M::Identity();
  result.block<3,3>(0,0) = Q::FromTwoVectors(n, V(0,0,-1)).toRotationMatrix();
  result(2,3) = support(s,n);
  return result;
}
static RRB_Pose export_pose(const RRB_Solver &s, const M &normalized, const V &normal)
{
  RRB_Pose result{};
  M transform = normalized;
  transform.block<3,1>(0,3) = s.scale * normalized.block<3,1>(0,3)
      - normalized.block<3,3>(0,0) * s.center + V(s.center.x(),s.center.y(),0);
  std::copy(transform.data(),transform.data()+16,result.matrix);
  std::copy(normal.data(),normal.data()+3,result.normal);
  result.height = normalized(2,3)*s.scale;
  return result;
}
extern "C" {
RRB_Solver *rrb_create(const double *positions, int count, const double center[3], char error[512])
{
  try {
    if (count < 4) throw std::runtime_error("A volumetric mesh requires at least four vertices");
    auto s = std::make_unique<RRB_Solver>();
    s->center = Eigen::Map<const V>(center);
    if (!s->center.allFinite()) throw std::runtime_error("Center of mass is not finite");
    s->scale = 0;
    for (int i=0;i<count;i++) {
      V p = Eigen::Map<const V>(positions+3*i)-s->center;
      if (!p.allFinite()) throw std::runtime_error("Mesh contains nonfinite positions");
      s->scale = std::max(s->scale,p.norm());
      s->points.push_back(p);
    }
    if (!(s->scale > 0)) throw std::runtime_error("Mesh has zero extent");
    std::vector<Vector3> points;
    for (V &p : s->points) { p /= s->scale; points.push_back(gv(p)); }
    auto [faces, mapping, hull_points] = get_convex_hull(points);
    s->mesh = std::make_unique<ManifoldSurfaceMesh>(faces);
    s->geometry = std::make_unique<VertexPositionGeometry>(*s->mesh);
    s->points.clear();
    for (Vertex v : s->mesh->vertices()) {
      s->geometry->inputVertexPositions[v] = hull_points[v.getIndex()];
      s->points.push_back(ev(hull_points[v.getIndex()]));
    }
    for (Face f : s->mesh->faces()) {
      const V n = ev(s->geometry->faceNormal(f));
      if (n.dot(s->points[f.halfedge().vertex().getIndex()]) <= 1e-10)
        throw std::runtime_error("Center of mass must lie strictly inside the convex hull");
    }
    s->core = std::make_unique<Forward3DSolver>(s->mesh.get(),s->geometry.get(),Vector3{0,0,0},false);
    s->core->initialize_pre_computes();
    BoundaryBuilder boundaries(s->core.get());
    boundaries.build_boundary_normals();
    double sum=0;
    for (Face f : s->mesh->faces()) {
      double probability = boundaries.face_region_area[f]/(4*PI);
      if (!std::isfinite(probability) || probability < -1e-8)
        throw std::runtime_error("Invalid spherical basin area; check degenerate hull geometry");
      if (probability <= 1e-12) continue;
      V normal = ev(s->geometry->faceNormal(f));
      sum += probability;
      auto found = std::find_if(s->poses.begin(),s->poses.end(),[&](const RRB_Pose &p) {
        return (Eigen::Map<const V>(p.normal)-normal).squaredNorm()<1e-18;
      });
      if (found != s->poses.end()) found->probability += probability;
      else {
        auto pose = export_pose(*s,grounded_transform(*s,normal),normal);
        pose.probability = probability;
        s->poses.push_back(pose);
      }
    }
    if (std::abs(sum-1)>1e-6 || s->poses.empty())
      throw std::runtime_error("Resting basin probabilities do not sum to one");
    std::stable_sort(s->poses.begin(),s->poses.end(),[](const auto &a,const auto &b) {
      return a.probability>b.probability;
    });
    /* Export the same ascending manifolds used for the basin-area integral.
     * A boundary point at a saddle carries its two terminal stable faces;
     * its neighbors are the backward trace toward maxima, not other saddles
     * in iteration order. Keep these arcs independent in the Blender output. */
    const auto pose_index = [&](Face face) {
      const V normal = ev(s->geometry->faceNormal(face));
      for (int i = 0; i < int(s->poses.size()); i++) {
        if ((Eigen::Map<const V>(s->poses[i].normal)-normal).squaredNorm() < 1e-18) {
          return i;
        }
      }
      throw std::runtime_error("Separatrix has no corresponding stable pose");
    };
    std::set<std::pair<size_t, size_t>> exported;
    for (Edge edge : s->mesh->edges()) {
      for (const BoundaryNormal *point : boundaries.edge_boundary_normals[edge]) {
        const int a = pose_index(point->f1), b = pose_index(point->f2);
        /* Triangles on one planar support are one stable pose, so a seam
         * between them is not an attraction-basin boundary. */
        if (a == b) continue;
        for (const BoundaryNormal *neighbor : point->neighbors) {
          const auto key = std::minmax(point->index, neighbor->index);
          if (!exported.emplace(key.first, key.second).second) continue;
          V start = ev(point->normal), end = ev(neighbor->normal);
          if (!start.allFinite() || !end.allFinite() || start.norm() < 1e-12 || end.norm() < 1e-12) {
            throw std::runtime_error("Invalid separatrix direction");
          }
          start.normalize(); end.normalize();
          if ((start-end).squaredNorm() < 1e-20) continue;
          if (start.dot(end) < -1.0+1e-12) {
            throw std::runtime_error("Ambiguous antipodal separatrix arc");
          }
          RRB_Separatrix arc{};
          std::copy(start.data(), start.data()+3, arc.start);
          std::copy(end.data(), end.data()+3, arc.end);
          arc.pose_a = std::min(a,b); arc.pose_b = std::max(a,b);
          s->separatrices.push_back(arc);
        }
      }
    }
    error[0]=0;
    return s.release();
  } catch (const std::exception &e) { error_text(error,e.what()); return nullptr; }
}
void rrb_free(RRB_Solver *s) { delete s; }
int rrb_pose_count(const RRB_Solver *s) { return int(s->poses.size()); }
void rrb_pose(const RRB_Solver *s,int i,RRB_Pose *p) { *p=s->poses.at(i); }
int rrb_separatrix_count(const RRB_Solver *s) { return int(s->separatrices.size()); }
void rrb_separatrix(const RRB_Solver *s,int i,RRB_Separatrix *arc) { *arc=s->separatrices.at(i); }
int rrb_trace(RRB_Solver *s,const double direction[3],double angle_step,char error[512])
{
  try {
    V n=Eigen::Map<const V>(direction);
    if (!n.allFinite() || n.norm()<1e-12) throw std::runtime_error("Initial down direction must be nonzero");
    n.normalize();
    angle_step=std::clamp(angle_step,0.002,0.5);
    if (n==s->last_direction && angle_step==s->last_step && !s->steps.empty()) return 1;
    std::vector<V> events{n};
    bool at_rest=false;
    for (const auto &p : s->poses) {
      if ((Eigen::Map<const V>(p.normal)-n).norm()<1e-10) at_rest=true;
    }
    if (!at_rest) {
      s->core->find_contact(gv(n));
      for (size_t i=0;!s->core->stable_state;i++) {
        if (i>8*s->mesh->nEdges()+32) throw std::runtime_error("Contact trace did not converge");
        s->core->next_state();
        V next=ev(s->core->curr_g_vec);
        if (!next.allFinite() || next.norm()<1e-10)
          throw std::runtime_error("Initial orientation is a singular equilibrium; choose a nearby direction");
        next.normalize();
        if (support(*s,next)>support(*s,events.back())+1e-8)
          throw std::runtime_error("Contact trace increased gravitational potential");
        if ((next-events.back()).norm()>1e-10) events.push_back(next);
      }
      if (!s->core->face_is_stable(s->core->curr_f)) throw std::runtime_error("Contact trace ended at an unstable support");
    }
    std::vector<Step> steps;
    M current=grounded_transform(*s,n);
    steps.push_back({current,n,V::Zero(),0});
    double total_angle=0;
    for (size_t i=1;i<events.size();i++) {
      V a=events[i-1],b=events[i];
      double theta=std::acos(std::clamp(a.dot(b),-1.0,1.0));
      if(theta<1e-10) continue;
      V midpoint=(a+b).normalized();
      auto contact=std::max_element(s->points.begin(),s->points.end(),[&](const V &p,const V &q) {
        return p.dot(midpoint)<q.dot(midpoint);
      });
      V pivot=current.block<3,3>(0,0)*(*contact)+current.block<3,1>(0,3);
      const M start=current;
      int subdivisions=int(std::ceil(theta/angle_step));
      for(int j=1;j<=subdivisions;j++) {
        double t=double(j)/subdivisions;
        V normal=(std::sin((1-t)*theta)*a+std::sin(t*theta)*b)/std::sin(theta);
        auto delta=Q::FromTwoVectors(start.block<3,3>(0,0)*normal,V(0,0,-1)).toRotationMatrix();
        current.block<3,3>(0,0)=delta*start.block<3,3>(0,0);
        current.block<3,1>(0,3)=pivot+delta*(start.block<3,1>(0,3)-pivot);
        double lowest=std::numeric_limits<double>::infinity();
        for(const V &p:s->points) lowest=std::min(lowest,(current.block<3,3>(0,0)*p+current.block<3,1>(0,3)).z());
        if(std::abs(lowest)>1e-7) throw std::runtime_error("Rolling contact left the ground plane");
        steps.push_back({current,normal,pivot,total_angle+t*theta});
      }
      total_angle+=theta;
    }
    s->steps=std::move(steps); s->last_direction=n; s->last_step=angle_step;
    error[0]=0;
    return 1;
  } catch(const std::exception &e) { error_text(error,e.what()); return 0; }
}
int rrb_step_count(const RRB_Solver *s) { return int(s->steps.size()); }
void rrb_step(const RRB_Solver *s,int i,RRB_Pose *p) {
  const Step &step=s->steps.at(i); *p=export_pose(*s,step.transform,step.normal);
}
void rrb_sample(const RRB_Solver *s,double progress,RRB_Pose *p) {
  double angle=std::clamp(progress,0.0,1.0)*s->steps.back().angle;
  auto found=std::lower_bound(s->steps.begin(),s->steps.end(),angle,[](const Step &step,double a) { return step.angle<a; });
  if(found==s->steps.begin()) { rrb_step(s,0,p); return; }
  if(found==s->steps.end()) { rrb_step(s,int(s->steps.size())-1,p); return; }
  const Step &b=*found,&a=*(found-1);
  double t=(angle-a.angle)/(b.angle-a.angle);
  Q rotation=Q(a.transform.block<3,3>(0,0)).slerp(t,Q(b.transform.block<3,3>(0,0)));
  M result=M::Identity(); result.block<3,3>(0,0)=rotation.toRotationMatrix();
  result.block<3,1>(0,3)=b.pivot+result.block<3,3>(0,0)*a.transform.block<3,3>(0,0).transpose()*(a.transform.block<3,1>(0,3)-b.pivot);
  V normal=result.block<3,3>(0,0).transpose()*V(0,0,-1);
  *p=export_pose(*s,result,normal);
}
}
