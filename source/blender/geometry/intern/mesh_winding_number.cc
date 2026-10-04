/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_winding_number.hh"

#include "BLI_array.hh"
#include "BLI_index_range.hh"
#include "BLI_math_base.hh"
#include "BLI_math_constants.hh"
#include "BLI_math_geom.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_offset_indices.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"

#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

namespace blender::geometry {
namespace {

constexpr double inv_four_pi = 1.0 / (4.0 * double(M_PI));
constexpr int leaf_size = 4;

static double3 to_d(const float3 &v)
{
  return double3(double(v.x), double(v.y), double(v.z));
}

/**
 * Signed solid angle of triangle abc as seen from p (van Oosterom & Strackee).
 * Uses the HDK-stable numerator. Result is in steradians; divide by 4π for winding.
 */
static double solid_angle(const float3 &p, const float3 &a, const float3 &b, const float3 &c)
{
  const double3 qa = to_d(a) - to_d(p);
  const double3 qb = to_d(b) - to_d(p);
  const double3 qc = to_d(c) - to_d(p);
  const double la = math::length(qa);
  const double lb = math::length(qb);
  const double lc = math::length(qc);
  if (la == 0.0 || lb == 0.0 || lc == 0.0) {
    return 0.0;
  }
  const double3 ua = qa / la;
  const double3 ub = qb / lb;
  const double3 uc = qc / lc;
  const double numerator = math::dot(ua, math::cross(ub - ua, uc - ua));
  if (numerator == 0.0) {
    return 0.0;
  }
  const double denominator = 1.0 + math::dot(ua, ub) + math::dot(ua, uc) + math::dot(ub, uc);
  return 2.0 * std::atan2(numerator, denominator);
}

struct Tri {
  float3 a, b, c;
  float3 centroid;
  float3 dipole;
  float area = 0.0f;
};

struct Node {
  float3 center = float3(0.0f);
  float3 dipole = float3(0.0f);
  float3 Nij_diag = float3(0.0f);
  float Nxy_Nyx = 0.0f;
  float Nyz_Nzy = 0.0f;
  float Nzx_Nxz = 0.0f;
  float radius2 = 0.0f;
  int left = -1;
  int right = -1;
  int tri_start = 0;
  int tri_count = 0;
};

class WindingTree {
 public:
  Array<Tri> tris;
  Array<int> order;
  Vector<Node> nodes;
  float beta = 2.0f;

  void build(const Mesh &mesh)
  {
    const Span<float3> positions = mesh.vert_positions();
    const OffsetIndices faces = mesh.faces();
    const Span<int> corner_verts = mesh.corner_verts();

    Vector<Tri> packed;
    packed.reserve(poly_to_tri_count(int(faces.size()), int(corner_verts.size())));
    for (const int face_i : faces.index_range()) {
      const IndexRange face = faces[face_i];
      if (face.size() < 3) {
        continue;
      }
      const int i0 = corner_verts[face[0]];
      for (int k = 2; k < face.size(); k++) {
        const float3 a = positions[i0];
        const float3 b = positions[corner_verts[face[k - 1]]];
        const float3 c = positions[corner_verts[face[k]]];
        const float3 dipole = math::cross(b - a, c - a) * 0.5f;
        const float area_sq = math::length_squared(dipole);
        if (area_sq < 1.0e-24f) {
          continue;
        }
        Tri tri;
        tri.a = a;
        tri.b = b;
        tri.c = c;
        tri.centroid = (a + b + c) / 3.0f;
        tri.dipole = dipole;
        tri.area = math::sqrt(area_sq);
        packed.append(tri);
      }
    }

    tris.reinitialize(packed.size());
    tris.as_mutable_span().copy_from(packed);
    order.reinitialize(tris.size());
    for (const int i : tris.index_range()) {
      order[i] = i;
    }
    nodes.clear();
    if (!tris.is_empty()) {
      nodes.reserve(size_t(tris.size()) * 2);
      build_range(0, int(tris.size()));
    }
  }

  float eval(const float3 &p, const bool exact) const
  {
    if (nodes.is_empty()) {
      return 0.0f;
    }
    if (exact) {
      return float(eval_exact(p));
    }
    return float(eval_fast(p));
  }

 private:
  double eval_exact(const float3 &p) const
  {
    double sum = 0.0;
    for (const Tri &tri : tris) {
      sum += solid_angle(p, tri.a, tri.b, tri.c);
    }
    return sum * inv_four_pi;
  }

  double eval_fast(const float3 &p) const
  {
    int stack[128];
    int sp = 0;
    stack[sp++] = 0;
    double w = 0.0;
    const double beta2 = double(beta) * double(beta);
    while (sp > 0) {
      const Node &node = nodes[stack[--sp]];
      const double3 q = to_d(p) - to_d(node.center);
      const double r2 = math::length_squared(q);
      const bool far = r2 > double(node.radius2) * beta2 && r2 > 0.0;
      if (far) {
        w += far_field(q, r2, node);
        continue;
      }
      if (node.left < 0 || sp + 2 > 128) {
        for (int i = 0; i < node.tri_count; i++) {
          const Tri &tri = tris[order[node.tri_start + i]];
          w += solid_angle(p, tri.a, tri.b, tri.c);
        }
        continue;
      }
      stack[sp++] = node.right;
      stack[sp++] = node.left;
    }
    return w * inv_four_pi;
  }

  static double far_field(const double3 &q, const double r2, const Node &node)
  {
    const double r_inv = 1.0 / std::sqrt(r2);
    const double r_inv2 = 1.0 / r2;
    const double r_inv3 = r_inv2 * r_inv;
    const double3 qhat = q * r_inv;
    const double3 N = to_d(node.dipole);
    double omega = -r_inv2 * math::dot(qhat, N);
    const double3 q2(qhat.x * qhat.x, qhat.y * qhat.y, qhat.z * qhat.z);
    const double3 Nij = to_d(node.Nij_diag);
    const double trace = Nij.x + Nij.y + Nij.z;
    const double quad = math::dot(q2, Nij) + qhat.x * qhat.y * double(node.Nxy_Nyx) +
                         qhat.x * qhat.z * double(node.Nzx_Nxz) +
                         qhat.y * qhat.z * double(node.Nyz_Nzy);
    omega += r_inv3 * (trace - 3.0 * quad);
    return omega;
  }

  int build_range(const int start, const int count)
  {
    Node node;
    const float inf = std::numeric_limits<float>::max();
    float3 bb_min(inf, inf, inf);
    float3 bb_max(-inf, -inf, -inf);
    float3 dipole(0.0f);
    float3 mass_center(0.0f);
    float area_sum = 0.0f;
    for (int i = 0; i < count; i++) {
      const Tri &tri = tris[order[start + i]];
      bb_min = math::min(bb_min, math::min(tri.a, math::min(tri.b, tri.c)));
      bb_max = math::max(bb_max, math::max(tri.a, math::max(tri.b, tri.c)));
      dipole += tri.dipole;
      mass_center += tri.centroid * tri.area;
      area_sum += tri.area;
    }
    node.dipole = dipole;
    node.center = area_sum > 1.0e-18f ? mass_center / area_sum : (bb_min + bb_max) * 0.5f;
    const float3 extent = math::max(node.center - bb_min, bb_max - node.center);
    node.radius2 = math::length_squared(extent);
    node.tri_start = start;
    node.tri_count = count;

    const int index = int(nodes.size());
    nodes.append(node);

    if (count <= leaf_size) {
      nodes[index].left = -1;
      nodes[index].right = -1;
      return index;
    }

    const float3 box = bb_max - bb_min;
    int axis = 0;
    if (box.y > box.x) {
      axis = 1;
    }
    if (box.z > box[axis]) {
      axis = 2;
    }
    const int mid = count / 2;
    std::nth_element(order.data() + start,
                     order.data() + start + mid,
                     order.data() + start + count,
                     [&](const int ia, const int ib) {
                       return tris[ia].centroid[axis] < tris[ib].centroid[axis];
                     });
    const int left = build_range(start, mid);
    const int right = build_range(start + mid, count - mid);
    nodes[index].left = left;
    nodes[index].right = right;
    combine_moments(nodes[index], nodes[left]);
    combine_moments(nodes[index], nodes[right]);
    return index;
  }

  static void combine_moments(Node &parent, const Node &child)
  {
    const float3 d = child.center - parent.center;
    parent.Nij_diag += child.Nij_diag +
                       float3(child.dipole.x * d.x, child.dipole.y * d.y, child.dipole.z * d.z);
    parent.Nxy_Nyx += child.Nxy_Nyx + child.dipole.x * d.y + child.dipole.y * d.x;
    parent.Nyz_Nzy += child.Nyz_Nzy + child.dipole.y * d.z + child.dipole.z * d.y;
    parent.Nzx_Nxz += child.Nzx_Nxz + child.dipole.z * d.x + child.dipole.x * d.z;
  }
};

}  // namespace

struct MeshWindingNumber::Impl {
  WindingTree tree;
  bool use_exact = false;
};

MeshWindingNumber::MeshWindingNumber() : impl_(std::make_unique<Impl>()) {}
MeshWindingNumber::~MeshWindingNumber() = default;
MeshWindingNumber::MeshWindingNumber(MeshWindingNumber &&) noexcept = default;
MeshWindingNumber &MeshWindingNumber::operator=(MeshWindingNumber &&) noexcept = default;

bool MeshWindingNumber::build(const Mesh &mesh,
                               const float accuracy,
                               const WindingNumberMethod method,
                               std::string &r_error)
{
  impl_->tree.beta = math::clamp(accuracy, 1.0f, 8.0f);
  impl_->tree.build(mesh);
  if (impl_->tree.tris.is_empty()) {
    r_error = "Mesh has no triangles";
    return false;
  }
  impl_->use_exact = method == WindingNumberMethod::Exact;
  return true;
}

bool MeshWindingNumber::is_valid() const
{
  return impl_ && !impl_->tree.tris.is_empty();
}

float MeshWindingNumber::eval(const float3 &p) const
{
  if (!this->is_valid()) {
    return 0.0f;
  }
  return impl_->tree.eval(p, impl_->use_exact);
}

}  // namespace blender::geometry
