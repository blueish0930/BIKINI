/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_make_it_stand.hh"

#include "GEO_cgal.hh"

#include "BLI_array.hh"
#include "BLI_bounds.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <string>

namespace blender::geometry {
namespace {

constexpr float COM_VOLUME_EPS = 1.0e-12f;
constexpr float GRAVITY_EPS = 1.0e-8f;

static float3 safe_normalize(const float3 v)
{
  const float len = math::length(v);
  if (len < GRAVITY_EPS) {
    return float3(0.0f, 0.0f, -1.0f);
  }
  return v / len;
}

static void gravity_basis(const float3 g, float3 &r_u, float3 &r_v)
{
  const float3 ref = math::abs(g.z) < 0.9f ? float3(0.0f, 0.0f, 1.0f) : float3(0.0f, 1.0f, 0.0f);
  r_u = math::normalize(math::cross(g, ref));
  r_v = math::cross(g, r_u);
}

static float3 project_to_plane(const float3 p, const float3 origin, const float3 g)
{
  return p - g * math::dot(p - origin, g);
}

static float3 perp_to_gravity(const float3 p, const float3 g)
{
  return p - g * math::dot(p, g);
}

static float2 to_plane2(const float3 p, const float3 u, const float3 v)
{
  return float2(math::dot(p, u), math::dot(p, v));
}

static float cross2(const float2 a, const float2 b)
{
  return a.x * b.y - a.y * b.x;
}

static Vector<int> convex_hull_2d(Span<float2> pts)
{
  Vector<int> hull;
  const int n = int(pts.size());
  if (n == 0) {
    return hull;
  }
  Vector<int> order(n);
  for (int i = 0; i < n; i++) {
    order[i] = i;
  }
  std::sort(order.begin(), order.end(), [&](const int a, const int b) {
    if (pts[a].x != pts[b].x) {
      return pts[a].x < pts[b].x;
    }
    return pts[a].y < pts[b].y;
  });

  auto ccw = [&](const int a, const int b, const int c) {
    return cross2(pts[b] - pts[a], pts[c] - pts[a]) > 0.0f;
  };

  Vector<int> lower;
  for (const int i : order) {
    while (lower.size() >= 2 && !ccw(lower[lower.size() - 2], lower.last(), i)) {
      lower.remove_last();
    }
    lower.append(i);
  }
  Vector<int> upper;
  for (int k = n - 1; k >= 0; k--) {
    const int i = order[k];
    while (upper.size() >= 2 && !ccw(upper[upper.size() - 2], upper.last(), i)) {
      upper.remove_last();
    }
    upper.append(i);
  }
  if (!lower.is_empty()) {
    lower.remove_last();
  }
  if (!upper.is_empty()) {
    upper.remove_last();
  }
  hull.extend(lower);
  hull.extend(upper);
  return hull;
}

static Mesh *polygon_from_hull(Span<float3> hull_world, const float3 u, const float3 v)
{
  const int n = int(hull_world.size());
  if (n == 0) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }
  if (n == 1) {
    const float3 p = hull_world[0];
    const float s = 0.005f;
    const float3 pts[3] = {p, p + u * s, p + v * s};
    const int corners[3] = {0, 1, 2};
    return cgal_triangle_mesh_from_buffers(Span(pts, 3), Span(corners, 3));
  }
  if (n == 2) {
    const float3 a = hull_world[0];
    const float3 b = hull_world[1];
    const float3 t = math::normalize(b - a);
    const float3 s = math::normalize(math::cross(t, math::cross(u, v))) * 0.002f;
    const float3 pts[4] = {a + s, b + s, b - s, a - s};
    const int corners[6] = {0, 1, 2, 0, 2, 3};
    return cgal_triangle_mesh_from_buffers(Span(pts, 4), Span(corners, 6));
  }
  Vector<int> corners;
  corners.reserve((n - 2) * 3);
  for (int i = 1; i < n - 1; i++) {
    corners.append(0);
    corners.append(i);
    corners.append(i + 1);
  }
  return cgal_triangle_mesh_from_buffers(hull_world, corners.as_span());
}

static PointCloud *pointcloud_from_positions(Span<float3> positions)
{
  PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, int(positions.size()));
  if (!positions.is_empty()) {
    pc->positions_for_write().copy_from(positions);
  }
  return pc;
}

static float hull_margin(Span<float3> hull,
                         const float3 projected,
                         const float3 gravity,
                         const float contact_epsilon)
{
  const int n = int(hull.size());
  if (n == 0) {
    return -FLT_MAX;
  }
  if (n == 1) {
    return contact_epsilon - math::distance(projected, hull[0]);
  }
  float3 u, v;
  gravity_basis(gravity, u, v);
  const float2 p = to_plane2(projected, u, v);
  if (n == 2) {
    const float2 a = to_plane2(hull[0], u, v);
    const float2 b = to_plane2(hull[1], u, v);
    const float2 ab = b - a;
    const float len_sq = math::length_squared(ab);
    if (len_sq < 1.0e-20f) {
      return contact_epsilon - math::distance(projected, hull[0]);
    }
    const float t = math::clamp(math::dot(p - a, ab) / len_sq, 0.0f, 1.0f);
    const float2 closest = a + ab * t;
    return contact_epsilon - math::distance(p, closest);
  }
  float margin = FLT_MAX;
  for (int i = 0; i < n; i++) {
    const float2 a = to_plane2(hull[i], u, v);
    const float2 b = to_plane2(hull[(i + 1) % n], u, v);
    const float2 e = b - a;
    const float len = math::length(e);
    if (len < 1.0e-12f) {
      continue;
    }
    const float signed_dist = cross2(e, p - a) / len;
    margin = math::min(margin, signed_dist);
  }
  return margin;
}

static bool query_inside_distance(const Mesh &mesh,
                                  Span<float3> query,
                                  MutableSpan<bool> r_inside,
                                  MutableSpan<float> r_distance,
                                  std::string &r_error)
{
  if (query.is_empty()) {
    return true;
  }
  PointCloud *side = cgal_mesh_side_of(mesh, query, nullptr, nullptr, r_error);
  if (!side) {
    return false;
  }
  const VArraySpan<bool> inside = *side->attributes().lookup<bool>("Inside",
                                                                   bke::AttrDomain::Point);
  for (const int i : query.index_range()) {
    r_inside[i] = inside[i];
  }
  BKE_id_free(nullptr, side);

  r_error.clear();
  PointCloud *dist_pc = cgal_mesh_distance_to(mesh, query, r_error);
  if (!dist_pc) {
    return false;
  }
  const VArraySpan<float> dist = *dist_pc->attributes().lookup<float>("Distance",
                                                                      bke::AttrDomain::Point);
  for (const int i : query.index_range()) {
    r_distance[i] = dist[i];
  }
  BKE_id_free(nullptr, dist_pc);
  return true;
}

static bool deform_com_to_landing(Mesh *&mesh,
                                  const float3 landing,
                                  const float3 gravity,
                                  const float3 com,
                                  Span<bool> pin,
                                  const float strength)
{
  /* Paper: Laplacian editing driven by handles, so the CoM projection moves onto
   * the support. Not base-scale, not umbrella smoothing. */
  if (strength <= 0.0f || !mesh || mesh->verts_num == 0) {
    return false;
  }
  const float3 error = perp_to_gravity(com - landing, gravity);
  const float error_len = math::length(error);
  if (error_len < 1.0e-8f) {
    return false;
  }
  const float3 error_n = error / error_len;
  const Span<float3> positions = mesh->vert_positions();
  const int n = mesh->verts_num;

  float max_side = 0.0f;
  for (const int i : positions.index_range()) {
    if (i < pin.size() && pin[i]) {
      continue;
    }
    max_side = math::max(max_side, math::dot(positions[i] - landing, error_n));
  }
  if (max_side < 1.0e-8f) {
    return false;
  }
  const float heavy_cut = 0.45f * max_side;

  Array<uint8_t> roi(n, 1);
  Array<uint8_t> control(n, 0);
  Array<float3> targets(n);
  targets.as_mutable_span().copy_from(positions);

  int n_pin = 0;
  int n_heavy = 0;
  for (const int i : positions.index_range()) {
    const bool is_pin = i < pin.size() && pin[i];
    const float side = math::dot(positions[i] - landing, error_n);
    if (is_pin) {
      control[i] = 1;
      n_pin++;
    }
    else if (side >= heavy_cut) {
      control[i] = 1;
      n_heavy++;
    }
  }
  if (n_heavy == 0) {
    return false;
  }

  /* Translating mass fraction f by d moves CoM by f*d. Solve for d that cancels `error`. */
  const float f = math::clamp(float(n_heavy) / float(n), 0.2f, 0.85f);
  float3 delta = error_n * (-error_len * math::clamp(strength, 0.0f, 1.0f) / f);
  if (const std::optional<Bounds<float3>> b = mesh->bounds_min_max()) {
    const float cap = 0.35f * math::length(b->max - b->min);
    const float dlen = math::length(delta);
    if (dlen > cap) {
      delta *= cap / dlen;
    }
  }

  for (const int i : positions.index_range()) {
    if (control[i] && !(i < pin.size() && pin[i])) {
      targets[i] = positions[i] + delta;
    }
  }
  if (n_pin == 0) {
    /* Keep at least the lowest vertex from sliding off the landing. */
    int best = 0;
    float best_d = -FLT_MAX;
    for (const int i : positions.index_range()) {
      const float d = math::dot(positions[i], gravity);
      if (d > best_d) {
        best_d = d;
        best = i;
      }
    }
    control[best] = 1;
    targets[best] = positions[best];
  }

  std::string error_msg;
  Mesh *out = cgal_mesh_laplace_deform(
      *mesh, roi.as_span(), control.as_span(), targets.as_span(), 1.0e6f, error_msg);
  if (out && out->faces_num > 0 && out->verts_num == n) {
    BKE_id_free(nullptr, mesh);
    mesh = out;
    return true;
  }
  if (out) {
    BKE_id_free(nullptr, out);
  }

  /* Fallback: translate the heavy side only. Locally rigid, not a smooth. */
  MutableSpan<float3> pos = mesh->vert_positions_for_write();
  for (const int i : pos.index_range()) {
    if (i < pin.size() && pin[i]) {
      continue;
    }
    const float side = math::dot(pos[i] - landing, error_n);
    const float w = math::clamp(side / max_side, 0.0f, 1.0f);
    pos[i] += delta * (w * w);
  }
  mesh->tag_positions_changed();
  return true;
}

static Mesh *make_detail_cavity(const Mesh &outer,
                                const float shell,
                                const float3 plane_point,
                                const float3 plane_normal,
                                const bool clip_heavy_side,
                                std::string &r_error)
{
  Mesh *inner = BKE_mesh_copy_for_eval(outer);
  const Span<float3> normals = inner->vert_normals();
  Array<float3> ncopy(normals.size());
  ncopy.as_mutable_span().copy_from(normals);
  MutableSpan<float3> positions = inner->vert_positions_for_write();
  const float inset = math::max(shell, 1.0e-5f);
  for (const int i : positions.index_range()) {
    positions[i] -= ncopy[i] * inset;
  }
  inner->tag_positions_changed();

  if (!clip_heavy_side || math::length_squared(plane_normal) < 1.0e-12f) {
    return inner;
  }
  Mesh *clipped = cgal_mesh_clip_plane(*inner, plane_point, plane_normal, r_error);
  if (clipped && clipped->faces_num > 0) {
    BKE_id_free(nullptr, inner);
    r_error.clear();
    return clipped;
  }
  if (clipped) {
    BKE_id_free(nullptr, clipped);
  }
  r_error.clear();
  return inner;
}

static Mesh *copy_mesh_optional(const Mesh *mesh)
{
  if (!mesh || mesh->faces_num == 0) {
    return nullptr;
  }
  return BKE_mesh_copy_for_eval(*mesh);
}

static Mesh *union_voids(Mesh *a, Mesh *b, std::string &r_warning)
{
  if (!a) {
    return b;
  }
  if (!b) {
    return a;
  }
  std::string error;
  Mesh *united = cgal_mesh_boolean(*a, *b, CgalBooleanOperation::Union, error);
  if (united && united->faces_num > 0) {
    BKE_id_free(nullptr, a);
    BKE_id_free(nullptr, b);
    return united;
  }
  if (united) {
    BKE_id_free(nullptr, united);
  }
  if (r_warning.empty() && !error.empty()) {
    r_warning = error;
  }
  BKE_id_free(nullptr, b);
  return a;
}

}  // namespace

VolumeMassResult mesh_volume_center_of_mass(const Mesh &mesh, float density, std::string &r_error)
{
  VolumeMassResult result;
  if (mesh.faces_num == 0 || mesh.verts_num == 0) {
    r_error = "Empty mesh";
    return result;
  }
  const Span<float3> positions = mesh.vert_positions();
  const Span<int> corner_verts = mesh.corner_verts();
  const Span<int3> tris = mesh.corner_tris();
  if (tris.is_empty()) {
    r_error = "Mesh has no triangles";
    return result;
  }

  double volume = 0.0;
  double3 moment(0.0, 0.0, 0.0);
  for (const int3 &tri : tris) {
    const float3 p0 = positions[corner_verts[tri[0]]];
    const float3 p1 = positions[corner_verts[tri[1]]];
    const float3 p2 = positions[corner_verts[tri[2]]];
    const double vi = double(math::dot(p0, math::cross(p1, p2))) / 6.0;
    volume += vi;
    moment += double3(double(p0.x + p1.x + p2.x),
                      double(p0.y + p1.y + p2.y),
                      double(p0.z + p1.z + p2.z)) *
              (vi / 4.0);
  }

  if (std::abs(volume) < COM_VOLUME_EPS) {
    float3 acc(0.0f);
    for (const float3 &p : positions) {
      acc += p;
    }
    result.center = acc / float(positions.size());
    result.volume = 0.0f;
    result.mass = 0.0f;
    result.ok = true;
    r_error = "Near-zero volume; using vertex centroid";
    return result;
  }

  result.volume = float(volume);
  result.mass = std::abs(float(volume)) * math::max(density, 0.0f);
  result.center = float3(float(moment.x / volume),
                         float(moment.y / volume),
                         float(moment.z / volume));
  result.ok = true;
  return result;
}

SupportPolygonResult mesh_support_polygon(const Mesh &mesh,
                                          float3 gravity,
                                          float contact_epsilon,
                                          Span<bool> contact_selection,
                                          Span<float3> extra_points,
                                          std::string &r_error)
{
  SupportPolygonResult result;
  gravity = safe_normalize(gravity);
  result.gravity = gravity;
  contact_epsilon = math::max(contact_epsilon, 0.0f);

  Vector<float3> contacts;
  const Span<float3> positions = mesh.vert_positions();
  bool any_selected = false;
  if (contact_selection.size() == positions.size()) {
    for (const int i : positions.index_range()) {
      if (contact_selection[i]) {
        contacts.append(positions[i]);
        any_selected = true;
      }
    }
  }
  if (!any_selected) {
    contacts.extend(extra_points);
  }
  if (contacts.is_empty() && !positions.is_empty()) {
    float max_dot = -FLT_MAX;
    for (const float3 &p : positions) {
      max_dot = math::max(max_dot, math::dot(p, gravity));
    }
    const float thresh = max_dot - contact_epsilon;
    for (const float3 &p : positions) {
      if (math::dot(p, gravity) >= thresh) {
        contacts.append(p);
      }
    }
  }
  if (contacts.is_empty()) {
    r_error = "No contact points";
    return result;
  }

  float max_dot = -FLT_MAX;
  for (const float3 &p : contacts) {
    max_dot = math::max(max_dot, math::dot(p, gravity));
  }
  const float3 plane_origin = gravity * max_dot;

  float3 u, v;
  gravity_basis(gravity, u, v);
  Array<float2> pts2(contacts.size());
  for (const int i : contacts.index_range()) {
    const float3 projected = project_to_plane(contacts[i], plane_origin, gravity);
    contacts[i] = projected;
    pts2[i] = to_plane2(projected, u, v);
  }

  const Vector<int> hull_idx = convex_hull_2d(pts2);
  Vector<float3> hull_pts;
  hull_pts.reserve(hull_idx.size());
  float3 target(0.0f);
  for (const int i : hull_idx) {
    hull_pts.append(contacts[i]);
    target += contacts[i];
  }
  if (!hull_pts.is_empty()) {
    target /= float(hull_pts.size());
  }

  result.hull = std::move(hull_pts);
  result.target = target;
  result.polygon = polygon_from_hull(result.hull, u, v);
  result.contact_points = pointcloud_from_positions(contacts);
  return result;
}

BalanceStatus mesh_balance_status(const float3 center,
                                  const float3 gravity,
                                  Span<float3> hull,
                                  const float contact_epsilon)
{
  BalanceStatus status;
  const float3 g = safe_normalize(gravity);
  status.center = center;
  if (hull.is_empty()) {
    status.projected = perp_to_gravity(center, g);
    status.margin = -FLT_MAX;
    status.is_stable = false;
    return status;
  }
  float3 target(0.0f);
  for (const float3 &p : hull) {
    target += p;
  }
  target /= float(hull.size());
  status.target = target;
  status.projected = project_to_plane(center, hull[0], g);
  status.margin = hull_margin(hull, status.projected, g, contact_epsilon);
  status.is_stable = status.margin >= 0.0f;
  return status;
}

MakeItStandResult mesh_make_it_stand(const Mesh &mesh,
                                     const MakeItStandParams &params,
                                     std::string &r_error)
{
  MakeItStandResult result;
  if (mesh.faces_num == 0 || mesh.verts_num == 0) {
    r_error = "Empty mesh";
    return result;
  }

  const float3 gravity = safe_normalize(params.gravity);
  const int resolution = math::clamp(params.resolution, 8, 64);
  const float max_empty = math::clamp(params.max_empty_fraction, 0.05f, 0.95f);
  const int iterations = math::max(params.iterations, 1);
  const float density = math::max(params.density, 0.0f);

  SupportPolygonResult support = mesh_support_polygon(mesh,
                                                      gravity,
                                                      params.contact_epsilon,
                                                      params.contact_selection,
                                                      params.extra_contact_points,
                                                      r_error);
  if (support.hull.is_empty()) {
    if (support.polygon) {
      BKE_id_free(nullptr, support.polygon);
    }
    if (support.contact_points) {
      BKE_id_free(nullptr, support.contact_points);
    }
    result.mesh = BKE_mesh_copy_for_eval(mesh);
    result.inner_void = copy_mesh_optional(params.existing_void);
    if (!result.inner_void) {
      result.inner_void = BKE_mesh_new_nomain(0, 0, 0, 0);
    }
    return result;
  }
  r_error.clear();

  result.support = support.polygon;
  result.contact_points = support.contact_points;
  support.polygon = nullptr;
  support.contact_points = nullptr;

  const float3 target = params.use_target ?
                            project_to_plane(params.target, support.hull[0], gravity) :
                            support.target;
  result.target = target;

  Array<bool> pin(mesh.verts_num, false);
  if (params.contact_selection.size() == mesh.verts_num) {
    pin.as_mutable_span().copy_from(params.contact_selection);
  }
  else {
    const float max_dot = math::dot(support.hull[0], gravity);
    const Span<float3> verts = mesh.vert_positions();
    for (const int i : verts.index_range()) {
      pin[i] = math::dot(verts[i], gravity) >= max_dot - params.contact_epsilon;
    }
  }

  Mesh *outer = BKE_mesh_copy_for_eval(mesh);
  Mesh *acc_void = copy_mesh_optional(params.existing_void);

  auto finish_status = [&](const float3 com, const float volume) {
    const BalanceStatus st = mesh_balance_status(
        com, gravity, support.hull, params.contact_epsilon);
    result.center = st.center;
    result.projected = st.projected;
    result.margin = st.margin;
    result.is_stable = st.is_stable;
    result.volume = volume;
  };

  VolumeMassResult mass = mesh_volume_center_of_mass(*outer, density, r_error);
  r_error.clear();
  finish_status(mass.center, mass.volume);

  int total_carved = 0;
  float3 last_plane_n(0.0f);
  bool have_plane = false;

  for (int iter = 0; iter < iterations; iter++) {
    const std::optional<Bounds<float3>> bounds_opt = outer->bounds_min_max();
    if (!bounds_opt) {
      r_error = "Could not compute bounds";
      break;
    }
    Bounds<float3> bounds = *bounds_opt;
    const float3 diag = math::max(bounds.max - bounds.min, float3(1.0e-5f));
    const float pad = math::reduce_max(diag) / float(resolution);
    bounds.min -= float3(pad);
    bounds.max += float3(pad);
    const float3 size = math::max(bounds.max - bounds.min, float3(1.0e-5f));
    const int nx = resolution;
    const int ny = math::max(
        8, int(std::round(float(resolution) * size.y / math::reduce_max(size))));
    const int nz = math::max(
        8, int(std::round(float(resolution) * size.z / math::reduce_max(size))));
    const float3 voxel = size / float3(float(nx), float(ny), float(nz));
    const float voxel_len = math::reduce_max(voxel);
    const float shell = math::max(params.shell_thickness, voxel_len * 1.25f);
    const float3 origin = bounds.min;
    const int tot = nx * ny * nz;

    Array<float3> samples(tot);
    int index = 0;
    for (int k = 0; k < nz; k++) {
      for (int j = 0; j < ny; j++) {
        for (int i = 0; i < nx; i++) {
          samples[index++] = origin +
                             voxel * float3(float(i) + 0.5f, float(j) + 0.5f, float(k) + 0.5f);
        }
      }
    }

    Array<bool> inside(tot, false);
    Array<float> distance(tot, 0.0f);
    std::string query_error;
    if (!query_inside_distance(*outer, samples, inside, distance, query_error)) {
      result.warning = query_error.empty() ?
                           "Inside/distance query failed (needs a closed mesh)" :
                           query_error;
      break;
    }

    Array<bool> in_prev_void(tot, false);
    if (acc_void && acc_void->faces_num > 0) {
      Array<float> void_dist(tot, 0.0f);
      std::string void_error;
      query_inside_distance(*acc_void, samples, in_prev_void, void_dist, void_error);
    }

    Array<uint8_t> remaining(tot, 0);
    int solid_count = 0;
    float3 solid_sum(0.0f);
    Vector<int> carvable;
    carvable.reserve(tot / 4);
    for (int i = 0; i < tot; i++) {
      const bool material = inside[i] && !in_prev_void[i];
      if (!material) {
        continue;
      }
      remaining[i] = 1;
      solid_count++;
      solid_sum += samples[i];
      if (distance[i] >= shell) {
        carvable.append(i);
      }
    }
    if (solid_count == 0) {
      result.warning =
          "No interior voxels. Keep Boolean off when chaining, and feed Inner Void → Inner Void.";
      break;
    }

    float3 com = solid_sum / float(solid_count);
    finish_status(com, float(solid_count) * voxel.x * voxel.y * voxel.z);
    if (result.is_stable && params.deform_strength <= 0.0f) {
      if (iter == 0 && total_carved == 0) {
        result.warning = "Already stable; no carving needed";
      }
      break;
    }

    const float3 delta_h = perp_to_gravity(com - target, gravity);
    const float delta_len = math::length(delta_h);
    if (delta_len < 1.0e-8f) {
      break;
    }
    const float3 plane_n = delta_h / delta_len;
    last_plane_n = plane_n;
    have_plane = true;

    struct Candidate {
      int index;
      float score;
    };
    Vector<Candidate> candidates;
    candidates.reserve(carvable.size());
    for (const int vi : carvable) {
      if (!remaining[vi]) {
        continue;
      }
      const float di = math::dot(samples[vi] - target, plane_n);
      if (di <= 0.0f) {
        continue;
      }
      const float height = -math::dot(samples[vi], gravity);
      candidates.append({vi, di + params.lower_mass * height});
    }

    int best_k = 0;
    if (!candidates.is_empty()) {
      std::sort(candidates.begin(),
                candidates.end(),
                [](const Candidate &a, const Candidate &b) { return a.score > b.score; });
      const int kmax = math::min(int(candidates.size()),
                                 math::max(1, int(float(carvable.size()) * max_empty)));
      float3 prefix(0.0f);
      float best_energy = math::length_squared(delta_h);
      const int n0 = solid_count;
      const float3 s0 = solid_sum;
      for (int k = 1; k <= kmax; k++) {
        prefix += samples[candidates[k - 1].index];
        const int n1 = n0 - k;
        if (n1 <= 0) {
          break;
        }
        const float3 ck = (s0 - prefix) / float(n1);
        const float energy = math::length_squared(perp_to_gravity(ck - target, gravity));
        if (energy + 1.0e-12f < best_energy) {
          best_energy = energy;
          best_k = k;
        }
      }
      for (int k = 0; k < best_k; k++) {
        const int vi = candidates[k].index;
        remaining[vi] = 0;
        solid_sum -= samples[vi];
        solid_count--;
      }
      total_carved += best_k;
      if (solid_count > 0) {
        com = solid_sum / float(solid_count);
      }
      finish_status(com, float(solid_count) * voxel.x * voxel.y * voxel.z);
    }

    if (!result.is_stable && params.deform_strength > 0.0f && iter + 1 < iterations) {
      deform_com_to_landing(outer, target, gravity, com, pin, params.deform_strength);
      continue;
    }
    break;
  }

  result.carved_voxels = total_carved;

  Mesh *out_void = nullptr;
  if (total_carved > 0) {
    float shell = math::max(params.shell_thickness, 1.0e-4f);
    if (const std::optional<Bounds<float3>> b = outer->bounds_min_max()) {
      shell = math::max(shell, math::reduce_max(b->max - b->min) * 0.02f);
    }
    std::string cavity_error;
    out_void = make_detail_cavity(
        *outer, shell, target, last_plane_n, have_plane, cavity_error);
    if (!cavity_error.empty() && result.warning.empty()) {
      result.warning = cavity_error;
    }
  }
  if (!out_void && acc_void) {
    out_void = acc_void;
    acc_void = nullptr;
  }
  else if (acc_void) {
    if (out_void) {
      out_void = union_voids(acc_void, out_void, result.warning);
      acc_void = nullptr;
    }
    else {
      BKE_id_free(nullptr, acc_void);
      acc_void = nullptr;
    }
  }

  Mesh *out_mesh = outer;
  outer = nullptr;
  if (params.boolean_cavity && out_void && out_void->faces_num > 0) {
    std::string bool_error;
    Mesh *hollow = cgal_mesh_boolean(
        *out_mesh, *out_void, CgalBooleanOperation::Difference, bool_error);
    if (hollow && hollow->faces_num > 0) {
      BKE_id_free(nullptr, out_mesh);
      out_mesh = hollow;
    }
    else {
      if (hollow) {
        BKE_id_free(nullptr, hollow);
      }
      if (result.warning.empty()) {
        result.warning = bool_error.empty() ?
                             "Boolean skipped; Mesh is the outer surface, cavity is Inner Void" :
                             bool_error;
      }
    }
  }

  mass = mesh_volume_center_of_mass(*out_mesh, density, r_error);
  r_error.clear();
  finish_status(mass.ok ? mass.center : result.center, mass.volume);

  result.mesh = out_mesh;
  result.inner_void = out_void ? out_void : BKE_mesh_new_nomain(0, 0, 0, 0);
  return result;
}

}  // namespace blender::geometry
