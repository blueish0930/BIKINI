/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Geometry Nodes: Jolt Solver — black-box rigid body step (Jolt Physics v5.6).
 *
 * Bodies: Instances (preferred), Points, or Mesh.
 * Static bodies are colliders. No separate Colliders socket.
 * Constraints: Point Cloud from Jolt Set Constraint.
 */

#include "BKE_attribute.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "BLI_array.hh"
#include "BLI_bounds.hh"
#include "BLI_map.hh"
#include "BLI_set.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion.hh"
#include "BLI_math_rotation.hh"
#include "BLI_math_vector.hh"
#include "BLI_vector.hh"

#include "DNA_ID.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_pointcloud_types.h"

#include "DEG_depsgraph_query.hh"

#include "box_engine_shared.hh"
#include "deformable_solver_shared.hh"
#include "node_geometry_util.hh"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fmt/format.h>
#include <mutex>
#include <string>

#if defined(WITH_JOLT)
#  include "jolt_bridge.h"
#endif

namespace blender::nodes::node_geo_jolt_solver_cc {

enum class ShapeType {
  Sphere = 0,
  Box = 1,
  Capsule = 2,
  ConvexHull = 3,
  BoundingBox = 4,
  TriangleMesh = 5,
  Compound = 6,
  TaperedCapsule = 7,
  Cylinder = 8,
  TaperedCylinder = 9,
  Plane = 10,
  HeightField = 11,
  Empty = 12,
};

namespace attr {
constexpr StringRefNull body_type = "body_type";
constexpr StringRefNull shape_type = "shape_type";
constexpr StringRefNull shape_size = "shape_size";
constexpr StringRefNull density = "density";
constexpr StringRefNull friction = "friction";
constexpr StringRefNull restitution = "restitution";
constexpr StringRefNull velocity = "velocity";
constexpr StringRefNull angular_velocity = "angular_velocity";
constexpr StringRefNull acceleration = "acceleration";
constexpr StringRefNull angular_acceleration = "angular_acceleration";
constexpr StringRefNull mass = "mass";
constexpr StringRefNull inertia = "inertia";
constexpr StringRefNull prev_position = "prev_position";
constexpr StringRefNull prev_rotation = "prev_rotation";
constexpr StringRefNull has_prev = "has_prev";
constexpr StringRefNull radius = "radius";
constexpr StringRefNull linear_damping = "linear_damping";
constexpr StringRefNull angular_damping = "angular_damping";
constexpr StringRefNull sensor = "sensor";
constexpr StringRefNull allowed_dofs = "allowed_dofs";
constexpr StringRefNull convex_radius = "convex_radius";
constexpr StringRefNull radius_top = "radius_top";
constexpr StringRefNull ccd = "ccd";
constexpr StringRefNull body_a = "body_a";
constexpr StringRefNull body_b = "body_b";
constexpr StringRefNull body_name = "body_name";
constexpr StringRefNull joint_type = "joint_type";
constexpr StringRefNull axis = "axis";
constexpr StringRefNull axis2 = "axis2";
constexpr StringRefNull anchor2 = "anchor2";
constexpr StringRefNull min_limit = "min_limit";
constexpr StringRefNull max_limit = "max_limit";
constexpr StringRefNull min_limit2 = "min_limit2";
constexpr StringRefNull max_limit2 = "max_limit2";
constexpr StringRefNull ratio = "ratio";
constexpr StringRefNull frequency = "frequency";
constexpr StringRefNull damping = "damping";
constexpr StringRefNull motor_state = "motor_state";
constexpr StringRefNull motor_target = "motor_target";
constexpr StringRefNull motor_force = "motor_force";
constexpr StringRefNull path_id = "path_id";
constexpr StringRefNull wheel_radius = "wheel_radius";
constexpr StringRefNull wheel_width = "wheel_width";
constexpr StringRefNull suspension_min = "suspension_min";
constexpr StringRefNull suspension_max = "suspension_max";
constexpr StringRefNull max_steer = "max_steer";
constexpr StringRefNull num_wheels = "num_wheels";
}  // namespace attr

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Geometry"_ustr)
      .description(
          "Bodies tagged by Jolt Set Rigid Body. Static = colliders, Dynamic = simulated, "
          "Kinematic = graph-driven");
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Bodies with updated transforms plus velocity / acceleration / mass / inertia");

  b.add_input<decl::Geometry>("Constraints"_ustr)
      .description("Point Cloud from Jolt Set Constraint (optional Path mesh)");

  b.add_input<decl::Vector>("Gravity"_ustr)
      .default_value(float3(0.0f, 0.0f, -9.81f))
      .subtype(PROP_ACCELERATION)
      .description("World gravity applied each step to dynamic bodies (Blender Z-up)");

  b.add_input<decl::Float>("Delta Time"_ustr)
      .default_value(1.0f / 24.0f)
      .min(0.0f)
      .subtype(PROP_TIME_ABSOLUTE)
      .description("Use Scene Time delta inside a Simulation Zone");

  b.add_input<decl::Int>("Substeps"_ustr)
      .default_value(1)
      .min(1)
      .max(16)
      .description("Jolt collision steps of this frame (1 per ~1/60s is typical)");
  b.add_input<decl::Int>("Velocity Steps"_ustr)
      .default_value(10)
      .min(2)
      .max(30)
      .description("Constraint velocity iterations (Jolt default 10)");
  b.add_input<decl::Int>("Position Steps"_ustr)
      .default_value(2)
      .min(1)
      .max(10)
      .description("Constraint position iterations (Jolt default 2)");
  b.add_input<decl::Bool>("CCD"_ustr)
      .default_value(false)
      .description("LinearCast motion quality on dynamic bodies (slower, fewer tunnels)");
}

struct BodyRead {
  float3 position = float3(0.0f);
  math::Quaternion rotation = math::Quaternion::identity();
  float3 velocity = float3(0.0f);
  float3 angular_velocity = float3(0.0f);
  float mass = 0.0f;
  float3 inertia = float3(0.0f);
  float3 prev_position = float3(0.0f);
  math::Quaternion prev_rotation = math::Quaternion::identity();
  bool has_prev = false;
  float3 shape_size = float3(0.25f);
  float density = 1.0f;
  float friction = 0.4f;
  float restitution = 0.05f;
  float linear_damping = 0.05f;
  float angular_damping = 0.05f;
  float convex_radius = 0.05f;
  float radius_top = 0.0f;
  int body_type = 2;
  int shape_type = int(ShapeType::BoundingBox);
  int allowed_dofs = 0;
  bool sensor = false;
  bool ccd = false;
  Vector<float3> local_mesh_verts;
  Vector<int> mesh_tri_indices;
  Vector<Vector<float3>> island_verts;
  Vector<int> slave_instances;
  Vector<float4x4> slave_local_tf;
  int source_instance = -1;
  std::string name;
  bool write_back = true;
  bool graph_driven = false;
};

struct JointRead {
  int body_a = 0;
  int body_b = -1;
  std::string name_a;
  std::string name_b;
  int joint_type = 3;
  float3 anchor = float3(0.0f);
  float3 axis = float3(0.0f, 0.0f, 1.0f);
  float3 axis2 = float3(1.0f, 0.0f, 0.0f);
  float3 anchor2 = float3(0.0f);
  float min_limit = 0.0f;
  float max_limit = 0.0f;
  float min_limit2 = 0.0f;
  float max_limit2 = 0.0f;
  float ratio = 1.0f;
  float frequency = 0.0f;
  float damping = 0.0f;
  int motor_state = 0;
  float motor_target = 0.0f;
  float motor_force = 0.0f;
  Vector<float3> path;
  float wheel_radius = 0.3f;
  float wheel_width = 0.1f;
  float suspension_min = 0.2f;
  float suspension_max = 0.4f;
  float max_steer = 0.0f;
  int num_wheels = 4;
  int path_id = -1;
};

static VArray<float> get_float(const bke::AttributeAccessor &attributes,
                               const StringRef name,
                               const bke::AttrDomain domain,
                               const float fallback)
{
  return *attributes.lookup_or_default<float>(name, domain, fallback);
}
static VArray<float3> get_float3(const bke::AttributeAccessor &attributes,
                                 const StringRef name,
                                 const bke::AttrDomain domain,
                                 const float3 fallback)
{
  return *attributes.lookup_or_default<float3>(name, domain, fallback);
}
static VArray<int> get_int(const bke::AttributeAccessor &attributes,
                           const StringRef name,
                           const bke::AttrDomain domain,
                           const int fallback)
{
  return *attributes.lookup_or_default<int>(name, domain, fallback);
}
static VArray<bool> get_bool(const bke::AttributeAccessor &attributes,
                             const StringRef name,
                             const bke::AttrDomain domain,
                             const bool fallback)
{
  return *attributes.lookup_or_default<bool>(name, domain, fallback);
}
static VArray<MStringProperty> get_mstring(const bke::AttributeAccessor &attributes,
                                           const StringRef name,
                                           const bke::AttrDomain domain)
{
  static const MStringProperty empty{};
  return *attributes.lookup_or_default<MStringProperty>(name, domain, empty);
}

static void fill_body_common(BodyRead &b,
                             const int i,
                             const VArray<int> &body_types,
                             const VArray<int> &shape_types,
                             const VArray<float3> &shape_sizes,
                             const VArray<float> &radii,
                             const VArray<float> &densities,
                             const VArray<float> &frictions,
                             const VArray<float> &restitutions,
                             const VArray<float3> &velocities,
                             const VArray<float> &lin_damp,
                             const VArray<float> &ang_damp,
                             const VArray<float> &convex_r,
                             const VArray<float> &radius_top,
                             const VArray<int> &dofs,
                             const VArray<bool> &sensors,
                             const VArray<bool> &ccds)
{
  b.body_type = body_types[i];
  b.shape_type = shape_types[i];
  b.shape_size = shape_sizes[i];
  if (math::length_squared(b.shape_size) < 1e-12f) {
    const float r = math::max(radii[i], 0.25f);
    b.shape_size = float3(r, r, r);
  }
  b.shape_size = math::max(b.shape_size, float3(1e-5f));
  const float dens = densities[i] > 0.0f ? densities[i] : 1.0f;
  b.density = (b.body_type == 2) ? math::max(dens, 1e-3f) : dens;
  b.friction = frictions[i];
  b.restitution = restitutions[i];
  b.velocity = velocities[i];
  b.linear_damping = lin_damp[i];
  b.angular_damping = ang_damp[i];
  b.convex_radius = convex_r[i];
  b.radius_top = radius_top[i];
  b.allowed_dofs = dofs[i];
  b.sensor = sensors[i];
  b.ccd = ccds[i];
  b.graph_driven = (b.body_type == 1);
}

static void consume_collision_mesh(const Mesh &mesh, BodyRead &b)
{
  if (mesh.verts_num == 0) {
    return;
  }
  const std::optional<Bounds<float3>> bounds = mesh.bounds_min_max();
  if (bounds) {
    const float3 half = math::max(bounds->size() * 0.5f, float3(1e-5f));
    if (ELEM(b.shape_type,
             int(ShapeType::BoundingBox),
             int(ShapeType::ConvexHull),
             int(ShapeType::TriangleMesh),
             int(ShapeType::Compound),
             int(ShapeType::HeightField)) ||
        math::length_squared(b.shape_size) < 1e-12f)
    {
      b.shape_size = half;
    }
  }
  if (b.shape_type == int(ShapeType::ConvexHull) ||
      (b.shape_type == int(ShapeType::TriangleMesh) && b.body_type == 2))
  {
    b.local_mesh_verts.extend(mesh.vert_positions());
    if (b.shape_type == int(ShapeType::TriangleMesh)) {
      b.shape_type = int(ShapeType::ConvexHull);
    }
  }
  else if (b.shape_type == int(ShapeType::TriangleMesh)) {
    box_engine::fill_triangle_mesh_from_mesh(mesh, b.local_mesh_verts, b.mesh_tri_indices);
  }
  else if (b.shape_type == int(ShapeType::Compound)) {
    box_engine::fill_islands_from_mesh(mesh, b.island_verts);
  }
}

static void apply_mesh_shape_from_reference(const bke::InstanceReference &ref, BodyRead &b)
{
  const GeometrySet geo = box_engine::collision_geometry_from_reference(ref);
  if (const Mesh *mesh = geo.get_mesh()) {
    consume_collision_mesh(*mesh, b);
  }
}

static void read_bodies_instances(const GeometrySet &geometry_set, Vector<BodyRead> &r_bodies)
{
  const bke::Instances &instances = *geometry_set.get_instances();
  const int n = instances.instances_num();
  const Span<float4x4> transforms = instances.transforms();
  const bke::AttributeAccessor attributes = instances.attributes();
  const bke::AttrDomain domain = bke::AttrDomain::Instance;

  const VArray<int> body_types = get_int(attributes, attr::body_type, domain, 2);
  const VArray<int> shape_types = get_int(
      attributes, attr::shape_type, domain, int(ShapeType::BoundingBox));
  const VArray<float3> shape_sizes = get_float3(attributes, attr::shape_size, domain, float3(0.0f));
  const VArray<float> radii = get_float(attributes, attr::radius, domain, 0.25f);
  const VArray<float> densities = get_float(attributes, attr::density, domain, 1.0f);
  const VArray<float> frictions = get_float(attributes, attr::friction, domain, 0.5f);
  const VArray<float> restitutions = get_float(attributes, attr::restitution, domain, 0.0f);
  const VArray<float3> velocities = get_float3(attributes, attr::velocity, domain, float3(0.0f));
  const VArray<float3> ang_vel = get_float3(
      attributes, attr::angular_velocity, domain, float3(0.0f));
  const VArray<float> masses = get_float(attributes, attr::mass, domain, 0.0f);
  const VArray<float3> inertias = get_float3(attributes, attr::inertia, domain, float3(0.0f));
  const VArray<float3> prev_positions = get_float3(
      attributes, attr::prev_position, domain, float3(0.0f));
  const VArray<math::Quaternion> prev_rots = *attributes.lookup_or_default<math::Quaternion>(
      attr::prev_rotation, domain, math::Quaternion::identity());
  const VArray<bool> has_prev_flags = get_bool(attributes, attr::has_prev, domain, false);
  const VArray<MStringProperty> body_names = get_mstring(attributes, attr::body_name, domain);
  const VArray<float> lin_damp = get_float(attributes, attr::linear_damping, domain, 0.1f);
  const VArray<float> ang_damp = get_float(attributes, attr::angular_damping, domain, 0.25f);
  const VArray<float> convex_r = get_float(attributes, attr::convex_radius, domain, 0.05f);
  const VArray<float> radius_top = get_float(attributes, attr::radius_top, domain, 0.0f);
  const VArray<int> dofs = get_int(attributes, attr::allowed_dofs, domain, 0);
  const VArray<bool> sensors = get_bool(attributes, attr::sensor, domain, false);
  const VArray<bool> ccds = get_bool(attributes, attr::ccd, domain, false);
  const VArray<bool> is_hull_inst = *attributes.lookup_or_default<bool>(
      box_engine::attr_compound_hull, domain, false);

  const Span<int> handles = instances.reference_handles();
  const Span<bke::InstanceReference> refs = instances.references();

  Vector<int> hull_indices;
  Vector<int> compound_indices;
  for (const int i : IndexRange(n)) {
    if (is_hull_inst[i]) {
      hull_indices.append(i);
    }
    else if (shape_types[i] == int(ShapeType::Compound)) {
      compound_indices.append(i);
    }
  }

  Vector<Vector<float3>> baked_islands;
  float3 compound_center(0.0f);
  if (!compound_indices.is_empty()) {
    box_engine::resolve_compound_islands(geometry_set, compound_indices, baked_islands);
    if (!baked_islands.is_empty()) {
      int count = 0;
      for (const Vector<float3> &island : baked_islands) {
        for (const float3 &p : island) {
          compound_center += p;
          count++;
        }
      }
      if (count > 0) {
        compound_center /= float(count);
      }
    }
  }

  for (const int i : IndexRange(n)) {
    if (!compound_indices.is_empty() && i == compound_indices[0] && !baked_islands.is_empty()) {
      BodyRead cb;
      fill_body_common(cb,
                       i,
                       body_types,
                       shape_types,
                       shape_sizes,
                       radii,
                       densities,
                       frictions,
                       restitutions,
                       velocities,
                       lin_damp,
                       ang_damp,
                       convex_r,
                       radius_top,
                       dofs,
                       sensors,
                       ccds);
      cb.shape_type = int(ShapeType::Compound);
      cb.position = compound_center;
      cb.island_verts = std::move(baked_islands);
      const float4x4 body_rest = math::from_location<float4x4>(compound_center);
      bool inv_ok = false;
      const float4x4 inv_rest = math::invert(body_rest, inv_ok);
      auto add_slaves = [&](const Span<int> indices) {
        for (const int ii : indices) {
          cb.slave_instances.append(ii);
          cb.slave_local_tf.append(inv_ok ? inv_rest * transforms[ii] : transforms[ii]);
        }
      };
      add_slaves(compound_indices);
      add_slaves(hull_indices);
      cb.mass = masses[i];
      cb.inertia = inertias[i];
      cb.velocity = velocities[i];
      cb.angular_velocity = ang_vel[i];
      cb.prev_position = prev_positions[i];
      cb.prev_rotation = prev_rots[i];
      cb.has_prev = has_prev_flags[i];
      cb.name = box_engine::mstring_to_std(body_names[i]);
      r_bodies.append(std::move(cb));
      continue;
    }
    if (is_hull_inst[i]) {
      continue;
    }
    if (!compound_indices.is_empty() &&
        std::find(compound_indices.begin(), compound_indices.end(), i) != compound_indices.end())
    {
      continue;
    }

    BodyRead b;
    const float4x4 &tf = transforms[i];
    b.position = tf.location();
    b.rotation = math::to_quaternion(tf);
    b.angular_velocity = ang_vel[i];
    b.mass = masses[i];
    b.inertia = inertias[i];
    b.prev_position = prev_positions[i];
    b.prev_rotation = prev_rots[i];
    b.has_prev = has_prev_flags[i];
    fill_body_common(b,
                     i,
                     body_types,
                     shape_types,
                     shape_sizes,
                     radii,
                     densities,
                     frictions,
                     restitutions,
                     velocities,
                     lin_damp,
                     ang_damp,
                     convex_r,
                     radius_top,
                     dofs,
                     sensors,
                     ccds);
    const int handle = handles[i];
    if (handle >= 0 && handle < refs.size()) {
      apply_mesh_shape_from_reference(refs[handle], b);
    }
    const float3 scale = math::max(math::abs(math::to_scale(tf)), float3(1e-5f));
    if (ELEM(b.shape_type,
             int(ShapeType::Box),
             int(ShapeType::BoundingBox),
             int(ShapeType::ConvexHull),
             int(ShapeType::TriangleMesh),
             int(ShapeType::Compound)))
    {
      b.shape_size *= scale;
      for (float3 &p : b.local_mesh_verts) {
        p *= scale;
      }
      for (Vector<float3> &island : b.island_verts) {
        for (float3 &p : island) {
          p *= scale;
        }
      }
    }
    else if (b.shape_type == int(ShapeType::Sphere)) {
      b.shape_size.x *= (scale.x + scale.y + scale.z) / 3.0f;
    }
    else if (ELEM(b.shape_type,
                 int(ShapeType::Capsule),
                 int(ShapeType::TaperedCapsule),
                 int(ShapeType::Cylinder),
                 int(ShapeType::TaperedCylinder)))
    {
      /* Capsule / cylinder are Z-up after the bridge rotates Jolt's Y-axis shape. */
      b.shape_size.x *= (scale.x + scale.z) * 0.5f;
      b.shape_size.y *= scale.z;
    }
    b.source_instance = i;
    b.name = box_engine::mstring_to_std(body_names[i]);
    r_bodies.append(std::move(b));
  }
}

static void read_bodies_points(const PointCloud &points, Vector<BodyRead> &r_bodies)
{
  const int n = points.totpoint;
  r_bodies.reinitialize(n);
  const Span<float3> positions = points.positions();
  const bke::AttributeAccessor attributes = points.attributes();
  const bke::AttrDomain domain = bke::AttrDomain::Point;
  const VArray<int> body_types = get_int(attributes, attr::body_type, domain, 2);
  const VArray<int> shape_types = get_int(attributes, attr::shape_type, domain, 0);
  const VArray<float3> shape_sizes = get_float3(attributes, attr::shape_size, domain, float3(0.0f));
  const VArray<float> radii = get_float(attributes, attr::radius, domain, 0.25f);
  const VArray<float> densities = get_float(attributes, attr::density, domain, 1.0f);
  const VArray<float> frictions = get_float(attributes, attr::friction, domain, 0.5f);
  const VArray<float> restitutions = get_float(attributes, attr::restitution, domain, 0.0f);
  const VArray<float3> velocities = get_float3(attributes, attr::velocity, domain, float3(0.0f));
  const VArray<float3> ang = get_float3(attributes, attr::angular_velocity, domain, float3(0.0f));
  const VArray<float> masses = get_float(attributes, attr::mass, domain, 0.0f);
  const VArray<float3> inertias = get_float3(attributes, attr::inertia, domain, float3(0.0f));
  const VArray<float3> prev_positions = get_float3(
      attributes, attr::prev_position, domain, float3(0.0f));
  const VArray<math::Quaternion> prev_rots = *attributes.lookup_or_default<math::Quaternion>(
      attr::prev_rotation, domain, math::Quaternion::identity());
  const VArray<bool> has_prev_flags = get_bool(attributes, attr::has_prev, domain, false);
  const VArray<MStringProperty> body_names = get_mstring(attributes, attr::body_name, domain);
  const VArray<float> lin_damp = get_float(attributes, attr::linear_damping, domain, 0.1f);
  const VArray<float> ang_damp = get_float(attributes, attr::angular_damping, domain, 0.25f);
  const VArray<float> convex_r = get_float(attributes, attr::convex_radius, domain, 0.05f);
  const VArray<float> radius_top = get_float(attributes, attr::radius_top, domain, 0.0f);
  const VArray<int> dofs = get_int(attributes, attr::allowed_dofs, domain, 0);
  const VArray<bool> sensors = get_bool(attributes, attr::sensor, domain, false);
  const VArray<bool> ccds = get_bool(attributes, attr::ccd, domain, false);
  for (const int i : IndexRange(n)) {
    BodyRead &b = r_bodies[i];
    b.position = positions[i];
    b.angular_velocity = ang[i];
    b.mass = masses[i];
    b.inertia = inertias[i];
    b.prev_position = prev_positions[i];
    b.prev_rotation = prev_rots[i];
    b.has_prev = has_prev_flags[i];
    b.name = box_engine::mstring_to_std(body_names[i]);
    fill_body_common(b,
                     i,
                     body_types,
                     shape_types,
                     shape_sizes,
                     radii,
                     densities,
                     frictions,
                     restitutions,
                     velocities,
                     lin_damp,
                     ang_damp,
                     convex_r,
                     radius_top,
                     dofs,
                     sensors,
                     ccds);
  }
}

static void append_mesh_as_collider(const Mesh &mesh, Vector<BodyRead> &r_bodies)
{
  BodyRead b;
  const std::optional<Bounds<float3>> bounds = mesh.bounds_min_max();
  if (bounds) {
    b.position = bounds->center();
    b.shape_size = math::max(bounds->size() * 0.5f, float3(1e-5f));
  }
  b.body_type = 0;
  b.shape_type = int(ShapeType::TriangleMesh);
  b.graph_driven = false;
  b.write_back = false;
  box_engine::fill_triangle_mesh_from_mesh(mesh, b.local_mesh_verts, b.mesh_tri_indices);
  for (float3 &p : b.local_mesh_verts) {
    p -= b.position;
  }
  r_bodies.append(std::move(b));
}

static void read_joints(const PointCloud &points,
                       const GeometrySet &constraints_set,
                       Vector<JointRead> &r_joints)
{
  const int n = points.totpoint;
  r_joints.reinitialize(n);
  const Span<float3> positions = points.positions();
  const bke::AttributeAccessor attributes = points.attributes();
  const bke::AttrDomain domain = bke::AttrDomain::Point;
  const VArray<MStringProperty> a_names = get_mstring(attributes, attr::body_a, domain);
  const VArray<MStringProperty> b_names = get_mstring(attributes, attr::body_b, domain);
  const VArray<int> t = get_int(attributes, attr::joint_type, domain, 3);
  const VArray<float3> axes = get_float3(
      attributes, attr::axis, domain, float3(0.0f, 0.0f, 1.0f));
  const VArray<float3> axes2 = get_float3(
      attributes, attr::axis2, domain, float3(1.0f, 0.0f, 0.0f));
  const VArray<float3> anchor2 = get_float3(attributes, attr::anchor2, domain, float3(0.0f));
  const VArray<float> min_l = get_float(attributes, attr::min_limit, domain, 0.0f);
  const VArray<float> max_l = get_float(attributes, attr::max_limit, domain, 0.0f);
  const VArray<float> min_l2 = get_float(attributes, attr::min_limit2, domain, 0.0f);
  const VArray<float> max_l2 = get_float(attributes, attr::max_limit2, domain, 0.0f);
  const VArray<float> ratio = get_float(attributes, attr::ratio, domain, 1.0f);
  const VArray<float> freq = get_float(attributes, attr::frequency, domain, 0.0f);
  const VArray<float> damp = get_float(attributes, attr::damping, domain, 0.0f);
  const VArray<int> motor = get_int(attributes, attr::motor_state, domain, 0);
  const VArray<float> mtarget = get_float(attributes, attr::motor_target, domain, 0.0f);
  const VArray<float> mforce = get_float(attributes, attr::motor_force, domain, 0.0f);
  const VArray<int> path_id = get_int(attributes, attr::path_id, domain, -1);
  const VArray<float> wr = get_float(attributes, attr::wheel_radius, domain, 0.3f);
  const VArray<float> ww = get_float(attributes, attr::wheel_width, domain, 0.1f);
  const VArray<float> smin = get_float(attributes, attr::suspension_min, domain, 0.2f);
  const VArray<float> smax = get_float(attributes, attr::suspension_max, domain, 0.4f);
  const VArray<float> steer = get_float(attributes, attr::max_steer, domain, 0.0f);
  const VArray<int> nw = get_int(attributes, attr::num_wheels, domain, 4);

  Map<int, Vector<float3>> paths;
  if (const Mesh *mesh = constraints_set.get_mesh()) {
    const VArray<int> ids = *mesh->attributes().lookup_or_default<int>(
        attr::path_id, bke::AttrDomain::Point, -1);
    const Span<float3> vp = mesh->vert_positions();
    for (const int vi : vp.index_range()) {
      paths.lookup_or_add(ids[vi], {}).append(vp[vi]);
    }
  }

  for (const int i : IndexRange(n)) {
    JointRead &j = r_joints[i];
    j.name_a = box_engine::mstring_to_std(a_names[i]);
    j.name_b = box_engine::mstring_to_std(b_names[i]);
    j.joint_type = t[i];
    j.anchor = positions[i];
    j.axis = axes[i];
    j.axis2 = axes2[i];
    j.anchor2 = anchor2[i];
    j.min_limit = min_l[i];
    j.max_limit = max_l[i];
    j.min_limit2 = min_l2[i];
    j.max_limit2 = max_l2[i];
    j.ratio = ratio[i];
    j.frequency = freq[i];
    j.damping = damp[i];
    j.motor_state = motor[i];
    j.motor_target = mtarget[i];
    j.motor_force = mforce[i];
    j.path_id = path_id[i];
    j.wheel_radius = wr[i];
    j.wheel_width = ww[i];
    j.suspension_min = smin[i];
    j.suspension_max = smax[i];
    j.max_steer = steer[i];
    j.num_wheels = nw[i];
    if (const Vector<float3> *pv = paths.lookup_ptr(j.path_id)) {
      j.path = *pv;
    }
  }
}

static void resolve_joint_names(const Span<BodyRead> bodies, MutableSpan<JointRead> joints)
{
  Map<std::string, int> names;
  for (const int i : bodies.index_range()) {
    if (!bodies[i].name.empty()) {
      names.add_overwrite(bodies[i].name, i);
    }
  }
  for (JointRead &j : joints) {
    if (!j.name_a.empty()) {
      if (const int *idx = names.lookup_ptr(j.name_a)) {
        j.body_a = *idx;
      }
      else {
        j.body_a = -1;
      }
    }
    if (!j.name_b.empty()) {
      if (const int *idx = names.lookup_ptr(j.name_b)) {
        j.body_b = *idx;
      }
      else {
        j.body_b = -1;
      }
    }
  }
}

static bool body_pose_from_graph(const BodyRead &b)
{
  /* Static + kinematic (and anything tagged graph_driven) keep the incoming matrix.
   * Rebuilding from pos+quat flips quaternion sign / scale polar decomposition and
   * makes a spinning platform flash when it is re-joined every frame. */
  return !b.write_back || b.graph_driven || b.body_type != 2;
}

static void mark_instance_keep(const BodyRead &b, const int n_tf, MutableSpan<char> keep)
{
  if (!b.slave_instances.is_empty()) {
    for (const int ii : b.slave_instances) {
      if (ii >= 0 && ii < n_tf) {
        keep[ii] = 1;
      }
    }
    return;
  }
  const int ii = (b.source_instance >= 0) ? b.source_instance : -1;
  if (ii >= 0 && ii < n_tf) {
    keep[ii] = 1;
  }
}

static void write_instance_results(bke::Instances &instances,
                                   const Span<BodyRead> bodies,
                                   const Span<float3> positions,
                                   const Span<math::Quaternion> rotations,
                                   const Span<float3> velocities,
                                   const Span<float3> ang_vel,
                                   const Span<float3> accelerations,
                                   const Span<float3> ang_accels,
                                   const Span<float> masses,
                                   const Span<float3> inertias)
{
  MutableSpan<float4x4> transforms = instances.transforms_for_write();
  const int n_tf = transforms.size();
  /* Snapshot graph-owned poses before any dynamic write can clobber a slot. */
  Array<char> keep(n_tf, 0);
  for (const BodyRead &b : bodies) {
    if (body_pose_from_graph(b)) {
      mark_instance_keep(b, n_tf, keep);
    }
  }
  Array<float4x4> keep_tf(n_tf);
  for (int i = 0; i < n_tf; i++) {
    if (keep[i]) {
      keep_tf[i] = transforms[i];
    }
  }

  Array<float3> inst_vel(n_tf, float3(0.0f));
  Array<float3> inst_ang(n_tf, float3(0.0f));
  Array<float3> inst_accel(n_tf, float3(0.0f));
  Array<float3> inst_ang_accel(n_tf, float3(0.0f));
  Array<float> inst_mass(n_tf, 0.0f);
  Array<float3> inst_inertia(n_tf, float3(0.0f));
  Array<float3> inst_prev_pos(n_tf, float3(0.0f));
  Array<math::Quaternion> inst_prev_rot(n_tf, math::Quaternion::identity());
  Array<bool> inst_has_prev(n_tf, false);
  auto stamp_state = [&](const int ii, const int bi, const BodyRead &b) {
    if (ii < 0 || ii >= n_tf) {
      return;
    }
    inst_vel[ii] = velocities[bi];
    inst_ang[ii] = ang_vel[bi];
    inst_accel[ii] = accelerations[bi];
    inst_ang_accel[ii] = ang_accels[bi];
    inst_mass[ii] = masses[bi];
    inst_inertia[ii] = inertias[bi];
    inst_prev_pos[ii] = b.position;
    inst_prev_rot[ii] = b.rotation;
    inst_has_prev[ii] = true;
  };
  for (const int bi : bodies.index_range()) {
    const BodyRead &b = bodies[bi];
    if (body_pose_from_graph(b)) {
      const int ii = (b.source_instance >= 0) ? b.source_instance : bi;
      stamp_state(ii, bi, b);
      for (const int k : b.slave_instances.index_range()) {
        stamp_state(b.slave_instances[k], bi, b);
      }
      continue;
    }
    if (!b.slave_instances.is_empty()) {
      const float4x4 body_tf = math::from_loc_rot<float4x4>(positions[bi], rotations[bi]);
      for (const int k : b.slave_instances.index_range()) {
        const int ii = b.slave_instances[k];
        if (ii < 0 || ii >= n_tf) {
          continue;
        }
        if (!keep[ii]) {
          transforms[ii] = body_tf * b.slave_local_tf[k];
        }
        stamp_state(ii, bi, b);
      }
      continue;
    }
    const int ii = (b.source_instance >= 0) ? b.source_instance : bi;
    if (ii < 0 || ii >= n_tf) {
      continue;
    }
    if (!keep[ii]) {
      const float3 scale = math::to_scale(transforms[ii]);
      transforms[ii] = math::from_loc_rot_scale<float4x4>(positions[bi], rotations[bi], scale);
    }
    stamp_state(ii, bi, b);
  }
  for (int i = 0; i < n_tf; i++) {
    if (keep[i]) {
      transforms[i] = keep_tf[i];
    }
  }
  bke::MutableAttributeAccessor attributes = instances.attributes_for_write();
  bke::SpanAttributeWriter<float3> vel = attributes.lookup_or_add_for_write_only_span<float3>(
      attr::velocity, bke::AttrDomain::Instance);
  bke::SpanAttributeWriter<float3> ang = attributes.lookup_or_add_for_write_only_span<float3>(
      attr::angular_velocity, bke::AttrDomain::Instance);
  if (vel) {
    vel.span.copy_from(inst_vel.as_span());
    vel.finish();
  }
  if (ang) {
    ang.span.copy_from(inst_ang.as_span());
    ang.finish();
  }
  bke::SpanAttributeWriter<float3> accel_w = attributes.lookup_or_add_for_write_only_span<float3>(
      attr::acceleration, bke::AttrDomain::Instance);
  bke::SpanAttributeWriter<float3> ang_accel_w =
      attributes.lookup_or_add_for_write_only_span<float3>(attr::angular_acceleration,
                                                          bke::AttrDomain::Instance);
  if (accel_w) {
    accel_w.span.copy_from(inst_accel.as_span());
    accel_w.finish();
  }
  if (ang_accel_w) {
    ang_accel_w.span.copy_from(inst_ang_accel.as_span());
    ang_accel_w.finish();
  }
  bke::SpanAttributeWriter<float> mass_w = attributes.lookup_or_add_for_write_only_span<float>(
      attr::mass, bke::AttrDomain::Instance);
  bke::SpanAttributeWriter<float3> inertia_w = attributes.lookup_or_add_for_write_only_span<float3>(
      attr::inertia, bke::AttrDomain::Instance);
  bke::SpanAttributeWriter<float3> prev_pos_w = attributes.lookup_or_add_for_write_only_span<float3>(
      attr::prev_position, bke::AttrDomain::Instance);
  bke::SpanAttributeWriter<math::Quaternion> prev_rot_w =
      attributes.lookup_or_add_for_write_only_span<math::Quaternion>(attr::prev_rotation,
                                                                    bke::AttrDomain::Instance);
  if (mass_w) {
    mass_w.span.copy_from(inst_mass.as_span());
    mass_w.finish();
  }
  if (inertia_w) {
    inertia_w.span.copy_from(inst_inertia.as_span());
    inertia_w.finish();
  }
  if (prev_pos_w) {
    prev_pos_w.span.copy_from(inst_prev_pos.as_span());
    prev_pos_w.finish();
  }
  if (prev_rot_w) {
    prev_rot_w.span.copy_from(inst_prev_rot.as_span());
    prev_rot_w.finish();
  }
  bke::SpanAttributeWriter<bool> has_prev_w = attributes.lookup_or_add_for_write_only_span<bool>(
      attr::has_prev, bke::AttrDomain::Instance);
  if (has_prev_w) {
    has_prev_w.span.copy_from(inst_has_prev.as_span());
    has_prev_w.finish();
  }
}

static uint64_t hash_topology(const Span<BodyRead> bodies, const Span<JointRead> joints)
{
  uint64_t h = uint64_t(bodies.size()) * 1000003ull ^ uint64_t(joints.size());
  for (const BodyRead &b : bodies) {
    h = h * 1000003ull ^ uint64_t(b.shape_type);
    h = h * 1000003ull ^ uint64_t(b.body_type);
    h = h * 1000003ull ^ uint64_t(b.local_mesh_verts.size());
    h = h * 1000003ull ^ uint64_t(b.mesh_tri_indices.size());
    h = h * 1000003ull ^ uint64_t(b.island_verts.size());
  }
  for (const JointRead &j : joints) {
    h = h * 1000003ull ^ uint64_t(j.joint_type);
    h = h * 1000003ull ^ uint64_t(uint32_t(j.body_a));
    h = h * 1000003ull ^ uint64_t(uint32_t(j.body_b));
    h = h * 1000003ull ^ uint64_t(j.path.size());
  }
  return h;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry"_ustr);
#ifndef WITH_JOLT
  params.error_message_add(NodeWarningType::Error, "Built without WITH_JOLT");
  params.set_output("Geometry"_ustr, std::move(geometry_set));
  return;
#else
  GeometrySet constraints_set = params.extract_input<GeometrySet>("Constraints"_ustr);
  const float3 gravity = params.extract_input<float3>("Gravity"_ustr);
  const float dt = params.extract_input<float>("Delta Time"_ustr);
  const int substeps = std::max(params.extract_input<int>("Substeps"_ustr), 1);
  const int vel_steps = std::max(params.extract_input<int>("Velocity Steps"_ustr), 2);
  const int pos_steps = std::max(params.extract_input<int>("Position Steps"_ustr), 1);
  const bool ccd = params.extract_input<bool>("CCD"_ustr);
  const Object *self = params.self_object();
  const unsigned int uid = self ? self->id.session_uid : 0;
  const std::string cache_ns = fmt::format("jolt:{}:{}", uid, params.node().identifier);

  if (geometry_set.is_empty()) {
    params.set_output("Geometry"_ustr, std::move(geometry_set));
    return;
  }

  Vector<BodyRead> bodies;
  enum class DomainKind { None, Points, Instances, Mesh } domain = DomainKind::None;
  if (const bke::Instances *instances = geometry_set.get_instances()) {
    if (instances->instances_num() > 0) {
      domain = DomainKind::Instances;
      read_bodies_instances(geometry_set, bodies);
      if (const Mesh *mesh = geometry_set.get_mesh()) {
        if (box_engine::mesh_is_compound_proxy(*mesh)) {
          geometry_set.keep_only({GeometryComponent::Type::Instance});
        }
      }
    }
  }
  if (domain == DomainKind::None) {
    if (const PointCloud *points = geometry_set.get_pointcloud()) {
      if (points->totpoint > 0) {
        domain = DomainKind::Points;
        read_bodies_points(*points, bodies);
      }
    }
  }
  if (domain == DomainKind::None) {
    if (const Mesh *mesh = geometry_set.get_mesh()) {
      if (mesh->verts_num > 0) {
        domain = DomainKind::Mesh;
        /* Bare mesh is a static collider. Tag Dynamic via Jolt Set Rigid Body. */
        append_mesh_as_collider(*mesh, bodies);
        if (!bodies.is_empty()) {
          bodies.last().write_back = false;
          bodies.last().graph_driven = false;
          bodies.last().body_type = 0;
        }
      }
    }
  }
  if (domain == DomainKind::None || bodies.is_empty()) {
    params.error_message_add(
        NodeWarningType::Error,
        TIP_("Jolt: no bodies. Use Instances, Points, or Mesh (try Jolt Set Rigid Body)"));
    params.set_output("Geometry"_ustr, std::move(geometry_set));
    return;
  }

  Vector<JointRead> joints;
  if (const PointCloud *cpoints = constraints_set.get_pointcloud()) {
    if (cpoints->totpoint > 0) {
      read_joints(*cpoints, constraints_set, joints);
      resolve_joint_names(bodies, joints);
    }
  }

  /* Pack mesh data so pointers stay valid across JOLT_step. */
  Vector<Vector<float>> packed_xyz(bodies.size());
  Vector<Vector<int>> packed_idx(bodies.size());
  Vector<Vector<int>> packed_islands(bodies.size());
  Vector<Vector<float>> packed_height(bodies.size());
  Vector<std::string> packed_names(bodies.size());
  Array<JoltBodyDesc> descs(bodies.size());
  Array<JoltBodyState> states(bodies.size());
  int kin_slot = 0;

  for (const int i : bodies.index_range()) {
    const BodyRead &b = bodies[i];
    JoltBodyDesc &d = descs[i];
    memset(&d, 0, sizeof(d));
    d.position = {b.position.x, b.position.y, b.position.z};
    d.rotation = {b.rotation.w, b.rotation.x, b.rotation.y, b.rotation.z};
    d.linear_velocity = {b.velocity.x, b.velocity.y, b.velocity.z};
    d.angular_velocity = {b.angular_velocity.x, b.angular_velocity.y, b.angular_velocity.z};
    d.shape_size = {b.shape_size.x, b.shape_size.y, b.shape_size.z};
    d.density = b.density;
    d.friction = b.friction;
    d.restitution = b.restitution;
    d.linear_damping = b.linear_damping;
    d.angular_damping = b.angular_damping;
    d.convex_radius = b.convex_radius;
    d.radius_top = b.radius_top;
    d.motion_type = b.body_type;
    d.shape_type = b.shape_type;
    d.allowed_dofs = b.allowed_dofs;
    d.sensor = b.sensor ? 1 : 0;
    d.ccd = (b.ccd || ccd) ? 1 : 0;
    d.graph_driven = b.graph_driven ? 1 : 0;
    d.mass = b.mass;
    d.inertia = {b.inertia.x, b.inertia.y, b.inertia.z};
    d.prev_position = {b.prev_position.x, b.prev_position.y, b.prev_position.z};
    d.prev_rotation = {b.prev_rotation.w, b.prev_rotation.x, b.prev_rotation.y, b.prev_rotation.z};
    d.has_prev = b.has_prev ? 1 : 0;
    if (!b.name.empty()) {
      packed_names[i] = b.name;
    }
    else if (b.body_type == 1) {
      /* Stable across re-join: do not bake AABB size (a spinning plane's AABB
       * changes every frame and would destroy+create the kinematic body). */
      packed_names[i] = fmt::format("kin#{}", kin_slot++);
    }
    else {
      packed_names[i] = fmt::format("idx:{}", i);
    }
    d.name = packed_names[i].c_str();

    if (!b.local_mesh_verts.is_empty()) {
      packed_xyz[i].resize(size_t(b.local_mesh_verts.size()) * 3);
      for (const int k : b.local_mesh_verts.index_range()) {
        packed_xyz[i][size_t(k) * 3 + 0] = b.local_mesh_verts[k].x;
        packed_xyz[i][size_t(k) * 3 + 1] = b.local_mesh_verts[k].y;
        packed_xyz[i][size_t(k) * 3 + 2] = b.local_mesh_verts[k].z;
      }
      d.verts_xyz = packed_xyz[i].data();
      d.nverts = b.local_mesh_verts.size();
    }
    if (!b.mesh_tri_indices.is_empty()) {
      packed_idx[i] = Vector<int>(b.mesh_tri_indices.as_span());
      d.tri_indices = packed_idx[i].data();
      d.nindices = packed_idx[i].size();
    }
    if (!b.island_verts.is_empty() && b.shape_type == int(ShapeType::Compound)) {
      packed_xyz[i].clear();
      packed_islands[i].clear();
      for (const Vector<float3> &island : b.island_verts) {
        packed_islands[i].append(island.size());
        for (const float3 &p : island) {
          /* Island verts are world-space; hulls must be body-local. */
          packed_xyz[i].append(p.x - b.position.x);
          packed_xyz[i].append(p.y - b.position.y);
          packed_xyz[i].append(p.z - b.position.z);
        }
      }
      d.verts_xyz = packed_xyz[i].data();
      d.nverts = packed_xyz[i].size() / 3;
      d.island_counts = packed_islands[i].data();
      d.nislands = packed_islands[i].size();
    }
    if (b.shape_type == int(ShapeType::HeightField) && b.local_mesh_verts.size() >= 4) {
      const int side = int(std::floor(std::sqrt(double(b.local_mesh_verts.size()))));
      if (side >= 2) {
        packed_height[i].resize(size_t(side) * size_t(side));
        float minx = 1e30f, maxx = -1e30f, miny = 1e30f, maxy = -1e30f;
        for (const float3 &p : b.local_mesh_verts) {
          minx = std::min(minx, p.x);
          maxx = std::max(maxx, p.x);
          miny = std::min(miny, p.y);
          maxy = std::max(maxy, p.y);
        }
        for (float &s : packed_height[i]) {
          s = 0.0f;
        }
        const float dx = std::max(maxx - minx, 1e-4f);
        const float dy = std::max(maxy - miny, 1e-4f);
        for (const float3 &p : b.local_mesh_verts) {
          const int ix = std::clamp(int((p.x - minx) / dx * float(side - 1) + 0.5f), 0, side - 1);
          const int iy = std::clamp(int((p.y - miny) / dy * float(side - 1) + 0.5f), 0, side - 1);
          packed_height[i][size_t(iy) * size_t(side) + size_t(ix)] = p.z;
        }
        d.height_samples = packed_height[i].data();
        d.height_size = side;
        d.height_cell = dx / float(std::max(side - 1, 1));
      }
    }
  }

  {
    Set<std::string> used;
    for (const int i : bodies.index_range()) {
      std::string n = packed_names[i];
      if (used.contains(n)) {
        n = (bodies[i].body_type == 1) ? (n + ":kin") : fmt::format("{}#{}", n, i);
        while (used.contains(n)) {
          n += "_";
        }
      }
      used.add(n);
      packed_names[i] = std::move(n);
      descs[i].name = packed_names[i].c_str();
    }
  }

  Vector<Vector<float>> packed_path(joints.size());
  Array<JoltConstraintDesc> cdescs(joints.size());
  for (const int i : joints.index_range()) {
    const JointRead &j = joints[i];
    JoltConstraintDesc &d = cdescs[i];
    memset(&d, 0, sizeof(d));
    d.type = j.joint_type;
    d.body_a = j.body_a;
    d.body_b = j.body_b;
    d.anchor = {j.anchor.x, j.anchor.y, j.anchor.z};
    d.axis = {j.axis.x, j.axis.y, j.axis.z};
    d.axis2 = {j.axis2.x, j.axis2.y, j.axis2.z};
    d.anchor2 = {j.anchor2.x, j.anchor2.y, j.anchor2.z};
    d.min_limit = j.min_limit;
    d.max_limit = j.max_limit;
    d.min_limit2 = j.min_limit2;
    d.max_limit2 = j.max_limit2;
    d.ratio = j.ratio;
    d.frequency = j.frequency;
    d.damping = j.damping;
    d.motor_state = j.motor_state;
    d.motor_target = j.motor_target;
    d.motor_force = j.motor_force;
    d.wheel_radius = j.wheel_radius;
    d.wheel_width = j.wheel_width;
    d.suspension_min = j.suspension_min;
    d.suspension_max = j.suspension_max;
    d.max_steer = j.max_steer;
    d.num_wheels = j.num_wheels;
    if (!j.path.is_empty()) {
      packed_path[i].resize(size_t(j.path.size()) * 3);
      for (const int k : j.path.index_range()) {
        packed_path[i][size_t(k) * 3 + 0] = j.path[k].x;
        packed_path[i][size_t(k) * 3 + 1] = j.path[k].y;
        packed_path[i][size_t(k) * 3 + 2] = j.path[k].z;
      }
      d.path_xyz = packed_path[i].data();
      d.npath = j.path.size();
    }
  }

  JoltStepSettings settings{};
  settings.gravity = {gravity.x, gravity.y, gravity.z};
  settings.dt = dt;
  settings.collision_steps = substeps;
  settings.velocity_steps = vel_steps;
  settings.position_steps = pos_steps;
  settings.ccd = ccd ? 1 : 0;
  settings.topology_hash = hash_topology(bodies, joints);
  settings.cache_key = cache_ns.c_str();
  settings.scene_time = 0.0f;
  if (const Depsgraph *depsgraph = params.depsgraph()) {
    settings.scene_time = float(DEG_get_ctime(depsgraph));
  }

  char err[256] = {};
  std::lock_guard<std::mutex> solver_lock(deformable_solver::jolt_engine_mutex());
  const int ok = JOLT_step(&settings,
                           bodies.size(),
                           descs.data(),
                           states.data(),
                           joints.size(),
                           joints.is_empty() ? nullptr : cdescs.data(),
                           err,
                           int(sizeof(err)));
  if (!ok) {
    params.error_message_add(NodeWarningType::Error, err[0] ? err : "Jolt step failed");
    params.set_output("Geometry"_ustr, std::move(geometry_set));
    return;
  }

  Array<float3> new_positions(bodies.size());
  Array<math::Quaternion> new_rots(bodies.size());
  Array<float3> new_vel(bodies.size());
  Array<float3> new_ang(bodies.size());
  for (const int i : bodies.index_range()) {
    const BodyRead &src = bodies[i];
    if (body_pose_from_graph(src)) {
      new_positions[i] = src.position;
      new_rots[i] = src.rotation;
    }
    else {
      new_positions[i] = float3(
          states[i].position.x, states[i].position.y, states[i].position.z);
      new_rots[i] = math::Quaternion(states[i].rotation.w,
                                     states[i].rotation.x,
                                     states[i].rotation.y,
                                     states[i].rotation.z);
    }
    new_vel[i] = float3(
        states[i].linear_velocity.x, states[i].linear_velocity.y, states[i].linear_velocity.z);
    new_ang[i] = float3(states[i].angular_velocity.x,
                        states[i].angular_velocity.y,
                        states[i].angular_velocity.z);
  }

  Array<float> new_mass(bodies.size(), 0.0f);
  Array<float3> new_inertia(bodies.size(), float3(0.0f));
  Array<float3> new_accel(bodies.size(), float3(0.0f));
  Array<float3> new_ang_accel(bodies.size(), float3(0.0f));
  Array<float3> new_prev_pos(bodies.size(), float3(0.0f));
  Array<math::Quaternion> new_prev_rot(bodies.size(), math::Quaternion::identity());
  Array<bool> new_has_prev(bodies.size(), true);
  for (const int i : bodies.index_range()) {
    new_mass[i] = states[i].mass > 1.0e-8f ? states[i].mass : bodies[i].mass;
    const float3 si(states[i].inertia.x, states[i].inertia.y, states[i].inertia.z);
    new_inertia[i] = math::length_squared(si) > 1.0e-16f ? si : bodies[i].inertia;
    new_accel[i] = float3(states[i].linear_acceleration.x,
                          states[i].linear_acceleration.y,
                          states[i].linear_acceleration.z);
    new_ang_accel[i] = float3(states[i].angular_acceleration.x,
                              states[i].angular_acceleration.y,
                              states[i].angular_acceleration.z);
    new_prev_pos[i] = bodies[i].position;
    new_prev_rot[i] = bodies[i].rotation;
  }

  if (domain == DomainKind::Instances) {
    if (bke::Instances *instances = geometry_set.get_instances_for_write()) {
      write_instance_results(*instances,
                             bodies,
                             new_positions,
                             new_rots,
                             new_vel,
                             new_ang,
                             new_accel,
                             new_ang_accel,
                             new_mass,
                             new_inertia);
    }
  }
  else if (domain == DomainKind::Points) {
    if (PointCloud *points = geometry_set.get_pointcloud_for_write()) {
      const int n = std::min(points->totpoint, int(new_positions.size()));
      MutableSpan<float3> pos = points->positions_for_write();
      for (int i = 0; i < n; i++) {
        pos[i] = new_positions[i];
      }
      deformable_solver::write_float3(
          points->attributes_for_write(), attr::velocity, bke::AttrDomain::Point, new_vel);
      deformable_solver::write_float3(points->attributes_for_write(),
                                      attr::angular_velocity,
                                      bke::AttrDomain::Point,
                                      new_ang);
      deformable_solver::write_float3(
          points->attributes_for_write(), attr::acceleration, bke::AttrDomain::Point, new_accel);
      deformable_solver::write_float3(points->attributes_for_write(),
                                      attr::angular_acceleration,
                                      bke::AttrDomain::Point,
                                      new_ang_accel);
      deformable_solver::write_float(
          points->attributes_for_write(), attr::mass, bke::AttrDomain::Point, new_mass);
      deformable_solver::write_float3(
          points->attributes_for_write(), attr::inertia, bke::AttrDomain::Point, new_inertia);
      deformable_solver::write_float3(points->attributes_for_write(),
                                      attr::prev_position,
                                      bke::AttrDomain::Point,
                                      new_prev_pos);
      {
        bke::MutableAttributeAccessor attributes = points->attributes_for_write();
        bke::SpanAttributeWriter<math::Quaternion> prev_rot_w =
            attributes.lookup_or_add_for_write_only_span<math::Quaternion>(attr::prev_rotation,
                                                                          bke::AttrDomain::Point);
        if (prev_rot_w) {
          const int nwrite = std::min(int(prev_rot_w.span.size()), int(new_prev_rot.size()));
          for (int i = 0; i < nwrite; i++) {
            prev_rot_w.span[i] = new_prev_rot[i];
          }
          prev_rot_w.finish();
        }
        bke::SpanAttributeWriter<bool> has_prev_w = attributes.lookup_or_add_for_write_only_span<bool>(
            attr::has_prev, bke::AttrDomain::Point);
        if (has_prev_w) {
          const int nwrite = std::min(int(has_prev_w.span.size()), int(new_has_prev.size()));
          for (int i = 0; i < nwrite; i++) {
            has_prev_w.span[i] = new_has_prev[i];
          }
          has_prev_w.finish();
        }
      }
    }
  }
  else if (domain == DomainKind::Mesh) {
    if (!bodies.is_empty() && !body_pose_from_graph(bodies[0])) {
      if (Mesh *mesh = geometry_set.get_mesh_for_write()) {
        const float3 delta = new_positions[0] - bodies[0].position;
        MutableSpan<float3> verts = mesh->vert_positions_for_write();
        for (float3 &v : verts) {
          v += delta;
        }
        mesh->tag_positions_changed();
      }
    }
  }

  params.set_output("Geometry"_ustr, std::move(geometry_set));
#endif
}

static void register_jolt_solver()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeJoltSolver"_ustr, GEO_NODE_JOLT_SOLVER);
  ntype.ui_name = "Jolt Solver";
  ntype.ui_description =
      "Black-box Jolt step from input geometry. Writes velocity, angular_velocity, "
      "acceleration, angular_acceleration, mass and inertia (I, not angular accel)";
  ntype.enum_name_legacy = "JOLT_SOLVER";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(register_jolt_solver)

}  // namespace blender::nodes::node_geo_jolt_solver_cc
