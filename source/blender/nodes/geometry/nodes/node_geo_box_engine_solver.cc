/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Geometry Nodes: Box Engine Solver — Box2D / Box3D black-box rigid body step.
 *
 * Bodies: Instances (preferred), Points, or Mesh.
 * Shape types: Sphere, Box, Capsule, ConvexHull, BoundingBox, TriangleMesh, Compound.
 * Density: only for Dynamic mass; Static ignores it. Internal density fallback = 1.
 * Constraints: optional Point Cloud from Set Constraint.
 *   Distance / Weld / Prismatic: two named bodies (body_a, body_b).
 *   Revolute: body_a + point position = world hinge. Always pinned to the world.
 * Compound: one convex hull per mesh island (never the overall AABB).
 * TriangleMesh (Box3D Static): real non-convex mesh contacts vs convex shapes.
 * TriangleMesh (Box2D): one convex polygon per triangle.
 * TriangleMesh (dynamic Box3D): per-island convex hulls. Mesh data kept until after world destroy.
 * The physics world is kept across frames (contact warm-start). Static / Kinematic
 * rigid bodies on Geometry are the obstacles (no separate Colliders socket).
 * Changing static count does not reset stacked dynamics. Moving obstacles: Kinematic.
 * Box2D is locked to the XY plane: 2D (x,y) = world (x,y); rotation about Z.
 * Gravity uses (gx, gy); if gy≈0, gz is used so the default (0,0,-9.81) still falls.
 */

#include "BKE_attribute.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "BLI_array.hh"
#include "BLI_bounds.hh"
#include "BLI_convexhull_2d.hh"
#include "BLI_math_matrix.hh"
#include "BLI_vector.hh"
#include "BLI_math_quaternion.hh"
#include "BLI_math_rotation.hh"
#include "BLI_math_vector.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_pointcloud_types.h"

#include "BLI_map.hh"
#include "BLI_set.hh"
#include "BLI_string_ref.hh"

#include "box_engine_shared.hh"
#include "deformable_solver_shared.hh"
#include "node_geometry_util.hh"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fmt/format.h>
#include <limits>
#include <mutex>
#include <string>

#if defined(WITH_BOX2D)
#  include "box2d/box2d.h"
#endif
#if defined(WITH_BOX3D)
#  include "box3d/box3d.h"
#endif

namespace blender::nodes::node_geo_box_engine_solver_cc {

enum class EngineKind { Box2D = 0, Box3D = 1 };
enum class PlaneAxis { XY = 0, XZ = 1 };

enum class ShapeType {
  Sphere = 0,
  Box = 1,
  Capsule = 2,
  ConvexHull = 3,
  BoundingBox = 4,
  TriangleMesh = 5,
  Compound = 6,
};
enum class JointType { Distance = 0, Revolute = 1, Weld = 2, Prismatic = 3 };

static const EnumPropertyItem engine_items[] = {
    {int(EngineKind::Box2D), "BOX2D", 0, N_("Box2D"), N_("2D rigid body engine")},
    {int(EngineKind::Box3D), "BOX3D", 0, N_("Box3D"), N_("3D rigid body engine")},
    {0, nullptr, 0, nullptr, nullptr},
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
constexpr StringRefNull radius = "radius";
constexpr StringRefNull body_a = "body_a";
constexpr StringRefNull body_b = "body_b";
constexpr StringRefNull body_name = "body_name";
constexpr StringRefNull joint_type = "joint_type";
constexpr StringRefNull axis = "axis";
constexpr StringRefNull length = "length";
}  // namespace attr

static std::string mstring_to_std(const MStringProperty &p)
{
  const int n = std::clamp(int(p.s_len), 0, 127);
  if (n <= 0) {
    return {};
  }
  return std::string(p.s, p.s + n);
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  /* No supported_type filter — Instances often carry mesh references and were wrongly rejected. */
  b.add_input<decl::Geometry>("Geometry"_ustr)
      .description(
          "Bodies: Instances (preferred). Join Dynamic boxes with Static walls / "
          "Kinematic movers. Use Set Rigid Body to tag type and shape");
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Bodies with updated transforms and velocity attributes");

  b.add_input<decl::Geometry>("Constraints"_ustr)
      .description(
          "Optional Point Cloud from Set Constraint. Revolute hinges Body A to a world "
          "Anchor; Distance / Weld / Prismatic join two named bodies");

  b.add_input<decl::Menu>("Engine"_ustr)
      .static_items(engine_items)
      .default_value(EngineKind::Box2D)
      .optional_label()
      .description("Box2D (XY) or Box3D solver");

  b.add_input<decl::Vector>("Gravity"_ustr)
      .default_value(float3(0.0f, 0.0f, -9.81f))
      .subtype(PROP_ACCELERATION)
      .description("World gravity. Box2D uses (X, Y); if Y≈0, Z is used so (0,0,-9.81) still falls");

  b.add_input<decl::Float>("Delta Time"_ustr)
      .default_value(1.0f / 24.0f)
      .min(0.0f)
      .subtype(PROP_TIME_ABSOLUTE)
      .description("Use Scene Time delta inside a Simulation Zone");

  b.add_input<decl::Int>("Substeps"_ustr)
      .default_value(16)
      .min(1)
      .max(64)
      .description(
          "Collide/solve slices this frame. 16 means 16 physics steps of dt/16. "
          "This is the collision rate, not a hidden solver knob");

  /* No Default Density/Radius: shapes and mass come from Set Rigid Body attributes.
   * Internal fallbacks (density=1, half=0.25) only if attributes are missing. */
}

struct BodyRead {
  float3 position = float3(0.0f);
  float3 velocity = float3(0.0f);
  float3 shape_size = float3(0.25f);
  float density = 1.0f;
  float friction = 0.3f;
  float restitution = 0.1f;
  float angle = 0.0f;
  math::Quaternion rotation = math::Quaternion::identity();
  float3 angular_velocity = float3(0.0f);
  int body_type = 2;
  int shape_type = int(ShapeType::BoundingBox);
  /** Local-space mesh verts for ConvexHull / TriangleMesh / offset bounds (empty if unused). */
  Vector<float3> local_mesh_verts;
  /** Triangle mesh (local) for TriangleMesh shape: 3 indices per triangle. */
  Vector<int> mesh_tri_indices;
  /** Per-island vertex lists for Compound (full local positions per island). */
  Vector<Vector<float3>> island_verts;
  /** Compound (and optional multi-instance) write-back: body_tf * local = instance_tf. */
  Vector<int> slave_instances;
  Vector<float4x4> slave_local_tf;
  /** Regular 1:1 body → instance (Compound uses slaves instead). */
  int source_instance = -1;
  /** Instance/point body_name used by Set Constraint. */
  std::string name;
  /** Write solver pose back onto Geometry (false for Colliders / leftover mesh). */
  bool write_back = true;
  /** Pose comes from the node graph (static/kinematic); dynamics are owned by the world. */
  bool graph_driven = false;
};

struct JointRead {
  int body_a = 0;
  int body_b = -1;
  std::string name_a;
  std::string name_b;
  int joint_type = int(JointType::Revolute);
  float3 anchor = float3(0.0f);
  float3 axis = float3(1.0f, 0.0f, 0.0f);
  float length = 0.0f;
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
                             const float default_radius,
                             const float default_density)
{
  b.body_type = body_types[i];
  b.shape_type = shape_types[i];
  b.shape_size = shape_sizes[i];
  /* If shape_size was default zeros, fall back to radius. */
  if (math::length_squared(b.shape_size) < 1e-12f) {
    const float r = math::max(radii[i], math::max(default_radius, 1e-3f));
    b.shape_size = float3(r, r, r);
  }
  b.shape_size = math::max(b.shape_size, float3(1e-5f));
  /* Dynamic with density 0 → zero mass → floats forever (user Default Density=0 bug). */
  const float dens = densities[i] > 0.0f ? densities[i] : math::max(default_density, 1e-3f);
  if (b.body_type == 2) {
    b.density = math::max(dens, 1e-3f);
  }
  else {
    b.density = dens;
  }
  b.friction = frictions[i];
  b.restitution = restitutions[i];
  b.velocity = velocities[i];
  b.graph_driven = (b.body_type != 2);
}

static void consume_collision_mesh(const Mesh &mesh, BodyRead &b)
{
  if (mesh.verts_num == 0) {
    return;
  }
  const std::optional<Bounds<float3>> bounds = mesh.bounds_min_max();
  if (!bounds) {
    return;
  }
  const float3 half = math::max(bounds->size() * 0.5f, float3(1e-5f));
  const float3 local_center = bounds->center();

  if (ELEM(b.shape_type,
           int(ShapeType::BoundingBox),
           int(ShapeType::ConvexHull),
           int(ShapeType::TriangleMesh),
           int(ShapeType::Compound)) ||
      math::length_squared(b.shape_size) < 1e-12f)
  {
    b.shape_size = half;
  }

  if (b.shape_type == int(ShapeType::ConvexHull)) {
    b.local_mesh_verts.extend(mesh.vert_positions());
  }
  else if (b.shape_type == int(ShapeType::TriangleMesh)) {
    box_engine::fill_triangle_mesh_from_mesh(mesh, b.local_mesh_verts, b.mesh_tri_indices);
    /* Dynamic Box3D cannot generate mesh contacts — keep islands for hull fallback. */
    box_engine::fill_islands_from_mesh(mesh, b.island_verts);
  }
  else if (b.shape_type == int(ShapeType::Compound)) {
    box_engine::fill_islands_from_mesh(mesh, b.island_verts);
  }
  else if (ELEM(b.shape_type, int(ShapeType::BoundingBox), int(ShapeType::Box)) &&
           math::length(local_center) > 1e-4f)
  {
    b.local_mesh_verts.extend(mesh.vert_positions());
  }
}

/**
 * Pull collision geometry from an instance reference **while the GeometrySet is alive**.
 * Realizes Object / Collection / nested instances so a Mesh is actually available.
 */
static void apply_mesh_shape_from_reference(const bke::InstanceReference &ref, BodyRead &b)
{
  const GeometrySet geo = box_engine::collision_geometry_from_reference(ref);
  if (const Mesh *mesh = geo.get_mesh()) {
    consume_collision_mesh(*mesh, b);
  }
}

static void read_bodies_instances(const GeometrySet &geometry_set,
                                  const float default_radius,
                                  const float default_density,
                                  Vector<BodyRead> &r_bodies,
                                  const int default_body_type = 2)
{
  const bke::Instances &instances = *geometry_set.get_instances();
  const int n = instances.instances_num();
  const Span<float4x4> transforms = instances.transforms();
  const bke::AttributeAccessor attributes = instances.attributes();
  const bke::AttrDomain domain = bke::AttrDomain::Instance;

  const VArray<int> body_types = get_int(attributes, attr::body_type, domain, default_body_type);
  const VArray<int> shape_types = get_int(
      attributes, attr::shape_type, domain, int(ShapeType::BoundingBox));
  const VArray<float3> shape_sizes = get_float3(
      attributes, attr::shape_size, domain, float3(0.0f));
  const VArray<float> radii = get_float(attributes, attr::radius, domain, default_radius);
  const VArray<float> densities = get_float(attributes, attr::density, domain, default_density);
  const VArray<float> frictions = get_float(attributes, attr::friction, domain, 0.4f);
  const VArray<float> restitutions = get_float(attributes, attr::restitution, domain, 0.05f);
  const VArray<float3> velocities = get_float3(attributes, attr::velocity, domain, float3(0.0f));
  const VArray<float3> ang_vel = get_float3(
      attributes, attr::angular_velocity, domain, float3(0.0f));
  const VArray<MStringProperty> body_names = get_mstring(attributes, attr::body_name, domain);

  const Span<int> handles = instances.reference_handles();
  const Span<bke::InstanceReference> refs = instances.references();

  const VArray<bool> is_hull_inst = *attributes.lookup_or_default<bool>(
      box_engine::attr_compound_hull, domain, false);

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
  float3 compound_half(0.25f);
  if (!hull_indices.is_empty()) {
    /* Each hull instance is already one convex piece — do not merge their verts. */
    for (const int hi : hull_indices) {
      const int handle = handles[hi];
      if (handle < 0 || handle >= refs.size()) {
        continue;
      }
      const GeometrySet extracted = box_engine::collision_geometry_from_reference(refs[handle]);
      const Mesh *mesh = extracted.get_mesh();
      if (!mesh || mesh->verts_num < 3) {
        continue;
      }
      Vector<float3> verts;
      verts.reserve(mesh->verts_num);
      for (const float3 &p : mesh->vert_positions()) {
        verts.append(math::transform_point(transforms[hi], p));
      }
      baked_islands.append(std::move(verts));
    }
  }
  else if (!compound_indices.is_empty()) {
    box_engine::resolve_compound_islands(geometry_set, compound_indices, baked_islands);
  }
  if (!baked_islands.is_empty()) {
    Vector<float3> all;
    for (const Vector<float3> &island : baked_islands) {
      all.extend(island);
    }
    if (const std::optional<Bounds<float3>> bounds = bounds::min_max(all.as_span())) {
      compound_center = bounds->center();
      compound_half = math::max(bounds->size() * 0.5f, float3(1e-5f));
      for (Vector<float3> &island : baked_islands) {
        for (float3 &p : island) {
          p -= compound_center;
        }
      }
    }
  }

  const bool use_baked_compound = !baked_islands.is_empty() &&
                                  (!compound_indices.is_empty() || !hull_indices.is_empty());

  bool emitted_compound = false;
  for (const int i : IndexRange(n)) {
    if (use_baked_compound &&
        (is_hull_inst[i] || shape_types[i] == int(ShapeType::Compound)))
    {
      if (emitted_compound) {
        continue;
      }
      emitted_compound = true;
      BodyRead cb;
      const int attr_i = compound_indices.is_empty() ? i : compound_indices[0];
      fill_body_common(cb,
                       attr_i,
                       body_types,
                       shape_types,
                       shape_sizes,
                       radii,
                       densities,
                       frictions,
                       restitutions,
                       velocities,
                       default_radius,
                       default_density);
      cb.shape_type = int(ShapeType::Compound);
      cb.position = compound_center;
      cb.rotation = math::Quaternion::identity();
      cb.angle = 0.0f;
      cb.shape_size = compound_half;
      cb.island_verts = std::move(baked_islands);

      float3 vsum(0.0f);
      float3 asum(0.0f);
      int slave_n = 0;
      const float4x4 body_rest = math::from_location<float4x4>(compound_center);
      bool inv_ok = false;
      const float4x4 inv_rest = math::invert(body_rest, inv_ok);
      auto add_slaves = [&](const Span<int> indices) {
        for (const int ii : indices) {
          cb.slave_instances.append(ii);
          cb.slave_local_tf.append(inv_ok ? inv_rest * transforms[ii] : transforms[ii]);
          vsum += velocities[ii];
          asum += ang_vel[ii];
          slave_n++;
        }
      };
      add_slaves(compound_indices);
      add_slaves(hull_indices);
      if (slave_n > 0) {
        const float inv_n = 1.0f / float(slave_n);
        cb.velocity = vsum * inv_n;
        cb.angular_velocity = asum * inv_n;
      }
      cb.name = mstring_to_std(body_names[attr_i]);
      r_bodies.append(std::move(cb));
      continue;
    }
    if (is_hull_inst[i]) {
      continue;
    }

    BodyRead b;
    const float4x4 &tf = transforms[i];
    b.position = tf.location();
    b.rotation = math::to_quaternion(tf);
    {
      const math::EulerXYZ e = math::to_euler(b.rotation);
      b.angle = float(e.z());
    }
    b.angular_velocity = ang_vel[i];
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
                     default_radius,
                     default_density);

    const int handle = handles[i];
    if (handle >= 0 && handle < refs.size()) {
      apply_mesh_shape_from_reference(refs[handle], b);
    }

    /* Per-instance Compound fallback (no baked proxy): still hull that instance's islands. */
    if (b.shape_type == int(ShapeType::Compound) && b.island_verts.is_empty() &&
        handle >= 0 && handle < refs.size())
    {
      const GeometrySet extracted = box_engine::collision_geometry_from_reference(refs[handle]);
      if (const Mesh *mesh = extracted.get_mesh()) {
        box_engine::fill_islands_from_mesh(*mesh, b.island_verts);
      }
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
    else if (b.shape_type == int(ShapeType::Capsule)) {
      b.shape_size.x *= (scale.x + scale.z) * 0.5f;
      b.shape_size.y *= scale.y;
    }
    /* 1:1 write-back via source_instance (do not use slave scale multiply — it shears). */
    b.source_instance = i;
    b.name = mstring_to_std(body_names[i]);
    r_bodies.append(std::move(b));
  }
}

static void read_bodies_points(const PointCloud &points,
                               const float default_radius,
                               const float default_density,
                               Vector<BodyRead> &r_bodies)
{
  const int n = points.totpoint;
  r_bodies.reinitialize(n);
  const Span<float3> positions = points.positions();
  const bke::AttributeAccessor attributes = points.attributes();
  const bke::AttrDomain domain = bke::AttrDomain::Point;

  const VArray<int> body_types = get_int(attributes, attr::body_type, domain, 2);
  const VArray<int> shape_types = get_int(attributes, attr::shape_type, domain, 0);
  const VArray<float3> shape_sizes = get_float3(
      attributes, attr::shape_size, domain, float3(0.0f));
  const VArray<float> radii = get_float(attributes, attr::radius, domain, default_radius);
  const VArray<float> densities = get_float(attributes, attr::density, domain, default_density);
  const VArray<float> frictions = get_float(attributes, attr::friction, domain, 0.3f);
  const VArray<float> restitutions = get_float(attributes, attr::restitution, domain, 0.1f);
  const VArray<float3> velocities = get_float3(attributes, attr::velocity, domain, float3(0.0f));
  const VArray<float> ang_z = get_float(attributes, attr::angular_velocity, domain, 0.0f);
  const VArray<MStringProperty> body_names = get_mstring(attributes, attr::body_name, domain);

  for (const int i : IndexRange(n)) {
    BodyRead &b = r_bodies[i];
    b.position = positions[i];
    b.angular_velocity = float3(0.0f, 0.0f, ang_z[i]);
    b.name = mstring_to_std(body_names[i]);
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
                     default_radius,
                     default_density);
  }
}

static void read_bodies_mesh(const Mesh &mesh,
                             const float default_radius,
                             const float default_density,
                             Vector<BodyRead> &r_bodies)
{
  BodyRead b;
  const std::optional<Bounds<float3>> bounds = mesh.bounds_min_max();
  float3 center(0.0f);
  if (bounds) {
    center = bounds->center();
    b.position = center;
    b.shape_size = math::max(bounds->size() * 0.5f, float3(1e-5f));
  }
  else {
    b.shape_size = float3(default_radius);
  }
  b.shape_type = int(ShapeType::BoundingBox);
  b.body_type = 0; /* Static by default for bare mesh colliders. */
  b.density = default_density;
  b.friction = 0.3f;
  b.restitution = 0.1f;

  const bke::AttributeAccessor attributes = mesh.attributes();
  /* Optional overrides from point domain (first element). */
  if (mesh.verts_num > 0) {
    b.body_type = get_int(attributes, attr::body_type, bke::AttrDomain::Point, b.body_type)[0];
    b.shape_type = get_int(attributes, attr::shape_type, bke::AttrDomain::Point, b.shape_type)[0];
    const float3 ss = get_float3(
        attributes, attr::shape_size, bke::AttrDomain::Point, float3(0.0f))[0];
    if (math::length_squared(ss) > 1e-12f) {
      b.shape_size = ss;
    }
  }
  /* Mesh domain: verts are world-space; body origin is AABB center → store local verts. */
  auto to_local = [&](const float3 &p) { return p - center; };

  if (b.shape_type == int(ShapeType::ConvexHull)) {
    for (const float3 &p : mesh.vert_positions()) {
      b.local_mesh_verts.append(to_local(p));
    }
  }
  else if (b.shape_type == int(ShapeType::TriangleMesh)) {
    box_engine::fill_triangle_mesh_from_mesh(mesh, b.local_mesh_verts, b.mesh_tri_indices);
    for (float3 &p : b.local_mesh_verts) {
      p = to_local(p);
    }
    box_engine::fill_islands_from_mesh(mesh, b.island_verts);
    for (Vector<float3> &island : b.island_verts) {
      for (float3 &p : island) {
        p = to_local(p);
      }
    }
  }
  else if (b.shape_type == int(ShapeType::Compound)) {
    box_engine::fill_islands_from_mesh(mesh, b.island_verts);
    for (Vector<float3> &island : b.island_verts) {
      for (float3 &p : island) {
        p = to_local(p);
      }
    }
  }
  else if (ELEM(b.shape_type, int(ShapeType::BoundingBox), int(ShapeType::Box)) &&
           math::length(center) > 1e-4f)
  {
    /* Offset box when mesh is not already origin-centered. */
    for (const float3 &p : mesh.vert_positions()) {
      b.local_mesh_verts.append(to_local(p));
    }
  }
  b.graph_driven = (b.body_type != 2);
  r_bodies.append(std::move(b));
}

/** Leftover / Colliders mesh: real triangles, not a giant AABB that swallows the boxes. */
static void append_mesh_as_collider(const Mesh &mesh, Vector<BodyRead> &r_bodies)
{
  const int start = r_bodies.size();
  read_bodies_mesh(mesh, 0.25f, 1.0f, r_bodies);
  if (r_bodies.size() <= start) {
    return;
  }
  BodyRead &b = r_bodies.last();
  b.write_back = false;
  b.graph_driven = true;
  b.source_instance = -1;
  if (b.body_type == 2) {
    b.body_type = 0;
  }
  if (b.mesh_tri_indices.size() >= 3 && b.local_mesh_verts.size() >= 3) {
    b.shape_type = int(ShapeType::TriangleMesh);
    return;
  }
  /* No triangles stored (default BoundingBox path). Rebuild as triangle mesh / hull. */
  const float3 center = b.position;
  auto to_local = [&](const float3 &p) { return p - center; };
  b.local_mesh_verts.clear();
  b.mesh_tri_indices.clear();
  b.island_verts.clear();
  box_engine::fill_triangle_mesh_from_mesh(mesh, b.local_mesh_verts, b.mesh_tri_indices);
  for (float3 &p : b.local_mesh_verts) {
    p = to_local(p);
  }
  if (b.mesh_tri_indices.size() >= 3) {
    b.shape_type = int(ShapeType::TriangleMesh);
  }
  else {
    b.shape_type = int(ShapeType::ConvexHull);
    if (b.local_mesh_verts.is_empty()) {
      for (const float3 &p : mesh.vert_positions()) {
        b.local_mesh_verts.append(to_local(p));
      }
    }
  }
}

static void read_joints(const PointCloud &points, Vector<JointRead> &r_joints)
{
  const int n = points.totpoint;
  r_joints.reinitialize(n);
  const Span<float3> positions = points.positions();
  const bke::AttributeAccessor attributes = points.attributes();
  const bke::AttrDomain domain = bke::AttrDomain::Point;
  const VArray<int> a = get_int(attributes, attr::body_a, domain, 0);
  const VArray<int> b = get_int(attributes, attr::body_b, domain, -1);
  const VArray<MStringProperty> a_names = get_mstring(attributes, attr::body_a, domain);
  const VArray<MStringProperty> b_names = get_mstring(attributes, attr::body_b, domain);
  const VArray<int> t = get_int(attributes, attr::joint_type, domain, 1);
  const VArray<float> len = get_float(attributes, attr::length, domain, 0.0f);
  const VArray<float3> axes = get_float3(attributes, attr::axis, domain, float3(1.0f, 0.0f, 0.0f));
  for (const int i : IndexRange(n)) {
    JointRead &j = r_joints[i];
    j.body_a = a[i];
    j.body_b = b[i];
    j.name_a = mstring_to_std(a_names[i]);
    j.name_b = mstring_to_std(b_names[i]);
    j.joint_type = t[i];
    j.anchor = positions[i];
    j.axis = axes[i];
    j.length = len[i];
  }
}

static Map<std::string, int> build_body_name_map(const Span<BodyRead> bodies)
{
  Map<std::string, int> map;
  for (const int i : bodies.index_range()) {
    if (!bodies[i].name.empty()) {
      map.add_overwrite(bodies[i].name, i);
    }
  }
  return map;
}

/** Resolve constraint names onto body indices. Revolute always pins Body A to the world. */
static void resolve_joint_names(const Span<BodyRead> bodies, MutableSpan<JointRead> joints)
{
  const Map<std::string, int> names = build_body_name_map(bodies);
  if (names.is_empty()) {
    return;
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
    if (j.joint_type == int(JointType::Revolute)) {
      j.body_b = -1;
      continue;
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

static void write_point_results(PointCloud &points,
                                const Span<float3> positions,
                                const Span<float3> velocities,
                                const Span<float> ang_z)
{
  points.positions_for_write().copy_from(positions);
  bke::MutableAttributeAccessor attributes = points.attributes_for_write();
  bke::SpanAttributeWriter<float3> vel = attributes.lookup_or_add_for_write_only_span<float3>(
      attr::velocity, bke::AttrDomain::Point);
  bke::SpanAttributeWriter<float> ang = attributes.lookup_or_add_for_write_only_span<float>(
      attr::angular_velocity, bke::AttrDomain::Point);
  if (vel) {
    vel.span.copy_from(velocities);
    vel.finish();
  }
  if (ang) {
    ang.span.copy_from(ang_z);
    ang.finish();
  }
}

static void write_instance_results(bke::Instances &instances,
                                   const Span<BodyRead> bodies,
                                   const Span<float3> positions,
                                   const Span<math::Quaternion> rotations,
                                   const Span<float3> velocities,
                                   const Span<float3> ang_vel)
{
  MutableSpan<float4x4> transforms = instances.transforms_for_write();
  Array<float3> inst_vel(transforms.size(), float3(0.0f));
  Array<float3> inst_ang(transforms.size(), float3(0.0f));

  for (const int bi : bodies.index_range()) {
    const BodyRead &b = bodies[bi];
    if (!b.write_back) {
      continue;
    }
    if (!b.slave_instances.is_empty()) {
      const float4x4 body_tf = math::from_loc_rot<float4x4>(positions[bi], rotations[bi]);
      for (const int k : b.slave_instances.index_range()) {
        const int ii = b.slave_instances[k];
        if (ii < 0 || ii >= transforms.size()) {
          continue;
        }
        transforms[ii] = body_tf * b.slave_local_tf[k];
        inst_vel[ii] = velocities[bi];
        inst_ang[ii] = ang_vel[bi];
      }
      continue;
    }
    const int ii = (b.source_instance >= 0) ? b.source_instance : bi;
    if (ii < 0 || ii >= transforms.size()) {
      continue;
    }
    const float3 scale = math::to_scale(transforms[ii]);
    transforms[ii] = math::from_loc_rot_scale<float4x4>(positions[bi], rotations[bi], scale);
    inst_vel[ii] = velocities[bi];
    inst_ang[ii] = ang_vel[bi];
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
}

/** Identity of a body subset (not poses). Dynamics and static/kinematic hashed separately. */
static uint64_t hash_body_indices(const Span<BodyRead> bodies, const Span<int> idx)
{
  uint64_t h = 14695981039346656037ull;
  auto mix_u = [&](uint64_t v) {
    h ^= v;
    h *= 1099511628211ull;
  };
  auto mix_f = [&](float v) {
    uint32_t u = 0;
    std::memcpy(&u, &v, sizeof(u));
    mix_u(u);
  };
  mix_u(uint64_t(idx.size()));
  for (const int i : idx) {
    if (i < 0 || i >= bodies.size()) {
      mix_u(0xFFFFFFFFull);
      continue;
    }
    const BodyRead &b = bodies[i];
    mix_u(uint64_t(uint8_t(b.body_type)) | (uint64_t(uint8_t(b.shape_type)) << 8) |
          (uint64_t(b.write_back) << 16));
    mix_f(b.shape_size.x);
    mix_f(b.shape_size.y);
    mix_f(b.shape_size.z);
    mix_f(b.density);
    mix_f(b.friction);
    mix_f(b.restitution);
    mix_u(uint64_t(b.local_mesh_verts.size()));
    mix_u(uint64_t(b.mesh_tri_indices.size()));
    mix_u(uint64_t(b.island_verts.size()));
    mix_u(uint64_t(b.name.size()));
    for (const char c : b.name) {
      mix_u(uint8_t(c));
    }
  }
  return h;
}

static uint64_t joints_layout_hash(const Span<JointRead> joints)
{
  uint64_t h = 14695981039346656037ull;
  auto mix_u = [&](uint64_t v) {
    h ^= v;
    h *= 1099511628211ull;
  };
  mix_u(uint64_t(joints.size()));
  for (const JointRead &j : joints) {
    mix_u(uint64_t(uint8_t(j.joint_type)));
    mix_u(uint64_t(uint32_t(j.body_a)));
    mix_u(uint64_t(uint32_t(j.body_b)));
  }
  return h;
}

static void collect_dyn_obs(const Span<BodyRead> bodies, Vector<int> &dyn, Vector<int> &obs)
{
  dyn.clear();
  obs.clear();
  for (const int i : bodies.index_range()) {
    if (bodies[i].body_type == 2) {
      dyn.append(i);
    }
    else {
      obs.append(i);
    }
  }
}

#if defined(WITH_BOX2D)

static void decimate_hull_2d(Vector<float2> &hull, const int max_count)
{
  while (hull.size() > max_count && hull.size() > 3) {
    int best = 0;
    float best_area = std::numeric_limits<float>::max();
    const int n = hull.size();
    for (const int i : IndexRange(n)) {
      const float2 prev = hull[(i + n - 1) % n];
      const float2 cur = hull[i];
      const float2 next = hull[(i + 1) % n];
      const float area = math::abs(prev.x * (cur.y - next.y) + cur.x * (next.y - prev.y) +
                                   next.x * (prev.y - cur.y));
      if (area < best_area) {
        best_area = area;
        best = i;
      }
    }
    hull.remove(best);
  }
}

/** Convex polygon from any number of verts. Only falls back to the *input* AABB. */
static bool attach_poly_or_box_2d(const b2BodyId body,
                                  b2ShapeDef &shape_def,
                                  const Span<float3> verts,
                                  const float3 shape_size,
                                  const PlaneAxis plane,
                                  const bool allow_box_fallback)
{
  auto project = [&](const float3 &p) -> float2 {
    return (plane == PlaneAxis::XY) ? float2(p.x, p.y) : float2(p.x, p.z);
  };

  if (verts.size() >= 3) {
    Array<float2> pts2(verts.size());
    for (const int i : verts.index_range()) {
      pts2[i] = project(verts[i]);
    }
    Array<int> hull_idx(verts.size());
    const int hcount = BLI_convexhull_2d(pts2, hull_idx.data());
    if (hcount >= 3) {
      Vector<float2> hull;
      hull.reserve(hcount);
      for (const int i : IndexRange(hcount)) {
        hull.append(pts2[hull_idx[i]]);
      }
      if (hull.size() > B2_MAX_POLYGON_VERTICES) {
        decimate_hull_2d(hull, B2_MAX_POLYGON_VERTICES);
      }
      Array<b2Vec2> b2pts(hull.size());
      for (const int i : hull.index_range()) {
        b2pts[i] = b2Vec2{hull[i].x, hull[i].y};
      }
      const b2Hull b2hull = b2ComputeHull(b2pts.data(), int(b2pts.size()));
      if (b2hull.count >= 3) {
        const b2Polygon poly = b2MakePolygon(&b2hull, 0.0f);
        b2CreatePolygonShape(body, &shape_def, &poly);
        return true;
      }
    }
  }

  if (!allow_box_fallback) {
    return false;
  }

  float hx = math::max(shape_size.x, 1e-5f);
  float hy = math::max((plane == PlaneAxis::XY) ? shape_size.y : shape_size.z, 1e-5f);
  b2Vec2 local_center = {0.0f, 0.0f};
  if (!verts.is_empty()) {
    if (const std::optional<Bounds<float3>> bounds = bounds::min_max(verts)) {
      const float2 c = project(bounds->center());
      local_center = {c.x, c.y};
      const float3 half = math::max(bounds->size() * 0.5f, float3(1e-5f));
      hx = math::max(half.x, 1e-5f);
      hy = math::max((plane == PlaneAxis::XY) ? half.y : half.z, 1e-5f);
    }
  }
  const b2Polygon box = b2MakeOffsetBox(hx, hy, local_center, b2Rot_identity);
  b2CreatePolygonShape(body, &shape_def, &box);
  return true;
}

static void attach_shape_2d(const b2BodyId body, const BodyRead &src, const PlaneAxis plane)
{
  b2ShapeDef shape_def = b2DefaultShapeDef();
  shape_def.density = (src.body_type == 2) ? math::max(src.density, 1e-3f) : 0.0f;
  shape_def.material.friction = src.friction;
  shape_def.material.restitution = src.restitution;

  if (src.shape_type == int(ShapeType::Compound)) {
    for (const Vector<float3> &island : src.island_verts) {
      attach_poly_or_box_2d(body, shape_def, island, src.shape_size, plane, true);
    }
    return;
  }

  /* Triangle mesh: one convex polygon per triangle. */
  if (src.shape_type == int(ShapeType::TriangleMesh) && src.mesh_tri_indices.size() >= 3 &&
      src.local_mesh_verts.size() >= 3)
  {
    bool any = false;
    float3 tri[3];
    for (int t = 0; t + 2 < src.mesh_tri_indices.size(); t += 3) {
      const int i0 = src.mesh_tri_indices[t];
      const int i1 = src.mesh_tri_indices[t + 1];
      const int i2 = src.mesh_tri_indices[t + 2];
      if (!src.local_mesh_verts.index_range().contains(i0) ||
          !src.local_mesh_verts.index_range().contains(i1) ||
          !src.local_mesh_verts.index_range().contains(i2))
      {
        continue;
      }
      tri[0] = src.local_mesh_verts[i0];
      tri[1] = src.local_mesh_verts[i1];
      tri[2] = src.local_mesh_verts[i2];
      any |= attach_poly_or_box_2d(body, shape_def, Span<float3>(tri, 3), src.shape_size, plane, false);
    }
    if (any) {
      return;
    }
    if (!src.island_verts.is_empty()) {
      for (const Vector<float3> &island : src.island_verts) {
        attach_poly_or_box_2d(body, shape_def, island, src.shape_size, plane, true);
      }
      return;
    }
  }

  if (ELEM(src.shape_type, int(ShapeType::ConvexHull), int(ShapeType::TriangleMesh)) &&
      !src.local_mesh_verts.is_empty())
  {
    attach_poly_or_box_2d(body, shape_def, src.local_mesh_verts, src.shape_size, plane, true);
    return;
  }

  if (ELEM(src.shape_type, int(ShapeType::Box), int(ShapeType::BoundingBox))) {
    attach_poly_or_box_2d(body, shape_def, src.local_mesh_verts, src.shape_size, plane, true);
  }
  else if (src.shape_type == int(ShapeType::Capsule)) {
    b2Capsule cap{};
    const float r = math::max(src.shape_size.x, 1e-5f);
    const float h = math::max(src.shape_size.y, 0.0f);
    cap.center1 = {0.0f, -h};
    cap.center2 = {0.0f, h};
    cap.radius = r;
    b2CreateCapsuleShape(body, &shape_def, &cap);
  }
  else {
    b2Circle circle{};
    circle.center = {0.0f, 0.0f};
    circle.radius = math::max(src.shape_size.x, 1e-5f);
    b2CreateCircleShape(body, &shape_def, &circle);
  }
}

static float plane_angle_of(const math::Quaternion &q, const PlaneAxis plane)
{
  const math::EulerXYZ e = math::to_euler(q);
  return (plane == PlaneAxis::XY) ? float(e.z()) : float(e.y());
}

static float plane_omega_of(const float3 &w, const PlaneAxis plane)
{
  return (plane == PlaneAxis::XY) ? w.z : w.y;
}

static b2WorldTransform world_tf_2d(const float3 &p,
                                    const math::Quaternion &q,
                                    const PlaneAxis plane)
{
  b2WorldTransform xf{};
  xf.p = (plane == PlaneAxis::XY) ? b2Pos{p.x, p.y} : b2Pos{p.x, p.z};
  xf.q = b2MakeRot(plane_angle_of(q, plane));
  return xf;
}

static b2BodyId create_one_body_2d(const b2WorldId world,
                                   const BodyRead &src,
                                   const PlaneAxis plane)
{
  b2BodyDef def = b2DefaultBodyDef();
  def.type = (src.body_type == 0) ? b2_staticBody :
             (src.body_type == 1) ? b2_kinematicBody :
                                    b2_dynamicBody;
  def.isBullet = (def.type == b2_dynamicBody);
  def.enableSleep = (def.type == b2_dynamicBody);
  def.isAwake = true;
  if (plane == PlaneAxis::XY) {
    def.position = {src.position.x, src.position.y};
    def.linearVelocity = {src.velocity.x, src.velocity.y};
  }
  else {
    def.position = {src.position.x, src.position.z};
    def.linearVelocity = {src.velocity.x, src.velocity.z};
  }
  def.rotation = b2MakeRot(plane_angle_of(src.rotation, plane));
  def.angularVelocity = plane_omega_of(src.angular_velocity, plane);
  const b2BodyId id = b2CreateBody(world, &def);
  if (b2Body_IsValid(id)) {
    attach_shape_2d(id, src, plane);
  }
  return id;
}

/** Sweep kinematic movers. Static ground stays static so stacks can sleep.
 * Static that actually moved is recreated in place (no sweep) — rain indices change. */
static void drive_or_snap_2d(b2BodyId &id,
                             const b2WorldId world,
                             const BodyRead &src,
                             const PlaneAxis plane,
                             const float h)
{
  if (!b2Body_IsValid(id) || src.body_type == 2) {
    return;
  }
  const b2WorldTransform target = world_tf_2d(src.position, src.rotation, plane);
  if (src.body_type == 1) {
    if (b2Body_GetType(id) != b2_kinematicBody) {
      b2Body_SetType(id, b2_kinematicBody);
    }
    b2Body_SetTargetTransform(id, target, h, true);
    return;
  }
  const b2Pos pp = b2Body_GetPosition(id);
  const float dx = float(target.p.x - pp.x);
  const float dy = float(target.p.y - pp.y);
  const float dang = b2RelativeAngle(b2Body_GetRotation(id), target.q);
  constexpr float pos_eps2 = 1.0e-6f; /* 1 mm */
  constexpr float ang_eps = 0.001f;
  if (dx * dx + dy * dy <= pos_eps2 && math::abs(dang) <= ang_eps) {
    if (b2Body_GetType(id) != b2_staticBody) {
      b2Body_SetType(id, b2_staticBody);
    }
    return;
  }
  /* Large jump (recycled rain slot): spawn at the new pose, do not sweep the world. */
  b2DestroyBody(id);
  id = create_one_body_2d(world, src, plane);
}

static void destroy_ids_2d(Vector<b2BodyId> &ids)
{
  for (const b2BodyId id : ids) {
    if (b2Body_IsValid(id)) {
      b2DestroyBody(id);
    }
  }
  ids.clear();
}

struct Persist2D {
  b2WorldId world = b2_nullWorldId;
  Vector<b2BodyId> dyn_ids;
  Vector<b2BodyId> obs_ids;
  uint64_t dyn_hash = 0;
  uint64_t obs_hash = 0;
};

static Map<std::string, Persist2D> &persist_2d_map()
{
  static Map<std::string, Persist2D> map;
  return map;
}

static void persist_2d_reset(Persist2D &p)
{
  if (b2World_IsValid(p.world)) {
    b2DestroyWorld(p.world);
  }
  p.world = b2_nullWorldId;
  p.dyn_ids.clear();
  p.obs_ids.clear();
  p.dyn_hash = 0;
  p.obs_hash = 0;
}

static void assemble_body_ids_2d(const Span<BodyRead> bodies,
                                 const Persist2D &p,
                                 MutableSpan<b2BodyId> out)
{
  int di = 0;
  int oi = 0;
  for (const int i : bodies.index_range()) {
    if (bodies[i].body_type == 2) {
      out[i] = (di < p.dyn_ids.size()) ? p.dyn_ids[di++] : b2_nullBodyId;
    }
    else {
      out[i] = (oi < p.obs_ids.size()) ? p.obs_ids[oi++] : b2_nullBodyId;
    }
  }
}

static bool step_box2d(const Span<BodyRead> bodies,
                       const Span<JointRead> joints,
                       const float3 gravity,
                       const float dt,
                       const int substeps,
                       const PlaneAxis plane,
                       const StringRef cache_ns,
                       MutableSpan<float3> r_positions,
                       MutableSpan<float3> r_velocities,
                       MutableSpan<float> r_ang_z,
                       MutableSpan<math::Quaternion> r_rotations,
                       std::string &r_error)
{
  if (bodies.is_empty()) {
    r_error = "No bodies";
    return false;
  }
  if (dt <= 0.0f) {
    std::lock_guard<std::mutex> lock(deformable_solver::box2d_engine_mutex());
    persist_2d_reset(persist_2d_map().lookup_or_add_default(std::string(cache_ns)));
    for (const int i : bodies.index_range()) {
      r_positions[i] = bodies[i].position;
      r_velocities[i] = bodies[i].velocity;
      r_ang_z[i] = plane_omega_of(bodies[i].angular_velocity, plane);
      r_rotations[i] = bodies[i].rotation;
    }
    return true;
  }

  /* b2_worlds[] is process-global; depsgraph may step several hosts at once. */
  std::lock_guard<std::mutex> lock(deformable_solver::box2d_engine_mutex());
  const float h = math::min(dt, 0.05f);
  const int slices = math::clamp(substeps, 1, 64);

  b2Vec2 g2;
  if (plane == PlaneAxis::XY) {
    const float g_y = (math::abs(gravity.y) > 1e-8f) ? gravity.y : gravity.z;
    g2 = b2Vec2{gravity.x, g_y};
  }
  else {
    const float g_down = (math::abs(gravity.z) > 1e-8f) ? gravity.z : gravity.y;
    g2 = b2Vec2{gravity.x, g_down};
  }

  const std::string ns(cache_ns);
  Persist2D &persist = persist_2d_map().lookup_or_add_default(ns);
  Vector<int> dyn_idx;
  Vector<int> obs_idx;
  collect_dyn_obs(bodies, dyn_idx, obs_idx);
  const uint64_t dyn_hash = hash_body_indices(bodies, dyn_idx) ^ joints_layout_hash(joints);
  const uint64_t obs_hash = hash_body_indices(bodies, obs_idx);

  bool reuse_dyn = b2World_IsValid(persist.world) && persist.dyn_hash == dyn_hash &&
                   persist.dyn_ids.size() == dyn_idx.size();
  if (reuse_dyn) {
    for (const int k : dyn_idx.index_range()) {
      if (!b2Body_IsValid(persist.dyn_ids[k])) {
        reuse_dyn = false;
        break;
      }
      const int i = dyn_idx[k];
      const b2Pos pp = b2Body_GetPosition(persist.dyn_ids[k]);
      const float3 gn = bodies[i].position;
      const float3 phys = (plane == PlaneAxis::XY) ? float3(float(pp.x), float(pp.y), gn.z) :
                                                     float3(float(pp.x), gn.y, float(pp.y));
      if (math::distance_squared(phys, gn) > 4.0f) {
        reuse_dyn = false;
        break;
      }
    }
  }

  b2WorldId world = persist.world;
  bool reuse_obs = false;
  if (!reuse_dyn) {
    persist_2d_reset(persist);
    b2WorldDef world_def = b2DefaultWorldDef();
    world_def.workerCount = 1;
    world_def.enableContinuous = true;
    world_def.gravity = g2;
    world = b2CreateWorld(&world_def);
    if (!b2World_IsValid(world)) {
      r_error = "Box2D CreateWorld failed";
      return false;
    }
    persist.world = world;
    persist.dyn_ids.reserve(dyn_idx.size());
    persist.obs_ids.reserve(obs_idx.size());
    for (const int i : bodies.index_range()) {
      const b2BodyId id = create_one_body_2d(world, bodies[i], plane);
      if (bodies[i].body_type == 2) {
        persist.dyn_ids.append(id);
      }
      else {
        persist.obs_ids.append(id);
      }
    }
    persist.dyn_hash = dyn_hash;
    persist.obs_hash = obs_hash;
    reuse_obs = true;
  }
  else {
    world = persist.world;
    b2World_SetGravity(world, g2);
    reuse_obs = persist.obs_hash == obs_hash && persist.obs_ids.size() == obs_idx.size();
    if (reuse_obs) {
      for (const b2BodyId id : persist.obs_ids) {
        if (!b2Body_IsValid(id)) {
          reuse_obs = false;
          break;
        }
      }
    }
    if (reuse_obs) {
      for (const int k : obs_idx.index_range()) {
        drive_or_snap_2d(persist.obs_ids[k], world, bodies[obs_idx[k]], plane, h);
      }
    }
    else {
      destroy_ids_2d(persist.obs_ids);
      persist.obs_ids.reserve(obs_idx.size());
      for (const int i : obs_idx) {
        persist.obs_ids.append(create_one_body_2d(world, bodies[i], plane));
      }
      persist.obs_hash = obs_hash;
    }
    for (const int k : dyn_idx.index_range()) {
      drive_or_snap_2d(persist.dyn_ids[k], world, bodies[dyn_idx[k]], plane, h);
    }
  }

  Array<b2BodyId> body_ids(bodies.size());
  assemble_body_ids_2d(bodies, persist, body_ids);
  const bool create_all_joints = !reuse_dyn;
  const bool create_obs_joints = reuse_dyn && !reuse_obs;
  if (create_all_joints || create_obs_joints) {
  /* World body for Revolute (single body + world Anchor). */
  b2BodyId world_body = b2_nullBodyId;
  auto ensure_world_body = [&]() {
    if (b2Body_IsValid(world_body)) {
      return world_body;
    }
    b2BodyDef def = b2DefaultBodyDef();
    def.type = b2_staticBody;
    def.position = {0.0f, 0.0f};
    world_body = b2CreateBody(world, &def);
    return world_body;
  };

  auto to_b2_pos = [&](const float3 &p) -> b2Pos {
    return (plane == PlaneAxis::XY) ? b2Pos{p.x, p.y} : b2Pos{p.x, p.z};
  };
  auto to_b2_axis = [&](const float3 &v) -> b2Vec2 {
    b2Vec2 a = (plane == PlaneAxis::XY) ? b2Vec2{v.x, v.y} : b2Vec2{v.x, v.z};
    if (b2LengthSquared(a) < 1e-12f) {
      a = {1.0f, 0.0f};
    }
    return b2Normalize(a);
  };

  for (const JointRead &j : joints) {
    if (j.body_a < 0 || j.body_a >= bodies.size()) {
      continue;
    }
    const bool is_revolute = (j.joint_type == int(JointType::Revolute));
    if (!is_revolute && (j.body_b < 0 || j.body_b >= bodies.size() || j.body_b == j.body_a)) {
      continue;
    }
    if (create_obs_joints) {
      const bool a_obs = bodies[j.body_a].body_type != 2;
      const bool b_obs = !is_revolute && bodies[j.body_b].body_type != 2;
      if (!a_obs && !b_obs) {
        continue;
      }
    }
    b2BodyId bodyA = body_ids[j.body_a];
    b2BodyId bodyB = is_revolute ? ensure_world_body() : body_ids[j.body_b];
    if (!b2Body_IsValid(bodyA) || !b2Body_IsValid(bodyB)) {
      continue;
    }

    auto fill_base = [&](b2JointDef &base, const b2Vec2 localA, const b2Vec2 localB, const b2Rot qA, const b2Rot qB) {
      base.bodyIdA = bodyA;
      base.bodyIdB = bodyB;
      base.localFrameA.p = localA;
      base.localFrameA.q = qA;
      base.localFrameB.p = localB;
      base.localFrameB.q = qB;
      base.collideConnected = false;
    };

    if (j.joint_type == int(JointType::Distance)) {
      b2DistanceJointDef def = b2DefaultDistanceJointDef();
      const b2Pos pa = b2Body_GetPosition(bodyA);
      const b2Pos pb = b2Body_GetPosition(bodyB);
      const float dx = float(pb.x - pa.x);
      const float dy = float(pb.y - pa.y);
      def.length = math::max(math::sqrt(dx * dx + dy * dy), 1e-4f);
      fill_base(def.base, {0.0f, 0.0f}, {0.0f, 0.0f}, b2Rot_identity, b2Rot_identity);
      b2CreateDistanceJoint(world, &def);
    }
    else if (j.joint_type == int(JointType::Weld)) {
      b2WeldJointDef def = b2DefaultWeldJointDef();
      const b2Pos pa = b2Body_GetPosition(bodyA);
      const b2Pos pb = b2Body_GetPosition(bodyB);
      const b2Pos mid = {0.5f * (pa.x + pb.x), 0.5f * (pa.y + pb.y)};
      fill_base(def.base,
                b2Body_GetLocalPoint(bodyA, mid),
                b2Body_GetLocalPoint(bodyB, mid),
                b2InvertRot(b2Body_GetRotation(bodyA)),
                b2InvertRot(b2Body_GetRotation(bodyB)));
      b2CreateWeldJoint(world, &def);
    }
    else if (j.joint_type == int(JointType::Prismatic)) {
      b2PrismaticJointDef def = b2DefaultPrismaticJointDef();
      const b2Vec2 axis = to_b2_axis(j.axis);
      const b2Vec2 axis_a = b2Body_GetLocalVector(bodyA, axis);
      const b2Vec2 axis_b = b2Body_GetLocalVector(bodyB, axis);
      const b2Rot qA = (b2LengthSquared(axis_a) > 1e-12f) ?
                           b2MakeRotFromUnitVector(b2Normalize(axis_a)) :
                           b2Rot_identity;
      const b2Rot qB = (b2LengthSquared(axis_b) > 1e-12f) ?
                           b2MakeRotFromUnitVector(b2Normalize(axis_b)) :
                           b2Rot_identity;
      const b2Pos pa = b2Body_GetPosition(bodyA);
      fill_base(def.base, {0.0f, 0.0f}, b2Body_GetLocalPoint(bodyB, pa), qA, qB);
      b2CreatePrismaticJoint(world, &def);
    }
    else {
      b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
      const b2Pos anchor = to_b2_pos(j.anchor);
      fill_base(def.base,
                b2Body_GetLocalPoint(bodyA, anchor),
                b2Body_GetLocalPoint(bodyB, anchor),
                b2Rot_identity,
                b2Rot_identity);
      b2CreateRevoluteJoint(world, &def);
    }
  }
  } /* create joints */

  /* Substeps = collide/solve slices this frame (dt/N each). Inner 4 is Box2D's solver. */
  const float step_h = h / float(slices);
  for (int s = 0; s < slices; s++) {
    b2World_Step(world, step_h, 4);
  }

  for (const int i : bodies.index_range()) {
    if (i >= body_ids.size() || !b2Body_IsValid(body_ids[i])) {
      r_positions[i] = bodies[i].position;
      r_velocities[i] = bodies[i].velocity;
      r_ang_z[i] = plane_omega_of(bodies[i].angular_velocity, plane);
      r_rotations[i] = bodies[i].rotation;
      continue;
    }
    const b2BodyId body = body_ids[i];
    const b2Pos pos = b2Body_GetPosition(body);
    const b2Vec2 vel = b2Body_GetLinearVelocity(body);
    const float ang = b2Body_GetAngularVelocity(body);
    const float angle = b2Rot_GetAngle(b2Body_GetRotation(body));
    const BodyRead &src = bodies[i];
    if (src.graph_driven || src.body_type != 2) {
      /* Graph owns the pose (turntable/hands). Keep the visual in sync; keep sweep velocity. */
      r_positions[i] = src.position;
      r_rotations[i] = src.rotation;
      if (plane == PlaneAxis::XY) {
        r_velocities[i] = float3(vel.x, vel.y, 0.0f);
      }
      else {
        r_velocities[i] = float3(vel.x, 0.0f, vel.y);
      }
      r_ang_z[i] = ang;
      continue;
    }
    if (plane == PlaneAxis::XY) {
      r_positions[i] = float3(float(pos.x), float(pos.y), src.position.z);
      r_velocities[i] = float3(vel.x, vel.y, 0.0f);
    }
    else {
      r_positions[i] = float3(float(pos.x), src.position.y, float(pos.y));
      r_velocities[i] = float3(vel.x, 0.0f, vel.y);
    }
    r_ang_z[i] = ang;
    if (plane == PlaneAxis::XY) {
      r_rotations[i] = math::to_quaternion(math::EulerXYZ(0.0f, 0.0f, angle));
    }
    else {
      r_rotations[i] = math::to_quaternion(math::EulerXYZ(0.0f, angle, 0.0f));
    }
  }

  /* Keep the world: contact warm-starting is what makes stacks settle. */
  return true;
}

#endif /* WITH_BOX2D */

#if defined(WITH_BOX3D)

#ifndef B3_MAX_HULL_VERTICES
#  define B3_MAX_HULL_VERTICES 128
#endif

static void subsample_points(const Span<float3> src, const int max_count, Vector<float3> &dst)
{
  dst.clear();
  if (src.size() <= max_count) {
    dst.extend(src);
    return;
  }

  if (const std::optional<Bounds<float3>> bounds = bounds::min_max(src)) {
    const float3 ext = bounds->size();
    const int axis = (ext.x >= ext.y && ext.x >= ext.z) ? 0 : (ext.y >= ext.z) ? 1 : 2;
    float min_v = src[0][axis];
    float max_v = min_v;
    int min_i = 0;
    int max_i = 0;
    for (const int i : src.index_range()) {
      if (src[i][axis] < min_v) {
        min_v = src[i][axis];
        min_i = i;
      }
      if (src[i][axis] > max_v) {
        max_v = src[i][axis];
        max_i = i;
      }
    }
    dst.append(src[min_i]);
    if (max_i != min_i) {
      dst.append(src[max_i]);
    }
  }
  else {
    dst.append(src[0]);
  }

  Array<float> min_dist(src.size(), std::numeric_limits<float>::max());
  auto refresh = [&](const float3 &p) {
    for (const int i : src.index_range()) {
      min_dist[i] = math::min(min_dist[i], math::distance_squared(src[i], p));
    }
  };
  for (const float3 &p : dst) {
    refresh(p);
  }
  while (dst.size() < max_count) {
    int best = 0;
    float best_d = -1.0f;
    for (const int i : src.index_range()) {
      if (min_dist[i] > best_d) {
        best_d = min_dist[i];
        best = i;
      }
    }
    if (best_d <= 1e-16f) {
      break;
    }
    dst.append(src[best]);
    refresh(src[best]);
  }
}

/** Convex hull of `verts`. Fallback is the AABB of those verts — never the body-wide box. */
static void attach_hull_3d(const b3BodyId body,
                           b3ShapeDef &shape_def,
                           const Span<float3> verts,
                           const float3 shape_size)
{
  Vector<float3> sampled;
  const Span<float3> use = [&]() -> Span<float3> {
    if (verts.size() > B3_MAX_HULL_VERTICES) {
      subsample_points(verts, B3_MAX_HULL_VERTICES, sampled);
      return sampled;
    }
    return verts;
  }();

  if (use.size() >= 4) {
    Array<b3Vec3> pts(use.size());
    for (const int i : use.index_range()) {
      pts[i] = {use[i].x, use[i].y, use[i].z};
    }
    b3HullData *hull = b3CreateHull(pts.data(), int(pts.size()), int(pts.size()));
    if (hull) {
      b3CreateHullShape(body, &shape_def, hull);
      b3DestroyHull(hull);
      return;
    }
  }

  float3 half = math::max(shape_size, float3(1e-5f));
  float3 center(0.0f);
  if (!verts.is_empty()) {
    if (const std::optional<Bounds<float3>> bounds = bounds::min_max(verts)) {
      center = bounds->center();
      half = math::max(bounds->size() * 0.5f, float3(1e-5f));
    }
  }
  b3BoxHull box = b3MakeOffsetBoxHull(
      math::max(half.x, 1e-5f),
      math::max(half.y, 1e-5f),
      math::max(half.z, 1e-5f),
      {center.x, center.y, center.z});
  b3CreateHullShape(body, &shape_def, &box.base);
}

static bool attach_triangle_mesh_3d(const b3BodyId body,
                                    b3ShapeDef &shape_def,
                                    const BodyRead &src,
                                    Vector<b3MeshData *> &r_owned_meshes)
{
  Array<b3Vec3> verts(src.local_mesh_verts.size());
  for (const int i : src.local_mesh_verts.index_range()) {
    verts[i] = {src.local_mesh_verts[i].x, src.local_mesh_verts[i].y, src.local_mesh_verts[i].z};
  }
  Array<int32_t> indices(src.mesh_tri_indices.size());
  for (const int i : src.mesh_tri_indices.index_range()) {
    indices[i] = int32_t(src.mesh_tri_indices[i]);
  }
  b3MeshDef def{};
  def.vertices = verts.data();
  def.indices = indices.data();
  def.materialIndices = nullptr;
  /* Don't weld: small / high-detail meshes were collapsing into nothing. */
  def.weldTolerance = 0.0f;
  def.vertexCount = int(verts.size());
  def.triangleCount = int(indices.size() / 3);
  def.weldVertices = false;
  def.useMedianSplit = false;
  def.identifyEdges = true;
  b3MeshData *mesh = b3CreateMesh(&def, nullptr, 0);
  if (!mesh) {
    return false;
  }
  const b3ShapeId shape_id = b3CreateMeshShape(body, &shape_def, mesh, {1.0f, 1.0f, 1.0f});
  if (b3Shape_IsValid(shape_id)) {
    r_owned_meshes.append(mesh);
    return true;
  }
  b3DestroyMesh(mesh);
  return false;
}

static void attach_shape_3d(const b3BodyId body,
                            const BodyRead &src,
                            Vector<b3MeshData *> &r_owned_meshes)
{
  b3ShapeDef shape_def = b3DefaultShapeDef();
  shape_def.density = (src.body_type == 2) ? math::max(src.density, 1e-3f) : 0.0f;
  shape_def.baseMaterial.friction = src.friction;
  shape_def.baseMaterial.restitution = src.restitution;

  if (src.shape_type == int(ShapeType::Compound) && !src.island_verts.is_empty()) {
    for (const Vector<float3> &island : src.island_verts) {
      attach_hull_3d(body, shape_def, island, src.shape_size);
    }
    return;
  }

  /* Triangle mesh: Box3D only generates mesh contacts on static bodies. */
  if (src.shape_type == int(ShapeType::TriangleMesh) && src.mesh_tri_indices.size() >= 3 &&
      src.local_mesh_verts.size() >= 3)
  {
    if (src.body_type == 0) {
      if (attach_triangle_mesh_3d(body, shape_def, src, r_owned_meshes)) {
        return;
      }
    }
    if (!src.island_verts.is_empty()) {
      for (const Vector<float3> &island : src.island_verts) {
        attach_hull_3d(body, shape_def, island, src.shape_size);
      }
      return;
    }
    if (!src.local_mesh_verts.is_empty()) {
      attach_hull_3d(body, shape_def, src.local_mesh_verts, src.shape_size);
      return;
    }
  }

  if (src.shape_type == int(ShapeType::Compound)) {
    /* Never collapse Compound to one overall hull. */
    return;
  }

  if (src.shape_type == int(ShapeType::ConvexHull)) {
    if (!src.local_mesh_verts.is_empty()) {
      attach_hull_3d(body, shape_def, src.local_mesh_verts, src.shape_size);
      return;
    }
  }

  if (ELEM(src.shape_type, int(ShapeType::Box), int(ShapeType::BoundingBox))) {
    attach_hull_3d(body, shape_def, src.local_mesh_verts, src.shape_size);
  }
  else if (src.shape_type == int(ShapeType::Capsule)) {
    b3Capsule cap{};
    const float r = math::max(src.shape_size.x, 1e-5f);
    const float h = math::max(src.shape_size.y, 0.0f);
    cap.center1 = {0.0f, -h, 0.0f};
    cap.center2 = {0.0f, h, 0.0f};
    cap.radius = r;
    b3CreateCapsuleShape(body, &shape_def, &cap);
  }
  else {
    b3Sphere sphere{};
    sphere.center = {0.0f, 0.0f, 0.0f};
    sphere.radius = math::max(src.shape_size.x, 1e-5f);
    b3CreateSphereShape(body, &shape_def, &sphere);
  }
}

static b3WorldTransform world_tf_3d(const BodyRead &src)
{
  b3WorldTransform xf{};
  xf.p = {src.position.x, src.position.y, src.position.z};
  xf.q = {{src.rotation.x, src.rotation.y, src.rotation.z}, src.rotation.w};
  return xf;
}

static b3BodyId create_one_body_3d(const b3WorldId world,
                                   const BodyRead &src,
                                   Vector<b3MeshData *> &r_meshes)
{
  b3BodyDef def = b3DefaultBodyDef();
  def.type = (src.body_type == 0) ? b3_staticBody :
             (src.body_type == 1) ? b3_kinematicBody :
                                    b3_dynamicBody;
  def.isBullet = (def.type == b3_dynamicBody);
  def.enableSleep = (def.type == b3_dynamicBody);
  def.isAwake = true;
  def.position = {src.position.x, src.position.y, src.position.z};
  def.linearVelocity = {src.velocity.x, src.velocity.y, src.velocity.z};
  def.angularVelocity = {src.angular_velocity.x, src.angular_velocity.y, src.angular_velocity.z};
  def.rotation = {{src.rotation.x, src.rotation.y, src.rotation.z}, src.rotation.w};
  const b3BodyId id = b3CreateBody(world, &def);
  if (b3Body_IsValid(id)) {
    attach_shape_3d(id, src, r_meshes);
  }
  return id;
}

static void destroy_meshes_3d(Vector<b3MeshData *> &meshes)
{
  for (b3MeshData *mesh : meshes) {
    if (mesh) {
      b3DestroyMesh(mesh);
    }
  }
  meshes.clear();
}

static void drive_or_snap_3d(b3BodyId &id,
                             const b3WorldId world,
                             const BodyRead &src,
                             Vector<b3MeshData *> &r_meshes,
                             const float h)
{
  if (!b3Body_IsValid(id) || src.body_type == 2) {
    return;
  }
  const b3WorldTransform target = world_tf_3d(src);
  if (src.body_type == 1) {
    if (b3Body_GetType(id) != b3_kinematicBody) {
      b3Body_SetType(id, b3_kinematicBody);
    }
    b3Body_SetTargetTransform(id, target, h, true);
    return;
  }
  const b3Pos pp = b3Body_GetPosition(id);
  const float3 d(float(target.p.x - pp.x), float(target.p.y - pp.y), float(target.p.z - pp.z));
  const b3Quat q = b3Body_GetRotation(id);
  const float qdot = q.s * target.q.s + q.v.x * target.q.v.x + q.v.y * target.q.v.y +
                     q.v.z * target.q.v.z;
  constexpr float pos_eps2 = 1.0e-6f;
  if (math::length_squared(d) <= pos_eps2 && (1.0f - math::abs(qdot)) <= 5.0e-7f) {
    if (b3Body_GetType(id) != b3_staticBody) {
      b3Body_SetType(id, b3_staticBody);
    }
    return;
  }
  b3DestroyBody(id);
  id = create_one_body_3d(world, src, r_meshes);
}

static void destroy_ids_3d(Vector<b3BodyId> &ids)
{
  for (const b3BodyId id : ids) {
    if (b3Body_IsValid(id)) {
      b3DestroyBody(id);
    }
  }
  ids.clear();
}

struct Persist3D {
  b3WorldId world = b3_nullWorldId;
  Vector<b3BodyId> dyn_ids;
  Vector<b3BodyId> obs_ids;
  Vector<b3MeshData *> dyn_meshes;
  Vector<b3MeshData *> obs_meshes;
  uint64_t dyn_hash = 0;
  uint64_t obs_hash = 0;
};

static Map<std::string, Persist3D> &persist_3d_map()
{
  static Map<std::string, Persist3D> map;
  return map;
}

static void persist_3d_reset(Persist3D &p)
{
  if (b3World_IsValid(p.world)) {
    b3DestroyWorld(p.world);
  }
  p.world = b3_nullWorldId;
  p.dyn_ids.clear();
  p.obs_ids.clear();
  destroy_meshes_3d(p.dyn_meshes);
  destroy_meshes_3d(p.obs_meshes);
  p.dyn_hash = 0;
  p.obs_hash = 0;
}

static void assemble_body_ids_3d(const Span<BodyRead> bodies,
                                 const Persist3D &p,
                                 MutableSpan<b3BodyId> out)
{
  int di = 0;
  int oi = 0;
  for (const int i : bodies.index_range()) {
    if (bodies[i].body_type == 2) {
      out[i] = (di < p.dyn_ids.size()) ? p.dyn_ids[di++] : b3_nullBodyId;
    }
    else {
      out[i] = (oi < p.obs_ids.size()) ? p.obs_ids[oi++] : b3_nullBodyId;
    }
  }
}

static bool step_box3d(const Span<BodyRead> bodies,
                       const Span<JointRead> joints,
                       const float3 gravity,
                       const float dt,
                       const int substeps,
                       const StringRef cache_ns,
                       MutableSpan<float3> r_positions,
                       MutableSpan<float3> r_velocities,
                       MutableSpan<float3> r_ang,
                       MutableSpan<math::Quaternion> r_rotations,
                       std::string &r_error)
{
  if (bodies.is_empty()) {
    r_error = "No bodies";
    return false;
  }
  if (dt <= 0.0f) {
    std::lock_guard<std::mutex> lock(deformable_solver::box3d_engine_mutex());
    persist_3d_reset(persist_3d_map().lookup_or_add_default(std::string(cache_ns)));
    for (const int i : bodies.index_range()) {
      r_positions[i] = bodies[i].position;
      r_velocities[i] = bodies[i].velocity;
      r_ang[i] = bodies[i].angular_velocity;
      r_rotations[i] = bodies[i].rotation;
    }
    return true;
  }

  /* b3_worlds[] is process-global; same race as Box2D under parallel GN eval. */
  std::lock_guard<std::mutex> lock(deformable_solver::box3d_engine_mutex());
  const float h = math::min(dt, 0.05f);
  const int slices = math::clamp(substeps, 1, 64);
  const b3Vec3 g3 = {gravity.x, gravity.y, gravity.z};

  const std::string ns(cache_ns);
  Persist3D &persist = persist_3d_map().lookup_or_add_default(ns);
  Vector<int> dyn_idx;
  Vector<int> obs_idx;
  collect_dyn_obs(bodies, dyn_idx, obs_idx);
  const uint64_t dyn_hash = hash_body_indices(bodies, dyn_idx) ^ joints_layout_hash(joints);
  const uint64_t obs_hash = hash_body_indices(bodies, obs_idx);

  bool reuse_dyn = b3World_IsValid(persist.world) && persist.dyn_hash == dyn_hash &&
                   persist.dyn_ids.size() == dyn_idx.size();
  if (reuse_dyn) {
    for (const int k : dyn_idx.index_range()) {
      if (!b3Body_IsValid(persist.dyn_ids[k])) {
        reuse_dyn = false;
        break;
      }
      const b3Pos pp = b3Body_GetPosition(persist.dyn_ids[k]);
      const float3 phys(float(pp.x), float(pp.y), float(pp.z));
      if (math::distance_squared(phys, bodies[dyn_idx[k]].position) > 4.0f) {
        reuse_dyn = false;
        break;
      }
    }
  }

  b3WorldId world = persist.world;
  bool reuse_obs = false;
  if (!reuse_dyn) {
    persist_3d_reset(persist);
    b3WorldDef world_def = b3DefaultWorldDef();
    world_def.workerCount = 1;
    world_def.enableContinuous = true;
    world_def.gravity = g3;
    world = b3CreateWorld(&world_def);
    if (!b3World_IsValid(world)) {
      r_error = "Box3D CreateWorld failed";
      return false;
    }
    persist.world = world;
    persist.dyn_ids.reserve(dyn_idx.size());
    persist.obs_ids.reserve(obs_idx.size());
    for (const int i : bodies.index_range()) {
      Vector<b3MeshData *> &meshes = (bodies[i].body_type == 2) ? persist.dyn_meshes :
                                                                 persist.obs_meshes;
      const b3BodyId id = create_one_body_3d(world, bodies[i], meshes);
      if (bodies[i].body_type == 2) {
        persist.dyn_ids.append(id);
      }
      else {
        persist.obs_ids.append(id);
      }
    }
    persist.dyn_hash = dyn_hash;
    persist.obs_hash = obs_hash;
    reuse_obs = true;
  }
  else {
    world = persist.world;
    b3World_SetGravity(world, g3);
    reuse_obs = persist.obs_hash == obs_hash && persist.obs_ids.size() == obs_idx.size();
    if (reuse_obs) {
      for (const b3BodyId id : persist.obs_ids) {
        if (!b3Body_IsValid(id)) {
          reuse_obs = false;
          break;
        }
      }
    }
    if (reuse_obs) {
      for (const int k : obs_idx.index_range()) {
        drive_or_snap_3d(persist.obs_ids[k], world, bodies[obs_idx[k]], persist.obs_meshes, h);
      }
    }
    else {
      destroy_ids_3d(persist.obs_ids);
      destroy_meshes_3d(persist.obs_meshes);
      persist.obs_ids.reserve(obs_idx.size());
      for (const int i : obs_idx) {
        persist.obs_ids.append(create_one_body_3d(world, bodies[i], persist.obs_meshes));
      }
      persist.obs_hash = obs_hash;
    }
  }

  Array<b3BodyId> body_ids(bodies.size());
  assemble_body_ids_3d(bodies, persist, body_ids);
  const bool create_all_joints = !reuse_dyn;
  const bool create_obs_joints = reuse_dyn && !reuse_obs;
  if (create_all_joints || create_obs_joints) {
  b3BodyId world_body = b3_nullBodyId;
  auto ensure_world_body = [&]() {
    if (b3Body_IsValid(world_body)) {
      return world_body;
    }
    b3BodyDef def = b3DefaultBodyDef();
    def.type = b3_staticBody;
    world_body = b3CreateBody(world, &def);
    return world_body;
  };

  auto quat_x_to = [](const b3Vec3 &unit_axis) -> b3Quat {
    const b3Vec3 x = {1.0f, 0.0f, 0.0f};
    if (b3LengthSquared(unit_axis) < 1e-12f) {
      return b3Quat_identity;
    }
    return b3ComputeQuatBetweenUnitVectors(x, b3Normalize(unit_axis));
  };

  for (const JointRead &j : joints) {
    if (j.body_a < 0 || j.body_a >= bodies.size()) {
      continue;
    }
    const bool is_revolute = (j.joint_type == int(JointType::Revolute));
    if (!is_revolute && (j.body_b < 0 || j.body_b >= bodies.size() || j.body_b == j.body_a)) {
      continue;
    }
    if (create_obs_joints) {
      const bool a_obs = bodies[j.body_a].body_type != 2;
      const bool b_obs = !is_revolute && bodies[j.body_b].body_type != 2;
      if (!a_obs && !b_obs) {
        continue;
      }
    }
    b3BodyId bodyA = body_ids[j.body_a];
    b3BodyId bodyB = is_revolute ? ensure_world_body() : body_ids[j.body_b];
    if (!b3Body_IsValid(bodyA) || !b3Body_IsValid(bodyB)) {
      continue;
    }

    auto fill_base = [&](b3JointDef &base,
                         const b3Vec3 localA,
                         const b3Vec3 localB,
                         const b3Quat qA,
                         const b3Quat qB) {
      base.bodyIdA = bodyA;
      base.bodyIdB = bodyB;
      base.localFrameA.p = localA;
      base.localFrameA.q = qA;
      base.localFrameB.p = localB;
      base.localFrameB.q = qB;
      base.collideConnected = false;
    };

    if (j.joint_type == int(JointType::Distance)) {
      b3DistanceJointDef def = b3DefaultDistanceJointDef();
      const b3Pos pa = b3Body_GetPosition(bodyA);
      const b3Pos pb = b3Body_GetPosition(bodyB);
      const float3 d(float(pb.x - pa.x), float(pb.y - pa.y), float(pb.z - pa.z));
      def.length = math::max(math::length(d), 1e-4f);
      fill_base(def.base, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, b3Quat_identity, b3Quat_identity);
      b3CreateDistanceJoint(world, &def);
    }
    else if (j.joint_type == int(JointType::Weld)) {
      b3WeldJointDef def = b3DefaultWeldJointDef();
      const b3Pos pa = b3Body_GetPosition(bodyA);
      const b3Pos pb = b3Body_GetPosition(bodyB);
      const b3Pos mid = {
          0.5f * (pa.x + pb.x), 0.5f * (pa.y + pb.y), 0.5f * (pa.z + pb.z)};
      fill_base(def.base,
                b3Body_GetLocalPoint(bodyA, mid),
                b3Body_GetLocalPoint(bodyB, mid),
                b3Conjugate(b3Body_GetRotation(bodyA)),
                b3Conjugate(b3Body_GetRotation(bodyB)));
      b3CreateWeldJoint(world, &def);
    }
    else if (j.joint_type == int(JointType::Prismatic)) {
      b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
      b3Vec3 axis = {j.axis.x, j.axis.y, j.axis.z};
      if (b3LengthSquared(axis) < 1e-12f) {
        axis = {1.0f, 0.0f, 0.0f};
      }
      axis = b3Normalize(axis);
      const b3Vec3 axis_a = b3Body_GetLocalVector(bodyA, axis);
      const b3Vec3 axis_b = b3Body_GetLocalVector(bodyB, axis);
      const b3Pos pa = b3Body_GetPosition(bodyA);
      fill_base(def.base,
                {0.0f, 0.0f, 0.0f},
                b3Body_GetLocalPoint(bodyB, pa),
                quat_x_to(axis_a),
                quat_x_to(axis_b));
      b3CreatePrismaticJoint(world, &def);
    }
    else {
      b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
      const b3Pos anchor = {j.anchor.x, j.anchor.y, j.anchor.z};
      /* Hinge about world Z so the body swings in XY (matches Box2D). */
      fill_base(def.base,
                b3Body_GetLocalPoint(bodyA, anchor),
                b3Body_GetLocalPoint(bodyB, anchor),
                b3Conjugate(b3Body_GetRotation(bodyA)),
                b3Conjugate(b3Body_GetRotation(bodyB)));
      b3CreateRevoluteJoint(world, &def);
    }
  }
  } /* create joints */

  const float step_h = h / float(slices);
  for (int s = 0; s < slices; s++) {
    b3World_Step(world, step_h, 4);
  }

  for (const int i : bodies.index_range()) {
    if (i >= body_ids.size() || !b3Body_IsValid(body_ids[i])) {
      r_positions[i] = bodies[i].position;
      r_velocities[i] = bodies[i].velocity;
      r_ang[i] = bodies[i].angular_velocity;
      r_rotations[i] = bodies[i].rotation;
      continue;
    }
    const b3BodyId body = body_ids[i];
    const b3Pos pos = b3Body_GetPosition(body);
    const b3Vec3 vel = b3Body_GetLinearVelocity(body);
    const b3Vec3 ang = b3Body_GetAngularVelocity(body);
    const b3Quat rot = b3Body_GetRotation(body);
    const BodyRead &src = bodies[i];
    if (src.graph_driven || src.body_type != 2) {
      r_positions[i] = src.position;
      r_rotations[i] = src.rotation;
      r_velocities[i] = float3(vel.x, vel.y, vel.z);
      r_ang[i] = float3(ang.x, ang.y, ang.z);
      continue;
    }
    r_positions[i] = float3(float(pos.x), float(pos.y), float(pos.z));
    r_velocities[i] = float3(vel.x, vel.y, vel.z);
    r_ang[i] = float3(ang.x, ang.y, ang.z);
    r_rotations[i] = math::Quaternion(rot.s, rot.v.x, rot.v.y, rot.v.z);
  }

  return true;
}

#endif /* WITH_BOX3D */

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry"_ustr);
  GeometrySet constraints_set = params.extract_input<GeometrySet>("Constraints"_ustr);
  const EngineKind engine = params.extract_input<EngineKind>("Engine"_ustr);
  constexpr PlaneAxis plane = PlaneAxis::XY;
  const float3 gravity = params.extract_input<float3>("Gravity"_ustr);
  const float dt = params.extract_input<float>("Delta Time"_ustr);
  const int substeps = params.extract_input<int>("Substeps"_ustr);
  /* Fallbacks only if Set Rigid Body attributes are missing. */
  const float default_radius = 0.25f;
  const float default_density = 1.0f;
  const std::string cache_ns = fmt::format(
      "{}:{}", fmt::ptr(params.self_object()), params.node().identifier);

  if (geometry_set.is_empty()) {
    params.set_output("Geometry"_ustr, std::move(geometry_set));
    return;
  }

  Vector<BodyRead> bodies;
  enum class DomainKind { None, Points, Instances, Mesh } domain = DomainKind::None;

  /* Prefer Instances — common case after Set Rigid Body / Instance on Points.
   * Optional Mesh component = Compound collision proxy (visual stays on instances). */
  if (const bke::Instances *instances = geometry_set.get_instances()) {
    if (instances->instances_num() > 0) {
      domain = DomainKind::Instances;
      read_bodies_instances(geometry_set, default_radius, default_density, bodies);
      if (const Mesh *mesh = geometry_set.get_mesh()) {
        if (box_engine::mesh_is_compound_proxy(*mesh)) {
          /* Drop collision-proxy mesh so Geometry output is visual instances only. */
          geometry_set.keep_only({GeometryComponent::Type::Instance});
        }
        else if (mesh->verts_num > 0) {
          /* Join Geometry(instances, static wall mesh). Tag walls with Set Rigid Body Static. */
          append_mesh_as_collider(*mesh, bodies);
        }
      }
    }
  }
  if (domain == DomainKind::None) {
    if (const PointCloud *points = geometry_set.get_pointcloud()) {
      if (points->totpoint > 0) {
        domain = DomainKind::Points;
        read_bodies_points(*points, default_radius, default_density, bodies);
      }
    }
  }
  if (domain == DomainKind::None) {
    if (const Mesh *mesh = geometry_set.get_mesh()) {
      if (mesh->verts_num > 0) {
        domain = DomainKind::Mesh;
        read_bodies_mesh(*mesh, default_radius, default_density, bodies);
      }
    }
  }

  if (domain == DomainKind::None || bodies.is_empty()) {
    params.error_message_add(
        NodeWarningType::Error,
        TIP_("Box Engine: no bodies. Use Instances, Points, or Mesh (try Set Rigid Body)"));
    params.set_output("Geometry"_ustr, std::move(geometry_set));
    return;
  }

  int tri_missing = 0;
  int compound_empty = 0;
  for (const BodyRead &b : bodies) {
    if (b.shape_type == int(ShapeType::TriangleMesh) && b.mesh_tri_indices.size() < 3) {
      tri_missing++;
    }
    if (b.shape_type == int(ShapeType::Compound) && b.island_verts.is_empty()) {
      compound_empty++;
    }
  }
  if (tri_missing > 0) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("Triangle Mesh: no triangles on some bodies — check the mesh input "
             "(Box3D Static uses the real mesh; otherwise per-island hulls)"));
  }
  if (compound_empty > 0) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("Compound: no mesh islands found — connect a mesh (or Proxy) with "
             "disconnected pieces"));
  }
  else {
    int compound_hulls = 0;
    for (const BodyRead &b : bodies) {
      if (b.shape_type == int(ShapeType::Compound)) {
        compound_hulls += b.island_verts.size();
      }
    }
    if (compound_hulls > 0) {
      const std::string msg = fmt::format(
          fmt::runtime(TIP_("Box Engine: Compound using {} hulls")), compound_hulls);
      params.error_message_add(NodeWarningType::Info, msg);
    }
  }

  Vector<JointRead> joints;
  if (const PointCloud *cpoints = constraints_set.get_pointcloud()) {
    if (cpoints->totpoint > 0) {
      read_joints(*cpoints, joints);
      resolve_joint_names(bodies, joints);
    }
  }

  Array<float3> new_positions(bodies.size());
  Array<float3> new_velocities(bodies.size());
  Array<float> new_ang_z(bodies.size());
  Array<float3> new_ang(bodies.size());
  Array<math::Quaternion> new_rots(bodies.size());
  std::string error;
  bool ok = false;

  if (engine == EngineKind::Box2D) {
#if defined(WITH_BOX2D)
    ok = step_box2d(bodies,
                    joints,
                    gravity,
                    dt,
                    substeps,
                    plane,
                    cache_ns,
                    new_positions,
                    new_velocities,
                    new_ang_z,
                    new_rots,
                    error);
    /* XY: ω about world Z. XZ: ω about world Y. */
    for (const int i : bodies.index_range()) {
      if (plane == PlaneAxis::XY) {
        new_ang[i] = float3(0.0f, 0.0f, new_ang_z[i]);
      }
      else {
        new_ang[i] = float3(0.0f, new_ang_z[i], 0.0f);
      }
    }
#else
    error = "Built without WITH_BOX2D";
#endif
  }
  else {
#if defined(WITH_BOX3D)
    ok = step_box3d(bodies,
                    joints,
                    gravity,
                    dt,
                    substeps,
                    cache_ns,
                    new_positions,
                    new_velocities,
                    new_ang,
                    new_rots,
                    error);
    for (const int i : bodies.index_range()) {
      new_ang_z[i] = new_ang[i].z;
    }
#else
    error = "Built without WITH_BOX3D";
#endif
  }

  if (!ok) {
    params.error_message_add(NodeWarningType::Error, error.c_str());
    params.set_output("Geometry"_ustr, std::move(geometry_set));
    return;
  }

  if (domain == DomainKind::Instances) {
    if (bke::Instances *instances = geometry_set.get_instances_for_write()) {
      write_instance_results(
          *instances, bodies, new_positions, new_rots, new_velocities, new_ang);
    }
  }
  else if (domain == DomainKind::Points) {
    if (PointCloud *points = geometry_set.get_pointcloud_for_write()) {
      write_point_results(*points, new_positions, new_velocities, new_ang_z);
    }
  }
  else if (domain == DomainKind::Mesh) {
    /* Mesh-as-single-body: only write velocity attrs if present; transform via points not mesh. */
    if (Mesh *mesh = geometry_set.get_mesh_for_write()) {
      const float3 delta = new_positions[0] - bodies[0].position;
      MutableSpan<float3> verts = mesh->vert_positions_for_write();
      for (float3 &v : verts) {
        v += delta;
      }
      mesh->tag_positions_changed();
    }
  }

  params.set_output("Geometry"_ustr, std::move(geometry_set));
}

static void register_box_solver()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeBoxEngineSolver"_ustr, GEO_NODE_BOX_ENGINE_SOLVER);
  ntype.ui_name = "Box Solver";
  ntype.ui_description =
      "Step Box2D/Box3D rigid bodies. Static/Kinematic bodies on Geometry are obstacles. "
      "Substeps = collide slices this frame. Prefer Instances inside a Simulation Zone";
  ntype.enum_name_legacy = "BOX_ENGINE_SOLVER";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(register_box_solver)

}  // namespace blender::nodes::node_geo_box_engine_solver_cc
