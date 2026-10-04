/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: MIT
 *
 * Glue only: drive official Jolt Physics. Do not reimplement the solver here.
 */

#include "jolt_bridge.h"

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/MotionProperties.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/PulleyConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Geometry/IndexedTriangle.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

JPH_SUPPRESS_WARNINGS

using namespace JPH;

namespace {

constexpr ObjectLayer LAYER_NON_MOVING = 0;
constexpr ObjectLayer LAYER_MOVING = 1;

class ObjectLayerPairFilterImpl final : public ObjectLayerPairFilter {
 public:
  bool ShouldCollide(ObjectLayer a, ObjectLayer b) const override
  {
    switch (a) {
      case LAYER_NON_MOVING:
        return b == LAYER_MOVING;
      case LAYER_MOVING:
        return true;
      default:
        return false;
    }
  }
};

class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface {
 public:
  BPLayerInterfaceImpl()
  {
    m_map[LAYER_NON_MOVING] = BroadPhaseLayer(0);
    m_map[LAYER_MOVING] = BroadPhaseLayer(1);
  }
  uint GetNumBroadPhaseLayers() const override
  {
    return 2;
  }
  BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override
  {
    return m_map[inLayer < 2 ? inLayer : LAYER_MOVING];
  }

 private:
  BroadPhaseLayer m_map[2];
};

class ObjectVsBroadPhaseLayerFilterImpl final : public ObjectVsBroadPhaseLayerFilter {
 public:
  bool ShouldCollide(ObjectLayer inLayer, BroadPhaseLayer inBP) const override
  {
    if (inLayer == LAYER_NON_MOVING) {
      return inBP == BroadPhaseLayer(1);
    }
    return true;
  }
};

static void TraceImpl(const char *, ...) {}

#ifdef JPH_ENABLE_ASSERTS
static bool AssertFailedImpl(const char *, const char *, const char *, uint)
{
  return false;
}
#endif

static std::once_flag g_init_once;
static std::mutex g_mu;
static std::unique_ptr<TempAllocatorImpl> g_temp;
static std::unique_ptr<JobSystemThreadPool> g_jobs;

static void ensure_jolt()
{
  std::call_once(g_init_once, []() {
    RegisterDefaultAllocator();
    Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(AssertFailed = AssertFailedImpl;)
    Factory::sInstance = new Factory();
    RegisterTypes();
    g_temp = std::make_unique<TempAllocatorImpl>(32 * 1024 * 1024);
    const int threads = std::max(1, int(std::thread::hardware_concurrency()) - 1);
    g_jobs = std::make_unique<JobSystemThreadPool>(cMaxPhysicsJobs, cMaxPhysicsBarriers, threads);
  });
}

static Vec3 v3(const JoltVec3 &v)
{
  return Vec3(v.x, v.y, v.z);
}

static RVec3 rv3(const JoltVec3 &v)
{
  return RVec3(v.x, v.y, v.z);
}

static Quat quat(const JoltQuat &q)
{
  const float n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n < 1.0e-8f) {
    return Quat::sIdentity();
  }
  return Quat(q.x / n, q.y / n, q.z / n, q.w / n);
}

static JoltVec3 jv(Vec3Arg v)
{
  return {v.GetX(), v.GetY(), v.GetZ()};
}

static JoltQuat jq(QuatArg q)
{
  return {q.GetW(), q.GetX(), q.GetY(), q.GetZ()};
}

static Vec3 safe_axis(const JoltVec3 &v, Vec3 fallback)
{
  const Vec3 a(v.x, v.y, v.z);
  const float len = a.Length();
  if (len < 1.0e-6f) {
    return fallback;
  }
  return a / len;
}

static Vec3 perp_axis(Vec3Arg axis)
{
  return axis.GetNormalizedPerpendicular();
}

static EAllowedDOFs dofs_from_int(int v)
{
  switch (v) {
    case JOLT_DOFS_PLANE_XY:
      return EAllowedDOFs::Plane2D;
    case JOLT_DOFS_PLANE_XZ:
      return EAllowedDOFs::TranslationX | EAllowedDOFs::TranslationZ | EAllowedDOFs::RotationY;
    case JOLT_DOFS_PLANE_YZ:
      return EAllowedDOFs::TranslationY | EAllowedDOFs::TranslationZ | EAllowedDOFs::RotationX;
    case JOLT_DOFS_TRANSLATION:
      return EAllowedDOFs::TranslationX | EAllowedDOFs::TranslationY | EAllowedDOFs::TranslationZ;
    case JOLT_DOFS_ROTATION:
      return EAllowedDOFs::RotationX | EAllowedDOFs::RotationY | EAllowedDOFs::RotationZ;
    default:
      return EAllowedDOFs::All;
  }
}

static EMotionType motion_from_int(int v)
{
  switch (v) {
    case JOLT_MOTION_STATIC:
      return EMotionType::Static;
    case JOLT_MOTION_KINEMATIC:
      return EMotionType::Kinematic;
    default:
      return EMotionType::Dynamic;
  }
}

/** Paper-thin Grid/plane AABBs tunnel. Inflate the thinnest axis to a 5 cm slab. */
static Vec3 box_half_extents(const JoltBodyDesc &b)
{
  float hx = std::max(b.shape_size.x, 1.0e-4f);
  float hy = std::max(b.shape_size.y, 1.0e-4f);
  float hz = std::max(b.shape_size.z, 1.0e-4f);
  constexpr float k_min_half = 0.025f;
  const float mn = std::min(hx, std::min(hy, hz));
  if (mn < k_min_half) {
    if (hz <= hx && hz <= hy) {
      hz = k_min_half;
    }
    else if (hy <= hx) {
      hy = k_min_half;
    }
    else {
      hx = k_min_half;
    }
  }
  return Vec3(hx, hy, hz);
}

static float convex_r(const JoltBodyDesc &b)
{
  const Vec3 h = box_half_extents(b);
  const float cap = 0.2f * std::min(h.GetX(), std::min(h.GetY(), h.GetZ()));
  const float r = b.convex_radius > 0.0f ? b.convex_radius : 0.02f;
  return std::max(0.0f, std::min(r, cap));
}

static ShapeRefC make_box(const JoltBodyDesc &b)
{
  const Vec3 h = box_half_extents(b);
  BoxShapeSettings s(h, convex_r(b));
  s.SetDensity(std::max(b.density, 1.0e-4f));
  const Shape::ShapeResult r = s.Create();
  return r.IsValid() ? r.Get() : ShapeRefC();
}

static ShapeRefC make_sphere(const JoltBodyDesc &b)
{
  SphereShapeSettings s(std::max(b.shape_size.x, 1.0e-4f));
  s.SetDensity(std::max(b.density, 1.0e-4f));
  const Shape::ShapeResult r = s.Create();
  return r.IsValid() ? r.Get() : ShapeRefC();
}

static ShapeRefC rotate_y_to_z(ShapeRefC shape)
{
  if (shape == nullptr) {
    return shape;
  }
  RotatedTranslatedShapeSettings rt(
      Vec3::sZero(), Quat::sRotation(Vec3::sAxisX(), 0.5f * JPH_PI), shape);
  const Shape::ShapeResult r = rt.Create();
  return r.IsValid() ? r.Get() : shape;
}

static ShapeRefC make_capsule(const JoltBodyDesc &b)
{
  /* shape_size.x = radius, shape_size.y = half of total height (caps included). */
  const float radius = std::max(b.shape_size.x, 1.0e-4f);
  const float half_total = std::max(b.shape_size.y, radius);
  const float half_cyl = std::max(half_total - radius, 0.0f);
  CapsuleShapeSettings s(half_cyl, radius);
  s.SetDensity(std::max(b.density, 1.0e-4f));
  const Shape::ShapeResult r = s.Create();
  return r.IsValid() ? rotate_y_to_z(r.Get()) : make_sphere(b);
}

static ShapeRefC make_tapered_capsule(const JoltBodyDesc &b)
{
  return make_capsule(b);
}

static ShapeRefC make_cylinder(const JoltBodyDesc &b)
{
  const float radius = std::max(b.shape_size.x, 1.0e-4f);
  const float half = std::max(b.shape_size.y, 1.0e-4f);
  const float cr = std::min(convex_r(b), 0.5f * radius);
  CylinderShapeSettings s(half, radius, cr);
  s.SetDensity(std::max(b.density, 1.0e-4f));
  Shape::ShapeResult r = s.Create();
  if (!r.IsValid()) {
    s.mConvexRadius = 0.0f;
    r = s.Create();
  }
  return r.IsValid() ? rotate_y_to_z(r.Get()) : make_box(b);
}

static ShapeRefC make_tapered_cylinder(const JoltBodyDesc &b)
{
  return make_cylinder(b);
}

static Array<Vec3> verts_from(const JoltBodyDesc &b)
{
  Array<Vec3> pts;
  if (!b.verts_xyz || b.nverts <= 0) {
    return pts;
  }
  pts.reserve(uint(b.nverts));
  for (int i = 0; i < b.nverts; i++) {
    pts.push_back(Vec3(b.verts_xyz[i * 3 + 0], b.verts_xyz[i * 3 + 1], b.verts_xyz[i * 3 + 2]));
  }
  return pts;
}

static ShapeRefC make_hull(const JoltBodyDesc &b)
{
  Array<Vec3> pts = verts_from(b);
  if (pts.size() < 4) {
    return make_box(b);
  }
  ConvexHullShapeSettings s(pts, 0.0f);
  s.SetDensity(std::max(b.density, 1.0e-4f));
  s.mMaxConvexRadius = convex_r(b);
  Shape::ShapeResult r = s.Create();
  if (!r.IsValid()) {
    s.mMaxConvexRadius = 0.0f;
    r = s.Create();
  }
  return r.IsValid() ? r.Get() : make_box(b);
}

static ShapeRefC make_mesh(const JoltBodyDesc &b)
{
  if (!b.verts_xyz || b.nverts < 3 || !b.tri_indices || b.nindices < 3) {
    return make_hull(b);
  }
  VertexList verts;
  verts.reserve(uint(b.nverts));
  for (int i = 0; i < b.nverts; i++) {
    verts.push_back(Float3(b.verts_xyz[i * 3 + 0], b.verts_xyz[i * 3 + 1], b.verts_xyz[i * 3 + 2]));
  }
  IndexedTriangleList tris;
  tris.reserve(uint(b.nindices / 3));
  for (int i = 0; i + 2 < b.nindices; i += 3) {
    const uint32 i0 = uint32(b.tri_indices[i]);
    const uint32 i1 = uint32(b.tri_indices[i + 1]);
    const uint32 i2 = uint32(b.tri_indices[i + 2]);
    if (i0 == i1 || i1 == i2 || i2 == i0) {
      continue;
    }
    if (i0 >= uint32(b.nverts) || i1 >= uint32(b.nverts) || i2 >= uint32(b.nverts)) {
      continue;
    }
    tris.push_back(IndexedTriangle(i0, i1, i2, 0));
  }
  if (tris.empty()) {
    return make_hull(b);
  }
  MeshShapeSettings s(std::move(verts), std::move(tris));
  const Shape::ShapeResult r = s.Create();
  return r.IsValid() ? r.Get() : make_hull(b);
}

static ShapeRefC make_compound(const JoltBodyDesc &b)
{
  if (!b.verts_xyz || !b.island_counts || b.nislands <= 0) {
    return make_hull(b);
  }
  StaticCompoundShapeSettings compound;
  int cursor = 0;
  int added = 0;
  for (int k = 0; k < b.nislands; k++) {
    const int n = b.island_counts[k];
    if (n < 3 || cursor + n > b.nverts) {
      cursor += std::max(n, 0);
      continue;
    }
    Array<Vec3> pts;
    pts.reserve(uint(n));
    for (int i = 0; i < n; i++) {
      const int vi = cursor + i;
      pts.push_back(Vec3(b.verts_xyz[vi * 3 + 0], b.verts_xyz[vi * 3 + 1], b.verts_xyz[vi * 3 + 2]));
    }
    cursor += n;
    ConvexHullShapeSettings hull(pts, convex_r(b));
    hull.SetDensity(std::max(b.density, 1.0e-4f));
    const Shape::ShapeResult hr = hull.Create();
    if (!hr.IsValid()) {
      continue;
    }
    compound.AddShape(Vec3::sZero(), Quat::sIdentity(), hr.Get());
    added++;
  }
  if (added <= 0) {
    return make_hull(b);
  }
  const Shape::ShapeResult r = compound.Create(*g_temp);
  return r.IsValid() ? r.Get() : make_hull(b);
}

static ShapeRefC make_plane(const JoltBodyDesc &b)
{
  const JoltVec3 nsrc = (std::fabs(b.shape_size.x) + std::fabs(b.shape_size.y) +
                         std::fabs(b.shape_size.z)) > 1.0e-6f ?
                            b.shape_size :
                            JoltVec3{0.0f, 0.0f, 1.0f};
  const Vec3 n = safe_axis(nsrc, Vec3::sAxisZ());
  /* Plane through local origin; body pose places it in world. */
  PlaneShapeSettings s(Plane(n, 0.0f), nullptr, 1000.0f);
  const Shape::ShapeResult r = s.Create();
  return r.IsValid() ? r.Get() : make_box(b);
}

static ShapeRefC make_heightfield(const JoltBodyDesc &b)
{
  if (!b.height_samples || b.height_size < 2) {
    return make_mesh(b);
  }
  const uint32 n = uint32(b.height_size);
  const float cell = b.height_cell > 0.0f ? b.height_cell : 1.0f;
  const Vec3 offset(-0.5f * cell * float(n - 1), 0.0f, -0.5f * cell * float(n - 1));
  const Vec3 scale(cell, 1.0f, cell);
  HeightFieldShapeSettings s(b.height_samples, offset, scale, n);
  const Shape::ShapeResult r = s.Create();
  return r.IsValid() ? r.Get() : make_mesh(b);
}

static ShapeRefC make_shape(const JoltBodyDesc &b)
{
  ShapeRefC shape;
  switch (b.shape_type) {
    case JOLT_SHAPE_SPHERE:
      shape = make_sphere(b);
      break;
    case JOLT_SHAPE_BOX:
    case JOLT_SHAPE_BOUNDING_BOX:
      shape = make_box(b);
      break;
    case JOLT_SHAPE_CAPSULE:
      shape = make_capsule(b);
      break;
    case JOLT_SHAPE_TAPERED_CAPSULE:
      shape = make_tapered_capsule(b);
      break;
    case JOLT_SHAPE_CYLINDER:
      shape = make_cylinder(b);
      break;
    case JOLT_SHAPE_TAPERED_CYLINDER:
      shape = make_tapered_cylinder(b);
      break;
    case JOLT_SHAPE_CONVEX_HULL:
      shape = make_hull(b);
      break;
    case JOLT_SHAPE_MESH:
      shape = make_mesh(b);
      break;
    case JOLT_SHAPE_COMPOUND:
      shape = make_compound(b);
      break;
    case JOLT_SHAPE_PLANE:
    case JOLT_SHAPE_HEIGHTFIELD:
    case JOLT_SHAPE_EMPTY:
      /* Removed from the UI. Old files fall back to a box. */
      shape = make_box(b);
      break;
    default:
      shape = make_box(b);
      break;
  }
  if (shape == nullptr) {
    shape = new BoxShape(Vec3(0.25f, 0.25f, 0.25f));
  }
  /* Body position is COM in Jolt. Snap COM to the instance origin so write-back
   * matches Geometry Nodes and add/remove does not pop the mesh. */
  const Vec3 com = shape->GetCenterOfMass();
  if (com.LengthSq() > 1.0e-12f) {
    OffsetCenterOfMassShapeSettings off(-com, shape);
    const Shape::ShapeResult cr = off.Create();
    if (cr.IsValid()) {
      shape = cr.Get();
    }
  }
  return shape;
}

static BodyID invalid_if_world(int index, const std::vector<BodyID> &ids)
{
  if (index < 0 || index >= int(ids.size())) {
    return BodyID();
  }
  return ids[uint(index)];
}

static void apply_motor(TwoBodyConstraint *c, int type, const JoltConstraintDesc &d)
{
  if (!c || d.motor_state == JOLT_MOTOR_OFF) {
    return;
  }
  const EMotorState state = (d.motor_state == JOLT_MOTOR_POSITION) ? EMotorState::Position :
                                                                     EMotorState::Velocity;
  const float force = d.motor_force > 0.0f ? d.motor_force : 1.0e6f;
  switch (type) {
    case JOLT_CONSTRAINT_HINGE: {
      auto *h = static_cast<HingeConstraint *>(c);
      h->GetMotorSettings().SetForceLimit(force);
      h->SetMotorState(state);
      if (state == EMotorState::Velocity) {
        h->SetTargetAngularVelocity(d.motor_target);
      }
      else {
        h->SetTargetAngle(d.motor_target);
      }
      break;
    }
    case JOLT_CONSTRAINT_SLIDER: {
      auto *s = static_cast<SliderConstraint *>(c);
      s->GetMotorSettings().SetForceLimit(force);
      s->SetMotorState(state);
      if (state == EMotorState::Velocity) {
        s->SetTargetVelocity(d.motor_target);
      }
      else {
        s->SetTargetPosition(d.motor_target);
      }
      break;
    }
    default:
      break;
  }
}

struct Engine {
  BPLayerInterfaceImpl bp;
  ObjectVsBroadPhaseLayerFilterImpl obj_vs_bp;
  ObjectLayerPairFilterImpl obj_vs_obj;
  std::unique_ptr<PhysicsSystem> physics;
  std::vector<BodyID> bodies;
  std::vector<std::string> names;
  std::vector<uint64_t> shape_fp;
  std::vector<Ref<Constraint>> constraints;
  std::vector<Ref<VehicleConstraint>> vehicles;
  uint64 topology = 0;
  uint64 constraint_fp = 0;
  int n_bodies = 0;
  int n_constraints = 0;
  std::vector<JoltVec3> last_pos;
  std::vector<int> graph_driven;
  float last_scene_time = -1.0e30f;

  void clear_constraints_only()
  {
    if (!physics) {
      return;
    }
    for (Ref<VehicleConstraint> &v : vehicles) {
      if (v) {
        physics->RemoveStepListener(v.GetPtr());
        physics->RemoveConstraint(v.GetPtr());
      }
    }
    for (Ref<Constraint> &c : constraints) {
      if (c) {
        physics->RemoveConstraint(c.GetPtr());
      }
    }
    vehicles.clear();
    constraints.clear();
    constraint_fp = 0;
  }

  void reset_bodies()
  {
    if (!physics) {
      return;
    }
    BodyInterface &bi = physics->GetBodyInterface();
    clear_constraints_only();
    if (!bodies.empty()) {
      bi.RemoveBodies(bodies.data(), uint(bodies.size()));
      bi.DestroyBodies(bodies.data(), uint(bodies.size()));
    }
    bodies.clear();
    names.clear();
    shape_fp.clear();
    last_pos.clear();
    graph_driven.clear();
    n_bodies = 0;
    n_constraints = 0;
    topology = 0;
    last_scene_time = -1.0e30f;
  }

  void clear()
  {
    reset_bodies();
    physics.reset();
  }

  ~Engine()
  {
    clear();
  }
};

static std::unordered_map<std::string, std::unique_ptr<Engine>> g_engines;

static bool positions_match(const Engine &eng, int n, const JoltBodyDesc *bodies)
{
  if (int(eng.last_pos.size()) != n) {
    return false;
  }
  float err = 0.0f;
  const int stride = std::max(n / 64, 1);
  for (int i = 0; i < n; i += stride) {
    if (eng.graph_driven.size() == uint(n) && eng.graph_driven[uint(i)]) {
      continue;
    }
    const float dx = eng.last_pos[uint(i)].x - bodies[i].position.x;
    const float dy = eng.last_pos[uint(i)].y - bodies[i].position.y;
    const float dz = eng.last_pos[uint(i)].z - bodies[i].position.z;
    err = std::max(err, dx * dx + dy * dy + dz * dz);
  }
  return err < 0.0025f; /* 5 cm */
}

static void fill_mass_if_mesh(BodyCreationSettings &settings, const JoltBodyDesc &b, const Shape *shape)
{
  if (b.shape_type != JOLT_SHAPE_MESH && b.shape_type != JOLT_SHAPE_HEIGHTFIELD &&
      b.shape_type != JOLT_SHAPE_PLANE)
  {
    return;
  }
  if (settings.mMotionType != EMotionType::Dynamic) {
    return;
  }
  const float hx = std::max(b.shape_size.x, 0.05f);
  const float hy = std::max(b.shape_size.y, 0.05f);
  const float hz = std::max(b.shape_size.z, 0.05f);
  const float mass = std::max(b.density, 1.0e-3f) * (8.0f * hx * hy * hz);
  settings.mOverrideMassProperties = EOverrideMassProperties::MassAndInertiaProvided;
  settings.mMassPropertiesOverride.mMass = mass;
  const float i = mass * (hx * hx + hy * hy) / 6.0f;
  settings.mMassPropertiesOverride.mInertia = Mat44::sScale(std::max(i, 1.0e-4f));
  (void)shape;
}

static void apply_mass(BodyCreationSettings &settings, const JoltBodyDesc &b, const Shape *shape)
{
  if (settings.mMotionType != EMotionType::Dynamic) {
    return;
  }
  MassProperties mp;
  if (shape != nullptr) {
    mp = shape->GetMassProperties();
  }
  if (b.mass > 1.0e-6f) {
    if (mp.mMass > 1.0e-8f) {
      mp.ScaleToMass(b.mass);
    }
    else {
      mp.mMass = b.mass;
    }
  }
  else if (mp.mMass < 1.0e-8f) {
    fill_mass_if_mesh(settings, b, shape);
    return;
  }
  if (b.inertia.x > 1.0e-8f || b.inertia.y > 1.0e-8f || b.inertia.z > 1.0e-8f) {
    mp.mInertia = Mat44::sScale(Vec3(std::max(b.inertia.x, 1.0e-6f),
                                     std::max(b.inertia.y, 1.0e-6f),
                                     std::max(b.inertia.z, 1.0e-6f)));
  }
  if (mp.mMass > 1.0e-8f) {
    settings.mOverrideMassProperties = EOverrideMassProperties::MassAndInertiaProvided;
    settings.mMassPropertiesOverride = mp;
  }
}

static std::string key_of(const JoltBodyDesc &b, int i)
{
  if (b.name && b.name[0] != '\0') {
    return std::string(b.name);
  }
  char buf[24];
  std::snprintf(buf, sizeof(buf), "#%d", i);
  return buf;
}

static uint64_t mix64(uint64_t h, uint64_t v)
{
  h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
  return h;
}

static uint32 quant_mm(float x)
{
  return uint32(int32(std::lround(double(x) * 500.0))); /* 2 mm */
}

static uint64_t shape_fp_of(const JoltBodyDesc &b)
{
  uint64_t h = uint64_t(uint32_t(b.shape_type)) | (uint64_t(uint32_t(b.motion_type)) << 8);
  h = mix64(h, uint64_t(uint32_t(b.nverts)));
  h = mix64(h, uint64_t(uint32_t(b.nindices)));
  h = mix64(h, uint64_t(uint32_t(b.nislands)));
  h = mix64(h, quant_mm(b.shape_size.x));
  h = mix64(h, quant_mm(b.shape_size.y));
  h = mix64(h, quant_mm(b.shape_size.z));
  h = mix64(h, quant_mm(b.convex_radius));
  h = mix64(h, quant_mm(b.density));
  if (b.verts_xyz && b.nverts > 0) {
    h = mix64(h, quant_mm(b.verts_xyz[0]));
    h = mix64(h, quant_mm(b.verts_xyz[1]));
    h = mix64(h, quant_mm(b.verts_xyz[2]));
    const int last = (b.nverts - 1) * 3;
    h = mix64(h, quant_mm(b.verts_xyz[last]));
    h = mix64(h, quant_mm(b.verts_xyz[last + 1]));
    h = mix64(h, quant_mm(b.verts_xyz[last + 2]));
  }
  return h;
}

static uint64_t constraints_fp(int n, const JoltConstraintDesc *c)
{
  uint64_t h = uint64_t(uint32_t(n));
  for (int i = 0; i < n; i++) {
    h = h * 1000003ull ^ uint64_t(uint32_t(c[i].type));
    h = h * 1000003ull ^ uint64_t(uint32_t(c[i].body_a));
    h = h * 1000003ull ^ uint64_t(uint32_t(c[i].body_b));
    h = h * 1000003ull ^ uint64_t(uint32_t(c[i].npath));
  }
  return h;
}

static ShapeRefC shape_for_desc(const JoltBodyDesc &b, EMotionType &motion)
{
  ShapeRefC shape = make_shape(b);
  /* Mesh / heightfield / plane cannot be dynamic OR kinematic in Jolt. */
  if (shape && shape->MustBeStatic() && motion != EMotionType::Static) {
    shape = make_hull(b);
    if (shape == nullptr || shape->MustBeStatic()) {
      if (motion == EMotionType::Dynamic) {
        motion = EMotionType::Static;
      }
      else {
        shape = make_box(b);
      }
    }
  }
  if (shape == nullptr) {
    shape = new BoxShape(Vec3(0.25f, 0.25f, 0.25f));
  }
  return shape;
}

static BodyID spawn_body(Engine &eng,
                         const JoltStepSettings &st,
                         const JoltBodyDesc &b,
                         EActivation activation)
{
  EMotionType motion = motion_from_int(b.motion_type);
  ShapeRefC shape = shape_for_desc(b, motion);
  const ObjectLayer layer = (motion == EMotionType::Static) ? LAYER_NON_MOVING : LAYER_MOVING;
  BodyCreationSettings settings(shape.GetPtr(), rv3(b.position), quat(b.rotation), motion, layer);
  settings.mFriction = std::max(b.friction, 0.0f);
  settings.mRestitution = std::max(b.restitution, 0.0f);
  settings.mLinearDamping = std::max(b.linear_damping, 0.0f);
  settings.mAngularDamping = std::max(b.angular_damping, 0.0f);
  settings.mIsSensor = b.sensor != 0;
  settings.mAllowedDOFs = dofs_from_int(b.allowed_dofs);
  settings.mMotionQuality = (b.ccd || st.ccd) && motion == EMotionType::Dynamic ?
                                EMotionQuality::LinearCast :
                                EMotionQuality::Discrete;
  apply_mass(settings, b, shape.GetPtr());
  settings.mGravityFactor = 1.0f;
  BodyInterface &bi = eng.physics->GetBodyInterface();
  const BodyID id = bi.CreateAndAddBody(settings, activation);
  if (id.IsInvalid()) {
    return id;
  }
  if (motion != EMotionType::Static) {
    bi.SetGravityFactor(id, 1.0f);
    bi.SetLinearVelocity(id, v3(b.linear_velocity));
    bi.SetAngularVelocity(id, v3(b.angular_velocity));
    bi.ActivateBody(id);
  }
  return id;
}

static void apply_physics_quality(PhysicsSystem &physics, const JoltStepSettings &st, bool has_kinematic)
{
  PhysicsSettings ps = physics.GetPhysicsSettings();
  ps.mNumVelocitySteps = uint(std::max(2, st.velocity_steps));
  /* Stacks + moving platforms need extra position correction. */
  ps.mNumPositionSteps = uint(std::max(has_kinematic ? 4 : 2, st.position_steps));
  ps.mDeterministicSimulation = true;
  ps.mUseManifoldReduction = true;
  /* Persistent world: sleeping is what lets stacks rest. Airborne bodies
   * stay awake because their velocity is above the sleep threshold. */
  ps.mAllowSleeping = true;
  ps.mTimeBeforeSleep = 0.4f;
  ps.mPointVelocitySleepThreshold = 0.02f;
  /* Default 2 cm slop looks like tunneling on 8 cm platforms / Grid colliders. */
  ps.mPenetrationSlop = 0.004f;
  ps.mSpeculativeContactDistance = has_kinematic ? 0.05f : 0.02f;
  ps.mMaxPenetrationDistance = 0.08f;
  ps.mBaumgarte = 0.2f;
  physics.SetPhysicsSettings(ps);
}

/** Keep the BodyID (contacts / sleep) but replace the collision shape when size changes. */
static void refresh_existing_body(PhysicsSystem &physics,
                                  const JoltStepSettings &st,
                                  const JoltBodyDesc &b,
                                  const BodyID id,
                                  bool shape_changed)
{
  BodyInterface &bi = physics.GetBodyInterface();
  if (shape_changed) {
    EMotionType motion = motion_from_int(b.motion_type);
    ShapeRefC shape = shape_for_desc(b, motion);
    if (shape) {
      bi.SetShape(id, shape.GetPtr(), true, EActivation::Activate);
    }
  }
  bi.SetFriction(id, std::max(b.friction, 0.0f));
  bi.SetRestitution(id, std::max(b.restitution, 0.0f));
  const EMotionType mt = bi.GetMotionType(id);
  if (mt != EMotionType::Static) {
    const EMotionQuality q = (b.ccd || st.ccd) && mt == EMotionType::Dynamic ?
                                 EMotionQuality::LinearCast :
                                 EMotionQuality::Discrete;
    bi.SetMotionQuality(id, q);
    if (mt == EMotionType::Dynamic) {
      bi.SetGravityFactor(id, 1.0f);
    }
    BodyLockWrite lock(physics.GetBodyLockInterface(), id);
    if (lock.Succeeded()) {
      MotionProperties *mp = lock.GetBody().GetMotionProperties();
      if (mp) {
        mp->SetLinearDamping(std::max(b.linear_damping, 0.0f));
        mp->SetAngularDamping(std::max(b.angular_damping, 0.0f));
      }
    }
  }
}

static bool ensure_world(Engine &eng, int n_bodies)
{
  if (eng.physics) {
    return true;
  }
  const uint max_bodies = uint(std::max(4096, n_bodies * 8));
  const uint max_pairs = uint(std::max(65536, n_bodies * 32));
  const uint max_contacts = uint(std::max(20480, n_bodies * 16));
  eng.physics = std::make_unique<PhysicsSystem>();
  eng.physics->Init(max_bodies, 0, max_pairs, max_contacts, eng.bp, eng.obj_vs_bp, eng.obj_vs_obj);
  JoltStepSettings dummy{};
  dummy.velocity_steps = 10;
  dummy.position_steps = 2;
  apply_physics_quality(*eng.physics, dummy, false);
  return true;
}

static bool rebuild(Engine &eng,
                    const JoltStepSettings &st,
                    int n_bodies,
                    const JoltBodyDesc *bodies,
                    int n_constraints,
                    const JoltConstraintDesc *constraints,
                    char *error,
                    int error_max)
{
  if (eng.physics) {
    eng.reset_bodies();
  }
  else {
    ensure_world(eng, n_bodies);
  }
  bool has_kinematic = false;
  for (int i = 0; i < n_bodies; i++) {
    if (bodies[i].motion_type == JOLT_MOTION_KINEMATIC) {
      has_kinematic = true;
      break;
    }
  }
  apply_physics_quality(*eng.physics, st, has_kinematic);
  eng.physics->SetGravity(v3(st.gravity));

  BodyInterface &bi = eng.physics->GetBodyInterface();
  eng.bodies.resize(uint(n_bodies));
  eng.names.resize(uint(n_bodies));
  eng.shape_fp.resize(uint(n_bodies));
  eng.graph_driven.resize(uint(n_bodies));

  const float kdt = st.dt > 1.0e-6f ? st.dt : (1.0f / 24.0f);
  for (int i = 0; i < n_bodies; i++) {
    const JoltBodyDesc &b = bodies[i];
    JoltBodyDesc spawn = b;
    const std::string key = key_of(b, i);
    const bool kinematic = b.motion_type == JOLT_MOTION_KINEMATIC;
    if (kinematic) {
      /* Sweep uses only this frame's geometry: prev_* attributes, else velocity. */
      RVec3 p0 = rv3(b.position);
      Quat q0 = quat(b.rotation);
      if (b.has_prev) {
        p0 = rv3(b.prev_position);
        q0 = quat(b.prev_rotation);
      }
      else {
        const Vec3 lv = v3(b.linear_velocity);
        const Vec3 av = v3(b.angular_velocity);
        if (lv.LengthSq() > 1.0e-12f) {
          p0 = rv3(b.position) - RVec3(lv * kdt);
        }
        const float ang = av.Length();
        if (ang > 1.0e-6f) {
          const Quat dq = Quat::sRotation(av / ang, ang * kdt);
          q0 = dq.Conjugated() * quat(b.rotation);
        }
      }
      spawn.position = {float(p0.GetX()), float(p0.GetY()), float(p0.GetZ())};
      spawn.rotation = jq(q0);
    }
    /* Always activate: inactive bodies do not receive gravity. */
    const EActivation act = EActivation::Activate;
    const BodyID id = spawn_body(eng, st, spawn, act);
    if (id.IsInvalid()) {
      if (error && error_max > 0) {
        std::snprintf(error, uint(error_max), "Jolt: out of bodies at index %d", i);
      }
      return false;
    }
    if (kinematic) {
      Quat q_dst = quat(b.rotation);
      const Quat q_cur = bi.GetRotation(id);
      if (q_cur.Dot(q_dst) < 0.0f) {
        q_dst = -q_dst;
      }
      bi.ActivateBody(id);
      bi.MoveKinematic(id, rv3(b.position), q_dst, kdt);
    }
    eng.bodies[uint(i)] = id;
    eng.names[uint(i)] = key;
    eng.shape_fp[uint(i)] = shape_fp_of(b);
    eng.graph_driven[uint(i)] = kinematic ? 1 : 0;
  }

  auto add_two = [&](const TwoBodyConstraintSettings &s, int a, int b) -> TwoBodyConstraint * {
    const BodyID ida = invalid_if_world(a, eng.bodies);
    const BodyID idb = invalid_if_world(b, eng.bodies);
    if (ida.IsInvalid() && idb.IsInvalid()) {
      return nullptr;
    }
    TwoBodyConstraint *c = bi.CreateConstraint(&s, ida, idb);
    if (c) {
      eng.physics->AddConstraint(c);
      eng.constraints.push_back(c);
    }
    return c;
  };

  for (int i = 0; i < n_constraints; i++) {
    const JoltConstraintDesc &d = constraints[i];
    const Vec3 axis = safe_axis(d.axis, Vec3::sAxisX());
    const Vec3 nrm = safe_axis(d.axis2, perp_axis(axis));
    const RVec3 p1 = rv3(d.anchor);
    const RVec3 p2 = (d.anchor2.x != 0 || d.anchor2.y != 0 || d.anchor2.z != 0) ? rv3(d.anchor2) :
                                                                                  p1;
    switch (d.type) {
      case JOLT_CONSTRAINT_FIXED: {
        FixedConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mAutoDetectPoint = true;
        add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_DISTANCE: {
        DistanceConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPoint1 = p1;
        s.mPoint2 = p2;
        if (d.min_limit >= 0.0f) {
          s.mMinDistance = d.min_limit;
        }
        if (d.max_limit >= 0.0f) {
          s.mMaxDistance = d.max_limit;
        }
        if (d.frequency > 0.0f) {
          s.mLimitsSpringSettings.mFrequency = d.frequency;
          s.mLimitsSpringSettings.mDamping = std::max(d.damping, 0.0f);
        }
        add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_POINT: {
        PointConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPoint1 = p1;
        s.mPoint2 = p1;
        add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_HINGE: {
        HingeConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPoint1 = p1;
        s.mPoint2 = p1;
        s.mHingeAxis1 = axis;
        s.mHingeAxis2 = axis;
        s.mNormalAxis1 = nrm;
        s.mNormalAxis2 = nrm;
        if (d.min_limit > -3.1415f || d.max_limit < 3.1415f) {
          s.mLimitsMin = std::min(d.min_limit, 0.0f);
          s.mLimitsMax = std::max(d.max_limit, 0.0f);
        }
        if (d.motor_state != JOLT_MOTOR_OFF) {
          s.mMotorSettings.SetForceLimit(d.motor_force > 0.0f ? d.motor_force : 1.0e6f);
        }
        TwoBodyConstraint *c = add_two(s, d.body_a, d.body_b);
        apply_motor(c, d.type, d);
        break;
      }
      case JOLT_CONSTRAINT_CONE: {
        ConeConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPoint1 = p1;
        s.mPoint2 = p1;
        s.mTwistAxis1 = axis;
        s.mTwistAxis2 = axis;
        s.mHalfConeAngle = d.max_limit > 0.0f ? d.max_limit : 0.5f;
        add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_SLIDER: {
        SliderConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.SetSliderAxis(axis);
        s.mPoint1 = p1;
        s.mPoint2 = p1;
        if (d.min_limit < 0.0f || d.max_limit > 0.0f) {
          s.mLimitsMin = std::min(d.min_limit, 0.0f);
          s.mLimitsMax = std::max(d.max_limit, 0.0f);
        }
        TwoBodyConstraint *c = add_two(s, d.body_a, d.body_b);
        apply_motor(c, d.type, d);
        break;
      }
      case JOLT_CONSTRAINT_SWING_TWIST: {
        SwingTwistConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPosition1 = p1;
        s.mPosition2 = p1;
        s.mTwistAxis1 = axis;
        s.mTwistAxis2 = axis;
        s.mPlaneAxis1 = nrm;
        s.mPlaneAxis2 = nrm;
        const float swing = d.max_limit > 0.0f ? d.max_limit : 0.5f;
        s.mNormalHalfConeAngle = swing;
        s.mPlaneHalfConeAngle = swing;
        s.mTwistMinAngle = d.min_limit2;
        s.mTwistMaxAngle = d.max_limit2 != 0.0f ? d.max_limit2 : swing;
        add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_SIX_DOF: {
        SixDOFConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPosition1 = p1;
        s.mPosition2 = p1;
        s.mAxisX1 = axis;
        s.mAxisY1 = nrm;
        s.mAxisX2 = axis;
        s.mAxisY2 = nrm;
        /* Free translation along axis (slider-like) unless limits given. */
        if (d.min_limit == 0.0f && d.max_limit == 0.0f && d.min_limit2 == 0.0f &&
            d.max_limit2 == 0.0f)
        {
          /* default: all free */
        }
        add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_PATH: {
        if (!d.path_xyz || d.npath < 2) {
          break;
        }
        Ref<PathConstraintPathHermite> path = new PathConstraintPathHermite();
        for (int k = 0; k < d.npath; k++) {
          const Vec3 pos(d.path_xyz[k * 3 + 0], d.path_xyz[k * 3 + 1], d.path_xyz[k * 3 + 2]);
          Vec3 tan = axis;
          if (k + 1 < d.npath) {
            tan = Vec3(d.path_xyz[(k + 1) * 3 + 0] - pos.GetX(),
                       d.path_xyz[(k + 1) * 3 + 1] - pos.GetY(),
                       d.path_xyz[(k + 1) * 3 + 2] - pos.GetZ());
            if (tan.LengthSq() < 1.0e-10f) {
              tan = axis;
            }
          }
          else if (k > 0) {
            tan = pos - Vec3(d.path_xyz[(k - 1) * 3 + 0],
                             d.path_xyz[(k - 1) * 3 + 1],
                             d.path_xyz[(k - 1) * 3 + 2]);
          }
          path->AddPoint(pos, tan, nrm);
        }
        PathConstraintSettings s;
        s.mPath = path;
        s.mPathPosition = Vec3::sZero();
        s.mPathRotation = Quat::sIdentity();
        s.mPathFraction = 0.0f;
        s.mRotationConstraintType = EPathRotationConstraintType::ConstrainAroundTangent;
        add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_GEAR: {
        HingeConstraintSettings h1;
        h1.mSpace = EConstraintSpace::WorldSpace;
        h1.mPoint1 = p1;
        h1.mHingeAxis1 = axis;
        h1.mHingeAxis2 = axis;
        h1.mNormalAxis1 = nrm;
        h1.mNormalAxis2 = nrm;
        HingeConstraintSettings h2 = h1;
        h2.mPoint1 = p2;
        h2.mHingeAxis1 = nrm;
        h2.mHingeAxis2 = nrm;
        h2.mNormalAxis1 = axis;
        h2.mNormalAxis2 = axis;
        TwoBodyConstraint *c1 = add_two(h1, d.body_a, -1);
        TwoBodyConstraint *c2 = add_two(h2, d.body_b, -1);
        GearConstraintSettings g;
        g.mSpace = EConstraintSpace::WorldSpace;
        g.mHingeAxis1 = axis;
        g.mHingeAxis2 = nrm;
        g.mRatio = d.ratio != 0.0f ? d.ratio : 1.0f;
        TwoBodyConstraint *cg = add_two(g, d.body_a, d.body_b);
        if (cg && c1 && c2) {
          static_cast<GearConstraint *>(cg)->SetConstraints(c1, c2);
        }
        break;
      }
      case JOLT_CONSTRAINT_RACK_AND_PINION: {
        HingeConstraintSettings h;
        h.mSpace = EConstraintSpace::WorldSpace;
        h.mPoint1 = p1;
        h.mHingeAxis1 = axis;
        h.mHingeAxis2 = axis;
        h.mNormalAxis1 = nrm;
        h.mNormalAxis2 = nrm;
        SliderConstraintSettings sl;
        sl.mSpace = EConstraintSpace::WorldSpace;
        sl.SetSliderAxis(nrm);
        sl.mPoint1 = p2;
        TwoBodyConstraint *ch = add_two(h, d.body_a, -1);
        TwoBodyConstraint *cs = add_two(sl, d.body_b, -1);
        RackAndPinionConstraintSettings rp;
        rp.mSpace = EConstraintSpace::WorldSpace;
        rp.mHingeAxis = axis;
        rp.mSliderAxis = nrm;
        rp.mRatio = d.ratio != 0.0f ? d.ratio : 1.0f;
        TwoBodyConstraint *cr = add_two(rp, d.body_a, d.body_b);
        if (cr && ch && cs) {
          static_cast<RackAndPinionConstraint *>(cr)->SetConstraints(ch, cs);
        }
        break;
      }
      case JOLT_CONSTRAINT_PULLEY: {
        PulleyConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mBodyPoint1 = p1;
        s.mBodyPoint2 = p2;
        s.mFixedPoint1 = p1 + RVec3(axis.GetX(), axis.GetY(), axis.GetZ());
        s.mFixedPoint2 = p2 + RVec3(nrm.GetX(), nrm.GetY(), nrm.GetZ());
        s.mRatio = d.ratio != 0.0f ? d.ratio : 1.0f;
        if (d.max_limit >= 0.0f) {
          s.mMaxLength = d.max_limit;
        }
        add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_VEHICLE: {
        const BodyID chassis = invalid_if_world(d.body_a, eng.bodies);
        if (chassis.IsInvalid()) {
          break;
        }
        BodyLockWrite lock(eng.physics->GetBodyLockInterface(), chassis);
        if (!lock.Succeeded()) {
          break;
        }
        const float wr = d.wheel_radius > 0.0f ? d.wheel_radius : 0.3f;
        const float ww = d.wheel_width > 0.0f ? d.wheel_width : 0.1f;
        const float smin = d.suspension_min > 0.0f ? d.suspension_min : 0.2f;
        const float smax = d.suspension_max > smin ? d.suspension_max : smin + 0.2f;
        const int nw = (d.num_wheels >= 2 && d.num_wheels <= 4) ? d.num_wheels : 4;
        const JoltBodyDesc &chassis_b = bodies[d.body_a];
        const float hx = std::max(chassis_b.shape_size.x, wr * 2.0f);
        const float hy = std::max(chassis_b.shape_size.y, wr * 2.0f);
        const float hz = std::max(chassis_b.shape_size.z, wr);
        VehicleConstraintSettings vs;
        vs.mUp = Vec3(0, 0, 1);
        vs.mForward = Vec3(0, 1, 0);
        WheeledVehicleControllerSettings *ctrl = new WheeledVehicleControllerSettings();
        if (d.motor_force > 0.0f) {
          ctrl->mEngine.mMaxTorque = d.motor_force;
        }
        vs.mController = ctrl;
        auto add_wheel = [&](Vec3Arg pos, bool steer) {
          WheelSettingsWV *w = new WheelSettingsWV();
          w->mPosition = pos;
          w->mSuspensionDirection = Vec3(0, 0, -1);
          w->mSteeringAxis = Vec3(0, 0, 1);
          w->mWheelUp = Vec3(0, 0, 1);
          w->mWheelForward = Vec3(0, 1, 0);
          w->mSuspensionMinLength = smin;
          w->mSuspensionMaxLength = smax;
          w->mRadius = wr;
          w->mWidth = ww;
          w->mMaxSteerAngle = steer ? (d.max_steer > 0.0f ? d.max_steer : DegreesToRadians(35.0f)) :
                                      0.0f;
          vs.mWheels.push_back(w);
        };
        if (nw == 2) {
          add_wheel(Vec3(0, hy * 0.6f, -hz), true);
          add_wheel(Vec3(0, -hy * 0.6f, -hz), false);
        }
        else {
          add_wheel(Vec3(hx * 0.7f, hy * 0.7f, -hz), true);
          add_wheel(Vec3(-hx * 0.7f, hy * 0.7f, -hz), true);
          add_wheel(Vec3(hx * 0.7f, -hy * 0.7f, -hz), false);
          add_wheel(Vec3(-hx * 0.7f, -hy * 0.7f, -hz), false);
        }
        Ref<VehicleConstraint> vc = new VehicleConstraint(lock.GetBody(), vs);
        vc->SetVehicleCollisionTester(new VehicleCollisionTesterRay(LAYER_MOVING));
        eng.physics->AddConstraint(vc);
        eng.physics->AddStepListener(vc);
        eng.vehicles.push_back(vc);
        if (d.motor_state == JOLT_MOTOR_VELOCITY) {
          if (WheeledVehicleController *c = static_cast<WheeledVehicleController *>(
                  vc->GetController()))
          {
            c->SetDriverInput(std::clamp(d.motor_target, -1.0f, 1.0f), 0.0f, 0.0f, 0.0f);
          }
        }
        break;
      }
      default:
        break;
    }
  }

  eng.physics->OptimizeBroadPhase();
  eng.n_bodies = n_bodies;
  eng.n_constraints = n_constraints;
  eng.topology = st.topology_hash;
  eng.constraint_fp = constraints_fp(n_constraints, constraints);
  (void)error;
  (void)error_max;
  return true;
}

static void destroy_slot(Engine &eng, const BodyID id)
{
  if (id.IsInvalid() || !eng.physics) {
    return;
  }
  BodyInterface &bi = eng.physics->GetBodyInterface();
  if (bi.IsAdded(id)) {
    bi.RemoveBody(id);
  }
  bi.DestroyBody(id);
}

/** Keep Jolt bodies that still exist (by name). Add/remove without wiping the world. */
static bool sync_world(Engine &eng,
                       const JoltStepSettings &st,
                       int n_bodies,
                       const JoltBodyDesc *bodies,
                       int n_constraints,
                       const JoltConstraintDesc *constraints,
                       char *error,
                       int error_max)
{
  if (!eng.physics) {
    return rebuild(eng, st, n_bodies, bodies, n_constraints, constraints, error, error_max);
  }

  bool has_kinematic = false;
  for (int i = 0; i < n_bodies; i++) {
    if (bodies[i].motion_type == JOLT_MOTION_KINEMATIC) {
      has_kinematic = true;
      break;
    }
  }
  apply_physics_quality(*eng.physics, st, has_kinematic);
  eng.physics->SetGravity(v3(st.gravity));

  BodyInterface &bi = eng.physics->GetBodyInterface();
  std::unordered_map<std::string, int> old_at;
  old_at.reserve(uint(eng.n_bodies) * 2 + 1);
  for (int i = 0; i < eng.n_bodies; i++) {
    old_at.emplace(eng.names[uint(i)], i);
  }

  std::vector<char> consumed(size_t(std::max(eng.n_bodies, 0)), 0);
  std::vector<BodyID> next_ids;
  std::vector<std::string> next_names;
  std::vector<uint64_t> next_fp;
  std::vector<int> next_gd;
  std::vector<char> spawned;
  std::vector<char> shape_dirty;
  next_ids.resize(size_t(n_bodies));
  next_names.resize(size_t(n_bodies));
  next_fp.resize(size_t(n_bodies));
  next_gd.resize(size_t(n_bodies));
  spawned.assign(size_t(n_bodies), 0);
  shape_dirty.assign(size_t(n_bodies), 0);

  for (int i = 0; i < n_bodies; i++) {
    const JoltBodyDesc &b = bodies[i];
    next_names[uint(i)] = key_of(b, i);
    next_fp[uint(i)] = shape_fp_of(b);
    next_gd[uint(i)] = (motion_from_int(b.motion_type) == EMotionType::Kinematic) ? 1 : 0;
    const auto it = old_at.find(next_names[uint(i)]);
    /* Reuse the same BodyID even if size/scale changed — SetShape below. */
    if (it == old_at.end() || consumed[uint(it->second)]) {
      spawned[uint(i)] = 1;
      continue;
    }
    const BodyID id = eng.bodies[uint(it->second)];
    if (id.IsInvalid() || !bi.IsAdded(id) ||
        bi.GetMotionType(id) != motion_from_int(b.motion_type))
    {
      /* Leave unconsumed so the old slot is destroyed; spawn a replacement. */
      spawned[uint(i)] = 1;
      continue;
    }
    consumed[uint(it->second)] = 1;
    next_ids[uint(i)] = id;
    spawned[uint(i)] = 0;
    if (eng.shape_fp[uint(it->second)] != next_fp[uint(i)]) {
      shape_dirty[uint(i)] = 1;
    }
  }

  /* Animated kinematics are often re-joined every frame with a new idx name.
   * Reuse leftover kinematic/static bodies instead of destroy+create (flash). */
  for (int i = 0; i < n_bodies; i++) {
    if (!spawned[uint(i)]) {
      continue;
    }
    if (bodies[i].motion_type == JOLT_MOTION_DYNAMIC) {
      continue;
    }
    for (int j = 0; j < eng.n_bodies; j++) {
      if (consumed[uint(j)]) {
        continue;
      }
      /* Same motion class (static vs kinematic). Size is applied via SetShape. */
      if (bi.GetMotionType(eng.bodies[uint(j)]) != motion_from_int(bodies[i].motion_type)) {
        continue;
      }
      consumed[uint(j)] = 1;
      spawned[uint(i)] = 0;
      next_ids[uint(i)] = eng.bodies[uint(j)];
      if (eng.shape_fp[uint(j)] != next_fp[uint(i)]) {
        shape_dirty[uint(i)] = 1;
      }
      break;
    }
  }

  /* Graph rest pose is re-fed every frame. Comparing it to the cached world
   * made dynamics snap back after falling 50 cm. Only rewind is a real reset. */
  const bool timeline_reset = st.scene_time + 1.0e-3f < eng.last_scene_time;
  eng.last_scene_time = st.scene_time;

  bool added = false;
  bool removed = false;
  for (int i = 0; i < n_bodies; i++) {
    const JoltBodyDesc &b = bodies[i];
    if (spawned[uint(i)]) {
      JoltBodyDesc spawn = b;
      if (b.motion_type == JOLT_MOTION_KINEMATIC) {
        const float kdt = st.dt > 1.0e-6f ? st.dt : (1.0f / 24.0f);
        RVec3 p0 = rv3(b.position);
        Quat q0 = quat(b.rotation);
        if (b.has_prev) {
          p0 = rv3(b.prev_position);
          q0 = quat(b.prev_rotation);
        }
        spawn.position = {float(p0.GetX()), float(p0.GetY()), float(p0.GetZ())};
        spawn.rotation = jq(q0);
        const BodyID id = spawn_body(eng, st, spawn, EActivation::Activate);
        if (id.IsInvalid()) {
          if (error && error_max > 0) {
            std::snprintf(error, uint(error_max), "Jolt: out of bodies at index %d", i);
          }
          return false;
        }
        Quat q_dst = quat(b.rotation);
        const Quat q_cur = bi.GetRotation(id);
        if (q_cur.Dot(q_dst) < 0.0f) {
          q_dst = -q_dst;
        }
        bi.MoveKinematic(id, rv3(b.position), q_dst, kdt);
        next_ids[uint(i)] = id;
        added = true;
        continue;
      }
      const BodyID id = spawn_body(eng, st, spawn, EActivation::Activate);
      if (id.IsInvalid()) {
        if (error && error_max > 0) {
          std::snprintf(error, uint(error_max), "Jolt: out of bodies at index %d", i);
        }
        return false;
      }
      next_ids[uint(i)] = id;
      added = true;
      continue;
    }
    const BodyID id = next_ids[uint(i)];
    refresh_existing_body(*eng.physics, st, b, id, shape_dirty[uint(i)] != 0);
    if (b.motion_type == JOLT_MOTION_KINEMATIC) {
      Quat q_dst = quat(b.rotation);
      const Quat q_cur = bi.GetRotation(id);
      if (q_cur.Dot(q_dst) < 0.0f) {
        q_dst = -q_dst;
      }
      const RVec3 p_cur = bi.GetPosition(id);
      const RVec3 p_dst = rv3(b.position);
      const float dp2 = float((p_dst - p_cur).LengthSq());
      const float qdot = q_cur.Dot(q_dst);
      bi.ActivateBody(id);
      /* Teleport only on a real pop. Always Activate so stacked cubes wake. */
      if (dp2 > 4.0f || qdot < 0.1f) {
        bi.SetPositionAndRotation(id, p_dst, q_dst, EActivation::Activate);
      }
      else {
        const float kdt = st.dt > 1.0e-6f ? st.dt : (1.0f / 24.0f);
        bi.MoveKinematic(id, p_dst, q_dst, kdt);
      }
    }
    else if (b.motion_type == JOLT_MOTION_STATIC) {
      bi.SetPositionAndRotationWhenChanged(
          id, rv3(b.position), quat(b.rotation), EActivation::Activate);
    }
    else if (timeline_reset) {
      bi.SetPositionAndRotation(id, rv3(b.position), quat(b.rotation), EActivation::Activate);
      bi.SetLinearVelocity(id, v3(b.linear_velocity));
      bi.SetAngularVelocity(id, v3(b.angular_velocity));
    }
  }

  for (int i = 0; i < eng.n_bodies; i++) {
    if (!consumed[uint(i)]) {
      destroy_slot(eng, eng.bodies[uint(i)]);
      removed = true;
    }
  }

  eng.bodies = std::move(next_ids);
  eng.names = std::move(next_names);
  eng.shape_fp = std::move(next_fp);
  eng.graph_driven = std::move(next_gd);
  eng.n_bodies = n_bodies;

  const uint64_t cfp = constraints_fp(n_constraints, constraints);
  if (removed || cfp != eng.constraint_fp || n_constraints != eng.n_constraints) {
    /* Reuse rebuild's constraint path by temporarily running only constraints.
     * Clear then call rebuild's add via a nested rebuild of constraints only:
     * we fall through to a light rebuild_constraints by invoking rebuild on a
     * cloned engine is too heavy — duplicate the Optimize + fp store here and
     * re-run constraint creation through rebuild() would wipe bodies.
     * Instead mark constraint_fp dirty and call rebuild() only for constraints
     * by extracting... we inline a call to the existing rebuild constraint
     * block via a full rebuild IF constraints changed AND bodies were removed.
     * If only added bodies and constraints unchanged, skip. */
    if (removed || cfp != eng.constraint_fp) {
      eng.clear_constraints_only();
      /* Recreate constraints using rebuild() would destroy bodies. Call the
       * constraint loop by running a local copy: jump into rebuild is wrong.
       * Use rebuild_constraints pattern: temporarily store bodies and call
       * the remainder of rebuild — implemented as constraint_fp mismatch
       * triggering add via the same rebuild function's second half.
       * Simpler: call rebuild() only when constraints change; body sync
       * already happened. That would wipe bodies. So we need add_constraints.
       * Fallback: if constraints changed, rebuild entire world (rare). */
      if (n_constraints > 0 || eng.n_constraints > 0) {
        /* Keep dynamic state: snapshot, rebuild, restore. */
        std::vector<RVec3> pos;
        std::vector<Quat> rot;
        std::vector<Vec3> lv;
        std::vector<Vec3> av;
        std::vector<EMotionType> mt;
        pos.resize(size_t(n_bodies));
        rot.resize(size_t(n_bodies));
        lv.resize(size_t(n_bodies));
        av.resize(size_t(n_bodies));
        mt.resize(size_t(n_bodies));
        for (int i = 0; i < n_bodies; i++) {
          const BodyID id = eng.bodies[uint(i)];
          pos[uint(i)] = bi.GetPosition(id);
          rot[uint(i)] = bi.GetRotation(id);
          mt[uint(i)] = bi.GetMotionType(id);
          if (mt[uint(i)] != EMotionType::Static) {
            lv[uint(i)] = bi.GetLinearVelocity(id);
            av[uint(i)] = bi.GetAngularVelocity(id);
          }
        }
        if (!rebuild(eng, st, n_bodies, bodies, n_constraints, constraints, error, error_max)) {
          return false;
        }
        BodyInterface &bi2 = eng.physics->GetBodyInterface();
        for (int i = 0; i < n_bodies; i++) {
          const BodyID id = eng.bodies[uint(i)];
          if (mt[uint(i)] == EMotionType::Dynamic) {
            bi2.SetPositionAndRotation(id, pos[uint(i)], rot[uint(i)], EActivation::Activate);
            bi2.SetLinearVelocity(id, lv[uint(i)]);
            bi2.SetAngularVelocity(id, av[uint(i)]);
            bi2.ActivateBody(id);
          }
        }
        return true;
      }
    }
  }
  else if (added) {
    eng.physics->OptimizeBroadPhase();
  }

  eng.n_constraints = n_constraints;
  eng.topology = st.topology_hash;
  (void)added;
  return true;
}

static void write_states(
    Engine &eng, int n, const JoltBodyDesc *bodies, JoltBodyState *states, float dt)
{
  BodyInterface &bi = eng.physics->GetBodyInterface();
  const float inv_dt = dt > 1.0e-8f ? (1.0f / dt) : 0.0f;
  eng.last_pos.resize(uint(n));
  for (int i = 0; i < n; i++) {
    const BodyID id = eng.bodies[uint(i)];
    const RVec3 p = bi.GetPosition(id);
    const Quat r = bi.GetRotation(id);
    states[i].position = {float(p.GetX()), float(p.GetY()), float(p.GetZ())};
    states[i].rotation = jq(r);
    states[i].mass = bodies[i].mass;
    states[i].inertia = bodies[i].inertia;
    states[i].linear_acceleration = {0.0f, 0.0f, 0.0f};
    states[i].angular_acceleration = {0.0f, 0.0f, 0.0f};
    const EMotionType mt = bi.GetMotionType(id);
    if (mt == EMotionType::Static) {
      states[i].linear_velocity = {0.0f, 0.0f, 0.0f};
      states[i].angular_velocity = {0.0f, 0.0f, 0.0f};
    }
    else {
      states[i].linear_velocity = jv(bi.GetLinearVelocity(id));
      states[i].angular_velocity = jv(bi.GetAngularVelocity(id));
      if (inv_dt > 0.0f) {
        states[i].linear_acceleration = {
            (states[i].linear_velocity.x - bodies[i].linear_velocity.x) * inv_dt,
            (states[i].linear_velocity.y - bodies[i].linear_velocity.y) * inv_dt,
            (states[i].linear_velocity.z - bodies[i].linear_velocity.z) * inv_dt,
        };
        states[i].angular_acceleration = {
            (states[i].angular_velocity.x - bodies[i].angular_velocity.x) * inv_dt,
            (states[i].angular_velocity.y - bodies[i].angular_velocity.y) * inv_dt,
            (states[i].angular_velocity.z - bodies[i].angular_velocity.z) * inv_dt,
        };
      }
    }
    if (mt == EMotionType::Dynamic) {
      BodyLockRead lock(eng.physics->GetBodyLockInterface(), id);
      if (lock.Succeeded()) {
        const MotionProperties *mp = lock.GetBody().GetMotionProperties();
        if (mp != nullptr) {
          const float inv_m = mp->GetInverseMassUnchecked();
          if (inv_m > 1.0e-12f) {
            states[i].mass = 1.0f / inv_m;
          }
          const Vec3 inv_i = mp->GetInverseInertiaDiagonal();
          states[i].inertia = {
              inv_i.GetX() > 1.0e-12f ? (1.0f / inv_i.GetX()) : states[i].inertia.x,
              inv_i.GetY() > 1.0e-12f ? (1.0f / inv_i.GetY()) : states[i].inertia.y,
              inv_i.GetZ() > 1.0e-12f ? (1.0f / inv_i.GetZ()) : states[i].inertia.z,
          };
        }
      }
    }
    eng.last_pos[uint(i)] = states[i].position;
  }
}

}  // namespace

void JOLT_cache_invalidate(const char *cache_key)
{
  ensure_jolt();
  std::lock_guard<std::mutex> lock(g_mu);
  if (!cache_key || cache_key[0] == '\0') {
    g_engines.clear();
    return;
  }
  g_engines.erase(cache_key);
}

int JOLT_step(const JoltStepSettings *settings,
              int n_bodies,
              const JoltBodyDesc *bodies,
              JoltBodyState *states,
              int n_constraints,
              const JoltConstraintDesc *constraints,
              char *error,
              int error_max)
{
  ensure_jolt();
  if (!settings || n_bodies < 0 || (n_bodies > 0 && (!bodies || !states))) {
    if (error && error_max > 0) {
      std::snprintf(error, uint(error_max), "Jolt: invalid arguments");
    }
    return 0;
  }
  if (settings->dt <= 0.0f) {
    /* Viewport / frame-1 evals use dt=0. Do not destroy the cached world —
     * that made gravity look dead (next real step spawned from rest). */
    std::lock_guard<std::mutex> lock(g_mu);
    const std::string key = (settings->cache_key && settings->cache_key[0]) ?
                                settings->cache_key :
                                std::string("_");
    const auto it = g_engines.find(key);
    if (it != g_engines.end() && it->second && it->second->physics &&
        int(it->second->bodies.size()) == n_bodies)
    {
      write_states(*it->second, n_bodies, bodies, states, 0.0f);
      return 1;
    }
    for (int i = 0; i < n_bodies; i++) {
      states[i].position = bodies[i].position;
      states[i].rotation = bodies[i].rotation;
      states[i].linear_velocity = bodies[i].linear_velocity;
      states[i].angular_velocity = bodies[i].angular_velocity;
      states[i].mass = bodies[i].mass;
      states[i].inertia = bodies[i].inertia;
      states[i].linear_acceleration = {0.0f, 0.0f, 0.0f};
      states[i].angular_acceleration = {0.0f, 0.0f, 0.0f};
    }
    return 1;
  }

  std::lock_guard<std::mutex> lock(g_mu);
  const std::string key = (settings->cache_key && settings->cache_key[0]) ? settings->cache_key :
                                                                            std::string("_");
  std::unique_ptr<Engine> &slot = g_engines[key];
  if (!slot) {
    slot = std::make_unique<Engine>();
  }
  Engine &eng = *slot;

  if (!sync_world(eng,
                  *settings,
                  n_bodies,
                  bodies,
                  n_constraints,
                  constraints,
                  error,
                  error_max))
  {
    eng.clear();
    return 0;
  }

  const int steps = std::max(1, settings->collision_steps);
  const float h = settings->dt / float(steps);
  for (int s = 0; s < steps; s++) {
    eng.physics->Update(h, 1, g_temp.get(), g_jobs.get());
  }

  write_states(eng, n_bodies, bodies, states, settings->dt);
  return 1;
}
