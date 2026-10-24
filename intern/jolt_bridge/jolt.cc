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
  if (!c) {
    return;
  }
  const EMotorState state = (d.motor_state == JOLT_MOTOR_POSITION) ? EMotorState::Position :
                            (d.motor_state == JOLT_MOTOR_VELOCITY) ? EMotorState::Velocity :
                                                                     EMotorState::Off;
  const float force = d.motor_force > 0.0f ? d.motor_force : 1.0e6f;
  switch (type) {
    case JOLT_CONSTRAINT_HINGE: {
      auto *h = static_cast<HingeConstraint *>(c);
      /* Hinge motors are angular: the limit is a torque. */
      h->GetMotorSettings().SetTorqueLimit(force);
      h->SetMotorState(state);
      if (state == EMotorState::Velocity) {
        h->SetTargetAngularVelocity(d.motor_target);
      }
      else if (state == EMotorState::Position) {
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
      else if (state == EMotorState::Position) {
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
  /** Motion type requested by the graph (Jolt may have forced a mesh shape to static). */
  std::vector<int> req_motion;
  /** Pose / velocity the last step handed back to Geometry Nodes, per body slot. */
  std::vector<JoltVec3> out_pos;
  std::vector<JoltQuat> out_rot;
  std::vector<JoltVec3> out_lv;
  std::vector<JoltVec3> out_av;
  std::vector<char> has_out;
  std::vector<Ref<Constraint>> constraints;
  std::vector<Ref<VehicleConstraint>> vehicles;
  /** Main constraint per input constraint index, for motor updates. */
  std::vector<Ref<Constraint>> constraint_by_index;
  std::vector<Ref<VehicleConstraint>> vehicle_by_index;
  /**
   * Body A / B world transforms when a constraint was first created. Constraint anchors are
   * given in that (rest) space, so re-creating a constraint after bodies moved maps them along.
   */
  std::unordered_map<std::string, std::pair<RMat44, RMat44>> constraint_ref;
  uint64 topology = 0;
  uint64 constraint_fp = 0;
  int n_bodies = 0;
  int n_constraints = 0;
  float last_scene_time = -1.0e30f;
  uint64 last_use = 0;

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
    constraint_by_index.clear();
    vehicle_by_index.clear();
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
    req_motion.clear();
    out_pos.clear();
    out_rot.clear();
    out_lv.clear();
    out_av.clear();
    has_out.clear();
    n_bodies = 0;
    n_constraints = 0;
    topology = 0;
    last_scene_time = -1.0e30f;
  }

  void clear()
  {
    reset_bodies();
    constraint_ref.clear();
    physics.reset();
  }

  ~Engine()
  {
    clear();
  }
};

static std::unordered_map<std::string, std::unique_ptr<Engine>> g_engines;
static uint64 g_use_counter = 0;
/** Worlds kept alive at once (one per solver node and evaluation context). */
constexpr size_t k_max_engines = 32;

static void fill_mass_if_mesh(BodyCreationSettings &settings, const JoltBodyDesc &b, const Shape *shape)
{
  if (b.shape_type != JOLT_SHAPE_MESH && b.shape_type != JOLT_SHAPE_HEIGHTFIELD &&
      b.shape_type != JOLT_SHAPE_PLANE)
  {
    return;
  }
  if (!settings.HasMassProperties()) {
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
  /* Also for static / kinematic bodies that may become dynamic later. */
  if (!settings.HasMassProperties()) {
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
  uint64_t h = uint64_t(uint32_t(b.shape_type));
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

static uint64_t mix_vec(uint64_t h, const JoltVec3 &v)
{
  h = mix64(h, quant_mm(v.x));
  h = mix64(h, quant_mm(v.y));
  return mix64(h, quant_mm(v.z));
}

/** Everything that is baked into a constraint at creation. Motor values are applied live. */
static uint64_t constraints_fp(int n, const JoltConstraintDesc *c)
{
  uint64_t h = uint64_t(uint32_t(n));
  for (int i = 0; i < n; i++) {
    const JoltConstraintDesc &d = c[i];
    h = mix64(h, uint64_t(uint32_t(d.type)));
    h = mix64(h, uint64_t(uint32_t(d.body_a)));
    h = mix64(h, uint64_t(uint32_t(d.body_b)));
    h = mix64(h, uint64_t(uint32_t(d.npath)));
    h = mix_vec(h, d.anchor);
    h = mix_vec(h, d.anchor2);
    h = mix_vec(h, d.axis);
    h = mix_vec(h, d.axis2);
    h = mix64(h, quant_mm(d.min_limit));
    h = mix64(h, quant_mm(d.max_limit));
    h = mix64(h, quant_mm(d.min_limit2));
    h = mix64(h, quant_mm(d.max_limit2));
    h = mix64(h, quant_mm(d.ratio));
    h = mix64(h, quant_mm(d.frequency));
    h = mix64(h, quant_mm(d.damping));
    h = mix64(h, quant_mm(d.wheel_radius));
    h = mix64(h, quant_mm(d.wheel_width));
    h = mix64(h, quant_mm(d.suspension_min));
    h = mix64(h, quant_mm(d.suspension_max));
    h = mix64(h, uint64_t(uint32_t(d.num_wheels)));
    if (d.path_xyz && d.npath > 0) {
      h = mix64(h, quant_mm(d.path_xyz[0]));
      h = mix64(h, quant_mm(d.path_xyz[d.npath * 3 - 1]));
    }
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

static ObjectLayer layer_for(const EMotionType motion)
{
  return (motion == EMotionType::Static) ? LAYER_NON_MOVING : LAYER_MOVING;
}

static BodyID spawn_body(Engine &eng,
                         const JoltStepSettings &st,
                         const JoltBodyDesc &b,
                         EActivation activation)
{
  EMotionType motion = motion_from_int(b.motion_type);
  ShapeRefC shape = shape_for_desc(b, motion);
  BodyCreationSettings settings(
      shape.GetPtr(), rv3(b.position), quat(b.rotation), motion, layer_for(motion));
  /* Lets the graph switch Static / Kinematic / Dynamic later without a new BodyID. */
  settings.mAllowDynamicOrKinematic = !shape->MustBeStatic();
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

static RMat44 body_world_tf(BodyInterface &bi, const BodyID id)
{
  return id.IsInvalid() ? RMat44::sIdentity() : bi.GetWorldTransform(id);
}

static std::string constraint_key(const Engine &eng,
                                  const JoltConstraintDesc &d,
                                  std::unordered_map<std::string, int> &seen)
{
  auto name = [&](const int idx) -> std::string {
    return (idx >= 0 && idx < int(eng.names.size())) ? eng.names[uint(idx)] :
                                                        std::string("<world>");
  };
  const std::string k = std::to_string(d.type) + "|" + name(d.body_a) + "|" + name(d.body_b);
  const int n = seen[k]++;
  return k + "|" + std::to_string(n);
}

static bool has_range(const float min, const float max)
{
  return max > min;
}

/**
 * Create all constraints. Anchors and axes are world space in the pose the bodies had when a
 * constraint was first seen (normally the rest pose); `constraint_ref` maps them onto the
 * current poses so re-creating constraints mid-simulation does not yank bodies.
 */
static void add_constraints(Engine &eng,
                            const int n_bodies,
                            const JoltBodyDesc *bodies,
                            const int n_constraints,
                            const JoltConstraintDesc *constraints)
{
  BodyInterface &bi = eng.physics->GetBodyInterface();
  eng.constraint_by_index.assign(size_t(std::max(n_constraints, 0)), Ref<Constraint>());
  eng.vehicle_by_index.assign(size_t(std::max(n_constraints, 0)), Ref<VehicleConstraint>());
  std::unordered_map<std::string, int> seen;

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
    const BodyID ida = invalid_if_world(d.body_a, eng.bodies);
    const BodyID idb = invalid_if_world(d.body_b, eng.bodies);
    const RMat44 ta = body_world_tf(bi, ida);
    const RMat44 tb = body_world_tf(bi, idb);
    const std::string key = constraint_key(eng, d, seen);
    auto ref_it = eng.constraint_ref.find(key);
    if (ref_it == eng.constraint_ref.end()) {
      ref_it = eng.constraint_ref.emplace(key, std::make_pair(ta, tb)).first;
    }
    const RMat44 ta0 = ref_it->second.first;
    const RMat44 tb0 = ref_it->second.second;
    const RMat44 xa = ta * ta0.InversedRotationTranslation();
    const RMat44 xb = tb * tb0.InversedRotationTranslation();
    auto PA = [&](RVec3Arg p) { return xa * p; };
    auto PB = [&](RVec3Arg p) { return xb * p; };
    auto VA = [&](Vec3Arg v) { return xa.Multiply3x3(v).Normalized(); };
    auto VB = [&](Vec3Arg v) { return xb.Multiply3x3(v).Normalized(); };

    const Vec3 axis = safe_axis(d.axis, Vec3::sAxisX());
    Vec3 nrm = safe_axis(d.axis2, perp_axis(axis));
    /* Hinge / slider / twist frames need a normal perpendicular to the axis. */
    nrm = nrm - axis * axis.Dot(nrm);
    nrm = nrm.LengthSq() > 1.0e-8f ? nrm.Normalized() : perp_axis(axis);
    const RVec3 p1 = rv3(d.anchor);
    const bool has_anchor2 = d.anchor2.x != 0 || d.anchor2.y != 0 || d.anchor2.z != 0;
    const RVec3 p2 = has_anchor2 ? rv3(d.anchor2) : p1;
    TwoBodyConstraint *main = nullptr;
    switch (d.type) {
      case JOLT_CONSTRAINT_FIXED: {
        FixedConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mAutoDetectPoint = true;
        main = add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_DISTANCE: {
        DistanceConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        /* No Anchor 2: rod from Body A's origin to Anchor (on Body B or the world).
         * With Anchor 2: from Anchor (on A) to Anchor 2 (on B / world). */
        if (has_anchor2) {
          s.mPoint1 = PA(p1);
          s.mPoint2 = PB(p2);
        }
        else {
          s.mPoint1 = PA(ta0.GetTranslation());
          s.mPoint2 = PB(p1);
        }
        /* Max > 0: rope up to Max (Min..Max). Only Min: rod of that length.
         * Both 0: rod at the current distance (-1 = Jolt auto). */
        if (d.max_limit > 0.0f) {
          s.mMaxDistance = d.max_limit;
          s.mMinDistance = std::clamp(d.min_limit, 0.0f, d.max_limit);
        }
        else if (d.min_limit > 0.0f) {
          s.mMinDistance = d.min_limit;
          s.mMaxDistance = d.min_limit;
        }
        if (d.frequency > 0.0f) {
          s.mLimitsSpringSettings.mFrequency = d.frequency;
          s.mLimitsSpringSettings.mDamping = std::max(d.damping, 0.0f);
        }
        main = add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_POINT: {
        PointConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPoint1 = PA(p1);
        s.mPoint2 = PB(p1);
        main = add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_HINGE: {
        HingeConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPoint1 = PA(p1);
        s.mPoint2 = PB(p1);
        s.mHingeAxis1 = VA(axis);
        s.mHingeAxis2 = VB(axis);
        s.mNormalAxis1 = VA(nrm);
        s.mNormalAxis2 = VB(nrm);
        /* Min == Max (default 0, 0) = free swing. */
        if (has_range(d.min_limit, d.max_limit)) {
          s.mLimitsMin = std::clamp(d.min_limit, -JPH_PI, 0.0f);
          s.mLimitsMax = std::clamp(d.max_limit, 0.0f, JPH_PI);
        }
        main = add_two(s, d.body_a, d.body_b);
        apply_motor(main, d.type, d);
        break;
      }
      case JOLT_CONSTRAINT_CONE: {
        ConeConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPoint1 = PA(p1);
        s.mPoint2 = PB(p1);
        s.mTwistAxis1 = VA(axis);
        s.mTwistAxis2 = VB(axis);
        s.mHalfConeAngle = d.max_limit > 0.0f ? d.max_limit : 0.5f;
        main = add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_SLIDER: {
        SliderConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPoint1 = PA(p1);
        s.mPoint2 = PB(p1);
        const Vec3 snrm = perp_axis(axis);
        s.mSliderAxis1 = VA(axis);
        s.mSliderAxis2 = VB(axis);
        s.mNormalAxis1 = VA(snrm);
        s.mNormalAxis2 = VB(snrm);
        if (has_range(d.min_limit, d.max_limit)) {
          s.mLimitsMin = std::min(d.min_limit, 0.0f);
          s.mLimitsMax = std::max(d.max_limit, 0.0f);
        }
        main = add_two(s, d.body_a, d.body_b);
        apply_motor(main, d.type, d);
        break;
      }
      case JOLT_CONSTRAINT_SWING_TWIST: {
        SwingTwistConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPosition1 = PA(p1);
        s.mPosition2 = PB(p1);
        s.mTwistAxis1 = VA(axis);
        s.mTwistAxis2 = VB(axis);
        s.mPlaneAxis1 = VA(nrm);
        s.mPlaneAxis2 = VB(nrm);
        const float swing = d.max_limit > 0.0f ? std::min(d.max_limit, JPH_PI) : 0.5f;
        s.mNormalHalfConeAngle = swing;
        s.mPlaneHalfConeAngle = swing;
        if (has_range(d.min_limit2, d.max_limit2)) {
          s.mTwistMinAngle = std::clamp(d.min_limit2, -JPH_PI, JPH_PI);
          s.mTwistMaxAngle = std::clamp(d.max_limit2, -JPH_PI, JPH_PI);
        }
        else {
          s.mTwistMinAngle = -JPH_PI;
          s.mTwistMaxAngle = JPH_PI;
        }
        main = add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_SIX_DOF: {
        /* Moves along and turns about Axis (Min/Max = travel, Min/Max 2 = angle, equal = free).
         * The other two translations and rotations are locked. */
        SixDOFConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPosition1 = PA(p1);
        s.mPosition2 = PB(p1);
        s.mAxisX1 = VA(axis);
        s.mAxisY1 = VA(nrm);
        s.mAxisX2 = VB(axis);
        s.mAxisY2 = VB(nrm);
        using EAxis = SixDOFConstraintSettings::EAxis;
        if (has_range(d.min_limit, d.max_limit)) {
          s.SetLimitedAxis(EAxis::TranslationX, d.min_limit, d.max_limit);
        }
        else {
          s.MakeFreeAxis(EAxis::TranslationX);
        }
        if (has_range(d.min_limit2, d.max_limit2)) {
          s.SetLimitedAxis(EAxis::RotationX,
                           std::clamp(d.min_limit2, -JPH_PI, JPH_PI),
                           std::clamp(d.max_limit2, -JPH_PI, JPH_PI));
        }
        else {
          s.MakeFreeAxis(EAxis::RotationX);
        }
        s.MakeFixedAxis(EAxis::TranslationY);
        s.MakeFixedAxis(EAxis::TranslationZ);
        s.MakeFixedAxis(EAxis::RotationY);
        s.MakeFixedAxis(EAxis::RotationZ);
        main = add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_PATH: {
        if (!d.path_xyz || d.npath < 2) {
          break;
        }
        /* Body A slides along the path. The path belongs to Body B (or the world). */
        Ref<PathConstraintPathHermite> path = new PathConstraintPathHermite();
        for (int k = 0; k < d.npath; k++) {
          const Vec3 pos(d.path_xyz[k * 3 + 0], d.path_xyz[k * 3 + 1], d.path_xyz[k * 3 + 2]);
          Vec3 tan = axis;
          if (k + 1 < d.npath) {
            tan = Vec3(d.path_xyz[(k + 1) * 3 + 0] - pos.GetX(),
                       d.path_xyz[(k + 1) * 3 + 1] - pos.GetY(),
                       d.path_xyz[(k + 1) * 3 + 2] - pos.GetZ());
          }
          else {
            tan = pos - Vec3(d.path_xyz[(k - 1) * 3 + 0],
                             d.path_xyz[(k - 1) * 3 + 1],
                             d.path_xyz[(k - 1) * 3 + 2]);
          }
          if (tan.LengthSq() < 1.0e-10f) {
            tan = axis;
          }
          const Vec3 tn = tan.Normalized();
          Vec3 pn = nrm - tn * tn.Dot(nrm);
          pn = pn.LengthSq() > 1.0e-8f ? pn.Normalized() : perp_axis(tn);
          path->AddPoint(pos, tan, pn);
        }
        const RMat44 owner0_inv = tb0.InversedRotationTranslation();
        PathConstraintSettings s;
        s.mPath = path;
        s.mPathPosition = Vec3(owner0_inv.GetTranslation());
        s.mPathRotation = owner0_inv.GetQuaternion();
        /* Start at the point of the path closest to Body A now (in the path's rest space). */
        const RVec3 a_now = ida.IsInvalid() ? p1 : bi.GetPosition(ida);
        const RVec3 a_rest = tb0 * (tb.InversedRotationTranslation() * a_now);
        s.mPathFraction = path->GetClosestPoint(Vec3(a_rest), 0.0f);
        s.mRotationConstraintType = EPathRotationConstraintType::ConstrainAroundTangent;
        main = add_two(s, d.body_b, d.body_a);
        break;
      }
      case JOLT_CONSTRAINT_GEAR: {
        /* Gears turn on world-fixed hinges: A about Axis at Anchor, B about Axis 2 at Anchor 2. */
        HingeConstraintSettings h1;
        h1.mSpace = EConstraintSpace::WorldSpace;
        h1.mPoint1 = PA(p1);
        h1.mPoint2 = p1;
        h1.mHingeAxis1 = VA(axis);
        h1.mHingeAxis2 = axis;
        h1.mNormalAxis1 = VA(nrm);
        h1.mNormalAxis2 = nrm;
        HingeConstraintSettings h2;
        h2.mSpace = EConstraintSpace::WorldSpace;
        h2.mPoint1 = PB(p2);
        h2.mPoint2 = p2;
        h2.mHingeAxis1 = VB(nrm);
        h2.mHingeAxis2 = nrm;
        h2.mNormalAxis1 = VB(axis);
        h2.mNormalAxis2 = axis;
        TwoBodyConstraint *c1 = add_two(h1, d.body_a, -1);
        TwoBodyConstraint *c2 = add_two(h2, d.body_b, -1);
        GearConstraintSettings g;
        g.mSpace = EConstraintSpace::WorldSpace;
        g.mHingeAxis1 = VA(axis);
        g.mHingeAxis2 = VB(nrm);
        g.mRatio = d.ratio != 0.0f ? d.ratio : 1.0f;
        main = add_two(g, d.body_a, d.body_b);
        if (main && c1 && c2) {
          static_cast<GearConstraint *>(main)->SetConstraints(c1, c2);
        }
        break;
      }
      case JOLT_CONSTRAINT_RACK_AND_PINION: {
        /* Pinion A turns on a world hinge at Anchor; rack B slides along Axis 2 at Anchor 2. */
        HingeConstraintSettings h;
        h.mSpace = EConstraintSpace::WorldSpace;
        h.mPoint1 = PA(p1);
        h.mPoint2 = p1;
        h.mHingeAxis1 = VA(axis);
        h.mHingeAxis2 = axis;
        h.mNormalAxis1 = VA(nrm);
        h.mNormalAxis2 = nrm;
        SliderConstraintSettings sl;
        sl.mSpace = EConstraintSpace::WorldSpace;
        sl.mPoint1 = PB(p2);
        sl.mPoint2 = p2;
        const Vec3 snrm = perp_axis(nrm);
        sl.mSliderAxis1 = VB(nrm);
        sl.mSliderAxis2 = nrm;
        sl.mNormalAxis1 = VB(snrm);
        sl.mNormalAxis2 = snrm;
        TwoBodyConstraint *ch = add_two(h, d.body_a, -1);
        TwoBodyConstraint *cs = add_two(sl, d.body_b, -1);
        RackAndPinionConstraintSettings rp;
        rp.mSpace = EConstraintSpace::WorldSpace;
        rp.mHingeAxis = VA(axis);
        rp.mSliderAxis = VB(nrm);
        rp.mRatio = d.ratio != 0.0f ? d.ratio : 1.0f;
        main = add_two(rp, d.body_a, d.body_b);
        if (main && ch && cs) {
          static_cast<RackAndPinionConstraint *>(main)->SetConstraints(ch, cs);
        }
        break;
      }
      case JOLT_CONSTRAINT_PULLEY: {
        /* Rope from Anchor (on A) up to Anchor + Axis, over to Anchor 2 + Axis, down to
         * Anchor 2 (on B). Max Limit = max rope length (0 = current length). */
        PulleyConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mBodyPoint1 = PA(p1);
        s.mBodyPoint2 = PB(p2);
        s.mFixedPoint1 = p1 + RVec3(axis.GetX(), axis.GetY(), axis.GetZ());
        s.mFixedPoint2 = p2 + RVec3(axis.GetX(), axis.GetY(), axis.GetZ());
        s.mRatio = d.ratio != 0.0f ? d.ratio : 1.0f;
        if (d.max_limit > 0.0f) {
          s.mMaxLength = d.max_limit;
          s.mMinLength = std::clamp(d.min_limit, 0.0f, d.max_limit);
        }
        main = add_two(s, d.body_a, d.body_b);
        break;
      }
      case JOLT_CONSTRAINT_VEHICLE: {
        const BodyID chassis = invalid_if_world(d.body_a, eng.bodies);
        if (chassis.IsInvalid() || d.body_a >= n_bodies) {
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
        eng.vehicle_by_index[uint(i)] = vc;
        if (WheeledVehicleController *c = static_cast<WheeledVehicleController *>(
                vc->GetController()))
        {
          const float throttle = d.motor_state == JOLT_MOTOR_OFF ?
                                     0.0f :
                                     std::clamp(d.motor_target, -1.0f, 1.0f);
          c->SetDriverInput(throttle, 0.0f, 0.0f, 0.0f);
        }
        break;
      }
      default:
        break;
    }
    if (main) {
      eng.constraint_by_index[uint(i)] = main;
    }
  }
  eng.n_constraints = n_constraints;
  eng.constraint_fp = constraints_fp(n_constraints, constraints);
}

/** Motor state / target / force and vehicle throttle may be animated: apply them every step. */
static void update_constraint_controls(Engine &eng,
                                       const int n_constraints,
                                       const JoltConstraintDesc *constraints)
{
  BodyInterface &bi = eng.physics->GetBodyInterface();
  for (int i = 0; i < n_constraints && i < int(eng.constraint_by_index.size()); i++) {
    const JoltConstraintDesc &d = constraints[i];
    bool driven = false;
    if (Constraint *c = eng.constraint_by_index[uint(i)].GetPtr()) {
      if (d.type == JOLT_CONSTRAINT_HINGE || d.type == JOLT_CONSTRAINT_SLIDER) {
        apply_motor(static_cast<TwoBodyConstraint *>(c), d.type, d);
        driven = d.motor_state != JOLT_MOTOR_OFF;
      }
    }
    if (VehicleConstraint *vc = eng.vehicle_by_index[uint(i)].GetPtr()) {
      if (WheeledVehicleController *wc = static_cast<WheeledVehicleController *>(
              vc->GetController()))
      {
        const float throttle = d.motor_state == JOLT_MOTOR_OFF ?
                                   0.0f :
                                   std::clamp(d.motor_target, -1.0f, 1.0f);
        wc->SetDriverInput(throttle, 0.0f, 0.0f, 0.0f);
        driven = throttle != 0.0f;
      }
    }
    if (driven) {
      /* A sleeping body ignores motors. */
      const BodyID ida = invalid_if_world(d.body_a, eng.bodies);
      const BodyID idb = invalid_if_world(d.body_b, eng.bodies);
      if (!ida.IsInvalid()) {
        bi.ActivateBody(ida);
      }
      if (!idb.IsInvalid()) {
        bi.ActivateBody(idb);
      }
    }
  }
}

/** Create one body. Kinematics start at their previous pose and sweep to the current one. */
static BodyID spawn_from_desc(Engine &eng, const JoltStepSettings &st, const JoltBodyDesc &b)
{
  BodyInterface &bi = eng.physics->GetBodyInterface();
  const float kdt = st.dt > 1.0e-6f ? st.dt : (1.0f / 24.0f);
  const bool kinematic = b.motion_type == JOLT_MOTION_KINEMATIC;
  JoltBodyDesc spawn = b;
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
  const BodyID id = spawn_body(eng, st, spawn, EActivation::Activate);
  if (!id.IsInvalid() && kinematic) {
    Quat q_dst = quat(b.rotation);
    if (bi.GetRotation(id).Dot(q_dst) < 0.0f) {
      q_dst = -q_dst;
    }
    bi.ActivateBody(id);
    bi.MoveKinematic(id, rv3(b.position), q_dst, kdt);
  }
  return id;
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

  eng.bodies.resize(uint(n_bodies));
  eng.names.resize(uint(n_bodies));
  eng.shape_fp.resize(uint(n_bodies));
  eng.req_motion.resize(uint(n_bodies));
  for (int i = 0; i < n_bodies; i++) {
    const JoltBodyDesc &b = bodies[i];
    const BodyID id = spawn_from_desc(eng, st, b);
    if (id.IsInvalid()) {
      if (error && error_max > 0) {
        std::snprintf(error, uint(error_max), "Jolt: out of bodies at index %d", i);
      }
      return false;
    }
    eng.bodies[uint(i)] = id;
    eng.names[uint(i)] = key_of(b, i);
    eng.shape_fp[uint(i)] = shape_fp_of(b);
    eng.req_motion[uint(i)] = b.motion_type;
  }

  add_constraints(eng, n_bodies, bodies, n_constraints, constraints);

  eng.physics->OptimizeBroadPhase();
  eng.n_bodies = n_bodies;
  eng.topology = st.topology_hash;
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

/** Static is always possible; Kinematic / Dynamic need motion properties on the body. */
static bool can_switch_motion(Engine &eng, const BodyID id, const EMotionType want)
{
  if (want == EMotionType::Static) {
    return true;
  }
  BodyLockRead lock(eng.physics->GetBodyLockInterface(), id);
  return lock.Succeeded() && lock.GetBody().CanBeKinematicOrDynamic();
}

static float qdot_abs(const JoltQuat &a, const JoltQuat &b)
{
  const float na = std::sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
  const float nb = std::sqrt(b.w * b.w + b.x * b.x + b.y * b.y + b.z * b.z);
  if (na < 1.0e-8f || nb < 1.0e-8f) {
    return 1.0f;
  }
  return std::fabs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z) / (na * nb);
}

/**
 * The geometry written by the last step comes back unchanged inside a Simulation Zone. Small
 * tolerances only absorb the matrix / quaternion round trip of the instance transform.
 */
static bool pose_differs(const JoltBodyDesc &b, const JoltVec3 &p, const JoltQuat &q)
{
  const float dx = b.position.x - p.x;
  const float dy = b.position.y - p.y;
  const float dz = b.position.z - p.z;
  const float mag2 = p.x * p.x + p.y * p.y + p.z * p.z;
  if (dx * dx + dy * dy + dz * dz > 1.0e-8f * (1.0f + mag2)) {
    return true;
  }
  return 1.0f - qdot_abs(b.rotation, q) > 2.0e-6f;
}

static bool vec_differs(const JoltVec3 &a, const JoltVec3 &b)
{
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  const float dz = a.z - b.z;
  const float mag2 = b.x * b.x + b.y * b.y + b.z * b.z;
  return dx * dx + dy * dy + dz * dz > 1.0e-8f * (1.0f + mag2);
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

  std::vector<char> consumed(static_cast<size_t>(std::max(eng.n_bodies, 0)), 0);
  std::vector<BodyID> next_ids(static_cast<size_t>(n_bodies));
  std::vector<std::string> next_names(static_cast<size_t>(n_bodies));
  std::vector<uint64_t> next_fp(static_cast<size_t>(n_bodies));
  std::vector<int> next_motion(static_cast<size_t>(n_bodies));
  std::vector<int> prev_slot(static_cast<size_t>(n_bodies), -1);
  std::vector<char> spawned(static_cast<size_t>(n_bodies), 0);
  std::vector<char> shape_dirty(static_cast<size_t>(n_bodies), 0);
  std::vector<char> switch_motion(static_cast<size_t>(n_bodies), 0);

  auto take_slot = [&](const int i, const int slot) {
    const JoltBodyDesc &b = bodies[i];
    const EMotionType want = motion_from_int(b.motion_type);
    const BodyID id = eng.bodies[uint(slot)];
    if (id.IsInvalid() || !bi.IsAdded(id)) {
      return false;
    }
    if (eng.req_motion[uint(slot)] != b.motion_type) {
      if (!can_switch_motion(eng, id, want)) {
        return false;
      }
      switch_motion[uint(i)] = 1;
    }
    consumed[uint(slot)] = 1;
    next_ids[uint(i)] = id;
    prev_slot[uint(i)] = slot;
    spawned[uint(i)] = 0;
    if (eng.shape_fp[uint(slot)] != next_fp[uint(i)]) {
      shape_dirty[uint(i)] = 1;
    }
    return true;
  };

  for (int i = 0; i < n_bodies; i++) {
    const JoltBodyDesc &b = bodies[i];
    next_names[uint(i)] = key_of(b, i);
    next_fp[uint(i)] = shape_fp_of(b);
    next_motion[uint(i)] = b.motion_type;
    spawned[uint(i)] = 1;
    const auto it = old_at.find(next_names[uint(i)]);
    if (it == old_at.end() || consumed[uint(it->second)]) {
      continue;
    }
    /* Reuse the same BodyID even if size/scale or motion type changed. An unswitchable slot
     * stays unconsumed, so it is destroyed and a replacement spawned. */
    take_slot(i, it->second);
  }

  /* Animated kinematics are often re-joined every frame with a new idx name.
   * Reuse leftover kinematic/static bodies instead of destroy+create (flash). */
  for (int i = 0; i < n_bodies; i++) {
    if (!spawned[uint(i)] || bodies[i].motion_type == JOLT_MOTION_DYNAMIC) {
      continue;
    }
    for (int j = 0; j < eng.n_bodies; j++) {
      if (consumed[uint(j)] || eng.req_motion[uint(j)] != bodies[i].motion_type) {
        continue;
      }
      if (take_slot(i, j)) {
        break;
      }
    }
  }

  if (st.state_from_input) {
    /* The input is the simulation state. If a dynamic body is not where the last step left
     * it, the zone was reset, resumed from its cache or edited: start a fresh world from the
     * input. Deterministic, so a re-run gives the same result as the first run. */
    for (int i = 0; i < n_bodies; i++) {
      if (spawned[uint(i)] || switch_motion[uint(i)] ||
          bodies[i].motion_type != JOLT_MOTION_DYNAMIC)
      {
        continue;
      }
      const int slot = prev_slot[uint(i)];
      if (slot < 0 || slot >= int(eng.has_out.size()) || !eng.has_out[uint(slot)] ||
          pose_differs(bodies[i], eng.out_pos[uint(slot)], eng.out_rot[uint(slot)]))
      {
        return rebuild(eng, st, n_bodies, bodies, n_constraints, constraints, error, error_max);
      }
    }
  }

  /* Outside a Simulation Zone the graph usually re-feeds the rest pose every frame, so the
   * input cannot be compared to the world. Only a timeline rewind resets it. */
  const bool timeline_reset = !st.state_from_input &&
                              st.scene_time + 1.0e-3f < eng.last_scene_time;
  eng.last_scene_time = st.scene_time;
  const float kdt = st.dt > 1.0e-6f ? st.dt : (1.0f / 24.0f);

  bool added = false;
  bool removed = false;
  for (int i = 0; i < n_bodies; i++) {
    const JoltBodyDesc &b = bodies[i];
    if (spawned[uint(i)]) {
      const BodyID id = spawn_from_desc(eng, st, b);
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
    const EMotionType want = motion_from_int(b.motion_type);
    if (switch_motion[uint(i)]) {
      /* Same BodyID, so velocity, contacts and constraints carry over (animated → dynamic
       * hand-off keeps the animation's momentum). */
      bi.SetObjectLayer(id, layer_for(want));
      bi.SetMotionType(id, want, EActivation::Activate);
    }
    refresh_existing_body(*eng.physics, st, b, id, shape_dirty[uint(i)] != 0);
    if (want == EMotionType::Kinematic) {
      Quat q_dst = quat(b.rotation);
      const RVec3 p_dst = rv3(b.position);
      if (b.has_prev) {
        /* Start the sweep where the graph says the body was last frame. */
        const RVec3 p_prev = rv3(b.prev_position);
        Quat q_prev = quat(b.prev_rotation);
        if (float((bi.GetPosition(id) - p_prev).LengthSq()) > 1.0e-6f ||
            std::fabs(bi.GetRotation(id).Dot(q_prev)) < 0.99999f)
        {
          bi.SetPositionAndRotation(id, p_prev, q_prev, EActivation::Activate);
        }
      }
      const Quat q_cur = bi.GetRotation(id);
      if (q_cur.Dot(q_dst) < 0.0f) {
        q_dst = -q_dst;
      }
      const float dp2 = float((p_dst - bi.GetPosition(id)).LengthSq());
      bi.ActivateBody(id);
      /* Teleport only on a real pop. Always Activate so stacked cubes wake. */
      if (dp2 > 4.0f || q_cur.Dot(q_dst) < 0.1f) {
        bi.SetPositionAndRotation(id, p_dst, q_dst, EActivation::Activate);
      }
      else {
        bi.MoveKinematic(id, p_dst, q_dst, kdt);
      }
    }
    else if (want == EMotionType::Static) {
      bi.SetPositionAndRotationWhenChanged(
          id, rv3(b.position), quat(b.rotation), EActivation::Activate);
    }
    else if (st.state_from_input) {
      /* Pose matched. A changed velocity is an edit inside the zone (impulse, launch). */
      const int slot = prev_slot[uint(i)];
      if (!switch_motion[uint(i)] && slot >= 0 && slot < int(eng.has_out.size()) &&
          eng.has_out[uint(slot)] &&
          (vec_differs(b.linear_velocity, eng.out_lv[uint(slot)]) ||
           vec_differs(b.angular_velocity, eng.out_av[uint(slot)])))
      {
        bi.SetLinearVelocity(id, v3(b.linear_velocity));
        bi.SetAngularVelocity(id, v3(b.angular_velocity));
        bi.ActivateBody(id);
      }
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
  eng.req_motion = std::move(next_motion);
  eng.n_bodies = n_bodies;

  const uint64_t cfp = constraints_fp(n_constraints, constraints);
  if (removed || cfp != eng.constraint_fp || n_constraints != eng.n_constraints) {
    /* Body indices or constraint settings changed. Bodies stay; only constraints are
     * re-created, mapped onto the current poses through constraint_ref. */
    eng.clear_constraints_only();
    add_constraints(eng, n_bodies, bodies, n_constraints, constraints);
  }
  else {
    update_constraint_controls(eng, n_constraints, constraints);
  }
  if (added || removed) {
    eng.physics->OptimizeBroadPhase();
  }
  eng.topology = st.topology_hash;
  return true;
}

static void write_states(
    Engine &eng, int n, const JoltBodyDesc *bodies, JoltBodyState *states, float dt)
{
  BodyInterface &bi = eng.physics->GetBodyInterface();
  const float inv_dt = dt > 1.0e-8f ? (1.0f / dt) : 0.0f;
  eng.out_pos.resize(uint(n));
  eng.out_rot.resize(uint(n));
  eng.out_lv.resize(uint(n));
  eng.out_av.resize(uint(n));
  eng.has_out.assign(uint(n), 1);
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
    if (bi.GetBodyType(id) == EBodyType::RigidBody && mt != EMotionType::Static) {
      BodyLockRead lock(eng.physics->GetBodyLockInterface(), id);
      if (lock.Succeeded()) {
        const MotionProperties *mp = lock.GetBody().GetMotionProperties();
        if (mp != nullptr && mt == EMotionType::Dynamic) {
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
    /* Static / kinematic bodies keep the graph's pose in the output geometry. */
    const bool graph_pose = bodies[i].motion_type != JOLT_MOTION_DYNAMIC;
    eng.out_pos[uint(i)] = graph_pose ? bodies[i].position : states[i].position;
    eng.out_rot[uint(i)] = graph_pose ? bodies[i].rotation : states[i].rotation;
    eng.out_lv[uint(i)] = states[i].linear_velocity;
    eng.out_av[uint(i)] = states[i].angular_velocity;
  }
}

static void evict_old_engines(const Engine *keep)
{
  while (g_engines.size() > k_max_engines) {
    auto oldest = g_engines.end();
    for (auto it = g_engines.begin(); it != g_engines.end(); ++it) {
      if (it->second.get() == keep) {
        continue;
      }
      if (oldest == g_engines.end() || it->second->last_use < oldest->second->last_use) {
        oldest = it;
      }
    }
    if (oldest == g_engines.end()) {
      return;
    }
    g_engines.erase(oldest);
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
    /* First frame of a Simulation Zone (and paused evaluations) use dt = 0: return the input.
     * Reading the cached world here returned the pose of an earlier run, so a reset
     * simulation restarted from where the old one ended. */
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
  eng.last_use = ++g_use_counter;
  evict_old_engines(&eng);

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
