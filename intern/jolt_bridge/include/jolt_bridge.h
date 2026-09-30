/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: MIT
 *
 * C API in front of Jolt Physics. Geometry Nodes never include Jolt headers.
 * Named jolt_bridge.h (not jolt/jolt.h) so Windows include paths cannot collide
 * with upstream Jolt/Jolt.h.
 */

#pragma once

#ifdef _WIN32
#  ifdef JOLT_BRIDGE_EXPORTS
#    define JOLT_API __declspec(dllexport)
#  else
#    define JOLT_API __declspec(dllimport)
#  endif
#else
#  define JOLT_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum JoltShapeType {
  JOLT_SHAPE_SPHERE = 0,
  JOLT_SHAPE_BOX = 1,
  JOLT_SHAPE_CAPSULE = 2,
  JOLT_SHAPE_CONVEX_HULL = 3,
  JOLT_SHAPE_BOUNDING_BOX = 4,
  JOLT_SHAPE_MESH = 5,
  JOLT_SHAPE_COMPOUND = 6,
  JOLT_SHAPE_TAPERED_CAPSULE = 7,
  JOLT_SHAPE_CYLINDER = 8,
  JOLT_SHAPE_TAPERED_CYLINDER = 9,
  JOLT_SHAPE_PLANE = 10,
  JOLT_SHAPE_HEIGHTFIELD = 11,
  JOLT_SHAPE_EMPTY = 12,
} JoltShapeType;

typedef enum JoltMotionType {
  JOLT_MOTION_STATIC = 0,
  JOLT_MOTION_KINEMATIC = 1,
  JOLT_MOTION_DYNAMIC = 2,
} JoltMotionType;

typedef enum JoltAllowedDOFs {
  JOLT_DOFS_ALL = 0,
  JOLT_DOFS_PLANE_XY = 1,
  JOLT_DOFS_PLANE_XZ = 2,
  JOLT_DOFS_PLANE_YZ = 3,
  JOLT_DOFS_TRANSLATION = 4,
  JOLT_DOFS_ROTATION = 5,
} JoltAllowedDOFs;

/** Matches Jolt's constraint set (https://jrouwe.github.io/JoltPhysics/index.html#constraints). */
typedef enum JoltConstraintType {
  JOLT_CONSTRAINT_FIXED = 0,
  JOLT_CONSTRAINT_DISTANCE = 1,
  JOLT_CONSTRAINT_POINT = 2,
  JOLT_CONSTRAINT_HINGE = 3,
  JOLT_CONSTRAINT_CONE = 4,
  JOLT_CONSTRAINT_SLIDER = 5,
  JOLT_CONSTRAINT_SWING_TWIST = 6,
  JOLT_CONSTRAINT_SIX_DOF = 7,
  JOLT_CONSTRAINT_PATH = 8,
  JOLT_CONSTRAINT_GEAR = 9,
  JOLT_CONSTRAINT_RACK_AND_PINION = 10,
  JOLT_CONSTRAINT_PULLEY = 11,
  JOLT_CONSTRAINT_VEHICLE = 12,
} JoltConstraintType;

typedef enum JoltMotorState {
  JOLT_MOTOR_OFF = 0,
  JOLT_MOTOR_VELOCITY = 1,
  JOLT_MOTOR_POSITION = 2,
} JoltMotorState;

typedef struct JoltVec3 {
  float x, y, z;
} JoltVec3;

typedef struct JoltQuat {
  float w, x, y, z;
} JoltQuat;

typedef struct JoltBodyDesc {
  JoltVec3 position;
  JoltQuat rotation;
  JoltVec3 linear_velocity;
  JoltVec3 angular_velocity;
  /** Half-extents / radius depending on shape (same convention as Box Engine). */
  JoltVec3 shape_size;
  float density;
  float friction;
  float restitution;
  float linear_damping;
  float angular_damping;
  float convex_radius;
  /** Top radius for tapered capsule / cylinder. */
  float radius_top;
  int motion_type;
  int shape_type;
  int allowed_dofs;
  int sensor;
  int ccd;
  int graph_driven;
  /** Optional name for constraints. Not a physics cache key. */
  const char *name;
  /** 0 = compute from density * shape. Read from geometry every step. */
  float mass;
  /** Body-local inertia diagonal. (0,0,0) = compute from shape. */
  JoltVec3 inertia;
  /** Previous pose from geometry attributes (kinematic sweep). */
  JoltVec3 prev_position;
  JoltQuat prev_rotation;
  /** 1 = prev_* came from input geometry. 0 = reconstruct from velocity or teleport. */
  int has_prev;
  const float *verts_xyz;
  int nverts;
  const int *tri_indices;
  int nindices;
  const int *island_counts;
  int nislands;
  const float *height_samples;
  int height_size;
  float height_cell;
} JoltBodyDesc;

typedef struct JoltBodyState {
  JoltVec3 position;
  JoltQuat rotation;
  JoltVec3 linear_velocity;
  JoltVec3 angular_velocity;
  /** (v_out - v_in) / dt this step. Not moment of inertia. */
  JoltVec3 linear_acceleration;
  /** (w_out - w_in) / dt this step. Not moment of inertia. */
  JoltVec3 angular_acceleration;
  float mass;
  /** Body-local inertia diagonal (I), not angular acceleration. */
  JoltVec3 inertia;
} JoltBodyState;

typedef struct JoltConstraintDesc {
  int type;
  /** Body index, or -1 for world (Body::sFixedToWorld). */
  int body_a;
  int body_b;
  JoltVec3 anchor;
  JoltVec3 axis;
  JoltVec3 axis2;
  JoltVec3 anchor2;
  float min_limit;
  float max_limit;
  float min_limit2;
  float max_limit2;
  float ratio;
  float frequency;
  float damping;
  int motor_state;
  float motor_target;
  float motor_force;
  const float *path_xyz;
  int npath;
  float wheel_radius;
  float wheel_width;
  float suspension_min;
  float suspension_max;
  float max_steer;
  int num_wheels;
} JoltConstraintDesc;

typedef struct JoltStepSettings {
  JoltVec3 gravity;
  float dt;
  /** Scene time (frames). Used to detect timeline rewind vs playback. */
  float scene_time;
  int collision_steps;
  int velocity_steps;
  int position_steps;
  int ccd;
  unsigned long long topology_hash;
  const char *cache_key;
} JoltStepSettings;

/** Drop a cached PhysicsSystem. Pass NULL to drop every world. */
JOLT_API void JOLT_cache_invalidate(const char *cache_key);

/**
 * Step a cached PhysicsSystem (keyed by cache_key). Dynamic bodies keep their
 * Jolt BodyID / velocity / contacts across calls so gravity and stacks work
 * inside a Simulation Zone. Shape/size/friction updates use SetShape.
 * Add/remove is by body name. Dynamics keep the cached Jolt pose during
 * playback (the node graph usually re-feeds the rest pose every frame).
 * They are teleported from the input pose only on timeline rewind.
 * Returns 1 on success.
 */
JOLT_API int JOLT_step(const JoltStepSettings *settings,
                       int n_bodies,
                       const JoltBodyDesc *bodies,
                       JoltBodyState *states,
                       int n_constraints,
                       const JoltConstraintDesc *constraints,
                       char *error,
                       int error_max);

#ifdef __cplusplus
}
#endif
