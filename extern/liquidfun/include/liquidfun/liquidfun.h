/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: MIT
 *
 * Compact 2D particle fluid / elastic core in the spirit of Google LiquidFun
 * (Box2D + particles). Independent of the vendored Box2D 3.x C API.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum LiquidFunPlane {
  LIQUIDFUN_PLANE_XY = 0,
  LIQUIDFUN_PLANE_XZ = 1,
} LiquidFunPlane;

typedef struct LiquidFunWorld LiquidFunWorld;

LiquidFunWorld *LF_world_create(void);
void LF_world_destroy(LiquidFunWorld *world);
void LF_world_clear(LiquidFunWorld *world);

void LF_world_set_plane(LiquidFunWorld *world, LiquidFunPlane plane);
void LF_world_set_gravity(LiquidFunWorld *world, float x, float y, float z);
void LF_world_set_radius(LiquidFunWorld *world, float radius);
void LF_world_set_rest_density(LiquidFunWorld *world, float density);
/** If rest density is 0, sample SPH density from the current layout and use that. */
float LF_calibrate_rest_density(LiquidFunWorld *world);
float LF_world_get_rest_density(const LiquidFunWorld *world);
void LF_world_set_viscosity(LiquidFunWorld *world, float viscosity);
void LF_world_set_iterations(LiquidFunWorld *world, int iterations);
void LF_world_set_elastic(LiquidFunWorld *world, int enabled, float stiffness);
void LF_world_set_bounds(LiquidFunWorld *world,
                         float min0,
                         float min1,
                         float max0,
                         float max1,
                         int enabled);
void LF_add_collider_segment(LiquidFunWorld *world, float ax, float ay, float bx, float by);

int LF_add_particle(LiquidFunWorld *world, float x, float y, float z, float mass);
void LF_set_velocity(LiquidFunWorld *world, int i, float x, float y, float z);
void LF_set_rest_position(LiquidFunWorld *world, int i, float x, float y, float z);
void LF_capture_elastic_rest(LiquidFunWorld *world);

void LF_step(LiquidFunWorld *world, float dt);

int LF_particle_count(const LiquidFunWorld *world);
void LF_get_position(const LiquidFunWorld *world, int i, float *x, float *y, float *z);
void LF_get_velocity(const LiquidFunWorld *world, int i, float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif
