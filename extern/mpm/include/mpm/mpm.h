/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: MIT
 *
 * Compact MLS-MPM core (Jiang / Hu / Stomakhin / Teran / Ramamoorthi).
 * Elastic (fixed corotated) and fluid (J-based) materials.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum MPMMaterial {
  MPM_MATERIAL_ELASTIC = 0,
  MPM_MATERIAL_FLUID = 1,
} MPMMaterial;

typedef struct MPMWorld MPMWorld;

MPMWorld *MPM_world_create(void);
void MPM_world_destroy(MPMWorld *world);
void MPM_world_clear(MPMWorld *world);

void MPM_world_set_gravity(MPMWorld *world, float x, float y, float z);
void MPM_world_set_dx(MPMWorld *world, float dx);
void MPM_world_set_material(MPMWorld *world, MPMMaterial material);
void MPM_world_set_lame(MPMWorld *world, float young, float poisson);
void MPM_world_set_bounds(MPMWorld *world,
                          float minx,
                          float miny,
                          float minz,
                          float maxx,
                          float maxy,
                          float maxz,
                          int enabled);
void MPM_add_collider_triangle(MPMWorld *world,
                               float ax,
                               float ay,
                               float az,
                               float bx,
                               float by,
                               float bz,
                               float cx,
                               float cy,
                               float cz);

int MPM_add_particle(MPMWorld *world, float x, float y, float z, float mass, float volume);
void MPM_set_velocity(MPMWorld *world, int i, float x, float y, float z);
void MPM_set_F(MPMWorld *world, int i, const float F[9]);
void MPM_set_C(MPMWorld *world, int i, const float C[9]);
void MPM_set_J(MPMWorld *world, int i, float J);

void MPM_step(MPMWorld *world, float dt);

int MPM_particle_count(const MPMWorld *world);
void MPM_get_position(const MPMWorld *world, int i, float *x, float *y, float *z);
void MPM_get_velocity(const MPMWorld *world, int i, float *x, float *y, float *z);
void MPM_get_F(const MPMWorld *world, int i, float F[9]);
void MPM_get_C(const MPMWorld *world, int i, float C[9]);
float MPM_get_J(const MPMWorld *world, int i);

#ifdef __cplusplus
}
#endif
