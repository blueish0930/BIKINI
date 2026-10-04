/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: MIT
 *
 * Compact XPBD core (Macklin / Müller / Bender) for Geometry Nodes.
 * Not a git-subtree of InteractiveComputerGraphics/PositionBasedDynamics.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PBDWorld PBDWorld;

PBDWorld *PBD_world_create(void);
void PBD_world_destroy(PBDWorld *world);

void PBD_world_clear(PBDWorld *world);
void PBD_world_set_gravity(PBDWorld *world, float x, float y, float z);
void PBD_world_set_ground(PBDWorld *world, int enabled, int axis, float height, float friction);
void PBD_world_set_iterations(PBDWorld *world, int iterations);
void PBD_world_set_substeps(PBDWorld *world, int substeps);
void PBD_world_set_self_collision(PBDWorld *world, int enabled, float thickness);
void PBD_world_set_collision_thickness(PBDWorld *world, float thickness);
void PBD_world_set_collision_friction(PBDWorld *world, float friction);
void PBD_world_clear_colliders(PBDWorld *world);
void PBD_add_collider_triangle(PBDWorld *world,
                               float ax,
                               float ay,
                               float az,
                               float bx,
                               float by,
                               float bz,
                               float cx,
                               float cy,
                               float cz);

int PBD_add_particle(PBDWorld *world, float x, float y, float z, float mass, int pinned);
void PBD_set_velocity(PBDWorld *world, int i, float x, float y, float z);

int PBD_add_distance(PBDWorld *world, int a, int b, float rest, float compliance);
int PBD_add_bending(PBDWorld *world, int a, int b, float rest, float compliance);
void PBD_set_volume_constraint(PBDWorld *world,
                               const int *tri_indices,
                               int tri_count,
                               float rest_volume,
                               float compliance);

void PBD_step(PBDWorld *world, float dt);

int PBD_particle_count(const PBDWorld *world);
void PBD_get_position(const PBDWorld *world, int i, float *x, float *y, float *z);
void PBD_get_velocity(const PBDWorld *world, int i, float *x, float *y, float *z);
float PBD_compute_volume(const PBDWorld *world);

#ifdef __cplusplus
}
#endif
