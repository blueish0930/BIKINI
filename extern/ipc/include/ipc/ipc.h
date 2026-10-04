/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: MIT
 *
 * Compact Incremental Potential Contact core (Li / Ferguson / Jiang / Zorin / Panozzo).
 * Barrier + implicit Euler subset for Geometry Nodes. Not ipc-toolkit + PolyFEM.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct IPCWorld IPCWorld;

IPCWorld *IPC_world_create(void);
void IPC_world_destroy(IPCWorld *world);
void IPC_world_clear(IPCWorld *world);

void IPC_world_set_gravity(IPCWorld *world, float x, float y, float z);
void IPC_world_set_ground(IPCWorld *world, int enabled, int axis, float height);
void IPC_world_set_params(IPCWorld *world,
                          float thickness,
                          float barrier_stiffness,
                          float stretch_stiffness,
                          int iterations);

int IPC_add_particle(IPCWorld *world, float x, float y, float z, float mass, int pinned);
void IPC_set_velocity(IPCWorld *world, int i, float x, float y, float z);
int IPC_add_edge(IPCWorld *world, int a, int b, float rest);

void IPC_step(IPCWorld *world, float dt);

int IPC_particle_count(const IPCWorld *world);
void IPC_get_position(const IPCWorld *world, int i, float *x, float *y, float *z);
void IPC_get_velocity(const IPCWorld *world, int i, float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif
