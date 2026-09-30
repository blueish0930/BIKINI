/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Make It Stand: volume center of mass, support polygon, and inner-void carving
 * to shift the mass projection into the support (Prévost et al., SIGGRAPH 2013).
 */

#pragma once

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include <string>

namespace blender {

struct Mesh;
struct PointCloud;

namespace geometry {

struct VolumeMassResult {
  float3 center = float3(0.0f);
  float volume = 0.0f;
  float mass = 0.0f;
  bool ok = false;
};

/** Uniform-density volume centroid via signed tetrahedra (divergence theorem). */
VolumeMassResult mesh_volume_center_of_mass(const Mesh &mesh,
                                            float density,
                                            std::string &r_error);

struct SupportPolygonResult {
  Mesh *polygon = nullptr;
  PointCloud *contact_points = nullptr;
  Vector<float3> hull;
  float3 target = float3(0.0f);
  float3 gravity = float3(0.0f, 0.0f, -1.0f);
};

/**
 * Contact verts: explicit selection, else extra points, else the lowest verts along gravity.
 * Hull is the 2D convex hull in the plane perpendicular to gravity, placed on the contact plane.
 */
SupportPolygonResult mesh_support_polygon(const Mesh &mesh,
                                          float3 gravity,
                                          float contact_epsilon,
                                          Span<bool> contact_selection,
                                          Span<float3> extra_points,
                                          std::string &r_error);

struct BalanceStatus {
  bool is_stable = false;
  float3 center = float3(0.0f);
  float3 projected = float3(0.0f);
  float3 target = float3(0.0f);
  float margin = 0.0f;
  float volume = 0.0f;
};

BalanceStatus mesh_balance_status(const float3 center,
                                  const float3 gravity,
                                  Span<float3> hull,
                                  float contact_epsilon);

struct MakeItStandParams {
  float3 gravity = float3(0.0f, 0.0f, -1.0f);
  float3 target = float3(0.0f);
  bool use_target = false;
  float contact_epsilon = 0.01f;
  float density = 1.0f;
  float shell_thickness = 0.05f;
  int resolution = 32;
  float max_empty_fraction = 0.8f;
  int iterations = 4;
  float deform_strength = 0.0f;
  float lower_mass = 0.25f;
  bool boolean_cavity = false;
  Span<bool> contact_selection;
  Span<float3> extra_contact_points;
  const Mesh *existing_void = nullptr;
};

struct MakeItStandResult {
  Mesh *mesh = nullptr;
  Mesh *inner_void = nullptr;
  Mesh *support = nullptr;
  PointCloud *contact_points = nullptr;
  float3 center = float3(0.0f);
  float3 projected = float3(0.0f);
  float3 target = float3(0.0f);
  float volume = 0.0f;
  float margin = 0.0f;
  bool is_stable = false;
  int carved_voxels = 0;
  std::string warning;
};

/**
 * Keep the outer surface (same topology). Carve an inner void on the heavy side of the
 * vertical plane through the target. Optional Inner Void input lets nodes / simulation
 * zones accumulate cavities. Boolean is opt-in for a final printable hollow mesh.
 */
MakeItStandResult mesh_make_it_stand(const Mesh &mesh,
                                     const MakeItStandParams &params,
                                     std::string &r_error);

}  // namespace geometry
}  // namespace blender
