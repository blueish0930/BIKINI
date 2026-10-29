/* SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once
#ifdef _WIN32
#  ifdef RESTING_BRIDGE_BUILD
#    define RRB_API __declspec(dllexport)
#  else
#    define RRB_API __declspec(dllimport)
#  endif
#else
#  define RRB_API
#endif
#ifdef __cplusplus
extern "C" {
#endif
typedef struct RRB_Solver RRB_Solver;
/* Matrices are column-major; normals are downward directions in input coordinates. */
typedef struct RRB_Pose {
  double matrix[16];
  double normal[3];
  double probability;
  double height;
} RRB_Pose;
/* A great-circle separatrix arc on the unit Gauss sphere. Pose indices refer
 * to rrb_pose() after coincident stable normals have been merged and sorted. */
typedef struct RRB_Separatrix {
  double start[3];
  double end[3];
  int pose_a;
  int pose_b;
} RRB_Separatrix;
RRB_API RRB_Solver *rrb_create(const double *positions, int count, const double center[3], char error[512]);
RRB_API void rrb_free(RRB_Solver *solver);
RRB_API int rrb_pose_count(const RRB_Solver *solver);
RRB_API void rrb_pose(const RRB_Solver *solver, int index, RRB_Pose *pose);
RRB_API int rrb_separatrix_count(const RRB_Solver *solver);
RRB_API void rrb_separatrix(const RRB_Solver *solver, int index, RRB_Separatrix *arc);
RRB_API int rrb_trace(RRB_Solver *solver, const double direction[3], double angle_step, char error[512]);
RRB_API int rrb_step_count(const RRB_Solver *solver);
RRB_API void rrb_step(const RRB_Solver *solver, int index, RRB_Pose *pose);
RRB_API void rrb_sample(const RRB_Solver *solver, double progress, RRB_Pose *pose);
#ifdef __cplusplus
}
#endif
