/* SPDX-FileCopyrightText: 2013 Inigo Quilez
 * SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Smooth Voronoi:
 *
 * - https://wiki.blender.org/wiki/User:OmarSquircleArt/GSoC2019/Documentation/Smooth_Voronoi
 *
 * Distance To Edge based on:
 *
 * - https://www.iquilezles.org/www/articles/voronoilines/voronoilines.htm
 * - https://www.shadertoy.com/view/ldl3W8
 *
 * With optimization to change -2..2 scan window to -1..1 for better performance,
 * as explained in https://www.shadertoy.com/view/llG3zy.
 */

#pragma once

#include "gpu_shader_common_hash.bsl.hh"
#include "gpu_shader_material_noise.bsl.hh"
#include "gpu_shader_utildefines.bsl.hh"

#define SHD_VORONOI_EUCLIDEAN 0
#define SHD_VORONOI_MANHATTAN 1
#define SHD_VORONOI_CHEBYCHEV 2
#define SHD_VORONOI_MINKOWSKI 3

#define SHD_VORONOI_F1 0
#define SHD_VORONOI_F2 1
#define SHD_VORONOI_SMOOTH_F1 2
#define SHD_VORONOI_DISTANCE_TO_EDGE 3
#define SHD_VORONOI_N_SPHERE_RADIUS 4

struct VoronoiParams {
  float scale;
  float detail;
  float roughness;
  float lacunarity;
  float smoothness;
  float exponent;
  float randomness;
  float max_distance;
  bool normalize;
  int feature;
  int metric;
  float4 period;
};

struct VoronoiOutput {
  float Distance;
  float3 Color;
  float4 Position;
};

float voronoi_finish_edge_distance(float d)
{
  /* Tiling wrap can yield tiny negatives or +inf if every neighbor is degenerate. */
  return clamp(d, 0.0f, 1.0f);
}

/* **** Distance Functions **** */

float voronoi_distance(float a, float b)
{
  return abs(a - b);
}

float voronoi_distance(float2 a, float2 b, VoronoiParams params)
{
  if (params.metric == SHD_VORONOI_EUCLIDEAN) {
    return distance(a, b);
  }
  if (params.metric == SHD_VORONOI_MANHATTAN) {
    return abs(a.x - b.x) + abs(a.y - b.y);
  }
  if (params.metric == SHD_VORONOI_CHEBYCHEV) {
    return max(abs(a.x - b.x), abs(a.y - b.y));
  }
  if (params.metric == SHD_VORONOI_MINKOWSKI) {
    return pow(pow(abs(a.x - b.x), params.exponent) + pow(abs(a.y - b.y), params.exponent),
               1.0f / params.exponent);
  }
  return 0.0f;
}

float voronoi_distance(float3 a, float3 b, VoronoiParams params)
{
  if (params.metric == SHD_VORONOI_EUCLIDEAN) {
    return distance(a, b);
  }
  if (params.metric == SHD_VORONOI_MANHATTAN) {
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z);
  }
  if (params.metric == SHD_VORONOI_CHEBYCHEV) {
    return max(abs(a.x - b.x), max(abs(a.y - b.y), abs(a.z - b.z)));
  }
  if (params.metric == SHD_VORONOI_MINKOWSKI) {
    return pow(pow(abs(a.x - b.x), params.exponent) + pow(abs(a.y - b.y), params.exponent) +
                   pow(abs(a.z - b.z), params.exponent),
               1.0f / params.exponent);
  }
  return 0.0f;
}

float voronoi_distance(float4 a, float4 b, VoronoiParams params)
{
  if (params.metric == SHD_VORONOI_EUCLIDEAN) {
    return distance(a, b);
  }
  if (params.metric == SHD_VORONOI_MANHATTAN) {
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z) + abs(a.w - b.w);
  }
  if (params.metric == SHD_VORONOI_CHEBYCHEV) {
    return max(abs(a.x - b.x), max(abs(a.y - b.y), max(abs(a.z - b.z), abs(a.w - b.w))));
  }
  if (params.metric == SHD_VORONOI_MINKOWSKI) {
    return pow(pow(abs(a.x - b.x), params.exponent) + pow(abs(a.y - b.y), params.exponent) +
                   pow(abs(a.z - b.z), params.exponent) + pow(abs(a.w - b.w), params.exponent),
               1.0f / params.exponent);
  }
  return 0.0f;
}

/* **** 1D Voronoi **** */

float4 voronoi_position(float coord)
{
  return float4(0.0f, 0.0f, 0.0f, coord);
}

VoronoiOutput voronoi_f1(VoronoiParams params, float coord)
{
  float cellPosition = floor(coord);
  float localPosition = coord - cellPosition;

  float minDistance = FLT_MAX;
  float targetOffset = 0.0f;
  float targetPosition = 0.0f;
  for (int i = -1; i <= 1; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset +
                          hash_float_to_float(wrap_cell(cellPosition + cellOffset, params.period.x)) * params.randomness;
    float distanceToPoint = voronoi_distance(pointPosition, localPosition);
    if (distanceToPoint < minDistance) {
      targetOffset = cellOffset;
      minDistance = distanceToPoint;
      targetPosition = pointPosition;
    }
  }

  VoronoiOutput octave;
  octave.Distance = minDistance;
  octave.Color = hash_float_to_vec3(wrap_cell(cellPosition + targetOffset, params.period.x));
  octave.Position = voronoi_position(targetPosition + cellPosition);
  return octave;
}

VoronoiOutput voronoi_smooth_f1(VoronoiParams params, float coord)
{
  float cellPosition = floor(coord);
  float localPosition = coord - cellPosition;

  float smoothDistance = 0.0f;
  float smoothPosition = 0.0f;
  float3 smoothColor = float3(0.0f);
  float h = -1.0f;
  for (int i = -2; i <= 2; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset +
                          hash_float_to_float(wrap_cell(cellPosition + cellOffset, params.period.x)) * params.randomness;
    float distanceToPoint = voronoi_distance(pointPosition, localPosition);
    h = h == -1.0f ?
            1.0f :
            smoothstep(
                0.0f, 1.0f, 0.5f + 0.5f * (smoothDistance - distanceToPoint) / params.smoothness);
    float correctionFactor = params.smoothness * h * (1.0f - h);
    smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
    correctionFactor /= 1.0f + 3.0f * params.smoothness;
    float3 cellColor = hash_float_to_vec3(wrap_cell(cellPosition + cellOffset, params.period.x));
    smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
    smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
  }

  VoronoiOutput octave;
  octave.Distance = smoothDistance;
  octave.Color = smoothColor;
  octave.Position = voronoi_position(cellPosition + smoothPosition);
  return octave;
}

VoronoiOutput voronoi_f2(VoronoiParams params, float coord)
{
  float cellPosition = floor(coord);
  float localPosition = coord - cellPosition;

  float distanceF1 = FLT_MAX;
  float distanceF2 = FLT_MAX;
  float offsetF1 = 0.0f;
  float positionF1 = 0.0f;
  float offsetF2 = 0.0f;
  float positionF2 = 0.0f;
  for (int i = -1; i <= 1; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset +
                          hash_float_to_float(wrap_cell(cellPosition + cellOffset, params.period.x)) * params.randomness;
    float distanceToPoint = voronoi_distance(pointPosition, localPosition);
    if (distanceToPoint < distanceF1) {
      distanceF2 = distanceF1;
      distanceF1 = distanceToPoint;
      offsetF2 = offsetF1;
      offsetF1 = cellOffset;
      positionF2 = positionF1;
      positionF1 = pointPosition;
    }
    else if (distanceToPoint < distanceF2) {
      distanceF2 = distanceToPoint;
      offsetF2 = cellOffset;
      positionF2 = pointPosition;
    }
  }

  VoronoiOutput octave;
  octave.Distance = distanceF2;
  octave.Color = hash_float_to_vec3(wrap_cell(cellPosition + offsetF2, params.period.x));
  octave.Position = voronoi_position(positionF2 + cellPosition);
  return octave;
}

float voronoi_distance_to_edge(VoronoiParams params, float coord)
{
  float cellPosition = floor(coord);
  float localPosition = coord - cellPosition;

  float midPointPosition = hash_float_to_float(wrap_cell(cellPosition, params.period.x)) *
                           params.randomness;
  float leftPointPosition = -1.0f +
                            hash_float_to_float(wrap_cell(cellPosition - 1.0f, params.period.x)) *
                                params.randomness;
  float rightPointPosition = 1.0f +
                             hash_float_to_float(wrap_cell(cellPosition + 1.0f, params.period.x)) *
                                 params.randomness;
  float distanceToMidLeft = abs((midPointPosition + leftPointPosition) / 2.0f - localPosition);
  float distanceToMidRight = abs((midPointPosition + rightPointPosition) / 2.0f - localPosition);

  return voronoi_finish_edge_distance(min(distanceToMidLeft, distanceToMidRight));
}

float voronoi_n_sphere_radius(VoronoiParams params, float coord)
{
  float cellPosition = floor(coord);
  float localPosition = coord - cellPosition;

  float closestPoint = 0.0f;
  float closestPointOffset = 0.0f;
  float minDistance = FLT_MAX;
  for (int i = -1; i <= 1; i++) {
    float cellOffset = i;
    float pointPosition = cellOffset +
                          hash_float_to_float(wrap_cell(cellPosition + cellOffset, params.period.x)) * params.randomness;
    float distanceToPoint = abs(pointPosition - localPosition);
    if (distanceToPoint < minDistance) {
      minDistance = distanceToPoint;
      closestPoint = pointPosition;
      closestPointOffset = cellOffset;
    }
  }

  minDistance = FLT_MAX;
  float closestPointToClosestPoint = 0.0f;
  for (int i = -1; i <= 1; i++) {
    if (i == 0) {
      continue;
    }
    float cellOffset = i + closestPointOffset;
    float pointPosition = cellOffset +
                          hash_float_to_float(wrap_cell(cellPosition + cellOffset, params.period.x)) * params.randomness;
    float distanceToPoint = abs(closestPoint - pointPosition);
    if (distanceToPoint < minDistance) {
      minDistance = distanceToPoint;
      closestPointToClosestPoint = pointPosition;
    }
  }

  return abs(closestPointToClosestPoint - closestPoint) / 2.0f;
}

/* **** 2D Voronoi **** */

float4 voronoi_position(float2 coord)
{
  return float4(coord.x, coord.y, 0.0f, 0.0f);
}

VoronoiOutput voronoi_f1(VoronoiParams params, float2 coord)
{
  float2 cellPosition_f = floor(coord);
  float2 localPosition = coord - cellPosition_f;
  int2 cellPosition = int2(cellPosition_f);

  float minDistance = FLT_MAX;
  int2 targetOffset = int2(0);
  float2 targetPosition = float2(0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      int2 cellOffset = int2(i, j);
      float2 pointPosition = float2(cellOffset) +
                             hash_int2_to_vec2(wrap_cell(cellPosition + cellOffset, params.period.xy)) * params.randomness;
      float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
      if (distanceToPoint < minDistance) {
        targetOffset = cellOffset;
        minDistance = distanceToPoint;
        targetPosition = pointPosition;
      }
    }
  }

  VoronoiOutput octave;
  octave.Distance = minDistance;
  octave.Color = hash_int2_to_vec3(wrap_cell(cellPosition + targetOffset, params.period.xy));
  octave.Position = voronoi_position(targetPosition + cellPosition_f);
  return octave;
}

VoronoiOutput voronoi_smooth_f1(VoronoiParams params, float2 coord)
{
  float2 cellPosition_f = floor(coord);
  float2 localPosition = coord - cellPosition_f;
  int2 cellPosition = int2(cellPosition_f);

  float smoothDistance = 0.0f;
  float3 smoothColor = float3(0.0f);
  float2 smoothPosition = float2(0.0f);
  float h = -1.0f;
  for (int j = -2; j <= 2; j++) {
    for (int i = -2; i <= 2; i++) {
      int2 cellOffset = int2(i, j);
      float2 pointPosition = float2(cellOffset) +
                             hash_int2_to_vec2(wrap_cell(cellPosition + cellOffset, params.period.xy)) * params.randomness;
      float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
      h = h == -1.0f ?
              1.0f :
              smoothstep(0.0f,
                         1.0f,
                         0.5f + 0.5f * (smoothDistance - distanceToPoint) / params.smoothness);
      float correctionFactor = params.smoothness * h * (1.0f - h);
      smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
      correctionFactor /= 1.0f + 3.0f * params.smoothness;
      float3 cellColor = hash_int2_to_vec3(wrap_cell(cellPosition + cellOffset, params.period.xy));
      smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
      smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
    }
  }

  VoronoiOutput octave;
  octave.Distance = smoothDistance;
  octave.Color = smoothColor;
  octave.Position = voronoi_position(cellPosition_f + smoothPosition);
  return octave;
}

VoronoiOutput voronoi_f2(VoronoiParams params, float2 coord)
{
  float2 cellPosition_f = floor(coord);
  float2 localPosition = coord - cellPosition_f;
  int2 cellPosition = int2(cellPosition_f);

  float distanceF1 = FLT_MAX;
  float distanceF2 = FLT_MAX;
  int2 offsetF1 = int2(0);
  float2 positionF1 = float2(0.0f);
  int2 offsetF2 = int2(0);
  float2 positionF2 = float2(0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      int2 cellOffset = int2(i, j);
      float2 pointPosition = float2(cellOffset) +
                             hash_int2_to_vec2(wrap_cell(cellPosition + cellOffset, params.period.xy)) * params.randomness;
      float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
      if (distanceToPoint < distanceF1) {
        distanceF2 = distanceF1;
        distanceF1 = distanceToPoint;
        offsetF2 = offsetF1;
        offsetF1 = cellOffset;
        positionF2 = positionF1;
        positionF1 = pointPosition;
      }
      else if (distanceToPoint < distanceF2) {
        distanceF2 = distanceToPoint;
        offsetF2 = cellOffset;
        positionF2 = pointPosition;
      }
    }
  }

  VoronoiOutput octave;
  octave.Distance = distanceF2;
  octave.Color = hash_int2_to_vec3(wrap_cell(cellPosition + offsetF2, params.period.xy));
  octave.Position = voronoi_position(positionF2 + cellPosition_f);
  return octave;
}

float voronoi_distance_to_edge(VoronoiParams params, float2 coord)
{
  float2 cellPosition_f = floor(coord);
  float2 localPosition = coord - cellPosition_f;
  int2 cellPosition = int2(cellPosition_f);

  float2 vectorToClosest = float2(0.0f);
  float minDistance = FLT_MAX;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      int2 cellOffset = int2(i, j);
      float2 vectorToPoint = float2(cellOffset) +
                             hash_int2_to_vec2(wrap_cell(cellPosition + cellOffset, params.period.xy)) * params.randomness -
                             localPosition;
      float distanceToPoint = dot(vectorToPoint, vectorToPoint);
      if (distanceToPoint < minDistance) {
        minDistance = distanceToPoint;
        vectorToClosest = vectorToPoint;
      }
    }
  }

  minDistance = FLT_MAX;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      int2 cellOffset = int2(i, j);
      float2 vectorToPoint = float2(cellOffset) +
                             hash_int2_to_vec2(wrap_cell(cellPosition + cellOffset, params.period.xy)) * params.randomness -
                             localPosition;
      float2 perpendicularToEdge = vectorToPoint - vectorToClosest;
      if (dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
        float distanceToEdge = dot((vectorToClosest + vectorToPoint) / 2.0f,
                                   normalize(perpendicularToEdge));
        minDistance = min(minDistance, distanceToEdge);
      }
    }
  }

  return voronoi_finish_edge_distance(minDistance);
}

float voronoi_n_sphere_radius(VoronoiParams params, float2 coord)
{
  float2 cellPosition_f = floor(coord);
  float2 localPosition = coord - cellPosition_f;
  int2 cellPosition = int2(cellPosition_f);

  float2 closestPoint = float2(0.0f);
  int2 closestPointOffset = int2(0);
  float minDistance = FLT_MAX;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      int2 cellOffset = int2(i, j);
      float2 pointPosition = float2(cellOffset) +
                             hash_int2_to_vec2(wrap_cell(cellPosition + cellOffset, params.period.xy)) * params.randomness;
      float distanceToPoint = distance(pointPosition, localPosition);
      if (distanceToPoint < minDistance) {
        minDistance = distanceToPoint;
        closestPoint = pointPosition;
        closestPointOffset = cellOffset;
      }
    }
  }

  minDistance = FLT_MAX;
  float2 closestPointToClosestPoint = float2(0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      if (i == 0 && j == 0) {
        continue;
      }
      int2 cellOffset = int2(i, j) + closestPointOffset;
      float2 pointPosition = float2(cellOffset) +
                             hash_int2_to_vec2(wrap_cell(cellPosition + cellOffset, params.period.xy)) * params.randomness;
      float distanceToPoint = distance(closestPoint, pointPosition);
      if (distanceToPoint < minDistance) {
        minDistance = distanceToPoint;
        closestPointToClosestPoint = pointPosition;
      }
    }
  }

  return distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}

/* **** 3D Voronoi **** */

float4 voronoi_position(float3 coord)
{
  return float4(coord.x, coord.y, coord.z, 0.0f);
}

VoronoiOutput voronoi_f1(VoronoiParams params, float3 coord)
{
  float3 cellPosition_f = floor(coord);
  float3 localPosition = coord - cellPosition_f;
  int3 cellPosition = int3(cellPosition_f);

  float minDistance = FLT_MAX;
  int3 targetOffset = int3(0);
  float3 targetPosition = float3(0.0f);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        int3 cellOffset = int3(i, j, k);
        float3 pointPosition = float3(cellOffset) +
                               hash_int3_to_vec3(wrap_cell(cellPosition + cellOffset, params.period.xyz)) * params.randomness;
        float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
        if (distanceToPoint < minDistance) {
          targetOffset = cellOffset;
          minDistance = distanceToPoint;
          targetPosition = pointPosition;
        }
      }
    }
  }

  VoronoiOutput octave;
  octave.Distance = minDistance;
  octave.Color = hash_int3_to_vec3(wrap_cell(cellPosition + targetOffset, params.period.xyz));
  octave.Position = voronoi_position(targetPosition + cellPosition_f);
  return octave;
}

VoronoiOutput voronoi_smooth_f1(VoronoiParams params, float3 coord)
{
  float3 cellPosition_f = floor(coord);
  float3 localPosition = coord - cellPosition_f;
  int3 cellPosition = int3(cellPosition_f);

  float smoothDistance = 0.0f;
  float3 smoothColor = float3(0.0f);
  float3 smoothPosition = float3(0.0f);
  float h = -1.0f;
  for (int k = -2; k <= 2; k++) {
    for (int j = -2; j <= 2; j++) {
      for (int i = -2; i <= 2; i++) {
        int3 cellOffset = int3(i, j, k);
        float3 pointPosition = float3(cellOffset) +
                               hash_int3_to_vec3(wrap_cell(cellPosition + cellOffset, params.period.xyz)) * params.randomness;
        float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
        h = h == -1.0f ?
                1.0f :
                smoothstep(0.0f,
                           1.0f,
                           0.5f + 0.5f * (smoothDistance - distanceToPoint) / params.smoothness);
        float correctionFactor = params.smoothness * h * (1.0f - h);
        smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
        correctionFactor /= 1.0f + 3.0f * params.smoothness;
        float3 cellColor = hash_int3_to_vec3(wrap_cell(cellPosition + cellOffset, params.period.xyz));
        smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
        smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
      }
    }
  }

  VoronoiOutput octave;
  octave.Distance = smoothDistance;
  octave.Color = smoothColor;
  octave.Position = voronoi_position(cellPosition_f + smoothPosition);
  return octave;
}

VoronoiOutput voronoi_f2(VoronoiParams params, float3 coord)
{
  float3 cellPosition_f = floor(coord);
  float3 localPosition = coord - cellPosition_f;
  int3 cellPosition = int3(cellPosition_f);

  float distanceF1 = FLT_MAX;
  float distanceF2 = FLT_MAX;
  int3 offsetF1 = int3(0);
  float3 positionF1 = float3(0.0f);
  int3 offsetF2 = int3(0);
  float3 positionF2 = float3(0.0f);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        int3 cellOffset = int3(i, j, k);
        float3 pointPosition = float3(cellOffset) +
                               hash_int3_to_vec3(wrap_cell(cellPosition + cellOffset, params.period.xyz)) * params.randomness;
        float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
        if (distanceToPoint < distanceF1) {
          distanceF2 = distanceF1;
          distanceF1 = distanceToPoint;
          offsetF2 = offsetF1;
          offsetF1 = cellOffset;
          positionF2 = positionF1;
          positionF1 = pointPosition;
        }
        else if (distanceToPoint < distanceF2) {
          distanceF2 = distanceToPoint;
          offsetF2 = cellOffset;
          positionF2 = pointPosition;
        }
      }
    }
  }

  VoronoiOutput octave;
  octave.Distance = distanceF2;
  octave.Color = hash_int3_to_vec3(wrap_cell(cellPosition + offsetF2, params.period.xyz));
  octave.Position = voronoi_position(positionF2 + cellPosition_f);
  return octave;
}

float voronoi_distance_to_edge(VoronoiParams params, float3 coord)
{
  float3 cellPosition_f = floor(coord);
  float3 localPosition = coord - cellPosition_f;
  int3 cellPosition = int3(cellPosition_f);

  float3 vectorToClosest = float3(0.0f);
  float minDistance = FLT_MAX;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        int3 cellOffset = int3(i, j, k);
        float3 vectorToPoint = float3(cellOffset) +
                               hash_int3_to_vec3(wrap_cell(cellPosition + cellOffset, params.period.xyz)) * params.randomness -
                               localPosition;
        float distanceToPoint = dot(vectorToPoint, vectorToPoint);
        if (distanceToPoint < minDistance) {
          minDistance = distanceToPoint;
          vectorToClosest = vectorToPoint;
        }
      }
    }
  }

  minDistance = FLT_MAX;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        int3 cellOffset = int3(i, j, k);
        float3 vectorToPoint = float3(cellOffset) +
                               hash_int3_to_vec3(wrap_cell(cellPosition + cellOffset, params.period.xyz)) * params.randomness -
                               localPosition;
        float3 perpendicularToEdge = vectorToPoint - vectorToClosest;
        if (dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
          float distanceToEdge = dot((vectorToClosest + vectorToPoint) / 2.0f,
                                     normalize(perpendicularToEdge));
          minDistance = min(minDistance, distanceToEdge);
        }
      }
    }
  }

  return voronoi_finish_edge_distance(minDistance);
}

float voronoi_n_sphere_radius(VoronoiParams params, float3 coord)
{
  float3 cellPosition_f = floor(coord);
  float3 localPosition = coord - cellPosition_f;
  int3 cellPosition = int3(cellPosition_f);

  float3 closestPoint = float3(0.0f);
  int3 closestPointOffset = int3(0);
  float minDistance = FLT_MAX;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        int3 cellOffset = int3(i, j, k);
        float3 pointPosition = float3(cellOffset) +
                               hash_int3_to_vec3(wrap_cell(cellPosition + cellOffset, params.period.xyz)) * params.randomness;
        float distanceToPoint = distance(pointPosition, localPosition);
        if (distanceToPoint < minDistance) {
          minDistance = distanceToPoint;
          closestPoint = pointPosition;
          closestPointOffset = cellOffset;
        }
      }
    }
  }

  minDistance = FLT_MAX;
  float3 closestPointToClosestPoint = float3(0.0f);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        if (i == 0 && j == 0 && k == 0) {
          continue;
        }
        int3 cellOffset = int3(i, j, k) + closestPointOffset;
        float3 pointPosition = float3(cellOffset) +
                               hash_int3_to_vec3(wrap_cell(cellPosition + cellOffset, params.period.xyz)) * params.randomness;
        float distanceToPoint = distance(closestPoint, pointPosition);
        if (distanceToPoint < minDistance) {
          minDistance = distanceToPoint;
          closestPointToClosestPoint = pointPosition;
        }
      }
    }
  }

  return distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}

/* **** 4D Voronoi **** */

float4 voronoi_position(float4 coord)
{
  return coord;
}

VoronoiOutput voronoi_f1(VoronoiParams params, float4 coord)
{
  float4 cellPosition_f = floor(coord);
  float4 localPosition = coord - cellPosition_f;
  int4 cellPosition = int4(cellPosition_f);

  float minDistance = FLT_MAX;
  int4 targetOffset = int4(0);
  float4 targetPosition = float4(0.0f);
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          int4 cellOffset = int4(i, j, k, u);
          float4 pointPosition = float4(cellOffset) +
                                 hash_int4_to_vec4(wrap_cell(cellPosition + cellOffset, params.period)) * params.randomness;
          float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
          if (distanceToPoint < minDistance) {
            targetOffset = cellOffset;
            minDistance = distanceToPoint;
            targetPosition = pointPosition;
          }
        }
      }
    }
  }

  VoronoiOutput octave;
  octave.Distance = minDistance;
  octave.Color = hash_int4_to_vec3(wrap_cell(cellPosition + targetOffset, params.period));
  octave.Position = voronoi_position(targetPosition + cellPosition_f);
  return octave;
}

VoronoiOutput voronoi_smooth_f1(VoronoiParams params, float4 coord)
{
  float4 cellPosition_f = floor(coord);
  float4 localPosition = coord - cellPosition_f;
  int4 cellPosition = int4(cellPosition_f);

  float smoothDistance = 0.0f;
  float3 smoothColor = float3(0.0f);
  float4 smoothPosition = float4(0.0f);
  float h = -1.0f;
  for (int u = -2; u <= 2; u++) {
    for (int k = -2; k <= 2; k++) {
      for (int j = -2; j <= 2; j++) {
        for (int i = -2; i <= 2; i++) {
          int4 cellOffset = int4(i, j, k, u);
          float4 pointPosition = float4(cellOffset) +
                                 hash_int4_to_vec4(wrap_cell(cellPosition + cellOffset, params.period)) * params.randomness;
          float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
          h = h == -1.0f ?
                  1.0f :
                  smoothstep(0.0f,
                             1.0f,
                             0.5f + 0.5f * (smoothDistance - distanceToPoint) / params.smoothness);
          float correctionFactor = params.smoothness * h * (1.0f - h);
          smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
          correctionFactor /= 1.0f + 3.0f * params.smoothness;
          float3 cellColor = hash_int4_to_vec3(wrap_cell(cellPosition + cellOffset, params.period));
          smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
          smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
        }
      }
    }
  }

  VoronoiOutput octave;
  octave.Distance = smoothDistance;
  octave.Color = smoothColor;
  octave.Position = voronoi_position(cellPosition_f + smoothPosition);
  return octave;
}

VoronoiOutput voronoi_f2(VoronoiParams params, float4 coord)
{
  float4 cellPosition_f = floor(coord);
  float4 localPosition = coord - cellPosition_f;
  int4 cellPosition = int4(cellPosition_f);

  float distanceF1 = FLT_MAX;
  float distanceF2 = FLT_MAX;
  int4 offsetF1 = int4(0);
  float4 positionF1 = float4(0.0f);
  int4 offsetF2 = int4(0);
  float4 positionF2 = float4(0.0f);
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          int4 cellOffset = int4(i, j, k, u);
          float4 pointPosition = float4(cellOffset) +
                                 hash_int4_to_vec4(wrap_cell(cellPosition + cellOffset, params.period)) * params.randomness;
          float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
          if (distanceToPoint < distanceF1) {
            distanceF2 = distanceF1;
            distanceF1 = distanceToPoint;
            offsetF2 = offsetF1;
            offsetF1 = cellOffset;
            positionF2 = positionF1;
            positionF1 = pointPosition;
          }
          else if (distanceToPoint < distanceF2) {
            distanceF2 = distanceToPoint;
            offsetF2 = cellOffset;
            positionF2 = pointPosition;
          }
        }
      }
    }
  }

  VoronoiOutput octave;
  octave.Distance = distanceF2;
  octave.Color = hash_int4_to_vec3(wrap_cell(cellPosition + offsetF2, params.period));
  octave.Position = voronoi_position(positionF2 + cellPosition_f);
  return octave;
}

float voronoi_distance_to_edge(VoronoiParams params, float4 coord)
{
  float4 cellPosition_f = floor(coord);
  float4 localPosition = coord - cellPosition_f;
  int4 cellPosition = int4(cellPosition_f);

  float4 vectorToClosest = float4(0.0f);
  float minDistance = FLT_MAX;
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          int4 cellOffset = int4(i, j, k, u);
          float4 vectorToPoint = float4(cellOffset) +
                                 hash_int4_to_vec4(wrap_cell(cellPosition + cellOffset, params.period)) * params.randomness -
                                 localPosition;
          float distanceToPoint = dot(vectorToPoint, vectorToPoint);
          if (distanceToPoint < minDistance) {
            minDistance = distanceToPoint;
            vectorToClosest = vectorToPoint;
          }
        }
      }
    }
  }

  minDistance = FLT_MAX;
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          int4 cellOffset = int4(i, j, k, u);
          float4 vectorToPoint = float4(cellOffset) +
                                 hash_int4_to_vec4(wrap_cell(cellPosition + cellOffset, params.period)) * params.randomness -
                                 localPosition;
          float4 perpendicularToEdge = vectorToPoint - vectorToClosest;
          if (dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
            float distanceToEdge = dot((vectorToClosest + vectorToPoint) / 2.0f,
                                       normalize(perpendicularToEdge));
            minDistance = min(minDistance, distanceToEdge);
          }
        }
      }
    }
  }

  return voronoi_finish_edge_distance(minDistance);
}

float voronoi_n_sphere_radius(VoronoiParams params, float4 coord)
{
  float4 cellPosition_f = floor(coord);
  float4 localPosition = coord - cellPosition_f;
  int4 cellPosition = int4(cellPosition_f);

  float4 closestPoint = float4(0.0f);
  int4 closestPointOffset = int4(0);
  float minDistance = FLT_MAX;
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          int4 cellOffset = int4(i, j, k, u);
          float4 pointPosition = float4(cellOffset) +
                                 hash_int4_to_vec4(wrap_cell(cellPosition + cellOffset, params.period)) * params.randomness;
          float distanceToPoint = distance(pointPosition, localPosition);
          if (distanceToPoint < minDistance) {
            minDistance = distanceToPoint;
            closestPoint = pointPosition;
            closestPointOffset = cellOffset;
          }
        }
      }
    }
  }

  minDistance = FLT_MAX;
  float4 closestPointToClosestPoint = float4(0.0f);
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          if (i == 0 && j == 0 && k == 0 && u == 0) {
            continue;
          }
          int4 cellOffset = int4(i, j, k, u) + closestPointOffset;
          float4 pointPosition = float4(cellOffset) +
                                 hash_int4_to_vec4(wrap_cell(cellPosition + cellOffset, params.period)) * params.randomness;
          float distanceToPoint = distance(closestPoint, pointPosition);
          if (distanceToPoint < minDistance) {
            minDistance = distanceToPoint;
            closestPointToClosestPoint = pointPosition;
          }
        }
      }
    }
  }

  return distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}
