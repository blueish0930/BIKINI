/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/svm/node_types.h"
#include "kernel/svm/util.h"
#include "util/hash.h"

CCL_NAMESPACE_BEGIN

/*
 * SPDX-License-Identifier: MIT
 * Original code is copyright (c) 2013 Inigo Quilez.
 *
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
  NodeVoronoiFeature feature;
  NodeVoronoiDistanceMetric metric;
  /* Integer lattice wrap. Component <= 0 disables wrapping on that axis. */
  float4 period;
};

ccl_device int voronoi_period_cells(float period)
{
  if (period < 0.5f) {
    return 0;
  }
  return max(int(floorf(period + 0.5f)), 2);
}

ccl_device int voronoi_wrap_cell(int cell, float period)
{
  const int p = voronoi_period_cells(period);
  if (p <= 0) {
    return cell;
  }
  /* Floor modulo so negative neighbor cells wrap (cell -1 with period 4 -> 3). */
  return cell - p * int(floorf(float(cell) / float(p)));
}

ccl_device float voronoi_wrap_coord(float x, float period)
{
  const int p = voronoi_period_cells(period);
  if (p <= 0) {
    return x;
  }
  return x - float(p) * floorf(x / float(p));
}
ccl_device float2 voronoi_wrap_coord(float2 x, float2 period)
{
  return make_float2(voronoi_wrap_coord(x.x, period.x), voronoi_wrap_coord(x.y, period.y));
}
ccl_device float3 voronoi_wrap_coord(float3 x, float3 period)
{
  return make_float3(voronoi_wrap_coord(x.x, period.x),
                     voronoi_wrap_coord(x.y, period.y),
                     voronoi_wrap_coord(x.z, period.z));
}
ccl_device float4 voronoi_wrap_coord(float4 x, float4 period)
{
  return make_float4(voronoi_wrap_coord(x.x, period.x),
                     voronoi_wrap_coord(x.y, period.y),
                     voronoi_wrap_coord(x.z, period.z),
                     voronoi_wrap_coord(x.w, period.w));
}
ccl_device float voronoi_wrap_coord(float x, float4 period)
{
  return voronoi_wrap_coord(x, period.x);
}
ccl_device float2 voronoi_wrap_coord(float2 x, float4 period)
{
  return voronoi_wrap_coord(x, make_float2(period.x, period.y));
}
ccl_device float3 voronoi_wrap_coord(float3 x, float4 period)
{
  return voronoi_wrap_coord(x, make_float3(period.x, period.y, period.z));
}

ccl_device float4 voronoi_wrap_position(float4 pos, float4 period)
{
  pos.x = voronoi_wrap_coord(pos.x, period.x);
  pos.y = voronoi_wrap_coord(pos.y, period.y);
  pos.z = voronoi_wrap_coord(pos.z, period.z);
  pos.w = voronoi_wrap_coord(pos.w, voronoi_period_cells(period.w) > 0 ? period.w : period.x);
  return pos;
}

ccl_device float voronoi_wrap_cell(float cell, float period)
{
  return float(voronoi_wrap_cell(int(floorf(cell)), period));
}

ccl_device int2 voronoi_wrap_cell(int2 cell, float4 period)
{
  return make_int2(voronoi_wrap_cell(cell.x, period.x), voronoi_wrap_cell(cell.y, period.y));
}

ccl_device int3 voronoi_wrap_cell(int3 cell, float4 period)
{
  return make_int3(voronoi_wrap_cell(cell.x, period.x),
                   voronoi_wrap_cell(cell.y, period.y),
                   voronoi_wrap_cell(cell.z, period.z));
}

ccl_device int4 voronoi_wrap_cell(int4 cell, float4 period)
{
  return make_int4(voronoi_wrap_cell(cell.x, period.x),
                   voronoi_wrap_cell(cell.y, period.y),
                   voronoi_wrap_cell(cell.z, period.z),
                   voronoi_wrap_cell(cell.w, period.w));
}

struct VoronoiOutput {
  float distance = 0.0f;
  float3 color = zero_float3();
  float4 position = zero_float4();
};

/* ***** Distances ***** */

ccl_device float voronoi_distance(const float a, const float b)
{
  return fabsf(b - a);
}

template<typename T>
ccl_device float voronoi_distance(const T a, const T b, const ccl_private VoronoiParams &params)
{
  if (params.metric == NODE_VORONOI_EUCLIDEAN) {
    return distance(a, b);
  }
  if (params.metric == NODE_VORONOI_MANHATTAN) {
    return reduce_add(fabs(a - b));
  }
  if (params.metric == NODE_VORONOI_CHEBYCHEV) {
    return reduce_max(fabs(a - b));
  }
  if (params.metric == NODE_VORONOI_MINKOWSKI) {
    return powf(reduce_add(power(fabs(a - b), params.exponent)), 1.0f / params.exponent);
  }
  return 0.0f;
}

/* Possibly cheaper/faster version of Voronoi distance, in a way that does not change
 * logic of "which distance is the closest?". */
template<typename T>
ccl_device float voronoi_distance_bound(const T a,
                                        const T b,
                                        const ccl_private VoronoiParams &params)
{
  if (params.metric == NODE_VORONOI_EUCLIDEAN) {
    return len_squared(a - b);
  }
  if (params.metric == NODE_VORONOI_MANHATTAN) {
    return reduce_add(fabs(a - b));
  }
  if (params.metric == NODE_VORONOI_CHEBYCHEV) {
    return reduce_max(fabs(a - b));
  }
  if (params.metric == NODE_VORONOI_MINKOWSKI) {
    return reduce_add(power(fabs(a - b), params.exponent));
  }
  return 0.0f;
}

/* **** 1D Voronoi **** */

ccl_device float4 voronoi_position(const float coord)
{
  return make_float4(0.0f, 0.0f, 0.0f, coord);
}

ccl_device VoronoiOutput voronoi_f1(const ccl_private VoronoiParams &params, const float coord)
{
  const float cellPosition = floorf(coord);
  const float localPosition = coord - cellPosition;

  float minDistance = FLT_MAX;
  float targetOffset = 0.0f;
  float targetPosition = 0.0f;
  for (int i = -1; i <= 1; i++) {
    const float cellOffset = i;
    const float pointPosition = cellOffset +
                                hash_float_to_float(voronoi_wrap_cell(cellPosition + cellOffset, params.period.x)) * params.randomness;
    const float distanceToPoint = voronoi_distance(pointPosition, localPosition);
    if (distanceToPoint < minDistance) {
      targetOffset = cellOffset;
      minDistance = distanceToPoint;
      targetPosition = pointPosition;
    }
  }

  VoronoiOutput octave;
  octave.distance = minDistance;
  octave.color = hash_float_to_float3(voronoi_wrap_cell(cellPosition + targetOffset, params.period.x));
  octave.position = voronoi_position(targetPosition + cellPosition);
  return octave;
}

ccl_device VoronoiOutput voronoi_smooth_f1(const ccl_private VoronoiParams &params,
                                           const float coord)
{
  const float cellPosition = floorf(coord);
  const float localPosition = coord - cellPosition;

  float smoothDistance = 0.0f;
  float smoothPosition = 0.0f;
  float3 smoothColor = make_float3(0.0f, 0.0f, 0.0f);
  float h = -1.0f;
  for (int i = -2; i <= 2; i++) {
    const float cellOffset = i;
    const float pointPosition = cellOffset +
                                hash_float_to_float(voronoi_wrap_cell(cellPosition + cellOffset, params.period.x)) * params.randomness;
    const float distanceToPoint = voronoi_distance(pointPosition, localPosition);
    h = h == -1.0f ?
            1.0f :
            smoothstep(
                0.0f, 1.0f, 0.5f + 0.5f * (smoothDistance - distanceToPoint) / params.smoothness);
    float correctionFactor = params.smoothness * h * (1.0f - h);
    smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
    correctionFactor /= 1.0f + 3.0f * params.smoothness;
    const float3 cellColor = hash_float_to_float3(voronoi_wrap_cell(cellPosition + cellOffset, params.period.x));
    smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
    smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
  }

  VoronoiOutput octave;
  octave.distance = smoothDistance;
  octave.color = smoothColor;
  octave.position = voronoi_position(cellPosition + smoothPosition);
  return octave;
}

ccl_device VoronoiOutput voronoi_f2(const ccl_private VoronoiParams &params, const float coord)
{
  const float cellPosition = floorf(coord);
  const float localPosition = coord - cellPosition;

  float distanceF1 = FLT_MAX;
  float distanceF2 = FLT_MAX;
  float offsetF1 = 0.0f;
  float positionF1 = 0.0f;
  float offsetF2 = 0.0f;
  float positionF2 = 0.0f;
  for (int i = -1; i <= 1; i++) {
    const float cellOffset = i;
    const float pointPosition = cellOffset +
                                hash_float_to_float(voronoi_wrap_cell(cellPosition + cellOffset, params.period.x)) * params.randomness;
    const float distanceToPoint = voronoi_distance(pointPosition, localPosition);
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
  octave.distance = distanceF2;
  octave.color = hash_float_to_float3(voronoi_wrap_cell(cellPosition + offsetF2, params.period.x));
  octave.position = voronoi_position(positionF2 + cellPosition);
  return octave;
}

ccl_device float voronoi_distance_to_edge(const ccl_private VoronoiParams &params,
                                          const float coord)
{
  const float cellPosition = floorf(coord);
  const float localPosition = coord - cellPosition;

  const float midPointPosition = hash_float_to_float(voronoi_wrap_cell(cellPosition, params.period.x)) * params.randomness;
  const float leftPointPosition = -1.0f +
                                  hash_float_to_float(voronoi_wrap_cell(cellPosition - 1.0f, params.period.x)) * params.randomness;
  const float rightPointPosition = 1.0f +
                                   hash_float_to_float(voronoi_wrap_cell(cellPosition + 1.0f, params.period.x)) * params.randomness;
  const float distanceToMidLeft = fabsf((midPointPosition + leftPointPosition) / 2.0f -
                                        localPosition);
  const float distanceToMidRight = fabsf((midPointPosition + rightPointPosition) / 2.0f -
                                         localPosition);

  return min(distanceToMidLeft, distanceToMidRight);
}

ccl_device float voronoi_n_sphere_radius(const ccl_private VoronoiParams &params,
                                         const float coord)
{
  const float coord_p = voronoi_wrap_coord(coord, params.period.x);
  const float cellPosition = floorf(coord_p);
  const float localPosition = coord_p - cellPosition;

  float closestPoint = 0.0f;
  float closestPointOffset = 0.0f;
  float minDistance = FLT_MAX;
  for (int i = -1; i <= 1; i++) {
    const float cellOffset = i;
    const float pointPosition = cellOffset +
                                hash_float_to_float(voronoi_wrap_cell(cellPosition + cellOffset, params.period.x)) * params.randomness;
    const float distanceToPoint = fabsf(pointPosition - localPosition);
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
    const float cellOffset = i + closestPointOffset;
    const float pointPosition = cellOffset +
                                hash_float_to_float(voronoi_wrap_cell(cellPosition + cellOffset, params.period.x)) * params.randomness;
    const float distanceToPoint = fabsf(closestPoint - pointPosition);
    if (distanceToPoint < minDistance) {
      minDistance = distanceToPoint;
      closestPointToClosestPoint = pointPosition;
    }
  }

  return fabsf(closestPointToClosestPoint - closestPoint) / 2.0f;
}

/* **** 2D Voronoi **** */

ccl_device float4 voronoi_position(const float2 coord)
{
  return make_float4(coord.x, coord.y, 0.0f, 0.0f);
}

ccl_device VoronoiOutput voronoi_f1(const ccl_private VoronoiParams &params, const float2 coord)
{
  const float2 cellPosition_f = floor(coord);
  const float2 localPosition = coord - cellPosition_f;
  const int2 cellPosition = make_int2(cellPosition_f);

  float minDistance = FLT_MAX;
  int2 targetOffset = make_int2(0);
  float2 targetPosition = make_float2(0.0f, 0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      const int2 cellOffset = make_int2(i, j);
      const float2 pointPosition = make_float2(cellOffset) +
                                   hash_int2_to_float2(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                       params.randomness;
      const float distanceToPoint = voronoi_distance_bound(pointPosition, localPosition, params);
      if (distanceToPoint < minDistance) {
        targetOffset = cellOffset;
        minDistance = distanceToPoint;
        targetPosition = pointPosition;
      }
    }
  }

  VoronoiOutput octave;
  octave.distance = voronoi_distance(targetPosition, localPosition, params);
  octave.color = hash_int2_to_float3(voronoi_wrap_cell(cellPosition + targetOffset, params.period));
  octave.position = voronoi_position(targetPosition + cellPosition_f);
  return octave;
}

ccl_device VoronoiOutput voronoi_smooth_f1(const ccl_private VoronoiParams &params,
                                           const float2 coord)
{
  const float2 cellPosition_f = floor(coord);
  const float2 localPosition = coord - cellPosition_f;
  const int2 cellPosition = make_int2(cellPosition_f);

  float smoothDistance = 0.0f;
  float3 smoothColor = make_float3(0.0f, 0.0f, 0.0f);
  float2 smoothPosition = make_float2(0.0f, 0.0f);
  float h = -1.0f;
  for (int j = -2; j <= 2; j++) {
    for (int i = -2; i <= 2; i++) {
      const int2 cellOffset = make_int2(i, j);
      const float2 pointPosition = make_float2(cellOffset) +
                                   hash_int2_to_float2(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                       params.randomness;
      const float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
      h = h == -1.0f ?
              1.0f :
              smoothstep(0.0f,
                         1.0f,
                         0.5f + 0.5f * (smoothDistance - distanceToPoint) / params.smoothness);
      float correctionFactor = params.smoothness * h * (1.0f - h);
      smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
      correctionFactor /= 1.0f + 3.0f * params.smoothness;
      const float3 cellColor = hash_int2_to_float3(voronoi_wrap_cell(cellPosition + cellOffset, params.period));
      smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
      smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
    }
  }

  VoronoiOutput octave;
  octave.distance = smoothDistance;
  octave.color = smoothColor;
  octave.position = voronoi_position(cellPosition_f + smoothPosition);
  return octave;
}

ccl_device VoronoiOutput voronoi_f2(const ccl_private VoronoiParams &params, const float2 coord)
{
  const float2 cellPosition_f = floor(coord);
  const float2 localPosition = coord - cellPosition_f;
  const int2 cellPosition = make_int2(cellPosition_f);

  float distanceF1 = FLT_MAX;
  float distanceF2 = FLT_MAX;
  int2 offsetF1 = make_int2(0);
  float2 positionF1 = make_float2(0.0f, 0.0f);
  int2 offsetF2 = make_int2(0);
  float2 positionF2 = make_float2(0.0f, 0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      const int2 cellOffset = make_int2(i, j);
      const float2 pointPosition = make_float2(cellOffset) +
                                   hash_int2_to_float2(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                       params.randomness;
      const float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
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
  octave.distance = distanceF2;
  octave.color = hash_int2_to_float3(voronoi_wrap_cell(cellPosition + offsetF2, params.period));
  octave.position = voronoi_position(positionF2 + cellPosition_f);
  return octave;
}

ccl_device float voronoi_distance_to_edge(const ccl_private VoronoiParams &params,
                                          const float2 coord)
{
  const float2 cellPosition_f = floor(coord);
  const float2 localPosition = coord - cellPosition_f;
  const int2 cellPosition = make_int2(cellPosition_f);

  float2 vectorToClosest = make_float2(0.0f, 0.0f);
  float minDistance = FLT_MAX;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      const int2 cellOffset = make_int2(i, j);
      const float2 vectorToPoint = make_float2(cellOffset) +
                                   hash_int2_to_float2(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                       params.randomness -
                                   localPosition;
      const float distanceToPoint = dot(vectorToPoint, vectorToPoint);
      if (distanceToPoint < minDistance) {
        minDistance = distanceToPoint;
        vectorToClosest = vectorToPoint;
      }
    }
  }

  minDistance = FLT_MAX;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      const int2 cellOffset = make_int2(i, j);
      const float2 vectorToPoint = make_float2(cellOffset) +
                                   hash_int2_to_float2(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                       params.randomness -
                                   localPosition;
      const float2 perpendicularToEdge = vectorToPoint - vectorToClosest;
      if (dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
        const float distanceToEdge = dot((vectorToClosest + vectorToPoint) / 2.0f,
                                         normalize(perpendicularToEdge));
        minDistance = min(minDistance, distanceToEdge);
      }
    }
  }

  return minDistance;
}

ccl_device float voronoi_n_sphere_radius(const ccl_private VoronoiParams &params,
                                         const float2 coord)
{
  const float2 coord_p = voronoi_wrap_coord(coord, params.period);
  const float2 cellPosition_f = floor(coord_p);
  const float2 localPosition = coord_p - cellPosition_f;
  const int2 cellPosition = make_int2(cellPosition_f);

  float2 closestPoint = make_float2(0.0f, 0.0f);
  int2 closestPointOffset = make_int2(0);
  float minDistanceSq = FLT_MAX;
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      const int2 cellOffset = make_int2(i, j);
      const float2 pointPosition = make_float2(cellOffset) +
                                   hash_int2_to_float2(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                       params.randomness;
      const float distanceToPointSq = len_squared(pointPosition - localPosition);
      if (distanceToPointSq < minDistanceSq) {
        minDistanceSq = distanceToPointSq;
        closestPoint = pointPosition;
        closestPointOffset = cellOffset;
      }
    }
  }

  minDistanceSq = FLT_MAX;
  float2 closestPointToClosestPoint = make_float2(0.0f, 0.0f);
  for (int j = -1; j <= 1; j++) {
    for (int i = -1; i <= 1; i++) {
      if (i == 0 && j == 0) {
        continue;
      }
      const int2 cellOffset = make_int2(i, j) + closestPointOffset;
      const float2 pointPosition = make_float2(cellOffset) +
                                   hash_int2_to_float2(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                       params.randomness;
      const float distanceToPointSq = len_squared(closestPoint - pointPosition);
      if (distanceToPointSq < minDistanceSq) {
        minDistanceSq = distanceToPointSq;
        closestPointToClosestPoint = pointPosition;
      }
    }
  }

  return distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}

/* **** 3D Voronoi **** */

ccl_device float4 voronoi_position(const float3 coord)
{
  return make_float4(coord);
}

ccl_device VoronoiOutput voronoi_f1(const ccl_private VoronoiParams &params, const float3 coord)
{
  const float3 cellPosition_f = floor(coord);
  const float3 localPosition = coord - cellPosition_f;
  const int3 cellPosition = make_int3(cellPosition_f);

  float minDistance = FLT_MAX;
  int3 targetOffset = make_int3(0);
  float3 targetPosition = make_float3(0.0f, 0.0f, 0.0f);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        const int3 cellOffset = make_int3(i, j, k);
        const float3 pointPosition = make_float3(cellOffset) +
                                     hash_int3_to_float3(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                         params.randomness;
        const float distanceToPoint = voronoi_distance_bound(pointPosition, localPosition, params);
        if (distanceToPoint < minDistance) {
          targetOffset = cellOffset;
          minDistance = distanceToPoint;
          targetPosition = pointPosition;
        }
      }
    }
  }

  VoronoiOutput octave;
  octave.distance = voronoi_distance(targetPosition, localPosition, params);
  octave.color = hash_int3_to_float3(voronoi_wrap_cell(cellPosition + targetOffset, params.period));
  octave.position = voronoi_position(targetPosition + cellPosition_f);
  return octave;
}

ccl_device VoronoiOutput voronoi_smooth_f1(const ccl_private VoronoiParams &params,
                                           const float3 coord)
{
  const float3 cellPosition_f = floor(coord);
  const float3 localPosition = coord - cellPosition_f;
  const int3 cellPosition = make_int3(cellPosition_f);

  float smoothDistance = 0.0f;
  float3 smoothColor = make_float3(0.0f, 0.0f, 0.0f);
  float3 smoothPosition = make_float3(0.0f, 0.0f, 0.0f);
  float h = -1.0f;
  for (int k = -2; k <= 2; k++) {
    for (int j = -2; j <= 2; j++) {
      for (int i = -2; i <= 2; i++) {
        const int3 cellOffset = make_int3(i, j, k);
        const float3 pointPosition = make_float3(cellOffset) +
                                     hash_int3_to_float3(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                         params.randomness;
        const float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
        h = h == -1.0f ?
                1.0f :
                smoothstep(0.0f,
                           1.0f,
                           0.5f + 0.5f * (smoothDistance - distanceToPoint) / params.smoothness);
        float correctionFactor = params.smoothness * h * (1.0f - h);
        smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
        correctionFactor /= 1.0f + 3.0f * params.smoothness;
        const float3 cellColor = hash_int3_to_float3(voronoi_wrap_cell(cellPosition + cellOffset, params.period));
        smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
        smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
      }
    }
  }

  VoronoiOutput octave;
  octave.distance = smoothDistance;
  octave.color = smoothColor;
  octave.position = voronoi_position(cellPosition_f + smoothPosition);
  return octave;
}

ccl_device VoronoiOutput voronoi_f2(const ccl_private VoronoiParams &params, const float3 coord)
{
  const float3 cellPosition_f = floor(coord);
  const float3 localPosition = coord - cellPosition_f;
  const int3 cellPosition = make_int3(cellPosition_f);

  float distanceF1 = FLT_MAX;
  float distanceF2 = FLT_MAX;
  int3 offsetF1 = make_int3(0);
  float3 positionF1 = make_float3(0.0f, 0.0f, 0.0f);
  int3 offsetF2 = make_int3(0);
  float3 positionF2 = make_float3(0.0f, 0.0f, 0.0f);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        const int3 cellOffset = make_int3(i, j, k);
        const float3 pointPosition = make_float3(cellOffset) +
                                     hash_int3_to_float3(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                         params.randomness;
        const float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
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
  octave.distance = distanceF2;
  octave.color = hash_int3_to_float3(voronoi_wrap_cell(cellPosition + offsetF2, params.period));
  octave.position = voronoi_position(positionF2 + cellPosition_f);
  return octave;
}

ccl_device float voronoi_distance_to_edge(const ccl_private VoronoiParams &params,
                                          const float3 coord)
{
  const float3 cellPosition_f = floor(coord);
  const float3 localPosition = coord - cellPosition_f;
  const int3 cellPosition = make_int3(cellPosition_f);

  float3 vectorToClosest = make_float3(0.0f, 0.0f, 0.0f);
  float minDistance = FLT_MAX;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        const int3 cellOffset = make_int3(i, j, k);
        const float3 vectorToPoint = make_float3(cellOffset) +
                                     hash_int3_to_float3(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                         params.randomness -
                                     localPosition;
        const float distanceToPoint = dot(vectorToPoint, vectorToPoint);
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
        const int3 cellOffset = make_int3(i, j, k);
        const float3 vectorToPoint = make_float3(cellOffset) +
                                     hash_int3_to_float3(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                         params.randomness -
                                     localPosition;
        const float3 perpendicularToEdge = vectorToPoint - vectorToClosest;
        if (dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
          const float distanceToEdge = dot((vectorToClosest + vectorToPoint) / 2.0f,
                                           normalize(perpendicularToEdge));
          minDistance = min(minDistance, distanceToEdge);
        }
      }
    }
  }

  return minDistance;
}

ccl_device float voronoi_n_sphere_radius(const ccl_private VoronoiParams &params,
                                         const float3 coord)
{
  const float3 coord_p = voronoi_wrap_coord(coord, params.period);
  const float3 cellPosition_f = floor(coord_p);
  const float3 localPosition = coord_p - cellPosition_f;
  const int3 cellPosition = make_int3(cellPosition_f);

  float3 closestPoint = make_float3(0.0f, 0.0f, 0.0f);
  int3 closestPointOffset = make_int3(0);
  float minDistanceSq = FLT_MAX;
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        const int3 cellOffset = make_int3(i, j, k);
        const float3 pointPosition = make_float3(cellOffset) +
                                     hash_int3_to_float3(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                         params.randomness;
        const float distanceToPointSq = len_squared(pointPosition - localPosition);
        if (distanceToPointSq < minDistanceSq) {
          minDistanceSq = distanceToPointSq;
          closestPoint = pointPosition;
          closestPointOffset = cellOffset;
        }
      }
    }
  }

  minDistanceSq = FLT_MAX;
  float3 closestPointToClosestPoint = make_float3(0.0f, 0.0f, 0.0f);
  for (int k = -1; k <= 1; k++) {
    for (int j = -1; j <= 1; j++) {
      for (int i = -1; i <= 1; i++) {
        if (i == 0 && j == 0 && k == 0) {
          continue;
        }
        const int3 cellOffset = make_int3(i, j, k) + closestPointOffset;
        const float3 pointPosition = make_float3(cellOffset) +
                                     hash_int3_to_float3(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                         params.randomness;
        const float distanceToPointSq = len_squared(closestPoint - pointPosition);
        if (distanceToPointSq < minDistanceSq) {
          minDistanceSq = distanceToPointSq;
          closestPointToClosestPoint = pointPosition;
        }
      }
    }
  }

  return distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}

/* **** 4D Voronoi **** */

ccl_device float4 voronoi_position(const float4 coord)
{
  return coord;
}

ccl_device VoronoiOutput voronoi_f1(const ccl_private VoronoiParams &params, const float4 coord)
{
  const float4 cellPosition_f = floor(coord);
  const float4 localPosition = coord - cellPosition_f;
  const int4 cellPosition = make_int4(cellPosition_f);

  float minDistance = FLT_MAX;
  int4 targetOffset = zero_int4();
  float4 targetPosition = zero_float4();
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          const int4 cellOffset = make_int4(i, j, k, u);
          const float4 pointPosition = make_float4(cellOffset) +
                                       hash_int4_to_float4(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                           params.randomness;
          const float distanceToPoint = voronoi_distance_bound(
              pointPosition, localPosition, params);
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
  octave.distance = voronoi_distance(targetPosition, localPosition, params);
  octave.color = hash_int4_to_float3(voronoi_wrap_cell(cellPosition + targetOffset, params.period));
  octave.position = voronoi_position(targetPosition + cellPosition_f);
  return octave;
}

ccl_device VoronoiOutput voronoi_smooth_f1(const ccl_private VoronoiParams &params,
                                           const float4 coord)
{
  const float4 cellPosition_f = floor(coord);
  const float4 localPosition = coord - cellPosition_f;
  const int4 cellPosition = make_int4(cellPosition_f);

  float smoothDistance = 0.0f;
  float3 smoothColor = make_float3(0.0f, 0.0f, 0.0f);
  float4 smoothPosition = zero_float4();
  float h = -1.0f;
  for (int u = -2; u <= 2; u++) {
    for (int k = -2; k <= 2; k++) {
      for (int j = -2; j <= 2; j++) {
        for (int i = -2; i <= 2; i++) {
          const int4 cellOffset = make_int4(i, j, k, u);
          const float4 pointPosition = make_float4(cellOffset) +
                                       hash_int4_to_float4(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                           params.randomness;
          const float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
          h = h == -1.0f ?
                  1.0f :
                  smoothstep(0.0f,
                             1.0f,
                             0.5f + 0.5f * (smoothDistance - distanceToPoint) / params.smoothness);
          float correctionFactor = params.smoothness * h * (1.0f - h);
          smoothDistance = mix(smoothDistance, distanceToPoint, h) - correctionFactor;
          correctionFactor /= 1.0f + 3.0f * params.smoothness;
          const float3 cellColor = hash_int4_to_float3(voronoi_wrap_cell(cellPosition + cellOffset, params.period));
          smoothColor = mix(smoothColor, cellColor, h) - correctionFactor;
          smoothPosition = mix(smoothPosition, pointPosition, h) - correctionFactor;
        }
      }
    }
  }

  VoronoiOutput octave;
  octave.distance = smoothDistance;
  octave.color = smoothColor;
  octave.position = voronoi_position(cellPosition_f + smoothPosition);
  return octave;
}

ccl_device VoronoiOutput voronoi_f2(const ccl_private VoronoiParams &params, const float4 coord)
{
  const float4 cellPosition_f = floor(coord);
  const float4 localPosition = coord - cellPosition_f;
  const int4 cellPosition = make_int4(cellPosition_f);

  float distanceF1 = FLT_MAX;
  float distanceF2 = FLT_MAX;
  int4 offsetF1 = zero_int4();
  float4 positionF1 = zero_float4();
  int4 offsetF2 = zero_int4();
  float4 positionF2 = zero_float4();
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          const int4 cellOffset = make_int4(i, j, k, u);
          const float4 pointPosition = make_float4(cellOffset) +
                                       hash_int4_to_float4(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                           params.randomness;
          const float distanceToPoint = voronoi_distance(pointPosition, localPosition, params);
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
  octave.distance = distanceF2;
  octave.color = hash_int4_to_float3(voronoi_wrap_cell(cellPosition + offsetF2, params.period));
  octave.position = voronoi_position(positionF2 + cellPosition_f);
  return octave;
}

ccl_device float voronoi_distance_to_edge(const ccl_private VoronoiParams &params,
                                          const float4 coord)
{
  const float4 cellPosition_f = floor(coord);
  const float4 localPosition = coord - cellPosition_f;
  const int4 cellPosition = make_int4(cellPosition_f);

  float4 vectorToClosest = zero_float4();
  float minDistance = FLT_MAX;
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          const int4 cellOffset = make_int4(i, j, k, u);
          const float4 vectorToPoint = make_float4(cellOffset) +
                                       hash_int4_to_float4(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                           params.randomness -
                                       localPosition;
          const float distanceToPoint = dot(vectorToPoint, vectorToPoint);
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
          const int4 cellOffset = make_int4(i, j, k, u);
          const float4 vectorToPoint = make_float4(cellOffset) +
                                       hash_int4_to_float4(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                           params.randomness -
                                       localPosition;
          const float4 perpendicularToEdge = vectorToPoint - vectorToClosest;
          if (dot(perpendicularToEdge, perpendicularToEdge) > 0.0001f) {
            const float distanceToEdge = dot((vectorToClosest + vectorToPoint) / 2.0f,
                                             normalize(perpendicularToEdge));
            minDistance = min(minDistance, distanceToEdge);
          }
        }
      }
    }
  }

  return minDistance;
}

ccl_device float voronoi_n_sphere_radius(const ccl_private VoronoiParams &params,
                                         const float4 coord)
{
  const float4 coord_p = voronoi_wrap_coord(coord, params.period);
  const float4 cellPosition_f = floor(coord_p);
  const float4 localPosition = coord_p - cellPosition_f;
  const int4 cellPosition = make_int4(cellPosition_f);

  float4 closestPoint = zero_float4();
  int4 closestPointOffset = zero_int4();
  float minDistanceSq = FLT_MAX;
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          const int4 cellOffset = make_int4(i, j, k, u);
          const float4 pointPosition = make_float4(cellOffset) +
                                       hash_int4_to_float4(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                           params.randomness;
          const float distanceToPointSq = len_squared(pointPosition - localPosition);
          if (distanceToPointSq < minDistanceSq) {
            minDistanceSq = distanceToPointSq;
            closestPoint = pointPosition;
            closestPointOffset = cellOffset;
          }
        }
      }
    }
  }

  minDistanceSq = FLT_MAX;
  float4 closestPointToClosestPoint = zero_float4();
  for (int u = -1; u <= 1; u++) {
    for (int k = -1; k <= 1; k++) {
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          if (i == 0 && j == 0 && k == 0 && u == 0) {
            continue;
          }
          const int4 cellOffset = make_int4(i, j, k, u) + closestPointOffset;
          const float4 pointPosition = make_float4(cellOffset) +
                                       hash_int4_to_float4(voronoi_wrap_cell(cellPosition + cellOffset, params.period)) *
                                           params.randomness;
          const float distanceToPointSq = len_squared(closestPoint - pointPosition);
          if (distanceToPointSq < minDistanceSq) {
            minDistanceSq = distanceToPointSq;
            closestPointToClosestPoint = pointPosition;
          }
        }
      }
    }
  }

  return distance(closestPointToClosestPoint, closestPoint) / 2.0f;
}

/* **** Fractal Voronoi **** */

/* The fractalization logic is the same as for fBM Noise, except that some additions are replaced
 * by lerps. */
template<typename T>
ccl_device VoronoiOutput fractal_voronoi_x_fx(const ccl_private VoronoiParams &params,
                                              const T coord)
{
  float amplitude = 1.0f;
  float max_amplitude = 0.0f;
  float scale = 1.0f;

  VoronoiOutput output;
  const bool zero_input = params.detail == 0.0f || params.roughness == 0.0f;

  for (int i = 0; i <= ceilf(params.detail); ++i) {
    VoronoiParams octave_params = params;
    octave_params.period = params.period * scale;
    const T octave_coord = voronoi_wrap_coord(coord * scale, octave_params.period);
    VoronoiOutput octave = (params.feature == NODE_VORONOI_F2) ?
                               voronoi_f2(octave_params, octave_coord) :
                           (params.feature == NODE_VORONOI_SMOOTH_F1 &&
                            params.smoothness != 0.0f) ?
                               voronoi_smooth_f1(octave_params, octave_coord) :
                               voronoi_f1(octave_params, octave_coord);
    octave.position = voronoi_wrap_position(octave.position, octave_params.period);

    if (zero_input) {
      max_amplitude = 1.0f;
      output = octave;
      break;
    }
    if (i <= params.detail) {
      max_amplitude += amplitude;
      output.distance += octave.distance * amplitude;
      output.color += octave.color * amplitude;
      output.position = mix(output.position, octave.position / scale, amplitude);
      scale *= params.lacunarity;
      amplitude *= params.roughness;
    }
    else {
      const float remainder = params.detail - floorf(params.detail);
      if (remainder != 0.0f) {
        max_amplitude = mix(max_amplitude, max_amplitude + amplitude, remainder);
        output.distance = mix(
            output.distance, output.distance + octave.distance * amplitude, remainder);
        output.color = mix(output.color, output.color + octave.color * amplitude, remainder);
        output.position = mix(
            output.position, mix(output.position, octave.position / scale, amplitude), remainder);
      }
    }
  }

  if (params.normalize) {
    output.distance /= max_amplitude * params.max_distance;
    output.color /= max_amplitude;
  }

  output.position = safe_divide(output.position, params.scale);

  return output;
}

/* The fractalization logic is the same as for fBM Noise, except that some additions are replaced
 * by lerps. */
template<typename T>
ccl_device float fractal_voronoi_distance_to_edge(const ccl_private VoronoiParams &params,
                                                  const T coord)
{
  float amplitude = 1.0f;
  float max_amplitude = params.max_distance;
  float scale = 1.0f;
  float distance = 8.0f;

  const bool zero_input = params.detail == 0.0f || params.roughness == 0.0f;

  for (int i = 0; i <= ceilf(params.detail); ++i) {
    VoronoiParams octave_params = params;
    octave_params.period = params.period * scale;
    const float octave_distance = voronoi_distance_to_edge(
        octave_params, voronoi_wrap_coord(coord * scale, octave_params.period));

    if (zero_input) {
      distance = octave_distance;
      break;
    }
    if (i <= params.detail) {
      max_amplitude = mix(max_amplitude, params.max_distance / scale, amplitude);
      distance = mix(distance, min(distance, octave_distance / scale), amplitude);
      scale *= params.lacunarity;
      amplitude *= params.roughness;
    }
    else {
      const float remainder = params.detail - floorf(params.detail);
      if (remainder != 0.0f) {
        const float lerp_amplitude = mix(max_amplitude, params.max_distance / scale, amplitude);
        max_amplitude = mix(max_amplitude, lerp_amplitude, remainder);
        const float lerp_distance = mix(
            distance, min(distance, octave_distance / scale), amplitude);
        distance = mix(distance, min(distance, lerp_distance), remainder);
      }
    }
  }

  if (params.normalize) {
    distance /= max_amplitude;
  }

  return distance;
}

ccl_device void svm_voronoi_output(ccl_private float *ccl_restrict stack,
                                   const ccl_global SVMNodeTexVoronoi &ccl_restrict node,
                                   const float distance,
                                   const float3 color,
                                   const float3 position,
                                   const float w,
                                   const float radius)
{
  if (stack_valid(node.distance_offset)) {
    stack_store_float(stack, node.distance_offset, distance);
  }
  if (stack_valid(node.color_offset)) {
    stack_store_float3(stack, node.color_offset, color);
  }
  if (stack_valid(node.position_offset)) {
    stack_store_float3(stack, node.position_offset, position);
  }
  if (stack_valid(node.w_out_offset)) {
    stack_store_float(stack, node.w_out_offset, w);
  }
  if (stack_valid(node.radius_offset)) {
    stack_store_float(stack, node.radius_offset, radius);
  }
}

template<uint64_t node_feature_mask>
ccl_device_noinline void svm_node_tex_voronoi(
    ccl_private float *ccl_restrict stack, const ccl_global SVMNodeTexVoronoi &ccl_restrict node)
{
  /* Read from stack. */
  float3 coord = stack_load_float3(stack, node.coord);
  float w = stack_load(stack, node.w);

  VoronoiParams params;
  params.feature = node.feature;
  params.metric = node.metric;
  params.scale = stack_load(stack, node.scale);
  params.detail = stack_load(stack, node.detail);
  params.roughness = stack_load(stack, node.roughness);
  params.lacunarity = stack_load(stack, node.lacunarity);
  params.smoothness = stack_load(stack, node.smoothness);
  params.exponent = stack_load(stack, node.exponent);
  params.randomness = stack_load(stack, node.randomness);
  params.max_distance = 0.0f;
  params.normalize = node.normalize;
  {
    const float3 period3 = stack_load(stack, node.period);
    const float period_w = stack_load(stack, node.period_w);
    if (node.tiling) {
      params.period = make_float4(period3.x, period3.y, period3.z, period_w);
    }
    else {
      params.period = zero_float4();
    }
  }

  params.detail = clamp(params.detail, 0.0f, 15.0f);
  params.roughness = clamp(params.roughness, 0.0f, 1.0f);
  params.randomness = clamp(params.randomness, 0.0f, 1.0f);
  params.smoothness = clamp(params.smoothness / 2.0f, 0.0f, 0.5f);

  coord *= params.scale;
  w *= params.scale;

  /* Compute output, specialized for each dimension. */
  switch (params.feature) {
    case NODE_VORONOI_DISTANCE_TO_EDGE: {
      float distance = 0.0f;
      params.max_distance = 0.5f + 0.5f * params.randomness;
      switch (node.dimensions) {
        case 1:
          distance = fractal_voronoi_distance_to_edge(params, w);
          break;
        case 2:
          distance = fractal_voronoi_distance_to_edge(params, make_float2(coord));
          break;
        case 3:
          distance = fractal_voronoi_distance_to_edge(params, coord);
          break;
        case 4:
          distance = fractal_voronoi_distance_to_edge(params, make_float4(coord, w));
          break;
        default:
          kernel_assert(0);
          break;
      }

      svm_voronoi_output(stack, node, distance, zero_float3(), zero_float3(), 0.0f, 0.0f);
      break;
    }
    case NODE_VORONOI_N_SPHERE_RADIUS: {
      float radius = 0.0f;
      switch (node.dimensions) {
        case 1:
          radius = voronoi_n_sphere_radius(params, w);
          break;
        case 2:
          radius = voronoi_n_sphere_radius(params, make_float2(coord));
          break;
        case 3:
          radius = voronoi_n_sphere_radius(params, coord);
          break;
        case 4:
          radius = voronoi_n_sphere_radius(params, make_float4(coord, w));
          break;
        default:
          kernel_assert(0);
          break;
      }

      svm_voronoi_output(stack, node, 0.0f, zero_float3(), zero_float3(), 0.0f, radius);
      break;
    }
    default: {
      VoronoiOutput output;
      switch (node.dimensions) {
        case 1:
          params.max_distance = (0.5f + 0.5f * params.randomness) *
                                ((params.feature == NODE_VORONOI_F2) ? 2.0f : 1.0f);
          output = fractal_voronoi_x_fx(params, w);
          break;
        case 2:
          IF_KERNEL_NODES_FEATURE(VORONOI_EXTRA)
          {
            params.max_distance = voronoi_distance(zero_float2(),
                                                   make_float2(0.5f + 0.5f * params.randomness,
                                                               0.5f + 0.5f * params.randomness),
                                                   params) *
                                  ((params.feature == NODE_VORONOI_F2) ? 2.0f : 1.0f);
            output = fractal_voronoi_x_fx(params, make_float2(coord));
          }
          break;
        case 3:
          IF_KERNEL_NODES_FEATURE(VORONOI_EXTRA)
          {
            params.max_distance = voronoi_distance(zero_float3(),
                                                   make_float3(0.5f + 0.5f * params.randomness,
                                                               0.5f + 0.5f * params.randomness,
                                                               0.5f + 0.5f * params.randomness),
                                                   params) *
                                  ((params.feature == NODE_VORONOI_F2) ? 2.0f : 1.0f);
            output = fractal_voronoi_x_fx(params, coord);
          }
          break;
        case 4:
          IF_KERNEL_NODES_FEATURE(VORONOI_EXTRA)
          {
            params.max_distance = voronoi_distance(zero_float4(),
                                                   make_float4(0.5f + 0.5f * params.randomness,
                                                               0.5f + 0.5f * params.randomness,
                                                               0.5f + 0.5f * params.randomness,
                                                               0.5f + 0.5f * params.randomness),
                                                   params) *
                                  ((params.feature == NODE_VORONOI_F2) ? 2.0f : 1.0f);
            output = fractal_voronoi_x_fx(params, make_float4(coord, w));
          }
          break;
        default:
          kernel_assert(0);
          break;
      }

      svm_voronoi_output(stack,
                         node,
                         output.distance,
                         output.color,
                         make_float3(output.position),
                         output.position.w,
                         0.0f);
      break;
    }
  }
}

CCL_NAMESPACE_END
