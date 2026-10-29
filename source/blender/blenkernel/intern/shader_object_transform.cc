/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <cstdio>

#include "BKE_lib_id.hh"
#include "BKE_shader_object_transform.hh"
#include "BLI_math_euler.hh"
#include "BLI_math_matrix.hh"
#include "DEG_depsgraph_query.hh"
#include "DNA_object_types.h"

namespace blender::bke {

std::string shader_object_transform_attribute_name(const Object *object, const int field)
{
  const uint32_t uid = object ? DEG_get_original(object)->id.session_uid : 0;
  return ".shader_transform:" + std::to_string(uid) + ":" + std::to_string(field);
}

bool lookup_shader_object_transform_attribute(const StringRefNull name, const Depsgraph *depsgraph,
                                              float4 &r_value)
{
  if (!name.startswith(".shader_transform:")) {
    return false;
  }
  unsigned int uid;
  int field, consumed = 0;
  if (std::sscanf(name.c_str(), ".shader_transform:%u:%d%n", &uid, &field, &consumed) != 2 ||
      consumed != name.size() || field < 0 || field > 6) {
    return false;
  }
  const Object *object = nullptr;
  if (uid && depsgraph) {
    if (ID *id = BKE_libblock_find_session_uid(DEG_get_bmain(depsgraph), ID_OB, uid)) {
      object = DEG_get_evaluated(depsgraph, reinterpret_cast<Object *>(id));
    }
  }
  const float4x4 transform = object ? object->object_to_world() : float4x4::identity();
  if (field < 4) {
    r_value = transform[field];
  }
  else if (field == 6) {
    const float seed = object ? float(int(DEG_get_original(object)->id.session_uid)) : 0.0f;
    r_value = float4(seed);
  }
  else {
    float3 location, scale;
    math::Quaternion rotation;
    math::to_loc_rot_scale_safe<true>(transform, location, rotation, scale);
    if (!object) {
      scale = float3(0.0f);
    }
    r_value =
        field == 4 ? float4(rotation.w, rotation.x, rotation.y, rotation.z) : float4(scale, 0.0f);
  }
  return true;
}
} // namespace blender::bke
