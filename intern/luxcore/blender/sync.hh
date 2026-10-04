/* SPDX-FileCopyrightText: 2026 BIKINI Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <string>

struct Depsgraph;
struct RenderEngine;
struct Scene;
struct View3D;

namespace blender::render::luxcore {

class Session;

struct SyncOptions {
  bool require_camera = true;
  bool use_scene_lights = true;
  bool use_scene_world = true;
  float studiolight_intensity = 1.0f;
  float studiolight_rot_z = 0.0f;
  std::string studio_light;
};

bool sync_scene(Session &session,
                RenderEngine *engine,
                Depsgraph *depsgraph,
                std::string &r_error,
                const SyncOptions &options = {});

}  // namespace blender::render::luxcore
