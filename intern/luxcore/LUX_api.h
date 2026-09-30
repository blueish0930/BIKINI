/* SPDX-FileCopyrightText: 2026 BIKINI Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup luxcore
 *
 * Native LuxCore render engine registration. Not an addon.
 */

#pragma once

namespace blender {

void LUX_engines_register();
void LUX_engines_exit();
void LUX_view_engine_free(struct RenderEngine *engine);

}  // namespace blender
