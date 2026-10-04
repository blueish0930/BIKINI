/* SPDX-FileCopyrightText: 2026 BIKINI Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup intern_luxcore
 *
 * Forced in ahead of luxcore.cpp.
 *
 * OpenImageIO's detail/fmt.h defines FMT_HEADER_ONLY and then includes a
 * vendored copy of the same fmt 12.1 headers Blender ships as fmt.lib.
 * That instantiates fmt::v12 into the translation unit. Those definitions
 * collide with fmt.lib (LNK4006). /FORCE:MULTIPLE hides the collision and
 * produces LNK4088.
 *
 * Including the compiled fmt headers first sets FMT_FORMAT_H_ while
 * FMT_HEADER_ONLY is still unset, so format-inl.h is not pulled in and the
 * later header-only include is skipped. The symbols stay in fmt.lib.
 */

#pragma once

#include <fmt/format.h>
