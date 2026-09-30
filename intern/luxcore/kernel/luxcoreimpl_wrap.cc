/* SPDX-FileCopyrightText: 2026 BIKINI Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "luxcore/luxcore.h"

#include <fmt/format.h>

template<> struct fmt::formatter<luxcore::Camera::CameraType> : fmt::formatter<int> {
  auto format(luxcore::Camera::CameraType t, format_context &ctx) const
  {
    return fmt::formatter<int>::format(static_cast<int>(t), ctx);
  }
};

#include "luxcoreimpl.cpp"
