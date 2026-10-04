/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Make CGAL report failures as C++ exceptions instead of abort() / 闪退.
 */

#pragma once

#include <CGAL/assertions_behaviour.h>

#include <stdexcept>
#include <string>

namespace blender::cgal_bridge {

inline void cgal_throw_failure(const char *type,
                               const char * /*expr*/,
                               const char * /*file*/,
                               int /*line*/,
                               const char *msg)
{
  throw std::runtime_error(std::string("CGAL ") + (type ? type : "error") +
                           (msg && msg[0] ? (std::string(": ") + msg) : std::string()));
}

inline void cgal_ignore_failure(const char *, const char *, const char *, int, const char *) {}

struct CgalThrowGuard {
  CGAL::Failure_behaviour old_err;
  CGAL::Failure_behaviour old_warn;
  CGAL::Failure_function old_err_fn;
  CGAL::Failure_function old_warn_fn;

  CgalThrowGuard()
      : old_err(CGAL::set_error_behaviour(CGAL::THROW_EXCEPTION)),
        old_warn(CGAL::set_warning_behaviour(CGAL::CONTINUE)),
        old_err_fn(CGAL::set_error_handler(&cgal_throw_failure)),
        old_warn_fn(CGAL::set_warning_handler(&cgal_ignore_failure))
  {
  }

  ~CgalThrowGuard()
  {
    CGAL::set_error_handler(old_err_fn);
    CGAL::set_warning_handler(old_warn_fn);
    CGAL::set_error_behaviour(old_err);
    CGAL::set_warning_behaviour(old_warn);
  }

  CgalThrowGuard(const CgalThrowGuard &) = delete;
  CgalThrowGuard &operator=(const CgalThrowGuard &) = delete;
};

}  // namespace blender::cgal_bridge
