/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup blt
 */

#pragma once

namespace blender {

/**
 * Tooltip translation that also works outside of the main thread, for messages that are built
 * during evaluation (node warnings). #TIP_ leaves the text untranslated there, which made the
 * language of a message depend on the thread that happened to run the node.
 *
 * Only looks in the catalog, translations registered by add-ons are skipped.
 */
const char *BLT_translate_do_tooltip_any_thread(const char *msgid);

}  // namespace blender
