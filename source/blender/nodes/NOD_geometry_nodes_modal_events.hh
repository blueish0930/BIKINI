/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <string>

#include "BLI_vector.hh"

#include "DNA_node_types.h"

namespace blender {
struct bNodeTree;
}

namespace blender::nodes {

/** One event defined by one or more Modal Event nodes. */
struct ModalEvent {
  std::string name;
  std::string description;

  friend bool operator==(const ModalEvent &a, const ModalEvent &b) = default;
};

/**
 * One default key binding for an event, and the node group that defines it. Bindings from nested
 * groups are additive: an event ends up with the union of the bindings that every group in the
 * hierarchy defines for it. Mirrors the relevant fields of #wmKeyMapItem.
 */
struct ModalBinding {
  /** Copy of the binding as the defining node group stores it. */
  GeometryNodeModalKeymapItem item;
  /**
   * #ID::session_uid of the node group that defines this binding, which may be a nested group or
   * the node group itself.
   */
  uint32_t group_session_uid = 0;

  friend bool operator==(const ModalBinding &a, const ModalBinding &b);
};

/**
 * Modal node tools react to a set of named events that is defined by the Modal Event nodes in the
 * node group and all of its nested groups. The set is gathered here so that it can be used before
 * the node group is evaluated, to register a modal keymap for the tool's operator type and to
 * decide which events should be processed at all.
 */
struct ModalEventsInfo {
  /**
   * Deduplicated and sorted by name, so that comparing two sets only depends on their
   * content and not on the order the node groups happened to be visited in.
   */
  Vector<ModalEvent> events;
  Vector<ModalBinding> bindings;

  /** Add all events and bindings from the other set. */
  void merge(const ModalEventsInfo &other);
  /** Restore the invariants described above. Called once after everything has been added. */
  void deduplicate_and_sort();

  friend bool operator==(const ModalEventsInfo &a, const ModalEventsInfo &b) = default;
};

/**
 * Gather the events and default bindings of the node group and all of its nested groups. This does
 * not rely on any cached data, so it also works for node groups that are not part of the current
 * file yet.
 */
ModalEventsInfo gather_modal_events_recursive(const bNodeTree &ntree);
/**
 * Same as above, but assumes that the data is already cached on the referenced node groups.
 */
ModalEventsInfo gather_modal_events_with_cache(const bNodeTree &ntree);

}  // namespace blender::nodes
