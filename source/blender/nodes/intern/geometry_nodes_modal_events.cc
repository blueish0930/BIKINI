/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>

#include "BLI_map.hh"
#include "BLI_string.hh"

#include "DNA_node_types.h"

#include "BKE_node_runtime.hh"

#include "NOD_geometry_nodes_modal_events.hh"

namespace blender::nodes {

void ModalEventsInfo::merge(const ModalEventsInfo &other)
{
  this->events.extend(other.events);
  this->bindings.extend(other.bindings);
}

bool operator==(const ModalBinding &a, const ModalBinding &b)
{
  return a.group_session_uid == b.group_session_uid &&
         STREQ(a.item.event_name, b.item.event_name) && a.item.type == b.item.type &&
         a.item.val == b.item.val && a.item.keymodifier == b.item.keymodifier &&
         a.item.direction == b.item.direction && a.item.shift == b.item.shift &&
         a.item.ctrl == b.item.ctrl && a.item.alt == b.item.alt && a.item.oskey == b.item.oskey &&
         a.item.hyper == b.item.hyper && a.item.repeat == b.item.repeat;
}

static bool compare_binding(const ModalBinding &a, const ModalBinding &b)
{
  const int name_compare = BLI_strcasecmp_natural(a.item.event_name, b.item.event_name);
  if (name_compare != 0) {
    return name_compare < 0;
  }
  if (a.group_session_uid != b.group_session_uid) {
    return a.group_session_uid < b.group_session_uid;
  }
  if (a.item.type != b.item.type) {
    return a.item.type < b.item.type;
  }
  if (a.item.val != b.item.val) {
    return a.item.val < b.item.val;
  }
  if (a.item.keymodifier != b.item.keymodifier) {
    return a.item.keymodifier < b.item.keymodifier;
  }
  if (a.item.direction != b.item.direction) {
    return a.item.direction < b.item.direction;
  }
  if (a.item.shift != b.item.shift) {
    return a.item.shift < b.item.shift;
  }
  if (a.item.ctrl != b.item.ctrl) {
    return a.item.ctrl < b.item.ctrl;
  }
  if (a.item.alt != b.item.alt) {
    return a.item.alt < b.item.alt;
  }
  if (a.item.oskey != b.item.oskey) {
    return a.item.oskey < b.item.oskey;
  }
  if (a.item.hyper != b.item.hyper) {
    return a.item.hyper < b.item.hyper;
  }
  return a.item.repeat < b.item.repeat;
}

void ModalEventsInfo::deduplicate_and_sort()
{
  std::ranges::sort(this->events, [](const ModalEvent &a, const ModalEvent &b) {
    return BLI_strcasecmp_natural(a.name.c_str(), b.name.c_str()) < 0;
  });
  /* The first node that defines an event also provides its description. */
  const ModalEvent *events_end = std::unique(
      this->events.begin(), this->events.end(), [](const ModalEvent &a, const ModalEvent &b) {
        return a.name == b.name;
      });
  this->events.resize(events_end - this->events.begin());

  std::ranges::sort(this->bindings, compare_binding);
  const ModalBinding *bindings_end = std::unique(this->bindings.begin(), this->bindings.end());
  this->bindings.resize(bindings_end - this->bindings.begin());
}

static void gather_modal_events(
    const bNodeTree &ntree,
    ModalEventsInfo &events,
    FunctionRef<const ModalEventsInfo *(const bNodeTree &group)> get_group)
{
  ntree.ensure_topology_cache();
  for (const bNode *node : ntree.nodes_by_type("GeometryNodeModalEvent"_ustr)) {
    if (node->is_muted()) {
      continue;
    }
    const auto *storage = static_cast<const GeometryNodeModalEvent *>(node->storage);
    if (!storage->name || storage->name[0] == '\0') {
      continue;
    }
    events.events.append({storage->name, storage->description ? storage->description : ""});
  }
  if (const GeometryNodeAssetTraits *traits = ntree.geometry_node_asset_traits) {
    for (const int i : IndexRange(traits->modal_keymap_default_num)) {
      const GeometryNodeModalKeymapItem &item = traits->modal_keymap_default[i];
      if (item.event_name[0] == '\0') {
        continue;
      }
      events.bindings.append({item, ntree.id.session_uid});
    }
  }
  for (const bNode *node : ntree.group_nodes()) {
    if (!node->id) {
      continue;
    }
    const bNodeTree &group = *reinterpret_cast<const bNodeTree *>(node->id);
    if (const ModalEventsInfo *group_events = get_group(group)) {
      events.merge(*group_events);
    }
  }
}

ModalEventsInfo gather_modal_events_with_cache(const bNodeTree &ntree)
{
  ModalEventsInfo events;
  gather_modal_events(
      ntree, events, [](const bNodeTree &group) { return group.runtime->modal_events.get(); });
  events.deduplicate_and_sort();
  return events;
}

static void gather_modal_events_recursive_impl(
    const bNodeTree &ntree, Map<const bNodeTree *, ModalEventsInfo> &events_by_tree)
{
  if (events_by_tree.contains(&ntree)) {
    return;
  }
  /* Add an empty set before recursing, so that node group recursion terminates. */
  events_by_tree.add_new(&ntree, {});
  ModalEventsInfo new_events;
  gather_modal_events(ntree, new_events, [&](const bNodeTree &group) {
    gather_modal_events_recursive_impl(group, events_by_tree);
    return &events_by_tree.lookup(&group);
  });
  new_events.deduplicate_and_sort();
  events_by_tree.add_overwrite(&ntree, std::move(new_events));
}

ModalEventsInfo gather_modal_events_recursive(const bNodeTree &ntree)
{
  Map<const bNodeTree *, ModalEventsInfo> events_by_tree;
  gather_modal_events_recursive_impl(ntree, events_by_tree);
  return events_by_tree.lookup(&ntree);
}

}  // namespace blender::nodes
