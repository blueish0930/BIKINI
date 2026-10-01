/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 *
 * Tree of recursive geometry-node-group calls. A single depth cannot tell two calls apart
 * (Fibonacci calls the same group twice). Each row is one invocation, labeled by the group node
 * that made the call. Selecting a row shows that invocation's socket values.
 */

#include "MEM_guardedalloc.h"

#include "BLI_set.hh"
#include "BLI_span.hh"
#include "BLI_string_utf8.hh"
#include "BLI_vector.hh"

#include "BLT_translation.hh"

#include "BKE_compute_context_cache.hh"
#include "BKE_context.hh"
#include "BKE_node_runtime.hh"
#include "BKE_screen.hh"

#include "BLI_listbase.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_node_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"

#include "ED_node.hh"
#include "ED_screen.hh"

#include "NOD_eval_log.hh"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"
#include "UI_tree_view.hh"

#include "node_intern.hh"

namespace blender::ed::space_node {

static constexpr int recursive_call_row_limit = 256;

static bool is_self_group_call(const bNodeTree &tree, const bNode *node)
{
  if (node == nullptr || node->id == nullptr || GS(node->id->name) != ID_NT) {
    return false;
  }
  const bNodeTree &called = *reinterpret_cast<const bNodeTree *>(node->id);
  if (&called == &tree || called.id.session_uid == tree.id.session_uid) {
    return true;
  }
  if (const bNodeTree *orig = DEG_get_original(&called)) {
    return orig == &tree || orig->id.session_uid == tree.id.session_uid;
  }
  return false;
}

static std::string node_call_label(const bNode &node)
{
  if (node.label[0] != '\0') {
    return node.label;
  }
  return node.name;
}

class RecursiveCallItem : public ui::AbstractTreeViewItem {
  SpaceNode &snode_;
  bNodeTree &tree_;
  ComputeContextHash hash_;
  std::string name_;
  int depth_;
  bool is_root_;

 public:
  RecursiveCallItem(SpaceNode &snode,
                    bNodeTree &tree,
                    const ComputeContextHash hash,
                    std::string name,
                    const int depth,
                    const bool is_root)
      : snode_(snode),
        tree_(tree),
        hash_(hash),
        name_(std::move(name)),
        depth_(depth),
        is_root_(is_root)
  {
    label_ = name_ + ":" + std::to_string(hash_.v1) + ":" + std::to_string(hash_.v2);
  }

  void build_row(ui::Layout &row) override
  {
    row.label(name_, ICON_NODETREE);
    ui::Layout &suffix = row.row(true);
    suffix.alignment_set(ui::LayoutAlign::Right);
    suffix.active_set(false);
    suffix.label(std::string(IFACE_("depth ")) + std::to_string(depth_), ICON_NONE);
  }

  void on_activate(bContext &C) override
  {
    if (snode_.runtime == nullptr) {
      return;
    }
    snode_.runtime->recursive_inspection_tree_uid = tree_.id.session_uid;
    snode_.runtime->recursive_inspection_hash = hash_;
    if (ScrArea *area = CTX_wm_area(&C)) {
      ED_area_tag_redraw(area);
    }
  }

  std::optional<bool> should_be_active() const override
  {
    const bool selected = snode_.runtime->recursive_inspection_tree_uid == tree_.id.session_uid &&
                          snode_.runtime->recursive_inspection_hash.has_value() &&
                          *snode_.runtime->recursive_inspection_hash == hash_;
    if (!snode_.runtime->recursive_inspection_hash.has_value() ||
        snode_.runtime->recursive_inspection_tree_uid != tree_.id.session_uid)
    {
      return is_root_;
    }
    return selected;
  }
};

struct PendingCall {
  ComputeContextHash hash;
  int node_id;
  std::string name;
};

static void collect_direct_calls(nodes::eval_log::NodesEvalLog &log,
                                 const bNodeTree &tree,
                                 const ComputeContextHash hash,
                                 Set<ComputeContextHash> &visiting,
                                 int &budget,
                                 bool &truncated,
                                 Vector<PendingCall> &r_calls)
{
  if (!visiting.add(hash)) {
    return;
  }
  nodes::eval_log::NodeTreeLog &tree_log = log.get_tree_log(hash);
  tree_log.foreach_child_context_hash([&](const ComputeContextHash child_hash) {
    if (budget <= 0) {
      truncated = true;
      return;
    }
    nodes::eval_log::NodeTreeLog &child_log = log.get_tree_log(child_hash);
    const bNode *caller = nullptr;
    if (const std::optional<int32_t> caller_id = child_log.caller_node_id()) {
      caller = tree.node_by_id(*caller_id);
    }
    if (is_self_group_call(tree, caller)) {
      budget--;
      PendingCall call;
      call.hash = child_hash;
      call.node_id = caller->identifier;
      call.name = node_call_label(*caller);
      r_calls.append(std::move(call));
      return;
    }
    const std::optional<uint32_t> child_tree = child_log.tree_session_uid();
    if (child_tree.has_value() && *child_tree == tree.id.session_uid) {
      collect_direct_calls(log, tree, child_hash, visiting, budget, truncated, r_calls);
    }
  });
}

static void disambiguate_sibling_names(MutableSpan<PendingCall> calls)
{
  for (PendingCall &call : calls) {
    int total = 0;
    int index = 0;
    for (const PendingCall &other : calls) {
      if (other.node_id != call.node_id) {
        continue;
      }
      total++;
      if (&other == &call) {
        index = total;
      }
    }
    if (total > 1) {
      call.name += " #" + std::to_string(index);
    }
  }
}

static void add_call_items(ui::AbstractTreeViewItem &parent,
                           SpaceNode &snode,
                           bNodeTree &tree,
                           nodes::eval_log::NodesEvalLog &log,
                           const ComputeContextHash hash,
                           const int depth,
                           Set<ComputeContextHash> &visiting,
                           int &budget,
                           bool &truncated)
{
  Vector<PendingCall> calls;
  collect_direct_calls(log, tree, hash, visiting, budget, truncated, calls);
  disambiguate_sibling_names(calls);
  /* Create every sibling before descending, so a wide left branch cannot hide the right one. */
  Vector<RecursiveCallItem *> items;
  for (PendingCall &call : calls) {
    RecursiveCallItem &item = parent.add_tree_item<RecursiveCallItem>(
        snode, tree, call.hash, std::move(call.name), depth + 1, false);
    if (depth + 1 < 2) {
      item.uncollapse_by_default();
    }
    items.append(&item);
  }
  for (const int i : calls.index_range()) {
    add_call_items(
        *items[i], snode, tree, log, calls[i].hash, depth + 1, visiting, budget, truncated);
  }
}

class RecursiveCallTreeView : public ui::AbstractTreeView {
  SpaceNode &snode_;
  bNodeTree &tree_;
  const ComputeContext *root_context_;
  nodes::eval_log::NodesEvalLog *log_;
  bool &truncated_;

 public:
  RecursiveCallTreeView(SpaceNode &snode,
                        bNodeTree &tree,
                        const ComputeContext *root_context,
                        nodes::eval_log::NodesEvalLog *log,
                        bool &truncated)
      : snode_(snode),
        tree_(tree),
        root_context_(root_context),
        log_(log),
        truncated_(truncated)
  {
  }

  void build_tree() override
  {
    if (root_context_ == nullptr) {
      return;
    }
    RecursiveCallItem &root = this->add_tree_item<RecursiveCallItem>(
        snode_, tree_, root_context_->hash(), IFACE_("Root"), 0, true);
    root.uncollapse_by_default();
    if (log_ == nullptr) {
      return;
    }
    Set<ComputeContextHash> visiting;
    int budget = recursive_call_row_limit;
    add_call_items(root, snode_, tree_, *log_, root_context_->hash(), 0, visiting, budget, truncated_);
  }
};

static bool recursive_call_panel_poll(const bContext *C, PanelType * /*pt*/)
{
  const SpaceNode *snode = CTX_wm_space_node(C);
  if (snode == nullptr || snode->edittree == nullptr) {
    return false;
  }
  const bNodeTree &tree = *snode->edittree;
  if (tree.type != NTREE_GEOMETRY || (tree.id.flag & ID_FLAG_EMBEDDED_DATA)) {
    return false;
  }
  return (tree.flag & NTREE_GEOMETRY_RECURSIVE) != 0;
}

static void recursive_call_panel_draw(const bContext *C, Panel *panel)
{
  SpaceNode &snode = *CTX_wm_space_node(C);
  bNodeTree &tree = *snode.edittree;
  ui::Layout &layout = *panel->layout;

  layout.label(IFACE_("Each row is one call. The name is the group node that made it."),
               ICON_NONE);

  tree.ensure_topology_cache();

  bke::ComputeContextCache compute_context_cache;
  const ComputeContext *root = compute_context_for_edittree_base(snode, compute_context_cache);
  nodes::eval_log::NodesEvalLog *log = nodes::eval_log::NodesEvalLog::from_space_node(snode);

  bool truncated = false;
  ui::Block *block = layout.block();
  ui::AbstractTreeView *tree_view = ui::block_add_view(
      *block,
      "Geo Recursive Calls",
      std::make_unique<RecursiveCallTreeView>(snode, tree, root, log, truncated));
  tree_view->set_default_rows(8);
  ui::TreeViewBuilder::build_tree_view(*C, *tree_view, layout);

  if (root == nullptr || log == nullptr) {
    layout.label(IFACE_("Evaluate the node group to list calls."), ICON_INFO);
  }
  else if (truncated) {
    layout.label(IFACE_("Call tree truncated."), ICON_INFO);
  }
}

void node_recursive_call_panel_register(ARegionType *art)
{
  PanelType *pt = MEM_new_zeroed<PanelType>(__func__);
  STRNCPY_UTF8(pt->idname, "NODE_PT_recursive_calls");
  STRNCPY_UTF8(pt->label, N_("Recursive Calls"));
  STRNCPY_UTF8(pt->category, "Group");
  STRNCPY_UTF8(pt->translation_context, BLT_I18NCONTEXT_DEFAULT_BPYRNA);
  pt->draw = recursive_call_panel_draw;
  pt->poll = recursive_call_panel_poll;
  BLI_addtail(&art->paneltypes, pt);
}

}  // namespace blender::ed::space_node
