/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <cctype>
#include <memory>

#include "BLI_listbase.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_time.hh"
#include "BLI_vector.hh"

#include "BKE_context.hh"
#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_library.hh"
#include "BKE_main.hh"
#include "BKE_screen.hh"

#include "BLT_translation.hh"

#include "DNA_ID.h"
#include "DNA_node_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "ED_screen.hh"

#include "MEM_guardedalloc.h"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "UI_abstract_view.hh"
#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_interface_icons.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"
#include "UI_tree_view.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "datablock_graph_intern.hh"

namespace blender::ed::datablock_graph {

/** Browser icon: Object uses #ICON_OBJECT_DATA; node trees use subtype icons. */
static BIFIconID browser_icon_from_id(const ID *id)
{
  if (id == nullptr) {
    return ICON_NONE;
  }
  if (GS(id->name) == ID_OB) {
    return ICON_OBJECT_DATA;
  }
  /* Node groups: generic node-group icon; subtype is only a text tag. */
  if (GS(id->name) == ID_NT) {
    return ICON_NODETREE;
  }
  return BIFIconID(ui::icon_from_id(id));
}

/**
 * Browser leaf label: keep the data-block name, annotate node-group subtype.
 * e.g. "MyGroup  · GN" (type category is still Node Trees).
 */
static std::string browser_label_for_id(const ID &id)
{
  std::string label = BKE_id_name(id);
  if (GS(id.name) == ID_NT) {
    const bNodeTree &ntree = *reinterpret_cast<const bNodeTree *>(&id);
    const char *tag = nullptr;
    switch (ntree.type) {
      case NTREE_GEOMETRY:
        tag = "GN";
        break;
      case NTREE_SHADER:
        tag = "Shader";
        break;
      case NTREE_COMPOSIT:
        tag = "Compositor";
        break;
      case NTREE_IMAGE:
        tag = "GPU Texture Editor";
        break;
      case NTREE_OBJECT:
        tag = "Object Editor";
        break;
      case NTREE_TEXTURE:
        tag = "Texture";
        break;
      default:
        break;
    }
    if (tag) {
      label += "  · ";
      label += tag;
    }
  }
  return label;
}

static bool name_matches_search(const char *name, const char *search)
{
  if (search == nullptr || search[0] == '\0') {
    return true;
  }
  const size_t nlen = strlen(name);
  const size_t slen = strlen(search);
  if (slen > nlen) {
    return false;
  }
  for (size_t i = 0; i + slen <= nlen; i++) {
    if (BLI_strncasecmp(name + i, search, slen) == 0) {
      return true;
    }
  }
  return false;
}

/* -------------------------------------------------------------------- */
/** \name Drag controller — ID ghost following the cursor (Outliner-style)
 * \{ */

class DataBlockBrowserTreeView;

class DataBlockDragController : public ui::AbstractViewItemDragController {
 private:
  ID *id_ = nullptr;

 public:
  DataBlockDragController(ui::AbstractTreeView &view, ID *id)
      : ui::AbstractViewItemDragController(view), id_(id)
  {
  }

  std::optional<eWM_DragDataType> get_drag_type() const override
  {
    return WM_DRAG_ID;
  }

  void *create_drag_data() const override
  {
    /* #WM_event_start_drag copies this into #wmDragID list for the floating ghost. */
    return id_;
  }

  void on_drag_start(bContext &C, ui::AbstractViewItem & /*item*/) override
  {
    /* Set type icon so the drag ghost matches other ID drags. */
    wmWindowManager *wm = CTX_wm_manager(&C);
    if (wm == nullptr || id_ == nullptr || wm->runtime->drags.is_empty()) {
      return;
    }
    wmDrag *drag = static_cast<wmDrag *>(wm->runtime->drags.last());
    if (drag && drag->type == WM_DRAG_ID) {
      drag->icon = browser_icon_from_id(id_);
    }
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Data-block leaf item (double-click creates/highlights graph; drag has ghost)
 * \{ */

class DataBlockTreeViewItem : public ui::BasicTreeViewItem {
 private:
  ID *id_ = nullptr;
  SpaceDataBlockGraph &sdbg_;

  /** Double-click detection (tree view only fires activate on single click). */
  static uint last_click_uid_;
  static double last_click_time_;

 public:
  DataBlockTreeViewItem(SpaceDataBlockGraph &sdbg, ID *id, StringRef label, BIFIconID icon)
      : ui::BasicTreeViewItem(label, icon), id_(id), sdbg_(sdbg)
  {
    /* Do NOT use set_is_active_fn for seeds — active items force parents uncollapsed and
     * prevent closing category panels. Mark seeds with a label suffix instead. */
    if (datablock_graph_has_seed(sdbg_, id_)) {
      this->label_ = std::string(label) + "  ●";
    }
    /* Single-click does nothing for create; double-click handled in on_activate. */
    this->set_on_activate_fn([this](bContext &C, ui::BasicTreeViewItem & /*item*/) {
      this->handle_activate(C);
    });
  }

  std::unique_ptr<ui::AbstractViewItemDragController> create_drag_controller() const override
  {
    if (id_ == nullptr) {
      return nullptr;
    }
    return std::make_unique<DataBlockDragController>(
        static_cast<ui::AbstractTreeView &>(this->get_view()), id_);
  }

 private:
  void handle_activate(bContext &C)
  {
    if (id_ == nullptr) {
      return;
    }
    const double now = BLI_time_now_seconds();
    constexpr double double_click_sec = 0.4;
    const bool is_double = (id_->session_uid == last_click_uid_) &&
                           ((now - last_click_time_) < double_click_sec);
    last_click_uid_ = id_->session_uid;
    last_click_time_ = now;

    if (!is_double) {
      /* First click: only mark active in the browser list, do not create a graph. */
      return;
    }

    if (id_ == nullptr) {
      return;
    }
    /* Go through operator so OPTYPE_UNDO records the seed change for Ctrl+Z. */
    wmOperatorType *ot = WM_operatortype_find("DATABLOCK_GRAPH_OT_add_seed", false);
    if (ot == nullptr) {
      return;
    }
    PointerRNA props = WM_operator_properties_create_ptr(ot);
    RNA_int_set(&props, "session_uid", int(id_->session_uid));
    WM_operator_name_call(
        &C, "DATABLOCK_GRAPH_OT_add_seed", wm::OpCallContext::ExecDefault, &props, nullptr);
    WM_operator_properties_free(&props);
  }
};

uint DataBlockTreeViewItem::last_click_uid_ = 0;
double DataBlockTreeViewItem::last_click_time_ = 0.0;

/** \} */

/**
 * Category row (library / ID-type) that remembers open/closed across file load
 * via #SpaceDataBlockGraph.browser_collapsed.
 */
class BrowserCategoryItem : public ui::BasicTreeViewItem {
 private:
  SpaceDataBlockGraph &sdbg_;
  std::string path_;

 public:
  BrowserCategoryItem(SpaceDataBlockGraph &sdbg,
                      StringRef label,
                      BIFIconID icon,
                      std::string path)
      : ui::BasicTreeViewItem(label, icon), sdbg_(sdbg), path_(std::move(path))
  {
  }

  std::optional<bool> should_be_collapsed() const override
  {
    return datablock_graph_browser_path_collapsed(sdbg_, path_);
  }

  void on_collapse_change(bContext & /*C*/, const bool is_collapsed) override
  {
    datablock_graph_browser_path_set_collapsed(sdbg_, path_, is_collapsed);
  }
};

class DataBlockBrowserTreeView : public ui::AbstractTreeView {
 private:
  Main &bmain_;
  SpaceDataBlockGraph &sdbg_;

 public:
  DataBlockBrowserTreeView(Main &bmain, SpaceDataBlockGraph &sdbg) : bmain_(bmain), sdbg_(sdbg) {}

  void build_tree() override
  {
    build_library_contents(nullptr, IFACE_("Current File"));

    for (Library &lib : bmain_.libraries) {
      const char *name = (lib.filepath[0] != '\0') ? lib.filepath : BKE_id_name(lib.id);
      build_library_contents(&lib, name);
    }
  }

 private:
  void build_library_contents(Library *lib, StringRefNull lib_label)
  {
    const char *search = sdbg_.search_string;
    const std::string lib_path = std::string(lib_label);

    BrowserCategoryItem &lib_item = this->add_tree_item<BrowserCategoryItem>(
        sdbg_, lib_label, lib ? ICON_LIBRARY_DATA_DIRECT : ICON_FILE_BLEND, lib_path);

    MainListsArray lists = BKE_main_lists_get(bmain_);
    for (ListBaseT<ID> *lb : lists) {
      if (lb == nullptr || lb->first() == nullptr) {
        continue;
      }

      ID *first = lb->first();
      const short idcode = GS(first->name);
      if (ID_TYPE_IS_DEPRECATED(idcode) || idcode == ID_LI) {
        continue;
      }
      if (sdbg_.filter_id_type != 0 && sdbg_.filter_id_type != idcode) {
        continue;
      }

      Vector<ID *> ids_in_lib;
      for (ID &id : *lb) {
        if (id.lib != lib) {
          continue;
        }
        if (!name_matches_search(BKE_id_name(id), search)) {
          continue;
        }
        ids_in_lib.append(&id);
      }
      if (ids_in_lib.is_empty()) {
        continue;
      }

      std::sort(ids_in_lib.begin(), ids_in_lib.end(), [](const ID *a, const ID *b) {
        return BLI_strcasecmp_natural(a->name + 2, b->name + 2) < 0;
      });

      const char *type_name = BKE_idtype_idcode_to_name_plural(idcode);
      const char *type_label = type_name ? type_name : "?";
      const std::string type_path = lib_path + "/" + type_label;
      BrowserCategoryItem &type_item = lib_item.add_tree_item<BrowserCategoryItem>(
          sdbg_, type_label, BIFIconID(ui::icon_from_idcode(idcode)), type_path);

      for (ID *id : ids_in_lib) {
        type_item.add_tree_item<DataBlockTreeViewItem>(
            sdbg_, id, browser_label_for_id(*id), browser_icon_from_id(id));
      }
    }
  }
};

static void datablock_graph_browser_panel_draw(const bContext *C, Panel *panel)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  Main *bmain = CTX_data_main(C);
  if (sdbg == nullptr || bmain == nullptr) {
    return;
  }

  ui::Layout &layout = *panel->layout;
  ui::Block *block = layout.block();
  ui::block_layout_set_current(block, &layout);

  PointerRNA space_ptr = RNA_pointer_create_discrete(
      &CTX_wm_screen(C)->id, RNA_SpaceDataBlockGraph, sdbg);

  ui::Layout &search_row = layout.row(true);
  search_row.prop(&space_ptr, "search_string", UI_ITEM_NONE, "", ICON_VIEWZOOM);
  search_row.prop(&space_ptr, "filter_id_type", UI_ITEM_NONE, "", ICON_NONE);

  ui::AbstractTreeView *tree_view = block_add_view(
      *block,
      "Data-Block Browser",
      std::make_unique<DataBlockBrowserTreeView>(*bmain, *sdbg));
  tree_view->set_context_menu_title("Data-Block");
  ui::TreeViewBuilder::build_tree_view(*C, *tree_view, layout, false);
}

void datablock_graph_browser_panels_register(ARegionType &region_type)
{
  PanelType *panel_type = MEM_new_zeroed<PanelType>(__func__);
  STRNCPY_UTF8(panel_type->idname, "DATABLOCK_GRAPH_PT_browser");
  STRNCPY_UTF8(panel_type->label, N_("Data-Blocks"));
  STRNCPY_UTF8(panel_type->translation_context, BLT_I18NCONTEXT_DEFAULT_BPYRNA);
  panel_type->flag = PANEL_TYPE_NO_HEADER;
  panel_type->draw = datablock_graph_browser_panel_draw;
  BLI_addtail(&region_type.paneltypes, panel_type);
}

void datablock_graph_draw_browser(const bContext *C, ARegion *region)
{
  ED_region_panels(C, region);
}

}  // namespace blender::ed::datablock_graph
