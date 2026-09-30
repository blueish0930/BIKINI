/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_listbase.hh"
#include "BLI_string.hh"

#include "BKE_context.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
#include "BKE_lib_remap.hh"
#include "BKE_screen.hh"

#include "BLO_read_write.hh"

#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "ED_screen.hh"
#include "ED_space_api.hh"

#include "MEM_guardedalloc.h"

#include "GPU_framebuffer.hh"
#include "GPU_state.hh"

#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_resources.hh"
#include "UI_view2d.hh"

#include "WM_api.hh"
#include "WM_toolsystem.hh"
#include "WM_types.hh"

#include "datablock_graph_intern.hh"

namespace blender {

using namespace ed::datablock_graph;

/**
 * Ensure TOOLS (T-toolbar) + CHANNELS (libraries browser) exist.
 * Migrates older layouts that put the browser on RGN_TYPE_TOOLS.
 *
 * IMPORTANT: Any region added/changed here must get #ARegionRuntime::type set, because
 * #ED_area_init runs #ED_area_and_region_types_init *before* #SpaceType::init. Missing type
 * pointers cause an immediate crash when installing handlers.
 */
static void datablock_graph_ensure_regions(ScrArea *area, ListBaseT<ARegion> &regionbase)
{
  SpaceType *st = (area && area->type) ? area->type : BKE_spacetype_from_id(SPACE_DATABLOCK_GRAPH);
  if (st == nullptr) {
    return;
  }

  auto assign_type = [st](ARegion *region) {
    if (region == nullptr || region->runtime == nullptr) {
      return;
    }
    region->runtime->type = BKE_regiontype_from_id(st, region->regiontype);
  };

  ARegion *tools = nullptr;
  ARegion *channels = nullptr;
  ARegion *header = nullptr;

  for (ARegion &region : regionbase) {
    switch (region.regiontype) {
      case RGN_TYPE_TOOLS:
        tools = &region;
        break;
      case RGN_TYPE_CHANNELS:
        channels = &region;
        break;
      case RGN_TYPE_HEADER:
        header = &region;
        break;
      default:
        break;
    }
  }

  bool changed = false;

  /* Old builds used TOOLS as a wide browser (~220px). Convert that to CHANNELS. */
  if (tools && !channels && (tools->sizex >= 160 || tools->winx >= 160)) {
    tools->regiontype = RGN_TYPE_CHANNELS;
    tools->alignment = RGN_ALIGN_LEFT;
    tools->flag &= ~RGN_FLAG_HIDDEN;
    tools->sizex = short(220 + V2D_SCROLL_WIDTH);
    tools->v2d.scroll = V2D_SCROLL_RIGHT | V2D_SCROLL_VERTICAL_HIDE;
    assign_type(tools);
    channels = tools;
    tools = nullptr;
    changed = true;
  }

  if (!tools) {
    tools = BKE_area_region_new();
    tools->regiontype = RGN_TYPE_TOOLS;
    tools->alignment = RGN_ALIGN_LEFT;
    tools->flag = RGN_FLAG_HIDDEN;
    tools->sizex = short(UI_TOOLBAR_WIDTH);
    tools->sizey = 50;
    assign_type(tools);
    if (header) {
      BLI_insertlinkafter(&regionbase, header, tools);
    }
    else {
      BLI_addhead(&regionbase, tools);
    }
    changed = true;
  }
  else {
    if (tools->sizex <= 0) {
      tools->sizex = short(UI_TOOLBAR_WIDTH);
    }
    tools->alignment = RGN_ALIGN_LEFT;
    assign_type(tools);
  }

  if (!channels) {
    channels = BKE_area_region_new();
    channels->regiontype = RGN_TYPE_CHANNELS;
    channels->alignment = RGN_ALIGN_LEFT;
    channels->sizex = short(220 + V2D_SCROLL_WIDTH);
    channels->v2d.scroll = V2D_SCROLL_RIGHT | V2D_SCROLL_VERTICAL_HIDE;
    channels->flag &= ~RGN_FLAG_HIDDEN;
    assign_type(channels);
    if (tools) {
      BLI_insertlinkafter(&regionbase, tools, channels);
    }
    else if (header) {
      BLI_insertlinkafter(&regionbase, header, channels);
    }
    else {
      BLI_addhead(&regionbase, channels);
    }
    changed = true;
  }
  else {
    channels->alignment = RGN_ALIGN_LEFT;
    channels->flag &= ~RGN_FLAG_HIDDEN;
    if (channels->sizex < 100) {
      channels->sizex = short(220 + V2D_SCROLL_WIDTH);
    }
    assign_type(channels);
  }

  /* Re-bind every region type (defensive against stale runtime after conversion). */
  for (ARegion &region : regionbase) {
    if (region.runtime && region.runtime->type == nullptr) {
      assign_type(&region);
    }
  }

  if (changed && area) {
    area->flag |= AREA_FLAG_REGION_SIZE_UPDATE;
  }
}

static SpaceLink *datablock_graph_create(const ScrArea * /*area*/, const Scene * /*scene*/)
{
  SpaceDataBlockGraph *sdbg = MEM_new<SpaceDataBlockGraph>("data-block graph space");
  sdbg->spacetype = SPACE_DATABLOCK_GRAPH;
  sdbg->flag = DATABLOCK_GRAPH_FLAG_HIDE_LOOPBACK | DATABLOCK_GRAPH_FLAG_SNAP;
  sdbg->max_depth = 4;
  sdbg->runtime = MEM_new<SpaceDataBlockGraph_Runtime>(__func__);

  {
    ARegion *region = BKE_area_region_new();
    BLI_addtail(&sdbg->regionbase, region);
    region->regiontype = RGN_TYPE_HEADER;
    region->alignment = (U.uiflag & USER_HEADER_BOTTOM) ? RGN_ALIGN_BOTTOM : RGN_ALIGN_TOP;
  }

  /* T-key toolsystem toolbar (hidden by default, like Node Editor). */
  {
    ARegion *region = BKE_area_region_new();
    BLI_addtail(&sdbg->regionbase, region);
    region->regiontype = RGN_TYPE_TOOLS;
    region->alignment = RGN_ALIGN_LEFT;
    region->flag = RGN_FLAG_HIDDEN;
    region->sizex = short(UI_TOOLBAR_WIDTH);
  }

  /* Libraries browser (always available). */
  {
    ARegion *region = BKE_area_region_new();
    BLI_addtail(&sdbg->regionbase, region);
    region->regiontype = RGN_TYPE_CHANNELS;
    region->alignment = RGN_ALIGN_LEFT;
    region->sizex = short(220 + V2D_SCROLL_WIDTH);
    region->v2d.scroll = V2D_SCROLL_RIGHT | V2D_SCROLL_VERTICAL_HIDE;
  }

  {
    ARegion *region = BKE_area_region_new();
    BLI_addtail(&sdbg->regionbase, region);
    region->regiontype = RGN_TYPE_WINDOW;
  }

  return reinterpret_cast<SpaceLink *>(sdbg);
}

static void datablock_graph_free(SpaceLink *sl)
{
  SpaceDataBlockGraph *sdbg = reinterpret_cast<SpaceDataBlockGraph *>(sl);
  datablock_graph_clear_seeds(*sdbg);
  datablock_graph_free_persistent_state(*sdbg);
  datablock_graph_runtime_free(*sdbg);
}

static void datablock_graph_init(wmWindowManager * /*wm*/, ScrArea *area)
{
  if (area == nullptr) {
    return;
  }
  /* Active space stores regions on the area. */
  datablock_graph_ensure_regions(area, area->regionbase);

  /* Also fix inactive space copies (switch-back). */
  for (SpaceLink &sl : area->spacedata) {
    if (sl.spacetype == SPACE_DATABLOCK_GRAPH && &sl != area->spacedata.first()) {
      datablock_graph_ensure_regions(area, sl.regionbase);
    }
  }
}

static SpaceLink *datablock_graph_duplicate(SpaceLink *sl)
{
  const SpaceDataBlockGraph *sdbg_old = reinterpret_cast<SpaceDataBlockGraph *>(sl);
  SpaceDataBlockGraph *sdbg_new = MEM_dupalloc(sdbg_old);
  sdbg_new->runtime = MEM_new<SpaceDataBlockGraph_Runtime>(__func__);
  sdbg_new->runtime->graph_dirty = true;
  sdbg_new->runtime->layout_dirty = true;

  sdbg_new->seeds = {nullptr, nullptr};
  for (const SpaceDataBlockGraphSeed &seed : sdbg_old->seeds) {
    SpaceDataBlockGraphSeed *seed_new = MEM_new<SpaceDataBlockGraphSeed>(__func__);
    seed_new->id = seed.id;
    BLI_addtail(&sdbg_new->seeds, seed_new);
  }

  sdbg_new->node_positions = {nullptr, nullptr};
  for (const SpaceDataBlockGraphNodePos &pos : sdbg_old->node_positions) {
    SpaceDataBlockGraphNodePos *pos_new = MEM_new<SpaceDataBlockGraphNodePos>(__func__);
    *pos_new = pos;
    pos_new->next = pos_new->prev = nullptr;
    BLI_addtail(&sdbg_new->node_positions, pos_new);
  }

  sdbg_new->browser_collapsed = {nullptr, nullptr};
  for (const SpaceDataBlockGraphTreePath &path : sdbg_old->browser_collapsed) {
    SpaceDataBlockGraphTreePath *path_new = MEM_new<SpaceDataBlockGraphTreePath>(__func__);
    *path_new = path;
    path_new->next = path_new->prev = nullptr;
    BLI_addtail(&sdbg_new->browser_collapsed, path_new);
  }

  return reinterpret_cast<SpaceLink *>(sdbg_new);
}

static void datablock_graph_id_remap(ScrArea * /*area*/,
                                     SpaceLink *slink,
                                     const bke::id::IDRemapper &mappings)
{
  SpaceDataBlockGraph *sdbg = reinterpret_cast<SpaceDataBlockGraph *>(slink);
  for (SpaceDataBlockGraphSeed &seed : sdbg->seeds) {
    if (seed.id) {
      mappings.apply(reinterpret_cast<ID **>(&seed.id), ID_REMAP_APPLY_DEFAULT);
    }
  }
  for (SpaceDataBlockGraphNodePos &pos : sdbg->node_positions) {
    if (pos.id) {
      mappings.apply(reinterpret_cast<ID **>(&pos.id), ID_REMAP_APPLY_DEFAULT);
    }
    if (pos.seed_id) {
      mappings.apply(reinterpret_cast<ID **>(&pos.seed_id), ID_REMAP_APPLY_DEFAULT);
    }
  }
  datablock_graph_tag_rebuild(*sdbg);
}

static void datablock_graph_foreach_id(SpaceLink *space_link, LibraryForeachIDData *data)
{
  SpaceDataBlockGraph *sdbg = reinterpret_cast<SpaceDataBlockGraph *>(space_link);
  for (SpaceDataBlockGraphSeed &seed : sdbg->seeds) {
    BKE_LIB_FOREACHID_PROCESS_ID(data, seed.id, IDWALK_CB_DIRECT_WEAK_LINK);
  }
  for (SpaceDataBlockGraphNodePos &pos : sdbg->node_positions) {
    BKE_LIB_FOREACHID_PROCESS_ID(data, pos.id, IDWALK_CB_DIRECT_WEAK_LINK);
    BKE_LIB_FOREACHID_PROCESS_ID(data, pos.seed_id, IDWALK_CB_DIRECT_WEAK_LINK);
  }
}

static void datablock_graph_blend_read_data(BlendDataReader *reader, SpaceLink *sl)
{
  SpaceDataBlockGraph *sdbg = reinterpret_cast<SpaceDataBlockGraph *>(sl);
  sdbg->runtime = MEM_new<SpaceDataBlockGraph_Runtime>(__func__);
  BLO_read_struct_list(reader, SpaceDataBlockGraphSeed, &sdbg->seeds);
  BLO_read_struct_list(reader, SpaceDataBlockGraphNodePos, &sdbg->node_positions);
  BLO_read_struct_list(reader, SpaceDataBlockGraphTreePath, &sdbg->browser_collapsed);
  sdbg->flag &= ~uint32_t(DATABLOCK_GRAPH_FLAG_HIDE_EMBEDDED);
  /* File-load: space not yet active; assign types from spacetype registry. */
  datablock_graph_ensure_regions(nullptr, sdbg->regionbase);
}

static void datablock_graph_blend_read_after_liblink(BlendLibReader * /*reader*/,
                                                    ID * /*parent_id*/,
                                                    SpaceLink *sl)
{
  SpaceDataBlockGraph *sdbg = reinterpret_cast<SpaceDataBlockGraph *>(sl);
  datablock_graph_tag_rebuild(*sdbg);
}

static void datablock_graph_blend_write(BlendWriter *writer, SpaceLink *sl)
{
  SpaceDataBlockGraph *sdbg = reinterpret_cast<SpaceDataBlockGraph *>(sl);
  /* Flush runtime node positions into DNA before serializing. */
  if (sdbg->runtime && !sdbg->runtime->nodes.is_empty()) {
    datablock_graph_store_positions(*sdbg);
  }
  writer->write_struct_cast<SpaceDataBlockGraph>(sl);
  for (SpaceDataBlockGraphSeed &seed : sdbg->seeds) {
    writer->write_struct(&seed);
  }
  for (SpaceDataBlockGraphNodePos &pos : sdbg->node_positions) {
    writer->write_struct(&pos);
  }
  for (SpaceDataBlockGraphTreePath &path : sdbg->browser_collapsed) {
    writer->write_struct(&path);
  }
}

/* -------------------------------------------------------------------- */
/** \name Main Region
 * \{ */

static void datablock_graph_main_region_init(wmWindowManager *wm, ARegion *region)
{
  view2d_region_reinit(&region->v2d, ui::V2D_COMMONVIEW_CUSTOM, region->winx, region->winy);
  region->v2d.keepzoom |= V2D_KEEPASPECT | V2D_LIMITZOOM | V2D_KEEPZOOM;
  region->v2d.keeptot = eView2D_KeepTot{};
  region->v2d.minzoom = 0.01f;
  region->v2d.maxzoom = 10.0f;
  region->v2d.scroll = V2D_SCROLL_RIGHT | V2D_SCROLL_BOTTOM;
  region->v2d.align = eView2D_Align{};

  /* Generic editor keymap (select / transform / box). Same pattern as Node Editor. */
  datablock_graph_keymap(wm->runtime->defaultconf);
  if (wm->runtime->userconf) {
    datablock_graph_keymap(wm->runtime->userconf);
  }
  {
    wmKeyMap *keymap = WM_keymap_ensure(wm->runtime->defaultconf,
                                        "Data-Block Graph Generic",
                                        SPACE_DATABLOCK_GRAPH,
                                        RGN_TYPE_WINDOW);
    WM_event_add_keymap_handler_v2d_mask(&region->runtime->handlers, keymap);
  }
  {
    wmKeyMap *keymap = WM_keymap_ensure(
        wm->runtime->defaultconf, "View2D", SPACE_EMPTY, RGN_TYPE_WINDOW);
    WM_event_add_keymap_handler(&region->runtime->handlers, keymap);
  }
  {
    ListBaseT<wmDropBox> *lb = WM_dropboxmap_find(
        "Data-Block Graph", SPACE_DATABLOCK_GRAPH, RGN_TYPE_WINDOW);
    WM_event_add_dropbox_handler(&region->runtime->handlers, lb);
  }
}

static void datablock_graph_main_region_draw(const bContext *C, ARegion *region)
{
  /* Match Geometry Nodes editor background (SPACE_NODE theme). */
  float back[4];
  ui::theme::get_color_type_4fv(TH_BACK, SPACE_NODE, back);
  GPU_clear_color(back[0], back[1], back[2], 1.0f);
  datablock_graph_draw_main(C, region);
}

static void datablock_graph_on_undo_rebuild(ScrArea *area)
{
  if (area == nullptr) {
    return;
  }
  SpaceDataBlockGraph *sdbg = area->spacedata.first_as<SpaceDataBlockGraph>();
  if (sdbg == nullptr || sdbg->spacetype != SPACE_DATABLOCK_GRAPH) {
    return;
  }
  /* DNA (seeds / node_positions) was restored by memfile undo; drop stale runtime and rebuild. */
  datablock_graph_runtime_free(*sdbg);
  datablock_graph_ensure_runtime(*sdbg);
  datablock_graph_tag_rebuild(*sdbg);
}

static void datablock_graph_main_region_listener(const wmRegionListenerParams *params)
{
  ARegion *region = params->region;
  const wmNotifier *wmn = params->notifier;

  switch (wmn->category) {
    case NC_SPACE:
      if (wmn->data == ND_SPACE_DATABLOCK_GRAPH) {
        ED_region_tag_redraw(region);
      }
      break;
    case NC_WM:
      if (wmn->data == ND_UNDO) {
        datablock_graph_on_undo_rebuild(params->area);
        ED_region_tag_redraw(region);
      }
      break;
    case NC_ID:
    case NC_WINDOW:
      if (ScrArea *area = params->area) {
        SpaceDataBlockGraph *sdbg = area->spacedata.first_as<SpaceDataBlockGraph>();
        if (sdbg && sdbg->spacetype == SPACE_DATABLOCK_GRAPH) {
          datablock_graph_tag_rebuild(*sdbg);
        }
      }
      ED_region_tag_redraw(region);
      break;
    case NC_WORKSPACE:
      ED_region_tag_redraw(region);
      break;
    default:
      break;
  }
}

static void datablock_graph_area_listener(const wmSpaceTypeListenerParams *params)
{
  const wmNotifier *wmn = params->notifier;
  if (wmn->category == NC_WM && wmn->data == ND_UNDO) {
    datablock_graph_on_undo_rebuild(params->area);
    if (params->area) {
      ED_area_tag_redraw(params->area);
    }
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Header Region
 * \{ */

static void datablock_graph_header_region_init(wmWindowManager * /*wm*/, ARegion *region)
{
  ED_region_header_init(region);
}

static void datablock_graph_header_region_draw(const bContext *C, ARegion *region)
{
  ED_region_header(C, region);
}

static void datablock_graph_header_region_listener(const wmRegionListenerParams *params)
{
  ARegion *region = params->region;
  const wmNotifier *wmn = params->notifier;
  switch (wmn->category) {
    case NC_SPACE:
      if (wmn->data == ND_SPACE_DATABLOCK_GRAPH) {
        ED_region_tag_redraw(region);
      }
      break;
    default:
      break;
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Toolsystem Toolbar (T)
 * \{ */

static void datablock_graph_toolbar_region_init(wmWindowManager *wm, ARegion *region)
{
  ED_region_panels_init(wm, region);
}

static void datablock_graph_toolbar_region_draw(const bContext *C, ARegion *region)
{
  ED_region_panels(C, region);
}

static void datablock_graph_toolbar_region_listener(const wmRegionListenerParams *params)
{
  ARegion *region = params->region;
  const wmNotifier *wmn = params->notifier;
  switch (wmn->category) {
    case NC_WORKSPACE:
    case NC_SPACE:
    case NC_WINDOW:
      ED_region_tag_redraw(region);
      break;
    default:
      break;
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Browser (Channels) Region
 * \{ */

static void datablock_graph_browser_region_init(wmWindowManager *wm, ARegion *region)
{
  region->v2d.scroll = V2D_SCROLL_RIGHT | V2D_SCROLL_VERTICAL_HIDE;
  ED_region_panels_init(wm, region);
}

static void datablock_graph_browser_region_draw(const bContext *C, ARegion *region)
{
  datablock_graph_draw_browser(C, region);
}

static void datablock_graph_browser_region_listener(const wmRegionListenerParams *params)
{
  ARegion *region = params->region;
  const wmNotifier *wmn = params->notifier;
  switch (wmn->category) {
    case NC_ID:
    case NC_WINDOW:
    case NC_SPACE:
      ED_region_tag_redraw(region);
      break;
    default:
      break;
  }
}

/** \} */

void ED_spacetype_datablock_graph()
{
  std::unique_ptr<SpaceType> st = std::make_unique<SpaceType>();
  ARegionType *art;

  st->spaceid = SPACE_DATABLOCK_GRAPH;
  STRNCPY(st->name, "Data-Block Relations");

  st->create = datablock_graph_create;
  st->free = datablock_graph_free;
  st->init = datablock_graph_init;
  st->duplicate = datablock_graph_duplicate;
  st->operatortypes = datablock_graph_operatortypes;
  st->keymap = datablock_graph_keymap;
  st->listener = datablock_graph_area_listener;
  st->dropboxes = datablock_graph_dropboxes;
  st->id_remap = datablock_graph_id_remap;
  st->foreach_id = datablock_graph_foreach_id;
  st->blend_read_data = datablock_graph_blend_read_data;
  st->blend_read_after_liblink = datablock_graph_blend_read_after_liblink;
  st->blend_write = datablock_graph_blend_write;

  /* Main window — TOOL so toolsystem box/circle/lasso tools work (node-editor style). */
  art = MEM_new_zeroed<ARegionType>("spacetype data-block graph main");
  art->regionid = RGN_TYPE_WINDOW;
  art->keymapflag = ED_KEYMAP_TOOL | ED_KEYMAP_GIZMO | ED_KEYMAP_VIEW2D | ED_KEYMAP_FRAMES;
  art->init = datablock_graph_main_region_init;
  art->draw = datablock_graph_main_region_draw;
  art->listener = datablock_graph_main_region_listener;
  BLI_addhead(&st->regiontypes, art);

  /* Header. */
  art = MEM_new_zeroed<ARegionType>("spacetype data-block graph header");
  art->regionid = RGN_TYPE_HEADER;
  art->prefsizey = HEADERY;
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_VIEW2D | ED_KEYMAP_HEADER | ED_KEYMAP_FRAMES;
  art->init = datablock_graph_header_region_init;
  art->draw = datablock_graph_header_region_draw;
  art->listener = datablock_graph_header_region_listener;
  BLI_addhead(&st->regiontypes, art);

  /* Toolsystem toolbar (T). */
  art = MEM_new_zeroed<ARegionType>("spacetype data-block graph tools");
  art->regionid = RGN_TYPE_TOOLS;
  art->prefsizex = int(UI_TOOLBAR_WIDTH);
  art->prefsizey = 50;
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_FRAMES;
  art->listener = datablock_graph_toolbar_region_listener;
  art->message_subscribe = ED_region_generic_tools_region_message_subscribe;
  art->snap_size = ED_region_generic_tools_region_snap_size;
  art->init = datablock_graph_toolbar_region_init;
  art->draw = datablock_graph_toolbar_region_draw;
  BLI_addhead(&st->regiontypes, art);

  /* Left libraries browser. */
  art = MEM_new_zeroed<ARegionType>("spacetype data-block graph browser");
  art->regionid = RGN_TYPE_CHANNELS;
  art->prefsizex = 220 + V2D_SCROLL_WIDTH;
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_FRAMES;
  art->init = datablock_graph_browser_region_init;
  art->draw = datablock_graph_browser_region_draw;
  art->listener = datablock_graph_browser_region_listener;
  datablock_graph_browser_panels_register(*art);
  BLI_addhead(&st->regiontypes, art);

  BKE_spacetype_register(std::move(st));
}

}  // namespace blender
