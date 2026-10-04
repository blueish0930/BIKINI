/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 *
 * Photoshop-style texture paint layers on #Image.
 */

#include "BKE_idtype.hh"
#include "DNA_image_types.h"

#include "BLI_sys_types.hh"

namespace blender {

struct BlendDataReader;
struct BlendWriter;
struct Image;
struct ImagePaintLayer;
struct ImagePaintMask;
struct ImageUser;
struct ImBuf;
struct LibraryForeachIDData;
struct Main;
struct ReportList;
struct rcti;

bool BKE_image_paint_layers_supported(const Image &ima);
bool BKE_image_paint_layers_has_stack(const Image &ima);
int BKE_image_paint_layers_count(const Image &ima);
/**
 * True when painting can write the canvas in place: no stack, or a single
 * untouched Background layer. Extra layers / materialized tiles need compositing.
 */
bool BKE_image_paint_layers_is_identity_background(const Image &ima);

ImagePaintLayer *BKE_image_paint_layers_active(Image &ima);
int BKE_image_paint_layers_index_from_ibuf(const Image &ima, const ImBuf *ibuf);

/**
 * Ensure a default Background layer exists (copied from the canvas). No-op if the
 * stack is already populated or the image type does not support paint layers.
 */
bool BKE_image_paint_layers_ensure_default(Image &ima, ImageUser *iuser);

/** Add one transparent layer. If the stack is empty, create Background instead. */
ImagePaintLayer *BKE_image_paint_layers_add(Image &ima, ImageUser *iuser, ReportList *reports);
bool BKE_image_paint_layers_remove_active(Image &ima, ImageUser *iuser, ReportList *reports);
ImagePaintLayer *BKE_image_paint_layers_duplicate_active(Image &ima, ImageUser *iuser);
bool BKE_image_paint_layers_move(Image &ima, int direction);
bool BKE_image_paint_layers_merge_down(Image &ima, ImageUser *iuser, ReportList *reports);
bool BKE_image_paint_layers_flatten(Image &ima, ImageUser *iuser, ReportList *reports);

ImagePaintMask *BKE_image_paint_layers_mask_add(Image &ima, ImageUser *iuser, ReportList *reports);
bool BKE_image_paint_layers_mask_remove_active(Image &ima, ReportList *reports);
bool BKE_image_paint_layers_mask_move(Image &ima, int direction);
ImagePaintMask *BKE_image_paint_layers_mask_active(ImagePaintLayer &layer);
void BKE_image_paint_layers_set_active(Image &ima, int layer_index);
void BKE_image_paint_layers_set_active_mask(Image &ima, int mask_index);

/** Locate which layer/mask an ibuf belongs to. mask_index is -1 for layer pixels. */
bool BKE_image_paint_layers_find_ibuf(const Image &ima,
                                      const ImBuf *ibuf,
                                      int *r_layer_index,
                                      int *r_mask_index);
void BKE_image_paint_layers_set_source(ImagePaintLayer &layer, Image *source);
void BKE_image_paint_layers_mask_set_source(ImagePaintMask &mask, Image *source);

void BKE_image_paint_layers_foreach_id(Image *ima, LibraryForeachIDData *data);

ImBuf *BKE_image_paint_acquire_paint_ibuf(Image *ima, ImageUser *iuser, void **r_lock);
void BKE_image_paint_release_paint_ibuf(Image *ima, ImBuf *ibuf, void *lock);

void BKE_image_paint_layers_composite_region(Image &ima, ImageUser *iuser, const rcti *rect);
void BKE_image_paint_layers_notify_written(Image *ima, ImageUser *iuser, const rcti &rect);
void BKE_image_paint_layers_invalidate_composite(Image &ima);
/** Re-composite the stack into the Image cache and drop GPU textures so the 3D view updates. */
void BKE_image_paint_layers_refresh_display(Image *ima, ImageUser *iuser);

/**
 * A live Image used as a paint-layer / mask source just got new pixels (GTE Image Output, etc.).
 * Re-composite every paint stack that references it and tag those canvases so Texture Paint,
 * the Image Editor, and other CPU users pick up the change.
 */
void BKE_image_paint_layers_on_source_changed(Main &bmain, Image &source);

void BKE_image_paint_layers_pack_dirty(Image &ima);
void BKE_image_paint_layers_unpack(Image &ima);

void BKE_image_paint_layers_copy(Image &dst, const Image &src);
void BKE_image_paint_layers_free(Image &ima);
void BKE_image_paint_layers_foreach_cache(ID *id,
                                          IDTypeForeachCacheFunctionCallback function_callback,
                                          void *user_data);
void BKE_image_paint_layers_blend_write(BlendWriter *writer, Image &ima);
void BKE_image_paint_layers_blend_read(BlendDataReader *reader, Image &ima);

/** Called after cache ibuf is acquired so a stale cache is rebuilt from layers. */
void BKE_image_paint_layers_ensure_composite_ibuf(Image &ima, ImageUser *iuser, ImBuf *cache_ibuf);

}  // namespace blender
