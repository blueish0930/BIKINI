/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_image_paint_layers.hh"

#include <algorithm>
#include <cstring>

#include "BKE_image.hh"
#include "BKE_image_gpu.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
#include "BKE_main.hh"
#include "BKE_packedFile.hh"
#include "BKE_report.hh"
#include "DNA_ID.h"

#include "WM_api.hh"
#include "WM_types.hh"

#include "BLI_implicit_sharing.hh"
#include "BLI_listbase.hh"
#include "BLI_math_color_c.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_rect.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_string_utils.hh"
#include "BLI_utildefines.hh"

#include "BLO_read_write.hh"

#include "CLG_log.h"

#include "DEG_depsgraph.hh"

#include "DNA_packedFile_types.h"

#include "IMB_colormanagement.hh"
#include "IMB_imbuf.hh"
#include "IMB_imbuf_enums.h"
#include "IMB_imbuf_types.hh"
#include "IMB_partial_update.hh"

#include "MEM_guardedalloc.h"

namespace blender {

static CLG_LogRef LOG = {"image.paint_layers"};

static int tile_number_from_iuser(const Image &ima, const ImageUser *iuser)
{
  if (iuser && iuser->tile != 0) {
    return iuser->tile;
  }
  if (const ImageTile *tile = ima.tiles.first()) {
    return tile->tile_number;
  }
  return 1001;
}

static ImagePaintLayerTile *tiles_find(ListBaseT<ImagePaintLayerTile> &tiles,
                                       const int tile_number)
{
  for (ImagePaintLayerTile &tile : tiles) {
    if (tile.tile_number == tile_number) {
      return &tile;
    }
  }
  return nullptr;
}

static const ImagePaintLayerTile *tiles_find(const ListBaseT<ImagePaintLayerTile> &tiles,
                                             const int tile_number)
{
  for (const ImagePaintLayerTile &tile : tiles) {
    if (tile.tile_number == tile_number) {
      return &tile;
    }
  }
  return nullptr;
}

static ImagePaintLayerTile *layer_find_tile(ImagePaintLayer &layer, const int tile_number)
{
  return tiles_find(layer.tiles, tile_number);
}

static const ImagePaintLayerTile *layer_find_tile(const ImagePaintLayer &layer,
                                                  const int tile_number)
{
  return tiles_find(layer.tiles, tile_number);
}

static ImagePaintLayerTile *tiles_ensure(ListBaseT<ImagePaintLayerTile> &tiles,
                                         const int tile_number)
{
  if (ImagePaintLayerTile *tile = tiles_find(tiles, tile_number)) {
    return tile;
  }
  ImagePaintLayerTile *tile = MEM_new<ImagePaintLayerTile>(__func__);
  tile->tile_number = tile_number;
  tile->is_empty = 1;
  BLI_addtail(&tiles, tile);
  return tile;
}

static ImagePaintLayerTile *layer_ensure_tile(ImagePaintLayer &layer, const int tile_number)
{
  return tiles_ensure(layer.tiles, tile_number);
}

static void alloc_layer_ibuf_like(ImagePaintLayerTile &tile, const ImBuf &src)
{
  if (tile.ibuf) {
    return;
  }
  const bool use_float = src.float_data() != nullptr;
  tile.ibuf = IMB_allocImBuf(
      uint(src.x), uint(src.y), use_float ? ImBufFlags::FloatData : ImBufFlags::ByteData);
  if (tile.ibuf == nullptr) {
    return;
  }
  tile.ibuf->color_mode = src.color_mode;
  if (use_float) {
    const char *name = IMB_colormanagement_colorspace_get_name(src.float_buffer.colorspace);
    if (name) {
      IMB_colormanagement_assign_float_colorspace(tile.ibuf, name);
    }
  }
  else {
    const char *name = IMB_colormanagement_colorspace_get_name(src.byte_buffer.colorspace);
    if (name) {
      IMB_colormanagement_assign_byte_colorspace(tile.ibuf, name);
    }
  }
  tile.is_empty = 1;
}

static bool copy_ibuf_pixels(ImBuf &dst, ImBuf &src)
{
  if (dst.x != src.x || dst.y != src.y || dst.x < 1 || dst.y < 1) {
    return false;
  }
  IMB_ensure_host_buffer(&src);
  IMB_ensure_host_buffer(&dst);
  if (src.float_data() && dst.float_data_for_write()) {
    memcpy(dst.float_data_for_write(),
           src.float_data(),
           size_t(src.x) * size_t(src.y) * 4 * sizeof(float));
    return true;
  }
  if (src.byte_data() && dst.byte_data_for_write()) {
    memcpy(dst.byte_data_for_write(), src.byte_data(), size_t(src.x) * size_t(src.y) * 4);
    return true;
  }
  return false;
}

/**
 * Background starts as an alias of the canvas. The private copy is made only
 * when something is about to clear or paint the canvas, so adding a layer does
 * not memcpy an 8K/12K buffer on the UI thread.
 */
static void materialize_background_tiles(Image &ima, ImageUser *iuser, ImBuf &cache)
{
  const int tile_number = tile_number_from_iuser(ima, iuser);
  for (ImagePaintLayer &layer : ima.paint_layers) {
    if ((layer.flag & IMA_PAINT_LAYER_BACKGROUND) == 0) {
      continue;
    }
    ImagePaintLayerTile *tile = layer_ensure_tile(layer, tile_number);
    if (tile->ibuf) {
      continue;
    }
    alloc_layer_ibuf_like(*tile, cache);
    if (tile->ibuf && copy_ibuf_pixels(*tile->ibuf, cache)) {
      tile->is_empty = 0;
    }
  }
}

static void uniquename_layer(Image &ima, ImagePaintLayer &layer, const char *defname)
{
  BLI_uniquename(reinterpret_cast<ListBase *>(&ima.paint_layers),
                 &layer,
                 defname,
                 '.',
                 offsetof(ImagePaintLayer, name),
                 sizeof(layer.name));
}

static void uniquename_mask(ImagePaintLayer &layer, ImagePaintMask &mask)
{
  BLI_uniquename(reinterpret_cast<ListBase *>(&layer.masks),
                 &mask,
                 "Mask",
                 '.',
                 offsetof(ImagePaintMask, name),
                 sizeof(mask.name));
}

static void fill_ibuf_black(ImBuf &ibuf)
{
  /* Opaque black: luma 0 with A=1. A=0 would be treated as "no mask" (pass-through)
   * by mask_eval_at, which is the opposite of a default-hidden mask. */
  if (float *px = ibuf.float_data_for_write()) {
    const int n = ibuf.x * ibuf.y;
    for (int i = 0; i < n; i++) {
      px[i * 4 + 0] = 0.0f;
      px[i * 4 + 1] = 0.0f;
      px[i * 4 + 2] = 0.0f;
      px[i * 4 + 3] = 1.0f;
    }
  }
  else if (uchar *px = ibuf.byte_data_for_write()) {
    const int n = ibuf.x * ibuf.y;
    for (int i = 0; i < n; i++) {
      px[i * 4 + 0] = 0;
      px[i * 4 + 1] = 0;
      px[i * 4 + 2] = 0;
      px[i * 4 + 3] = 255;
    }
  }
}

static ImBuf *acquire_source_ibuf(Image *src, Image *self)
{
  if (src == nullptr || src == self) {
    return nullptr;
  }
  ImageUser iuser{};
  BKE_imageuser_default(&iuser);
  if (src->tiles.first()) {
    iuser.tile = src->tiles.first()->tile_number;
  }
  return BKE_image_acquire_ibuf(src, &iuser, nullptr);
}

static float sample_ibuf_luma(const ImBuf &ibuf, int x, int y)
{
  x = std::clamp(x, 0, ibuf.x - 1);
  y = std::clamp(y, 0, ibuf.y - 1);
  if (const float *s = ibuf.float_data()) {
    s += (y * ibuf.x + x) * 4;
    return std::clamp(0.2126f * s[0] + 0.7152f * s[1] + 0.0722f * s[2], 0.0f, 1.0f);
  }
  if (const uchar *s = ibuf.byte_data()) {
    s += (y * ibuf.x + x) * 4;
    return (0.2126f * s[0] + 0.7152f * s[1] + 0.0722f * s[2]) / 255.0f;
  }
  return 1.0f;
}

static void sample_ibuf_rgba(const ImBuf &ibuf, int x, int y, float out[4])
{
  x = std::clamp(x, 0, ibuf.x - 1);
  y = std::clamp(y, 0, ibuf.y - 1);
  if (const float *s = ibuf.float_data()) {
    copy_v4_v4(out, s + (y * ibuf.x + x) * 4);
    return;
  }
  if (const uchar *s = ibuf.byte_data()) {
    rgba_uchar_to_float(out, s + (y * ibuf.x + x) * 4);
    return;
  }
  zero_v4(out);
}

static float mask_eval_at(const ImagePaintMask &mask,
                          const ImBuf *paint_ibuf,
                          const ImBuf *source_ibuf,
                          const int x,
                          const int y)
{
  float v = 1.0f;
  if (source_ibuf) {
    v = sample_ibuf_luma(*source_ibuf, x, y);
  }
  if (paint_ibuf) {
    float p[4];
    sample_ibuf_rgba(*paint_ibuf, x, y, p);
    const float pl = std::clamp(0.2126f * p[0] + 0.7152f * p[1] + 0.0722f * p[2], 0.0f, 1.0f);
    v = pl * p[3] + v * (1.0f - p[3]);
  }
  if (mask.flag & IMA_PAINT_MASK_INVERT) {
    v = 1.0f - v;
  }
  return std::clamp(v, 0.0f, 1.0f);
}

bool BKE_image_paint_layers_supported(const Image &ima)
{
  if (!ELEM(ima.source, IMA_SRC_FILE, IMA_SRC_GENERATED, IMA_SRC_TILED)) {
    return false;
  }
  if (!ELEM(ima.type, IMA_TYPE_IMAGE, IMA_TYPE_UV_TEST)) {
    return false;
  }
  if (ima.flag & IMA_USE_VIEWS) {
    return false;
  }
  return true;
}

bool BKE_image_paint_layers_has_stack(const Image &ima)
{
  return !BLI_listbase_is_empty(&ima.paint_layers);
}

int BKE_image_paint_layers_count(const Image &ima)
{
  return int(BLI_listbase_count(&ima.paint_layers));
}

bool BKE_image_paint_layers_is_identity_background(const Image &ima)
{
  if (!BKE_image_paint_layers_has_stack(ima)) {
    return true;
  }
  if (BKE_image_paint_layers_count(ima) != 1) {
    return false;
  }
  const ImagePaintLayer *layer = ima.paint_layers.first();
  if (layer == nullptr) {
    return true;
  }
  if ((layer->flag & IMA_PAINT_LAYER_BACKGROUND) == 0) {
    return false;
  }
  if (layer->flag & (IMA_PAINT_LAYER_HIDE | IMA_PAINT_LAYER_USE_IMAGE)) {
    return false;
  }
  if (layer->opacity != 1.0f || layer->blend != IMB_BLEND_MIX) {
    return false;
  }
  if (!BLI_listbase_is_empty(&layer->masks)) {
    return false;
  }
  /* Tile copies are redundant with the canvas. Ignore them so a previous
   * 8K materialize does not trap painting on the slow composite path. */
  return true;
}

ImagePaintLayer *BKE_image_paint_layers_active(Image &ima)
{
  return static_cast<ImagePaintLayer *>(
      BLI_findlink(&ima.paint_layers, ima.active_paint_layer_index));
}

int BKE_image_paint_layers_index_from_ibuf(const Image &ima, const ImBuf *ibuf)
{
  int layer_index = -1;
  int mask_index = -1;
  if (BKE_image_paint_layers_find_ibuf(ima, ibuf, &layer_index, &mask_index)) {
    return layer_index;
  }
  return -1;
}

bool BKE_image_paint_layers_find_ibuf(const Image &ima,
                                      const ImBuf *ibuf,
                                      int *r_layer_index,
                                      int *r_mask_index)
{
  if (r_layer_index) {
    *r_layer_index = -1;
  }
  if (r_mask_index) {
    *r_mask_index = -1;
  }
  if (ibuf == nullptr) {
    return false;
  }
  int layer_index = 0;
  for (const ImagePaintLayer &layer : ima.paint_layers) {
    for (const ImagePaintLayerTile &tile : layer.tiles) {
      if (tile.ibuf == ibuf) {
        if (r_layer_index) {
          *r_layer_index = layer_index;
        }
        return true;
      }
    }
    int mask_index = 0;
    for (const ImagePaintMask &mask : layer.masks) {
      for (const ImagePaintLayerTile &tile : mask.tiles) {
        if (tile.ibuf == ibuf) {
          if (r_layer_index) {
            *r_layer_index = layer_index;
          }
          if (r_mask_index) {
            *r_mask_index = mask_index;
          }
          return true;
        }
      }
      mask_index++;
    }
    layer_index++;
  }
  return false;
}

static bool ensure_background_from_canvas(Image &ima, ImageUser *iuser, ReportList *reports)
{
  if (BKE_image_paint_layers_has_stack(ima)) {
    return true;
  }
  if (!BKE_image_paint_layers_supported(ima)) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "Paint layers are not supported on this image type");
    }
    return false;
  }

  ImBuf *cache = BKE_image_acquire_ibuf(&ima, iuser, nullptr);
  if (cache == nullptr) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "Image has no pixel buffer");
    }
    return false;
  }

  ImagePaintLayer *bg = MEM_new<ImagePaintLayer>(__func__);
  STRNCPY_UTF8(bg->name, "Background");
  bg->flag = IMA_PAINT_LAYER_BACKGROUND;
  bg->blend = IMB_BLEND_MIX;
  bg->opacity = 1.0f;
  BLI_addtail(&ima.paint_layers, bg);

  const int tile_number = tile_number_from_iuser(ima, iuser);
  ImagePaintLayerTile *tile = layer_ensure_tile(*bg, tile_number);
  /* `is_empty == 0` with a null buffer means "pixels are still the canvas". */
  tile->is_empty = 0;
  BKE_image_release_ibuf(&ima, cache, nullptr);

  ima.active_paint_layer_index = 0;
  if (ima.runtime) {
    ima.runtime->paint_layers_composite_valid = true;
  }
  return true;
}

bool BKE_image_paint_layers_ensure_default(Image &ima, ImageUser *iuser)
{
  return ensure_background_from_canvas(ima, iuser, nullptr);
}

ImagePaintLayer *BKE_image_paint_layers_add(Image &ima, ImageUser *iuser, ReportList *reports)
{
  /* Empty stack: first Add creates only Background. A populated stack gets one
   * new transparent layer — never Background+Layer in a single click. */
  if (!BKE_image_paint_layers_has_stack(ima)) {
    if (!ensure_background_from_canvas(ima, iuser, reports)) {
      return nullptr;
    }
    return BKE_image_paint_layers_active(ima);
  }

  ImagePaintLayer *layer = MEM_new<ImagePaintLayer>(__func__);
  STRNCPY_UTF8(layer->name, "Layer");
  layer->blend = IMB_BLEND_MIX;
  layer->opacity = 1.0f;
  BLI_addhead(&ima.paint_layers, layer);
  uniquename_layer(ima, *layer, "Layer");

  /* No pixel buffer yet. An empty layer does not change the composite, so
   * allocating and recompositing an 8K/12K canvas here only stalls the UI.
   * The buffer is created on the first stroke that paints this layer. */
  ima.active_paint_layer_index = 0;
  return layer;
}

static void free_layer_tile(ImagePaintLayerTile &tile)
{
  IMB_freeImBuf(tile.ibuf);
  tile.ibuf = nullptr;
  if (tile.packedfile) {
    BKE_packedfile_free(tile.packedfile);
    tile.packedfile = nullptr;
  }
}

static void free_mask(ImagePaintMask &mask, const bool do_id_users)
{
  for (ImagePaintLayerTile &tile : mask.tiles.items_mutable()) {
    free_layer_tile(tile);
    MEM_delete(&tile);
  }
  mask.tiles.clear_no_delete();
  if (do_id_users && mask.source_image) {
    id_us_min(&mask.source_image->id);
  }
  mask.source_image = nullptr;
}

static void free_layer(ImagePaintLayer &layer, const bool do_id_users = true)
{
  for (ImagePaintMask &mask : layer.masks.items_mutable()) {
    free_mask(mask, do_id_users);
    MEM_delete(&mask);
  }
  layer.masks.clear_no_delete();
  for (ImagePaintLayerTile &tile : layer.tiles.items_mutable()) {
    free_layer_tile(tile);
    MEM_delete(&tile);
  }
  layer.tiles.clear_no_delete();
  if (do_id_users && layer.source_image) {
    id_us_min(&layer.source_image->id);
  }
  layer.source_image = nullptr;
}

bool BKE_image_paint_layers_remove_active(Image &ima, ImageUser *iuser, ReportList *reports)
{
  ImagePaintLayer *layer = BKE_image_paint_layers_active(ima);
  if (layer == nullptr) {
    return false;
  }
  if (BLI_listbase_count(&ima.paint_layers) <= 1) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "Cannot remove the last paint layer");
    }
    return false;
  }

  const int index = ima.active_paint_layer_index;
  BLI_remlink(&ima.paint_layers, layer);
  free_layer(*layer);
  MEM_delete(layer);

  ima.active_paint_layer_index = std::min(index, BKE_image_paint_layers_count(ima) - 1);
  BKE_image_paint_layers_invalidate_composite(ima);
  if (iuser) {
    rcti full;
    ImBuf *cache = BKE_image_acquire_ibuf(&ima, iuser, nullptr);
    if (cache) {
      BLI_rcti_init(&full, 0, cache->x, 0, cache->y);
      BKE_image_release_ibuf(&ima, cache, nullptr);
      BKE_image_paint_layers_notify_written(&ima, iuser, full);
    }
  }
  return true;
}

static void copy_layer_tile(ImagePaintLayerTile &dst, const ImagePaintLayerTile &src)
{
  dst.tile_number = src.tile_number;
  dst.is_empty = src.is_empty;
  if (src.ibuf) {
    dst.ibuf = IMB_dupImBuf(src.ibuf);
  }
  if (src.packedfile) {
    dst.packedfile = BKE_packedfile_duplicate(src.packedfile);
  }
}

ImagePaintLayer *BKE_image_paint_layers_duplicate_active(Image &ima, ImageUser *iuser)
{
  ImagePaintLayer *src = BKE_image_paint_layers_active(ima);
  if (src == nullptr) {
    return nullptr;
  }

  if (src->flag & IMA_PAINT_LAYER_BACKGROUND) {
    ImBuf *cache = BKE_image_acquire_ibuf(&ima, iuser, nullptr);
    if (cache) {
      materialize_background_tiles(ima, iuser, *cache);
      BKE_image_release_ibuf(&ima, cache, nullptr);
    }
  }

  ImagePaintLayer *dst = MEM_new<ImagePaintLayer>(__func__);
  STRNCPY(dst->name, src->name);
  dst->flag = eImagePaintLayer_Flag(src->flag & ~IMA_PAINT_LAYER_BACKGROUND);
  dst->blend = src->blend;
  dst->opacity = src->opacity;
  dst->source_image = src->source_image;
  if (dst->source_image) {
    id_us_plus(&dst->source_image->id);
  }
  dst->active_mask_index = src->active_mask_index;
  for (const ImagePaintLayerTile &src_tile : src->tiles) {
    ImagePaintLayerTile *dst_tile = MEM_new<ImagePaintLayerTile>(__func__);
    copy_layer_tile(*dst_tile, src_tile);
    BLI_addtail(&dst->tiles, dst_tile);
  }
  for (const ImagePaintMask &src_mask : src->masks) {
    ImagePaintMask *dst_mask = MEM_new<ImagePaintMask>(__func__);
    STRNCPY(dst_mask->name, src_mask.name);
    dst_mask->flag = src_mask.flag;
    dst_mask->blend = src_mask.blend;
    dst_mask->opacity = src_mask.opacity;
    dst_mask->source_image = src_mask.source_image;
    if (dst_mask->source_image) {
      id_us_plus(&dst_mask->source_image->id);
    }
    for (const ImagePaintLayerTile &src_tile : src_mask.tiles) {
      ImagePaintLayerTile *dst_tile = MEM_new<ImagePaintLayerTile>(__func__);
      copy_layer_tile(*dst_tile, src_tile);
      BLI_addtail(&dst_mask->tiles, dst_tile);
    }
    BLI_addtail(&dst->masks, dst_mask);
  }
  BLI_insertlinkbefore(&ima.paint_layers, src, dst);
  uniquename_layer(ima, *dst, "Layer");
  ima.active_paint_layer_index = BLI_findindex(&ima.paint_layers, dst);
  BKE_image_paint_layers_invalidate_composite(ima);
  if (iuser) {
    rcti full{0, 0, 0, 0};
    ImBuf *cache = BKE_image_acquire_ibuf(&ima, iuser, nullptr);
    if (cache) {
      BLI_rcti_init(&full, 0, cache->x, 0, cache->y);
      BKE_image_release_ibuf(&ima, cache, nullptr);
      BKE_image_paint_layers_notify_written(&ima, iuser, full);
    }
  }
  return dst;
}

bool BKE_image_paint_layers_move(Image &ima, const int direction)
{
  ImagePaintLayer *layer = BKE_image_paint_layers_active(ima);
  if (layer == nullptr) {
    return false;
  }
  if (BLI_listbase_link_move(reinterpret_cast<ListBase *>(&ima.paint_layers), layer, direction)) {
    ima.active_paint_layer_index = BLI_findindex(&ima.paint_layers, layer);
    BKE_image_paint_layers_invalidate_composite(ima);
    return true;
  }
  return false;
}

ImagePaintMask *BKE_image_paint_layers_mask_active(ImagePaintLayer &layer)
{
  return static_cast<ImagePaintMask *>(BLI_findlink(&layer.masks, layer.active_mask_index));
}

void BKE_image_paint_layers_set_active(Image &ima, int layer_index)
{
  const int tot = BKE_image_paint_layers_count(ima);
  if (tot == 0) {
    ima.active_paint_layer_index = 0;
    return;
  }
  if (layer_index < 0) {
    if (ImagePaintLayer *layer = BKE_image_paint_layers_active(ima)) {
      layer->flag = eImagePaintLayer_Flag(layer->flag & ~IMA_PAINT_LAYER_PAINT_MASK);
    }
    ima.active_paint_layer_index = -1;
    return;
  }
  ima.active_paint_layer_index = std::clamp(layer_index, 0, tot - 1);
  if (ImagePaintLayer *layer = BKE_image_paint_layers_active(ima)) {
    layer->flag = eImagePaintLayer_Flag(layer->flag & ~IMA_PAINT_LAYER_PAINT_MASK);
  }
}

void BKE_image_paint_layers_set_active_mask(Image &ima, int mask_index)
{
  ImagePaintLayer *layer = BKE_image_paint_layers_active(ima);
  if (layer == nullptr) {
    return;
  }
  const int tot = int(BLI_listbase_count(&layer->masks));
  if (tot == 0) {
    layer->active_mask_index = 0;
    layer->flag = eImagePaintLayer_Flag(layer->flag & ~IMA_PAINT_LAYER_PAINT_MASK);
    return;
  }
  layer->active_mask_index = std::clamp(mask_index, 0, tot - 1);
  layer->flag = eImagePaintLayer_Flag(layer->flag | IMA_PAINT_LAYER_PAINT_MASK |
                                      IMA_PAINT_LAYER_SHOW_MASKS);
}

void BKE_image_paint_layers_set_source(ImagePaintLayer &layer, Image *source)
{
  if (layer.source_image == source) {
    return;
  }
  id_us_min(reinterpret_cast<ID *>(layer.source_image));
  layer.source_image = source;
  id_us_plus(reinterpret_cast<ID *>(layer.source_image));
}

void BKE_image_paint_layers_mask_set_source(ImagePaintMask &mask, Image *source)
{
  if (mask.source_image == source) {
    return;
  }
  id_us_min(reinterpret_cast<ID *>(mask.source_image));
  mask.source_image = source;
  id_us_plus(reinterpret_cast<ID *>(mask.source_image));
}

ImagePaintMask *BKE_image_paint_layers_mask_add(Image &ima, ImageUser *iuser, ReportList *reports)
{
  ImagePaintLayer *layer = BKE_image_paint_layers_active(ima);
  if (layer == nullptr) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "No active paint layer");
    }
    return nullptr;
  }
  ImagePaintMask *mask = MEM_new<ImagePaintMask>(__func__);
  STRNCPY_UTF8(mask->name, "Mask");
  mask->blend = IMB_BLEND_MIX;
  mask->opacity = 1.0f;
  uniquename_mask(*layer, *mask);

  const int tile_number = tile_number_from_iuser(ima, iuser);
  ImBuf *cache = BKE_image_acquire_ibuf(&ima, iuser, nullptr);
  if (cache) {
    ImagePaintLayerTile *tile = tiles_ensure(mask->tiles, tile_number);
    alloc_layer_ibuf_like(*tile, *cache);
    if (tile->ibuf) {
      fill_ibuf_black(*tile->ibuf);
      tile->is_empty = 0;
      IMB_mark_dirty(tile->ibuf);
    }
    BKE_image_release_ibuf(&ima, cache, nullptr);
  }
  BLI_addtail(&layer->masks, mask);
  layer->active_mask_index = BLI_findindex(&layer->masks, mask);
  layer->flag = eImagePaintLayer_Flag(layer->flag | IMA_PAINT_LAYER_SHOW_MASKS |
                                      IMA_PAINT_LAYER_PAINT_MASK);
  BKE_image_paint_layers_invalidate_composite(ima);
  return mask;
}

bool BKE_image_paint_layers_mask_remove_active(Image &ima, ReportList *reports)
{
  ImagePaintLayer *layer = BKE_image_paint_layers_active(ima);
  if (layer == nullptr) {
    return false;
  }
  ImagePaintMask *mask = BKE_image_paint_layers_mask_active(*layer);
  if (mask == nullptr) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "No active mask");
    }
    return false;
  }
  const int index = layer->active_mask_index;
  BLI_remlink(&layer->masks, mask);
  free_mask(*mask, true);
  MEM_delete(mask);
  const int tot = int(BLI_listbase_count(&layer->masks));
  layer->active_mask_index = tot ? std::min(index, tot - 1) : 0;
  if (tot == 0) {
    layer->flag = eImagePaintLayer_Flag(layer->flag & ~IMA_PAINT_LAYER_PAINT_MASK);
  }
  BKE_image_paint_layers_invalidate_composite(ima);
  return true;
}

bool BKE_image_paint_layers_mask_move(Image &ima, int direction)
{
  ImagePaintLayer *layer = BKE_image_paint_layers_active(ima);
  if (layer == nullptr) {
    return false;
  }
  ImagePaintMask *mask = BKE_image_paint_layers_mask_active(*layer);
  if (mask == nullptr) {
    return false;
  }
  if (BLI_listbase_link_move(reinterpret_cast<ListBase *>(&layer->masks), mask, direction)) {
    layer->active_mask_index = BLI_findindex(&layer->masks, mask);
    BKE_image_paint_layers_invalidate_composite(ima);
    return true;
  }
  return false;
}

static void blend_layer_rect(ImBuf &dst,
                             const ImBuf &src,
                             const IMB_BlendMode mode,
                             const float opacity,
                             const rcti &rect)
{
  rcti clip = rect;
  clip.xmin = std::max(clip.xmin, 0);
  clip.ymin = std::max(clip.ymin, 0);
  clip.xmax = std::min(clip.xmax, std::min(dst.x, src.x));
  clip.ymax = std::min(clip.ymax, std::min(dst.y, src.y));
  if (clip.xmin >= clip.xmax || clip.ymin >= clip.ymax) {
    return;
  }

  if (dst.float_data() && src.float_data()) {
    float *d = dst.float_data_for_write();
    const float *s = src.float_data();
    for (int y = clip.ymin; y < clip.ymax; y++) {
      for (int x = clip.xmin; x < clip.xmax; x++) {
        const int i = (y * dst.x + x) * 4;
        const int j = (y * src.x + x) * 4;
        float src_px[4] = {s[j + 0], s[j + 1], s[j + 2], s[j + 3]};
        mul_v4_fl(src_px, opacity);
        float dst_px[4] = {d[i + 0], d[i + 1], d[i + 2], d[i + 3]};
        IMB_blend_color_float(dst_px, dst_px, src_px, mode);
        copy_v4_v4(&d[i], dst_px);
      }
    }
  }
  else if (dst.byte_data() && src.byte_data()) {
    uchar *d = dst.byte_data_for_write();
    const uchar *s = src.byte_data();
    for (int y = clip.ymin; y < clip.ymax; y++) {
      for (int x = clip.xmin; x < clip.xmax; x++) {
        const int i = (y * dst.x + x) * 4;
        const int j = (y * src.x + x) * 4;
        uchar src_px[4] = {s[j + 0], s[j + 1], s[j + 2], s[j + 3]};
        src_px[3] = uchar(src_px[3] * opacity);
        uchar dst_px[4] = {d[i + 0], d[i + 1], d[i + 2], d[i + 3]};
        IMB_blend_color_byte(dst_px, dst_px, src_px, mode);
        copy_v4_v4_uchar(&d[i], dst_px);
      }
    }
  }
}

struct MaskEval {
  const ImagePaintMask *mask = nullptr;
  const ImBuf *paint_ibuf = nullptr;
  const ImBuf *source_ibuf = nullptr;
  Image *release_ima = nullptr;
};

static void blend_layer_rect_masked(ImBuf &dst,
                                    const ImBuf *paint_ibuf,
                                    const ImBuf *source_ibuf,
                                    const ImagePaintLayer &layer,
                                    const Span<MaskEval> masks,
                                    const rcti &rect)
{
  rcti clip = rect;
  clip.xmin = std::max(clip.xmin, 0);
  clip.ymin = std::max(clip.ymin, 0);
  clip.xmax = std::min(clip.xmax, dst.x);
  clip.ymax = std::min(clip.ymax, dst.y);
  if (clip.xmin >= clip.xmax || clip.ymin >= clip.ymax) {
    return;
  }

  const IMB_BlendMode mode = IMB_BlendMode(layer.blend);
  const bool dst_float = dst.float_data() != nullptr;
  float *df = dst_float ? dst.float_data_for_write() : nullptr;
  uchar *db = dst_float ? nullptr : dst.byte_data_for_write();

  for (int y = clip.ymin; y < clip.ymax; y++) {
    for (int x = clip.xmin; x < clip.xmax; x++) {
      float src_px[4];
      bool have_src = false;
      if (paint_ibuf) {
        sample_ibuf_rgba(*paint_ibuf, x, y, src_px);
        have_src = true;
        if (source_ibuf) {
          float base[4];
          sample_ibuf_rgba(*source_ibuf, x, y, base);
          IMB_blend_color_float(src_px, base, src_px, IMB_BLEND_MIX);
        }
      }
      else if (source_ibuf) {
        sample_ibuf_rgba(*source_ibuf, x, y, src_px);
        have_src = true;
      }
      if (!have_src) {
        continue;
      }

      float mix = 1.0f;
      for (const MaskEval &ev : masks) {
        const float v = mask_eval_at(*ev.mask, ev.paint_ibuf, ev.source_ibuf, x, y);
        float src_m[4] = {v, v, v, 1.0f};
        mul_v4_fl(src_m, ev.mask->opacity);
        float dst_m[4] = {mix, mix, mix, 1.0f};
        IMB_blend_color_float(dst_m, dst_m, src_m, IMB_BlendMode(ev.mask->blend));
        mix = std::clamp(dst_m[0], 0.0f, 1.0f);
      }
      mix *= layer.opacity;
      if (mix <= 0.0f) {
        continue;
      }

      const int i = (y * dst.x + x) * 4;
      if (df) {
        src_px[0] *= mix;
        src_px[1] *= mix;
        src_px[2] *= mix;
        src_px[3] *= mix;
        float dst_px[4] = {df[i + 0], df[i + 1], df[i + 2], df[i + 3]};
        IMB_blend_color_float(dst_px, dst_px, src_px, mode);
        copy_v4_v4(&df[i], dst_px);
      }
      else if (db) {
        uchar src_b[4];
        rgba_float_to_uchar(src_b, src_px);
        src_b[3] = uchar(src_b[3] * mix);
        uchar dst_px[4] = {db[i + 0], db[i + 1], db[i + 2], db[i + 3]};
        IMB_blend_color_byte(dst_px, dst_px, src_b, mode);
        copy_v4_v4_uchar(&db[i], dst_px);
      }
    }
  }
}

void BKE_image_paint_layers_composite_region(Image &ima, ImageUser *iuser, const rcti *rect)
{
  if (!BKE_image_paint_layers_has_stack(ima) ||
      BKE_image_paint_layers_is_identity_background(ima))
  {
    if (ima.runtime) {
      ima.runtime->paint_layers_composite_valid = true;
    }
    return;
  }

  ImBuf *dst = BKE_image_acquire_ibuf(&ima, iuser, nullptr);
  if (dst == nullptr) {
    return;
  }

  /* Copy the canvas into a private background buffer before it is cleared. */
  materialize_background_tiles(ima, iuser, *dst);

  rcti full;
  if (rect) {
    full = *rect;
  }
  else {
    BLI_rcti_init(&full, 0, dst->x, 0, dst->y);
  }

  if (dst->float_data()) {
    float *px = dst->float_data_for_write();
    const int n = dst->x * dst->y * 4;
    if (full.xmin == 0 && full.ymin == 0 && full.xmax >= dst->x && full.ymax >= dst->y) {
      memset(px, 0, sizeof(float) * size_t(n));
    }
    else {
      for (int y = std::max(full.ymin, 0); y < std::min(full.ymax, dst->y); y++) {
        memset(px + (y * dst->x + std::max(full.xmin, 0)) * 4,
               0,
               sizeof(float) * 4 * size_t(std::min(full.xmax, dst->x) - std::max(full.xmin, 0)));
      }
    }
  }
  else if (dst->byte_data()) {
    uchar *px = dst->byte_data_for_write();
    const int n = dst->x * dst->y * 4;
    if (full.xmin == 0 && full.ymin == 0 && full.xmax >= dst->x && full.ymax >= dst->y) {
      memset(px, 0, sizeof(uchar) * size_t(n));
    }
    else {
      for (int y = std::max(full.ymin, 0); y < std::min(full.ymax, dst->y); y++) {
        memset(px + (y * dst->x + std::max(full.xmin, 0)) * 4,
               0,
               sizeof(uchar) * 4 * size_t(std::min(full.xmax, dst->x) - std::max(full.xmin, 0)));
      }
    }
  }

  const int tile_number = tile_number_from_iuser(ima, iuser);
  /* Composite back to front: last in list is Background. */
  for (ImagePaintLayer *layer = ima.paint_layers.last(); layer; layer = layer->prev) {
    if (layer->flag & IMA_PAINT_LAYER_HIDE) {
      continue;
    }
    if (layer->opacity <= 0.0f) {
      continue;
    }
    const ImagePaintLayerTile *tile = layer_find_tile(*layer, tile_number);
    const ImBuf *paint_ibuf = (tile && tile->ibuf && !tile->is_empty) ? tile->ibuf : nullptr;
    ImBuf *source_ibuf = nullptr;
    if ((layer->flag & IMA_PAINT_LAYER_USE_IMAGE) && layer->source_image) {
      source_ibuf = acquire_source_ibuf(layer->source_image, &ima);
    }
    if (paint_ibuf == nullptr && source_ibuf == nullptr) {
      continue;
    }

    Vector<MaskEval> mask_evals;
    for (ImagePaintMask &mask : layer->masks) {
      if (mask.flag & IMA_PAINT_MASK_HIDE) {
        continue;
      }
      MaskEval ev;
      ev.mask = &mask;
      if ((mask.flag & IMA_PAINT_MASK_USE_IMAGE) && mask.source_image) {
        ImBuf *mibuf = acquire_source_ibuf(mask.source_image, &ima);
        ev.source_ibuf = mibuf;
        ev.release_ima = mask.source_image;
      }
      else {
        const ImagePaintLayerTile *mtile = tiles_find(mask.tiles, tile_number);
        if (mtile && mtile->ibuf && !mtile->is_empty) {
          ev.paint_ibuf = mtile->ibuf;
        }
      }
      mask_evals.append(ev);
    }

    blend_layer_rect_masked(*dst, paint_ibuf, source_ibuf, *layer, mask_evals, full);

    for (MaskEval &ev : mask_evals) {
      if (ev.release_ima && ev.source_ibuf) {
        BKE_image_release_ibuf(ev.release_ima, const_cast<ImBuf *>(ev.source_ibuf), nullptr);
      }
    }
    if (source_ibuf) {
      BKE_image_release_ibuf(layer->source_image, source_ibuf, nullptr);
    }
  }

  ima.runtime->paint_layers_composite_valid = true;
  BKE_image_release_ibuf(&ima, dst, nullptr);
}

static void paint_layers_tag_image_update(Image *ima)
{
  if (ima == nullptr || ima->runtime == nullptr) {
    return;
  }
  /* GENERIC_DATABLOCK (geometry nodes / modifiers) plus editors. SYNC_TO_EVAL
   * alone does not flush to those consumers. */
  DEG_id_tag_update(&ima->id, 0);
  DEG_id_tag_update(&ima->id, ID_RECALC_SOURCE | ID_RECALC_EDITORS | ID_RECALC_SYNC_TO_EVAL);
}

static bool paint_stack_uses_source(const Image &ima, const Image &source)
{
  for (const ImagePaintLayer &layer : ima.paint_layers) {
    if ((layer.flag & IMA_PAINT_LAYER_USE_IMAGE) && layer.source_image == &source) {
      return true;
    }
    for (const ImagePaintMask &mask : layer.masks) {
      if ((mask.flag & IMA_PAINT_MASK_USE_IMAGE) && mask.source_image == &source) {
        return true;
      }
    }
  }
  return false;
}

static void paint_layers_refresh_canvas(Image &ima)
{
  ImageUser iuser{};
  BKE_imageuser_default(&iuser);
  if (const ImageTile *tile = ima.tiles.first()) {
    iuser.tile = tile->tile_number;
  }
  BKE_image_paint_layers_refresh_display(&ima, &iuser);
  paint_layers_tag_image_update(&ima);
  WM_main_add_notifier(NC_IMAGE | ND_DISPLAY, &ima);
  WM_main_add_notifier(NC_IMAGE | NA_EDITED, &ima);
}

void BKE_image_paint_layers_on_source_changed(Main &bmain, Image &source)
{
  bool any = false;
  for (Image *ima = static_cast<Image *>(bmain.images.first()); ima;
       ima = static_cast<Image *>(ima->id.next))
  {
    if (ima == &source) {
      continue;
    }
    if (!BKE_image_paint_layers_has_stack(*ima)) {
      continue;
    }
    if (!paint_stack_uses_source(*ima, source)) {
      continue;
    }
    paint_layers_refresh_canvas(*ima);
    any = true;
  }
  if (any) {
    WM_main_add_notifier(NC_OBJECT | ND_DRAW, nullptr);
  }
}

void BKE_image_paint_layers_notify_written(Image *ima, ImageUser *iuser, const rcti &rect)
{
  if (ima == nullptr) {
    return;
  }
  if (!BKE_image_paint_layers_has_stack(*ima) ||
      BKE_image_paint_layers_is_identity_background(*ima))
  {
    ImBuf *cache = BKE_image_acquire_ibuf(ima, iuser, nullptr);
    if (cache) {
      IMB_mark_dirty(cache);
      IMB_partial_update_mark_region(cache, rect);
      BKE_image_release_ibuf(ima, cache, nullptr);
    }
    DEG_id_tag_update(&ima->id, ID_RECALC_EDITORS | ID_RECALC_SYNC_TO_EVAL);
    return;
  }

  BKE_image_paint_layers_composite_region(*ima, iuser, &rect);
  ImBuf *cache = BKE_image_acquire_ibuf(ima, iuser, nullptr);
  if (cache) {
    IMB_mark_dirty(cache);
    IMB_partial_update_mark_region(cache, rect);
    BKE_image_release_ibuf(ima, cache, nullptr);
  }
  paint_layers_tag_image_update(ima);
}

void BKE_image_paint_layers_refresh_display(Image *ima, ImageUser *iuser)
{
  if (ima == nullptr) {
    return;
  }
  BKE_image_paint_layers_invalidate_composite(*ima);
  rcti full{0, 0, 0, 0};
  ImBuf *cache = BKE_image_acquire_ibuf(ima, iuser, nullptr);
  if (cache) {
    BLI_rcti_init(&full, 0, cache->x, 0, cache->y);
    BKE_image_release_ibuf(ima, cache, nullptr);
    BKE_image_paint_layers_notify_written(ima, iuser, full);
    cache = BKE_image_acquire_ibuf(ima, iuser, nullptr);
    if (cache) {
      IMB_partial_update_mark_full(cache);
      BKE_image_release_ibuf(ima, cache, nullptr);
    }
  }
  BKE_image_free_gpu_texture_caches(ima);
  paint_layers_tag_image_update(ima);
}

void BKE_image_paint_layers_invalidate_composite(Image &ima)
{
  if (ima.runtime) {
    ima.runtime->paint_layers_composite_valid = false;
  }
}

void BKE_image_paint_layers_ensure_composite_ibuf(Image &ima, ImageUser *iuser, ImBuf *cache_ibuf)
{
  if (cache_ibuf == nullptr || !BKE_image_paint_layers_has_stack(ima) ||
      BKE_image_paint_layers_is_identity_background(ima))
  {
    if (ima.runtime) {
      ima.runtime->paint_layers_composite_valid = true;
    }
    return;
  }
  if (ima.runtime == nullptr || ima.runtime->paint_layers_composite_valid) {
    return;
  }
  /* Prevent re-entry: composite acquires the cache ibuf again. */
  ima.runtime->paint_layers_composite_valid = true;
  rcti full;
  BLI_rcti_init(&full, 0, cache_ibuf->x, 0, cache_ibuf->y);
  BKE_image_paint_layers_composite_region(ima, iuser, &full);
}

ImBuf *BKE_image_paint_acquire_paint_ibuf(Image *ima, ImageUser *iuser, void **r_lock)
{
  if (ima == nullptr) {
    return nullptr;
  }
  /* Background-only stacks are the canvas. Copying/compositing an 8K tile here
   * is what made experimental 3D Texture Paint hitch versus official dailies. */
  if (!BKE_image_paint_layers_has_stack(*ima) ||
      BKE_image_paint_layers_is_identity_background(*ima))
  {
    if (BKE_image_paint_layers_has_stack(*ima)) {
      /* Drop leftover materialize copies so later compositing copies from the
       * live canvas instead of a stale 8K tile. */
      for (ImagePaintLayer &layer : ima->paint_layers) {
        for (ImagePaintLayerTile &tile : layer.tiles.items_mutable()) {
          if (tile.ibuf) {
            IMB_freeImBuf(tile.ibuf);
            tile.ibuf = nullptr;
          }
          tile.is_empty = 0;
        }
      }
    }
    return BKE_image_acquire_ibuf(ima, iuser, r_lock);
  }

  ImagePaintLayer *layer = BKE_image_paint_layers_active(*ima);
  if (layer == nullptr) {
    return nullptr;
  }
  if (layer->flag & IMA_PAINT_LAYER_LOCK) {
    return nullptr;
  }

  ImBuf *cache = BKE_image_acquire_ibuf(ima, iuser, r_lock);
  if (cache == nullptr) {
    return nullptr;
  }

  const int tile_number = tile_number_from_iuser(*ima, iuser);
  ImagePaintLayerTile *tile = nullptr;
  if ((layer->flag & IMA_PAINT_LAYER_PAINT_MASK) && !layer->masks.is_empty()) {
    ImagePaintMask *mask = BKE_image_paint_layers_mask_active(*layer);
    if (mask == nullptr || (mask->flag & IMA_PAINT_MASK_LOCK)) {
      BKE_image_release_ibuf(ima, cache, r_lock ? *r_lock : nullptr);
      if (r_lock) {
        *r_lock = nullptr;
      }
      return nullptr;
    }
    tile = tiles_ensure(mask->tiles, tile_number);
    alloc_layer_ibuf_like(*tile, *cache);
    if (tile->ibuf && tile->is_empty) {
      fill_ibuf_black(*tile->ibuf);
    }
  }
  else {
    tile = layer_ensure_tile(*layer, tile_number);
    if ((layer->flag & IMA_PAINT_LAYER_BACKGROUND) && tile->ibuf == nullptr) {
      materialize_background_tiles(*ima, iuser, *cache);
      tile = layer_ensure_tile(*layer, tile_number);
    }
    alloc_layer_ibuf_like(*tile, *cache);
  }
  BKE_image_release_ibuf(ima, cache, r_lock ? *r_lock : nullptr);
  if (r_lock) {
    *r_lock = nullptr;
  }
  if (tile == nullptr || tile->ibuf == nullptr) {
    return nullptr;
  }
  tile->is_empty = 0;
  IMB_refImBuf(tile->ibuf);
  IMB_mark_dirty(tile->ibuf);
  IMB_ensure_host_buffer(tile->ibuf);
  return tile->ibuf;
}

void BKE_image_paint_release_paint_ibuf(Image *ima, ImBuf *ibuf, void *lock)
{
  BKE_image_release_ibuf(ima, ibuf, lock);
}

static constexpr char kPaintTileMagic[8] = {'I', 'P', 'L', 'Y', '0', '0', '0', '1'};

struct PaintTileHeader {
  char magic[8];
  int x = 0;
  int y = 0;
  int is_float = 0;
  int _pad = 0;
};

static PackedFile *pack_ibuf_to_packedfile(ImBuf &ibuf)
{
  const bool is_float = ibuf.float_data() != nullptr;
  const void *pixels = is_float ? static_cast<const void *>(ibuf.float_data()) :
                                  static_cast<const void *>(ibuf.byte_data());
  if (pixels == nullptr || ibuf.x < 1 || ibuf.y < 1) {
    return nullptr;
  }
  const size_t pix_bytes = size_t(ibuf.x) * size_t(ibuf.y) * 4 *
                           (is_float ? sizeof(float) : sizeof(uchar));
  Vector<uint8_t> encoded(sizeof(PaintTileHeader) + pix_bytes);
  PaintTileHeader hdr{};
  memcpy(hdr.magic, kPaintTileMagic, sizeof(kPaintTileMagic));
  hdr.x = ibuf.x;
  hdr.y = ibuf.y;
  hdr.is_float = is_float ? 1 : 0;
  memcpy(encoded.data(), &hdr, sizeof(hdr));
  memcpy(encoded.data() + sizeof(hdr), pixels, pix_bytes);
  auto *shared_data = new ImplicitSharedValue<Vector<uint8_t>>(std::move(encoded));
  return BKE_packedfile_new_from_memory(
      shared_data->data.data(), int(shared_data->data.size()), shared_data);
}

static ImBuf *unpack_ibuf_from_packedfile(const PackedFile &pf, char colorspace[])
{
  const uchar *data = static_cast<const uchar *>(pf.data);
  if (data == nullptr || pf.size < int(sizeof(PaintTileHeader))) {
    return nullptr;
  }
  PaintTileHeader hdr{};
  memcpy(&hdr, data, sizeof(hdr));
  if (memcmp(hdr.magic, kPaintTileMagic, sizeof(kPaintTileMagic)) == 0) {
    if (hdr.x < 1 || hdr.y < 1) {
      return nullptr;
    }
    const bool is_float = hdr.is_float != 0;
    const size_t pix_bytes = size_t(hdr.x) * size_t(hdr.y) * 4 *
                             (is_float ? sizeof(float) : sizeof(uchar));
    if (size_t(pf.size) < sizeof(PaintTileHeader) + pix_bytes) {
      return nullptr;
    }
    ImBuf *ibuf = IMB_allocImBuf(uint(hdr.x),
                                 uint(hdr.y),
                                 is_float ? ImBufFlags::FloatData : ImBufFlags::ByteData);
    if (ibuf == nullptr) {
      return nullptr;
    }
    if (is_float) {
      memcpy(ibuf->float_data_for_write(), data + sizeof(hdr), pix_bytes);
      if (colorspace && colorspace[0]) {
        IMB_colormanagement_assign_float_colorspace(ibuf, colorspace);
      }
    }
    else {
      memcpy(ibuf->byte_data_for_write(), data + sizeof(hdr), pix_bytes);
      if (colorspace && colorspace[0]) {
        IMB_colormanagement_assign_byte_colorspace(ibuf, colorspace);
      }
    }
    return ibuf;
  }
  ColorManagedColorspaceSettings colorspace_settings{};
  if (colorspace && colorspace[0]) {
    STRNCPY_UTF8(colorspace_settings.name, colorspace);
  }
  return IMB_load_image_from_memory(data,
                                    size_t(pf.size),
                                    ImBufFlags::ByteData | ImBufFlags::FloatData,
                                    "<paint layer>",
                                    nullptr,
                                    (colorspace && colorspace[0]) ? &colorspace_settings : nullptr);
}

static void pack_tiles_dirty(ListBaseT<ImagePaintLayerTile> &tiles)
{
  for (ImagePaintLayerTile &tile : tiles) {
    if (tile.ibuf == nullptr) {
      continue;
    }
    IMB_ensure_host_buffer(tile.ibuf);
    if (tile.packedfile) {
      BKE_packedfile_free(tile.packedfile);
      tile.packedfile = nullptr;
    }
    tile.packedfile = pack_ibuf_to_packedfile(*tile.ibuf);
    if (tile.packedfile) {
      tile.ibuf->userflags &= ~IB_BITMAPDIRTY;
    }
  }
}

static void unpack_tiles(ListBaseT<ImagePaintLayerTile> &tiles, char colorspace[])
{
  for (ImagePaintLayerTile &tile : tiles) {
    if (tile.ibuf || tile.packedfile == nullptr) {
      continue;
    }
    tile.ibuf = unpack_ibuf_from_packedfile(*tile.packedfile, colorspace);
    tile.is_empty = (tile.ibuf == nullptr) ? 1 : 0;
  }
}

void BKE_image_paint_layers_pack_dirty(Image &ima)
{
  for (ImagePaintLayer &layer : ima.paint_layers) {
    pack_tiles_dirty(layer.tiles);
    for (ImagePaintMask &mask : layer.masks) {
      pack_tiles_dirty(mask.tiles);
    }
  }
}

void BKE_image_paint_layers_unpack(Image &ima)
{
  for (ImagePaintLayer &layer : ima.paint_layers) {
    unpack_tiles(layer.tiles, ima.colorspace_settings.name);
    for (ImagePaintMask &mask : layer.masks) {
      unpack_tiles(mask.tiles, ima.colorspace_settings.name);
    }
  }
}

bool BKE_image_paint_layers_merge_down(Image &ima, ImageUser *iuser, ReportList *reports)
{
  ImagePaintLayer *active = BKE_image_paint_layers_active(ima);
  if (active == nullptr || active->next == nullptr) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "No layer below to merge into");
    }
    return false;
  }
  ImagePaintLayer *below = active->next;
  const int tile_number = tile_number_from_iuser(ima, iuser);
  ImagePaintLayerTile *src_tile = layer_find_tile(*active, tile_number);
  ImagePaintLayerTile *dst_tile = layer_ensure_tile(*below, tile_number);

  ImBuf *cache = BKE_image_acquire_ibuf(&ima, iuser, nullptr);
  if (cache && dst_tile->ibuf == nullptr) {
    alloc_layer_ibuf_like(*dst_tile, *cache);
  }
  BKE_image_release_ibuf(&ima, cache, nullptr);

  if (src_tile && src_tile->ibuf && dst_tile->ibuf) {
    rcti full;
    BLI_rcti_init(&full, 0, dst_tile->ibuf->x, 0, dst_tile->ibuf->y);
    blend_layer_rect(*dst_tile->ibuf,
                     *src_tile->ibuf,
                     IMB_BlendMode(active->blend),
                     active->opacity,
                     full);
    dst_tile->is_empty = 0;
    IMB_mark_dirty(dst_tile->ibuf);
  }

  BLI_remlink(&ima.paint_layers, active);
  free_layer(*active);
  MEM_delete(active);
  ima.active_paint_layer_index = BLI_findindex(&ima.paint_layers, below);
  BKE_image_paint_layers_invalidate_composite(ima);
  if (iuser) {
    rcti full{0, 0, 0, 0};
    ImBuf *ibuf = BKE_image_acquire_ibuf(&ima, iuser, nullptr);
    if (ibuf) {
      BLI_rcti_init(&full, 0, ibuf->x, 0, ibuf->y);
      BKE_image_release_ibuf(&ima, ibuf, nullptr);
      BKE_image_paint_layers_notify_written(&ima, iuser, full);
    }
  }
  return true;
}

bool BKE_image_paint_layers_flatten(Image &ima, ImageUser *iuser, ReportList *reports)
{
  if (!BKE_image_paint_layers_has_stack(ima)) {
    return true;
  }
  UNUSED_VARS(reports);

  rcti full;
  ImBuf *cache = BKE_image_acquire_ibuf(&ima, iuser, nullptr);
  if (cache == nullptr) {
    return false;
  }
  BLI_rcti_init(&full, 0, cache->x, 0, cache->y);
  BKE_image_release_ibuf(&ima, cache, nullptr);
  BKE_image_paint_layers_composite_region(ima, iuser, &full);

  cache = BKE_image_acquire_ibuf(&ima, iuser, nullptr);
  ImagePaintLayer *keep = ima.paint_layers.last();
  for (ImagePaintLayer &layer : ima.paint_layers.items_mutable()) {
    if (&layer == keep) {
      continue;
    }
    BLI_remlink(&ima.paint_layers, &layer);
    free_layer(layer);
    MEM_delete(&layer);
  }
  if (keep) {
    keep->flag = IMA_PAINT_LAYER_BACKGROUND;
    keep->blend = IMB_BLEND_MIX;
    keep->opacity = 1.0f;
    if (keep->source_image) {
      id_us_min(&keep->source_image->id);
      keep->source_image = nullptr;
    }
    for (ImagePaintMask &mask : keep->masks.items_mutable()) {
      free_mask(mask, true);
      MEM_delete(&mask);
    }
    keep->masks.clear_no_delete();
    keep->active_mask_index = 0;
    STRNCPY_UTF8(keep->name, "Background");
    const int tile_number = tile_number_from_iuser(ima, iuser);
    ImagePaintLayerTile *tile = layer_ensure_tile(*keep, tile_number);
    IMB_freeImBuf(tile->ibuf);
    tile->ibuf = cache ? IMB_dupImBuf(cache) : nullptr;
    tile->is_empty = 0;
  }
  BKE_image_release_ibuf(&ima, cache, nullptr);
  ima.active_paint_layer_index = 0;
  ima.runtime->paint_layers_composite_valid = true;
  return true;
}

void BKE_image_paint_layers_copy(Image &dst, const Image &src)
{
  BKE_image_paint_layers_free(dst);
  for (const ImagePaintLayer &src_layer : src.paint_layers) {
    ImagePaintLayer *dst_layer = MEM_new<ImagePaintLayer>(__func__);
    STRNCPY(dst_layer->name, src_layer.name);
    dst_layer->flag = src_layer.flag;
    dst_layer->blend = src_layer.blend;
    dst_layer->opacity = src_layer.opacity;
    dst_layer->source_image = src_layer.source_image;
    dst_layer->active_mask_index = src_layer.active_mask_index;
    for (const ImagePaintLayerTile &src_tile : src_layer.tiles) {
      ImagePaintLayerTile *dst_tile = MEM_new<ImagePaintLayerTile>(__func__);
      copy_layer_tile(*dst_tile, src_tile);
      BLI_addtail(&dst_layer->tiles, dst_tile);
    }
    for (const ImagePaintMask &src_mask : src_layer.masks) {
      ImagePaintMask *dst_mask = MEM_new<ImagePaintMask>(__func__);
      STRNCPY(dst_mask->name, src_mask.name);
      dst_mask->flag = src_mask.flag;
      dst_mask->blend = src_mask.blend;
      dst_mask->opacity = src_mask.opacity;
      dst_mask->source_image = src_mask.source_image;
      for (const ImagePaintLayerTile &src_tile : src_mask.tiles) {
        ImagePaintLayerTile *dst_tile = MEM_new<ImagePaintLayerTile>(__func__);
        copy_layer_tile(*dst_tile, src_tile);
        BLI_addtail(&dst_mask->tiles, dst_tile);
      }
      BLI_addtail(&dst_layer->masks, dst_mask);
    }
    BLI_addtail(&dst.paint_layers, dst_layer);
  }
  dst.active_paint_layer_index = src.active_paint_layer_index;
}

void BKE_image_paint_layers_free(Image &ima)
{
  for (ImagePaintLayer &layer : ima.paint_layers.items_mutable()) {
    free_layer(layer, false);
    MEM_delete(&layer);
  }
  ima.paint_layers.clear_no_delete();
  ima.active_paint_layer_index = 0;
}

void BKE_image_paint_layers_foreach_cache(ID *id,
                                          IDTypeForeachCacheFunctionCallback function_callback,
                                          void *user_data)
{
  Image *ima = id_cast<Image *>(id);
  IDCacheKey key;
  key.id_session_uid = id->session_uid;
  constexpr size_t paint_layer_cache_base = size_t(2) << 32;
  int layer_index = 0;
  for (ImagePaintLayer &layer : ima->paint_layers) {
    for (ImagePaintLayerTile &tile : layer.tiles) {
      key.identifier = paint_layer_cache_base + (size_t(layer_index) << 16) +
                       uint32_t(tile.tile_number);
      function_callback(id, &key, reinterpret_cast<void **>(&tile.ibuf), 0, user_data);
    }
    int mask_index = 0;
    for (ImagePaintMask &mask : layer.masks) {
      for (ImagePaintLayerTile &tile : mask.tiles) {
        key.identifier = paint_layer_cache_base + (size_t(1) << 48) +
                         (size_t(layer_index) << 24) + (size_t(mask_index) << 16) +
                         uint32_t(tile.tile_number);
        function_callback(id, &key, reinterpret_cast<void **>(&tile.ibuf), 0, user_data);
      }
      mask_index++;
    }
    layer_index++;
  }
}

void BKE_image_paint_layers_foreach_id(Image *ima, LibraryForeachIDData *data)
{
  for (ImagePaintLayer &layer : ima->paint_layers) {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, layer.source_image, IDWALK_CB_USER);
    for (ImagePaintMask &mask : layer.masks) {
      BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, mask.source_image, IDWALK_CB_USER);
    }
  }
}

void BKE_image_paint_layers_blend_write(BlendWriter *writer, Image &ima)
{
  if (!writer->is_undo()) {
    BKE_image_paint_layers_pack_dirty(ima);
  }
  writer->write_struct_list(&ima.paint_layers);
  for (ImagePaintLayer &layer : ima.paint_layers) {
    writer->write_struct_list(&layer.tiles);
    for (ImagePaintLayerTile &tile : layer.tiles) {
      BKE_packedfile_blend_write(writer, tile.packedfile);
    }
    writer->write_struct_list(&layer.masks);
    for (ImagePaintMask &mask : layer.masks) {
      writer->write_struct_list(&mask.tiles);
      for (ImagePaintLayerTile &tile : mask.tiles) {
        BKE_packedfile_blend_write(writer, tile.packedfile);
      }
    }
  }
}

void BKE_image_paint_layers_blend_read(BlendDataReader *reader, Image &ima)
{
  BLO_read_struct_list(reader, ImagePaintLayer, &ima.paint_layers);
  for (ImagePaintLayer &layer : ima.paint_layers) {
    BLO_read_struct_list(reader, ImagePaintLayerTile, &layer.tiles);
    for (ImagePaintLayerTile &tile : layer.tiles) {
      BKE_packedfile_blend_read(reader, &tile.packedfile, ima.filepath);
      if (!BLO_read_data_is_undo(reader)) {
        tile.ibuf = nullptr;
      }
    }
    BLO_read_struct_list(reader, ImagePaintMask, &layer.masks);
    for (ImagePaintMask &mask : layer.masks) {
      BLO_read_struct_list(reader, ImagePaintLayerTile, &mask.tiles);
      for (ImagePaintLayerTile &tile : mask.tiles) {
        BKE_packedfile_blend_read(reader, &tile.packedfile, ima.filepath);
        if (!BLO_read_data_is_undo(reader)) {
          tile.ibuf = nullptr;
        }
      }
    }
  }
  /* Unpack after foreach_cache: on file load that callback nulls runtime ibufs. */
}

}  // namespace blender
