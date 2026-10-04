/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* IMAGE_NODES_MVP: image node utilities. */

/** \file
 * \ingroup nodes
 */

#include <optional>

#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "NOD_socket_search_link.hh"

#include "UI_resources.hh"

#include "node_image_util.hh"

namespace blender {

bool img_node_poll_default(const bke::bNodeType * /*ntype*/,
                           const bNodeTree *ntree,
                           const char **r_disabled_hint)
{
  if (!STREQ(ntree->idname, "ImageNodeTree")) {
    *r_disabled_hint = RPT_("Not an image node tree");
    return false;
  }
  return true;
}

static int img_node_ui_icon(const UString &idname)
{
  /* Match the GTE Python add-on's bl_icon where there is a counterpart. */
  if (idname == "ImageNodeViewer"_ustr) {
    return ICON_RESTRICT_VIEW_OFF;
  }
  if (idname == "ImageNodeFileOutput"_ustr) {
    return ICON_OUTPUT;
  }
  if (idname == "ImageNodeBakeImage"_ustr) {
    return ICON_FILE_IMAGE;
  }
  if (idname == "ImageNodePaint"_ustr) {
    return ICON_BRUSH_DATA;
  }
  if (idname == "ImageNodeCameraView"_ustr) {
    return ICON_CAMERA_DATA;
  }
  if (idname == "ImageNodeSceneTime"_ustr || idname == "ImageNodeSceneFrame"_ustr) {
    return ICON_TIME;
  }
  if (idname == "ImageNodeFFT"_ustr) {
    return ICON_IPO_ELASTIC;
  }
  if (idname == "ImageNodeShaderToy"_ustr) {
    return ICON_SCRIPT;
  }
  if (idname == "ImageNodeTerrainPrimitive"_ustr) {
    return ICON_RNDCURVE;
  }
  if (idname == "ImageNodeTerrainErosion"_ustr || idname == "ImageNodeFluidSimInput"_ustr ||
      idname == "ImageNodeFluidSimOutput"_ustr)
  {
    return ICON_MOD_FLUIDSIM;
  }
  if (idname == "ImageNodeTerrainSimulate"_ustr) {
    return ICON_PHYSICS;
  }
  if (idname == "ImageNodeTerrainColor"_ustr || idname == "ImageNodeTerrainCombine"_ustr) {
    return ICON_COLOR;
  }
  if (idname == "ImageNodeTerrainDerive"_ustr) {
    return ICON_NORMALS_FACE;
  }
  if (idname == "ImageNodeTerrainModify"_ustr) {
    return ICON_MOD_SIMPLEDEFORM;
  }
  if (idname == "ImageNodeRasterizeGeometry"_ustr || idname == "ImageNodeGeoSDF"_ustr) {
    return ICON_MESH_DATA;
  }
  if (idname == "ImageNodeImportGeo"_ustr) {
    return ICON_IMPORT;
  }
  if (idname == "ImageNodeImportPoints"_ustr) {
    return ICON_POINTCLOUD_DATA;
  }
  if (idname == "ImageNodePointStamp"_ustr) {
    return ICON_PARTICLE_POINT;
  }
  if (idname == "ImageNodeSDFShape"_ustr) {
    return ICON_MESH_ICOSPHERE;
  }
  if (idname == "ImageNodeFractalPrimitive"_ustr) {
    return ICON_TEXTURE;
  }
  if (idname == "ImageNodeRenderMaterial"_ustr) {
    return ICON_MATERIAL;
  }
  if (idname == "ImageNodeSampleAtPixel"_ustr) {
    return ICON_EYEDROPPER;
  }
  if (idname == "ImageNodeWriteAtPixel"_ustr) {
    return ICON_MOD_DATA_TRANSFER;
  }
  if (idname == "ImageNodeIslandUV"_ustr) {
    return ICON_UV;
  }
  if (idname == "ImageNodeIslandPadding"_ustr) {
    return ICON_MOD_EDGESPLIT;
  }
  if (idname == "ImageNodeHistogram"_ustr) {
    return ICON_SEQ_HISTOGRAM;
  }
  if (idname == "ImageNodeHeightToNormal"_ustr) {
    return ICON_NORMALS_VERTEX;
  }
  if (idname == "ImageNodeNormalToHeight"_ustr) {
    return ICON_NORMALS_FACE;
  }
  if (idname == "ImageNodeDivergence"_ustr) {
    return ICON_FORCE_FORCE;
  }
  if (idname == "ImageNodeGradient"_ustr) {
    return ICON_IPO_LINEAR;
  }
  if (idname == "ImageNodeFlip"_ustr) {
    return ICON_MOD_MIRROR;
  }
  if (idname == "ImageNodeBrightContrast"_ustr) {
    return ICON_IMAGE_RGB;
  }
  return ICON_NODE;
}

void img_node_type_base(bke::bNodeType *ntype,
                        UString idname,
                        const std::optional<int16_t> legacy_type)
{
  bke::node_type_base(*ntype, idname, legacy_type);

  ntype->poll = img_node_poll_default;
  ntype->insert_link = node_insert_link_default;
  ntype->gather_link_search_ops = nodes::search_link_ops_for_basic_node;
  ntype->ui_icon = img_node_ui_icon(idname);
}

}  // namespace blender
