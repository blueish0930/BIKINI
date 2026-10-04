/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <fmt/format.h>

#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_string.hh"
#include "BLI_string_ref.hh"
#include "BLI_string_utf8.hh"

#include "DNA_ID.h"
#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_vec_types.h"
#include "DNA_windowmanager_types.h"

#include "BKE_context.hh"
#include "BKE_idtype.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_screen.hh"

#include "BLF_api.hh"

#include "BLT_translation.hh"

#include "ED_screen.hh"

#include "GPU_immediate.hh"
#include "GPU_state.hh"

#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_interface_icons.hh"
#include "UI_resources.hh"
#include "UI_view2d.hh"

#include "datablock_graph_intern.hh"

namespace blender::ed::datablock_graph {

static float2 edge_point(const float2 &p1,
                         const float2 &c1,
                         const float2 &c2,
                         const float2 &p2,
                         const float t)
{
  const float u = 1.0f - t;
  return u * u * u * p1 + 3.0f * u * u * t * c1 + 3.0f * u * t * t * c2 + t * t * t * p2;
}

/**
 * View units per screen pixel (same as SpaceNode.runtime->aspect).
 * Only for #BLF_ASPECT sharpness — never multiply layout positions by this (zoom would misalign).
 */
static float view_aspect(const View2D &v2d, const ARegion &region)
{
  const float winx = std::max(float(region.winx), 1.0f);
  return BLI_rctf_size_x(&v2d.cur) / winx;
}

/**
 * Sharp view2d text (Geometry Nodes trick): BLF_ASPECT + size/aspect cancel for layout size
 * but rasterize at higher resolution. Positions stay in plain view units.
 */
static void blf_setup_view_text(const int fontid, const float size_ui, const float aspect)
{
  const float a = std::max(aspect, 1e-6f);
  BLF_enable(fontid, BLF_ASPECT);
  BLF_aspect(fontid, a, a, 1.0f);
  BLF_size(fontid, size_ui * UI_SCALE_FAC / a);
}

static void draw_edge(const GraphNode &from,
                      const GraphNode &to,
                      const bool is_cycle,
                      const int usage_count,
                      const float aspect)
{
  const rctf a = datablock_graph_node_rect(from);
  const rctf b = datablock_graph_node_rect(to);
  /* User (from) is left of used (to) after longest-path layout. */
  const float2 p1(a.xmax, (a.ymin + a.ymax) * 0.5f);
  const float2 p2(b.xmin, (b.ymin + b.ymax) * 0.5f);

  /* Bright wires on dark node-editor background (TH_WIRE is often near-black). */
  float color[4];
  if (is_cycle) {
    ui::theme::get_color_type_4fv(TH_REDALERT, SPACE_NODE, color);
  }
  else {
    ui::theme::get_color_type_4fv(TH_WIRE_INNER, SPACE_NODE, color);
    /* Fallback if theme wire is too dark. */
    const float lum = 0.2126f * color[0] + 0.7152f * color[1] + 0.0722f * color[2];
    if (lum < 0.35f) {
      color[0] = 0.75f;
      color[1] = 0.82f;
      color[2] = 0.95f;
    }
  }
  color[3] = 1.0f;

  GPUVertFormat *format = immVertexFormat();
  const uint pos = GPU_vertformat_attr_add(format, "pos", gpu::VertAttrType::SFLOAT_32_32);
  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);
  immUniformColor4fv(color);

  /* Pixel-sharp line width (retina uses U.pixelsize). */
  const float line_w = ((usage_count > 1) ? 2.0f : 1.0f) * U.pixelsize;
  GPU_line_width(line_w);

  const float dx = std::max(40.0f, math::distance(p1, p2) * 0.35f);
  const bool almost_horizontal = math::abs(p1.y - p2.y) < 4.0f;
  const float2 p2_draw = almost_horizontal ? float2(p2.x, p1.y) : p2;
  const float2 c1(p1.x + dx, p1.y);
  const float2 c2(p2_draw.x - dx, p2_draw.y);

  if (almost_horizontal) {
    immBegin(GPU_PRIM_LINES, 2);
    immVertex2f(pos, p1.x, p1.y);
    immVertex2f(pos, p2_draw.x, p2_draw.y);
  }
  else {
    constexpr int segments = 24;
    immBegin(GPU_PRIM_LINE_STRIP, segments + 1);
    for (int i = 0; i <= segments; i++) {
      const float t = float(i) / float(segments);
      const float2 p = edge_point(p1, c1, c2, p2_draw, t);
      immVertex2f(pos, p.x, p.y);
    }
  }
  immEnd();
  GPU_line_width(1.0f);

  const float2 near_tip = almost_horizontal ?
                              float2(p1.x * 0.15f + p2_draw.x * 0.85f, p1.y) :
                              edge_point(p1, c1, c2, p2_draw, 0.85f);
  const float2 dir = math::normalize(p2_draw - near_tip);
  const float2 n(-dir.y, dir.x);
  const float as = 8.0f * UI_SCALE_FAC;
  immBegin(GPU_PRIM_TRIS, 3);
  immVertex2f(pos, p2_draw.x, p2_draw.y);
  immVertex2f(pos,
              p2_draw.x - dir.x * as + n.x * as * 0.5f,
              p2_draw.y - dir.y * as + n.y * as * 0.5f);
  immVertex2f(pos,
              p2_draw.x - dir.x * as - n.x * as * 0.5f,
              p2_draw.y - dir.y * as - n.y * as * 0.5f);
  immEnd();
  immUnbindProgram();

  const float2 mid = almost_horizontal ? float2((p1.x + p2_draw.x) * 0.5f, p1.y) :
                                         edge_point(p1, c1, c2, p2_draw, 0.5f);
  const int fontid = BLF_default();

  if (is_cycle) {
    blf_setup_view_text(fontid, 11.0f, aspect);
    float err_col[4];
    ui::theme::get_color_4fv(TH_REDALERT, err_col);
    BLF_color4fv(fontid, err_col);
    const char *msg = IFACE_("CYCLE");
    const float tw = BLF_width(fontid, msg, strlen(msg));
    BLF_position(fontid, mid.x - tw * 0.5f, mid.y + 4.0f * UI_SCALE_FAC, 0.0f);
    BLF_draw(fontid, msg, strlen(msg));
  }

  /* Usage count on every edge (how many ID pointer links this wire aggregates). */
  {
    blf_setup_view_text(fontid, 11.0f, aspect);
    float label_col[4] = {0.92f, 0.94f, 0.98f, 1.0f};
    if (usage_count > 1) {
      label_col[0] = 1.0f;
      label_col[1] = 0.85f;
      label_col[2] = 0.35f;
    }
    BLF_color4fv(fontid, label_col);
    const std::string msg = fmt::format("×{}", std::max(1, usage_count));
    const float tw = BLF_width(fontid, msg.c_str(), msg.size());
    const float y_off = (is_cycle ? -12.0f : 4.0f) * UI_SCALE_FAC;
    BLF_position(fontid, mid.x - tw * 0.5f, mid.y + y_off, 0.0f);
    BLF_draw(fontid, msg.c_str(), msg.size());
  }
  BLF_disable(fontid, BLF_ASPECT);
}

/** Subtype tag for node groups (appended to the base "NodeTree" type name). */
static const char *node_tree_subtype_tag(const bNodeTree &ntree)
{
  switch (ntree.type) {
    case NTREE_GEOMETRY:
      return "GN";
    case NTREE_SHADER:
      return "Shader";
    case NTREE_COMPOSIT:
      return "Compositor";
    case NTREE_IMAGE:
      return "GPU Texture Editor";
    case NTREE_OBJECT:
      return "Object Editor";
    case NTREE_TEXTURE:
      return "Texture";
    default:
      return nullptr;
  }
}

/**
 * Type line under the name.
 * Node groups keep the generic ID type ("NodeTree") and add a subtype tag:
 * "NodeTree · GN", not just "GN".
 */
static std::string graph_node_type_label(const GraphNode &node)
{
  if (node.kind == GraphNodeKind::VirtualModifier) {
    if (node.modifier_type == eModifierType_Nodes) {
      return "Modifier · GN";
    }
    return "Modifier";
  }
  if (node.kind == GraphNodeKind::VirtualUISpace) {
    return "Editor";
  }
  if (node.id == nullptr) {
    return "?";
  }
  const char *base = BKE_idtype_idcode_to_name(GS(node.id->name));
  if (!base) {
    base = "?";
  }
  if (GS(node.id->name) == ID_NT) {
    const char *tag = node_tree_subtype_tag(*reinterpret_cast<const bNodeTree *>(node.id));
    if (tag) {
      return fmt::format("{} · {}", base, tag);
    }
  }
  return base;
}

/**
 * Icons for graph cards. Objects always use #ICON_OBJECT_DATA (wireframe frame +
 * solid square) — #ui::icon_from_id follows the object's data and collides with Mesh.
 * Modifiers use the wrench #ICON_MODIFIER.
 */
static int graph_icon_from_node(const GraphNode &node)
{
  if (node.kind == GraphNodeKind::VirtualModifier) {
    return ICON_MODIFIER;
  }
  if (node.kind == GraphNodeKind::VirtualUISpace) {
    switch (node.space_type) {
      case SPACE_IMAGE:
        return ICON_IMAGE;
      case SPACE_NODE:
        return ICON_NODETREE;
      case SPACE_TEXT:
        return ICON_TEXT;
      case SPACE_CLIP:
        return ICON_TRACKER;
      default:
        return ICON_WINDOW;
    }
  }
  if (node.id == nullptr) {
    return ICON_NONE;
  }
  if (GS(node.id->name) == ID_OB) {
    return ICON_OBJECT_DATA;
  }
  /* Node groups: always the generic node-group icon; subtype is text-only. */
  if (GS(node.id->name) == ID_NT) {
    return ICON_NODETREE;
  }
  return ui::icon_from_id(node.id);
}

/**
 * Distinct header colors per ID type (Mesh / Material / NodeTree / Image / …).
 * Values are mid-saturation so text and icons stay readable on dark body.
 */
static void id_type_header_color(const short idcode, float r_col[4])
{
  /* r, g, b */
  float c[3] = {0.36f, 0.40f, 0.48f};
  switch (ID_Type(idcode)) {
    case ID_OB:
      c[0] = 0.55f;
      c[1] = 0.48f;
      c[2] = 0.32f;
      break; /* Object: warm tan */
    case ID_ME:
      c[0] = 0.28f;
      c[1] = 0.52f;
      c[2] = 0.72f;
      break; /* Mesh: blue */
    case ID_CU_LEGACY:
    case ID_CV:
    case ID_PT:
    case ID_VO:
      c[0] = 0.30f;
      c[1] = 0.58f;
      c[2] = 0.62f;
      break; /* Curves/etc: cyan */
    case ID_MA:
      c[0] = 0.72f;
      c[1] = 0.32f;
      c[2] = 0.38f;
      break; /* Material: red */
    case ID_TE:
      c[0] = 0.55f;
      c[1] = 0.35f;
      c[2] = 0.65f;
      break; /* Texture: purple */
    case ID_IM:
      c[0] = 0.35f;
      c[1] = 0.62f;
      c[2] = 0.42f;
      break; /* Image: green */
    case ID_NT:
      c[0] = 0.45f;
      c[1] = 0.38f;
      c[2] = 0.72f;
      break; /* Node tree: indigo */
    case ID_SCE:
      c[0] = 0.38f;
      c[1] = 0.50f;
      c[2] = 0.35f;
      break; /* Scene: olive */
    case ID_GR:
      c[0] = 0.42f;
      c[1] = 0.55f;
      c[2] = 0.48f;
      break; /* Collection: muted green */
    case ID_LA:
      c[0] = 0.72f;
      c[1] = 0.68f;
      c[2] = 0.28f;
      break; /* Light: yellow */
    case ID_CA:
      c[0] = 0.50f;
      c[1] = 0.42f;
      c[2] = 0.55f;
      break; /* Camera: mauve */
    case ID_AR:
      c[0] = 0.58f;
      c[1] = 0.40f;
      c[2] = 0.30f;
      break; /* Armature: brown */
    case ID_WO:
      c[0] = 0.28f;
      c[1] = 0.42f;
      c[2] = 0.58f;
      break; /* World: steel blue */
    case ID_AC:
      c[0] = 0.62f;
      c[1] = 0.45f;
      c[2] = 0.52f;
      break; /* Action: rose */
    case ID_TXT:
      c[0] = 0.48f;
      c[1] = 0.48f;
      c[2] = 0.42f;
      break; /* Text: gray-olive */
    case ID_SO:
      c[0] = 0.40f;
      c[1] = 0.55f;
      c[2] = 0.58f;
      break; /* Sound */
    case ID_PA:
      c[0] = 0.55f;
      c[1] = 0.50f;
      c[2] = 0.28f;
      break; /* Particles */
    case ID_GP:
    case ID_GD_LEGACY:
      c[0] = 0.55f;
      c[1] = 0.38f;
      c[2] = 0.48f;
      break; /* Grease Pencil */
    default:
      break;
  }
  r_col[0] = c[0];
  r_col[1] = c[1];
  r_col[2] = c[2];
  r_col[3] = 1.0f;
}

static void node_header_color(const GraphNode &node, float r_col[4])
{
  if (node.kind == GraphNodeKind::VirtualModifier) {
    /* Modifier intermediate: teal, distinct from Object tan / NodeTree indigo. */
    r_col[0] = 0.32f;
    r_col[1] = 0.58f;
    r_col[2] = 0.55f;
    r_col[3] = 1.0f;
    return;
  }
  if (node.kind == GraphNodeKind::VirtualUISpace) {
    /* Editor user: cool slate. */
    r_col[0] = 0.38f;
    r_col[1] = 0.44f;
    r_col[2] = 0.52f;
    r_col[3] = 1.0f;
    return;
  }
  if (node.id && GS(node.id->name) == ID_NT) {
    /* Subtype colors so GN / Shader / Compositor / GPU TE are readable at a glance. */
    const bNodeTree &ntree = *reinterpret_cast<const bNodeTree *>(node.id);
    switch (ntree.type) {
      case NTREE_GEOMETRY:
        r_col[0] = 0.30f;
        r_col[1] = 0.55f;
        r_col[2] = 0.42f;
        r_col[3] = 1.0f;
        return; /* GN: green-teal */
      case NTREE_SHADER:
        r_col[0] = 0.62f;
        r_col[1] = 0.36f;
        r_col[2] = 0.48f;
        r_col[3] = 1.0f;
        return; /* Shader: rose */
      case NTREE_COMPOSIT:
        r_col[0] = 0.40f;
        r_col[1] = 0.42f;
        r_col[2] = 0.68f;
        r_col[3] = 1.0f;
        return; /* Compositor: blue-indigo */
      case NTREE_IMAGE:
        r_col[0] = 0.50f;
        r_col[1] = 0.45f;
        r_col[2] = 0.28f;
        r_col[3] = 1.0f;
        return; /* GPU Texture Editor: olive-gold */
      case NTREE_OBJECT:
        r_col[0] = 0.38f;
        r_col[1] = 0.62f;
        r_col[2] = 0.42f;
        r_col[3] = 1.0f;
        return; /* Object Editor: green */
      case NTREE_TEXTURE:
        r_col[0] = 0.55f;
        r_col[1] = 0.35f;
        r_col[2] = 0.65f;
        r_col[3] = 1.0f;
        return;
      default:
        break;
    }
  }
  const short idcode = node.id ? GS(node.id->name) : 0;
  id_type_header_color(idcode, r_col);
}

static void draw_node_box(const GraphNode &node, const bool is_active)
{
  const rctf rect = datablock_graph_node_rect(node);
  const float radius = 6.0f * UI_SCALE_FAC;

  float header_col[4];
  node_header_color(node, header_col);

  /* Body derived from header (darker). Seed nodes get a slightly brighter body. */
  float body[4] = {header_col[0] * 0.38f + 0.08f,
                   header_col[1] * 0.38f + 0.08f,
                   header_col[2] * 0.38f + 0.09f,
                   1.0f};
  if (node.is_seed) {
    body[0] = math::min(1.0f, body[0] + 0.04f);
    body[1] = math::min(1.0f, body[1] + 0.05f);
    body[2] = math::min(1.0f, body[2] + 0.05f);
  }
  if (is_active || node.selected) {
    body[0] = math::min(1.0f, body[0] + 0.08f);
    body[1] = math::min(1.0f, body[1] + 0.08f);
    body[2] = math::min(1.0f, body[2] + 0.10f);
    header_col[0] = math::min(1.0f, header_col[0] + 0.10f);
    header_col[1] = math::min(1.0f, header_col[1] + 0.10f);
    header_col[2] = math::min(1.0f, header_col[2] + 0.10f);
  }

  ui::draw_roundbox_corner_set(ui::CNR_ALL);
  ui::draw_roundbox_4fv(&rect, true, radius, body);

  rctf header = rect;
  header.ymin = rect.ymax - 24.0f * UI_SCALE_FAC;
  ui::draw_roundbox_corner_set(ui::CNR_TOP_LEFT | ui::CNR_TOP_RIGHT);
  ui::draw_roundbox_4fv(&header, true, radius, header_col);

  rctf body_inner = rect;
  body_inner.ymax = header.ymin;
  float inner[4] = {body[0] + 0.05f, body[1] + 0.05f, body[2] + 0.05f, 1.0f};
  ui::draw_roundbox_corner_set(ui::CNR_BOTTOM_LEFT | ui::CNR_BOTTOM_RIGHT);
  ui::draw_roundbox_4fv(&body_inner, true, radius, inner);

  /* Outline: type-tinted, gold when selected. */
  float outline[4];
  if (is_active || node.selected) {
    outline[0] = 0.95f;
    outline[1] = 0.75f;
    outline[2] = 0.25f;
  }
  else {
    outline[0] = math::min(1.0f, header_col[0] + 0.18f);
    outline[1] = math::min(1.0f, header_col[1] + 0.18f);
    outline[2] = math::min(1.0f, header_col[2] + 0.18f);
  }
  outline[3] = 1.0f;
  ui::draw_roundbox_corner_set(ui::CNR_ALL);
  ui::draw_roundbox_4fv(&rect, false, radius, outline);
}

static void draw_node_type_icon(const GraphNode &node)
{
  const int icon = graph_icon_from_node(node);
  if (icon == ICON_NONE) {
    return;
  }
  const rctf rect = datablock_graph_node_rect(node);
  /* Fixed view-space size (zooms with the node card — same as header padding). */
  const float icon_size = 14.0f * UI_SCALE_FAC;
  const float pad = 5.0f * UI_SCALE_FAC;
  const float x = rect.xmin + pad;
  const float y = rect.ymax - icon_size - pad;
  const float draw_aspect = UI_ICON_SIZE / std::max(icon_size, 1e-6f);
  ui::icon_draw_ex(x,
                   y,
                   BIFIconID(icon),
                   draw_aspect,
                   1.0f,
                   0.0f,
                   nullptr,
                   false,
                   UI_NO_ICON_OVERLAY_TEXT);
}

static void draw_node_fake_user_icon(const GraphNode &node)
{
  if (node.kind != GraphNodeKind::ID || node.id == nullptr ||
      (node.id->flag & ID_FLAG_FAKEUSER) == 0)
  {
    return;
  }
  const rctf rect = datablock_graph_node_rect(node);
  const float icon_size = 14.0f * UI_SCALE_FAC;
  const float pad = 4.0f * UI_SCALE_FAC;
  const float x = rect.xmax - icon_size - pad;
  const float y = rect.ymax - icon_size - pad;
  const float draw_aspect = UI_ICON_SIZE / std::max(icon_size, 1e-6f);
  ui::icon_draw_ex(x,
                   y,
                   ICON_FAKE_USER_OFF,
                   draw_aspect,
                   1.0f,
                   0.0f,
                   nullptr,
                   false,
                   UI_NO_ICON_OVERLAY_TEXT);
}

static void draw_node_label(const GraphNode &node, const float aspect)
{
  if (node.kind == GraphNodeKind::ID && node.id == nullptr) {
    return;
  }
  const rctf rect = datablock_graph_node_rect(node);
  const int fontid = BLF_default();
  /* Same view units as node card header — do not multiply by zoom aspect. */
  const float left_pad = 24.0f * UI_SCALE_FAC;

  /* Light text on dark node cards. */
  float text_color[4] = {0.92f, 0.93f, 0.95f, 1.0f};
  BLF_color4fv(fontid, text_color);

  const std::string type_name = graph_node_type_label(node);
  std::string line1;
  std::string line2;

  if (node.kind == GraphNodeKind::VirtualModifier) {
    line1 = node.display_name[0] ? node.display_name : "Modifier";
    if (node.host_id) {
      line2 = fmt::format("{}   on {}", type_name, BKE_id_name(*node.host_id));
    }
    else {
      line2 = type_name;
    }
  }
  else if (node.kind == GraphNodeKind::VirtualUISpace) {
    line1 = node.display_name[0] ? node.display_name : "Editor";
    if (node.host_id) {
      line2 = fmt::format("{}   in {}", type_name, BKE_id_name(*node.host_id));
    }
    else {
      line2 = type_name;
    }
  }
  else {
    const char *name = BKE_id_name(*node.id);
    line1 = name;
    if (node.id->flag & ID_FLAG_EMBEDDED_DATA) {
      line1 += "  E";
    }

    const int fake = ID_FAKE_USERS(node.id);
    const int real = ID_REAL_USERS(node.id);
    if (fake) {
      line2 = fmt::format("{}   users: {} ({}+F)", type_name, node.id->us, real);
    }
    else if (node.id->flag & ID_FLAG_EMBEDDED_DATA) {
      line2 = fmt::format("{}   users: {}  (embedded)", type_name, node.id->us);
    }
    else {
      line2 = fmt::format("{}   users: {}", type_name, node.id->us);
    }
  }

  blf_setup_view_text(fontid, 12.0f, aspect);
  /* Do not clip labels to node width — "users: N" / "on ObjectName" must stay fully readable.
   * Text may draw past the card edge (no scissor on this pass). */
  const float name_y = rect.ymax - 17.0f * UI_SCALE_FAC;
  const float sub_y = name_y - 24.0f * UI_SCALE_FAC;
  BLF_position(fontid, rect.xmin + left_pad, name_y, 0.0f);
  BLF_draw(fontid, line1.c_str(), line1.size());

  blf_setup_view_text(fontid, 10.0f, aspect);
  float sub_color[4] = {0.72f, 0.74f, 0.78f, 1.0f};
  BLF_color4fv(fontid, sub_color);
  BLF_position(fontid, rect.xmin + left_pad, sub_y, 0.0f);
  BLF_draw(fontid, line2.c_str(), line2.size());
  BLF_disable(fontid, BLF_ASPECT);
}

static void draw_empty_hint_pixel_space(const ARegion *region)
{
  const int fontid = BLF_default();
  BLF_size(fontid, 13.0f * UI_SCALE_FAC);

  float text_color[4];
  ui::theme::get_color_4fv(TH_TEXT, text_color);
  BLF_color4fv(fontid, text_color);

  const char *line1 = IFACE_("Select a data-block in the left browser to show who references it.");
  const char *line2 = IFACE_(
      "Users-only chain: outer users on the left, seed on the right. LMB drag nodes, MMB pan, "
      "wheel zoom.");

  const float margin = 24.0f * UI_SCALE_FAC;
  const float max_w = std::max(40.0f, float(region->winx) - margin * 2.0f);
  const float line_h = BLF_height_max(fontid) * 1.35f;

  auto draw_wrapped = [&](const char *text, float y) {
    /* Simple wrap by measuring. */
    std::string remaining = text;
    while (!remaining.empty()) {
      size_t fit = remaining.size();
      while (fit > 1 && BLF_width(fontid, remaining.c_str(), fit) > max_w) {
        /* Step back to previous space. */
        size_t cut = fit;
        while (cut > 0 && remaining[cut - 1] != ' ') {
          cut--;
        }
        if (cut == 0) {
          /* Hard cut. */
          while (fit > 1 && BLF_width(fontid, remaining.c_str(), fit) > max_w) {
            fit--;
          }
          break;
        }
        fit = cut - 1;
      }
      if (fit == 0) {
        fit = 1;
      }
      const std::string line = remaining.substr(0, fit);
      const float tw = BLF_width(fontid, line.c_str(), line.size());
      BLF_position(fontid, (float(region->winx) - tw) * 0.5f, y, 0.0f);
      BLF_draw(fontid, line.c_str(), line.size());
      y -= line_h;
      size_t next = fit;
      while (next < remaining.size() && remaining[next] == ' ') {
        next++;
      }
      remaining = remaining.substr(next);
    }
    return y;
  };

  float y = float(region->winy) * 0.55f;
  y = draw_wrapped(line1, y);
  y -= line_h * 0.3f;
  draw_wrapped(line2, y);
}

void datablock_graph_draw_main(const bContext *C, ARegion *region)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  Main *bmain = CTX_data_main(C);
  if (sdbg == nullptr || bmain == nullptr) {
    return;
  }

  datablock_graph_rebuild_if_needed(*sdbg, *bmain);
  datablock_graph_update_view2d(*region, *sdbg, false);
  SpaceDataBlockGraph_Runtime &runtime = *sdbg->runtime;

  View2D *v2d = &region->v2d;
  ui::view2d_view_ortho(v2d);

  /* Same aspect definition as Geometry Nodes: view-units per framebuffer pixel. */
  const float aspect = view_aspect(*v2d, *region);

  GPU_blend(GPU_BLEND_ALPHA);

  for (const GraphEdge &edge : runtime.edges) {
    if (edge.from_index < 0 || edge.to_index < 0 || edge.from_index >= int(runtime.nodes.size()) ||
        edge.to_index >= int(runtime.nodes.size()))
    {
      continue;
    }
    draw_edge(runtime.nodes[edge.from_index],
              runtime.nodes[edge.to_index],
              edge.is_cycle,
              edge.usage_count,
              aspect);
  }

  for (const GraphNode &node : runtime.nodes) {
    const uint64_t key = datablock_graph_node_key(node.seed_uid, node.id_session_uid);
    draw_node_box(node, key == runtime.active_key || node.selected);
    draw_node_label(node, aspect);
    draw_node_type_icon(node);
    draw_node_fake_user_icon(node);
  }

  GPU_blend(GPU_BLEND_NONE);
  ui::view2d_view_restore(C);
  ui::view2d_scrollers_draw(v2d, nullptr);

  if (runtime.nodes.is_empty()) {
    draw_empty_hint_pixel_space(region);
  }
}

}  // namespace blender::ed::datablock_graph
