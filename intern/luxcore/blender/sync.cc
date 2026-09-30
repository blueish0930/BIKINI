/* SPDX-FileCopyrightText: 2026 BIKINI Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "sync.hh"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <string>

#include "session.hh"

#include "BKE_appdir.hh"
#include "BKE_attribute.hh"
#include "BKE_camera.h"
#include "BKE_image.hh"
#include "BKE_material.hh"
#include "BKE_mesh.hh"
#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_object.hh"
#include "BKE_scene.hh"

#include "BLI_fileops.hh"
#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_math_base_c.hh"
#include "BLI_math_constants.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_rotation.hh"
#include "BLI_math_vector.hh"
#include "BLI_path_utils.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"
#include "BLI_virtual_array.hh"

#include "DNA_ID.h"
#include "DNA_image_types.h"

#include <cmath>

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "DNA_camera_types.h"
#include "DNA_light_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_view3d_types.h"
#include "DNA_world_types.h"

#include "RE_engine.h"

namespace blender::render::luxcore {

static std::string sanitize_name(const char *src, const char *prefix, const void *ptr)
{
  std::string out = prefix;
  out += '_';
  if (src) {
    for (const char *p = src; *p && out.size() < 80; p++) {
      const unsigned char c = static_cast<unsigned char>(*p);
      out += (isalnum(c) || *p == '_') ? *p : '_';
    }
  }
  char tail[32];
  SNPRINTF(tail, "_%p", ptr);
  out += tail;
  return out;
}

static const bNodeSocket *find_input(const bNode &node, const char *identifier)
{
  for (const bNodeSocket &sock : node.inputs) {
    if (STREQ(sock.identifier, identifier) || STREQ(sock.name, identifier)) {
      return &sock;
    }
  }
  return nullptr;
}

static const bNode *follow_input_node(const bNodeSocket *sock)
{
  if (sock == nullptr || sock->link == nullptr || sock->link->fromnode == nullptr) {
    return nullptr;
  }
  const bNode *from = sock->link->fromnode;
  /* Skip a single reroute. */
  if (STREQ(from->idname, "NodeReroute")) {
    for (const bNodeSocket &in : from->inputs) {
      if (in.link && in.link->fromnode) {
        return in.link->fromnode;
      }
    }
  }
  return from;
}

static float3 socket_color(const bNodeSocket *sock, const float3 fallback)
{
  if (sock == nullptr) {
    return fallback;
  }
  const auto *val = sock->default_value_typed<bNodeSocketValueRGBA>();
  if (val) {
    return float3(val->value[0], val->value[1], val->value[2]);
  }
  return fallback;
}

static float socket_float(const bNodeSocket *sock, float fallback)
{
  if (sock == nullptr) {
    return fallback;
  }
  const auto *val = sock->default_value_typed<bNodeSocketValueFloat>();
  if (val) {
    return val->value;
  }
  return fallback;
}

static bool extract_image_path(const bNode *tex_node, char r_path[FILE_MAX])
{
  if (tex_node == nullptr || tex_node->id == nullptr) {
    return false;
  }
  if (GS(tex_node->id->name) != ID_IM) {
    return false;
  }
  Image *image = reinterpret_cast<Image *>(tex_node->id);
  const NodeTexImage *tex = static_cast<const NodeTexImage *>(tex_node->storage);
  if (tex) {
    BKE_image_user_file_path(&tex->iuser, image, r_path);
  }
  else {
    BLI_strncpy(r_path, image->filepath, FILE_MAX);
  }
  return r_path[0] != '\0';
}

struct MaterialExport {
  std::string name;
};

static const bNodeSocket *find_input_any(const bNode &node,
                                         std::initializer_list<const char *> names)
{
  for (const char *n : names) {
    if (const bNodeSocket *s = find_input(node, n)) {
      return s;
    }
  }
  return nullptr;
}

static float3 socket_vector(const bNodeSocket *sock, const float3 fallback)
{
  if (sock == nullptr) {
    return fallback;
  }
  const auto *val = sock->default_value_typed<bNodeSocketValueVector>();
  if (val) {
    return float3(val->value[0], val->value[1], val->value[2]);
  }
  return fallback;
}

static bool socket_is_linked(const bNodeSocket *sock)
{
  return sock && sock->link && sock->link->fromnode;
}

static float3 absorption_coeff(const float3 color, float depth)
{
  depth = math::max(depth, 1.0e-6f);
  return float3(-logf(math::max(color.x, 1.0e-6f)) / depth,
                -logf(math::max(color.y, 1.0e-6f)) / depth,
                -logf(math::max(color.z, 1.0e-6f)) / depth);
}

static float3 read_emission(const bNode &node)
{
  const bNodeSocket *sock = find_input_any(node, {"Emission", "Emission Color"});
  const bNode *from = follow_input_node(sock);
  if (from && STREQ(from->idname, "ShaderNodeLuxEmission")) {
    const float3 col = socket_color(find_input(*from, "Color"), float3(1.0f, 1.0f, 1.0f));
    const float gain = socket_float(find_input(*from, "Gain"), 1.0f);
    const float exposure = socket_float(find_input(*from, "Exposure"), 0.0f);
    return col * (gain * powf(2.0f, exposure));
  }
  return socket_color(sock, float3(0.0f, 0.0f, 0.0f));
}

static void apply_common(Session::MaterialParams &p, const bNode &node)
{
  p.opacity = socket_float(find_input(node, "Opacity"), 1.0f);
  p.emission = read_emission(node);
  p.filmthickness = socket_float(find_input(node, "Film Thickness (nm)"), 0.0f);
  p.filmior = socket_float(find_input(node, "Film IOR"), 1.5f);
  p.filmamount = socket_float(find_input(node, "Film Amount"), 1.0f);
}

static std::string export_volume_node(Session &session,
                                      const bNode *node,
                                      Map<const bNode *, std::string> &cache)
{
  if (node == nullptr) {
    return "";
  }
  if (std::string *found = cache.lookup_ptr(node)) {
    return *found;
  }
  if (!STRPREFIX(node->idname, "ShaderNodeLuxVolume")) {
    return "";
  }
  const std::string name = sanitize_name(node->name, "vol", node);
  cache.add(node, name);

  Session::VolumeParams p;
  p.name = name;
  p.absorption = absorption_coeff(
      socket_color(find_input(*node, "Absorption"), float3(1.0f, 1.0f, 1.0f)),
      socket_float(find_input(*node, "Absorption Depth"), 1.0f));
  p.ior = socket_float(find_input(*node, "IOR"), 1.5f);
  p.emission = socket_color(find_input(*node, "Emission"), float3(0.0f, 0.0f, 0.0f));
  p.scattering = socket_color(find_input(*node, "Scattering"), float3(1.0f, 1.0f, 1.0f));
  p.scattering_scale = socket_float(find_input(*node, "Scattering Scale"), 1.0f);
  p.asymmetry = socket_vector(find_input(*node, "Asymmetry"), float3(0.0f, 0.0f, 0.0f));
  p.multiscattering = socket_float(find_input(*node, "Multiscattering"), 0.0f) > 0.5f;
  p.step_size = socket_float(find_input(*node, "Step Size"), 0.1f);
  p.maxcount = int(socket_float(find_input(*node, "Max Steps"), 1024.0f));

  if (STREQ(node->idname, "ShaderNodeLuxVolumeHomogeneous")) {
    p.type = "homogeneous";
  }
  else if (STREQ(node->idname, "ShaderNodeLuxVolumeHeterogeneous")) {
    p.type = "heterogeneous";
  }
  else {
    p.type = "clear";
  }
  session.add_volume(p);
  return name;
}

static std::string export_shader_node(Session &session,
                                      const bNode *node,
                                      Map<const bNode *, std::string> &cache);

static void fill_glass(Session::MaterialParams &p, const bNode &node, const char *forced_type)
{
  p.kt = socket_color(
      find_input_any(node, {"Transmission Color", "Transmission"}), float3(1.0f, 1.0f, 1.0f));
  p.kr = socket_color(
      find_input_any(node, {"Reflection Color", "Reflection"}), float3(1.0f, 1.0f, 1.0f));
  p.ior = socket_float(find_input(node, "IOR"), 1.5f);
  p.cauchyb = socket_float(find_input(node, "Dispersion"), 0.0f);
  p.roughness = socket_float(find_input(node, "Roughness"), 0.0f);
  apply_common(p, node);
  if (forced_type) {
    p.type = forced_type;
    return;
  }
  const float architectural = socket_float(find_input(node, "Architectural"), 0.0f);
  if (p.roughness > 1.0e-4f) {
    p.type = "roughglass";
  }
  else if (architectural > 0.5f) {
    p.type = "archglass";
  }
  else {
    p.type = "glass";
  }
}

static std::string export_shader_node(Session &session,
                                      const bNode *node,
                                      Map<const bNode *, std::string> &cache)
{
  if (node == nullptr) {
    session.add_matte("mat_default", float3(0.8f, 0.8f, 0.8f), float3(0.0f, 0.0f, 0.0f));
    return "mat_default";
  }
  if (std::string *found = cache.lookup_ptr(node)) {
    return *found;
  }

  if (STREQ(node->idname, "ShaderNodeLuxFrontBackOpacity")) {
    const std::string child = export_shader_node(
        session, follow_input_node(find_input(*node, "Material")), cache);
    session.patch_transparency(child,
                               socket_float(find_input(*node, "Front Opacity"), 1.0f),
                               socket_float(find_input(*node, "Back Opacity"), 1.0f));
    cache.add(node, child);
    return child;
  }

  const std::string name = sanitize_name(node->name, "mat", node);
  cache.add(node, name);

  if (STREQ(node->idname, "ShaderNodeLuxMix")) {
    Session::MaterialParams p;
    p.name = name;
    p.type = "mix";
    p.mix1 = export_shader_node(
        session, follow_input_node(find_input(*node, "Material 1")), cache);
    p.mix2 = export_shader_node(
        session, follow_input_node(find_input(*node, "Material 2")), cache);
    p.mix_amount = socket_float(find_input_any(*node, {"Mix Factor", "Amount"}), 0.5f);
    apply_common(p, *node);
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxTwoSided")) {
    Session::MaterialParams p;
    p.name = name;
    p.type = "twosided";
    p.mix1 = export_shader_node(
        session,
        follow_input_node(find_input_any(*node, {"Front Material", "Front"})),
        cache);
    p.mix2 = export_shader_node(
        session,
        follow_input_node(find_input_any(*node, {"Back Material", "Back"})),
        cache);
    apply_common(p, *node);
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxGlossyCoating")) {
    Session::MaterialParams p;
    p.name = name;
    p.type = "glossycoating";
    p.base = export_shader_node(
        session, follow_input_node(find_input(*node, "Base Material")), cache);
    p.ks = socket_color(find_input_any(*node, {"Specular Color", "Specular"}),
                        float3(0.05f, 0.05f, 0.05f));
    p.ior = socket_float(find_input(*node, "IOR"), 1.5f);
    p.use_ior = socket_is_linked(find_input(*node, "IOR"));
    p.ka = socket_color(find_input(*node, "Absorption Color"), float3(0.0f, 0.0f, 0.0f));
    p.absorption_depth = socket_float(find_input(*node, "Absorption Depth (nm)"), 0.0f);
    p.roughness = socket_float(find_input(*node, "Roughness"), 0.05f);
    p.multibounce = socket_float(find_input(*node, "Multibounce"), 0.0f) > 0.5f;
    apply_common(p, *node);
    session.add_material(p);
    return name;
  }

  Session::MaterialParams p;
  p.name = name;
  apply_common(p, *node);

  if (STREQ(node->idname, "ShaderNodeLuxDisney")) {
    p.type = "disney";
    const bNodeSocket *base_sock = find_input(*node, "Base Color");
    p.kd = socket_color(base_sock, p.kd);
    p.subsurface = socket_float(find_input(*node, "Subsurface"), 0.0f);
    p.metallic = socket_float(find_input(*node, "Metallic"), 0.0f);
    p.specular = socket_float(find_input(*node, "Specular"), 0.5f);
    p.speculartint = socket_float(find_input(*node, "Specular Tint"), 0.0f);
    p.roughness = socket_float(find_input(*node, "Roughness"), 0.2f);
    p.anisotropic = socket_float(find_input(*node, "Anisotropic"), 0.0f);
    p.sheen = socket_float(find_input(*node, "Sheen"), 0.0f);
    p.sheentint = socket_float(find_input(*node, "Sheen Tint"), 0.0f);
    p.clearcoat = socket_float(find_input(*node, "Clearcoat"), 0.0f);
    p.clearcoatgloss = socket_float(find_input(*node, "Clearcoat Gloss"), 1.0f);
    const bNode *tex = follow_input_node(base_sock);
    if (tex && tex->type_legacy == SH_NODE_TEX_IMAGE) {
      char path[FILE_MAX];
      if (extract_image_path(tex, path)) {
        p.tex_name = sanitize_name(tex->name, "tex", tex);
        session.add_imagemap(p.tex_name, path, 2.2f);
      }
    }
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxMatte")) {
    p.type = "matte";
    p.kd = socket_color(find_input_any(*node, {"Diffuse Color", "Color"}), p.kd);
    p.sigma = socket_float(find_input(*node, "Sigma"), 0.0f);
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxGlass")) {
    fill_glass(p, *node, nullptr);
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxRoughGlass")) {
    fill_glass(p, *node, "roughglass");
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxArchGlass")) {
    fill_glass(p, *node, "archglass");
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxMetal")) {
    p.type = "metal2";
    p.kd = socket_color(find_input(*node, "Color"), float3(0.7f, 0.7f, 0.7f));
    p.n = socket_color(find_input(*node, "N"), p.n);
    p.k = socket_color(find_input(*node, "K"), p.k);
    p.roughness = socket_float(find_input(*node, "Roughness"), 0.05f);
    p.use_n_k = socket_is_linked(find_input(*node, "N")) || socket_is_linked(find_input(*node, "K"));
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxMirror")) {
    p.type = "mirror";
    p.kr = socket_color(find_input_any(*node, {"Reflection Color", "Reflection"}),
                        float3(1.0f, 1.0f, 1.0f));
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxGlossy")) {
    p.type = "glossy2";
    p.kd = socket_color(find_input_any(*node, {"Diffuse Color", "Diffuse"}), p.kd);
    p.ks = socket_color(find_input_any(*node, {"Specular Color", "Specular"}),
                        float3(0.05f, 0.05f, 0.05f));
    p.ior = socket_float(find_input(*node, "IOR"), 1.5f);
    p.use_ior = socket_is_linked(find_input(*node, "IOR"));
    p.ka = socket_color(find_input(*node, "Absorption Color"), float3(0.0f, 0.0f, 0.0f));
    p.absorption_depth = socket_float(find_input(*node, "Absorption Depth (nm)"), 0.0f);
    p.roughness = socket_float(find_input(*node, "Roughness"), 0.05f);
    p.multibounce = socket_float(find_input(*node, "Multibounce"), 0.0f) > 0.5f;
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxGlossyTranslucent")) {
    p.type = "glossytranslucent";
    p.kd = socket_color(find_input_any(*node, {"Diffuse Color", "Diffuse"}),
                        float3(0.5f, 0.5f, 0.5f));
    p.kt = socket_color(find_input_any(*node, {"Transmission Color", "Transmission"}),
                        float3(0.5f, 0.5f, 0.5f));
    p.ks = socket_color(find_input(*node, "Specular Color"), float3(0.05f, 0.05f, 0.05f));
    p.ks_bf = socket_color(find_input(*node, "BF Specular Color"), p.ks);
    p.ior = socket_float(find_input(*node, "IOR"), 1.5f);
    p.ior_bf = socket_float(find_input(*node, "BF IOR"), p.ior);
    p.use_ior = socket_is_linked(find_input(*node, "IOR"));
    p.ka = socket_color(find_input(*node, "Absorption Color"), float3(0.0f, 0.0f, 0.0f));
    p.ka_bf = socket_color(find_input(*node, "BF Absorption Color"), p.ka);
    p.absorption_depth = socket_float(find_input(*node, "Absorption Depth (nm)"), 0.0f);
    p.absorption_depth_bf = socket_float(find_input(*node, "BF Absorption Depth (nm)"),
                                         p.absorption_depth);
    p.roughness = socket_float(find_input(*node, "Roughness"), 0.05f);
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxMatteTranslucent")) {
    p.type = "mattetranslucent";
    p.kr = socket_color(find_input_any(*node, {"Reflection Color", "Reflection"}),
                        float3(0.5f, 0.5f, 0.5f));
    p.kt = socket_color(find_input_any(*node, {"Transmission Color", "Transmission"}),
                        float3(0.5f, 0.5f, 0.5f));
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxVelvet")) {
    p.type = "velvet";
    p.kd = socket_color(find_input_any(*node, {"Diffuse Color", "Color"}),
                        float3(1.0f, 1.0f, 1.0f));
    p.thickness = socket_float(find_input(*node, "Thickness"), 0.1f);
    p.p1 = socket_float(find_input(*node, "p1"), 2.0f);
    p.p2 = socket_float(find_input(*node, "p2"), 10.0f);
    p.p3 = socket_float(find_input(*node, "p3"), 2.0f);
    p.velvet_advanced = true;
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxCarpaint")) {
    p.type = "carpaint";
    p.kd = socket_color(find_input_any(*node, {"Diffuse Color", "Color"}),
                        float3(0.3f, 0.3f, 0.3f));
    p.ks1 = socket_color(find_input(*node, "Specular Color 1"), float3(1.0f, 1.0f, 1.0f));
    p.ks2 = socket_color(find_input(*node, "Specular Color 2"), float3(1.0f, 1.0f, 1.0f));
    p.ks3 = socket_color(find_input(*node, "Specular Color 3"), float3(1.0f, 1.0f, 1.0f));
    p.r1 = socket_float(find_input(*node, "R1"), 0.95f);
    p.r2 = socket_float(find_input(*node, "R2"), 0.9f);
    p.r3 = socket_float(find_input(*node, "R3"), 0.7f);
    p.m1 = socket_float(find_input(*node, "M1"), 0.25f);
    p.m2 = socket_float(find_input(*node, "M2"), 0.1f);
    p.m3 = socket_float(find_input(*node, "M3"), 0.015f);
    p.ka = socket_color(find_input(*node, "Absorption Color"), float3(0.0f, 0.0f, 0.0f));
    p.absorption_depth = socket_float(find_input(*node, "Absorption Depth (nm)"), 0.0f);
    p.carpaint_manual = true;
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxCloth")) {
    p.type = "cloth";
    p.warp_kd = socket_color(find_input_any(*node, {"Wrap Diffuse Color", "Color"}),
                             float3(0.7f, 0.05f, 0.05f));
    p.warp_ks = socket_color(find_input(*node, "Wrap Specular Color"),
                             float3(0.04f, 0.04f, 0.04f));
    p.weft_kd = socket_color(find_input_any(*node, {"Weft Diffuse Color", "Weft"}),
                             float3(0.64f, 0.64f, 0.64f));
    p.weft_ks = socket_color(find_input(*node, "Weft Specular Color"),
                             float3(0.04f, 0.04f, 0.04f));
    p.repeat_u = socket_float(find_input(*node, "Repeat U"), 100.0f);
    p.repeat_v = socket_float(find_input(*node, "Repeat V"), 100.0f);
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxNull")) {
    p.type = "null";
    p.kt = socket_color(find_input(*node, "Transmission Color"), float3(1.0f, 1.0f, 1.0f));
    session.add_material(p);
    return name;
  }
  if (STREQ(node->idname, "ShaderNodeLuxEmission")) {
    p.type = "matte";
    p.kd = float3(0.0f, 0.0f, 0.0f);
    const float3 col = socket_color(find_input(*node, "Color"), float3(1.0f, 1.0f, 1.0f));
    const float gain = socket_float(find_input(*node, "Gain"), 1.0f);
    const float exposure = socket_float(find_input(*node, "Exposure"), 0.0f);
    p.emission = col * (gain * powf(2.0f, exposure));
    session.add_material(p);
    return name;
  }

  if (node->type_legacy == SH_NODE_BSDF_GLASS) {
    p.type = "glass";
    p.kt = socket_color(find_input(*node, "Color"), float3(1.0f, 1.0f, 1.0f));
    p.ior = socket_float(find_input(*node, "IOR"), 1.45f);
    p.roughness = socket_float(find_input(*node, "Roughness"), 0.0f);
    if (p.roughness > 1.0e-4f) {
      p.type = "roughglass";
    }
    session.add_material(p);
    return name;
  }
  if (node->type_legacy == SH_NODE_BSDF_DIFFUSE) {
    p.type = "matte";
    p.kd = socket_color(find_input(*node, "Color"), p.kd);
    session.add_material(p);
    return name;
  }
  if (node->type_legacy == SH_NODE_EMISSION) {
    p.type = "matte";
    p.kd = float3(0.0f, 0.0f, 0.0f);
    p.emission = socket_color(find_input(*node, "Color"), float3(1.0f, 1.0f, 1.0f));
    p.emission *= socket_float(find_input(*node, "Strength"), 1.0f);
    session.add_material(p);
    return name;
  }
  if (node->type_legacy == SH_NODE_BSDF_PRINCIPLED) {
    const bNodeSocket *base_sock = find_input(*node, "Base Color");
    p.kd = socket_color(base_sock, p.kd);
    p.metallic = socket_float(find_input(*node, "Metallic"), 0.0f);
    p.roughness = socket_float(find_input(*node, "Roughness"), 0.5f);
    p.specular = 0.5f;
    p.ior = socket_float(find_input(*node, "IOR"), 1.5f);
    const float transmission = socket_float(find_input(*node, "Transmission Weight"), 0.0f);
    const bNodeSocket *emit_str = find_input(*node, "Emission Strength");
    const float strength = socket_float(emit_str, 0.0f);
    if (strength > 0.0f) {
      p.emission = socket_color(find_input(*node, "Emission Color"), float3(1.0f, 1.0f, 1.0f)) *
                   strength;
    }
    const bNode *tex = follow_input_node(base_sock);
    if (tex && tex->type_legacy == SH_NODE_TEX_IMAGE) {
      char path[FILE_MAX];
      if (extract_image_path(tex, path)) {
        p.tex_name = sanitize_name(tex->name, "tex", tex);
        session.add_imagemap(p.tex_name, path, 2.2f);
      }
    }
    if (transmission > 0.5f) {
      p.type = "glass";
      p.kt = p.kd;
      p.cauchyb = 0.0f;
    }
    else {
      p.type = "disney";
    }
    session.add_material(p);
    return name;
  }

  p.type = "disney";
  session.add_material(p);
  return name;
}

static const bNode *find_lux_output(const bNodeTree *ntree)
{
  if (ntree == nullptr) {
    return nullptr;
  }
  const bNode *output = nullptr;
  for (const bNode &node : ntree->nodes) {
    if (STREQ(node.idname, "ShaderNodeLuxOutput") && (node.flag & NODE_DO_OUTPUT)) {
      return &node;
    }
  }
  for (const bNode &node : ntree->nodes) {
    if (STREQ(node.idname, "ShaderNodeLuxOutput") ||
        (node.type_legacy == SH_NODE_OUTPUT_MATERIAL && (node.flag & NODE_DO_OUTPUT)))
    {
      output = &node;
      break;
    }
  }
  if (output) {
    return output;
  }
  for (const bNode &node : ntree->nodes) {
    if (node.type_legacy == SH_NODE_OUTPUT_MATERIAL) {
      return &node;
    }
  }
  return nullptr;
}

static MaterialExport export_and_define_material(Session &session,
                                                 const Material *ma,
                                                 const void *key)
{
  MaterialExport out;
  out.name = sanitize_name(ma ? (ma->id.name + 2) : "default", "mat", key);
  if (ma == nullptr) {
    session.add_matte(out.name, float3(0.8f, 0.8f, 0.8f), float3(0.0f, 0.0f, 0.0f));
    return out;
  }

  const bNodeTree *ntree = ma->luxcore_nodetree ? ma->luxcore_nodetree : ma->nodetree;
  const bNode *output = find_lux_output(ntree);
  if (output == nullptr) {
    session.add_disney(out.name,
                       float3(ma->r, ma->g, ma->b),
                       0.0f,
                       0.5f,
                       0.5f,
                       1.5f,
                       "");
    return out;
  }

  Map<const bNode *, std::string> cache;
  const bNode *shader = follow_input_node(
      find_input_any(*output, {"Surface", "Material"}));
  const std::string shader_name = export_shader_node(session, shader, cache);

  const std::string interior = export_volume_node(
      session, follow_input_node(find_input(*output, "Interior Volume")), cache);
  const std::string exterior = export_volume_node(
      session, follow_input_node(find_input(*output, "Exterior Volume")), cache);
  if (!interior.empty() || !exterior.empty()) {
    session.set_material_volumes(shader_name, interior, exterior);
  }

  out.name = shader_name;
  return out;
}

static void sync_mesh_object(Session &session, Object *ob)
{
  Mesh *mesh = BKE_object_get_evaluated_mesh(ob);
  if (mesh == nullptr) {
    return;
  }

  const Span<float3> positions = mesh->vert_positions();
  const Span<int> corner_verts = mesh->corner_verts();
  const Span<int3> corner_tris = mesh->corner_tris();
  if (positions.is_empty() || corner_tris.is_empty()) {
    return;
  }

  const bke::AttributeAccessor attributes = mesh->attributes();
  const VArray<int> material_indices = *attributes.lookup_or_default<int>(
      "material_index", bke::AttrDomain::Face, 0);
  const Span<int> tri_faces = mesh->corner_tri_faces();

  VArray<float2> uv_varray;
  const VectorSet<StringRefNull> uv_names = mesh->uv_map_names();
  if (!uv_names.is_empty()) {
    if (bke::AttributeReader<float2> uv_attr = attributes.lookup<float2>(
            uv_names[0], bke::AttrDomain::Corner))
    {
      uv_varray = *uv_attr;
    }
  }

  const float4x4 tfm = ob->object_to_world();
  const bool flip_winding = math::is_negative(tfm);
  /* Bake world positions with Blender's transform_point (column vectors).
   * Passing local verts + a 16-float LuxCore matrix is easy to transpose and
   * shows up as a camera-X mirror against the gizmos. */

  const int mat_count = max_ii(1, BKE_object_material_count_eval(ob));
  Vector<MaterialExport> materials(mat_count);
  for (int i = 0; i < mat_count; i++) {
    Material *ma = BKE_object_material_get_eval(ob, short(i + 1));
    materials[i] = export_and_define_material(
        session, ma, ma ? static_cast<const void *>(ma) : static_cast<const void *>(ob));
  }

  /* One LuxCore vertex per face corner so shade-smooth / shade-flat / sharp
   * edges follow Blender's corner normals. Sharing by vertex index and passing
   * n=nullptr made every mesh faceted. */
  const Span<float3> corner_normals = mesh->corner_normals();
  const float3x3 ntfm = float3x3(tfm);

  Vector<Vector<int>> tris_per_mat(mat_count);
  Vector<Vector<float3>> verts_per_mat(mat_count);
  Vector<Vector<float3>> normals_per_mat(mat_count);
  Vector<Vector<float2>> uvs_per_mat(mat_count);

  for (const int t : corner_tris.index_range()) {
    int mat_i = 0;
    if (!tri_faces.is_empty() && !material_indices.is_empty()) {
      mat_i = material_indices[tri_faces[t]];
    }
    mat_i = clamp_i(mat_i, 0, mat_count - 1);
    auto &verts = verts_per_mat[mat_i];
    auto &norms = normals_per_mat[mat_i];
    auto &uvs = uvs_per_mat[mat_i];
    auto &tris = tris_per_mat[mat_i];
    int idx[3];
    for (int k = 0; k < 3; k++) {
      const int corner = corner_tris[t][k];
      const int v = corner_verts[corner];
      idx[k] = int(verts.size());
      verts.append(math::transform_point(tfm, positions[v]));
      if (!corner_normals.is_empty()) {
        norms.append(corner_normals[corner]);
      }
      if (!uv_varray.is_empty()) {
        uvs.append(uv_varray[corner]);
      }
    }
    if (flip_winding) {
      std::swap(idx[0], idx[1]);
    }
    tris.append(idx[0]);
    tris.append(idx[1]);
    tris.append(idx[2]);
  }

  for (int m = 0; m < mat_count; m++) {
    if (tris_per_mat[m].is_empty()) {
      continue;
    }
    if (!normals_per_mat[m].is_empty()) {
      math::transform_normals(ntfm, normals_per_mat[m].as_mutable_span());
      if (flip_winding) {
        for (float3 &n : normals_per_mat[m]) {
          n = -n;
        }
      }
    }
    const std::string mesh_name = sanitize_name(ob->id.name + 2, "me", ob) + "_" +
                                  std::to_string(m);
    const std::string obj_name = sanitize_name(ob->id.name + 2, "ob", ob) + "_" +
                                 std::to_string(m);
    session.add_mesh(mesh_name,
                     verts_per_mat[m],
                     tris_per_mat[m],
                     uvs_per_mat[m],
                     normals_per_mat[m]);
    session.add_object(obj_name, mesh_name, materials[m].name, nullptr);
  }
}

static void sync_light(Session &session, Object *ob)
{
  Light *la = reinterpret_cast<Light *>(ob->data);
  if (la == nullptr) {
    return;
  }
  const float4x4 &tfm = ob->object_to_world();
  const float3 pos = tfm.location();
  /* Blender lamps shine along local -Z. Spot/area target that point.
   * LuxCore sun.dir is the vector toward the sun (BlendLuxCore
   * _calc_sun_dir = +Z of matrix_world), otherwise the sun sits under
   * the floor and the image is black. */
  const float3 shine = math::normalize(-tfm.z_axis());
  const float3 sun_dir = math::normalize(tfm.z_axis());
  const float3 target = pos + shine;
  float3 color = float3(la->r, la->g, la->b) * la->energy;
  const std::string name = sanitize_name(ob->id.name + 2, "la", ob);

  switch (la->type) {
    case LA_SUN:
      /* LuxCore sun SPD is in physical solar units. Official .scn files use
       * gain ~2.5e-5; passing Blender energy 1.0 as gain 1.0 overexposes the
       * floor so PhotonGI caustics are invisible. Do not add sky2 here — it
       * is already in display-referred units and would wash the caustic out. */
      session.add_sun(name, sun_dir, color * 0.000025f, 2.2f);
      break;
    case LA_SPOT: {
      session.add_spot_light(name, pos, target, color, RAD2DEGF(la->spotsize));
      break;
    }
    case LA_AREA:
      session.add_area_light(name, pos, target, tfm.y_axis(), color, la->area_size, la->area_sizey);
      break;
    case LA_LOCAL:
    default:
      session.add_point_light(name, pos, color, la->radius);
      break;
  }
}

static void sync_camera(Session &session, Object *camera_ob, const Scene *scene, int winx, int winy)
{
  CameraDesc cam;
  lux_lookat_from_matrix(camera_ob->object_to_world(), cam);

  const Camera *bcam = (camera_ob->type == OB_CAMERA) ?
                           reinterpret_cast<const Camera *>(camera_ob->data) :
                           nullptr;
  const float aspx = (scene && scene->r.xasp > 1e-8f) ? scene->r.xasp : 1.0f;
  const float aspy = (scene && scene->r.yasp > 1e-8f) ? scene->r.yasp : 1.0f;
  const float aw = float(max_ii(winx, 1)) * aspx;
  const float ah = float(max_ii(winy, 1)) * aspy;
  int sensor_fit = bcam ? bcam->sensor_fit : CAMERA_SENSOR_FIT_AUTO;
  const bool hor = BKE_camera_sensor_fit(sensor_fit, aw, ah) == CAMERA_SENSOR_FIT_HOR;
  float xa = 1.0f;
  float ya = 1.0f;
  lux_calc_aspect(aw, ah, xa, ya, hor);

  float zoom = 1.0f;
  float scale = 1.0f;
  float shift_x = 0.0f;
  float shift_y = 0.0f;
  cam.is_ortho = false;
  if (bcam) {
    shift_x = bcam->shiftx;
    shift_y = bcam->shifty;
    cam.clip_start = math::max(bcam->clip_start, 1e-8f);
    cam.clip_end = bcam->clip_end;
    if (bcam->type == CAM_ORTHO) {
      cam.is_ortho = true;
      const float ortho = math::max(bcam->ortho_scale, 1e-8f);
      zoom = 0.5f * ortho;
      scale = 0.5f * ortho;
    }
    else {
      const float sensor = BKE_camera_sensor_size(bcam->sensor_fit, bcam->sensor_x, bcam->sensor_y);
      cam.fov_deg = RAD2DEGF(2.0f * atanf(sensor / (2.0f * math::max(bcam->lens, 1e-8f))));
    }
  }
  lux_calc_screenwindow(zoom, shift_x, shift_y, 0.0f, 0.0f, xa, ya, scale, cam.screenwindow);
  session.set_camera(cam);
  printf("LuxCore: F12 cam orig=(%.4f %.4f %.4f) tgt=(%.4f %.4f %.4f) up=(%.4f %.4f %.4f) "
         "fov=%.3f ortho=%d sw=[%.4f %.4f %.4f %.4f]\n",
         cam.orig.x,
         cam.orig.y,
         cam.orig.z,
         cam.target.x,
         cam.target.y,
         cam.target.z,
         cam.up.x,
         cam.up.y,
         cam.up.z,
         cam.fov_deg,
         int(cam.is_ortho),
         cam.screenwindow[0],
         cam.screenwindow[1],
         cam.screenwindow[2],
         cam.screenwindow[3]);
}

static void sync_studio_world(Session &session, const SyncOptions &opt)
{
  if (!opt.studio_light.empty()) {
    if (const std::optional<std::string> dir = BKE_appdir_folder_id(
            BLENDER_DATAFILES, "studiolights/world"))
    {
      char path[FILE_MAX];
      BLI_path_join(path, sizeof(path), dir->c_str(), opt.studio_light.c_str());
      if (BLI_exists(path)) {
        session.add_infinite_map(
            "studio_infinite", path, math::max(opt.studiolight_intensity, 0.0f), opt.studiolight_rot_z);
        return;
      }
    }
  }
  const float g = math::max(opt.studiolight_intensity, 0.0f);
  session.add_infinite("studio_infinite", float3(1.0f, 1.0f, 1.0f), g);
}

static void sync_world(Session &session,
                       const Scene *scene,
                       bool have_sun,
                       const SyncOptions &opt)
{
  if (!opt.use_scene_world) {
    sync_studio_world(session, opt);
    return;
  }
  World *world = scene->world;
  float3 color = float3(0.05f, 0.05f, 0.05f);
  if (world) {
    color = float3(world->horr, world->horg, world->horb);
    if (world->nodetree) {
      for (const bNode &node : world->nodetree->nodes) {
        if (STREQ(node.idname, "ShaderNodeOutputWorld") && (node.flag & NODE_DO_OUTPUT)) {
          const bNodeSocket *surf = find_input(node, "Surface");
          const bNode *bg = follow_input_node(surf);
          if (bg && bg->type_legacy == SH_NODE_BACKGROUND) {
            color = socket_color(find_input(*bg, "Color"), color);
            color *= socket_float(find_input(*bg, "Strength"), 1.0f);
          }
        }
        else if (node.type_legacy == SH_NODE_BACKGROUND) {
          color = socket_color(find_input(node, "Color"), color);
          color *= socket_float(find_input(node, "Strength"), 1.0f);
        }
      }
    }
  }
  /* Keep a dim world even when a sun exists so glass has something to
   * refract besides a blown-out disc. Skip a true-black background. */
  if (math::length_squared(color) > 1e-8f) {
    session.add_infinite("world_infinite", color, 1.0f);
  }
  else if (!have_sun) {
    session.add_infinite("world_infinite", float3(0.05f, 0.05f, 0.05f), 1.0f);
  }
  (void)have_sun;
}

bool sync_scene(Session &session,
                RenderEngine *engine,
                Depsgraph *depsgraph,
                std::string &r_error,
                const SyncOptions &options)
{
  Scene *scene = DEG_get_evaluated_scene(depsgraph);
  if (scene == nullptr) {
    r_error = "LuxCore: no scene";
    return false;
  }

  const int winx = engine ? max_ii(engine->resolution_x, 1) : 1920;
  const int winy = engine ? max_ii(engine->resolution_y, 1) : 1080;

  Object *camera_ob = engine && engine->camera_override ?
                          DEG_get_evaluated(depsgraph, engine->camera_override) :
                          scene->camera;
  if (camera_ob) {
    sync_camera(session, camera_ob, scene, winx, winy);
  }
  else if (options.require_camera) {
    r_error = "LuxCore: no camera";
    return false;
  }

  bool have_sun = false;
  bool have_mesh = false;

  DEGObjectIterSettings deg_iter_settings{};
  deg_iter_settings.depsgraph = depsgraph;
  deg_iter_settings.flags = DEG_OBJECT_ITER_FOR_RENDER_ENGINE_FLAGS;
  DEG_OBJECT_ITER_BEGIN (&deg_iter_settings, ob) {
    if (ob->type == OB_LAMP) {
      if (!options.use_scene_lights) {
        continue;
      }
      Light *la = reinterpret_cast<Light *>(ob->data);
      if (la && la->type == LA_SUN) {
        have_sun = true;
      }
      sync_light(session, ob);
    }
    else if (ELEM(ob->type, OB_MESH, OB_CURVES_LEGACY, OB_SURF, OB_FONT, OB_MBALL)) {
      sync_mesh_object(session, ob);
      have_mesh = true;
    }
  }
  DEG_OBJECT_ITER_END;

  sync_world(session, scene, have_sun && options.use_scene_lights, options);

  (void)have_mesh;
  return true;
}

}  // namespace blender::render::luxcore
