/* SPDX-FileCopyrightText: 2026 BIKINI Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** Native LuxCore shader nodes in the standard ShaderNodeTree. Not a Python addon. */

#include "nodes.hh"

#include <cstdio>

#include "BKE_context.hh"
#include "BKE_node.hh"

#include "DNA_node_types.h"
#include "DNA_space_types.h"

#include "BLT_translation.hh"

#include "RE_engine.h"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "node_shader_util.hh"

namespace blender {

/**
 * Blender 5.3 `node_type_base()` looks up RNA by idname and skips registration when missing.
 * Native structs live in rna_nodetree.cc. If makesrna is stale, define them at runtime so intern
 * nodes still bind (`STRUCT_PUBLIC_NAMESPACE` is required for `RNA_struct_find`).
 */
static void lux_ensure_shader_node_rna(const char *idname,
                                       const char *ui_name,
                                       const char *ui_desc)
{
  if (RNA_struct_find(idname) != nullptr) {
    return;
  }
  StructRNA *base = RNA_struct_find("ShaderNode");
  if (base == nullptr) {
    fprintf(stderr, "LuxCore: ShaderNode RNA base missing, cannot define %s\n", idname);
    return;
  }
  BlenderRNA *brna = &RNA_blender_rna_get();
  StructRNA *srna = RNA_def_struct_ptr(brna, idname, base);
  RNA_def_struct_flag(srna, STRUCT_PUBLIC_NAMESPACE);
  RNA_def_struct_ui_text(srna, ui_name, ui_desc);
  RNA_def_struct_identifier(brna, srna, "");
  RNA_def_struct_identifier(brna, srna, idname);
}

namespace nodes::node_shader_lux_cc {

static bool lux_node_poll(const bke::bNodeType * /*ntype*/,
                          const bNodeTree *ntree,
                          const char **r_disabled_hint)
{
  if (ntree && STREQ(ntree->idname, "LuxCoreMaterialNodeTree")) {
    return true;
  }
  if (r_disabled_hint) {
    *r_disabled_hint = RPT_("Only available in the LuxCore Node Editor");
  }
  return false;
}

static bool lux_add_ui_poll(const bContext *C)
{
  const SpaceNode *snode = CTX_wm_space_node(C);
  return snode && snode->tree_idname[0] && STREQ(snode->tree_idname, "LuxCoreMaterialNodeTree");
}

static void common_mat(NodeDeclarationBuilder &b)
{
  /* BlendLuxCore LuxCoreNodeMaterial.add_common_inputs: Opacity, Bump, Emission.
   * Bump needs a texture node (constant bump is a no-op); keep Opacity + Emission. */
  b.add_input<decl::Float>("Opacity"_ustr).default_value(1.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Color>("Emission"_ustr).default_value({0.0f, 0.0f, 0.0f, 1.0f});
}

static void declare_disney(NodeDeclarationBuilder &b)
{
  /* LuxCoreNodeMatDisney.init — official sockets. IOR kept for old intern trees. */
  b.add_input<decl::Color>("Base Color"_ustr).default_value({0.7f, 0.7f, 0.7f, 1.0f});
  b.add_input<decl::Float>("Subsurface"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Metallic"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Specular"_ustr).default_value(0.5f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Specular Tint"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Roughness"_ustr).default_value(0.2f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Anisotropic"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Sheen"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Sheen Tint"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Clearcoat"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Clearcoat Gloss"_ustr).default_value(1.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Film Amount"_ustr).default_value(1.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Film Thickness (nm)"_ustr).default_value(0.0f).min(0.0f).max(2000.0f);
  b.add_input<decl::Float>("Film IOR"_ustr).default_value(1.5f).min(1.0f).max(5.0f);
  b.add_input<decl::Float>("IOR"_ustr).default_value(1.5f).min(1.0f).max(3.0f);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_matte(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Diffuse Color"_ustr).default_value({0.7f, 0.7f, 0.7f, 1.0f});
  b.add_input<decl::Float>("Sigma"_ustr).default_value(0.0f).min(0.0f).max(45.0f);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_glass(NodeDeclarationBuilder &b)
{
  /* Official LuxCoreNodeMatGlass: Transmission Color, Reflection Color, IOR,
   * Dispersion, thin film, Roughness, Architectural. */
  b.add_input<decl::Color>("Transmission Color"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Color>("Reflection Color"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("IOR"_ustr).default_value(1.5f).min(1.0f).max(3.0f);
  b.add_input<decl::Float>("Dispersion"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .max(0.04f)
      .subtype(PROP_NONE);
  b.add_input<decl::Float>("Film Thickness (nm)"_ustr).default_value(0.0f).min(0.0f).max(2000.0f);
  b.add_input<decl::Float>("Film IOR"_ustr).default_value(1.5f).min(1.0f).max(5.0f);
  b.add_input<decl::Float>("Roughness"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Architectural"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_metal(NodeDeclarationBuilder &b)
{
  /* Official: Color + Fresnel texture. Intern also keeps measured n/k. */
  b.add_input<decl::Color>("Color"_ustr).default_value({0.7f, 0.7f, 0.7f, 1.0f});
  b.add_input<decl::Color>("N"_ustr).default_value({0.2f, 0.2f, 0.2f, 1.0f});
  b.add_input<decl::Color>("K"_ustr).default_value({3.0f, 3.0f, 3.0f, 1.0f});
  b.add_input<decl::Float>("Roughness"_ustr).default_value(0.05f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_mirror(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Reflection Color"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_glossy(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Diffuse Color"_ustr).default_value({0.7f, 0.7f, 0.7f, 1.0f});
  b.add_input<decl::Color>("Specular Color"_ustr).default_value({0.05f, 0.05f, 0.05f, 1.0f});
  b.add_input<decl::Float>("IOR"_ustr).default_value(1.5f).min(1.0f).max(5.0f);
  b.add_input<decl::Color>("Absorption Color"_ustr).default_value({0.0f, 0.0f, 0.0f, 1.0f});
  b.add_input<decl::Float>("Absorption Depth (nm)"_ustr).default_value(0.0f).min(0.0f).max(1e6f);
  b.add_input<decl::Float>("Roughness"_ustr).default_value(0.05f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Multibounce"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_mix(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Shader>("Material 1"_ustr);
  b.add_input<decl::Shader>("Material 2"_ustr);
  b.add_input<decl::Float>("Mix Factor"_ustr).default_value(0.5f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_null(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Transmission Color"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_velvet(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Diffuse Color"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("Thickness"_ustr).default_value(0.1f).min(0.0f).max(1.0f);
  b.add_input<decl::Float>("p1"_ustr).default_value(2.0f);
  b.add_input<decl::Float>("p2"_ustr).default_value(10.0f);
  b.add_input<decl::Float>("p3"_ustr).default_value(2.0f);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_glossy_translucent(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Diffuse Color"_ustr).default_value({0.5f, 0.5f, 0.5f, 1.0f});
  b.add_input<decl::Color>("Transmission Color"_ustr).default_value({0.5f, 0.5f, 0.5f, 1.0f});
  b.add_input<decl::Color>("Specular Color"_ustr).default_value({0.05f, 0.05f, 0.05f, 1.0f});
  b.add_input<decl::Float>("IOR"_ustr).default_value(1.5f).min(1.0f).max(5.0f);
  b.add_input<decl::Color>("Absorption Color"_ustr).default_value({0.0f, 0.0f, 0.0f, 1.0f});
  b.add_input<decl::Float>("Absorption Depth (nm)"_ustr).default_value(0.0f).min(0.0f).max(1e6f);
  b.add_input<decl::Float>("Roughness"_ustr).default_value(0.05f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Color>("BF Specular Color"_ustr).default_value({0.05f, 0.05f, 0.05f, 1.0f});
  b.add_input<decl::Float>("BF IOR"_ustr).default_value(1.5f).min(1.0f).max(5.0f);
  b.add_input<decl::Color>("BF Absorption Color"_ustr).default_value({0.0f, 0.0f, 0.0f, 1.0f});
  b.add_input<decl::Float>("BF Absorption Depth (nm)"_ustr).default_value(0.0f).min(0.0f).max(1e6f);
  b.add_input<decl::Float>("BF Roughness"_ustr).default_value(0.05f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_matte_translucent(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Reflection Color"_ustr).default_value({0.5f, 0.5f, 0.5f, 1.0f});
  b.add_input<decl::Color>("Transmission Color"_ustr).default_value({0.5f, 0.5f, 0.5f, 1.0f});
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_roughglass(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Transmission Color"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Color>("Reflection Color"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("IOR"_ustr).default_value(1.5f).min(1.0f).max(3.0f);
  b.add_input<decl::Float>("Dispersion"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .max(0.04f)
      .subtype(PROP_NONE);
  b.add_input<decl::Float>("Roughness"_ustr).default_value(0.05f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Film Thickness (nm)"_ustr).default_value(0.0f).min(0.0f).max(2000.0f);
  b.add_input<decl::Float>("Film IOR"_ustr).default_value(1.5f).min(1.0f).max(5.0f);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_archglass(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Transmission Color"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Color>("Reflection Color"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("IOR"_ustr).default_value(1.5f).min(1.0f).max(3.0f);
  b.add_input<decl::Float>("Film Thickness (nm)"_ustr).default_value(0.0f).min(0.0f).max(2000.0f);
  b.add_input<decl::Float>("Film IOR"_ustr).default_value(1.5f).min(1.0f).max(5.0f);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_carpaint(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Diffuse Color"_ustr).default_value({0.3f, 0.3f, 0.3f, 1.0f});
  b.add_input<decl::Color>("Specular Color 1"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("R1"_ustr).default_value(0.95f).min(0.0f).max(1.0f).subtype(PROP_FACTOR);
  b.add_input<decl::Float>("M1"_ustr).default_value(0.25f).min(0.0f).max(1.0f).subtype(PROP_FACTOR);
  b.add_input<decl::Color>("Specular Color 2"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("R2"_ustr).default_value(0.9f).min(0.0f).max(1.0f).subtype(PROP_FACTOR);
  b.add_input<decl::Float>("M2"_ustr).default_value(0.1f).min(0.0f).max(1.0f).subtype(PROP_FACTOR);
  b.add_input<decl::Color>("Specular Color 3"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("R3"_ustr).default_value(0.7f).min(0.0f).max(1.0f).subtype(PROP_FACTOR);
  b.add_input<decl::Float>("M3"_ustr).default_value(0.015f).min(0.0f).max(1.0f).subtype(PROP_FACTOR);
  b.add_input<decl::Color>("Absorption Color"_ustr).default_value({0.0f, 0.0f, 0.0f, 1.0f});
  b.add_input<decl::Float>("Absorption Depth (nm)"_ustr).default_value(0.0f).min(0.0f).max(1e6f);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_cloth(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Wrap Diffuse Color"_ustr).default_value({0.7f, 0.05f, 0.05f, 1.0f});
  b.add_input<decl::Color>("Wrap Specular Color"_ustr).default_value({0.04f, 0.04f, 0.04f, 1.0f});
  b.add_input<decl::Color>("Weft Diffuse Color"_ustr).default_value({0.64f, 0.64f, 0.64f, 1.0f});
  b.add_input<decl::Color>("Weft Specular Color"_ustr).default_value({0.04f, 0.04f, 0.04f, 1.0f});
  b.add_input<decl::Float>("Repeat U"_ustr).default_value(100.0f).min(0.0f).max(10000.0f);
  b.add_input<decl::Float>("Repeat V"_ustr).default_value(100.0f).min(0.0f).max(10000.0f);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_twosided(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Shader>("Front Material"_ustr);
  b.add_input<decl::Shader>("Back Material"_ustr);
  common_mat(b);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_glossy_coating(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Shader>("Base Material"_ustr);
  b.add_input<decl::Color>("Specular Color"_ustr).default_value({0.05f, 0.05f, 0.05f, 1.0f});
  b.add_input<decl::Float>("IOR"_ustr).default_value(1.5f).min(1.0f).max(5.0f);
  b.add_input<decl::Color>("Absorption Color"_ustr).default_value({0.0f, 0.0f, 0.0f, 1.0f});
  b.add_input<decl::Float>("Absorption Depth (nm)"_ustr).default_value(0.0f).min(0.0f).max(1e6f);
  b.add_input<decl::Float>("Roughness"_ustr).default_value(0.05f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Multibounce"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Color>("Emission"_ustr).default_value({0.0f, 0.0f, 0.0f, 1.0f});
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_frontback(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Shader>("Material"_ustr);
  b.add_input<decl::Float>("Front Opacity"_ustr).default_value(1.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Back Opacity"_ustr).default_value(1.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_output<decl::Shader>("BSDF"_ustr);
}

static void declare_emission(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Color"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("Gain"_ustr).default_value(1.0f).min(0.0f).max(1e6f);
  b.add_input<decl::Float>("Exposure"_ustr).default_value(0.0f).min(-10.0f).max(10.0f);
  b.add_output<decl::Color>("Emission"_ustr);
}

static void declare_vol_clear(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Absorption"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("IOR"_ustr).default_value(1.5f).min(1.0f).max(5.0f);
  b.add_input<decl::Color>("Emission"_ustr).default_value({0.0f, 0.0f, 0.0f, 1.0f});
  b.add_input<decl::Float>("Absorption Depth"_ustr).default_value(1.0f).min(0.000001f).max(1e6f);
  b.add_output<decl::Shader>("Volume"_ustr);
}

static void declare_vol_homogeneous(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Absorption"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("IOR"_ustr).default_value(1.5f).min(1.0f).max(5.0f);
  b.add_input<decl::Color>("Emission"_ustr).default_value({0.0f, 0.0f, 0.0f, 1.0f});
  b.add_input<decl::Float>("Absorption Depth"_ustr).default_value(1.0f).min(0.000001f).max(1e6f);
  b.add_input<decl::Color>("Scattering"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("Scattering Scale"_ustr).default_value(1.0f).min(0.0f).max(1e6f);
  b.add_input<decl::Vector>("Asymmetry"_ustr).default_value({0.0f, 0.0f, 0.0f});
  b.add_input<decl::Float>("Multiscattering"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_output<decl::Shader>("Volume"_ustr);
}

static void declare_vol_heterogeneous(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Absorption"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("IOR"_ustr).default_value(1.5f).min(1.0f).max(5.0f);
  b.add_input<decl::Color>("Emission"_ustr).default_value({0.0f, 0.0f, 0.0f, 1.0f});
  b.add_input<decl::Float>("Absorption Depth"_ustr).default_value(1.0f).min(0.000001f).max(1e6f);
  b.add_input<decl::Color>("Scattering"_ustr).default_value({1.0f, 1.0f, 1.0f, 1.0f});
  b.add_input<decl::Float>("Scattering Scale"_ustr).default_value(1.0f).min(0.0f).max(1e6f);
  b.add_input<decl::Vector>("Asymmetry"_ustr).default_value({0.0f, 0.0f, 0.0f});
  b.add_input<decl::Float>("Multiscattering"_ustr).default_value(0.0f).min(0.0f).max(1.0f).subtype(
      PROP_FACTOR);
  b.add_input<decl::Float>("Step Size"_ustr).default_value(0.1f).min(0.0001f).max(10.0f);
  b.add_input<decl::Float>("Max Steps"_ustr).default_value(1024.0f).min(1.0f).max(65536.0f);
  b.add_output<decl::Shader>("Volume"_ustr);
}

static void declare_output(NodeDeclarationBuilder &b)
{
  /* Official: Material, Interior Volume, Exterior Volume, Shape. */
  /* Keep Surface as the intern identifier so existing trees stay linked.
   * Official BlendLuxCore name is Material; export accepts both. */
  b.add_input<decl::Shader>("Surface"_ustr);
  b.add_input<decl::Shader>("Interior Volume"_ustr);
  b.add_input<decl::Shader>("Exterior Volume"_ustr);
}

}  // namespace nodes::node_shader_lux_cc

#define LUX_REG_NODE(fn, idname, uiname, desc, declare_fn, node_class, use_gpu) \
  void fn() \
  { \
    namespace file_ns = nodes::node_shader_lux_cc; \
    static bke::bNodeType ntype; \
    lux_ensure_shader_node_rna(idname, uiname, desc); \
    bke::node_type_base(ntype, idname##_ustr); \
    ntype.ui_name = N_(uiname); \
    ntype.ui_description = N_(desc); \
    ntype.nclass = node_class; \
    ntype.declare = file_ns::declare_fn; \
    ntype.poll = file_ns::lux_node_poll; \
    ntype.add_ui_poll = file_ns::lux_add_ui_poll; \
    /* Never attach EEVEE GPU shaders. node_bsdf_diffuse does not match Lux \
     * sockets; a mixed/wrong tree would crash the viewport silently. */ \
    (void)use_gpu; \
    ntype.gpu_fn = nullptr; \
    bke::node_register_type(ntype); \
  }

namespace {

LUX_REG_NODE(reg_disney,
             "ShaderNodeLuxDisney",
             "Lux Disney",
             "LuxCore Disney principled material",
             declare_disney,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_matte,
             "ShaderNodeLuxMatte",
             "Lux Matte",
             "LuxCore matte",
             declare_matte,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_glass,
             "ShaderNodeLuxGlass",
             "Lux Glass",
             "LuxCore glass",
             declare_glass,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_metal,
             "ShaderNodeLuxMetal",
             "Lux Metal",
             "LuxCore metal2",
             declare_metal,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_mirror,
             "ShaderNodeLuxMirror",
             "Lux Mirror",
             "LuxCore mirror",
             declare_mirror,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_glossy,
             "ShaderNodeLuxGlossy",
             "Lux Glossy",
             "LuxCore glossy2",
             declare_glossy,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_mix,
             "ShaderNodeLuxMix",
             "Lux Mix",
             "LuxCore mix material",
             declare_mix,
             NODE_CLASS_SHADER,
             false)
LUX_REG_NODE(reg_null,
             "ShaderNodeLuxNull",
             "Lux Null",
             "LuxCore transparent null",
             declare_null,
             NODE_CLASS_SHADER,
             false)
LUX_REG_NODE(reg_velvet,
             "ShaderNodeLuxVelvet",
             "Lux Velvet",
             "LuxCore velvet",
             declare_velvet,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_gt,
             "ShaderNodeLuxGlossyTranslucent",
             "Lux Glossy Translucent",
             "LuxCore glossy translucent",
             declare_glossy_translucent,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_mt,
             "ShaderNodeLuxMatteTranslucent",
             "Lux Matte Translucent",
             "LuxCore matte translucent",
             declare_matte_translucent,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_rg,
             "ShaderNodeLuxRoughGlass",
             "Lux Rough Glass",
             "LuxCore rough glass",
             declare_roughglass,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_ag,
             "ShaderNodeLuxArchGlass",
             "Lux Architectural Glass",
             "LuxCore architectural glass",
             declare_archglass,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_cp,
             "ShaderNodeLuxCarpaint",
             "Lux Carpaint",
             "LuxCore carpaint",
             declare_carpaint,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_cloth,
             "ShaderNodeLuxCloth",
             "Lux Cloth",
             "LuxCore cloth",
             declare_cloth,
             NODE_CLASS_SHADER,
             true)
LUX_REG_NODE(reg_ts,
             "ShaderNodeLuxTwoSided",
             "Lux Two Sided",
             "LuxCore two-sided material",
             declare_twosided,
             NODE_CLASS_SHADER,
             false)
LUX_REG_NODE(reg_out,
             "ShaderNodeLuxOutput",
             "Lux Material Output",
             "LuxCore material output",
             declare_output,
             NODE_CLASS_OUTPUT,
             false)
LUX_REG_NODE(reg_gc,
             "ShaderNodeLuxGlossyCoating",
             "Lux Glossy Coating",
             "LuxCore glossy coating",
             declare_glossy_coating,
             NODE_CLASS_SHADER,
             false)
LUX_REG_NODE(reg_fb,
             "ShaderNodeLuxFrontBackOpacity",
             "Lux Front/Back Opacity",
             "LuxCore front/back opacity",
             declare_frontback,
             NODE_CLASS_SHADER,
             false)
LUX_REG_NODE(reg_em,
             "ShaderNodeLuxEmission",
             "Lux Emission",
             "LuxCore mesh light emission",
             declare_emission,
             NODE_CLASS_SHADER,
             false)
LUX_REG_NODE(reg_vclear,
             "ShaderNodeLuxVolumeClear",
             "Lux Clear Volume",
             "LuxCore clear volume",
             declare_vol_clear,
             NODE_CLASS_SHADER,
             false)
LUX_REG_NODE(reg_vhomo,
             "ShaderNodeLuxVolumeHomogeneous",
             "Lux Homogeneous Volume",
             "LuxCore homogeneous volume",
             declare_vol_homogeneous,
             NODE_CLASS_SHADER,
             false)
LUX_REG_NODE(reg_vhet,
             "ShaderNodeLuxVolumeHeterogeneous",
             "Lux Heterogeneous Volume",
             "LuxCore heterogeneous volume",
             declare_vol_heterogeneous,
             NODE_CLASS_SHADER,
             false)

}  // namespace

void LUX_nodes_register()
{
  LUX_node_tree_register();
  reg_disney();
  reg_matte();
  reg_glass();
  reg_metal();
  reg_mirror();
  reg_glossy();
  reg_mix();
  reg_null();
  reg_velvet();
  reg_gt();
  reg_mt();
  reg_rg();
  reg_ag();
  reg_cp();
  reg_cloth();
  reg_ts();
  reg_out();
  reg_gc();
  reg_fb();
  reg_em();
  reg_vclear();
  reg_vhomo();
  reg_vhet();
}

}  // namespace blender
