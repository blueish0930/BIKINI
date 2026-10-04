/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup shdnodes
 *
 * Parallax Occlusion Mapping — offset UVs from a height map (EEVEE + Cycles).
 */

#include "node_shader_util.hh"
#include "node_util.hh"

#include "BKE_image.hh"
#include "BKE_lib_id.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "DNA_customdata_types.h"
#include "DNA_node_types.h"

#include "BLT_translation.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "BKE_scene.hh"
#include "NOD_node_extra_info.hh"

namespace blender {

namespace nodes::node_shader_parallax_occlusion_cc {

NODE_STORAGE_FUNCS(NodeShaderParallax)

static Image *parallax_image_from_socket(const bNode &node)
{
  const bNodeSocket *sock = bke::node_find_socket(node, SOCK_IN, "Image"_ustr);
  if (!sock) {
    return nullptr;
  }
  const auto *val = sock->default_value_typed<bNodeSocketValueImage>();
  return val ? val->value : nullptr;
}

static Image *parallax_get_image(const bNode &node)
{
  const bNodeSocket *sock = bke::node_find_socket(node, SOCK_IN, "Image"_ustr);
  if (sock && !sock->link) {
    if (Image *ima = parallax_image_from_socket(node)) {
      return ima;
    }
  }
  return id_cast<Image *>(node.id);
}

static void parallax_sync_id_from_socket(bNode &node)
{
  bNodeSocket *sock = bke::node_find_socket(node, SOCK_IN, "Image"_ustr);
  if (!sock || sock->link) {
    return;
  }
  auto *val = sock->default_value_typed<bNodeSocketValueImage>();
  if (!val) {
    return;
  }
  Image *sock_ima = val->value;
  Image *id_ima = id_cast<Image *>(node.id);
  if (sock_ima == id_ima) {
    return;
  }
  if (sock_ima) {
    id_us_min(node.id);
    node.id = &sock_ima->id;
    id_us_plus(node.id);
  }
  else if (id_ima) {
    val->value = id_ima;
    id_us_plus(&id_ima->id);
  }
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Image>("Image"_ustr)
      .optional_label()
      .description("Height map. White is high (towards the camera); Invert treats it as depth");

  b.add_input<decl::Vector>("Vector"_ustr)
      .hide_value()
      .description("Texture coordinates. Unconnected: UV Map");

  b.add_input<decl::Float>("Scale"_ustr)
      .default_value(0.05f)
      .min(0.0f)
      .max(10.0f)
      .description("UV-space height of a 0–1 range. Typical 0.01–0.1");

  b.add_input<decl::Float>("Midlevel"_ustr)
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .description(
          "Height of the geometric surface in the map. Values below Midlevel inset; "
          "values above cannot extrude and are treated as the surface");

  auto &samples = b.add_input<decl::Float>("Samples"_ustr)
                      .default_value(16.0f)
                      .min(4.0f)
                      .max(32.0f)
                      .description(
                          "Ray-march steps. More steps, fewer artifacts, higher cost. "
                          "Keep this low in Cycles (path tracing evaluates the shader many times)")
                      .make_available([](bNode &node) {
                        auto &storage = node_storage(node);
                        if (storage.mode == NODE_PARALLAX_OFFSET) {
                          storage.mode = NODE_PARALLAX_POM;
                        }
                      });

  auto &refine = b.add_input<decl::Float>("Refine"_ustr)
                     .default_value(2.0f)
                     .min(0.0f)
                     .max(8.0f)
                     .description("Binary-search refinement steps after the coarse march")
                     .make_available([](bNode &node) {
                       auto &storage = node_storage(node);
                       if (storage.mode < NODE_PARALLAX_POM) {
                         storage.mode = NODE_PARALLAX_POM;
                       }
                     });

  b.add_input<decl::Vector>("Normal"_ustr)
      .hide_value()
      .description("World-space shading normal. Unconnected: Geometry Normal");

  b.add_input<decl::Vector>("Incoming"_ustr)
      .hide_value()
      .description("World-space view vector towards the camera. Unconnected: Geometry Incoming");

  auto &light = b.add_input<decl::Vector>("Light"_ustr)
                    .hide_value()
                    .description(
                        "World-space direction towards the light, for self-shadowing. "
                        "Unconnected: no self-shadow")
                    .make_available([](bNode &node) {
                      node_storage(node).mode = NODE_PARALLAX_POM_SHADOW;
                    });

  b.add_output<decl::Vector>("Vector"_ustr)
      .description("Offset texture coordinates. Plug into Image Texture Vector");
  b.add_output<decl::Float>("Height"_ustr)
      .description("Height sampled at the hit UV");
  b.add_output<decl::Float>("Shadow"_ustr)
      .description("Self-shadow factor (1 = lit). 1 when Light is unused");
  b.add_output<decl::Float>("Alpha"_ustr)
      .description("Always 1 for Parallax Occlusion");

  const bNode *node = b.node_or_null();
  const int mode = (node && node->storage) ? int(node_storage(*node).mode) : int(NODE_PARALLAX_POM);
  samples.available(mode != NODE_PARALLAX_OFFSET);
  refine.available(mode == NODE_PARALLAX_POM || mode == NODE_PARALLAX_POM_SHADOW);
  light.available(mode == NODE_PARALLAX_POM_SHADOW);
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "mode", ui::ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
  layout.prop(ptr, "invert", ui::ITEM_R_SPLIT_EMPTY_NAME, std::nullopt, ICON_NONE);
  layout.prop(ptr, "clip", ui::ITEM_R_SPLIT_EMPTY_NAME, std::nullopt, ICON_NONE);
  layout.prop(ptr, "channel", ui::ITEM_R_SPLIT_EMPTY_NAME, IFACE_("Channel"), ICON_NONE);
  layout.prop(ptr, "interpolation", ui::ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
  layout.prop(ptr, "extension", ui::ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  NodeShaderParallax *storage = MEM_new<NodeShaderParallax>(__func__);
  BKE_imageuser_default(&storage->iuser);
  storage->interpolation = SHD_INTERP_LINEAR;
  storage->extension = SHD_IMAGE_EXTENSION_REPEAT;
  storage->mode = NODE_PARALLAX_POM;
  storage->channel = NODE_PARALLAX_CHANNEL_R;
  storage->flag = 0;
  node->storage = storage;
}

static void node_update(bNodeTree * /*ntree*/, bNode *node)
{
  parallax_sync_id_from_socket(*node);
  NodeShaderParallax &storage = node_storage(*node);
  if (storage.mode == NODE_PARALLAX_SPOM) {
    storage.mode = NODE_PARALLAX_POM;
  }
}

static int node_gpu(GPUMaterial *mat,
                    bNode *node,
                    bNodeExecData * /*execdata*/,
                    GPUNodeStack *in,
                    GPUNodeStack *out)
{
  GPUNodeStack &vector_in = GPU_node_get_input(*node, in, "Vector");
  GPUNodeStack &out_vector = GPU_node_get_output(*node, out, "Vector");
  GPUNodeStack &out_height = GPU_node_get_output(*node, out, "Height");
  GPUNodeStack &out_shadow = GPU_node_get_output(*node, out, "Shadow");
  GPUNodeStack &out_alpha = GPU_node_get_output(*node, out, "Alpha");

  if (!vector_in.link) {
    vector_in.link = GPU_attribute(mat, CD_AUTO_FROM_NAME, "");
    node_shader_gpu_bump_tex_coord(mat, node, &vector_in.link);
  }

  bNode *node_original = node->runtime->original ? node->runtime->original : node;
  Image *ima = parallax_get_image(*node);
  if (!ima) {
    ima = parallax_get_image(*node_original);
  }

  if (!ima) {
    static const float zero = 0.0f;
    static const float one = 1.0f;
    GPU_link(mat, "vector_copy", vector_in.link, &out_vector.link);
    GPU_link(mat, "set_value", GPU_constant(&zero), &out_height.link);
    GPU_link(mat, "set_value", GPU_constant(&one), &out_shadow.link);
    GPU_link(mat, "set_value", GPU_constant(&one), &out_alpha.link);
    return true;
  }

  const NodeShaderParallax &storage = node_storage(*node);
  NodeShaderParallax &storage_original = node_storage(*node_original);
  ImageUser *iuser = &storage_original.iuser;

  GPUSamplerState sampler_state = GPUSamplerState::default_sampler();
  switch (storage.extension) {
    case SHD_IMAGE_EXTENSION_EXTEND:
      sampler_state.extend_x = GPU_SAMPLER_EXTEND_MODE_EXTEND;
      sampler_state.extend_yz = GPU_SAMPLER_EXTEND_MODE_EXTEND;
      break;
    case SHD_IMAGE_EXTENSION_REPEAT:
      sampler_state.extend_x = GPU_SAMPLER_EXTEND_MODE_REPEAT;
      sampler_state.extend_yz = GPU_SAMPLER_EXTEND_MODE_REPEAT;
      break;
    case SHD_IMAGE_EXTENSION_CLIP:
      sampler_state.extend_x = GPU_SAMPLER_EXTEND_MODE_CLAMP_TO_BORDER;
      sampler_state.extend_yz = GPU_SAMPLER_EXTEND_MODE_CLAMP_TO_BORDER;
      break;
    case SHD_IMAGE_EXTENSION_MIRROR:
      sampler_state.extend_x = GPU_SAMPLER_EXTEND_MODE_MIRRORED_REPEAT;
      sampler_state.extend_yz = GPU_SAMPLER_EXTEND_MODE_MIRRORED_REPEAT;
      break;
    default:
      break;
  }
  if (storage.interpolation != SHD_INTERP_CLOSEST) {
    sampler_state.filtering = GPU_SAMPLER_FILTERING_ANISOTROPIC_ENABLE |
                              GPU_SAMPLER_FILTERING_LINEAR | GPU_SAMPLER_FILTERING_MIPMAP;
  }

  GPUNodeLink *gpu_image = GPU_image(mat, ima, iuser, sampler_state);

  float params[4];
  params[0] = float(storage.mode);
  params[1] = (storage.flag & NODE_PARALLAX_INVERT) ? 1.0f : 0.0f;
  params[2] = (storage.flag & NODE_PARALLAX_CLIP) ? 1.0f : 0.0f;
  params[3] = float(storage.channel);

  return GPU_stack_link(mat,
                        node,
                        "node_parallax_occlusion",
                        in,
                        out,
                        gpu_image,
                        GPU_constant(params),
                        GPU_kernel_globals(),
                        GPU_shading_data());
}

NODE_SHADER_MATERIALX_BEGIN
#ifdef WITH_MATERIALX
{
  return get_output_default(socket_out_->identifier, NodeItem::Type::Any);
}
#endif
NODE_SHADER_MATERIALX_END

}  // namespace nodes::node_shader_parallax_occlusion_cc

void register_node_type_sh_parallax_occlusion()
{
  namespace file_ns = nodes::node_shader_parallax_occlusion_cc;

  static bke::bNodeType ntype;

  sh_node_type_base(&ntype, "ShaderNodeParallaxOcclusion"_ustr, SH_NODE_PARALLAX_OCCLUSION);
  ntype.ui_name = "Parallax Occlusion";
  ntype.ui_description =
      "Offset UVs from a height map using parallax occlusion mapping. "
      "Works in EEVEE and Cycles";
  ntype.enum_name_legacy = "PARALLAX_OCCLUSION";
  ntype.nclass = NODE_CLASS_OP_VECTOR;
  ntype.declare = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  ntype.gpu_fn = file_ns::node_gpu;
  ntype.materialx_fn = file_ns::node_shader_materialx;
  ntype.labelfunc = node_image_label;
  ntype.add_ui_poll = object_shader_nodes_poll;
  bke::node_type_storage(
      ntype, "NodeShaderParallax", node_free_standard_storage, node_copy_standard_storage);

  bke::node_register_type(ntype);
}

}  // namespace blender
