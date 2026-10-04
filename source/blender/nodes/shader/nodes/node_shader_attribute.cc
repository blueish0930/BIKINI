/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup shdnodes
 */

#include "node_shader_util.hh"

#include "node_util.hh"

#include "BLI_string.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "BLI_ghash.hh"

#include "RNA_access.hh"

namespace blender {

namespace nodes::node_shader_attribute_cc {

static const char *attribute_name_from_node(const bNode &node)
{
  if (const bNodeSocket *sock = bke::node_find_socket(node, SOCK_IN, "Name"_ustr)) {
    if (sock->default_value) {
      const char *value = sock->default_value_typed<bNodeSocketValueString>()->value;
      if (value && value[0] != '\0') {
        return value;
      }
    }
  }
  if (const auto *attr = static_cast<const NodeShaderAttribute *>(node.storage)) {
    return attr->name;
  }
  return "";
}

static void sync_attribute_name_to_storage(bNode &node)
{
  auto *attr = static_cast<NodeShaderAttribute *>(node.storage);
  if (!attr) {
    return;
  }
  STRNCPY(attr->name, attribute_name_from_node(node));
}

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNodeTree *ntree = b.tree_or_null();
  const bool is_gpu_internal = ntree && (ntree->flag & NTREE_IS_GPU_SHADER_INTERNAL);

  /* Must stay first: light-iteration wiring uses input socket 0 as LightIndex. */
  b.add_input<decl::Int>("LightIndex"_ustr).available(is_gpu_internal);
  b.add_input<decl::String>("Name"_ustr).is_attribute_name();
  b.add_output<decl::Color>("Color"_ustr);
  b.add_output<decl::Vector>("Vector"_ustr);
  b.add_output<decl::Float>("Factor"_ustr, "Fac"_ustr);
  b.add_output<decl::Float>("Alpha"_ustr);
}

static void node_shader_buts_attribute(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "attribute_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_shader_init_attribute(bNodeTree * /*ntree*/, bNode *node)
{
  NodeShaderAttribute *attr = MEM_new<NodeShaderAttribute>("NodeShaderAttribute");
  attr->data_type = SHD_ATTRIBUTE_DATA_LEGACY;
  node->storage = attr;
}

static void node_update(bNodeTree * /*ntree*/, bNode *node)
{
  sync_attribute_name_to_storage(*node);
}

static int node_shader_gpu_attribute(GPUMaterial *mat,
                                     bNode *node,
                                     bNodeExecData * /*execdata*/,
                                     GPUNodeStack *in,
                                     GPUNodeStack *out)
{
  NodeShaderAttribute *attr = static_cast<NodeShaderAttribute *>(node->storage);
  const char *attr_name = attribute_name_from_node(*node);
  sync_attribute_name_to_storage(*node);
  float attr_hash = 0.0f;

  GPUNodeLink *cd_attr;

  switch (attr->type) {
    case SHD_ATTRIBUTE_LIGHT: {
      if (!in[0].link) {
        /* Error: Attribute node is not linked to a light accumulation node. */
        return false;
      }
      else if (STREQ(attr_name, "is_sun")) {
        GPU_link(mat, "node_attribute_light_is_sun", in[0].link, GPU_kernel_globals(), &cd_attr);
      }
      else if (STREQ(attr_name, "is_point")) {
        GPU_link(mat, "node_attribute_light_is_point", in[0].link, GPU_kernel_globals(), &cd_attr);
      }
      else if (STREQ(attr_name, "is_spot")) {
        GPU_link(mat, "node_attribute_light_is_spot", in[0].link, GPU_kernel_globals(), &cd_attr);
      }
      else if (STREQ(attr_name, "is_area")) {
        GPU_link(mat, "node_attribute_light_is_area", in[0].link, GPU_kernel_globals(), &cd_attr);
      }
      else if (STREQ(attr_name, "cutoff_distance")) {
        GPU_link(mat,
                 "node_attribute_light_cutoff_distance",
                 in[0].link,
                 GPU_kernel_globals(),
                 &cd_attr);
      }
      else {
        GPU_material_flag_set(mat, GPU_MATFLAG_LIGHT_ATTRIBUTE);
        const bool use_dupli = false;
        /* Mimic gpu_node_graph_add_uniform_attribute */
        uint hash_code = BLI_ghashutil_strhash_p(attr_name) << 1 | (use_dupli ? 0 : 1);

        attr_hash = *reinterpret_cast<float *>(&hash_code);
        GPU_link(mat,
                 "node_attribute_light",
                 in[0].link,
                 GPU_uniform(&attr_hash),
                 GPU_kernel_globals(),
                 &cd_attr);
      }
      break;
    }
    case SHD_ATTRIBUTE_GEOMETRY: {
      cd_attr = GPU_attribute(mat, CD_AUTO_FROM_NAME, attr_name);

      if (STREQ(attr_name, "color")) {
        GPU_link(mat, "node_attribute_color", GPU_kernel_globals(), cd_attr, &cd_attr);
      }
      else if (STREQ(attr_name, "temperature")) {
        GPU_link(mat, "node_attribute_temperature", GPU_kernel_globals(), cd_attr, &cd_attr);
      }
      else if (STREQ(attr_name, "radiance")) {
        GPU_link(mat, "node_attribute_radiance", GPU_kernel_globals(), cd_attr, &cd_attr);
      }
      break;
    }
    case SHD_ATTRIBUTE_VIEW_LAYER: {
      cd_attr = GPU_layer_attribute(mat, attr_name);
      break;
    }
    case SHD_ATTRIBUTE_OBJECT:
    case SHD_ATTRIBUTE_INSTANCER: {
      cd_attr = GPU_uniform_attribute(mat,
                                      attr_name,
                                      attr->type == SHD_ATTRIBUTE_INSTANCER,
                                      reinterpret_cast<uint32_t *>(&attr_hash));

      GPU_link(mat,
               "node_attribute_uniform",
               cd_attr,
               GPU_constant(&attr_hash),
               GPU_kernel_globals(),
               GPU_shading_data(),
               &cd_attr);
      break;
    }
  }

  GPU_link(mat, "node_attribute", cd_attr, &out[0].link, &out[1].link, &out[2].link, &out[3].link);

  if (attr->type == SHD_ATTRIBUTE_GEOMETRY) {
    for (const auto [i, sock] : node->outputs.enumerate()) {
      node_shader_gpu_bump_tex_coord(mat, node, &out[i].link);
    }
  }

  return 1;
}

NODE_SHADER_MATERIALX_BEGIN
#ifdef WITH_MATERIALX
{
  /* TODO: some outputs expected be implemented within the next iteration
   * (see node-definition `<geompropvalue>`). */
  return get_output_default(socket_out_->identifier, NodeItem::Type::Any);
}
#endif
NODE_SHADER_MATERIALX_END

}  // namespace nodes::node_shader_attribute_cc

/* node type definition */
void register_node_type_sh_attribute()
{
  namespace file_ns = nodes::node_shader_attribute_cc;

  static bke::bNodeType ntype;

  sh_node_type_base(&ntype, "ShaderNodeAttribute"_ustr, SH_NODE_ATTRIBUTE);
  ntype.ui_name = "Attribute";
  ntype.ui_description = "Retrieve attributes attached to objects or geometry";
  ntype.enum_name_legacy = "ATTRIBUTE";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_shader_buts_attribute;
  ntype.initfunc = file_ns::node_shader_init_attribute;
  ntype.updatefunc = file_ns::node_update;
  bke::node_type_storage(
      ntype, "NodeShaderAttribute", node_free_standard_storage, node_copy_standard_storage);
  ntype.gather_link_search_ops = search_link_ops_for_shader_material_lighting_node;
  ntype.gpu_fn = file_ns::node_shader_gpu_attribute;
  ntype.materialx_fn = file_ns::node_shader_materialx;

  bke::node_register_type(ntype);
}

}  // namespace blender
