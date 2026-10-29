/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup shdnodes
 */

#include "node_shader_util.hh"

#include "node_util.hh"

#include "BLI_string.hh"
#include "BLI_shader_matrix_attribute.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "BLI_ghash.hh"

#include "RNA_access.hh"
#include "NOD_socket_search_link.hh"

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
  const bNode *node = b.node_or_null();
  const int data_type = node && node->storage ?
                            static_cast<const NodeShaderAttribute *>(node->storage)->data_type :
                            SHD_ATTRIBUTE_DATA_FLOAT;
  const bool legacy = data_type == SHD_ATTRIBUTE_DATA_LEGACY;

  /* Must stay first: light-iteration wiring uses input socket 0 as LightIndex. */
  b.add_input<decl::Int>("LightIndex"_ustr).available(is_gpu_internal);
  b.add_input<decl::String>("Name"_ustr).is_attribute_name().optional_label();
  /* Preserve identifiers and indices of existing material links. New nodes expose one typed
   * Attribute output, as Named Attribute does in Geometry Nodes. */
  b.add_output<decl::Color>("Color"_ustr).available(legacy);
  b.add_output<decl::Vector>("Vector"_ustr).available(legacy);
  b.add_output<decl::Float>("Factor"_ustr, "Fac"_ustr).available(legacy);
  b.add_output<decl::Float>("Alpha"_ustr).available(legacy);
  b.add_output<decl::Matrix>("Matrix"_ustr)
      .available(legacy)
      .description("4x4 matrix from an ordinary Geometry Nodes matrix attribute");
  switch (data_type) {
    case SHD_ATTRIBUTE_DATA_LEGACY:
      break;
    case SHD_ATTRIBUTE_DATA_INT:
      b.add_output<decl::Int>("Attribute"_ustr);
      break;
    case SHD_ATTRIBUTE_DATA_VECTOR:
      b.add_output<decl::Vector>("Attribute"_ustr);
      break;
    case SHD_ATTRIBUTE_DATA_ROTATION:
      b.add_output<decl::Rotation>("Attribute"_ustr);
      break;
    case SHD_ATTRIBUTE_DATA_MATRIX:
      b.add_output<decl::Matrix>("Attribute"_ustr);
      break;
    case SHD_ATTRIBUTE_DATA_COLOR:
      b.add_output<decl::Color>("Attribute"_ustr);
      break;
    case SHD_ATTRIBUTE_DATA_BOOLEAN:
      b.add_output<decl::Bool>("Attribute"_ustr);
      break;
    default:
      b.add_output<decl::Float>("Attribute"_ustr);
      break;
  }
}

static void node_shader_buts_attribute(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "attribute_type", UI_ITEM_NONE, "", ICON_NONE);
  layout.prop(ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_shader_init_attribute(bNodeTree * /*ntree*/, bNode *node)
{
  NodeShaderAttribute *attr = MEM_new<NodeShaderAttribute>("NodeShaderAttribute");
  attr->data_type = SHD_ATTRIBUTE_DATA_FLOAT;
  node->storage = attr;
}

static void node_update(bNodeTree * /*ntree*/, bNode *node)
{
  sync_attribute_name_to_storage(*node);
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  if (params.in_out() == SOCK_IN) {
    search_link_ops_for_shader_material_lighting_node(params);
    return;
  }
  int data_type;
  switch (params.other_socket().type) {
    case SOCK_FLOAT: data_type = SHD_ATTRIBUTE_DATA_FLOAT; break;
    case SOCK_INT: data_type = SHD_ATTRIBUTE_DATA_INT; break;
    case SOCK_BOOLEAN: data_type = SHD_ATTRIBUTE_DATA_BOOLEAN; break;
    case SOCK_VECTOR: data_type = SHD_ATTRIBUTE_DATA_VECTOR; break;
    case SOCK_RGBA: data_type = SHD_ATTRIBUTE_DATA_COLOR; break;
    case SOCK_ROTATION: data_type = SHD_ATTRIBUTE_DATA_ROTATION; break;
    case SOCK_MATRIX: data_type = SHD_ATTRIBUTE_DATA_MATRIX; break;
    default: return;
  }
  params.add_item(IFACE_("Attribute"), [data_type](LinkSearchOpParams &params) {
    bNode &node = params.add_node("ShaderNodeAttribute"_ustr);
    static_cast<NodeShaderAttribute *>(node.storage)->data_type = data_type;
    params.update_and_connect_available_socket(node, "Attribute"_ustr);
  });
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
  const bool need_legacy = out[0].hasoutput || out[1].hasoutput || out[2].hasoutput ||
                           out[3].hasoutput;
  const bool need_matrix = out[4].hasoutput;
  const bool typed = attr->data_type != SHD_ATTRIBUTE_DATA_LEGACY;
  if ((!need_legacy && !typed) || attr->data_type == SHD_ATTRIBUTE_DATA_MATRIX) {
    if (attr->data_type == SHD_ATTRIBUTE_DATA_MATRIX) {
      if (attr->type == SHD_ATTRIBUTE_GEOMETRY) {
        out[5].link = GPU_attribute_matrix(mat, attr_name);
      }
      else {
        GPU_link(mat, "node_attribute_matrix_zero", &out[5].link);
      }
      return 1;
    }
    if (need_matrix) {
      if (attr->type == SHD_ATTRIBUTE_GEOMETRY) {
        out[4].link = GPU_attribute_matrix(mat, attr_name);
      }
      else {
        GPU_link(mat, "node_attribute_matrix_zero", &out[4].link);
      }
    }
    return 1;
  }
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
      const std::string request = attr->data_type == SHD_ATTRIBUTE_DATA_ROTATION ?
                                      shader_rotation_attribute_name(attr_name) : attr_name;
      cd_attr = GPU_attribute(mat, CD_AUTO_FROM_NAME, request.c_str());

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

  if (typed) {
    const char *function = "node_attribute_as_float";
    switch (attr->data_type) {
      case SHD_ATTRIBUTE_DATA_INT:
        function = "node_attribute_as_int";
        break;
      case SHD_ATTRIBUTE_DATA_VECTOR:
        function = "node_attribute_as_vector";
        break;
      case SHD_ATTRIBUTE_DATA_ROTATION:
        function = "node_attribute_as_rotation";
        break;
      case SHD_ATTRIBUTE_DATA_COLOR:
        function = "node_attribute_as_color";
        break;
      case SHD_ATTRIBUTE_DATA_BOOLEAN:
        function = "node_attribute_as_boolean";
        break;
    }
    GPU_link(mat, function, cd_attr, &out[5].link);
    if (attr->type == SHD_ATTRIBUTE_GEOMETRY &&
        attr->data_type != SHD_ATTRIBUTE_DATA_ROTATION)
    {
      node_shader_gpu_bump_tex_coord(mat, node, &out[5].link);
    }
    return 1;
  }
  GPU_link(mat, "node_attribute", cd_attr, &out[0].link, &out[1].link, &out[2].link, &out[3].link);

  if (need_matrix) {
    if (attr->type == SHD_ATTRIBUTE_GEOMETRY) {
      out[4].link = GPU_attribute_matrix(mat, attr_name);
    }
    else {
      GPU_link(mat, "node_attribute_matrix_zero", &out[4].link);
    }
  }

  if (attr->type == SHD_ATTRIBUTE_GEOMETRY) {
    for (const int i : IndexRange(4)) {
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
  ntype.gather_link_search_ops = file_ns::node_gather_link_searches;
  ntype.gpu_fn = file_ns::node_shader_gpu_attribute;
  ntype.materialx_fn = file_ns::node_shader_materialx;

  bke::node_register_type(ntype);
}

}  // namespace blender
