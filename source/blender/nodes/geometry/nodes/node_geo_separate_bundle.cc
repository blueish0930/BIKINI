/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "ED_screen.hh"

#include "NOD_geo_bundle.hh"
#include "NOD_geometry_nodes_bundle.hh"
#include "NOD_image_points.hh"
#include "NOD_socket_items_blend.hh"
#include "NOD_socket_items_ops.hh"
#include "NOD_socket_items_ui.hh"
#include "NOD_socket_search_link.hh"
#include "NOD_sync_sockets.hh"

#include "BKE_idprop.hh"
#include "BKE_node_runtime.hh"
#include "BKE_type_conversions.hh"

#include "BLO_read_write.hh"

#include "COM_bundle_item.hh"
#include "COM_conversion_operation.hh"
#include "COM_domain.hh"
#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "GPU_shader.hh"

#include "UI_interface_layout.hh"
#include "shader/node_shader_util.hh"

#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"

#include <fmt/format.h>

namespace blender {

namespace nodes::node_geo_separate_bundle_cc {

NODE_STORAGE_FUNCS(NodeSeparateBundle);

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Bundle>("Bundle"_ustr).structure_type(StructureType::Single);
  const bNodeTree *tree = b.tree_or_null();
  const bNode *node = b.node_or_null();
  if (tree && node) {
    const NodeSeparateBundle &storage = node_storage(*node);
    for (const int i : IndexRange(storage.items_num)) {
      const NodeSeparateBundleItem &item = storage.items[i];
      const eNodeSocketDatatype socket_type = item.socket_type;
      const UString name = item.name ? UString(item.name) : ""_ustr;
      const UString identifier(SeparateBundleItemsAccessor::socket_identifier_for_item(item));
      auto &decl = b.add_output(socket_type, name, identifier)
                       .socket_name_ptr(
                           &tree->id, *SeparateBundleItemsAccessor::item_srna, &item, "name")
                       .propagate_all();
      if (item.structure_type != NodeSocketInterfaceStructureType::Auto) {
        decl.structure_type(StructureType(item.structure_type));
      }
      else {
        decl.structure_type(StructureType::Dynamic);
      }
      decl.custom_draw([i](CustomSocketDrawParams &params) {
        socket_items::ui::draw_item_socket_with_remove<SeparateBundleItemsAccessor>(
            params, i, std::nullopt, socket_items::ui::ItemRemoveButtonSide::Left);
      });
    }
  }
  b.add_output<decl::Extend>(""_ustr, "__extend__"_ustr)
      .custom_draw(socket_items::ui::draw_extend_socket_fn<SeparateBundleItemsAccessor>());
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  auto *storage = MEM_new<NodeSeparateBundle>(__func__);
  node->storage = storage;
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeSeparateBundle &src_storage = node_storage(*src_node);
  auto *dst_storage = MEM_new<NodeSeparateBundle>(__func__, dna::shallow_copy(src_storage));
  dst_node->storage = dst_storage;

  socket_items::copy_array<SeparateBundleItemsAccessor>(*src_node, *dst_node);
}

static void node_free_storage(bNode *node)
{
  socket_items::destruct_array<SeparateBundleItemsAccessor>(*node);
  MEM_delete(static_cast<NodeSeparateBundle *>(node->storage));
}

static bool node_insert_link(bke::NodeInsertLinkParams &params)
{
  if (params.C && params.link.tonode == &params.node && params.link.fromsock->type == SOCK_BUNDLE)
  {
    const NodeSeparateBundle &storage = node_storage(params.node);
    if (storage.items_num == 0) {
      SpaceNode *snode = CTX_wm_space_node(params.C);
      if (snode && snode->edittree == &params.ntree) {
        sync_sockets_separate_bundle(*snode, params.node, nullptr, params.link.fromsock);
      }
    }
    return true;
  }
  return socket_items::try_add_item_via_any_extend_socket<SeparateBundleItemsAccessor>(
      params.ntree, params.node, params.node, params.link);
}

static void node_layout_ex(ui::Layout &layout, bContext *C, PointerRNA *node_ptr)
{
  bNodeTree &ntree = *reinterpret_cast<bNodeTree *>(node_ptr->owner_id);
  bNode &node = *static_cast<bNode *>(node_ptr->data);

  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);

  layout.op("node.sockets_sync", IFACE_("Sync"), ICON_FILE_REFRESH);
  layout.prop(node_ptr, "define_signature", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  if (ui::Layout *panel = layout.panel(C, "bundle_items", false, IFACE_("Bundle Items"))) {
    socket_items::ui::draw_items_list_with_operators<SeparateBundleItemsAccessor>(
        C, panel, ntree, node);
    socket_items::ui::draw_active_item_props<SeparateBundleItemsAccessor>(
        ntree, node, [&](PointerRNA *item_ptr) {
          panel->use_property_split_set(true);
          panel->use_property_decorate_set(false);
          panel->prop(item_ptr, "socket_type", UI_ITEM_NONE, IFACE_("Type"), ICON_NONE);
          panel->prop(item_ptr, "structure_type", UI_ITEM_NONE, IFACE_("Shape"), ICON_NONE);
        });
  }
}

static void node_operators()
{
  socket_items::ops::make_common_operators<SeparateBundleItemsAccessor>();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  nodes::BundlePtr bundle = params.extract_input<nodes::BundlePtr>("Bundle"_ustr);
  if (!bundle) {
    params.set_default_remaining_outputs();
    return;
  }

  const bNode &node = params.node();
  const NodeSeparateBundle &storage = node_storage(node);

  lf::Params &lf_params = params.low_level_lazy_function_params();

  for (const int i : IndexRange(storage.items_num)) {
    const NodeSeparateBundleItem &item = storage.items[i];
    const StringRef name = item.name;
    std::optional<BundleKey> key = BundleKey::from_str(name);
    if (!key) {
      continue;
    }
    const bke::bNodeSocketType *stype = bke::node_socket_type_find_static(item.socket_type);
    if (!stype || !stype->geometry_nodes_default_value) {
      continue;
    }
    const BundleItemValue *value = bundle->lookup(*key);
    if (!value) {
      params.error_message_add(
          NodeWarningType::Error,
          fmt::format(fmt::runtime(TIP_("Value not found in bundle: \"{}\"")), name));
      continue;
    }
    const auto *socket_value = std::get_if<BundleItemSocketValue>(&value->value);
    if (!socket_value) {
      params.error_message_add(
          NodeWarningType::Error,
          fmt::format("{}: \"{}\"", TIP_("Cannot get internal value from bundle"), name));
      continue;
    }

    SocketValueVariant output_value = std::move(socket_value->value);
    if (socket_value->type->type != stype->type) {
      if (std::optional<SocketValueVariant> converted_value = implicitly_convert_socket_value(
              *socket_value->type, output_value, *stype))
      {
        output_value = std::move(*converted_value);
        params.error_message_add(
            NodeWarningType::Info,
            fmt::format("{}: \"{}\" ({} " BLI_STR_UTF8_BLACK_RIGHT_POINTING_SMALL_TRIANGLE " {})",
                        TIP_("Implicit type conversion when separating bundle"),
                        name,
                        TIP_(socket_value->type->label),
                        TIP_(stype->label)));
      }
      else {
        params.error_message_add(
            NodeWarningType::Error,
            fmt::format("{}: \"{}\" ({} " BLI_STR_UTF8_BLACK_RIGHT_POINTING_SMALL_TRIANGLE " {})",
                        TIP_("Conversion not supported when separating bundle"),
                        name,
                        TIP_(socket_value->type->label),
                        TIP_(stype->label)));
        output_value = *stype->geometry_nodes_default_value;
      }
    }
    lf_params.set_output(i, std::move(output_value));
  }

  params.set_default_remaining_outputs();
}

using namespace blender::compositor;

class SeparateBundleOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    nodes::BundlePtr bundle = this->get_input("Bundle").get_single_value<nodes::BundlePtr>();

    const NodeSeparateBundle &storage = node_storage(this->node());
    for (const int i : IndexRange(storage.items_num)) {
      Result &result = this->get_result(this->node().output_socket(i).identifier);
      if (!result.should_compute()) {
        continue;
      }

      const NodeSeparateBundleItem &item = storage.items[i];
      const StringRef name = item.name;
      std::optional<BundleKey> key = BundleKey::from_str(name);
      if (!key) {
        continue;
      }

      const BundleItemValue *item_value = bundle->lookup(key.value());
      if (!item_value) {
        this->add_warning(
            NodeWarningType::Error,
            fmt::format(fmt::runtime(TIP_("Value not found in bundle: \"{}\"")), name));
        continue;
      }

      Result bundle_result = BundleItem::get_result(this->context(), *item_value);
      BLI_SCOPED_DEFER([&]() { bundle_result.release(); });
      if (result.type() == bundle_result.type()) {
        result.share_data(bundle_result);
        continue;
      }

      const bke::DataTypeConversions &conversions = bke::get_implicit_type_conversions();
      if (!conversions.is_convertible(bundle_result.get_cpp_type(), result.get_cpp_type())) {
        this->add_warning(
            NodeWarningType::Error,
            fmt::format("{}: \"{}\" ({} " BLI_STR_UTF8_BLACK_RIGHT_POINTING_SMALL_TRIANGLE " {})",
                        TIP_("Conversion not supported when separating bundle"),
                        name,
                        TIP_(Result::type_name(bundle_result.type())),
                        TIP_(Result::type_name(result.type()))));
        continue;
      }

      ConversionOperation conversion_operation(
          this->context(), bundle_result.type(), result.type());
      Result conversion_input = this->context().create_result(bundle_result.type(),
                                                              bundle_result.precision());
      conversion_input.share_data(bundle_result);
      conversion_operation.map_input_to_result(&conversion_input);
      conversion_operation.evaluate();
      result.share_data(conversion_operation.get_result());
      conversion_operation.get_result().release();

      this->add_warning(
          NodeWarningType::Info,
          fmt::format("{}: \"{}\" ({} " BLI_STR_UTF8_BLACK_RIGHT_POINTING_SMALL_TRIANGLE " {})",
                      TIP_("Implicit type conversion when separating bundle"),
                      name,
                      TIP_(Result::type_name(bundle_result.type())),
                      TIP_(Result::type_name(result.type()))));
    }

    this->allocate_default_remaining_outputs();
  }
};

static NodeOperation *get_compositor_bundle_operation(Context &context, const bNode &node)
{
  return new SeparateBundleOperation(context, node);
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const bNodeSocket &other_socket = params.other_socket();
  if (other_socket.in_out == SOCK_IN) {
    if (!SeparateBundleItemsAccessor::supports_socket_type(other_socket.typeinfo->type,
                                                           params.node_tree().type))
    {
      return;
    }
    params.add_item(IFACE_("Item"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("NodeSeparateBundle"_ustr);
      const auto *item =
          socket_items::add_item_with_socket_type_and_name<SeparateBundleItemsAccessor>(
              params.node_tree, node, params.socket.typeinfo->type, params.socket.name);
      params.update_and_connect_available_socket(node, UString(item->name));
    });
  }
  else {
    if (other_socket.type != SOCK_BUNDLE) {
      return;
    }
    params.add_item(IFACE_("Bundle"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("NodeSeparateBundle"_ustr);
      params.connect_available_socket(node, "Bundle"_ustr);

      SpaceNode &snode = *CTX_wm_space_node(&params.C);
      sync_sockets_separate_bundle(snode, node, nullptr);
    });
  }
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  socket_items::blend_write<SeparateBundleItemsAccessor>(&writer, node);
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  socket_items::blend_read_data<SeparateBundleItemsAccessor>(&reader, node);
}

using namespace blender::compositor;
using namespace blender::nodes::image_points;

/**
 * Image Process / Compositor: Bundle is a type marker (String). Pixel items come from the
 * linked origin node — Point Stamp publishes premultiplied maps into StampAttrMapsCache
 * (float4 storage); outputs use the Separate Bundle item socket types (Float / Vector / Color).
 */
class SeparateBundleImageOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void publish_from_gpu_color(const StampAttrGpuTexture &gpu_entry, Result &out)
  {
    if (!gpu_entry.texture) {
      out.allocate_invalid();
      return;
    }
    Result color = this->context().create_result(ResultType::Color);
    color.share_data(gpu_entry.texture, gpu_entry.sharing_info);
    const ResultType out_type = out.type();
    if (out_type == ResultType::Color) {
      out.share_data(color);
      color.release();
      return;
    }
    const char *info = nullptr;
    switch (out_type) {
      case ResultType::Float:
        info = "compositor_convert_color_to_float";
        break;
      case ResultType::Float2:
        info = "compositor_convert_color_to_float2";
        break;
      case ResultType::Float3:
        info = "compositor_convert_color_to_float3";
        break;
      case ResultType::Float4:
        info = "compositor_convert_color_to_float4";
        break;
      case ResultType::Int:
        info = "compositor_convert_color_to_int";
        break;
      default:
        out.share_data(color);
        color.release();
        return;
    }
    gpu::Shader *shader = this->context().get_shader(info);
    if (!shader) {
      out.share_data(color);
      color.release();
      return;
    }
    GPU_shader_bind(shader);
    if (out_type == ResultType::Float) {
      const float luma[3] = {1.0f, 0.0f, 0.0f};
      GPU_shader_uniform_3fv(shader, "luminance_coefficients_u", luma);
    }
    color.bind_as_texture(shader, "input_tx");
    out.allocate_texture(color.domain());
    out.bind_as_image(shader, "output_img");
    compute_dispatch_threads_at_least(shader, color.domain().data_size);
    color.unbind_as_texture();
    out.unbind_as_image();
    GPU_shader_unbind();
    color.release();
  }

  void execute() override
  {
    const NodeSeparateBundle &storage = node_storage(this->node());
    const StampAttrMaps *stamp_maps = nullptr;
    const StampAttrGpuMaps *stamp_gpu_maps = nullptr;
    std::string bundle_key;

    if (this->has_input("Bundle")) {
      Result &bundle_in = this->get_input("Bundle");
      if (bundle_in.type() == ResultType::String && bundle_in.is_allocated()) {
        bundle_key = bundle_in.get_single_value_default<std::string>();
        if (StampAttrGpuCache *gpu_cache = active_stamp_attr_gpu_cache()) {
          stamp_gpu_maps = gpu_cache->lookup(bundle_key);
        }
        if (StampAttrMapsCache *cache = active_stamp_attr_cache()) {
          stamp_maps = cache->lookup(bundle_key);
        }
      }
    }

    for (const int i : IndexRange(storage.items_num)) {
      const NodeSeparateBundleItem &item = storage.items[i];
      if (!item.name) {
        continue;
      }
      const std::string item_name(item.name);
      const std::string identifier = SeparateBundleItemsAccessor::socket_identifier_for_item(item);
      Result &out = this->get_result(identifier);
      if (!out.should_compute()) {
        continue;
      }

      /* Result type already matches Separate Bundle socket (Float / Float2 / Float3 / Color). */
      const ResultType out_type = out.type();

      /* 0) GPU texture cache (Rasterize / fast path) — no GPU→CPU round-trip. */
      if (stamp_gpu_maps) {
        if (const StampAttrGpuTexture *gpu_tex = stamp_gpu_maps->maps.lookup_ptr(item_name)) {
          if (this->context().use_gpu() && gpu_tex->texture) {
            this->publish_from_gpu_color(*gpu_tex, out);
            continue;
          }
        }
      }

      /* 1) Point Stamp attr cache (float4 storage → typed pixel result). */
      if (stamp_maps) {
        if (const Array<float4> *pixels = stamp_maps->maps.lookup_ptr(item_name)) {
          const int2 size = math::max(stamp_maps->size, int2(1));
          Result cpu = this->context().create_result(out_type);
          cpu.allocate_texture(Domain(size), false, ResultStorageType::CPUImage);
          parallel_for(size, [&](const int2 texel) {
            const float4 v = (*pixels)[int64_t(texel.y) * size.x + texel.x];
            /* Cache stores premultiplied RGBA; Color keeps that, scalars/vecs un-premultiply. */
            const float inv_a = (v.w > 1e-8f) ? (1.0f / v.w) : 0.0f;
            switch (out_type) {
              case ResultType::Float:
                cpu.store_pixel(texel, v.x * inv_a);
                break;
              case ResultType::Float2:
                cpu.store_pixel(texel, float2(v.x, v.y) * inv_a);
                break;
              case ResultType::Float3:
                cpu.store_pixel(texel, float3(v.x, v.y, v.z) * inv_a);
                break;
              case ResultType::Float4:
                cpu.store_pixel(texel, float4(v.xyz() * inv_a, v.w));
                break;
              case ResultType::Int:
                cpu.store_pixel(texel, int(v.x * inv_a));
                break;
              case ResultType::Color:
              default:
                cpu.store_pixel(texel, Color(v.x, v.y, v.z, v.w));
                break;
            }
          });
          if (this->context().use_gpu()) {
            Result gpu = cpu.upload_to_gpu(true);
            out.share_data(gpu);
            gpu.release();
          }
          else {
            out.share_data(cpu);
          }
          cpu.release();
          continue;
        }
      }

      /* Empty typed map (no stamp cache entry for this item). */
      const int2 res = math::max(this->context().get_compositing_domain().data_size, int2(1));
      Result cpu = this->context().create_result(out_type);
      cpu.allocate_texture(Domain(res), false, ResultStorageType::CPUImage);
      parallel_for(res, [&](const int2 texel) {
        switch (out_type) {
          case ResultType::Float:
            cpu.store_pixel(texel, 0.0f);
            break;
          case ResultType::Float2:
            cpu.store_pixel(texel, float2(0.0f));
            break;
          case ResultType::Float3:
            cpu.store_pixel(texel, float3(0.0f));
            break;
          case ResultType::Float4:
            cpu.store_pixel(texel, float4(0.0f));
            break;
          case ResultType::Int:
            cpu.store_pixel(texel, 0);
            break;
          case ResultType::Color:
          default:
            cpu.store_pixel(texel, Color(0.0f, 0.0f, 0.0f, 0.0f));
            break;
        }
      });
      if (this->context().use_gpu()) {
        Result gpu = cpu.upload_to_gpu(true);
        out.share_data(gpu);
        gpu.release();
      }
      else {
        out.share_data(cpu);
      }
      cpu.release();
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  if (node.owner_tree().type == NTREE_IMAGE) {
    return new SeparateBundleImageOperation(context, node);
  }
  return get_compositor_bundle_operation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  common_node_type_base(&ntype, "NodeSeparateBundle"_ustr, NODE_SEPARATE_BUNDLE);
  ntype.ui_name = "Separate Bundle";
  ntype.ui_description = "Split a bundle into multiple sockets.";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.insert_link = node_insert_link;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons_ex = node_layout_ex;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.register_operators = node_operators;
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  ntype.get_compositor_operation = get_compositor_operation;
  bke::node_type_storage(ntype, "NodeSeparateBundle", node_free_storage, node_copy_storage);
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace nodes::node_geo_separate_bundle_cc

namespace nodes {

StructRNA **SeparateBundleItemsAccessor::item_srna = &RNA_NodeSeparateBundleItem;

void SeparateBundleItemsAccessor::blend_write_item(BlendWriter *writer, const ItemT &item)
{
  writer->write_string(item.name);
}

void SeparateBundleItemsAccessor::blend_read_data_item(BlendDataReader *reader, ItemT &item)
{
  BLO_read_string(reader, &item.name);
}

}  // namespace nodes
}  // namespace blender
