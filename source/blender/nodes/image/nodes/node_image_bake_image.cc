/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Image Process: Bake Image — write evaluated images to a directory (manual Bake only).
 *
 * - Dynamic Color inputs (like compositor File Output): extend socket / items list.
 * - Path (directory) and Prefix (filename prefix) sockets at the top.
 * - Disk write only when the Bake button/operator runs (not on every tree cook). */

#include <cstring>
#include <fmt/format.h>

#include "BLI_array.hh"
#include "BLI_fileops.hh"
#include "BLI_math_base.hh"
#include "BLI_path_utils.hh"
#include "BLI_span.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_vector.hh"

#include "BKE_context.hh"
#include "BKE_main.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_report.hh"
#include "BKE_scene.hh"

#include "BLO_read_write.hh"

#include "DNA_node_types.h"
#include "DNA_scene_types.h"
#include "DNA_space_types.h"

#include "MEM_guardedalloc.h"

#include "DEG_depsgraph.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"
#include "RNA_prototypes.hh"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "IMB_colormanagement.hh"
#include "IMB_imbuf.hh"
#include "IMB_imbuf_enums.h"

#include "BLI_math_vector_types.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_undefined_node_operation.hh"
#include "COM_utilities.hh"

#include "NOD_image.hh"
#include "NOD_image_bake.hh"
#include "NOD_socket.hh"
#include "NOD_socket_items.hh"
#include "NOD_socket_items_blend.hh"
#include "NOD_socket_items_ops.hh"
#include "NOD_socket_items_ui.hh"
#include "NOD_socket_search_link.hh"

#include "ED_node.hh"
#include "ED_screen.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_bake_image_cc {

NODE_STORAGE_FUNCS(NodeImageBakeImage)

enum class BakeFormat : int {
  PNG = 0,
  JPEG = 1,
  OpenEXR = 2,
  TIFF = 3,
  BMP = 4,
  Targa = 5,
  WebP = 6,
  HDR = 7,
};

static const char *format_extension(const BakeFormat format)
{
  switch (format) {
    case BakeFormat::PNG:
      return ".png";
    case BakeFormat::JPEG:
      return ".jpg";
    case BakeFormat::OpenEXR:
      return ".exr";
    case BakeFormat::TIFF:
      return ".tif";
    case BakeFormat::BMP:
      return ".bmp";
    case BakeFormat::Targa:
      return ".tga";
    case BakeFormat::WebP:
      return ".webp";
    case BakeFormat::HDR:
      return ".hdr";
  }
  return ".png";
}

static eImbFileType format_to_ftype(const BakeFormat format)
{
  switch (format) {
    case BakeFormat::PNG:
      return IMB_FTYPE_PNG;
    case BakeFormat::JPEG:
      return IMB_FTYPE_JPG;
    case BakeFormat::OpenEXR:
      return IMB_FTYPE_OPENEXR;
    case BakeFormat::TIFF:
      return IMB_FTYPE_TIF;
    case BakeFormat::BMP:
      return IMB_FTYPE_BMP;
    case BakeFormat::Targa:
      return IMB_FTYPE_TGA;
    case BakeFormat::WebP:
#ifdef WITH_IMAGE_WEBP
      return IMB_FTYPE_WEBP;
#else
      return IMB_FTYPE_PNG;
#endif
    case BakeFormat::HDR:
      return IMB_FTYPE_RADHDR;
  }
  return IMB_FTYPE_PNG;
}

static bool format_is_float(const BakeFormat format)
{
  return ELEM(format, BakeFormat::OpenEXR, BakeFormat::HDR);
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  /* Geometry Nodes style: draw_buttons (Bake / Format / Color Space) above sockets. */
  b.add_default_layout();

  /* Path and Prefix first (above dynamic image slots), matching user request. */
  b.add_input<decl::String>("Path"_ustr)
      .subtype(PROP_FILEPATH)
      .default_value("//")
      .structure_type(StructureType::Single)
      .description(
          "Output directory. // = relative to the saved .blend file (e.g. //textures/). "
          "Requires the blend to be saved — never falls back to the exe folder. "
          "Or use an absolute path (e.g. E:/out/). Click the folder icon to browse");

  b.add_input<decl::String>("Prefix"_ustr)
      .default_value("")
      .structure_type(StructureType::Single)
      .description(
          "Optional filename prefix prepended to each image item name "
          "(e.g. prefix \"shot_\" + item \"Color\" → shot_Color.png)");

  const bNodeTree *node_tree = b.tree_or_null();
  const bNode *node = b.node_or_null();
  if (node_tree && node) {
    const NodeImageBakeImage &storage = node_storage(*node);
    for (const int i : IndexRange(storage.items_num)) {
      const NodeImageBakeItem &item = storage.items[i];
      const std::string identifier = ImageBakeItemsAccessor::socket_identifier_for_item(item);
      b.add_input<decl::Color>(UString(item.name ? item.name : "Image"), UString(identifier))
          .default_value({0.0f, 0.0f, 0.0f, 1.0f})
          .structure_type(StructureType::Dynamic)
          .compositor_realization_mode(CompositorInputRealizationMode::None)
          .socket_name_ptr(&node_tree->id, *ImageBakeItemsAccessor::item_srna, &item, "name")
          .description("Image to bake; socket name is the output filename (with Prefix)")
          .custom_draw([i](CustomSocketDrawParams &params) {
            socket_items::ui::draw_item_socket_with_remove<ImageBakeItemsAccessor>(params, i);
          });
    }
  }

  b.add_input<decl::Extend>(""_ustr, "__extend__"_ustr)
      .custom_draw(socket_items::ui::draw_extend_socket_fn<ImageBakeItemsAccessor>());
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  auto *storage = MEM_new<NodeImageBakeImage>(__func__);
  storage->format = int(BakeFormat::PNG);
  storage->color_space[0] = '\0'; /* Auto */
  node->storage = storage;

  /* One default Color slot so the node is usable without first using Extend. */
  socket_items::add_item_with_name<ImageBakeItemsAccessor>(*node, "Image");
}

static void node_free_storage(bNode *node)
{
  socket_items::destruct_array<ImageBakeItemsAccessor>(*node);
  MEM_delete(static_cast<NodeImageBakeImage *>(node->storage));
}

static void node_copy_storage(bNodeTree * /*dst*/, bNode *dst_node, const bNode *src_node)
{
  const NodeImageBakeImage &src = node_storage(*src_node);
  auto *dst = MEM_new<NodeImageBakeImage>(__func__, dna::shallow_copy(src));
  dst_node->storage = dst;
  socket_items::copy_array<ImageBakeItemsAccessor>(*src_node, *dst_node);
}

static bool node_insert_link(bke::NodeInsertLinkParams &params)
{
  return socket_items::try_add_item_via_any_extend_socket<ImageBakeItemsAccessor>(
      params.ntree, params.node, params.node, params.link);
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);

  {
    ui::Layout &row = layout.row(true);
    row.scale_y_set(1.2f);
    row.op("node.image_process_bake", IFACE_("Bake"), ICON_RENDER_STILL);
  }

  layout.prop(ptr, "format", UI_ITEM_NONE, IFACE_("Format"), ICON_NONE);
  layout.prop(ptr, "color_space", UI_ITEM_NONE, IFACE_("Color Space"), ICON_NONE);
}

static void node_layout_ex(ui::Layout &layout, bContext *C, PointerRNA *ptr)
{
  node_layout(layout, C, ptr);

  if (ui::Layout *panel = layout.panel(C, "image_bake_items", false, IFACE_("Images"))) {
    bNodeTree &tree = *reinterpret_cast<bNodeTree *>(ptr->owner_id);
    bNode &node = *static_cast<bNode *>(ptr->data);
    socket_items::ui::draw_items_list_with_operators<ImageBakeItemsAccessor>(
        C, panel, tree, node);
    socket_items::ui::draw_active_item_props<ImageBakeItemsAccessor>(
        tree, node, [&](PointerRNA *item_pointer) {
          panel->prop(item_pointer, "name", UI_ITEM_NONE, IFACE_("Name"), ICON_NONE);
        });
  }
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  socket_items::blend_write<ImageBakeItemsAccessor>(&writer, node);
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  socket_items::blend_read_data<ImageBakeItemsAccessor>(&reader, node);
}

using namespace blender::compositor;

static const char *resolve_output_colorspace(const NodeImageBakeImage &storage,
                                             const BakeFormat format)
{
  if (storage.color_space[0] != '\0') {
    return storage.color_space;
  }
  if (format_is_float(format)) {
    return IMB_colormanagement_role_colorspace_name_get(COLOR_ROLE_SCENE_LINEAR);
  }
  return IMB_colormanagement_role_colorspace_name_get(COLOR_ROLE_DEFAULT_BYTE);
}

static bool blendfile_is_saved(const Main &bmain)
{
  const char *path = BKE_main_blendfile_path(&bmain);
  return path && path[0] != '\0';
}

static bool bake_path_ok_for_write(const Main &bmain, const char *filepath_or_dir)
{
  if (!filepath_or_dir || filepath_or_dir[0] == '\0') {
    return false;
  }
  if (BLI_path_is_rel(filepath_or_dir)) {
    return blendfile_is_saved(bmain);
  }
  return true;
}

static std::string bake_node_path_default(const bNode &node)
{
  const bNodeSocket *sock = node.input_by_identifier("Path"_ustr);
  if (!sock || !sock->default_value) {
    return "//";
  }
  const auto *val = static_cast<const bNodeSocketValueString *>(sock->default_value);
  return val->value ? std::string(val->value) : std::string("//");
}

static bool save_float_rgba_to_path(Context &context,
                                    const Span<float> rgba,
                                    const int2 size,
                                    const char *filepath,
                                    const BakeFormat format,
                                    const char *output_colorspace)
{
  if (size.x < 1 || size.y < 1) {
    return false;
  }
  const int64_t pixel_count = int64_t(size.x) * size.y;
  if (rgba.size() < pixel_count * 4) {
    return false;
  }

  const Main &bmain = context.get_main();
  if (!bake_path_ok_for_write(bmain, filepath)) {
    return false;
  }

  ImBuf *ibuf = IMB_allocImBuf(uint(size.x), uint(size.y), ImBufFlags::FloatData);
  if (!ibuf) {
    return false;
  }

  float *dst = ibuf->float_data_for_write();
  std::memcpy(dst, rgba.data(), size_t(pixel_count) * 4 * sizeof(float));

  const char *scene_linear = IMB_colormanagement_role_colorspace_name_get(COLOR_ROLE_SCENE_LINEAR);
  IMB_colormanagement_assign_float_colorspace(ibuf, scene_linear);

  if (output_colorspace && output_colorspace[0] && !STREQ(output_colorspace, scene_linear)) {
    IMB_colormanagement_transform_float(
        dst, size.x, size.y, 4, scene_linear, output_colorspace, false);
    IMB_colormanagement_assign_float_colorspace(ibuf, output_colorspace);
  }

  ibuf->ftype = format_to_ftype(format);
  if (format == BakeFormat::JPEG) {
    ibuf->foptions.quality = 90;
  }

  ImBufFlags flags = ImBufFlags::FloatData;
  if (!format_is_float(format)) {
    IMB_byte_from_float(ibuf);
    flags = ImBufFlags::ByteData;
  }

  char abs_path[FILE_MAX];
  STRNCPY(abs_path, filepath);
  BLI_path_abs(abs_path, BKE_main_blendfile_path(&bmain));

  if (BLI_path_is_rel(abs_path) || abs_path[0] == '\0') {
    IMB_freeImBuf(ibuf);
    return false;
  }

  char dir[FILE_MAX];
  BLI_path_split_dir_part(abs_path, dir, sizeof(dir));
  if (dir[0]) {
    BLI_dir_create_recursive(dir);
  }

  const bool ok = IMB_save_image(ibuf, abs_path, flags);
  IMB_freeImBuf(ibuf);
  return ok;
}

static std::string join_dir_filename(const StringRef directory,
                                     const StringRef filename_no_ext,
                                     const BakeFormat format)
{
  char dir[FILE_MAX];
  STRNCPY(dir, std::string(directory).c_str());
  BLI_path_slash_ensure(dir, sizeof(dir));

  std::string base(filename_no_ext);
  const size_t dot = base.find_last_of('.');
  if (dot != std::string::npos && dot > 0) {
    base = base.substr(0, dot);
  }
  if (base.empty()) {
    base = "bake";
  }

  char safe_base[FILE_MAXFILE];
  STRNCPY(safe_base, base.c_str());
  BLI_path_make_safe(safe_base);

  char filename[FILE_MAXFILE];
  BLI_snprintf(filename, sizeof(filename), "%s%s", safe_base, format_extension(format));

  char out[FILE_MAX];
  BLI_path_join(out, sizeof(out), dir, filename);
  return out;
}

/** Download Color/Float/Vector result into packed float RGBA. */
static bool result_to_rgba(Context &context,
                           Result &result,
                           Array<float> &r_rgba,
                           int2 &r_size)
{
  auto fill_from_cpu = [&](Result &cpu) {
    r_size = math::max(cpu.domain().data_size, int2(1));
    const int64_t n = int64_t(r_size.x) * r_size.y;
    r_rgba.reinitialize(n * 4);
    for (int y = 0; y < r_size.y; y++) {
      for (int x = 0; x < r_size.x; x++) {
        const int2 texel(x, y);
        const int64_t i = int64_t(y) * r_size.x + x;
        float4 c(0.0f, 0.0f, 0.0f, 1.0f);
        if (cpu.type() == ResultType::Color) {
          const Color col = cpu.load_pixel<Color>(texel);
          c = float4(col.r, col.g, col.b, col.a);
        }
        else if (cpu.type() == ResultType::Float) {
          const float v = cpu.load_pixel<float>(texel);
          c = float4(v, v, v, 1.0f);
        }
        else if (cpu.type() == ResultType::Float2) {
          const float2 v = cpu.load_pixel<float2>(texel);
          c = float4(v.x, v.y, 0.0f, 1.0f);
        }
        else if (cpu.type() == ResultType::Float3) {
          const float3 v = cpu.load_pixel<float3>(texel);
          c = float4(v.x, v.y, v.z, 1.0f);
        }
        r_rgba[i * 4 + 0] = c.x;
        r_rgba[i * 4 + 1] = c.y;
        r_rgba[i * 4 + 2] = c.z;
        r_rgba[i * 4 + 3] = c.w;
      }
    }
  };

  if (result.is_single_value()) {
    r_size = math::max(context.get_compositing_domain().data_size, int2(1));
    const int64_t n = int64_t(r_size.x) * r_size.y;
    r_rgba.reinitialize(n * 4);
    float4 c(0.0f, 0.0f, 0.0f, 1.0f);
    if (result.type() == ResultType::Color) {
      const Color col = result.get_single_value_default<Color>();
      c = float4(col.r, col.g, col.b, col.a);
    }
    else if (result.type() == ResultType::Float) {
      const float v = result.get_single_value_default<float>();
      c = float4(v, v, v, 1.0f);
    }
    else if (result.type() == ResultType::Float2) {
      const float2 v = result.get_single_value_default<float2>();
      c = float4(v.x, v.y, 0.0f, 1.0f);
    }
    else if (result.type() == ResultType::Float3) {
      const float3 v = result.get_single_value_default<float3>();
      c = float4(v.x, v.y, v.z, 1.0f);
    }
    for (const int64_t i : IndexRange(n)) {
      r_rgba[i * 4 + 0] = c.x;
      r_rgba[i * 4 + 1] = c.y;
      r_rgba[i * 4 + 2] = c.z;
      r_rgba[i * 4 + 3] = c.w;
    }
    return true;
  }

  if (!result.is_allocated()) {
    return false;
  }

  if (context.use_gpu()) {
    Result cpu = result.download_to_cpu();
    fill_from_cpu(cpu);
    cpu.release();
  }
  else {
    fill_from_cpu(result);
  }
  return true;
}

class BakeImageOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  BakeImageOperation(Context &context, const bNode &node) : NodeOperation(context, node)
  {
    if (this->has_input("Path")) {
      this->get_input_descriptor("Path").expects_single_value = true;
    }
    if (this->has_input("Prefix")) {
      this->get_input_descriptor("Prefix").expects_single_value = true;
    }
    const NodeImageBakeImage &storage = node_storage(node);
    for (const int i : IndexRange(storage.items_num)) {
      const std::string id = ImageBakeItemsAccessor::socket_identifier_for_item(storage.items[i]);
      if (this->has_input(id)) {
        InputDescriptor &desc = this->get_input_descriptor(id);
        desc.realization_mode = InputRealizationMode::None;
      }
    }
  }

  void execute() override
  {
    if (!compositor::image_process_bake_write_enabled()) {
      return;
    }

    const NodeImageBakeImage &storage = node_storage(this->node());
    const BakeFormat format = BakeFormat(storage.format);
    const char *output_cs = resolve_output_colorspace(storage, format);

    const std::string directory = this->get_input("Path").get_single_value_default<std::string>();
    if (directory.empty()) {
      this->add_warning(NodeWarningType::Error, "Bake Path is empty");
      return;
    }
    if (!bake_path_ok_for_write(this->context().get_main(), directory.c_str())) {
      this->add_warning(
          NodeWarningType::Error,
          "Save the blend file first. Relative Path (//) needs a saved .blend location "
          "(will not write next to the Blender executable). "
          "Or set Path to an absolute directory");
      return;
    }

    if (storage.items_num < 1) {
      this->add_warning(NodeWarningType::Error,
                        "Add at least one Image slot (drag onto the extend socket or use the "
                        "Images list in the sidebar)");
      return;
    }

    const std::string prefix = this->has_input("Prefix") ?
                                   this->get_input("Prefix").get_single_value_default<std::string>() :
                                   std::string();

    int written = 0;
    for (const int i : IndexRange(storage.items_num)) {
      const NodeImageBakeItem &item = storage.items[i];
      const std::string identifier = ImageBakeItemsAccessor::socket_identifier_for_item(item);
      if (!this->has_input(identifier)) {
        continue;
      }
      Result &input = this->get_input(identifier);
      if (!(input.is_allocated() || input.is_single_value())) {
        this->add_warning(NodeWarningType::Error,
                          fmt::format("Image slot \"{}\" has no data",
                                      item.name ? item.name : identifier));
        continue;
      }

      Array<float> rgba;
      int2 size(1);
      if (!result_to_rgba(this->context(), input, rgba, size)) {
        this->add_warning(NodeWarningType::Error,
                          fmt::format("Failed to read image slot \"{}\"",
                                      item.name ? item.name : identifier));
        continue;
      }

      std::string base_name = prefix;
      base_name += (item.name && item.name[0]) ? item.name : "Image";
      const std::string path = join_dir_filename(directory, base_name, format);
      if (save_float_rgba_to_path(this->context(), rgba, size, path.c_str(), format, output_cs)) {
        written++;
      }
      else {
        this->add_warning(NodeWarningType::Error, fmt::format("Failed to write \"{}\"", path));
      }
    }

    if (written == 0) {
      this->add_warning(NodeWarningType::Error, "Bake wrote no files");
    }
    else {
      this->add_warning(NodeWarningType::Info, fmt::format("Wrote {} file(s)", written));
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new BakeImageOperation(context, node);
}

/* -------------------------------------------------------------------- */
/** \name Bake operator
 * \{ */

static bool image_process_bake_poll(bContext *C)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return false;
  }
  return snode->edittree->type == NTREE_IMAGE;
}

static wmOperatorStatus image_process_bake_exec(bContext *C, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!bmain || !scene || !snode || !snode->edittree) {
    BKE_report(op->reports, RPT_ERROR, "No active GPU Texture Editor tree");
    return OPERATOR_CANCELLED;
  }
  bNodeTree &ntree = *snode->edittree;
  if (ntree.type != NTREE_IMAGE) {
    BKE_report(op->reports, RPT_ERROR, "Active tree is not a GPU Texture Editor tree");
    return OPERATOR_CANCELLED;
  }

  {
    const bool blend_saved = blendfile_is_saved(*bmain);
    bool needs_saved_blend = false;
    bool has_bake_node = false;
    for (bNode *node : ntree.all_nodes()) {
      if (!node->is_type("ImageNodeBakeImage"_ustr) || node->is_muted()) {
        continue;
      }
      has_bake_node = true;
      const std::string path = bake_node_path_default(*node);
      if (path.empty()) {
        BKE_report(op->reports,
                   RPT_ERROR,
                   "Bake Image Path is empty — set an absolute folder or // relative path");
        return OPERATOR_CANCELLED;
      }
      if (BLI_path_is_rel(path.c_str())) {
        needs_saved_blend = true;
      }
    }
    if (!has_bake_node) {
      BKE_report(op->reports, RPT_ERROR, "No Bake Image node in the tree");
      return OPERATOR_CANCELLED;
    }
    if (needs_saved_blend && !blend_saved) {
      BKE_report(op->reports,
                 RPT_ERROR,
                 "Save the blend file first. Relative Path (//) is based on the .blend location "
                 "and will not write next to the Blender executable. "
                 "Or set Path to an absolute directory (e.g. E:/export/)");
      return OPERATOR_CANCELLED;
    }
  }

  bool used_gpu = false;
  int2 resolution(1024, 1024);
  if (snode->image_resolution[0] > 0 && snode->image_resolution[1] > 0) {
    resolution = int2(snode->image_resolution[0], snode->image_resolution[1]);
  }

  compositor::image_process_set_bake_write_enabled(true);
  const bool ok = ntreeImageNodesEvaluate(*bmain, *scene, ntree, &used_gpu, resolution);
  compositor::image_process_set_bake_write_enabled(false);

  if (!ok) {
    BKE_report(op->reports, RPT_ERROR, "GPU Texture Editor evaluation failed");
    return OPERATOR_CANCELLED;
  }

  BKE_report(op->reports, RPT_INFO, "Bake complete");
  WM_event_add_notifier(C, NC_NODE | NA_EDITED, &ntree);
  return OPERATOR_FINISHED;
}

static void NODE_OT_image_process_bake(wmOperatorType *ot)
{
  ot->name = "Bake GPU Texture Editor";
  ot->idname = "NODE_OT_image_process_bake";
  ot->description =
      "Evaluate the GPU Texture Editor tree and write Bake Image node outputs to disk";
  ot->exec = image_process_bake_exec;
  ot->poll = image_process_bake_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static void node_operators()
{
  WM_operatortype_append(NODE_OT_image_process_bake);
  socket_items::ops::make_common_operators<ImageBakeItemsAccessor>();
}

/** \} */

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const bNodeSocket &other = params.other_socket();
  if (other.in_out != SOCK_OUT) {
    return;
  }
  if (other.type == SOCK_RGBA) {
    params.add_item(IFACE_("Image"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("ImageNodeBakeImage"_ustr);
      const char *name = params.socket.name[0] ? params.socket.name : "Image";
      socket_items::add_item_with_name<ImageBakeItemsAccessor>(node, name);
      params.update_and_connect_available_socket(node, UString(name));
    });
  }
  else if (other.type == SOCK_STRING) {
    params.add_item(IFACE_("Path"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("ImageNodeBakeImage"_ustr);
      params.update_and_connect_available_socket(node, "Path"_ustr);
    });
    params.add_item(IFACE_("Prefix"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("ImageNodeBakeImage"_ustr);
      params.update_and_connect_available_socket(node, "Prefix"_ustr);
    });
  }
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeBakeImage"_ustr);
  ntype.ui_name = "Bake Image";
  ntype.ui_description =
      "Write evaluated images to disk when Bake is pressed. Path is the output folder "
      "(// = blend-relative). Prefix is prepended to each image slot name. "
      "Add image slots via the extend socket (like File Output)";
  ntype.nclass = NODE_CLASS_OUTPUT;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.get_compositor_operation = get_compositor_operation;
  ntype.insert_link = node_insert_link;
  ntype.draw_buttons = node_layout;
  ntype.draw_buttons_ex = node_layout_ex;
  ntype.register_operators = node_operators;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  ntype.no_muting = true;
  bke::node_type_storage(ntype, "NodeImageBakeImage", node_free_storage, node_copy_storage);

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_bake_image_cc

namespace blender::nodes {

StructRNA **ImageBakeItemsAccessor::item_srna = &RNA_NodeImageBakeItem;

void ImageBakeItemsAccessor::blend_write_item(BlendWriter *writer, const ItemT &item)
{
  writer->write_string(item.name);
}

void ImageBakeItemsAccessor::blend_read_data_item(BlendDataReader *reader, ItemT &item)
{
  BLO_read_string(reader, &item.name);
}

std::string ImageBakeItemsAccessor::validate_name(const StringRef name)
{
  char file_name[FILE_MAX] = "";
  STRNCPY(file_name, std::string(name).c_str());
  BLI_path_make_safe(file_name);
  return file_name;
}

}  // namespace blender::nodes
