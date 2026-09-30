/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* IMAGE_NODES: write an evaluated image into a named Blender Image datablock for reuse. */

#include <cstring>

#include "BLI_math_base.hh"
#include "BLI_string_ref.hh"
#include "BLI_threads.hh"

#include "BKE_image.hh"
#include "BKE_image_gpu.hh"
#include "BKE_image_paint_layers.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"

#include "DNA_image_types.h"

#include "DEG_depsgraph.hh"

#include "IMB_colormanagement.hh"
#include "IMB_imbuf.hh"
#include "IMB_partial_update.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "COM_input_descriptor.hh"
#include "COM_node_operation.hh"
#include "COM_realize_on_domain_operation.hh"
#include "COM_result.hh"
#include "COM_simple_operation.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_file_output_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Color>("Image"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Image written into the named Image datablock");
  b.add_input<decl::String>("Name"_ustr)
      .default_value("GPU Texture Editor Output")
      .description("Name of the Blender Image datablock to create or update (appears in Image list)");
}

using namespace blender::compositor;

class FileOutputOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    Result &image_result = this->get_input("Image");
    const std::string name = this->get_input("Name").get_single_value_default<std::string>();
    if (name.empty()) {
      return;
    }

    /* Bake domain transforms (Scale/Rotate/Translate/Transform) onto compositing domain
     * before writing pixels — same contract as ImageNodesContext::write_viewer. */
    Result *result_to_write = &image_result;
    SimpleOperation *realization_operation = nullptr;
    Result realize_input = this->context().create_result(image_result.type(),
                                                         image_result.precision());
    if (!image_result.is_single_value() && image_result.is_allocated()) {
      const InputDescriptor input_descriptor = {image_result.type(),
                                                InputRealizationMode::OperationDomain};
      realization_operation = RealizeOnDomainOperation::construct_if_needed(
          this->context(),
          image_result,
          input_descriptor,
          this->context().get_compositing_domain());
      if (realization_operation) {
        realize_input.share_data(image_result);
        realization_operation->map_input_to_result(&realize_input);
        realization_operation->evaluate();
        result_to_write = &realization_operation->get_result();
      }
    }

    Main &bmain = const_cast<Main &>(this->context().get_main());
    /* Single-value Color expands to the editor texture resolution (compositing domain). */
    const int2 size = result_to_write->is_single_value() ?
                          math::max(int2(1), this->context().get_compositing_domain().data_size) :
                          math::max(int2(1), result_to_write->domain().data_size);
    if (size.x < 1 || size.y < 1) {
      if (realization_operation) {
        realization_operation->get_result().release();
        delete realization_operation;
      }
      return;
    }

    Image *image = nullptr;
    for (Image *ima = static_cast<Image *>(bmain.images.first()); ima;
         ima = static_cast<Image *>(ima->id.next))
    {
      if (STREQ(ima->id.name + 2, name.c_str())) {
        image = ima;
        break;
      }
    }
    if (image == nullptr) {
      /* Same creation path as bpy.data.images.new — appears in Image lists. */
      const float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
      image = BKE_image_add_generated(&bmain,
                                      uint(size.x),
                                      uint(size.y),
                                      name.c_str(),
                                      32,
                                      true,
                                      0,
                                      black,
                                      false,
                                      false,
                                      false);
      if (image) {
        /* Keep visible in the Image list even with no external users. */
        id_us_ensure_real(&image->id);
        id_fake_user_set(&image->id);
      }
    }
    if (!image) {
      if (realization_operation) {
        realization_operation->get_result().release();
        delete realization_operation;
      }
      return;
    }

    id_us_ensure_real(&image->id);

    ImageUser image_user = {nullptr};
    BLI_thread_lock(LOCK_DRAW_IMAGE);

    void *lock = nullptr;
    ImBuf *image_buffer = BKE_image_acquire_ibuf(image, &image_user, &lock);
    if (!image_buffer) {
      BLI_thread_unlock(LOCK_DRAW_IMAGE);
      if (realization_operation) {
        realization_operation->get_result().release();
        delete realization_operation;
      }
      return;
    }

    if (int2(image_buffer->x, image_buffer->y) != size) {
      IMB_free_byte_pixels(image_buffer);
      IMB_free_float_pixels(image_buffer);
      IMB_free_gpu_textures(image_buffer);
      image_buffer->x = size.x;
      image_buffer->y = size.y;
      /* Keep generation metadata in sync for the Image Editor. */
      if (ImageTile *tile = static_cast<ImageTile *>(image->tiles.first())) {
        tile->gen_x = size.x;
        tile->gen_y = size.y;
      }
    }

    IMB_alloc_float_pixels(image_buffer, 4, false);
    float *dst = image_buffer->float_data_for_write();
    const int pixel_count = size.x * size.y;
    std::memset(dst, 0, size_t(pixel_count) * 4 * sizeof(float));

    if (result_to_write->is_single_value()) {
      const Color color = result_to_write->get_single_value_default<Color>();
      for (int i = 0; i < pixel_count; i++) {
        dst[i * 4 + 0] = color.r;
        dst[i * 4 + 1] = color.g;
        dst[i * 4 + 2] = color.b;
        dst[i * 4 + 3] = color.a;
      }
    }
    else if (result_to_write->is_allocated()) {
      if (this->context().use_gpu()) {
        Result cpu_result = result_to_write->download_to_cpu();
        if (cpu_result.cpu_data().data()) {
          std::memcpy(dst, cpu_result.cpu_data().data(), size_t(pixel_count) * 4 * sizeof(float));
        }
        cpu_result.release();
      }
      else if (result_to_write->cpu_data().data()) {
        std::memcpy(
            dst, result_to_write->cpu_data().data(), size_t(pixel_count) * 4 * sizeof(float));
      }
    }

    image->flag |= IMA_VIEW_AS_RENDER;
    const char *to_colorspace = IMB_colormanagement_role_colorspace_name_get(
        COLOR_ROLE_SCENE_LINEAR);
    IMB_colormanagement_assign_float_colorspace(image_buffer, to_colorspace);

    IMB_mark_dirty(image_buffer);
    IMB_partial_update_mark_full(image_buffer);
    BKE_image_release_ibuf(image, image_buffer, lock);
    BLI_thread_unlock(LOCK_DRAW_IMAGE);

    /* Drop cached GPU textures so materials / Image Editor / 3D View re-upload
     * from the CPU buffer. Leaving them made other editors show the last cook. */
    BKE_image_free_gpu_texture_caches(image);

    if (realization_operation) {
      realization_operation->get_result().release();
      delete realization_operation;
    }

    /* Shaders sample GPU textures at draw time, so freeing the GPU cache is enough
     * for materials. Geometry nodes / compositor / modifiers evaluate in the
     * depsgraph and depend on GENERIC_DATABLOCK — ID_RECALC_SOURCE alone only
     * tags PARAMETERS. Tag like IMAGE_OT_reload so those consumers recook in the
     * same event cycle (area refresh, then wm_event_do_depsgraph). */
    DEG_id_tag_update(&image->id, 0);
    DEG_id_tag_update(&image->id, ID_RECALC_SOURCE | ID_RECALC_EDITORS);
    WM_main_add_notifier(NC_IMAGE | NA_EDITED, image);
    WM_main_add_notifier(NC_IMAGE | NA_ADDED, image);
    /* Texture Paint layers / masks that use this Image as a live source keep a
     * composited cache. Invalidate and rebuild those stacks so the 3D view and
     * layer panel match the new pixels. */
    BKE_image_paint_layers_on_source_changed(bmain, *image);
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new FileOutputOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeFileOutput"_ustr);
  ntype.ui_name = "Image Output";
  ntype.ui_description =
      "Write the input into a named Image datablock that appears in the Image list for reuse";
  ntype.nclass = NODE_CLASS_OUTPUT;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;
  ntype.no_muting = true;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_file_output_cc
