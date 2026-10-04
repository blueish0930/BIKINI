/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * GPU Texture Editor hydraulic / thermal erosion (Gaea Erosion / Erosion2 / Thermal / HydroFix).
 */

#include "BLI_array.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"

#include <cmath>
#include <utility>

#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_zones.hh"

#include "DNA_node_types.h"

#include "GPU_shader.hh"

#include "BLT_translation.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "NOD_node_extra_info.hh"

#include "node_image_terrain_common.hh"
#include "node_image_util.hh"

namespace blender::nodes::node_image_terrain_erosion_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Color>("Height"_ustr)
      .default_value({0.5f, 0.5f, 0.5f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Input heightfield");
  b.add_output<decl::Color>("Height"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Eroded heightfield");
  b.add_output<decl::Color>("Flow"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("Flow map (RG = direction, B = magnitude)");
  b.add_output<decl::Color>("Sediment"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("Deposited sediment");
  b.add_output<decl::Color>("Wear"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("Erosion wear amount");
  b.add_output<decl::Color>("Water"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("Residual water");

  b.add_input<decl::Color>("Mask"_ustr)
      .default_value({1.0f, 1.0f, 1.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Restrict erosion (white = full effect)");
  b.add_input<decl::Float>("Duration"_ustr)
      .default_value(0.45f)
      .min(0.0f)
      .max(1.0f)
      .description("Artistic duration (scales rain / erosion per step)");
  b.add_input<decl::Float>("Iterations"_ustr)
      .default_value(24.0f)
      .min(1.0f)
      .max(128.0f)
      .description(
          "Steps per cook. Outside Simulation use 8–64. Inside Simulation / Repeat this is forced to 1");
  b.add_input<decl::Float>("Scale"_ustr)
      .default_value(1.0f)
      .min(0.05f)
      .max(8.0f)
      .description("Time-step / feature scale");
  b.add_input<decl::Float>("Flow"_ustr)
      .default_value(0.35f)
      .min(0.0f)
      .max(4.0f)
      .description("Rain / water supply");
  b.add_input<decl::Float>("Rock Softness"_ustr)
      .default_value(0.35f)
      .min(0.0f)
      .max(2.0f)
      .description("How easily rock erodes");
  b.add_input<decl::Float>("Thermal"_ustr)
      .default_value(0.35f)
      .min(0.0f)
      .max(2.0f)
      .description("Thermal talus mix (Combined / Thermal types)");
  b.add_input<decl::Float>("Talus"_ustr)
      .default_value(35.0f)
      .min(1.0f)
      .max(80.0f)
      .description("Talus angle in degrees");
  b.add_input<decl::Float>("Capacity"_ustr)
      .default_value(0.6f)
      .min(0.0f)
      .max(8.0f)
      .description("Sediment transport capacity");
  b.add_input<decl::Float>("Evaporation"_ustr)
      .default_value(0.12f)
      .min(0.0f)
      .max(1.0f)
      .description("Water evaporation per step");
  b.add_input<decl::Float>("Downcutting"_ustr)
      .default_value(0.25f)
      .min(0.0f)
      .max(4.0f)
      .description("Valley carving (Erosion2)");
  b.add_input<decl::Float>("Orographic"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .max(1.0f)
      .description("Windward rainfall (Erosion2)");
  b.add_input<decl::Vector>("Rain Direction"_ustr)
      .default_value({1.0f, 0.2f, 0.0f})
      .dimensions(3)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Wind / rain direction for orographic rain");
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  ui::Layout &col = layout.column(true);
  col.prop(ptr, "erosion_type", UI_ITEM_NONE, IFACE_("Type"), ICON_NONE);
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = NODE_TERRAIN_ERO_COMBINED;
}

static bool node_is_in_feedback_zone(const bNodeTree &tree, const bNode &node)
{
  const bke::bNodeTreeZones *zones = tree.zones();
  if (zones == nullptr) {
    return false;
  }
  const bke::bNodeTreeZone *zone = zones->get_zone_by_node(node.identifier);
  if (zone == nullptr) {
    return false;
  }
  const bNode *input = zone->input_node();
  if (input == nullptr) {
    return false;
  }
  return input->is_type("GeometryNodeSimulationInput"_ustr) ||
         input->is_type("GeometryNodeRepeatInput"_ustr);
}

static void node_extra_info(NodeExtraInfoParams &params)
{
  params.tree.ensure_topology_cache();
  if (node_is_in_feedback_zone(params.tree, params.node)) {
    NodeExtraInfoRow row;
    row.text = RPT_("In Simulation: Iterations = 1");
    row.tooltip = TIP_(
        "Accumulate one GPU step per frame. Multi-step erosion belongs outside the zone");
    row.icon = ICON_INFO;
    params.rows.append(std::move(row));
  }
}

using namespace blender::compositor;
using namespace blender::nodes::terrain;

class TerrainErosionOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    const Result &height = this->get_input("Height");
    if (!height.is_single_value() && height.is_allocated()) {
      return height.domain();
    }
    return this->context().get_compositing_domain();
  }

  int erosion_type() const
  {
    return math::clamp(int(this->node().custom1), 0, 7);
  }

  bool shaders_ok() const
  {
    static const char *names[] = {
        "compositor_terrain_pack",
        "compositor_terrain_erosion_flux",
        "compositor_terrain_erosion_apply",
        "compositor_terrain_thermal",
        "compositor_terrain_hydrofix",
        "compositor_terrain_outputs",
    };
    for (const char *name : names) {
      if (try_shader(this->context(), name) == nullptr) {
        return false;
      }
    }
    return true;
  }

  void execute() override
  {
    if (this->context().use_gpu() && this->shaders_ok()) {
      this->execute_gpu();
      return;
    }
    this->execute_cpu();
  }

  void execute_gpu()
  {
    const Domain domain = this->compute_domain();
    const int2 size = domain.data_size;
    const int type = this->erosion_type();
    const float duration = math::clamp(read_float(this->get_input("Duration"), 0.45f), 0.0f, 1.0f);
    int iterations = math::clamp(int(read_float(this->get_input("Iterations"), 24.0f)), 1, 128);
    if (type == NODE_TERRAIN_ERO_EASY || type == NODE_TERRAIN_ERO_WIZARD) {
      iterations = math::clamp(int(4.0f + duration * 48.0f), 4, 64);
    }
    if (is_plausible_pointer(this->node().runtime) &&
        is_plausible_pointer(this->node().runtime->owner_tree) &&
        node_is_in_feedback_zone(*this->node().runtime->owner_tree, this->node()))
    {
      iterations = 1;
    }
    const float scale = math::max(read_float(this->get_input("Scale"), 1.0f), 0.05f);
    float rain = read_float(this->get_input("Flow"), 0.35f) * duration;
    float softness = read_float(this->get_input("Rock Softness"), 0.35f);
    float thermal = read_float(this->get_input("Thermal"), 0.35f);
    float talus = read_float(this->get_input("Talus"), 35.0f);
    float capacity = read_float(this->get_input("Capacity"), 0.6f);
    float evap = read_float(this->get_input("Evaporation"), 0.12f);
    float downcut = read_float(this->get_input("Downcutting"), 0.25f);
    float oro = read_float(this->get_input("Orographic"), 0.0f);
    const float3 rain_d = read_float3(this->get_input("Rain Direction"), float3(1.0f, 0.2f, 0.0f));
    const float2 rain_dir(rain_d.x, rain_d.y);
    const float dt = 0.08f * scale;

    if (type == NODE_TERRAIN_ERO_HYDRAULIC) {
      thermal = 0.0f;
    }
    else if (type == NODE_TERRAIN_ERO_HYDRAULIC2) {
      downcut *= 1.8f;
      capacity *= 1.3f;
      oro = math::max(oro, 0.35f);
    }
    else if (type == NODE_TERRAIN_ERO_THERMAL || type == NODE_TERRAIN_ERO_THERMAL2) {
      rain = 0.0f;
    }
    else if (type == NODE_TERRAIN_ERO_EASY) {
      downcut = 0.15f;
      oro = 0.0f;
    }
    else if (type == NODE_TERRAIN_ERO_WIZARD) {
      downcut = 0.4f;
      thermal = math::max(thermal, 0.4f);
    }

    Result field_a = this->context().create_result(ResultType::Color);
    Result field_b = this->context().create_result(ResultType::Color);
    Result flux_a = this->context().create_result(ResultType::Color);
    Result flux_b = this->context().create_result(ResultType::Color);
    field_a.allocate_texture(domain, false);
    field_b.allocate_texture(domain, false);
    flux_a.allocate_texture(domain, false);
    flux_b.allocate_texture(domain, false);
    const float4 zero(0.0f);
    GPU_texture_clear(flux_a, GPU_DATA_FLOAT, &zero);
    GPU_texture_clear(flux_b, GPU_DATA_FLOAT, &zero);

    Result &height_in = this->get_input("Height");
    Result &mask_in = this->get_input("Mask");
    {
      gpu::Shader *pack = this->context().get_shader("compositor_terrain_pack");
      GPU_shader_bind(pack);
      GPU_shader_uniform_2iv(pack, "domain_size", size);
      gpu::Texture *ht = height_in.bind_as_texture_or_single_value(pack, "height_tx");
      field_a.bind_as_image(pack, "field_img");
      compute_dispatch_threads_at_least(pack, size);
      field_a.unbind_as_image();
      height_in.unbind_as_texture_or_single_value(ht);
      GPU_shader_unbind();
      gpu_barrier();
    }

    Result *field_src = &field_a;
    Result *field_dst = &field_b;
    Result *flux_src = &flux_a;
    Result *flux_dst = &flux_b;

    auto dispatch_img = [&](const char *name, Result &out, auto &&bind) {
      gpu::Shader *shader = this->context().get_shader(name);
      GPU_shader_bind(shader);
      GPU_shader_uniform_2iv(shader, "domain_size", size);
      bind(shader);
      out.bind_as_image(shader, "field_img");
      compute_dispatch_threads_at_least(shader, size);
      out.unbind_as_image();
      GPU_shader_unbind();
      gpu_barrier();
    };

    for (int i = 0; i < iterations; i++) {
      if (type != NODE_TERRAIN_ERO_THERMAL && type != NODE_TERRAIN_ERO_THERMAL2 &&
          type != NODE_TERRAIN_ERO_HYDROFIX)
      {
        gpu::Shader *flux_sh = this->context().get_shader("compositor_terrain_erosion_flux");
        GPU_shader_bind(flux_sh);
        GPU_shader_uniform_2iv(flux_sh, "domain_size", size);
        GPU_shader_uniform_1f(flux_sh, "dt", dt);
        GPU_shader_uniform_1f(flux_sh, "gravity", 9.8f);
        field_src->bind_as_texture(flux_sh, "field_tx");
        flux_src->bind_as_texture(flux_sh, "flux_tx");
        flux_dst->bind_as_image(flux_sh, "flux_img");
        compute_dispatch_threads_at_least(flux_sh, size);
        flux_dst->unbind_as_image();
        field_src->unbind_as_texture();
        flux_src->unbind_as_texture();
        GPU_shader_unbind();
        gpu_barrier();
        std::swap(flux_src, flux_dst);

        gpu::Shader *apply = this->context().get_shader("compositor_terrain_erosion_apply");
        GPU_shader_bind(apply);
        GPU_shader_uniform_2iv(apply, "domain_size", size);
        GPU_shader_uniform_1f(apply, "dt", dt);
        GPU_shader_uniform_1f(apply, "rain", rain);
        GPU_shader_uniform_1f(apply, "evaporation", evap);
        GPU_shader_uniform_1f(apply, "capacity", capacity);
        GPU_shader_uniform_1f(apply, "solubility", softness);
        GPU_shader_uniform_1f(apply, "deposition", 0.45f);
        GPU_shader_uniform_1f(apply, "downcutting", downcut);
        GPU_shader_uniform_1f(
            apply, "thermal", (type == NODE_TERRAIN_ERO_COMBINED || type == NODE_TERRAIN_ERO_WIZARD) ? thermal : 0.0f);
        GPU_shader_uniform_1f(apply, "talus", talus);
        GPU_shader_uniform_1f(apply, "orographic", oro);
        GPU_shader_uniform_2fv(apply, "rain_dir", rain_dir);
        field_src->bind_as_texture(apply, "field_tx");
        flux_src->bind_as_texture(apply, "flux_tx");
        gpu::Texture *mt = mask_in.bind_as_texture_or_single_value(apply, "mask_tx");
        field_dst->bind_as_image(apply, "field_img");
        compute_dispatch_threads_at_least(apply, size);
        field_dst->unbind_as_image();
        field_src->unbind_as_texture();
        flux_src->unbind_as_texture();
        mask_in.unbind_as_texture_or_single_value(mt);
        GPU_shader_unbind();
        gpu_barrier();
        std::swap(field_src, field_dst);
      }

      if (type == NODE_TERRAIN_ERO_THERMAL || type == NODE_TERRAIN_ERO_THERMAL2 ||
          type == NODE_TERRAIN_ERO_COMBINED || type == NODE_TERRAIN_ERO_WIZARD)
      {
        gpu::Shader *th = this->context().get_shader("compositor_terrain_thermal");
        GPU_shader_bind(th);
        GPU_shader_uniform_2iv(th, "domain_size", size);
        GPU_shader_uniform_1f(th, "strength", math::max(thermal, 0.15f));
        GPU_shader_uniform_1f(th, "talus", talus);
        GPU_shader_uniform_1i(th, "corners", type == NODE_TERRAIN_ERO_THERMAL2 ? 1 : 0);
        field_src->bind_as_texture(th, "field_tx");
        gpu::Texture *mt = mask_in.bind_as_texture_or_single_value(th, "mask_tx");
        field_dst->bind_as_image(th, "field_img");
        compute_dispatch_threads_at_least(th, size);
        field_dst->unbind_as_image();
        field_src->unbind_as_texture();
        mask_in.unbind_as_texture_or_single_value(mt);
        GPU_shader_unbind();
        gpu_barrier();
        std::swap(field_src, field_dst);
      }

      if (type == NODE_TERRAIN_ERO_HYDROFIX) {
        dispatch_img("compositor_terrain_hydrofix", *field_dst, [&](gpu::Shader *shader) {
          GPU_shader_uniform_1f(shader, "amount", math::clamp(duration, 0.05f, 1.0f));
          GPU_shader_uniform_1f(shader, "slope", 0.8f);
          field_src->bind_as_texture(shader, "field_tx");
        });
        field_src->unbind_as_texture();
        std::swap(field_src, field_dst);
      }
    }

    gpu::Shader *out = this->context().get_shader("compositor_terrain_outputs");
    GPU_shader_bind(out);
    GPU_shader_uniform_2iv(out, "domain_size", size);
    field_src->bind_as_texture(out, "field_tx");
    flux_src->bind_as_texture(out, "flux_tx");
    Result &h_out = this->get_result("Height");
    Result &flow_out = this->get_result("Flow");
    Result &sed_out = this->get_result("Sediment");
    Result &wear_out = this->get_result("Wear");
    Result &water_out = this->get_result("Water");
    h_out.allocate_texture(domain);
    flow_out.allocate_texture(domain);
    sed_out.allocate_texture(domain);
    wear_out.allocate_texture(domain);
    water_out.allocate_texture(domain);
    h_out.bind_as_image(out, "height_img");
    flow_out.bind_as_image(out, "flow_img");
    sed_out.bind_as_image(out, "sediment_img");
    wear_out.bind_as_image(out, "wear_img");
    water_out.bind_as_image(out, "water_img");
    compute_dispatch_threads_at_least(out, size);
    h_out.unbind_as_image();
    flow_out.unbind_as_image();
    sed_out.unbind_as_image();
    wear_out.unbind_as_image();
    water_out.unbind_as_image();
    field_src->unbind_as_texture();
    flux_src->unbind_as_texture();
    GPU_shader_unbind();

    field_a.release();
    field_b.release();
    flux_a.release();
    flux_b.release();
  }

  void execute_cpu()
  {
    const Domain domain = this->compute_domain();
    Result height_store = this->context().create_result(this->get_input("Height").type());
    Result mask_store = this->context().create_result(this->get_input("Mask").type());
    const Result *height_cpu = ensure_cpu(this->context(), this->get_input("Height"), height_store);
    const Result *mask_cpu = ensure_cpu(this->context(), this->get_input("Mask"), mask_store);
    const int2 size = domain.data_size;
    const int n = size.x * size.y;
    Array<float> H(n), W(n), S(n), Wear(n);
    Array<float> H2(n), W2(n), S2(n), Wear2(n);
    Array<float> fx(n * 4, 0.0f);
    auto at = [&](int x, int y) {
      x = math::clamp(x, 0, size.x - 1);
      y = math::clamp(y, 0, size.y - 1);
      return y * size.x + x;
    };
    for (int y = 0; y < size.y; y++) {
      for (int x = 0; x < size.x; x++) {
        const int i = at(x, y);
        Color hc(0.5f, 0.5f, 0.5f, 1.0f);
        if (height_cpu->is_single_value()) {
          hc = height_cpu->get_single_value_default<Color>();
        }
        else if (height_cpu->is_allocated()) {
          hc = height_cpu->load_pixel<Color>(int2(x, y));
        }
        H[i] = luma(hc);
        W[i] = 0.0f;
        S[i] = 0.0f;
        Wear[i] = 0.0f;
      }
    }
    const float duration = math::clamp(read_float(this->get_input("Duration"), 0.45f), 0.0f, 1.0f);
    int iterations = math::clamp(int(read_float(this->get_input("Iterations"), 24.0f)), 1, 64);
    if (is_plausible_pointer(this->node().runtime) &&
        is_plausible_pointer(this->node().runtime->owner_tree) &&
        node_is_in_feedback_zone(*this->node().runtime->owner_tree, this->node()))
    {
      iterations = 1;
    }
    const float rain = read_float(this->get_input("Flow"), 0.35f) * duration;
    const float softness = read_float(this->get_input("Rock Softness"), 0.35f);
    const float dt = 0.08f * math::max(read_float(this->get_input("Scale"), 1.0f), 0.05f);
    const float evap = read_float(this->get_input("Evaporation"), 0.12f);
    const float capacity = read_float(this->get_input("Capacity"), 0.6f);
    const int type = this->erosion_type();

    for (int it = 0; it < iterations; it++) {
      if (type != NODE_TERRAIN_ERO_THERMAL && type != NODE_TERRAIN_ERO_HYDROFIX) {
        parallel_for(size, [&](const int2 texel) {
          const int i = at(texel.x, texel.y);
          const float h = H[i] + W[i];
          const int nb[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
          float outf[4];
          float sum = 0.0f;
          for (int k = 0; k < 4; k++) {
            const int j = at(texel.x + nb[k][0], texel.y + nb[k][1]);
            const float dh = h - (H[j] + W[j]);
            outf[k] = math::max(0.0f, fx[i * 4 + k] + dt * 9.8f * dh);
            sum += outf[k];
          }
          float scale = 1.0f;
          if (sum * dt > W[i] && sum > 1.0e-8f) {
            scale = W[i] / (sum * dt);
          }
          for (int k = 0; k < 4; k++) {
            fx[i * 4 + k] = outf[k] * scale;
          }
        });
        parallel_for(size, [&](const int2 texel) {
          const int i = at(texel.x, texel.y);
          const int L = at(texel.x - 1, texel.y);
          const int R = at(texel.x + 1, texel.y);
          const int D = at(texel.x, texel.y - 1);
          const int U = at(texel.x, texel.y + 1);
          const float inflow = fx[L * 4 + 1] + fx[R * 4 + 0] + fx[D * 4 + 3] + fx[U * 4 + 2];
          const float outflow = fx[i * 4 + 0] + fx[i * 4 + 1] + fx[i * 4 + 2] + fx[i * 4 + 3];
          float water = math::max(W[i] + (inflow - outflow) * dt, 0.0f);
          Color mc(1.0f, 1.0f, 1.0f, 1.0f);
          if (mask_cpu->is_single_value()) {
            mc = mask_cpu->get_single_value_default<Color>();
          }
          else if (mask_cpu->is_allocated()) {
            mc = mask_cpu->load_pixel<Color>(int2(texel.x, texel.y));
          }
          const float mask = luma(mc);
          water += rain * mask * dt;
          const float slope = 0.5f *
                               math::length(float2(H[R] - H[L], H[U] - H[D]));
          const float cap = capacity * (0.15f + slope);
          float sed = S[i];
          const float erode = softness * math::max(cap - sed, 0.0f) * mask;
          const float dep = 0.45f * math::max(sed - cap, 0.0f);
          H2[i] = math::max(H[i] + (dep - erode) * dt, 0.0f);
          S2[i] = math::max(sed - dep + erode, 0.0f);
          Wear2[i] = Wear[i] + erode * dt;
          W2[i] = water * math::max(1.0f - evap * dt, 0.0f);
        });
        std::swap(H, H2);
        std::swap(W, W2);
        std::swap(S, S2);
        std::swap(Wear, Wear2);
      }
      if (type == NODE_TERRAIN_ERO_THERMAL || type == NODE_TERRAIN_ERO_COMBINED ||
          type == NODE_TERRAIN_ERO_HYDROFIX)
      {
        const float rest = std::tan(read_float(this->get_input("Talus"), 35.0f) * 0.01745329252f) /
                            float(math::max(size.x, 1));
        parallel_for(size, [&](const int2 texel) {
          const int i = at(texel.x, texel.y);
          float h = H[i];
          if (type == NODE_TERRAIN_ERO_HYDROFIX) {
            float mn = h;
            mn = math::min(mn, H[at(texel.x - 1, texel.y)]);
            mn = math::min(mn, H[at(texel.x + 1, texel.y)]);
            mn = math::min(mn, H[at(texel.x, texel.y - 1)]);
            mn = math::min(mn, H[at(texel.x, texel.y + 1)]);
            if (h + 1.0e-6f < mn) {
              h = math::interpolate(h, mn - rest, duration);
            }
          }
          else {
            const float hn[4] = {H[at(texel.x - 1, texel.y)],
                                 H[at(texel.x + 1, texel.y)],
                                 H[at(texel.x, texel.y - 1)],
                                 H[at(texel.x, texel.y + 1)]};
            float delta = 0.0f;
            for (int k = 0; k < 4; k++) {
              const float dh_to = h - hn[k];
              if (dh_to > rest) {
                delta -= (dh_to - rest) * 0.12f;
              }
              const float dh_from = hn[k] - h;
              if (dh_from > rest) {
                delta += (dh_from - rest) * 0.12f;
              }
            }
            h = math::max(h + delta * math::max(read_float(this->get_input("Thermal"), 0.35f), 0.15f),
                          0.0f);
          }
          H2[i] = h;
        });
        H = H2;
      }
    }

    Result h_cpu = this->context().create_result(ResultType::Color);
    Result flow_cpu = this->context().create_result(ResultType::Color);
    Result sed_cpu = this->context().create_result(ResultType::Color);
    Result wear_cpu = this->context().create_result(ResultType::Color);
    Result water_cpu = this->context().create_result(ResultType::Color);
    h_cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    flow_cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    sed_cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    wear_cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    water_cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    parallel_for(size, [&](const int2 texel) {
      const int i = at(texel.x, texel.y);
      const float2 vel(fx[i * 4 + 1] - fx[i * 4 + 0], fx[i * 4 + 3] - fx[i * 4 + 2]);
      h_cpu.store_pixel(texel, gray(H[i]));
      flow_cpu.store_pixel(texel, Color(vel.x * 0.5f + 0.5f, vel.y * 0.5f + 0.5f, math::length(vel), 1.0f));
      sed_cpu.store_pixel(texel, gray(S[i]));
      wear_cpu.store_pixel(texel, gray(Wear[i]));
      water_cpu.store_pixel(texel, gray(W[i]));
    });
    share_cpu_to_output(this->context(), this->get_result("Height"), h_cpu);
    share_cpu_to_output(this->context(), this->get_result("Flow"), flow_cpu);
    share_cpu_to_output(this->context(), this->get_result("Sediment"), sed_cpu);
    share_cpu_to_output(this->context(), this->get_result("Wear"), wear_cpu);
    share_cpu_to_output(this->context(), this->get_result("Water"), water_cpu);
    if (height_store.is_allocated()) {
      height_store.release();
    }
    if (mask_store.is_allocated()) {
      mask_store.release();
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new TerrainErosionOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;
  img_node_type_base(&ntype, "ImageNodeTerrainErosion"_ustr, IMG_NODE_TERRAIN_EROSION);
  ntype.ui_name = "Terrain Erosion";
  ntype.ui_description =
      "Gaea-style hydraulic and thermal erosion. HydroFix is drainage prep. "
      "Inside Simulation / Repeat, Iterations is forced to 1";
  ntype.nclass = NODE_CLASS_OP_FILTER;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.get_extra_info = node_extra_info;
  ntype.get_compositor_operation = get_compositor_operation;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_terrain_erosion_cc
