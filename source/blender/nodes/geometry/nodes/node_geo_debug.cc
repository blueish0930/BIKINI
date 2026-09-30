/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <fmt/format.h>

#include "BKE_attribute_legacy_convert.hh"
#include "BKE_type_conversions.hh"

#include "GEO_foreach_geometry.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"
#include "NOD_socket_usage_inference.hh"

#include "RNA_access.hh"
#include "RNA_enum_types.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_debug_cc {

NODE_STORAGE_FUNCS(NodeGeometryDebug)

/**
 * One Debug node owns one uniquely named layer. The compute-context hash distinguishes group uses,
 * modifiers and nested executions; the persistent node identifier distinguishes nodes in a tree.
 * This avoids collisions when branches are joined.
 */
static std::string debug_layer_prefix(const GeoNodeExecParams &params)
{
  const ComputeContextHash &context_hash = params.user_data()->compute_context->hash();
  return fmt::format(".debug_{:016x}{:016x}_{:x}",
                     context_hash.v1,
                     context_hash.v2,
                     uint32_t(params.node().identifier));
}

static bool debug_data_type_supported(const eCustomDataType type)
{
  return ELEM(type,
              CD_PROP_FLOAT,
              CD_PROP_FLOAT3,
              CD_PROP_COLOR,
              CD_PROP_BOOL,
              CD_PROP_INT32,
              CD_PROP_STRING,
              CD_PROP_QUATERNION,
              CD_PROP_FLOAT4X4);
}

/**
 * Types that expose Color Overlay + Opacity in the node UI.
 * Vector only in Color mode (Arrows mode has no opacity row).
 */
static bool type_shows_color_overlay_ui(const eCustomDataType type, const int8_t vector_display)
{
  if (type == CD_PROP_FLOAT3) {
    return vector_display == NODE_GEO_DEBUG_VECTOR_COLOR;
  }
  return ELEM(type, CD_PROP_FLOAT, CD_PROP_COLOR, CD_PROP_BOOL, CD_PROP_INT32);
}

/**
 * Whether this layer should currently composite into `.debug_color`.
 * Vector only colors when display mode is Color (not Arrows).
 */
static bool type_applies_color_overlay(const eCustomDataType type, const int8_t vector_display)
{
  if (type == CD_PROP_FLOAT3) {
    return vector_display == NODE_GEO_DEBUG_VECTOR_COLOR;
  }
  return ELEM(type, CD_PROP_FLOAT, CD_PROP_COLOR, CD_PROP_BOOL, CD_PROP_INT32);
}

static bool uses_arrow_display(const eCustomDataType type,
                               const int8_t vector_display,
                               const int8_t matrix_display)
{
  if (type == CD_PROP_FLOAT3) {
    return vector_display == NODE_GEO_DEBUG_VECTOR_ARROWS;
  }
  if (type == CD_PROP_FLOAT4X4) {
    return matrix_display == NODE_GEO_DEBUG_MATRIX_AXES;
  }
  return false;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_default_layout();

  const bNode *node = b.node_or_null();
  const NodeGeometryDebug *storage = node ? &node_storage(*node) : nullptr;
  const eCustomDataType data_type = storage ? eCustomDataType(storage->data_type) :
                                              CD_PROP_FLOAT;
  const int8_t vector_display = storage ? storage->vector_display : NODE_GEO_DEBUG_VECTOR_ARROWS;
  const int8_t matrix_display = storage ? storage->matrix_display :
                                          NODE_GEO_DEBUG_MATRIX_COMPONENTS;
  const bool show_color_ui = storage && type_shows_color_overlay_ui(data_type, vector_display);
  const bool show_arrow_controls = storage &&
                                   uses_arrow_display(data_type, vector_display, matrix_display);
  const bool show_arrow_color = storage && data_type == CD_PROP_FLOAT3 &&
                                vector_display == NODE_GEO_DEBUG_VECTOR_ARROWS;

  b.add_input<decl::Geometry>("Geometry"_ustr)
      .description(
          "Geometry that receives this node's independent debug preview without changing "
          "user-visible named attributes");
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Same geometry with this Debug layer attached");

  b.add_input<decl::Bool>("Selection"_ustr)
      .default_value(true)
      .hide_value()
      .evaluated_geometry_field()
      .description("Elements on which this Debug layer is visible");

  /* Always declare Value (and optional rows). Using if-add/remove would destroy sockets when
   * switching data_type / display mode and reset defaults + drop links. Prefer .available(). */
  b.add_input(data_type, "Value"_ustr)
      .evaluated_geometry_field()
      .description("Field displayed by this Debug node");

  auto &text_size = b.add_input<decl::Float>("Text Size"_ustr)
                        .default_value(11.0f)
                        .min(1.0f)
                        .max(64.0f)
                        .subtype(PROP_PIXEL)
                        .description(
                            "Font size for this Debug layer. Only used when Text Overlay is "
                            "enabled")
                        .custom_draw([](CustomSocketDrawParams &params) {
                          const NodeGeometryDebug &storage = node_storage(params.node);
                          const bool text_overlay_on = storage.use_text_overlay != 0;

                          ui::Layout &row = params.layout.row(true);

                          ui::Layout &toggle_col = row.row(true);
                          /* Empty label: checkbox only, no "Text Overlay" text. */
                          toggle_col.prop(
                              &params.node_ptr, "use_text_overlay", UI_ITEM_NONE, "", ICON_NONE);

                          ui::Layout &size_col = row.row(true);
                          if (!text_overlay_on) {
                            size_col.active_set(false);
                          }
                          size_col.prop(&params.socket_ptr,
                                        "default_value",
                                        ui::ITEM_R_SLIDER | ui::ITEM_R_SPLIT_EMPTY_NAME,
                                        IFACE_("Text Size"),
                                        ICON_NONE);
                        })
                        .usage_inference(
                            [](const socket_usage_inference::SocketUsageParams &params)
                                -> std::optional<bool> {
                              if (params.socket.is_input()) {
                                if (const std::optional<bool> any_output_used =
                                        params.any_output_is_used())
                                {
                                  if (!*any_output_used) {
                                    return false;
                                  }
                                }
                                else {
                                  return std::nullopt;
                                }
                              }
                              const NodeGeometryDebug &storage = node_storage(params.node);
                              if (uses_arrow_display(eCustomDataType(storage.data_type),
                                                     storage.vector_display,
                                                     storage.matrix_display))
                              {
                                return false;
                              }
                              return storage.use_text_overlay != 0;
                            });

  auto &opacity = b.add_input<decl::Float>("Opacity"_ustr)
                      .default_value(1.0f)
                      .min(0.0f)
                      .max(1.0f)
                      .subtype(PROP_FACTOR)
                      .description(
                          "Color overlay opacity. Values of 0 leave any previous Debug color "
                          "unchanged. Only used when Color Overlay is enabled. Selection does not "
                          "affect color")
                      .custom_draw([](CustomSocketDrawParams &params) {
                        const NodeGeometryDebug &storage = node_storage(params.node);
                        const bool color_overlay_on = storage.use_color_overlay != 0;

                        ui::Layout &row = params.layout.row(true);

                        ui::Layout &toggle_col = row.row(true);
                        toggle_col.prop(
                            &params.node_ptr, "use_color_overlay", UI_ITEM_NONE, "", ICON_NONE);

                        ui::Layout &opacity_col = row.row(true);
                        if (!color_overlay_on) {
                          opacity_col.active_set(false);
                        }
                        opacity_col.prop(&params.socket_ptr,
                                         "default_value",
                                         ui::ITEM_R_SLIDER | ui::ITEM_R_SPLIT_EMPTY_NAME,
                                         IFACE_("Opacity"),
                                         ICON_NONE);
                      })
                      .usage_inference(
                          [](const socket_usage_inference::SocketUsageParams &params)
                              -> std::optional<bool> {
                            if (params.socket.is_input()) {
                              if (const std::optional<bool> any_output_used =
                                      params.any_output_is_used())
                              {
                                if (!*any_output_used) {
                                  return false;
                                }
                              }
                              else {
                                return std::nullopt;
                              }
                            }
                            const NodeGeometryDebug &storage = node_storage(params.node);
                            return storage.use_color_overlay != 0 &&
                                   type_applies_color_overlay(eCustomDataType(storage.data_type),
                                                              storage.vector_display);
                          });

  auto &normalize = b.add_input<decl::Bool>("Normalize Length"_ustr)
                        .default_value(false)
                        .description(
                            "Normalize arrow directions to unit length before applying Arrow "
                            "Scale");
  auto &arrow_scale = b.add_input<decl::Float>("Arrow Scale"_ustr)
                          .default_value(1.0f)
                          .min(0.0f)
                          .description("Multiply arrow length after optional normalization");
  auto &arrow_color = b.add_input<decl::Color>("Arrow Color"_ustr)
                          .default_value({1.0f, 0.85f, 0.15f, 1.0f})
                          .evaluated_geometry_field()
                          .description(
                              "Per-element arrow color field (Color on the same domain as Value)");

  b.add_input<decl::Bool>("Exclude Backfaces"_ustr)
      .default_value(true)
      .description(
          "Hide text when its geometry position is occluded by the same mesh from the current "
          "view");

  /* When building the static declaration (node == null), leave all sockets available so link
   * search can list them. On a real node, hide rows that do not apply — without destroying them. */
  if (storage != nullptr) {
    text_size.available(!show_arrow_controls);
    opacity.available(show_color_ui);
    normalize.available(show_arrow_controls);
    arrow_scale.available(show_arrow_controls);
    arrow_color.available(show_arrow_color);
  }
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
  layout.prop(ptr, "domain", UI_ITEM_NONE, "", ICON_NONE);
  const eCustomDataType data_type = eCustomDataType(RNA_enum_get(ptr, "data_type"));
  if (data_type == CD_PROP_FLOAT3) {
    layout.prop(ptr, "vector_display", UI_ITEM_NONE, "", ICON_NONE);
  }
  else if (data_type == CD_PROP_FLOAT4X4) {
    layout.prop(ptr, "matrix_display", UI_ITEM_NONE, "", ICON_NONE);
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryDebug *data = MEM_new<NodeGeometryDebug>(__func__);
  data->data_type = CD_PROP_FLOAT;
  data->domain = int8_t(bke::AttrDomainSelection::Auto);
  data->exclude_backfaces = 1;
  data->use_color_overlay = 1;
  data->use_text_overlay = 0; /* Text off by default; enable per node when needed. */
  data->text_size = 11.0f;
  data->matrix_display = NODE_GEO_DEBUG_MATRIX_COMPONENTS;
  data->vector_display = NODE_GEO_DEBUG_VECTOR_ARROWS;
  node->storage = data;
}

static void node_update(bNodeTree * /*ntree*/, bNode *node)
{
  NodeGeometryDebug &storage = node_storage(*node);
  if (!debug_data_type_supported(eCustomDataType(storage.data_type))) {
    storage.data_type = CD_PROP_FLOAT;
  }
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const NodeDeclaration &declaration = *params.node_type().static_declaration;
  search_link_ops_for_declarations(params, declaration.inputs);
  search_link_ops_for_declarations(params, declaration.outputs);

  if (params.in_out() == SOCK_IN) {
    const std::optional<eCustomDataType> type = bke::socket_type_to_custom_data_type(
        params.other_socket().type);
    if (type && debug_data_type_supported(*type)) {
      params.add_item(IFACE_("Value"), [type](LinkSearchOpParams &params) {
        bNode &node = params.add_node("GeometryNodeDebug"_ustr);
        node_storage(node).data_type = *type;
        params.update_and_connect_available_socket(node, "Value"_ustr);
      });
    }
  }
}

static void write_constant_float_attribute(MutableAttributeAccessor attributes,
                                           const StringRef name,
                                           const AttrDomain domain,
                                           const float value)
{
  attributes.remove(name);
  if (bke::SpanAttributeWriter<float> writer =
          attributes.lookup_or_add_for_write_only_span<float>(name, domain))
  {
    writer.span.fill(value);
    writer.finish();
  }
}

static void write_constant_bool_attribute(MutableAttributeAccessor attributes,
                                          const StringRef name,
                                          const AttrDomain domain,
                                          const bool value)
{
  attributes.remove(name);
  if (bke::SpanAttributeWriter<bool> writer =
          attributes.lookup_or_add_for_write_only_span<bool>(name, domain))
  {
    writer.span.fill(value);
    writer.finish();
  }
}

static void write_constant_int_attribute(MutableAttributeAccessor attributes,
                                         const StringRef name,
                                         const AttrDomain domain,
                                         const int value)
{
  attributes.remove(name);
  if (bke::SpanAttributeWriter<int> writer =
          attributes.lookup_or_add_for_write_only_span<int>(name, domain))
  {
    writer.span.fill(value);
    writer.finish();
  }
}

static void composite_debug_color(MutableAttributeAccessor attributes,
                                  const StringRef value_name,
                                  const AttrDomain domain,
                                  const float opacity)
{
  if (opacity <= 0.0f) {
    /* Keep any previous Debug color on this geometry. */
    return;
  }

  /* Color is independent of Selection: always composite on the full domain. */
  const bke::AttributeReader<ColorGeometry4f> source_colors =
      attributes.lookup_or_default<ColorGeometry4f>(
          value_name, domain, ColorGeometry4f(1.0f, 0.0f, 1.0f, 1.0f));

  /* Default ColorGeometry4f is opaque white (1,1,1,1). Blending against that made opacity→0 look
   * like a fade to white. Create new layers as write-only (we fill every element) and alpha-over
   * onto existing layers so opacity only thins the overlay toward the underlying viewport. */
  const bool existed = attributes.contains(".debug_color");
  bke::SpanAttributeWriter<ColorGeometry4f> debug_colors =
      existed ? attributes.lookup_or_add_for_write_span<ColorGeometry4f>(
                    ".debug_color", domain, bke::AttributeInitDefaultValue()) :
                attributes.lookup_or_add_for_write_only_span<ColorGeometry4f>(".debug_color",
                                                                              domain);
  if (!debug_colors) {
    return;
  }

  for (const int i : debug_colors.span.index_range()) {
    const ColorGeometry4f src = source_colors.varray[i];
    /* Viewport attribute viewer uses non-premultiplied RGB + alpha blend
     * (`out_color.a *= opacity`). Opacity only scales source alpha so the overlay fades out. */
    const float src_a = math::clamp(src.a * opacity, 0.0f, 1.0f);

    if (!existed) {
      debug_colors.span[i] = ColorGeometry4f(src.r, src.g, src.b, src_a);
      continue;
    }

    ColorGeometry4f &dst = debug_colors.span[i];
    const float dst_a = math::clamp(dst.a, 0.0f, 1.0f);
    const float out_a = src_a + dst_a * (1.0f - src_a);
    if (out_a <= 1.0e-8f) {
      dst = ColorGeometry4f(0.0f, 0.0f, 0.0f, 0.0f);
      continue;
    }
    /* Non-premultiplied alpha-over: Co = (Cs*As + Cd*Ad*(1-As)) / Ao */
    const float keep = dst_a * (1.0f - src_a);
    const float inv_out_a = 1.0f / out_a;
    dst.r = (src.r * src_a + dst.r * keep) * inv_out_a;
    dst.g = (src.g * src_a + dst.g * keep) * inv_out_a;
    dst.b = (src.b * src_a + dst.b * keep) * inv_out_a;
    dst.a = out_a;
  }
  debug_colors.finish();
}

static bool capture_on_component(GeometryComponent &component,
                                 const StringRef layer_prefix,
                                 const bke::AttrDomainSelection domain_or_auto,
                                 const Field<bool> &selection,
                                 GField field,
                                 const float text_size,
                                 const bool exclude_backfaces,
                                 const bool use_color_overlay,
                                 const bool use_text_overlay,
                                 const float opacity,
                                 const int display_mode,
                                 const bool arrow_normalize,
                                 const float arrow_scale,
                                 const bool has_arrow_color,
                                 const Field<ColorGeometry4f> *arrow_color_field)
{
  std::optional<MutableAttributeAccessor> attributes_opt = component.attributes_for_write();
  if (!attributes_opt) {
    return false;
  }
  MutableAttributeAccessor attributes = *attributes_opt;

  AttrDomain used_domain;
  if (domain_or_auto == bke::AttrDomainSelection::Auto) {
    if (const std::optional<AttrDomain> domain = bke::try_detect_field_domain(component, field)) {
      used_domain = *domain;
    }
    else {
      used_domain = AttrDomain::Point;
    }
  }
  else {
    used_domain = AttrDomain(domain_or_auto);
  }
  if (component.attribute_domain_size(used_domain) == 0) {
    return false;
  }

  const std::string value_name = layer_prefix + "_value";
  const std::string valid_name = layer_prefix + "_valid";
  const std::string text_size_name = layer_prefix + "_text_size";
  const std::string exclude_name = layer_prefix + "_exclude_backfaces";
  const std::string opacity_name = layer_prefix + "_opacity";
  const std::string display_name = layer_prefix + "_display";
  const std::string color_flag_name = layer_prefix + "_use_color";
  const std::string text_flag_name = layer_prefix + "_use_text";
  const std::string normalize_name = layer_prefix + "_arrow_normalize";
  const std::string scale_name = layer_prefix + "_arrow_scale";
  const std::string arrow_color_name = layer_prefix + "_arrow_color";
  const std::string has_arrow_color_name = layer_prefix + "_has_arrow_color";

  attributes.remove(value_name);
  attributes.remove(valid_name);
  attributes.remove(arrow_color_name);

  /* Capture Value on the full domain so color is not gated by Selection. */
  const Field<bool> all_true(true);
  if (!bke::try_capture_field_on_geometry(
          component, value_name, used_domain, all_true, field))
  {
    return false;
  }

  /* Valid mask still follows Selection (text / arrows only). After Join Geometry a missing branch
   * keeps valid=false so overlays do not leak across paths. */
  if (!bke::try_capture_field_on_geometry(component, valid_name, used_domain, selection)) {
    return false;
  }

  if (has_arrow_color && arrow_color_field != nullptr) {
    if (!bke::try_capture_field_on_geometry(
            component, arrow_color_name, used_domain, all_true, *arrow_color_field))
    {
      return false;
    }
  }

  write_constant_float_attribute(attributes, text_size_name, used_domain, text_size);
  write_constant_bool_attribute(attributes, exclude_name, used_domain, exclude_backfaces);
  write_constant_float_attribute(attributes, opacity_name, used_domain, opacity);
  write_constant_int_attribute(attributes, display_name, used_domain, display_mode);
  write_constant_bool_attribute(attributes, color_flag_name, used_domain, use_color_overlay);
  write_constant_bool_attribute(attributes, text_flag_name, used_domain, use_text_overlay);
  write_constant_bool_attribute(attributes, normalize_name, used_domain, arrow_normalize);
  write_constant_float_attribute(attributes, scale_name, used_domain, arrow_scale);
  write_constant_bool_attribute(attributes, has_arrow_color_name, used_domain, has_arrow_color);

  if (use_color_overlay) {
    composite_debug_color(attributes, value_name, used_domain, opacity);
  }
  return true;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry"_ustr);
  const NodeGeometryDebug &storage = node_storage(params.node());
  const eCustomDataType cd_type = eCustomDataType(storage.data_type);
  const bke::AttrType data_type = *bke::custom_data_type_to_attr_type(cd_type);
  const auto domain_or_auto = bke::AttrDomainSelection(storage.domain);
  const std::string layer_prefix = debug_layer_prefix(params);

  const int display_mode = (cd_type == CD_PROP_FLOAT4X4) ? int(storage.matrix_display) :
                           (cd_type == CD_PROP_FLOAT3)   ? int(storage.vector_display) :
                                                           0;

  Field<bool> selection = params.extract_input<Field<bool>>("Selection"_ustr);
  GField field = params.extract_input<GField>("Value"_ustr);

  const bool is_arrow = uses_arrow_display(
      cd_type, storage.vector_display, storage.matrix_display);
  /* Arrow modes never draw text; non-arrow modes honor the Text Overlay checkbox. */
  const bool use_text_overlay = !is_arrow && storage.use_text_overlay != 0;
  const float text_size = is_arrow ?
                              11.0f :
                              math::clamp(params.extract_input<float>("Text Size"_ustr), 1.0f, 64.0f);

  const bool applies_color = type_applies_color_overlay(cd_type, storage.vector_display);
  const bool use_color_overlay = storage.use_color_overlay != 0 && applies_color;
  /* Default opacity to 1 when Color Overlay is on. Only read the Opacity socket when it is
   * shown for this type/mode — otherwise extract would hit a disabled socket (0) and kill color. */
  float opacity = 0.0f;
  if (use_color_overlay) {
    opacity = 1.0f;
    if (type_shows_color_overlay_ui(cd_type, storage.vector_display)) {
      opacity = math::clamp(params.extract_input<float>("Opacity"_ustr), 0.0f, 1.0f);
    }
  }

  bool arrow_normalize = false;
  float arrow_scale = 1.0f;
  bool has_arrow_color = false;
  Field<ColorGeometry4f> arrow_color_field;
  if (is_arrow) {
    arrow_normalize = params.extract_input<bool>("Normalize Length"_ustr);
    arrow_scale = math::max(params.extract_input<float>("Arrow Scale"_ustr), 0.0f);
    if (cd_type == CD_PROP_FLOAT3 && storage.vector_display == NODE_GEO_DEBUG_VECTOR_ARROWS) {
      arrow_color_field = params.extract_input<Field<ColorGeometry4f>>("Arrow Color"_ustr);
      has_arrow_color = true;
    }
  }

  const bool exclude_backfaces = params.extract_input<bool>("Exclude Backfaces"_ustr);

  if (data_type == bke::AttrType::String) {
    field = *bke::get_implicit_type_conversions().try_convert(
        std::move(field), bke::attribute_type_to_cpp_type(data_type));
  }

  auto capture = [&](GeometryComponent &component) {
    capture_on_component(component,
                         layer_prefix,
                         domain_or_auto,
                         selection,
                         field,
                         text_size,
                         exclude_backfaces,
                         use_color_overlay,
                         use_text_overlay,
                         opacity,
                         display_mode,
                         arrow_normalize,
                         arrow_scale,
                         has_arrow_color,
                         has_arrow_color ? &arrow_color_field : nullptr);
  };

  if (domain_or_auto == bke::AttrDomainSelection::Instance) {
    if (geometry_set.has_instances()) {
      capture(geometry_set.get_component_for_write(GeometryComponent::Type::Instance));
    }
  }
  else {
    geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry) {
      for (const GeometryComponent::Type type : {GeometryComponent::Type::Mesh,
                                                 GeometryComponent::Type::PointCloud,
                                                 GeometryComponent::Type::Curve,
                                                 GeometryComponent::Type::GreasePencil})
      {
        if (geometry.has(type)) {
          capture(geometry.get_component_for_write(type));
        }
      }
    });
  }

  params.set_output("Geometry"_ustr, std::move(geometry_set));
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem matrix_display_items[] = {
      {NODE_GEO_DEBUG_MATRIX_COMPONENTS,
       "COMPONENTS",
       0,
       "Components",
       "Show location, rotation and scale text"},
      {NODE_GEO_DEBUG_MATRIX_AXES,
       "AXES",
       0,
       "Axes",
       "Draw the upper-left 3x3 as XYZ-colored arrows"},
      {NODE_GEO_DEBUG_MATRIX_VALUES,
       "VALUES",
       0,
       "Values",
       "Show the full 4x4 numeric matrix"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem vector_display_items[] = {
      {NODE_GEO_DEBUG_VECTOR_COLOR,
       "COLOR",
       0,
       "Color",
       "Preview the vector as surface color (XYZ mapped to RGB)"},
      {NODE_GEO_DEBUG_VECTOR_ARROWS,
       "ARROWS",
       0,
       "Arrows",
       "Draw direction arrows with optional normalize and scale"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(
      srna,
      "data_type",
      "Data Type",
      "Type of field displayed by the debug overlay",
      rna_enum_attribute_type_items,
      NOD_storage_enum_accessors(data_type),
      CD_PROP_FLOAT,
      [](bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free) {
        *r_free = true;
        return enum_items_filter(rna_enum_attribute_type_items, [](const EnumPropertyItem &item) {
          return debug_data_type_supported(eCustomDataType(item.value));
        });
      });

  RNA_def_node_enum(srna,
                    "domain",
                    "Domain",
                    "Domain used to sample the field for the overlay",
                    rna_enum_attribute_domain_with_auto_items,
                    NOD_storage_enum_accessors(domain),
                    int(bke::AttrDomainSelection::Auto));

  RNA_def_node_enum(srna,
                    "matrix_display",
                    "Matrix Display",
                    "How matrix fields are drawn in the viewport",
                    matrix_display_items,
                    NOD_storage_enum_accessors(matrix_display),
                    NODE_GEO_DEBUG_MATRIX_COMPONENTS);

  /* Changing vector display mode shows/hides Normalize / Arrow Scale sockets. */
  PropertyRNA *vector_display_prop = RNA_def_node_enum(
      srna,
      "vector_display",
      "Vector Display",
      "How vector fields are drawn in the viewport",
      vector_display_items,
      NOD_storage_enum_accessors(vector_display),
      NODE_GEO_DEBUG_VECTOR_ARROWS);
  RNA_def_property_update_runtime(vector_display_prop, rna_Node_socket_update);

  /* Must use storage accessors: ptr->data is bNode*, not NodeGeometryDebug. */
  RNA_def_node_boolean(
      srna,
      "use_color_overlay",
      "Color Overlay",
      "Composite this field into the shared color overlay. Disable so later Debug nodes do not "
      "replace earlier colors. For vectors, only applies when Vector Display is Color",
      NOD_storage_boolean_accessors(use_color_overlay, 1),
      true);

  RNA_def_node_boolean(
      srna,
      "use_text_overlay",
      "Text Overlay",
      "Draw attribute values as viewport text. Off by default because per-element text is "
      "expensive; enable when numbers are needed",
      NOD_storage_boolean_accessors(use_text_overlay, 1),
      false);
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeDebug"_ustr, GEO_NODE_DEBUG);
  ntype.ui_name = "Debug";
  ntype.ui_description =
      "Attach one independent viewport debug layer to geometry; chain multiple Debug nodes to "
      "display multiple fields without path collisions";
  ntype.enum_name_legacy = "DEBUG";
  ntype.nclass = NODE_CLASS_OUTPUT;
  bke::node_type_storage(
      ntype, "NodeGeometryDebug", node_free_standard_storage, node_copy_standard_storage);
  ntype.initfunc = node_init;
  ntype.updatefunc = node_update;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.default_width = bke::NodeWidth::_160;
  bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_debug_cc
