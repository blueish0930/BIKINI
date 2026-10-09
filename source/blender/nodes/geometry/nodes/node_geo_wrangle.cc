/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup geo_nodes
 *
 * Geometry Wrangle: VEX-like per-element script evaluated by a bytecode VM
 * (not Repeat Zone unrolling, not compile-to-fields).
 */

#include "BLI_array.hh"
#include "BLI_function_ref.hh"
#include "BLI_listbase.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_set.hh"
#include "BLI_string.hh"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>

#include <fmt/format.h>

#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_report.hh"
#include "BKE_type_conversions.hh"
#include "ED_screen.hh"

#include "BLO_read_write.hh"

#include "BLT_translation.hh"
#include "BLT_translation_any_thread.hh"

#include "DNA_array_utils.hh"
#include "DNA_color_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_node_types.h"

#include "GEO_foreach_geometry.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "FN_field.hh"
#include "FN_field_evaluation.hh"

#include "NOD_geo_wrangle.hh"
#include "NOD_geometry_nodes_list.hh"
#include "NOD_socket_items_blend.hh"
#include "NOD_socket_items_ops.hh"
#include "NOD_socket_items_ui.hh"
#include "NOD_vex.hh"

#include "node_geometry_util.hh"

#include "vex/vex_program.hh"

#include "../../../editors/interface/interface_intern.hh"

namespace blender::nodes::node_geo_wrangle_cc {

NODE_STORAGE_FUNCS(NodeGeometryWrangle)

static const char *default_wrangle_code()
{
  return "";
}

/* -------------------------------------------------------------------- */
/** \name Tab autocomplete (TextBox is not a SearchMenu; do not call button_func_search_set)
 * \{ */

static constexpr int WRANGLE_STR_MAX = 65536;

static bool item_matches_token(const expression::CompletionItem &item,
                               const StringRef token,
                               const bool is_member,
                               const bool show_all)
{
  if (item.kind == expression::CompletionKind::Header) {
    return false;
  }
  if (is_member) {
    return item.kind == expression::CompletionKind::Member;
  }
  if (item.kind == expression::CompletionKind::Member) {
    return false;
  }
  if (show_all) {
    return true;
  }
  if (token.is_empty()) {
    return false;
  }
  const StringRef key = item.insert_text.is_empty() ? item.name : item.insert_text;
  if (key.size() < token.size()) {
    return false;
  }
  for (const int64_t i : token.index_range()) {
    const char a = key[i];
    const char b = token[i];
    const char al = (a >= 'A' && a <= 'Z') ? char(a + 32) : a;
    const char bl = (b >= 'A' && b <= 'Z') ? char(b + 32) : b;
    if (al != bl) {
      return false;
    }
  }
  return true;
}

static StringRef completion_line(const StringRef text)
{
  int end = int(text.size());
  while (end > 0 && ELEM(text[end - 1], ' ', '\t', '\r')) {
    end--;
  }
  int start = end;
  while (start > 0 && text[start - 1] != '\n') {
    start--;
  }
  return text.substr(start, end - start);
}

static std::string tab_complete_result(const StringRef expression,
                                       const int token_start,
                                       const int token_len,
                                       const Span<const expression::CompletionItem *> matches)
{
  BLI_assert(!matches.is_empty());
  if (matches.size() == 1) {
    return expression::apply_completion(expression,
                                        token_start,
                                        token_len,
                                        matches[0]->insert_text,
                                        matches[0]->insert_call_parens);
  }
  StringRef lcp = matches[0]->insert_text;
  for (const expression::CompletionItem *item : matches.drop_front(1)) {
    const StringRef t = item->insert_text;
    int64_t n = 0;
    while (n < lcp.size() && n < t.size() && lcp[n] == t[n]) {
      n++;
    }
    lcp = lcp.substr(0, n);
  }
  const StringRef token = expression.substr(token_start, token_len);
  if (lcp.size() <= token.size()) {
    return expression::apply_completion(expression,
                                        token_start,
                                        token_len,
                                        matches[0]->insert_text,
                                        matches[0]->insert_call_parens);
  }
  return expression::apply_completion(expression, token_start, token_len, lcp, false);
}

static void collect_matches(const StringRef token,
                            const bool is_member,
                            const bool show_all,
                            Vector<std::string> &r_owned,
                            Vector<expression::CompletionItem> &r_all,
                            Vector<const expression::CompletionItem *> &r_matches)
{
  r_owned.clear();
  r_all.clear();
  r_matches.clear();
  vex::gather_completions(r_owned, r_all);
  for (const expression::CompletionItem &item : r_all) {
    if (item_matches_token(item, token, is_member, show_all)) {
      r_matches.append(&item);
    }
  }
}

static int wrangle_autocomplete_fn(bContext * /*C*/, char *str, void * /*arg_v*/)
{
  if (!str) {
    return AUTOCOMPLETE_NO_MATCH;
  }
  const StringRef expression = str;
  const StringRef line = completion_line(expression);
  int token_start = 0;
  int token_len = 0;
  bool is_member = false;
  if (!expression::find_completion_token(line, token_start, token_len, is_member)) {
    return AUTOCOMPLETE_NO_MATCH;
  }
  if (token_len == 0) {
    return AUTOCOMPLETE_NO_MATCH;
  }
  const int line_off = int(line.data() - expression.data());
  token_start += line_off;

  Vector<std::string> owned;
  Vector<expression::CompletionItem> all_items;
  Vector<const expression::CompletionItem *> matches;
  collect_matches(
      expression.substr(token_start, token_len), is_member, false, owned, all_items, matches);
  if (matches.is_empty()) {
    return AUTOCOMPLETE_NO_MATCH;
  }
  Vector<const expression::CompletionItem *> unique;
  for (const expression::CompletionItem *item : matches) {
    bool seen = false;
    for (const expression::CompletionItem *u : unique) {
      if (u->insert_text == item->insert_text) {
        seen = true;
        break;
      }
    }
    if (!seen) {
      unique.append(item);
    }
  }
  const std::string result = tab_complete_result(expression, token_start, token_len, unique);
  if (result.size() >= WRANGLE_STR_MAX) {
    return AUTOCOMPLETE_NO_MATCH;
  }
  BLI_strncpy(str, result.c_str(), WRANGLE_STR_MAX);
  return AUTOCOMPLETE_PARTIAL_MATCH;
}

static int wrangle_call_help(const StringRef fn, char (*out)[192], const int max_num)
{
  int n = 0;
  auto row = [&](const char *s) {
    if (n >= max_num) {
      return;
    }
    StringRef(s).copy_utf8_truncated(out[n], 192);
    n++;
  };
  if (fn == "delete_geometry" || fn == "deletegeometry" || fn == "deletegeo") {
    row("delete_geometry(geo, domain, index, behavior)");
    row("  geo is always 0 (current geometry)");
    row("  domain: point|edge|face or 0,1,2");
    row("  index default=current; maps if domain differs");
    row("  behavior: all|edge_face|only_face");
    return n;
  }
  if (fn == "addpoint") {
    row("addpoint(geo, pos)  or  addpoint(geo, idx)");
    row("  geo is always 0 (current geometry)");
    row("  pos: vector position   idx: copy P from that point");
    row("  returns the new point index");
    return n;
  }
  if (fn == "addprim" || fn == "add_prim") {
    row("addprim(geo, type, pts, close)");
    row("  geo is always 0 (current geometry)");
    row("  type: edge|face|curve  (or 0|1|2)");
    row("  pts: int[] of point indices");
    row("  close: extra last-first edge / cyclic curve; ignored for face");
    return n;
  }
  if (fn == "setattribute" || fn == "set_attribute" || fn == "setattr") {
    row("setattribute(geo, domain, index, name, value, behavior)");
    row("  geo is always 0 (current geometry)");
    row("  domain: point|edge|face|corner");
    row("  name: attribute  behavior: set|add|min|max");
    return n;
  }
  if (fn == "noise") {
    row("noise(P, scale, detail, roughness, lacunarity, distortion, dim, type)");
    row("  dim: 1-4");
    row("  type: fbm|multifractal|hybrid|ridged|hetero");
    return n;
  }
  if (fn == "voronoi" || fn == "voronoinoise") {
    row("voronoi(P, scale, detail, roughness, lacunarity,");
    row("  smoothness, randomness, feature, metric, dim, distortion)");
    row("  feature: f1|f2|smooth_f1|edge|radius");
    row("  metric: euclidean|manhattan|chebychev|minkowski  dim 1-4");
    return n;
  }
  if (fn == "geometry_proximity" || fn == "proximity") {
    row("geometry_proximity(geo, domain, sample, pos, dist)");
    row("  domain: 'point'|'edge'|'face' or 0,1,2");
    row("  pos: out vec   dist: out float");
    return n;
  }
  if (fn == "raycast") {
    row("raycast(geo, origin, dir, length, is_hit, hit_pos, hit_n, hit_dist)");
    row("  is_hit: out int    hit_pos: out vec");
    row("  hit_n: out vec     hit_dist: out float");
    return n;
  }
  if (fn == "raycastall" || fn == "raycast_all") {
    row("ray hits[] = raycastall(geo, origin, dir, length)");
    row("raycastall(..., is_hit, hit_pos, hit_n, hit_dist)");
    row("  returns ray[] (nearest first). Read each with rayishit(hits[i])");
    row("  is_hit: out int[]     hit_pos: out vector[]");
    row("  hit_n: out vector[]    hit_dist: out float[]");
    return n;
  }
  if (fn == "nearestpoints" || fn == "nearpoints") {
    row("nearestpoints(geo, k_or_r, mode)");
    row("  mode 0 or \"k\": k nearest (k_or_r is count)");
    row("  mode 1 or \"r\": all within radius");
    row("  mode 2 or \"rk\": at most k within r");
    row("  rk: nearestpoints(geo, r, k, \"rk\")");
    row("  returns int[] point indices");
    return n;
  }
  if (fn == "facesedge") {
    row("facesedge(geo, face0, face1)");
  }
  if (fn == "equivalentcorner") {
    row("equivalentcorner(geo, edge, corner)");
  }
  if (fn == "prevedge") {
    row("prevedge(geo, corner)");
  }
  if (fn == "nextedge") {
    row("nextedge(geo, corner)");
  }
  if (fn == "offsetcorner" || fn == "offset_corner" || fn == "offsetcornerinface" ||
      fn == "offset_corner_in_face")
  {
    row("offsetcorner(geo, index, offset)");
    row("  walk offset corners around the same face");
    row("  offset default 1 if omitted");
    return n;
  }
  if (fn == "sample_nearest_surface" || fn == "samplenearestsurface") {
    row("sample_nearest_surface(geo, type, sample, value)");
    row("  type: 'position'|'float'|'int'|'color'|attr name");
    row("  value: out sampled attribute");
    return n;
  }
  if (fn == "bounding_box" || fn == "boundingbox") {
    row("bounding_box(min, max)");
    row("  min, max: out vec");
    return n;
  }
  if (fn == "matrix") {
    row("matrix(...)  16 floats column-major, or 4 vectors");
    row("  matrix m = {c0x,c0y,c0z,c0w, c1x,...};");
    return n;
  }
  if (fn == "cornerface" || fn == "corner_face") {
    row("cornerface(geo, corner, faceindex, order)");
    row("  faceindex: out int   order: out int");
    return n;
  }
  if (fn == "pointcurve" || fn == "point_curve") {
    row("pointcurve(geo, point, curveindex, orderincurve)");
    row("  curveindex: out int   orderincurve: out int");
    return n;
  }
  if (fn == "foreach") {
    row("foreach (int pt; pts) { }");
    row("foreach (int i; int pt; pts) { }");
    row("  pts is an array (i[]@ids, pointneighbours, ...)");
    return n;
  }
  if (fn == "point" || fn == "edge" || fn == "face" || fn == "corner" || fn == "curve" ||
      fn == "instance")
  {
    row("point/edge/face/corner/curve/instance(geo, name, index)");
    row("  name: \"P\" / \"position\" -> vector, else attr type");
    return n;
  }
  if (fn == "valuetostring" || fn == "value_to_string") {
    row("valuetostring(int, digits)");
    row("valuetostring(float, before, after)");
    row("  extra integer digits are kept, not truncated");
    return n;
  }
  if (fn == "format" || fn == "sprintf") {
    row("format(\"{:.2f} {:04d}\", f, i)");
    row("sprintf(\"%s_%05d\", s, i)");
    row("  {} / {0} / {:.2f} / {:04d} or %s %d %.2f");
    return n;
  }
  if (fn == "pointneighbours" || fn == "point_neighbours" || fn == "pointneighbors" ||
      fn == "neighbours" || fn == "neighbors")
  {
    row("pointneighbours(geo, index)  -> int[]");
    row("  neighboring point indices of a vertex");
    return n;
  }
  if (fn == "clamp") {
    row("clamp(x, lo, hi)");
    return n;
  }
  if (fn == "min" || fn == "max") {
    row(fn == "min" ? "min(a, b)  smaller value, per component for vectors" :
                      "max(a, b)  larger value, per component for vectors");
    row(fn == "min" ? "min(arr)  smallest item of an int, float, vector or string array" :
                      "max(arr)  largest item of an int, float, vector or string array");
    return n;
  }
  if (fn == "sum") {
    row("sum(arr)  total of an int, float or vector array");
    return n;
  }
  if (fn == "union" || fn == "subtract" || fn == "intersect") {
    row(fn == "union"    ? "union(a, b)  -> every value that is in a or in b" :
        fn == "subtract" ? "subtract(a, b)  -> the values of a that are not in b" :
                           "intersect(a, b)  -> the values that are in both a and b");
    row("  new array without duplicates, in the order of a; a and b are not changed");
    row("  a and b: two int, float, vector or string arrays");
    return n;
  }
  if (fn == "find") {
    row("find(arr, value)  -> int[] with the index of every match");
    row("  empty when the value is not in the array: len(find(arr, x)) == 0");
    return n;
  }
  if (fn == "slice") {
    row("slice(arr, start, end, step)  -> new array, end is not included");
    row("  negative indices count from the end: slice(arr, -3) is the last three");
    row("  end and step are optional, a negative step walks backwards");
    return n;
  }
  if (fn == "unique") {
    row("unique(arr)  -> new array without duplicates, first occurrences in order");
    row("  arr itself is not changed: arr = unique(arr);");
    return n;
  }
  if (fn == "mix" || fn == "lerp") {
    row("mix(a, b, t)  t in 0..1");
    return n;
  }
  if (fn == "map") {
    row("map(value, from_min, from_max, to_min, to_max)");
    row("  float or vector (component-wise; ranges can be float)");
    return n;
  }
  if (fn == "smooth") {
    row("smooth(value)  hermite 0..1 (clamped)");
    row("smooth(min, max, value)  smoothstep");
    return n;
  }
  if (fn == "chramp") {
    row("chramp(\"name\", position)  color ramp input, position 0..1");
    row("  color c = chramp(...)   float f = chramp(...)");
    row("  Compile creates the Color Ramp input in the Constant panel");
    return n;
  }
  if (fn == "chcurve") {
    row("chcurve(\"name\", value, factor)  curve input, factor is optional");
    row("  float value: float curve    vector value: vector curve");
    row("  color value: color curve    returns the type of value");
    return n;
  }
  if (fn == "hsvtorgb" || fn == "rgbtohsv") {
    row("hsvtorgb(h, s, v) / hsvtorgb(hsv)   rgbtohsv(r, g, b) / rgbtohsv(rgb)");
    row("  all components 0..1, hue wraps around");
    return n;
  }
  if (fn == "rotate") {
    row("rotate(a, b)  multiply two rotations");
    row("rotate(x, angle, axis)  x: vector, matrix, matrix3 or rotation");
    row("  angle in radians, returns the rotated x");
    return n;
  }
  if (fn == "quaternion" || fn == "rotation" || fn == "quat") {
    row("quaternion(x, y, z, w)");
    row("quaternion(angle, axis)  angle in radians");
    row("quaternion(matrix)  quaternion(euler_xyz)");
    return n;
  }
  if (fn == "eulertoquaternion" || fn == "quaterniontoeuler") {
    row("eulertoquaternion(radians, order)   quaterniontoeuler(q, order)");
    row("  order: 0 xyz, 1 xzy, 2 yxz, 3 yzx, 4 zxy, 5 zyx  or \"xyz\"");
    return n;
  }
  if (fn == "lookat") {
    row("lookat(from, to, up)  -> matrix3");
    row("  -Z points from `from` to `to`, Y towards up (default +Z)");
    return n;
  }
  if (fn == "array") {
    row("array(a, b, ...)  values and arrays are joined into one array");
    row("array(chi(\"name\"))  a channel here becomes a List input");
    row("  chi/chb: int[]  chf: float[]  chs: string[]");
    row("  chu/chv/chq/chc: vector[]  chm/chr: matrix[]");
    return n;
  }
  /* Fallback: signature from the completion table so `fn(` is never just a name list. */
  {
    Vector<std::string> owned;
    Vector<expression::CompletionItem> items;
    vex::gather_completions(owned, items);
    for (const expression::CompletionItem &item : items) {
      if (item.kind != expression::CompletionKind::Function || item.name != fn) {
        continue;
      }
      const std::string line = item.usage.is_empty() ? std::string(item.name) :
                                                       std::string(item.usage);
      row(line.c_str());
      return n;
    }
  }
  return 0;
}

static void wrangle_suggest_fn(const char *str,
                               int cursor,
                               void * /*arg*/,
                               char (*out)[192],
                               int *r_num,
                               const int max_num)
{
  *r_num = 0;
  if (!str || max_num <= 0) {
    return;
  }
  const bool show_all = cursor < 0;
  const int len = int(strlen(str));
  cursor = std::clamp(show_all ? len : cursor, 0, len);
  bool is_member = false;
  int start = cursor;
  while (start > 0) {
    const char c = str[start - 1];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' ||
        c == '@')
    {
      start--;
    }
    else {
      if (c == '.') {
        is_member = true;
      }
      break;
    }
  }
  bool in_call = false;
  if (!is_member) {
    int depth = 0;
    int i = cursor;
    while (i > 0) {
      const char c = str[i - 1];
      if (c == '"' || c == '\'') {
        const char q = c;
        i--;
        while (i > 0 && str[i - 1] != q) {
          if (str[i - 1] == '\\' && i > 1) {
            i--;
          }
          i--;
        }
        if (i > 0) {
          i--;
        }
        continue;
      }
      if (c == '\n' || c == ';') {
        break;
      }
      if (c == ')') {
        depth++;
      }
      else if (c == '(') {
        if (depth == 0) {
          int p = i - 1;
          while (p > 0 && ELEM(str[p - 1], ' ', '\t')) {
            p--;
          }
          const int ident_end = p;
          while (p > 0) {
            const char c2 = str[p - 1];
            if ((c2 >= 'a' && c2 <= 'z') || (c2 >= 'A' && c2 <= 'Z') ||
                (c2 >= '0' && c2 <= '9') || c2 == '_')
            {
              p--;
            }
            else {
              break;
            }
          }
          if (ident_end > p) {
            in_call = true;
            start = p;
            cursor = ident_end;
          }
          break;
        }
        depth--;
      }
      i--;
    }
  }
  const StringRef match_token(str + start, std::max(cursor - start, 0));
  /* After `fn(` show usage docs (enums, out-params), not a second name list. */
  if (in_call) {
    const int help_n = wrangle_call_help(match_token, out, max_num);
    if (help_n > 0) {
      const StringRef types = vex::function_param_types(match_token);
      if (!types.is_empty()) {
        std::string s(out[0]);
        if (s.find('\x1f') == std::string::npos && s.size() + types.size() + 2 < 192) {
          s.push_back('\x1f');
          s += types;
          StringRef(s).copy_utf8_truncated(out[0], 192);
        }
      }
      *r_num = help_n;
      return;
    }
  }
  /* Only pop the list while typing an identifier / member, or on explicit Ctrl-Space. */
  if (match_token.is_empty() && !is_member && !show_all) {
    return;
  }
  Vector<std::string> owned;
  Vector<expression::CompletionItem> all_items;
  Vector<const expression::CompletionItem *> matches;
  collect_matches(match_token, is_member, show_all, owned, all_items, matches);
  /* Tab inserts the first row: an exact match first, then the shortest names, which are the
   * closest to what is typed, then alphabetically. Ctrl-Space on nothing keeps the table order,
   * which is grouped by topic. */
  if (!match_token.is_empty()) {
    auto key = [](const expression::CompletionItem *item) {
      return item->insert_text.is_empty() ? item->name : item->insert_text;
    };
    std::stable_sort(matches.begin(),
                     matches.end(),
                     [&](const expression::CompletionItem *a, const expression::CompletionItem *b) {
                       const StringRef key_a = key(a);
                       const StringRef key_b = key(b);
                       if (key_a.size() != key_b.size()) {
                         return key_a.size() < key_b.size();
                       }
                       return key_a < key_b;
                     });
  }
  int n = 0;
  for (const expression::CompletionItem *item : matches) {
    if (n >= max_num) {
      break;
    }
    if (in_call && item->kind != expression::CompletionKind::Function) {
      continue;
    }
    const StringRef shown = item->usage.is_empty() ?
                                (item->insert_text.is_empty() ? item->name : item->insert_text) :
                                item->usage;
    const StringRef ins = item->insert_text.is_empty() ? item->name : item->insert_text;
    /* `text \x1f parameter types \x1f category`, the list draws the three apart. */
    std::string packed = std::string(shown);
    packed.push_back('\x1f');
    if (item->kind == expression::CompletionKind::Function) {
      packed += vex::function_param_types(item->name);
    }
    packed.push_back('\x1f');
    packed += item->category_label;
    bool dup = false;
    for (int i = 0; i < n; i++) {
      if (StringRef(packed) == StringRef(out[i])) {
        dup = true;
        break;
      }
    }
    if (dup) {
      continue;
    }
    StringRef(packed).copy_utf8_truncated(out[n], 192);
    n++;
    (void)ins;
  }
  *r_num = n;
}

/** \} */

static void draw_geo_extend_socket(CustomSocketDrawParams &params)
{
  ui::Layout &layout = params.layout;
  layout.emboss_set(ui::EmbossType::None);
  PointerRNA op_ptr = layout.op(WrangleGeoItemsAccessor::operator_idnames::add_item, "", ICON_ADD);
  RNA_int_set(&op_ptr, "node_identifier", params.node.identifier);
  RNA_boolean_set(&op_ptr, "show_dialog", false);
  RNA_boolean_set(&op_ptr, "init_from_active", false);
  RNA_enum_set(&op_ptr, "socket_type", SOCK_GEOMETRY);
}

struct ChannelParmRef {
  std::string name;
  eNodeSocketDatatype type = SOCK_FLOAT;
  bool is_code = false;
  /** Passed directly to `array()`, which reads a whole list. */
  bool is_list = false;
  /** Vector size: `chu` is 2D, `chv` is 3D and `chq` is 4D, like the `u@`/`v@`/`q@` attributes. */
  int dimensions = 3;
};

static vex::CompileOutput wrangle_compile_editor_preview(StringRef code);

static Vector<ChannelParmRef> wrangle_scan_channel_parms(StringRef src);

/**
 * How the compiled code uses the channel behind \a item. The list shape and the vector size follow
 * the code instead of being stored on the item, so a socket can never disagree with what the
 * script reads.
 */
static const ChannelParmRef *wrangle_item_channel(const NodeGeometryWrangleInputItem &item,
                                                  const Span<ChannelParmRef> channel_parms)
{
  if (!item.name) {
    return nullptr;
  }
  for (const ChannelParmRef &parm : channel_parms) {
    if (parm.name == item.name) {
      return &parm;
    }
  }
  return nullptr;
}

/** A channel used as `array(chi("name"))` reads a whole list. Code inputs are never lists. */
static bool wrangle_channel_is_list(const ChannelParmRef *parm)
{
  return parm && parm->is_list && !parm->is_code;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_default_layout();

  b.add_input<decl::Geometry>("0"_ustr, "Geometry"_ustr)
      .description("Geometry to run the wrangle script over (geo 0)");
  b.add_output<decl::Geometry>("Geometry"_ustr).propagate_all_geometry().align_with_previous();

  const bNode *node = b.node_or_null();
  const bNodeTree *tree = b.tree_or_null();
  if (!node || !tree) {
    return;
  }
  const NodeGeometryWrangle &storage = node_storage(*node);

  for (const int i : IndexRange(storage.input_items.items_num)) {
    const NodeGeometryWrangleInputItem &item = storage.input_items.items[i];
    if (wrangle_item_kind(item) != GEO_NODE_WRANGLE_ITEM_GEO) {
      continue;
    }
    const UString identifier{WrangleGeoItemsAccessor::socket_identifier_for_item(item)};
    const UString name{item.name ? item.name : ""};
    b.add_input(SOCK_GEOMETRY, name, identifier)
        .socket_name_ptr(&tree->id, *WrangleGeoItemsAccessor::item_srna, &item, "name")
        .custom_draw([i](CustomSocketDrawParams &params) {
          socket_items::ui::draw_item_socket_with_remove<WrangleGeoItemsAccessor>(params, i);
        });
  }
  b.add_input<decl::Extend>(""_ustr, "__extend__wrangle_geo"_ustr)
      .custom_draw(draw_geo_extend_socket);

  b.add_input<decl::Bool>("Selection"_ustr)
      .default_value(true)
      .hide_value()
      .evaluated_geometry_field()
      .description("Only run the script on selected elements");

  auto &constant_panel = b.add_panel("Constant"_ustr)
                             .description(
                                 "Single values from chf/chi/..., lists from channels inside "
                                 "array(), ramps and curves from chramp/chcurve and code text "
                                 "from cht, created by Compile");
  const Vector<ChannelParmRef> channel_parms = wrangle_scan_channel_parms(
      storage.code_run ? storage.code_run : "");
  /* Whether a `chcurve()` is a float, vector or color curve follows the type of the value that
   * the script passes to it, which only the compiler knows. */
  Vector<vex::Program::RampParm> ramp_parms;
  bool ramp_parms_known = false;
  for (const ChannelParmRef &parm : channel_parms) {
    if (parm.type == SOCK_CURVE) {
      const vex::CompileOutput compiled = wrangle_compile_editor_preview(
          storage.code_run ? storage.code_run : "");
      if (compiled.program) {
        ramp_parms = compiled.program->ramp_parms;
        ramp_parms_known = true;
      }
      break;
    }
  }
  auto curve_subtype_for_item = [&](const NodeGeometryWrangleInputItem &item,
                                    const StringRef identifier) {
    if (ramp_parms_known) {
      for (const vex::Program::RampParm &parm : ramp_parms) {
        if (item.name && parm.name == item.name) {
          switch (parm.kind) {
            case vex::RampKind::VectorCurve:
              return NODE_SOCKET_CURVE_VECTOR;
            case vex::RampKind::ColorCurve:
              return NODE_SOCKET_CURVE_COLOR;
            default:
              return NODE_SOCKET_CURVE_FLOAT;
          }
        }
      }
    }
    /* The code does not compile right now: keep the curve that is already there. */
    for (const bNodeSocket &socket : node->inputs) {
      if (socket.type == SOCK_CURVE && identifier == StringRef(socket.identifier)) {
        if (const auto *value = socket.default_value_typed<bNodeSocketValueCurve>()) {
          return value->curve_subtype;
        }
      }
    }
    return NODE_SOCKET_CURVE_FLOAT;
  };
  for (const int i : IndexRange(storage.input_items.items_num)) {
    const NodeGeometryWrangleInputItem &item = storage.input_items.items[i];
    if (wrangle_item_kind(item) != GEO_NODE_WRANGLE_ITEM_CONSTANT) {
      continue;
    }
    const eNodeSocketDatatype socket_type = eNodeSocketDatatype(item.socket_type);
    const UString identifier{WrangleInputItemsAccessor::socket_identifier_for_item(item)};
    const UString name{item.name ? item.name : ""};
    const ChannelParmRef *parm = wrangle_item_channel(item, channel_parms);
    BaseSocketDeclarationBuilder *input = nullptr;
    if (socket_type == SOCK_VECTOR) {
      input = &constant_panel.add_input<decl::Vector>(name, identifier)
                   .dimensions(parm ? parm->dimensions : 3);
    }
    else if (socket_type == SOCK_CURVE) {
      input = &constant_panel.add_input<decl::Curve>(name, identifier)
                   .subtype(curve_subtype_for_item(item, identifier.ref()));
    }
    else {
      input = &constant_panel.add_input(socket_type, name, identifier);
    }
    input->socket_name_ptr(&tree->id, *WrangleInputItemsAccessor::item_srna, &item, "name");
    if (wrangle_channel_is_list(parm)) {
      input->structure_type(StructureType::List).hide_value();
    }
    else {
      input->structure_type(StructureType::Single);
    }
  }

}

struct TextParmValue {
  std::string name;
  std::string code;
};

static bool wrangle_is_ident_char(const char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
}

static std::optional<eNodeSocketDatatype> wrangle_channel_fn_type(const StringRef ident)
{
  if (ident == "chf" || ident == "ch") {
    return SOCK_FLOAT;
  }
  if (ident == "chi") {
    return SOCK_INT;
  }
  /* Like the attribute prefixes: `u@` is a 2D vector, `q@` a 4D vector and `r@` a rotation. */
  if (ELEM(ident, "chu", "chv", "chq")) {
    return SOCK_VECTOR;
  }
  if (ident == "chb") {
    return SOCK_BOOLEAN;
  }
  if (ident == "chc") {
    return SOCK_RGBA;
  }
  if (ident == "chm") {
    return SOCK_MATRIX;
  }
  if (ident == "chr") {
    return SOCK_ROTATION;
  }
  if (ident == "chs" || ident == "cht") {
    return SOCK_STRING;
  }
  if (ident == "chramp") {
    return SOCK_COLOR_RAMP;
  }
  if (ident == "chcurve") {
    return SOCK_CURVE;
  }
  return std::nullopt;
}

static Vector<ChannelParmRef> wrangle_scan_channel_parms(const StringRef src)
{
  Vector<ChannelParmRef> out;
  const int n = int(src.size());
  /* For every open parenthesis, whether it starts the arguments of an `array(` call. */
  Vector<bool> array_parens;
  StringRef last_ident;
  int last_ident_end = -1;
  auto skip_space = [&](int k) {
    while (k < n && ELEM(src[k], ' ', '\t', '\n', '\r')) {
      k++;
    }
    return k;
  };
  int i = 0;
  while (i < n) {
    if (src[i] == '/' && i + 1 < n && src[i + 1] == '/') {
      i += 2;
      while (i < n && src[i] != '\n') {
        i++;
      }
      continue;
    }
    if (src[i] == '/' && i + 1 < n && src[i + 1] == '*') {
      i += 2;
      while (i + 1 < n && !(src[i] == '*' && src[i + 1] == '/')) {
        i++;
      }
      i = std::min(i + 2, n);
      continue;
    }
    if (src[i] == '"' || src[i] == '\'') {
      const char q = src[i];
      i++;
      while (i < n && src[i] != q) {
        if (src[i] == '\\' && i + 1 < n) {
          i += 2;
        }
        else {
          i++;
        }
      }
      if (i < n) {
        i++;
      }
      continue;
    }
    if (wrangle_is_ident_char(src[i]) && (i == 0 || !wrangle_is_ident_char(src[i - 1]))) {
      const int ident_start = i;
      while (i < n && wrangle_is_ident_char(src[i])) {
        i++;
      }
      const StringRef ident = src.substr(ident_start, i - ident_start);
      last_ident = ident;
      last_ident_end = i;
      const std::optional<eNodeSocketDatatype> type = wrangle_channel_fn_type(ident);
      if (!type) {
        continue;
      }
      int j = i;
      while (j < n && ELEM(src[j], ' ', '\t')) {
        j++;
      }
      if (j >= n || src[j] != '(') {
        continue;
      }
      j++;
      while (j < n && ELEM(src[j], ' ', '\t')) {
        j++;
      }
      if (j >= n || (src[j] != '"' && src[j] != '\'')) {
        continue;
      }
      const char q = src[j];
      j++;
      std::string inner;
      while (j < n && src[j] != q) {
        if (src[j] == '\\' && j + 1 < n) {
          inner += src[j + 1];
          j += 2;
        }
        else {
          inner += src[j];
          j++;
        }
      }
      if (j >= n || inner.empty()) {
        continue;
      }
      bool is_list = false;
      if (ident != "cht" && !array_parens.is_empty() && array_parens.last()) {
        /* Only a whole argument counts: `array(chi("a"), 2)` but not `array(chi("a") + 1)`. */
        int before = ident_start - 1;
        while (before >= 0 && ELEM(src[before], ' ', '\t', '\n', '\r')) {
          before--;
        }
        int after = skip_space(j + 1);
        if (before >= 0 && ELEM(src[before], '(', ',') && after < n && src[after] == ')') {
          after = skip_space(after + 1);
          is_list = after < n && ELEM(src[after], ',', ')');
        }
      }
      /* The channel call's own parenthesis was consumed here; its `)` is popped by the loop. */
      array_parens.append(false);
      i = j + 1;
      bool seen = false;
      for (ChannelParmRef &prev : out) {
        if (prev.name == inner) {
          prev.is_code |= ident == "cht";
          prev.is_list |= is_list;
          seen = true;
          break;
        }
      }
      if (!seen) {
        const int dimensions = (ident == "chu") ? 2 : (ident == "chq") ? 4 : 3;
        out.append({std::move(inner), *type, ident == "cht", is_list, dimensions});
      }
      continue;
    }
    if (src[i] == '(') {
      array_parens.append(last_ident == "array" && skip_space(last_ident_end) == i);
    }
    else if (src[i] == ')' && !array_parens.is_empty()) {
      array_parens.pop_last();
    }
    i++;
  }
  return out;
}

/**
 * Copy a native list into the array that `array(chi("name"))` returns. VEX has no color or
 * rotation arrays, so colors become RGB vectors and rotations become matrices.
 */
static void wrangle_read_list(const GListPtr &values,
                              const eNodeSocketDatatype socket_type,
                              vex::ChList &r_list)
{
  using Kind = vex::ChList::Kind;
  const CPPType *type = nullptr;
  switch (socket_type) {
    case SOCK_INT:
      type = &CPPType::get<int>();
      r_list.kind = Kind::Int;
      break;
    case SOCK_BOOLEAN:
      type = &CPPType::get<bool>();
      r_list.kind = Kind::Int;
      break;
    case SOCK_FLOAT:
      type = &CPPType::get<float>();
      r_list.kind = Kind::Float;
      break;
    case SOCK_VECTOR:
      /* 2D and 4D vectors are read below, a vector array holds their first three components. */
      type = &CPPType::get<float3>();
      r_list.kind = Kind::Vector;
      break;
    case SOCK_RGBA:
      type = &CPPType::get<ColorGeometry4f>();
      r_list.kind = Kind::Vector;
      break;
    case SOCK_MATRIX:
      type = &CPPType::get<float4x4>();
      r_list.kind = Kind::Matrix;
      break;
    case SOCK_ROTATION:
      type = &CPPType::get<math::Quaternion>();
      r_list.kind = Kind::Matrix;
      break;
    case SOCK_STRING:
      type = &CPPType::get<std::string>();
      r_list.kind = Kind::String;
      break;
    default:
      return;
  }
  if (!values) {
    return;
  }
  GVArray varray = values->varray();
  if (socket_type == SOCK_VECTOR && varray.type().is<float2>()) {
    const VArray<float2> src = varray.typed<float2>();
    r_list.vectors.reserve(src.size());
    for (const int64_t i : src.index_range()) {
      const float2 value = src[i];
      r_list.vectors.append(float3(value.x, value.y, 0.0f));
    }
    return;
  }
  if (socket_type == SOCK_VECTOR && varray.type().is<float4>()) {
    const VArray<float4> src = varray.typed<float4>();
    r_list.vectors.reserve(src.size());
    for (const int64_t i : src.index_range()) {
      r_list.vectors.append(float3(src[i]));
    }
    return;
  }
  if (varray.type() != *type) {
    varray = bke::get_implicit_type_conversions().try_convert(std::move(varray), *type);
    if (!varray) {
      return;
    }
  }
  const int64_t size = varray.size();
  switch (socket_type) {
    case SOCK_INT:
      r_list.ints.resize(size);
      varray.typed<int>().materialize(r_list.ints);
      break;
    case SOCK_BOOLEAN: {
      const VArray<bool> src = varray.typed<bool>();
      r_list.ints.reserve(size);
      for (const int64_t i : IndexRange(size)) {
        r_list.ints.append(src[i] ? 1 : 0);
      }
      break;
    }
    case SOCK_FLOAT:
      r_list.floats.resize(size);
      varray.typed<float>().materialize(r_list.floats);
      break;
    case SOCK_VECTOR:
      r_list.vectors.resize(size);
      varray.typed<float3>().materialize(r_list.vectors);
      break;
    case SOCK_RGBA: {
      const VArray<ColorGeometry4f> src = varray.typed<ColorGeometry4f>();
      r_list.vectors.reserve(size);
      for (const int64_t i : IndexRange(size)) {
        const ColorGeometry4f color = src[i];
        r_list.vectors.append(float3(color.r, color.g, color.b));
      }
      break;
    }
    case SOCK_MATRIX:
      r_list.matrices.resize(size);
      varray.typed<float4x4>().materialize(r_list.matrices);
      break;
    case SOCK_ROTATION: {
      const VArray<math::Quaternion> src = varray.typed<math::Quaternion>();
      r_list.matrices.reserve(size);
      for (const int64_t i : IndexRange(size)) {
        r_list.matrices.append(math::from_rotation<float4x4>(src[i]));
      }
      break;
    }
    case SOCK_STRING: {
      const VArray<std::string> src = varray.typed<std::string>();
      r_list.strings.reserve(size);
      for (const int64_t i : IndexRange(size)) {
        r_list.strings.append(src[i]);
      }
      break;
    }
    default:
      break;
  }
}

static const std::string *wrangle_find_text_parm(const Span<TextParmValue> values,
                                                 const StringRef name)
{
  for (const TextParmValue &value : values) {
    if (value.name == name) {
      return &value.code;
    }
  }
  return nullptr;
}

static void wrangle_append_string_literal(std::string &code, const StringRef value)
{
  code.push_back('"');
  for (const char c : value) {
    switch (c) {
      case '\\':
        code += "\\\\";
        break;
      case '"':
        code += "\\\"";
        break;
      case '\n':
        code += "\\n";
        break;
      case '\t':
        code += "\\t";
        break;
      case '\r':
        break;
      default:
        code.push_back(c);
        break;
    }
  }
  code.push_back('"');
}

/**
 * Expand `cht("name")` and fold constant `chs("name")` before VEX parsing. Calls inside comments
 * and string literals stay literal. Missing cht values use a zero expression padded to equal width
 * for editor diagnostics before Compile.
 */
static bool wrangle_expand_text_code(const StringRef src,
                                     const Span<TextParmValue> values,
                                     const bool expand_cht,
                                     const bool allow_missing,
                                     std::string &r_code,
                                     std::string &r_error)
{
  r_code.clear();
  r_error.clear();
  r_code.reserve(src.size());
  const int n = int(src.size());
  int i = 0;
  while (i < n) {
    const int token_start = i;
    if (src[i] == '/' && i + 1 < n && src[i + 1] == '/') {
      i += 2;
      while (i < n && src[i] != '\n') {
        i++;
      }
      r_code.append(src.data() + token_start, size_t(i - token_start));
      continue;
    }
    if (src[i] == '/' && i + 1 < n && src[i + 1] == '*') {
      i += 2;
      while (i + 1 < n && !(src[i] == '*' && src[i + 1] == '/')) {
        i++;
      }
      i = std::min(i + 2, n);
      r_code.append(src.data() + token_start, size_t(i - token_start));
      continue;
    }
    if (src[i] == '"' || src[i] == '\'') {
      const char quote = src[i++];
      while (i < n && src[i] != quote) {
        i += (src[i] == '\\' && i + 1 < n) ? 2 : 1;
      }
      if (i < n) {
        i++;
      }
      r_code.append(src.data() + token_start, size_t(i - token_start));
      continue;
    }
    if (!(wrangle_is_ident_char(src[i]) &&
          (i == 0 || !wrangle_is_ident_char(src[i - 1]))))
    {
      r_code.push_back(src[i++]);
      continue;
    }

    while (i < n && wrangle_is_ident_char(src[i])) {
      i++;
    }
    const StringRef ident = src.substr(token_start, i - token_start);
    const bool is_cht = ident == "cht";
    const bool is_chs = ident == "chs";
    if ((!is_cht && !is_chs) || (is_cht && !expand_cht)) {
      r_code.append(src.data() + token_start, size_t(i - token_start));
      continue;
    }
    int j = i;
    while (j < n && ELEM(src[j], ' ', '\t', '\r', '\n')) {
      j++;
    }
    if (j >= n || src[j] != '(') {
      r_code.append(src.data() + token_start, size_t(i - token_start));
      continue;
    }
    j++;
    while (j < n && ELEM(src[j], ' ', '\t', '\r', '\n')) {
      j++;
    }
    if (j >= n || !ELEM(src[j], '"', '\'')) {
      r_error = fmt::format(
          fmt::runtime(BLT_translate_do_tooltip_any_thread("{} expects a quoted parameter name, for example {}(\"name\")")),
          std::string(ident),
          std::string(ident));
      return false;
    }
    const char quote = src[j++];
    std::string name;
    while (j < n && src[j] != quote) {
      if (src[j] == '\\' && j + 1 < n) {
        name.push_back(src[j + 1]);
        j += 2;
      }
      else {
        name.push_back(src[j++]);
      }
    }
    if (j >= n) {
      r_error = fmt::format(fmt::runtime(BLT_translate_do_tooltip_any_thread("Unterminated parameter name in {}")),
                            std::string(ident));
      return false;
    }
    j++;
    while (j < n && ELEM(src[j], ' ', '\t', '\r', '\n')) {
      j++;
    }
    if (j >= n || src[j] != ')') {
      r_error = fmt::format(fmt::runtime(BLT_translate_do_tooltip_any_thread("{} expects exactly one quoted parameter name")),
                            std::string(ident));
      return false;
    }
    j++;
    if (name.empty()) {
      r_error = fmt::format(fmt::runtime(BLT_translate_do_tooltip_any_thread("{} parameter name cannot be empty")),
                            std::string(ident));
      return false;
    }
    if (const std::string *value = wrangle_find_text_parm(values, name)) {
      if (is_chs) {
        wrangle_append_string_literal(r_code, *value);
      }
      else {
        r_code.append(*value);
      }
    }
    else if (is_chs) {
      r_code.append(src.data() + token_start, size_t(j - token_start));
    }
    else if (allow_missing) {
      r_code.push_back('0');
      r_code.append(size_t(std::max(j - token_start - 1, 0)), ' ');
    }
    else {
      r_error = fmt::format(
          fmt::runtime(BLT_translate_do_tooltip_any_thread("Missing cht parameter '{}'; click Compile to create its input")),
          name);
      return false;
    }
    i = j;
  }
  return true;
}

static vex::CompileOutput wrangle_compile_editor_preview(const StringRef code)
{
  std::string expanded;
  std::string error;
  if (!wrangle_expand_text_code(code, {}, true, true, expanded, error)) {
    vex::CompileOutput output;
    output.error = error;
    output.diags.append({0, std::max(1, int(code.size())), true, error});
    return output;
  }
  return vex::compile(expanded);
}

/**
 * Return text that is already knowable while the Compile operator is running. This lets a second
 * Compile discover channel calls inside code supplied to `cht()` by an unlinked input or a direct
 * String node. More complex string fields are still expanded only during geometry evaluation.
 */
static std::optional<std::string> wrangle_compile_text_input(const bNode &node,
                                                             const StringRef name)
{
  for (const bNodeSocket *socket : node.input_sockets()) {
    if (socket->in_out != SOCK_IN || socket->type != SOCK_STRING || socket->name != name) {
      continue;
    }
    if (socket->link) {
      const bNode *from_node = socket->link->fromnode;
      if (!from_node || from_node->type_legacy != FN_NODE_INPUT_STRING || !from_node->storage) {
        return std::nullopt;
      }
      const NodeInputString &storage = *static_cast<const NodeInputString *>(from_node->storage);
      return std::string(storage.string ? storage.string : "");
    }
    if (!socket->default_value) {
      return std::nullopt;
    }
    const auto &value = *socket->default_value_typed<bNodeSocketValueString>();
    return std::string(value.value);
  }
  return std::nullopt;
}

static void wrangle_commit_run_code(NodeGeometryWrangle &storage)
{
  const char *code = storage.code ? storage.code : "";
  MEM_SAFE_DELETE(storage.code_run);
  storage.code_run = BLI_strdup(code);
}

static wmOperatorStatus wrangle_compile_exec(bContext *C, wmOperator *op)
{
  PointerRNA node_ptr = socket_items::ops::get_active_node_to_operate_on(
      C, op, WrangleInputItemsAccessor::node_idname);
  if (!node_ptr) {
    return OPERATOR_CANCELLED;
  }
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(node_ptr.owner_id);
  bNode &node = *static_cast<bNode *>(node_ptr.data);
  NodeGeometryWrangle &storage = node_storage(node);
  wrangle_commit_run_code(storage);
  const char *code = storage.code ? storage.code : "";
  Vector<ChannelParmRef> wanted = wrangle_scan_channel_parms(code);
  Vector<TextParmValue> compile_text_values;
  for (const ChannelParmRef &parm : wanted) {
    if (!parm.is_code) {
      continue;
    }
    if (std::optional<std::string> value = wrangle_compile_text_input(node, parm.name)) {
      compile_text_values.append({parm.name, std::move(*value)});
    }
  }
  std::string expanded_for_scan;
  std::string expansion_error;
  if (wrangle_expand_text_code(
          code, compile_text_values, true, true, expanded_for_scan, expansion_error))
  {
    for (ChannelParmRef &parm : wrangle_scan_channel_parms(expanded_for_scan)) {
      bool seen = false;
      for (ChannelParmRef &existing : wanted) {
        if (existing.name == parm.name) {
          existing.is_code |= parm.is_code;
          seen = true;
          break;
        }
      }
      if (!seen) {
        wanted.append(std::move(parm));
      }
    }
  }

  socket_items::SocketItemsRef ref = WrangleInputItemsAccessor::get_items_from_node(node);
  if (*ref.items_num > 0) {
    dna::array::remove_if<NodeGeometryWrangleInputItem>(
        ref.items,
        ref.items_num,
        [&](const NodeGeometryWrangleInputItem &item) -> bool {
          if (wrangle_item_kind(item) != GEO_NODE_WRANGLE_ITEM_CONSTANT) {
            return false;
          }
          if (item.name == nullptr || item.name[0] == '\0') {
            return true;
          }
          for (const ChannelParmRef &parm : wanted) {
            if (parm.name == item.name) {
              return false;
            }
          }
          return true;
        },
        WrangleInputItemsAccessor::destruct_item);
  }
  if (ref.active_index) {
    *ref.active_index = std::clamp(*ref.active_index, 0, std::max(*ref.items_num - 1, 0));
  }

  for (const ChannelParmRef &parm : wanted) {
    socket_items::SocketItemsRef live = WrangleInputItemsAccessor::get_items_from_node(node);
    NodeGeometryWrangleInputItem *existing = nullptr;
    bool name_taken = false;
    for (const int i : IndexRange(*live.items_num)) {
      NodeGeometryWrangleInputItem &item = (*live.items)[i];
      if (!(item.name && parm.name == item.name)) {
        continue;
      }
      if (wrangle_item_kind(item) == GEO_NODE_WRANGLE_ITEM_CONSTANT) {
        existing = &item;
      }
      else {
        name_taken = true;
      }
      break;
    }
    if (existing) {
      existing->socket_type = parm.type;
      existing->kind = GEO_NODE_WRANGLE_ITEM_CONSTANT;
      continue;
    }
    if (name_taken) {
      continue;
    }
    NodeGeometryWrangleInputItem *item =
        socket_items::add_item_with_socket_type_and_name<WrangleInputItemsAccessor>(
            *ntree, node, parm.type, parm.name.c_str());
    if (item) {
      MEM_delete(item->name);
      item->name = BLI_strdup(parm.name.c_str());
      item->kind = GEO_NODE_WRANGLE_ITEM_CONSTANT;
    }
  }

  socket_items::ops::update_after_node_change(C, node_ptr);
  return OPERATOR_FINISHED;
}

static void wrangle_draw_message(ui::Layout &layout, const StringRef text, const int icon, const bool red)
{
  if (red) {
    layout.red_alert_set(true);
  }
  const char *p = text.data();
  const char *end = p + text.size();
  while (p < end) {
    const char *nl = static_cast<const char *>(memchr(p, '\n', size_t(end - p)));
    const StringRef line = nl ? StringRef(p, nl) : StringRef(p, end);
    if (!line.is_empty()) {
      layout.label(line, icon);
    }
    if (!nl) {
      break;
    }
    p = nl + 1;
  }
  if (red) {
    layout.red_alert_set(false);
  }
}

static void wrangle_draw_compile_error(ui::Layout &layout, const StringRef error)
{
  wrangle_draw_message(layout, error, ICON_ERROR, true);
}

static void wrangle_code_diag_fn(
    const char *str, void *arg, ui::uiCodeDiag *out, int *r_num, int max_num);

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  bNode &node = *static_cast<bNode *>(ptr->data);
  NodeGeometryWrangle &storage = node_storage(node);

  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "domain", UI_ITEM_NONE, IFACE_("Run Over"), ICON_NONE);

  layout.use_property_split_set(false);
  layout.alignment_set(ui::LayoutAlign::Expand);

  ui::Layout &compile_row = layout.row(true);
  compile_row.alignment_set(ui::LayoutAlign::Right);
  PointerRNA edit_ptr = compile_row.op("NODE_OT_wrangle_edit", IFACE_("Edit"), ICON_TEXT);
  RNA_int_set(&edit_ptr, "node_identifier", node.identifier);
  PointerRNA op_ptr = compile_row.op("NODE_OT_wrangle_compile", IFACE_("Compile"), ICON_FILE_REFRESH);
  RNA_int_set(&op_ptr, "node_identifier", node.identifier);
  PointerRNA docs_ptr = compile_row.op("WM_OT_url_open", IFACE_("Docs"), ICON_HELP);
  RNA_string_set(&docs_ptr, "url", vex::docs_url);

  ui::Block *block = layout.block();
  const int64_t buttons_before = block ? block->buttons_ptrs.size() : 0;
  layout.textbox_with_state(ptr, "code", &storage.textbox_state, std::nullopt);
  if (block) {
    for (int64_t i = buttons_before; i < block->buttons_ptrs.size(); i++) {
      ui::Button *but = block->buttons_ptrs[i].get();
      if (but && but->type == ui::ButtonType::TextBox) {
        button_flag_enable(but, ui::BUT_CODE_EDITOR);
        button_drawflag_enable(but, ui::BUT_TEXT_LEFT);
        button_drawflag_disable(but, ui::BUT_NO_TEXT_PADDING);
        button_func_complete_set(but, wrangle_autocomplete_fn, nullptr);
        button_func_suggest_set(but, wrangle_suggest_fn, nullptr);
        button_func_code_diag_set(but, wrangle_code_diag_fn, nullptr);
      }
    }
  }
  const char *code = storage.code ? storage.code : "";
  if (code[0] != '\0') {
    const vex::CompileOutput compiled = wrangle_compile_editor_preview(code);
    if (!compiled.program && !compiled.error.empty()) {
      wrangle_draw_compile_error(layout, compiled.error);
    }
    else if (!compiled.warning.empty()) {
      wrangle_draw_message(layout, compiled.warning, ICON_INFO, false);
    }
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryWrangle *storage = MEM_new<NodeGeometryWrangle>(__func__);
  storage->textbox_state.visible_lines = 14;
  storage->domain = GEO_NODE_WRANGLE_POINT;
  storage->code = BLI_strdup(default_wrangle_code());
  storage->code_run = BLI_strdup(default_wrangle_code());
  node->storage = storage;
  node->width = 300.0f;
}

static void node_free_storage(bNode *node)
{
  NodeGeometryWrangle *storage = static_cast<NodeGeometryWrangle *>(node->storage);
  if (storage == nullptr) {
    return;
  }
  socket_items::destruct_array<WrangleInputItemsAccessor>(*node);
  MEM_SAFE_DELETE(storage->code);
  MEM_SAFE_DELETE(storage->code_run);
  MEM_delete(storage);
  node->storage = nullptr;
}

static void node_copy_storage(bNodeTree * /*dst_tree*/, bNode *dst_node, const bNode *src_node)
{
  const NodeGeometryWrangle &src_storage = node_storage(*src_node);
  NodeGeometryWrangle *dst_storage = MEM_new<NodeGeometryWrangle>(__func__,
                                                                  dna::shallow_copy(src_storage));
  dst_node->storage = dst_storage;
  if (src_storage.code) {
    dst_storage->code = BLI_strdup(src_storage.code);
  }
  if (src_storage.code_run) {
    dst_storage->code_run = BLI_strdup(src_storage.code_run);
  }
  socket_items::copy_array<WrangleInputItemsAccessor>(*src_node, *dst_node);
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  const NodeGeometryWrangle &storage = node_storage(node);
  writer.write_string(storage.code);
  writer.write_string(storage.code_run);
  socket_items::blend_write<WrangleInputItemsAccessor>(&writer, node);
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  NodeGeometryWrangle &storage = node_storage(node);
  BLO_read_string(&reader, &storage.code);
  BLO_read_string(&reader, &storage.code_run);
  /* Pre-code_run files: cook whatever was saved in the editor. */
  if (storage.code_run == nullptr && storage.code) {
    storage.code_run = BLI_strdup(storage.code);
  }
  socket_items::blend_read_data<WrangleInputItemsAccessor>(&reader, node);
  /* Dynamic Geometry Wrangle Field sockets were removed. Keep their old DNA enum value reserved,
   * but discard the saved items so old files rebuild a valid attribute-only node declaration. */
  socket_items::SocketItemsRef ref = WrangleInputItemsAccessor::get_items_from_node(node);
  if (*ref.items_num > 0) {
    dna::array::remove_if<NodeGeometryWrangleInputItem>(
        ref.items,
        ref.items_num,
        [](const NodeGeometryWrangleInputItem &item) {
          return item.kind == GEO_NODE_WRANGLE_ITEM_FIELD_LEGACY;
        },
        WrangleInputItemsAccessor::destruct_item);
  }
  if (ref.active_index) {
    *ref.active_index = std::clamp(*ref.active_index, 0, std::max(*ref.items_num - 1, 0));
  }
}

static vex::Domain storage_domain(const NodeGeometryWrangle &storage)
{
  switch (storage.domain) {
    case GEO_NODE_WRANGLE_EDGE:
      return vex::Domain::Edge;
    case GEO_NODE_WRANGLE_FACE:
      return vex::Domain::Face;
    case GEO_NODE_WRANGLE_CORNER:
      return vex::Domain::Corner;
    case GEO_NODE_WRANGLE_INSTANCE:
      return vex::Domain::Instance;
    case GEO_NODE_WRANGLE_CURVE:
      return vex::Domain::Curve;
    case GEO_NODE_WRANGLE_POINT:
    default:
      return vex::Domain::Point;
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const bool profile = std::getenv("BLENDER_VEX_PROFILE") != nullptr;
  const auto profile_begin = std::chrono::steady_clock::now();
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry"_ustr);
  const Field<bool> selection = params.extract_input<Field<bool>>("Selection"_ustr);

  const NodeGeometryWrangle &storage = node_storage(params.node());
  const char *code = storage.code_run ? storage.code_run : "";
  const Vector<ChannelParmRef> channel_parms = wrangle_scan_channel_parms(code);

  Vector<vex::ChField> parms;
  Vector<vex::ChList> lists;
  Vector<vex::ChRamp> ramps;
  /* Evaluating a curve needs its tables, building them on the shared socket value would race
   * with other evaluations of this node. */
  struct OwnedCurves {
    Vector<CurveMapping *> curves;
    ~OwnedCurves()
    {
      for (CurveMapping *curve : curves) {
        BKE_curvemapping_free(curve);
      }
    }
  } owned_curves;
  Vector<TextParmValue> text_values;
  std::string text_error;
  Vector<GeometrySet> extra_owned;
  for (const int i : IndexRange(storage.input_items.items_num)) {
    const NodeGeometryWrangleInputItem &item = storage.input_items.items[i];
    if (item.name == nullptr || item.name[0] == '\0') {
      continue;
    }
    const std::string identifier = WrangleInputItemsAccessor::socket_identifier_for_item(item);
    const NodeGeometryWrangleItemKind kind = wrangle_item_kind(item);
    if (kind == GEO_NODE_WRANGLE_ITEM_FIELD_LEGACY) {
      continue;
    }
    if (kind == GEO_NODE_WRANGLE_ITEM_GEO) {
      extra_owned.append(params.extract_input<GeometrySet>(UString(identifier)));
      continue;
    }
    if (kind != GEO_NODE_WRANGLE_ITEM_CONSTANT) {
      continue;
    }
    if (wrangle_channel_is_list(wrangle_item_channel(item, channel_parms))) {
      vex::ChList list;
      list.name = item.name;
      wrangle_read_list(params.extract_input<GListPtr>(UString(identifier)),
                        eNodeSocketDatatype(item.socket_type),
                        list);
      lists.append(std::move(list));
      continue;
    }
    if (ELEM(item.socket_type, SOCK_COLOR_RAMP, SOCK_CURVE)) {
      const SocketValueVariant value = params.extract_input<SocketValueVariant>(
          UString(identifier));
      vex::ChRamp ramp;
      ramp.name = item.name;
      if (const ColorBand *const *band = value.get_if<ColorBand *>()) {
        ramp.color_ramp = *band;
      }
      else if (const CurveMapping *const *curve = value.get_if<CurveMapping *>()) {
        if (*curve) {
          CurveMapping *copy = BKE_curvemapping_copy(*curve);
          BKE_curvemapping_init(copy);
          owned_curves.curves.append(copy);
          ramp.curve = copy;
        }
      }
      ramps.append(std::move(ramp));
      continue;
    }
    GField field = params.extract_input<GField>(UString(identifier));
    bool is_code = false;
    for (const ChannelParmRef &channel : channel_parms) {
      if (channel.name == item.name) {
        is_code = channel.is_code;
        break;
      }
    }
    /* Geometry nodes pass every vector socket as a float3, which drops the fourth component of a
     * `chq` socket. An unlinked socket still stores all four, so read it from there. */
    if (item.socket_type == SOCK_VECTOR) {
      const ChannelParmRef *channel = wrangle_item_channel(item, channel_parms);
      const bNodeSocket *socket = params.node().input_by_identifier(UString(identifier));
      if (channel && channel->dimensions == 4 && socket && !socket->is_directly_linked() &&
          socket->default_value)
      {
        const auto &value = *socket->default_value_typed<bNodeSocketValueVector>();
        const float4 default_4d(value.value[0], value.value[1], value.value[2], value.value[3]);
        field = GField::from_constant(CPPType::get<float4>(), &default_4d);
      }
    }
    if (field.cpp_type().is<std::string>() && !field.depends_on_input()) {
      text_values.append(
          {item.name, fn::evaluate_constant_field(field.typed<std::string>())});
    }
    else if (is_code) {
      if (!field.cpp_type().is<std::string>()) {
        text_error = fmt::format(fmt::runtime(BLT_translate_do_tooltip_any_thread("cht parameter '{}' must be a string")),
                                 item.name);
      }
      else {
        text_error = fmt::format(
            fmt::runtime(BLT_translate_do_tooltip_any_thread("cht parameter '{}' must be a single constant string")), item.name);
      }
    }
    parms.append(vex::ChField(item.name, std::move(field)));
  }

  if (code[0] == '\0') {
    params.set_output("Geometry"_ustr, std::move(geometry_set));
    return;
  }

  std::string expanded_text;
  if (text_error.empty() &&
      !wrangle_expand_text_code(code, text_values, true, false, expanded_text, text_error))
  {
    text_error = fmt::format(fmt::runtime(BLT_translate_do_tooltip_any_thread("Code parameter expansion failed: {}")),
                             text_error);
  }
  std::string expanded_code;
  if (text_error.empty() &&
      !wrangle_expand_text_code(
          expanded_text, text_values, false, false, expanded_code, text_error))
  {
    text_error = fmt::format(fmt::runtime(BLT_translate_do_tooltip_any_thread("chs folding failed: {}")), text_error);
  }
  if (!text_error.empty()) {
    params.error_message_add(NodeWarningType::Error, text_error);
    params.set_output("Geometry"_ustr, std::move(geometry_set));
    return;
  }

  const auto profile_before_compile = std::chrono::steady_clock::now();
  vex::CompileOutput compiled = vex::compile(expanded_code);
  const auto profile_compiled = std::chrono::steady_clock::now();
  if (!compiled.program) {
    if (profile) {
      std::fprintf(stderr,
                   "VEX_DIAG error=%s count=%lld\n",
                   compiled.error.c_str(),
                   static_cast<long long>(compiled.diags.size()));
      for (const vex::Diagnostic &diag : compiled.diags) {
        std::fprintf(stderr,
                     "VEX_DIAG_ITEM offset=%d length=%d error=%d message=%s\n",
                     diag.offset,
                     diag.length,
                     diag.is_error ? 1 : 0,
                     diag.message.c_str());
      }
    }
    params.error_message_add(NodeWarningType::Error,
                             compiled.error.empty() ? BLT_translate_do_tooltip_any_thread("Wrangle compile failed") :
                                                      compiled.error);
    params.set_output("Geometry"_ustr, std::move(geometry_set));
    return;
  }

  for (const vex::AttrInfo &attr : compiled.program->attrs) {
    if (attr.write) {
      params.used_named_attribute(attr.name, NamedAttributeUsage::Write);
    }
    else if (attr.read) {
      params.used_named_attribute(attr.name, NamedAttributeUsage::Read);
    }
  }

  Vector<const GeometrySet *> extra_ptrs;
  extra_ptrs.reserve(extra_owned.size());
  for (const GeometrySet &g : extra_owned) {
    extra_ptrs.append(&g);
  }

  if (!compiled.warning.empty()) {
    params.error_message_add(NodeWarningType::Info, compiled.warning);
  }

  vex::ExecOutput result = vex::execute(*compiled.program,
                                        geometry_set,
                                        extra_ptrs,
                                        storage_domain(storage),
                                        selection,
                                        parms,
                                        lists,
                                        ramps);
  if (profile) {
    const auto profile_end = std::chrono::steady_clock::now();
    const double setup_ms =
        std::chrono::duration<double, std::milli>(profile_before_compile - profile_begin).count();
    const double compile_ms =
        std::chrono::duration<double, std::milli>(profile_compiled - profile_before_compile).count();
    const double execute_ms =
        std::chrono::duration<double, std::milli>(profile_end - profile_compiled).count();
    if (compile_ms > 0.05) {
      int op_counts[256] = {};
      for (const vex::Inst &inst : compiled.program->code) {
        const int op = int(inst.op);
        if (op >= 0 && op < 256) {
          op_counts[op]++;
        }
      }
      std::fprintf(stderr, "VEX_OPS");
      for (int op = 0; op < 256; op++) {
        if (op_counts[op] > 0) {
          std::fprintf(stderr, " %d:%d", op, op_counts[op]);
        }
      }
      std::fprintf(stderr, "\n");
    }
    std::fprintf(stderr,
                 "VEX_PROFILE node=\"%s\" ops=%lld gathers=%lld setup_ms=%.3f compile_ms=%.3f execute_ms=%.3f "
                 "total_ms=%.3f\n",
                 params.node().name,
                 static_cast<long long>(compiled.program->code.size()),
                 static_cast<long long>(compiled.program->gather_samples.size()),
                 setup_ms,
                 compile_ms,
                 execute_ms,
                 setup_ms + compile_ms + execute_ms);
  }
  if (!result.ok) {
    params.error_message_add(NodeWarningType::Error,
                             result.error.empty() ? BLT_translate_do_tooltip_any_thread("Wrangle evaluation failed") :
                                                    result.error);
  }
  else if (!result.warning.empty()) {
    params.error_message_add(NodeWarningType::Info, result.warning);
  }

  params.set_output("Geometry"_ustr, std::move(geometry_set));
}

struct WrangleEditPopupArg {
  bNode *node = nullptr;
  bNodeTree *ntree = nullptr;
  TextboxState editor_state;
  char find[256] = {};
  char replace[256] = {};
  char status[256] = {};
  int find_from = 0;
  bool pinned = false;
  /** Only activate the text box the first time the popup is built. */
  bool activate_editor = true;
  /** Editor font size in points. 0 means "initialize from stored scale". */
  float font_pt = 0.0f;
};

static void wrangle_code_diag_fn(const char *str,
                                 void * /*arg*/,
                                 ui::uiCodeDiag *out,
                                 int *r_num,
                                 const int max_num)
{
  *r_num = 0;
  if (!str || max_num <= 0) {
    return;
  }
  const vex::CompileOutput compiled = wrangle_compile_editor_preview(str);
  const int n = std::min(max_num, int(compiled.diags.size()));
  for (int i = 0; i < n; i++) {
    out[i].offset = compiled.diags[i].offset;
    out[i].length = std::max(1, compiled.diags[i].length);
    out[i].is_error = compiled.diags[i].is_error ? 1 : 0;
  }
  *r_num = n;
}

static void wrangle_apply_code_editor_flags(ui::Block *block, const int64_t buttons_before)
{
  if (!block) {
    return;
  }
  for (int64_t i = buttons_before; i < block->buttons_ptrs.size(); i++) {
    ui::Button *but = block->buttons_ptrs[i].get();
    if (but && but->type == ui::ButtonType::TextBox) {
      button_flag_enable(but, ui::BUT_CODE_EDITOR);
      button_drawflag_enable(but, ui::BUT_TEXT_LEFT);
      button_func_complete_set(but, wrangle_autocomplete_fn, nullptr);
      button_func_suggest_set(but, wrangle_suggest_fn, nullptr);
      button_func_code_diag_set(but, wrangle_code_diag_fn, nullptr);
    }
  }
}

static void wrangle_set_code(WrangleEditPopupArg &arg, const std::string &code)
{
  NodeGeometryWrangle &storage = node_storage(*arg.node);
  MEM_SAFE_DELETE(storage.code);
  storage.code = BLI_strdup(code.c_str());
}

static ui::Block *wrangle_edit_popup(bContext *C, ARegion *region, void *arg_v)
{
  WrangleEditPopupArg *arg = static_cast<WrangleEditPopupArg *>(arg_v);
  if (!arg || !arg->node || !arg->ntree) {
    return nullptr;
  }
  NodeGeometryWrangle &storage = node_storage(*arg->node);
  if (arg->editor_state.visible_lines < 20) {
    arg->editor_state.visible_lines = 26;
  }
  if (arg->editor_state.font_scale > 0.05f) {
    storage.textbox_state.font_scale = arg->editor_state.font_scale;
  }
  else if (storage.textbox_state.font_scale > 0.05f) {
    arg->editor_state.font_scale = storage.textbox_state.font_scale;
  }
  if (arg->editor_state.width > 0) {
    storage.textbox_state.width = arg->editor_state.width;
  }
  else if (storage.textbox_state.width > 0) {
    arg->editor_state.width = storage.textbox_state.width;
  }

  const uiStyle *style = ui::style_get_dpi();
  ui::Block *block = ui::block_begin(C, region, "wrangle_code_editor", ui::EmbossType::Emboss);
  ui::block_flag_enable(block,
                        ui::BLOCK_KEEP_OPEN | ui::BLOCK_LOOP | ui::BLOCK_NO_ACCELERATOR_KEYS);
  if (arg->pinned) {
    ui::block_flag_enable(block, ui::BLOCK_PINNED);
  }
  /* Regular theme: the POPUP style is a beige/menu chrome that fights the dark editor. */
  ui::block_theme_style_set(block, ui::BLOCK_THEME_STYLE_REGULAR);

  const int layout_w = std::clamp(arg->editor_state.width > 0 ? arg->editor_state.width :
                                                                int(50 * UI_UNIT_X),
                                  int(22 * UI_UNIT_X),
                                  int(140 * UI_UNIT_X));
  ui::Layout &layout = ui::block_layout(block,
                                        ui::LayoutDirection::Vertical,
                                        ui::LayoutType::Panel,
                                        0,
                                        0,
                                        layout_w,
                                        int(2 * UI_UNIT_Y),
                                        4,
                                        style);

  ui::Layout &header = layout.row(true);
  header.label(IFACE_("Wrangle"), ICON_NODETREE);
  {
    const float base_pt = std::max(1.0f, style->widget.points);
    if (arg->font_pt < 1.0f) {
      const float scale = arg->editor_state.font_scale > 0.05f ? arg->editor_state.font_scale :
                                                                1.0f;
      arg->font_pt = std::clamp(base_pt * scale, 8.0f, 36.0f);
    }
    arg->editor_state.font_scale = std::clamp(arg->font_pt / base_pt, 0.5f, 4.0f);
    storage.textbox_state.font_scale = arg->editor_state.font_scale;
    ui::Button *size_but = ui::uiDefButV(block,
                                        ui::ButtonType::Num,
                                        IFACE_("Size"),
                                        0,
                                        0,
                                        short(6 * UI_UNIT_X),
                                        UI_UNIT_Y,
                                        &arg->font_pt,
                                        8.0f,
                                        36.0f,
                                        TIP_("Code editor font size"));
    ui::button_number_step_size_set(size_but, 1.0f);
    ui::button_number_precision_set(size_but, 0.0f);
    ui::button_func_set(size_but, [arg, base_pt](bContext &C) {
      arg->editor_state.font_scale = std::clamp(arg->font_pt / base_pt, 0.5f, 4.0f);
      node_storage(*arg->node).textbox_state.font_scale = arg->editor_state.font_scale;
      if (ARegion *popup = CTX_wm_region_popup(&C)) {
        ED_region_tag_refresh_ui(popup);
        ED_region_tag_redraw(popup);
      }
    });
  }
  header.separator();
  PointerRNA compile_hdr = header.op(
      "NODE_OT_wrangle_compile", IFACE_("Compile"), ICON_FILE_REFRESH);
  RNA_int_set(&compile_hdr, "node_identifier", arg->node->identifier);
  PointerRNA docs_hdr = header.op("WM_OT_url_open", IFACE_("Docs"), ICON_HELP);
  RNA_string_set(&docs_hdr, "url", vex::docs_url);
  header.separator_spacer();
  header.button("",
                arg->pinned ? ICON_PINNED : ICON_UNPINNED,
                [arg](bContext &C) {
                  arg->pinned = !arg->pinned;
                  if (ARegion *popup = CTX_wm_region_popup(&C)) {
                    ED_region_tag_refresh_ui(popup);
                    ED_region_tag_redraw(popup);
                  }
                },
                arg->pinned ? TIP_("Unpin") : TIP_("Keep this window open"));
  header.button("", ICON_X, [block](bContext & /*C*/) { ui::popup_menu_close(block, false); }, IFACE_("Close"));

  ui::Layout &find_abs = layout.absolute(true);
  find_abs.ui_units_y_set(1.15f);
  ui::Block *abs_block = find_abs.block();
  const int find_w = int(17 * UI_UNIT_X);
  ui::Button *find_but = uiDefBut(abs_block,
                                  ui::ButtonType::Text,
                                  "",
                                  0,
                                  0,
                                  find_w,
                                  UI_UNIT_Y,
                                  arg->find,
                                  0,
                                  255,
                                  TIP_("Find"));
  ui::button_placeholder_set(find_but, IFACE_("Find"));
  ui::Button *repl_but = uiDefBut(abs_block,
                                  ui::ButtonType::Text,
                                  "",
                                  find_w + int(0.4f * UI_UNIT_X),
                                  0,
                                  find_w,
                                  UI_UNIT_Y,
                                  arg->replace,
                                  0,
                                  255,
                                  TIP_("Replace"));
  ui::button_placeholder_set(repl_but, IFACE_("Replace"));

  ui::Layout &tools = layout.row(true);
  tools.button(IFACE_("Find Next"), ICON_VIEWZOOM, [arg](bContext &C) {
    const char *code = node_storage(*arg->node).code;
    if (!code || arg->find[0] == '\0') {
      BLI_strncpy(arg->status, "Nothing to find", sizeof(arg->status));
      return;
    }
    const char *hit = strstr(code + arg->find_from, arg->find);
    if (!hit && arg->find_from > 0) {
      hit = strstr(code, arg->find);
    }
    if (!hit) {
      BLI_strncpy(arg->status, "Not found", sizeof(arg->status));
      return;
    }
    arg->find_from = int(hit - code) + int(strlen(arg->find));
    int line = 1;
    int col = 1;
    for (const char *p = code; p < hit; p++) {
      if (*p == '\n') {
        line++;
        col = 1;
      }
      else {
        col++;
      }
    }
    arg->editor_state.scroll = std::max(0, line - 1);
    SNPRINTF(arg->status, "Line %d, column %d", line, col);
    ED_region_tag_redraw(CTX_wm_region(&C));
  });
  tools.button(IFACE_("Replace"), ICON_LOOP_FORWARDS, [arg](bContext &C) {
    const char *code = node_storage(*arg->node).code;
    if (!code || arg->find[0] == '\0') {
      return;
    }
    std::string s(code);
    const size_t pos = s.find(arg->find, size_t(arg->find_from > 0 ? arg->find_from - 1 : 0));
    if (pos == std::string::npos) {
      BLI_strncpy(arg->status, "Not found", sizeof(arg->status));
      return;
    }
    s.replace(pos, strlen(arg->find), arg->replace);
    wrangle_set_code(*arg, s);
    arg->find_from = int(pos) + int(strlen(arg->replace));
    BLI_strncpy(arg->status, "Replaced 1", sizeof(arg->status));
    if (ARegion *popup = CTX_wm_region_popup(&C)) {
      ED_region_tag_refresh_ui(popup);
      ED_region_tag_redraw(popup);
    }
  });
  tools.button(IFACE_("All"), ICON_FILE_REFRESH, [arg](bContext &C) {
    const char *code = node_storage(*arg->node).code;
    if (!code || arg->find[0] == '\0') {
      return;
    }
    std::string s(code);
    int n = 0;
    size_t pos = 0;
    const std::string find(arg->find);
    const std::string repl(arg->replace);
    while ((pos = s.find(find, pos)) != std::string::npos) {
      s.replace(pos, find.size(), repl);
      pos += repl.size();
      n++;
    }
    wrangle_set_code(*arg, s);
    SNPRINTF(arg->status, "Replaced %d", n);
    if (ARegion *popup = CTX_wm_region_popup(&C)) {
      ED_region_tag_refresh_ui(popup);
      ED_region_tag_redraw(popup);
    }
  });
  if (arg->status[0]) {
    layout.label(arg->status, ICON_INFO);
  }
  const char *code = storage.code ? storage.code : "";
  const vex::CompileOutput compiled = wrangle_compile_editor_preview(code);
  if (!compiled.program && !compiled.error.empty()) {
    wrangle_draw_compile_error(layout, compiled.error);
  }
  else if (!compiled.warning.empty()) {
    wrangle_draw_message(layout, compiled.warning, ICON_INFO, false);
  }

  PointerRNA node_ptr = RNA_pointer_create_discrete(&arg->ntree->id, RNA_Node, arg->node);
  const int64_t buttons_before = block->buttons_ptrs.size();
  layout.textbox_with_state(&node_ptr, "code", &arg->editor_state, std::nullopt);
  wrangle_apply_code_editor_flags(block, buttons_before);
  if (arg->activate_editor) {
    for (int64_t i = buttons_before; i < block->buttons_ptrs.size(); i++) {
      ui::Button *but = block->buttons_ptrs[i].get();
      if (but && but->type == ui::ButtonType::TextBox) {
        button_flag_enable(but, ui::BUT_ACTIVATE_ON_INIT);
        button_flag_enable(but, ui::BUT_ACTIVATE_ON_INIT_NO_SELECT);
      }
    }
    arg->activate_editor = false;
  }

  const int bounds_offset[2] = {0, -UI_UNIT_Y};
  ui::block_bounds_set_popup(block, int(0.3f * U.widget_unit), bounds_offset);
  return block;
}

static void wrangle_edit_popup_free(void *arg_v)
{
  WrangleEditPopupArg *arg = static_cast<WrangleEditPopupArg *>(arg_v);
  if (arg && arg->node) {
    if (arg->editor_state.font_scale > 0.05f) {
      node_storage(*arg->node).textbox_state.font_scale = arg->editor_state.font_scale;
    }
    if (arg->editor_state.width > 0) {
      node_storage(*arg->node).textbox_state.width = arg->editor_state.width;
    }
  }
  MEM_delete(arg);
}

static bool node_insert_link(bke::NodeInsertLinkParams &params)
{
  bNodeSocket *sock = nullptr;
  if (params.link.tonode == &params.node) {
    sock = params.link.tosock;
  }
  else if (params.link.fromnode == &params.node) {
    sock = params.link.fromsock;
  }
  if (sock == nullptr || !STREQ(sock->idname, "NodeSocketVirtual")) {
    return true;
  }
  const StringRef identifier = sock->identifier;
  if (identifier == "__extend__wrangle_geo") {
    const bool ok = socket_items::try_add_item_via_any_extend_socket<WrangleGeoItemsAccessor>(
        params.ntree, params.node, params.node, params.link, "__extend__wrangle_geo");
    if (!ok && params.C) {
      BKE_report(CTX_wm_reports(params.C),
                 RPT_ERROR,
                 "Wrangle geometry input only accepts Geometry");
    }
    return ok;
  }
  return false;
}

static wmOperatorStatus wrangle_edit_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  PointerRNA node_ptr = socket_items::ops::get_active_node_to_operate_on(
      C, op, WrangleInputItemsAccessor::node_idname);
  if (!node_ptr) {
    return OPERATOR_CANCELLED;
  }
  WrangleEditPopupArg *arg = MEM_new<WrangleEditPopupArg>(__func__);
  arg->node = static_cast<bNode *>(node_ptr.data);
  arg->ntree = reinterpret_cast<bNodeTree *>(node_ptr.owner_id);
  arg->editor_state.visible_lines = 26;
  const float stored = node_storage(*arg->node).textbox_state.font_scale;
  arg->editor_state.font_scale = stored > 0.05f ? std::clamp(stored, 0.5f, 4.0f) : 1.0f;
  arg->editor_state.width = node_storage(*arg->node).textbox_state.width;
  ui::popup_block_invoke(C, wrangle_edit_popup, arg, wrangle_edit_popup_free);
  return OPERATOR_CANCELLED;
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeWrangle"_ustr, GEO_NODE_WRANGLE);
  ntype.ui_name = "Wrangle";
  ntype.ui_description =
      "Run a VEX-like per-element script on geometry (bytecode VM with loops, "
      "attribute access, raycast, raycastall, bounding_box, proximity, and delete_geometry)";
  ntype.enum_name_legacy = "WRANGLE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.insert_link = node_insert_link;
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  ntype.register_operators = []() {
    socket_items::ops::make_common_operators<WrangleInputItemsAccessor>();
    socket_items::ops::make_common_operators<WrangleGeoItemsAccessor>();
    WM_operatortype_append([](wmOperatorType *ot) {
      ot->name = "Compile";
      ot->idname = "NODE_OT_wrangle_compile";
      ot->description =
          "Commit the editor draft and evaluate (also happens on Ctrl+Enter or clicking outside "
          "the code editor). Creates Constant panel inputs from "
          "chf/chi/chu/chv/chq/chb/chc/chm/chr/chs/cht/chramp/chcurve and removes unused "
          "channel sockets";
      ot->exec = wrangle_compile_exec;
      ot->poll = socket_items::ops::editable_node_active_poll<WrangleInputItemsAccessor>;
      ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
      socket_items::ops::add_node_identifier_property(ot);
    });
    WM_operatortype_append([](wmOperatorType *ot) {
      ot->name = "Edit Wrangle Code";
      ot->idname = "NODE_OT_wrangle_edit";
      ot->description = "Open a full wrangle code editor (search, replace, compile)";
      ot->invoke = wrangle_edit_invoke;
      ot->poll = socket_items::ops::editable_node_active_poll<WrangleInputItemsAccessor>;
      ot->flag = OPTYPE_REGISTER;
      socket_items::ops::add_node_identifier_property(ot);
    });
  };
  bke::node_type_storage(ntype, "NodeGeometryWrangle", node_free_storage, node_copy_storage);

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_wrangle_cc

namespace blender::nodes {

StructRNA **WrangleInputItemsAccessor::item_srna = &RNA_NodeGeometryWrangleInputItem;
int WrangleInputItemsAccessor::node_type = GEO_NODE_WRANGLE;
StructRNA **WrangleGeoItemsAccessor::item_srna = &RNA_NodeGeometryWrangleInputItem;
int WrangleGeoItemsAccessor::node_type = GEO_NODE_WRANGLE;

void WrangleInputItemsAccessor::blend_write_item(BlendWriter *writer, const ItemT &item)
{
  writer->write_string(item.name);
}

void WrangleInputItemsAccessor::blend_read_data_item(BlendDataReader *reader, ItemT &item)
{
  BLO_read_string(reader, &item.name);
}

}  // namespace blender::nodes
