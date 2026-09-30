/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Fill every closed 3D polyline with triangles (CGAL triangulate_hole_polyline).
 */

#include "BLI_array.hh"
#include "BLI_math_vector.hh"
#include "BLI_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_fill_polyline_cc {

static bool same_pt(const float3 &a, const float3 &b)
{
  const float d2 = math::distance_squared(a, b);
  const float s2 = math::length_squared(a) + math::length_squared(b);
  return d2 <= 1.0e-12f * math::max(1.0f, s2);
}

static Mesh *concat_tri_meshes(Mesh *a, Mesh *b)
{
  if (!a) {
    return b;
  }
  if (!b) {
    return a;
  }
  const int va = a->verts_num;
  const int vb = b->verts_num;
  const int fa = a->faces_num;
  const int fb = b->faces_num;
  const int ca = a->corners_num;
  const int cb = b->corners_num;
  Mesh *m = BKE_mesh_new_nomain(va + vb, 0, fa + fb, ca + cb);
  m->vert_positions_for_write().slice(0, va).copy_from(a->vert_positions());
  m->vert_positions_for_write().slice(va, vb).copy_from(b->vert_positions());
  MutableSpan<int> fo = m->face_offsets_for_write();
  fo.slice(0, fa + 1).copy_from(a->face_offsets());
  const Span<int> bfo = b->face_offsets();
  for (int i = 0; i < fb; i++) {
    fo[fa + i] = bfo[i] + ca;
  }
  fo[fa + fb] = ca + cb;
  MutableSpan<int> corners = m->corner_verts_for_write();
  corners.slice(0, ca).copy_from(a->corner_verts());
  const Span<int> bc = b->corner_verts();
  for (int i = 0; i < cb; i++) {
    corners[ca + i] = bc[i] + va;
  }
  bke::mesh_calc_edges(*m, false, false);
  m->tag_overlapping_none();
  BKE_id_free(nullptr, a);
  BKE_id_free(nullptr, b);
  return m;
}

static Mesh *wire_mesh_from_cyclic_curves(const Curves &curves_id)
{
  const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
  const OffsetIndices points_by_curve = curves.points_by_curve();
  const Span<float3> positions = curves.positions();
  const VArray<bool> cyclic = curves.cyclic();

  Vector<Vector<int>> rings;
  Vector<int> ring_curve;
  rings.reserve(curves.curves_num());
  for (const int i : curves.curves_range()) {
    const IndexRange pts = points_by_curve[i];
    if (pts.size() < 3) {
      continue;
    }
    Vector<int> ring;
    ring.reserve(pts.size());
    for (const int p : pts) {
      if (!ring.is_empty() && ring.last() == p) {
        continue;
      }
      ring.append(p);
    }
    bool closed = cyclic[i];
    if (ring.size() >= 2) {
      const int a = ring.first();
      const int b = ring.last();
      if (a == b || same_pt(positions[a], positions[b])) {
        ring.pop_last();
        closed = true;
      }
    }
    if (!closed || ring.size() < 3) {
      continue;
    }
    ring_curve.append(i);
    rings.append(std::move(ring));
  }
  if (rings.is_empty()) {
    return nullptr;
  }

  int totv = 0;
  for (const Vector<int> &ring : rings) {
    totv += ring.size();
  }
  const int totf = rings.size();
  Mesh *mesh = BKE_mesh_new_nomain(totv, 0, totf, totv);
  MutableSpan<float3> dst_pos = mesh->vert_positions_for_write();
  MutableSpan<int> face_offsets = mesh->face_offsets_for_write();
  MutableSpan<int> corners = mesh->corner_verts_for_write();
  Array<int> vert_to_curve_point(totv);
  int v = 0;
  face_offsets[0] = 0;
  for (int fi = 0; fi < totf; fi++) {
    const Vector<int> &ring = rings[fi];
    for (const int p : ring) {
      dst_pos[v] = positions[p];
      vert_to_curve_point[v] = p;
      corners[v] = v;
      v++;
    }
    face_offsets[fi + 1] = v;
  }

  const bke::AttributeAccessor src_attrs = curves.attributes();
  bke::MutableAttributeAccessor dst_attrs = mesh->attributes_for_write();
  src_attrs.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.name == "position") {
      return;
    }
    if (iter.domain == bke::AttrDomain::Point) {
      const GVArraySpan src_data = *iter.get();
      bke::GSpanAttributeWriter dst_w = dst_attrs.lookup_or_add_for_write_only_span(
          iter.name, bke::AttrDomain::Point, iter.data_type);
      if (!dst_w) {
        return;
      }
      bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
        const Span<T> s = src_data.typed<T>();
        MutableSpan<T> d = dst_w.span.typed<T>();
        for (const int vi : d.index_range()) {
          const int sp = vert_to_curve_point[vi];
          d[vi] = (sp >= 0 && sp < s.size()) ? s[sp] : T();
        }
      });
      dst_w.finish();
    }
    else if (iter.domain == bke::AttrDomain::Curve) {
      const GVArraySpan src_data = *iter.get();
      bke::GSpanAttributeWriter dst_w = dst_attrs.lookup_or_add_for_write_only_span(
          iter.name, bke::AttrDomain::Face, iter.data_type);
      if (!dst_w) {
        return;
      }
      bke::attribute_math::to_static_type(src_data.type(), [&]<typename T>() {
        const Span<T> s = src_data.typed<T>();
        MutableSpan<T> d = dst_w.span.typed<T>();
        for (const int fi : d.index_range()) {
          const int ci = ring_curve[fi];
          d[fi] = (ci >= 0 && ci < s.size()) ? s[ci] : T();
        }
      });
      dst_w.finish();
    }
  });

  bke::mesh_calc_edges(*mesh, false, false);
  mesh->tag_overlapping_none();
  return mesh;
}

static Mesh *fill_all_curve_rings(const Curves &curves_id, std::string &error)
{
  Mesh *wire = wire_mesh_from_cyclic_curves(curves_id);
  if (!wire) {
    error = "Fill Polyline needs closed cyclic curves";
    return nullptr;
  }
  Mesh *out = geometry::cgal_mesh_fill_polyline(*wire, error);
  BKE_id_free(nullptr, wire);
  if (out && out->faces_num == 0) {
    BKE_id_free(nullptr, out);
    return nullptr;
  }
  return out;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Geometry"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::Mesh, GeometryComponent::Type::Curve})
      .description(
          "Closed polylines: mesh wire loops, n-gons, or cyclic curves. Every loop is filled.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Triangle patches, one island per closed loop.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  const Curves *curves_id = geometry.get_curves();
  std::string error;
  Mesh *from_mesh = nullptr;
  Mesh *from_curves = nullptr;
  if (mesh && mesh->verts_num >= 3) {
    from_mesh = geometry::cgal_mesh_fill_polyline(*mesh, error);
    if (from_mesh && from_mesh->faces_num == 0) {
      BKE_id_free(nullptr, from_mesh);
      from_mesh = nullptr;
    }
  }
  if (curves_id) {
    std::string curve_error;
    from_curves = fill_all_curve_rings(*curves_id, curve_error);
    if (!from_mesh && from_curves == nullptr) {
      error = curve_error;
    }
  }
  Mesh *out = concat_tri_meshes(from_mesh, from_curves);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Fill Polyline failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalFillPolyline"_ustr, GEO_NODE_CGAL_FILL_POLYLINE);
  ntype.ui_name = "Fill Polyline";
  ntype.ui_description =
      "Triangulate every closed 3D polyline into a patch "
      "(CGAL triangulate_hole_polyline). Mesh wire loops, n-gons, and cyclic "
      "curves are all filled. Point and edge attributes on the input ring are "
      "kept; interior diagonals are new. Isolated rings only — Fair Hole Fill "
      "patches holes in an existing surface.";
  ntype.enum_name_legacy = "CGAL_FILL_POLYLINE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_fill_polyline_cc
