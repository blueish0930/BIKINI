/* Included at the end of mesh_cgal.cc WITH_CGAL block. Uses local static helpers. */

Mesh *cgal_mesh_approximate_convex_decomposition(const Mesh &mesh,
                                                 int count,
                                                 int resolution,
                                                 std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_approximate_convex_decomposition(
      mesh_to_cgal_in(mesh), count, resolution);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  if (out && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "Part", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
  }
  return out;
}
