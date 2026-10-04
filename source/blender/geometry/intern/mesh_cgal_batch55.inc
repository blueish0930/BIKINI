/* Included at the end of mesh_cgal.cc WITH_CGAL block. Uses local static helpers. */
bool cgal_points_sphere_intersect(Span<float3> points,
                                  Span<float> radii,
                                  float uniform_radius,
                                  int segments,
                                  Vector<Vector<float3>> &r_polylines,
                                  std::string &r_error)
{
  std::vector<std::vector<float>> packed;
  const float *rad = radii.is_empty() ? nullptr : radii.data();
  if (!cgal_bridge::points_sphere_intersect(points.cast<float>().data(),
                                            int(points.size()),
                                            rad,
                                            uniform_radius,
                                            segments,
                                            packed,
                                            r_error))
  {
    return false;
  }
  return packed_polylines_to_vectors(packed, r_polylines);
}

Mesh *cgal_mesh_sphere_arrangement(const Mesh &mesh, float3 origin, std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_sphere_arrangement(
      mesh_to_cgal_in(mesh), origin.x, origin.y, origin.z);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "circle_id", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
  }
  return out;
}
Mesh *cgal_mesh_graphcut_segment(const Mesh &mesh,
                                 float angle_deg,
                                 int min_size,
                                 std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_graphcut_segment(
      mesh_to_cgal_in(mesh), double(angle_deg), min_size);
  Mesh *out = result_to_mesh_or_positions(result, mesh, r_error);
  if (out && int(result.face_tag.size()) == out->faces_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<int> w = attrs.lookup_or_add_for_write_only_span<int>(
        "Segment", bke::AttrDomain::Face);
    w.span.copy_from(Span(result.face_tag.data(), result.face_tag.size()));
    w.finish();
  }
  return out;
}

PointCloud *cgal_points_classification_features(Span<float3> points,
                                                int knn,
                                                const PointCloud *src_pc,
                                                const Mesh *src_mesh,
                                                std::string &r_error)
{
  std::vector<float> lin, pla, sph, omn, ani, ver;
  if (!cgal_bridge::points_classification_features(points.cast<float>().data(),
                                                   int(points.size()),
                                                   knn,
                                                   lin,
                                                   pla,
                                                   sph,
                                                   omn,
                                                   ani,
                                                   ver,
                                                   r_error))
  {
    return nullptr;
  }
  PointCloud *pc = pointcloud_from_positions_copy(points);
  if (src_pc) {
    pointcloud_copy_point_attributes(*src_pc, *pc, {"position"});
  }
  else if (src_mesh) {
    pointcloud_copy_from_mesh_point_attributes(*src_mesh, *pc, {"position"});
  }
  bke::MutableAttributeAccessor attrs = pc->attributes_for_write();
  auto write_f = [&](const char *name, const std::vector<float> &v) {
    bke::SpanAttributeWriter<float> w = attrs.lookup_or_add_for_write_only_span<float>(
        name, bke::AttrDomain::Point);
    w.span.copy_from(Span(v.data(), int(v.size())));
    w.finish();
  };
  write_f("sphericity", sph);
  write_f("omnivariance", omn);
  write_f("anisotropy", ani);
  return pc;
}

static PointCloud *xyz_to_pc(const std::vector<float> &xyz)
{
  const int n = int(xyz.size() / 3);
  Array<float3> pts(n);
  for (int i = 0; i < n; i++) {
    pts[i] = float3(xyz[size_t(i) * 3], xyz[size_t(i) * 3 + 1], xyz[size_t(i) * 3 + 2]);
  }
  return pointcloud_from_positions_copy(pts.as_span());
}

static void register_copy_source_attrs(PointCloud &pc,
                                       const PointCloud *src_pc,
                                       const Mesh *src_mesh)
{
  if (src_pc) {
    pointcloud_copy_point_attributes(*src_pc, pc, {"position"});
  }
  else if (src_mesh) {
    pointcloud_copy_from_mesh_point_attributes(*src_mesh, pc, {"position"});
  }
}

PointCloud *cgal_points_register_icp(Span<float3> src,
                                     Span<float3> tgt,
                                     int iterations,
                                     const PointCloud *src_pc,
                                     const Mesh *src_mesh,
                                     std::string &r_error)
{
  std::vector<float> out;
  if (!cgal_bridge::points_register_icp(src.cast<float>().data(),
                                        int(src.size()),
                                        tgt.cast<float>().data(),
                                        int(tgt.size()),
                                        iterations,
                                        out,
                                        r_error))
  {
    return nullptr;
  }
  PointCloud *pc = xyz_to_pc(out);
  if (pc) {
    register_copy_source_attrs(*pc, src_pc, src_mesh);
  }
  return pc;
}

PointCloud *cgal_points_register_4pcs(Span<float3> src,
                                      Span<float3> tgt,
                                      int samples,
                                      int icp_iterations,
                                      const PointCloud *src_pc,
                                      const Mesh *src_mesh,
                                      std::string &r_error)
{
  std::vector<float> out;
  if (!cgal_bridge::points_register_4pcs(src.cast<float>().data(),
                                         int(src.size()),
                                         tgt.cast<float>().data(),
                                         int(tgt.size()),
                                         samples,
                                         icp_iterations,
                                         out,
                                         r_error))
  {
    return nullptr;
  }
  PointCloud *pc = xyz_to_pc(out);
  if (pc) {
    register_copy_source_attrs(*pc, src_pc, src_mesh);
  }
  return pc;
}
Mesh *cgal_points_alpha_wrap_2(Span<float3> points,
                               const Mesh *segments,
                               float alpha,
                               float offset,
                               std::string &r_error)
{
  cgal_bridge::MeshIn seg_in;
  const cgal_bridge::MeshIn *seg_ptr = nullptr;
  if (segments) {
    seg_in = mesh_to_cgal_in(*segments);
    seg_ptr = &seg_in;
  }
  cgal_bridge::MeshResult result = cgal_bridge::points_alpha_wrap_2(
      points.cast<float>().data(), int(points.size()), seg_ptr, alpha, offset);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

Mesh *cgal_mesh_cage_deform_3(const Mesh &cage_rest,
                              const Mesh &cage_pose,
                              const Mesh &interior,
                              std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_cage_deform_3(
      mesh_to_cgal_in(cage_rest), mesh_to_cgal_in(cage_pose), mesh_to_cgal_in(interior));
  return result_to_mesh_or_positions(result, interior, r_error);
}

Mesh *cgal_dual_contour_grid(Span<float> sdf,
                             int nx,
                             int ny,
                             int nz,
                             float3 origin,
                             float3 voxel,
                             float isovalue,
                             std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_dual_contour_grid(
      sdf.data(), nx, ny, nz, origin.x, origin.y, origin.z, voxel.x, voxel.y, voxel.z, isovalue);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}
Mesh *cgal_mesh_tet_remesh(const Mesh & /*mesh*/, float /*target_edge*/, std::string &r_error)
{
  r_error = "Tet remesh is not in this CGAL bridge build";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
Mesh *cgal_mesh_constrained_delaunay_3(const Mesh & /*mesh*/, std::string &r_error)
{
  r_error = "Constrained Delaunay 3D is not in this CGAL bridge build";
  return BKE_mesh_new_nomain(0, 0, 0, 0);
}
