/* Included at the end of mesh_cgal.cc WITH_CGAL block. Uses local static helpers. */

Mesh *cgal_marching_cubes_grid(Span<float> sdf,
                               int nx,
                               int ny,
                               int nz,
                               float3 origin,
                               float3 voxel,
                               float isovalue,
                               bool tcmc,
                               std::string &r_error)
{
  cgal_bridge::MeshResult result = cgal_bridge::mesh_marching_cubes_grid(sdf.data(),
                                                                         nx,
                                                                         ny,
                                                                         nz,
                                                                         origin.x,
                                                                         origin.y,
                                                                         origin.z,
                                                                         voxel.x,
                                                                         voxel.y,
                                                                         voxel.z,
                                                                         isovalue,
                                                                         tcmc);
  Mesh *out = result_to_mesh(result, r_error);
  if (out && out->faces_num > 0) {
    shade_all_sharp(*out);
  }
  return out;
}

PointCloud *cgal_points_extreme_point_3(Span<float3> points,
                                        float3 direction,
                                        std::string &r_error)
{
  float xyz[3];
  if (!cgal_bridge::points_extreme_point_3(points.cast<float>().data(),
                                           int(points.size()),
                                           direction.x,
                                           direction.y,
                                           direction.z,
                                           xyz,
                                           r_error))
  {
    return nullptr;
  }
  PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, 1);
  pc->positions_for_write()[0] = float3(xyz[0], xyz[1], xyz[2]);
  pc->tag_positions_changed();
  return pc;
}

PointCloud *cgal_mesh_barycentric_3(const Mesh &cage,
                                    Span<float3> query,
                                    int method,
                                    PointCloud **r_weights,
                                    std::string &r_error)
{
  if (r_weights) {
    *r_weights = nullptr;
  }
  std::vector<float> w;
  int n_cage = 0;
  if (!cgal_bridge::mesh_barycentric_3(mesh_to_cgal_in(cage),
                                       query.cast<float>().data(),
                                       int(query.size()),
                                       method,
                                       w,
                                       n_cage,
                                       r_error))
  {
    return nullptr;
  }
  const int nq = int(query.size());
  PointCloud *out = pointcloud_from_positions_copy(query);
  if (r_weights && n_cage > 0 && int(w.size()) == nq * n_cage) {
    PointCloud *wt = BKE_pointcloud_new_nomain(PointCloudType::Points, nq * n_cage);
    MutableSpan<float3> pos = wt->positions_for_write();
    bke::MutableAttributeAccessor attrs = wt->attributes_for_write();
    bke::SpanAttributeWriter<int> wq = attrs.lookup_or_add_for_write_only_span<int>(
        "Query", bke::AttrDomain::Point);
    bke::SpanAttributeWriter<int> wc = attrs.lookup_or_add_for_write_only_span<int>(
        "Cage", bke::AttrDomain::Point);
    bke::SpanAttributeWriter<float> ww = attrs.lookup_or_add_for_write_only_span<float>(
        "Weight", bke::AttrDomain::Point);
    int t = 0;
    for (int q = 0; q < nq; q++) {
      for (int c = 0; c < n_cage; c++, t++) {
        pos[t] = query[q];
        wq.span[t] = q;
        wc.span[t] = c;
        ww.span[t] = w[size_t(t)];
      }
    }
    wq.finish();
    wc.finish();
    ww.finish();
    wt->tag_positions_changed();
    *r_weights = wt;
  }
  return out;
}
