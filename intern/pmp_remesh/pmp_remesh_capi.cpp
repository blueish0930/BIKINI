/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * In-memory wrapper around pmp::uniform_remeshing / pmp::adaptive_remeshing.
 */

#include <cstdlib>
// Allocations use malloc/free so this CAPI can live in a separate DLL.

#include "pmp_remesh_capi.hpp"

#include "pmp/surface_mesh.h"
#include "pmp/algorithms/remeshing.h"
#include "pmp/algorithms/features.h"
#include "pmp/exceptions.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <vector>

bool PMP_triangle_remesh(PMPRemeshData *data)
{
  data->out_verts = nullptr;
  data->out_faces = nullptr;
  data->out_totverts = 0;
  data->out_totfaces = 0;

  if (!data->verts || !data->faces || data->totverts < 3 || data->totfaces < 1) {
    return false;
  }

  const int iterations = std::max(1, data->iterations > 0 ? data->iterations : 10);

  try {
    pmp::SurfaceMesh mesh;

    /* Build triangle mesh from flat arrays. */
    std::vector<pmp::Vertex> vhandles;
    vhandles.reserve(size_t(data->totverts));
    for (int i = 0; i < data->totverts; i++) {
      const float *p = data->verts + i * 3;
      vhandles.push_back(mesh.add_vertex(pmp::Point(p[0], p[1], p[2])));
    }

    auto tri_area2 = [&](int a, int b, int c) -> float {
      const float *pa = data->verts + a * 3;
      const float *pb = data->verts + b * 3;
      const float *pc = data->verts + c * 3;
      const float abx = pb[0] - pa[0], aby = pb[1] - pa[1], abz = pb[2] - pa[2];
      const float acx = pc[0] - pa[0], acy = pc[1] - pa[1], acz = pc[2] - pa[2];
      const float nx = aby * acz - abz * acy;
      const float ny = abz * acx - abx * acz;
      const float nz = abx * acy - aby * acx;
      return nx * nx + ny * ny + nz * nz;
    };

    auto try_add_tri = [&](int a, int b, int c) -> bool {
      try {
        mesh.add_triangle(vhandles[size_t(a)], vhandles[size_t(b)], vhandles[size_t(c)]);
        return true;
      }
      catch (const pmp::TopologyException &) {
        return false;
      }
    };

    for (int f = 0; f < data->totfaces; f++) {
      const int *tri = data->faces + f * 3;
      if (tri[0] < 0 || tri[1] < 0 || tri[2] < 0 || tri[0] >= data->totverts ||
          tri[1] >= data->totverts || tri[2] >= data->totverts)
      {
        continue;
      }
      if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2]) {
        continue;
      }
      /* Zero-area triangles poison split/collapse/smooth. */
      if (!(tri_area2(tri[0], tri[1], tri[2]) > 0.0f)) {
        continue;
      }
      /* pmp add_face requires a manifold patch. Opposite winding on a
         neighbor is the usual Blender-mesh inconsistency — retry reversed
         rather than silently dropping the triangle (that punches a hole). */
      if (!try_add_tri(tri[0], tri[1], tri[2])) {
        try_add_tri(tri[0], tri[2], tri[1]);
      }
    }

    if (mesh.n_faces() == 0) {
      return false;
    }

    /* Optional feature locks (Houdini-like hard edges / boundaries). */
    if (data->crease_angle_deg > 0.0f) {
      pmp::detect_features(mesh, pmp::Scalar(data->crease_angle_deg));
    }
    if (data->protect_boundaries) {
      pmp::detect_boundary(mesh);
    }

    auto vfeature = mesh.vertex_property<bool>("v:feature", false);
    auto efeature = mesh.edge_property<bool>("e:feature", false);
    if (data->hard_verts && data->hard_verts_num > 0) {
      for (int i = 0; i < data->hard_verts_num; i++) {
        const int vi = data->hard_verts[i];
        if (vi >= 0 && vi < int(vhandles.size())) {
          vfeature[vhandles[size_t(vi)]] = true;
        }
      }
    }
    if (data->hard_edge_v0 && data->hard_edge_v1 && data->hard_edges_num > 0) {
      for (int i = 0; i < data->hard_edges_num; i++) {
        const int a = data->hard_edge_v0[i];
        const int b = data->hard_edge_v1[i];
        if (a < 0 || b < 0 || a >= int(vhandles.size()) || b >= int(vhandles.size()) || a == b) {
          continue;
        }
        const pmp::Edge e = mesh.find_edge(vhandles[size_t(a)], vhandles[size_t(b)]);
        if (!mesh.is_valid(e)) {
          continue;
        }
        efeature[e] = true;
        vfeature[mesh.vertex(e, 0)] = true;
        vfeature[mesh.vertex(e, 1)] = true;
      }
    }

    /* Isolated vertices (not referenced by any kept face) confuse sizing and
       stay as leftover points in the output. */
    {
      std::vector<pmp::Vertex> isolated;
      for (auto v : mesh.vertices()) {
        if (mesh.is_isolated(v)) {
          isolated.push_back(v);
        }
      }
      for (auto v : isolated) {
        mesh.delete_vertex(v);
      }
    }
    mesh.garbage_collection();
    if (mesh.n_faces() == 0) {
      return false;
    }

    if (data->mode == PMP_REMESH_ADAPTIVE) {
      const float min_e = data->min_edge_length;
      const float max_e = data->max_edge_length;
      const float err = data->approx_error;
      if (!(min_e > 0.0f) || !(max_e > 0.0f) || !(err > 0.0f) || min_e > max_e) {
        return false;
      }
      pmp::adaptive_remeshing(mesh,
                              pmp::Scalar(min_e),
                              pmp::Scalar(max_e),
                              pmp::Scalar(err),
                              unsigned(iterations),
                              data->use_projection,
                              data->freeze_boundary_topology);
    }
    else {
      const float edge = data->edge_length;
      if (!(edge > 0.0f)) {
        return false;
      }
      pmp::uniform_remeshing(mesh,
                             pmp::Scalar(edge),
                             unsigned(iterations),
                             data->use_projection,
                             data->freeze_boundary_topology);
    }

    if (mesh.n_faces() == 0 || mesh.n_vertices() == 0) {
      return false;
    }

    /* Compact garbage and export. */
    mesh.garbage_collection();

    const int nV = int(mesh.n_vertices());
    const int nF = int(mesh.n_faces());
    if (nV < 3 || nF < 1) {
      return false;
    }

    /* Map pmp vertex handles (may have holes before GC; after GC they are dense). */
    auto points = mesh.get_vertex_property<pmp::Point>("v:point");
    if (!points) {
      return false;
    }

    data->out_verts = static_cast<float *>(std::malloc(sizeof(float) * size_t(size_t(nV) * 3)));
    data->out_faces = static_cast<int *>(std::malloc(sizeof(int) * size_t(size_t(nF) * 3)));
    if (!data->out_verts || !data->out_faces) {
      std::free(data->out_verts);
      std::free(data->out_faces);
      data->out_verts = nullptr;
      data->out_faces = nullptr;
      return false;
    }

    auto finite3 = [](const pmp::Point &p) -> bool {
      return std::isfinite(float(p[0])) && std::isfinite(float(p[1])) &&
             std::isfinite(float(p[2]));
    };

    std::vector<int> vremap(size_t(nV), -1);
    std::vector<int> faces_tmp;
    faces_tmp.reserve(size_t(nF) * 3);

    for (auto f : mesh.faces()) {
      int corners[3] = {-1, -1, -1};
      int c = 0;
      for (auto v : mesh.vertices(f)) {
        if (c >= 3) {
          c = 4;
          break;
        }
        corners[c++] = int(v.idx());
      }
      if (c != 3) {
        continue;
      }
      if (corners[0] == corners[1] || corners[1] == corners[2] ||
          corners[0] == corners[2])
      {
        continue;
      }
      if (corners[0] < 0 || corners[1] < 0 || corners[2] < 0 ||
          corners[0] >= nV || corners[1] >= nV || corners[2] >= nV)
      {
        continue;
      }
      const pmp::Point &p0 = points[pmp::Vertex(corners[0])];
      const pmp::Point &p1 = points[pmp::Vertex(corners[1])];
      const pmp::Point &p2 = points[pmp::Vertex(corners[2])];
      if (!finite3(p0) || !finite3(p1) || !finite3(p2)) {
        continue;
      }
      const pmp::Point n = pmp::cross(p1 - p0, p2 - p0);
      if (!(pmp::sqrnorm(n) > 0)) {
        continue;
      }
      vremap[size_t(corners[0])] = 0;
      vremap[size_t(corners[1])] = 0;
      vremap[size_t(corners[2])] = 0;
      faces_tmp.push_back(corners[0]);
      faces_tmp.push_back(corners[1]);
      faces_tmp.push_back(corners[2]);
    }

    int nV_used = 0;
    for (int i = 0; i < nV; i++) {
      if (vremap[size_t(i)] == 0) {
        vremap[size_t(i)] = nV_used++;
      }
    }

    const int nF_used = int(faces_tmp.size() / 3);
    if (nV_used < 3 || nF_used < 1) {
      std::free(data->out_verts);
      std::free(data->out_faces);
      data->out_verts = nullptr;
      data->out_faces = nullptr;
      return false;
    }

    /* Reallocate tightly. */
    std::free(data->out_verts);
    std::free(data->out_faces);
    data->out_verts = static_cast<float *>(
        std::malloc(sizeof(float) * size_t(size_t(nV_used) * 3)));
    data->out_faces = static_cast<int *>(
        std::malloc(sizeof(int) * size_t(size_t(nF_used) * 3)));
    if (!data->out_verts || !data->out_faces) {
      std::free(data->out_verts);
      std::free(data->out_faces);
      data->out_verts = nullptr;
      data->out_faces = nullptr;
      return false;
    }

    for (auto v : mesh.vertices()) {
      const int src = int(v.idx());
      const int dst = vremap[size_t(src)];
      if (dst < 0) {
        continue;
      }
      const pmp::Point &p = points[v];
      data->out_verts[dst * 3 + 0] = float(p[0]);
      data->out_verts[dst * 3 + 1] = float(p[1]);
      data->out_verts[dst * 3 + 2] = float(p[2]);
    }

    for (int i = 0; i < nF_used; i++) {
      data->out_faces[i * 3 + 0] = vremap[size_t(faces_tmp[size_t(i) * 3 + 0])];
      data->out_faces[i * 3 + 1] = vremap[size_t(faces_tmp[size_t(i) * 3 + 1])];
      data->out_faces[i * 3 + 2] = vremap[size_t(faces_tmp[size_t(i) * 3 + 2])];
    }

    data->out_totverts = nV_used;
    data->out_totfaces = nF_used;
    return true;
  }
  catch (const pmp::InvalidInputException &) {
    return false;
  }
  catch (const pmp::TopologyException &) {
    return false;
  }
  catch (const std::exception &) {
    return false;
  }
  catch (...) {
    return false;
  }
}
