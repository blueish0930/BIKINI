/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"

#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "BLI_array_utils.hh"
#include "BLI_atomic_disjoint_set.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_mesh_edge_rings_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Int>("Edge Ring ID"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "Identifier of the edge ring that contains the edge, edges of the same ring have the "
          "same value. A ring connects the edges on opposite sides of faces with an even number "
          "of sides");
  b.add_output<decl::Int>("Edge Loop ID"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "Identifier of the edge loop that contains the edge, edges of the same loop have the "
          "same value. A loop continues through the opposite edge of vertices with an even "
          "number of edges, and along mesh boundaries");
}

/**
 * For every face corner, the edge that starts at the corner and the other edge of the same face
 * that uses the vertex of the corner.
 */
static Array<int2> corner_edge_pairs(const OffsetIndices<int> faces, const Span<int> corner_edges)
{
  Array<int2> pairs(corner_edges.size());
  threading::parallel_for(faces.index_range(), 1024, [&](const IndexRange range) {
    for (const int face_i : range) {
      const IndexRange face = faces[face_i];
      for (const int corner : face) {
        pairs[corner] = int2(corner_edges[corner],
                             corner_edges[bke::mesh::face_corner_prev(face, corner)]);
      }
    }
  });
  return pairs;
}

/**
 * Move the pair that continues the chain ending with \a edge to the front of \a pairs, oriented
 * so that it starts with \a edge.
 */
static bool chain_next_pair(MutableSpan<int2> pairs, const int edge)
{
  for (int2 &pair : pairs) {
    if (!ELEM(edge, pair[0], pair[1])) {
      continue;
    }
    if (pair[1] == edge) {
      std::swap(pair[0], pair[1]);
    }
    std::swap(pairs.first(), pair);
    return true;
  }
  return false;
}

/**
 * Order the face corners around every vertex into fans, so that neighboring corners in a fan
 * share an edge. Fans of the same vertex are stored one after the other, \a r_fan_indices is
 * different for the corners of different fans.
 */
static void sort_corner_edge_pairs(const IndexMask &vert_mask,
                                   const OffsetIndices<int> vert_to_corner_offsets,
                                   MutableSpan<int2> r_pairs,
                                   MutableSpan<int> r_fan_indices)
{
  vert_mask.foreach_index(
      [&](const int vert) {
        MutableSpan<int2> pairs = r_pairs.slice(vert_to_corner_offsets[vert]);
        MutableSpan<int> fan_indices = r_fan_indices.slice(vert_to_corner_offsets[vert]);
        int fan_index = 0;
        for (int start = 0; start < pairs.size();) {
          int end = start + 1;
          const auto grow_fan = [&]() {
            while (end < pairs.size() && chain_next_pair(pairs.drop_front(end), pairs[end - 1][1]))
            {
              end++;
            }
          };
          grow_fan();
          MutableSpan<int2> fan = pairs.slice(start, end - start);
          if (fan.first()[0] != fan.last()[1] && end < pairs.size()) {
            /* The fan is open and was not necessarily started at its first corner, so continue in
             * the other direction. */
            std::reverse(fan.begin(), fan.end());
            for (int2 &pair : fan) {
              std::swap(pair[0], pair[1]);
            }
            grow_fan();
          }
          fan_indices.slice(start, end - start).fill(fan_index);
          fan_index++;
          start = end;
        }
      },
      exec_mode::parallel);
}

class EdgesLineGroupFieldInput final : public bke::MeshFieldInput {
 public:
  EdgesLineGroupFieldInput() : bke::MeshFieldInput(CPPType::get<int>(), "Edges Line Rings Field")
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    const Span<int2> edges = mesh.edges();
    const Span<int> corner_edges = mesh.corner_edges();
    const Span<int> corner_verts = mesh.corner_verts();

    Array<int> vert_to_edge_offsets;
    Array<int> vert_to_edge_indices;
    const GroupedSpan<int> vert_to_edge_map = bke::mesh::build_vert_to_edge_map(
        edges, mesh.verts_num, vert_to_edge_offsets, vert_to_edge_indices);

    Array<int> vert_to_corner_offset_data;
    Array<int> vert_to_corner_indices;
    const GroupedSpan<int> vert_to_corner_map = bke::mesh::build_vert_to_corner_map(
        corner_verts, mesh.verts_num, vert_to_corner_offset_data, vert_to_corner_indices);
    const OffsetIndices<int> vert_to_corner_offsets = vert_to_corner_map.offsets;

    Array<int> edge_faces_total(mesh.edges_num, 0);
    array_utils::count_indices(corner_edges, edge_faces_total.as_mutable_span());

    /* Only vertices with manifold edges have a clear order of the faces around them. */
    IndexMaskMemory memory;
    const IndexMask vert_mask = IndexMask::from_predicate(
        IndexMask(mesh.verts_num),
        memory,
        [&](const int vert) {
          for (const int edge : vert_to_edge_map[vert]) {
            if (edge_faces_total[edge] > 2) {
              return false;
            }
          }
          return true;
        },
        exec_mode::parallel);

    Array<int2> fan_pairs(mesh.corners_num);
    const Array<int2> corner_pairs = corner_edge_pairs(mesh.faces(), corner_edges);
    array_utils::gather(
        corner_pairs.as_span(), vert_to_corner_indices.as_span(), fan_pairs.as_mutable_span());

    Array<int> fan_indices(mesh.corners_num);
    sort_corner_edge_pairs(vert_mask, vert_to_corner_offsets, fan_pairs, fan_indices);

    const auto first_fan_size = [](const Span<int> fan_indices) {
      int size = 1;
      while (size < fan_indices.size() && fan_indices[size] == fan_indices.first()) {
        size++;
      }
      return size;
    };

    AtomicDisjointSet linear_edges(mesh.edges_num);

    /* Connect the opposite edges of every fan. */
    vert_mask.foreach_index(
        [&](const int vert) {
          for (IndexRange range = vert_to_corner_offsets[vert]; !range.is_empty();) {
            const int size = first_fan_size(fan_indices.as_span().slice(range));
            const Span<int2> fan = fan_pairs.as_span().slice(range.take_front(size));
            range = range.drop_front(size);
            const bool is_cyclic = fan.first()[0] == fan.last()[1];
            if (!is_cyclic) {
              /* The loop continues along the boundary. */
              linear_edges.join(fan.first()[0], fan.last()[1]);
              continue;
            }
            if (size % 2) {
              continue;
            }
            const int semicircle_size = size / 2;
            const Span<int2> north_corners = fan.take_front(semicircle_size);
            const Span<int2> south_corners = fan.take_back(semicircle_size);
            for (const int i : IndexRange(semicircle_size)) {
              linear_edges.join(north_corners[i][0], south_corners[i][0]);
            }
          }
        },
        exec_mode::parallel);

    /* Chains of loose edges. */
    threading::parallel_for(IndexRange(mesh.verts_num), 2048, [&](const IndexRange range) {
      for (const int vert : range) {
        const Span<int> vert_edges = vert_to_edge_map[vert];
        if (vert_edges.size() != 2) {
          continue;
        }
        if (edge_faces_total[vert_edges[0]] != 0 || edge_faces_total[vert_edges[1]] != 0) {
          continue;
        }
        linear_edges.join(vert_edges[0], vert_edges[1]);
      }
    });

    /* A loose edge continues the loop that ends in between two boundary faces. */
    vert_mask.foreach_index(
        [&](const int vert) {
          const Span<int> vert_edges = vert_to_edge_map[vert];
          if (vert_edges.size() != 4) {
            return;
          }
          const IndexRange range = vert_to_corner_offsets[vert];
          if (range.size() != 2) {
            return;
          }
          if (first_fan_size(fan_indices.as_span().slice(range)) != range.size()) {
            return;
          }
          const int central_edge = fan_pairs[range.first()][1];
          for (const int edge : vert_edges) {
            if (edge_faces_total[edge] == 0) {
              linear_edges.join(central_edge, edge);
              break;
            }
          }
        },
        exec_mode::parallel);

    Array<int> edge_group(mesh.edges_num);
    linear_edges.calc_reduced_ids(edge_group);
    return mesh.attributes().adapt_domain<int>(
        VArray<int>::from_container(std::move(edge_group)), AttrDomain::Edge, domain);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep & /*deep_hash_cache*/) const override
  {
    static constexpr int8_t id = 0;
    hash.add(&id);
  }

  std::optional<AttrDomain> preferred_domain(const Mesh & /*mesh*/) const final
  {
    return AttrDomain::Edge;
  }
};

class ParallelEdgeGroupFieldInput final : public bke::MeshFieldInput {
 public:
  ParallelEdgeGroupFieldInput()
      : bke::MeshFieldInput(CPPType::get<int>(), "Parallel Edge Rings Field")
  {
  }

  GVArray get_varray_for_context(const Mesh &mesh,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    AtomicDisjointSet parallel_edges(mesh.edges_num);

    const Span<int> corner_edges = mesh.corner_edges();
    const OffsetIndices<int> faces = mesh.faces();
    threading::parallel_for(faces.index_range(), 2048, [&](const IndexRange range) {
      for (const int face_i : range) {
        const IndexRange face = faces[face_i];
        if (face.size() % 2) {
          continue;
        }
        /* Split the corners of the face into two semicircles. */
        const int semicircle_size = face.size() / 2;
        const Span<int> north_corners = corner_edges.slice(face).take_front(semicircle_size);
        const Span<int> south_corners = corner_edges.slice(face).take_back(semicircle_size);
        for (const int i : IndexRange(semicircle_size)) {
          parallel_edges.join(north_corners[i], south_corners[i]);
        }
      }
    });

    Array<int> edge_group(mesh.edges_num);
    parallel_edges.calc_reduced_ids(edge_group);

    return mesh.attributes().adapt_domain<int>(
        VArray<int>::from_container(std::move(edge_group)), AttrDomain::Edge, domain);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep & /*deep_hash_cache*/) const override
  {
    static constexpr int8_t id = 0;
    hash.add(&id);
  }

  std::optional<AttrDomain> preferred_domain(const Mesh & /*mesh*/) const final
  {
    return AttrDomain::Edge;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  params.set_output("Edge Ring ID"_ustr,
                    Field<int>::from_input<ParallelEdgeGroupFieldInput>());
  params.set_output("Edge Loop ID"_ustr, Field<int>::from_input<EdgesLineGroupFieldInput>());
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeInputMeshEdgeRings"_ustr);
  ntype.ui_name = "Edge Rings";
  ntype.ui_description =
      "Retrieve an identifier of the edge ring and of the edge loop that each edge is part of";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_mesh_edge_rings_cc
