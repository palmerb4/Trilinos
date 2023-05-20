// Copyright 2002 - 2008, 2010, 2011 National Technology Engineering
// Solutions of Sandia, LLC (NTESS). Under the terms of Contract
// DE-NA0003525 with NTESS, the U.S. Government retains certain rights
// in this software.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of NTESS nor the names of its contributors
//       may be used to endorse or promote products derived from this
//       software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#ifndef STKTOPOLOGY_DETAIL_LINKER_HELPER_HPP
#define STKTOPOLOGY_DETAIL_LINKER_HELPER_HPP

#include <Kokkos_Core.hpp>

namespace stk
{
namespace topology_detail
{

// // Helper class for merging topologies connected by a linker
// template <topology::topology_t Topology0, topology::topology_t Topology1>
// struct linker_helper {
//   using tdata0 = topology_data<topology0>;
//   using tdata1 = topology_data<topology1>;

//   STK_INLINE_FUNCTION
//   static constexpr auto get_edge_topology_vector()
//   {
//     if constexpr (tdata0::num_edges > 0) {
//       if constexpr (tdata1::num_edges > 0) {
//         return stk_concat_arrays(tdata0::edge_topology_vector, tdata1::edge_topology_vector);
//       } else {
//         return tdata0::edge_topology_vector;
//       }
//     } else {
//       if constexpr (tdata1::num_edges > 0) {
//         return tdata1::edge_topology_vector;
//       } else {
//         return {topology::INVALID_TOPOLOGY};
//       }
//     }
//   }

//   STK_INLINE_FUNCTION
//   static constexpr auto get_face_topology_vector()
//   {
//     if constexpr (tdata0::num_faces > 0) {
//       if constexpr (tdata1::num_faces > 0) {
//         return stk_concat_arrays(tdata0::face_topology_vector, tdata1::face_topology_vector);
//       } else {
//         return tdata0::face_topology_vector;
//       }
//     } else {
//       if constexpr (tdata1::num_faces > 0) {
//         return tdata1::face_topology_vector;
//       } else {
//         return {topology::INVALID_TOPOLOGY};
//       }
//     }
//   }

//   STK_INLINE_FUNCTION
//   static constexpr Kokkos::Array<topology::topology_t, 2> get_element_topology_vector()
//   {
//     return {Topology0, Topology1};
//   }

//   STK_INLINE_FUNCTION
//   static constexpr bool get_is_valid() { return tdata0::is_valid && tdata1::is_valid; }

//   STK_INLINE_FUNCTION
//   static constexpr bool get_has_homogeneous_faces()
//   {
//     return tdata0::has_homogeneous_faces && tdata1::has_homogeneous_faces;
//   }

//   STK_INLINE_FUNCTION
//   static constexpr bool get_is_shell() { return tdata0::is_shell && tdata1::is_shell; }

//   STK_INLINE_FUNCTION
//   static constexpr uint8_t get_dimension() { return Kokkos::max(tdata0::dimension, tdata1::dimension); }

//   STK_INLINE_FUNCTION
//   static constexpr unsigned get_num_nodes() { return data0::num_nodes + tdata1::num_nodes; }

//   STK_INLINE_FUNCTION
//   static constexpr unsigned get_num_vertices() { return tdata0::num_vertices + tdata1::num_vertices; }

//   STK_INLINE_FUNCTION
//   static constexpr unsigned get_num_edges() { return tdata0::num_edges + tdata1::num_edges; }

//   STK_INLINE_FUNCTION
//   static constexpr unsigned get_num_faces() { return tdata0::num_faces + tdata1::num_faces; }

//   STK_INLINE_FUNCTION
//   static constexpr Kokkos::Array<bool, 4> get_spatial_dimension_vector()
//   {
//     return {tdata0::spatial_dimension_vector[0] && tdata1::spatial_dimension_vector[0],  // 0d
//         tdata0::spatial_dimension_vector[1] && tdata1::spatial_dimension_vector[1],      // 1d
//         tdata0::spatial_dimension_vector[2] && tdata1::spatial_dimension_vector[2],      // 2d
//         tdata0::spatial_dimension_vector[3] && tdata1::spatial_dimension_vector[3]};     // 3d
//   }

//   STK_INLINE_FUNCTION
//   static constexpr auto get_edge_node_ordinals_offsets()
//   {
//     if constexpr (tdata0::num_edges > 0) {
//       if constexpr (tdata1::num_edges > 0) {
//         return stk_concat_ordinal_offsets(tdata0::edge_node_ordinals_offsets, tdata1::edge_node_ordinals_offsets);
//       } else {
//         return tdata0::edge_node_ordinals_offsets;
//       }
//     } else {
//       if constexpr (tdata1::num_edges > 0) {
//         return tdata1::edge_node_ordinals_offsets;
//       } else {
//         return {0};
//       }
//     }
//   }

//   STK_INLINE_FUNCTION
//   static constexpr auto get_edge_node_ordinals_vector()
//   {
//     if constexpr (tdata0::num_edges > 0) {
//       if constexpr (tdata1::num_edges > 0) {
//         return stk_concat_ordinal_vectors(
//             tdata0::num_nodes, tdata0::edge_node_ordinals_vector, tdata1::edge_node_ordinals_vector);
//       } else {
//         return tdata0::edge_node_ordinals_vector;
//       }
//     } else {
//       if constexpr (tdata1::num_edges > 0) {
//         return tdata1::edge_node_ordinals_vector;
//       } else {
//         return {0};
//       }
//     }
//   }

//   STK_INLINE_FUNCTION
//   static constexpr auto get_face_node_ordinals_offsets()
//   {
//     if constexpr (tdata0::num_faces > 0) {
//       if constexpr (tdata1::num_faces > 0) {
//         return stk_concat_ordinal_offsets(tdata0::face_node_ordinals_offsets, tdata1::face_node_ordinals_offsets);
//       } else {
//         return tdata0::face_node_ordinals_offsets;
//       }
//     } else {
//       if constexpr (tdata1::num_faces > 0) {
//         return tdata1::face_node_ordinals_offsets;
//       } else {
//         return {0};
//       }
//     }
//   }

//   STK_INLINE_FUNCTION
//   static constexpr auto get_face_node_ordinals_vector()
//   {
//     if constexpr (tdata0::num_faces > 0) {
//       if constexpr (tdata1::num_faces > 0) {
//         return stk_concat_ordinal_vectors(
//             tdata0::num_nodes, tdata0::face_node_ordinals_vector, tdata1::face_node_ordinals_vector);
//       } else {
//         return tdata0::face_node_ordinals_vector;
//       }
//     } else {
//       if constexpr (tdata1::num_faces > 0) {
//         return tdata1::face_node_ordinals_vector;
//       } else {
//         return {0};
//       }
//     }
//   }

//   STK_INLINE_FUNCTION
//   static constexpr constexpr Kokkos::Array<uint8_t, 3> get_element_node_ordinals_offsets()
//   {
//     return {0, tdata0::num_nodes, num_nodes};
//   }

//   STK_INLINE_FUNCTION
//   static constexpr auto get_element_node_ordinals_vector() { return stk_arange<uint8_t, num_nodes>(); }
// };

// Using parameter packs to perform constexpr concatenation of two arrays
template <typename T, size_t ArraySize1, size_t ArraySize2, size_t... Indices1, size_t... Indices2>
STK_INLINE_FUNCTION constexpr Kokkos::Array<T, ArraySize1 + ArraySize2> stk_concat_arrays_impl(
    const Kokkos::Array<T, ArraySize1>& array1,
    const Kokkos::Array<T, ArraySize2>& array2,
    std::index_sequence<Indices1...>,
    std::index_sequence<Indices2...>)
{
  return Kokkos::Array<T, ArraySize1 + ArraySize2>({array1[Indices1]..., array2[Indices2]...});
}

template <typename T, size_t ArraySize1, size_t ArraySize2>
STK_INLINE_FUNCTION constexpr Kokkos::Array<T, ArraySize1 + ArraySize2> stk_concat_arrays(
    const Kokkos::Array<T, ArraySize1>& array1, const Kokkos::Array<T, ArraySize2>& array2)
{
  return stk_concat_arrays_impl(
      array1, array2, std::make_index_sequence<ArraySize1>{}, std::make_index_sequence<ArraySize2>{});
}

// Using parameter packs to perform constexpr concatenation of ordinal offsets
template <typename T, size_t ArraySize1, size_t ArraySize2, size_t... Indices1, size_t... Indices2>
STK_INLINE_FUNCTION constexpr Kokkos::Array<T, ArraySize1 - 1 + ArraySize2> stk_concat_ordinal_offsets_impl(
    const Kokkos::Array<T, ArraySize1>& ordinal_offsets1,
    const Kokkos::Array<T, ArraySize2>& ordinal_offsets2,
    std::index_sequence<Indices1...>,
    std::index_sequence<Indices2...>)
{
  return Kokkos::Array<T, ArraySize1 - 1 + ArraySize2>({ordinal_offsets1[Indices1]...,
      static_cast<T>(ordinal_offsets1[ArraySize1 - 1] + ordinal_offsets2[Indices2])...});
}

template <typename T, size_t ArraySize1, size_t ArraySize2>
STK_INLINE_FUNCTION constexpr Kokkos::Array<T, ArraySize1 - 1 + ArraySize2> stk_concat_ordinal_offsets(
    const Kokkos::Array<T, ArraySize1>& ordinal_offsets1, const Kokkos::Array<T, ArraySize2>& ordinal_offsets2)
{
  return stk_concat_ordinal_offsets_impl(ordinal_offsets1, ordinal_offsets2, std::make_index_sequence<ArraySize1 - 1>{},
      std::make_index_sequence<ArraySize2>{});
}

// Using parameter packs to perform constexpr concatenation of ordinal vectors
template <typename T, size_t ArraySize1, size_t ArraySize2, size_t... Indices1, size_t... Indices2>
STK_INLINE_FUNCTION constexpr Kokkos::Array<T, ArraySize1 + ArraySize2> stk_concat_ordinal_vectors_impl(
    const size_t shift,
    const Kokkos::Array<T, ArraySize1>& ordinal_vector1,
    const Kokkos::Array<T, ArraySize2>& ordinal_vector2,
    std::index_sequence<Indices1...>,
    std::index_sequence<Indices2...>)
{
  return Kokkos::Array<T, ArraySize1 + ArraySize2>(
      {ordinal_vector1[Indices1]..., static_cast<T>(shift + ordinal_vector2[Indices2])...});
}

template <typename T, size_t ArraySize1, size_t ArraySize2>
STK_INLINE_FUNCTION constexpr Kokkos::Array<T, ArraySize1 + ArraySize2> stk_concat_ordinal_vectors(const size_t shift,
    const Kokkos::Array<T, ArraySize1>& ordinal_vector1,
    const Kokkos::Array<T, ArraySize2>& ordinal_vector2)
{
  return stk_concat_ordinal_vectors_impl(shift, ordinal_vector1, ordinal_vector2,
      std::make_index_sequence<ArraySize1>{}, std::make_index_sequence<ArraySize2>{});
}

// Using parameter packs to generate a constexpr vector containing 0, 1, 2, ..., N-1
template <typename T, size_t N, size_t... Indices>
STK_INLINE_FUNCTION constexpr Kokkos::Array<T, N> stk_arange_impl(std::index_sequence<Indices...>)
{
  return Kokkos::Array<T, N>({static_cast<T>(Indices)...});
}

template <typename T, size_t N>
STK_INLINE_FUNCTION constexpr Kokkos::Array<T, N> stk_arange()
{
  return stk_arange_impl<T, N>(std::make_index_sequence<N>{});
}

}  // namespace topology_detail
}  // namespace stk

#endif  // STKTOPOLOGY_DETAIL_LINKER_HELPER_HPP
