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
