//@HEADER
// ************************************************************************
//
//                        Kokkos v. 4.0
//       Copyright (2022) National Technology & Engineering
//               Solutions of Sandia, LLC (NTESS).
//
// Under the terms of Contract DE-NA0003525 with NTESS,
// the U.S. Government retains certain rights in this software.
//
// Part of Kokkos, under the Apache License v2.0 with LLVM Exceptions.
// See https://kokkos.org/LICENSE for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//@HEADER

#ifndef KOKKOSBLAS1_FILL_HPP_
#define KOKKOSBLAS1_FILL_HPP_

#include <Kokkos_Core.hpp>

namespace KokkosBlas {

/// \brief Fill the multivector or single vector X with the given value.
///
/// \tparam XMV 1-D or 2-D output View
///
/// \param X [out] Output View (1-D or 2-D).
/// \param val [in] Value with which to fill the entries of X.
template <class XMV>
void fill(const XMV& X, const typename XMV::non_const_value_type& val) {
  Kokkos::Profiling::pushRegion("KokkosBlas::fill");
  Kokkos::deep_copy(X, val);
  Kokkos::Profiling::popRegion();
}

}  // namespace KokkosBlas

#endif  // KOKKOSBLAS1_ABS_HPP_
