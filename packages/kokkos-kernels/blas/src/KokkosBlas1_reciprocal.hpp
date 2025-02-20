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

#ifndef KOKKOSBLAS1_RECIPROCAL_HPP_
#define KOKKOSBLAS1_RECIPROCAL_HPP_

#include <KokkosBlas1_reciprocal_spec.hpp>
#include <KokkosKernels_helpers.hpp>
#include <KokkosKernels_Error.hpp>

namespace KokkosBlas {

/// \brief R(i,j) = reciprocal(X(i,j))
///
/// Replace each entry in R with the absolute value (magnitude), of the
/// reciprocal of the corresponding entry in X.
///
/// \tparam RMV 1-D or 2-D Kokkos::View specialization.
/// \tparam XMV 1-D or 2-D Kokkos::View specialization.  It must have
///   the same rank as RMV, and its entries must be assignable to
///   those of RMV.
template <class RMV, class XMV>
void reciprocal(const RMV& R, const XMV& X) {
  static_assert(Kokkos::is_view<RMV>::value,
                "KokkosBlas::reciprocal: "
                "R is not a Kokkos::View.");
  static_assert(Kokkos::is_view<XMV>::value,
                "KokkosBlas::reciprocal: "
                "X is not a Kokkos::View.");
  static_assert(std::is_same<typename RMV::value_type,
                             typename RMV::non_const_value_type>::value,
                "KokkosBlas::reciprocal: R is const.  "
                "It must be nonconst, because it is an output argument "
                "(we have to be able to write to its entries).");
  static_assert(int(RMV::rank) == int(XMV::rank),
                "KokkosBlas::reciprocal: "
                "R and X must have the same rank.");
  static_assert(RMV::rank == 1 || RMV::rank == 2,
                "KokkosBlas::reciprocal: "
                "RMV and XMV must either have rank 1 or rank 2.");

  // Check compatibility of dimensions at run time.
  if (X.extent(0) != R.extent(0) || X.extent(1) != R.extent(1)) {
    std::ostringstream os;
    os << "KokkosBlas::reciprocal (MV): Dimensions of R and X do not match: "
       << "R: " << R.extent(0) << " x " << R.extent(1) << ", X: " << X.extent(0)
       << " x " << X.extent(1);
    KokkosKernels::Impl::throw_runtime_exception(os.str());
  }

  // Create unmanaged versions of the input Views.  RMV and XMV may be
  // rank 1 or rank 2.
  typedef Kokkos::View<
      typename std::conditional<RMV::rank == 1,
                                typename RMV::non_const_value_type*,
                                typename RMV::non_const_value_type**>::type,
      typename KokkosKernels::Impl::GetUnifiedLayout<RMV>::array_layout,
      typename RMV::device_type, Kokkos::MemoryTraits<Kokkos::Unmanaged> >
      RMV_Internal;
  typedef Kokkos::View<
      typename std::conditional<XMV::rank == 1, typename XMV::const_value_type*,
                                typename XMV::const_value_type**>::type,
      typename KokkosKernels::Impl::GetUnifiedLayout<XMV>::array_layout,
      typename XMV::device_type, Kokkos::MemoryTraits<Kokkos::Unmanaged> >
      XMV_Internal;

  RMV_Internal R_internal = R;
  XMV_Internal X_internal = X;

  Impl::Reciprocal<RMV_Internal, XMV_Internal>::reciprocal(R_internal,
                                                           X_internal);
}
}  // namespace KokkosBlas

#endif  // KOKKOSBLAS1_RECIPROCAL_HPP_
