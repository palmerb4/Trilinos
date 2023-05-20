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
#ifndef STKTOPOLOGY_APPLY_FUNCTOR_TCC
#define STKTOPOLOGY_APPLY_FUNCTOR_TCC

#include "stk_topology/topology_decl.hpp"
#include "stk_topology/types.hpp"

namespace stk {

namespace topology_detail
{

/// Static functor wrapper
template <typename Functor, topology::topology_t T>
struct FunctorWrapper {
  typedef typename Functor::result_type result_type;
  static result_type apply(Functor functor) { return functor(topology::topology_type<T>()); }
};

/// Static jump table that maps run-time topology_t to compile-time topology_type<topology_t>
template <typename Functor, unsigned... Ts>
const auto get_functor_jump_table(std::integer_sequence<unsigned, Ts...> /* int_seq */)
{
  typedef typename Functor::result_type result_type;
  static constexpr result_type (*jump_table[])(Functor functor) = {
      FunctorWrapper<Functor, static_cast<topology::topology_t>(Ts)>::apply...};
  return jump_table;
}

}  // namespace topology_detail

//*****************************************************************************
// Converts a runtime topology to a compile-time topology_type<Topology>
// and calls the given functor on the compile-time topology
//*****************************************************************************
template <typename Functor>
struct topology::apply_host_functor {
  typedef typename Functor::result_type result_type;

  apply_host_functor() : m_functor() {}

  apply_host_functor(Functor f) : m_functor(f) {}

  result_type operator()(topology_t t) const
  {
    auto jump_table = topology_detail::get_functor_jump_table<Functor>(
        std::make_integer_sequence<unsigned, static_cast<unsigned>(topology::LINKER_END + 1)>{});

    if (t <= topology::LINKER_END) {
      return jump_table[t](m_functor);
    } else {
      return m_functor(topology_type<INVALID_TOPOLOGY>());
    }
  }

  result_type operator()(topology_t t)
  {
    auto jump_table = topology_detail::get_functor_jump_table<Functor>(
        std::make_integer_sequence<unsigned, static_cast<unsigned>(topology::LINKER_END + 1)>{});

    if (t <= topology::LINKER_END) {
      return jump_table[t](m_functor);
    } else {
      return m_functor(topology_type<INVALID_TOPOLOGY>());
    }
  }

  Functor m_functor;
};

template <typename Functor>
struct topology::apply_functor
{
  typedef typename Functor::result_type result_type;

  STK_FUNCTION
  apply_functor()
    : m_functor()
  {}

  STK_FUNCTION
  apply_functor(Functor f)
    : m_functor(f)
  {}

  STK_FUNCTION
  result_type operator()(topology_t t) const
  {
    auto jump_table = topology_detail::get_functor_jump_table<Functor>(
        std::make_integer_sequence<unsigned, static_cast<unsigned>(topology::LINKER_END + 1)>{});

    if (t <= topology::LINKER_END) {
      return jump_table[t](m_functor);
    } else {
      return m_functor(topology_type<INVALID_TOPOLOGY>());
    }
  }

  STK_FUNCTION
  result_type operator()(topology_t t)
  {
    auto jump_table = topology_detail::get_functor_jump_table<Functor>(
        std::make_integer_sequence<unsigned, static_cast<unsigned>(topology::LINKER_END + 1)>{});

    if (t <= topology::LINKER_END) {
      return jump_table[t](m_functor);
    } else {
      return m_functor(topology_type<INVALID_TOPOLOGY>());
    }
  }

  Functor m_functor;
};

} //namespace stk

#endif //STKTOPOLOGY_APPLY_FUNCTOR_TCC
