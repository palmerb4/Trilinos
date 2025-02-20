// @HEADER
// ************************************************************************
//
//                           Intrepid2 Package
//                 Copyright (2007) Sandia Corporation
//
// Under terms of Contract DE-AC04-94AL85000, there is a non-exclusive
// license for use of this work by or on behalf of the U.S. Government.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the Corporation nor the names of the
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY SANDIA CORPORATION "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SANDIA CORPORATION OR THE
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Questions? Contact Kyungjoo Kim  (kyukim@sandia.gov), or
//                    Mauro Perego  (mperego@sandia.gov)
//
// ************************************************************************
// @HEADER

/** \file   Intrepid2_HGRAD_QUAD_C2_FEMDef.hpp
    \brief  Definition file for FEM basis functions of degree 2 for H(grad) functions on QUAD cells.
    \author Created by P. Bochev and D. Ridzal.
            Kokkorized by Kyungjoo Kim
*/

#ifndef __INTREPID2_HGRAD_QUAD_C2_FEM_DEF_HPP__
#define __INTREPID2_HGRAD_QUAD_C2_FEM_DEF_HPP__

namespace Intrepid2 {

  // -------------------------------------------------------------------------------------

  namespace Impl {                                                                                                                                                       

    template<bool serendipity>                                                                                                                                                          
    template<EOperator opType>                                                                                                                                           
    template<typename OutputViewType,                                                                                                                                    
             typename inputViewType>                                                                                                                                     
    KOKKOS_INLINE_FUNCTION                                                                                                                                               
    void                                                                                                                                                                 
    Basis_HGRAD_QUAD_DEG2_FEM<serendipity>::Serial<opType>::                                                                                                                             
    getValues(       OutputViewType output,                                                                                                                              
               const inputViewType input ) {   
    switch (opType) {
    case OPERATOR_VALUE : {
      const auto x = input(0);
      const auto y = input(1);

      // output is a rank-2 array with dimensions (basisCardinality_, dim0)
      output.access(0) = x*(x - 1.0)*y*(y - 1.0)/4.0;
      output.access(1) = x*(x + 1.0)*y*(y - 1.0)/4.0;
      output.access(2) = x*(x + 1.0)*y*(y + 1.0)/4.0;
      output.access(3) = x*(x - 1.0)*y*(y + 1.0)/4.0;
      // edge midpoints basis functions
      output.access(4) = (1.0 - x)*(1.0 + x)*y*(y - 1.0)/2.0;
      output.access(5) = x*(x + 1.0)*(1.0 - y)*(1.0 + y)/2.0;
      output.access(6) = (1.0 - x)*(1.0 + x)*y*(y + 1.0)/2.0;
      output.access(7) = x*(x - 1.0)*(1.0 - y)*(1.0 + y)/2.0;

      // quad bubble basis function
      if constexpr (!serendipity) {
        output.access(8) = (1.0 - x)*(1.0 + x)*(1.0 - y)*(1.0 + y); 
      }
      break;
    }
    case OPERATOR_D1 :
    case OPERATOR_GRAD : {
      const auto x = input(0);
      const auto y = input(1);

      // output.access is a rank-3 array with dimensions (basisCardinality_, dim0, spaceDim)
      output.access(0, 0) = (-0.25 + 0.5*x)*(-1. + y)*y;
      output.access(0, 1) = (-1.0 + x)*x*(-0.25 + 0.5*y);
      
      output.access(1, 0) = (0.25 + 0.5*x)*(-1. + y)*y;
      output.access(1, 1) = x*(1. + x)*(-0.25 + 0.5*y);
      
      output.access(2, 0) = (0.25 + 0.5*x)*y*(1. + y);
      output.access(2, 1) = x*(1. + x)*(0.25 + 0.5*y);
 
      output.access(3, 0) = (-0.25 + 0.5*x)*y*(1. + y);
      output.access(3, 1) = (-1. + x)*x*(0.25 + 0.5*y);

      output.access(4, 0) = x*(1.0 - y)*y;
      output.access(4, 1) = 0.5*(1.0 - x)*(1.0 + x)*(-1.0 + 2.0*y);
        
      output.access(5, 0) = 0.5*(1.0 - y)*(1.0 + y)*(1.0 + 2.0*x);
      output.access(5, 1) =-x*(1.0 + x)*y;
        
      output.access(6, 0) =-y*(1.0 + y)*x;
      output.access(6, 1) = 0.5*(1.0 - x)*(1.0 + x)*(1.0 + 2.0*y);
        
      output.access(7, 0) = 0.5*(1.0 - y)*(1.0+ y)*(-1.0 + 2.0*x);
      output.access(7, 1) = (1.0 - x)*x*y;
 
      if constexpr (!serendipity) {
        output.access(8, 0) =-2.0*(1.0 - y)*(1.0 + y)*x;
        output.access(8, 1) =-2.0*(1.0 - x)*(1.0 + x)*y;
      }
      break;
    }
    case OPERATOR_CURL : {
      const auto x = input(0);
      const auto y = input(1);

      // output.access is a rank-3 array with dimensions (basisCardinality_, dim0, spaceDim)
      // CURL(u) = (u_y, -u_x), is rotated GRAD
      output.access(0, 1) =-(-0.25 + 0.5*x)*(-1. + y)*y;
      output.access(0, 0) = (-1.0 + x)*x*(-0.25 + 0.5*y);
      
      output.access(1, 1) =-(0.25 + 0.5*x)*(-1. + y)*y;
      output.access(1, 0) = x*(1. + x)*(-0.25 + 0.5*y);
      
      output.access(2, 1) =-(0.25 + 0.5*x)*y*(1. + y);
      output.access(2, 0) = x*(1. + x)*(0.25 + 0.5*y);
      
      output.access(3, 1) =-(-0.25 + 0.5*x)*y*(1. + y);
      output.access(3, 0) = (-1. + x)*x*(0.25 + 0.5*y);
      
      output.access(4, 1) =-x*(1.0 - y)*y;
      output.access(4, 0) = 0.5*(1.0 - x)*(1.0 + x)*(-1.0 + 2.0*y);
      
      output.access(5, 1) =-0.5*(1.0 - y)*(1.0 + y)*(1.0 + 2.0*x);
      output.access(5, 0) =-x*(1.0 + x)*y;
      
      output.access(6, 1) = y*(1.0 + y)*x;
      output.access(6, 0) = 0.5*(1.0 - x)*(1.0 + x)*(1.0 + 2.0*y);
      
      output.access(7, 1) =-0.5*(1.0 - y)*(1.0 + y)*(-1.0 + 2.0*x);
      output.access(7, 0) = (1.0 - x)*x*y;

      if constexpr (!serendipity) {
        output.access(8, 1) = 2.0*(1.0 - y)*(1.0 + y)*x;
        output.access(8, 0) =-2.0*(1.0 - x)*(1.0 + x)*y;
      }          
      break;
    }
    case OPERATOR_D2 : {
      const auto x = input(0);
      const auto y = input(1);
      // output.access is a rank-3 array with dimensions (basisCardinality_, dim0, D2Cardinality=3) 
      output.access(0, 0) = 0.5*(-1.0 + y)*y;
      output.access(0, 1) = 0.25 - 0.5*y + x*(-0.5 + 1.*y);
      output.access(0, 2) = 0.5*(-1.0 + x)*x;

      output.access(1, 0) = 0.5*(-1.0 + y)*y;
      output.access(1, 1) =-0.25 + 0.5*y + x*(-0.5 + 1.*y);
      output.access(1, 2) = 0.5*x*(1.0 + x);
      
      output.access(2, 0) = 0.5*y*(1.0 + y);
      output.access(2, 1) = 0.25 + 0.5*y + x*(0.5 + 1.*y);
      output.access(2, 2) = 0.5*x*(1.0 + x);
      
      output.access(3, 0) = 0.5*y*(1.0 + y);
      output.access(3, 1) =-0.25 - 0.5*y + x*(0.5 + 1.*y);
      output.access(3, 2) = 0.5*(-1.0 + x)*x;
      
      output.access(4, 0) = (1.0 - y)*y;
      output.access(4, 1) = x*(1. - 2.*y);
      output.access(4, 2) = (1.0 - x)*(1.0 + x);

      output.access(5, 0) = (1.0 - y)*(1.0 + y);
      output.access(5, 1) = x*(0. - 2.*y) - 1.*y;
      output.access(5, 2) =-x*(1.0 + x);

      output.access(6, 0) =-y*(1.0 + y);
      output.access(6, 1) = x*(-1. - 2.*y);
      output.access(6, 2) = (1.0 - x)*(1.0 + x);

      output.access(7, 0) = (1.0 - y)*(1.0 + y);
      output.access(7, 1) = x*(0. - 2.*y) + 1.*y;
      output.access(7, 2) = (1.0 - x)*x;

      if constexpr (!serendipity) {
        output.access(8, 0) =-2.0 + 2.0*y*y;
        output.access(8, 1) = 4*x*y;
        output.access(8, 2) =-2.0 + 2.0*x*x;
      }
      break;
    }
    case OPERATOR_D3 : {
      const auto x = input(0);
      const auto y = input(1);
      output.access(0, 0) = 0.0;
      output.access(0, 1) =-0.5 + y;
      output.access(0, 2) =-0.5 + x;
      output.access(0, 3) = 0.0;

      output.access(1, 0) = 0.0;
      output.access(1, 1) =-0.5 + y;
      output.access(1, 2) = 0.5 + x;
      output.access(1, 3) = 0.0;

      output.access(2, 0) = 0.0;
      output.access(2, 1) = 0.5 + y;
      output.access(2, 2) = 0.5 + x;
      output.access(2, 3) = 0.0;

      output.access(3, 0) = 0.0;
      output.access(3, 1) = 0.5 + y;
      output.access(3, 2) =-0.5 + x;
      output.access(3, 3) = 0.0;

      output.access(4, 0) = 0.0;
      output.access(4, 1) = 1.0 - 2.0*y;
      output.access(4, 2) =-2.0*x;
      output.access(4, 3) = 0.0;

      output.access(5, 0) = 0.0;
      output.access(5, 1) =-2.0*y;
      output.access(5, 2) =-1.0 - 2.0*x;
      output.access(5, 3) = 0.0;

      output.access(6, 0) = 0.0;
      output.access(6, 1) =-1.0 - 2.0*y;
      output.access(6, 2) =-2.0*x;
      output.access(6, 3) = 0.0;

      output.access(7, 0) = 0.0;
      output.access(7, 1) =-2.0*y;
      output.access(7, 2) = 1.0 - 2.0*x;
      output.access(7, 3) = 0.0;        
      
      if constexpr (!serendipity) {
        output.access(8, 0) = 0.0;
        output.access(8, 1) = 4.0*y;
        output.access(8, 2) = 4.0*x;
        output.access(8, 3) = 0.0;
      }              
      break;
    }
    case OPERATOR_D4 : {
      output.access(0, 0) = 0.0;
      output.access(0, 1) = 0.0;
      output.access(0, 2) = 1.0;
      output.access(0, 3) = 0.0;                
      output.access(0, 4) = 0.0;                

      output.access(1, 0) = 0.0;
      output.access(1, 1) = 0.0;
      output.access(1, 2) = 1.0;
      output.access(1, 3) = 0.0;                
      output.access(1, 4) = 0.0;                
      
      output.access(2, 0) = 0.0;
      output.access(2, 1) = 0.0;
      output.access(2, 2) = 1.0;
      output.access(2, 3) = 0.0;                
      output.access(2, 4) = 0.0;                
      
      output.access(3, 0) = 0.0;
      output.access(3, 1) = 0.0;
      output.access(3, 2) = 1.0;
      output.access(3, 3) = 0.0;                
      output.access(3, 4) = 0.0;                
      
      output.access(4, 0) = 0.0;
      output.access(4, 1) = 0.0;
      output.access(4, 2) =-2.0;
      output.access(4, 3) = 0.0;                
      output.access(4, 4) = 0.0;                

      output.access(5, 0) = 0.0;
      output.access(5, 1) = 0.0;
      output.access(5, 2) =-2.0;
      output.access(5, 3) = 0.0;                
      output.access(5, 4) = 0.0;                
      
      output.access(6, 0) = 0.0;
      output.access(6, 1) = 0.0;
      output.access(6, 2) =-2.0;
      output.access(6, 3) = 0.0;                
      output.access(6, 4) = 0.0;                
      
      output.access(7, 0) = 0.0;
      output.access(7, 1) = 0.0;
      output.access(7, 2) =-2.0;
      output.access(7, 3) = 0.0;                
      output.access(7, 4) = 0.0;                
      
      if constexpr (!serendipity) {
        output.access(8, 0) = 0.0;
        output.access(8, 1) = 0.0;
        output.access(8, 2) = 4.0;
        output.access(8, 3) = 0.0;                
        output.access(8, 4) = 0.0;
      }            
      break;
    }
    case OPERATOR_MAX : {
      const ordinal_type jend = output.extent(1);
      const ordinal_type iend = output.extent(0);

      for (ordinal_type j=0;j<jend;++j)
        for (ordinal_type i=0;i<iend;++i)
          output.access(i, j) = 0.0;
      break;
    }
    default: {
      INTREPID2_TEST_FOR_ABORT( opType != OPERATOR_VALUE &&
                                opType != OPERATOR_GRAD &&
                                opType != OPERATOR_CURL &&
                                opType != OPERATOR_D1 &&
                                opType != OPERATOR_D2 &&
                                opType != OPERATOR_D3 &&
                                opType != OPERATOR_D4 &&
                                opType != OPERATOR_MAX,
                                ">>> ERROR: (Intrepid2::Basis_HGRAD_QUAD_C2_FEM::Serial::getValues) operator is not supported");

    }
    }
  }
 
  template<bool serendipity>
  template<typename DT, 
  typename outputValueValueType, class ...outputValueProperties,
           typename inputPointValueType,  class ...inputPointProperties>
  void 
  Basis_HGRAD_QUAD_DEG2_FEM<serendipity>::
  getValues(       Kokkos::DynRankView<outputValueValueType,outputValueProperties...> outputValues,
             const Kokkos::DynRankView<inputPointValueType, inputPointProperties...>  inputPoints,
             const EOperator operatorType ) {
    typedef          Kokkos::DynRankView<outputValueValueType,outputValueProperties...>         outputValueViewType;
    typedef          Kokkos::DynRankView<inputPointValueType, inputPointProperties...>          inputPointViewType;
    typedef typename ExecSpace<typename inputPointViewType::execution_space,typename DT::execution_space>::ExecSpaceType ExecSpaceType;

    // Number of evaluation points = dim 0 of inputPoints
    const auto loopSize = inputPoints.extent(0);
    Kokkos::RangePolicy<ExecSpaceType,Kokkos::Schedule<Kokkos::Static> > policy(0, loopSize);
  
    switch (operatorType) {
    
    case OPERATOR_VALUE: {
      typedef Functor<outputValueViewType,inputPointViewType,OPERATOR_VALUE> FunctorType;
      Kokkos::parallel_for( policy, FunctorType(outputValues, inputPoints) );
      break;
    } 
    case OPERATOR_GRAD:
    case OPERATOR_D1: {
      typedef Functor<outputValueViewType,inputPointViewType,OPERATOR_GRAD> FunctorType;
      Kokkos::parallel_for( policy, FunctorType(outputValues, inputPoints) );
      break;
    } 
    case OPERATOR_CURL: {
      typedef Functor<outputValueViewType,inputPointViewType,OPERATOR_CURL> FunctorType;
      Kokkos::parallel_for( policy, FunctorType(outputValues, inputPoints) );
      break;
    } 
    case OPERATOR_DIV: {
      INTREPID2_TEST_FOR_EXCEPTION( (operatorType == OPERATOR_DIV), std::invalid_argument,
                                    ">>> ERROR (Basis_HGRAD_QUAD_C2_FEM): DIV is invalid operator for rank-0 (scalar) functions in 2D");
      break;
    } 
    case OPERATOR_D2: {
      typedef Functor<outputValueViewType,inputPointViewType,OPERATOR_D2> FunctorType;
      Kokkos::parallel_for( policy, FunctorType(outputValues, inputPoints) );
      break;
    } 
    case OPERATOR_D3: {
      // outputValues is a rank-3 array with dimensions (basisCardinality_, dim0, D3Cardinality=4) 
      typedef Functor<outputValueViewType,inputPointViewType,OPERATOR_D3> FunctorType;
      Kokkos::parallel_for( policy, FunctorType(outputValues, inputPoints) );
      break;
    } 
    case OPERATOR_D4: {
      // outputValues is a rank-3 array with dimensions (basisCardinality_, dim0, D4Cardinality=5) 
      typedef Functor<outputValueViewType,inputPointViewType,OPERATOR_D4> FunctorType;
      Kokkos::parallel_for( policy, FunctorType(outputValues, inputPoints) );
      break;
    } 
    case OPERATOR_D5:
    case OPERATOR_D6:
    case OPERATOR_D7:
    case OPERATOR_D8:
    case OPERATOR_D9:
    case OPERATOR_D10: {
      typedef Functor<outputValueViewType,inputPointViewType,OPERATOR_MAX> FunctorType;
      Kokkos::parallel_for( policy, FunctorType(outputValues, inputPoints) );
      break;
    }
    default: {
      INTREPID2_TEST_FOR_EXCEPTION( !( Intrepid2::isValidOperator(operatorType) ), std::invalid_argument,
                                    ">>> ERROR (Basis_HGRAD_QUAD_C2_FEM): Invalid operator type");
    }
    }
  }

  }
  // -------------------------------------------------------------------------------------


  template<bool serendipity, typename DT, typename OT, typename PT>
  Basis_HGRAD_QUAD_DEG2_FEM<serendipity, DT,OT,PT>::
  Basis_HGRAD_QUAD_DEG2_FEM() {
    this->basisCardinality_  = serendipity ? 8 : 9;
    this->basisDegree_       = 2;    
    this->basisCellTopology_ = shards::CellTopology(shards::getCellTopologyData<shards::Quadrilateral<4> >() );
    this->basisType_         = BASIS_FEM_DEFAULT;
    this->basisCoordinates_  = COORDINATES_CARTESIAN;
    this->functionSpace_     = FUNCTION_SPACE_HGRAD;

    {
      // Basis-dependent intializations
      const ordinal_type tagSize  = 4;        // size of DoF tag, i.e., number of fields in the tag
      const ordinal_type posScDim = 0;        // position in the tag, counting from 0, of the subcell dim 
      const ordinal_type posScOrd = 1;        // position in the tag, counting from 0, of the subcell ordinal
      const ordinal_type posDfOrd = 2;        // position in the tag, counting from 0, of DoF ordinal relative to the subcell
  
      // An array with local DoF tags assigned to basis functions, in the order of their local enumeration 
      ordinal_type tags[36]  = { 0, 0, 0, 1,
                                 0, 1, 0, 1,
                                 0, 2, 0, 1,
                                 0, 3, 0, 1,
                                 // edge midpoints
                                 1, 0, 0, 1,
                                 1, 1, 0, 1,
                                 1, 2, 0, 1,
                                 1, 3, 0, 1,
                                 // quad center, not used for serendipity elements
                                 2, 0, 0, 1};
  
      //host view
      OrdinalTypeArray1DHost tagView(&tags[0], serendipity ? 32 : 36);
    
      // Basis-independent function sets tag and enum data in tagToOrdinal_ and ordinalToTag_ arrays:
      this->setOrdinalTagData(this->tagToOrdinal_,
                              this->ordinalToTag_,
                              tagView,
                              this->basisCardinality_,
                              tagSize,
                              posScDim,
                              posScOrd,
                              posDfOrd);
    }

    // dofCoords on host and create its mirror view to device
    Kokkos::DynRankView<typename ScalarViewType::value_type,typename DT::execution_space::array_layout,Kokkos::HostSpace>
      dofCoords("dofCoordsHost", this->basisCardinality_,this->basisCellTopology_.getDimension());
    
    dofCoords(0,0) = -1.0;   dofCoords(0,1) = -1.0;
    dofCoords(1,0) =  1.0;   dofCoords(1,1) = -1.0;
    dofCoords(2,0) =  1.0;   dofCoords(2,1) =  1.0;
    dofCoords(3,0) = -1.0;   dofCoords(3,1) =  1.0;
  
    dofCoords(4,0) =  0.0;   dofCoords(4,1) = -1.0;
    dofCoords(5,0) =  1.0;   dofCoords(5,1) =  0.0;
    dofCoords(6,0) =  0.0;   dofCoords(6,1) =  1.0;
    dofCoords(7,0) = -1.0;   dofCoords(7,1) =  0.0;
  
    if constexpr (!serendipity) {
      dofCoords(8,0) =  0.0;   dofCoords(8,1) =  0.0;
    }

    this->dofCoords_ = Kokkos::create_mirror_view(typename DT::memory_space(), dofCoords);
    Kokkos::deep_copy(this->dofCoords_, dofCoords);   
  }

}// namespace Intrepid2
#endif
