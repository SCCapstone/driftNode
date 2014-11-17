#ifndef _LINALG_TOOLS_HPP_
#define _LINALG_TOOLS_HPP_

#include "types.hpp"

// courtesy of Thomas Lemaire

#if BOOST_VERSION < 103301 // missing includes in lu.hpp
#include "boost/numeric/ublas/vector.hpp"
#include "boost/numeric/ublas/vector_proxy.hpp"
#include "boost/numeric/ublas/matrix.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "boost/numeric/ublas/triangular.hpp"
#include "boost/numeric/ublas/operation.hpp"
#endif

#include <boost/numeric/ublas/lu.hpp>

// forget about JFR_PRECOND macros...
typedef boost::numeric::ublas::matrix<double> mat;
typedef boost::numeric::ublas::identity_matrix<double> identity_mat;



/** General matrix inversion routine.
* It uses lu_factorize and lu_substitute in uBLAS to invert a matrix
*/
template<class M1, class M2>
void lu_inv(M1 const& m, M2& inv) {
  
  assert( m.size1() == m.size2() );
  // ublasExtra::lu_inv(): input matrix must be squared
  assert( inv.size1() == m.size1() && inv.size1() == m.size2() );
  // ublasExtra::lu_inv(): invalid size for inverse matrix

  using namespace boost::numeric::ublas;
  // create a working copy of the input
  mat mLu(m);

  // perform LU-factorization
  lu_factorize(mLu);

  // create identity matrix of "inverse"
  inv.assign(identity_mat(m.size1()));

  // backsubstitute to get the inverse
  lu_substitute<mat const, M2>(mLu, inv);
};



/** General matrix determinant.
* It uses lu_factorize in uBLAS.
*/
template<class M>
double lu_det(M const& m) {

  using namespace boost::numeric::ublas;

  assert( m.size1() == m.size2() );
  // ublasExtra::lu_det: matrix must be square

  // create a working copy of the input
  mat mLu(m);
  permutation_matrix<std::size_t> pivots(m.size1());

  lu_factorize(mLu, pivots);

  double det = 1.0;

  for (std::size_t i=0; i < pivots.size(); ++i) {
    if (pivots(i) != i)
      det *= -1.0;
    det *= mLu(i,i);
  }
  return det;
};



#endif //_LINALG_TOOLS_HPP_
