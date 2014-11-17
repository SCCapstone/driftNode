#ifndef _TYPES_HPP_
#define _TYPES_HPP_

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>

#define TVector boost::numeric::ublas::vector
#define TDoubleVector TVector<double>
#define TMatrix boost::numeric::ublas::matrix
#define TDoubleMatrix TMatrix<double>
#define TDoubleMatrixColumn boost::numeric::ublas::matrix_column< TDoubleMatrix >
#define TDoubleMatrixRow boost::numeric::ublas::matrix_row< TDoubleMatrix >

#ifndef MAXFLOAT
#define MAXFLOAT 3.40282347e+38F
#endif



#endif //_TYPES_HPP_
