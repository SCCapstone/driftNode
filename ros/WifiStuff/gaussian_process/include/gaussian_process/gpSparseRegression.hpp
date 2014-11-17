#ifndef _GP_SPARSE_UMFPACK_REGRESSION_HPP_
#define _GP_SPARSE_UMFPACK_REGRESSION_HPP_

#include <vector>

#include <assert.h>

#include "gpReg/covarianceFunction.hpp"

//#include "umfpack.hpp"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "gpReg/Matrix.hh"

// strange UBLAS compile errors concerning "same_impl_ex" (sth. with int vs. size_t)
// source: ttp://archives.free.net.ph/message/20061016.230807.43f957a0.en.html
// solutions: added -DNDEBUG to the compile options
//template<class T>
//BOOST_UBLAS_INLINE size_t same_impl_ex (const size_t &size1, const int &size2, const char *file, int line) {
//  BOOST_UBLAS_CHECK_EX (size1 == size2, file, line, bad_argument ());
  // WARNING: this fails if size2 is negative!
//  return (size1<size2?(size_t)size1:(size_t)size2);
//}

#define TVector boost::numeric::ublas::vector<double>
#define TMatrix boost::numeric::ublas::compressed_matrix<double>


using namespace std;

#define PROFILING

#ifdef PROFILING
#include "gpReg/profiler.hpp"
#endif


// -----------------------------------------------------------------------
class GPSparseRegression
{
  
  public:
  

  
    // -----------------------------------------------------------------------
    GPSparseRegression( CovarianceFunction &covFunc, double sigmaNoise,
        GPSparseRegression *noiseGP = NULL ) :
        m_dataPoints(0), m_covFunc(covFunc), m_sigmaNoise(sigmaNoise)
    {
      s_iC  = NULL;
      s_t   = NULL;
      s_k   = NULL;
      s_iCt = NULL;
      s_Cs  = NULL;
      m_numDataPoints = 0;
      m_noiseGP = noiseGP;
    }
    

 
  // -----------------------------------------------------------------------
  // copy constructor
  GPSparseRegression( const GPSparseRegression &orig ) : m_covFunc(orig.m_covFunc),
      m_numDataPoints(orig.m_numDataPoints), m_dataPoints(orig.m_dataPoints),
      s_t(orig.s_t), m_sigmaNoise(orig.m_sigmaNoise)
  {
	fprintf( stderr, "ERROR (no copy const)\n" );
  	assert( false ); // not implemented
  }
  
  // -----------------------------------------------------------------------
  // copy constructor
  /*
  GPSparseRegression( const GPSparseRegression &orig ) : m_covFunc(orig.m_covFunc),
        m_numDataPoints(orig.m_numDataPoints), m_dataPoints(orig.m_dataPoints),
        s_t(orig.s_t), m_sigmaNoise(orig.m_sigmaNoise) {
      s_iC = NULL;
      if (orig.s_iC) {
        s_iC = new umfpack::umfpack_matrix_operator<double>( *(orig.s_iC) );
      }
      s_t = NULL;
      if (orig.s_t) {
        s_t = new TVector( *(orig.s_t) );
      }
      s_k = NULL;
      if (orig.s_k) {
        s_k = new TVector( *(orig.s_k) );
      }
      s_iCt = NULL;
      if (orig.s_iCt) {
        s_iCt = new TVector( *(orig.s_iCt) );
      }
      s_Cs = NULL;
      if (orig.s_Cs) {
        s_Cs = new TMatrix( *(orig.s_Cs) );
      }
      s_noiseGP = NULL;
      if (orig.s_noiseGP) {
        s_noiseGP = new GPRegression( *(orig.s_noiseGP) );
      }
    }
   */
    


    // -----------------------------------------------------------------------
    ~GPSparseRegression()
    {
      if (s_iC) delete (s_iC);
      if (s_t) delete (s_t);
      if (s_k) delete (s_k);
      if (s_iCt) delete (s_iCt);
      if (s_Cs) delete (s_Cs);
    }



    // -----------------------------------------------------------------------
    void setSigmaNoise( double sigma ) { m_sigmaNoise = sigma; }



    // -----------------------------------------------------------------------
    void setDataPoints( vector<Vector> &dataPoints, vector<double> &dataTargets )
    {
      // copy
      m_dataPoints = dataPoints;
      m_numDataPoints = dataPoints.size();
      if (s_t) delete (s_t);
      s_t = new (TVector)(m_numDataPoints);
      for (int i=0; i<m_numDataPoints; i++) {
        (*s_t)(i) = dataTargets[i];
      }
    }      



    // -----------------------------------------------------------------------
    void buildGP()
    {
      assert( m_numDataPoints>0 );
      using namespace boost::numeric::ublas;
      using namespace umfpack;

      // coordinate_matrix: incredibly slow
      // matrix and compressed_matrix: fast and similar
      // default: row_major -> always have first matrix dimension in outer loop!!!
      //PROFILER_start();
      TMatrix s_C( m_numDataPoints, m_numDataPoints );
      double cov;
      double logNoise;
      for (int i1=0; i1<m_numDataPoints; i1++) {
        for (int i2=0; i2<m_numDataPoints; i2++) {
        	  cov = m_covFunc.getCov( m_dataPoints[i1], m_dataPoints[i2] );
          if (i1==i2) {
            cov += exp(2.0 * m_sigmaNoise);
            if (m_noiseGP) {
              m_noiseGP->evalGP(m_dataPoints[i1],logNoise);
              cov += exp(logNoise);
            }
          }
          s_C(i1,i2) = cov;
        }
      }
      //fprintf( stdout, "  build s_C: %f\n", PROFILER_getElapsed() );

      if (s_k) delete (s_k);
      s_k = new TVector(m_numDataPoints);
      if (s_Cs) delete (s_Cs);
      s_Cs = new umfpack_matrix_operator<double>::matrix_type(s_C);

      //PROFILER_start();
      if (s_iC) delete (s_iC);
      s_iC = new (umfpack_matrix_operator<double>)( *s_Cs );
      //fprintf( stdout, "build s_iC: %f\n", PROFILER_getElapsed() );

      if (s_iCt) delete (s_iCt);
      s_iCt = new umfpack_matrix_operator<double>::vector_type(m_numDataPoints);

      //PROFILER_start();
      s_iC->apply( *s_t, *s_iCt );
      //fprintf( stdout, "build s_iCt: %f\n", PROFILER_getElapsed() );

      //if (s_iCk) delete (s_iCk);
      //s_iCk = new (umfpack_matrix_operator<double>::vector_type)(m_numDataPoints);
    }



    // -----------------------------------------------------------------------
    void evalGP( Vector &x, double &mean, double &var )
    {
      using namespace boost::numeric::ublas;
      using namespace umfpack;

      for (int i1=0; i1<m_numDataPoints; i1++) {
        (*s_k)(i1) =  m_covFunc.getCov( x, m_dataPoints[i1] );
      }

      TVector s_iCk(m_numDataPoints);
      s_iC->apply( *s_k, s_iCk );

      mean = inner_prod( *s_k, *s_iCt );
      var = m_covFunc.getCov(x,x) - (inner_prod( *s_k, s_iCk ) );

      if (m_noiseGP) {
        double logNoise;
        m_noiseGP->evalGP( x, logNoise );
        var += exp(logNoise);
      }
    }



    // -----------------------------------------------------------------------
    void evalGP( Vector &x, double &mean )
    {
      using namespace boost::numeric::ublas;
      using namespace umfpack;

      for (int i1=0; i1<m_numDataPoints; i1++) {
        (*s_k)(i1) = m_covFunc.getCov( x, m_dataPoints[i1] );
      }

      mean = inner_prod( *s_k, *s_iCt );
    }



    // -----------------------------------------------------------------------
    void save()
    {}
    // -----------------------------------------------------------------------
    void load()
    {}



    // -----------------------------------------------------------------------
    double getObservationLikelihood( vector<Vector> &dataPoints, vector<double> &observations )
    {
      int numObs = dataPoints.size();
      using namespace boost::numeric::ublas;

      //TMatrix sK__(numObs,numObs);
      // compressed matrix: default: row_major
      //PROFILER_start();
      TMatrix sK__(numObs,numObs);
      double cov;
      double logNoise;
      for (int i=0; i<numObs; i++) {
        for (int j=0; j<numObs; j++) {
        	  cov = m_covFunc.getCov( dataPoints[i], dataPoints[j] );
          if (i==j) {
            cov += exp(2.0 * m_sigmaNoise);
            if (m_noiseGP) {
              m_noiseGP->evalGP( dataPoints[i], logNoise );
              cov += exp(logNoise);
            }
          }
          sK__(i,j) = cov; 
        }
      }
      //fprintf( stderr, "  spReg.getLh(1): %f\n", PROFILER_getElapsed() );

      //PROFILER_start();
      TMatrix sK_(numObs,m_numDataPoints);
      for (int i=0; i<numObs; i++) {
        for (int j=0; j<m_numDataPoints; j++) {
          sK_(i,j) = m_covFunc.getCov( dataPoints[i], m_dataPoints[j] );
        }
      }

      TVector y_(numObs);
      double mean;
      for (int i=0; i<numObs; i++) {
        evalGP( dataPoints[i], mean );
        y_(i) = (observations[i]-mean);
      }

      double y__ = inner_prod( y_, TVector(prod(sK__,y_)) );
      //fprintf( stderr, "  ** y_ %f\n", y__ );

      TVector ytB = prod( trans(sK_), y_ );
      
      TVector iCytB(m_numDataPoints);
      s_iC->apply( ytB, iCytB );

      double dotProd = inner_prod( ytB, iCytB );
      //fprintf( stderr, "  ** dp %f\n", dotProd );
      
      double dataFit = -0.5 * ( y__ - dotProd );

      //fprintf( stderr, "  ** log df %f\n", dataFit );

      //fprintf( stderr, "  spReg.getLh(2): %f\n", PROFILER_getElapsed() );
      return dataFit;
      //double complPenalty = - 0.5 * log( K.Det() );
      //double norm = - 0.5 * numObs*log(2.0*M_PI);
      //double logLh = dataFit + complPenalty + norm;
      //fprintf( stderr, "# dataFit: %f\t complPenalty: %f\t norm: %f\t logLh: %f\n", dataFit, complPenalty, norm, logLh );
      //FILE *lhFile = fopen( "logLH.dat", "a" );
      //fprintf( lhFile, "%f %f %f %f\n", dataFit, complPenalty, norm, logLh );
      //fclose(lhFile);
      //return logLh;
    }


    // -----------------------------------------------------------------------
  public:
    int m_numDataPoints;
    vector<Vector> m_dataPoints;
    vector<double> m_dataVariances;
    CovarianceFunction &m_covFunc;
    double              m_sigmaNoise;
    GPSparseRegression *m_noiseGP;
  
  //private:
    umfpack::umfpack_matrix_operator<double> *s_iC;
    TVector *s_t;
    TVector *s_k;
    TVector *s_iCt;
    umfpack::umfpack_matrix_operator<double>::matrix_type *s_Cs;

};



#endif //_GP_SPARSE_UMFPACK_REGRESSION_HPP_
