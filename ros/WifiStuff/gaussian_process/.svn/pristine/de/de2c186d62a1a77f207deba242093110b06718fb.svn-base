#ifndef _GP_HET_REGRESSION_HPP_
#define _GP_HET_REGRESSION_HPP_

#include <gpReg/types.hpp>

#ifdef SPARSE_REGRESSION
  #include "gpSparseRegression.hpp"
#else
  #include "gpRegression.hpp"
#endif

#include <assert.h>
#include "covarianceFunction.hpp"
#ifdef DO_PROFILING
  #include "profiler.hpp"
#endif



/*
 * m_primGP: GP to predict mean of data process
 * m_secGP:  GP to predict input-dependent noise variances (in log space)
 * m_combGP: GP for data process (using log noise variances predicted by m_secGP)
 * 
 * defines:
 * 
 * WRITE_DEBUG       write debug data files
 * SPARSE_REGRESSION use efficient, sparse matrix libraries (currently disabled!) 
 * DO_PROFILING      enable the benchmarking code (global functions for profiling the algorithms)
 */

// -----------------------------------------------------------------------
template <class TInput>
class GPHetReg
{
  
  public:
  

 
    // -----------------------------------------------------------------------
    GPHetReg(
        CovFunc<TInput> &covPrimGP,
        CovFunc<TInput> &covSecGP,
        CovFunc<TInput> &covCombGP,
        double noisePrimGP,
        double noiseSecGP,
        double noiseCombGP ) :
      m_numDataPoints(0), m_dataPointsPrimGP(0), m_dataPointsSecGP(0), m_useEmpiricalVariances(false),
      m_primTargets(0), m_secTargets(0), m_covPrimGP(covPrimGP), m_covSecGP(covSecGP), m_covCombGP(covCombGP),
      m_noisePrimGP(noisePrimGP),m_noiseSecGP(noiseSecGP), m_noiseCombGP(noiseCombGP),
      m_primGP(m_covPrimGP,m_noisePrimGP), m_secGP(m_covSecGP,m_noiseSecGP),
      m_combGP(m_covCombGP,m_noiseCombGP, &m_secGP) 
    {}



    // -----------------------------------------------------------------------
    ~GPHetReg()
    {}



    // -----------------------------------------------------------------------
    void setDataPoints( TVector<TInput> &dataPoints, TVector<double> &dataTargets )
    {
      m_dataPointsPrimGP = dataPoints;
      m_numDataPoints = dataPoints.size();
      m_primTargets = dataTargets;
    }      



    // -----------------------------------------------------------------------
    void setEmpiricalVariances( TVector<TInput> &varPoints, TVector<double> &variances )
    {
      m_dataPointsSecGP = varPoints;
      m_secTargets = variances;

      #ifdef WRITE_DEBUG
        FILE *trainDatFile = fopen( "trainSecGP.dat", "w" );
      #endif

      for (unsigned int i=0; i<m_secTargets.size(); i++) {
      	m_secTargets[i] = log( m_secTargets[i] );
      	if (m_secTargets[i] < -5.0) {
      	  m_secTargets[i] = -5.0;
      	}

        #ifdef WRITE_DEBUG
          fprintf( trainDatFile, "%i %f %f\n", i, m_dataPointsSecGP[i][0], m_secTargets[i] );
        #endif

      }
      #ifdef WRITE_DEBUG
        fclose( trainDatFile );
      #endif

      m_useEmpiricalVariances = true;
    }  



    // -----------------------------------------------------------------------
    void buildGP()
    {
      //-------------------------------
      // build primGP
      //-------------------------------

      m_primGP.setDataPoints( m_dataPointsPrimGP, m_primTargets );
      m_primGP.buildGP();

      //-------------------------------
      // build secGP
      //-------------------------------

      if (!m_useEmpiricalVariances) {
        double mean;
        double target;

        #ifdef WRITE_DEBUG
          FILE *trainDatFile = fopen( "trainSecGP.dat", "w" );
        #endif

        m_dataPointsSecGP.resize( m_dataPointsPrimGP.size() );
        m_secTargets.resize( m_dataPointsPrimGP.size() );
        for (unsigned int i=0; i<m_primTargets.size(); i++) {
          m_dataPointsSecGP[i] = m_dataPointsPrimGP[i];
    
          m_primGP.evalGP( m_dataPointsSecGP[i], mean );
          target = log( pow( mean-m_primTargets[i], 2.0 ) / 0.33 );
          //target = log( pow( mean-m_primTargets[i], 2.0 ) / 1.0 );
        
          #ifdef WRITE_DEBUG
            fprintf( trainDatFile, "%i %f %f\n", i, m_dataPointsSecGP[i][0], target );
          #endif
          m_secTargets[i] = target;
        }
        #ifdef WRITE_DEBUG
          fclose( trainDatFile );
        #endif
      }
      m_secGP.setDataPoints( m_dataPointsSecGP, m_secTargets );
      m_secGP.buildGP();

      //-------------------------------
      // build combGP
      //-------------------------------

      m_combGP.setDataPoints( m_dataPointsPrimGP, m_primTargets );
      m_combGP.buildGP();
    }



    // -----------------------------------------------------------------------
    void evalGP( const TInput &x, double &mean, double &var )
    {
      m_combGP.evalGP( x, mean, var );
    }



    // -----------------------------------------------------------------------
    void evalGP( const TInput &x, double &mean )
    {
      m_combGP.evalGP( x, mean );
    }



    // -----------------------------------------------------------------------
    /*
    double getObservationLikelihood( TVector<Vector> &dataPoints, TVector<double> &observations )
    {
      return m_combGP.getObservationLikelihood( dataPoints, observations );
    }
    */



  // -----------------------------------------------------------------------
  public:
    int m_numDataPoints;
    TVector<TInput> m_dataPointsPrimGP;
    TVector<TInput> m_dataPointsSecGP;
    bool            m_useEmpiricalVariances;
    TVector<double> m_primTargets;
    TVector<double> m_secTargets;

    CovFunc<TInput> &m_covPrimGP;
    CovFunc<TInput> &m_covSecGP;
    CovFunc<TInput> &m_covCombGP;

    double m_noisePrimGP;
    double m_noiseSecGP;
    double m_noiseCombGP;

#ifdef SPARSE_REGRESSION
  GPSparseRegression m_primGP;
  GPSparseRegression m_secGP;
  GPSparseRegression m_combGP;
#else
  GPReg<TInput> m_primGP;
  GPReg<TInput> m_secGP;
  GPReg<TInput> m_combGP;
#endif
};



#endif //_GP_HET_REGRESSION_HPP_
