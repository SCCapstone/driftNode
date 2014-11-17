#ifndef _GP_HET_REGRESSION_HPP_
#define _GP_HET_REGRESSION_HPP_

#include "gpReg/types.hpp"
#include "gpReg/global.h"
#include "gpRegression.hpp"

#include <assert.h>
#include "covarianceFunction.hpp"
#ifdef DO_PROFILING
  #include "profiler.hpp"
#endif



// -----------------------------------------------------------------------
class CovNonStatSE : public CovFunc<TDoubleVector>
{
  public:

    CovNonStatSE( GPReg<TDoubleVector> &ellGP, double ellMean, double sf2 ) : 
        m_ellGP(ellGP), m_ellMean(ellMean), m_sf2(sf2)
    {
      m_ell = 0.13;
    }
    //Cov1dSEPolar( ConfigFile &cfg, const char *cfgSection );

    virtual double getCov( const TDoubleVector &x, const TDoubleVector &y )
    {
      double dist = fabs(x[0]-y[0]);
    	  ///*
      double covBlock = 0.0;
      double a, b;
      if (x[0]<y[0]) {
      	a = x[0];
      	b = y[0];
      } else {
      	a = y[0];
      	b = x[0];
      }
      double temp;
      TDoubleVector z(1);
      for (z[0]=a; z[0]<=b; z[0]+=0.01) {
      	m_ellGP.evalGP( z, temp );
      	temp += m_ellMean;
      	covBlock += 1.0 / (1.0+exp(-temp));
      }
      double f = (1.0-covBlock/0.01);
      if (f<0) f=0.0;
      //*/
      
      /*
      double covBlock = 1.0;
      double f = 1.0;
      double blockPoint = 0.5;
      if ( (x[0]<blockPoint && y[0]>blockPoint) || (x[0]>blockPoint && y[0]<blockPoint) ) {
        f=0.0;
      }
      */

      double cov = (m_sf2*m_sf2) * f * exp( -0.5 * pow(dist,2.0)/m_ell ); 

      //fprintf( stderr, "[%f %f] (%f %f) %f\n", x[0], y[0], ell_x, ell_y, cov );
      FILE *covFile = fopen( "cov.dat", "a" );
      fprintf( covFile, "%f %f %f %f %f\n", x[0], y[0], cov, covBlock, f );
      fclose( covFile );
      return cov; 
      //return (m_sf2*m_sf2) * f1 * f2 * f3 * exp( -0.5 * dist*dist/(ell_x-ell_y) ); 
    }

    // paciorek 1d
    /*
    virtual double getCov( const TDoubleVector &x, const TDoubleVector &y )
    {
      double ell_x, ell_y;
      m_ellGP.evalGP( x, ell_x );
      m_ellGP.evalGP( y, ell_y );
      double dist = x[0]-y[0];
      double f1 = pow( ell_x, 0.25 );
      double f2 = pow( ell_y, 0.25 );
      double f3 = pow( (ell_x+ell_y)/2.0, -0.5 );

      double cov = (m_sf2*m_sf2) * f1 * f2 * f3 * exp( -0.5 * pow(dist,2.0)/((ell_x+ell_y)/2.0) ); 
      //double cov = (m_sf2*m_sf2) * exp( -0.5 * pow(dist,2.0)/((ell_x+ell_y)/2.0) ); 

      //fprintf( stderr, "[%f %f] (%f %f) %f\n", x[0], y[0], ell_x, ell_y, cov );
      FILE *covFile = fopen( "cov.dat", "a" );
      fprintf( covFile, "%f %f %f %f %f %f\n", x[0], y[0], ell_x, ell_y, f1*f2*f3, cov );
      fclose( covFile );
      return cov;
      //return (m_sf2*m_sf2) * f1 * f2 * f3 * exp( -0.5 * dist*dist/(ell_x-ell_y) ); 
    }
    */



  private:
    ConfigFile m_cfg;
  
  public:
    GPReg<TDoubleVector> &m_ellGP;
    double m_ellMean;
    double m_ell;
    double m_sf2;
};



// -----------------------------------------------------------------------
template <class TInput>
class GPNonStatReg
{
  
  public:
  

 
    // -----------------------------------------------------------------------
    GPNonStatReg(
        double ell, double ellMean, double sf2, double noise ) :
      m_numDataPoints(0), m_dataPoints(0), m_targets(0),
      m_ellPriorCovFunc(ell,0.0,sf2), m_ellPriorGP(m_ellPriorCovFunc,noise),
      m_covFunc(m_ellPriorGP,ellMean,sf2), m_dataGP(m_covFunc,noise)
    {}



    // -----------------------------------------------------------------------
    ~GPNonStatReg()
    {}



    // -----------------------------------------------------------------------
    void setDataPoints( TVector<TInput> &dataPoints, TVector<double> &targets, TVector<double> &ellTargets )
    {
      m_dataPoints = dataPoints;
      m_numDataPoints = dataPoints.size();
      m_targets = targets;
      m_ellTargets = ellTargets;
    }      



    // -----------------------------------------------------------------------
    void buildGP()
    {
      // dummy ellPrior (all targets = ell)
      m_ellPriorGP.setDataPoints( m_dataPoints, m_ellTargets );
      m_ellPriorGP.buildGP();
      
      m_dataGP.setDataPoints( m_dataPoints, m_targets );
      m_dataGP.buildGP();
    }



    // -----------------------------------------------------------------------
    void evalGP( const TInput &x, double &mean, double &var )
    {
      m_dataGP.evalGP( x, mean, var );
    }



    // -----------------------------------------------------------------------
    void evalGP( const TInput &x, double &mean )
    {
      m_dataGP.evalGP( x, mean );
    }



  // -----------------------------------------------------------------------
  public:
    int m_numDataPoints;
    TVector<TInput> m_dataPoints;
    TVector<double> m_targets;

    Cov1dSE         m_ellPriorCovFunc;
    GPReg<TInput>   m_ellPriorGP;
    CovNonStatSE    m_covFunc;
    GPReg<TInput>   m_dataGP;

    TVector<double> m_ellTargets;
    double          m_noise;

};



#endif //_GP_HET_REGRESSION_HPP_
