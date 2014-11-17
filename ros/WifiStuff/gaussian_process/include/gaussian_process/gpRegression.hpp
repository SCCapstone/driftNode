#ifndef _GPREGRESSION_HPP_
#define _GPREGRESSION_HPP_

#define NDEBUG


#include "cholesky.hpp"
#include "types.hpp"
#include "linAlgTools.hpp"
#include <assert.h>
#include <vector>
#include <gsl/gsl_vector.h>

#include "covarianceFunction.hpp"

// -----------------------------------------------------------------------
template <class TInput>
class GPReg
{
  
  public:
  
    GPReg( CovFunc<TInput> *covFunc, double sigmaNoise, GPReg<TInput> *noiseGP = NULL);
    GPReg( CovFunc<TInput> *covFunc, double *sigmaNoise, GPReg<TInput> *noiseGP = NULL);
   	GPReg(GPReg<TInput> *copyFromGP);
    ~GPReg();

    void setSigmaNoise( double sigma );
    void setDataPoints( TVector<TInput> &dataPoints, TVector<double> &dataTargets );
    void buildGP(bool useTargets=true);
    void buildTargets();
    void evalGP( const TInput &x, double &mean, double &var );
    void evalGP( const TInput &x, double &mean );
    double getDataLikelihood();
    double getAvgVariance();

	void getParameters(std::vector<double> *r );
	void setParameters(std::vector<double> *r );
    void getDerivative(std::vector<double> *r);

    // GPRegression( const GPRegression &orig ) : m_covFunc(orig.m_covFunc),
    // void addDataPoints( vector<Vector> &dataPoints, vector<double> &dataTargets )
    double getDataFit();
    double getComplexity();
    double getMarginalDataLikelihood();

    
    // double getObservationLikelihood( vector<Vector> &dataPoints, vector<double> &observations )

  // -----------------------------------------------------------------------
  public:
    GPReg<TInput> *m_copyFromGP;

    int m_numDataPoints;
    
    TVector<TInput>  *m_dataPoints;
    TVector<double> *m_t;                // target values of samples
    TMatrix<double> *m_C;                // cov matrix

    TMatrix<double> *m_iC;               // inverted cov matrix
    TVector<double> *m_iCt;              // tmp
    CovFunc<TInput>  *m_covFunc;
    double          *m_sigmaNoise;
    double			m_ownSigmaNoise;
    
    GPReg<TInput> *m_noiseGP; // never delete!!! It is basically just a reference

	static double gsl_my_f(const gsl_vector *v, void *params);
	static void gsl_my_df (const gsl_vector *v, void *params, gsl_vector *df);
	static void gsl_my_fdf (const gsl_vector *v, void *params, double *f, gsl_vector *df);
	bool minimizeGSL(unsigned maxIt);
};
#endif //_GPREGRESSION_HPP_
