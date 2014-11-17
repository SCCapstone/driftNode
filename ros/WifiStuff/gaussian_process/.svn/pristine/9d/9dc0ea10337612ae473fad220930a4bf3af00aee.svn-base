#ifndef COVARIANCEFUNCTIONS_H_
#define COVARIANCEFUNCTIONS_H_

#include "covarianceFunction.hpp"

class CovFunc1D : public CovFunc<TDoubleVector>
{
  public:

    CovFunc1D();
    CovFunc1D(double ell_0, double sigma);

    virtual double getCov( const TDoubleVector &x, const TDoubleVector &y );
    virtual double getDerivative(const  TDoubleVector &p1, const  TDoubleVector &p2, int parameter);
    virtual int getNumParameter();

    void setHyperparameter(vector<double>& newHyperParam);
    void setHyperparameter(double ell0,double sigma);
    double ell_0;
    double sigma;
};

// -----------------------------------------------------------------------
class CovFunc2D : public CovFunc<TDoubleVector>
{
  public:

    CovFunc2D();
    CovFunc2D(double ell_0, double ell_1, double sigma);

    virtual double getCov( const TDoubleVector &x, const TDoubleVector &y );
    virtual double getDerivative(const  TDoubleVector &p1, const  TDoubleVector &p2, int parameter);
    virtual int getNumParameter();

    void setHyperparameter(vector<double>& newHyperParam);
    void setHyperparameter(double ell_0, double ell_1, double sigma);

    double ell_0;
    double ell_1;
    double sigma;
};

// -----------------------------------------------------------------------
class CovFuncND : public CovFunc<TDoubleVector>
{
  public:
	int n;
	vector<double> evalParam; 
    CovFuncND();
    CovFuncND(int n,double ell,double sigma);

    virtual double getCov( const TDoubleVector &x, const TDoubleVector &y );
    virtual double getDerivative(const  TDoubleVector &p1, const  TDoubleVector &p2, int parameter);
    virtual int getNumParameter();

    void setHyperparameter(vector<double>& newHyperParam);
};


#endif /*COVARIANCEFUNCTIONS_H_*/
