#include "gaussian_process/covarianceFunctions.h"

CovFunc1D::CovFunc1D() {
	hyperParam = vector<double>(2);
    setHyperparameter(-1.6,1.4);
}

CovFunc1D::CovFunc1D(double ell_0, double sigma) {
	hyperParam = vector<double>(2);
	setHyperparameter(ell_0,sigma);
}

void CovFunc1D::setHyperparameter(vector<double>& newHyperParam) {
	CovFunc<TDoubleVector>::setHyperparameter(newHyperParam);
	
	// do some pre-evaluation
    this->ell_0 = exp(newHyperParam[0]);
    this->sigma = exp(2*newHyperParam[1]);
  }
  
void CovFunc1D::setHyperparameter(double ell0,double sigma) {
	hyperParam[0] = ell0;
	hyperParam[1] = sigma;

	// do some pre-evaluation
    this->ell_0 = exp(hyperParam[0]);
    this->sigma = exp(2*hyperParam[1]);
}
 
  
double CovFunc1D::getCov( const TDoubleVector &x, const TDoubleVector &y ) {

    double dist_0 = fabs(x[0]-y[0]);
    
    double tt2 = 1./(ell_0 * ell_0) * dist_0 * dist_0;
    
    double rr2 =  sigma * exp(-0.5 * tt2);
    
    return rr2;
  }


double CovFunc1D::getDerivative(const TDoubleVector &x, const TDoubleVector &y, int parameter) {

    double r=0;
    switch (parameter) {
    case 0:
      r =  getCov(x, y) * (fabs((x[0]-y[0])/ell_0) * fabs((x[0]-y[0])/ell_0)) ;
      break;
        //covariance for sigma 
    case 1:
      r = 2*getCov(x,y);
      break;
    }
    return r;
  }
  
int CovFunc1D::getNumParameter() {
    return 2;
  }

//-------------------------------------------------------------------------------------------------

CovFunc2D::CovFunc2D() {
	hyperParam = vector<double>(3);
    setHyperparameter(-1.6,-1.6,1.4);
}

CovFunc2D::CovFunc2D(double ell_0, double ell_1, double sigma) {
	hyperParam = vector<double>(3);
	setHyperparameter(ell_0,ell_1,sigma);
}

void CovFunc2D::setHyperparameter(vector<double>& hyperParam) {
	CovFunc<TDoubleVector>::setHyperparameter(hyperParam);
	
	// do some pre-evaluation
    this->ell_0 = exp(hyperParam[0]);
    this->ell_1 = exp(hyperParam[1]);
    this->sigma = exp(2*hyperParam[2]);
  }

void CovFunc2D::setHyperparameter(double ell_0, double ell_1,double sigma) {
	hyperParam[0] = ell_0;
	hyperParam[1] = ell_1;
	hyperParam[2] = sigma;

	// do some pre-evaluation
    this->ell_0 = exp(hyperParam[0]);
    this->ell_1 = exp(hyperParam[1]);
    this->sigma = exp(2*hyperParam[2]);
}
 
double CovFunc2D::getCov( const TDoubleVector &x, const TDoubleVector &y ) {

    double dist_0 = fabs(x[0]-y[0]);
    double dist_1 = fabs(x[1]-y[1]);
    double tt2 = 1./(ell_0 * ell_0) * dist_0 * dist_0 + 1./(ell_1 * ell_1) * dist_1 * dist_1;
    double rr2 =  sigma * exp(-0.5 * tt2);
    
    return rr2;
  }

double CovFunc2D::getDerivative(const TDoubleVector &x, const TDoubleVector &y, int parameter) {

    double r=0;
    switch (parameter) {
    case 0:
      //covariance times squared difference
      r =  getCov(x, y) * (fabs((x[0]-y[0])/ell_0) * fabs((x[0]-y[0])/ell_0)) ;
      break;
        
    case 1:
      //covariance times squared difference
      r =  getCov(x, y) * (fabs((x[1]-y[1])/ell_1) * fabs((x[1]-y[1])/ell_1)) ;
      break;
        //covariance for sigma 
    case 2:
      r = 2*getCov(x,y);
      break;
    }
    return r;
  }
  
int CovFunc2D::getNumParameter() {
    return 3;
  }


//-------------------------------------------------------------------------------------------------
CovFuncND::CovFuncND() {
	hyperParam = vector<double>(2);
	evalParam = vector<double>(2);
	n = 1;
	hyperParam[0] = 5;
	hyperParam[1] = 0.8;
	
	setHyperparameter(hyperParam);
}

CovFuncND::CovFuncND(int n,double ell, double sigma) {
	hyperParam = vector<double>(n+1);
	evalParam = vector<double>(n+1);
	for(int i=0;i<n;i++) 
		hyperParam[i] = ell;
	hyperParam[n] = sigma;

	setHyperparameter(hyperParam);
}

void CovFuncND::setHyperparameter(vector<double>& hyperParam) {
	CovFunc<TDoubleVector>::setHyperparameter(hyperParam);
	n = hyperParam.size()-1;
	
	// do some pre-evaluation
	evalParam = vector<double>(n+1);
	for(int i=0;i<n;i++) 
		evalParam[i] = exp(hyperParam[i]);
	evalParam[n] = exp(2*hyperParam[n]); 
  }

double CovFuncND::getCov( const TDoubleVector &x, const TDoubleVector &y ) {
	double tt2=0;
	double dist;
	for(int i=0;i<n;i++) {
		dist = fabs(x[i]-y[i]);
		tt2 += 1./(evalParam[i] * evalParam[i]) * dist*dist; 
	}
    double rr2 =  evalParam[n] * exp(-0.5 * tt2);
    
    return rr2;
  }

double CovFuncND::getDerivative(const TDoubleVector &x, const TDoubleVector &y, int parameter) {
    if(parameter<n) {
      return  getCov(x, y) 
      	* fabs((x[parameter]-y[parameter])/evalParam[parameter]) 
      	* fabs((x[parameter]-y[parameter])/evalParam[parameter]);
    } else {
    	return 2*getCov(x,y);
    }
  }
  
int CovFuncND::getNumParameter() {
    return n+1;
  }

