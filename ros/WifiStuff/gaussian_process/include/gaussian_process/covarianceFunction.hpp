#ifndef _COVARIANCEFUNCTION_HPP_
#define _COVARIANCEFUNCTION_HPP_

#include "types.hpp"
#include <assert.h>
#include <vector>

using namespace std;

// -----------------------------------------------------------------------
template <class TInput>
class CovFunc
{
  public:
    vector<double> hyperParam;
    virtual double getCov( const TInput &p1, const TInput &p2 ) = 0;
    virtual ~CovFunc() {};

    virtual double getDerivative(const TInput &p1, const TInput &p2, int parameter) = 0;
    virtual void setHyperparameter(vector<double>& newHyperParam) { hyperParam = newHyperParam; }
    
    virtual vector<double>& getHyperparameter() { return(hyperParam); }
    virtual int getNumParameter() = 0;

};


#endif //_COVARIANCEFUNCTION_HPP_
