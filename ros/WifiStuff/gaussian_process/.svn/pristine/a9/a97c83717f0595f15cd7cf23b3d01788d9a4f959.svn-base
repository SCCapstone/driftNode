/*
 * SingleGP.h
 *
 *  Created on: Feb 10, 2010
 *      Author: sturm
 */

#include "covarianceFunctions.h"
#include "gpRegression.hpp"

namespace gaussian_process {

class SingleGP {
public:
	CovFuncND initialCovFunc;
	double initialSigmaNoise;

	CovFuncND covFunc;
	double sigmaNoise;

	TVector<TDoubleVector> dataPoints;
	TVector<double> targetPoints;
	GPReg<TDoubleVector> GP;
	double mean,var;

	SingleGP(CovFuncND initialCovFunc,double initialSigmaNoise);
	~SingleGP();
	void Reset();
	void SetData(TVector<TDoubleVector> &dataPoints,TVector<double> &targetPoints);
	void BuildGP();
	void OptimizeGP();
	void Evaluate(TDoubleVector data, double &targetMean, double &targetVar);
	double GetDataLikelihood();
};

}
