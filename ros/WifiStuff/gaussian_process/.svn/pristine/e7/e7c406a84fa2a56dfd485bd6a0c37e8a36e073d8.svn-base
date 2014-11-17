/*
 * SingleGP.cpp
 *
 *  Created on: Feb 10, 2010
 *      Author: sturm
 */


#include "gaussian_process/SingleGP.h"

namespace gaussian_process {

#define sqr(a) ((a)*(a))

SingleGP::SingleGP(CovFuncND initialCovFunc,double initialSigmaNoise)
:
	initialCovFunc(initialCovFunc), initialSigmaNoise(initialSigmaNoise),
	covFunc(initialCovFunc), sigmaNoise(initialSigmaNoise),
	GP(&covFunc, &sigmaNoise)
{
	GP.m_dataPoints = &dataPoints;
	GP.m_t = &targetPoints;
	GP.m_numDataPoints = 0;

}

SingleGP::~SingleGP() {
	GP.m_dataPoints = NULL;
	GP.m_t = NULL;
}

void SingleGP::Reset() {
	dataPoints.clear();
	targetPoints.clear();
	covFunc = initialCovFunc;
	sigmaNoise = initialSigmaNoise;
	GP.m_numDataPoints = 0;
}

void SingleGP::SetData(TVector<TDoubleVector> &dataPoints,TVector<double> &targetPoints) {
	this->dataPoints = dataPoints;
	this->targetPoints = targetPoints;
	GP.m_numDataPoints = dataPoints.size();
	GP.m_dataPoints = &this->dataPoints;
	GP.m_t = &this->targetPoints;
	BuildGP();
}

void SingleGP::BuildGP() {
	if(GP.m_numDataPoints ==0) {
		return;
	}

	if(dataPoints[0].size() == 0) {
		double sum = 0;
		for(unsigned int i=0;i<dataPoints.size();i++) {
			sum += targetPoints[i];
		}
		mean = sum/dataPoints.size();

		sum = 0;
		for(unsigned int i=0;i<dataPoints.size();i++) {
			sum += sqr(targetPoints[i]-mean);
		}
		var = sum/dataPoints.size();
		sum = 0;
		return;
	} else {
		GP.buildGP(true);	// build iCov and targets
	}
}

void SingleGP::OptimizeGP() {
	if(GP.m_numDataPoints ==0) return;

	if(dataPoints[0].size() != 0) {

		if(!GP.minimizeGSL(100)) {
			covFunc = initialCovFunc;
			sigmaNoise = initialSigmaNoise;
//			cout << "minimize failed, resetting values"<<endl;
		} else {
			return; // done!
		}

	}
	BuildGP();
}

void SingleGP::Evaluate(TDoubleVector data, double &targetMean, double &targetVar) {
	targetMean = NAN;
	targetVar = NAN;
	if(GP.m_numDataPoints ==0) return;

	if(dataPoints[0].size() == 0) {
		targetMean = mean;
		targetVar = var;
	} else {
		//cout << "datapoitns="<<GP.m_numDataPoints<<endl;
		GP.evalGP(data, targetMean, targetVar);
	}
}

double SingleGP::GetDataLikelihood() {
	if(GP.m_numDataPoints ==0) return(NAN);

	if(dataPoints[0].size() == 0) {
		return(NAN);
	}

	return(GP.getDataLikelihood());
}

}

#include "gaussian_process/gpRegression.hxx"
