
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_blas.h>

#define GSL_VERBOSE false
#define MIN_EXP -6
#define MAX_EXP +6

#ifndef MIN 
#define MIN(a,b) ((a>b)?b:a)
#endif

#ifndef MAX 
#define MAX(a,b) ((a>b)?a:b)
#endif

#define BOUND(a) (MIN(MAX_EXP,MAX(MIN_EXP,a)))


// -----------------------------------------------------------------------
template <class TInput>
GPReg<TInput>::GPReg( CovFunc<TInput> *covFunc, double sigmaNoise,
    GPReg<TInput> *noiseGP ) :
    m_covFunc(covFunc), m_sigmaNoise(&m_ownSigmaNoise)
{
	//printf("gp-create\n");
  m_t = NULL;
  m_C = NULL;
  m_iC = NULL;
  m_iCt = NULL;
  m_dataPoints = NULL;
  m_numDataPoints = 0;
  m_noiseGP = noiseGP;
  m_copyFromGP = NULL;
  (*m_sigmaNoise) = sigmaNoise;
}

template <class TInput>
GPReg<TInput>::GPReg( CovFunc<TInput> *covFunc, double *sigmaNoise,
    GPReg<TInput> *noiseGP ) :
    m_covFunc(covFunc), m_sigmaNoise(&m_ownSigmaNoise)
{
	//printf("gp-create\n");
  m_t = NULL;
  m_C = NULL;
  m_iC = NULL;
  m_iCt = NULL;
  m_dataPoints = NULL;
  m_numDataPoints = 0;
  m_noiseGP = noiseGP;
  m_copyFromGP = NULL;
  m_sigmaNoise = sigmaNoise;
}

template <class TInput>
GPReg<TInput>::GPReg(GPReg<TInput> *copyFromGP) :
m_copyFromGP(copyFromGP),

m_numDataPoints(copyFromGP->m_numDataPoints),

m_dataPoints(copyFromGP->m_dataPoints),
m_t(copyFromGP->m_t),
m_C(copyFromGP->m_C),

m_iC(copyFromGP->m_iC),
m_iCt(copyFromGP->m_iCt),

m_covFunc(copyFromGP->m_covFunc), 
m_sigmaNoise(copyFromGP->m_sigmaNoise),
m_noiseGP(copyFromGP->m_noiseGP)
{
}


// -----------------------------------------------------------------------
/*

template <class TInput>
GPRegression<TInput>::
GPRegression( const GPRegression &orig ) : m_covFunc(orig.m_covFunc),
    m_numDataPoints(orig.m_numDataPoints), m_dataPoints(orig.m_dataPoints),
    m_t(orig.m_t), (*m_sigmaNoise)(orig.(*m_sigmaNoise))
{
  m_C = NULL;
  if (orig.m_C) {
    m_C = new Matrix( *(orig.m_C) );
  }
  m_iC = NULL;
  if (orig.m_iC) {
    m_iC = new Matrix( *(orig.m_iC) );
  }
  m_iCt = NULL;
  if (orig.m_iCt) {
    m_iCt = new Vector( *(orig.m_iCt) );
  }
  m_k = NULL;
  if (orig.m_k) {
    m_k = new Vector( *(orig.m_k) );
  }
  if (m_noiseGP) {
  	assert( false ); // sparse GPs have no copy constructor yet
  }
  m_noiseGP = orig.m_noiseGP;
}

*/


// -----------------------------------------------------------------------
template <class TInput>
GPReg<TInput>::~GPReg()
{
  if(!m_copyFromGP) {
	//printf("gp-destroy&delete\n");
	if (m_dataPoints) delete (m_dataPoints);
  	if (m_t) delete (m_t);
  	if (m_C) delete (m_C);
  	if (m_iC) delete (m_iC);
  	if (m_iCt) delete (m_iCt);
  } else {
  }
  // never delete m_noiseGP !!! It is basically just a reference
}



// -----------------------------------------------------------------------
template <class TInput>
void GPReg<TInput>::setSigmaNoise( double sigma )
{
  (*m_sigmaNoise) = sigma;
}



// -----------------------------------------------------------------------
template <class TInput>
void GPReg<TInput>::setDataPoints( TVector<TInput> &dataPoints, TVector<double> &dataTargets )
{
  m_dataPoints = new TVector<TInput>();
  (*m_dataPoints) = dataPoints;
  
  m_numDataPoints = m_dataPoints->size();
  
  m_t = new TVector<double>( dataTargets.size() );
  for (unsigned int i=0; i<dataTargets.size(); i++) {
    (*m_t)[i] = dataTargets[i];
  }
}



// -----------------------------------------------------------------------
/*
template <class TInput>
GPRegression<TInput>::addDataPoints( vector<Vector> &dataPoints, vector<double> &dataTargets )
{
  // copy
  //fprintf( stderr, "adding %i %i\n", dataPoints.size(), dataTargets.size() );
  m_numDataPoints += dataPoints.size();
  Vector m_t_new( m_numDataPoints );
  for (unsigned int i=0; i<m_t.Size(); i++) {
    m_t_new[i] = m_t[i];
  }
  for (unsigned int i=0; i<dataTargets.size(); i++) {
    m_dataPoints.push_back( dataPoints[i] );
    m_t_new[m_t.Size()+i] = dataTargets[i];
  }
  m_t = m_t_new;
}
*/

// -----------------------------------------------------------------------
template <class TInput>
void GPReg<TInput>::setParameters(std::vector<double> *r ) {
	std::vector<double> cp;
	cp.push_back((*r)[0]);
	cp.push_back((*r)[1]);
	m_covFunc.setHyperparameter(cp);
	(*m_sigmaNoise) = (*r)[2]; 
}

template <class TInput>
void GPReg<TInput>::getParameters(std::vector<double> *r ) {
	(*r) = m_covFunc->hyperParam;
	(*r).push_back(*m_sigmaNoise);
}

template <class TInput>
void GPReg<TInput>::getDerivative(std::vector<double> *r ) {
  // alpha = m_iCt
  TMatrix<double> alphaOuterProd = outer_prod( *m_iCt, *m_iCt );
  TMatrix<double> W = alphaOuterProd - (*m_iC);

  // for all paramter of the covariance function
  for (int i=0; i<m_covFunc->getNumParameter(); i++) {

    // Covariance derivatives
    //vorher:     TMatrix<double> dC = new TMatrix<double>( m_numDataPoints, m_numDataPoints );
    TMatrix<double> dC( m_numDataPoints, m_numDataPoints );
    for (int i1=0; i1<m_numDataPoints; i1++) {
      for (int i2=0; i2<m_numDataPoints; i2++) { 
        dC(i1,i2) = m_covFunc->getDerivative( (*m_dataPoints)[i1], (*m_dataPoints)[i2], i);
      }
    }

    TMatrix<double> res = prod(W,dC);

    double trace = 0.0;
    for (int i=0; i<m_numDataPoints; i++) {
      trace += res(i,i);
    }
    r->push_back( 0.5 * trace );
  }

  // derivative sigmaNoise
  double trace = 0.0;
  double n = 2 * exp(2.0 * (*m_sigmaNoise));
  for (int i=0; i<m_numDataPoints; i++) {
    trace += W(i,i) * n;
  }
  r->push_back( 0.5 * trace );

}



// -----------------------------------------------------------------------
template <class TInput>
double GPReg<TInput>::getDataLikelihood()
{
  double dataFit = -0.5 * inner_prod( *m_t, *m_iCt );

  double log_determinant=0;
  boost::numeric::ublas::triangular_matrix<double> L (m_numDataPoints,m_numDataPoints);  

  cholesky_decompose(*m_C, L);
  log_determinant= 2*log((L)(0,0));
  for (int i1=1; i1<m_numDataPoints; i1++) {
    log_determinant+= 2*log((L)(i1,i1));
  }

  double complPenalty = -0.5*log_determinant;
  double norm = - 0.5 * m_numDataPoints * log( 2.0*M_PI );

  double logLh = dataFit + complPenalty + norm;

  return logLh;
}



// -----------------------------------------------------------------------
template <class TInput>
void GPReg<TInput>::buildGP(bool useTargets)
{
#ifdef PROFILE
  PROFILER_start();
#endif
  assert( m_numDataPoints>0 );
  assert (m_copyFromGP == NULL);
  if (m_C) delete (m_C);
    m_C = new TMatrix<double>( m_numDataPoints, m_numDataPoints );
    for (int i1=0; i1<m_numDataPoints; i1++) {
       for (int i2=0; i2<m_numDataPoints; i2++) {
         (*m_C)(i1,i2) = m_covFunc->getCov( (*m_dataPoints)[i1], (*m_dataPoints)[i2] );
	 
	 	 //cout << (*m_C)(i1,i2) << " ";

         if (i1==i2) {
           (*m_C)(i1,i2) += exp(2.0 * (*m_sigmaNoise));
           if (m_noiseGP) {
             double logNoise;
             m_noiseGP->evalGP((*m_dataPoints)[i1],logNoise);
             (*m_C)(i1,i2) += exp(logNoise);
           }
         }
       }
             // cout << endl;
       //fprintf( stderr, "b" );
    }
    //cout << endl;
  //fprintf( stderr, "[inverting]" );
  if (m_iC) delete (m_iC);
  m_iC = new TMatrix<double>( m_C->size1(), m_C->size1() );
  lu_inv( *m_C, *m_iC );
 

  if(useTargets) buildTargets();

  //fprintf( stderr, "\n" );
#ifdef PROFILE
  fprintf( stdout, " dtime: %f\n", PROFILER_getElapsed() );
#endif
}

template <class TInput>
void GPReg<TInput>::buildTargets() {
  //PROFILER_start();
  if (m_iCt) delete (m_iCt);
  m_iCt = new TVector<double>( prod( *m_iC, *m_t ) );
  //fprintf( stdout, "build s_iCt: %f\n", PROFILER_getElapsed() );
}

// -----------------------------------------------------------------------
template <class TInput>
void GPReg<TInput>::evalGP( const TInput &x, double &mean, double &var )
{
  assert( m_numDataPoints>0 ); // first load or build new gp
  TVector<double> m_k(m_numDataPoints);
  for (int i=0; i<m_numDataPoints; i++) {
    m_k[i] = m_covFunc->getCov( x, (*m_dataPoints)[i] );
  }

  mean = inner_prod( m_k, *m_iCt );
  var = m_covFunc->getCov(x,x) - inner_prod( m_k, prod( *m_iC, m_k ) );

  var += exp(2.0 * (*m_sigmaNoise) );

  if (m_noiseGP) {
    double logNoise;
    m_noiseGP->evalGP( x, logNoise );
    var += exp(logNoise);
  }
  if(var<0) {
  	/*
  	cerr << endl<<"negative variance for x="<<x(0)<< endl;
  	cerr << "sigmaNoise="<<(*m_sigmaNoise)<< endl;
  	cerr << "cov.ell0="<<((CovFunc1D*)m_covFunc)->hyperParam[0] << " cov=sigma="<< ((CovFunc1D*)m_covFunc)->hyperParam[1]<< endl;
  	for(int i=0;i<m_numDataPoints;i++) {
  		cerr << (*m_dataPoints)[i](0) << "\t" << (*m_t)[i] << endl;
  	}
  	cerr <<"trying again.."<<endl;
  	buildGP(true);
	mean = inner_prod( m_k, *m_iCt );
  	var = m_covFunc->getCov(x,x) - inner_prod( m_k, prod( *m_iC, m_k ) );
  	cerr<<"mean="<<mean<<" var="<<var<<endl<<endl;
  	*/
  	var = -var;
  }
}



// -----------------------------------------------------------------------
template <class TInput>
void GPReg<TInput>::evalGP( const TInput &x, double &mean )
{
  assert( m_numDataPoints>0 ); // first load or build new gp
  TVector<double> m_k(m_numDataPoints);
  for (int i=0; i<m_numDataPoints; i++) {
    m_k[i] = m_covFunc->getCov( x, (*m_dataPoints)[i] );
  }
  mean = inner_prod( m_k, *m_iCt );
}



// -----------------------------------------------------------------------
template <class TInput>
double GPReg<TInput>::getDataFit()
{
  double dataFit = -0.5 * inner_prod( *m_t, *m_iCt );
  return dataFit;
}



// -----------------------------------------------------------------------
template <class TInput>
double GPReg<TInput>::getComplexity()
{
  double complPenalty = - 0.5 * log( lu_det( *m_C ) );
  return complPenalty;
}



// -----------------------------------------------------------------------
template <class TInput>
double GPReg<TInput>::getMarginalDataLikelihood()
{
  double dataFit = getDataFit();
  double complPenalty = getComplexity();
  double norm = - 0.5 * m_numDataPoints * log( 2.0*M_PI );
  double logLh = dataFit + complPenalty + norm;
  //fprintf( stderr, "   \t\t\t\t\t\t [%f %f -> %f]\n", dataFit, complPenalty, logLh );
  return logLh;
}

template <class TInput>
double GPReg<TInput>::getAvgVariance() {
  assert( m_numDataPoints>0 ); // first load or build new gp

  double mean,var, varSum=0,dist,distSum=0;
  for (int i1=0; i1<m_numDataPoints; i1++) {
	  TVector<double> m_k(m_numDataPoints);
	  for (int i2=0; i2<m_numDataPoints; i2++) {
	    m_k[i2] = (*m_C)(i1,i2);
	  }
	  mean = inner_prod( m_k, *m_iCt );
	  
	  dist = (*m_t)(i1) - mean;
	  distSum += dist*dist;
	  
	  var = (*m_C)(i1,i1) - inner_prod( m_k, prod( *m_iC, m_k ) );
	  var += exp(2.0 * (*m_sigmaNoise) );
	  varSum += var;
  }
  
  return(varSum/m_numDataPoints);
}



// -----------------------------------------------------------------------
/*
template <class TInput>
GPRegression<TInput>::double getObservationLikelihood( vector<Vector> &dataPoints, vector<double> &observations )
{
  int numObs = dataPoints.size();

  Matrix K__(numObs,numObs);
  //PROFILER_start();
  for (int i=0; i<numObs; i++) {
    for (int j=0; j<numObs; j++) {
      K__[i][j] = m_covFunc.getCov( dataPoints[i], dataPoints[j] );
      if (i==j) {
        K__[i][j] += exp(2.0 * (*m_sigmaNoise));
        if (m_noiseGP) {
          double logNoise;
          m_noiseGP->evalGP(dataPoints[i],logNoise);
          K__[i][j] += exp(logNoise);
        }
      }
    }
  }
  //fprintf( stderr, "  reg.getLh(1): %f\n", PROFILER_getElapsed() );

  //PROFILER_start();
  Matrix K_(numObs,m_numDataPoints);
  for (int i=0; i<numObs; i++) {
    for (int j=0; j<m_numDataPoints; j++) {
      K_[i][j] =  m_covFunc.getCov( dataPoints[i], m_dataPoints[j] );
    }
  }

  Matrix K = K__ - K_*(*m_iC)*(K_.Transp());

  //[cp] TEST!!
  for (int i=0; i<numObs; i++) {
  	K[i][i] += exp(2.0 * (*m_sigmaNoise) );
    if (m_noiseGP) {
      double logNoise;
      m_noiseGP->evalGP( dataPoints[i], logNoise );
      K[i][i] += exp(logNoise);
    }
  }

  //Matrix Ki = K.Pinv();
  //Matrix Ki = K.Invert();

  Matrix Ki2;
  K.Chol(Ki2);
  Matrix Kit = Ki2.Invert().Transp()*Ki2.Invert();

  Vector y_(numObs);
  double mean;
  for (int i=0; i<numObs; i++) {
    evalGP( dataPoints[i], mean );
    y_[i] = (observations[i]-mean);
  }

  double dotProd = (Kit*y_).Dot(y_);
  
  //fprintf( stderr, "  reg.getLh(2): %f\n", PROFILER_getElapsed() );
  double dataFit = -0.5 * dotProd;
  // doesn't take long...
  //double complPenalty = - 0.5 * log( K.Det() );
  //double norm = - 0.5 * numObs*log(2.0*M_PI);
  //double logLh = dataFit + complPenalty + norm;

  //fprintf( stderr, "# dataFit: %f\t complPenalty: %f\t norm: %f\t logLh: %f\n", dataFit, complPenalty, norm, logLh );
  //FILE *lhFile = fopen( "logLH.dat", "a" );
  //fprintf( lhFile, "%f %f %f %f\n", dataFit, complPenalty, norm, logLh );
  //fclose(lhFile);
  return dataFit;
}
*/

// ***************************************
// maximizing stuff with gsl
// ***************************************

template <class TInput>
double GPReg<TInput>::gsl_my_f(const gsl_vector *v, void *params){
  GPReg<TInput>* This = (GPReg<TInput>*) params;

  //cerr << "gsl_my_f" << endl;
  if (isnan(gsl_vector_get(v, 0)) || isnan(gsl_vector_get(v, 1)) || 
      isnan(gsl_vector_get(v, 2)) ) {
    if(GSL_VERBOSE) cout << "gsl_my_f: Abort (NaN)" << endl;
    return 0.0;
  }

  vector<double> hyperParam(This->m_covFunc->getNumParameter());
  for (int i=0; i<This->m_covFunc->getNumParameter(); i++) 
	  hyperParam[i] = BOUND(gsl_vector_get (v, i));
  This->m_covFunc->setHyperparameter(hyperParam);
  This->setSigmaNoise(BOUND(gsl_vector_get(v, This->m_covFunc->getNumParameter())));
  
  This->buildGP();
  //cerr << "dataLikelihood=" << This->getDataLikelihood() << endl;

  return -This->getDataLikelihood();
}

template <class TInput>
void GPReg<TInput>::gsl_my_df (const gsl_vector *v, void *params, gsl_vector *df) {
  GPReg<TInput>* This = (GPReg<TInput>*) params;

  //cerr << "gsl_my_df" << endl;
  if (isnan(gsl_vector_get(v, 0)) || isnan(gsl_vector_get(v, 1)) || 
      isnan(gsl_vector_get(v, 2))) {
    gsl_vector_set(df, 0, 0);
    gsl_vector_set(df, 1, 0);
    gsl_vector_set(df, 2, 0);
    if(GSL_VERBOSE) cout << "gsl_my_df: Abort (NaN)" << endl;
   return ;
  }

  
  vector<double> hyperParam(This->m_covFunc->getNumParameter());
  for (int i=0; i<This->m_covFunc->getNumParameter(); i++) 
	  hyperParam[i] = BOUND(gsl_vector_get (v, i));
  This->m_covFunc->setHyperparameter(hyperParam);
  This->setSigmaNoise(BOUND(gsl_vector_get(v, This->m_covFunc->getNumParameter())));
  
  This->buildGP();
  
  vector<double> vec;
  This->getDataLikelihood();
  This->getDerivative(&vec);

  for (unsigned i=0; i<vec.size(); i++) 
	  gsl_vector_set(df, i, -vec[i]);
}

template <class TInput>
void GPReg<TInput>::gsl_my_fdf (const gsl_vector *v, void *params, double *f, gsl_vector *df) {
  GPReg<TInput>* This = (GPReg<TInput>*) params;

  //cerr << "gsl_my_fdf" << endl;
  if (isnan(gsl_vector_get(v, 0)) || isnan(gsl_vector_get(v, 1)) || isnan(gsl_vector_get(v, 2)) ) {
    gsl_vector_set(df, 0, 0);
    gsl_vector_set(df, 1, 0);
    gsl_vector_set(df, 2, 0);
    *f = 0.0;
    if(GSL_VERBOSE) cout << "gsl_my_fdf: Abort NaN" << endl;
    return;
  }
  
  vector<double> hyperParam(This->m_covFunc->getNumParameter());
  for (int i=0; i<This->m_covFunc->getNumParameter(); i++) 
	  hyperParam[i] = BOUND(gsl_vector_get (v, i));
  This->m_covFunc->setHyperparameter(hyperParam);
  This->setSigmaNoise(BOUND(gsl_vector_get(v, This->m_covFunc->getNumParameter())));
  
  This->buildGP();
  
  vector<double> vec;
  *f = -This->getDataLikelihood();

  This->getDerivative(&vec);
  for (unsigned i=0; i<vec.size(); i++) 
	  gsl_vector_set(df, i, -vec[i]);

}

template <class TInput>
bool GPReg<TInput>::minimizeGSL(unsigned maxIt) {
  double logLH;

  if(GSL_VERBOSE) cout << "minimizeGSL\n";
  size_t iter = 0;
  gsl_vector *x;
  int status;
  
  x = gsl_vector_alloc (m_covFunc->getNumParameter()+1);
  for (int i=0; i<m_covFunc->getNumParameter(); i++)
  	  gsl_vector_set (x, i, m_covFunc->hyperParam[i]); 
  gsl_vector_set (x, m_covFunc->getNumParameter(), (*m_sigmaNoise));
  
  if(GSL_VERBOSE) cout << "starting with "<< gsl_vector_get (x, 0) << "; "<< gsl_vector_get (x, 1) << "; "<< gsl_vector_get (x, 2) << "; "<<"\n";
  
     
  const gsl_multimin_fdfminimizer_type *T;
  gsl_multimin_fdfminimizer *s; // = NULL;
  gsl_multimin_function_fdf my_func;
  my_func.f = &gsl_my_f;
  my_func.df = &gsl_my_df;
  my_func.fdf = &gsl_my_fdf;
  my_func.n = m_covFunc->getNumParameter()+1;
  my_func.params = this;

  T = gsl_multimin_fdfminimizer_conjugate_pr;
  s = gsl_multimin_fdfminimizer_alloc (T, m_covFunc->getNumParameter()+1);
  gsl_multimin_fdfminimizer_set (s, &my_func, x, 0.0001, 0.01);

  iter=0;
  do  {

    iter++;
    status = gsl_multimin_fdfminimizer_iterate (s);
    if (status) {
      // fatal error -> abort
      gsl_multimin_fdfminimizer_free (s);
      gsl_vector_free (x);
      if(GSL_VERBOSE) {
      	cout << "minimizeGSL: Abort" << endl;
      	cout << "minimizeGSL status " << status << endl;
      }
      return false;
      break;
    }
      
    status = gsl_multimin_test_gradient (s->gradient, 2);
    
  	vector<double> hyperParam(m_covFunc->getNumParameter());
  	for (int i=0; i<m_covFunc->getNumParameter(); i++) 
		  hyperParam[i] = BOUND(gsl_vector_get (s->x, i));
  	m_covFunc->setHyperparameter(hyperParam);
  	double sigmaNoise = BOUND(gsl_vector_get(s->x, m_covFunc->getNumParameter())) ;
	setSigmaNoise(sigmaNoise);
	if(sigmaNoise==MAX_EXP || sigmaNoise==MIN_EXP) iter=maxIt;

    logLH = s->f;
    
    if(GSL_VERBOSE) {
	    cout << "minimize_gsl: ell/sigma  "   << endl;
	    for(unsigned int i=0;i<m_covFunc->hyperParam.size();i++)
	    	cout << m_covFunc->hyperParam[i] << " ";
	    cout << endl;
	    
	    cout << "minimize_gsl: noise  "   << (*m_sigmaNoise) << endl;
	    cout << "minimize_gsl: Data Likelyhood " <<  -logLH << endl;
	    cout << "iteration:  " << iter << endl;
	    cout << "continue:  " << (status == GSL_CONTINUE) << endl;
    }
    
  }  while (status == GSL_CONTINUE && iter < maxIt);

  double norm = gsl_blas_dnrm2(s->gradient);
  gsl_multimin_fdfminimizer_free (s);
  gsl_vector_free (x);
  if (logLH == 0 && norm == 0) {
  	cout << "return false"<<endl;
    return false;
  } else
    return true;
}

