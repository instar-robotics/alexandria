/*
Copyright INSTAR Robotics

Author: Pierre Delarboulas

This software is governed by the CeCILL v2.1 license under French law and abiding by the rules of distribution of free software. 
You can use, modify and/ or redistribute the software under the terms of the CeCILL v2.1 license as circulated by CEA, CNRS and INRIA at the following URL "http://www.cecill.info".
As a counterpart to the access to the source code and  rights to copy, modify and redistribute granted by the license, 
users are provided only with a limited warranty and the software's author, the holder of the economic rights,  and the successive licensors have only limited liability.  
In this respect, the user's attention is drawn to the risks associated with loading, using, modifying and/or developing or reproducing the software by the user in light of its specific status of free software, 
that may mean  that it is complicated to manipulate, and that also therefore means that it is reserved for developers and experienced professionals having in-depth computer knowledge. 
Users are therefore encouraged to load and test the software's suitability as regards their requirements in conditions enabling the security of their systems and/or data to be ensured 
and, more generally, to use and operate it in the same conditions as regards security. 
The fact that you are presently reading this means that you have had knowledge of the CeCILL v2.1 license and that you accept its terms.
*/

#ifndef _FIELD_GENERATOR_H_
#define _FIELD_GENERATOR_H_

#include "kheops/kernel/kernel.h"
#include "kheops/kernel/function.h"
#include <limits>
#include <algorithm>

/*
TODO : 
- Wave : SquarWave and TriangleWave
- Gabor
- Ondelette
*/

/*******************************************************************************************************/
/*******************************************  Dirac Field  *********************************************/
/*******************************************************************************************************/

class DiracField1D : public FMatrix
{
	private : 

		ISInput x; 
		ISInput N; 
		MatrixXd::Index lastIndex;

	public : 

		DiracField1D() : lastIndex(0){}
		virtual ~DiracField1D(){}

		virtual void compute();
		virtual void setparameters();
};

class DiracField2D : public FMatrix
{
	private : 

		ISInput x;
		ISInput y;
		ISInput N; 

		MatrixXd::Index lastIx;
		MatrixXd::Index lastIy;
	
	public : 

		DiracField2D() : lastIx(0), lastIy(0){}
		virtual ~DiracField2D(){}

		virtual void compute();
		virtual void setparameters();
};

/*******************************************************************************************************/
/*****************************************  Heaviside Field  *******************************************/
/*******************************************************************************************************/

template<class ArgType>
class Heavi1D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Scalar &thx;
  const typename ArgType::Index &max;

public:
  Heavi1D_functor(const typename ArgType::Scalar & N, const typename ArgType::Scalar& thx, const typename ArgType::Index& max) : N(N), thx(thx), max(max) {}
  const typename ArgType::Scalar operator() (Index ind) const {
	 
	double x = double(ind) * 2.0* N / max - (N);
	return  (x >= thx) ? 1.0 : 0.0  ;
  }
};

class HeavisideField1D : public FMatrix
{
	private : 

		ISInput th; 
		ISInput N; 

	public : 

		virtual ~HeavisideField1D(){}

		virtual void compute();
		virtual void setparameters();
};

template<class ArgType>
class Heavi2D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Scalar &thx;
  const typename ArgType::Scalar &thy;
  const typename ArgType::Index &max_x;
  const typename ArgType::Index &max_y;

public:
  Heavi2D_functor(const typename ArgType::Scalar &N , const typename ArgType::Scalar& thx, const typename ArgType::Scalar& thy,  const typename ArgType::Index& max_x , const typename ArgType::Index& max_y) : N(N), thx(thx),thy(thy), max_x(max_x), max_y(max_y) {}
  const  typename ArgType::Scalar operator() (Index row, Index col) const {
	double x = double(col) * 2.0* N / max_x - N;
	double y = double(row) * 2.0* N / max_y - N;
	return  ( y >= thy && x >= thx) ? 1.0 : 0.0  ;
  }
};


class HeavisideField2D : public FMatrix
{
	private : 

		ISInput thx;
		ISInput thy;
		ISInput N; 
	
	public : 

		virtual ~HeavisideField2D(){}

		virtual void compute();
		virtual void setparameters();
};

/*******************************************************************************************************/
/*******************************************  Gate Field  **********************************************/
/*******************************************************************************************************/

template<class ArgType>
class Gate1D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Scalar &thx;
  const typename ArgType::Index &max;

public:
  Gate1D_functor(const typename ArgType::Scalar & N, const typename ArgType::Scalar &thx , const typename ArgType::Index& max ) : N(N), thx(thx), max(max) {}
  const typename ArgType::Scalar operator() (Index ind) const {
    double x = double(ind) * 2.0* N / max - (N);
    return  (x >= -thx && x <= thx) ? 1.0 : 0.0  ;
  }
};

class GateField1D : public FMatrix
{
        private :

		ISInput wth;
		ISInput N; 

        public :
                virtual ~GateField1D(){}
                virtual void compute();
                virtual void setparameters();
};

template<class ArgType>
class Gate2D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Scalar &thx;
  const typename ArgType::Scalar &thy;
  const typename ArgType::Index &max_x;
  const typename ArgType::Index &max_y;

public:
  Gate2D_functor(const typename ArgType::Scalar& N, const typename ArgType::Scalar& thx, const typename ArgType::Scalar &thy, const typename ArgType::Index& max_x , const typename ArgType::Index& max_y ) : N(N), thx(thx),thy(thy), max_x(max_x), max_y(max_y) {}
  const  typename ArgType::Scalar operator() (Index row, Index col) const {
    double x = double(col) * 2.0* N / max_x - N;
    double y = double(row) * 2.0* N / max_y - N;
    return  (y >= -thy && y <= thy  && x >= -thx && x <= thx) ? 1.0 : 0.0  ;
  }
};

class GateField2D : public FMatrix
{
        private :
		ISInput wthx;
		ISInput wthy;
		ISInput N; 

        public :
                virtual ~GateField2D(){}
                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/****************************************  Triangular Field  *******************************************/
/*******************************************************************************************************/

template<class ArgType>
class Triangular1D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Scalar &a;
  const typename ArgType::Index &max;

public:
  Triangular1D_functor(const typename ArgType::Scalar & N, const typename ArgType::Scalar &a  ,const typename ArgType::Index& max ) : N(N), a(a), max(max) {}
  const typename ArgType::Scalar operator() (Index ind) const {
    double x = double(ind) * 2.0* N / max - N;
    return  std::max( 1 - fabs(a*x), 0.0) ;
  }
};

class TriangularField1D : public FMatrix
{
        private :
                ISInput a;
                ISInput N;

        public :
                virtual ~TriangularField1D(){}
                virtual void compute();
                virtual void setparameters();
};

template<class ArgType>
class Triangular2D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Scalar &ax;
  const typename ArgType::Scalar &ay;
  const typename ArgType::Index &max_x;
  const typename ArgType::Index &max_y;

public:
  Triangular2D_functor(const typename ArgType::Scalar& N, const typename ArgType::Scalar& ax, const typename ArgType::Scalar &ay, const typename ArgType::Index& max_x , const typename ArgType::Index& max_y ) : N(N), ax(ax),ay(ay), max_x(max_x), max_y(max_y) {}
  const  typename ArgType::Scalar operator() (Index row, Index col) const {
    double x = double(col) * 2.0* N / max_x - N;
    double y = double(row) * 2.0* N / max_y - N;
    return std::max( 1 - (fabs( ax *x ) + fabs( ay * y)), 0.0);
  }
};

class TriangularField2D : public FMatrix
{
        private :
                ISInput ax;
                ISInput ay;
                ISInput N;

        public :
                virtual ~TriangularField2D(){}
                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/*******************************************  Sinus Field  *********************************************/
/*******************************************************************************************************/

template<class ArgType>
class Sin1D_functor {

  const typename ArgType::Scalar &freq;
  const typename ArgType::Scalar &off;
  const typename ArgType::Index &max;

public:
  Sin1D_functor(const typename ArgType::Scalar & freq,const typename ArgType::Scalar& off, const typename ArgType::Index& max) : freq(freq), off(off), max(max) {}
  const typename ArgType::Scalar operator() (Index ind) const {
    return  sin(  (double(ind) * 2.0* M_PI / max - (M_PI)) * freq + off );
  }
};

class SinusField1D : public FMatrix
{
	private :

		ISInput freq;
		ISInput offset;

	public : 

		virtual ~SinusField1D(){}

		virtual void compute();
		virtual void setparameters();
};

template<class ArgType>
class Sin2D_functor {

  const typename ArgType::Scalar &freq_x;
  const typename ArgType::Scalar &off_x;

  const typename ArgType::Scalar &freq_y;
  const typename ArgType::Scalar &off_y;

  const typename ArgType::Index &max_x;
  const typename ArgType::Index &max_y;
public:
  Sin2D_functor(const typename ArgType::Scalar & freq_x, const typename ArgType::Scalar& off_x, const typename ArgType::Scalar & freq_y , const typename ArgType::Scalar & off_y, const typename ArgType::Index & max_x, const typename ArgType::Index & max_y ) : freq_x(freq_x), off_x(off_x), freq_y(freq_y), off_y(off_y), max_x(max_x),max_y(max_y) {}
  const typename ArgType::Scalar operator() (Index row, Index col) const {
    return  sin(  (double(col) * 2.0* M_PI / max_x - (M_PI))   * freq_x + off_x ) * sin(  (double(row) * 2.0* M_PI / max_y - (M_PI))  * freq_y + off_y ); 
  }
};

class SinusField2D : public FMatrix
{
	private :

		ISInput freq_x;
		ISInput offset_x;
		
		ISInput freq_y;
		ISInput offset_y;

	public : 

		virtual ~SinusField2D(){}

		virtual void compute();
		virtual void setparameters();
};

/*******************************************************************************************************/
/*****************************************  Cosinus Field  *********************************************/
/*******************************************************************************************************/

template<class ArgType>
class Cos1D_functor {

  const typename ArgType::Scalar &freq;
  const typename ArgType::Scalar &off;
  const typename ArgType::Index &max;

public:
  Cos1D_functor(const typename ArgType::Scalar & freq,const typename ArgType::Scalar& off, const typename ArgType::Index& max) : freq(freq), off(off), max(max) {}
  const typename ArgType::Scalar operator() (Index ind) const {
    return  cos( (double(ind) * 2.0* M_PI / max - (M_PI)) * freq + off );
  }
};


class CosinusField1D : public FMatrix
{
	private :

		ISInput freq;
		ISInput offset;

	public : 

		virtual ~CosinusField1D(){}

		virtual void compute();
		virtual void setparameters();
};

template<class ArgType>
class Cos2D_functor {
  const typename ArgType::Scalar &freq_x;
  const typename ArgType::Scalar &off_x;

  const typename ArgType::Scalar &freq_y;
  const typename ArgType::Scalar &off_y;

  const typename ArgType::Index &max_x;
  const typename ArgType::Index &max_y;
public:
  Cos2D_functor(const typename ArgType::Scalar & freq_x, const typename ArgType::Scalar& off_x, const typename ArgType::Scalar & freq_y , const typename ArgType::Scalar & off_y, const typename ArgType::Index & max_x, const typename ArgType::Index & max_y ) : freq_x(freq_x), off_x(off_x), freq_y(freq_y), off_y(off_y), max_x(max_x),max_y(max_y) {}
  const typename ArgType::Scalar operator() (Index row, Index col) const {
    return  cos(  (double(col) * 2.0* M_PI / max_x - (M_PI))   * freq_x + off_x ) * sin(  (double(row) * 2.0* M_PI / max_y - (M_PI))  * freq_y + off_y );
  }
};


class CosinusField2D : public FMatrix
{
	private :

		ISInput freq_x;
		ISInput offset_x;
		
		ISInput freq_y;
		ISInput offset_y;

	public : 

		virtual ~CosinusField2D(){}

		virtual void compute();
		virtual void setparameters();
};

/*******************************************************************************************************/
/*****************************************  Gaussian Field  ********************************************/
/*******************************************************************************************************/

template<class ArgType>
class Gauss1D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Scalar &sigma;
  const typename ArgType::Scalar &mu;
  const typename ArgType::Index &max;

public:
  Gauss1D_functor(const typename ArgType::Scalar& N, const typename ArgType::Scalar& sigma, const typename ArgType::Scalar& mu, const typename ArgType::Index& max) : N(N), sigma(sigma), mu(mu),max(max) {}
  const typename ArgType::Scalar operator() (Index ind) const {
	double x = double(ind) * 2.0* N / max - (N);
	return  (1.0/(sigma * sqrt(2*M_PI) )) * exp( -pow(x - mu, 2) / (2 * pow(sigma,2) ));
  }
};

class GaussianField1D : public FMatrix
{
	private :
		ISInput mu;
		ISInput sigma;
		ISInput N;
		
	public : 
		virtual ~GaussianField1D(){}

		virtual void compute();
		virtual void setparameters();
};

template<class ArgType>
class Gauss2D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Scalar &sigmax;
  const typename ArgType::Scalar &mux;
  const typename ArgType::Scalar &sigmay;
  const typename ArgType::Scalar &muy;
  const typename ArgType::Index &max_x;
  const typename ArgType::Index &max_y;

public:
  Gauss2D_functor( const typename ArgType::Scalar& N , const typename ArgType::Scalar& sigmax, const typename ArgType::Scalar& mux,const typename ArgType::Scalar& sigmay, const typename ArgType::Scalar& muy, const typename ArgType::Index& max_x , const typename ArgType::Index& max_y) : N(N), sigmax(sigmax), mux(mux),sigmay(sigmay),muy(muy),max_x(max_x), max_y(max_y) {}
  const typename ArgType::Scalar operator() (Index row, Index col) const {

        double x = double(col) * 2.0* N / max_x - (N);
        double y = double(row) * 2.0* N / max_y - (N);
        return (1.0/(2.0*M_PI*sigmax*sigmay)) * exp(-(pow(x-mux,2)/(2.0*pow(sigmax,2))) - (pow(y-muy,2)/(2.0*pow(sigmay,2))));
  }
};


class GaussianField2D : public FMatrix
{
	private :
		ISInput mu_x;
		ISInput sigma_x;
		ISInput mu_y;
		ISInput sigma_y;
		ISInput N;
	
	public : 
		virtual ~GaussianField2D(){}

		virtual void compute();
		virtual void setparameters();
};

/*******************************************************************************************************/
/*******************************************  DoG Field  ***********************************************/
/*******************************************************************************************************/

template<class ArgType>
class DoG1D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Scalar &sigma1;
  const typename ArgType::Scalar &sigma2;
  const typename ArgType::Index &max;

public:
  DoG1D_functor(const typename ArgType::Scalar& N, const typename ArgType::Scalar& sigma1, const typename ArgType::Scalar& sigma2, const typename ArgType::Index& max) : N(N), sigma1(sigma1), sigma2(sigma2), max(max) {}
  const typename ArgType::Scalar operator() (Index ind) const {
	double x = double(ind) * 2.0* N / max - (N) ;
		
	return (1.0/(sigma1 * sqrt(2*M_PI) )) *  exp( - pow(x, 2) / (2 * pow(sigma1,2) )) - (1.0/(sigma2 * sqrt(2*M_PI) )) *  exp( - pow(x, 2) / (2 * pow(sigma2,2) ));
  }
};

class DoGField1D : public FMatrix
{
	private :
		ISInput sigma1;
		ISInput sigma2;
		ISInput N;
		
	public : 
		virtual ~DoGField1D(){}

		virtual void compute();
		virtual void setparameters();
};

template<class ArgType>
class DoG2D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Scalar &sigma1_x;
  const typename ArgType::Scalar &sigma1_y;
  const typename ArgType::Scalar &sigma2_x;
  const typename ArgType::Scalar &sigma2_y;
  const typename ArgType::Index &max_x;
  const typename ArgType::Index &max_y;

public:
  DoG2D_functor(const typename ArgType::Scalar& N , const typename ArgType::Scalar& sigma1_x, const typename ArgType::Scalar& sigma1_y,const typename ArgType::Scalar& sigma2_x, const typename ArgType::Scalar& sigma2_y, const typename ArgType::Index& max_x , const typename ArgType::Index& max_y) : N(N), sigma1_x(sigma1_x),sigma1_y(sigma1_y),sigma2_x(sigma2_x), sigma2_y(sigma2_y),max_x(max_x), max_y(max_y) {}
  const typename ArgType::Scalar operator() (Index row, Index col) const {

        double x = double(col) * 2.0* N / max_x - (N);
        double y = double(row) * 2.0* N / max_y - (N);
        
	return (1.0/(2.0*M_PI*sigma1_x*sigma1_y)) * exp(-(pow(x,2)/(2.0*pow(sigma1_x,2))) - (pow(y,2)/(2.0*pow(sigma1_y,2)))) - (1.0/(2.0*M_PI*sigma2_x*sigma2_y)) * exp(-(pow(x,2)/(2.0*pow(sigma2_x,2))) - (pow(y,2)/(2.0*pow(sigma2_y,2))));
  }
};

class DoGField2D : public FMatrix
{
	private :
		ISInput sigma1_x;
		ISInput sigma1_y;
		ISInput sigma2_x;
		ISInput sigma2_y;
		ISInput N;
		
	public : 
		virtual ~DoGField2D(){}

		virtual void compute();
		virtual void setparameters();
};

/*******************************************************************************************************/
/*******************************************  Sinc Field  **********************************************/
/*******************************************************************************************************/

template<class ArgType>
class Sinc1D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Scalar &freq;
  const typename ArgType::Index &max;

public:
  Sinc1D_functor(const typename ArgType::Scalar & N,const typename ArgType::Scalar& freq, const typename ArgType::Index& max) : N(N), freq(freq), max(max) {}
  const typename ArgType::Scalar operator() (Index ind) const {
    double x = double(ind) * 2.0* N / max - N ;
    if( x == 0.0 ) x = std::numeric_limits<double>::epsilon();
    return  sin(x  * freq ) / (x * freq ) ;
  }
};

class SincField1D : public FMatrix
{
	private : 

                ISInput freq;
		ISInput N;

	public : 
		virtual ~SincField1D(){}
		virtual void compute();
		virtual void setparameters();
};

template<class ArgType>
class Sinc2D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Scalar &freq_x;
  const typename ArgType::Scalar &freq_y;
  const typename ArgType::Index &max_x;
  const typename ArgType::Index &max_y;

public:
  Sinc2D_functor(const typename ArgType::Scalar& N, const typename ArgType::Scalar& freq_x, const typename ArgType::Scalar& freq_y,  const typename ArgType::Index & max_x, const typename ArgType::Index & max_y) : N(N), freq_x(freq_x), freq_y(freq_y), max_x(max_x), max_y(max_y){}
  const typename ArgType::Scalar operator() (Index row, Index col) const {
	double x = double(col) * 2.0 * N / max_x - N;
	double y = double(row) * 2.0 * N / max_y - N; 
    if( x == 0.0 ) x = std::numeric_limits<double>::epsilon();
    if( y == 0.0 ) y = std::numeric_limits<double>::epsilon();
    return  sin( x * freq_x ) * sin( y + freq_y  ) / ( freq_x * x * freq_y * y ); 
  }
};

class SincField2D : public FMatrix
{
	private : 

                ISInput freq_x;
                ISInput freq_y;
		ISInput N;

	public : 
		virtual ~SincField2D(){}
		virtual void compute();
		virtual void setparameters();
};

/*******************************************************************************************************/
/****************************************  ChineseHat Field  *******************************************/
/*******************************************************************************************************/

template<class ArgType>
class ChineseHat1D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Index &max;
public:
  ChineseHat1D_functor(const typename ArgType::Scalar & N,const typename ArgType::Index & max) : N(N), max(max) {}
  const typename ArgType::Scalar operator() (Index ind) const {
    double x = double(ind) * 2.0* N / max - N ;
    return (N - log( cosh(x))) / N ;
  }
};

template<class ArgType>
class ChineseHat2D_functor {
  const typename ArgType::Scalar &N;
  const typename ArgType::Index &max_x;
  const typename ArgType::Index &max_y;
public:
  ChineseHat2D_functor(const typename ArgType::Scalar & N,const typename ArgType::Index & max_x, const typename ArgType::Index & max_y) : N(N), max_x(max_x), max_y(max_y) {}
  const typename ArgType::Scalar operator() (Index row, Index col) const {
    double x = double(col) * 2.0* N / max_x - N ;
    double y = double(row) * 2.0* N / max_y - N ;
    return (N - log( cosh( sqrt( x * x + y * y )))) / N ;
  }
};

class ChineseHatField : public FMatrix
{
	private : 
		ISInput N;
		unsigned int dim;

	public : 
		ChineseHatField() : dim(0) {}
		virtual ~ChineseHatField(){}
		virtual void compute();
		virtual void setparameters();
};

#endif // _FIELD_GENERATOR_H_