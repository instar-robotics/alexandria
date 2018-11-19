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
/*
Gaussian
GaussianDiff
Cos/Sin
Tanh
Sinus Cardinal
Arc tangente
SquareSignal or RectangularWave
TriangleSignal
Fonction porte  or Rectangular Function
*/

/*******************************************************************************************************/
/*******************************************  Dirac Field  *********************************************/
/*******************************************************************************************************/

class DiracField1D : public FMatrix
{
	private : 

		ISInput x; 
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
  const typename ArgType::Index &thx;
public:
  Heavi1D_functor(const typename ArgType::Index & thx) : thx(thx) {}
  const typename ArgType::Scalar operator() (Index ind) const {
    return  (ind >= thx) ? 1.0 : 0.0  ;
  }
};


class HeavisideField1D : public FMatrix
{
	private : 

		ISInput thres; 

	public : 

		virtual ~HeavisideField1D(){}

		virtual void compute();
		virtual void setparameters();
};

template<class ArgType>
class Heavi2D_functor {
  const typename ArgType::Index &thx;
  const typename ArgType::Index &thy;
public:
  Heavi2D_functor(const typename ArgType::Index& thx, const typename ArgType::Index& thy) : thx(thx),thy(thy) {}
  const  typename ArgType::Scalar operator() (Index row, Index col) const {
    return  (row >= thx && col >= thy) ? 1.0 : 0.0  ;
  }
};


class HeavisideField2D : public FMatrix
{
	private : 

		ISInput thres_x;
		ISInput thres_y;
	
	public : 

		virtual ~HeavisideField2D(){}

		virtual void compute();
		virtual void setparameters();
};


/*******************************************************************************************************/
/*******************************************  Sinus Field  *********************************************/
/*******************************************************************************************************/

template<class ArgType>
class Sin_functor {
  const typename ArgType::Index &max;
  const typename ArgType::Scalar &off;
  const typename ArgType::Scalar &freq;
public:
  Sin_functor(const typename ArgType::Index & max,const typename ArgType::Scalar& off, const typename ArgType::Scalar& freq) : max(max), off(off), freq(freq) {}
  const typename ArgType::Scalar operator() (Index row) const {
    return  sin( ((row * 2 * M_PI) / max) * freq + off );
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

/*******************************************************************************************************/
/*****************************************  Cosinus Field  *********************************************/
/*******************************************************************************************************/

template<class ArgType>
class Cosin_functor {
  const typename ArgType::Index &max;
  const typename ArgType::Scalar &off;
  const typename ArgType::Scalar &freq;
public:
  Cosin_functor(const typename ArgType::Index & max,const typename ArgType::Scalar& off, const typename ArgType::Scalar& freq) : max(max), off(off), freq(freq) {}
  const typename ArgType::Scalar operator() (Index row) const {
    return  sin( ((row * 2 * M_PI) / max) * freq + off );
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

/*******************************************************************************************************/
/*****************************************  Gaussian Field  ********************************************/
/*******************************************************************************************************/

class GaussianField1D : public FMatrix
{
	private :
		ISInput mu;
		ISInput sigma;
};

class GaussianField2D : public FMatrix
{

};

#endif // _FIELD_GENERATOR_H_
