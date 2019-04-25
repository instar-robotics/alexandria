/*
  Copyright (C) INSTAR Robotics

  Author: Pierre Delarboulas
 
  This file is part of alexandria <https://github.com/instar-robotics/alexandria>.
 
  alexandria is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  alexandria is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with dogtag. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _FILTER_H_
#define _FILTER_H_

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

// See here for more Filter functions : https://fr.wikipedia.org/wiki/Fonction_d%27activation
// TODO : 
// Relu / PRelu / ELU
// Sinc
// SoftSign
// SoftExpo

/*******************************************************************************************************/
/******************************************  Front Detection   *****************************************/
/*******************************************************************************************************/

const IString FD_SUP = "up" ;
const IString FD_SDOWN = "down" ;
const IString FD_SBOTH = "both" ;

const int FD_IUP = 1 ;
const int FD_IDOWN = 0 ;
const int FD_IBOTH = -1 ;

bool checkMode(const std::string & smode, int &mode);
SCALAR getFront(const SCALAR& z_1,const  SCALAR& Z,const SCALAR &thres, int mode);

class SFrontDetection : public FScalar
{
        private :

                ISInput inScalar;
                ISInput threshold;
		IString mode;
		
		SCALAR z_1;
		int imode;

        public :

                virtual ~SFrontDetection(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();
};

template<class ArgType>
class MSFD_Functor{
	const int &mode;
	const typename ArgType::Scalar &thres;
	const ArgType &Z_1;
	const ArgType &Z;

	public :
	MSFD_Functor(const int &mode, const typename ArgType::Scalar &thres,const ArgType &Z_1,const ArgType &Z) : mode(mode), thres(thres),Z_1(Z_1), Z(Z) {} 

	const typename ArgType::Scalar operator() (Index r, Index c) const {
		return getFront(Z_1(r,c), Z(r,c), thres, mode);
 	}
};


class MSFrontDetection : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput threshold;
		IString mode;

		MATRIX z_1;
		int imode;

        public :

                virtual ~MSFrontDetection(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();
};

template<class ArgType>
class MMFD_Functor{
	const int &mode;
	const ArgType &thres;
	const ArgType &Z_1;
	const ArgType &Z;

	public :
	MMFD_Functor(const int &mode, const ArgType &thres,const ArgType &Z_1,const ArgType &Z) : mode(mode), thres(thres),Z_1(Z_1), Z(Z) {} 

	const typename ArgType::Scalar operator() (Index r, Index c) const {
		return getFront(Z_1(r,c), Z(r,c), thres(r,c), mode);
 	}
};

class MMFrontDetection : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISMInput threshold;
		IString mode;
		
		MATRIX z_1;
		int imode;

        public :

                virtual ~MMFrontDetection(){}

                virtual void prerun();
                virtual void compute();
                virtual void setparameters();
};


/*******************************************************************************************************/
/******************************************  PiecewiseLin   ********************************************/
/*******************************************************************************************************/

// bounded [0,1]
class MPiecewiseLin : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MPiecewiseLin(){}

                virtual void compute();
                virtual void setparameters();
};

class SPiecewiseLin : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SPiecewiseLin(){}

                virtual void compute();
                virtual void setparameters();
};

// bounded [min,max]
class MPiecewiseLinCustom : public FMatrix
{
        private :

                ISMInput thresMin;
                ISMInput thresMax;

                ISMInput inMatrix;

        public :

                virtual ~MPiecewiseLinCustom(){}

                virtual void compute();
                virtual void setparameters();
};

class SPiecewiseLinCustom : public FScalar
{
        private :

                ISInput thresMin;
                ISInput thresMax;

                ISInput inScalar;

        public :

                virtual ~SPiecewiseLinCustom(){}

                virtual void compute();
                virtual void setparameters();
};

class MSSPiecewiseLinCustom : public FMatrix
{
        private :

                ISMInput inMatrix;

                ISInput thresMin;
                ISInput thresMax;

        public :

                virtual ~MSSPiecewiseLinCustom(){}

                virtual void compute();
                virtual void setparameters();
};


class MMSPiecewiseLinCustom : public FMatrix
{
        private :
                ISMInput inMatrix;

                ISMInput thresMin;
                ISInput thresMax;

        public :

                virtual ~MMSPiecewiseLinCustom(){}

                virtual void compute();
                virtual void setparameters();
};

class MSMPiecewiseLinCustom : public FMatrix
{
        private :

                ISMInput inMatrix;

                ISInput thresMin;
                ISMInput thresMax;

        public :

                virtual ~MSMPiecewiseLinCustom(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/*******************************************  Heaviside   **********************************************/
/*******************************************************************************************************/

template<typename Scalar>
struct HeaviFunc {
  HeaviFunc(const Scalar& thres) : thres(thres) {}
  const Scalar operator()(const Scalar& x) const { return  x < thres ? 0.0 : 1.0;   }
  Scalar thres;
};


// bounded [0,1]
class MHeaviside : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MHeaviside(){}

                virtual void compute();
                virtual void setparameters();
};

class SHeaviside : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SHeaviside(){}

                virtual void compute();
                virtual void setparameters();
};



class SHeavisideCustom : public FScalar
{
        private :

                ISInput inScalar;
		ISInput thres;

        public :

                virtual ~SHeavisideCustom(){}

                virtual void compute();
                virtual void setparameters();
};

class MSHeavisideCustom : public FMatrix
{
        private :

                ISMInput inMatrix;
		ISInput thres;

        public :

                virtual ~MSHeavisideCustom(){}

                virtual void compute();
                virtual void setparameters();
};

class MMHeavisideCustom : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISMInput thres;

        public :

                virtual ~MMHeavisideCustom(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/*********************************************  SIGMOID   **********************************************/
/*******************************************************************************************************/

class SSigmoid : public FScalar
{
	private : 
		ISInput inScalar;

	public : 

		virtual ~SSigmoid(){}

		virtual void compute();
		virtual void setparameters();
};

class MSigmoid : public FMatrix
{
	private : 
		ISMInput inMatrix;

	public : 

		virtual ~MSigmoid(){}

		virtual void compute();
		virtual void setparameters();
};

class SSigmoidLambda : public FScalar
{
	private : 
		ISInput inScalar;
		ISInput lambda;

	public : 

		virtual ~SSigmoidLambda(){}

		virtual void compute();
		virtual void setparameters();
};

class MSSigmoidLambda : public FMatrix
{
	private : 
		ISMInput inMatrix;
		ISInput lambda;

	public : 

		virtual ~MSSigmoidLambda(){}

		virtual void compute();
		virtual void setparameters();
};

class MMSigmoidLambda : public FMatrix
{
	private : 
		ISMInput inMatrix;
		ISMInput lambda;

	public : 

		virtual ~MMSigmoidLambda(){}

		virtual void compute();
		virtual void setparameters();
};

/*******************************************************************************************************/
/**********************************************  Gaussian   ********************************************/
/*******************************************************************************************************/

class SGauss : public FScalar
{
        private :

                ISInput inScalar;
                ISInput mu;
                ISInput sigma;

        public :
                virtual ~SGauss(){}

                virtual void compute();
                virtual void setparameters();
};

class MSGauss : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput mu;
                ISInput sigma;

        public :
                virtual ~MSGauss(){}

                virtual void compute();
                virtual void setparameters();
};

class MMGauss : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISMInput mu;
                ISMInput sigma;

        public :
                virtual ~MMGauss(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/**************************************************  DoG ***********************************************/
/*******************************************************************************************************/

class SDoG : public FScalar
{
        private :
                ISInput inScalar;
                ISInput mu1;
                ISInput sigma1;

                ISInput mu2;
                ISInput sigma2;

        public :
                virtual ~SDoG(){}

                virtual void compute();
                virtual void setparameters();
};

class MSDoG : public FMatrix
{
        private :
                ISMInput inMatrix;
                ISInput mu1;
                ISInput sigma1;

                ISInput mu2;
                ISInput sigma2;

        public :
                virtual ~MSDoG(){}

                virtual void compute();
                virtual void setparameters();
};

class MMDoG : public FMatrix
{
        private :
                ISMInput inMatrix;
                ISMInput mu1;
                ISMInput sigma1;

                ISMInput mu2;
                ISMInput sigma2;

        public :
                virtual ~MMDoG(){}

                virtual void compute();
                virtual void setparameters();
};


#endif // _FILTER_H_
