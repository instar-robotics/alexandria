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


#ifndef _MATH_H_
#define _MATH_H_

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

/*
 *  Could add lpnorm ?
 */

/*******************************************************************************************************/
/***********************************************  ArgMax   *********************************************/
/*******************************************************************************************************/

class ArgMax1D : public FScalar
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~ArgMax1D(){}

                virtual void upreload();
                virtual void compute();
                virtual void setparameters();
};

class ArgMax2D : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~ArgMax2D(){}

                virtual void upreload();
                virtual void compute();
                virtual void setparameters();
};


/*******************************************************************************************************/
/***********************************************  ArgMin   *********************************************/
/*******************************************************************************************************/

class ArgMin1D : public FScalar
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~ArgMin1D(){}

                virtual void upreload();
                virtual void compute();
                virtual void setparameters();
};

class ArgMin2D : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~ArgMin2D(){}

                virtual void upreload();
                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/**********************************************  MaxCoeff   ********************************************/
/*******************************************************************************************************/

class MaxCoeff : public FScalar
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MaxCoeff(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/*********************************************  MinCoeff   *********************************************/
/*******************************************************************************************************/

class MinCoeff : public FScalar
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MinCoeff(){}

                virtual void compute();
                virtual void setparameters();
};


/*******************************************************************************************************/
/************************************************  Max   ***********************************************/
/*******************************************************************************************************/

class SMax : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SMax(){}

                virtual void compute();
                virtual void setparameters();
};

class FMax : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~FMax(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/************************************************  Min   ***********************************************/
/*******************************************************************************************************/

class SMin : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SMin(){}

                virtual void compute();
                virtual void setparameters();
};

class FMin : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~FMin(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/************************************************  Abs   ***********************************************/
/*******************************************************************************************************/

class MAbs : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MAbs(){}

                virtual void compute();
                virtual void setparameters();
};

class SAbs : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SAbs(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/*********************************************  Modulo   ***********************************************/
/*******************************************************************************************************/

class MModulo : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISMInput modulo;

        public :

                virtual ~MModulo(){}

                virtual void compute();
                virtual void setparameters();
};

class MSModulo : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput modulo;

        public :

                virtual ~MSModulo(){}

                virtual void compute();
                virtual void setparameters();
};

class SModulo : public FScalar
{
        private :

                ISInput inScalar;
                ISInput modulo;

        public :

                virtual ~SModulo(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/********************************************  Derivative   ********************************************/
/*******************************************************************************************************/

class MDerivative : public FMatrix
{
        private :

                ISMInput inMatrix;

		MATRIX z_1;


        public :

                virtual ~MDerivative(){}

                virtual void compute();
                virtual void setparameters();
};

class SDerivative : public FScalar
{
        private :

                ISInput inScalar;

		SCALAR z_1;

        public :

                virtual ~SDerivative(){}

                virtual void compute();
                virtual void setparameters();
};

class MZ_1 : public FMatrix
{
        private :

                ISMInput inMatrix;
		
		MATRIX z_1;

        public :

                virtual ~MZ_1(){}

                virtual void compute();
                virtual void setparameters();
};

class SZ_1 : public FScalar
{
        private :

                ISInput inScalar;
		
		SCALAR z_1;

        public :

                virtual ~SZ_1(){}

                virtual void compute();
                virtual void setparameters();
};

class SZ_N : public FScalar
{
	private : 

		ISInput inScalar;
		ISInput N;

		std::list<SCALAR>::iterator index;
		std::list<SCALAR> z_n;
        
	public :

                virtual ~SZ_N(){}

                virtual void compute();
                virtual void setparameters();
};

class PolarToCart : public FMatrix 
{
        private :

		ISInput r;
		ISInput theta;

	public : 

		virtual ~PolarToCart() {}
                virtual void compute();
                virtual void setparameters();
};

class PolarToCartX : public FScalar
{
        private :

		ISInput r;
		ISInput theta;

	public : 

		virtual ~PolarToCartX() {}
                virtual void compute();
                virtual void setparameters();
};

class PolarToCartY : public FScalar
{
        private :

		ISInput r;
		ISInput theta;

	public : 

		virtual ~PolarToCartY() {}
                virtual void compute();
                virtual void setparameters();
};

class CartToPolar : public FMatrix 
{
        private :

		ISInput x;
		ISInput y;

	public : 

		virtual ~CartToPolar() {}
                virtual void compute();
                virtual void setparameters();
};

class CartToPolarR : public FScalar
{
        private :

		ISInput x;
		ISInput y;

	public : 

		virtual ~CartToPolarR() {}
                virtual void compute();
                virtual void setparameters();
};

class CartToPolarTheta : public FScalar
{
        private :

		ISInput x;
		ISInput y;

	public : 

		virtual ~CartToPolarTheta() {}
                virtual void compute();
                virtual void setparameters();
};

#endif // _MATH_H_
