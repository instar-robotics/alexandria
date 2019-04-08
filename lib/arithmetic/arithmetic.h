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


#ifndef _ARITHMETIC_H_
#define _ARITHMETIC_H_

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

/*
// TODO : 
*   Tensor product of 2 vector
*   Inner Product
*   Outer Product 
*/

/*******************************************************************************************************/
/*********************************************   SUM   *************************************************/
/*******************************************************************************************************/

class MSum : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MSum(){}

                virtual void compute();
                virtual void setparameters();
};

class SSum : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SSum(){}

                virtual void compute();
                virtual void setparameters();
};

class MSSum : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput inScalar;

        public :

                virtual ~MSSum(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/****************************************   Substraction   *********************************************/
/*******************************************************************************************************/

class MMSub : public FMatrix
{
        private :

		// Inputs are unique !
                ISMInput diminuende;
                ISMInput subtrahend;

        public :

                virtual ~MMSub(){}

                virtual void compute();
                virtual void setparameters();
};

class MSSub : public FMatrix
{
        private :

		// Inputs are unique !
                ISMInput diminuende;
                ISInput subtrahend;

        public :

                virtual ~MSSub(){}

                virtual void compute();
                virtual void setparameters();
};

class SSSub : public FScalar
{
        private :

		// Inputs are unique !
                ISInput diminuende;
                ISInput subtrahend;

        public :

                virtual ~SSSub(){}

                virtual void compute();
                virtual void setparameters();
};


/*******************************************************************************************************/
/******************************************   Product   ************************************************/
/*******************************************************************************************************/

// Wise Product : coefficient-wise operator 
class MMul : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :
                virtual ~MMul(){}

                virtual void compute();
                virtual void setparameters();
};

// Scalar Multiplication
class SMul : public FScalar
{
        private :

                ISInput inScalar;

        public :
                virtual ~SMul(){}

                virtual void compute();
                virtual void setparameters();
};

//Combination of Wise Product and Scalar product 
// For example : 
// With m1, m2 two matrix
// And s1, s2 two scalar 
// output = m1 cwiseProduct m2 * s1 * s2 
class MSMul : public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput inScalar;

        public :
                virtual ~MSMul(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/*****************************************   Division   ************************************************/
/*******************************************************************************************************/

class MMDiv : public FMatrix
{
        private :
		// Inputs are unique
                ISMInput numerator;
                ISMInput denumerator;

        public :

                virtual ~MMDiv(){}

                virtual void compute();
                virtual void setparameters();
};

class MSDiv : public FMatrix
{
        private :

		// Inputs are unique
                ISMInput numerator;
                ISInput denumerator;

        public :

                virtual ~MSDiv(){}

                virtual void compute();
                virtual void setparameters();
};

class SSDiv : public FScalar
{
        private :

		// Inputs are unique
                ISInput numerator;
                ISInput denumerator;

        public :

                virtual ~SSDiv(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/********************************************  Normalization   *****************************************/
/*******************************************************************************************************/

class Norm : public FScalar
{
        private :

                ISMInput inMatrix;

        public :
                virtual ~Norm(){}

                virtual void compute();
                virtual void setparameters();
};

class SquaredNorm : public FScalar
{
        private :

                ISMInput inMatrix;

        public :
                virtual ~SquaredNorm(){}

                virtual void compute();
                virtual void setparameters();
};

class Normalize : public FMatrix
{
        private :
                ISMInput inMatrix;

        public :
                virtual ~Normalize(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/**********************************************  Transpose   *******************************************/
/*******************************************************************************************************/

class MTranspose : public FMatrix
{
	private :
		ISMInput inMatrix;

	public :
		virtual ~MTranspose(){}

		virtual void compute();
		virtual void setparameters();
		virtual void prerun();
};

/*******************************************************************************************************/
/***************************************  Matrix/Dot/Cross Product  ************************************/
/*******************************************************************************************************/

// Matrix product 
class MatrixProd : public FMatrix
{
        private :

                ISMInput inMatrix1;
                ISMInput inMatrix2;

        public :
                virtual ~MatrixProd(){}

                virtual void compute();
                virtual void setparameters();
                virtual void prerun();
};

//Dot Product
class DotProd : public FScalar
{
        private :

                ISMInput inVector1;
                ISMInput inVector2;

        public :

                virtual ~DotProd(){}

                virtual void compute();
                virtual void setparameters();
                virtual void prerun();
};

//Cross Product
class CrossProd : public FMatrix
{
        private :

                ISMInput inVector1;
                ISMInput inVector2;

        public :

                virtual ~CrossProd(){}

                virtual void compute();
                virtual void setparameters();
                virtual void prerun();
};

//Outer Product is a Tensor Product for Vector
//This is the same operation same a Matrix Product for the two vector 
class OuterProd : public FMatrix
{
	private : 
                ISMInput inVector1;
                ISMInput inVector2;

	public : 
		virtual ~OuterProd(){}

                virtual void compute();
                virtual void setparameters();
                virtual void prerun();
};

#endif // _ARITHMETIC_H_
