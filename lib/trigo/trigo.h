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


#ifndef _TRIGO_H_
#define _TRIGO_H_

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

/*******************************************************************************************************/
/****************************************  SIN, COS, TAN   *********************************************/
/*******************************************************************************************************/

class MCos : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MCos(){}

                virtual void compute();
                virtual void setparameters();
};

class SCos : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SCos(){}

                virtual void compute();
                virtual void setparameters();
};

class MSin : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MSin(){}

                virtual void compute();
                virtual void setparameters();
};

class SSin : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SSin(){}

                virtual void compute();
                virtual void setparameters();
};

class MTan : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MTan(){}

                virtual void compute();
                virtual void setparameters();
};

class STan : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~STan(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/****************************************  ASIN, ACOS, ATAN  *******************************************/
/*******************************************************************************************************/

class MAcos : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MAcos(){}

                virtual void compute();
                virtual void setparameters();
};

class SAcos : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SAcos(){}

                virtual void compute();
                virtual void setparameters();
};

class MAsin : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MAsin(){}

                virtual void compute();
                virtual void setparameters();
};

class SAsin : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SAsin(){}

                virtual void compute();
                virtual void setparameters();
};

class MAtan : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MAtan(){}

                virtual void compute();
                virtual void setparameters();
};

class SAtan : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SAtan(){}

                virtual void compute();
                virtual void setparameters();
};


/*******************************************************************************************************/
/****************************************  SINH, COSH, TANH  *******************************************/
/*******************************************************************************************************/

class MCosh : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MCosh(){}

                virtual void compute();
                virtual void setparameters();
};

class SCosh : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SCosh(){}

                virtual void compute();
                virtual void setparameters();
};

class MSinh : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MSinh(){}

                virtual void compute();
                virtual void setparameters();
};

class SSinh : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SSinh(){}

                virtual void compute();
                virtual void setparameters();
};

class MTanh : public FMatrix
{
	private : 
		ISMInput inMatrix;

        public :

                virtual ~MTanh(){}

                virtual void compute();
                virtual void setparameters();
};

class STanh : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~STanh(){}

                virtual void compute();
                virtual void setparameters();
};

/*******************************************************************************************************/
/*********************************************  ATAN2   ************************************************/
/*******************************************************************************************************/

class SAtan2 : public FScalar
{
	private :
		ISInput x;
		ISInput y;

        public :

                virtual ~SAtan2(){}

                virtual void compute();
                virtual void setparameters();
};

#endif // _TRIGO_H_
