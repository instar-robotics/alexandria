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


#ifndef _POW_H_
#define _POW_H_

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

/********************************************************************************************************/
/*************************************************  Exp   ***********************************************/
/********************************************************************************************************/

class MExp : public FMatrix
{
        private :

                ISMInput exponent;

        public :

                virtual ~MExp(){}

                virtual void compute();
                virtual void setparameters();
};

class SExp : public FScalar
{
        private :

                ISInput exponent;

        public :

                virtual ~SExp(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/***********************************************   LOG   ************************************************/
/********************************************************************************************************/

class MLog : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MLog(){}

                virtual void compute();
                virtual void setparameters();
};

class SLog : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SLog(){}

                virtual void compute();
                virtual void setparameters();
};

class MLog10 : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MLog10(){}

                virtual void compute();
                virtual void setparameters();
};

class SLog10 : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SLog10(){}

                virtual void compute();
                virtual void setparameters();
};


/********************************************************************************************************/
/*************************************************  POW   ***********************************************/
/********************************************************************************************************/

class MPow : public FMatrix
{
        private :

                ISMInput base;
                ISMInput exponent;

        public :

                virtual ~MPow(){}

                virtual void compute();
                virtual void setparameters();
};

class SPow : public FScalar
{
        private :

                ISInput base;
                ISInput exponent;

        public :

                virtual ~SPow(){}

                virtual void compute();
                virtual void setparameters();

};

class MSPow : public FMatrix
{
        private :

                ISMInput base;
                ISInput exponent;

        public :

                virtual ~MSPow(){}

                virtual void compute();
                virtual void setparameters();
};

class SMPow : public FMatrix
{
        private :

                ISInput base;
                ISMInput exponent;

        public :

                virtual ~SMPow(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/***********************************************   SQRT   ***********************************************/
/********************************************************************************************************/

class MSqrt : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MSqrt(){}

                virtual void compute();
                virtual void setparameters();
};

class SSqrt : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SSqrt(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/*********************************************   SQUARE   ***********************************************/
/********************************************************************************************************/

class MSquare : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MSquare(){}

                virtual void compute();
                virtual void setparameters();
};

class SSquare : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SSquare(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/**********************************************   CUBE   ************************************************/
/********************************************************************************************************/

class MCube : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MCube(){}

                virtual void compute();
                virtual void setparameters();
};

class SCube : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SCube(){}

                virtual void compute();
                virtual void setparameters();
};

/********************************************************************************************************/
/*********************************************   Inverse   **********************************************/
/********************************************************************************************************/

class MInverse : public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MInverse(){}

                virtual void compute();
                virtual void setparameters();
};

class SInverse : public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SInverse(){}

                virtual void compute();
                virtual void setparameters();
};

#endif // _POW_H_
