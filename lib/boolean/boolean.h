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


#ifndef _BOOLEAN_H_
#define _BOOLEAN_H_

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

// Note : 
//  Input lower than 0.5 -> 0 logic
//  Input greater than 0.5 -> 1 logic

/*******************************************************************************************************/
/*************************************************  AND  ***********************************************/
/*******************************************************************************************************/

template<typename Scalar>
struct FuncOR {
  FuncOR(const Scalar& sOR) : sOR(sOR) {}
  const Scalar operator()(const Scalar& x) const 
  { 
	  Scalar v = x < 0.5 ? 0.0 : 1.0;
	  return std::max( v , sOR ) ;
  }
  Scalar sOR;
};


class MAND :  public FMatrix
{
	private :

                ISMInput inMatrix;

        public :

                virtual ~MAND(){}

                virtual void compute();
                virtual void setparameters();
};

class MSAND :  public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput inScalar;

        public :

                virtual ~MSAND(){}

                virtual void compute();
                virtual void setparameters();
};


class SAND :  public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SAND(){}

                virtual void compute();
                virtual void setparameters();

};

/******************************************************************************************************/
/************************************************  OR  ************************************************/
/******************************************************************************************************/

class MOR :  public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MOR(){}

                virtual void compute();
                virtual void setparameters();
};

class MSOR :  public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput inScalar;

        public :

                virtual ~MSOR(){}

                virtual void compute();
                virtual void setparameters();
};


class SOR :  public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SOR(){}

                virtual void compute();
                virtual void setparameters();
};

/******************************************************************************************************/
/************************************************  XOR  ***********************************************/
/******************************************************************************************************/

class MXOR :  public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MXOR(){}

                virtual void compute();
                virtual void setparameters();
};

class MSXOR :  public FMatrix
{
        private :

                ISMInput inMatrix;
                ISInput inScalar;

        public :

                virtual ~MSXOR(){}

                virtual void compute();
                virtual void setparameters();
};


class SXOR :  public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SXOR(){}

                virtual void compute();
                virtual void setparameters();
};

/******************************************************************************************************/
/************************************************  NOT  ***********************************************/
/******************************************************************************************************/

class MNOT :  public FMatrix
{
        private :

                ISMInput inMatrix;

        public :

                virtual ~MNOT(){}

                virtual void compute();
                virtual void setparameters();
};

class SNOT :  public FScalar
{
        private :

                ISInput inScalar;

        public :

                virtual ~SNOT(){}

                virtual void compute();
                virtual void setparameters();
};


/******************************************************************************************************/
/*********************************************  FLIP-FLOP  ********************************************/
/******************************************************************************************************/

class SFLIPFLOP :  public FScalar
{
        private :

                ISInput set;
								ISInput reset;

        public :
 
                virtual ~SFLIPFLOP(){}

                virtual void compute();
                virtual void setparameters();
};

class MFLIPFLOP :  public FMatrix
{
        private :

                ISMInput set;
								ISMInput reset;

								MATRIX mem;

        public :

                virtual ~MFLIPFLOP(){}

                virtual void compute();
                virtual void setparameters();
};


#endif // _BOOLEAN_H_
