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


#ifndef _MEMORY_H_
#define _MEMORY_H_

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

/*******************************************************************************************************/
/**********************************************  STORAGE  **********************************************/
/*******************************************************************************************************/

class MStorage : public FMatrix
{
        private :
                ISMInput inMatrix;
                ISInput index;
                ISInput record;

                std::vector<MATRIX> memory;

        public :
                virtual ~MStorage(){}
                virtual void compute();
                virtual void setparameters();
};

class SStorage : public FScalar
{
        private :
                ISInput inScalar;
                ISInput index;
                ISInput record;

                std::vector<SCALAR> memory;

        public :
                virtual ~SStorage(){}
                virtual void compute();
                virtual void setparameters();
};



/*******************************************************************************************************/
/***********************************************  MEMORY  **********************************************/
/*******************************************************************************************************/
/*
class MMemory : public FMatrix
{
        private :
                ISMInput inMatrix;
                ISInput reset;

        public :
                virtual void compute();
                virtual void setparameters();
};

class SMemory : public FScalar
{
        private :
                ISInput inScalar;
                ISInput reset;

        public :
                virtual void compute();
                virtual void setparameters();
};
*/
#endif // _MEMORY_H_
