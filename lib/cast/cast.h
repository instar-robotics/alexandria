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


#ifndef __CAST_HPP__
#define __CAST_HPP__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"

/*
 * Cast Scalar to Matrix
 * Output Matrix dimension must be 1
 */
class Cast_SToM : public FMatrix
{
	private : 
		ISInput inScalar;

	public :

		Cast_SToM() : FMatrix(POINT){}
		virtual ~Cast_SToM(){}

		virtual void setparameters();
		virtual void compute();
};

/*
 * Cast Matrix to Scalar
 * Input Matrix dimension must be 1
 */
class Cast_MToS : public FScalar
{
	private : 
		ISMInput inMatrix;

	public :

		Cast_MToS(){}
		virtual ~Cast_MToS(){}

		virtual void setparameters();
		virtual void compute();
		virtual void prerun();
};


/*
 * Concat 3 Scalars to 3D Vector
 */
class Concat_STo3DV : public FMatrix
{
	private :
                ISInput x;
                ISInput y;
                ISInput z;

        public :

                Concat_STo3DV() : FMatrix(VECTOR){}
                virtual ~Concat_STo3DV(){}

                virtual void setparameters();
                virtual void compute();
};

/*
 * Concat 2 Scalars to 2D Vector
 */
class Concat_STo2DV : public FMatrix
{
	private :
                ISInput x;
                ISInput y;

        public :

                Concat_STo2DV() : FMatrix(VECTOR){}
                virtual ~Concat_STo2DV(){}

                virtual void setparameters();
                virtual void compute();
};

#endif // __CAST_HPP__
