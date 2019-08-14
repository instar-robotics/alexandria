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

/*
 * TODO : 
 *
 * 2D Rotation
 * 3D Rotation angle+axis
 * 3D Rotation as a quaternion
 *
 * Translation
 *
 * Scaling
 *
 * Quaternion Normalize ?
 * Quaternion to Rotation Matrix ?
 * Euler to RotationMatrix ? 
 */

#ifndef __SPACE_TRANSFORMATION__
#define __SPACE_TRANSFORMATION__

#include "kheops/kernel/function.h"
#include "kheops/kernel/kernel.h"


/***************************************************************************************************/
/************************************   EulerToQuaternion   ****************************************/
/***************************************************************************************************/

class EulerToQuaternion : public FMatrix 
{
	private :

		ISMInput inEuler;

	public : 
	
		EulerToQuaternion() : FMatrix(VECTOR) {}

		virtual void compute();
		virtual void setparameters();
		virtual void prerun();
};

/***************************************************************************************************/
/************************************   QuaternionToEuler   ****************************************/
/***************************************************************************************************/

class QuaternionToEuler : public FMatrix
{
        private :

                ISMInput inQuater;

        public :

                QuaternionToEuler() : FMatrix(VECTOR) {}

                virtual void compute();
                virtual void setparameters();
                virtual void prerun();
};

#endif // __SPACE_TRANSFORMATION__
