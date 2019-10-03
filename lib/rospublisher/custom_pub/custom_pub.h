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


#ifndef __CUSTOM_PUB_HPP__
#define __CUSTOM_PUB_HPP__

#include "kheops/ros/fpub.h"
#include <hieroglyph/JointVel.h>
#include <hieroglyph/JointPos.h>
#include <hieroglyph/Attractor.h>

/* Note :
 *      1- Each FMatrixPub or FScalarPub object has 2 default Kheops Input:
 *      - IString topic_name : for the topic Name
 *      - ISInput size_queue : define the size of the queue
 *
 *      2- This 2 inputs MUST BE BIND to the kernel in the method setparameters.
 *      If you extend setparameters to add other Inputs, don't forget to call FMatrixPub::setparameters or FScalarPub::setparameters !
 *
 *      3- And Most important : don't forget to add this Inputs in the XML description !
 *      For now, we don't have mechanisms to load automatically the input in the XML description
 *      Using XML ENTITY could be a good way to do this.
 */


/*
 * JointVelPub : Send command velocity for a joint in hieroglyph/JointVel format 
 * 2 Scalars input (accel,vel)
 */
class JointVelPub : public FMatrixPub<hieroglyph::JointVel>
{
	private :

		ISInput accel;
		ISInput vel;

	public :
		
		JointVelPub() : FMatrixPub<hieroglyph::JointVel>(VECTOR) {}
		virtual ~JointVelPub(){}

		virtual void compute();
                virtual void setparameters();
};

/*
 * JointPosPub : Send command velocity for a joint in hieroglyph/JointPos format 
 * 3 Scalars input (accel,vel,pos)
 */
class JointPosPub : public FMatrixPub<hieroglyph::JointPos>
{
        private :

                ISInput accel;
                ISInput vel;
                ISInput pos;

        public :

                JointPosPub() : FMatrixPub<hieroglyph::JointPos>(VECTOR) {}
                virtual ~JointPosPub(){}

                virtual void compute();
                virtual void setparameters();
};

/*
 * AttractorPub : Send an hieroglyph/Attractor message
 * Matrix input (norm,theta,pose_theta,force)
 */
class AttractorPub : public FMatrixPub<hieroglyph::Attractor>
{
        private :

		IString frame_id;
		IString pose_frame_id;
                ISMInput attractor;
		hieroglyph::Attractor msg;


        public :

                AttractorPub() : FMatrixPub<hieroglyph::Attractor>(VECTOR) {}
                virtual ~AttractorPub(){}

                virtual void compute();
                virtual void setparameters();
                virtual void prerun();
};

#endif // __CUSTOM_PUB_HPP__
