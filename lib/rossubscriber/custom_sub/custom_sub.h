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

#ifndef __CUSTOM_MSGS_HPP__
#define __CUSTOM_MSGS_HPP__

#include "hieroglyph/JointPos.h"
#include "hieroglyph/JointVel.h"
#include "kheops/ros/fsub.h"

/* Note :
 *	1- Each FMatrixSub or FScalarSub object has 3 default Kheops Input:
 *	- IString topic_name : for the topic Name
 *	- ISInput size_queue : define the size of the queue
 *	- ISInput sleep      : define the behavior of the Function [blocking Function if sleep < 0 or non-blocking Function and time to wait if sleep >= 0]
 *
 *	2- This 3 inputs MUST BE BIND to the kernel in the method setparameters.
 *	If you extend setparameters to add other Inputs, don't forget to call FMatrixSub::setparameters or FScalarSub::setparameters ! 
 *
 *	3- And Most important : don't forget to add this Inputs in the XML description !
 *	For now, we don't have mechanisms to load automatically the input in the XML description
 *	Using XML ENTITY could be a good way to do this.
 */

/*******************************************************************************************************/
/*****************                             JointPos                              *******************/
/*******************************************************************************************************/

/*
 * JointPosSub : read topic to control a joint in accel/vel/pos
 */
class JointPosSub: public FMatrixSub<hieroglyph::JointPos>
{
        public :
                JointPosSub() : FMatrixSub<hieroglyph::JointPos>() {}
                virtual ~JointPosSub(){}

                virtual void setparameters();
                virtual void callback(const hieroglyph::JointPos::ConstPtr &msg);
};

/*******************************************************************************************************/
/******************                             JointVel                            ********************/
/*******************************************************************************************************/

/*
 * JointVelSub : read topic to control a joint in accel/vel
 */
class JointVelSub : public FMatrixSub<hieroglyph::JointVel>
{
        public :
                JointVelSub() : FMatrixSub<hieroglyph::JointVel>() {}
                virtual ~JointVelSub(){}

                virtual void setparameters();
                virtual void callback(const hieroglyph::JointVel::ConstPtr &msg);
};

#endif //__CUSTOM_MSGS_HPP__
