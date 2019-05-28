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

#include "custom_sub.h"

REGISTER_FUNCTION(JointPosSub);
REGISTER_FUNCTION(JointVelSub);

/*******************************************************************************************************/
/*********************                          JointPos                            ********************/
/*******************************************************************************************************/

void JointPosSub::setparameters()
{
	FMatrixSub<hieroglyph::JointPos>::setparameters();

	if( output.size() != 3 )  throw std::invalid_argument("JointPosSub : Output dimension should be 3 !");
}

void JointPosSub::callback(const hieroglyph::JointPos::ConstPtr &msg)
{
	auto mout = getMapVect(output);
        mout[0] = msg->accel;
        mout[1] = msg->vel;
        mout[2] = msg->pos;
}


/*******************************************************************************************************/
/*********************                           JointVel                           ********************/
/*******************************************************************************************************/

void JointVelSub::setparameters()
{
	FMatrixSub<hieroglyph::JointVel>::setparameters();

	if(output.size() != 2) throw std::invalid_argument("JointVelSub : Output dimension should be 2 !");
}

void JointVelSub::callback(const hieroglyph::JointVel::ConstPtr &msg)
{
	auto mout = getMapVect(output);
        mout[0] = msg->accel;
        mout[1] = msg->vel;
}

