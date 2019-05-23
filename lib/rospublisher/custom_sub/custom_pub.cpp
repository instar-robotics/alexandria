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

#include "custom_pub.h"

REGISTER_FUNCTION(JointVelPub);
REGISTER_FUNCTION(JointPosPub);

/*******************************************************************************************************/
/********************                        JointVelPub                           *********************/
/*******************************************************************************************************/

void JointVelPub::compute()
{
	auto mout = getMapVect(output);
	mout[0] = accel()();
	mout[1] = vel()();

	hieroglyph::JointVel msg;
	msg.accel = mout[0];
	msg.vel = mout[1];

	pub.publish(msg);
}

void JointVelPub::setparameters()
{
	if( output.size() != 2 ) throw std::invalid_argument("JointVelPub : Output dimension should be 2 !");
	FMatrixPub<hieroglyph::JointVel>::setparameters();

 	Kernel::iBind(accel,"accel", getUuid());
 	Kernel::iBind(vel,"vel", getUuid());
}

/*******************************************************************************************************/
/********************                         JointPosPub                           ********************/
/*******************************************************************************************************/

void JointPosPub::compute()
{
        auto mout = getMapVect(output);
        mout[0] = accel()();
        mout[1] = vel()();
        mout[2] = pos()();

	hieroglyph::JointPos msg;
        msg.accel = mout[0];
        msg.vel = mout[1];
        msg.pos = mout[2];

        pub.publish(msg);
}

void JointPosPub::setparameters()
{
        if( output.size() != 3 ) throw std::invalid_argument("JointPosPub : Output dimension should be 3 !");
        FMatrixPub<hieroglyph::JointPos>::setparameters();

        Kernel::iBind(accel,"accel", getUuid());
        Kernel::iBind(vel,"vel", getUuid());
        Kernel::iBind(pos,"pos", getUuid());
}
