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
REGISTER_FUNCTION(AttractorPub);

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
	if( output.size() != 2 ) throw std::invalid_argument("JointVelPub : Output must be a 2D Vector.");
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
        if( output.size() != 3 ) throw std::invalid_argument("JointPosPub : Output must be a 3D Vector.");
        FMatrixPub<hieroglyph::JointPos>::setparameters();

        Kernel::iBind(accel,"accel", getUuid());
        Kernel::iBind(vel,"vel", getUuid());
        Kernel::iBind(pos,"pos", getUuid());
}

/*******************************************************************************************************/
/********************                        AttractorPub                           ********************/
/*******************************************************************************************************/

void AttractorPub::compute()
{
	output = attractor()();
        auto mout = getMapVect(output);

	msg.norm = mout[0];
	msg.theta = mout[1];
	msg.pose_theta = mout[2];
	msg.force = mout[3];

	pub.publish(msg);
}

void AttractorPub::setparameters()
{
        if( output.size() != 4 ) throw std::invalid_argument("AttractorPub : Output must be a 4D Vector.");
        FMatrixPub<hieroglyph::Attractor>::setparameters();

        Kernel::iBind(attractor,"attractor", getUuid());
        Kernel::iBind(frame_id,"frame_id", getUuid());
        Kernel::iBind(pose_frame_id,"pose_frame_id", getUuid());
}

void AttractorPub::prerun()
{
        if( attractor().iSize() != 4 || !attractor().isVect()) throw std::invalid_argument("AttractorPub : attractor must be a 4D Vector.");

	FMatrixPub<hieroglyph::Attractor>::prerun();

	msg.header.frame_id = frame_id;
	msg.pose_frame_id = pose_frame_id;
}
