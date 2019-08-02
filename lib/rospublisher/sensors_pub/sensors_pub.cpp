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

#include "sensors_pub.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>


REGISTER_FUNCTION(ImuPub);

/*******************************************************************************************************/
/***********************                       ImuPub                            ***********************/
/*******************************************************************************************************/

void ImuPub::compute()
{
	auto mout = getMapVect(output);

	MATRIX tmp = orientation()();

        auto mtmp = getMapVect(tmp);

	mout[0] = mtmp[0];
	mout[1] = mtmp[1];
	mout[2] = mtmp[2];

	tmp = angular_velocity()();
        mtmp = getMapVect(tmp);
	
	mout[3] = mtmp[0];
	mout[4] = mtmp[1];
	mout[5] = mtmp[2];
	
	tmp = linear_acceleration()();
        mtmp = getMapVect(tmp);
	
	mout[6] = mtmp[0];
	mout[7] = mtmp[1];
	mout[8] = mtmp[2];

	tf2::Quaternion q;
	q.setRPY(mout[0],mout[1],mout[2]);
	msg.orientation = tf2::toMsg(q);

	msg.angular_velocity.x = mout[3];
	msg.angular_velocity.y = mout[4];
	msg.angular_velocity.z = mout[5];

	msg.linear_acceleration.x = mout[6]; 
	msg.linear_acceleration.y = mout[7]; 
	msg.linear_acceleration.z = mout[8]; 

	pub.publish(msg);
}

void ImuPub::setparameters()
{
	if( output.size() != 9 ) throw std::invalid_argument("ImuPub : Output must be a 9D vector");

	FMatrixPub<sensor_msgs::Imu>::setparameters();

	Kernel::iBind(orientation,"orientation",getUuid());
	Kernel::iBind(angular_velocity,"angular_velocity",getUuid());
	Kernel::iBind(linear_acceleration,"linear_acceleration",getUuid());
}

void ImuPub::prerun()
{
	if( orientation().iSize() != 3 || !orientation().isVect())  throw std::invalid_argument("ImuPub : orientation must be a 3D Vector.");
	if( angular_velocity().iSize() != 3 || !angular_velocity().isVect())  throw std::invalid_argument("ImuPub : angular_velocity must be a 3D Vector.");
	if( linear_acceleration().iSize() != 3 || !linear_acceleration().isVect())  throw std::invalid_argument("ImuPub : linear_acceleration must be a 3D Vector.");

	msg.header.frame_id = "imu";

	FMatrixPub<sensor_msgs::Imu>::prerun();

}

