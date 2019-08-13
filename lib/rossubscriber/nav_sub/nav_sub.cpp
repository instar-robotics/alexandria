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

#include "nav_sub.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

REGISTER_FUNCTION(OdoPosSub);
REGISTER_FUNCTION(OdoPosXSub);
REGISTER_FUNCTION(OdoPosYSub);
REGISTER_FUNCTION(OdoPosZSub);
REGISTER_FUNCTION(OdoEulerSub);
REGISTER_FUNCTION(OdoEulerRollSub);
REGISTER_FUNCTION(OdoEulerPitchSub);
REGISTER_FUNCTION(OdoEulerYawSub);
REGISTER_FUNCTION(OdoQuaterSub);
REGISTER_FUNCTION(OdoQuaterXSub);
REGISTER_FUNCTION(OdoQuaterYSub);
REGISTER_FUNCTION(OdoQuaterZSub);
REGISTER_FUNCTION(OdoQuaterWSub);
REGISTER_FUNCTION(OdoTwistLinSub);
REGISTER_FUNCTION(OdoTwistLinXSub);
REGISTER_FUNCTION(OdoTwistLinYSub);
REGISTER_FUNCTION(OdoTwistLinZSub);
REGISTER_FUNCTION(OdoTwistAngSub);
REGISTER_FUNCTION(OdoTwistAngRollSub);
REGISTER_FUNCTION(OdoTwistAngPitchSub);
REGISTER_FUNCTION(OdoTwistAngYawSub);

/*******************************************************************************************************/
/*********************                            OdoPos                            ********************/
/*******************************************************************************************************/

void OdoPosSub::setparameters()
{
        if( output.size() != 3 ) throw std::invalid_argument("OdoPosSub : Output must be a 3D Vector.");
	FMatrixSub<nav_msgs::Odometry>::setparameters();

	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoPosSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
       	 auto mout = getMapVect(output);
       	 mout[0] = msg->pose.pose.position.x;
       	 mout[1] = msg->pose.pose.position.y;
       	 mout[2] = msg->pose.pose.position.z;
	}
}

/*******************************************************************************************************/
/*********************                           OdoPosX                            ********************/
/*******************************************************************************************************/

void OdoPosXSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoPosXSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{ 
		output = msg->pose.pose.position.x;
	}
}

/*******************************************************************************************************/
/*********************                           OdoPosY                            ********************/
/*******************************************************************************************************/

void OdoPosYSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoPosYSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
        	output = msg->pose.pose.position.y;
	}
}

/*******************************************************************************************************/
/*********************                           OdoPosZ                            ********************/
/*******************************************************************************************************/

void OdoPosZSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoPosZSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		output = msg->pose.pose.position.z;
	}
}

/*******************************************************************************************************/
/*********************                          OdoEuler                            ********************/
/*******************************************************************************************************/

void OdoEulerSub::setparameters()
{
        if( output.size() != 3 ) throw std::invalid_argument("OdoEulerSub : Output must be 3D Vector.");
	FMatrixSub<nav_msgs::Odometry>::setparameters();
	
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoEulerSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{

	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		tf2::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

		tf2::Matrix3x3 m(q);
		SCALAR roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		auto mout = getMapVect(output);
		mout[0] = roll;
		mout[1] = pitch;
		mout[2] = yaw;
	}
}

/*******************************************************************************************************/
/*********************                       OdoEulerRoll                           ********************/
/*******************************************************************************************************/

void OdoEulerRollSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoEulerRollSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		tf2::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

		tf2::Matrix3x3 m(q);
		SCALAR roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		output = roll;
	}
}

/*******************************************************************************************************/
/*********************                      OdoEulerPitch                           ********************/
/*******************************************************************************************************/

void OdoEulerPitchSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoEulerPitchSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		tf2::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

		tf2::Matrix3x3 m(q);
		SCALAR roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		output = pitch;
	}
}

/*******************************************************************************************************/
/*********************                        OdoEulerYaw                           ********************/
/*******************************************************************************************************/

void OdoEulerYawSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoEulerYawSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		tf2::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

		tf2::Matrix3x3 m(q);
		SCALAR roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		output = yaw;
	}
}

/*******************************************************************************************************/
/*********************                         OdoQuater                            ********************/
/*******************************************************************************************************/

void OdoQuaterSub::setparameters()
{
        if( output.size() != 4 ) throw std::invalid_argument("OdoQuaterSub : Output must be 4D Vector.");
	FMatrixSub<nav_msgs::Odometry>::setparameters();
	
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoQuaterSub::callback( const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		auto mout = getMapVect(output);
		mout[0] = msg->pose.pose.orientation.x;
		mout[1] = msg->pose.pose.orientation.y;
		mout[2] = msg->pose.pose.orientation.z;
		mout[3] = msg->pose.pose.orientation.w;
	}
}

/*******************************************************************************************************/
/*********************                         OdoQuaterX                           ********************/
/*******************************************************************************************************/

void OdoQuaterXSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoQuaterXSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		output = msg->pose.pose.orientation.x;
	}
}

/*******************************************************************************************************/
/*********************                         OdoQuaterY                           ********************/
/*******************************************************************************************************/

void OdoQuaterYSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoQuaterYSub::callback( const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		output = msg->pose.pose.orientation.y;
	}
}

/*******************************************************************************************************/
/*********************                         OdoQuaterZ                           ********************/
/*******************************************************************************************************/

void OdoQuaterZSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoQuaterZSub::callback( const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		output = msg->pose.pose.orientation.z;
	}
}

/*******************************************************************************************************/
/*********************                         OdoQuaterW                           ********************/
/*******************************************************************************************************/

void OdoQuaterWSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoQuaterWSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
        	output = msg->pose.pose.orientation.w;
	}
}

/*******************************************************************************************************/
/*********************                        OdoTwistLin                           ********************/
/*******************************************************************************************************/

void OdoTwistLinSub::setparameters()
{
        if( output.size() != 3 ) throw std::invalid_argument("OdoTwistLinSub : Output must be 3D Vector.");
	FMatrixSub<nav_msgs::Odometry>::setparameters();
	
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoTwistLinSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		auto mout = getMapVect(output);
		mout[0] = msg->twist.twist.linear.x;
		mout[1] = msg->twist.twist.linear.y;
		mout[2] = msg->twist.twist.linear.z;
	}
	else output = MATRIX::Constant(output.rows(),output.cols(),0);
}

/*******************************************************************************************************/
/*********************                      OdoTwistLinX                            ********************/
/*******************************************************************************************************/

void OdoTwistLinXSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoTwistLinXSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
        	output = msg->twist.twist.linear.x;
	}
	else output = 0;
}

/*******************************************************************************************************/
/*********************                       OdoTwistLinY                           ********************/
/*******************************************************************************************************/

void OdoTwistLinYSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoTwistLinYSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		output = msg->twist.twist.linear.y;
	}
	else output = 0;
}

/*******************************************************************************************************/
/*********************                      OdoTwistLinZ                            ********************/
/*******************************************************************************************************/

void OdoTwistLinZSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoTwistLinZSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		output = msg->twist.twist.linear.z;
	}
	else output = 0;
}

/*******************************************************************************************************/
/*********************                       OdoTwistAng                            ********************/
/*******************************************************************************************************/

void OdoTwistAngSub::setparameters()
{
        if( output.size() != 3 ) throw std::invalid_argument("OdoTwistAngSub : Output must be 3D Vector.");
	FMatrixSub<nav_msgs::Odometry>::setparameters();
	
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoTwistAngSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		auto mout = getMapVect(output);
		mout[0] = msg->twist.twist.angular.x;
		mout[1] = msg->twist.twist.angular.y;
		mout[2] = msg->twist.twist.angular.z;
	}
	else output = MATRIX::Constant(output.rows(),output.cols(),0);
}

/*******************************************************************************************************/
/*********************                     OdoTwistAngRoll                          ********************/
/*******************************************************************************************************/

void OdoTwistAngRollSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoTwistAngRollSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		output = msg->twist.twist.angular.x;
	}
	else output = 0;
}

/*******************************************************************************************************/
/***********************                   OdoTwistAngPitch                       **********************/
/*******************************************************************************************************/

void OdoTwistAngPitchSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoTwistAngPitchSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		output = msg->twist.twist.angular.y;
	}
	else output = 0;
}

/*******************************************************************************************************/
/*********************                       OdoTwistAngYaw                         ********************/
/*******************************************************************************************************/

void OdoTwistAngYawSub::setparameters()
{
	FScalarSub<nav_msgs::Odometry>::setparameters();
	Kernel::iBind(frame_id,"frame_id", getUuid());
}

void OdoTwistAngYawSub::callback(const nav_msgs::Odometry::ConstPtr &msg )
{
	if( msg->header.frame_id == frame_id  || msg->header.frame_id == ALL )
	{
		output = msg->twist.twist.angular.z;
	}
	else output = 0;
}
